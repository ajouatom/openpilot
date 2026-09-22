import os
import types

import pytest

from openpilot.common import display_scheduling as ds


@pytest.fixture
def fake_scheduler(monkeypatch):
  workers = {11: {'aff': {0}, 'nice': 0, 'policy': 0}, 12: {'aff': {0}, 'nice': 0, 'policy': 0}}
  state = {'online': True, 'now': 1.0, 'race': False, 'deny_restore': False}
  calls = []
  monkeypatch.setattr(ds, 'thread_ids', lambda: list(workers))
  monkeypatch.setattr(ds, 'core_online', lambda core: state['online'])
  monkeypatch.setattr(ds.time, 'monotonic', lambda: state['now'])
  monkeypatch.setattr(os, 'SCHED_OTHER', 0, raising=False)
  monkeypatch.setattr(os, 'PRIO_PROCESS', 0, raising=False)
  monkeypatch.setattr(os, 'sched_param', lambda priority: priority, raising=False)
  monkeypatch.setattr(os, 'sched_getscheduler', lambda tid: workers[tid]['policy'], raising=False)
  monkeypatch.setattr(os, 'sched_getaffinity', lambda tid: workers[tid]['aff'], raising=False)
  monkeypatch.setattr(os, 'getpriority', lambda kind, tid: workers[tid]['nice'], raising=False)

  def affinity(tid, cores):
    calls.append(('aff', tid, cores))
    if state['race'] and cores == {6}:
      raise OSError('CPU offlined between check and syscall')
    workers[tid]['aff'] = set(cores)

  def priority(kind, tid, nice):
    calls.append(('nice', tid, nice))
    if state['deny_restore'] and nice == 0:
      raise PermissionError('RLIMIT_NICE')
    workers[tid]['nice'] = nice

  def policy(tid, value, param):
    workers[tid]['policy'] = value

  monkeypatch.setattr(os, 'sched_setaffinity', affinity, raising=False)
  monkeypatch.setattr(os, 'setpriority', priority, raising=False)
  monkeypatch.setattr(os, 'sched_setscheduler', policy, raising=False)
  scheduler = ds.DisplayScheduler(6, enabled=False)
  scheduler.enabled = True
  return types.SimpleNamespace(scheduler=scheduler, workers=workers, state=state, calls=calls)


def test_onroad_workers_lower_priority_before_moving_and_offroad_returns_immediately(fake_scheduler):
  f = fake_scheduler
  f.scheduler.update(True)
  assert all(w == {'aff': {6}, 'nice': 19, 'policy': 0} for w in f.workers.values())
  assert f.calls[0] == ('nice', 11, 19)
  f.calls.clear()
  f.scheduler.update(False)  # transition bypasses the half-second polling interval
  assert all(w == {'aff': {0, 1, 2, 3}, 'nice': 0, 'policy': 0} for w in f.workers.values())
  assert f.calls[0] == ('aff', 11, {0, 1, 2, 3})


def test_new_workers_and_affinity_drift_are_corrected_without_per_frame_sweeps(fake_scheduler):
  f = fake_scheduler
  f.scheduler.update(True)
  f.workers[13] = {'aff': {0}, 'nice': 0, 'policy': 1}
  f.scheduler.update(True)
  assert f.workers[13]['aff'] == {0}
  f.state['now'] += 0.6
  f.scheduler.update(True)
  assert f.workers[13] == {'aff': {6}, 'nice': 19, 'policy': 0}


@pytest.mark.parametrize('online,race', [(False, False), (True, True)])
def test_unavailable_big_core_falls_back_and_recovers(fake_scheduler, online, race):
  f = fake_scheduler
  f.state.update(online=online, race=race)
  f.scheduler.update(True)
  assert all(w['aff'] == {0, 1, 2, 3} for w in f.workers.values())
  f.state.update(online=True, race=False, now=2.0)
  f.scheduler.update(True)
  assert all(w['aff'] == {6} and w['nice'] == 19 for w in f.workers.values())


def test_offroad_affinity_remains_safe_when_nice_cannot_be_raised(fake_scheduler):
  f = fake_scheduler
  f.scheduler.update(True)
  f.state['deny_restore'] = True
  f.scheduler.update(False)
  assert all(w['aff'] == {0, 1, 2, 3} and w['nice'] == 19 for w in f.workers.values())


def test_cluster_uses_core7_with_the_same_transition_contract(fake_scheduler):
  f = fake_scheduler
  f.scheduler.core = 7
  f.scheduler.update(True)
  assert all(w['aff'] == {7} and w['nice'] == 19 for w in f.workers.values())
  f.scheduler.update(False)
  assert all(w['aff'] == {0, 1, 2, 3} for w in f.workers.values())


def test_desktop_does_not_change_scheduling(monkeypatch):
  monkeypatch.setattr(ds, 'thread_ids', lambda: pytest.fail('desktop scheduling changed'))
  ds.DisplayScheduler(6, enabled=False).update(True)


def test_encoder_process_workers_follow_onroad_and_offroad(fake_scheduler, monkeypatch):
  f = fake_scheduler
  f.workers[21] = {'aff': {0}, 'nice': 0, 'policy': 0}
  monkeypatch.setattr(ds, 'thread_ids', lambda pid='self': [11, 12] if pid == 'self' else [21])
  f.scheduler.core = 7
  f.scheduler.update(True, child_pid=20)
  assert f.workers[21] == {'aff': {7}, 'nice': 19, 'policy': 0}
  f.scheduler.update(False, child_pid=20)
  assert f.workers[21] == {'aff': {0, 1, 2, 3}, 'nice': 0, 'policy': 0}
