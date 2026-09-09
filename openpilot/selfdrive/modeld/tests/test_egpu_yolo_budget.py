import pytest

from openpilot.selfdrive.modeld.egpu_yolo import IdleBudget
from openpilot.selfdrive.modeld.egpu_yolo_budget import RevalidatingBudget


def prepared():
  budget = RevalidatingBudget(IdleBudget(.0054))
  budget.settled = 25
  budget.deadline = 100.009
  return budget


def test_completed_slow_sample_cannot_permanently_starve_resident_yolo():
  budget = prepared()
  assert budget.admit(100., min_interval=0.) == 'run'
  budget.finish(100., 100.00766)  # fits 9 ms, but padded reservation becomes 10.192 ms
  assert not budget.disabled_reason and budget.overruns == 0
  for frame in range(1, 600):
    now = 100+frame*.05
    budget.deadline = now+.009
    assert budget.admit(now, min_interval=0.) == 'no_budget'
  budget.deadline = 130.059
  assert budget.admit(130.05, min_interval=0.) == 'run'
  assert budget.estimate == pytest.approx(.0054*1.2)
  assert budget.probes == 5
  budget.finish(130.05, 130.055)
  budget.deadline = 130.109
  assert budget.admit(130.1, min_interval=0.) == 'rate_limit'
  for start in [131.05, 132.05, 133.05, 134.05]:
    budget.deadline = start+.009
    assert budget.admit(start, min_interval=0.) == 'run'
    budget.finish(start, start+.005)
  budget.deadline = 134.109
  assert budget.admit(134.1, min_interval=0.) == 'run'


def test_completed_overrun_needs_authorization_backoff_and_stable_frames():
  budget = prepared()
  floor = budget.floor
  budget.deadline = 100.006
  assert budget.admit(100.) == 'no_budget'
  budget.deadline = 100.009
  assert budget.admit(100.) == 'run'
  budget.finish(100., 100.011)
  assert budget.disabled_reason == 'overrun'
  assert budget.floor == floor
  assert budget.overrun_estimate > floor
  budget.deadline = 200.009
  assert budget.admit(200.) == 'overrun'
  budget.disabled_reason = ''  # existing supervisor may authorize completed-miss recovery
  assert budget.admit(200.) == 'no_budget'
  budget.settled = 200
  assert budget.admit(200., camera_pending=True) == 'camera_pending'
  assert budget.overrun_estimate > floor
  assert budget.admit(200.) == 'run'
  assert budget.probes == 5
  assert budget.estimate == floor
  assert budget.overruns == 1


def test_recorded_guard_miss_recovers_at_one_hz_without_forgetting_startup_floor():
  budget = prepared()
  budget.deadline = 100.00924439
  assert budget.admit(100., min_interval=0.) == 'run'
  budget.finish(100., 100.008751307)
  assert budget.disabled_reason == 'overrun'
  assert budget.estimate+.001 == pytest.approx(.0115015684)
  budget.disabled_reason = ''
  budget.settled = 200
  budget.deadline = 129.909
  assert budget.admit(129.9, min_interval=0.) == 'no_budget'
  budget.deadline = 131.006
  assert budget.admit(131., min_interval=0.) == 'no_budget'
  budget.deadline = 131.009
  assert budget.admit(131., min_interval=0.) == 'run'
  budget.finish(131., 131.005)
  budget.deadline = 131.509
  assert budget.admit(131.5, min_interval=0.) == 'rate_limit'
  for now in [132., 133., 134., 135.]:
    budget.deadline = now+.009
    assert budget.admit(now, min_interval=0.) == 'run'
    budget.finish(now, now+.005)
  budget.deadline = 135.059
  assert budget.admit(135.05, min_interval=0.) == 'run'
  assert budget.overruns == 1 and budget.floor == pytest.approx(.00648)


def test_repeated_probe_overruns_increase_backoff_and_keep_gpu_errors_latched():
  budget = prepared()
  now = 100.
  for expected_delay in [30., 60., 120., 240., 300., 300.]:
    budget.settled = 200
    budget.deadline = now+.009
    assert budget.admit(now, min_interval=0.) == 'run'
    budget.finish(now, now+.011)
    assert budget.recover_after == pytest.approx(now+.011+expected_delay)
    budget.disabled_reason = ''
    budget.deadline = budget.recover_after-.001+.009
    assert budget.admit(budget.recover_after-.001, min_interval=0.) == 'no_budget'
    now = budget.recover_after+.001
  assert budget.overruns == 6
  budget.disabled_reason = 'error'
  budget.deadline = now+1000
  assert budget.admit(now+600, min_interval=0.) == 'error'
  assert budget.overrun_estimate > budget.floor


@pytest.mark.parametrize('reason', ['warming', 'camera_pending', 'error'])
def test_revalidation_keeps_primary_guards_and_gpu_error_latch(reason):
  budget = prepared()
  budget.settled = 0 if reason == 'warming' else 25
  budget.disabled_reason = 'error' if reason == 'error' else ''
  assert budget.admit(100., camera_pending=reason == 'camera_pending') == reason


def test_continuing_slow_work_refreshes_the_reservation():
  budget = prepared()
  for now in [100., 120., 140.]:
    budget.deadline = now+.02
    assert budget.admit(now) == 'run'
    budget.finish(now, now+.008)
  budget.deadline = 160.009
  assert budget.admit(160.) == 'no_budget'
  assert budget.estimate == pytest.approx(.0096)
