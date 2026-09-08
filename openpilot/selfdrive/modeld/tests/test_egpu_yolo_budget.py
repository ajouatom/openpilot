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


def test_revalidation_never_weakens_startup_measurements_or_real_overrun():
  budget = prepared()
  floor = budget.floor
  budget.deadline = 100.006
  assert budget.admit(100.) == 'no_budget'
  budget.deadline = 100.009
  assert budget.admit(100.) == 'run'
  budget.finish(100., 100.011)
  assert budget.disabled_reason == 'overrun'
  assert budget.floor > floor
  budget.deadline = 200.009
  assert budget.admit(200.) == 'overrun'
  budget.disabled_reason = ''  # existing supervisor may authorize completed-miss recovery
  assert budget.admit(200.) == 'no_budget'
  assert budget.overruns == 1


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
