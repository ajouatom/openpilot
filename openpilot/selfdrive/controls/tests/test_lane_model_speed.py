import math

import pytest

from openpilot.selfdrive.controls.lib.lane_model_speed import LaneModelSpeedGuard


def warmed_guard():
  guard = LaneModelSpeedGuard(recovery_frames=20)
  for _ in range(21):
    guard.update(30.0, 30.0, 30.0)
  return guard


def test_startup_and_recovery_require_continuous_good_frames():
  guard = LaneModelSpeedGuard(recovery_frames=20)
  assert not any(guard.update(30.0, 30.0, 30.0) for _ in range(20))
  assert guard.update(30.0, 30.0, 30.0)
  assert not guard.update(29.04, 9.93, 12.09)
  assert not any(guard.update(30.0, 30.0, 30.0) for _ in range(20))
  assert guard.update(30.0, 30.0, 30.0)


def test_whole_trajectory_collapse_despite_increasing_profile():
  guard = warmed_guard()
  # Tucson incident: original end/start condition passes at highway speed.
  assert 12.09 > 9.93 * 0.7
  assert not guard.update(104.54 / 3.6, 9.93, 12.09)
  assert guard.valid_frames == 0


def test_original_predicted_deceleration_still_blocks():
  guard = warmed_guard()
  assert not guard.update(30.0, 30.0, 20.9)


@pytest.mark.parametrize(
  'actual,start,end',
  [
    (30.0, 21.0, 21.0),
    (30.0, 30.0, 21.0),
    (10.0, 9.0, 12.0),
    (0.0, 0.0, 0.0),
    (1.0, 1.0, 2.0),
    (30.0, 32.0, 34.0),
  ],
)
def test_usable_trajectories_and_ratio_boundary(actual, start, end):
  assert warmed_guard().update(actual, start, end)


@pytest.mark.parametrize(
  'actual,start,end',
  [
    (30.0, 20.999, 30.0),
    (30.0, 30.0, 20.999),
    (10.0, 0.0, 12.0),
    (math.nan, 30.0, 30.0),
    (30.0, math.nan, 30.0),
    (30.0, 30.0, math.nan),
    (math.inf, 30.0, 30.0),
    (30.0, math.inf, 30.0),
    (30.0, 30.0, math.inf),
    (-1.0, 30.0, 30.0),
    (30.0, -1.0, 30.0),
    (30.0, 30.0, -1.0),
  ],
)
def test_invalid_or_mismatched_speeds_block_immediately(actual, start, end):
  assert not warmed_guard().update(actual, start, end)


def test_intermittent_mismatch_cannot_accumulate_recovery_credit():
  guard = warmed_guard()
  for _ in range(10):
    assert not guard.update(30.0, 10.0, 15.0)
    assert not any(guard.update(30.0, 30.0, 30.0) for _ in range(19))
  assert not guard.update(30.0, 30.0, 30.0)
  assert guard.update(30.0, 30.0, 30.0)


def test_normal_profiles_preserve_original_gate_sequence():
  guard = LaneModelSpeedGuard(recovery_frames=20)
  count = 0
  for i in range(1000):
    actual = 5 + (i % 40)
    start = actual * 0.95
    end = start * (0.5 if i % 71 == 0 else 1.1)
    count = 0 if end < start * 0.7 else count + 1
    assert guard.update(actual, start, end) == (count > 20)
