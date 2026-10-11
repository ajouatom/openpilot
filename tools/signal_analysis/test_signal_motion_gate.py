from unittest.mock import patch

import pytest

from tools.signal_analysis.signal_motion_gate import GreenMotionGate, GreenPoseGate
from tools.signal_analysis import signal_tracker as module
from tools.signal_analysis.test_signal_revalidation import picture
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard


def straight_gate():
  gate = GreenMotionGate()
  for i in range(11):
    gate.update(1+i*.05, 0., 0., 5.)
  assert gate.allows_green(1.5)
  return gate


@pytest.mark.parametrize('angle,rate', [(15., 0.), (-15., 0.), (0., 90.), (0., -90.)])
def test_turn_rejects_immediately_and_requires_new_straight_dwell(angle, rate):
  gate = straight_gate()
  gate.update(1.55, angle, rate, 5.)
  assert not gate.allows_green(1.55)
  for i in range(10):
    gate.update(1.6+i*.05, 0., 0., 5.)
    assert not gate.allows_green(1.6+i*.05)
  gate.update(2.1, 0., 0., 5.)
  assert gate.allows_green(2.1)


def test_deadband_cannot_reacquire_but_does_not_chatter_after_straight():
  gate = GreenMotionGate()
  for i in range(30): gate.update(1+i*.05, 10., 0., 5.)
  assert not gate.allows_green(2.45)
  gate = straight_gate()
  gate.update(1.55, 10., 0., 5.)
  assert gate.allows_green(1.55)


def test_stale_future_invalid_nonfinite_and_repeated_context_cannot_grant_green():
  gate = straight_gate()
  assert not gate.allows_green(1.49)
  assert not gate.allows_green(1.701)
  for kwargs in [dict(valid=False), dict(angle=float('nan')), dict(speed=float('nan')), dict(timestamp=1.5)]:
    gate = straight_gate()
    sample = dict(timestamp=1.55, angle=0., rate=0., speed=5., valid=True)
    sample.update(kwargs)
    gate.update(**sample)
    assert not gate.allows_green(1.55)


def test_stationary_turned_wheel_can_observe_but_moving_turn_rejects():
  gate = GreenMotionGate()
  for i in range(11): gate.update(1+i*.05, -39.5, 0., 0.)
  assert gate.allows_green(1.5)
  gate.update(1.55, -39.5, 0., 1.)
  assert not gate.allows_green(1.55)


def test_motion_rejection_removes_confirmed_green_and_never_releases_red_guard():
  tracker = module.SignalTracker(conservative_green=True)
  guard = SignalStartGuard()
  box = [630, 230, 678, 242]
  for i in range(12):
    timestamp = 1+i*.2
    raw = 'red' if i < 4 else 'green'
    detection = dict(box=box, raw=raw, quality=.9, source='test')
    with patch.object(module, 'detect', return_value=[detection]):
      result = tracker.process(picture(raw), timestamp, allow_green=i < 4)
    assert all(t['state'] != 'green' for t in result['tracks'])
    tracks = [dict(box=t['box'], raw=t['evidence']['raw'] if t['age']==0 and t['observations']>=3 else 'unknown') for t in result['tracks']]
    obs = dict(timestamp=timestamp, frame_id=i, session='camera', tracks=tracks)
    decision = guard.update(timestamp, obs, enabled=True, valid=True, drive=True, gas=False, speed=0., departure=True)
    if i >= 3:
      assert decision.hold and not decision.released
  # Independently allow a fresh green streak, then reject it immediately.
  for i in range(12, 16):
    with patch.object(module, 'detect', return_value=[dict(box=box, raw='green', quality=.9, source='test')]):
      result = tracker.process(picture('green'), 1+i*.2)
  assert result['state'] == 'green'
  with patch.object(module, 'detect', return_value=[]):
    result = tracker.process(picture('unknown'), 4.2, allow_green=False)
  assert result['state'] != 'green'


def stable_pose_gate():
  gate = GreenPoseGate()
  for i in range(11): gate.update(1.02+i*.05, 1+i*.05, 0., .8)
  assert gate.allows_green(1.52)
  return gate


@pytest.mark.parametrize('yaw', [-13.7, -3., 3., 13.7])
def test_pose_rejects_actual_rotation_then_requires_new_stable_interval(yaw):
  gate = stable_pose_gate()
  gate.update(1.57, 1.55, yaw, .8)
  assert not gate.allows_green(1.57)
  for i in range(10):
    gate.update(1.62+i*.05, 1.6+i*.05, 0., .8)
    assert not gate.allows_green(1.62+i*.05)
  gate.update(2.12, 2.1, 0., .8)
  assert gate.allows_green(2.12)


@pytest.mark.parametrize('changes', [dict(valid=False), dict(sample=1.5), dict(sample=1.58),
                                    dict(sample=1.0), dict(yaw_std=3.), dict(yaw_std=-1.),
                                    dict(yaw=float('nan')), dict(arrival=1.5)])
def test_bad_or_frozen_pose_cannot_grant_green(changes):
  gate = stable_pose_gate()
  sample = dict(arrival=1.57, sample=1.55, yaw=0., yaw_std=.8, valid=True)
  sample.update(changes)
  gate.update(**sample)
  assert not gate.allows_green(1.57)


def test_pose_measurement_age_and_arrival_both_bound_permission():
  gate = stable_pose_gate()
  assert not gate.allows_green(1.51)  # Measurement exists but was not available yet.
  assert not gate.allows_green(1.701)  # Fresh arrival cannot refresh old measurement.
