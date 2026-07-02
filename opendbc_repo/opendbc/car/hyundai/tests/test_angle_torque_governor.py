from types import SimpleNamespace

from opendbc.car.hyundai.angle_torque_governor import AngleTorqueGovernor


def state(speed, angle, rate, eps, pressed=False):
  return SimpleNamespace(vEgo=speed, steeringAngleDeg=angle, steeringRateDeg=rate,
                         steeringTorqueEps=eps, steeringPressed=pressed)


def test_normal_authority_and_inactive_reset():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  assert governor.update(True, state(15.0, 10.0, 2.0, 5.0), 10.0, 10.0) == 250.0
  governor.driver_unwind_frames = 10
  assert governor.update(False, state(0.0, 0.0, 0.0, 0.0), 0.0, 0.0) == 250.0
  assert governor.driver_unwind_frames == 0


def test_overlapping_caps_keep_strictest_limit():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  assert governor.update(True, state(.5, 80.0, 2.0, 25.0), 110.0, 90.0) == 40.0
  assert governor.update(True, state(2.0, 80.0, 100.0, 25.0), 110.0, 90.0) <= 80.0


def test_post_driver_unwind_and_angle_cap():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  governor.update(True, state(4.0, 80.0, 2.0, 5.0, pressed=True), 80.0, 80.0)
  assert governor.driver_unwind_frames == 150
  assert governor.update(True, state(4.0, 80.0, 2.0, 25.0), 110.0, 90.0) <= 55.0
  assert governor.driver_unwind_frames == 149
  assert governor.update(True, state(5.0, 100.0, 2.0, 25.0), 220.0, 174.0) == 25.0
