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


def test_low_eps_tracking_error_recovers_authority():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  recovered_cap = governor.update(True, state(5.0, 85.0, 2.0, 10.0), 150.0, 150.0)
  assert 125.0 < recovered_cap < 127.0


def test_recovery_does_not_override_hard_protection():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  assert governor.update(True, state(5.0, 85.0, 2.0, 20.0), 150.0, 150.0) <= 88.0

  governor.update(True, state(5.0, 85.0, 2.0, 10.0, pressed=True), 150.0, 150.0)
  assert governor.update(True, state(5.0, 85.0, 2.0, 10.0), 150.0, 150.0) <= 50.0

  assert governor.update(True, state(5.0, 100.0, 2.0, 10.0), 220.0, 174.0) == 25.0


def test_standstill_does_not_recover_authority():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  assert governor.update(True, state(1.0, 85.0, 2.0, 10.0), 150.0, 150.0) == 50.0


def test_low_load_angle_cap_tapers_instead_of_cutting_authority():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  cap = governor.update(True, state(5.0, 100.0, 2.0, 10.0), 220.0, 174.0)
  assert 71.0 < cap < 72.0


def test_angle_cap_keeps_strict_limit_when_loaded_or_too_slow():
  governor = AngleTorqueGovernor(250.0, 25.0, 175.0)
  assert governor.update(True, state(5.0, 100.0, 2.0, 20.0), 220.0, 174.0) == 25.0
  assert governor.update(True, state(2.0, 100.0, 2.0, 10.0), 220.0, 174.0) == 25.0
