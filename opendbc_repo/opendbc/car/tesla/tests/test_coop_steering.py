from types import SimpleNamespace

import pytest

from opendbc.car.tesla.coop_steering import (
  CoopSteeringCarController,
  STEER_OVERRIDE_MIN_TORQUE,
)


class LinearVehicleModel:
  def get_steer_from_curvature(self, curvature, _speed, _roll):
    return curvature


def fake_state(speed=10.0, torque=0.0):
  return SimpleNamespace(out=SimpleNamespace(
    vEgo=speed,
    vEgoRaw=speed,
    steeringTorque=torque,
    steeringAngleDeg=0.0,
    steeringRateDeg=0.0,
  ))


def test_override_deadzone_does_not_move_the_wheel():
  controller = CoopSteeringCarController()
  target, torque = controller.compute_override_targets(10.0, STEER_OVERRIDE_MIN_TORQUE, LinearVehicleModel())

  assert target == 0.0
  assert torque == 0.0


def test_override_centers_using_its_holding_torque():
  controller = CoopSteeringCarController()
  controller.angle_override = 3.0
  target, torque = controller.compute_override_targets(10.0, 0.0, LinearVehicleModel())

  assert target == 0.0
  assert torque < 0.0
  assert controller.override_slew_step(target, torque) < 0.0


def test_planner_motion_is_not_double_counted_with_driver_override():
  assert CoopSteeringCarController.adjust_slew_for_planner(0.2, 0.1, 1.0) == pytest.approx(0.1)
  assert CoopSteeringCarController.adjust_slew_for_planner(0.2, 0.5, 1.0) == 0.0
  assert CoopSteeringCarController.adjust_slew_for_planner(0.2, -0.2, 1.0) > 0.2


def test_saturation_unwinds_only_the_excess_override():
  assert CoopSteeringCarController.unwind_on_saturation(2.0, 0.5) == pytest.approx(1.5)
  assert CoopSteeringCarController.unwind_on_saturation(2.0, -0.5) == pytest.approx(2.0)


def test_update_resets_override_when_lateral_control_disables(monkeypatch):
  monkeypatch.setattr("opendbc.car.tesla.coop_steering.apply_steer_angle_limits_vm", lambda angle, *_args: angle)
  controller = CoopSteeringCarController()
  controller.angle_override = 2.0

  result = controller.update(5.0, False, fake_state(), LinearVehicleModel())

  assert result.steeringAngleDeg == 0.0
  assert controller.angle_override == 0.0
