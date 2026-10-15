from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs, apply_steer_angle_limits_vm
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR, CarControllerParams


@pytest.mark.parametrize("speed_kph", [0.0, 24.9, 25.0, 25.1, 36.0])
@pytest.mark.parametrize("cooperative", [False, True])
@pytest.mark.parametrize("active,override", [(False, False), (True, False), (True, True)])
def test_feedback_matches_final_steering_request_and_holds_between_transmissions(speed_kph, cooperative, active, override):
  cp = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  controller = CarController({Bus.party: "tesla_model3_party"}, cp)
  controller.coop_steering = cooperative
  state = SimpleNamespace(
    out=structs.CarState(vEgo=speed_kph / 3.6, vEgoRaw=speed_kph / 3.6,
                        steeringAngleDeg=12.3, steeringRateDeg=80.0, steeringTorque=1.0),
    steering_disengage=override, tesla_manual_speed_adjustment_counter=0,
    tesla_speed_auto_resume_gesture_counter=0, tesla_speed_limit_target_nanos=0,
    das_accCancel=False, cruise_override=False, das_control={"DAS_controlCounter": 0},
  )
  command = structs.CarControl(latActive=active)
  command.actuators.steeringAngleDeg = 30.0
  parser = CANParser("tesla_model3_party", [("DAS_steeringControl", 50)], 0)

  output, messages = controller.update(command.as_reader(), state, 0)
  parser.update([(0, messages)])
  request = parser.vl["DAS_steeringControl"]
  assert output.steeringAngleDeg == pytest.approx(-request["DAS_steeringAngleRequest"], abs=0.051)
  assert bool(request["DAS_steeringControlType"]) == (active and not override)
  if cooperative and (not active or override):
    assert output.steeringAngleDeg == pytest.approx(state.out.steeringAngleDeg)
  if cooperative and active and not override and speed_kph <= 25.0:
    assert controller.coop_steer.angle_override == 0.0
    assert output.steeringAngleDeg == pytest.approx(controller.apply_angle_last)

  command.actuators.steeringAngleDeg = -30.0
  next_output, next_messages = controller.update(command.as_reader(), state, 10_000_000)
  assert all(message[0] != 0x488 for message in next_messages)
  assert next_output.steeringAngleDeg == output.steeringAngleDeg


@pytest.mark.parametrize("torque", [-1.5, 1.5])
def test_speed_transitions_preserve_active_steering_and_final_request_limits(torque):
  cp = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  controller = CarController({Bus.party: "tesla_model3_party"}, cp)
  state = SimpleNamespace(
    out=structs.CarState(steeringTorque=torque),
    steering_disengage=False, tesla_manual_speed_adjustment_counter=0,
    tesla_speed_auto_resume_gesture_counter=0, tesla_speed_limit_target_nanos=0,
    das_accCancel=False, cruise_override=False, das_control={"DAS_controlCounter": 0},
  )
  command = structs.CarControl(latActive=True)
  command.actuators.steeringAngleDeg = 30.0
  previous_output = 0.0
  parser = CANParser("tesla_model3_party", [("DAS_steeringControl", 50)], 0)

  for speed_kph in (30.0, 25.1, 25.0, 24.9, 10.0, 0.0, 24.9, 25.0, 25.1, 30.0):
    state.out.vEgo = state.out.vEgoRaw = speed_kph / 3.6
    for _ in range(40):
      state.out.steeringAngleDeg = previous_output
      output, messages = controller.update(command.as_reader(), state, controller.frame * 10_000_000)
      if any(message[0] == 0x488 for message in messages):
        parser.update([(0, messages)])
        request = parser.vl["DAS_steeringControl"]
        assert request["DAS_steeringControlType"] == 1
        assert output.steeringAngleDeg == pytest.approx(-request["DAS_steeringAngleRequest"], abs=0.051)
        limited = apply_steer_angle_limits_vm(output.steeringAngleDeg, previous_output, state.out.vEgoRaw,
                                              state.out.steeringAngleDeg, True, CarControllerParams, controller.VM)
        assert output.steeringAngleDeg == pytest.approx(limited, abs=1e-5)
        if speed_kph <= 25.0:
          assert controller.coop_steer.angle_override == 0.0
      else:
        assert output.steeringAngleDeg == previous_output
      previous_output = output.steeringAngleDeg


def test_low_speed_engagement_uses_last_inactive_request_not_phase_lead():
  cp = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_3)
  controller = CarController({Bus.party: "tesla_model3_party"}, cp)
  state = SimpleNamespace(
    out=structs.CarState(vEgo=25.0 / 3.6, vEgoRaw=25.0 / 3.6,
                        steeringAngleDeg=12.3, steeringRateDeg=80.0, steeringTorque=1.0),
    steering_disengage=False, tesla_manual_speed_adjustment_counter=0,
    tesla_speed_auto_resume_gesture_counter=0, tesla_speed_limit_target_nanos=0,
    das_accCancel=False, cruise_override=False, das_control={"DAS_controlCounter": 0},
  )
  command = structs.CarControl()
  command.actuators.steeringAngleDeg = 30.0
  inactive, _ = controller.update(command.as_reader(), state, 0)
  assert controller.coop_steer.coop_apply_angle_sat_last > inactive.steeringAngleDeg

  command.latActive = True
  controller.update(command.as_reader(), state, 10_000_000)
  output, _ = controller.update(command.as_reader(), state, 20_000_000)
  expected = apply_steer_angle_limits_vm(command.actuators.steeringAngleDeg, inactive.steeringAngleDeg,
                                        state.out.vEgoRaw, state.out.steeringAngleDeg, True, CarControllerParams, controller.VM)
  assert output.steeringAngleDeg == pytest.approx(expected)
  assert controller.coop_steer.angle_override == 0.0
