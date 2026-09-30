from types import SimpleNamespace as NS

import pytest

from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def setup_controller(monkeypatch, *, mode=0, angle=True, camera=True):
  settings = {"SteerHandoverMode": mode, "MaxAngleFrames": 89}
  params = NS(get_int=lambda key: settings.get(key, 0), get_bool=lambda key: False)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: params)
  # Keep actual angle limiting, legacy authority, setting polling and steering
  # CAN packing. Unrelated cluster/button paths are outside this fixture.
  monkeypatch.setattr(hyundaicanfd, "create_lfahda_cluster", lambda *a, **kw: [])
  monkeypatch.setattr(hyundaicanfd, "create_lfa_icon_non_camera_scc", lambda *a, **kw: [])
  monkeypatch.setattr(carcontroller.CarController, "create_button_messages", lambda *a, **kw: [])
  flags = HyundaiFlags.CANFD
  if angle:
    flags |= HyundaiFlags.ANGLE_CONTROL
  if camera:
    flags |= HyundaiFlags.CAMERA_SCC
  cp = structs.CarParams(carFingerprint=CAR.KIA_EV6, flags=int(flags), wheelbase=3.0, steerRatio=14.26)
  controller = carcontroller.CarController({Bus.pt: "hyundai_canfd_generated"}, cp)
  controller.lkas_max_torque = 25
  cs = NS(out=structs.CarState(canValid=True, steeringPressed=True, steeringTorque=300, vEgoRaw=15),
          modelV2=NS(frameId=1, position=NS(yStd=[0.1] * 33)), is_metric=True,
          adrv_0x161=None, mdps=None, steer_touch_2af=None, lfa_alt={}, lfa={})
  cc = structs.CarControl(enabled=True, latActive=True)
  cc.actuators.steeringAngleDeg = 0.5
  return controller, cs, cc, settings


def step(controller, cs, cc, *, fresh=True):
  if fresh:
    cs.modelV2.frameId = controller.frame // 5 + 1
  return controller.update(cc.as_reader(), cs, (controller.frame + 1) * 10_000_000)


@pytest.mark.parametrize("camera", [False, True])
def test_experimental_output_reaches_can_without_mutating_legacy(monkeypatch, camera):
  controller, cs, cc, _ = setup_controller(monkeypatch, mode=3, camera=camera)
  for _ in range(115):
    actuators, messages = step(controller, cs, cc)
  assert 70 < actuators.torqueOutputCan <= 80
  assert controller.lkas_max_torque == 25
  assert cs.out.steeringPressed
  if camera:
    data = next(data for address, data, bus in messages if address == 0xCB)
    assert abs(data[6] - actuators.torqueOutputCan) <= 0.5
  else:
    data = next(data for address, data, bus in messages if address == 0x12A)
    assert abs(data[12] - actuators.torqueOutputCan) <= 0.5
  assert actuators.steeringAngleDeg == pytest.approx(0.5)


@pytest.mark.parametrize("before,after", [(a, b) for a in range(4) for b in range(4) if a != b])
def test_all_runtime_mode_transitions_clear_only_experimental_history(monkeypatch, before, after):
  controller, cs, cc, settings = setup_controller(monkeypatch, mode=before)
  for _ in range(150):
    step(controller, cs, cc)
  assert controller.override_latched
  settings["SteerHandoverMode"] = after
  output, _ = step(controller, cs, cc)
  assert controller.steer_handover_mode == after
  assert controller.steer_handover.effort is None
  assert controller.override_latched
  assert controller.lkas_max_torque == output.torqueOutputCan == 25


def test_live_switch_is_polled_and_same_value_keeps_state(monkeypatch):
  controller, cs, cc, settings = setup_controller(monkeypatch)
  for _ in range(20):
    assert step(controller, cs, cc)[0].torqueOutputCan == 25
  settings["SteerHandoverMode"] = 1
  for _ in range(30):
    step(controller, cs, cc)
    assert controller.steer_handover_mode == 0
  step(controller, cs, cc)
  assert controller.steer_handover_mode == 1
  assert controller.steer_handover.effort is None
  for _ in range(99):
    actuators, _ = step(controller, cs, cc)
  assert actuators.torqueOutputCan > 70  # several unchanged polls did not reset
  settings["SteerHandoverMode"] = 0
  for _ in range(50):
    actuators, _ = step(controller, cs, cc)
  assert controller.steer_handover_mode == 0
  assert actuators.torqueOutputCan == 25


@pytest.mark.parametrize("bad_input", ["can", "temporary", "permanent", "model", "uncertain", "frozen"])
def test_controller_rejects_unhealthy_or_stale_input(monkeypatch, bad_input):
  controller, cs, cc, _ = setup_controller(monkeypatch, mode=3)
  for _ in range(115):
    step(controller, cs, cc)
  if bad_input == "can":
    cs.out.canValid = False
  elif bad_input == "temporary":
    cs.out.steerFaultTemporary = True
  elif bad_input == "permanent":
    cs.out.steerFaultPermanent = True
  elif bad_input == "model":
    cs.modelV2 = None
  elif bad_input == "uncertain":
    cs.modelV2.position.yStd = [0.5] * 33
  for _ in range(20):
    actuators, _ = step(controller, cs, cc, fresh=bad_input not in ("frozen", "model"))
  assert actuators.torqueOutputCan == 25


@pytest.mark.parametrize("camera", [False, True])
def test_torque_control_can_is_unchanged_in_all_modes(monkeypatch, camera):
  traces = []
  for mode in range(4):
    controller, cs, cc, _ = setup_controller(monkeypatch, mode=mode, angle=False, camera=camera)
    cc.actuators.torque = 0.1
    trace = [step(controller, cs, cc) for _ in range(120)]
    traces.append([(a.torqueOutputCan, a.steeringAngleDeg, messages) for a, messages in trace])
    assert controller.steer_handover.mode == 0
  assert all(trace == traces[0] for trace in traces[1:])


def test_disengagement_clears_authority_and_experimental_evidence(monkeypatch):
  controller, cs, cc, _ = setup_controller(monkeypatch, mode=3)
  for _ in range(115):
    step(controller, cs, cc)
  cc.latActive = False
  output, messages = step(controller, cs, cc)
  assert output.torqueOutputCan == 0
  assert next(data[6] for address, data, bus in messages if address == 0xCB) == 0
  assert controller.steer_handover.effort is None


@pytest.mark.parametrize("camera", [False, True])
def test_all_modes_preserve_angle_commands_during_release_and_model_changes(monkeypatch, camera):
  angles = []
  for mode in range(4):
    controller, cs, cc, _ = setup_controller(monkeypatch, mode=mode, camera=camera)
    trace = []
    for tick in range(300):
      cc.actuators.steeringAngleDeg = 30 if tick < 150 else -10
      cs.out.steeringAngleDeg = 25 if tick < 120 else 20
      cs.out.steeringTorque = 350 if tick < 100 else 0
      cs.out.steeringPressed = tick < 100
      output, _ = step(controller, cs, cc)
      trace.append(output.steeringAngleDeg)
    angles.append(trace)
  assert all(trace == angles[0] for trace in angles[1:])


@pytest.mark.parametrize("camera", [False, True])
def test_recovery_selected_total_cap_reaches_both_can_paths(monkeypatch, camera):
  controller, cs, cc, _ = setup_controller(monkeypatch, mode=2, camera=camera)
  cc.actuators.steeringAngleDeg = 12
  for _ in range(100):
    step(controller, cs, cc)
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0
  for _ in range(10):
    step(controller, cs, cc)
  assert controller.steer_handover.state == "recover"
  # Force the independent legacy recovery ahead: it cannot bypass the helper.
  controller.lkas_max_torque = 250
  output, messages = step(controller, cs, cc)
  assert 25 < output.torqueOutputCan < 80
  address, byte = (0xCB, 6) if camera else (0x12A, 12)
  data = next(data for addr, data, _ in messages if addr == address)
  assert abs(data[byte] - output.torqueOutputCan) <= 0.5
