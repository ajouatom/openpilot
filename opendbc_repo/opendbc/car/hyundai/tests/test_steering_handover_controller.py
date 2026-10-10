from types import SimpleNamespace as NS

import pytest

from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller, hyundaicanfd
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def setup_controller(monkeypatch, *, mode=None, angle=True, camera=True):
  settings = {"MaxAngleFrames": 89}
  if mode is not None:
    settings["SteerHandoverMode"] = mode  # leftover value on an upgraded device

  def get_int(key):
    assert key != "SteerHandoverMode", "retired setting must not be read"
    return settings.get(key, 0)

  params = NS(get_int=get_int, get_bool=lambda key: False)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  monkeypatch.setattr(hyundaicanfd, "Params", lambda: params)
  # Keep actual angle limiting, legacy authority and steering
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


@pytest.mark.parametrize("saved", [None, 0, 1, 2, 3, -1, 99])
def test_combined_recovery_is_standard_regardless_of_retired_setting(monkeypatch, saved):
  controller, cs, cc, _ = setup_controller(monkeypatch, mode=saved)
  for _ in range(115):
    output, _ = step(controller, cs, cc)
  assert controller.steer_handover.mode == 3
  assert 70 < output.torqueOutputCan <= 80
  assert controller.lkas_max_torque == 25


def test_retired_setting_changes_cannot_reset_recovery(monkeypatch):
  controller, cs, cc, settings = setup_controller(monkeypatch)
  for _ in range(100):
    step(controller, cs, cc)
  assert controller.steer_handover.state == "offering"
  offer_since = controller.steer_handover.offer_since
  for saved in (0, 1, 2, 3):
    settings["SteerHandoverMode"] = saved
    for _ in range(5):
      output, _ = step(controller, cs, cc)
    assert controller.steer_handover.mode == 3
    assert controller.steer_handover.offer_since == offer_since
    assert controller.steer_handover.effort is not None
  # The unchanged no-response timeout may start gradual withdrawal here.
  assert output.torqueOutputCan > 25


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
def test_torque_control_does_not_call_handover(monkeypatch, camera):
  def unexpected_handover(*args, **kwargs):
    pytest.fail("torque-control platform must not use angle handover")

  traces = []
  for mode in range(4):
    controller, cs, cc, _ = setup_controller(monkeypatch, mode=mode, angle=False, camera=camera)
    monkeypatch.setattr(controller.steer_handover, "update", unexpected_handover)
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
def test_combined_recovery_preserves_legacy_angle_commands(monkeypatch, camera):
  angles = []
  for legacy in (False, True):
    controller, cs, cc, _ = setup_controller(monkeypatch, camera=camera)
    if legacy:
      monkeypatch.setattr(controller.steer_handover, "update", lambda **kw: kw["baseline"])
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
  controller, cs, cc, _ = setup_controller(monkeypatch, camera=camera)
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


@pytest.mark.parametrize('angle', [False, True])
@pytest.mark.parametrize('previously_active', [False, True])
def test_missing_camera_template_cannot_accumulate_first_transmitted_command(monkeypatch, angle, previously_active):
  controller, cs, cc, settings = setup_controller(monkeypatch, angle=angle)
  settings.update(CustomSteerMax=409, CustomSteerDeltaUp=3)
  cs.out.steeringPressed = False
  cs.out.steeringTorque = 0
  cs.out.steeringAngleDeg = -2.2
  cc.actuators.torque = 1.0
  cc.actuators.steeringAngleDeg = 30
  if previously_active:
    for _ in range(150):
      step(controller, cs, cc)
  attr = 'lfa_alt' if angle else 'lfa'
  setattr(cs, attr, None)
  address = 0xCB if angle else 0x12A
  for _ in range(200):
    output, messages = step(controller, cs, cc)
    assert not any(addr == address for addr, _, _ in messages)
    assert output.torqueOutputCan == 0
    assert controller.apply_torque_last == 0
    assert controller.apply_angle_last == pytest.approx(-2.2)
    assert controller.lkas_max_torque == 0
    assert cc.latActive  # local guard must not mutate the subscription
  setattr(cs, attr, {})
  output, messages = step(controller, cs, cc)
  data = next(data for addr, data, _ in messages if addr == address)
  if angle:
    assert output.torqueOutputCan == controller.params.ANGLE_MIN_TORQUE
    assert abs(output.steeringAngleDeg - cs.out.steeringAngleDeg) <= 1.0
    assert data[6] == controller.params.ANGLE_MIN_TORQUE
  else:
    assert output.torqueOutputCan == 3
    assert ((int.from_bytes(data, 'little') >> 41) & 0x7ff) - 1024 == 3
