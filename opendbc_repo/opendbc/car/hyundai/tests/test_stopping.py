from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.hyundaicanfd import create_acc_control, create_acc_control_scc2
from opendbc.car.hyundai.stopping import CanfdStopping, StopPhase, DT, ENTRY_SPEED, STOP_LOWER_BAND
from opendbc.car.hyundai.values import CAR, HyundaiFlags


def make_car_controller(monkeypatch, *, canfd=True, longitudinal=True):
  def get_bool(key):
    assert key != "CanfdStopRetry", "Removed stop retry setting must never be read"
    return False
  params = SimpleNamespace(get_bool=get_bool, get_int=lambda key: 0)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  cp = structs.CarParams(carFingerprint=CAR.KIA_EV6, flags=int(HyundaiFlags.CANFD) if canfd else 0,
                         openpilotLongitudinalControl=longitudinal, stoppingDecelRate=0.8)
  return carcontroller.CarController({Bus.pt: "hyundai_canfd_generated"}, cp)


@pytest.mark.parametrize("canfd", [False, True])
@pytest.mark.parametrize("longitudinal", [False, True])
def test_controller_enables_canfd_stopping_by_vehicle_scope(monkeypatch, canfd, longitudinal):
  controller = make_car_controller(monkeypatch, canfd=canfd, longitudinal=longitudinal)
  assert isinstance(controller.canfd_stopping, CanfdStopping) == (canfd and longitudinal)


def step(controller, **overrides):
  args = dict(active=True, requested=True, speed=0.3, held=False, accel=-0.5,
              previous_value=-0.5, jerk_u=2.0, jerk_l=1.0)
  args.update(overrides)
  return controller.update(**args)


def test_normal_stop_keeps_request_asserted_without_toggle():
  controller = CanfdStopping()
  for tick in range(150):
    cmd = step(controller, speed=max(0, 0.6 - tick * DT * 0.4), held=tick >= 80)
    assert (cmd.stop_req, cmd.raw, cmd.value) == (1, -0.5, -0.5)
    assert cmd.lower == STOP_LOWER_BAND
  assert controller.phase == StopPhase.held
  assert not controller.retried


@pytest.mark.parametrize('initial', [-1.0, -0.5, -0.3, 0.0, 0.2])
@pytest.mark.parametrize('jerk_u, jerk_l', [(2.0, 1.0), (0.4, 0.6)])
def test_stop_acceleration_converges_from_both_sides_without_overshoot(initial, jerk_u, jerk_l):
  controller = CanfdStopping(stopping_rate=0.8)
  commands = [step(controller, speed=0.0, held=True, previous_value=initial,
                   accel=-2.0, jerk_u=jerk_u, jerk_l=jerk_l) for _ in range(160)]
  assert commands[0].raw == commands[0].value == min(initial, 0.0)
  values = [c.value for c in commands]
  assert all(c.stop_req == 1 and c.raw == c.value for c in commands)
  lo, hi = sorted((min(initial, 0.0), -0.5))
  assert all(lo - 1e-9 <= v <= hi + 1e-9 for v in values)
  for previous, current in zip(values, values[1:], strict=False):
    assert abs(current + 0.5) <= abs(previous + 0.5) + 1e-9
    assert -min(0.8, jerk_l) * DT - 1e-9 <= current - previous <= min(0.8, jerk_u) * DT + 1e-9
  assert commands[-1].raw == commands[-1].value == pytest.approx(-0.5)


def test_creep_releases_reasserts_once_then_retains_deceleration():
  controller = CanfdStopping()
  phases = []
  commands = []
  for _ in range(400):
    cmd = step(controller)
    if not phases or phases[-1] != controller.phase:
      phases.append(controller.phase)
    commands.append(cmd)
  assert phases == [StopPhase.request, StopPhase.release, StopPhase.retry, StopPhase.fallback]
  assert commands[-1].stop_req == 0
  assert commands[-1].raw == commands[-1].value == pytest.approx(-0.5)
  assert all(cmd.raw <= 0 and cmd.value <= 0 for cmd in commands)
  # StopReq stays released through persistent motion, regardless of elapsed time.
  for _ in range(1000):
    assert step(controller).stop_req == 0
  for _ in range(15):
    cmd = step(controller, speed=0)
  assert controller.phase == StopPhase.held
  assert (cmd.stop_req, cmd.raw, cmd.value) == (1, -0.5, -0.5)


def test_recovery_value_continues_actual_stop_output_and_respects_jerk():
  controller = CanfdStopping()
  previous = -2.0
  while controller.phase != StopPhase.release:
    cmd = step(controller, previous_value=-2.0, accel=-2.0)
    if controller.phase != StopPhase.release:
      previous = cmd.value
  assert cmd.raw == -2.0
  assert cmd.value == pytest.approx(previous - DT)
  for i in range(1, 20):
    cmd = step(controller, previous_value=-2.0, accel=-2.0)
    assert cmd.value == pytest.approx(max(-2.0, previous - (i + 1) * DT))


def test_stop_during_release_reasserts_immediately_when_held():
  controller = CanfdStopping()
  while controller.phase != StopPhase.release:
    step(controller)
  cmd = step(controller, speed=0.02, held=True)
  assert (cmd.stop_req, cmd.raw, cmd.value) == (1, -0.5, -0.5)
  assert controller.phase == StopPhase.held


def test_hold_flag_does_not_hide_motion_and_dropout_does_not_release():
  controller = CanfdStopping()
  for _ in range(50):
    cmd = step(controller, speed=0.02, held=True)
  for _ in range(50):
    cmd = step(controller, speed=0.02, held=False)
    assert cmd.stop_req == 1
  for _ in range(12):
    cmd = step(controller, speed=0.3, held=True)
  assert cmd.stop_req == 0
  assert controller.phase == StopPhase.release


def test_single_speed_spike_does_not_release_hold():
  controller = CanfdStopping()
  step(controller, speed=0.02, held=True)
  assert step(controller, speed=0.3, held=True).stop_req == 1
  assert step(controller, speed=0.02, held=True).stop_req == 1


def test_approach_keeps_negative_acceleration_until_low_speed():
  controller = CanfdStopping()
  for _ in range(100):
    cmd = step(controller, speed=4.0, accel=-2.0, previous_value=-2.0)
    assert cmd.stop_req == 0
    assert cmd.raw == cmd.value == -2.0
  cmd = step(controller, speed=ENTRY_SPEED)
  assert (cmd.stop_req, cmd.raw, cmd.value) == (1, -2.0, -2.0)
  assert step(controller, speed=ENTRY_SPEED).value == pytest.approx(-2.0 + 0.8 * DT)


def test_request_speed_excursion_releases_without_waiting():
  controller = CanfdStopping()
  step(controller)
  assert step(controller, speed=ENTRY_SPEED + 0.1).stop_req == 0


def test_near_zero_speed_is_given_time_to_settle_but_cannot_creep_forever():
  controller = CanfdStopping()
  for _ in range(149):
    assert step(controller, speed=0.07).stop_req == 1
  assert step(controller, speed=0.07).stop_req == 0
  assert controller.reason == "request_timeout"


@pytest.mark.parametrize("reason", ["inactive", "departing"])
def test_cancel_resets_recovery_and_next_episode(reason):
  controller = CanfdStopping()
  for _ in range(200):
    step(controller)
  assert controller.retried
  assert step(controller, active=reason != "inactive", requested=reason != "departing") is None
  assert controller.phase == StopPhase.idle
  assert not controller.retried
  assert step(controller).stop_req == 1


def make_cs():
  return SimpleNamespace(
    scc_control={"ACC_ObjRelSpd": 0.0, "InfoDisplay": 5, "ZEROS_7": 1, "AccelLimitBandLower": 1.26},
    canfdSccHoldActive=False, softHoldActive=0, paddle_button_prev=0,
    out=SimpleNamespace(aEgo=0.0, vEgo=0.3, vEgoRaw=0.3,
                        wheelSpeeds=SimpleNamespace(fl=0.3, fr=0.3, rl=0.3, rr=0.3),
                        canValid=True, gearShifter="drive", brakePressed=False, gasPressed=False,
                        brakeHoldActive=False, parkingBrake=False, cruiseState=SimpleNamespace(available=True, standstill=True)),
  )


def send(camera, controller, CS, *, enabled=True, override=False, stopping=True, accel=-0.5,
         previous_value=-0.5, previous_target=-0.5):
  packer = CANPacker("hyundai_canfd_generated")
  can = SimpleNamespace(ECAN=0)
  hud = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
  jerk = SimpleNamespace(carrot_cruise=0, jerk_u=2.0, jerk_l=1.0)
  if camera:
    msg, actual_value = create_acc_control_scc2(packer, can, enabled, previous_value, accel, stopping, override, 30.0, hud, jerk, CS, controller)
  else:
    msg, actual_value = create_acc_control(packer, can, enabled, previous_target, accel, stopping, override, 30.0, hud, 2.0, 1.0, CS,
                                         controller, accel_value_last=previous_value)
  if msg is None:
    return None
  parser = CANParser("hyundai_canfd_generated", [("SCC_CONTROL", 50)], 0)
  parser.update([1_000_000_000, [msg]])
  values = parser.vl["SCC_CONTROL"]
  assert values["aReqValue"] == pytest.approx(actual_value, abs=0.0051)  # CAN resolution is 0.01 m/s^2
  # Independent little-endian positions from the supplied OEM SCC definition.
  packed = int.from_bytes(msg[1], "little")
  if controller is not None:
    assert msg[1][7] == 0
    assert values["InfoDisplay"] == 0
  assert packed >> 74 & 7 == values["InfoDisplay"]
  assert packed >> 184 & 3 == values["StopReq"]
  assert (packed >> 176 & 63) * 0.02 == pytest.approx(values["AccelLimitBandLower"])
  return values


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("initial", [-1.0, -0.3])
def test_packed_stop_starts_from_previous_output(camera, initial):
  cs = make_cs()
  controller = CanfdStopping()
  # Ordinary braking can have a raw target different from its limited output.
  approach = send(camera, controller, cs, stopping=False, accel=-2.0, previous_value=initial, previous_target=initial)
  assert approach["aReqRaw"] != approach["aReqValue"]
  first = send(camera, controller, cs, accel=-2.0, previous_value=approach["aReqValue"], previous_target=approach["aReqRaw"])
  assert first["StopReq"] == 1
  assert first["aReqRaw"] == first["aReqValue"] == approach["aReqValue"]

  # Confirm hold to isolate convergence from the separate persistent-motion retry.
  cs.canfdSccHoldActive = True
  cs.out.vEgo = cs.out.vEgoRaw = 0.0
  cs.out.wheelSpeeds = SimpleNamespace(fl=0., fr=0., rl=0., rr=0.)
  previous = first["aReqValue"]
  for _ in range(100):
    values = send(camera, controller, cs, accel=-2.0, previous_value=previous, previous_target=-2.0)
    assert values["StopReq"] == 1
    assert values["aReqRaw"] == values["aReqValue"]
    assert abs(values["aReqValue"] + 0.5) <= abs(previous + 0.5) + 1e-9
    assert abs(values["aReqValue"] - previous) <= 0.02 + 1e-9  # 0.016 plus CAN quantization
    previous = values["aReqValue"]
  assert previous == pytest.approx(-0.5)


@pytest.mark.parametrize("camera", [True, False])
def test_packed_retry_restarts_convergence_from_release_output(camera):
  controller = CanfdStopping()
  cs = make_cs()
  previous = -1.0
  for _ in range(150):
    old_phase = controller.phase
    values = send(camera, controller, cs, accel=-1.0, previous_value=previous, previous_target=-1.0)
    if controller.phase == StopPhase.retry:
      assert old_phase == StopPhase.release
      assert values["StopReq"] == 1
      assert values["aReqRaw"] == values["aReqValue"] == previous
      assert previous < -0.5
      following = send(camera, controller, cs, accel=-1.0, previous_value=previous, previous_target=-1.0)
      assert following["StopReq"] == 1
      assert previous < following["aReqValue"] <= -0.5
      break
    previous = values["aReqValue"]
  else:
    pytest.fail("persistent motion did not trigger the retained retry")


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("stock_info", [0, 4, 5])
def test_base_packet_builder_without_controller_preserves_legacy_fields(camera, stock_info):
  cs = make_cs()
  cs.scc_control["InfoDisplay"] = stock_info
  # Direct use of the base packet builder does not create a retry controller.
  for _ in range(200):
    values = send(camera, None, cs)
    assert values["StopReq"] == 1
    assert values["aReqRaw"] == pytest.approx(-0.5)
    assert values["aReqValue"] == pytest.approx(-0.5)
    assert values["AccelLimitBandLower"] == pytest.approx(0)
    assert values["InfoDisplay"] == (5 if camera and stock_info == 5 else 4)
    assert values["ZEROS_7"] == (1 if camera else 0)


@pytest.mark.parametrize("camera", [True, False])
def test_default_controller_sends_stop_commands_and_retains_retry_progress(camera, monkeypatch):
  controller = make_car_controller(monkeypatch)
  cs = make_cs()
  values = send(camera, controller.canfd_stopping, cs)
  assert values["StopReq"] == 1
  assert values["aReqRaw"] == pytest.approx(-0.5)
  assert values["InfoDisplay"] == values["ZEROS_7"] == 0
  assert values["AccelLimitBandLower"] == pytest.approx(STOP_LOWER_BAND)
  initial = controller.canfd_stopping
  for _ in range(170):
    send(camera, controller.canfd_stopping, cs)
  assert controller.canfd_stopping is initial
  assert initial.phase == StopPhase.fallback


@pytest.mark.parametrize("camera", [True, False])
def test_packed_scc_stop_acceleration_zero_display_fields_and_fixed_band(camera):
  controller = CanfdStopping()
  values = send(camera, controller, make_cs())
  assert values["ACCMode"] == 1
  assert values["StopReq"] == 1
  assert values["aReqRaw"] == pytest.approx(-0.5)
  assert values["aReqValue"] == pytest.approx(-0.5)
  assert values["InfoDisplay"] == values["ZEROS_7"] == 0
  assert values["AccelLimitBandLower"] == pytest.approx(STOP_LOWER_BAND)


@pytest.mark.parametrize("camera", [True, False])
def test_packed_recovery_and_reentry(camera):
  controller = CanfdStopping()
  cs = make_cs()
  phases = set()
  for _ in range(170):
    values = send(camera, controller, cs)
    phases.add(controller.phase)
    if controller.phase in (StopPhase.release, StopPhase.fallback):
      assert values["StopReq"] == 0
      assert values["aReqRaw"] == pytest.approx(-0.5)
      assert values["aReqValue"] < 0
    else:
      assert values["StopReq"] == 1
      assert values["aReqRaw"] == pytest.approx(-0.5)
      assert values["aReqValue"] == pytest.approx(-0.5)
  assert phases == {StopPhase.request, StopPhase.release, StopPhase.retry, StopPhase.fallback}


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("block", ["brakePressed", "gasPressed", "brakeHoldActive", "parkingBrake", "canValid", "gearShifter",
                                   "unavailable", "disabled", "override", "nan_speed", "wheel_motion"])
def test_interlocks_and_motion_sources(camera, block):
  controller = CanfdStopping()
  cs = make_cs()
  for _ in range(40):
    send(camera, controller, cs)
  assert controller.phase == StopPhase.release
  if block in ("brakePressed", "gasPressed", "brakeHoldActive", "parkingBrake"):
    setattr(cs.out, block, True)
  elif block == "canValid":
    cs.out.canValid = False
  elif block == "gearShifter":
    cs.out.gearShifter = "reverse"
  elif block == "unavailable":
    cs.out.cruiseState.available = False
  elif block == "nan_speed":
    cs.out.vEgo = float("nan")
  elif block == "wheel_motion":
    cs.out.vEgo = cs.out.vEgoRaw = 0.0
    cs.canfdSccHoldActive = True
  values = send(camera, controller, cs, enabled=block != "disabled", override=block == "override")
  assert values["StopReq"] == 0
  if block == "wheel_motion":
    assert controller.phase == StopPhase.release
  else:
    assert values["aReqRaw"] == pytest.approx(0)
    assert values["aReqValue"] == pytest.approx(0)
    assert controller.phase == StopPhase.idle


def test_missing_stock_message_resets_camera_experiment():
  cs = make_cs()
  controller = CanfdStopping()
  for _ in range(40):
    send(True, controller, cs)
  cs.scc_control = None
  assert send(True, controller, cs) is None
  assert controller.phase == StopPhase.idle


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("release_after", [None, 1, 10, 26])
def test_soft_hold_prepares_two_negative_scc_frames_before_stop_req(camera, release_after):
  cs = make_cs()
  cs.softHoldActive = 1
  cs.out.brakePressed = True
  cs.out.vEgo = cs.out.vEgoRaw = 0.0
  cs.out.wheelSpeeds = SimpleNamespace(fl=0., fr=0., rl=0., rr=0.)
  controller = CanfdStopping()
  negative_frames = 0
  for frame in range(50):
    if release_after is not None and frame >= release_after:
      cs.out.brakePressed = False
      cs.softHoldActive = 2
    # While braking, controlsd's requested acceleration can still be zero.
    values = send(camera, controller, cs, enabled=False, stopping=False, accel=0.)
    assert values['ACCMode'] == 1
    if values['StopReq']:
      assert negative_frames >= 2
      assert values['aReqRaw'] == values['aReqValue'] == pytest.approx(-.5)
      break
    assert values['aReqRaw'] == pytest.approx(-.5)
    assert values['aReqValue'] < 0.
    negative_frames = negative_frames + 1 if values['aReqValue'] <= -.5 + 1e-6 else 0
  else:
    pytest.fail('soft hold never completed preparation')
  assert controller.phase == StopPhase.request
  for _ in range(12):
    assert send(camera, controller, cs, enabled=False, stopping=False, accel=0.)['StopReq'] == 1
  assert controller.phase == StopPhase.held


@pytest.mark.parametrize('camera', [True, False])
@pytest.mark.parametrize('block', ['gasPressed', 'brakeHoldActive', 'parkingBrake', 'canValid', 'gearShifter', 'moving'])
def test_soft_hold_brake_exception_preserves_other_interlocks(camera, block):
  cs = make_cs()
  cs.softHoldActive = 1
  cs.out.brakePressed = True
  cs.out.vEgo = cs.out.vEgoRaw = 0.0
  cs.out.wheelSpeeds = SimpleNamespace(fl=0., fr=0., rl=0., rr=0.)
  controller = CanfdStopping()
  send(camera, controller, cs, enabled=False, stopping=False, accel=0.)
  assert controller.phase == StopPhase.prepare
  if block == 'canValid':
    cs.out.canValid = False
  elif block == 'gearShifter':
    cs.out.gearShifter = 'reverse'
  elif block == 'moving':
    cs.out.wheelSpeeds.fl = .11
  else:
    setattr(cs.out, block, True)
  values = send(camera, controller, cs, enabled=False, stopping=False, accel=0.)
  assert values['StopReq'] == 0
  assert values['aReqRaw'] == values['aReqValue'] == 0.
  assert controller.phase == StopPhase.idle
  assert controller.prepare_cycles == 0


def test_soft_hold_does_not_release_confirmed_existing_hold_to_prepare():
  controller = CanfdStopping()
  command = step(controller, speed=0., held=True, soft_hold=True)
  assert command.stop_req == 1
  assert controller.phase == StopPhase.held
