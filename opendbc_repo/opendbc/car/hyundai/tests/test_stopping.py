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
  args = dict(active=True, requested=True, speed=0.3, a_ego=0.0, held=False, accel=-0.5,
              value=-0.5, previous_value=-0.5, jerk_u=2.0, jerk_l=1.0)
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


@pytest.mark.parametrize('raw,value', [(-1., -.7), (-.5, -.3), (-.3, -.8), (0., 0.)])
def test_stop_passes_normal_requests_without_fixed_convergence(raw, value):
  controller = CanfdStopping()
  for _ in range(160):
    cmd = step(controller, speed=0., held=True, accel=raw, value=value, previous_value=value)
    assert (cmd.stop_req, cmd.raw, cmd.value) == (1, raw, value)


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
  previous = -.3
  while controller.phase != StopPhase.release:
    cmd = step(controller, previous_value=previous, accel=-2., value=-.3)
    if controller.phase != StopPhase.release:
      previous = cmd.value
  assert cmd.raw == -2.
  assert cmd.value == pytest.approx(previous - DT)
  for _ in range(20):
    previous = cmd.value
    cmd = step(controller, previous_value=previous, accel=-2., value=-.3)
    assert cmd.value == pytest.approx(previous - DT)


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
  for _ in range(16):
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
    cmd = step(controller, speed=4.0, accel=-2.0, value=-2.0, previous_value=-2.0)
    assert cmd.stop_req == 0
    assert cmd.raw == cmd.value == -2.0
  cmd = step(controller, speed=ENTRY_SPEED, accel=-2., value=-2., previous_value=-2.)
  assert (cmd.stop_req, cmd.raw, cmd.value) == (1, -2., -2.)
  assert step(controller, speed=ENTRY_SPEED, accel=-2., value=-2., previous_value=-2.).value == -2.


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


def test_gentle_deceleration_does_not_spend_retry_on_distance_or_timeout():
  controller = CanfdStopping()
  # More than 0.5 m and 3 s, with less than 0.03 m/s reduction per 0.3 s.
  for tick in range(500):
    assert step(controller, speed=.65 - tick * DT * .04, a_ego=-.04).stop_req == 1
  assert controller.phase == StopPhase.request
  assert not controller.retried


def test_acceleration_rising_toward_zero_during_settling_does_not_retry():
  controller = CanfdStopping()
  speed = .65
  for tick in range(220):
    a_ego = -.3 + .25 * min(tick / 150, 1.)
    speed = max(0., speed + a_ego * DT)
    assert step(controller, speed=speed, a_ego=a_ego).stop_req == 1
  assert not controller.retried


@pytest.mark.parametrize('start_speed', [.03, .3])
def test_confirmed_speed_rebound_retries_with_positive_measured_acceleration(start_speed):
  controller = CanfdStopping()
  for tick in range(20):
    speed = start_speed + (.1 - tick * .005 if start_speed > .05 else 0.)
    step(controller, speed=speed, a_ego=-.04)
  for tick in range(35):
    cmd = step(controller, speed=start_speed + tick * DT * .15, a_ego=.15)
    if cmd.stop_req == 0:
      break
  assert controller.phase == StopPhase.release
  assert controller.reason == "speed_rebound"
  assert cmd.raw == cmd.value == -.5


def test_short_acceleration_spike_during_deceleration_does_not_release():
  controller = CanfdStopping()
  for _ in range(160):
    step(controller, speed=.3, a_ego=-.05)
  assert step(controller, speed=.34, a_ego=.2).stop_req == 1
  assert step(controller, speed=.29, a_ego=-.05).stop_req == 1
  assert not controller.retried


def test_creep_watchdog_requires_sustained_loss_of_deceleration():
  controller = CanfdStopping()
  for _ in range(160):
    assert step(controller, speed=.2, a_ego=-.05).stop_req == 1
  for _ in range(10):
    assert step(controller, speed=.2, a_ego=0.).stop_req == 1
  for _ in range(6):
    cmd = step(controller, speed=.2, a_ego=0.)
  assert cmd.stop_req == 0
  assert controller.phase == StopPhase.release


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
@pytest.mark.parametrize("initial", [-1., -.3])
@pytest.mark.parametrize("target", [-.5, -.8, -1.])
def test_packed_normal_stop_matches_legacy_acceleration(camera, initial, target):
  cs = make_cs()
  cs.canfdSccHoldActive = True
  cs.out.vEgo = cs.out.vEgoRaw = 0.
  cs.out.wheelSpeeds = SimpleNamespace(fl=0., fr=0., rl=0., rr=0.)
  controller = CanfdStopping()
  previous = previous_target = initial
  for _ in range(100):
    values = send(camera, controller, cs, accel=target, previous_value=previous, previous_target=previous_target)
    legacy = send(camera, None, cs, accel=target, previous_value=previous, previous_target=previous_target)
    assert values['StopReq'] == legacy['StopReq'] == 1
    assert values['aReqRaw'] == legacy['aReqRaw'] == pytest.approx(target)
    assert values['aReqValue'] == legacy['aReqValue']
    previous, previous_target = values['aReqValue'], target
  assert previous == pytest.approx(target)
  assert not controller.retried


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("target", [-.5, -.8, -1.])
def test_packed_retry_keeps_requested_stopping_acceleration(camera, target):
  controller = CanfdStopping()
  cs = make_cs()
  previous = target
  for _ in range(150):
    old_phase = controller.phase
    values = send(camera, controller, cs, accel=target, previous_value=previous, previous_target=target)
    if controller.phase == StopPhase.retry:
      assert old_phase == StopPhase.release
      assert values['StopReq'] == 1
      assert values['aReqRaw'] == values['aReqValue'] == pytest.approx(target)
      following = send(camera, controller, cs, accel=target, previous_value=values['aReqValue'], previous_target=target)
      assert following['StopReq'] == 1
      assert following['aReqRaw'] == following['aReqValue'] == pytest.approx(target)
      break
    previous = values['aReqValue']
  else:
    pytest.fail('persistent motion did not trigger the retained retry')


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
def test_packet_path_uses_measured_deceleration_to_preserve_stop_request(camera):
  controller = CanfdStopping()
  cs = make_cs()
  cs.out.aEgo = -.04
  for _ in range(180):
    assert send(camera, controller, cs)['StopReq'] == 1
  assert not controller.retried
  cs.out.aEgo = 0.
  for _ in range(16):
    values = send(camera, controller, cs)
  assert values['StopReq'] == 0
  assert controller.phase == StopPhase.release


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("a_ego", [float('nan'), float('inf')])
def test_invalid_measured_acceleration_blocks_retry_commands(camera, a_ego):
  cs = make_cs()
  cs.out.aEgo = a_ego
  controller = CanfdStopping()
  values = send(camera, controller, cs)
  assert values['ACCMode'] == values['StopReq'] == 0
  assert values['aReqRaw'] == values['aReqValue'] == 0.
  assert controller.phase == StopPhase.idle


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
@pytest.mark.parametrize("brake_pressed", [False, True])
def test_soft_hold_uses_original_request_without_preparation(camera, brake_pressed):
  cs = make_cs()
  cs.softHoldActive = 1 if brake_pressed else 2
  cs.out.brakePressed = brake_pressed
  cs.out.vEgo = cs.out.vEgoRaw = 0.
  cs.out.wheelSpeeds = SimpleNamespace(fl=0., fr=0., rl=0., rr=0.)
  controller = CanfdStopping()
  previous = previous_target = 0.
  for _ in range(110):
    values = send(camera, controller, cs, enabled=False, stopping=False, accel=-2.,
                  previous_value=previous, previous_target=previous_target)
    legacy = send(camera, None, cs, enabled=False, stopping=False, accel=-2.,
                  previous_value=previous, previous_target=previous_target)
    assert values['ACCMode'] == values['StopReq'] == 1
    assert values['aReqRaw'] == legacy['aReqRaw'] == -2.
    assert values['aReqValue'] == legacy['aReqValue']
    previous, previous_target = values['aReqValue'], -2.
  assert controller.phase == StopPhase.held
  assert previous == -2.


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
  assert controller.phase == StopPhase.request
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


def test_confirmed_existing_hold_is_not_released():
  controller = CanfdStopping()
  command = step(controller, speed=0., held=True)
  assert command.stop_req == 1
  assert controller.phase == StopPhase.held
