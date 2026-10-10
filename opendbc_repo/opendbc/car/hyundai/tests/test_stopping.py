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


def test_normal_stop_keeps_zero_request_without_toggle():
  controller = CanfdStopping()
  for tick in range(150):
    cmd = step(controller, speed=max(0, 0.6 - tick * DT * 0.4), held=tick >= 80)
    assert (cmd.stop_req, cmd.raw, cmd.value, cmd.lower) == (1, 0., 0., STOP_LOWER_BAND)
  assert controller.phase == StopPhase.held


@pytest.mark.parametrize('speed', [.07, .3, .65])
def test_persistent_creep_never_releases_request(speed):
  controller = CanfdStopping()
  for _ in range(1500):  # exceed every old timeout, distance and retry limit
    cmd = step(controller, speed=speed)
    assert (cmd.stop_req, cmd.raw, cmd.value) == (1, 0., 0.)
  assert controller.phase == StopPhase.request


def test_motion_after_hold_changes_diagnostic_without_releasing():
  controller = CanfdStopping()
  step(controller, speed=.02, held=True)
  assert controller.phase == StopPhase.held
  for tick in range(300):
    cmd = step(controller, speed=.2 + tick * .005, held=True)
    assert (cmd.stop_req, cmd.raw, cmd.value) == (1, 0., 0.)
  assert controller.phase == StopPhase.request
  assert controller.reason == 'motion_after_hold'


def test_hold_flag_dropout_at_zero_does_not_release():
  controller = CanfdStopping()
  for _ in range(50):
    step(controller, speed=.02, held=True)
  for _ in range(50):
    assert step(controller, speed=.02, held=False).stop_req == 1
  assert controller.phase == StopPhase.held


def test_approach_preserves_braking_until_first_entry_then_zeros_immediately():
  controller = CanfdStopping()
  previous = -.3
  for _ in range(20):
    cmd = step(controller, speed=4., accel=-2., previous_value=previous)
    assert cmd.stop_req == 0
    assert cmd.raw == -2.
    assert cmd.value == pytest.approx(previous - DT)
    previous = cmd.value
  cmd = step(controller, speed=ENTRY_SPEED, accel=-2., previous_value=previous)
  assert (cmd.stop_req, cmd.raw, cmd.value) == (1, 0., 0.)
  for _ in range(100):
    assert step(controller, speed=ENTRY_SPEED + .3).stop_req == 1


@pytest.mark.parametrize('reason', ['inactive', 'departing'])
def test_cancel_resets_episode_and_restores_approach_gate(reason):
  controller = CanfdStopping()
  step(controller)
  assert step(controller, active=reason != 'inactive', requested=reason != 'departing') is None
  assert controller.phase == StopPhase.idle
  assert step(controller, speed=1.).stop_req == 0
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
         previous_value=-0.5, previous_target=-0.5, jerk_upper=2.0):
  packer = CANPacker("hyundai_canfd_generated")
  can = SimpleNamespace(ECAN=0)
  hud = SimpleNamespace(leadDistanceBars=2, leadVisible=False)
  jerk = SimpleNamespace(carrot_cruise=0, jerk_u=jerk_upper, jerk_l=1.0)
  if camera:
    msg, actual_value = create_acc_control_scc2(packer, can, enabled, previous_value, accel, stopping, override, 30.0, hud, jerk, CS, controller)
  else:
    msg, actual_value = create_acc_control(packer, can, enabled, previous_target, accel, stopping, override, 30.0, hud, jerk_upper, 1.0, CS,
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
  assert ((packed >> 152) & 127) * 0.1 == pytest.approx(values["JerkUpperLimit"])
  assert (packed >> 176 & 63) * 0.02 == pytest.approx(values["AccelLimitBandLower"])
  return values


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("initial", [-1., -.3, .3])
@pytest.mark.parametrize("target", [-.5, -.8, -1.])
def test_packed_stop_zeros_both_requests_on_first_frame_and_keeps_zero(camera, initial, target):
  cs = make_cs()
  controller = CanfdStopping()
  previous = initial
  for tick in range(200):
    # Rebound beyond the entry threshold must not create another release.
    speed = .3 if tick < 40 else 1.2
    cs.out.vEgo = cs.out.vEgoRaw = speed
    cs.out.wheelSpeeds = SimpleNamespace(fl=speed, fr=speed, rl=speed, rr=speed)
    cs.out.aEgo = -.3 if tick < 40 else .5
    values = send(camera, controller, cs, accel=target, previous_value=previous, previous_target=target)
    assert values['StopReq'] == 1
    assert values['aReqRaw'] == values['aReqValue'] == 0.
    previous = values['aReqValue']


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
def test_default_controller_keeps_stop_request_during_persistent_motion(camera, monkeypatch):
  controller = make_car_controller(monkeypatch)
  cs = make_cs()
  values = send(camera, controller.canfd_stopping, cs)
  assert values["StopReq"] == 1
  assert values["aReqRaw"] == pytest.approx(0.)
  assert values["InfoDisplay"] == values["ZEROS_7"] == 0
  assert values["AccelLimitBandLower"] == pytest.approx(STOP_LOWER_BAND)
  initial = controller.canfd_stopping
  for _ in range(170):
    send(camera, controller.canfd_stopping, cs)
  assert controller.canfd_stopping is initial
  assert initial.phase == StopPhase.request


@pytest.mark.parametrize("camera", [True, False])
def test_packed_scc_stop_acceleration_zero_display_fields_and_fixed_band(camera):
  controller = CanfdStopping()
  values = send(camera, controller, make_cs())
  assert values["ACCMode"] == 1
  assert values["StopReq"] == 1
  assert values["aReqRaw"] == pytest.approx(0.)
  assert values["aReqValue"] == pytest.approx(0.)
  assert values["InfoDisplay"] == values["ZEROS_7"] == 0
  assert values["AccelLimitBandLower"] == pytest.approx(STOP_LOWER_BAND)


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("a_ego", [float('nan'), float('inf')])
def test_invalid_measured_acceleration_blocks_stop_commands(camera, a_ego):
  cs = make_cs()
  cs.out.aEgo = a_ego
  controller = CanfdStopping()
  values = send(camera, controller, cs)
  assert values['ACCMode'] == values['StopReq'] == 0
  assert values['aReqRaw'] == values['aReqValue'] == 0.
  assert controller.phase == StopPhase.idle


@pytest.mark.parametrize("camera", [True, False])
@pytest.mark.parametrize("block", ["brakePressed", "gasPressed", "brakeHoldActive", "parkingBrake", "canValid", "gearShifter",
                                   "unavailable", "disabled", "override", "nan_speed"])
def test_interlocks_and_motion_sources(camera, block):
  controller = CanfdStopping()
  cs = make_cs()
  for _ in range(40):
    send(camera, controller, cs)
  assert controller.phase == StopPhase.request
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
  values = send(camera, controller, cs, enabled=block != "disabled", override=block == "override")
  assert values["StopReq"] == 0
  assert values["aReqRaw"] == pytest.approx(0)
  assert values["aReqValue"] == pytest.approx(0)
  assert controller.phase == StopPhase.idle


@pytest.mark.parametrize("camera", [True, False])
def test_wheel_motion_overrides_zero_ego_for_initial_entry_but_never_retries(camera):
  controller = CanfdStopping()
  cs = make_cs()
  cs.out.vEgo = cs.out.vEgoRaw = 0.
  cs.canfdSccHoldActive = True
  cs.out.wheelSpeeds.fl = 1.
  assert send(camera, controller, cs)['StopReq'] == 0
  cs.out.wheelSpeeds.fl = .3
  assert send(camera, controller, cs)['StopReq'] == 1
  cs.out.wheelSpeeds.fl = 1.
  assert send(camera, controller, cs)['StopReq'] == 1
  assert controller.phase == StopPhase.request


@pytest.mark.parametrize("camera", [True, False])
def test_departure_releases_and_resumes_normal_acceleration_limiter(camera):
  controller = CanfdStopping()
  cs = make_cs()
  stopped = send(camera, controller, cs)
  assert stopped['aReqValue'] == 0.
  actual = dict(send(camera, controller, cs, stopping=False, accel=.5, previous_value=0., previous_target=0.))
  expected = dict(send(camera, None, cs, stopping=False, accel=.5, previous_value=0., previous_target=0.))
  assert actual['StopReq'] == expected['StopReq'] == 0
  assert actual['aReqRaw'] == expected['aReqRaw'] == .5
  assert actual['aReqValue'] == expected['aReqValue']
  assert actual['aReqValue'] == pytest.approx(.04 if camera else .1)
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
def test_soft_hold_zeros_requests_without_preparation(camera, brake_pressed):
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
    assert values['ACCMode'] == values['StopReq'] == 1
    assert values['aReqRaw'] == 0.
    assert values['aReqValue'] == 0.
    previous, previous_target = values['aReqValue'], -2.
  assert controller.phase == StopPhase.held
  assert previous == 0.


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


@pytest.mark.parametrize('camera', [True, False])
@pytest.mark.parametrize('previous_upper', [.5, 1., 2., 5.])
@pytest.mark.parametrize('soft_hold', [False, True])
def test_stop_upper_increase_waits_one_packet_without_delaying_stop(camera, previous_upper, soft_hold):
  cs = make_cs()
  controller = CanfdStopping()
  first = send(camera, controller, cs, stopping=False, jerk_upper=previous_upper)
  assert first['StopReq'] == 0
  assert first['JerkUpperLimit'] == previous_upper
  if soft_hold:
    cs.softHoldActive = 2
    cs.out.vEgo = cs.out.vEgoRaw = 0.
    cs.out.wheelSpeeds = SimpleNamespace(fl=0., fr=0., rl=0., rr=0.)
  first = send(camera, controller, cs, stopping=not soft_hold, jerk_upper=2.)
  second = send(camera, controller, cs, stopping=not soft_hold, jerk_upper=2.)
  assert first['StopReq'] == second['StopReq'] == 1
  assert first['JerkUpperLimit'] == min(previous_upper, 2.)
  assert second['JerkUpperLimit'] == 2.
  assert first['aReqRaw'] == second['aReqRaw'] == 0.
  assert first['aReqValue'] == second['aReqValue'] == 0.
  assert first['AccelLimitBandLower'] == second['AccelLimitBandLower'] == STOP_LOWER_BAND


@pytest.mark.parametrize('camera', [True, False])
def test_initial_stop_without_previous_packet_uses_upper_one(camera):
  controller = CanfdStopping()
  cs = make_cs()
  assert send(camera, controller, cs)['JerkUpperLimit'] == 1.
  assert send(camera, controller, cs)['JerkUpperLimit'] == 2.


@pytest.mark.parametrize('camera', [True, False])
def test_upper_history_survives_departure_and_interlock_episode_resets(camera):
  cs = make_cs()
  controller = CanfdStopping()
  for _ in range(2):
    send(camera, controller, cs)
  send(camera, controller, cs, stopping=False, jerk_upper=.5)
  assert controller.phase == StopPhase.idle
  assert send(camera, controller, cs)['JerkUpperLimit'] == .5
  cs.out.canValid = False
  blocked = send(camera, controller, cs, stopping=False, jerk_upper=.5)
  assert blocked['StopReq'] == 0
  assert controller.phase == StopPhase.idle
  cs.out.canValid = True
  assert send(camera, controller, cs)['JerkUpperLimit'] == .5
  assert send(camera, controller, cs)['JerkUpperLimit'] == 2.


@pytest.mark.parametrize('camera', [True, False])
def test_upper_edge_uses_final_request_after_approach(camera):
  cs = make_cs()
  controller = CanfdStopping()
  cs.out.vEgo = cs.out.vEgoRaw = 1.
  cs.out.wheelSpeeds = SimpleNamespace(fl=1., fr=1., rl=1., rr=1.)
  approach = send(camera, controller, cs)
  assert approach['StopReq'] == 0
  assert approach['JerkUpperLimit'] == 2.
  cs.out.vEgo = cs.out.vEgoRaw = .3
  cs.out.wheelSpeeds = SimpleNamespace(fl=.3, fr=.3, rl=.3, rr=.3)
  request = send(camera, controller, cs)
  assert request['StopReq'] == 1
  assert request['JerkUpperLimit'] == 2.  # already sent before the real edge
  for _ in range(170):
    packet = send(camera, controller, cs)
    assert packet['JerkUpperLimit'] == 2.
    assert packet['StopReq'] == 1


def test_missing_camera_snapshot_does_not_consume_first_upper_packet():
  cs = make_cs()
  controller = CanfdStopping()
  send(True, controller, cs, stopping=False, jerk_upper=.5)
  snapshot = cs.scc_control
  cs.scc_control = None
  assert send(True, controller, cs) is None
  cs.scc_control = snapshot
  assert send(True, controller, cs)['JerkUpperLimit'] == .5
  assert send(True, controller, cs)['JerkUpperLimit'] == 2.


@pytest.mark.parametrize('camera', [True, False])
def test_upper_sequence_changes_no_other_decoded_packet_fields(camera):
  controller, reference = CanfdStopping(), CanfdStopping()
  reference.limit_scc_jerk_upper = lambda stop_req, upper: upper
  cs = make_cs()
  for tick in range(220):
    # Include ordinary PID frames and sustained stopping.
    args = dict(stopping=tick >= 5, jerk_upper=1. if tick < 5 else 2.,
                accel=-.5, previous_value=-.8, previous_target=-.8)
    actual = dict(send(camera, controller, cs, **args))
    expected = dict(send(camera, reference, cs, **args))
    actual.pop('CHECKSUM')
    expected.pop('CHECKSUM')
    actual.pop('JerkUpperLimit')
    expected.pop('JerkUpperLimit')
    assert actual == expected
    assert controller.phase == reference.phase
