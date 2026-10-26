from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.tests.test_desire_helper_stalk import TestDesireHelperStalkRequest as StalkFixture


@pytest.fixture
def setup():
  fixture = StalkFixture()
  fixture.setup_method()
  fixture.helper.tesla_auto_signals = True
  fixture.carrot_man.atcType = "fork left"
  return fixture


@pytest.mark.parametrize("position,direction", [
  (1, 2), (2, 2), (3, 2), (4, 2),
  (5, 1), (6, 1), (7, 1), (8, 1),
])
@pytest.mark.parametrize("bus", [0, 1])
@pytest.mark.parametrize("release_in_batch", [False, True])
def test_received_stalk_frame_selects_matching_lane_change(setup, monkeypatch, position, direction, bus, release_in_batch):
  from opendbc.can import CANPacker
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.tests.test_auto_turn_signals import N, params, stalk_frame

  ci = CarInterface(params())
  setup.carrot_man.atcType = ""
  h = setup.helper
  packer = CANPacker("tesla_model3_party")
  monkeypatch.setattr("opendbc.car.tesla.carstate.time.clock_gettime_ns", lambda clock: N)

  def receive(positions, lamp_direction=0):
    frames = [stalk_frame(p, bus) for p in positions]
    frames.append(packer.make_can_msg("UI_warning", 0, {"leftBlinkerBlinking": 2 if lamp_direction == 1 else 0,
                                                      "rightBlinkerBlinking": 2 if lamp_direction == 2 else 0}))
    cs = ci.update([(N, frames)])
    cs.canValid, cs.vEgo = True, 25.
    h.update(cs, NS(), True, .5, setup.carrot_man, NS())

  receive([0])
  receive([position, 0] if release_in_batch else [position], direction)
  assert h.lane_change_state == log.LaneChangeState.preLaneChange
  assert h.driver_blinker_state == direction
  receive([0] if release_in_batch else [position], direction)
  assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
  assert h.lane_change_direction == direction


@pytest.mark.parametrize("invalid", ["sna", "crc", "dlc", "three_byte"])
def test_untrusted_stalk_cannot_start_a_manual_lane_change(setup, monkeypatch, invalid):
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.tests.test_auto_turn_signals import N, params, stalk_frame

  ci = CarInterface(params())
  setup.carrot_man.atcType = ""
  frame = stalk_frame(9 if invalid == "sna" else 4)
  if invalid == "crc":
    frame = (frame[0], bytes((frame[1][0] ^ 1,)) + frame[1][1:], frame[2])
  elif invalid == "dlc":
    frame = (frame[0], frame[1][:2], frame[2])
  elif invalid == "three_byte":
    frame = (frame[0], frame[1][:3], frame[2])
  monkeypatch.setattr("opendbc.car.tesla.carstate.time.clock_gettime_ns", lambda clock: N)
  for packet in (stalk_frame(0), frame, frame):
    cs = ci.update([(N, [packet])])
    cs.canValid, cs.vEgo = True, 25.
    setup.helper.update(cs, NS(), True, .5, setup.carrot_man, NS())
  assert setup.helper.lane_change_state == log.LaneChangeState.off
  assert not setup.helper.auto_signal_request_id
  assert cs.leftBlinkerStalkCount == cs.rightBlinkerStalkCount == 0


@pytest.mark.parametrize("valid,counts", [(True, (1, 1)), (False, (1, 0)), (False, (0, 1))])
def test_ambiguous_or_invalid_counter_changes_never_default_to_left(setup, valid, counts):
  cs = NS(leftBlinker=False, rightBlinker=False, autoTurnSignalDirection=0, turnSignalDriverDirection=0,
          turnSignalDriverValid=True, leftBlinkerStalkCount=0, rightBlinkerStalkCount=0)
  h = setup.helper
  assert h._update_driver_blinker(cs)[0] == h._update_stalk(cs) == 0
  cs.turnSignalDriverValid = valid
  cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount = counts
  assert h._update_driver_blinker(cs)[0] == h._update_stalk(cs) == 0


def step(f, lamp=False, owned=0, driver=0, active=True, prob=.5, valid=True, trailer=False, model=None, speed=25., angle=0.,
         **state_overrides):
  # FSM unit cases assume a confirmed executor; coupled cases override with the real controller's readiness.
  cs = NS(canValid=valid, leftBlinker=lamp and owned != 2, rightBlinker=lamp and owned == 2, vEgo=speed, aEgo=0., trailerConnected=trailer,
          steeringAngleDeg=angle,
          steeringTorque=0., steeringPressed=False, leftBlinkerStalkCount=0, rightBlinkerStalkCount=0,
          turnSignalDriverValid=True, turnSignalDriverDirection=driver, autoTurnSignalDirection=owned,
          autoTurnSignalReadyRequestId=f.helper.auto_signal_request_id)
  cs.__dict__.update(state_overrides)
  f.helper.update(cs, model or NS(), active, prob, f.carrot_man, NS())
  return f.helper


def app_request(f, direction):
  f.carrot_man.atcType = ""
  f.carrot_man.carrotCmd = "LANECHANGE"
  f.carrot_man.carrotCmdIndex = 1
  f.carrot_man.carrotArg = "LEFT" if direction == 1 else "RIGHT"


@pytest.mark.parametrize("enabled", [False, True])
@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("valid", [False, True])
@pytest.mark.parametrize("press", [False, True])
def test_hazards_preserve_manual_interlock_and_consume_stalk_presses(setup, enabled, direction, valid, press):
  setup.helper.tesla_auto_signals = enabled
  setup.carrot_man.atcType = ""
  h = step(setup)
  counts = {"leftBlinkerStalkCount": int(press and direction == 1),
            "rightBlinkerStalkCount": int(press and direction == 2)}
  for _ in range(3):
    step(setup, driver=direction, turnSignalDriverValid=valid, leftBlinker=True, rightBlinker=True,
         autoTurnSignalReadyRequestId=0, **counts)
    assert h.driver_blinker_state == 3
    assert h.lane_change_state == log.LaneChangeState.off
    assert not h.auto_signal_request_id
    if enabled:
      assert not h._manual_signal_direction
      assert not h._manual_signal_grace
  for _ in range(3):
    step(setup, autoTurnSignalReadyRequestId=0, **counts)
    assert h.lane_change_state == log.LaneChangeState.off
    assert not h.auto_signal_request_id
  if enabled:
    assert not h._update_stalk(NS(turnSignalDriverValid=True, turnSignalDriverDirection=0,
                                 leftBlinker=False, rightBlinker=False, **counts))


@pytest.mark.parametrize("direction", [1, 2])
def test_hazard_stalk_press_is_consumed_without_confirmation_or_replay(setup, direction):
  h = setup.helper
  cs = NS(leftBlinker=False, rightBlinker=False, turnSignalDriverValid=True, turnSignalDriverDirection=0,
          leftBlinkerStalkCount=0, rightBlinkerStalkCount=0)
  assert not h._update_stalk(cs)
  cs.leftBlinker = cs.rightBlinker = True
  cs.leftBlinkerStalkCount = int(direction == 1)
  cs.rightBlinkerStalkCount = int(direction == 2)
  assert not h._update_stalk(cs)
  cs.leftBlinker = cs.rightBlinker = False
  assert not h._update_stalk(cs)
  if direction == 1:
    cs.leftBlinkerStalkCount += 1
  else:
    cs.rightBlinkerStalkCount += 1
  assert h._update_stalk(cs) == direction


@pytest.mark.parametrize("direction", [1, 2])
def test_hazards_clear_existing_manual_signal_latch(setup, direction):
  setup.carrot_man.atcType = ""
  step(setup, driver=direction)
  assert setup.helper._manual_signal_direction == direction
  h = step(setup, driver=direction, leftBlinker=True, rightBlinker=True)
  assert h.driver_blinker_state == 3
  assert not h._manual_signal_direction
  assert not h._manual_signal_grace
  assert h.lane_change_state == log.LaneChangeState.off


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("ready", [0, 99])
def test_automatic_lane_change_cannot_start_without_matching_lamp_confirmation(setup, direction, ready):
  app_request(setup, direction)
  h = step(setup, autoTurnSignalReadyRequestId=0)
  request = h.auto_signal_request_id
  assert request and request != 99
  for _ in range(20):
    step(setup, autoTurnSignalReadyRequestId=ready)
    assert h.lane_change_state == log.LaneChangeState.preLaneChange
    assert h.auto_signal_request_id == request
  step(setup, autoTurnSignalReadyRequestId=request)
  assert h.lane_change_state == log.LaneChangeState.laneChangeStarting


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("source", ["app", "bluetooth"])
@pytest.mark.parametrize("wait", ["delay", "bsd", "torque"])
@pytest.mark.parametrize("template_period", [20, 40], ids=["1s", "2s"])
def test_single_app_action_waits_then_emits_through_all_lane_phases_with_native_safety(setup, direction, source, wait,
                                                                                   template_period):
  from opendbc.can import CANPacker
  from opendbc.car.structs import CarParams
  from opendbc.car.tesla.values import TeslaSafetyFlags
  from opendbc.safety.tests.libsafety import libsafety_py
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, echo, feedback, state, template
  from openpilot.selfdrive.car.tesla_turn_signals import BODY, FRONT, UI, TeslaTurnSignals

  app_request(setup, direction)
  if source == "bluetooth":
    setup.carrot_man.carrotCmd = ""
    setup.helper.bluetooth_commands.read.side_effect = ["laneLeft" if direction == 1 else "laneRight"] + [None] * 199
  h = setup.helper
  side = h.left if direction == 1 else h.right
  h.laneChangeDelay = 1.5 if wait == "delay" else 0.
  h.laneChangeBsd = int(wait == "bsd")
  h.laneChangeNeedTorque = int(wait == "torque")
  core = TeslaTurnSignals(True)
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.tesla, TeslaSafetyFlags.AUTO_TURN_SIGNALS) == 0
  safety.init_tests()
  request = None
  phases = set()
  cancelled = False
  pending_echoes = []
  for frame in range(200):
    now = N + frame * 50_000_000
    for due, sent in pending_echoes:
      if due == frame:
        echo(core, sent, now)
    pending_echoes = [(due, sent) for due, sent in pending_echoes if due > frame]
    side.bsd_hold_counter = 10 if wait == "bsd" and frame < 30 else 0
    side.lane_change_available = not side.bsd_hold_counter
    torque = (1. if direction == 1 else -1.) if wait == "torque" and frame >= 30 else 0.
    step(setup, prob=.5 if frame < 70 else 0., steeringPressed=bool(torque), steeringTorque=torque,
         autoTurnSignalReadyRequestId=core.ready_request_id(now))
    if frame == 0:
      request = h.auto_signal_request_id
      assert request
    if frame < 30:
      assert h.lane_change_state == log.LaneChangeState.preLaneChange
    if h.auto_signal_request_id:
      assert h.auto_signal_request_id == request and not cancelled
    elif frame > 30:
      assert h.lane_change_state == log.LaneChangeState.off
    # Real DBC blink/dark feedback; physical channel is only 1 Hz after confirmation.
    lit = frame % 16 < 8
    ui = ((2 if lit else 1), 0) if direction == 1 else (0, (2 if lit else 1))
    front = (int(lit), 0) if direction == 1 else (0, int(lit))
    if frame == 0 or cancelled:
      ui = front = (0, 0)
    feedback_frames = [CANPacker("tesla_model3_party").make_can_msg(
      "UI_warning", 0, {"leftBlinkerBlinking": ui[0], "rightBlinkerBlinking": ui[1]})]
    if frame in (0, 1) or frame % 20 == 1 or cancelled:
      feedback_frames.append(CANPacker("tesla_model3_vehicle").make_can_msg(
        "ID3F5VCFRONT_lighting", 1, {"VCFRONT_turnSignalLeftStatus": front[0], "VCFRONT_turnSignalRightStatus": front[1]}))
    assert all(f[0] in (UI, FRONT) for f in feedback_frames)
    safety.set_timer(frame * 50_000)
    if frame % template_period == 0:
      original = template((frame // template_period) % 16)
      feedback_frames.append((BODY, original, 1))
      safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, original))
    core.observe([(now, feedback_frames)], now)
    meta = log.ModelDataV2.new_message().meta
    meta.autoTurnSignalDirection, meta.autoTurnSignalRequestId = h.auto_signal_direction, h.auto_signal_request_id
    for sent in core.update(state(), NS(latActive=True), meta.as_reader(), True, now):
      packet = libsafety_py.make_CANPacket(BODY, 1, sent.dat)
      assert safety.safety_tx_hook(packet)
      assert not safety.safety_tx_hook(packet)  # No OEM template replay.
      if sent.dat[1] & 3 == direction:
        assert not cancelled and h.auto_signal_request_id == request
        phases.add(h.lane_change_state)
      else:
        assert sent.dat[1] & 3 == 3 and not h.auto_signal_request_id
        cancelled = True
      pending_echoes.append((frame + 2, sent))
    if cancelled and core.session is None:
      feedback(core, now + 2)
  assert phases == {log.LaneChangeState.preLaneChange, log.LaneChangeState.laneChangeStarting,
                    log.LaneChangeState.laneChangeFinishing}
  assert cancelled and core.session is None and core.status == "cancelled"
  assert not h.auto_signal_request_id and h.lane_change_state == log.LaneChangeState.off


@pytest.mark.parametrize("direction", [1, 2])
def test_no_echo_keeps_app_in_preparation_then_expires_without_rearming(setup, direction):
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, feedback, state
  from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals

  app_request(setup, direction)
  core = TeslaTurnSignals(True)
  h = setup.helper
  active_sends = 0
  for frame in range(240):
    now = N + frame * 50_000_000
    step(setup, autoTurnSignalReadyRequestId=core.ready_request_id(now))
    assert h.lane_change_state in (log.LaneChangeState.off, log.LaneChangeState.preLaneChange)
    feedback(core, now, ui=(2, 0) if frame and direction == 1 else (0, 2) if frame else (0, 0),
             front=(1, 0) if frame and direction == 1 else (0, 1) if frame else (0, 0))
    meta = log.ModelDataV2.new_message().meta
    meta.autoTurnSignalDirection, meta.autoTurnSignalRequestId = h.auto_signal_direction, h.auto_signal_request_id
    for sent in core.update(state(), NS(latActive=True), meta.as_reader(), True, now):
      active_sends += int(sent.dat[1] & 3 == direction)
    assert core.ready_request_id(now) == 0
  assert active_sends == 1 and core.failed
  assert not h.auto_signal_request_id and not h._auto_app_direction


@pytest.mark.parametrize("direction", [1, 2])
def test_app_recorded_two_one_feedback_runs_full_fsm_with_native_safety(setup, direction):
  from opendbc.car.structs import CarParams
  from opendbc.car.tesla.values import TeslaSafetyFlags
  from opendbc.safety.tests.libsafety import libsafety_py
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, echo, feedback, state, template
  from openpilot.selfdrive.car.tesla_turn_signals import BODY, TeslaTurnSignals

  app_request(setup, direction)
  h = setup.helper
  core = TeslaTurnSignals(True)
  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.tesla, TeslaSafetyFlags.AUTO_TURN_SIGNALS) == 0
  safety.init_tests()
  phases = set()
  cancelled = False
  request = 0
  for tick in range(120):
    now = N + tick * 50_000_000
    step(setup, prob=.5 if tick < 45 else 0., autoTurnSignalReadyRequestId=core.ready_request_id(now))
    if tick == 0:
      request = h.auto_signal_request_id
    if tick <= 8:
      assert h.lane_change_state == log.LaneChangeState.preLaneChange
    if h.auto_signal_request_id:
      assert h.auto_signal_request_id == request
    front_value = 2 if (tick - 1) % 14 < 7 else 1
    ui, front = [0, 0], [0, 0]
    if tick and not cancelled:
      ui[direction - 1] = 2 if front_value == 2 else 1
      front[direction - 1] = front_value
    feedback(core, now, ui=tuple(ui), front=tuple(front), body=False)
    safety.set_timer(tick * 50_000)
    if tick % 10 == 0:
      original = template((tick // 10) % 16)
      core.observe([(now, [(BODY, original, 1)])], now)
      safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, original))
    meta = log.ModelDataV2.new_message().meta
    meta.autoTurnSignalDirection, meta.autoTurnSignalRequestId = h.auto_signal_direction, h.auto_signal_request_id
    for sent in core.update(state(), NS(latActive=True), meta.as_reader(), True, now):
      assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, sent.dat))
      if sent.dat[1] & 3 == direction:
        assert not cancelled and h.auto_signal_request_id == request
        phases.add(h.lane_change_state)
      else:
        assert sent.dat[1] & 3 == 3 and not h.auto_signal_request_id
        cancelled = True
      echo(core, sent, now + 1)
  assert phases == {log.LaneChangeState.preLaneChange, log.LaneChangeState.laneChangeStarting,
                    log.LaneChangeState.laneChangeFinishing}
  assert cancelled and core.status == "cancelled" and core.session is None


@pytest.mark.parametrize("direction", [1, 2])
def test_manual_driver_bypasses_automatic_lamp_readiness_gate(setup, direction):
  setup.carrot_man.atcType = ""
  step(setup, driver=direction, autoTurnSignalReadyRequestId=0)
  h = step(setup, driver=direction, autoTurnSignalReadyRequestId=0)
  assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
  assert not h.auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
def test_lamp_readiness_loss_during_execution_does_not_abruptly_reset_steering(setup, direction):
  app_request(setup, direction)
  h = step(setup, autoTurnSignalReadyRequestId=0)
  step(setup, autoTurnSignalReadyRequestId=h.auto_signal_request_id)
  assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
  request = h.auto_signal_request_id
  for _ in range(10):
    step(setup, autoTurnSignalReadyRequestId=0)
    assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert h.auto_signal_request_id == request


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("gate", ["expiry", "speed", "can", "sensors", "driver_invalid", "lateral", "trailer", "opposite_torque"])
def test_pending_app_is_consumed_on_expiry_or_terminal_cancel_without_delayed_surprise(setup, direction, gate):
  from openpilot.selfdrive.controls.lib.desire_helper import AUTO_APP_WAIT_S

  app_request(setup, direction)
  h = setup.helper
  h.laneChangeNeedTorque = 1
  request = step(setup).auto_signal_request_id
  for _ in range(25):
    assert step(setup).auto_signal_request_id == request
    assert h.lane_change_state == log.LaneChangeState.preLaneChange
  if gate == "expiry":
    for _ in range(int(AUTO_APP_WAIT_S / .05)):
      step(setup)
  else:
    step(setup, speed=0. if gate == "speed" else 25., valid=gate != "can", active=gate != "lateral",
         trailer=gate == "trailer", vehicleSensorsInvalid=gate == "sensors", turnSignalDriverValid=gate != "driver_invalid",
         steeringPressed=gate == "opposite_torque", steeringTorque=(-1. if direction == 1 else 1.))
  assert not h.auto_signal_request_id and not h._auto_app_direction
  h.laneChangeNeedTorque = 0
  for _ in range(50):
    assert not step(setup).auto_signal_request_id
    assert h.lane_change_state == log.LaneChangeState.off
  setup.carrot_man.carrotCmdIndex = 2
  assert step(setup).auto_signal_request_id > request


def test_pending_app_deadline_expires_across_model_update_gap(setup, monkeypatch):
  from openpilot.selfdrive.controls.lib.desire_helper import AUTO_APP_WAIT_S

  clock = [100.]
  monkeypatch.setattr("openpilot.selfdrive.controls.lib.desire_helper.time.monotonic", lambda: 100.)
  monkeypatch.setattr("openpilot.selfdrive.controls.lib.desire_helper.time.clock_gettime", lambda clock_id: clock[0])
  app_request(setup, 1)
  setup.helper.laneChangeNeedTorque = 1
  assert step(setup).auto_signal_request_id
  clock[0] += AUTO_APP_WAIT_S + 1
  setup.helper.laneChangeNeedTorque = 0
  h = step(setup)
  assert not h.auto_signal_request_id and h.lane_change_state == log.LaneChangeState.off
  assert not step(setup).auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("phase", ["starting", "finishing"])
def test_app_execution_keeps_original_fsm_completion_when_speed_drops(setup, direction, phase):
  app_request(setup, direction)
  step(setup)
  h = step(setup)
  request = h.auto_signal_request_id
  expected = log.LaneChangeState.laneChangeStarting
  if phase == "finishing":
    expected = h.lane_change_state = log.LaneChangeState.laneChangeFinishing
    h.lane_change_ll_prob = .5
  for _ in range(5):
    step(setup, speed=7.)
    assert h.lane_change_state == expected and h.auto_signal_request_id == request
  for _ in range(60):
    step(setup, speed=7., prob=0.)
    if h.lane_change_state == log.LaneChangeState.off:
      break
    assert h.auto_signal_request_id == request
  assert h.lane_change_state == log.LaneChangeState.off and not h.auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("navigation_type", [
  "atc left", "atc right", "fork left", "fork right",
  "turn left", "turn right", "turn left prepare", "turn right prepare",
])
def test_explicit_app_request_does_not_inherit_navigation_only_confirmation_rules(setup, direction, navigation_type):
  app_request(setup, direction)
  setup.carrot_man.atcType = navigation_type
  h = step(setup)
  request = h.auto_signal_request_id
  assert h.lane_change_state == log.LaneChangeState.preLaneChange and request
  for _ in range(20):
    step(setup)
    assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert h.lane_change_direction == direction and h.auto_signal_request_id == request
    assert not h._auto_turn_direction


@pytest.mark.parametrize("gate", ["speed", "lateral", "can", "trailer", "driver", "hazards", "stalk", "sensors", "gear"])
def test_initially_rejected_app_command_is_not_replayed_after_recovery(setup, gate):
  app_request(setup, 1)
  h = step(setup, speed=0. if gate == "speed" else 25., active=gate != "lateral", valid=gate != "can",
           trailer=gate == "trailer", driver=1 if gate == "driver" else 0,
           leftBlinker=gate == "hazards", rightBlinker=gate == "hazards", turnSignalDriverValid=gate != "stalk",
           vehicleSensorsInvalid=gate == "sensors", gearShifter="park" if gate == "gear" else "drive")
  assert not h.auto_signal_request_id and not h._auto_app_direction
  for _ in range(50):
    assert not step(setup).auto_signal_request_id
  assert h.lane_change_state == log.LaneChangeState.off
  setup.carrot_man.carrotCmdIndex = 2
  assert step(setup).auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
def test_non_tesla_app_pulse_behavior_is_unchanged(setup, direction):
  app_request(setup, direction)
  h = setup.helper
  h.tesla_auto_signals = False
  h.laneChangeDelay = 1.
  for _ in range(40):
    step(setup)
    assert h.lane_change_state != log.LaneChangeState.laneChangeStarting
    assert not h.auto_signal_request_id and not h._auto_app_direction
  assert h.lane_change_state == log.LaneChangeState.off
  assert h.carrot_lane_change_count == 0


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("block", ["solid", "geometry", "object"])
def test_pending_app_never_bypasses_geometry_solid_lines_or_objects(setup, direction, block):
  app_request(setup, direction)
  h = setup.helper
  side = h.left if direction == 1 else h.right
  h.laneLineCheck = 2
  side.lane_change_available = False
  side.lane_change_available_geom = block == "object"
  side.lane_available = block != "geometry"
  side.side_object_detected = block == "object"
  side.lane_line_info_mod = 1 if block == "solid" else 0
  request = step(setup).auto_signal_request_id
  for _ in range(40):
    assert step(setup).auto_signal_request_id == request
    assert h.lane_change_state == log.LaneChangeState.preLaneChange


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("phase", ["starting", "finishing"])
def test_app_opposite_torque_cancels_even_after_short_command_has_expired(setup, direction, phase):
  app_request(setup, direction)
  for _ in range(10):
    h = step(setup)
  if phase == "finishing":
    h.lane_change_state = log.LaneChangeState.laneChangeFinishing
  step(setup, steeringPressed=True, steeringTorque=-1. if direction == 1 else 1.)
  assert not h.auto_signal_request_id and not h._auto_app_direction
  assert h.lane_change_state == log.LaneChangeState.off
  for _ in range(30):
    assert not step(setup).auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
def test_app_owns_its_lane_change_when_navigation_changes_to_a_turn(setup, direction):
  app_request(setup, direction)
  h = step(setup)
  request = h.auto_signal_request_id
  for frame in range(60):
    if frame == 10:
      setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
      h.left.dist_to_edge_far = h.right.dist_to_edge_far = 5.
    step(setup, prob=.5 if frame < 20 else 0.)
    if not h.auto_signal_request_id:
      break
    assert h.auto_signal_request_id == request and h.maneuver_type == "lane_change"
    assert not h._auto_turn_direction
  assert not h.auto_signal_request_id and h.lane_change_state == log.LaneChangeState.off


@pytest.mark.parametrize("direction", [1, 2])
def test_navigation_lane_change_holds_lamps_from_preparation_to_finishing(setup, direction):
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, echo, feedback, state
  from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals

  h = setup.helper
  side = h.left if direction == 1 else h.right
  setup.carrot_man.atcType = "atc left" if direction == 1 else "atc right"
  setup.carrot_man.xTurnInfo = direction
  setup.carrot_man.xDistToTurn = 100
  core = TeslaTurnSignals(True)
  request = None
  phases = set()
  cancelled = False
  for frame in range(130):
    side.lane_exist_count.counter = 0 if frame < 30 else 10
    side.lane_change_available_geom = side.lane_change_available = frame >= 30
    side.edge_available = True
    side.lane_appeared = frame == 30
    # Left navigation still requires the existing driver torque confirmation.
    now = N + frame * 50_000_000
    step(setup, prob=.5 if frame < 60 else 0., steeringPressed=direction == 1 and frame == 30,
         steeringTorque=1. if direction == 1 and frame == 30 else 0.,
         autoTurnSignalReadyRequestId=core.ready_request_id(now))
    if frame == 0:
      request = h.auto_signal_request_id
      assert request
    if h.auto_signal_request_id:
      assert h.auto_signal_request_id == request
    ui, front = [0, 0], [0, 0]
    if frame and not cancelled:
      ui[direction - 1] = 2 if frame % 16 < 8 else 1
      front[direction - 1] = int(frame % 16 < 8)
    feedback(core, now, ui=tuple(ui), front=tuple(front))
    meta = log.ModelDataV2.new_message().meta
    meta.autoTurnSignalDirection, meta.autoTurnSignalRequestId = h.auto_signal_direction, h.auto_signal_request_id
    for sent in core.update(state(), NS(latActive=True), meta.as_reader(), True, now):
      if sent.dat[1] & 3 == direction:
        assert not cancelled and h.auto_signal_request_id
        phases.add(h.lane_change_state)
      else:
        assert sent.dat[1] & 3 == 3 and not h.auto_signal_request_id
        cancelled = True
      echo(core, sent, now + 1)
  assert phases == {log.LaneChangeState.off, log.LaneChangeState.preLaneChange,
                    log.LaneChangeState.laneChangeStarting, log.LaneChangeState.laneChangeFinishing}
  assert cancelled and core.session is None and core.status == "cancelled"


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("phase", ["pre", "starting", "finishing"])
@pytest.mark.parametrize("manual", ["tap_same", "opposite", "hazards"])
def test_manual_ownership_at_every_app_phase_never_sends_cancel_into_driver_lamp(setup, direction, phase, manual):
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, echo, feedback, state
  from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals

  app_request(setup, direction)
  h = setup.helper
  h.laneChangeNeedTorque = int(phase == "pre")
  core = TeslaTurnSignals(True)
  for frame in range(5):
    now = N + frame * 50_000_000
    step(setup, autoTurnSignalReadyRequestId=core.ready_request_id(now))
    if phase == "finishing" and frame >= 2:
      h.lane_change_state = log.LaneChangeState.laneChangeFinishing
      h.lane_change_ll_prob = .5
    ui = (2, 0) if direction == 1 else (0, 2)
    front = (1, 0) if direction == 1 else (0, 1)
    feedback(core, now, ui=ui if frame else (0, 0), front=front if frame else (0, 0))
    meta = log.ModelDataV2.new_message().meta
    meta.autoTurnSignalDirection, meta.autoTurnSignalRequestId = h.auto_signal_direction, h.auto_signal_request_id
    for sent in core.update(state(), NS(latActive=True), meta.as_reader(), True, now):
      assert sent.dat[1] & 3 == direction
      echo(core, sent, now + 1)
  assert core.session.on_seen and not core.session.cancelling
  now += 50_000_000
  driver = 3 - direction if manual == "opposite" else 0
  counts = {"leftBlinkerStalkCount" if direction == 1 else "rightBlinkerStalkCount": int(manual == "tap_same")}
  if manual == "hazards":
    feedback(core, now, ui=(2, 2), front=(1, 1))
  step(setup, driver=driver, leftBlinker=manual == "hazards", rightBlinker=manual == "hazards", **counts)
  assert not h.auto_signal_request_id
  meta = log.ModelDataV2.new_message().meta
  assert not core.update(state(turnSignalDriverDirection=driver, **counts), NS(latActive=True), meta.as_reader(), True, now)
  assert core.session.manual and core.owned_direction == 0
  assert not core.take_sends(now, cancel_only=True)
  manual_direction = driver or direction
  for _ in range(30):
    now += 50_000_000
    step(setup, lamp=True, owned=manual_direction, **counts,
         **({"leftBlinker": True, "rightBlinker": True} if manual == "hazards" else {}))
    feedback(core, now, ui=(2, 2) if manual == "hazards" else (2, 0) if manual_direction == 1 else (0, 2),
             front=(1, 1) if manual == "hazards" else (1, 0) if manual_direction == 1 else (0, 1))
    assert not h.auto_signal_request_id
    assert not core.update(state(**counts), NS(latActive=True), meta.as_reader(), True, now)
    assert core.session.manual and not core.failed


def test_automatic_lamp_never_substitutes_driver_confirmation(setup):
  h = step(setup)
  h = step(setup)
  assert h.lane_change_state == log.LaneChangeState.preLaneChange
  assert h.auto_signal_direction == log.LaneChangeDirection.left
  request = h.auto_signal_request_id
  for _ in range(5):
    h = step(setup, lamp=True, owned=1)
    assert h.lane_change_state == log.LaneChangeState.preLaneChange
    assert h.auto_signal_request_id == request


def test_cancellation_tail_cannot_start_a_second_change(setup):
  setup.carrot_man.atcType = ""
  for _ in range(4):
    h = step(setup, lamp=True, owned=1)
    assert h.lane_change_state == log.LaneChangeState.off
    assert h.auto_signal_request_id == 0


def test_real_driver_still_confirms_despite_owned_feedback(setup):
  step(setup)
  step(setup)
  h = step(setup, lamp=True, owned=1, driver=1)
  assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
  assert h.auto_signal_request_id == 0


def test_turn_desire_can_request_lamp_while_lane_change_is_off(setup):
  setup.carrot_man.atcType = "turn left"
  setup.helper.left.dist_to_edge_far = 5.
  step(setup)
  h = step(setup)
  assert h.desire == log.Desire.turnLeft
  assert h.lane_change_state == log.LaneChangeState.off
  assert h.auto_signal_direction == log.LaneChangeDirection.left
  h = step(setup, active=False)
  assert not h.auto_signal_request_id


def test_finishing_does_not_cancel_early_and_completed_pre_does_not_rearm(setup):
  step(setup)
  step(setup)
  h = setup.helper
  h.lane_change_state = log.LaneChangeState.laneChangeFinishing
  h.lane_change_direction = log.LaneChangeDirection.left
  h.lane_change_ll_prob = .5
  step(setup)
  assert h.auto_signal_request_id
  h.lane_change_ll_prob = .99
  step(setup)
  assert h.lane_change_state == log.LaneChangeState.preLaneChange
  assert h.auto_signal_request_id == 0
  step(setup, lamp=True, owned=1)
  assert h.auto_signal_request_id == 0


def test_pre_turn_signal_does_not_enable_a_maneuver(setup):
  setup.carrot_man.atcType = "turn left prepare"
  setup.carrot_man.xTurnInfo = 1
  setup.carrot_man.xDistToTurn = 100
  h = step(setup)
  assert h.desire == log.Desire.none
  assert h.lane_change_state == log.LaneChangeState.off
  assert h.auto_signal_direction == log.LaneChangeDirection.left


def test_driver_rejection_does_not_rearm_the_same_atc_pre_signal(setup):
  setup.carrot_man.atcType = "atc left"
  setup.carrot_man.xTurnInfo = 1
  setup.carrot_man.xDistToTurn = 100
  step(setup)
  step(setup)
  step(setup, driver=2)
  assert setup.helper.atc_active == 2
  assert not step(setup).auto_signal_request_id


def test_completion_latch_survives_off_until_a_new_navigation_request(setup):
  setup.carrot_man.atcType = "atc left"
  setup.carrot_man.xTurnInfo = 1
  setup.carrot_man.xDistToTurn = 100
  step(setup)
  h = step(setup)
  h.lane_change_state = log.LaneChangeState.laneChangeFinishing
  h.lane_change_ll_prob = .99
  step(setup)
  assert h.auto_signal_request_id == 0
  step(setup, active=False)
  assert not step(setup).auto_signal_request_id
  setup.carrot_man.atcType = ""
  step(setup)
  setup.carrot_man.atcType = "turn left prepare"
  assert step(setup).auto_signal_request_id


@pytest.mark.parametrize("argument,direction", [("LEFT", 1), ("RIGHT", 2)])
def test_app_short_command_lamp_lasts_until_fsm_completion(setup, argument, direction):
  setup.carrot_man.atcType = ""
  setup.carrot_man.carrotCmd = "LANECHANGE"
  setup.carrot_man.carrotCmdIndex = 1
  setup.carrot_man.carrotArg = argument
  h = step(setup)
  request = h.auto_signal_request_id
  assert request
  for _ in range(120):
    h = step(setup)
    assert h.lane_change_state == log.LaneChangeState.laneChangeStarting
    assert h.auto_signal_direction == direction
    assert h.auto_signal_request_id == request
  assert h.carrot_lane_change_count == 0
  for _ in range(40):
    h = step(setup, prob=0.)
    if h.lane_change_state == log.LaneChangeState.off:
      break
    assert h.auto_signal_direction == direction
    assert h.auto_signal_request_id == request
  assert h.lane_change_state == log.LaneChangeState.off
  assert not h.auto_signal_request_id
  for _ in range(5):
    assert not step(setup, lamp=True, owned=direction).auto_signal_request_id
  setup.carrot_man.carrotCmdIndex = 2
  assert step(setup).auto_signal_request_id > request


@pytest.mark.parametrize("gate", ["lateral", "driver", "can", "trailer"])
def test_app_latch_does_not_survive_cancellation_or_resume_same_session(setup, gate):
  setup.carrot_man.atcType = ""
  setup.carrot_man.carrotCmd = "LANECHANGE"
  setup.carrot_man.carrotCmdIndex = 1
  setup.carrot_man.carrotArg = "LEFT"
  for _ in range(8):
    step(setup)
  h = step(setup, active=gate != "lateral", driver=2 if gate == "driver" else 0,
           valid=gate != "can", trailer=gate == "trailer")
  assert not h.auto_signal_request_id
  assert not step(setup).auto_signal_request_id


@pytest.mark.parametrize("argument,direction", [("LEFT", 1), ("RIGHT", 2)])
def test_app_can_send_when_vehicle_feedback_arrives_after_command_pulse(setup, argument, direction):
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, feedback, state
  from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals

  setup.carrot_man.atcType = ""
  setup.carrot_man.carrotCmd = "LANECHANGE"
  setup.carrot_man.carrotCmdIndex = 1
  setup.carrot_man.carrotArg = argument
  core = TeslaTurnSignals(True)
  for frame in range(12):
    h = step(setup)
    meta = log.ModelDataV2.new_message().meta
    meta.autoTurnSignalDirection = h.auto_signal_direction
    meta.autoTurnSignalRequestId = h.auto_signal_request_id
    now = N + frame * 50_000_000
    if frame == 7:
      assert h.carrot_lane_change_count == 0
      feedback(core, now)
    sends = core.update(state(), NS(latActive=True), meta.as_reader(), True, now)
    if frame < 7:
      assert not sends and core.session is None
    elif frame == 7:
      assert len(sends) == 1 and sends[0].dat[1] & 3 == direction
    else:
      assert core.session.direction == direction and not core.session.cancelling


@pytest.mark.parametrize("side,direction", [("left", 1), ("right", 2)])
def test_turn_preparation_to_turn_keeps_one_lamp_session(setup, side, direction):
  setup.carrot_man.atcType = f"turn {side} prepare"
  setup.carrot_man.xTurnInfo = direction
  setup.carrot_man.xDistToTurn = 100
  getattr(setup.helper, side).dist_to_edge_far = 5.
  h = step(setup)
  request = h.auto_signal_request_id
  assert request and h.auto_signal_direction == direction
  setup.carrot_man.atcType = f"turn {side}"
  h = step(setup)
  assert h.desire == log.Desire.none  # Keep the original one-frame maneuver suppression.
  assert h.auto_signal_request_id == request
  h = step(setup)
  assert h.desire == (log.Desire.turnLeft if direction == 1 else log.Desire.turnRight)
  assert h.auto_signal_direction == direction and h.auto_signal_request_id == request
  h.turn_disable_count = 10
  assert step(setup).auto_signal_request_id == request


def test_unknown_lamp_is_not_driver_even_without_an_automatic_owner(setup):
  setup.carrot_man.atcType = ""
  for _ in range(5):
    h = step(setup, lamp=True)
    assert h.driver_blinker_state == 0
    assert h.lane_change_state == log.LaneChangeState.off


@pytest.mark.parametrize("driver", [1, 2])
@pytest.mark.parametrize("opposite_tail", [False, True])
def test_manual_provenance_survives_release_and_fast_tap(setup, driver, opposite_tail):
  h = setup.helper
  cs = NS(leftBlinker=False, rightBlinker=False, turnSignalDriverValid=True, turnSignalDriverDirection=0,
          leftBlinkerStalkCount=0, rightBlinkerStalkCount=0)
  assert h._update_driver_blinker(cs)[0] == 0
  setattr(cs, "leftBlinkerStalkCount" if driver == 1 else "rightBlinkerStalkCount", 1)
  assert h._update_driver_blinker(cs)[0] == driver
  cs.leftBlinker, cs.rightBlinker = opposite_tail and driver == 2, opposite_tail and driver == 1
  for _ in range(5):
    assert h._update_driver_blinker(cs)[0] == 0  # The physical lamp can arrive after a quick release.
  cs.leftBlinker, cs.rightBlinker = driver == 1, driver == 2
  for _ in range(30):
    assert h._update_driver_blinker(cs)[0] == driver
  cs.leftBlinker = cs.rightBlinker = False
  for _ in range(12):
    assert h._update_driver_blinker(cs)[0] == 0
  cs.leftBlinker, cs.rightBlinker = driver == 1, driver == 2
  assert h._update_driver_blinker(cs)[0] == 0


@pytest.mark.parametrize("direction", [1, 2])
def test_real_turn_disable_does_not_complete_lamps_until_started_and_stably_straight(setup, direction):
  from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

  h = setup.helper
  sign = 1 if direction == 1 else -1
  setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
  getattr(h, "left" if direction == 1 else "right").dist_to_edge_far = 5.
  step(setup)
  request = step(setup).auto_signal_request_id
  model = NS(meta=NS(desireState=[0.] * 5), orientationRate=NS(z=[sign * .4] * 16), orientation=NS(z=[sign * .2] * 16))
  h._check_desire_state = DesireHelper._check_desire_state.__get__(h)
  model.orientationRate.z[15] = sign * .2
  for _ in range(10):
    step(setup, model=model, speed=5., angle=sign * 100.)
    assert h.turn_disable_count == 200  # Original steering behavior is unchanged.
    assert h.auto_signal_request_id == request
  for speed, angle, yaw in ((0., 0., 0.), (.5, 100., .02), (2., 70., .08)):
    model.orientationRate.z = [sign * yaw] * 16
    for _ in range(30):
      assert step(setup, model=model, speed=speed, angle=sign * angle).auto_signal_request_id == request
  model.orientationRate.z = [sign * .01] * 16
  for _ in range(19):
    assert step(setup, model=model, speed=5., angle=sign * 10.).auto_signal_request_id == request
  assert not step(setup, model=model, speed=5., angle=sign * 10.).auto_signal_request_id
  for _ in range(10):
    assert not step(setup, model=model, speed=5., angle=sign * 10.).auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("reader", [False, True], ids=["builder", "reader"])
def test_turn_lifecycle_with_real_capnp_model_lists(setup, direction, reader):
  sign = 1 if direction == 1 else -1
  setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
  getattr(setup.helper, "left" if direction == 1 else "right").dist_to_edge_far = 5.
  model = log.ModelDataV2.new_message()
  model.meta.desireState = [0.] * 8
  model.orientationRate.z = [sign * .2] * 33
  model.orientation.z = [sign * .2] * 33
  step(setup, model=model.as_reader() if reader else model, speed=5., angle=sign * 90.)
  request = step(setup, model=model.as_reader() if reader else model, speed=5., angle=sign * 90.).auto_signal_request_id
  assert request

  for _ in range(10):
    h = step(setup, model=model.as_reader() if reader else model, speed=5., angle=sign * 90.)
    assert h.auto_signal_direction == direction
    assert h.auto_signal_request_id == request
  assert h._auto_turn_started >= .3

  model.orientationRate.z = [sign * .01] * 33
  for _ in range(19):
    assert step(setup, model=model.as_reader() if reader else model, speed=5., angle=sign * 10.).auto_signal_request_id == request
  assert not step(setup, model=model.as_reader() if reader else model, speed=5., angle=sign * 10.).auto_signal_request_id
  for _ in range(10):
    assert not step(setup, model=model.as_reader() if reader else model, speed=5., angle=sign * 10.).auto_signal_request_id


@pytest.mark.parametrize("reader", [False, True], ids=["builder", "reader"])
@pytest.mark.parametrize("invalid", ["short_rates", "short_headings", "nan_rate", "nan_heading", "angle", "speed"])
def test_real_capnp_invalid_geometry_cannot_complete_turn(setup, reader, invalid):
  h = setup.helper
  h._auto_turn_direction = log.LaneChangeDirection.left
  h._auto_turn_started = .3
  h._auto_turn_peak_angle = 90.
  h._auto_turn_settled = .95
  model = log.ModelDataV2.new_message()
  model.orientationRate.z = [.01] * (15 if invalid == "short_rates" else 33)
  model.orientation.z = [0.] * (15 if invalid == "short_headings" else 33)
  if invalid == "nan_rate":
    model.orientationRate.z[0] = float("nan")
  elif invalid == "nan_heading":
    model.orientation.z[15] = float("nan")
  cs = NS(vEgo=float("inf") if invalid == "speed" else 5., steeringAngleDeg=float("nan") if invalid == "angle" else 5.)
  for _ in range(40):
    h._update_auto_turn_geometry(cs, model.as_reader() if reader else model)
    assert h._auto_turn_direction == log.LaneChangeDirection.left
    assert not h._auto_turn_consumed
    assert h._auto_turn_settled == 0.


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("template_period", [1, 40], ids=["50ms", "2s"])
def test_real_turn_model_roundtrip_reaches_lamp_controller(setup, direction, template_period):
  from openpilot.selfdrive.car.tests.test_tesla_turn_signals import N, echo, feedback, state
  from openpilot.selfdrive.car.tesla_turn_signals import TeslaTurnSignals

  sign = 1 if direction == 1 else -1
  setup.carrot_man.atcType = "turn left prepare" if direction == 1 else "turn right prepare"
  setup.carrot_man.xTurnInfo = direction
  setup.carrot_man.xDistToTurn = 30
  getattr(setup.helper, "left" if direction == 1 else "right").dist_to_edge_far = 5.
  signals = TeslaTurnSignals(True)
  lamp_on = cancelled = False
  active_sends = 0
  request = None
  for frame in range(150):
    now = N + frame * 50_000_000
    if frame == 20:
      setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
    elif frame == 60:
      setup.carrot_man.atcType = "none"
    turning = 20 <= frame < 70
    stopped = frame < 20 or 70 <= frame < 100
    model = log.ModelDataV2.new_message()
    model.orientationRate.z = [sign * (.2 if turning else .01)] * 33
    model.orientation.z = [sign * .2] * 33
    h = step(setup, model=model, speed=0. if stopped else 5., angle=sign * (90. if turning else 10.))
    if frame == 0:
      request = h.auto_signal_request_id
    if frame < 119:
      assert request and h.auto_signal_request_id == request
    model.meta.autoTurnSignalDirection = h.auto_signal_direction
    model.meta.autoTurnSignalRequestId = h.auto_signal_request_id
    ui = ((2 if frame % 16 < 8 else 1), 0) if direction == 1 else (0, (2 if frame % 16 < 8 else 1))
    front = (int(frame % 16 < 8), 0) if direction == 1 else (0, int(frame % 16 < 8))
    feedback(signals, now, ui=ui if lamp_on else (0, 0), front=front if lamp_on else (0, 0),
             body=frame % template_period == 0)
    with log.ModelDataV2.from_bytes(model.to_bytes()) as received:
      sends = signals.update(state(), NS(latActive=True), received.meta, True, now)
    for send in sends:
      if send.dat[1] & 3 == direction:
        assert not cancelled
        active_sends += 1
        lamp_on = True
      else:
        assert send.dat[1] & 3 == 3 and not h.auto_signal_request_id
        cancelled = True
        lamp_on = False
      echo(signals, send, now + 1)
  assert active_sends == len(range(0, 119, template_period))
  assert cancelled and not lamp_on
  assert signals.status == "cancelled" and signals.session is None
  assert not h.auto_signal_request_id


@pytest.mark.parametrize("navigation_type", ["", "none", "none prepare", "straight", "straight prepare"])
@pytest.mark.parametrize("direction", [1, 2])
def test_turn_navigation_can_end_before_geometry_settles_and_abort_does_not_relight(setup, navigation_type, direction):
  sign = 1 if direction == 1 else -1
  setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
  getattr(setup.helper, "left" if direction == 1 else "right").dist_to_edge_far = 5.
  step(setup)
  request = step(setup).auto_signal_request_id
  model = NS(orientationRate=NS(z=[sign * .2] * 16), orientation=NS(z=[sign * .2] * 16))
  for _ in range(10):
    assert step(setup, model=model, speed=4., angle=sign * 90.).auto_signal_request_id == request
  setup.carrot_man.atcType = navigation_type
  for _ in range(100):
    assert step(setup, model=model, speed=4., angle=sign * 90.).auto_signal_request_id == request
  assert not step(setup, model=model, active=False).auto_signal_request_id
  assert not step(setup, model=model).auto_signal_request_id


@pytest.mark.parametrize("navigation_type", ["", "none", "none prepare", "straight", "straight prepare"])
def test_navigation_exit_lamp_hold_finishes_after_stable_geometry(setup, navigation_type):
  setup.carrot_man.atcType = "turn left"
  setup.helper.left.dist_to_edge_far = 5.
  step(setup)
  request = step(setup).auto_signal_request_id
  model = NS(orientationRate=NS(z=[.2] * 16), orientation=NS(z=[.2] * 16))
  for _ in range(10):
    step(setup, model=model, speed=4., angle=90.)
  setup.carrot_man.atcType = navigation_type
  assert step(setup, model=model, speed=4., angle=90.).auto_signal_request_id == request
  model.orientationRate.z = [.01] * 16
  for _ in range(19):
    assert step(setup, model=model, speed=4., angle=10.).auto_signal_request_id == request
  assert not step(setup, model=model, speed=4., angle=10.).auto_signal_request_id
  for _ in range(10):
    assert not step(setup, model=model, speed=4., angle=10.).auto_signal_request_id


@pytest.mark.parametrize("direction", [1, 2])
def test_opposite_navigation_turn_starts_a_new_request_without_reusing_geometry(setup, direction):
  h = setup.helper
  sign = 1 if direction == 1 else -1
  setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
  h.left.dist_to_edge_far = h.right.dist_to_edge_far = 5.
  step(setup)
  request = step(setup).auto_signal_request_id
  model = NS(orientationRate=NS(z=[sign * .2] * 16), orientation=NS(z=[sign * .2] * 16))
  for _ in range(10):
    step(setup, model=model, speed=4., angle=sign * 90.)
  setup.carrot_man.atcType = "turn right" if direction == 1 else "turn left"
  setup.carrot_man.xTurnInfo = 3 - direction
  step(setup, model=model, speed=4., angle=sign * 90.)
  for _ in range(10):
    step(setup, model=model, speed=4., angle=sign * 90.)
    assert h.auto_signal_direction == 3 - direction
    assert h.auto_signal_request_id > request
    assert h._auto_turn_started == 0.
  model.orientationRate.z = [-sign * .2] * 16
  for _ in range(10):
    step(setup, model=model, speed=4., angle=-sign * 90.)
  assert h._auto_turn_started >= .3


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("opposite_input", ["angle", "yaw"])
def test_turn_start_geometry_must_match_requested_direction(setup, direction, opposite_input):
  sign = 1 if direction == 1 else -1
  setup.carrot_man.atcType = "turn left" if direction == 1 else "turn right"
  getattr(setup.helper, "left" if direction == 1 else "right").dist_to_edge_far = 5.
  step(setup)
  request = step(setup).auto_signal_request_id
  model = NS(orientationRate=NS(z=[sign * (-.2 if opposite_input == "yaw" else .2)] * 16), orientation=NS(z=[0.] * 16))
  for _ in range(10):
    step(setup, model=model, speed=4., angle=sign * (-90. if opposite_input == "angle" else 90.))
  assert setup.helper._auto_turn_started == 0.
  model.orientationRate.z = [.01] * 16
  for _ in range(30):
    assert step(setup, model=model, speed=4., angle=5.).auto_signal_request_id == request


@pytest.mark.parametrize("geometry", ["missing", "nan", "heading", "not_started"])
def test_turn_completion_requires_valid_started_and_road_geometry(setup, geometry):
  setup.carrot_man.atcType = "turn left"
  setup.helper.left.dist_to_edge_far = 5.
  step(setup)
  request = step(setup).auto_signal_request_id
  model = NS(orientationRate=NS(z=[.2] * 16), orientation=NS(z=[.2] * 16))
  if geometry != "not_started":
    for _ in range(10):
      step(setup, model=model, speed=4., angle=90.)
  model.orientationRate.z = [.01] * 16
  if geometry == "missing":
    model.orientation.z = []
  elif geometry == "nan":
    model.orientationRate.z[0] = float("nan")
  elif geometry == "heading":
    model.orientation.z[15] = .5
  for _ in range(40):
    assert step(setup, model=model, speed=4., angle=5.).auto_signal_request_id == request
