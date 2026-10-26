from types import SimpleNamespace as NS

import pytest
from opendbc.can import CANPacker
from openpilot.cereal import car, log
from openpilot.selfdrive.car.tesla_turn_signals import (
  BODY, CANCEL_NS, CONTEXT_NS, FEEDBACK_DEGRADED_NS, FRONT, FRONT_FAULT_GRACE_NS, SESSION_NS, UI,
  TeslaTurnSignals, body_checksum, body_frame,
)

N = 10_000_000_000


def template(counter=4):
  data = bytearray.fromhex("a58c61b45ac34700")
  data[6] = (counter << 4) | 7
  data[7] = body_checksum(data)
  return bytes(data)


def state(**kwargs):
  values = {"leftBlinkerStalkCount": 0, "rightBlinkerStalkCount": 0, "turnSignalDriverValid": True,
            "turnSignalDriverDirection": 0, "canValid": True, "vehicleSensorsInvalid": False, "gearShifter": "drive"}
  return NS(**(values | kwargs))


def intent(direction=1, request=1):
  meta = log.ModelDataV2.new_message().meta
  meta.autoTurnSignalDirection = direction
  meta.autoTurnSignalRequestId = request
  return meta.as_reader()


def feedback(c, now, ui=(0, 0), front=(0, 0), body=True):
  p = CANPacker("tesla_model3_party")
  v = CANPacker("tesla_model3_vehicle")
  frames = [p.make_can_msg("UI_warning", 0, {"leftBlinkerBlinking": ui[0], "rightBlinkerBlinking": ui[1]}),
            v.make_can_msg("ID3F5VCFRONT_lighting", 1,
                          {"VCFRONT_turnSignalLeftStatus": front[0], "VCFRONT_turnSignalRightStatus": front[1]})]
  if body:
    frames.append((BODY, template(), 1))
  c.observe([(now, frames)], now)


def update(c, now=N, direction=1, request=1, fresh=True, cs=None, active=True):
  return c.update(cs or state(), NS(latActive=active), intent(direction, request), fresh, now)


def echo(c, frame, now, source=0x81):
  c.observe([(now, [(frame.address, frame.dat, source)])], now)


def opened(direction=1):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  frame = update(c, direction=direction)[0]
  echo(c, frame, N + 1)
  feedback(c, N + 2, ui=(2, 0) if direction == 1 else (0, 2),
           front=(1, 0) if direction == 1 else (0, 1))
  assert c.session.on_seen
  return c


def channel_frame(channel, direction=1, lit=True, fault=False):
  if channel == UI:
    values = [0, 0]
    values[direction - 1] = 3 if fault else 2 if lit else 1
    return CANPacker("tesla_model3_party").make_can_msg(
      "UI_warning", 0, {"leftBlinkerBlinking": values[0], "rightBlinkerBlinking": values[1]})
  values = [0, 0]
  values[direction - 1] = 2 if fault else int(lit)
  return CANPacker("tesla_model3_vehicle").make_can_msg(
    "ID3F5VCFRONT_lighting", 1, {"VCFRONT_turnSignalLeftStatus": values[0], "VCFRONT_turnSignalRightStatus": values[1]})


@pytest.mark.parametrize("direction", [1, 2])
def test_recorded_front_two_one_waveform_does_not_cancel_or_falsely_confirm_light(direction):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  sent = update(c, direction=direction)[0]
  echo(c, sent, N + 1)
  active_count = 1
  for tick in range(1, 41):
    now = N + tick * 50_000_000
    phase = 2 if (tick - 1) % 14 < 7 else 1
    raw = bytes.fromhex(("0200c838820d0800" if phase == 2 else "0100c838820d0400") if direction == 1 else
                        ("0800c838820c2000" if phase == 2 else "0400c838820c1000"))
    c.observe([(now, [channel_frame(UI, direction), (FRONT, raw, 1)])], now)
    assert not c.session.cancelling
    if tick < 8:
      assert not c.session.on_seen and c.ready_request_id(now) == 0
    else:
      assert c.ready_request_id(now) == 1
    if tick % 10 == 0:
      c.observe([(now, [(BODY, template(tick % 16), 1)])], now)
    for sent in update(c, now, direction=direction):
      assert sent.dat[1] & 3 == direction
      echo(c, sent, now + 1)
      active_count += 1
  assert active_count == 5 and c.session.on_seen


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("on_seen", [False, True])
def test_front_two_persistent_fault_has_fixed_deadline_not_renewed_by_more_frames(direction, on_seen):
  c = opened(direction) if on_seen else TeslaTurnSignals(True)
  if not on_seen:
    feedback(c, N)
    echo(c, update(c, direction=direction)[0], N + 1)
  start = N + 50_000_000
  for tick in range(12):
    now = start + tick * 50_000_000
    feedback(c, now, ui=(2, 0) if direction == 1 else (0, 2),
             front=(2, 0) if direction == 1 else (0, 2))
    if tick < 10:
      assert not c.session.cancelling
    else:
      assert c.status == "cancelling:invalid_lamp_feedback" and c.ready_request_id(now) == 0
    for sent in update(c, now, direction=direction):
      assert sent.dat[1] & 3 == (direction if tick < 10 else 3)
      echo(c, sent, now + 1)
  assert c.blocked_id == 1


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("late", [False, True])
def test_front_two_deadline_checked_before_recovery_frame_can_clear_it(direction, late):
  c = opened(direction)
  start = N + 50_000_000
  feedback(c, start, ui=(2, 0) if direction == 1 else (0, 2),
           front=(2, 0) if direction == 1 else (0, 2), body=False)
  now = start + FRONT_FAULT_GRACE_NS - int(not late)
  c.context_time = now
  feedback(c, now, ui=(2, 0) if direction == 1 else (0, 2),
           front=(1, 0) if direction == 1 else (0, 1), body=False)
  assert c.session.cancelling == late
  assert c.ready_request_id(now) == (0 if late else 1)


def test_front_two_fault_deadline_runs_without_another_front_frame():
  c = opened()
  start = N + 50_000_000
  feedback(c, start, ui=(2, 0), front=(2, 0), body=False)
  for tick in range(1, 11):
    now = start + tick * 50_000_000
    c.context_time = now
    c.observe([(now, [channel_frame(UI)])], now)
  assert c.status == "cancelling:invalid_lamp_feedback"
  assert c.ready_request_id(now) == 0


def test_transient_two_never_renews_physical_on_or_confirms_cancellation():
  c = opened()
  last_on = c.session.last_front_on
  feedback(c, N + 50_000_000, ui=(2, 0), front=(2, 0), body=False)
  assert c.session.last_front_on == last_on and not c._off(N + 50_000_000)
  cancel = update(c, N + 50_000_001, direction=0)[0]
  echo(c, cancel, N + 50_000_002)
  for tick in range(2, 20):
    now = N + tick * 50_000_000
    feedback(c, now, front=(2, 0), body=False)
    assert c.session is not None and c.status != "cancelled"


def test_opposite_front_two_is_not_ignored_and_cannot_extend_its_deadline():
  c = opened()
  for tick in range(12):
    now = N + 50_000_000 + tick * 50_000_000
    feedback(c, now, ui=(2, 0), front=(1, 2))
    sends = update(c, now)
    for sent in sends:
      echo(c, sent, now + 1)
    assert c.session.cancelling == (tick >= 10)
  assert c.status == "cancelling:invalid_lamp_feedback"


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("slow", [UI, FRONT])
def test_confirmed_session_bridges_periodic_single_channel_gaps_with_continued_sends(direction, slow):
  c = opened(direction)
  phases = set()
  sent_count = 0
  for tick in range(1, 51):
    now = N + tick * 100_000_000
    frames = [channel_frame(FRONT if slow == UI else UI, direction, lit=tick % 8 < 4)]
    if tick % 10 == 1:
      frames.append(channel_frame(slow, direction, lit=tick % 8 < 4))
    frames.append((BODY, template(tick % 16), 1))
    c.observe([(now, frames)], now)
    sends = update(c, now, direction=direction)
    assert len(sends) == 1 and sends[0].dat[1] & 3 == direction
    echo(c, sends[0], now + 1)
    phases.add(c.diagnostics(now + 1)["feedback"])
    sent_count += 1
    assert c.status == "active" and not c.session.cancelling
  assert sent_count == 50 and phases == {"fresh", "degraded"}


@pytest.mark.parametrize("remaining", [None, UI, FRONT])
def test_total_or_extended_single_channel_failure_is_terminal_even_after_feedback_recovers(remaining):
  c = opened()
  cancelled = False
  for tick in range(1, 19):
    now = N + tick * 100_000_000
    frames = [(BODY, template(tick % 16), 1)]
    if remaining:
      frames.append(channel_frame(remaining))
    c.observe([(now, frames)], now)
    for sent in update(c, now):
      cancelled |= sent.dat[1] & 3 == 3
      if tick <= (5 if remaining is None else 15):
        assert sent.dat[1] & 3 == 1
      else:
        assert sent.dat[1] & 3 == 3
      echo(c, sent, now + 1)
  assert cancelled and c.status == "cancelling:feedback_stale" and c.blocked_id == 1
  assert FEEDBACK_DEGRADED_NS == 1_500_000_000
  for tick in range(19, 40):
    now = N + tick * 100_000_000
    feedback(c, now)
    assert not update(c, now)
  assert c.session is None
  assert not update(c, now + 1)


@pytest.mark.parametrize("channel", [UI, FRONT])
def test_sna_on_only_fresh_channel_cancels_immediately(channel):
  c = opened()
  now = N + 600_000_000
  c.context_time = now
  frame = channel_frame(channel, fault=True) if channel == UI else CANPacker("tesla_model3_vehicle").make_can_msg(
    "ID3F5VCFRONT_lighting", 1, {"VCFRONT_turnSignalLeftStatus": 3})
  c.observe([(now, [frame])], now)
  assert c.status == "cancelling:invalid_lamp_feedback"
  assert c.take_sends(now)[0].dat[1] & 3 == 3


@pytest.mark.parametrize("channel", [UI, FRONT])
@pytest.mark.parametrize("invalid", ["old", "future", "source", "dlc"])
def test_untrusted_or_out_of_order_feedback_cannot_renew_or_replace_latest_valid_state(channel, invalid):
  c = opened()
  now = N + 100_000_000
  feedback(c, now, ui=(2, 0), front=(1, 0))
  frame = channel_frame(channel, fault=True)
  timestamp = now - 1 if invalid == "old" else now + 1 if invalid == "future" else now
  if invalid == "source":
    frame = (frame[0], frame[1], 1 if channel == UI else 0)
  elif invalid == "dlc":
    frame = (frame[0], frame[1][:-1], frame[2])
  c.observe([(timestamp, [frame])], now)
  assert c.ui_time == c.front_time == now
  assert c.ui == (2, 0) and c.front == (1, 0)
  assert not c.session.cancelling


@pytest.mark.parametrize("slow", [UI, FRONT])
def test_single_channel_tolerance_does_not_remove_echo_deadline(slow):
  c = opened()
  first = c.take_sends(N + 3)[0]
  for tick in range(1, 14):
    now = N + tick * 100_000_000
    frames = [channel_frame(FRONT if slow == UI else UI), (BODY, template(tick % 16), 1)]
    if tick == 10:
      frames.append(channel_frame(slow))
    c.observe([(now, frames)], now)
    sends = update(c, now)
    assert all(sent.dat[1] & 3 == 3 for sent in sends)
  assert first.dat[1] & 3 == 1 and c.status == "cancelling:tx_echo_timeout"


@pytest.mark.parametrize("channel", [UI, FRONT])
def test_feedback_startup_grace_still_bounds_a_missing_channel(channel):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  first = update(c)[0]
  echo(c, first, N + 1)
  for tick in range(1, 19):
    now = N + tick * 100_000_000
    c.observe([(now, [channel_frame(channel), (BODY, template(tick % 16), 1)])], now)
    for sent in update(c, now):
      echo(c, sent, now + 1)
  assert not c.session.on_seen and c.status == "cancelling:feedback_stale"


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("slow", [UI, FRONT])
def test_initial_active_feedback_can_arrive_after_500ms_but_still_requires_confirmation(direction, slow):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  first = update(c, direction=direction)[0]
  echo(c, first, N + 1)
  for tick in range(1, 31):
    now = N + tick * 100_000_000
    frames = [channel_frame(FRONT if slow == UI else UI, direction), (BODY, template(tick % 16), 1)]
    if tick % 10 == 0:
      frames.append(channel_frame(slow, direction))
    c.observe([(now, frames)], now)
    sends = update(c, now, direction=direction)
    assert len(sends) == 1 and sends[0].dat[1] & 3 == direction
    echo(c, sends[0], now + 1)
    assert not c.session.cancelling
    assert c.session.on_seen == (tick >= 10)
  assert c.status == "active"


@pytest.mark.parametrize("direction", [1, 2])
def test_slow_fresh_feedback_cannot_skip_initial_physical_lamp_confirmation(direction):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  first = update(c, direction=direction)[0]
  echo(c, first, N + 1)
  for tick in range(1, 27):
    now = N + tick * 100_000_000
    frames = [channel_frame(UI, direction), (BODY, template(tick % 16), 1)]
    if tick % 10 == 0:
      frames.append(channel_frame(FRONT, direction, lit=False))
    c.observe([(now, frames)], now)
    for sent in update(c, now, direction=direction):
      assert sent.dat[1] & 3 == (direction if tick < 25 else 3)
      echo(c, sent, now + 1)
  assert not c.session.on_seen and c.status == "cancelling:lamp_not_confirmed"


@pytest.mark.parametrize("direction", [1, 2])
def test_ui_cannot_mask_a_periodically_reported_physical_lamp_stuck_off(direction):
  c = opened(direction)
  for tick in range(1, 43):
    now = N + tick * 100_000_000
    frames = [channel_frame(UI, direction), (BODY, template(tick % 16), 1)]
    if tick % 10 == 0:
      frames.append(channel_frame(FRONT, direction, lit=False))
    c.observe([(now, frames)], now)
    for sent in update(c, now, direction=direction):
      assert sent.dat[1] & 3 == (direction if tick <= 40 else 3)
      echo(c, sent, now + 1)
  assert c.status == "cancelling:physical_lamp_feedback_lost" and c.blocked_id == 1
  assert c.diagnostics(now)["physical_on_age_ms"] >= 4000


def test_diagnostics_distinguish_submissions_echoes_and_rejections_without_side_effects():
  c = opened()
  assert c.diagnostics(N + 2)["active_submissions"] == 1
  assert c.diagnostics(N + 2)["active_tx_echoes"] == 1
  sent = c.take_sends(N + 3)[0]
  snapshot = c.diagnostics(N + 3)
  assert snapshot["active_submissions"] == 2 and snapshot["active_tx_echoes"] == 1
  assert snapshot["awaiting_echo"] and snapshot["on_seen"]
  echo(c, sent, N + 4, source=0xC1)
  assert c.diagnostics(N + 4)["panda_rejections"] == 1
  assert c.status == "cancelling:panda_rejected"
  feedback(c, N + 5, ui=(2, 0), front=(1, 0))
  cancel = c.take_sends(N + 6)[0]
  assert cancel.dat[1] & 3 == 3
  echo(c, cancel, N + 7)
  echo(c, cancel, N + 8)  # Duplicate RX cannot inflate counters.
  snapshot = c.diagnostics(N + 8)
  assert snapshot["active_submissions"] == 2 and snapshot["active_tx_echoes"] == 1
  assert snapshot["cancel_submissions"] == snapshot["cancel_tx_echoes"] == 1
  assert not snapshot["awaiting_echo"]


def test_invalid_driver_input_does_not_masquerade_as_manual_takeover():
  c = opened()
  update(c, N + 3, cs=state(turnSignalDriverValid=False, rightBlinkerStalkCount=1))
  assert c.status == "cancelling:driver_input_unavailable"
  assert not c.session.manual


@pytest.mark.parametrize("direction", [1, 2])
def test_invalid_vehicle_sensors_cancel_active_lamp_session(direction):
  c = opened(direction)
  sends = update(c, N + 3, direction=direction, cs=state(vehicleSensorsInvalid=True))
  assert c.status == "cancelling:vehicle_sensors_invalid"
  assert not c.session.manual
  assert sends and all(frame.dat[1] & 3 == 3 for frame in sends)


def cancelling(direction=1):
  c = opened(direction)
  sent = update(c, N + 3, direction=0)[0]
  assert sent.dat[1] & 3 == 3
  echo(c, sent, N + 4)
  return c


@pytest.mark.parametrize("direction", [1, 2])
def test_full_lamp_cycle_requires_stable_ui_and_physical_off(direction):
  c = cancelling(direction)
  for dt in (10, 200_000_010, 400_000_010, 600_000_010):
    feedback(c, N + dt)
  assert c.session is None
  assert c.status == "cancelled"
  assert not update(c, N + 600_000_011, direction=direction)  # consumed request
  assert update(c, N + 600_000_012, direction=direction, request=2)


@pytest.mark.parametrize("front", [0, 2, 3])
def test_flashing_dark_phase_fault_and_sna_never_confirm_cancellation(front):
  c = cancelling()
  for dt in (10, 200_000_010, 400_000_010, 600_000_010):
    feedback(c, N + dt, ui=(1, 0), front=(front, 0))
  assert c.session is not None
  assert c.status != "cancelled"
  c.advance(N + CANCEL_NS + 100)
  assert c.status == "fault:cancel_not_confirmed"
  assert c.owned_direction == 1
  assert not c.take_sends(N + CANCEL_NS + 101)


def test_independent_receive_clock_cancels_when_controlsd_stops():
  c = opened()
  c.advance(N + CONTEXT_NS + 1)
  assert c.session.cancelling
  assert c.take_sends(N + CONTEXT_NS + 2, cancel_only=True)[0].dat[1] & 3 == 3


@pytest.mark.parametrize("gate", ["disabled", "model", "lateral", "can", "sensors", "gear", "driver", "stalk", "no_template"])
def test_start_gates(gate):
  c = TeslaTurnSignals(gate != "disabled")
  feedback(c, N, body=gate != "no_template")
  cs = state(canValid=gate != "can", vehicleSensorsInvalid=gate == "sensors", gearShifter="park" if gate == "gear" else "drive",
             turnSignalDriverValid=gate != "stalk", turnSignalDriverDirection=1 if gate == "driver" else 0)
  assert not update(c, fresh=gate != "model", active=gate != "lateral", cs=cs)
  expected = {"disabled": "disabled", "model": "blocked:context_stale", "lateral": "blocked:lateral_inactive",
              "can": "blocked:can_invalid", "sensors": "blocked:vehicle_sensors_invalid",
              "gear": "blocked:not_in_drive", "driver": "blocked:driver",
              "stalk": "blocked:driver_input_unavailable", "no_template": "waiting:body_template"}
  assert c.status == expected[gate]


@pytest.mark.parametrize("direction", [1, 2, 3])
def test_body_only_changes_owned_fields(direction):
  original = template(15)
  sent = body_frame(original, direction)
  assert sent[0] == original[0] and sent[3:6] == original[3:6]
  assert sent[1] & 0xFC == original[1] & 0xFC
  assert sent[2] & 0xE1 == original[2] & 0xE1
  assert sent[6] == 7
  assert sent[7] == body_checksum(sent)


def test_bad_template_and_reused_template_are_rejected():
  with pytest.raises(ValueError):
    body_frame(bytes(8), 1)
  c = opened()
  assert c.take_sends(N + 3)
  assert not c.take_sends(N + 4)


@pytest.mark.parametrize("driver", [1, 2])
def test_driver_takes_ownership_without_cancelling_manual_lamp(driver):
  from opendbc.car.structs import CarParams
  from opendbc.car.tesla.values import TeslaSafetyFlags
  from opendbc.safety.tests.libsafety import libsafety_py

  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.tesla, TeslaSafetyFlags.AUTO_TURN_SIGNALS) == 0
  safety.init_tests()
  safety.set_timer(0)
  safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, template()))
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, body_frame(template(), 1)))
  c = opened()
  sends = update(c, N + 3, cs=state(leftBlinkerStalkCount=1, turnSignalDriverDirection=driver))
  assert not sends
  assert c.owned_direction == 0
  assert c.session.manual and c.status == "driver_takeover:waiting_off"
  for tick in range(1, 651):
    now = N + tick * 100_000_000
    feedback(c, now, ui=(2, 0) if driver == 1 else (0, 2), front=(1, 0) if driver == 1 else (0, 1))
    assert not update(c, now, cs=state(leftBlinkerStalkCount=1))
  for tick in range(651, 666):
    now = N + tick * 100_000_000
    safety.set_timer(tick * 100_000)
    safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, template()))
    c.observe_driver(state(leftBlinkerStalkCount=1), now)
    feedback(c, now)
    sends = c.take_sends(now, cancel_only=True)
    if sends:
      assert sends[0].dat[1] & 3 == 3
      assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, sends[0].dat))
      echo(c, sends[0], now + 1)
  assert c.session is None
  sends = update(c, now + 2, direction=2, request=2, cs=state(leftBlinkerStalkCount=1))
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, sends[0].dat))


def test_reverse_request_cancels_before_other_direction():
  c = opened()
  sends = update(c, N + 3, direction=2, request=2)
  assert len(sends) == 1 and sends[0].dat[1] & 3 == 3
  assert c.session.direction == 1


def test_session_limit_requests_cancel_even_with_fresh_controls():
  c = opened()
  now = N + SESSION_NS
  feedback(c, now, ui=(2, 0), front=(1, 0))
  assert c.session.cancelling  # no control update can revive an expired lease
  assert update(c, now)[0].dat[1] & 3 == 3


def test_schema_defaults_disable_automatic_provenance():
  assert car.CarState.new_message().autoTurnSignalDirection == 0
  assert car.CarState.new_message().autoTurnSignalReadyRequestId == 0
  assert not car.CarState.new_message().turnSignalDriverValid
  assert log.ModelDataV2.new_message().meta.autoTurnSignalRequestId == 0


def test_ui_decoder_uses_real_dbc_and_does_not_accept_corrupt_checksum():
  c = TeslaTurnSignals(True)
  feedback(c, N, ui=(2, 1))
  assert c.ui == (2, 1)
  c.observe([(N + 1, [(UI, bytes(7), 0)])], N + 1)
  assert c.ui_time == N
  assert FRONT != BODY


def test_real_capnp_inputs_and_can_serialization():
  from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp

  c = TeslaTurnSignals(True)
  feedback(c, N)
  cs = car.CarState.new_message(canValid=True, gearShifter="drive", turnSignalDriverValid=True)
  cc = car.CarControl.new_message(latActive=True)
  meta = log.ModelDataV2.new_message().meta
  meta.autoTurnSignalDirection = "left"
  meta.autoTurnSignalRequestId = 42
  sends = c.update(cs.as_reader(), cc.as_reader(), meta.as_reader(), True, N)
  packet = can_list_to_can_capnp(sends, msgtype="sendcan", valid=True)
  assert can_capnp_to_list([packet], msgtype="sendcan")[0][1] == [tuple(sends[0])]


@pytest.mark.parametrize("gate", ["model", "lateral", "can", "gear", "stalk"])
def test_loss_of_any_start_gate_requests_cancel_instead_of_relighting(gate):
  c = opened()
  sends = update(c, N + 3, fresh=gate != "model", active=gate != "lateral",
                 cs=state(canValid=gate != "can", gearShifter="park" if gate == "gear" else "drive",
                          turnSignalDriverValid=gate != "stalk"))
  assert sends[0].dat[1] & 3 == 3
  assert c.owned_direction == 1


def test_lost_echo_allows_bounded_cancel_retries_then_latches_fault():
  c = opened()
  sends = update(c, N + 3, direction=0)
  assert sends[0].dat[1] & 3 == 3
  for attempt in (1, 2):
    now = N + 3 + attempt * 1_200_000_001
    feedback(c, now)
    assert c.take_sends(now)[0].dat[1] & 3 == 3
  feedback(c, N + 3_600_000_010)
  assert not c.take_sends(N + 3_600_000_010)
  c.advance(N + CANCEL_NS + 4)
  assert c.failed
  assert c.owned_direction == 1
  for dt in (5_100_000_010, 5_300_000_010, 5_500_000_010, 5_700_000_010):
    feedback(c, N + dt)
  assert c.owned_direction == 0
  assert c.failed and c.status.startswith("fault:")
  assert not update(c, N + 5_700_000_011, request=99)


def test_rejected_cancel_is_never_reported_as_success():
  c = opened()
  sends = update(c, N + 3, direction=0)
  echo(c, sends[0], N + 4, source=0xC1)
  assert c.failed and c.status == "fault:panda_rejected_cancel"
  assert c.owned_direction == 1


def test_driver_handoff_cannot_hide_latched_cancel_failure():
  c = opened()
  cancel = update(c, N + 3, direction=0)[0]
  echo(c, cancel, N + 4, source=0xC1)
  assert not update(c, N + 5, cs=state(turnSignalDriverDirection=2))
  assert c.failed and c.status == "fault:panda_rejected_cancel"
  assert not c.owned_direction


def test_hazards_and_fast_driver_tap_release_automatic_ownership():
  for hazards in (False, True):
    c = opened()
    if hazards:
      feedback(c, N + 3, ui=(2, 2), front=(1, 1))
    sends = update(c, N + 4, cs=state(leftBlinkerStalkCount=1 if not hazards else 0))
    assert not sends
    assert not c.owned_direction
    assert c.status == "driver_takeover:waiting_off"


@pytest.mark.parametrize("direction", [1, 2])
def test_sustained_requests_use_fresh_counters_echo_and_real_c_safety(direction):
  from opendbc.car.structs import CarParams
  from opendbc.car.tesla.values import TeslaSafetyFlags
  from opendbc.safety.tests.libsafety import libsafety_py

  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.tesla, TeslaSafetyFlags.AUTO_TURN_SIGNALS) == 0
  safety.init_tests()
  c = TeslaTurnSignals(True)
  for tick in range(101):
    now = N + tick * 100_000_000
    safety.set_timer(tick * 100_000)
    feedback(c, now, ui=((1 if tick % 2 else 2), 0) if direction == 1 else (0, (1 if tick % 2 else 2)),
             front=(1 - tick % 2, 0) if direction == 1 else (0, 1 - tick % 2), body=False) if tick else feedback(c, now, body=False)
    original = template(tick % 16)
    c.observe([(now, [(BODY, original, 1)])], now)
    safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, original))
    sends = update(c, now, direction=direction)
    assert len(sends) == 1 and sends[0].dat == body_frame(original, direction)
    packet = libsafety_py.make_CANPacket(BODY, 1, sends[0].dat)
    assert safety.safety_tx_hook(packet)
    assert not safety.safety_tx_hook(packet)
    echo(c, sends[0], now + 1)
    assert c.status == "active"
  assert c.session.sent == 101
  feedback(c, now + 2)
  cancel = update(c, now + 2, direction=0)[0]
  safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, template()))
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, cancel.dat))
  echo(c, cancel, now + 3)
  for tick in range(1, 8):
    feedback(c, now + tick * 100_000_000)
  assert c.session is None
  assert not update(c, now + 700_000_001, direction=direction)


@pytest.mark.parametrize("direction", [1, 2])
def test_missing_template_waits_and_resumes_same_request_with_fresh_template(direction):
  c = opened(direction)
  c.template = None
  for tick in range(1, 21):
    now = N + tick * 100_000_000
    feedback(c, now, ui=(2, 0) if direction == 1 else (0, 2),
             front=(1, 0) if direction == 1 else (0, 1), body=False)
    assert not update(c, now, direction=direction)
    assert not c.session.cancelling and c.blocked_id == 0
  c.observe([(now, [(BODY, template(7), 1)])], now)
  resumed = update(c, now, direction=direction)
  assert len(resumed) == 1 and resumed[0].dat == body_frame(template(7), direction)
  assert c.session.request_id == 1


def test_missing_template_wait_is_bounded_by_session_deadline():
  c = opened()
  c.template = None
  for tick in range(1, 601):
    now = N + tick * 100_000_000
    feedback(c, now, ui=(2, 0), front=(1, 0), body=False)
    assert not update(c, now)
    assert c.session.cancelling == (tick == 600)
  assert c.status == "cancelling:session_timeout" and c.blocked_id == 1
  feedback(c, now + 1, ui=(2, 0), front=(1, 0))
  assert c.take_sends(now + 1)[0].dat[1] & 3 == 3
  c.advance(now + CANCEL_NS + 2)
  assert c.failed


@pytest.mark.parametrize("direction", [1, 2])
def test_missing_template_does_not_hide_physical_lamp_loss(direction):
  c = opened(direction)
  c.template = None
  for tick in range(1, 8):
    now = N + tick * 100_000_000
    feedback(c, now, body=False)
    assert not update(c, now, direction=direction)
  assert c.status == "cancelling:lamps_lost" and c.blocked_id == 1
  feedback(c, now + 1)
  assert c.take_sends(now + 1)[0].dat[1] & 3 == 3


@pytest.mark.parametrize("direction", [1, 2])
def test_readiness_requires_both_physical_confirmation_and_exact_tx_echo(direction):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  sent = update(c, direction=direction)[0]
  assert c.ready_request_id(N) == 0
  feedback(c, N + 1, ui=(2, 0) if direction == 1 else (0, 2),
           front=(1, 0) if direction == 1 else (0, 1), body=False)
  assert c.session.on_seen and c.ready_request_id(N + 1) == 0
  echo(c, sent, N + 2)
  assert c.ready_request_id(N + 2) == 1
  cs = car.CarState.new_message(autoTurnSignalReadyRequestId=c.ready_request_id(N + 2))
  with car.CarState.from_bytes(cs.to_bytes()) as received:
    assert received.autoTurnSignalReadyRequestId == 1
  assert c.ready_request_id(N + CONTEXT_NS + 1) == 0
  assert c.ready_request_id(N - 1) == 0


@pytest.mark.parametrize("gate", ["manual", "cancel", "fault", "context", "feedback", "session"])
def test_readiness_drops_on_execution_failure_or_handoff(gate):
  c = opened()
  now = N + 3
  assert c.ready_request_id(now) == 1
  if gate == "manual":
    c.observe_driver(state(turnSignalDriverDirection=1), now)
  elif gate == "cancel":
    update(c, now, direction=0)
  elif gate == "fault":
    cancel = update(c, now, direction=0)[0]
    echo(c, cancel, now, source=0xC1)
  elif gate == "context":
    c.context_time = 0
  elif gate == "feedback":
    c.front_time = 0
  else:
    now = N + SESSION_NS
    c.context_time = c.ui_time = c.front_time = now
  assert c.ready_request_id(now) == 0


def test_consumed_or_out_of_order_template_cannot_be_reused_after_echo():
  c = TeslaTurnSignals(True)
  feedback(c, N)
  first = update(c)[0]
  echo(c, first, N + 1)
  for timestamp in (N - 1, N):
    c.observe([(timestamp, [(BODY, template(), 1)])], N + 2)
    assert not c.take_sends(N + 2)
  c.observe([(N + 3, [(BODY, template(5), 1), (BODY, template(6), 1)])], N + 3)
  c.observe([(N + 2, [(BODY, template(4), 1)])], N + 3)
  assert c.take_sends(N + 3)[0].dat == body_frame(template(6), 1)
  assert not c.take_sends(N + 4)


def test_template_diagnostics_separate_filtered_rx_waiting_and_awaiting_echo():
  c = opened()
  c.template = None
  now = N + 3
  snapshot = c.diagnostics(now)
  assert snapshot["send_state"] == "waiting:body_template" and snapshot["ready_request_id"] == 1
  c.observe([(now, [(BODY, bytes(8), 1), (BODY, body_frame(template(), 1), 1)])], now)
  snapshot = c.diagnostics(now)
  assert snapshot["body_rx"] == 4 and snapshot["body_idle_rx"] == 2
  assert snapshot["body_invalid_rx"] == snapshot["body_non_idle_rx"] == 1
  assert not c.take_sends(now)
  c.observe([(now + 1, [(BODY, template(6), 1)])], now + 1)
  assert c.diagnostics(now + 1)["send_state"] == "ready"
  sent = c.take_sends(now + 1)[0]
  snapshot = c.diagnostics(now + 1)
  assert snapshot["send_state"] == "waiting:tx_echo"
  assert snapshot["pending_counter"] == 7 and not snapshot["template_ready"]
  echo(c, sent, now + 2)
  assert c.diagnostics(now + 2)["send_state"] == "waiting:body_template"


@pytest.mark.parametrize("direction", [1, 2])
def test_native_panda_template_race_is_rejected_and_never_retried_as_active(direction):
  from opendbc.car.structs import CarParams
  from opendbc.car.tesla.values import TeslaSafetyFlags
  from opendbc.safety.tests.libsafety import libsafety_py

  safety = libsafety_py.libsafety
  assert safety.set_safety_hooks(CarParams.SafetyModel.tesla, TeslaSafetyFlags.AUTO_TURN_SIGNALS) == 0
  safety.init_tests()
  safety.set_timer(0)
  safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, template()))
  c = TeslaTurnSignals(True)
  feedback(c, N)
  first = update(c, direction=direction)[0]
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, first.dat))
  echo(c, first, N + 1)
  feedback(c, N + 2, ui=(2, 0) if direction == 1 else (0, 2),
           front=(1, 0) if direction == 1 else (0, 1), body=False)
  assert c.ready_request_id(N + 2) == 1
  c.observe([(N + 3, [(BODY, template(5), 1)])], N + 3)
  safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, template(5)))
  second = update(c, N + 3, direction=direction)[0]
  # OEM RX advances on Panda after the host has built its next frame.
  safety.safety_rx_hook(libsafety_py.make_CANPacket(BODY, 1, template(6)))
  assert not safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, second.dat))
  echo(c, second, N + 4, source=0xC1)
  assert c.status == "cancelling:panda_rejected" and c.ready_request_id(N + 4) == 0
  c.observe([(N + 5, [(BODY, template(6), 1)])], N + 5)
  cancel = update(c, N + 5, direction=direction)[0]
  assert cancel.dat[1] & 3 == 3
  assert safety.safety_tx_hook(libsafety_py.make_CANPacket(BODY, 1, cancel.dat))
  echo(c, cancel, N + 6)
  assert not update(c, N + 7, direction=direction)
  assert c.blocked_id == 1 and c.session.active_echoes == 1 and c.session.rejected == 1


@pytest.mark.parametrize("missing", [0, UI, FRONT])
def test_unknown_opposite_lamp_yields_without_cancelling_it(missing):
  c = opened()
  if missing == 0:
    feedback(c, N + 10, ui=(0, 2), front=(0, 1))
  else:
    if missing == UI:
      c.ui_time = 0
      frame = CANPacker("tesla_model3_vehicle").make_can_msg("ID3F5VCFRONT_lighting", 1, {"VCFRONT_turnSignalRightStatus": 1})
    else:
      c.front_time = 0
      frame = CANPacker("tesla_model3_party").make_can_msg("UI_warning", 0, {"rightBlinkerBlinking": 2})
    c.observe([(N + 10, [frame])], N + 10)
  assert c.session.manual and c.owned_direction == 0
  assert not c.take_sends(N + 10, cancel_only=True)
  assert not update(c, N + 11)


@pytest.mark.parametrize("gate", ["held", "stale", "dark", "hazards"])
def test_manual_cleanup_cannot_send_until_fresh_idle_and_true_off(gate):
  c = opened()
  update(c, N + 3, cs=state(turnSignalDriverDirection=1))
  for tick in range(1, 80):
    now = N + tick * 100_000_000
    c.observe_driver(state(turnSignalDriverDirection=1 if gate == "held" else 0,
                           turnSignalDriverValid=gate != "stale"), now)
    feedback(c, now, ui=(1, 0) if gate == "dark" else (2, 2) if gate == "hazards" else (0, 0),
             front=(1, 1) if gate == "hazards" else (0, 0))
    assert not c.take_sends(now, cancel_only=True)
  assert c.session.manual and not c.failed


@pytest.mark.parametrize("dark", [True, False])
def test_lamp_dark_phase_is_not_real_off_but_lost_lamps_cancel(dark):
  c = opened()
  cancelled = False
  for tick in range(1, 13):
    now = N + tick * 100_000_000
    feedback(c, now, ui=(1, 0) if dark else (0, 0), front=(0, 0))
    sends = update(c, now)
    if sends:
      cancelled |= sends[0].dat[1] & 3 == 3
      echo(c, sends[0], now + 1)
  assert cancelled != dark
  assert c.status == ("active" if dark else "cancelled")


def test_no_confirmed_lit_phase_after_initial_blink_is_bounded():
  c = opened()
  for tick in range(1, 28):
    now = N + tick * 100_000_000
    feedback(c, now, ui=(1, 0), front=(0, 0))
    for sent in update(c, now):
      echo(c, sent, now + 1)
  assert c.status == "cancelling:lamp_feedback_lost"


@pytest.mark.parametrize("source", [0x81, 0xC1, None])
def test_delayed_rejected_or_lost_active_echo_never_sends_unconfirmed_repeats(source):
  c = TeslaTurnSignals(True)
  feedback(c, N)
  first = update(c)[0]
  for tick in range(1, 12):
    now = N + tick * 100_000_000
    feedback(c, now, ui=(2, 0), front=(1, 0))
    assert not update(c, now)
  if source is not None:
    echo(c, first, now + 1, source=source)
    sends = update(c, now + 2)
    assert sends[0].dat[1] & 3 == (1 if source == 0x81 else 3)
  else:
    now = N + 1_200_000_001
    feedback(c, now, ui=(2, 0), front=(1, 0))
    assert update(c, now)[0].dat[1] & 3 == 3
    assert c.status == "cancelling:tx_echo_timeout"


@pytest.mark.parametrize("ui,front", [((3, 0), (0, 0)), ((2, 3), (1, 0)), ((2, 0), (3, 0)), ((2, 0), (1, 3))])
def test_fault_or_unavailable_feedback_cancels_an_active_session(ui, front):
  c = opened()
  feedback(c, N + 3, ui=ui, front=front)
  assert c.session.cancelling
  assert c.take_sends(N + 4)[0].dat[1] & 3 == 3


@pytest.mark.parametrize("gate,expected", [
  ("ready", "idle"), ("off", "disabled:setting_off"), ("vehicle_bus", "blocked:vehicle_bus_unavailable"),
  ("restart", "blocked:restart_required"), ("safety", "blocked:safety_configuration"),
  ("passive", "disabled:passive"), ("platform", "disabled:unsupported_platform"),
])
def test_configuration_reports_why_extension_signals_are_not_enabled(gate, expected):
  from opendbc.car.tesla.tests.test_auto_turn_signals import params
  from opendbc.car.tesla.values import TeslaFlags

  cp = params(vehicle_bus=gate != "vehicle_bus", enabled=gate != "restart")
  if gate == "safety":
    cp.safetyConfigs[0].safetyParam = 0
  elif gate == "passive":
    cp.passive = True
  elif gate == "platform":
    cp.carFingerprint = "TESLA_MODEL_X"
  c = TeslaTurnSignals.from_car_params(cp, enabled=gate != "off")
  assert c.status == expected
  assert c.configured == (gate == "ready")
  assert bool(cp.flags & TeslaFlags.HAS_VEHICLE_BUS) == (gate != "vehicle_bus")


@pytest.mark.parametrize("gate,expected", [
  ("ui", "waiting:ui_feedback"), ("front", "waiting:front_feedback"),
  ("lamp", "waiting:lamps_off"), ("invalid", "waiting:invalid_lamp_feedback"),
])
def test_feedback_waits_report_specific_cause(gate, expected):
  c = TeslaTurnSignals(True)
  feedback(c, N, ui=(2, 0) if gate == "lamp" else (0, 0), front=(3, 0) if gate == "invalid" else (0, 0))
  if gate == "ui":
    c.ui_time = 0
  elif gate == "front":
    c.front_time = 0
  assert not update(c)
  assert c.status == expected
  feedback(c, N + 1)
  assert update(c, N + 1)


@pytest.mark.parametrize("direction", [1, 2])
@pytest.mark.parametrize("stalk_present", [False, True])
@pytest.mark.parametrize("size", [3, 4])
def test_detected_extension_and_real_can1_stalk_reach_lamp_sender(direction, stalk_present, size, monkeypatch):
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.tests.test_auto_turn_signals import params, stalk_frame
  from opendbc.car.tesla.values import TeslaFlags

  cp = params(longitudinal=True)
  assert cp.flags & TeslaFlags.AUTO_SPEED_LIMIT
  ci = CarInterface(cp)
  decoder = ci.CS
  packet = stalk_frame(0, size=size)
  monkeypatch.setattr("opendbc.car.tesla.carstate.time.clock_gettime_ns", lambda clock: N)
  monkeypatch.setattr("opendbc.car.tesla.carstate.time.monotonic_ns", lambda: N - 5_000_000_000)
  cs = ci.update([(N, [packet] if stalk_present else [])])
  assert cs.turnSignalDriverValid == (stalk_present and size == 4)
  # Other required driving messages are covered separately; this isolates the extension-input boundary.
  cs.canValid, cs.gearShifter = True, "drive"
  c = TeslaTurnSignals.from_car_params(cp, enabled=True)
  feedback(c, N)
  sends = update(c, direction=direction, cs=cs)
  if stalk_present and size == 4:
    assert len(sends) == 1 and sends[0].src == 1
    assert sends[0].address == BODY and sends[0].dat[1] & 3 == direction
  else:
    assert not sends and c.status == "blocked:driver_input_unavailable"
  assert decoder.tesla_manual_speed_adjustment_counter == 0
