from opendbc.can import CANDefine, CANParser
from opendbc.can.dbc import DBC
from opendbc.car.hyundai.carstate import CANFD_AVH_RELEASE_GRACE_FRAMES, update_canfd_auto_hold_interlock_state


def test_canfd_avh_status_definition():
  dbc = DBC("hyundai_canfd_generated")
  signals = dbc.name_to_msg["ESP_STATUS"].sigs
  signal = signals["AVH_Sta"]

  assert signal.start_bit == 192
  assert signal.size == 2
  assert signal.is_little_endian
  assert signals["AVH_I_LAMP"].start_bit == 218
  assert signals["AVH_I_LAMP"].size == 2
  assert signals["AVH_I_LAMP"].is_little_endian
  assert signals["AVH_LAMP"].start_bit == 220
  assert signals["AVH_LAMP"].size == 3
  assert signals["AVH_LAMP"].is_little_endian

  definitions = CANDefine("hyundai_canfd_generated").dv["ESP_STATUS"]
  assert definitions["AVH_Sta"] == {
    0: "NO_APPLY",
    1: "VEHICLE_IS_HELD_BY_THE_SERVICE_BRAKE",
    2: "BEING_RELEASED",
    3: "ERROR_INDICATOR",
  }
  assert definitions["AVH_I_LAMP"] == {
    0: "LAMP_ON",
    1: "LAMP_OFF",
    2: "RESERVED",
    3: "RESERVED",
  }
  assert definitions["AVH_LAMP"] == {
    0: "AVH_OFF",
    1: "AVH_FAILURE",
    2: "AVH_ACTIVE",
    3: "AVH_READY",
    4: "RESERVED",
    5: "RESERVED",
    6: "RESERVED",
    7: "RESERVED",
  }


def test_canfd_autohold_switch_log_frames():
  parser = CANParser("hyundai_canfd_generated", [("ESP_STATUS", 100)], 0)
  frames = [
    ("b0d44800000500000202000140ff00ff930005000008ff0044000004fffa0000", 1, 0),
    ("6442f300000000000202000240ff00ff000005000008ff0040000030fffa0000", 0, 3),
  ]

  for timestamp, (raw, indicator_lamp, mode_lamp) in enumerate(frames, 1):
    assert parser.update([timestamp * 10_000_000, [(0x60, bytes.fromhex(raw), 0)]]) == {0x60}
    assert parser.vl["ESP_STATUS"]["AVH_I_LAMP"] == indicator_lamp
    assert parser.vl["ESP_STATUS"]["AVH_LAMP"] == mode_lamp


def test_canfd_parking_brake_status_definition():
  dbc = DBC("hyundai_canfd_generated")
  signal = dbc.name_to_msg["TCS"].sigs["ESC_PrkBrkActvSta"]

  assert signal.start_bit == 86
  assert signal.size == 2
  assert signal.is_little_endian

  definitions = CANDefine("hyundai_canfd_generated").dv["TCS"]["ESC_PrkBrkActvSta"]
  assert definitions == {
    0: "PARKING_BRAKE_IS_NOT_ACTIVATED",
    1: "PARKING_BRAKE_IS_ACTIVATED",
    2: "NOT_USED",
    3: "ERROR_INDICATOR",
  }


def _classify_avh_sequence(sequence):
  oem_hold_latched = False
  release_grace_frames = 0
  results = []
  for avh_state, avh_lamp in sequence:
    oem_hold_latched, release_grace_frames = update_canfd_auto_hold_interlock_state(
      avh_state, avh_lamp, oem_hold_latched, release_grace_frames,
    )
    results.append(oem_hold_latched)
  return results


def test_canfd_oem_autohold_lamp_latches_interlock():
  # OEM logs: READY -> ACTIVE while held -> READY while releasing.
  sequence = [(0, 3), (1, 2), (2, 3)]

  assert _classify_avh_sequence(sequence) == [False, True, True]


def test_canfd_soft_hold_with_autohold_off_does_not_latch_interlock():
  # Ioniq 5 and GV80 logs: SCC/soft hold asserts AVH while AutoHold stays off.
  sequence = [(0, 0), (1, 0), (2, 0), (0, 0)]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_scc_hold_with_autohold_ready_does_not_latch_interlock():
  # K5 mixed log: SCC can assert AVH while AutoHold remains READY, not ACTIVE.
  sequence = [(0, 3), (1, 3), (0, 3)]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_oem_autohold_interlock_survives_short_avh_dropout():
  sequence = [(1, 2), (0, 3), (0, 3)]

  assert _classify_avh_sequence(sequence) == [True, True, True]


def test_canfd_oem_autohold_interlock_clears_after_release_grace():
  sequence = [(1, 2)] + [(0, 3)] * CANFD_AVH_RELEASE_GRACE_FRAMES
  results = _classify_avh_sequence(sequence)

  assert all(results[:-1])
  assert not results[-1]


def test_canfd_autohold_switch_log_does_not_report_hold_while_ready():
  # Ioniq 5 PE switch log: OFF -> READY -> OFF -> READY -> OFF, without braking.
  sequence = [(0, 0), (0, 3), (0, 0), (0, 3), (0, 0)]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_avh_release_state_does_not_start_oem_hold_latch():
  sequence = [(0, 3), (2, 3)]

  assert _classify_avh_sequence(sequence) == [False, False]
