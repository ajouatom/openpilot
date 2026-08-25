from opendbc.can import CANDefine
from opendbc.can.dbc import DBC
from opendbc.car.hyundai.carstate import CANFD_AVH_RELEASE_GRACE_FRAMES, update_canfd_avh_interlock_state


def test_canfd_avh_status_definition():
  dbc = DBC("hyundai_canfd_generated")
  signal = dbc.name_to_msg["ESP_STATUS"].sigs["AVH_Sta"]

  assert signal.start_bit == 192
  assert signal.size == 2
  assert signal.is_little_endian

  definitions = CANDefine("hyundai_canfd_generated").dv["ESP_STATUS"]["AVH_Sta"]
  assert definitions == {
    0: "NO_APPLY",
    1: "VEHICLE_IS_HELD_BY_THE_SERVICE_BRAKE",
    2: "BEING_RELEASED",
    3: "ERROR_INDICATOR",
  }


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
  avh_active_prev = False
  oem_hold_latched = False
  release_grace_frames = 0
  results = []
  for avh_state, acc_req, brake_pressed, soft_hold_active in sequence:
    oem_hold_latched, avh_active_prev, release_grace_frames = update_canfd_avh_interlock_state(
      avh_state, acc_req, brake_pressed, soft_hold_active, avh_active_prev, oem_hold_latched, release_grace_frames,
    )
    results.append(oem_hold_latched)
  return results


def test_canfd_oem_autohold_precedes_acc_request_and_latches_interlock():
  # OEM AutoHold log: driver braking is asserted when AVH rises.
  sequence = [(0, 0, False, False), (1, 0, True, False), (1, 1, False, False), (2, 1, False, False)]

  assert _classify_avh_sequence(sequence) == [False, True, True, True]


def test_canfd_soft_hold_follows_acc_request_without_latching_interlock():
  # Soft-hold log: SCC/TCS ACC_REQ rises first and AVH follows roughly 160 ms later.
  # A brief ACC_REQ dropout while AVH remains active must not reclassify the hold.
  sequence = [(0, 0, True, True), (0, 1, False, True), (1, 1, False, True),
              (1, 0, False, True), (2, 1, False, True), (0, 0, False, False)]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_cruise_stop_ignores_avh_during_acc_req_dropout():
  # K5 cruise-stop log: AVH is asserted without driver braking. Do not cancel
  # active cruise even if ACC_REQ has a transient timing gap at the AVH edge.
  sequence = [(0, 1, False, False), (1, 0, False, False), (1, 1, False, False), (0, 1, False, False)]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_oem_autohold_interlock_survives_short_avh_dropout():
  sequence = [(1, 0, True, False), (0, 0, False, False), (1, 1, False, False)]

  assert _classify_avh_sequence(sequence) == [True, True, True]


def test_canfd_oem_autohold_interlock_clears_after_release_grace():
  sequence = [(1, 0, True, False)] + [(0, 0, False, False)] * CANFD_AVH_RELEASE_GRACE_FRAMES
  results = _classify_avh_sequence(sequence)

  assert all(results[:-1])
  assert not results[-1]


def test_canfd_gv80_soft_hold_avh_precedes_delayed_acc_request():
  # GV80 incident: soft hold commands SCC first, AVH rises 22 ms later while
  # the driver brake is still asserted, and TCS.ACC_REQ follows another 8 ms
  # later. The soft-hold intent must own this hydraulic hold from its first AVH
  # frame instead of briefly latching it as OEM AutoHold.
  sequence = [
    (0, 0, True, True),
    (1, 0, True, True),
    (1, 1, True, True),
    (1, 1, False, True),
    (2, 0, False, True),
    (0, 0, False, False),
  ]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_avh_release_state_does_not_start_oem_hold_latch():
  sequence = [(0, 0, True, False), (2, 0, True, False)]

  assert _classify_avh_sequence(sequence) == [False, False]
