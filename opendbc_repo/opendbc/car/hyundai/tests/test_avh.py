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
  for avh_state, acc_req in sequence:
    oem_hold_latched, avh_active_prev, release_grace_frames = update_canfd_avh_interlock_state(
      avh_state, acc_req, avh_active_prev, oem_hold_latched, release_grace_frames,
    )
    results.append(oem_hold_latched)
  return results


def test_canfd_oem_autohold_precedes_acc_request_and_latches_interlock():
  # OEM AutoHold log: AVH rises first, followed by the attempted SCC request.
  sequence = [(0, 0), (1, 0), (1, 1), (2, 1)]

  assert _classify_avh_sequence(sequence) == [False, True, True, True]


def test_canfd_soft_hold_follows_acc_request_without_latching_interlock():
  # Soft-hold log: SCC/TCS ACC_REQ rises first and AVH follows roughly 160 ms later.
  # A brief ACC_REQ dropout while AVH remains active must not reclassify the hold.
  sequence = [(0, 0), (0, 1), (1, 1), (1, 0), (2, 1), (0, 0)]

  assert _classify_avh_sequence(sequence) == [False] * len(sequence)


def test_canfd_oem_autohold_interlock_survives_short_avh_dropout():
  sequence = [(1, 0), (0, 0), (1, 1)]

  assert _classify_avh_sequence(sequence) == [True, True, True]


def test_canfd_oem_autohold_interlock_clears_after_release_grace():
  sequence = [(1, 0)] + [(0, 0)] * CANFD_AVH_RELEASE_GRACE_FRAMES
  results = _classify_avh_sequence(sequence)

  assert all(results[:-1])
  assert not results[-1]
