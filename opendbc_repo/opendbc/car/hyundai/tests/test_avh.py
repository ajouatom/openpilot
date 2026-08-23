from opendbc.can import CANDefine
from opendbc.can.dbc import DBC


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
