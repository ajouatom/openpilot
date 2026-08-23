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
