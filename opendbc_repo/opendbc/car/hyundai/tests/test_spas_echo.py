from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car.hyundai import hyundaicanfd

# GV80 stock 0x16A, 10-26 rlog 00000331 (the only body it sent, checksum/counter zeroed)
GV80_SPAS2 = bytes.fromhex('000000' '0101' '0000000000' '80808080' '00' '03' '00' '000000' '10') + bytes(11)


def test_gv80_spas2_echo_keeps_stock_bytes():
  packer = CANPacker('hyundai_canfd_generated')
  sigs = packer.dbc.name_to_msg['SPAS2'].sigs
  template = {k: get_raw_value(GV80_SPAS2, s) for k, s in sigs.items()}
  for blink, left, right in ((3, True, False), (4, False, True), (0, False, False)):
    (_, spas1, _), (addr, dat, _) = hyundaicanfd.create_spas_messages(packer, SimpleNamespace(ECAN=0), 0, left, right, template)
    assert addr == 0x16A and get_raw_value(dat, sigs['BLINKER_CONTROL']) == blink
    assert dat[3:16] == GV80_SPAS2[3:16] and dat[17:] == GV80_SPAS2[17:] and dat[16] == blink << 5
    assert not any(spas1[3:])


def test_other_cars_stay_zero_filled():
  packer = CANPacker('hyundai_canfd_generated')
  (_, _, _), (_, dat, _) = hyundaicanfd.create_spas_messages(packer, SimpleNamespace(ECAN=0), 0, True, False)
  assert dat[3:16] == bytes(13) and dat[16] == 3 << 5 and not any(dat[17:])
