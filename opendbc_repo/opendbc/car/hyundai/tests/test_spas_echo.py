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
    assert dat[3:16] == GV80_SPAS2[3:16] and dat[16] == blink << 5
    # byte 18 = 0x40 with a side request, as the parking ECU's own lamp frames (11-20 auto-parking logs); idle stays 0
    assert dat[18] == (0x40 if blink else 0) and dat[17] == GV80_SPAS2[17] and dat[19:] == GV80_SPAS2[19:]
    assert not any(spas1[3:])


def test_gv80_menu_test_3_4_sends_the_parking_active_body():
  # CanfdBlinkerMenuTest 3/4 (11-20, 02_logs s76): park adds the stock lamp frames' parking-active body, only with the GV80
  # template and only on a side request; without park the frame is the plain echo (idle b5-b7 never copied)
  packer = CANPacker('hyundai_canfd_generated')
  sigs = packer.dbc.name_to_msg['SPAS2'].sigs
  template = {k: get_raw_value(GV80_SPAS2, s) for k, s in sigs.items()}
  spas2 = lambda *a, **kw: hyundaicanfd.create_spas_messages(packer, SimpleNamespace(ECAN=0), 0, *a, **kw)[-1][1]
  for left, right, blink in ((True, False, 3), (False, True, 4)):
    dat = spas2(left, right, template, park=True)
    assert dat[3:8] == bytes([0x26, 0x41, 0x19, 0x04, 0x01]) and dat[15] == 0x13 and dat[16] == blink << 5 and dat[18] == 0x40
    assert dat[10:14] == GV80_SPAS2[10:14] and dat[14] == GV80_SPAS2[14] and dat[20] == GV80_SPAS2[20]
  assert spas2(True, False, template)[3:8] == GV80_SPAS2[3:8]  # menu 1/2: no body
  assert spas2(False, False, template, hazard=True, park=True)[3:8] == GV80_SPAS2[3:8]  # hazards never carry it
  assert spas2(False, False, template, park=True)[3:8] == GV80_SPAS2[3:8]  # idle
  other = spas2(True, False, None, park=True)  # no template (every other car): zero-filled as before
  assert other[3:16] == bytes(13) and other[16] == 3 << 5 and not any(other[17:])
  busy = dict(template, BYTE5=0x15, BYTE6=0x08, BYTE7=0x0d)  # stock parking values in the template are not echoed
  assert spas2(True, False, busy)[5:8] == bytes(3)


def test_other_cars_stay_zero_filled():
  packer = CANPacker('hyundai_canfd_generated')
  (_, _, _), (_, dat, _) = hyundaicanfd.create_spas_messages(packer, SimpleNamespace(ECAN=0), 0, True, False)
  assert dat[3:16] == bytes(13) and dat[16] == 3 << 5 and not any(dat[17:])
