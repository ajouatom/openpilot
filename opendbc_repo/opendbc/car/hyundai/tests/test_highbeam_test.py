from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController, stalk_checksum

# Ioniq 5 PE lever neutral, light knob 3 (byte2 0x30), COUNTER_ALT 8 (05_can/hba_ifs.md, blinker_route19)
STOCK = {"CHECKSUM_MAYBE": 0, "COUNTER_ALT": 8, "HIGHBEAM_FORWARD": 0, "LIGHT_KNOB_POSITION": 3, "HIGHBEAM_BACKWARD": 0,
         "LEFT_BLINKER": 0, "RIGHT_BLINKER": 0, "LEFT_BLINKER_TAP": 0, "RIGHT_BLINKER_TAP": 0}
CGW1 = bytes([0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00])  # classic stock CGW1 (test_hyundai_palisade_kick.py)


def make_cc(test=0):
  cc = CarController.__new__(CarController)  # only the high-beam state, no CP/packer setup
  cc.__dict__.update(highbeam_frame=None, highbeam_kind=1, highbeam_test=test, highbeam_test_sent=False, frame=0,
                     cgw_last_tx=-1000, cgw_trial=0, CAN=SimpleNamespace(ECAN=0), packer=CANPacker('hyundai_canfd_generated'))
  return cc


def drive(cc, frames, stalks=STOCK):
  sent = []
  for _ in range(frames):
    sent += [(cc.frame, m) for m in cc.highbeam_push(SimpleNamespace(blinker_stalks=dict(stalks) if stalks else None))]
    cc.frame += 1
  return sent


def drive_cgw(cc, frames, raw=CGW1):
  sent = []
  for _ in range(frames):
    sent += [(cc.frame, m) for m in cc.cgw_highbeam(SimpleNamespace(cgw_test_raw=raw))]
    cc.frame += 1
  return sent


def test_canfd_push_once_then_rearm_at_zero():
  cc = make_cc(test=1)
  sent = drive(cc, 100)
  assert [f for f, _ in sent] == [0, carcontroller.HIGHBEAM_PRESS_FRAMES]
  (_, (addr, press, bus)), (_, (_, release, _)) = sent
  assert addr == 0x3C1 and bus == 0
  assert press[2] == 0x34 and release[2] == 0x30           # HIGHBEAM_FORWARD (byte2 bit2) on, then released
  assert press[1] >> 4 == 9 and release[1] >> 4 == 10      # one lever change each
  for dat in (press, release):
    assert dat[0] == stalk_checksum(dat[1:])
    # panda's 0x3C1 shape check (safety_hyundai_canfd.h): byte2 within 0x34, byte3 within 0x44, byte4 stock forms, tail 0
    assert dat[2] & ~0x34 == 0 and dat[3] & ~0x44 == 0 and dat[4] in (0x00, 0x01, 0x10, 0x41) and dat[5:] == bytes(3)
  assert drive(cc, 100) == []                              # value stays 1: no repeat
  cc.highbeam_test = 2                                     # straight from 1 to 2: still nothing until 0
  assert drive(cc, 50) == []
  cc.highbeam_test = 0
  assert drive(cc, 10) == []
  cc.highbeam_test = 2                                     # flash-to-pass: HIGHBEAM_BACKWARD (byte3 bit2)
  (_, (_, press, _)), (_, (_, release, _)) = drive(cc, 100)
  assert press[2] == 0x30 and press[3] == 0x04 and release[3] == 0x00


def test_off_or_pending_until_the_stalk_message_is_seen():
  assert drive(make_cc(test=0), 50) == []
  cc = make_cc(test=1)
  assert drive(cc, 20, stalks=None) == [] and cc.highbeam_frame is None
  assert len(drive(cc, 20)) == 2


def test_classic_can_sets_one_cgw1_bit():
  for test, byte, bit in ((1, 4, 0x01), (2, 6, 0x08)):
    cc = make_cc(test=test)
    sent = drive_cgw(cc, 100)
    assert [f for f, _ in sent] == [0, carcontroller.CGW_TX_FRAMES, 2 * carcontroller.CGW_TX_FRAMES]
    for _, (addr, data, bus) in sent:
      expected = bytearray(CGW1)
      expected[byte] |= bit
      assert (addr, data, bus) == (0x541, bytes(expected), 0)
    assert cc.cgw_last_tx == 2 * carcontroller.CGW_TX_FRAMES and drive_cgw(cc, 100) == []
  assert drive_cgw(make_cc(test=1), 50, raw=b"") == []
