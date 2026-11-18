from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import CarController, stalk_checksum

# Ioniq 5 PE lever neutral, light knob 3 (byte2 0x30), COUNTER_ALT 8 (05_can/hba_ifs.md, blinker_route19)
STOCK = {"CHECKSUM_MAYBE": 0, "COUNTER_ALT": 8, "HIGHBEAM_FORWARD": 0, "LIGHT_KNOB_POSITION": 3, "HIGHBEAM_BACKWARD": 0,
         "LEFT_BLINKER": 0, "RIGHT_BLINKER": 0, "LEFT_BLINKER_TAP": 0, "RIGHT_BLINKER_TAP": 0}


def make_cc(on=True):
  return SimpleNamespace(highbeam_on_engage=on, highbeam_frame=None, highbeam_prev_enabled=False, frame=0,
                         CAN=SimpleNamespace(ECAN=0), packer=CANPacker('hyundai_canfd_generated'))


def drive(cc, enabled, frames, stalks=STOCK):
  sent = []
  for _ in range(frames):
    sent += [(cc.frame, m) for m in CarController.highbeam_push(cc, SimpleNamespace(enabled=enabled),
                                                                SimpleNamespace(blinker_stalks=dict(stalks) if stalks else None))]
    cc.frame += 1
  return sent


def test_first_engagement_pushes_the_high_beam_lever_once():
  cc = make_cc()
  assert drive(cc, False, 50) == []
  sent = drive(cc, True, 100)
  assert [f for f, _ in sent] == [50, 50 + carcontroller.HIGHBEAM_PRESS_FRAMES]
  (_, (addr, press, bus)), (_, (_, release, _)) = sent
  assert addr == 0x3C1 and bus == 0
  assert press[2] == 0x34 and release[2] == 0x30           # HIGHBEAM_FORWARD (byte2 bit2) on, then released
  assert press[1] >> 4 == 9 and release[1] >> 4 == 10      # one lever change each
  for dat in (press, release):
    assert dat[0] == stalk_checksum(dat[1:])
    # panda's 0x3C1 shape check (safety_hyundai_canfd.h): byte2 within 0x34, byte3 0x00/0x40, byte4 stock forms, tail 0
    assert dat[2] & ~0x34 == 0 and dat[3] in (0x00, 0x40) and dat[4] in (0x00, 0x01, 0x10, 0x41) and dat[5:] == bytes(3)
  # the lever is a toggle: disengage and engage again in the same start sends nothing
  assert drive(cc, False, 20) == [] and drive(cc, True, 50) == []


def test_off_or_no_stalk_message_sends_nothing():
  assert drive(make_cc(on=False), True, 50) == []
  cc = make_cc()
  assert drive(cc, True, 50, stalks=None) == [] and cc.highbeam_frame is None
