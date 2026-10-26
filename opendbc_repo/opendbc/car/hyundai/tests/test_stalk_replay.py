from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.can.parser import get_raw_value
from opendbc.car.hyundai.carcontroller import CarController, stalk_checksum

# real 0x3C1 frames, Ioniq 5 PE 10-26 rlog 00000007 (every lever change of the drive)
REAL = ['3e50300000000000', '5260304010000000', 'ce70304000000000', 'f880300000000000', 'de90304000000000',
        '97a0300000000000', '27b0300041000000', 'bbc0300001000000', '9fd0300000000000']


@pytest.mark.parametrize('frame', REAL)
def test_checksum_matches_real_frames(frame):
  dat = bytes.fromhex(frame)
  assert stalk_checksum(dat[1:]) == dat[0]


@pytest.mark.parametrize(('real', 'stalk', 'expected'), (
  ('f880300000000000', 1, 'de90304000000000'),  # off (counter 8) -> left with counter 9, the frame the car itself sent next
  ('97a0300000000000', 2, None),
))
def test_replay_is_the_next_lever_frame(real, stalk, expected):
  packer = CANPacker('hyundai_canfd_generated')
  sigs = packer.dbc.name_to_msg['BLINKER_STALKS'].sigs
  CS = SimpleNamespace(blinker_stalks={k: get_raw_value(bytes.fromhex(real), s) for k, s in sigs.items()})
  cc = SimpleNamespace(frame=0, blinker_stalk=stalk, CAN=SimpleNamespace(ECAN=0), packer=packer)
  (addr, dat, bus), = CarController.stalk_blinkers(cc, CS)
  assert (addr, bus) == (0x3C1, 0) and stalk_checksum(dat[1:]) == dat[0]
  assert get_raw_value(dat, sigs['COUNTER_ALT']) == (get_raw_value(bytes.fromhex(real), sigs['COUNTER_ALT']) + 1) % 16
  assert get_raw_value(dat, sigs['LEFT_BLINKER']) == (stalk == 1) and get_raw_value(dat, sigs['RIGHT_BLINKER']) == (stalk == 2)
  if expected:
    assert dat.hex() == expected


def test_repeated_state_direction_change_wrap_and_restart():
  packer = CANPacker('hyundai_canfd_generated')
  sigs = packer.dbc.name_to_msg['BLINKER_STALKS'].sigs
  stock = {k: get_raw_value(bytes.fromhex('e860300000000000'), s) for k, s in sigs.items()}
  CS = SimpleNamespace(blinker_stalks=stock)
  cc = SimpleNamespace(frame=0, blinker_stalk=1, CAN=SimpleNamespace(ECAN=0), packer=packer)

  def send():
    (addr, dat, bus), = CarController.stalk_blinkers(cc, CS)
    cc.frame += 5
    assert (addr, bus) == (0x3C1, 0)
    assert stalk_checksum(dat[1:]) == dat[0]
    return dat

  left = send()
  assert left.hex() == 'ce70304000000000'
  for _ in range(20):
    assert send() == left
  cc.blinker_stalk = 2
  right = send()
  assert get_raw_value(right, sigs['COUNTER_ALT']) == 8
  assert get_raw_value(right, sigs['RIGHT_BLINKER']) == 1
  assert send() == right
  stock['COUNTER_ALT'] = 15
  stock['LIGHT_KNOB_POSITION'] = 0
  off_knob = send()
  assert get_raw_value(off_knob, sigs['COUNTER_ALT']) == 0
  assert get_raw_value(off_knob, sigs['LIGHT_KNOB_POSITION']) == 0
  cc.blinker_stalk = 0
  assert CarController.stalk_blinkers(cc, CS) == []
  assert cc.stalk_counter is None
  stock['COUNTER_ALT'] = 3
  cc.blinker_stalk = 1
  assert get_raw_value(send(), sigs['COUNTER_ALT']) == 4
