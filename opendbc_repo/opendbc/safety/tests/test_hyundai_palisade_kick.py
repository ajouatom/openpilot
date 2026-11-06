# CanfdBlinkerKick on the classic-CAN Palisade: while driving, the host's CGW1 (0x541) frame is held by the safety
# model and sent right behind the next stock CGW1 (safety_rx_tx_hook) instead of being queued next to the BCM's own
# 10 Hz frame (00000e46 10-27: bus errors at the start of direct sending). The parked PalisadeBlinkerTest still sends
# directly, and so does CANFD_SPAS_DIRECT (CanfdBlinkerKickMode 2/3).
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py

PARAM = 32768  # PALISADE_CGW_TEST, as interface.py sets it for every Palisade
DIRECT = 1024  # CANFD_SPAS_DIRECT
CGW1 = 0x541
STOCK = bytes([0x01, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00])


def frame(direction, lamp):
  data = bytearray(STOCK)
  data[2] |= 0x08 if direction == 1 and lamp else 0
  data[7] |= 0x40 if direction == 2 and lamp else 0
  data[5] |= 0x04 if direction == 1 else 0x02
  return bytes(data)


class Device:
  def __init__(self, param=PARAM):
    self.s = libsafety_py.libsafety
    assert self.s.set_safety_hooks(CarParams.SafetyModel.hyundai, param) == 0
    self.s.init_tests()
    self.t = 1_000_000
    self.s.set_timer(self.t)

  def advance(self, us):
    self.t += us
    self.s.set_timer(self.t)

  def stock(self, data=STOCK):
    """a stock CGW1 arrives on bus 0: what panda sends right behind it, or None"""
    pkt = libsafety_py.make_CANPacket(CGW1, 0, data)
    assert self.s.safety_rx_hook(pkt)
    out = libsafety_py.ffi.new("CANPacket_t *")
    if not self.s.safety_rx_tx_hook(pkt, out):
      return None
    assert out[0].addr == CGW1 and out[0].bus == 0
    return bytes(out[0].data)[:8]

  def tx(self, data):
    pkt = libsafety_py.make_CANPacket(CGW1, 0, data)
    self.s.safety_tx_buffered_for_fwd = False
    return bool(self.s.safety_tx_hook(pkt)), bool(self.s.safety_tx_buffered_for_fwd)


def test_driving_frame_is_held_and_sent_behind_the_stock_one():
  d = Device()
  d.stock()  # no gear / wheel frames: not the parked test
  d.advance(30_000)
  assert d.tx(frame(1, True)) == (True, True)  # accepted, held
  d.advance(70_000)
  assert d.stock() == frame(1, True)
  assert d.stock() is None  # sent once
  d.advance(60_000)
  assert d.tx(frame(1, False)) == (True, True)
  d.advance(200_000)  # older than HYUNDAI_CGW_PB_HOLD_US: dropped
  assert d.stock() is None


def test_stock_echoing_our_lamp_bit_does_not_block_the_kick():
  d = Device()
  d.stock(frame(1, True)[:5] + STOCK[5:])  # BCM echoes the left lamp bit, lever still neutral
  d.advance(30_000)
  assert d.tx(frame(1, False)) == (True, True)


def test_held_frame_is_rechecked_against_the_stock_frame_it_rides_behind():
  d = Device()
  d.stock()
  d.advance(30_000)
  assert d.tx(frame(1, True)) == (True, True)
  lever = bytearray(STOCK); lever[5] |= 0x02  # the driver moved the lever meanwhile: theirs
  d.advance(20_000)
  assert d.stock(bytes(lever)) is None
  d.stock()
  d.advance(60_000)  # past the 50 ms rate limit
  assert d.tx(frame(2, True)) == (True, True)
  door = bytearray(STOCK); door[1] |= 0x01  # another field changed: our copy is stale
  assert d.stock(bytes(door)) is None
  d.stock()
  d.advance(60_000)
  assert d.tx(frame(2, False)) == (True, True)
  echo = bytearray(STOCK); echo[7] |= 0x40  # the BCM echoing our lamp bit is fine
  assert d.stock(bytes(echo)) == frame(2, False)


def test_driver_lever_or_hazard_blocks_everything():
  d = Device()
  hazard = bytearray(STOCK); hazard[4] |= 0x02
  d.stock(bytes(hazard))
  assert d.tx(frame(1, True)) == (False, False)
  lever = bytearray(STOCK); lever[5] |= 0x02
  d.stock(bytes(lever))
  assert d.tx(frame(2, True)) == (False, False)
  d.stock()
  bad = bytearray(frame(1, True)); bad[1] = 0x55  # a byte that is not lamp or lever
  assert d.tx(bytes(bad)) == (False, False)


def test_direct_flag_queues_the_frame_at_once():
  d = Device(PARAM | DIRECT)
  d.stock()
  d.advance(30_000)
  assert d.tx(frame(2, True)) == (True, False)
  assert d.stock() is None


def test_stale_or_missing_stock_frame_rejects():
  d = Device()
  assert d.tx(frame(1, True)) == (False, False)  # no stock frame yet
  d.stock()
  d.advance(300_000)
  assert d.tx(frame(1, True)) == (False, False)
