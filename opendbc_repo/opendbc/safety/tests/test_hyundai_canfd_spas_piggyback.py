# CanfdBlinkerKick on a car whose parking ECU keeps sending SPAS1/SPAS2 (Ioniq 5 PE: no 0x7B9 reply, 10-24..11-05):
# while the stock stream is on bus 0, panda holds our frame and sends it right behind the next stock frame
# (safety_rx_tx_hook) instead of queueing it at a random moment next to the stock one (same-id collision, bus-off).
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.spas import SPAS, frame
from opendbc.safety.tests.libsafety import libsafety_py

PARAM = 1 | 4 | 8 | 16  # EV gas, long, camera SCC, HDA2 (Ioniq 5 PE)
SPAS1, SPAS2 = 0x165, 0x16A


class Device:
  def __init__(self, param=PARAM):
    self.s = libsafety_py.libsafety
    assert self.s.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, param) == 0
    self.s.init_tests()
    self.t = 1_000_000
    self.s.set_timer(self.t)

  def advance(self, us):
    self.t += us
    self.s.set_timer(self.t)

  def tx(self, addr, counter, blink="left"):
    pkt = libsafety_py.make_CANPacket(addr, 0, frame(addr, counter, blink, 0).dat)
    self.s.safety_tx_buffered_for_fwd = False
    allowed = bool(self.s.safety_tx_hook(pkt))
    return allowed, bool(self.s.safety_tx_buffered_for_fwd)

  def stock(self, addr, counter):
    """a stock frame arrives on bus 0: what panda sends right behind it, or None"""
    pkt = libsafety_py.make_CANPacket(addr, 0, frame(addr, counter, None, 0).dat)
    assert self.s.safety_rx_hook(pkt)
    out = libsafety_py.ffi.new("CANPacket_t *")
    if not self.s.safety_rx_tx_hook(pkt, out):
      return None
    assert out[0].addr == addr and out[0].bus == 0
    return bytes(out[0].data)[:SPAS[addr]]


def test_direct_while_the_ecu_is_silent():
  d = Device()
  assert d.tx(SPAS2, 10) == (True, False)  # no stock stream seen: queued as before
  d.stock(SPAS2, 1)
  d.advance(150_000)  # stream gone for longer than HYUNDAI_SPAS_PB_STOCK_US
  assert d.tx(SPAS2, 11) == (True, False)
  assert d.stock(SPAS2, 2) is None


def test_held_and_sent_right_behind_the_next_stock_frame():
  d = Device()
  d.stock(SPAS2, 1)
  d.advance(7_000)
  assert d.tx(SPAS2, 10) == (True, True)  # accepted, but held instead of queued
  d.advance(13_000)
  assert d.stock(SPAS2, 2) == bytes(frame(SPAS2, 10, "left", 0).dat)
  assert d.stock(SPAS2, 3) is None  # sent once
  # newest frame wins while held; SPAS1 has its own slot
  d.tx(SPAS2, 11)
  assert d.tx(SPAS2, 12) == (True, True)
  assert d.tx(SPAS1, 5) == (True, False)  # no stock SPAS1 seen yet
  d.stock(SPAS1, 1)
  assert d.tx(SPAS1, 6) == (True, True)
  assert d.stock(SPAS2, 4) == bytes(frame(SPAS2, 12, "left", 0).dat)
  assert d.stock(SPAS1, 2) == bytes(frame(SPAS1, 6, "left", 0).dat)


def test_direct_flag_turns_the_piggyback_off():
  d = Device(PARAM | 1024)  # CANFD_SPAS_DIRECT: CanfdBlinkerKickMode 2/3
  d.stock(SPAS2, 1)
  assert d.tx(SPAS2, 10) == (True, False)  # queued at once, next to the stock stream
  assert d.stock(SPAS2, 2) is None


def test_a_stale_held_frame_is_dropped():
  d = Device()
  d.stock(SPAS2, 1)
  assert d.tx(SPAS2, 10) == (True, True)
  d.advance(60_000)  # longer than HYUNDAI_SPAS_PB_HOLD_US: the host stopped sending
  assert d.stock(SPAS2, 2) is None


def test_invalid_frames_are_neither_sent_nor_held():
  d = Device()
  d.stock(SPAS2, 1)
  bad = bytearray(frame(SPAS2, 10, "left", 0).dat)
  bad[20] = 0x55
  pkt = libsafety_py.make_CANPacket(SPAS2, 0, bytes(bad))
  d.s.safety_tx_buffered_for_fwd = False
  assert not d.s.safety_tx_hook(pkt) and not d.s.safety_tx_buffered_for_fwd
  assert d.stock(SPAS2, 2) is None
