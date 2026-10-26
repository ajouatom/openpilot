import pytest

from opendbc.car.structs import CarParams
from opendbc.car.hyundai.spas import SPAS, APRK, RESPONSE, EXTENDED, DISABLE, ENABLE, DEFAULT, PRESENT, frame, checksum
from opendbc.safety.tests.libsafety import libsafety_py


class Device:
  def __init__(self, flags=4096, bus=0):
    self.s = libsafety_py.libsafety
    assert self.s.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, flags) == 0
    self.s.init_tests()
    self.t = 1_000_000
    self.bus = bus
    self.s.set_timer(self.t)

  def advance(self, us):
    self.t += us
    self.s.set_timer(self.t)

  def tx(self, addr, data, bus=None):
    return bool(self.s.safety_tx_hook(libsafety_py.make_CANPacket(addr, self.bus if bus is None else bus, data)))

  def rx(self, addr, data, bus=None):
    return self.s.safety_rx_hook(libsafety_py.make_CANPacket(addr, self.bus if bus is None else bus, data))

  def stock(self):
    for i in range(10):
      self.advance(20000)
      for a in SPAS:
        self.rx(a, frame(a, i, None, self.bus).dat)

  def reply(self, body):
    self.rx(RESPONSE, bytes([len(body)]) + body + bytes(7 - len(body)))

  def open(self):
    self.stock()
    assert self.tx(APRK, EXTENDED)
    self.advance(10000)
    self.reply(b"\x50\x03")
    assert self.tx(APRK, DISABLE)
    self.advance(10000)
    self.reply(b"\x68\x03")
    self.advance(110000)


@pytest.mark.parametrize("flags,bus", [(4096, 0), (4096 | 16, 1), (4096 | 16 | 8, 0), (4096 | 4, 0), (4096 | 16 | 128, 1)])
def test_real_hooks_authorize_exact_compatible_bus_after_ack(flags, bus):
  d = Device(flags, bus)
  d.open()
  assert d.tx(0x165, frame(0x165, 10, None, bus).dat)
  assert d.tx(0x16A, frame(0x16A, 10, "left", bus).dat)
  assert not d.tx(0x16A, frame(0x16A, 11, "left", bus).dat, (bus + 1) % 3)
  assert not d.tx(0x16A, frame(0x16A, 11, "left", bus).dat)  # too fast
  d.advance(20000)
  assert d.tx(0x16A, frame(0x16A, 11, "left", bus).dat)


@pytest.mark.parametrize("flags", [0, 4, 8, 16, 16 | 4, 16 | 8 | 4 | 128, 2048])
def test_disabled_never_admits_uds_or_spas(flags):
  bus = 1 if flags & 16 and not flags & 8 else 0
  d = Device(flags, bus)
  d.stock()
  for data in (EXTENDED, DISABLE, ENABLE, DEFAULT, PRESENT, bytes.fromhex("0328830100000000")):
    assert not d.tx(APRK, data)
  for a in SPAS:
    assert not d.tx(a, frame(a, 0, "left", bus).dat)


def test_requires_stock_and_each_positive_reply():
  d = Device()
  assert not d.tx(APRK, EXTENDED)
  d.stock()
  assert d.tx(APRK, EXTENDED)
  assert not d.tx(APRK, DISABLE)
  d.reply(b"\x50\x01")
  assert not d.tx(APRK, DISABLE)
  d.reply(b"\x50\x03")
  assert d.tx(APRK, DISABLE)
  d.advance(110000)
  assert not d.tx(0x16A, frame(0x16A, 10, "left", 0).dat)
  d.reply(b"\x68\x03")
  d.advance(110000)
  assert d.tx(0x16A, frame(0x16A, 10, "left", 0).dat)


@pytest.mark.parametrize("byte,value", [(3, 1), (16, 0x20), (16, 0xA0), (16, 0x61), (31, 1)])
def test_rejects_non_indicator_payload_even_with_valid_crc(byte, value):
  d = Device()
  d.open()
  data = bytearray(frame(0x16A, 10, "left", 0).dat)
  data[byte] = value
  data[:2] = checksum(0x16A, data).to_bytes(2, "little")
  assert not d.tx(0x16A, data)


def test_crc_dlc_counter_and_recovery():
  d = Device()
  d.open()
  data = frame(0x16A, 10, "left", 0).dat
  assert not d.tx(0x16A, data[:-8])
  assert not d.tx(0x16A, bytes([data[0] ^ 1]) + data[1:])
  assert d.tx(0x16A, data)
  d.advance(20000)
  assert not d.tx(0x16A, data)
  assert d.tx(0x16A, frame(0x16A, 11, None, 0).dat)
  assert d.tx(APRK, ENABLE)
  assert d.tx(APRK, DEFAULT)
  d.advance(20000)
  assert not d.tx(0x16A, frame(0x16A, 12, "left", 0).dat)


@pytest.mark.parametrize("cause", ["stock", "watchdog", "lease", "reinitialize"])
def test_session_cannot_continue_after_revocation(cause):
  d = Device()
  d.open()
  assert d.tx(0x16A, frame(0x16A, 10, "left", 0).dat)
  if cause == "stock":
    d.rx(0x165, frame(0x165, 11, None, 0).dat)
  elif cause == "watchdog":
    d.advance(1600000)
  elif cause == "lease":
    d.advance(300000001)
  else:
    d.s.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, 4096)
  d.advance(20000)
  assert not d.tx(0x16A, frame(0x16A, 11, "left", 0).dat)
  assert d.tx(APRK, ENABLE) and d.tx(APRK, DEFAULT)


def test_disallow_arbitrary_diagnostics_and_suppressed_ack_disable():
  d = Device()
  d.stock()
  for data in (bytes.fromhex("0328830100000000"), bytes.fromhex("023e000000000000"), bytes.fromhex("0211030000000000"), EXTENDED[:-1] + b"\x01"):
    assert not d.tx(APRK, data)
