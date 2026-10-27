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


def park_inputs(d, count, *, gear=0, wheel=0, driver=False, stock_direction=None, include_stock=True):
  for addr, size in ((0x35, 32), (0xA0, 24)):
    data = bytearray(size)
    data[2] = count % 256
    if addr == 0x35:
      data[24] = gear
    else:
      data[8] = wheel
    data[:2] = checksum(addr, data).to_bytes(2, 'little')
    d.rx(addr, data)
  if include_stock:
    for addr in SPAS:
      d.rx(addr, frame(addr, count % 256, stock_direction, d.bus).dat)
  data = bytearray.fromhex('12b0000000000000')
  if driver:
    data[4] = 1
    crc = 0
    for byte in data[1:]:
      crc ^= byte
      for _ in range(8):
        crc = ((crc << 1) ^ (0x1d if crc & 128 else 0)) & 255
    data[0] = crc ^ 0x65
  d.rx(0x3C1, data)


def parked(d, **kwargs):
  for i in range(30):
    d.advance(20000)
    park_inputs(d, i, **kwargs)


@pytest.mark.parametrize('flags', [4096, 4253])
def test_parked_comparison_no_uds_ten_seconds_and_cooldown(flags):
  d = Device(flags)
  parked(d)
  for count in range(30, 530):
    assert d.tx(0x165, frame(0x165, count % 256, None, 0).dat)
    assert d.tx(0x16A, frame(0x16A, count % 256, 'left', 0).dat)
    assert not d.tx(APRK, EXTENDED)
    assert not d.tx(APRK, DISABLE)
    assert not d.tx(APRK, PRESENT)
    d.advance(20000)
    park_inputs(d, count)
  assert not d.tx(0x165, frame(0x165, 530 % 256, None, 0).dat)
  assert not d.tx(0x16A, frame(0x16A, 530 % 256, 'left', 0).dat)
  parked(d)
  assert not d.tx(0x165, frame(0x165, 30, None, 0).dat)
  d.advance(30000000)
  parked(d)
  assert d.tx(0x165, frame(0x165, 30, None, 0).dat)


@pytest.mark.parametrize('bad', ['moving', 'gear', 'driver', 'controls', 'stock', 'stale', 'bus', 'disabled', 'bad_crc'])
def test_parked_comparison_rejects_missing_or_unsafe_inputs(bad):
  d = Device(flags=0 if bad == 'disabled' else 4096 | (16 if bad == 'bus' else 0), bus=1 if bad == 'bus' else 0)
  parked(d, wheel=1 if bad == 'moving' else 0, gear=5 if bad == 'gear' else 0,
         driver=bad == 'driver', stock_direction='right' if bad == 'stock' else None)
  if bad == 'controls':
    d.s.set_controls_allowed(True)
  if bad == 'stale':
    d.advance(100000)
  if bad == 'bad_crc':
    d.rx(0x35, bytes(32))
  assert not d.tx(0x165, frame(0x165, 30, None, d.bus).dat)
  assert not d.tx(0x16A, frame(0x16A, 30, 'left', d.bus).dat)


@pytest.mark.parametrize('bad', ['moving', 'gear', 'driver', 'stock', 'controls', 'stale'])
def test_parked_comparison_revokes_without_resuming(bad):
  d = Device()
  parked(d)
  assert d.tx(0x165, frame(0x165, 30, None, 0).dat)
  assert d.tx(0x16A, frame(0x16A, 30, 'left', 0).dat)
  d.advance(20000)
  park_inputs(d, 30, wheel=1 if bad == 'moving' else 0, gear=5 if bad == 'gear' else 0,
              driver=bad == 'driver', stock_direction='right' if bad == 'stock' else None)
  if bad == 'controls':
    d.s.set_controls_allowed(True)
  if bad == 'stale':
    d.advance(500000)
  assert not d.tx(0x16A, frame(0x16A, 31, 'left', 0).dat)
  d.s.set_controls_allowed(False)
  parked(d)
  assert not d.tx(0x165, frame(0x165, 30, None, 0).dat)


def test_parked_comparison_preserves_payload_crc_rate_direction_counter_checks():
  d = Device()
  parked(d)
  bad = bytearray(frame(0x165, 30, None, 0).dat)
  bad[4] = 1
  bad[:2] = checksum(0x165, bad).to_bytes(2, 'little')
  assert not d.tx(0x165, bad)
  assert d.tx(0x165, frame(0x165, 30, None, 0).dat)
  assert d.tx(0x16A, frame(0x16A, 30, 'left', 0).dat)
  assert not d.tx(0x16A, frame(0x16A, 31, 'left', 0).dat)
  d.advance(20000)
  park_inputs(d, 30)
  assert not d.tx(0x16A, frame(0x16A, 31, 'right', 0).dat)
  assert not d.tx(0x16A, frame(0x16A, 30, 'left', 0).dat)
  bad = bytearray(frame(0x16A, 31, 'left', 0).dat)
  bad[3] = 1
  bad[:2] = checksum(0x16A, bad).to_bytes(2, 'little')
  assert not d.tx(0x16A, bad)
  assert d.tx(0x16A, frame(0x16A, 31, 'left', 0).dat)


@pytest.mark.parametrize('flags', [4096, 4253])
@pytest.mark.parametrize('keep_stock', [False, True])
@pytest.mark.parametrize('reply', [None, b'\x50\x03', b'\x7f\x10\x22'])
def test_original_patch_parked_sequence_no_ack_required_and_ten_second_bound(flags, keep_stock, reply):
  from opendbc.car.hyundai.spas import PATCH_DISABLE
  d = Device(flags)
  parked(d)
  start = d.t
  assert d.tx(APRK, EXTENDED)
  if reply is not None:
    d.reply(reply)
  for i in range(1, 6):
    d.advance(10000)
    park_inputs(d, 29+i)
  assert d.tx(APRK, PATCH_DISABLE)
  count = 0
  for tick in range(6, 1000):
    d.advance(10000)
    park_inputs(d, 29+tick, include_stock=keep_stock)
    if tick % 100 == 0:
      assert d.tx(APRK, PRESENT)
    if tick % 2 == 0:
      for addr in SPAS:
        assert d.tx(addr, frame(addr, count % 256, 'left', 0).dat)
      count += 1
  d.advance(10000)
  park_inputs(d, 1029, include_stock=keep_stock)
  assert d.t-start == 10000000
  assert not d.tx(0x16A, frame(0x16A, count % 256, 'left', 0).dat)
  assert not d.tx(APRK, PRESENT)
  assert d.tx(APRK, DEFAULT)
  assert not d.tx(0x16A, frame(0x16A, count % 256, 'left', 0).dat)


@pytest.mark.parametrize('fault', ['moving', 'gear', 'driver', 'controls', 'stale', 'watchdog'])
def test_original_patch_parked_revocation_always_allows_default_restore(fault):
  from opendbc.car.hyundai.spas import PATCH_DISABLE
  d = Device()
  parked(d)
  assert d.tx(APRK, EXTENDED)
  assert not d.tx(APRK, PATCH_DISABLE)  # cannot skip session delay
  for i in range(1, 6):
    d.advance(10000)
    park_inputs(d, 29+i)
  assert d.tx(APRK, PATCH_DISABLE)
  d.advance(10000)
  park_inputs(d, 35)
  assert d.tx(0x165, frame(0x165, 0, 'right', 0).dat)
  assert d.tx(0x16A, frame(0x16A, 0, 'right', 0).dat)
  if fault == 'watchdog':
    for i in range(76):
      d.advance(20000)
      park_inputs(d, 36+i, include_stock=False)
  elif fault == 'stale':
    d.advance(500000)
  else:
    d.advance(20000)
    park_inputs(d, 36, wheel=1 if fault == 'moving' else 0,
                gear=5 if fault == 'gear' else 0, driver=fault == 'driver')
    if fault == 'controls':
      d.s.set_controls_allowed(True)
  assert not d.tx(0x16A, frame(0x16A, 1, 'right', 0).dat)
  assert d.tx(APRK, DEFAULT)
  assert not d.tx(APRK, PATCH_DISABLE)


@pytest.mark.parametrize('flags', [4096, 4253])
@pytest.mark.parametrize('fault', ['none', 'driver', 'moving', 'gear', 'stale', 'crc', 'knob', 'rate', 'direction', 'counter', 'gap', 'duration'])
def test_stalk_replay_native_guards(flags, fault):
  from opendbc.car.hyundai.stalk_trial import replay, crc
  d = Device(flags)
  parked(d)
  original = bytes.fromhex('12b0000000000000')
  packet = replay(original, 'left').dat
  assert d.tx(0x3C1, packet)
  assert not d.tx(APRK, EXTENDED)
  assert not d.tx(0x165, frame(0x165, 30, None, 0).dat)
  d.advance(10000 if fault == 'rate' else 160000 if fault == 'gap' else 50000)
  park_inputs(d, 30, driver=fault == 'driver', wheel=1 if fault == 'moving' else 0, gear=5 if fault == 'gear' else 0)
  if fault == 'stale':
    d.advance(600000)
  if fault == 'crc':
    packet = bytes([packet[0] ^ 1]) + packet[1:]
  if fault in ('knob', 'counter'):
    data = bytearray(packet)
    data[2 if fault == 'knob' else 1] ^= 0x10
    data[0] = crc(data[1:])
    packet = bytes(data)
  if fault == 'direction':
    packet = replay(original, 'right').dat
  if fault == 'duration':
    for tick in range(1, 200):
      assert d.tx(0x3C1, packet)
      d.advance(50000)
      park_inputs(d, 30+tick)
  assert d.tx(0x3C1, packet) == (fault == 'none')


def test_stalk_requires_opt_in_and_parked_freshness():
  from opendbc.car.hyundai.stalk_trial import replay
  packet = replay(bytes.fromhex('12b0000000000000'), 'right').dat
  for flags in (0, 4096):
    d = Device(flags)
    assert not d.tx(0x3C1, packet)
    if not flags:
      parked(d)
      assert not d.tx(0x3C1, packet)


def test_stalk_driver_change_latches_off_even_before_next_transmit():
  from opendbc.car.hyundai.stalk_trial import replay
  d = Device(4253)
  parked(d)
  packet = replay(bytes.fromhex('12b0000000000000'), 'left').dat
  assert d.tx(0x3C1, packet)
  d.advance(20000)
  park_inputs(d, 30, driver=True)
  d.advance(30000)
  park_inputs(d, 31)
  assert not d.tx(0x3C1, packet)
