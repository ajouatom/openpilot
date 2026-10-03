import binascii
import ctypes
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest


LAYOUTS = {0x161: 32, 0x162: 32, 0x1E0: 16, 0x1EA: 32, 0x200: 8}
DIRECT_TX = 2048


def checksum(address, data):
  data = bytearray(data)
  crc = binascii.crc_hqx(data[2:] + address.to_bytes(2, "little"), 0)
  crc ^= {8: 0x5F29, 16: 0x041D, 24: 0x819D, 32: 0x9F5B}[len(data)]
  data[:2] = crc.to_bytes(2, "little")
  return bytes(data)


def message(address, counter=0, marker=17, length=None):
  size = LAYOUTS[address] if length is None else length
  data = bytearray((i + marker) % 256 for i in range(size))
  data[2] = counter
  return checksum(address, data)


@pytest.fixture(scope="session")
def native(tmp_path_factory):
  root = next(p for p in Path(__file__).resolve().parents if (p / "panda/board").is_dir())
  output = tmp_path_factory.mktemp("cluster-native") / ("safety.dll" if sys.platform == "win32" else "safety.so")
  compiler = shlex.split(os.environ.get("CC", "cc"))
  subprocess.run([*compiler, "-shared", "-fPIC", "-O2", f"-I{root / 'panda/board'}",
                  f"-I{root / 'opendbc_repo/opendbc/safety'}", f"-I{root / 'panda'}",
                  str(Path(__file__).with_name("hyundai_canfd_cluster.c")), "-o", str(output)], check=True)
  return load_native(output)


def load_native(output):
  lib = ctypes.CDLL(str(output))
  lib.alt2_test_init.argtypes = [ctypes.c_int]
  lib.cluster_test_packet.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_uint8),
                                    ctypes.c_uint32, ctypes.c_bool, ctypes.c_bool, ctypes.c_bool]
  lib.cluster_test_packet.restype = ctypes.c_int
  return lib


class Hooks:
  def __init__(self, lib, param=157):
    self.lib = lib
    lib.alt2_test_init(param)

  def packet(self, address, data, now, tx=False, bus=None, extended=False, relay=False):
    if bus is None:
      bus = 0 if tx else 2
    buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
    result = self.lib.cluster_test_packet(address, bus, len(data), buf, now, tx, extended, relay)
    return result, bytes(buf)

  def tx(self, address, data, now=1000, **kwargs):
    return self.packet(address, data, now, tx=True, **kwargs)[0]

  def rx(self, address, data, now=2000, **kwargs):
    return self.packet(address, data, now, **kwargs)


@pytest.fixture
def hooks(native):
  return Hooks(native)


@pytest.mark.parametrize("address", LAYOUTS)
@pytest.mark.parametrize("counter", [0, 1, 15, 16, 254, 255])
def test_one_rx_one_output_with_original_counter_and_independent_crc(hooks, address, counter):
  host = message(address, counter=83, marker=42)
  stock = message(address, counter=counter)
  assert hooks.tx(address, host) == 3  # accepted, consumed; NO direct transmission
  target, output = hooks.rx(address, stock)
  assert target == 0
  assert output[2] == counter
  assert output[3:] == host[3:]
  assert output == checksum(address, output)


@pytest.mark.parametrize("address", LAYOUTS)
@pytest.mark.parametrize("period", [5000, 10000, 20000, 33333, 50000, 100000, 200000, 1000000])
def test_vehicle_rate_not_host_rate_and_no_fifo_backlog(hooks, address, period):
  # 20 Hz producer against 1..200 Hz stock, including a non-integral ratio.
  host = None
  update = 1000
  for index, now in enumerate(range(2000, 3000001, period)):
    while update <= now:
      host = message(address, counter=77, marker=(update // 50000) % 200)
      assert hooks.tx(address, host, update) == 3
      update += 50000
    stock = message(address, counter=index % 256, marker=222)
    target, output = hooks.rx(address, stock, now)
    assert target == 0
    assert output[2] == index % 256
    assert output[3:] == host[3:]  # newest value, never a queued older value
    assert output == checksum(address, output)


@pytest.mark.parametrize("address", LAYOUTS)
def test_startup_host_loss_resume_and_timer_wrap(hooks, address):
  stock = message(address)
  host = message(address, marker=42)
  assert hooks.rx(address, stock, 0) == (0, stock)
  assert hooks.tx(address, host, 0xFFFFFF00) == 3
  assert hooks.rx(address, stock, (0xFFFFFF00 + 149999) % 2**32)[1][3:] == host[3:]
  assert hooks.rx(address, stock, (0xFFFFFF00 + 150000) % 2**32) == (0, stock)
  assert hooks.rx(address, stock, 200000) == (0, stock)
  assert hooks.tx(address, host, 210000) == 3
  assert hooks.rx(address, stock, 210001)[1][3:] == host[3:]
  hooks.lib.alt2_test_init(157)
  assert hooks.rx(address, stock, 210002) == (0, stock)


@pytest.mark.parametrize("address", LAYOUTS)
def test_absent_stock_has_no_direct_output_or_backlog_on_return(hooks, address):
  for now in range(0, 2000001, 50000):
    host = message(address, marker=now // 50000)
    assert hooks.tx(address, host, now) == 3
  assert hooks.rx(address, message(address), 2000001)[1][3:] == host[3:]
  # Host stopped too: the stored copy must expire even without intervening RX.
  assert hooks.rx(address, message(address), 2200000) == (0, message(address))


@pytest.mark.parametrize("address", LAYOUTS)
@pytest.mark.parametrize("bad_kind", ["crc", "length", "extended"])
def test_bad_stock_is_not_repaired_and_invalidates_cache(hooks, address, bad_kind):
  host = message(address, marker=42)
  assert hooks.tx(address, host) == 3
  stock = bytearray(message(address, length=16 if LAYOUTS[address] != 16 else 32) if bad_kind == "length" else message(address))
  if bad_kind == "crc":
    stock[0] ^= 1
  stock = bytes(stock)
  assert hooks.rx(address, stock, extended=bad_kind == "extended") == (0, stock)
  assert hooks.rx(address, message(address), 3000) == (0, message(address))


@pytest.mark.parametrize("address", LAYOUTS)
@pytest.mark.parametrize("bad_kind", ["crc", "length", "extended", "bus", "relay"])
def test_rejected_host_cannot_poison_forwarding(hooks, address, bad_kind):
  host = bytearray(message(address, marker=42, length=16 if LAYOUTS[address] != 16 else 32)
                   if bad_kind == "length" else message(address, marker=42))
  if bad_kind == "crc":
    host[0] ^= 1
  result = hooks.tx(address, bytes(host), extended=bad_kind == "extended", bus=2 if bad_kind == "bus" else 0,
                    relay=bad_kind == "relay")
  assert not result & 1
  stock = message(address)
  assert hooks.rx(address, stock, 3000) == (0, stock)


def test_caches_are_independent_and_wrong_direction_untouched(hooks):
  for address in LAYOUTS:
    assert hooks.tx(address, message(address, marker=address % 200)) == 3
  for address in LAYOUTS:
    raw = message(address)
    assert hooks.rx(address, raw, bus=0) == (2, raw)
    assert hooks.rx(address, raw, bus=1) == (-1, raw)
    assert hooks.rx(address, raw)[1][3:] == message(address, marker=address % 200)[3:]
    assert hooks.rx(address, raw, relay=True) == (-1, raw)


@pytest.mark.parametrize("param", [0, 1, 4, 5, 16, 20, 149])
def test_non_camera_modes_keep_direct_tx(native, param):
  hooks = Hooks(native, param)
  result = hooks.tx(0x1E0, message(0x1E0))
  assert not result & 2  # no cache consumption, regardless of whitelist result


@pytest.mark.parametrize("param", [8, 9, 12, 13])
def test_camera_hda1_whitelist_still_blocks_1ea(native, param):
  hooks = Hooks(native, param)
  assert hooks.tx(0x1EA, message(0x1EA, marker=42)) == 0
  raw = message(0x1EA)
  assert hooks.rx(0x1EA, raw) == (0, raw)


@pytest.mark.parametrize("address", LAYOUTS)
def test_counter_repeats_skips_and_wrap_are_preserved_from_rx(hooks, address):
  assert hooks.tx(address, message(address, counter=77)) == 3
  for counter in [253, 253, 255, 0, 9, 10]:
    _, out = hooks.rx(address, message(address, counter=counter))
    assert out[2] == counter
    assert out == checksum(address, out)


@pytest.mark.parametrize("address", LAYOUTS)
def test_changing_rx_rate_jitter_host_pause_and_burst(hooks, address):
  # Independent expected latest-value model across abrupt rate/phase changes.
  events = [(t, True) for t in range(1000, 2000000, 50000) if not 600000 <= t < 1000000]
  events += [(450001, True), (450002, True)]
  events += [(t, False) for t in range(2000, 400000, 10000)]
  events += [(t, False) for t in range(400000, 1200000, 100000)]
  events += [(t + (i % 3) * 3000, False) for i, t in enumerate(range(1200000, 2000000, 33333))]
  latest = None
  for index, (now, tx) in enumerate(sorted(events)):
    if tx:
      latest = now, message(address, marker=index % 256)
      assert hooks.tx(address, latest[1], now) == 3
    else:
      raw = message(address, counter=index % 256, marker=231)
      dest, out = hooks.rx(address, raw, now)
      assert dest == 0
      if latest is None or now - latest[0] >= 150000:
        assert out == raw
      else:
        assert out[3:] == latest[1][3:]
        assert out[2] == raw[2]
        assert out == checksum(address, out)


@pytest.mark.parametrize("address", LAYOUTS)
def test_other_tx_bus_does_not_populate_cluster_cache(hooks, address):
  # Some HDA2 whitelist entries allow bus 1; they keep their direct-TX path.
  assert not hooks.tx(address, message(address, marker=42), bus=1) & 2
  raw = message(address)
  assert hooks.rx(address, raw) == (0, raw)


@pytest.mark.parametrize("param", [29, 157])
@pytest.mark.parametrize("address", LAYOUTS)
def test_direct_tx_without_stock_rx_and_legacy_forwarding_timeout(native, param, address):
  hooks = Hooks(native, param | DIRECT_TX)
  stock = message(address, counter=77)
  assert hooks.rx(address, stock, 100000) == (0, stock)
  # Direct mode preserves the host's counter/CRC, including +2 and wrap.
  for index, counter in enumerate([252, 254, 0, 2, 3]):
    now = 200000 + index * 50000
    host = message(address, counter=counter, marker=42)
    assert hooks.packet(address, host, now, tx=True) == (1, host)
    assert hooks.rx(address, stock, now + 1) == (-1, stock)
  # When host TX stops, the original 20 Hz + 20 ms suppression expires.
  assert hooks.rx(address, stock, now + 69999) == (-1, stock)
  assert hooks.rx(address, stock, now + 70000) == (0, stock)


@pytest.mark.parametrize("address", LAYOUTS)
def test_direct_tx_mode_switch_clears_cache(native, address):
  hooks = Hooks(native)
  host = message(address, marker=42)
  stock = message(address)
  assert hooks.tx(address, host, 100000) == 3
  native.alt2_test_init(157 | DIRECT_TX)
  assert hooks.rx(address, stock, 200000) == (0, stock)
  assert hooks.tx(address, host, 210000) == 1
  native.alt2_test_init(157)
  assert hooks.rx(address, stock, 220000) == (0, stock)
  assert hooks.tx(address, host, 230000) == 3
  assert hooks.rx(address, stock, 230001)[1][3:] == host[3:]


@pytest.mark.parametrize("address", LAYOUTS)
def test_direct_tx_retains_whitelist_and_relay_protection(native, address):
  hooks = Hooks(native, 157 | DIRECT_TX)
  assert hooks.tx(address, message(address), relay=True) == 0
  assert hooks.tx(address, message(address), bus=2) == 0
  wrong_size = 16 if LAYOUTS[address] != 16 else 32
  assert hooks.tx(address, message(address, length=wrong_size)) == 0


@pytest.mark.parametrize("param", [8, 9, 12, 13])
def test_direct_tx_does_not_expand_hda1_allowlist(native, param):
  hooks = Hooks(native, param | DIRECT_TX)
  assert hooks.tx(0x1EA, message(0x1EA)) == 0


@pytest.mark.parametrize("param", [1, 5, 17, 21, 145, 149])
def test_direct_tx_flag_has_no_effect_without_camera_scc(native, param):
  def run(selected):
    hooks = Hooks(native, selected)
    result = []
    for address in LAYOUTS:
      result.append(hooks.tx(address, message(address), 100000))
      result.append(hooks.rx(address, message(address), 110000))
    return result
  assert run(param) == run(param | DIRECT_TX)


@pytest.mark.parametrize("address,bus,size", [(0x1A0, 0, 32), (0x12A, 0, 16), (0xCB, 0, 24),
                                            (0xEA, 2, 24), (0x1AA, 2, 16), (0x175, 2, 24)])
def test_direct_cluster_setting_preserves_control_fifo_and_reuse(native, address, bus, size):
  def run(param):
    hooks = Hooks(native, param)
    results = []
    # Push a burst, drain it, then exercise empty-FIFO reuse and exhaustion.
    for index in range(4):
      host = bytearray(size)
      host[2] = index
      if address == 0x1A0:  # zero acceleration in both offset-encoded fields
        host[16:19] = bytes([0xFF, 0xF3, 0x3F])
      elif address == 0x12A:
        host[6] = 8  # zero torque, no request
      result = hooks.tx(address, checksum(address, host), 100000 + index, bus=bus)
      assert result == 3
      results.append(result)
    for index in range(20):
      stock = message(address, counter=index, marker=64, length=size)
      results.append(hooks.rx(address, stock, 110000 + index * 10000, bus=2 - bus))
    return results
  assert run(157) == run(157 | DIRECT_TX)
