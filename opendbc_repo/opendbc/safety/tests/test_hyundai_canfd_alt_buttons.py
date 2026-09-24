import ctypes
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest

from opendbc.car.hyundai.hyundaicanfd import hkg_can_fd_checksum


def checksum(data):
  data = bytearray(data)
  data[:2] = hkg_can_fd_checksum(0x10B, None, data).to_bytes(2, "little")
  return bytes(data)


def command(button=0):
  data = bytearray(16)
  data[10] = button
  return checksum(data)


def stock(button=0, counter=173):
  # Exercise unknown fields and reserved bits, not just an all-zero fixture.
  data = bytearray(range(16))
  data[2] = counter
  data[10] = 0x70 | button
  return checksum(data)


@pytest.fixture(scope="session")
def native(tmp_path_factory):
  root = next(p for p in Path(__file__).resolve().parents if (p / "panda/board").is_dir())
  output = tmp_path_factory.mktemp("alt2-native") / ("safety.dll" if sys.platform == "win32" else "safety.so")
  compiler = shlex.split(os.environ.get("CC", "cc"))
  subprocess.run([*compiler, "-shared", "-fPIC", "-O1", f"-I{root / 'panda/board'}",
                  f"-I{root / 'opendbc_repo/opendbc/safety'}", f"-I{root / 'panda'}",
                  str(Path(__file__).with_name("hyundai_canfd_alt_buttons.c")), "-o", str(output)], check=True)
  lib = ctypes.CDLL(str(output))
  lib.alt2_test_init.argtypes = [ctypes.c_int]
  signature = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32]
  lib.alt2_test_tx.argtypes = lib.alt2_test_fwd.argtypes = signature
  lib.alt2_test_tx.restype = lib.alt2_test_fwd.restype = ctypes.c_int
  return lib


class Hooks:
  def __init__(self, lib, param=189):
    self.lib = lib
    lib.alt2_test_init(param)

  def tx(self, data, now=1000, bus=2, address=0x10B):
    buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
    return self.lib.alt2_test_tx(address, bus, len(data), buf, now)

  def fwd(self, data, now=2000, bus=0, address=0x10B):
    buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
    result = self.lib.alt2_test_fwd(address, bus, len(data), buf, now)
    return result, bytes(buf)


@pytest.fixture
def hooks(native):
  return Hooks(native)


@pytest.mark.parametrize("button", (0x80, 2, 8))
@pytest.mark.parametrize("counter", (0, 1, 173, 255))
def test_request_is_consumed_and_only_buttons_and_checksum_change(hooks, button, counter):
  assert hooks.tx(command(button)) == 3  # allowed AND buffered, no direct transmission
  raw = stock(counter=counter)
  destination, result = hooks.fwd(raw)
  assert destination == 2
  assert result[2:10] == raw[2:10] and result[11:] == raw[11:]
  assert result[10] == 0x70 | button
  assert result == checksum(result)


@pytest.mark.parametrize("button", (1, 2, 4, 8, 15, 0x80, 0x84))
def test_physical_input_wins_and_requires_release_before_new_request(hooks, button):
  assert hooks.tx(command(0x80)) == 3
  raw = stock(button)
  assert hooks.fwd(raw) == (2, raw)
  assert hooks.tx(command(0x80), 3000) == 3
  assert hooks.fwd(stock(), 4000) == (2, stock())
  assert hooks.tx(command(), 5000) == 3
  assert hooks.tx(command(0x80), 6000) == 3
  assert hooks.fwd(stock(), 7000)[1][10] == 0xF0


def test_host_release_and_missing_host_release(hooks):
  assert hooks.tx(command(0x80)) == 3
  assert hooks.fwd(stock(), 120999)[1][10] == 0xF0
  assert hooks.fwd(stock(), 121000) == (2, stock())
  assert hooks.tx(command(0x80), 122000) == 3  # must not rearm without neutral
  assert hooks.fwd(stock(), 123000) == (2, stock())
  assert hooks.tx(command(), 124000) == 3
  assert hooks.tx(command(0x80), 125000) == 3
  assert hooks.fwd(stock(), 126000)[1][10] == 0xF0
  assert hooks.tx(command(), 127000) == 3
  assert hooks.fwd(stock(), 128000) == (2, stock())


def test_stale_request_does_not_rearm_if_no_stock_frame_arrived(hooks):
  assert hooks.tx(command(0x80)) == 3
  assert hooks.tx(command(0x80), 121000) == 3
  assert hooks.fwd(stock(), 122000) == (2, stock())


def test_continuously_refreshed_request_has_bounded_press(hooks):
  for now in range(1000, 222000, 20000):
    assert hooks.tx(command(8), now) == 3
    result = hooks.fwd(stock(), now + 1)[1]
    assert result[10] == (0x78 if now < 201000 else 0x70)


def test_change_of_button_requires_neutral(hooks):
  assert hooks.tx(command(0x80)) == 3
  assert hooks.tx(command(8), 2000) == 3
  assert hooks.fwd(stock(), 3000) == (2, stock())


def test_reset_clears_request(native):
  hooks = Hooks(native)
  assert hooks.tx(command(0x80)) == 3
  native.alt2_test_init(189)
  assert hooks.fwd(stock()) == (2, stock())


def test_invalid_stock_checksum_is_not_repaired(hooks):
  assert hooks.tx(command(0x80)) == 3
  bad = bytearray(stock())
  bad[0] ^= 1
  assert hooks.fwd(bad) == (2, bytes(bad))


@pytest.mark.parametrize("button", (1, 3, 4, 9, 0x82, 0x88, 0xFF))
def test_unsupported_host_button_is_rejected(hooks, button):
  assert hooks.tx(command(button)) == 0
  assert hooks.fwd(stock()) == (2, stock())


@pytest.mark.parametrize("param", (0, 1, 8, 9, 16, 17, 181))
def test_other_safety_modes_cannot_inject(native, param):
  hooks = Hooks(native, param)
  assert hooks.tx(command(0x80)) == 0
  assert hooks.fwd(stock())[1] == stock()


@pytest.mark.parametrize("bus", (0, 1, 3))
def test_wrong_tx_bus_is_rejected(hooks, bus):
  assert hooks.tx(command(0x80), bus=bus) == 0
  assert hooks.fwd(stock()) == (2, stock())


def test_bad_host_checksum_and_length_are_rejected(hooks):
  bad = bytearray(command(0x80))
  bad[0] ^= 1
  assert hooks.tx(bad) == 0
  assert hooks.tx(bytes(8)) == 0
  assert hooks.fwd(stock()) == (2, stock())


@pytest.mark.parametrize("bus", (1, 2))
def test_other_input_buses_are_never_modified(hooks, bus):
  assert hooks.tx(command(0x80)) == 3
  assert hooks.fwd(stock(), bus=bus)[1] == stock()


def test_timer_wraparound(hooks):
  assert hooks.tx(command(0x80), 0xFFFFFF00) == 3
  assert hooks.fwd(stock(), 0x100)[1][10] == 0xF0
  assert hooks.fwd(stock(), 0xFFFFFF00 + 120000 & 0xFFFFFFFF)[1] == stock()
