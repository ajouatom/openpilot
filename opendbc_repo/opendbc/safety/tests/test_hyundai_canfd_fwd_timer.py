import ctypes
import os
from pathlib import Path
import shlex
import subprocess
import sys

import pytest

from opendbc.safety.tests.test_hyundai_canfd_cluster import Hooks, load_native


# IDs that can pass stock frames without any host replacement. Cover both buses
# and every suppression duration implicated by the timer rollover.
MESSAGES = [(0x160, 0, 16, 40000), (0x1AA, 2, 16, 40000),
            (0x1FA, 2, 32, 120000), (0x4B4, 2, 8, 120000),
            (0x345, 0, 8, 220000), (0x4A3, 2, 8, 220000),
            (0x1DA, 0, 32, 1020000)]
DIRECT_MESSAGES = [m for m in MESSAGES if m[0] != 0x1AA]  # 0x1AA uses the control FIFO in CAMERA_SCC
MASK = 2**32 - 1


@pytest.fixture(scope="session")
def native(tmp_path_factory):
  root = next(p for p in Path(__file__).resolve().parents if (p / "panda/board").is_dir())
  output = tmp_path_factory.mktemp("fwd-timer-native") / ("safety.dll" if sys.platform == "win32" else "safety.so")
  compiler = shlex.split(os.environ.get("CC", "cc"))
  subprocess.run([*compiler, "-shared", "-fPIC", "-O2", f"-I{root / 'panda/board'}",
                  f"-I{root / 'opendbc_repo/opendbc/safety'}", f"-I{root / 'panda'}",
                  str(Path(__file__).with_name("hyundai_canfd_fwd_timer.c")), "-o", str(output)], check=True)
  lib = load_native(output)
  lib.fwd_timer_set_tick.argtypes = [ctypes.c_uint32]
  lib.fwd_timer_set_tick.restype = None
  return lib


@pytest.fixture(params=[29, 29 | 2048, 1 | 4 | 16])
def hooks(native, request):
  return Hooks(native, request.param)


@pytest.mark.parametrize("address,bus,size,timeout", MESSAGES)
def test_never_transmitted_stock_survives_startup_and_rollovers(hooks, address, bus, size, timeout):
  stock = bytes(size)
  for tick in (0, 4312, 8624):
    hooks.lib.fwd_timer_set_tick(tick)
    for now in (MASK - 1, 0, 1, timeout - 1, timeout):
      assert hooks.rx(address, stock, now, bus=2 - bus) == (bus, stock)


@pytest.mark.parametrize("address,bus,size,timeout", DIRECT_MESSAGES)
@pytest.mark.parametrize("start", [0, 123456, MASK - 1000])
def test_real_tx_blocks_only_until_deadline_across_wrap(hooks, address, bus, size, timeout, start):
  stock = bytes(size)
  hooks.lib.fwd_timer_set_tick(100)
  assert hooks.tx(address, stock, start, bus=bus) == 1
  # A 1.02 s timeout can cross two nominal-second ticks. The coarse clock
  # must never shorten the existing microsecond deadline.
  hooks.lib.fwd_timer_set_tick(102 if timeout > 1000000 else 101)
  assert hooks.rx(address, stock, (start + timeout - 1) & MASK, bus=2 - bus)[0] == -1
  assert hooks.rx(address, stock, (start + timeout) & MASK, bus=2 - bus) == (bus, stock)
  # Once expired, the same microsecond timestamp cannot resurrect blocking.
  assert hooks.rx(address, stock, start, bus=2 - bus) == (bus, stock)


@pytest.mark.parametrize("address,bus,size,timeout", DIRECT_MESSAGES)
@pytest.mark.parametrize("tick", [100, MASK - 2])
def test_silent_id_cannot_revive_after_full_timer_wrap(hooks, address, bus, size, timeout, tick):
  stock = bytes(size)
  hooks.lib.fwd_timer_set_tick(tick)
  assert hooks.tx(address, stock, 123456, bus=bus) == 1
  # No intermediate RX, TX or expiry call. The fine clock has made a full lap;
  # the coarse clock also exercises its own unsigned rollover independently.
  hooks.lib.fwd_timer_set_tick((tick + 4312) & MASK)
  assert hooks.rx(address, stock, 123457, bus=2 - bus) == (bus, stock)
  assert hooks.tx(address, stock, 123458, bus=bus) == 1
  assert hooks.rx(address, stock, 123459, bus=2 - bus)[0] == -1


@pytest.mark.parametrize("address,bus,size,timeout", DIRECT_MESSAGES)
def test_rejected_tx_does_not_arm_block(hooks, address, bus, size, timeout):
  assert hooks.tx(address, bytes(12), 0, bus=bus) == 0  # wrong DLC
  assert hooks.rx(address, bytes(size), 1, bus=2 - bus) == (bus, bytes(size))
  assert hooks.tx(address, bytes(size), 2, bus=bus, relay=True) == 0
  assert hooks.rx(address, bytes(size), 3, bus=2 - bus) == (bus, bytes(size))


@pytest.mark.parametrize("address,bus,size,timeout", DIRECT_MESSAGES)
def test_mode_reset_clears_block_and_relay_protection_stays(hooks, address, bus, size, timeout):
  stock = bytes(size)
  assert hooks.tx(address, stock, 100, bus=bus) == 1
  hooks.lib.alt2_test_init(29)
  assert hooks.rx(address, stock, 101, bus=2 - bus) == (bus, stock)
  assert hooks.rx(address, stock, 102, bus=2 - bus, relay=True)[0] == -1


@pytest.mark.parametrize("address,bus,size,timeout", DIRECT_MESSAGES)
def test_fresh_tx_renews_only_its_own_deadline(hooks, address, bus, size, timeout):
  stock = bytes(size)
  assert hooks.tx(address, stock, 100, bus=bus) == 1
  assert hooks.tx(address, stock, timeout, bus=bus) == 1
  assert hooks.rx(address, stock, timeout + 100, bus=2 - bus)[0] == -1
  # The same address on the other source bus remains independent.
  assert hooks.rx(address, stock, timeout + 100, bus=bus) == (2 - bus, stock)
  assert hooks.rx(address, stock, 2 * timeout, bus=2 - bus) == (bus, stock)
