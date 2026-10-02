"""Differential checks: require compiled kernels, never silently test fallback."""
from collections import deque
import importlib
import math
import random
import struct
import sys
from types import SimpleNamespace

import pytest

# The CAN module imports Hyundai helpers that refer to the device Params module.
# These tests exercise no Params operations; desktops need only the import name.
try:
  import openpilot.common.params  # noqa: F401
except ModuleNotFoundError as exc:
  if exc.name != 'openpilot.common.params_pyx':
    raise
  sys.modules['openpilot.common.params'] = SimpleNamespace(Params=type('UnusedParams', (), {}))

from opendbc.can import native as can_backend
from opendbc.can._can_native import raw_values, set_value as native_set_value
from opendbc.can.dbc import DBC, Signal
from opendbc.can.packer import CANPacker, set_value
from opendbc.can.parser import MessageState, get_raw_value
from openpilot.selfdrive.carrot.radar_motion import native as radar_backend
from openpilot.selfdrive.carrot.radar_motion._motion_native import motion, nearest_segment
from openpilot.selfdrive.carrot.radar_motion.predictor import _nearest_segment_python
from openpilot.selfdrive.carrot.radar_motion.trajectory_cutin import _median_slope_python, _motion_metrics_python


def exact(actual, expected):
  if isinstance(expected, (tuple, list)):
    assert len(actual) == len(expected)
    for a, b in zip(actual, expected, strict=True):
      exact(a, b)
  elif isinstance(expected, float):
    assert struct.pack('!d', actual) == struct.pack('!d', expected)
  else:
    assert actual == expected


def test_native_loaded():
  assert radar_backend.BACKEND == can_backend.BACKEND == 'cython'


def test_explicit_python_comparison_mode(monkeypatch):
  try:
    with monkeypatch.context() as patch:
      patch.setenv('CARROT_NATIVE_CPU', '0')
      importlib.reload(radar_backend)
      importlib.reload(can_backend)
      assert radar_backend.BACKEND == can_backend.BACKEND == 'python'
      assert radar_backend.motion is radar_backend.nearest_segment is None
      assert can_backend.pack is can_backend.raw_values is None
  finally:
    importlib.reload(radar_backend)
    importlib.reload(can_backend)


@pytest.mark.parametrize('seed', range(20))
def test_motion_exact(seed):
  rng = random.Random(seed)
  for count in range(33):
    time = 0.0
    observations = deque()
    for _ in range(count):
      time += rng.choice((0.0, 0.01, 0.05, 0.1, 0.2))
      observations.append(SimpleNamespace(time_s=time, d_path=rng.choice((0.0, -0.0, rng.uniform(-8, 8)))))
    for window in (0.0, 0.1, 0.35, 0.9, 1.5):
      exact(motion(observations, window, 'd_path', False), _median_slope_python(observations, 'd_path', window))
      exact(motion(observations, window, 'd_path', True), _motion_metrics_python(observations, window))


@pytest.mark.parametrize('bad', (math.nan, math.inf, -math.inf))
def test_nonfinite_motion_uses_reference(bad):
  observations = [SimpleNamespace(time_s=0.0, d_path=1.0), SimpleNamespace(time_s=0.2, d_path=bad)]
  assert motion(observations, 0.9, 'd_path', True) is None


@pytest.mark.parametrize('seed', range(20))
def test_projection_exact(seed):
  rng = random.Random(seed)
  segments = []
  x = y = distance = 0.0
  for _ in range(32):
    dx, dy = rng.uniform(0.1, 5), rng.uniform(-3, 3)
    length = math.hypot(dx, dy)
    segments.append((x, y, dx / length, dy / length, length, distance))
    x += dx
    y += dy
    distance += length
  for x, y in [(0., -0.), (0., 0.)] + [(rng.uniform(-10, 100), rng.uniform(-10, 10)) for _ in range(100)]:
    exact(nearest_segment(segments, x, y), _nearest_segment_python(segments, x, y))


@pytest.mark.parametrize('little', (False, True))
@pytest.mark.parametrize('bits', (1, 7, 8, 9, 16, 32, 63, 64, 65, 128))
def test_can_bits(little, bits):
  rng = random.Random(bits)
  for shift in range(8):
    lsb = shift if little else 8 * ((bits + shift - 1) // 8) + shift
    msb = lsb + bits - 1 if little else lsb - ((bits - 1 + 7 - shift) // 8) * 8 + (bits - 1) % 8
    # Use DBC's actual Motorola conversion instead of inventing a second layout.
    if not little:
      start = 7 - shift
      msb = start
      end = (start // 8) * 8 + (7 - start % 8) + bits - 1
      lsb = (end // 8) * 8 + 7 - end % 8
    sig = Signal('x', 0, msb, lsb, bits, False, 1., 0., little)
    for length in (0, 1, 8, 16, 64):
      for value in (0, -1, 1 << bits, -(1 << (bits + 1)), rng.getrandbits(bits)):
        a = bytearray(rng.randbytes(length))
        b = a.copy()
        set_value(a, sig, value)
        native_set_value(b, sig, value)
        assert a == b
        assert raw_values(bytes(a), [sig]) == [get_raw_value(a, sig)]
        sig.is_signed = True
        raw = get_raw_value(a, sig)
        assert raw_values(a, [sig]) == [raw - ((raw >> (bits - 1)) & 1) * (1 << bits)]
        sig.is_signed = False


@pytest.mark.parametrize('name', ['hyundai_canfd_generated', 'hyundai_kia_generic', 'toyota_nodsu_pt_generated',
                                  'honda_civic_touring_2016_can_generated', 'vw_mqb_2010', 'subaru_global_2017_generated'])
def test_packing_and_decode_dbc(name):
  rng = random.Random(82)
  py = CANPacker(name)
  accelerated = CANPacker(name)
  for msg in py.dbc.addr_to_msg.values():
    for iteration in range(5):
      values = {}
      for sig in msg.sigs.values():
        if sig.factor and iteration != 0:
          raw = rng.randrange(1 << min(sig.size, 20))
          values[sig.name] = raw * sig.factor + sig.offset
      # Include automatic, explicit and RX-initialized counter paths.
      a = py.pack_python(msg.address, values, iteration)
      b = accelerated.pack(msg.address, values, iteration)
      assert a == b
      assert py.counters == accelerated.counters
      expected = []
      for sig in msg.sigs.values():
        value = get_raw_value(a, sig)
        if sig.is_signed:
          value -= ((value >> (sig.size - 1)) & 1) * (1 << sig.size)
        expected.append(value)
      assert raw_values(a, list(msg.sigs.values())) == expected


def test_counter_checksum_rejection_state(monkeypatch):
  dbc = DBC('hyundai_canfd_generated')
  msg = next(m for m in dbc.addr_to_msg.values() if any(s.calc_checksum for s in m.sigs.values())
             and any(s.type == 1 for s in m.sigs.values()))
  states = [MessageState(msg.address, msg.name, msg.size, list(msg.sigs.values())) for _ in range(2)]
  packer = CANPacker('hyundai_canfd_generated')
  for n in range(100):
    data = packer.pack(msg.address, {})
    if n % 7 == 0:
      data[-1] ^= 0x80  # Reject checksum but preserve reference counter processing.
    if n % 11 == 0:
      data = data[:3]  # Truncated messages must behave identically too.
    with monkeypatch.context() as patch:
      patch.setattr(can_backend, 'raw_values', None)
      expected = states[0].parse(1_000_000_000 + n * 10_000_000, bytes(data))
    actual = states[1].parse(1_000_000_000 + n * 10_000_000, bytes(data))
    assert actual == expected
    assert vars(states[0]) == vars(states[1])
