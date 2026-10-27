"""Exact native image conversion; map/protocol/state behavior is preserved."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_nav_rl import SPEC
from test_output_projection import pair, infer
from jetlink import input_cast
from jetlink.queues import make_queues
from jetlink.packed_codec import encode, decode, _layout
from jetlink.server.session import EngineHost


@pytest.fixture
def converter():
  value = input_cast.native_u8_to_f16()
  if value is None:
    pytest.skip('Native image converter needs a Linux C compiler')
  return value


@pytest.mark.parametrize('length', [0, 1, 7, 8, 15, 16, 17, 31, 32, 33, 255, 256, 257, 393216])
def test_all_bytes_and_guard_regions(converter, length):
  backing = np.arange(length + 16, dtype=np.uint8)
  source = backing[8:8 + length]
  original = backing.tobytes()
  target = np.full(length + 16, 0xdead, np.uint16)
  converter(source, target[8:8 + length].view(np.float16))
  assert target[8:8 + length].tobytes() == source.astype(np.float16).tobytes()
  assert np.all(target[:8] == 0xdead) and np.all(target[8 + length:] == 0xdead)
  assert backing.tobytes() == original


@pytest.mark.parametrize('fault', ['source_dtype', 'dest_dtype', 'length', 'strided_source',
                                 'strided_dest', 'unaligned', 'readonly', 'alias', 'byteorder'])
def test_invalid_buffer_rejected_before_native_write(converter, fault):
  source = np.arange(32, dtype=np.uint8)
  dest = np.ones(32, np.float16)
  if fault == 'source_dtype': source = source.astype(np.float16)
  if fault == 'dest_dtype': dest = dest.astype(np.float32)
  if fault == 'length': dest = dest[:-1]
  if fault == 'strided_source': source = np.arange(64, dtype=np.uint8)[::2]
  if fault == 'strided_dest': dest = np.ones(64, np.float16)[::2]
  if fault == 'unaligned': dest = np.zeros(65, np.uint8)[1:].view(np.float16)
  if fault == 'readonly': dest.flags.writeable = False
  if fault == 'alias': source = dest.view(np.uint8)[:32]
  if fault == 'byteorder': dest = dest.astype('>f2')
  before = dest.tobytes()
  with pytest.raises(ValueError): converter(source, dest)
  assert dest.tobytes() == before


@pytest.mark.parametrize('dtype', [np.float16, np.float32])
def test_image_only_preserves_every_input_and_raw_map_fallback(converter, dtype):
  a = {n: np.zeros(s, dtype) for n, s in SPEC.input_shapes.items()}
  b = {n: v.copy() for n, v in a.items()}
  old, new = make_queues(SPEC, a), make_queues(SPEC, b, native_image=True)
  assert old.input_cast is None and new.input_cast is converter
  assert new.direct_map is None
  rng = np.random.default_rng(20261023)
  for frame in range(12):
    image = rng.integers(0, 256, SPEC.warped_shape, np.uint8)
    saved = image.tobytes()
    packed = rng.uniform(-1, 1, SPEC.packed_nelem).astype(np.float32)
    offset = _layout(SPEC) // 4
    packed[offset:offset + 65536] = 0 if frame % 3 == 0 else rng.integers(0, 256, 65536)
    if frame % 4 == 0: packed[offset] = .5
    wire, compact = encode(packed, SPEC)
    decoded = np.frombuffer(decode(wire, SPEC) if compact else wire, np.float32)
    old.step_into(image, decoded, a)
    new.step_into(image, decoded, b)
    assert all(a[n].tobytes() == b[n].tobytes() for n in a)
    assert image.tobytes() == saved
  for v in b.values(): v.fill(1)
  packed[0] = np.nan
  with pytest.raises(ValueError): new.step_into(image, packed, b)
  assert all(not np.any(b[n]) for n in SPEC.state_pairs)
  assert np.all(b['new_img'] == 1)


@pytest.mark.parametrize('projection', [False, True])
def test_session_compatibility(converter, projection):
  client, _, loaded, _ = pair()
  loaded.host_inputs['new_img'] = np.zeros(SPEC.input_shapes['new_img'], np.float16)
  calls = []
  def checked(source, dest):
    converter(source, dest)
    calls.append(source.size)
  loaded.queues.input_cast = checked
  assert loaded.queues.direct_map is None
  client.project_output = projection
  for compact in (False, True):
    client._nav_u8 = compact
    assert np.isfinite(infer(client, reset=True)).all()
  assert calls == [SPEC.warped_nbytes] * 2


def test_load_failure_uses_original_converter(monkeypatch):
  input_cast.native_u8_to_f16.cache_clear()
  try:
    monkeypatch.setattr(input_cast.shutil, 'which', lambda _: None)
    dest = {n: np.zeros(s, np.float16) for n, s in SPEC.input_shapes.items()}
    queue = make_queues(SPEC, dest, native_image=True)
    assert queue.input_cast is None
    image = np.arange(SPEC.warped_nbytes, dtype=np.uint8).reshape(SPEC.warped_shape)
    queue.step_into(image, np.zeros(SPEC.packed_nelem, np.float32), dest)
    assert dest['new_img'].tobytes() == image.astype(np.float16).tobytes()
  finally:
    input_cast.native_u8_to_f16.cache_clear()


@pytest.mark.parametrize('enabled', [False, True])
def test_engine_warm_loads_conversion_policy(monkeypatch, converter, enabled):
  # Exercise actual model initialization, not only a manually constructed queue.
  monkeypatch.setenv('CARROT_TRT_DIRECT_MAP', '0')
  monkeypatch.setenv('CARROT_TRT_NATIVE_IMAGE', '1' if enabled else '0')
  dest = {n: np.zeros(s, np.float16 if n == 'new_img' else np.float32)
          for n, s in SPEC.input_shapes.items()}
  engine = NS(inputs={n: NS(shape=s) for n, s in SPEC.input_shapes.items()},
              outputs={n: NS(shape=s) for n, s in SPEC.output_shapes.items()},
              host_input=dest.__getitem__, warm=lambda: 'warm')
  loaded = EngineHost._warm(None, engine, SPEC)
  assert loaded.queues.input_cast is (converter if enabled else None)
  assert loaded.queues.direct_map is None
