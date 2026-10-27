"""Map-only staging equivalence, ownership, finite checks and wire compatibility."""
import numpy as np
import pytest

from test_nav_rl import SPEC
from test_output_projection import pair, infer
from jetlink.direct_map import DirectMap
from jetlink.packed_codec import encode, decode, _layout
from jetlink.queues import StatefulQueues
from jetlink.transport.base import LinkError


def buffers(dtype):
  return {name: np.zeros(shape, dtype) for name, shape in SPEC.input_shapes.items()}


@pytest.mark.parametrize('dtype', [np.float16, np.float32])
def test_every_input_bit_and_buffer_identity_with_expiry(dtype):
  a, b = buffers(dtype), buffers(dtype)
  old, new = StatefulQueues(SPEC, a), StatefulQueues(SPEC, b)
  new.direct_map = DirectMap(SPEC, b)
  pointers = {k: v.ctypes.data for k, v in b.items()}
  rng = np.random.default_rng(129)
  for frame in range(12):
    image = rng.integers(0, 256, SPEC.warped_shape, np.uint8)
    context = rng.uniform(-1, 1, SPEC.packed_nelem).astype(np.float32)
    offset = _layout(SPEC) // 4
    context[offset:offset + 65536] = rng.integers(0, 256, 65536) if frame % 3 else 0
    wire, compact = encode(context, SPEC)
    assert compact
    old.step_into(image, np.frombuffer(decode(wire, SPEC), np.float32), a)
    new.step_compact_into(image, wire, b)
    assert all(a[k].tobytes() == b[k].tobytes() for k in a)
    assert pointers == {k: v.ctypes.data for k, v in b.items()}
    # No view of the transport's reusable receive buffer may be retained.
    wire = bytearray(wire)
    new.step_compact_into(image, wire, b)
    wire[:] = bytes(len(wire))
    assert all(a[k].tobytes() == b[k].tobytes() for k in a)


@pytest.mark.parametrize('fault', ['short', 'long', 'nan', 'inf', 'replaced', 'readonly'])
def test_invalid_context_clears_history_before_destination_write(fault):
  dest = buffers(np.float16)
  queue = StatefulQueues(SPEC, dest)
  queue.direct_map = DirectMap(SPEC, dest)
  for value in dest.values(): value.fill(1)
  wire, _ = encode(np.zeros(SPEC.packed_nelem, np.float32), SPEC)
  wire = bytearray(wire)
  if fault == 'short': wire = wire[:-1]
  if fault == 'long': wire += b'x'
  if fault in ('nan', 'inf'): wire[:4] = np.array([np.nan if fault == 'nan' else np.inf], np.float32).tobytes()
  if fault == 'replaced': dest['nav_map'] = dest['nav_map'].copy()
  if fault == 'readonly': dest['nav_map'].flags.writeable = False
  with pytest.raises(ValueError):
    queue.step_compact_into(np.zeros(SPEC.warped_shape, np.uint8), wire, dest)
  assert all(not np.any(v) for v in queue.state.values())
  assert np.all(dest['new_img'] == 1) and np.all(dest['nav_map'] == 1)


@pytest.mark.parametrize('value', [.5, -0., 256., -1., np.nan])
def test_noncanonical_map_retains_raw_float_payload(value):
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  packed[_layout(SPEC) // 4] = value
  wire, compact = encode(packed, SPEC)
  assert not compact and bytes(wire) == packed.tobytes()


@pytest.mark.parametrize('projection', [False, True])
def test_real_session_compact_and_raw_paths_with_projected_reply(projection):
  client, _, loaded, _ = pair()
  loaded.queues.direct_map = DirectMap(SPEC, loaded.host_inputs)
  client.project_output = projection
  for compact in (True, False, True):
    client._nav_u8 = compact
    assert np.isfinite(infer(client, reset=True)).all()
  # NaNs in scalar fields still fail before engine execution.
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  packed[0] = np.nan
  with pytest.raises((ValueError, LinkError)):
    client.infer(np.zeros(SPEC.warped_shape, np.uint8), packed)


def test_opt_in_preserves_device_state_reset_and_validation():
  from jetlink.queues import make_queues, DeviceStateQueues

  class State:
    input_names = set(SPEC.state_pairs)
    output_names = set(SPEC.state_pairs.values())
    resets = 0
    validations = 0

    def reset(self): self.resets += 1
    def validate(self): self.validations += 1

  dest = buffers(np.float32)
  assert make_queues(SPEC, dest).direct_map is None
  state = State()
  queue = make_queues(SPEC, dest, state, direct_map=True)
  assert isinstance(queue, DeviceStateQueues) and state.resets == 1
  wire, _ = encode(np.zeros(SPEC.packed_nelem, np.float32), SPEC)
  queue.step_compact_into(np.zeros(SPEC.warped_shape, np.uint8), wire, dest)
  queue.feedback({'outputs': np.zeros(SPEC.output_shapes['outputs'], np.float32)})
  assert state.validations == 1 and state.resets == 1
  invalid = bytearray(wire)
  invalid[:4] = np.array([np.inf], np.float32).tobytes()
  with pytest.raises(ValueError):
    queue.step_compact_into(np.zeros(SPEC.warped_shape, np.uint8), invalid, dest)
  assert state.resets == 2
