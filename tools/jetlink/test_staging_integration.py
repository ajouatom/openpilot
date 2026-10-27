"""Combined staging acceptance: real converter, session, recurrent ownership."""
from dataclasses import replace
from hashlib import sha256
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_nav_rl import SPEC
from test_output_projection import pair
from jetlink import input_cast
from jetlink.queues import make_queues
from jetlink.packed_codec import _layout
from jetlink.server.session import EngineHost


def native():
  converter = input_cast.native_u8_to_f16()
  if converter is None:
    pytest.skip('Requires a real Linux native converter; covered by Linux CI')
  return converter


@pytest.mark.parametrize('policy', ['default', 'off', 'other_model', 'other_contract', 'no_compiler'])
def test_warm_policy_is_pinned_and_has_fallback(monkeypatch, policy):
  monkeypatch.delenv('CARROT_TRT_DIRECT_MAP', raising=False)
  monkeypatch.delenv('CARROT_TRT_NATIVE_IMAGE', raising=False)
  spec = SPEC
  if policy == 'off':
    monkeypatch.setenv('CARROT_TRT_DIRECT_MAP', '0')
    monkeypatch.setenv('CARROT_TRT_NATIVE_IMAGE', '0')
  if policy == 'other_model': spec = replace(SPEC, sha256='0' * 64)
  if policy == 'other_contract': spec = replace(SPEC, output_slices={**SPEC.output_slices, 'plan': slice(1, 496)})
  # Only policy/fallback uses a stub; combined tests below require actual C.
  calls = []
  def load():
    calls.append(True)
    return None
  monkeypatch.setattr(input_cast, 'native_u8_to_f16', load)
  dest = {n: np.zeros(s, np.float16 if n == 'new_img' else np.float32) for n, s in spec.input_shapes.items()}
  engine = NS(inputs={n: NS(shape=s) for n, s in spec.input_shapes.items()},
              outputs={n: NS(shape=s) for n, s in spec.output_shapes.items()},
              host_input=dest.__getitem__, warm=lambda: 'warm')
  loaded = EngineHost._warm(None, engine, spec)
  enabled = policy in ('default', 'no_compiler')
  assert (loaded.queues.direct_map is not None) == enabled
  assert bool(calls) == enabled
  assert loaded.queues.input_cast is None
  image = np.arange(spec.warped_nbytes, dtype=np.uint8).reshape(spec.warped_shape)
  loaded.queues.step_into(image, np.zeros(spec.packed_nelem, np.float32), dest)
  assert dest['new_img'].tobytes() == image.astype(np.float16).tobytes()


@pytest.mark.parametrize('dtype', [np.float16, np.float32])
@pytest.mark.parametrize('device_state', [False, True])
def test_combined_session_inputs_recurrence_and_failure_recovery(dtype, device_state):
  converter = native()
  baseline, candidate = pair(), pair()
  states = []
  snapshots = [[], []]
  for index, (_, _, loaded, outputs) in enumerate((baseline, candidate)):
    inputs = {n: np.zeros(s, np.float16 if n == 'new_img' else dtype) for n, s in SPEC.input_shapes.items()}
    loaded.host_inputs = inputs
    class State:
      input_names = set(SPEC.state_pairs)
      output_names = set(SPEC.state_pairs.values())
      resets = 0
      validations = 0
      def reset(self): self.resets += 1
      def validate(self): self.validations += 1
    state = State() if device_state else None
    states.append(state)
    loaded.queues = make_queues(SPEC, inputs, state, direct_map=bool(index), native_image=bool(index))
    if index: assert loaded.queues.input_cast is converter
    def run(inputs=inputs, index=index):
      snapshots[index].append({n: sha256(memoryview(v).cast('B')).digest() for n, v in inputs.items()})
      # Input-dependent CPU reference; does not stand in for a TensorRT trial.
      result = {'outputs': np.full(SPEC.output_shapes['outputs'], float(inputs['nav_map'].sum(dtype=np.float64) % 7), np.float32)}
      if not device_state:
        result.update({out: np.full(SPEC.output_shapes[out], float(inputs[name].flat[0]) + .125, inputs[name].dtype)
                       for name, out in SPEC.state_pairs.items()})
      return result
    loaded.engine.run = run
  rng = np.random.default_rng(20261023)
  offset = _layout(SPEC) // 4
  for frame in range(32):
    image = rng.integers(0, 256, SPEC.warped_shape, np.uint8)
    packed = rng.uniform(-1, 1, SPEC.packed_nelem).astype(np.float32)
    packed[offset:offset + 65536] = 0 if frame % 3 == 0 else rng.integers(0, 256, 65536)
    if frame % 5 == 1: packed[offset] = .5  # raw, lossless fractional-map fallback
    if frame % 5 == 2: packed[offset] = -0.  # preserve sign of zero
    outputs = []
    for client, _, _, _ in (baseline, candidate):
      client.project_output = frame % 2 == 0
      outputs.append(client.infer(image, packed, frame, reset=frame in (0, 16), want_state=True))
    assert snapshots[0][-1] == snapshots[1][-1]
    assert outputs[0].tobytes() == outputs[1].tobytes()
  for client, _, loaded, _ in (baseline, candidate):
    bad = np.zeros(SPEC.packed_nelem, np.float32)
    bad[0] = np.nan
    with pytest.raises(ValueError): client.infer(image, bad, 33)
    assert all(not np.any(v) for v in loaded.queues.state.values())
    client.infer(image, np.zeros_like(bad), 34)
  assert snapshots[0] == snapshots[1]
  if device_state:
    assert states[0].resets == states[1].resets
    assert states[0].validations == states[1].validations == 33
