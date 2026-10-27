"""Pinned state staging validates contracts and clears invalid shared outputs."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'third_party/jetlink')]
from jetlink.queues import StatefulQueues
from test_resaction import SPEC, inputs, advanced

path = Path(__file__).resolve().parents[2] / 'third_party/jetlink/jetlink/server/backends/trt/state_buffers.py'
module_spec = importlib.util.spec_from_file_location('state_buffers_test', path)
module = importlib.util.module_from_spec(module_spec)
module_spec.loader.exec_module(module)


def test_staging_uses_outputs_without_modifying_device_bindings():
  dest = inputs()
  outputs = advanced(dest)
  ins = {n: NS(shape=a.shape, dtype=a.dtype, host=a) for n, a in dest.items()}
  outs = {n: NS(shape=a.shape, dtype=a.dtype, host=a) for n, a in outputs.items()}
  sources = module.feedback_sources(ins, outs, SPEC.state_pairs)
  for name, output in SPEC.state_pairs.items():
    assert sources[name] is outs[output]
    assert ins[name].host is dest[name]


@pytest.mark.parametrize('mismatch', ['shape', 'dtype'])
def test_incompatible_state_contract_rejected(mismatch):
  target = NS(shape=(2,), dtype=np.dtype('float32'))
  source = NS(shape=(3,) if mismatch == 'shape' else (2,),
              dtype=np.dtype('float16' if mismatch == 'dtype' else 'float32'))
  with pytest.raises(ValueError, match='contract mismatch'):
    module.feedback_sources({'state': target}, {'next': source}, {'state': 'next'})


@pytest.mark.parametrize('bad_output', ['outputs', *SPEC.state_pairs.values()])
def test_invalid_shared_feedback_clears_every_state_before_next_frame(bad_output):
  dest = inputs()
  outputs = advanced(dest, 2)
  for name, output in SPEC.state_pairs.items():
    dest[name] = outputs[output]
  queues = StatefulQueues(SPEC, dest)
  for a in outputs.values():
    a.fill(3)
  outputs[bad_output].flat[0] = np.nan
  with pytest.raises(FloatingPointError):
    queues.feedback(outputs)
  assert all(not np.any(a) for a in queues.state.values())
  queues.step_into(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), dest)
  assert all(not np.any(dest[n]) for n in SPEC.state_pairs)
