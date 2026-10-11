"""Native CPU checks against the exact MDM runtime; no GPU/vehicle claim."""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(not os.getenv('MDM_RUNTIME_ROOT'), reason='requires the verified pinned MDM runtime')


def test_pinned_warp_capture_reload_and_persistent_arena(tmp_path, monkeypatch):
  monkeypatch.syspath_prepend(os.environ['MDM_RUNTIME_ROOT'])
  monkeypatch.setenv('DEV', 'CPU')
  monkeypatch.setenv('DEBUG', '0')
  assert 'tinygrad' not in sys.modules, 'run the isolated runtime test in its own process'
  from tinygrad import Tensor
  from examples.openpilot.compile_warp import NV12Frame
  from examples.openpilot.helpers import dump_pickle, benchmark
  from openpilot.selfdrive.modeld.precompiled_artifact import compile_warp, load_artifact, prepare_jit, load_warp
  import pickle

  artifact = compile_warp(NV12Frame(32, 16, 32, 16, 8, 768), (16, 8), layout='yuv420', frames=2, benchmark_runs=1)
  path = tmp_path / 'warp.pkl'
  with path.open('wb') as stream:
    pickle.dump(artifact, stream)
  reloaded = load_warp(path)
  frame = Tensor(np.arange(1536, dtype=np.uint8).reshape(2, 768)).realize()
  transforms = Tensor(np.tile(np.eye(3, dtype=np.float32), (2, 1, 1))).realize()
  inputs = {'input_frame': frame, 'M_inv': transforms}
  expected = benchmark(artifact['run'], **inputs)
  np.testing.assert_array_equal(benchmark(reloaded, **inputs), expected)

  # Real upstream persistent buffer writer and reader, including loaded JIT.
  path = tmp_path / 'persistent.pkl'
  dump_pickle(artifact, path)
  loaded = load_artifact(path, 'persistent-buffer-v1')
  run = prepare_jit(loaded['run'])
  np.testing.assert_array_equal(benchmark(run, **inputs), expected)
  assert Path(os.environ['MDM_RUNTIME_ROOT']) in Path(sys.modules['tinygrad'].__file__).parents
