"""Native CPU checks against the exact MDM runtime; no GPU/vehicle claim."""
import os
from pathlib import Path
import sys

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(not os.getenv('MDM_RUNTIME_ROOT'), reason='requires the verified pinned MDM runtime')


def test_pinned_shared_input_views_preserve_offsets_and_state_aliases(monkeypatch):
  monkeypatch.syspath_prepend(os.environ['MDM_RUNTIME_ROOT'])
  monkeypatch.setenv('DEV', 'CPU')
  monkeypatch.setenv('DEBUG', '0')
  from tinygrad import Tensor, dtypes
  from openpilot.selfdrive.modeld.generic_model_runtime import input_view

  # Mixed packed input types exercise byte offsets and float element counts.
  packed = np.zeros(128, np.uint8)
  packed[32:56].view(np.float32)[:] = np.arange(6, dtype=np.float32)
  packed[64:76] = np.arange(12, dtype=np.uint8)
  host = Tensor(packed, device='NPY')._buffer()
  device = Tensor(np.zeros_like(packed), device='CPU')._buffer()
  device.copy_from(host)
  floats = input_view(device, (2, 3), dtypes.float32, 32)
  image = input_view(device, (2, 6), dtypes.uint8, 64)
  np.testing.assert_array_equal(floats.numpy(), np.arange(6).reshape(2, 3))
  np.testing.assert_array_equal(image.numpy(), np.arange(12).reshape(2, 6))
  # Reuse views after a new frame upload; they must never snapshot old data.
  packed[32:56].view(np.float32)[:] += 10
  device.copy_from(host)
  np.testing.assert_array_equal(floats.numpy(), (np.arange(6) + 10).reshape(2, 3))
  state = Tensor(np.arange(8, dtype=np.float32), device='CPU').realize()
  feedback = input_view(state._buffer(), state.shape, state.dtype)
  feedback._buffer().copy_from(Tensor(np.arange(8, dtype=np.float32) + 1, device='NPY')._buffer())
  np.testing.assert_array_equal(state.numpy(), np.arange(8) + 1)
  assert Path(os.environ['MDM_RUNTIME_ROOT']) in Path(sys.modules['tinygrad'].__file__).parents


def test_pinned_warp_capture_reload_and_persistent_arena(tmp_path, monkeypatch):
  monkeypatch.syspath_prepend(os.environ['MDM_RUNTIME_ROOT'])
  monkeypatch.setenv('DEV', 'CPU')
  monkeypatch.setenv('DEBUG', '0')
  from tinygrad import Tensor
  from examples.openpilot.compile_warp import NV12Frame
  from examples.openpilot.helpers import dump_pickle, benchmark, make_retargetable
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
  # Strip the export host's CPU dispatch binary just as upstream does for the
  # AMD model; loading must compile dispatch for the installation host again.
  make_retargetable(artifact['run'])
  path = tmp_path / 'persistent.pkl'
  dump_pickle(artifact, path)
  loaded = load_artifact(path, 'persistent-buffer-v1')
  run = prepare_jit(loaded['run'])
  np.testing.assert_array_equal(benchmark(run, **inputs), expected)
  assert Path(os.environ['MDM_RUNTIME_ROOT']) in Path(sys.modules['tinygrad'].__file__).parents


def test_pinned_state_checkpoint_restores_in_place_aliases(monkeypatch):
  monkeypatch.syspath_prepend(os.environ['MDM_RUNTIME_ROOT'])
  monkeypatch.setenv('DEV', 'CPU')
  from tinygrad import Tensor
  from openpilot.selfdrive.modeld.generic_model_runtime import GenericModelRuntime, input_view

  runtime = object.__new__(GenericModelRuntime)
  runtime.device = 'CPU'
  runtime.state_pairs = {'state_a': 'next_state_a', 'state_b': 'next_state_b'}
  runtime.specs = {'state_a': ((2, 3), 'float32', 'CPU'), 'state_b': ((4,), 'float16', 'CPU')}
  runtime.queues = {name: Tensor(np.arange(np.prod(shape), dtype=dtype).reshape(shape), device=device).realize()
                    for name, (shape, dtype, device) in runtime.specs.items()}
  aliases = {name: input_view(state._buffer(), state.shape, state.dtype) for name, state in runtime.queues.items()}
  originals = {name: state.numpy().copy() for name, state in runtime.queues.items()}
  runtime.init_recovery_state()
  runtime.save_recovery_state()
  for name, alias in aliases.items():
    alias._buffer().copy_from(Tensor(np.full_like(originals[name], np.nan), device='NPY')._buffer())
  runtime.restore_recovery_state()
  for name, alias in aliases.items():
    np.testing.assert_array_equal(alias.numpy(), originals[name])
    np.testing.assert_array_equal(runtime.queues[name].numpy(), originals[name])
  # Exercise cached replay with changing external writes, not just the first
  # capture's data. State/output aliases and backup allocations must survive.
  identities = {name: state._buffer() for name, state in runtime.queues.items()}
  saved = {name: state._buffer() for name, state in runtime.recovery_state.items()}
  for frame in range(1, 6):
    for name, alias in aliases.items():
      alias._buffer().copy_from(Tensor(originals[name] + frame, device='NPY')._buffer())
    runtime.save_recovery_state()
    for name, alias in aliases.items():
      alias._buffer().copy_from(Tensor(np.full_like(originals[name], np.nan), device='NPY')._buffer())
    runtime.restore_recovery_state()
    for name, alias in aliases.items():
      np.testing.assert_array_equal(alias.numpy(), originals[name] + frame)
      assert runtime.queues[name]._buffer() is identities[name]
      assert runtime.recovery_state[name]._buffer() is saved[name]
  backup = runtime.recovery_state['state_a']
  backup._buffer().copy_from(Tensor(np.full_like(originals['state_a'], np.inf), device='NPY')._buffer())
  with pytest.raises(ValueError, match='invalid eGPU recovery state'):
    runtime.restore_recovery_state()
  # Recovery warmup must not leave synthetic history in captured live buffers.
  runtime.reset_state()
  for name, alias in aliases.items():
    np.testing.assert_array_equal(alias.numpy(), np.zeros_like(originals[name]))
    np.testing.assert_array_equal(runtime.queues[name].numpy(), np.zeros_like(originals[name]))


@pytest.mark.skipif(os.getenv('MDM_COMPILED_COPIES') != '1', reason='captured transfers require the d39227 runtime')
def test_compiled_transfers_replay_mutable_inputs_and_current_outputs(monkeypatch):
  monkeypatch.syspath_prepend(os.environ['MDM_RUNTIME_ROOT'])
  monkeypatch.setenv('DEV', 'CPU')
  from tinygrad import Tensor, dtypes
  from openpilot.selfdrive.modeld import generic_model_runtime as generic

  runtime = object.__new__(generic.GenericModelRuntime)
  runtime.device, runtime.compiled_copies = 'CPU:1', True
  # Distinct CPU devices exercise actual copy schedules rather than a to-self no-op.
  source = Tensor(np.zeros((1, 18452), np.float32), device=runtime.device).realize()
  runtime.outputs = {'outputs': source}
  shared = np.zeros(3244544, np.uint8)
  captured = []
  compile_copy = generic.compile_copy
  def capture(tensor):
    captured.append(tensor)
    return compile_copy(tensor)
  monkeypatch.setattr(generic, 'compile_copy', capture)
  runtime.bind_transfers(shared)
  assert len(captured) == 2
  storage = runtime.device_buffer
  view = generic.input_view(storage, (2, 3), dtypes.float32, 128)
  original = np.arange(18452, dtype=np.float32).reshape(1, -1)
  for frame in range(16):
    shared[:] = frame
    shared[128:152].view(np.float32)[:] = frame + np.arange(6, dtype=np.float32)
    runtime.copy_inputs()
    np.testing.assert_array_equal(view.numpy(), (frame + np.arange(6)).reshape(2, 3))
    np.testing.assert_array_equal(captured[0].numpy(), shared)
    source._buffer().copy_from(Tensor(original + frame, device='NPY')._buffer())
    result = runtime.copy_outputs().numpy()
    np.testing.assert_array_equal(result, original + frame)
    assert runtime.device_buffer is storage and len(captured) == 2
  # Non-finite results must reach the existing validator, never a cached good result.
  source._buffer().copy_from(Tensor(np.full_like(original, np.nan), device='NPY')._buffer())
  assert np.isnan(runtime.copy_outputs().numpy()).all()


@pytest.mark.skipif(os.getenv('MDM_COMPILED_COPIES') != '1', reason='USB wrap rule belongs to d39227')
def test_usb_ring_wrap_cast_zero_uses_usb_fill(monkeypatch):
  monkeypatch.syspath_prepend(os.environ['MDM_RUNTIME_ROOT'])
  from tinygrad.device import Compiled
  from tinygrad.dtype import dtypes
  from tinygrad.uop.ops import UOp, Ops
  from tinygrad.runtime.support import usb
  from openpilot.selfdrive.modeld.egpu_usb_wrap import configure_usb_wrap
  from openpilot.selfdrive.modeld.egpu_usb_wait import LIGHT_SPEED_RUNTIME_SHA

  original = Compiled.pm_lower
  monkeypatch.setattr(Compiled, 'pm_lower', original)
  monkeypatch.setattr(usb, '_carrot_sdma_wrap_fixed', False, raising=False)
  destination = UOp.alloc((16,), dtypes.uint8, device='AMD', name='ring').index(0)
  zeros = UOp.alloc((16,), dtypes.uint8, device='CPU', name='zeros')
  sentinel = UOp(Ops.NOOP)
  calls = []
  def usb_fill(dst, src, n, fill=False):
    calls.append((dst, src, n, fill))
    return sentinel
  monkeypatch.setattr(usb, 'usb_memcpy', usb_fill)
  monkeypatch.setattr(usb, 'usb_zeros', lambda device: zeros)
  zero = UOp.const(0)
  cast_zero = zero.cast(dtypes.uint64)
  length = UOp.const(4, dtypes.uint64)  # last dword before wrap
  def memset(value):
    return UOp.custom_function('memset', dtype=dtypes.uint64).call(destination, value, length)
  # The exact pinned matcher misses a cast zero; no device access is needed
  # to reproduce this compiler-rule bug. Native USB ring execution is separate.
  assert original.rewrite(memset(cast_zero)) is None
  assert configure_usb_wrap({'runtime': {'sha256': LIGHT_SPEED_RUNTIME_SHA}})
  fixed = Compiled.pm_lower
  assert configure_usb_wrap({'runtime': {'sha256': LIGHT_SPEED_RUNTIME_SHA}})
  assert Compiled.pm_lower is fixed
  for value in (zero, cast_zero):
    assert fixed.rewrite(memset(value)) is sentinel
    assert calls[-1] == (destination, zeros.index(0), length, True)
  calls.clear()
  assert fixed.rewrite(memset(UOp.const(7, dtypes.int))) is None
  assert not calls
