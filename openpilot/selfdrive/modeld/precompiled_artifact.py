"""Versioned loading and warp compilation inside an artifact's pinned runtime."""
import pickle

SERIALIZATIONS = ('oob-v1', 'persistent-buffer-v1')


def load_artifact(path, serialization):
  if serialization == 'persistent-buffer-v1':
    from examples.openpilot.helpers import load_pickle
    # The upstream loader maps the packed weight arena and resolves persistent
    # buffer IDs. The older protocol-5 loader cannot read this format.
    return load_pickle(path, out_of_band=True)
  if serialization != 'oob-v1':
    raise ValueError('unsupported precompiled serialization')
  from openpilot.selfdrive.modeld.helpers import load_oob
  with path.open('rb') as stream:
    artifact = load_oob(stream)
    if stream.read(1):
      raise ValueError('trailing precompiled model data')
    return artifact


def prepare_jit(run):
  # New retargetable artifacts retain GPU kernels but must compile the host
  # dispatch for the current architecture (the export host can be x86).
  if hasattr(getattr(run, 'captured', None), '_linear'):
    from tinygrad.engine.realize import lower_and_compile
    run.captured._linear = lower_and_compile(run.captured._linear)
  return run


def load_warp(path):
  with path.open('rb') as stream:
    return prepare_jit(pickle.load(stream)['run'])


def compile_warp(frame, size, *, layout, frames, benchmark_runs):
  from examples.openpilot import compile_warp as upstream
  if hasattr(upstream, 'compile_warp'):
    return upstream.compile_warp(frame, size, layout=layout, frames=frames, benchmark_runs=benchmark_runs)

  # The pinned MDM runtime exposes this compiler as a CLI only. Match its
  # yuv420 two-camera construction, capture and changed-input verification.
  import numpy as np
  from tinygrad import Tensor, Device, TinyJit
  from examples.openpilot.helpers import allocate_inputs, benchmark

  if layout != 'yuv420' or frames != 2:
    raise ValueError('unsupported driving warp layout')
  warp = upstream.make_frame_prepare(frame, *size)
  specs = {'input_frame': ((frames, frame.size), np.dtype(np.uint8).str, Device.DEFAULT),
           'M_inv': ((frames, 3, 3), np.dtype(np.float32).str, Device.DEFAULT)}

  def make_inputs(seed):
    rng = np.random.default_rng(seed)
    def initialize(views):
      views['input_frame'][:] = rng.integers(0, 256, views['input_frame'].shape, dtype=np.uint8)
      views['M_inv'][:] = rng.standard_normal(views['M_inv'].shape) * 8
    return allocate_inputs(specs, initialize)

  @TinyJit(prune=True)
  def run(input_frame, M_inv):
    return Tensor.stack(*(warp(input_frame[i], M_inv[i]) for i in range(frames)))

  inputs = make_inputs(42)
  expected = benchmark(run, **inputs)
  for _ in range(2 + benchmark_runs):
    np.testing.assert_array_equal(benchmark(run, **inputs), expected)
  with np.testing.assert_raises(AssertionError):
    np.testing.assert_array_equal(benchmark(run, **make_inputs(43)), expected)
  return {'metadata': {}, 'run': run, 'input_specs': specs}
