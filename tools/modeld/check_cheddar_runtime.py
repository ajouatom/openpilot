"""Native CPU execution checks for the exact internal runtime and bundle format."""
import argparse
from pathlib import Path
import pickle
import tempfile

import numpy as np

from openpilot.selfdrive.modeld.cheddar_runtime import activate, dump_bundle, load_bundle


def check(directory):
  activate(directory / 'runtime')
  from tinygrad import Tensor, TinyJit
  from openpilot.selfdrive.modeld.generic_model_runtime import compile_copy, input_view
  from openpilot.selfdrive.modeld.precompiled_artifact import prepare_jit
  from openpilot.common.file_chunker import chunk_file, get_chunk_targets

  @TinyJit
  def policy(state, control, outputs):
    # Materialize the next value before either destination is written. A toy
    # graph that re-reads state after writing its alias is not alias-safe.
    next_value = (state + control).contiguous().realize()
    Tensor.realize(outputs['next'].assign(next_value), outputs['result'].assign(next_value * 2))

  @TinyJit
  def warp(frame):
    return (frame * 3).realize()

  state, control = Tensor.zeros(4).realize(), Tensor.ones(4).realize()
  outputs = {'next': Tensor.zeros(4).realize(), 'result': Tensor.zeros(4).realize()}
  for _ in range(3):
    policy(state, control, outputs)
    warp(control)
  bundle = {'run': policy, 'warp': pickle.dumps(warp)}
  path = directory / 'bundle.pkl'
  dump_bundle(bundle, path)
  chunk_file(path, get_chunk_targets(path, path.stat().st_size))
  loaded = load_bundle(path)
  run, prepare = prepare_jit(loaded['run']), prepare_jit(pickle.loads(loaded['warp']))
  # As in upstream: distinct Tensor views preserve the state/output alias while
  # retaining all captured JIT arguments. Reusing the same Tensor is incorrect.
  outputs['next'] = input_view(state._buffer(), state.shape, state.dtype)
  download = compile_copy(outputs['result'].to('CPU'))
  download().numpy()
  captures = []
  for _ in range(2):
    state._buffer().copy_from(Tensor(np.zeros(4, np.float32), device='NPY')._buffer())
    values = []
    for index in range(10):
      control._buffer().copy_from(Tensor(np.full(4, index + 1, np.float32), device='NPY')._buffer())
      warped = prepare(control)
      run(state, warped, outputs)
      values.append(download().numpy().copy())
      np.testing.assert_array_equal(values[-1], np.full(4, 3 * (index + 1) * (index + 2), np.float32))
    captures.append(values)
  np.testing.assert_array_equal(*captures)
  print('Cheddar runtime: independent JIT serialization, chunk loading, aliases, fresh inputs and reset PASS')


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--directory', type=Path)
  args = parser.parse_args()
  if args.directory:
    args.directory.mkdir(parents=True, exist_ok=True)
    check(args.directory)
  else:
    with tempfile.TemporaryDirectory() as temporary:
      check(Path(temporary))
