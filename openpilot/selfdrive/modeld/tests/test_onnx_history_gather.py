"""Constant history selection must preserve order, repeats and fresh JIT inputs."""
import numpy as np
import pytest

from tinygrad import Tensor, TinyJit
from tinygrad.nn.onnx import get_onnx_ops


HISTORY = [i for start in range(9) for i in range(start, start + 25)]
GATHER = get_onnx_ops()['Gather']


@pytest.mark.parametrize('axis', [0, 1, 2, -1, -2, -3])
@pytest.mark.parametrize('indices', [HISTORY, [0, 1, 2, 2, 3, 4, 0], [-3, -2, -1, 0, 1], [4, 3, 2, 1, 0], 4])
def test_constant_gather_matches_numpy(axis, indices):
  shape = [2, 3, 4]
  shape[axis] = 33
  values = np.arange(np.prod(shape), dtype=np.float32).reshape(shape)
  np.testing.assert_array_equal(GATHER(Tensor(values), indices, axis).numpy(), np.take(values, indices, axis=axis))


def test_history_gather_preserves_changed_input_after_capture():
  @TinyJit
  def gather_history(x):
    return GATHER(x, HISTORY, 1).reshape(1, 9, 200).contiguous().realize()

  for seed in (10, 10, 11, 12):
    values = np.random.default_rng(seed).integers(-100, 100, (1, 33, 8)).astype(np.float32)
    actual = gather_history(Tensor(values).realize()).numpy()
    np.testing.assert_array_equal(actual, np.take(values, HISTORY, axis=1).reshape(1, 9, 200))


def test_history_gather_after_noncontiguous_input():
  values = np.arange(264, dtype=np.float32).reshape(1, 8, 33)
  tensor = Tensor(values).permute(0, 2, 1)
  np.testing.assert_array_equal(GATHER(tensor, HISTORY, 1).numpy(), np.take(values.transpose(0, 2, 1), HISTORY, axis=1))
