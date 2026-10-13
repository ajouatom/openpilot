"""Bounded diagnostics for rejected compiled outputs; never repair model values."""
import numpy as np


class NonFiniteModelOutput(ValueError):
  """A completed inference with invalid numbers, not a device/protocol failure."""


def validate_compiled_output(result, expected_count):
  if result.size != expected_count:
    raise ValueError(f'invalid precompiled model output: count={result.size}, expected={expected_count}, shape={result.shape}')
  finite = np.isfinite(result)
  if not finite.all():
    flat = result.reshape(-1)
    bad = np.flatnonzero(~finite.reshape(-1))[:8].tolist()
    # Only on failure. No camera, map, raw tensor or unbounded index dump.
    counts = f'nan={np.count_nonzero(np.isnan(flat))}, posinf={np.count_nonzero(np.isposinf(flat))}'
    raise NonFiniteModelOutput(f'invalid precompiled model output: {counts}, '
                               + f'neginf={np.count_nonzero(np.isneginf(flat))}, first_bad_indices={bad}')
