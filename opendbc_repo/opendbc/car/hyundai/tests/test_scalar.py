import itertools
import struct

import numpy as np

from opendbc.car.hyundai.scalar import clip_float


def test_float_clip_matches_numpy_boundaries_and_random_values():
  edges = [-np.inf, -3., -0., 0., 2., np.inf, np.nan]
  rng = np.random.default_rng(42)
  values = itertools.chain(itertools.product(edges, repeat=3), rng.normal(size=(10000, 3)))
  for value, lower, upper in values:
    expected = float(np.clip(value, lower, upper))
    actual = clip_float(value, lower, upper)
    assert struct.pack('d', actual) == struct.pack('d', expected)
