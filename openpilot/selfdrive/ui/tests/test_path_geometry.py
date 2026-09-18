from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.onroad.path_geometry import project_path, sample_path


def scalar_reference(line, width, z_start, z_end, transform, clip, invert):
  left, right = [], []
  for x, y, z in line:
    z_off = np.interp(float(x), [0., 100.], [z_start, z_end])
    y_off = np.interp(z_off, [-3., 0., 3.], [1.5, .5, 1.5]) * width
    pair = []
    for side in (-1, 1):
      point = transform @ np.array([x, y + side * y_off, z + z_off])
      if abs(point[2]) < 1e-6:
        break
      px, py = point[:2] / point[2]
      if not (clip.x <= px <= clip.x + clip.width and clip.y <= py <= clip.y + clip.height):
        break
      pair.append((px, py))
    if len(pair) == 2 and (invert or not left or pair[0][1] <= left[-1][1]):
      left.append(pair[0])
      right.insert(0, pair[1])
  return np.asarray(left + right, dtype=np.float32).reshape(-1, 2)


@pytest.mark.parametrize('invert', [False, True])
@pytest.mark.parametrize('offsets', [(1.22, 1.22), (-3., 3.), (0., 0.)])
def test_batched_projection_matches_scalar_clipping_and_vertex_order(invert, offsets):
  rng = np.random.default_rng(42)
  clip = SimpleNamespace(x=0, y=0, width=2160, height=1080)
  transform = np.array([[1080., -950., 0.], [540., 0., 950.], [1., 0., 0.]])
  for _ in range(30):
    line = np.column_stack((np.linspace(0., 180., 33), rng.normal(0, 10, 33), rng.normal(0, 1, 33)))
    actual = project_path(line, 1.2, *offsets, transform, clip, invert)
    expected = scalar_reference(line, 1.2, *offsets, transform, clip, invert)
    np.testing.assert_allclose(actual, expected, atol=.0002, rtol=1e-6)


def test_distance_interpolation_preserves_repeated_and_reversed_nodes():
  line = np.array([[0, 1, 2], [10, 2, 3], [5, 5, 4], [30, 3, 5]], dtype=np.float32)
  distances = [0., 5., 10., 15., 30., 40.]
  points = sample_path(line, distances)
  indices = np.interp(distances, [0., 10., 10., 30.], [0., 1., 2., 3.])
  np.testing.assert_allclose(points[:, 1], np.interp(indices, np.arange(4), line[:, 1]))
  np.testing.assert_allclose(points[:, 2], np.interp(indices, np.arange(4), line[:, 2]))


def test_empty_path_is_an_empty_polygon():
  assert project_path(np.empty((0, 3)), 1., 1., 1., np.eye(3), None).shape == (0, 2)
