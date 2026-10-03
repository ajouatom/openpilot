"""Projection equivalence, clipping boundaries and optional-backend fallback."""
from types import SimpleNamespace
from pathlib import Path
import ast

import numpy as np
import pytest

from openpilot.system.ui.lib import _draw_native as native, native_draw, native_geometry as geometry
from openpilot.selfdrive.ui.onroad.path_geometry import project_path
from openpilot.selfdrive.ui.road_markings import lane_dash_segments, project_lane_segments, project_blindspot_barrier


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
@pytest.mark.parametrize('invert', [True, False])
def test_clip_boundaries_and_nonfinite_values(monkeypatch, dtype, invert):
  eps = dtype(1e-6)
  values = [0., -0., -1., 1., np.nextafter(dtype(1), dtype(2)), np.nextafter(dtype(1), dtype(0)), np.nan, np.inf, -np.inf]
  rng = np.random.default_rng(234)
  p = rng.uniform(-1.5, 1.5, (3, 2, 200)).astype(dtype)
  p[2] = 1
  for i, value in enumerate(values):
    p[0, :, i] = value
    p[1, :, i] = 0
  for i, depth in enumerate([0, eps, -eps, np.nextafter(eps, dtype(0)), np.nan, np.inf, -np.inf]):
    p[:, :, i+20] = 0
    p[2, :, i+20] = depth
  clip = SimpleNamespace(x=-1., y=-1., width=2., height=2.)
  p.flags.writeable = False
  with np.errstate(invalid='ignore', divide='ignore'):
    monkeypatch.setattr(native_draw, '_ENABLED', False)
    expected = geometry.clip_ribbon(p, clip, invert)
    actual = native.clip_ribbon(p, -1., 1., -1., 1., invert)
  np.testing.assert_array_equal(actual, expected)
  assert actual.dtype == np.float32 and actual.flags.c_contiguous


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
@pytest.mark.parametrize('count', [0, 1, 33, 257])
def test_offset_rounding_strides_and_readonly(dtype, count):
  points = np.random.default_rng(42).normal(size=(3, count)).astype(dtype).T
  points.flags.writeable = False
  offsets = np.array([[0., -.9-.16, 1.22], [0., -.9+.16, .6]], dtype=np.float32)
  expected = points[None, :, :] + offsets[:, None, :]
  actual = native.offset_sides(points, -.9-.16, -.9+.16, 1.22, .6)
  np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
@pytest.mark.parametrize('invert', [True, False])
def test_shared_projections_match_python(monkeypatch, dtype, invert):
  rng = np.random.default_rng(871)
  clip = SimpleNamespace(x=-100.5, y=-100.5, width=2361., height=1281.)
  for _ in range(40):
    line = np.array([np.linspace(-2, 120, 33), rng.normal(0, 4, 33), rng.normal(0, 2, 33)], dtype=dtype).T
    transform = np.array([[1080, -950, 0], [540, 0, 950], [1, 0, 0]], dtype=dtype)
    transform += rng.normal(0, .02, (3, 3)).astype(dtype)
    line.flags.writeable = False
    segments = lane_dash_segments(line, 95.)

    def outputs(line=line, transform=transform, segments=segments):
      return [geometry.project_ribbon(line, .16, 1.22, 20, transform, clip, invert, 76.5, -.9, 2),
              geometry.project_ribbon(line, .5, 0., 29, transform, clip, invert),
              project_path(line, 1.2, -3., 3., transform, clip, invert),
              project_blindspot_barrier(line, -.7, transform, clip),
              *project_lane_segments(segments, .05, transform, clip)]

    monkeypatch.setattr(native_draw, '_ENABLED', False)
    expected = outputs()
    monkeypatch.setattr(native_draw, '_ENABLED', True)
    actual = outputs()
    assert len(actual) == len(expected)
    for a, b in zip(actual, expected, strict=True):
      np.testing.assert_array_equal(a, b)


@pytest.mark.parametrize('backend', [None, SimpleNamespace()])
def test_missing_or_older_binary_falls_back(monkeypatch, backend):
  monkeypatch.setattr(native_draw, '_draw_native', backend)
  assert not geometry.active()
  line = np.array([[2., 0., 0.], [4., 0., 0.]], dtype=np.float32)
  transform = np.array([[1, 0, 0], [0, 0, 1], [1, 0, 0]], dtype=np.float32)
  clip = SimpleNamespace(x=0., y=0., width=2., height=2.)
  np.testing.assert_array_equal(geometry.project_ribbon(line, .1, 1., 1, transform, clip),
                                [[1., .5], [1., .25], [1., .25], [1., .5]])


def test_invalid_native_shapes_rejected():
  with pytest.raises(ValueError):
    native.clip_ribbon(np.zeros((3, 1, 8), np.float32), 0., 1., 0., 1.)
  with pytest.raises(ValueError):
    native.offset_sides(np.zeros((3, 2), np.float32), 0., 0., 0., 0.)
  with pytest.raises(ValueError):
    native.path_sides(np.zeros((3, 3)), np.zeros(2), np.zeros(3))
  for lengths in [[-1, 9], [9], [7], [1, 8]]:
    with pytest.raises(ValueError):
      native.clip_dashes(np.zeros((3, 2, 8), np.float32), np.ones(8, np.float32), np.array(lengths, dtype=np.int64), 0., 1., 0., 1.)


@pytest.mark.parametrize('enabled', [True, False])
def test_c3_endpoint_and_c4_node_only_policy(monkeypatch, enabled):
  monkeypatch.setattr(native_draw, '_ENABLED', enabled)
  # Identity-like screen projection exposes the interpolated distance endpoint.
  line = np.array([[0., 0., 1.], [10., 0., 1.], [20., 0., 1.]], np.float32)
  clip = SimpleNamespace(x=0., y=-10., width=100., height=20.)
  transform = np.eye(3, dtype=np.float32)
  c3 = geometry.project_ribbon(line, .16, 0., 1, transform, clip, True, 15., .9, 1)
  c4 = geometry.project_ribbon(line, .16, 0., 1, transform, clip)
  np.testing.assert_array_equal(c3[:, 0], [10., 15., 15., 10.])
  np.testing.assert_array_equal(c4[:, 0], [0., 10., 10., 0.])
  assert geometry.project_ribbon(line[:0], .16, 0., 1, transform, None).shape == (0, 2)
  assert geometry.project_ribbon(line, .16, 0., -1, transform, None).shape == (0, 2)


@pytest.mark.parametrize('variant', ['onroad', 'mici/onroad'])
def test_real_renderer_uses_shared_geometry(variant):
  path = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/ui' / variant / 'model_renderer.py'
  tree = ast.parse(path.read_text(encoding='utf8'))
  node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_map_line_to_polygon')
  scope = {'np': np, 'native_geometry': geometry}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), scope)
  line = np.array([[0., 0., 1.], [10., 0., 1.], [20., 0., 1.]], np.float32)
  renderer = SimpleNamespace(_car_space_transform=np.eye(3, dtype=np.float32),
                             _clip_region=SimpleNamespace(x=0., y=-10., width=100., height=20.))
  if variant == 'onroad':
    result = scope[node.name](renderer, line, .16, 0., 1, 15., True, .9, 1)
    np.testing.assert_array_equal(result[:, 0], [10., 15., 15., 10.])
  else:
    result = scope[node.name](renderer, line, .16, 0., 1)
    np.testing.assert_array_equal(result[:, 0], [0., 10., 10., 0.])


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
@pytest.mark.parametrize('invert', [True, False])
def test_batched_preparation_boundary_cases(monkeypatch, dtype, invert):
  line = np.array([np.linspace(-2, 100, 33), np.sin(np.arange(33)), np.zeros(33)], dtype=dtype).T
  transform = np.array([[540, -900, 0], [360, 0, 900], [1, 0, 0]], dtype=dtype)
  clip = SimpleNamespace(x=-500., y=-500., width=2080., height=1720.)
  line[4:6, 0] = line[3, 0]  # repeated interpolation nodes
  line[8, 1] = np.nan
  for start in (0, 3, -4, 50):
    for end in (-1, 0, 4, 16, 32, 50):
      for distance in (None, -1., 0., 5., 50., np.nan):
        args = (line, .16, 1.22, end, transform, clip, invert, distance, -.9, start)
        monkeypatch.setattr(native_draw, '_ENABLED', False)
        expected = geometry.project_ribbon(*args)
        monkeypatch.setattr(native_draw, '_ENABLED', True)
        actual = geometry.project_ribbon(*args)
        np.testing.assert_array_equal(actual, expected)
