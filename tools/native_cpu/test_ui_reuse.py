"""Content freshness, bounded storage and cold/hot native projection equivalence."""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.system.ui.lib import geometry_cache, native_draw, native_geometry
from openpilot.selfdrive.ui.onroad.path_geometry import project_path, sample_path


def inputs(dtype=np.float32):
  line = np.array([np.linspace(-2, 100, 33), np.sin(np.arange(33)), np.zeros(33)], dtype=dtype).T
  transform = np.array([[540, -900, 0], [360, 0, 900], [1, 0, 0]], dtype=dtype)
  return line, transform, SimpleNamespace(x=-500., y=-500., width=2080., height=1720.)


@pytest.mark.parametrize('function', ['ribbon', 'path'])
def test_projection_cache_invalidation_and_readonly(monkeypatch, function):
  monkeypatch.setattr(geometry_cache, 'ENABLED', True)
  line, transform, clip = inputs()
  fn = native_geometry.project_ribbon if function == 'ribbon' else project_path
  fn.clear_cache()
  args = [line, .16, 1.22, 20, transform, clip] if function == 'ribbon' else [line, 1., -3., 3., transform, clip]
  first = fn(*args)
  assert fn(*args) is first and not first.flags.writeable
  for mutate in (lambda: line.__setitem__((10, 1), 4.), lambda: transform.__setitem__((0, 1), -800.),
                 lambda: setattr(clip, 'width', 1500.), lambda: args.__setitem__(1, .25)):
    old = fn(*args)
    mutate()
    new = fn(*args)
    assert new is not old
    monkeypatch.setattr(geometry_cache, 'ENABLED', False)
    np.testing.assert_array_equal(new, fn(*args))
    monkeypatch.setattr(geometry_cache, 'ENABLED', True)


def test_cache_size_and_inplace_reused_buffers(monkeypatch):
  monkeypatch.setattr(geometry_cache, 'ENABLED', True)
  line, transform, clip = inputs()
  fn = native_geometry.project_ribbon
  fn.clear_cache()
  for i in range(200):
    line[0, 1] = i
    fn(line, .16, 1., 30, transform, clip, max_distance=95. + i/1000.)
  assert fn.cache_stats()['entries'] <= geometry_cache.MAX_ENTRIES
  assert fn.cache_stats()['bytes'] <= geometry_cache.MAX_BYTES


def test_sampled_path_reuses_only_identical_curve_and_distances(monkeypatch):
  monkeypatch.setattr(geometry_cache, 'ENABLED', True)
  line, _, _ = inputs()
  distances = np.linspace(0., 80., 40)
  sample_path.clear_cache()
  old = sample_path(line, distances)
  assert sample_path(line.copy(), distances.copy()) is old
  line[12, 1] += 1.
  fresh = sample_path(line, distances)
  assert not np.array_equal(old, fresh)
  distances[12] += .1
  newest = sample_path(line, distances)
  assert not np.array_equal(fresh, newest)


def test_changing_inputs_bypass_cache_cost_and_recover_on_repetition(monkeypatch):
  monkeypatch.setattr(geometry_cache, 'ENABLED', True)
  key_calls = []
  original = geometry_cache._key
  def count(value):
    key_calls.append(1)
    return original(value)
  monkeypatch.setattr(geometry_cache, '_key', count)
  @geometry_cache.cached_projection
  def compute(value):
    return np.array([value])
  for i in range(geometry_cache.MAX_ENTRIES):
    np.testing.assert_array_equal(compute(i), [i])
  key_calls.clear()
  for i in range(1000, 1032):
    np.testing.assert_array_equal(compute(i), [i])
  assert len(key_calls) == 2  # one full cache probe per 16 calculations
  for _ in range(33):
    result = compute(9999)
  assert compute(9999) is result


@pytest.mark.parametrize('dtype', [np.float32, np.float64])
@pytest.mark.parametrize('invert', [True, False])
def test_batched_preparation_boundary_cases(monkeypatch, dtype, invert):
  monkeypatch.setattr(geometry_cache, 'ENABLED', False)
  line, transform, clip = inputs(dtype)
  line[4:6, 0] = line[3, 0]  # repeated interpolation nodes
  line[8, 1] = np.nan
  for start in (0, 3, -4, 50):
    for end in (-1, 0, 4, 16, 32, 50):
      for distance in (None, -1., 0., 5., 50., np.nan):
        args = (line, .16, 1.22, end, transform, clip, invert, distance, -.9, start)
        monkeypatch.setattr(native_draw, '_ENABLED', False)
        expected = native_geometry.project_ribbon(*args)
        monkeypatch.setattr(native_draw, '_ENABLED', True)
        actual = native_geometry.project_ribbon(*args)
        np.testing.assert_array_equal(actual, expected)
