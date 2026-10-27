import numpy as np
import pytest

from openpilot.selfdrive.navigation import renderer
from openpilot.selfdrive.navigation.tests.test_rendering import local_archive, make_archive, road_tile


@pytest.mark.skipif(renderer._native_draw_segments is None, reason='native rasterizer not built')
@pytest.mark.parametrize('overlay', [False, True])
@pytest.mark.parametrize('intensity', [0, 128, 255])
def test_strokes_match_reference(overlay, intensity):
  rng = np.random.default_rng(173)
  for _ in range(12):
    starts = rng.uniform(-50, 100, (40, 2))
    ends = rng.uniform(-50, 100, (40, 2))
    ends[::9] = starts[::9]  # points and clipped/empty bounding boxes
    radii = rng.uniform(0, 18, 40)
    caps = rng.integers(0, 2, (40, 2), dtype=np.uint8).astype(bool)
    expected = rng.integers(0, 256, (48, 64), dtype=np.uint8)
    actual = expected.copy()
    renderer.draw_segments_reference(expected, starts, ends, radii, intensity, overlay=overlay, caps=caps)
    renderer.draw_segments(actual, starts, ends, radii, intensity, overlay=overlay, caps=caps)
    np.testing.assert_array_equal(actual, expected)


@pytest.mark.skipif(renderer._native_draw_segments is None, reason='native rasterizer not built')
@pytest.mark.parametrize('crossing', [None, 'bridge', 'tunnel'])
def test_complete_render_matches_reference(crossing, monkeypatch):
  with local_archive(make_archive(road_tile(crossing))) as archive:
    source = renderer.MapRenderer(archive)
    position = renderer.Position(0, 0, 23)
    route = np.array([[0, -.001], [0, 0], [.001, .001]])
    actual = source.render(position, route)
    monkeypatch.setattr(renderer, '_native_draw_segments', None)
    expected = source.render(position, route)
    np.testing.assert_array_equal(actual.image, expected.image)
    assert actual.valid == expected.valid
