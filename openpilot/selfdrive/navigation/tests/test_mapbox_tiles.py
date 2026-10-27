from types import SimpleNamespace as NS

import pytest
import requests

from openpilot.selfdrive.navigation.mapbox_tiles import MapboxTiles
from openpilot.selfdrive.navigation.renderer import road_style, MAJOR, MINOR
from openpilot.selfdrive.navigation.tests.test_rendering import field, varint
from openpilot.selfdrive.navigation.vector_tile import decode_roads


@pytest.mark.parametrize('road_class,structure,style', [('motorway', 'bridge', MAJOR), ('service', 'tunnel', MINOR)])
def test_streets_v8_uses_training_style_and_level(road_class, structure, style):
  geometry = b''.join(varint(v) for v in [9, 4096, 0, 10, 0, 8192])
  feature = field(2, b'\x00\x00\x01\x01') + field(3, 2) + field(4, geometry)
  layer = (field(15, 2) + field(1, b'road') + field(2, feature) + field(3, b'class') + field(3, b'structure')
           + field(4, field(1, road_class.encode())) + field(4, field(1, structure.encode())) + field(5, 4096))
  road, = decode_roads(field(3, layer))
  assert road_style(road) == style
  assert road.bridge == (structure == 'bridge') and road.tunnel == (structure == 'tunnel')
  assert road.points.tolist() == [[2048, 0], [2048, 4096]]
  assert not decode_roads(field(3, layer.replace(road_class.encode(), b'railway_'[:len(road_class)])))


def test_mapbox_network_error_never_exposes_key(monkeypatch):
  def fail(*args, **kwargs):
    raise requests.RequestException('url?access_token=secret')
  monkeypatch.setattr(requests, 'Session', lambda: NS(get=fail))
  with pytest.raises(OSError, match='^Mapbox tiles unavailable$'):
    MapboxTiles('secret').get(15, 0, 0)
