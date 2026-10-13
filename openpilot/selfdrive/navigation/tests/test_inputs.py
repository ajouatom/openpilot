import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.generic_model_runtime import host_input_layout, model_metadata
from openpilot.selfdrive.navigation.helpers import navigation_input
from openpilot.selfdrive.navigation.map_client import MapInput
from openpilot.selfdrive.navigation.source import RouteSource, coordinates


@pytest.mark.parametrize('filename,dtype,command_shape,count,states,suffix', [
  ('nav_metadata.json', np.uint8, (47,), 18452, {'state_map_q', 'state_nav_q'}, '56320'),
  ('nav_rl_metadata.json', np.float32, (1, 47), 18580,
   {'state_map_features_q', 'state_navigation_features_q'}, '80eedcdd-f97b-46d0-9b94-780685292acf/12864'),
])
def test_real_nav_artifact_mixed_width_inputs_and_feedback(filename, dtype, command_shape, count, states, suffix):
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  artifact = json.loads((Path(__file__).parents[2] / 'modeld/tests/fixtures' / filename).read_text())
  checkpoint, slices, pairs, output_count = model_metadata(artifact)
  assert checkpoint.endswith(suffix)
  assert output_count == count
  assert states <= pairs.keys()
  raw = np.zeros((1, count), np.float32)
  parsed = Parser().parse_outputs({name: raw[:, section].copy() for name, section in slices.items()})
  assert parsed['plan'].shape == (1, 33, 15) and np.isfinite(parsed['action']).all()
  packed, views, offset = host_input_layout(artifact['input_specs'], 1234, pairs)
  assert views['nav_map'].shape == (1, 1, 256, 256)
  assert views['nav_map'].dtype == dtype
  assert views['navigation'].shape == command_shape
  assert views['navigation'].dtype == np.float32
  views['nav_map'][:] = 255
  views['navigation'].flat[-1] = 1.25
  assert np.all(views['nav_map'] == 255) and views['navigation'].flat[-1] == 1.25
  assert np.all(views['img'] == 0) and packed.size == offset + 2468
  for name, view in views.items():
    if name not in ('tfm', 'big_tfm', 'img', 'big_img'):
      assert (view.ctypes.data - packed.ctypes.data) % 128 == 0


@pytest.mark.parametrize('dtype,shape', [('float16', [1, 47]), ('float32', [47]), ('uint8', [1, 47])])
def test_unrecognized_nav_layout_is_rejected(dtype, shape):
  artifact = json.loads((Path(__file__).parents[2] / 'modeld/tests/fixtures/nav_rl_metadata.json').read_text())
  artifact['input_specs']['nav_map'][1] = dtype
  artifact['input_specs']['navigation'][0] = shape
  with pytest.raises(ValueError, match='navigation input contract'):
    model_metadata(artifact)


def test_unknown_tbt_does_not_become_straight_and_distance_is_finite():
  maneuvers = [NS(type='invalid', modifier='', distance=50), NS(type='turn', modifier='left', distance=120)]
  result = navigation_input(maneuvers, 20)
  assert result[15] == 1 and result[-1] == 1.
  for distance in (float('nan'), float('inf'), -1):
    assert not navigation_input([NS(type='turn', modifier='right', distance=distance)], 0).any()


def test_source_priority_no_route_mixing_and_disconnect_resumes_saved_mapbox():
  source = RouteSource()
  source.mapbox = ((37., 127.), (37.1, 127.1))
  source.legacy = ((38., 128.), (38.1, 128.1))
  assert source.select(False)[0]['latitude'] == 37.
  assert source.select(True)[0]['latitude'] == 38.
  assert source.select(True) is None
  v2 = NS(route=NS(present=False, polyline=()), off_route=False)
  assert source.select(True, v2) == []
  assert source.select(False)[0]['latitude'] == 37.
  assert source.legacy == ()
  assert source.select(True) == []  # reconnect cannot revive an old route


@pytest.mark.parametrize('lat,lon', [(float('nan'), 0), (0, float('inf')), (90, 0), (0, 181)])
def test_reject_bad_route_coordinates(lat, lon):
  assert not coordinates([{'latitude': lat, 'longitude': lon}])


def test_map_input_rejects_stale_image_wrong_route_and_invalid_tbt(monkeypatch):
  from openpilot.selfdrive.navigation import map_client
  monkeypatch.setattr(map_client.time, 'monotonic', lambda: 10.)
  client = MapInput()
  client.client = NS(is_connected=lambda: True, recv=lambda _: None)
  client.image, client.image_time, client.route_time = np.ones((256, 256), np.uint8), 9.8, 123
  class SM(dict):
    pass
  sm = SM(navInstructionCarrot=NS(allManeuvers=[NS(type='turn', modifier='right', distance=100)]))
  sm.valid = {'navInstructionCarrot': True, 'navRoute': True}
  sm.recv_time = {'navInstructionCarrot': 9.9}
  sm['navRoute'] = NS(carrotRouteId=123)
  model = NS(views={'nav_map': None})
  assert 'nav_map' in client.update(model, sm, 10)
  client.route_time = 122
  assert 'nav_map' not in client.update(model, sm, 10)
  client.route_time, client.image_time = 123, 9.5
  assert 'nav_map' not in client.update(model, sm, 10)
  sm.valid['navInstructionCarrot'] = False
  assert client.update(model, sm, 10) == {}
  assert client.update(NS(views={}), sm, 10) == {}
