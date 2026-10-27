"""RL navigation contract, comma packing, host history and map demand."""
from concurrent.futures import Future
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / 'third_party/jetlink'), str(Path(__file__).parent)]
from jetlink.spec import ModelSpec
from openpilot.common.jetson_model import selected_release
from openpilot.selfdrive.modeld.jetlink.contracts import JETSON_SPEC as SPEC, validate_model_spec, parse_outputs
from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
from openpilot.selfdrive.modeld.jetlink import model
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
from openpilot.selfdrive.navigation.jetlink_demand import jetlink_map_demand
from test_hotplug_resources import joining_model


def buffers():
  packed = np.zeros(SPEC.packed_nelem, np.float32)
  views = {name: array.reshape(shape) for (name, shape), array in zip(
    SPEC.packed_shapes.items(), np.split(packed, np.cumsum(SPEC.packed_sizes[:-1])), strict=True)}
  return packed, views


def frame_inputs():
  return {'desire_pulse': np.eye(8, dtype=np.float32)[2], 'traffic_convention': np.array([1, 0], np.float32),
          'action_t': np.array([.2, .3], np.float32),
          'nav_map': np.arange(65536, dtype=np.uint8).reshape(256, 256),
          'navigation': np.arange(47, dtype=np.float32)}


def test_n16_pin_keeps_nav_parser_shapes_and_matching_signed_release():
  fixture = json.loads((ROOT / 'openpilot/selfdrive/modeld/tests/fixtures/nav_rl_metadata.json').read_text())
  assert SPEC.input_shapes == {k: tuple(v[0]) for k, v in fixture['input_specs'].items()}
  assert SPEC.output_shapes == {k: tuple(v[0]) for k, v in fixture['output_specs'].items()}
  assert fixture['input_specs']['nav_map'][1] == fixture['input_specs']['navigation'][1] == 'float32'
  # The older fixture establishes the unchanged tensor/parser ABI, not model identity.
  assert SPEC.checkpoint == '2931d3d9-7c90-4988-a6af-654ed71e0af5/56320/96244b1f-6c4e-4adc-848d-eccd0b44b182'
  assert SPEC.sha256 == '01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a'
  assert SPEC.nbytes == 814591107
  assert len(SPEC.state_pairs) == 5 and SPEC.output_nelem == 18580
  assert SPEC.packed_nelem == 65595 and SPEC.packed_nbytes == 262380
  selection = selected_release(ROOT / 'openpilot/selfdrive/modeld/jetlink/host_release.json')
  assert selection['model'] == {
    'url': 'https://upload.shind0.synology.me/models/comma4-big-nav-n16-96244b1f/big_driving_supercombo.onnx',
    'sha256': SPEC.sha256, 'size': SPEC.nbytes}
  assert 'signature' in selection and selection.get('delivery') != 'local'
  parsed = parse_outputs(Parser(), SPEC, np.zeros(SPEC.output_nelem, np.float32))
  assert parsed['plan'].shape == (1, 33, 15) and parsed['action'].shape == (1, 4)
  assert all(np.isfinite(value).all() for value in parsed.values())


def test_legacy_resaction_and_official_cinque_wire_contracts_still_work():
  legacy = ModelSpec.from_dict(json.loads((Path(__file__).parent / 'fixtures/resaction_model.json').read_text()))
  from openpilot.selfdrive.modeld.jetlink.contracts import DEFAULT_SPEC
  assert validate_model_spec(legacy) == legacy
  assert legacy.packed_nbytes == 48 and len(legacy.state_pairs) == 3
  assert validate_model_spec(DEFAULT_SPEC) == DEFAULT_SPEC
  assert 'nav_map' not in DEFAULT_SPEC.packed_shapes


@pytest.mark.parametrize('change', ['missing_map', 'missing_instruction', 'map_shape', 'instruction_shape', 'state_shape'])
def test_bad_navigation_contract_rejected(change):
  shapes = dict(SPEC.input_shapes)
  if change == 'missing_map':
    del shapes['nav_map']
  elif change == 'missing_instruction':
    del shapes['navigation']
  elif change == 'map_shape':
    shapes['nav_map'] = (1, 1, 128, 128)
  elif change == 'instruction_shape':
    shapes['navigation'] = (47,)
  else:
    shapes['state_navigation_features_q'] = (128, 1, 63)
  with pytest.raises(ValueError):
    validate_model_spec(replace(SPEC, input_shapes=shapes))


def test_float_map_preserves_pixel_range_and_missing_fields_clear_each_frame():
  packed, views = buffers()
  previous = np.zeros(8, np.float32)
  inputs = frame_inputs()
  fill_inputs(views, inputs, previous)
  assert views['nav_map'].dtype == np.float32 and views['nav_map'].max() == 255
  np.testing.assert_array_equal(views['nav_map'][0, 0], inputs['nav_map'])
  np.testing.assert_array_equal(views['navigation'][0], inputs['navigation'])
  assert views['desire'][2] == 1
  del inputs['nav_map']
  fill_inputs(views, inputs, previous)
  assert not views['nav_map'].any() and not views['desire'].any()
  assert views['navigation'].any()  # fresh instruction can exist without tiles
  del inputs['navigation']
  fill_inputs(views, inputs, previous)
  assert not packed[12:].any()


@pytest.mark.parametrize('name,value', [('nav_map', np.ones((1,))), ('navigation', np.ones(46)),
                                      ('nav_map', np.full((256, 256), 256.)),
                                      ('nav_map', np.full((256, 256), -1.)),
                                      ('navigation', np.full(47, np.nan))])
def test_invalid_nav_input_is_not_transmitted(name, value):
  _, views = buffers()
  inputs = frame_inputs()
  inputs[name] = value
  with pytest.raises(ValueError):
    fill_inputs(views, inputs, np.zeros(8, np.float32))


def test_real_joining_adapter_transmits_nav_then_clears_it_and_falls_back(monkeypatch, tmp_path):
  m = joining_model(monkeypatch)
  received = []
  def infer(images, packed, frame, reset, source_sof):
    received.append(packed.copy())
    if len(received) == 3:
      raise ConnectionError('lost USB')
    return np.zeros(SPEC.output_nelem, np.float32)
  client = NS(spec=SPEC, image_buffer=lambda: None, infer=infer, timings=(0, 0, 0), close=lambda: None)
  future = Future()
  future.set_result(client)
  m.connection = NS(future=future)
  class Warp:
    timings = (0, 0, 0)
    def __call__(self, *args, prepare=None, output_buffer=None):
      if prepare is not None: prepare()
      return np.zeros(SPEC.warped_shape, np.uint8)
  m.warp = Warp()
  m.small.reset = lambda: None
  monkeypatch.setattr(model, 'FAULT', tmp_path / 'fault')
  inputs = frame_inputs()
  assert m.run({}, {}, inputs, False) == 'external'
  np.testing.assert_array_equal(received[0][12:12 + 65536], inputs['nav_map'].ravel())
  np.testing.assert_array_equal(received[0][-47:], inputs['navigation'])
  del inputs['nav_map'], inputs['navigation']
  assert m.run({}, {}, inputs, False) == 'external'
  assert not received[1][12:].any()
  assert m.run({}, {}, inputs, False) == 'local'
  assert not m.active and m.client is None and (tmp_path / 'fault').exists()


@pytest.mark.parametrize('active,nav,updated,expected', [
  (True, True, 10., True), (False, True, 10., False), (True, False, 10., False),
  (True, True, 7.5, False), (True, True, 11., False), (True, True, float('nan'), False),
  (True, True, True, False), (1, True, 10., False)])
def test_map_demand_is_fresh_and_does_not_reuse_egpu_flag(tmp_path, active, nav, updated, expected):
  path = tmp_path / 'state.json'
  path.write_text(json.dumps({'active': active, 'nav_model': nav, 'updated': updated}))
  assert jetlink_map_demand(path, now=10.) is expected


def test_missing_or_partial_status_does_not_enable_maps(tmp_path):
  path = tmp_path / 'state.json'
  assert not jetlink_map_demand(path)
  for text in ('{', 'null', '[]', '{}'):
    path.write_text(text)
    assert not jetlink_map_demand(path)


def test_model_status_publishes_nav_demand_and_releases_it_on_fallback(monkeypatch, tmp_path):
  m = joining_model(monkeypatch)
  m.spec = SPEC
  m.phase_enabled = False
  m.active = True
  path = tmp_path / 'status.json'
  monkeypatch.setattr(model, 'MODEL_STATUS', path)
  monkeypatch.setattr(model, 'state', lambda: {'state': 'ready'})
  monkeypatch.setattr(model, 'may_join', lambda *a: True)
  class SM(dict):
    valid = alive = dict.fromkeys(('carState', 'selfdriveState', 'carControl'), True)
    recv_time = {}
  sm = SM.fromkeys(SM.valid, NS())
  m.update(sm, NS(timestamp_sof=0))
  assert jetlink_map_demand(path)
  m.active, m.next_status = False, 0
  m.update(sm, NS(timestamp_sof=0))
  assert not jetlink_map_demand(path)
