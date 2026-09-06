import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.ui.tests.test_carrot_model_renderer_lane_visibility import model_renderer_module as model_renderer_module
from openpilot.selfdrive.ui.tests.test_vision_status import NOW, packet


def load_module(monkeypatch, name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, name, module)
  spec.loader.exec_module(module)
  return module


@pytest.fixture
def mici_modules(model_renderer_module, monkeypatch):
  shared = model_renderer_module
  ui = sys.modules['openpilot.selfdrive.ui.ui_state']
  ui.UIStatus = SimpleNamespace(DISENGAGED=0, ENGAGED=1, OVERRIDE=2)
  application = sys.modules['openpilot.system.ui.lib.application']
  application.FontWeight.MEDIUM = 1
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.multilang', SimpleNamespace(tr=lambda text: text))
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.text_measure', SimpleNamespace(measure_text_cached=lambda *_args: shared.rl.Vector2(30, 13)))
  directory = Path(__file__).parents[1] / 'onroad'
  model = load_module(monkeypatch, '_test_mici_vision_model', directory / 'model_renderer.py')
  vision = load_module(monkeypatch, '_test_mici_vision_status', directory / 'vision_renderer.py')
  return model, vision


def test_mici_draws_classified_dashes_and_double_yellow(mici_modules):
  module, _ = mici_modules
  renderer = object.__new__(module.ModelRenderer)
  line = np.array([[x, 0, 0] for x in range(0, 61, 2)], dtype=np.float32)
  renderer._lane_lines = [module.ModelPoints(raw_points=line.copy()) for _ in range(4)]
  renderer._road_edges = []
  renderer._path = module.ModelPoints(raw_points=line.copy())
  renderer._lane_line_probs = np.ones(4, dtype=np.float32)
  renderer._lane_marking_segments = [[] for _ in range(4)]
  renderer._map_line_to_polygon = lambda points, *_args, **_kwargs: points[:, :2]
  renderer._acceleration_x = []
  renderer._acceleration_x_filter = renderer._acceleration_x_filter2 = SimpleNamespace(update=lambda *_args: None)
  renderer._experimental_mode = False
  renderer._path_offset_z = 0
  renderer._update_model(None, line[:, 0], SimpleNamespace(leftLaneLine=24, rightLaneLine=0))
  assert len(renderer._lane_marking_segments[1]) == 2
  assert len(renderer._lane_marking_segments[2]) > 2
  assert all(np.ptp(points[:, 0]) <= module.lane_dash_segments(line, 60)[0][-1, 0] + 0.01 for points in renderer._lane_marking_segments[2])


@pytest.mark.parametrize('left,right,valid,expected_sides', [(True, False, True, 1), (True, True, True, 2), (False, False, True, 0), (True, False, False, 0)])
def test_mici_blindspot_is_independent_of_radar(mici_modules, monkeypatch, left, right, valid, expected_sides):
  module, _ = mici_modules
  renderer = object.__new__(module.ModelRenderer)
  renderer._path = module.ModelPoints(raw_points=np.array([[x, 0, 0] for x in range(0, 61, 2)], dtype=np.float32))
  renderer._car_space_transform = np.eye(3)
  renderer._clip_region = None
  renderer._rect = None
  calls = []
  monkeypatch.setattr(module, 'project_blindspot_barrier', lambda *args: calls.append(args) or np.empty((0, 2)))
  car_valid = valid
  class SM(dict):
    valid = {'carState': car_valid, 'modelV2': True}
    alive = {'carState': True, 'modelV2': True}
  renderer._draw_blindspots(SM(carState=SimpleNamespace(leftBlindspot=left, rightBlindspot=right)))
  assert len(calls) == expected_sides


@pytest.mark.parametrize('enabled,detected,clear_side,expected', [
  (False, True, '', 'BSD'), (True, False, '', 'STANDBY'),
  (True, False, 'left', 'NO DETECTION'), (True, True, 'left', 'DETECTED'),
])
def test_mici_status_preserves_oem_warning_and_never_assumes_clear(mici_modules, monkeypatch, enabled, detected, clear_side, expected):
  module, vision = mici_modules
  renderer = vision.VisionRenderer()
  renderer._packet = packet(side=clear_side)
  texts = []
  renderer._text = lambda text, *args: texts.append(text)
  class SM(dict):
    valid = {'carState': True}
    alive = {'carState': True}
    recv_frame = {'carState': 10}
  monkeypatch.setattr(vision, 'ui_state', SimpleNamespace(started=True, started_frame=1, share_data=enabled,
                                                        sm=SM(carState=SimpleNamespace(leftBlindspot=detected, rightBlindspot=False))))
  monkeypatch.setattr(vision.time, 'monotonic_ns', lambda: NOW)
  renderer._render(module.rl.Rectangle(0, 0, 476, 240))
  assert expected in texts
  if not enabled:
    assert 'VISION' not in texts
  elif detected or clear_side:
    assert 'BSD L' in texts


def test_mici_status_card_clears_speed_and_gear(mici_modules, monkeypatch):
  module, vision = mici_modules
  renderer = vision.VisionRenderer()
  rectangles = []
  renderer._text = lambda *_args: None
  monkeypatch.setattr(vision.rl, 'draw_rectangle_rounded', lambda rect, *_args: rectangles.append(rect))
  class SM(dict):
    valid = {'carState': False}
    alive = {'carState': False}
    recv_frame = {'carState': 0}
  monkeypatch.setattr(vision, 'ui_state', SimpleNamespace(started=True, started_frame=1, share_data=True, sm=SM()))
  renderer._render(module.rl.Rectangle(0, 0, 476, 240))
  card = rectangles[-1]
  # Actual HUD speed panel ends at 317; the gear box extends to 373.
  assert card.x > 373
  assert card.x + card.width <= 476 - 14
  assert card.y + card.height <= 240 - 18
