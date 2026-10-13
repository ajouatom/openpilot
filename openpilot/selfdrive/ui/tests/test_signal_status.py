"""Exercise both real HUD entry points and the C4 lamp without a display server."""
import ast
import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


ROOT = Path(__file__).parents[1]


@pytest.fixture
def display(monkeypatch):
  rl = Mock()
  rl.Color.side_effect = lambda *values: tuple(values)
  rl.Vector2.side_effect = lambda x, y: NS(x=x, y=y)
  rl.Rectangle.side_effect = lambda x, y, width, height: NS(x=x, y=y, width=width, height=height)
  monkeypatch.setitem(sys.modules, 'pyray', rl)
  spec = importlib.util.spec_from_file_location('openpilot.selfdrive.ui.signal_status', ROOT/'signal_status.py')
  module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
  monkeypatch.setitem(sys.modules, spec.name, module)

  class SM(dict):
    valid = {'longitudinalPlan': True}
    alive = {'longitudinalPlan': True}

  sm = SM(longitudinalPlan=NS(signalDetectionSuspended=True, trafficState=1))
  return module, rl, NS(sm=sm)


def methods(path, names, namespace):
  tree = ast.parse(path.read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
             and any(isinstance(m, ast.FunctionDef) and m.name in names for m in n.body))
  nodes = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
  exec(compile(ast.Module(nodes, type_ignores=[]), str(path), 'exec'), namespace)
  return namespace


def test_pause_is_gray_with_two_bars_and_requires_live_plan(display):
  module, rl, ui = display
  assert module.signal_detection_paused(ui.sm)
  module.draw_signal_pause(100, 200, 24)
  assert rl.draw_circle.call_args.args[-1] == (75, 80, 85, 245)
  assert rl.draw_line_ex.call_count == 2
  ui.sm.alive = {'longitudinalPlan': False}
  assert not module.signal_detection_paused(ui.sm)


def test_c3_pause_replaces_red_and_green_icons(display):
  _, rl, ui = display
  ns = methods(ROOT/'onroad/hud_renderer.py', {'_draw_carrot_traffic_light'}, dict(ui_state=ui))
  renderer = NS(_get_traffic_state=Mock(), _get_traffic_state_carrot=Mock(), _draw_texture_rect=Mock())
  ns['_draw_carrot_traffic_light'](renderer, 100, 200)
  assert rl.draw_circle.call_args.args[:3] == (100, 470, 32)
  renderer._get_traffic_state.assert_not_called()
  renderer._draw_texture_rect.assert_not_called()


def test_c4_navigation_lamp_replaces_counter_with_pause(display):
  _, rl, ui = display
  ns = methods(ROOT/'mici/onroad/hud_renderer.py', {'_get_traffic_light_info', '_draw_traffic_light_info'}, dict(ui_state=ui))
  renderer = NS()
  renderer._traffic_light_info = ns['_get_traffic_light_info'](renderer)
  assert renderer._traffic_light_info == dict(lamp='paused', remain=0)
  assert ns['_draw_traffic_light_info'](renderer, 100, 200)
  assert rl.draw_circle.call_args.args[:3] == (124, 200, 24)


def test_c4_model_lamp_switches_immediately_and_does_not_fade_green(display):
  module, rl, ui = display
  ns = methods(ROOT/'mici/onroad/traffic_light.py', {'_update_state', '_render'},
               dict(ui_state=ui, rl=rl, SIDE_PANEL_WIDTH=80,
                    signal_detection_paused=module.signal_detection_paused,
                    draw_signal_pause=module.draw_signal_pause, draw_circle_gradient=Mock()))
  filt = NS(x=.6, update=Mock())
  renderer = NS(_current_state=1, _green_start_time=None, _alpha_filter=filt,
                _radius=24, rect=NS(x=0, y=0, width=540, height=960))
  ns['_update_state'](renderer)
  assert renderer._current_state == 3 and filt.x == 1.
  ns['_render'](renderer, None)
  assert rl.draw_circle.call_count == 1
  ns['draw_circle_gradient'].assert_not_called()
  ui.sm['longitudinalPlan'].signalDetectionSuspended = False
  ui.sm['longitudinalPlan'].trafficState = 0
  ns['_update_state'](renderer)
  assert renderer._current_state == 0 and filt.x == 0.
