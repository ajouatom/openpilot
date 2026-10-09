"""Exercise the real C4 mode/render methods without native display dependencies."""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


SOURCE = Path(__file__).parents[1] / "mici" / "onroad" / "augmented_road_view.py"


class CameraBase:
  def _render(self, rect):
    self.camera(rect)


@pytest.fixture
def screen():
  tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "AugmentedRoadView")
  cls.bases = [ast.Name(id="CameraBase", ctx=ast.Load())]
  cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef)
              and node.name in ("_road_view_mode", "_render", "set_cluster_hud_connected")]
  state = SimpleNamespace(started=True, show_model_view=0, show_brightness_ratio=1.0, started_time=100.0, sm={})
  rl = Mock()
  rl.Rectangle.side_effect = lambda x, y, width, height: SimpleNamespace(x=x, y=y, width=width, height=height)
  namespace = {
    "CameraBase": CameraBase, "ui_state": state, "rl": rl, "SIDE_PANEL_WIDTH": 80,
    "time": SimpleNamespace(monotonic=lambda: 100.0),
    "gui_app": SimpleNamespace(is_recording=lambda: False), "messaging": Mock(),
    "native_draw": SimpleNamespace(active=lambda: False),
    "native_geometry": SimpleNamespace(active=lambda: False),
    "native_text": SimpleNamespace(active=lambda: False),
  }
  exec(compile(ast.fix_missing_locations(ast.Module(body=[cls], type_ignores=[])), str(SOURCE), "exec"), namespace)
  view = namespace["AugmentedRoadView"]()
  for name in ("camera", "_refresh_plot_mode", "_switch_stream_if_needed", "_update_calibration",
               "_model_renderer", "_hud_renderer", "_vision_renderer", "_alert_renderer", "_driver_state_renderer",
               "_traffic_light", "_confidence_ball", "_bookmark_icon", "_offroad_label", "_pm"):
    setattr(view, name, Mock())
  view.rect = SimpleNamespace(x=0, y=0, width=540, height=960)
  view._fade_texture = object()
  view._plot_mode = 0
  view._alert_renderer.will_render.return_value = (None, False)
  view._render_diagnostics = Mock()
  view._render_diagnostics.values = {}
  view._render_diagnostics.call.side_effect = lambda label, callback, *args: callback(*args)
  view.set_cluster_hud_connected(False)
  return view, state, rl


@pytest.mark.parametrize("mode", range(4))
@pytest.mark.parametrize("ratio", (0.0, 0.5, 1.0))
@pytest.mark.parametrize("brightness", (0, 5, 50, 100))
def test_mode_applies_at_onroad_start_at_every_brightness(screen, mode, ratio, brightness):
  view, state, _ = screen
  state.show_model_view = mode
  state.show_brightness_ratio = ratio
  state.sm = {"deviceState": SimpleNamespace(screenBrightnessPercent=brightness)}
  assert view._road_view_mode() == mode


def test_live_mode_changes_do_not_need_brightness_telemetry(screen):
  view, state, _ = screen
  for mode in (3, 1, 2, 0):
    state.show_model_view = mode
    assert view._road_view_mode() == mode


@pytest.mark.parametrize("mode", range(4))
def test_offroad_keeps_the_normal_view(screen, mode):
  view, state, _ = screen
  state.started = False
  state.show_model_view = mode
  assert view._road_view_mode() == 0


@pytest.mark.parametrize("stored,expected", ((-1, 0), (4, 3)))
def test_out_of_range_values_are_bounded(screen, stored, expected):
  view, state, _ = screen
  state.show_model_view = stored
  assert view._road_view_mode() == expected


@pytest.mark.parametrize("mode,camera,model", ((0, True, True), (1, True, False), (2, False, True), (3, False, False)))
@pytest.mark.parametrize("connected,show_camera", ((False, False), (True, False), (True, True)))
def test_render_combinations_preserve_alerts_and_cluster_preference(screen, mode, camera, model, connected, show_camera):
  view, state, rl = screen
  state.show_model_view = mode
  view.set_cluster_hud_connected(connected, show_camera)
  suppressed = connected and not show_camera
  view._render(None)
  assert view.camera.call_count == int(camera and not suppressed)
  assert view._model_renderer.render.call_count == int(model and not suppressed)
  assert rl.draw_rectangle_rec.call_count == int(suppressed or not camera)
  view._hud_renderer.render.assert_called_once()
  view._alert_renderer.render.assert_called_once()
  view._driver_state_renderer.draw_onroad.assert_called_once()
