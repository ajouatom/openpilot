import ast
import importlib.util
import sys
import time as stdlib_time
import types
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest


HUD_RENDERER_PATH = Path(__file__).parents[1] / "onroad" / "hud_renderer.py"


@pytest.fixture
def hud_module(monkeypatch):
  @dataclass(frozen=True)
  class Color:
    r: int
    g: int
    b: int
    a: int

  @dataclass
  class Rectangle:
    x: float
    y: float
    width: float
    height: float

  @dataclass
  class Vector2:
    x: float
    y: float

  raylib = types.ModuleType("pyray")
  raylib.Color = Color
  raylib.Rectangle = Rectangle
  raylib.Vector2 = Vector2
  raylib.WHITE = Color(255, 255, 255, 255)
  raylib.BLACK = Color(0, 0, 0, 255)
  raylib.BLANK = Color(0, 0, 0, 0)
  raylib.GREEN = Color(0, 255, 0, 255)
  raylib.YELLOW = Color(255, 255, 0, 255)
  for name in (
    "draw_line_ex",
    "draw_rectangle_gradient_v",
    "draw_rectangle_rounded",
    "draw_rectangle_rounded_lines_ex",
    "draw_text_ex",
    "draw_texture_pro",
  ):
    setattr(raylib, name, lambda *args, **kwargs: None)

  class FakeExpButton:
    def __init__(self, *args):
      self.is_pressed = False

    def render(self, rect):
      pass

  class Widget:
    def __init__(self):
      pass

  fake_ui_state = SimpleNamespace(
    params=None,
    sm=None,
    started_frame=0,
    status=0,
    is_metric=True,
  )
  gui_app = SimpleNamespace(
    font=lambda weight: ("font", weight),
    texture=lambda path: ("texture", path),
  )

  stubs = {
    "pyray": raylib,
    "openpilot.common.constants": SimpleNamespace(
      CV=SimpleNamespace(MS_TO_KPH=3.6, MS_TO_MPH=2.2369362920544),
    ),
    "openpilot.selfdrive.ui.onroad.exp_button": SimpleNamespace(ExpButton=FakeExpButton),
    "openpilot.selfdrive.ui.ui_state": SimpleNamespace(
      ui_state=fake_ui_state,
      UIStatus=SimpleNamespace(ENGAGED=1, DISENGAGED=2, OVERRIDE=3),
    ),
    "openpilot.system.ui.lib.application": SimpleNamespace(
      gui_app=gui_app,
      FontWeight=SimpleNamespace(SEMI_BOLD=1, BOLD=2, MEDIUM=3, DISPLAY=4),
    ),
    "openpilot.system.ui.lib.multilang": SimpleNamespace(tr=lambda text: text),
    "openpilot.system.ui.lib.text_measure": SimpleNamespace(
      measure_text_cached=lambda font, text, size: Vector2(len(text) * size, size),
    ),
    "openpilot.system.ui.lib.text_draw": SimpleNamespace(draw_text_ui_style=lambda *args, **kwargs: None),
    "openpilot.system.ui.widgets": SimpleNamespace(Widget=Widget),
  }
  for name, stub in stubs.items():
    monkeypatch.setitem(sys.modules, name, stub)

  module_name = f"_test_carrot_hud_renderer_{id(fake_ui_state)}"
  spec = importlib.util.spec_from_file_location(module_name, HUD_RENDERER_PATH)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, module_name, module)
  spec.loader.exec_module(module)
  return module, fake_ui_state


class FakeParams:
  def __init__(self, values, failures=None):
    self.values = values
    self.failures = dict(failures or {})
    self.calls = []

  def get_int(self, key):
    self.calls.append(key)
    if self.failures.get(key, 0) > 0:
      self.failures[key] -= 1
      raise RuntimeError(f"failed to read {key}")
    return self.values[key]


def make_param_renderer(module, *, next_refresh=0.0):
  renderer = object.__new__(module.HudRenderer)
  renderer._hud_params_next_refresh_time = next_refresh
  renderer._show_device_state = 91
  renderer._show_date_time = 92
  renderer._show_plot_mode = 93
  renderer._longitudinal_personality = 94
  return renderer


def test_direct_hud_param_reads_are_isolated_to_refresh():
  tree = ast.parse(HUD_RENDERER_PATH.read_text(encoding="utf-8"))
  reads = []

  for function in (node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)):
    for call in (node for node in ast.walk(function) if isinstance(node, ast.Call)):
      func = call.func
      if (
        isinstance(func, ast.Attribute)
        and func.attr == "get_int"
        and isinstance(func.value, ast.Attribute)
        and func.value.attr == "params"
        and isinstance(func.value.value, ast.Name)
        and func.value.value.id == "ui_state"
      ):
        assert len(call.args) == 1 and isinstance(call.args[0], ast.Constant)
        reads.append((function.name, call.args[0].value))

  assert reads == [
    ("_refresh_hud_params", "ShowDeviceState"),
    ("_refresh_hud_params", "ShowDateTime"),
    ("_refresh_hud_params", "ShowPlotMode"),
    ("_refresh_hud_params", "LongitudinalPersonality"),
  ]


def test_hud_renderer_has_no_subsection_profiling():
  tree = ast.parse(HUD_RENDERER_PATH.read_text(encoding="utf-8"))

  assert not any(
    isinstance(node, ast.ClassDef) and node.name == "HudRenderTimings"
    for node in ast.walk(tree)
  )
  assert not any(
    isinstance(node, ast.Call)
    and isinstance(node.func, ast.Attribute)
    and node.func.attr == "monotonic_ns"
    for node in ast.walk(tree)
  )


def test_initial_cruise_gap_preserves_legacy_fallback(hud_module):
  module, _ = hud_module

  renderer = module.HudRenderer()

  assert renderer._longitudinal_personality == 7
  assert renderer._get_cruise_gap() == 8


def test_hud_params_are_refreshed_once_per_interval(hud_module):
  module, fake_ui_state = hud_module
  params = FakeParams({
    "ShowDeviceState": 1,
    "ShowDateTime": 2,
    "ShowPlotMode": 3,
    "LongitudinalPersonality": 4,
  })
  fake_ui_state.params = params
  renderer = make_param_renderer(module)

  renderer._refresh_hud_params(0.0)
  assert (
    renderer._show_device_state,
    renderer._show_date_time,
    renderer._show_plot_mode,
    renderer._longitudinal_personality,
  ) == (1, 2, 3, 4)
  assert renderer._hud_params_next_refresh_time == 1.0
  assert params.calls == [
    "ShowDeviceState",
    "ShowDateTime",
    "ShowPlotMode",
    "LongitudinalPersonality",
  ]

  renderer._refresh_hud_params(0.999)
  assert len(params.calls) == 4

  renderer._refresh_hud_params(1.0)
  assert len(params.calls) == 8
  assert renderer._hud_params_next_refresh_time == 2.0


def test_show_param_failure_keeps_complete_snapshot_and_retries(hud_module):
  module, fake_ui_state = hud_module
  params = FakeParams({
    "ShowDeviceState": 1,
    "ShowDateTime": 2,
    "ShowPlotMode": 3,
    "LongitudinalPersonality": 4,
  }, failures={"ShowDateTime": 1})
  fake_ui_state.params = params
  renderer = make_param_renderer(module)

  renderer._refresh_hud_params(0.0)

  assert (
    renderer._show_device_state,
    renderer._show_date_time,
    renderer._show_plot_mode,
    renderer._longitudinal_personality,
  ) == (91, 92, 93, 94)
  assert renderer._hud_params_next_refresh_time == 0.0
  assert params.calls == ["ShowDeviceState", "ShowDateTime"]

  renderer._refresh_hud_params(0.1)
  assert (
    renderer._show_device_state,
    renderer._show_date_time,
    renderer._show_plot_mode,
    renderer._longitudinal_personality,
  ) == (1, 2, 3, 4)
  assert renderer._hud_params_next_refresh_time == pytest.approx(1.1)


def test_personality_failure_uses_gap_eight_and_retries_next_frame(hud_module):
  module, fake_ui_state = hud_module
  params = FakeParams({
    "ShowDeviceState": 1,
    "ShowDateTime": 2,
    "ShowPlotMode": 3,
    "LongitudinalPersonality": 4,
  }, failures={"LongitudinalPersonality": 1})
  fake_ui_state.params = params
  renderer = make_param_renderer(module)

  renderer._refresh_hud_params(0.0)

  assert (renderer._show_device_state, renderer._show_date_time, renderer._show_plot_mode) == (1, 2, 3)
  assert renderer._get_cruise_gap() == 8
  assert renderer._hud_params_next_refresh_time == 0.0

  renderer._refresh_hud_params(0.1)
  assert renderer._get_cruise_gap() == 5
  assert renderer._hud_params_next_refresh_time == pytest.approx(1.1)
  assert params.calls.count("LongitudinalPersonality") == 2


def test_speed_limit_snapshot_reads_submaster_once(hud_module):
  module, fake_ui_state = hud_module
  carrot_man = SimpleNamespace(xSpdLimit="80", xSignType=2.0, nRoadLimitSpeed=90)

  class CountingSubMaster:
    def __init__(self):
      self.calls = 0

    def __getitem__(self, key):
      assert key == "carrotMan"
      self.calls += 1
      if self.calls > 1:
        raise AssertionError("carrotMan was read more than once")
      return carrot_man

  fake_ui_state.sm = CountingSubMaster()
  renderer = object.__new__(module.HudRenderer)

  assert renderer._get_speed_limit_info() == (80, 2, 90)
  assert fake_ui_state.sm.calls == 1


def test_speed_panel_reuses_one_snapshot(hud_module, monkeypatch):
  module, _ = hud_module
  renderer = object.__new__(module.HudRenderer)
  renderer._blink_timer = 15
  renderer._disp_timer = 63
  speed_limit_info = (80, 2, 90)
  snapshots = []
  calls = []

  monkeypatch.setattr(renderer, "_get_speed_limit_info", lambda: calls.append("snapshot") or speed_limit_info)
  monkeypatch.setattr(renderer, "_draw_carrot_main_background", lambda bx, by, info: (calls.append("background"), snapshots.append(info)))
  monkeypatch.setattr(renderer, "_draw_carrot_traffic_light", lambda bx, by: calls.append("traffic"))
  monkeypatch.setattr(renderer, "_draw_carrot_speed_panel", lambda bx, by: calls.append("speed"))
  monkeypatch.setattr(renderer, "_draw_carrot_lower_status", lambda bx, by: calls.append("status"))
  monkeypatch.setattr(renderer, "_draw_carrot_speed_limit_box", lambda bx, by, info: (calls.append("limit"), snapshots.append(info)))
  monkeypatch.setattr(renderer, "_draw_carrot_device_state", lambda bx, by: calls.append("device"))
  monkeypatch.setattr(renderer, "_draw_turn_info_hud", lambda rect: calls.append("navigation"))

  renderer._draw_set_speed_carrot(module.rl.Rectangle(10, 20, 1000, 600))

  assert calls == ["snapshot", "background", "traffic", "speed", "status", "limit", "device", "navigation"]
  assert snapshots[0] is speed_limit_info
  assert snapshots[1] is speed_limit_info
  assert renderer._blink_timer == 0
  assert renderer._disp_timer == 0


def test_device_info_updates_only_on_first_frame_or_new_service_frame(hud_module, monkeypatch):
  module, fake_ui_state = hud_module
  renderer = object.__new__(module.HudRenderer)
  renderer._device_info_loaded = False
  renderer._device_info_recv_frames = (-1, -1)
  renderer.is_cruise_set = True
  renderer.set_speed = 10
  renderer.speed = 20
  fake_ui_state.started_frame = 2
  fake_ui_state.sm = SimpleNamespace(
    recv_frame={"carState": 1, "deviceState": 10, "peripheralState": 20},
  )
  refreshes = []

  def update_device_info():
    refreshes.append(True)
    renderer._device_info_loaded = True

  monkeypatch.setattr(renderer, "_update_device_info", update_device_info)

  renderer._update_state()
  assert len(refreshes) == 1

  renderer._update_state()
  assert len(refreshes) == 1

  fake_ui_state.sm.recv_frame["deviceState"] = 11
  renderer._update_state()
  assert len(refreshes) == 2

  fake_ui_state.sm.recv_frame["peripheralState"] = 21
  renderer._update_state()
  assert len(refreshes) == 3


def test_device_display_cache_follows_service_receive_frames(hud_module):
  module, fake_ui_state = hud_module
  renderer = object.__new__(module.HudRenderer)
  renderer._device_info_loaded = False
  renderer._device_info_recv_frames = (-1, -1)
  renderer.is_cruise_set = True
  renderer.set_speed = 10
  renderer.speed = 20
  device_state = SimpleNamespace(
    freeSpacePercent=55.0,
    memoryUsagePercent=20,
    cpuTempC=[40.0, 42.0],
    cpuUsagePercent=[10.0, 20.0],
  )
  peripheral_state = SimpleNamespace(voltage=12_500)

  class FakeSubMaster(dict):
    pass

  sm = FakeSubMaster(deviceState=device_state, peripheralState=peripheral_state)
  sm.recv_frame = {"carState": 1, "deviceState": 10, "peripheralState": 20}
  fake_ui_state.started_frame = 2
  fake_ui_state.sm = sm

  renderer._update_state()
  assert renderer._device_info_recv_frames == (10, 20)
  assert (
    renderer._cpu_temp_text,
    renderer._memory_usage_text,
    renderer._disk_usage_text,
    renderer._voltage_text,
  ) == ("41°C", "20%", "45%", "12.5V")

  device_state.cpuTempC = [50.0]
  device_state.memoryUsagePercent = 30
  device_state.freeSpacePercent = 40.0
  peripheral_state.voltage = 13_250
  renderer._update_state()
  assert (
    renderer._cpu_temp_text,
    renderer._memory_usage_text,
    renderer._disk_usage_text,
    renderer._voltage_text,
  ) == ("41°C", "20%", "45%", "12.5V")

  sm.recv_frame["deviceState"] = 11
  renderer._update_state()
  assert renderer._device_info_recv_frames == (11, 20)
  assert (
    renderer._cpu_temp_text,
    renderer._memory_usage_text,
    renderer._disk_usage_text,
    renderer._voltage_text,
  ) == ("50°C", "30%", "60%", "13.2V")


def test_incomplete_device_info_is_retried(hud_module):
  module, fake_ui_state = hud_module
  renderer = object.__new__(module.HudRenderer)
  device_state = SimpleNamespace(
    freeSpacePercent=55.0,
    memoryUsagePercent=20,
    cpuUsagePercent=[10.0, 20.0],
  )
  peripheral_state = SimpleNamespace(voltage=12_500)
  fake_ui_state.sm = {
    "deviceState": device_state,
    "peripheralState": peripheral_state,
  }

  renderer._update_device_info()
  assert renderer._device_info_loaded is False

  device_state.cpuTempC = [40.0, 42.0]
  renderer._update_device_info()
  assert renderer._device_info_loaded is True
  assert renderer._cpu_temp == pytest.approx(41.0)
  assert renderer._cpu_usage == pytest.approx(15.0)
  assert renderer._voltage == pytest.approx(12.5)
  assert renderer._cpu_temp_text == "41°C"
  assert renderer._memory_usage_text == "20%"
  assert renderer._disk_usage_text == "45%"
  assert renderer._voltage_text == "12.5V"


def test_round_box_reuses_scratch_rectangle_without_changing_draw_geometry(hud_module, monkeypatch):
  module, _ = hud_module
  renderer = object.__new__(module.HudRenderer)
  renderer._round_box_rect = module.rl.Rectangle(0, 0, 0, 0)
  fill_color = module.rl.Color(1, 2, 3, 4)
  line_color = module.rl.Color(5, 6, 7, 8)
  calls = []

  def snapshot(kind, rect, *args):
    calls.append((kind, id(rect), (rect.x, rect.y, rect.width, rect.height), args))

  monkeypatch.setattr(module.rl, "draw_rectangle_rounded", lambda rect, *args: snapshot("fill", rect, *args))
  monkeypatch.setattr(module.rl, "draw_rectangle_rounded_lines_ex", lambda rect, *args: snapshot("line", rect, *args))

  renderer._draw_round_box(1, 2, 3, 4, fill_color, line_color, 0.25, 8, 2)
  renderer._draw_round_box(5, 6, 7, 8, fill_color, line_color, 0.5, 4, 3)

  assert [call[0] for call in calls] == ["fill", "line", "fill", "line"]
  assert len({call[1] for call in calls}) == 1
  assert [call[2] for call in calls] == [
    (1.0, 2.0, 3.0, 4.0),
    (1.0, 2.0, 3.0, 4.0),
    (5.0, 6.0, 7.0, 8.0),
    (5.0, 6.0, 7.0, 8.0),
  ]
  assert calls[0][3] == (0.25, 8, fill_color)
  assert calls[1][3] == (0.25, 8, 2.0, line_color)
  assert calls[2][3] == (0.5, 4, fill_color)
  assert calls[3][3] == (0.5, 4, 3.0, line_color)


@pytest.mark.parametrize(("value", "expected_text", "expected_color_name"), (
  (4.9, "  -", "WHITE_220"),
  (5.0, "5", "TPMS_LOW"),
  (5.1, "5", "TPMS_LOW"),
  (30.9, "31", "TPMS_LOW"),
  (31.0, "31", "WHITE_220"),
  (31.1, "31", "WHITE_220"),
  (59.9, "60", "WHITE_220"),
  (60.0, "60", "WHITE_220"),
  (60.1, "  -", "WHITE_220"),
))
def test_tpms_legacy_display_boundaries(hud_module, value, expected_text, expected_color_name):
  module, _ = hud_module
  renderer = object.__new__(module.HudRenderer)

  assert renderer._get_tpms_text(value) == expected_text
  assert renderer._get_tpms_color(value) == getattr(module.COLORS, expected_color_name)


def test_date_text_formats_only_when_minute_key_changes(hud_module, monkeypatch):
  module, _ = hud_module
  renderer = object.__new__(module.HudRenderer)
  renderer._show_date_time = 1
  renderer._date_time_minute_key = None
  renderer._date_time_text = ""
  renderer._date_text = ""
  renderer._font_display = object()
  moments = iter((
    stdlib_time.struct_time((2026, 7, 16, 12, 1, 1, 3, 197, 0)),
    stdlib_time.struct_time((2026, 7, 16, 12, 1, 59, 3, 197, 0)),
    stdlib_time.struct_time((2026, 7, 16, 12, 2, 0, 3, 197, 0)),
    stdlib_time.struct_time((2026, 7, 16, 13, 2, 0, 3, 197, 0)),
  ))
  localtime_calls = []
  strftime_calls = []
  draw_calls = []

  def fake_localtime():
    localtime_calls.append(True)
    return next(moments)

  def fake_strftime(fmt, now):
    strftime_calls.append((fmt, now.tm_hour, now.tm_min))
    return f"{fmt}:{now.tm_hour:02d}:{now.tm_min:02d}"

  monkeypatch.setattr(module.time, "localtime", fake_localtime)
  monkeypatch.setattr(module.time, "strftime", fake_strftime)
  monkeypatch.setattr(module, "draw_text_ui_style", lambda *args, **kwargs: draw_calls.append((args, kwargs)))
  rect = module.rl.Rectangle(0, 0, 1000, 600)

  for _ in range(4):
    renderer._draw_date_time(rect)

  assert len(localtime_calls) == 4
  assert len(strftime_calls) == 6
  assert strftime_calls == [
    ("%H:%M", 12, 1), ("%m-%d", 12, 1),
    ("%H:%M", 12, 2), ("%m-%d", 12, 2),
    ("%H:%M", 13, 2), ("%m-%d", 13, 2),
  ]
  assert len(draw_calls) == 8
  assert renderer._date_time_minute_key == (2026, 197, 13, 2, 0)
  assert renderer._date_text.endswith(f"({module.WEEKDAYS_KO[4]})")

  renderer._show_date_time = 0
  monkeypatch.setattr(module.time, "localtime", lambda: pytest.fail("hidden date HUD read the clock"))
  renderer._draw_date_time(rect)
  assert len(draw_calls) == 8


def test_render_draws_each_hud_section_in_order(hud_module, monkeypatch):
  module, _ = hud_module
  renderer = object.__new__(module.HudRenderer)
  renderer.is_cruise_available = False
  renderer._show_plot_mode = 6
  renderer._font_display = object()
  calls = []

  renderer._exp_button = SimpleNamespace(render=lambda rect: calls.append("button"))
  renderer._plot_renderer = SimpleNamespace(
    draw=lambda rect, font, mode: calls.append(("plot", mode)),
  )
  monkeypatch.setattr(renderer, "_refresh_hud_params", lambda now: calls.append(("params", now)))
  monkeypatch.setattr(renderer, "_draw_date_time", lambda rect: calls.append("date"))
  monkeypatch.setattr(renderer, "_draw_tpms_top_right", lambda rect: calls.append("tpms"))
  monkeypatch.setattr(renderer, "_draw_cruise_speed_animation", lambda rect: calls.append("animation"))
  monkeypatch.setattr(module.rl, "draw_rectangle_gradient_v", lambda *args: calls.append("header"))
  monkeypatch.setattr(module.time, "monotonic", lambda: 12.5)

  renderer._render(module.rl.Rectangle(0, 0, 1000, 600))

  assert calls == [
    ("params", 12.5),
    "header",
    "button",
    ("plot", 6),
    "date",
    "tpms",
    "animation",
  ]
