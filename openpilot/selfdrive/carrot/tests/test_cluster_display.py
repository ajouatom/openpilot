from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_renderer
from cluster_config import DARK_CLUSTER_THEME, LIGHT_CLUSTER_THEME
from cluster_display import (
  cluster_text,
  display_speed,
  format_navi_distance,
  format_radar_distance,
  format_trip_distance,
  normalize_cluster_language,
  normalize_metric_setting,
  speed_unit,
)
from cluster_renderer import ClusterUiRenderer
from main import ClusterClockVisibilityParamReader, ClusterDisplayPreferencesParamReader


@pytest.mark.parametrize(
  ("value", "expected"),
  (
    ("ko", "ko"),
    ("main_ko", "ko"),
    ("kr", "ko"),
    (b"en", "en"),
    ("main_en", "en"),
    ("zh-CHT", "en"),
    ("unknown", "en"),
  ),
)
def test_cluster_language_supports_korean_and_english_only(value, expected):
  assert normalize_cluster_language(value) == expected


def test_metric_and_imperial_display_conversions():
  assert normalize_metric_setting(None) is True
  assert normalize_metric_setting(b"1") is True
  assert normalize_metric_setting("0") is False
  assert display_speed(100.0, True) == 100.0
  assert display_speed(100.0, False) == pytest.approx(62.1371192)
  assert speed_unit(True) == "km/h"
  assert speed_unit(False) == "mph"
  assert format_radar_distance(100.0, True) == "100 m"
  assert format_radar_distance(100.0, False) == "328 ft"
  assert format_navi_distance(50.0, False) == "164 ft"
  assert format_navi_distance(1609.344, False) == "1.0 mi"
  assert format_trip_distance(1609.344, False) == "1.00 mi"


def test_cluster_text_falls_back_to_english_for_unsupported_languages():
  assert cluster_text("ko", "driving_report") == "주행리포트"
  assert cluster_text("en", "driving_report") == "DRIVING REPORT"
  assert cluster_text("zh-CHT", "driving_report") == "DRIVING REPORT"


def test_display_preference_reader_uses_param_defaults_and_values():
  class FakeParams:
    def __init__(self, values):
      self.values = values

    def get(self, name, return_default=False):
      assert return_default
      return self.values.get(name)

  reader = object.__new__(ClusterDisplayPreferencesParamReader)
  reader._params = FakeParams({"LanguageSetting": "main_ko", "IsMetric": b"0"})
  assert reader.read() == ("ko", False)

  reader._params = FakeParams({})
  assert reader.read() == ("ko", True)


@pytest.mark.parametrize(
  ("show_date_time", "expected"),
  ((0, False), (1, True), (2, True), (3, False)),
)
def test_cluster_clock_visibility_follows_show_date_time(show_date_time, expected):
  class FakeParams:
    def get_int(self, name):
      assert name == "ShowDateTime"
      return show_date_time

  reader = object.__new__(ClusterClockVisibilityParamReader)
  reader._params = FakeParams()

  assert reader.read() is expected


def test_cluster_clock_visibility_fails_visible():
  class FailingParams:
    def get_int(self, _name):
      raise RuntimeError("read failed")

  reader = object.__new__(ClusterClockVisibilityParamReader)
  reader._params = FailingParams()

  assert reader.read() is True


def test_trip_report_draws_english_imperial_labels(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.language = "en"
  renderer.is_metric = False
  renderer._current_theme = lambda: DARK_CLUSTER_THEME
  renderer._system_stats = SimpleNamespace(sample=lambda: SimpleNamespace(
    cpu_used_percent=20.0,
    memory_used_percent=30.0,
    disk_used_percent=40.0,
  ))
  text_draws = []
  metrics = []
  gauges = []
  angles = []
  renderer._rounded_rect = lambda *_args, **_kwargs: None
  renderer._draw_text = lambda *args, **kwargs: text_draws.append((args, kwargs))
  renderer._draw_trip_metric = lambda *args, **kwargs: metrics.append((args, kwargs))
  renderer._draw_system_gauge = lambda *args, **kwargs: gauges.append((args, kwargs))
  renderer._draw_device_angle_indicator = lambda *args, **kwargs: angles.append((args, kwargs))
  renderer._system_metric_color = lambda _value: (1, 2, 3)
  monkeypatch.setattr(cluster_renderer.rl, "draw_line_ex", lambda *_args, **_kwargs: None)

  report = SimpleNamespace(
    duration_s=65.0,
    distance_m=1609.344,
    average_speed_kph=100.0,
    max_speed_kph=120.0,
    auto_ratio_percent=95.0,
    max_accel_mps2=1.23,
    max_decel_mps2=-2.34,
    hard_accel_count=1,
    hard_brake_count=2,
    hard_corner_count=3,
  )
  state = SimpleNamespace(
    trip_report=report,
    cpu_usage_percent=25.0,
    memory_used_percent=35.0,
    disk_used_percent=45.0,
    cpu_temp_c=62.0,
    camera_calibration_euler=(0.0, 0.01, -0.02),
  )

  renderer._draw_trip_report_panel(state)

  drawn_text = {args[0] for args, _kwargs in text_draws}
  assert {"TRIP SUMMARY", "SYSTEM", "DEVICE ANGLE"} <= drawn_text
  assert "DRIVING REPORT" not in drawn_text
  assert {"HARD ACCEL", "HARD BRAKE", "HARD CORNER"} <= drawn_text
  assert metrics[0][0][2:4] == ("DISTANCE", "1.00 mi")
  assert metrics[1][0][2:4] == ("AVG SPEED", "62.1")
  assert metrics[1][0][5] == "mph"
  assert metrics[2][0][2:4] == ("MAX SPEED", "74.6")
  assert metrics[2][0][5] == "mph"
  assert [gauge[0][2] for gauge in gauges] == ["CPU", "TEMP", "MEM", "DISK"]
  assert angles[0][0][4:6] == pytest.approx((0.5729578, -1.1459156))


@pytest.mark.parametrize("theme", (LIGHT_CLUSTER_THEME, DARK_CLUSTER_THEME), ids=("light", "dark"))
def test_trip_report_follows_current_cluster_theme(monkeypatch, theme):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.language = "en"
  renderer.is_metric = True
  renderer._current_theme = lambda: theme
  renderer._system_stats = SimpleNamespace(sample=lambda: SimpleNamespace())
  panels = []
  text_draws = []
  health_cards = []
  renderer._rounded_rect = lambda *args, **_kwargs: panels.append(args)
  renderer._draw_text = lambda *args, **kwargs: text_draws.append((args, kwargs))
  renderer._draw_system_health_card = lambda *args, **kwargs: health_cards.append((args, kwargs))
  monkeypatch.setattr(cluster_renderer.rl, "draw_line_ex", lambda *_args, **_kwargs: None)

  report = SimpleNamespace(
    duration_s=0.0,
    distance_m=0.0,
    average_speed_kph=0.0,
    max_speed_kph=0.0,
    auto_ratio_percent=0.0,
    max_accel_mps2=0.0,
    max_decel_mps2=0.0,
    hard_accel_count=0,
    hard_brake_count=0,
    hard_corner_count=0,
  )
  renderer._draw_trip_report_panel(SimpleNamespace(trip_report=report))

  assert panels[0][5:7] == (theme.panel_bg, theme.faint)
  assert panels[1][5:7] == (theme.route_panel_bg, theme.faint)
  assert all(panel[5] == theme.panel_bg for panel in panels[2:5])
  assert health_cards[0][1] == {
    "panel_bg": theme.route_panel_bg,
    "panel_outline": theme.faint,
    "text_color": theme.text,
    "muted_color": theme.muted,
    "target_fill": theme.panel_bg,
  }

  text_draws.clear()
  renderer._draw_trip_metric(10.0, 20.0, "LABEL", "VALUE", 100.0, "UNIT")
  assert [args[4] for args, _kwargs in text_draws] == [theme.muted, theme.text, theme.muted]
  assert renderer._trip_temp_color(None, theme.muted) == theme.muted

  angle_lines = []
  monkeypatch.setattr(cluster_renderer, "rl_color", lambda color: color)
  monkeypatch.setattr(cluster_renderer.rl, "draw_line_ex", lambda *args, **_kwargs: angle_lines.append(args))
  monkeypatch.setattr(cluster_renderer.rl, "draw_circle_v", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(cluster_renderer.rl, "draw_ring", lambda *_args, **_kwargs: None)
  renderer._draw_device_angle_indicator(
    0.0,
    0.0,
    100.0,
    100.0,
    None,
    None,
    theme.muted,
    theme.faint,
    text_color=theme.text,
    target_fill=theme.panel_bg,
  )
  assert [line[3] for line in angle_lines] == [theme.faint, theme.faint]


def test_trip_report_cache_refreshes_once_per_second_and_on_context_changes():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = cluster_renderer.DESIGN_WIDTH
  renderer.height = cluster_renderer.DESIGN_HEIGHT
  renderer.language = "ko"
  renderer.is_metric = True
  renderer.panel_layout = cluster_renderer.CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
  renderer._trip_report_target = None
  renderer._trip_report_cache_key = None
  renderer._trip_report_cache_valid = False
  renderer._trip_report_cache_visible = False
  renderer._trip_report_cache_next_refresh = 0.0
  renderer._profile_start = lambda: None
  renderer._profile_add = lambda *_args: None

  current_theme = [LIGHT_CLUSTER_THEME]
  renderer._current_theme = lambda: current_theme[0]
  renderer._effective_screen_mode = lambda state: state.mode
  refreshes = []

  def refresh(state):
    refreshes.append(state.mode)
    renderer._trip_report_target = SimpleNamespace()

  renderer._refresh_trip_report_cache = refresh
  state = SimpleNamespace(mode=cluster_renderer.CLUSTER_SCREEN_MODE_TRIP_REPORT)

  renderer._prepare_trip_report_cache(state, now=10.0)
  renderer._prepare_trip_report_cache(state, now=10.9)
  assert len(refreshes) == 1

  current_theme[0] = DARK_CLUSTER_THEME
  renderer._prepare_trip_report_cache(state, now=10.9)
  assert len(refreshes) == 2

  state.mode = cluster_renderer.CLUSTER_SCREEN_MODE_DEFAULT
  renderer._prepare_trip_report_cache(state, now=10.95)
  state.mode = cluster_renderer.CLUSTER_SCREEN_MODE_TRIP_REPORT
  renderer._prepare_trip_report_cache(state, now=10.96)
  assert len(refreshes) == 3


def test_trip_report_cache_draws_one_texture_instead_of_panel_contents(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.panel_layout = cluster_renderer.CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
  renderer.screen_mode = cluster_renderer.CLUSTER_SCREEN_MODE_TRIP_REPORT
  texture = SimpleNamespace(width=792, height=478)
  renderer._trip_report_target = SimpleNamespace(texture=texture)
  renderer._trip_report_cache_valid = True
  renderer._draw_trip_report_panel_contents = lambda _state: pytest.fail("cached panel was redrawn")
  draws = []
  monkeypatch.setattr(cluster_renderer, "rl_color", lambda color: color)
  monkeypatch.setattr(cluster_renderer.rl, "draw_texture_pro", lambda *args: draws.append(args))

  renderer._draw_trip_report_panel(SimpleNamespace())

  assert len(draws) == 1
  assert draws[0][0] is texture
  assert draws[0][1].height == -478.0
  assert (draws[0][2].x, draws[0][2].y) == (1124.0, 1.0)
  assert (draws[0][2].width, draws[0][2].height) == (792.0, 478.0)


@pytest.mark.parametrize(
  ("panel_layout", "expected_panel_x"),
  (
    (cluster_renderer.CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, 1124.0),
    (cluster_renderer.CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT, 0.0),
  ),
)
def test_mode_two_system_dashboard_combines_runtime_detail_and_health_cards(panel_layout, expected_panel_x):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.panel_layout = panel_layout
  stats = SimpleNamespace()
  samples = []
  renderer._system_stats = SimpleNamespace(sample=lambda: samples.append(stats) or stats)
  renderer._current_theme = lambda: SimpleNamespace(panel_bg=(1, 2, 3), faint=(4, 5, 6))
  outer_panels = []
  detail = {}
  health = {}

  renderer._rounded_rect = lambda *args, **_kwargs: outer_panels.append(args)

  def capture_detail(panel_state, **kwargs):
    detail["state"] = panel_state
    detail.update(kwargs)

  def capture_health(panel_state, panel_stats, *bounds, **kwargs):
    health["state"] = panel_state
    health["stats"] = panel_stats
    health["bounds"] = bounds
    health["kwargs"] = kwargs

  renderer._draw_system_stats_panel = capture_detail
  renderer._draw_system_health_card = capture_health
  state = SimpleNamespace()

  renderer._draw_system_dashboard_panel(state)

  assert samples == [stats]
  assert outer_panels[0][:4] == (expected_panel_x, 1, 792, 478)
  assert detail == {
    "state": state,
    "panel_x": expected_panel_x + 16.0,
    "panel_y": 9.0,
    "panel_w": 474.0,
    "panel_h": 462.0,
    "show_runtime_info": True,
    "title_text": "DETAIL",
    "stats": stats,
  }
  assert health == {
    "state": state,
    "stats": stats,
    "bounds": (expected_panel_x + 500.0, 9.0, 276.0, 462.0),
    "kwargs": {},
  }


@pytest.mark.parametrize(
  ("panel_layout", "expected_panel_x"),
  (
    (cluster_renderer.CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, 1124.0),
    (cluster_renderer.CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT, 0.0),
  ),
)
def test_mode_one_live_debug_uses_full_information_region_with_four_cards(panel_layout, expected_panel_x):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.panel_layout = panel_layout
  renderer._current_theme = lambda: SimpleNamespace(
    panel_bg=(1, 2, 3),
    route_panel_bg=(4, 5, 6),
    faint=(7, 8, 9),
    muted=(10, 11, 12),
    text=(13, 14, 15),
  )
  panels = []
  renderer._rounded_rect = lambda *args, **_kwargs: panels.append(args)
  renderer._draw_text = lambda *_args, **_kwargs: None
  renderer._ellipsize_text = lambda value, *_args: value
  state = SimpleNamespace(live_debug=None, lateral_plan_debug_text=None)

  sections = renderer._live_debug_sections(state)
  renderer._draw_live_debug_panel(state)

  assert [title for title, _rows in sections] == ["LIVE DELAY", "LIVE TORQUE", "STEERING", "LATERAL PLAN"]
  assert sections[0][1][0][1] == "--% / --"
  assert sections[3][1][0][1] == "--"
  assert panels[0][:4] == (expected_panel_x, 1, 792, 478)
  assert [panel[:4] for panel in panels[1:]] == [
    (expected_panel_x + 16.0, 9.0, 375.0, 226.0),
    (expected_panel_x + 401.0, 9.0, 375.0, 226.0),
    (expected_panel_x + 16.0, 245.0, 375.0, 226.0),
    (expected_panel_x + 401.0, 245.0, 375.0, 226.0),
  ]


def test_device_angle_target_maps_pitch_and_yaw_to_expected_axes():
  radius = 30.0

  assert ClusterUiRenderer._device_angle_target_offset(3.0, 2.0, radius) == pytest.approx((-6.8, 10.2))
  assert ClusterUiRenderer._device_angle_target_offset(-3.0, -2.0, radius) == pytest.approx((6.8, -10.2))
  assert ClusterUiRenderer._device_angle_target_offset(20.0, -20.0, radius) == pytest.approx((20.4, 20.4))
