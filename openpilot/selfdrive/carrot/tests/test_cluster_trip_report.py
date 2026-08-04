from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_renderer
from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_PANEL_LAYOUT_DRIVING_LEFT,
  CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
  CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
  CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
  CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
  CLUSTER_SCREEN_MODE_NAVI,
  CLUSTER_SCREEN_MODE_TRIP_REPORT,
  normalize_cluster_screen_mode,
)
from cluster_route_replay import RouteLogParser, frame_to_state
from cluster_trip_report import TripReportTracker
from cluster_renderer import (
  CAMERA_BACKGROUND_W,
  ClusterUiRenderer,
  NAVI_LIVE_PANEL_W,
  NAVI_WORLD_VIEW_SHIFT_X,
  TRIP_REPORT_PANEL_X,
)


def car_state(speed_mps: float, accel_mps2: float = 0.0, steering_angle_deg: float = 0.0) -> SimpleNamespace:
  return SimpleNamespace(
    vEgo=speed_mps,
    aEgo=accel_mps2,
    steeringAngleDeg=steering_angle_deg,
  )


def test_screen_mode_five_is_the_trip_report():
  assert CLUSTER_SCREEN_MODE_TRIP_REPORT == 5
  assert normalize_cluster_screen_mode(5) == CLUSTER_SCREEN_MODE_TRIP_REPORT
  assert normalize_cluster_screen_mode("trip-report") == CLUSTER_SCREEN_MODE_TRIP_REPORT
  assert normalize_cluster_screen_mode("report") == CLUSTER_SCREEN_MODE_TRIP_REPORT


def test_trip_tracker_counts_events_once_and_accumulates_summary():
  tracker = TripReportTracker()
  tracker.update(0.0, 30.0, 0.0, 0.0, False, 2.7, 15.0)
  for index in range(1, 2_601):
    accel = 3.0 if index in (2, 30) else -3.5 if index in (10, 40) else 0.0
    steering = 120.0 if index in (20, 50) else 0.0
    state = tracker.update(index * 0.5, 30.0, accel, steering, index >= 1_301, 2.7, 15.0)

  assert state.hard_accel_count == 2
  assert state.hard_brake_count == 2
  assert state.hard_corner_count == 2
  assert 45.0 <= state.auto_ratio_percent <= 55.0
  assert state.distance_m > 38_000.0


def test_trip_tracker_starts_only_onroad_stops_immediately_and_resets_next_trip():
  tracker = TripReportTracker()
  tracker.set_onroad(False)

  before_onroad = tracker.update(0.0, 10.0, 3.0, 120.0, True, 2.7, 15.0)
  assert before_onroad.distance_m == 0.0
  assert before_onroad.duration_s == 0.0
  assert before_onroad.hard_accel_count == 0

  tracker.set_onroad(True)
  tracker.update(0.0, 10.0, 0.0, 0.0, False, 2.7, 15.0)
  onroad = tracker.update(0.5, 10.0, 3.0, 120.0, True, 2.7, 15.0)
  assert onroad.distance_m == 5.0
  assert onroad.duration_s == 0.5

  stopped = tracker.set_onroad(False)
  after_offroad_sample = tracker.update(10.0, 30.0, -4.0, 120.0, True, 2.7, 15.0)
  assert stopped == onroad
  assert after_offroad_sample == onroad

  restarted = tracker.set_onroad(True)
  first_sample = tracker.update(20.0, 20.0, 0.0, 0.0, True, 2.7, 15.0)
  second_sample = tracker.update(20.5, 20.0, 0.0, 0.0, True, 2.7, 15.0)
  assert restarted.distance_m == 0.0
  assert restarted.duration_s == 0.0
  assert first_sample == restarted
  assert second_sample.distance_m == 10.0
  assert second_sample.duration_s == 0.5

def test_route_parser_carries_report_into_cluster_state():
  parser = RouteLogParser(recompute_cutins=False)
  parser._frame_from_car_state(car_state(10.0), 0.0)
  frame = parser._frame_from_car_state(car_state(10.0), 0.5)
  state = frame_to_state(frame)

  assert state.trip_report is not None
  assert state.trip_report.distance_m == 5.0
  assert state.trip_report.average_speed_kph == 36.0


def test_report_mode_reserves_the_right_panel_for_non_camera_world_view():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.screen_mode = CLUSTER_SCREEN_MODE_TRIP_REPORT

  assert renderer._world_view_shift_x(SimpleNamespace(camera_view_mode=0)) == NAVI_WORLD_VIEW_SHIFT_X
  assert renderer._world_view_shift_x(SimpleNamespace(camera_view_mode=2)) == 0.0


def test_swapped_panel_layout_moves_driving_and_information_regions_as_whole_panels():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = CLUSTER_SCREEN_MODE_TRIP_REPORT
  renderer.panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT

  camera_rect = renderer._camera_overlay_content_rect()
  assert renderer._driving_panel_offset_design_x() == NAVI_LIVE_PANEL_W
  assert renderer._information_panel_x(TRIP_REPORT_PANEL_X) == 0.0
  assert camera_rect.x == NAVI_LIVE_PANEL_W
  assert camera_rect.x + camera_rect.width == NAVI_LIVE_PANEL_W + CAMERA_BACKGROUND_W

  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEBUG_GRAPH
  assert renderer._driving_panel_offset_design_x() == 0.0
  assert renderer._information_panel_x(TRIP_REPORT_PANEL_X) == TRIP_REPORT_PANEL_X


def test_default_screen_uses_trip_report_until_navigation_is_received():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEFAULT
  disconnected = SimpleNamespace(
    camera_view_mode=0,
    external_nav_active=False,
    navi_live=None,
    navi_dashboard=SimpleNamespace(connected=False),
  )

  assert renderer._effective_screen_mode(disconnected) == CLUSTER_SCREEN_MODE_TRIP_REPORT
  assert renderer._world_view_shift_x(disconnected) == NAVI_WORLD_VIEW_SHIFT_X

  connected = SimpleNamespace(
    camera_view_mode=0,
    external_nav_active=False,
    navi_live=None,
    navi_dashboard=SimpleNamespace(connected=True),
  )
  assert renderer._effective_screen_mode(connected) == CLUSTER_SCREEN_MODE_DEFAULT

  external_navigation = SimpleNamespace(
    camera_view_mode=0,
    external_nav_active=True,
    navi_live=None,
    navi_dashboard=None,
  )
  assert renderer._effective_screen_mode(external_navigation) == CLUSTER_SCREEN_MODE_DEFAULT


def test_default_screen_shows_trip_report_in_park_and_restores_navigation_in_drive():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEFAULT
  state = SimpleNamespace(
    camera_view_mode=0,
    onroad=True,
    gear_text="P",
    external_nav_active=True,
    navi_live=None,
    navi_dashboard=SimpleNamespace(connected=True),
  )

  assert renderer._effective_screen_mode(state) == CLUSTER_SCREEN_MODE_TRIP_REPORT

  state.gear_text = "D"
  assert renderer._effective_screen_mode(state) == CLUSTER_SCREEN_MODE_DEFAULT

  renderer.screen_mode = CLUSTER_SCREEN_MODE_NAVI
  state.gear_text = "P"
  assert renderer._effective_screen_mode(state) == CLUSTER_SCREEN_MODE_NAVI


def test_mode_two_preserves_the_reference_default_system_screen_contract():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEBUG_SYSTEM
  disconnected = SimpleNamespace(
    camera_view_mode=0,
    external_nav_active=False,
    navi_live=None,
    navi_dashboard=SimpleNamespace(connected=False),
  )

  assert renderer._effective_screen_mode(disconnected) == CLUSTER_SCREEN_MODE_DEFAULT
  assert renderer._world_view_shift_x(disconnected) == NAVI_WORLD_VIEW_SHIFT_X

  no_navigation_source = SimpleNamespace(
    camera_view_mode=0,
    external_nav_active=False,
    navi_live=None,
    navi_dashboard=None,
  )
  assert renderer._effective_screen_mode(no_navigation_source) == CLUSTER_SCREEN_MODE_DEFAULT
  assert renderer._world_view_shift_x(no_navigation_source) == 0.0


def test_mode_two_dispatches_reference_default_system_content(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEBUG_SYSTEM
  calls = []
  route_overlay = object()

  monkeypatch.setattr(cluster_renderer.rl, "rl_push_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_scalef", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_pop_matrix", lambda: None)
  monkeypatch.setattr(renderer, "_profile_start", lambda: 0.0)
  monkeypatch.setattr(renderer, "_profile_add", lambda *_args: None)
  monkeypatch.setattr(
    renderer,
    "_draw_driving_hud_content",
    lambda _state, mode, *_signals: calls.append(("driving", mode)),
  )
  monkeypatch.setattr(renderer, "_draw_navi_live_panel", lambda _state: calls.append(("navi", None)))
  monkeypatch.setattr(renderer, "_draw_route_overlay", lambda overlay: calls.append(("route", overlay)))
  monkeypatch.setattr(
    renderer,
    "_draw_status_footer",
    lambda _state, **kwargs: calls.append(("footer", kwargs.get("include_core_usage"))),
  )

  no_navigation_source = SimpleNamespace(
    navi_debug=None,
    navi_live=None,
    navi_dashboard=None,
    route_overlay=route_overlay,
  )
  renderer._draw_hud(no_navigation_source, (False, False))
  assert calls == [
    ("driving", CLUSTER_SCREEN_MODE_DEFAULT),
    ("route", route_overlay),
    ("footer", True),
  ]

  calls.clear()
  disconnected_dashboard = SimpleNamespace(
    navi_debug=None,
    navi_live=None,
    navi_dashboard=SimpleNamespace(connected=False),
    route_overlay=route_overlay,
  )
  renderer._draw_hud(disconnected_dashboard, (False, False))
  assert calls == [
    ("driving", CLUSTER_SCREEN_MODE_DEFAULT),
    ("navi", None),
    ("footer", True),
  ]


def test_fullscreen_3d_mode_falls_back_to_mode_zero_outside_3d_views():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.screen_mode = CLUSTER_SCREEN_MODE_FULLSCREEN_3D

  for camera_view_mode in (0, 1):
    state = SimpleNamespace(camera_view_mode=camera_view_mode)
    assert renderer._effective_screen_mode(state) == CLUSTER_SCREEN_MODE_FULLSCREEN_3D
    assert renderer._world_view_shift_x(state) == 0.0
    assert renderer._turn_signal_center_x_offset(state, "left") == 0.0
    assert renderer._turn_signal_center_x_offset(state, "right") == 0.0

  no_navigation = SimpleNamespace(
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    external_nav_active=False,
    navi_live=None,
    navi_dashboard=None,
  )
  assert renderer._effective_screen_mode(no_navigation) == CLUSTER_SCREEN_MODE_TRIP_REPORT

  connected_navigation = SimpleNamespace(
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    external_nav_active=False,
    navi_live=None,
    navi_dashboard=SimpleNamespace(connected=True),
  )
  assert renderer._effective_screen_mode(connected_navigation) == CLUSTER_SCREEN_MODE_DEFAULT


def test_fullscreen_3d_uses_full_width_hud_layout_even_when_panels_are_swapped():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = CLUSTER_SCREEN_MODE_FULLSCREEN_3D
  renderer.panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT

  fullscreen_offset_x = renderer._side_gauge_offset_design_x(CLUSTER_SCREEN_MODE_FULLSCREEN_3D)
  assert renderer._driving_hud_offset_design_x(CLUSTER_SCREEN_MODE_FULLSCREEN_3D) == 0.0
  assert fullscreen_offset_x == 796.0
  assert renderer._tpms_offset_design_x(CLUSTER_SCREEN_MODE_FULLSCREEN_3D) == fullscreen_offset_x
  assert renderer._center_clock_x(CLUSTER_SCREEN_MODE_FULLSCREEN_3D) == 960.0
  assert renderer._traffic_panel_right(CLUSTER_SCREEN_MODE_FULLSCREEN_3D) == 1892.0
  assert renderer._core_usage_right_x(CLUSTER_SCREEN_MODE_FULLSCREEN_3D) == 1788.0
  assert (
    cluster_renderer.DESIGN_WIDTH
    - (cluster_renderer.SIDE_GAUGE_LEFT_CENTER_X + fullscreen_offset_x)
    == cluster_renderer.CAMERA_BACKGROUND_W - cluster_renderer.SIDE_GAUGE_LEFT_CENTER_X
  )
  assert (
    cluster_renderer.DESIGN_WIDTH
    - (cluster_renderer.TPMS_STATUS_CENTER_X + fullscreen_offset_x)
    == cluster_renderer.CAMERA_BACKGROUND_W - cluster_renderer.TPMS_STATUS_CENTER_X
  )

  road_camera_rect = renderer._camera_overlay_content_rect()
  assert renderer._driving_hud_offset_design_x(CLUSTER_SCREEN_MODE_TRIP_REPORT) == 792.0
  assert road_camera_rect.x == 792.0
  assert road_camera_rect.width == CAMERA_BACKGROUND_W


def test_graph_right_mode_places_side_gauges_next_to_graph_in_both_panel_layouts():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT

  for panel_layout in (CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT):
    renderer.panel_layout = panel_layout
    driving_offset_x = renderer._driving_hud_offset_design_x(renderer.screen_mode)
    gauge_offset_x = renderer._side_gauge_offset_design_x(renderer.screen_mode)
    plot_x = renderer._information_panel_x(cluster_renderer.DEBUG_PLOT_RIGHT_X)
    first_center_x = cluster_renderer.SIDE_GAUGE_LEFT_CENTER_X + driving_offset_x + gauge_offset_x
    second_center_x = first_center_x + cluster_renderer.SIDE_GAUGE_COLUMN_GAP
    gauge_half_width = cluster_renderer.SIDE_GAUGE_WIDTH * 0.5
    graph_gap = cluster_renderer.DEBUG_PLOT_SIDE_GAUGE_GAP

    assert second_center_x + gauge_half_width + graph_gap == plot_x

    assert renderer._tpms_offset_design_x(renderer.screen_mode) == 0.0


def test_fullscreen_3d_repositions_hud_widgets_and_suppresses_information_panels(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = CLUSTER_SCREEN_MODE_FULLSCREEN_3D
  renderer.panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  calls = []
  translations = []

  monkeypatch.setattr(cluster_renderer.rl, "rl_push_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_scalef", lambda *_args: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_pop_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_translatef", lambda *args: translations.append(args))
  monkeypatch.setattr(renderer, "_profile_start", lambda: 0.0)
  monkeypatch.setattr(renderer, "_profile_add", lambda *_args: None)
  monkeypatch.setattr(
    renderer,
    "_draw_speed_block",
    lambda _state, **kwargs: calls.append(("speed", kwargs["tpms_offset_x"])),
  )
  monkeypatch.setattr(renderer, "_draw_accel_block", lambda _state: calls.append(("accel", None)))
  monkeypatch.setattr(renderer, "_draw_steering_output_block", lambda _state: calls.append(("steer", None)))
  monkeypatch.setattr(renderer, "_draw_turn_signal", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(renderer, "_draw_drive_status", lambda _state: None)
  monkeypatch.setattr(
    renderer,
    "_draw_center_clock",
    lambda _state, **kwargs: calls.append(("clock", kwargs["center_x"])),
  )

  hud_state = SimpleNamespace(navi_dashboard=None, debug_ui_visible=False)
  renderer._draw_driving_hud_content(
    hud_state,
    CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
    False,
    False,
  )
  assert calls == [("speed", 796), ("accel", None), ("steer", None), ("clock", 960)]
  assert translations == [(796, 0.0, 0.0)]

  calls.clear()
  state = SimpleNamespace(
    camera_view_mode=0,
    navi_debug=object(),
    navi_live=None,
    navi_dashboard=None,
  )
  monkeypatch.setattr(
    renderer,
    "_draw_driving_hud_content",
    lambda _state, mode, *_signals: calls.append(("driving", mode)),
  )
  monkeypatch.setattr(renderer, "_draw_status_footer", lambda _state, **_kwargs: calls.append(("footer", None)))
  monkeypatch.setattr(renderer, "_draw_navi_debug_panel", lambda _state: calls.append(("navi-debug", None)))
  monkeypatch.setattr(renderer, "_draw_route_overlay", lambda _overlay: calls.append(("route", None)))
  renderer._draw_hud(state, (False, False))
  assert calls == [("driving", CLUSTER_SCREEN_MODE_FULLSCREEN_3D), ("footer", None)]


def test_trip_report_footer_keeps_left_status_without_right_core_usage(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  calls = []
  monkeypatch.setattr(renderer, "_profile_start", lambda: 0.0)
  monkeypatch.setattr(renderer, "_profile_add", lambda *_args: None)
  monkeypatch.setattr(renderer, "_draw_git_status", lambda *args: calls.append(("left", args)))
  monkeypatch.setattr(renderer, "_draw_cluster_core_usage", lambda text: calls.append(("right", text)))
  state = SimpleNamespace(
    git_status=SimpleNamespace(branch="carrot-wip"),
    network_address="192.168.0.10",
    actual_fps=20.0,
    cluster_core_usage_text="CPU 12%",
  )

  renderer._draw_status_footer(state, include_core_usage=False)

  assert [name for name, _value in calls] == ["left"]
