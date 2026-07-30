from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import (
  CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
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

  legacy_navigation = SimpleNamespace(
    camera_view_mode=0,
    external_nav_active=True,
    navi_live=None,
    navi_dashboard=None,
  )
  assert renderer._effective_screen_mode(legacy_navigation) == CLUSTER_SCREEN_MODE_DEFAULT


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
