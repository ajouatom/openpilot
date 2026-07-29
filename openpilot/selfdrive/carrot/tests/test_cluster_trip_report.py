from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import (
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_TRIP_REPORT,
  normalize_cluster_screen_mode,
)
from cluster_route_replay import RouteLogParser, frame_to_state
from cluster_trip_report import (
  TRIP_TRACE_MAX_RADIUS_M,
  TRIP_TRACE_RADIUS_QUANTUM_M,
  TripReportTracker,
)
from cluster_renderer import ClusterUiRenderer, NAVI_WORLD_VIEW_SHIFT_X


EARTH_RADIUS_M = 6_378_137.0


def live_pose(yaw_rad: float) -> SimpleNamespace:
  return SimpleNamespace(
    inputsOK=True,
    sensorsOK=True,
    orientationNED=SimpleNamespace(valid=True, z=yaw_rad, zStd=0.02),
    angularVelocityDevice=SimpleNamespace(valid=True, z=0.0),
  )


def gps(latitude: float, longitude: float, bearing_deg: float = 90.0) -> SimpleNamespace:
  return SimpleNamespace(
    hasFix=True,
    flags=1,
    latitude=latitude,
    longitude=longitude,
    horizontalAccuracy=1.0,
    speed=10.0,
    bearingDeg=bearing_deg,
    bearingAccuracyDeg=1.0,
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


def test_trip_tracker_uses_live_pose_and_gps_correction():
  tracker = TripReportTracker()
  tracker.update_live_pose(live_pose(math.pi / 2.0), 0.0)
  tracker.update_gps(gps(37.0, 127.0), 0.0)
  tracker.update(0.0, 10.0, 0.0, 0.0, True, 2.7, 15.0)
  state = tracker.update(0.5, 10.0, 0.0, 0.0, True, 2.7, 15.0)

  assert state.heading_source == "livePose"
  assert state.current_x_m > 4.5
  assert abs(state.current_y_m) < 0.5

  tracker.update_gps(gps(37.0, 127.00006), 0.6)
  state = tracker.update(0.6, 10.0, 0.0, 0.0, True, 2.7, 15.0)
  assert state.gps_corrected


def test_trip_tracker_aligns_live_pose_heading_to_gps_bearing():
  tracker = TripReportTracker()
  pose_yaw_rad = math.radians(123.55)
  gps_bearing_deg = 38.96
  tracker.update_live_pose(live_pose(pose_yaw_rad), 0.0)
  tracker.update_gps(gps(37.0, 127.0, gps_bearing_deg), 0.0)
  tracker.update(0.0, 10.0, 0.0, 0.0, True, 2.7, 15.0)
  state = tracker.update(0.5, 10.0, 0.0, 0.0, True, 2.7, 15.0)

  expected_heading_rad = math.radians(gps_bearing_deg)
  assert state.heading_source == "livePose"
  assert state.gps_corrected
  assert math.isclose(state.current_x_m, math.sin(expected_heading_rad) * 5.0, abs_tol=0.05)
  assert math.isclose(state.current_y_m, math.cos(expected_heading_rad) * 5.0, abs_tol=0.05)


def test_late_gps_heading_rotates_existing_trace_north_up_without_a_kink():
  tracker = TripReportTracker()
  tracker.update_live_pose(live_pose(math.pi / 2.0), 0.0)
  tracker.update(0.0, 10.0, 0.0, 0.0, True, 2.7, 15.0)
  tracker.update_live_pose(live_pose(math.pi / 2.0), 0.5)
  tracker.update(0.5, 10.0, 0.0, 0.0, True, 2.7, 15.0)

  tracker.update_gps(gps(37.0, 127.0, 0.0), 0.6)
  tracker.update_live_pose(live_pose(math.pi / 2.0), 0.6)
  state = tracker.update(0.6, 10.0, 0.0, 0.0, True, 2.7, 15.0)

  assert state.gps_corrected
  assert len(state.trace_points) == 2
  for point in state.trace_points:
    assert math.isclose(point.x_m, state.current_x_m, abs_tol=1e-6)
  assert state.trace_points[0].y_m < state.trace_points[1].y_m < state.current_y_m


def test_gps_position_correction_translates_existing_trace():
  tracker = TripReportTracker()
  tracker.update_gps(gps(37.0, 127.0, 0.0), 0.0)
  tracker.update(0.0, 10.0, 0.0, 0.0, True, 2.7, 15.0)
  before = tracker.update(0.5, 10.0, 0.0, 0.0, True, 2.7, 15.0)
  before_relative = tuple(
    (point.x_m - before.current_x_m, point.y_m - before.current_y_m)
    for point in before.trace_points
  )

  latitude_5m_north = 37.0 + math.degrees(5.0 / EARTH_RADIUS_M)
  longitude_5m_east = 127.0 + math.degrees(5.0 / (EARTH_RADIUS_M * math.cos(math.radians(37.0))))
  tracker.update_gps(gps(latitude_5m_north, longitude_5m_east, 0.0), 0.6)
  after = tracker.update(0.6, 0.0, 0.0, 0.0, True, 2.7, 15.0)
  after_relative = tuple(
    (point.x_m - after.current_x_m, point.y_m - after.current_y_m)
    for point in after.trace_points
  )

  assert after.current_x_m > before.current_x_m
  assert len(after.trace_points) == len(before.trace_points)
  for before_point, after_point in zip(before_relative, after_relative, strict=True):
    assert math.isclose(after_point[0], before_point[0], abs_tol=1e-6)
    assert math.isclose(after_point[1], before_point[1], abs_tol=1e-6)


def test_trip_tracker_counts_events_once_and_bounds_trace_work():
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
  assert len(state.trace_points) <= 512
  assert state.trace_radius_m == TRIP_TRACE_MAX_RADIUS_M
  assert all(
    math.hypot(point.x_m - state.current_x_m, point.y_m - state.current_y_m)
    <= TRIP_TRACE_MAX_RADIUS_M
    for point in state.trace_points
  )
  assert state.distance_m > 38_000.0


def test_trip_trace_radius_grows_in_ten_meter_increments():
  tracker = TripReportTracker()
  tracker.update(0.0, 10.0, 0.0, 0.0, False, 2.7, 15.0)
  for index in range(1, 47):
    state = tracker.update(index * 0.5, 10.0, 0.0, 0.0, False, 2.7, 15.0)

  assert state.trace_radius_m == 260.0
  assert state.trace_radius_m % TRIP_TRACE_RADIUS_QUANTUM_M == 0.0


def test_route_parser_carries_report_into_cluster_state():
  parser = RouteLogParser(recompute_cutins=False)
  parser._update_live_pose(live_pose(0.0), 0.0)
  parser._frame_from_car_state(car_state(10.0), 0.0)
  frame = parser._frame_from_car_state(car_state(10.0), 0.5)
  state = frame_to_state(frame)

  assert state.trip_report is not None
  assert state.trip_report.distance_m == 5.0
  assert state.trip_report.heading_source == "livePose"


def test_report_mode_reserves_the_right_panel_for_non_camera_world_view():
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.screen_mode = CLUSTER_SCREEN_MODE_TRIP_REPORT

  assert renderer._world_view_shift_x(SimpleNamespace(camera_view_mode=0)) == NAVI_WORLD_VIEW_SHIFT_X
  assert renderer._world_view_shift_x(SimpleNamespace(camera_view_mode=2)) == 0.0


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


def test_trip_report_screen_coordinates_are_north_up():
  center = SimpleNamespace(x=100.0, y=100.0)
  north = ClusterUiRenderer._trip_trace_screen_position(0.0, 10.0, center, 1.0)
  east = ClusterUiRenderer._trip_trace_screen_position(10.0, 0.0, center, 1.0)

  assert north.x == center.x
  assert north.y < center.y
  assert east.x > center.x
  assert east.y == center.y


def test_trip_trace_zoom_eases_toward_the_next_radius(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._trip_trace_display_radius_m = 250.0
  renderer._trip_trace_zoom_t = 10.0
  monkeypatch.setattr(sys.modules["cluster_renderer"].time, "monotonic", lambda: 10.1)

  shown_radius_m = renderer._trip_trace_radius(260.0)

  assert 250.0 < shown_radius_m < 260.0


def test_trip_trace_tuple_is_reused_until_a_new_display_point_is_sampled():
  tracker = TripReportTracker()
  first = tracker.update(0.0, 1.0, 0.0, 0.0, False, 2.7, 15.0)
  second = tracker.update(0.1, 1.0, 0.0, 0.0, False, 2.7, 15.0)

  assert second.trace_points is first.trace_points
