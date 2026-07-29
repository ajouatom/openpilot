from __future__ import annotations

import math
import sys
from pathlib import Path
from types import SimpleNamespace


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import CLUSTER_SCREEN_MODE_TRIP_REPORT, normalize_cluster_screen_mode
from cluster_route_replay import RouteLogParser, frame_to_state
from cluster_trip_report import TripReportTracker
from cluster_renderer import ClusterUiRenderer, NAVI_WORLD_VIEW_SHIFT_X


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
  assert state.trace_radius_m == 30_000.0
  assert state.distance_m > 38_000.0


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


def test_trip_trace_tuple_is_reused_until_a_new_display_point_is_sampled():
  tracker = TripReportTracker()
  first = tracker.update(0.0, 1.0, 0.0, 0.0, False, 2.7, 15.0)
  second = tracker.update(0.1, 1.0, 0.0, 0.0, False, 2.7, 15.0)

  assert second.trace_points is first.trace_points
