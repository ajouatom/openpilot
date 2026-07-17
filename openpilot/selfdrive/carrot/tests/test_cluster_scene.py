from dataclasses import replace
from pathlib import Path
import sys

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA, EGO_FORWARD_M, VEHICLE_LENGTH_M
from cluster_models import ClusterUiState, DetectedVehicle, LaneMarking, RadarPoint
from cluster_scene import build_cluster_scene, detected_vehicle_scene_forward_m, render_scene_forward_m


def _cluster_state(**changes) -> ClusterUiState:
  state = ClusterUiState(
    speed_kph=60.0,
    accel_mps2=0.0,
    steering=0.0,
    speed_limit_kph=None,
    speed_limit_source=None,
    cruise_kph=None,
    cruise_display_state="off",
    gear_text=None,
    cruise_gap=None,
    lfa_active=None,
    left_signal=False,
    right_signal=False,
    left_blindspot=False,
    right_blindspot=False,
    lane_change=None,
    lane_change_phase="off",
    lane_change_progress=0.0,
    highlight_lane=None,
    highlight_lane_offset=None,
    ego_lane_offset=0.0,
    road_view_lane_position=0.0,
    camera_lane_center_offset_m=None,
    lane_width_m=3.6,
    steering_angle_deg=0.0,
    surround_yaw_deg=0.0,
    surround_pitch_deg=0.0,
    surround_view_active=False,
    lanes=(LaneMarking(-1.8), LaneMarking(1.8)),
  )
  return replace(state, **changes)


def test_longitudinal_render_distance_is_halved_without_changing_lateral_data() -> None:
  vehicle = DetectedVehicle(
    "L1",
    longitudinal_m=40.0,
    lateral_m=2.25,
    source="radarState",
    primary=True,
  )

  state = _cluster_state(detected_vehicles=(vehicle,))
  assert vehicle.lateral_m == 2.25
  assert render_scene_forward_m(vehicle.longitudinal_m, state) == pytest.approx(EGO_FORWARD_M + 20.0)
  assert render_scene_forward_m(-20.0, state) == pytest.approx(EGO_FORWARD_M - 10.0)
  assert detected_vehicle_scene_forward_m(vehicle, state) == pytest.approx(
    EGO_FORWARD_M + 20.0 + VEHICLE_LENGTH_M * 0.5
  )
  scene = build_cluster_scene(state)
  detected_box = next(box for box in scene.vehicles if box.label == "L1")
  assert detected_box.center.x == pytest.approx(2.25)
  assert detected_box.center.y == pytest.approx(EGO_FORWARD_M + 20.0 + VEHICLE_LENGTH_M * 0.5)
  assert detected_box.longitudinal_m == 40.0


def test_road_camera_keeps_longitudinal_render_distance_one_to_one() -> None:
  vehicle = DetectedVehicle(
    "L1",
    longitudinal_m=40.0,
    lateral_m=2.25,
    source="radarState",
    primary=True,
  )
  state = _cluster_state(
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    detected_vehicles=(vehicle,),
  )

  assert render_scene_forward_m(vehicle.longitudinal_m, state) == pytest.approx(EGO_FORWARD_M + 40.0)
  assert render_scene_forward_m(-20.0, state) == pytest.approx(EGO_FORWARD_M - 20.0)
  scene = build_cluster_scene(state)
  detected_box = next(box for box in scene.vehicles if box.label == "L1")
  assert detected_box.center.x == pytest.approx(2.25)
  assert detected_box.center.y == pytest.approx(EGO_FORWARD_M + 40.0 + VEHICLE_LENGTH_M * 0.5)
  assert detected_box.longitudinal_m == 40.0


def test_raw_corner_points_remain_visible_when_vehicle_boxes_are_hidden() -> None:
  points = (
    RadarPoint("C1", 12.0, -2.0, "cornerRadar", relative_speed_mps=-1.0),
    RadarPoint("C2", 18.0, 0.5, "cornerRadar", relative_speed_mps=-1.0),
    RadarPoint("C3", 26.0, 2.0, "cornerRadar", relative_speed_mps=-1.0),
  )
  state = _cluster_state(
    detected_vehicles=(
      DetectedVehicle("V", 18.0, 0.5, source="modelV2", probability=0.9),
    ),
    radar_points=points,
  )

  scene = build_cluster_scene(state)

  assert sorted(marker.label for marker in scene.radar_points) == ["C1", "C2", "C3"]
