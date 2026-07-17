import ast
from dataclasses import replace
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA, EGO_FORWARD_M, LIGHT_CLUSTER_THEME, VEHICLE_LENGTH_M
from cluster_models import ClusterUiState, DetectedVehicle, LaneMarking, RadarPoint
import cluster_renderer
from cluster_renderer import ClusterUiRenderer
import cluster_scene
from cluster_scene import (
  SCENE_STATE_FIELDS,
  build_cluster_scene,
  cluster_scene_state_key,
  detected_vehicle_scene_forward_m,
  render_scene_forward_m,
)


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


def test_scene_cache_key_covers_every_cluster_scene_state_access() -> None:
  tree = ast.parse(Path(cluster_scene.__file__).read_text(encoding="utf-8"))
  accessed_fields = {
    node.attr
    for node in ast.walk(tree)
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "state"
  }

  assert accessed_fields == set(SCENE_STATE_FIELDS)


def test_renderer_reuses_scene_when_only_hud_state_changes(monkeypatch) -> None:
  renderer = object.__new__(ClusterUiRenderer)
  renderer._scene_cache_key = None
  renderer._scene_cache = None
  renderer.profile_enabled = False
  scenes = []

  def build_scene(*_args, **_kwargs):
    scene = SimpleNamespace(sequence=len(scenes))
    scenes.append(scene)
    return scene

  monkeypatch.setattr(cluster_renderer, "build_cluster_scene", build_scene)
  state = _cluster_state()
  first = renderer._scene_for_state(state, True, LIGHT_CLUSTER_THEME)
  hud_only = renderer._scene_for_state(replace(state, center_clock_text="12:34"), True, LIGHT_CLUSTER_THEME)
  changed_world = renderer._scene_for_state(replace(state, steering=0.1), True, LIGHT_CLUSTER_THEME)

  assert cluster_scene_state_key(state) == cluster_scene_state_key(replace(state, center_clock_text="12:34"))
  assert first is hud_only
  assert changed_world is not first
  assert len(scenes) == 2


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
