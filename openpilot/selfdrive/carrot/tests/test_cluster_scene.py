from pathlib import Path
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_models import ClusterUiState, DetectedVehicle, LaneMarking, RadarPoint
from cluster_scene import build_cluster_scene


def test_raw_corner_points_remain_visible_when_vehicle_boxes_are_hidden() -> None:
  points = (
    RadarPoint("C1", 12.0, -2.0, "cornerRadar", relative_speed_mps=-1.0),
    RadarPoint("C2", 18.0, 0.5, "cornerRadar", relative_speed_mps=-1.0),
    RadarPoint("C3", 26.0, 2.0, "cornerRadar", relative_speed_mps=-1.0),
  )
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
    detected_vehicles=(
      DetectedVehicle("V", 18.0, 0.5, source="modelV2", probability=0.9),
    ),
    radar_points=points,
  )

  scene = build_cluster_scene(state)

  assert sorted(marker.label for marker in scene.radar_points) == ["C1", "C2", "C3"]
