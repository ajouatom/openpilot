from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from cluster_config import DEFAULT_LANE_WIDTH_M, WHITE

CruiseDisplayState = Literal["off", "paused", "engaged"]
GitBranchStatusState = Literal["ok", "pull", "missing", "unknown"]


@dataclass(frozen=True, slots=True)
class GitBranchStatus:
    branch: str
    state: GitBranchStatusState
    detail: str = ""


@dataclass(frozen=True, slots=True)
class ModelPathPoint:
    forward_m: float
    lateral_m: float
    lateral_std_m: float | None = None
    speed_mps: float | None = None
    accel_mps2: float | None = None
    orientation_rad: float | None = None
    orientation_rate_rps: float | None = None


@dataclass(frozen=True, slots=True)
class ModelRiskPoint:
    t_s: float
    brake_disengage: float = 0.0
    gas_disengage: float = 0.0
    steer_override: float = 0.0
    hard_brake_3: float = 0.0
    hard_brake_4: float = 0.0
    hard_brake_5: float = 0.0
    gas_press: float = 0.0
    brake_press: float = 0.0


@dataclass(frozen=True, slots=True)
class LaneMarking:
    offset: float
    color: tuple[int, int, int] = WHITE
    style: str = "solid"
    visible: bool = True
    width: int = 5
    model_points: tuple[ModelPathPoint, ...] = ()
    model_lateral_shift_m: float = 0.0


@dataclass(frozen=True, slots=True)
class RouteOverlay:
    video_rgba: bytes | None = None
    video_width: int = 0
    video_height: int = 0
    video_frame_id: str | None = None
    video_status: str | None = None
    data_lines: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class LiveDebugInfo:
    live_delay_calibration_percent: float | None = None
    live_delay_lateral_s: float | None = None
    live_torque_calibration_percent: float | None = None
    live_torque_valid: bool | None = None
    live_torque_lat_accel_factor: float | None = None
    live_torque_friction: float | None = None
    live_steer_ratio: float | None = None
    custom_steer_ratio: float | None = None
    steer_actuator_delay_s: float | None = None


@dataclass(frozen=True, slots=True)
class DebugPlotSnapshot:
    mode: int
    title: str
    values: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class DetectedVehicle:
    label: str
    longitudinal_m: float
    lateral_m: float
    source: str = "route"
    probability: float = 1.0
    relative_speed_mps: float | None = None
    absolute_speed_kph: float | None = None
    acceleration_mps2: float | None = None
    cut_in: bool = False
    primary: bool = False
    ttc_s: float | None = None
    x_std_m: float | None = None
    y_std_m: float | None = None


@dataclass(frozen=True, slots=True)
class RadarPoint:
    label: str
    longitudinal_m: float
    lateral_m: float
    source: str
    relative_speed_mps: float | None = None
    absolute_speed_kph: float | None = None
    lateral_speed_mps: float | None = None
    relative_accel_mps2: float | None = None
    probability: float | None = None
    valid: int | None = None
    valid_count: int | None = None
    in_my_lane: int | None = None


@dataclass(frozen=True, slots=True)
class SimulatorInput:
    throttle: float = 0.0
    brake: float = 0.0
    steering: float = 0.0
    steering_angle_deg: float | None = None
    camera_lane_center_offset_m: float | None = None
    camera_lane_width_m: float = DEFAULT_LANE_WIDTH_M
    surround_yaw_deg: float = 0.0
    surround_pitch_deg: float = 0.0
    surround_view_active: bool = False
    left_signal_requested: bool = False
    right_signal_requested: bool = False


@dataclass(frozen=True, slots=True)
class ClusterUiState:
    speed_kph: float
    accel_mps2: float
    steering: float
    speed_limit_kph: int | None
    cruise_kph: int | None
    cruise_display_state: CruiseDisplayState
    left_signal: bool
    right_signal: bool
    left_blindspot: bool
    right_blindspot: bool
    lane_change: str | None
    lane_change_phase: str
    lane_change_progress: float
    highlight_lane: str | None
    highlight_lane_offset: float | None
    ego_lane_offset: float
    road_view_lane_position: float
    camera_lane_center_offset_m: float | None
    lane_width_m: float
    steering_angle_deg: float | None
    surround_yaw_deg: float
    surround_pitch_deg: float
    surround_view_active: bool
    lanes: tuple[LaneMarking, ...]
    extra_left_lane_visible: bool = False
    extra_right_lane_visible: bool = False
    left_road_edge_offset: float | None = None
    right_road_edge_offset: float | None = None
    left_road_edge_points: tuple[ModelPathPoint, ...] = ()
    right_road_edge_points: tuple[ModelPathPoint, ...] = ()
    left_road_edge_lateral_shift_m: float = 0.0
    right_road_edge_lateral_shift_m: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    model_path: tuple[ModelPathPoint, ...] = ()
    detected_vehicles: tuple[DetectedVehicle, ...] = ()
    radar_points: tuple[RadarPoint, ...] = ()
    route_overlay: RouteOverlay | None = None
    live_debug: LiveDebugInfo | None = None
    debug_plot: DebugPlotSnapshot | None = None
    center_clock_text: str | None = None
    planned_speed_kph: float | None = None
    planned_accel_mps2: float | None = None
    planned_curvature_m_inv: float | None = None
    should_stop: bool = False
    model_confidence: str | None = None
    model_turn_speed_kph: float | None = None
    engaged_prob: float | None = None
    desire_state: tuple[float, ...] = ()
    desire_prediction: tuple[tuple[float, ...], ...] = ()
    risk_points: tuple[ModelRiskPoint, ...] = ()
    brake_disengage_risk: float = 0.0
    gas_disengage_risk: float = 0.0
    steer_override_risk: float = 0.0
    hard_brake_risk: float = 0.0
    gas_press_prob: float = 0.0
    brake_press_prob: float = 0.0
    disengage_risk: float = 0.0
    hard_brake_predicted: bool = False
    lane_change_available_left: bool | None = None
    lane_change_available_right: bool | None = None
    lane_change_prob: float = 0.0
    left_lane_width_m: float | None = None
    right_lane_width_m: float | None = None
    left_road_edge_distance_m: float | None = None
    right_road_edge_distance_m: float | None = None
    left_road_edge_confidence: float = 0.0
    right_road_edge_confidence: float = 0.0
    frame_age: int | None = None
    frame_drop_perc: float | None = None
    model_execution_time_ms: float | None = None
    vision_speed_mps: float | None = None
    vision_yaw_rate_rps: float | None = None
    vision_speed_std_mps: float | None = None
    vision_yaw_rate_std_rps: float | None = None
    camera_calibration_euler: tuple[float, float, float] | None = None
    road_transform_trans: tuple[float, float, float] | None = None
    road_transform_std: tuple[float, float, float] | None = None
    camera_odometry_valid: bool | None = None
    longitudinal_plan_source: str | None = None
    longitudinal_plan_speeds_kph: tuple[float, ...] = ()
    longitudinal_plan_accels_mps2: tuple[float, ...] = ()
    longitudinal_plan_jerks_mps3: tuple[float, ...] = ()
    longitudinal_plan_fcw: bool = False
    longitudinal_plan_should_stop: bool = False
    longitudinal_plan_allow_throttle: bool | None = None
    longitudinal_plan_allow_brake: bool | None = None
    longitudinal_t_follow_s: float | None = None
    longitudinal_desired_distance_m: float | None = None
    longitudinal_v_target_kph: float | None = None
    longitudinal_jerk_target_mps3: float | None = None
    lateral_plan_valid: bool | None = None
    lateral_plan_use_lane_lines: bool | None = None
    lateral_plan_solver_cost: float | None = None
    lateral_plan_debug_text: str | None = None
    lateral_plan_curvatures: tuple[float, ...] = ()
    lateral_plan_curvature_rates: tuple[float, ...] = ()
    display_speed_kph: float | None = None
    git_status: GitBranchStatus | None = None


@dataclass(frozen=True, slots=True)
class SceneCamera:
    active: bool
    position_x_m: float
    position_y_m: float
    position_z_m: float
    right_x: float
    right_y: float
    right_z: float
    up_x: float
    up_y: float
    up_z: float
    forward_x: float
    forward_y: float
    forward_z: float
    center_y: float
    focal_x: float
    focal_y: float
