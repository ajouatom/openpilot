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
    panel_visible: bool = True
    cutin_status: str | None = None
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
class TripReportState:
    duration_s: float = 0.0
    moving_time_s: float = 0.0
    distance_m: float = 0.0
    average_speed_kph: float = 0.0
    max_speed_kph: float = 0.0
    max_accel_mps2: float = 0.0
    max_decel_mps2: float = 0.0
    auto_ratio_percent: float = 0.0
    hard_accel_count: int = 0
    hard_brake_count: int = 0
    hard_corner_count: int = 0


@dataclass(frozen=True, slots=True)
class NaviItemMeta:
    sequence: int
    source_timestamp_ms: int
    received_mono_s: float


@dataclass(frozen=True, slots=True)
class NaviVehicleInfo:
    meta: NaviItemMeta
    latitude: float
    longitude: float
    heading_deg: float
    speed_kph: float
    road_name: str = ""
    virtual_gps: bool = False


@dataclass(frozen=True, slots=True)
class NaviGuidanceInfo:
    meta: NaviItemMeta
    distance_m: int
    time_s: int
    turn_type: int
    road_name: str = ""
    main_text: str = ""
    near_direction: str = ""
    mid_direction: str = ""
    far_direction: str = ""


@dataclass(frozen=True, slots=True)
class NaviLaneInfo:
    meta: NaviItemMeta
    count: int
    distance_m: int
    visible: bool
    lane_play: bool
    current_lane: int
    turn_code: int
    turn_info: tuple[int, ...] = ()
    available: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class NaviSpeedInfo:
    meta: NaviItemMeta
    current_kph: float
    road_limit_kph: int | None = None
    sdi_type: int | None = None
    sdi_distance_m: int | None = None
    sdi_speed_limit_kph: int | None = None
    secondary_sdi_type: int | None = None
    secondary_sdi_distance_m: int | None = None
    secondary_sdi_speed_limit_kph: int | None = None
    section_active: bool = False
    section_speed_limit_kph: int | None = None
    section_average_kph: float | None = None
    section_remaining_distance_m: float | None = None
    section_remaining_time_s: int | None = None
    section_progress: float | None = None


@dataclass(frozen=True, slots=True)
class NaviCrossroadInfo:
    meta: NaviItemMeta
    visible: bool
    distance_m: int
    image_code: int


@dataclass(frozen=True, slots=True)
class NaviRouteInfo:
    meta: NaviItemMeta
    remaining_distance_m: int
    remaining_time_s: int
    moved_distance_m: int
    moved_time_s: int
    total_distance_m: int
    polyline: tuple[tuple[float, float], ...] = ()


@dataclass(frozen=True, slots=True)
class NaviStatusInfo:
    meta: NaviItemMeta
    mode: str
    guidance_active: bool
    off_route: bool
    route_present: bool


@dataclass(frozen=True, slots=True)
class NaviLiveState:
    generation: int
    session_id: str
    vehicle: NaviVehicleInfo | None = None
    current: NaviGuidanceInfo | None = None
    next: NaviGuidanceInfo | None = None
    lane_current: NaviLaneInfo | None = None
    lane_ahead: tuple[NaviLaneInfo, ...] = ()
    speed: NaviSpeedInfo | None = None
    traffic_light: NaviTrafficLightInfo | None = None
    crossroad: NaviCrossroadInfo | None = None
    route: NaviRouteInfo | None = None
    status: NaviStatusInfo | None = None


@dataclass(frozen=True, slots=True)
class NaviDebugInfo:
    title: str
    lines: tuple[str, ...] = ()
    severity: str = "normal"
    speed_limit_kph: int | None = None
    traffic_light: NaviTrafficLightInfo | None = None
    guidance_image: NaviGuidanceImage | None = None


@dataclass(frozen=True, slots=True)
class NaviTrafficLightInfo:
    distance_m: int | None = None
    red_s: int | None = None
    straight_s: int | None = None
    left_s: int | None = None
    right_s: int | None = None
    uturn_s: int | None = None
    red_on: bool | None = None
    straight_on: bool | None = None
    left_on: bool | None = None
    right_on: bool | None = None
    uturn_on: bool | None = None
    meta: NaviItemMeta | None = None


@dataclass(frozen=True, slots=True)
class NaviGuidanceImage:
    image_base64: str = ""
    image_mime: str = ""
    image_hash: str = ""
    width: int = 0
    height: int = 0


@dataclass(frozen=True, slots=True)
class NaviMediaFrame:
    key: str
    sequence: int
    present: bool
    mime: str = ""
    width: int = 0
    height: int = 0
    data: bytes | None = None
    reason: str | None = None
    plane_data: tuple[bytes, bytes, bytes] | None = None
    plane_strides: tuple[int, int, int] | None = None
    hardware_buffer: object | None = None


@dataclass(frozen=True, slots=True)
class NaviItemStatus:
    key: str
    sequence: int
    present: bool
    reason: str | None = None


@dataclass(frozen=True, slots=True)
class NaviDashboardState:
    connected: bool
    endpoint: str
    session_id: str = ""
    app_version: str = ""
    manifest_revision: int = 0
    received_count: int = 0
    last_received_age_ms: int | None = None
    map_frame_age_ms: int | None = None
    map_stream_stalled: bool = False
    peer: str = "-"
    error: str | None = None
    media: tuple[NaviMediaFrame, ...] = ()
    items: tuple[NaviItemStatus, ...] = ()
    app_status: str = ""
    camera_status: str = ""
    composition_status: str = ""


@dataclass(frozen=True, slots=True)
class DetectedVehicle:
    label: str
    longitudinal_m: float
    lateral_m: float
    source: str = "route"
    probability: float = 1.0
    relative_speed_mps: float | None = None
    absolute_speed_kph: float | None = None
    lateral_speed_mps: float | None = None
    acceleration_mps2: float | None = None
    cut_in: bool = False
    primary: bool = False
    ttc_s: float | None = None
    x_std_m: float | None = None
    y_std_m: float | None = None
    radar_track_id: int | None = None


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
    motion_consistent: bool | None = None
    promotion_held: bool = False


RADAR_ZERO_POSITION_EPS_M = 1e-3
RADAR_MOVING_VEHICLE_MIN_SPEED_KPH = 12.0
RADAR_CROSS_TRAFFIC_MIN_LATERAL_SPEED_MPS = 2.0


def radar_position_is_zero(longitudinal_m: float, lateral_m: float) -> bool:
    return abs(longitudinal_m) <= RADAR_ZERO_POSITION_EPS_M and abs(lateral_m) <= RADAR_ZERO_POSITION_EPS_M


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
class TpmsInfo:
    fl: float | None = None
    fr: float | None = None
    rl: float | None = None
    rr: float | None = None


@dataclass(frozen=True, slots=True)
class ClusterAlert:
    text1: str
    text2: str = ""
    size: int = 0
    status: int = 0
    alert_type: str = ""


@dataclass(frozen=True, slots=True)
class ClusterUiState:
    speed_kph: float
    accel_mps2: float
    steering: float
    speed_limit_kph: int | None
    speed_limit_source: str | None
    cruise_kph: int | None
    cruise_display_state: CruiseDisplayState
    gear_text: str | None
    cruise_gap: int | None
    lfa_active: bool | None
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
    onroad: bool = False
    active_lane_line: bool | None = None
    camera_view_mode: int = 0
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
    corner_radar_supported: bool = False
    tpms: TpmsInfo = TpmsInfo()
    radar_info_mode: int = 4
    radar_display_mode: int = 0
    radar_source_color_mode: int = 0
    route_overlay: RouteOverlay | None = None
    live_debug: LiveDebugInfo | None = None
    debug_plot: DebugPlotSnapshot | None = None
    trip_report: TripReportState | None = None
    navi_debug: NaviDebugInfo | None = None
    navi_live: NaviLiveState | None = None
    navi_dashboard: NaviDashboardState | None = None
    debug_ui_visible: bool = False
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
    camera_device_type: str | None = None
    camera_sensor: str | None = None
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
    traffic_state: int = 0
    driving_mode: int | None = None
    git_status: GitBranchStatus | None = None
    actual_fps: float | None = None
    cluster_core_usage_text: str | None = None
    cpu_usage_percent: float | None = None
    cpu_temp_c: float | None = None
    memory_used_percent: float | None = None
    disk_used_percent: float | None = None
    network_address: str | None = None
    network_connected: bool = False
    external_nav_active: bool = False
    steering_output: float | None = None
    steering_output_normalized: float | None = None
    steering_output_kind: Literal["angle", "torque"] | None = None
    fuel_gauge: float | None = None
    energy_gauge_label: Literal["fuel", "battery"] = "fuel"
    urea_gauge: float | None = None
    ev_mode_valid: bool = False
    ev_mode_active: bool = False
    cruise_override_kph: float | None = None
    cruise_override_label: str | None = None
    cruise_override_color_mode: int = 0
    recorded_cutin_active: bool = False
    recorded_cutin_sound: bool = False
    alert: ClusterAlert | None = None


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
