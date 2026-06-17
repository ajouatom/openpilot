from __future__ import annotations

import math
import time
from collections import OrderedDict
from collections.abc import Callable, Iterable
from dataclasses import dataclass, replace

from cluster_config import (
    AMBER,
    BLUE,
    BLUE_SOFT,
    CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM,
    CLUSTER_RADAR_DISPLAY_DETAIL,
    CLUSTER_RADAR_SOURCE_COLOR_BY_SOURCE,
    ClusterTheme,
    DEFAULT_LANE_WIDTH_M,
    EGO,
    EGO_FORWARD_M,
    LIGHT_CLUSTER_THEME,
    PATH_END_M,
    PATH_HEIGHT_M,
    PATH_LANE_CHANGE_CURVE_END_M,
    PATH_LANE_CHANGE_CURVE_START_M,
    PATH_START_M,
    GREEN,
    RED,
    ROAD_CURVE_M_PER_M2,
    ROAD_FAR_M,
    ROAD_NEAR_M,
    SURROUND_CAMERA_DISTANCE_M,
    SURROUND_CAMERA_HEIGHT_M,
    SURROUND_MAX_PITCH_DEG,
    SURROUND_MAX_YAW_DEG,
    SURROUND_ROAD_FRONT_M,
    SURROUND_ROAD_REAR_M,
    SURROUND_TARGET_FORWARD_M,
    SURROUND_TARGET_HEIGHT_M,
    VEHICLE_HEIGHT_M,
    VEHICLE_LANE_CHANGE_SLOPE,
    VEHICLE_LENGTH_M,
    VEHICLE_WIDTH_M,
)
from cluster_models import ClusterUiState, DetectedVehicle, LaneMarking, ModelPathPoint, RadarPoint
from cluster_utils import clamp, darken, lighten, smoothstep


Color = tuple[int, int, int, int]
RoadEdgeLayer = tuple[int, Color, float, float]
ProfileAdd = Callable[[str, float], None]
PATH_BLOCKER_CLEARANCE_M = 1.25
PATH_BLOCKER_LANE_TOLERANCE = 0.42
RADAR_VEHICLE_MIN_VALID_COUNT = 20
RADAR_VEHICLE_MAX_DISTANCE_M = 150.0
RADAR_VEHICLE_MAX_LATERAL_LANES = 2.75
RADAR_ROAD_EDGE_HARD_CLEARANCE_M = 0.55
RADAR_ROAD_EDGE_STATIONARY_CLEARANCE_M = 1.05
RADAR_ROAD_EDGE_OUTSIDE_MARGIN_M = 0.25
RADAR_ROAD_EDGE_KEEP_OUTSIDE_MARGIN_M = 0.85
RADAR_ROAD_EDGE_STABLE_VEHICLE_OUTSIDE_MARGIN_M = 2.25
RADAR_ROAD_EDGE_KEEP_SPEED_KPH = 18.0
RADAR_ROAD_EDGE_KEEP_MIN_VALID_COUNT = 20
RADAR_ROAD_EDGE_STABLE_VEHICLE_MIN_VALID_COUNT = 20
RADAR_ROAD_EDGE_STABLE_VEHICLE_MAX_ACCEL_MPS2 = 5.0
RADAR_STATIC_OBJECT_SPEED_MPS = 1.25
RADAR_STATIC_OBJECT_SPEED_KPH = 8.0
RADAR_SIDE_STATIC_LATERAL_LANES = 0.58
RADAR_EGO_MOVING_SPEED_KPH = 10.0
RADAR_CENTER_RAW_LATERAL_LANES = 0.72
RADAR_ADJACENT_RAW_LATERAL_LANES = 1.45
RADAR_OUTER_RAW_LATERAL_LANES = 2.65
RADAR_RAW_MOVING_SPEED_KPH = 8.0
RADAR_RAW_CENTER_MIN_VALID_COUNT = 20
RADAR_RAW_ADJACENT_MIN_VALID_COUNT = 24
RADAR_RAW_OUTER_MIN_VALID_COUNT = 35
RADAR_PROBABLE_VEHICLE_LATERAL_LANES = 2.75
RADAR_VEHICLE_MIN_PROBABILITY = 0.35
RADAR_VEHICLE_DEDUP_LONGITUDINAL_M = 7.0
RADAR_VEHICLE_DEDUP_LATERAL_M = 1.6
RADAR_POINT_MERGE_BASE_LONGITUDINAL_M = 0.75
RADAR_POINT_MERGE_MAX_LONGITUDINAL_M = 2.4
RADAR_POINT_MERGE_LATERAL_M = 0.65
RADAR_POINT_MERGE_SPEED_KPH = 3.0
RADAR_MERGE_LONGITUDINAL_MIN_M = 3.0
RADAR_MERGE_LONGITUDINAL_MAX_M = 7.0
RADAR_MERGE_LATERAL_M = 1.35
RADAR_FRONT_DETECTED_MERGE_LONGITUDINAL_MIN_M = 5.0
RADAR_FRONT_DETECTED_MERGE_LONGITUDINAL_MAX_M = 11.0
RADAR_FRONT_DETECTED_MERGE_LATERAL_M = 2.25
RADAR_MERGED_SOURCE_TAG = "+radar:"
CORNER_RADAR_LABELS = frozenset(("LF", "RF", "LR", "RR"))
DRIVE_CAMERA_FORWARD_SHIFT_M = 5.0
DRIVE_CAMERA_EGO_BOTTOM_POSITION_M = (0.0, -6.0, 5.00)
DRIVE_CAMERA_EGO_BOTTOM_TARGET_M = (0.0, 14.0, -1.00)
DRIVE_VIEW_REAR_RELATIVE_M = -5.0
DRIVE_VIEW_REAR_ROAD_MARGIN_M = 8.0
LONGITUDINAL_RENDER_DISTANCE_SCALE = 0.5
DRIVE_VIEW_REAR_VISIBLE_M = EGO_FORWARD_M + DRIVE_VIEW_REAR_RELATIVE_M
DRIVE_VIEW_ROAD_START_M = (
    DRIVE_VIEW_REAR_VISIBLE_M - DRIVE_VIEW_REAR_ROAD_MARGIN_M
)
VEHICLE_BADGE_TTC_S = 9.9
VEHICLE_BADGE_ACCEL_MPS2 = 1.0
MODEL_LINE_STRIP_GROUP_CACHE_LIMIT = 48
MODEL_LINE_STRIP_GROUP_CACHE_GRID_M = 0.5
MODEL_LINE_STRIP_GROUP_CACHE_POINT_GRID_M = 0.05
MODEL_LINE_STRIP_GROUP_CACHE_COLOR: Color = (0, 0, 0, 0)
MODEL_LINE_RENDER_POINT_KEY_CACHE_LIMIT = 256
MODEL_LINE_RENDER_POINT_LIMIT = 0
LANE_OFFSET_STRIP_CACHE_LIMIT = 64
LANE_OFFSET_STRIP_CACHE_OFFSET_GRID = 0.01
LANE_OFFSET_STRIP_CACHE_STEERING_GRID = 0.002
LANE_OFFSET_STRIP_CACHE_LANE_WIDTH_GRID_M = 0.02
ROAD_EDGE_OFFSET_STRIP_CACHE_LIMIT = 48
ROAD_EDGE_OFFSET_STRIP_CACHE_OFFSET_GRID = 0.01
ROAD_EDGE_OFFSET_STRIP_CACHE_STEERING_GRID = 0.002
ROAD_EDGE_OFFSET_STRIP_CACHE_LANE_WIDTH_GRID_M = 0.02
PLANNED_PATH_STRIP_CACHE_LIMIT = 48
ROAD_STEPS_SURROUND = 96
ROAD_STEPS_MODEL = 48
ROAD_STEPS_SIM = 64
STATIC_LINE_STEPS = 56
ROAD_EDGE_OFFSET_STEPS = STATIC_LINE_STEPS
PLANNED_PATH_FALLBACK_STEPS = 32
MODEL_PATH_METRIC_SEGMENT_LIMIT = 14
LANE_MARKING_SHADOW_HEIGHT_M = 0.026
LANE_MARKING_HEIGHT_M = 0.044
LANE_MARKING_BORDER_EXTRA_WIDTH_PX = 3
LANE_MARKING_BORDER_COLOR = LIGHT_CLUSTER_THEME.lane_marking_border
ROAD_EDGE_SHADOW_HEIGHT_M = 0.032
ROAD_EDGE_BODY_HEIGHT_M = 0.058
ROAD_EDGE_HEIGHT_M = 0.074
ROAD_EDGE_CREST_HEIGHT_M = 0.106
ROAD_EDGE_OUTSIDE_SHADOW_OFFSET_M = 0.13
ROAD_EDGE_BODY_OFFSET_M = 0.055
ROAD_EDGE_CREST_OFFSET_M = -0.045
ROAD_EDGE_BACKING_COLOR = LIGHT_CLUSTER_THEME.road_edge_backing
ROAD_EDGE_MODEL_POINT_LIMIT = 0
STYLE_MESH_STRIP_GROUP_CACHE_LIMIT = 128
MERGED_MESH_STRIP_CACHE_LIMIT = 128
PATH_SHADOW_LAYER_M = 0.024
PATH_UNCERTAINTY_LAYER_M = PATH_HEIGHT_M + 0.002
PATH_BODY_LAYER_M = PATH_HEIGHT_M + 0.046
PATH_METRIC_LAYER_M = PATH_HEIGHT_M + 0.066
PATH_HIGHLIGHT_LAYER_M = PATH_HEIGHT_M + 0.088
FOLLOW_DISTANCE_MARKER_BACKING_LAYER_M = PATH_HEIGHT_M + 0.116
FOLLOW_DISTANCE_MARKER_BODY_LAYER_M = PATH_HEIGHT_M + 0.132
FOLLOW_DISTANCE_MARKER_BACKING_FORWARD_M = 0.28
FOLLOW_DISTANCE_MARKER_BODY_FORWARD_M = 0.14
FOLLOW_DISTANCE_MARKER_BACKING_EXTRA_WIDTH_M = 0.22
FOLLOW_DISTANCE_MARKER_BACKING_COLOR: Color = (42, 0, 38, 230)
FOLLOW_DISTANCE_MARKER_BODY_COLOR: Color = (255, 0, 220, 248)
EGO_VEHICLE_CENTER_FORWARD_M = EGO_FORWARD_M - VEHICLE_LENGTH_M * 0.5
LANE_HIGHLIGHT_COLOR = (64, 148, 255)
LANE_HIGHLIGHT_ALPHA = 220
LANE_HIGHLIGHT_ROUTE_ALPHA = 170
BSD_LANE_MARKING_MATCH_TOLERANCE = 0.45
LANE_DASH_LENGTH_M = 5.2
LANE_DASH_GAP_M = 4.2


@dataclass(frozen=True, slots=True)
class Vec3:
    x: float
    y: float
    z: float = 0.0


@dataclass(frozen=True, slots=True)
class CameraSpec:
    position: Vec3
    target: Vec3
    fovy_deg: float


@dataclass(frozen=True, slots=True)
class MeshStrip:
    left: tuple[Vec3, ...]
    right: tuple[Vec3, ...]
    color: Color
    x_offset_m: float = 0.0


@dataclass(frozen=True, slots=True)
class VehicleBox:
    center: Vec3
    right_x: float
    right_y: float
    forward_x: float
    forward_y: float
    width_m: float
    length_m: float
    height_m: float
    body_color: Color
    side_color: Color
    rear_color: Color
    top_highlight: Color
    outline_color: Color
    confidence: float = 1.0
    label: str = ""
    source: str = ""
    longitudinal_m: float | None = None
    relative_speed_mps: float | None = None
    absolute_speed_kph: float | None = None
    acceleration_mps2: float | None = None
    ttc_s: float | None = None
    cut_in: bool = False
    primary: bool = False
    annotate: bool = False


@dataclass(frozen=True, slots=True)
class RadarPointMarker:
    center: Vec3
    radius_m: float
    color: Color
    label: str
    longitudinal_m: float
    lateral_m: float
    relative_speed_mps: float | None = None
    absolute_speed_kph: float | None = None
    lateral_speed_mps: float | None = None
    relative_accel_mps2: float | None = None
    probability: float | None = None
    valid: int | None = None
    in_my_lane: int | None = None


@dataclass(frozen=True, slots=True)
class PathBlocker:
    offset: float
    forward_m: float
    length_m: float


MeshStripGroups = tuple[tuple[MeshStrip, ...], ...]
ModelLineStripGroups = MeshStripGroups | None
ModelLineStripGeometrySpecs = tuple[tuple[int, float], ...]
ModelLineStripPointKey = tuple[tuple[int, int], ...]
ModelLineStripCacheKey = tuple[
    ModelLineStripPointKey,
    float,
    float,
    str,
    bool,
    ModelLineStripGeometrySpecs,
]
_MODEL_LINE_STRIP_GROUP_CACHE: OrderedDict[ModelLineStripCacheKey, ModelLineStripGroups] = OrderedDict()
ModelLineRenderPointKeyCacheKey = tuple[int, int]
_MODEL_LINE_RENDER_POINT_KEY_CACHE: OrderedDict[
    ModelLineRenderPointKeyCacheKey,
    tuple[tuple[ModelPathPoint, ...], tuple[ModelPathPoint, ...], ModelLineStripPointKey],
] = OrderedDict()
LaneOffsetStripCacheKey = tuple[
    float,
    float,
    float,
    float,
    float,
    str,
    ModelLineStripGeometrySpecs,
]
_LANE_OFFSET_STRIP_CACHE: OrderedDict[LaneOffsetStripCacheKey, tuple[tuple[MeshStrip, ...], ...]] = OrderedDict()
RoadEdgeOffsetLayerGeometrySpecs = tuple[tuple[int, float, float], ...]
RoadEdgeOffsetStripGroups = tuple[tuple[MeshStrip, ...], ...]
RoadEdgeOffsetStripCacheKey = tuple[
    float,
    float,
    float,
    float,
    float,
    float,
    RoadEdgeOffsetLayerGeometrySpecs,
]
_ROAD_EDGE_OFFSET_STRIP_CACHE: OrderedDict[RoadEdgeOffsetStripCacheKey, RoadEdgeOffsetStripGroups] = OrderedDict()
PlannedPathStripSpecs = tuple[tuple[float, Color, float], ...]
PlannedPathStripCacheKey = tuple[tuple[Vec3, ...], PlannedPathStripSpecs]
_PLANNED_PATH_STRIP_CACHE: OrderedDict[PlannedPathStripCacheKey, tuple[MeshStrip, ...]] = OrderedDict()
StyledMeshStripGroupCacheKey = tuple[int, tuple[tuple[int, Color, float], ...], tuple[float, ...]]
_STYLE_MESH_STRIP_GROUP_CACHE: OrderedDict[
    StyledMeshStripGroupCacheKey,
    tuple[MeshStripGroups, MeshStripGroups],
] = OrderedDict()
MergedMeshStripCacheKey = tuple[tuple[int, int], ...]
MergedMeshStripRefs = tuple[tuple[tuple[Vec3, ...], tuple[Vec3, ...]], ...]
_MERGED_MESH_STRIP_CACHE: OrderedDict[
    MergedMeshStripCacheKey,
    tuple[MergedMeshStripRefs, tuple[Vec3, ...], tuple[Vec3, ...]],
] = OrderedDict()


@dataclass(frozen=True, slots=True)
class ClusterScene:
    camera: CameraSpec
    scene_shift_x_m: float
    road_surface: MeshStrip
    road_edges: tuple[MeshStrip, ...]
    highlight_lanes: tuple[MeshStrip, ...]
    lane_markings: tuple[MeshStrip, ...]
    planned_path: tuple[MeshStrip, ...]
    radar_points: tuple[RadarPointMarker, ...]
    vehicles: tuple[VehicleBox, ...]


def vec3_with_x_offset(vec: Vec3, x_offset_m: float) -> Vec3:
    return Vec3(vec.x + x_offset_m, vec.y, vec.z)


def vehicle_box_with_x_offset(vehicle: VehicleBox, x_offset_m: float) -> VehicleBox:
    if abs(x_offset_m) <= 0.0001:
        return vehicle
    return replace(vehicle, center=vec3_with_x_offset(vehicle.center, x_offset_m))


def rgba(color: tuple[int, int, int], alpha: int = 255) -> Color:
    return color[0], color[1], color[2], alpha


def rgba_with_alpha(color: tuple[int, int, int], alpha: float) -> Color:
    return color[0], color[1], color[2], int(clamp(alpha, 0, 255))


def road_curve_m(forward_m: float, steering: float) -> float:
    return clamp(steering, -1.0, 1.0) * ROAD_CURVE_M_PER_M2 * forward_m * forward_m


def road_world_x(offset: float, forward_m: float, steering: float, lane_width_m: float) -> float:
    return offset * lane_width_m + road_curve_m(forward_m, steering)


def normalize2(x: float, y: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    if length <= 0.0001:
        return 0.0, 1.0
    return x / length, y / length


def vehicle_heading(
    offset: float,
    forward_m: float,
    steering: float,
    lane_width_m: float,
    target_offset: float | None = None,
) -> tuple[float, float, float, float]:
    road_slope = 2.0 * clamp(steering, -1.0, 1.0) * ROAD_CURVE_M_PER_M2 * forward_m
    lane_change_slope = 0.0
    if target_offset is not None:
        lane_delta_m = (target_offset - offset) * lane_width_m
        lane_change_slope = clamp(
            lane_delta_m / 18.0,
            -VEHICLE_LANE_CHANGE_SLOPE,
            VEHICLE_LANE_CHANGE_SLOPE,
        )
    forward_x, forward_y = normalize2(road_slope + lane_change_slope, 1.0)
    right_x = forward_y
    right_y = -forward_x
    return right_x, right_y, forward_x, forward_y


def sample_range(start_m: float, end_m: float, steps: int) -> tuple[float, ...]:
    steps = max(1, steps)
    return tuple(start_m + (end_m - start_m) * index / steps for index in range(steps + 1))


def data_scene_forward_m(relative_forward_m: float) -> float:
    return EGO_FORWARD_M + relative_forward_m


def render_relative_forward_m(relative_forward_m: float) -> float:
    return relative_forward_m * LONGITUDINAL_RENDER_DISTANCE_SCALE


def render_scene_forward_m(relative_forward_m: float) -> float:
    return data_scene_forward_m(render_relative_forward_m(relative_forward_m))


def scene_data_relative_forward_m(forward_m: float) -> float:
    return forward_m - EGO_FORWARD_M


def lane_centerline(
    offset: float,
    steering: float,
    lane_width_m: float,
    start_m: float,
    end_m: float,
    steps: int,
    height_m: float = 0.0,
) -> tuple[Vec3, ...]:
    return tuple(
        Vec3(
            road_world_x(offset, forward_m, steering, lane_width_m),
            forward_m,
            height_m,
        )
        for forward_m in sample_range(start_m, end_m, steps)
    )


def strip_between_offsets(
    left_offset: float,
    right_offset: float,
    steering: float,
    lane_width_m: float,
    start_m: float,
    end_m: float,
    steps: int,
    color: Color,
    height_m: float = 0.0,
) -> MeshStrip:
    return MeshStrip(
        left=lane_centerline(left_offset, steering, lane_width_m, start_m, end_m, steps, height_m),
        right=lane_centerline(right_offset, steering, lane_width_m, start_m, end_m, steps, height_m),
        color=color,
    )


def model_line_lateral_at_forward(
    points: tuple[ModelPathPoint, ...],
    relative_forward_m: float,
    lateral_shift_m: float = 0.0,
) -> float | None:
    if not points or relative_forward_m < 0.0:
        return None
    previous = points[0]
    if relative_forward_m <= previous.forward_m:
        return previous.lateral_m + lateral_shift_m
    for point in points[1:]:
        if relative_forward_m <= point.forward_m:
            span = max(0.001, point.forward_m - previous.forward_m)
            amount = clamp((relative_forward_m - previous.forward_m) / span, 0.0, 1.0)
            return previous.lateral_m + (point.lateral_m - previous.lateral_m) * amount + lateral_shift_m
        previous = point
    return None


def strip_between_model_lines(
    left_points: tuple[ModelPathPoint, ...],
    right_points: tuple[ModelPathPoint, ...],
    left_lateral_shift_m: float,
    right_lateral_shift_m: float,
    start_m: float,
    end_m: float,
    steps: int,
    color: Color,
    height_m: float,
    extend_before_model: bool = False,
) -> MeshStrip | None:
    if len(left_points) < 2 or len(right_points) < 2:
        return None

    relative_start_m = max(0.0, scene_data_relative_forward_m(start_m))
    if not extend_before_model:
        relative_start_m = max(relative_start_m, left_points[0].forward_m, right_points[0].forward_m)
    relative_end_m = min(
        scene_data_relative_forward_m(end_m),
        left_points[-1].forward_m,
        right_points[-1].forward_m,
    )
    scene_start_m = start_m if extend_before_model else data_scene_forward_m(relative_start_m)
    scene_end_m = min(end_m, data_scene_forward_m(relative_end_m))
    if scene_end_m <= scene_start_m + 1.0:
        return None

    left: list[Vec3] = []
    right: list[Vec3] = []
    for forward_m in sample_range(scene_start_m, scene_end_m, steps):
        relative_forward_m = scene_data_relative_forward_m(forward_m)
        left_lateral = (
            left_points[0].lateral_m + left_lateral_shift_m
            if extend_before_model and relative_forward_m < left_points[0].forward_m
            else model_line_lateral_at_forward(left_points, relative_forward_m, left_lateral_shift_m)
        )
        right_lateral = (
            right_points[0].lateral_m + right_lateral_shift_m
            if extend_before_model and relative_forward_m < right_points[0].forward_m
            else model_line_lateral_at_forward(right_points, relative_forward_m, right_lateral_shift_m)
        )
        if left_lateral is None or right_lateral is None:
            continue
        if left_lateral <= right_lateral:
            left.append(Vec3(left_lateral, forward_m, height_m))
            right.append(Vec3(right_lateral, forward_m, height_m))
        else:
            left.append(Vec3(right_lateral, forward_m, height_m))
            right.append(Vec3(left_lateral, forward_m, height_m))

    if len(left) < 2 or len(right) < 2:
        return None
    return MeshStrip(tuple(left), tuple(right), color)


def marking_near_offset(markings: tuple[LaneMarking, ...], offset: float) -> LaneMarking | None:
    candidates = [marking for marking in markings if marking.visible]
    if not candidates:
        return None
    marking = min(candidates, key=lambda candidate: abs(candidate.offset - offset))
    return marking if abs(marking.offset - offset) <= 0.30 else None


def lane_floor_strip(
    state: ClusterUiState,
    lane_center_offset: float,
    color: Color,
    lane_width_m: float,
    road_start_m: float,
    road_end_m: float,
    road_steps: int,
    route_mode: bool,
    height_m: float,
) -> MeshStrip | None:
    left_marking = marking_near_offset(state.lanes, lane_center_offset - 0.5)
    right_marking = marking_near_offset(state.lanes, lane_center_offset + 0.5)
    if left_marking is not None and right_marking is not None:
        model_strip = strip_between_model_lines(
            left_marking.model_points,
            right_marking.model_points,
            left_marking.model_lateral_shift_m,
            right_marking.model_lateral_shift_m,
            road_start_m,
            road_end_m,
            road_steps,
            color,
            height_m,
            extend_before_model=True,
        )
        if model_strip is not None:
            return model_strip

    if route_mode:
        return None
    return strip_between_offsets(
        lane_center_offset - 0.5,
        lane_center_offset + 0.5,
        state.steering,
        lane_width_m,
        road_start_m,
        road_end_m,
        road_steps,
        color,
        height_m,
    )


def strip_from_centerline(points: tuple[Vec3, ...], width_m: float, color: Color) -> MeshStrip:
    if len(points) < 2:
        return MeshStrip(points, points, color)

    left: list[Vec3] = []
    right: list[Vec3] = []
    half_width = width_m * 0.5
    for index, point in enumerate(points):
        previous_point = points[max(0, index - 1)]
        next_point = points[min(len(points) - 1, index + 1)]
        tangent_x, tangent_y = normalize2(
            next_point.x - previous_point.x,
            next_point.y - previous_point.y,
        )
        right_x = tangent_y
        right_y = -tangent_x
        left.append(Vec3(point.x - right_x * half_width, point.y - right_y * half_width, point.z))
        right.append(Vec3(point.x + right_x * half_width, point.y + right_y * half_width, point.z))
    return MeshStrip(tuple(left), tuple(right), color)


def model_line_centerline(
    model_points: tuple[ModelPathPoint, ...],
    start_m: float,
    end_m: float,
    height_m: float,
    lateral_shift_m: float = 0.0,
) -> tuple[Vec3, ...]:
    centerline: list[Vec3] = []
    ego_forward_m = EGO_FORWARD_M
    for point in model_points:
        forward_m = ego_forward_m + point.forward_m
        if start_m <= forward_m <= end_m:
            centerline.append(Vec3(point.lateral_m + lateral_shift_m, forward_m, height_m))
    return tuple(centerline)


def extend_centerline_rearward_to_first_point(
    centerline: tuple[Vec3, ...],
    start_m: float,
    height_m: float,
) -> tuple[Vec3, ...]:
    if len(centerline) < 2 or start_m >= centerline[0].y - 0.10:
        return centerline

    first_point = centerline[0]
    extension_end_m = first_point.y
    extension_length_m = extension_end_m - start_m
    rear_points = [Vec3(first_point.x, start_m, height_m)]
    transition_gap_m = min(1.0, extension_length_m * 0.25)
    transition_m = extension_end_m - transition_gap_m
    if transition_m > start_m + 0.10:
        rear_points.append(Vec3(first_point.x, transition_m, height_m))
    return (*rear_points, *centerline)


def extend_model_centerline_rearward(
    centerline: tuple[Vec3, ...],
    start_m: float,
) -> tuple[Vec3, ...]:
    return extend_centerline_rearward_to_first_point(
        centerline,
        start_m,
        0.0,
    )


def append_unique_point(points: list[Vec3], point: Vec3) -> None:
    if not points or points[-1] != point:
        points.append(point)


def lerp_vec3(start: Vec3, end: Vec3, amount: float) -> Vec3:
    return Vec3(
        start.x + (end.x - start.x) * amount,
        start.y + (end.y - start.y) * amount,
        start.z + (end.z - start.z) * amount,
    )


def lane_dash_cycle_offset(distance_m: float, dash_phase_m: float, cycle_m: float, dash_m: float, eps: float) -> float:
    cycle_offset_m = (distance_m + dash_phase_m) % cycle_m
    if cycle_offset_m < eps or abs(cycle_offset_m - cycle_m) < eps:
        return 0.0
    if abs(cycle_offset_m - dash_m) < eps:
        return dash_m
    return cycle_offset_m


def lane_dash_phase_m(
    centerline: tuple[Vec3, ...],
    dash_m: float = LANE_DASH_LENGTH_M,
    gap_m: float = LANE_DASH_GAP_M,
) -> float:
    if not centerline:
        return 0.0
    # Start the visible rear bound with paint even when road geometry begins below the screen.
    visible_rear_distance_m = max(0.0, DRIVE_VIEW_REAR_VISIBLE_M - centerline[0].y)
    return (-visible_rear_distance_m) % (dash_m + gap_m)


def dashed_lane_start_cursor_m(start_m: float, dash_m: float, gap_m: float) -> float:
    if start_m >= DRIVE_VIEW_REAR_VISIBLE_M:
        return start_m
    cycle_m = dash_m + gap_m
    cycles_to_visible = math.ceil((DRIVE_VIEW_REAR_VISIBLE_M - start_m) / cycle_m)
    return DRIVE_VIEW_REAR_VISIBLE_M - cycles_to_visible * cycle_m


def dashed_centerline_segments(
    centerline: tuple[Vec3, ...],
    dash_m: float = LANE_DASH_LENGTH_M,
    gap_m: float = LANE_DASH_GAP_M,
) -> tuple[tuple[Vec3, ...], ...]:
    if len(centerline) < 2:
        return ()

    cycle_m = dash_m + gap_m
    segments: list[tuple[Vec3, ...]] = []
    current_dash: list[Vec3] = []
    distance_m = 0.0
    dash_phase_m = lane_dash_phase_m(centerline, dash_m, gap_m)
    previous = centerline[0]
    eps = 0.0001

    for current in centerline[1:]:
        segment_dx = current.x - previous.x
        segment_dy = current.y - previous.y
        segment_dz = current.z - previous.z
        segment_m = math.sqrt(segment_dx * segment_dx + segment_dy * segment_dy + segment_dz * segment_dz)
        if segment_m <= 0.001:
            previous = current
            continue

        segment_start_m = distance_m
        segment_end_m = distance_m + segment_m
        cursor_m = segment_start_m
        cursor_point = previous

        while cursor_m < segment_end_m - eps:
            cycle_offset_m = lane_dash_cycle_offset(cursor_m, dash_phase_m, cycle_m, dash_m, eps)
            in_dash = cycle_offset_m < dash_m
            boundary_m = cursor_m + (dash_m - cycle_offset_m if in_dash else cycle_m - cycle_offset_m)
            next_m = min(segment_end_m, boundary_m)
            if next_m <= cursor_m + eps:
                next_m = segment_end_m

            next_point = lerp_vec3(previous, current, (next_m - segment_start_m) / segment_m)
            if in_dash:
                append_unique_point(current_dash, cursor_point)
                append_unique_point(current_dash, next_point)
                if boundary_m <= next_m + eps and len(current_dash) >= 2:
                    segments.append(tuple(current_dash))
                    current_dash = []
            elif len(current_dash) >= 2:
                segments.append(tuple(current_dash))
                current_dash = []

            cursor_m = next_m
            cursor_point = next_point

        distance_m = segment_end_m
        previous = current

    if len(current_dash) >= 2:
        segments.append(tuple(current_dash))
    return tuple(segments)


def lane_marking_segments_for_marking(
    marking: LaneMarking,
    steering: float,
    lane_width_m: float,
    start_m: float,
    end_m: float,
    extend_before_model: bool = False,
) -> tuple[tuple[Vec3, ...], ...]:
    if marking.model_points:
        centerline = model_line_centerline(
            marking.model_points,
            start_m,
            end_m,
            0.0,
            marking.model_lateral_shift_m,
        )
        if len(centerline) < 2:
            if not extend_before_model:
                return ()
        else:
            if extend_before_model:
                centerline = extend_model_centerline_rearward(
                    centerline,
                    start_m,
                )
            if marking.style == "solid":
                return (centerline,)
            return dashed_centerline_segments(centerline)

    if marking.style == "solid":
        return (lane_centerline(marking.offset, steering, lane_width_m, start_m, end_m, STATIC_LINE_STEPS, 0.0),)

    segments: list[tuple[Vec3, ...]] = []
    dash_m = LANE_DASH_LENGTH_M
    cycle_m = dash_m + LANE_DASH_GAP_M
    cursor = dashed_lane_start_cursor_m(start_m, dash_m, LANE_DASH_GAP_M)
    while cursor < end_m:
        dash_start = max(cursor, start_m)
        dash_end = min(cursor + dash_m, end_m)
        if dash_end > dash_start + 0.001:
            segment = lane_centerline(marking.offset, steering, lane_width_m, dash_start, dash_end, 6, 0.0)
            segments.append(segment)
        cursor += cycle_m
    return tuple(segments)


def strips_from_centerline_width_specs(
    points: tuple[Vec3, ...],
    specs: tuple[tuple[float, Color, float], ...],
) -> tuple[MeshStrip, ...]:
    if len(points) < 2:
        return ()

    half_widths = tuple(max(0.001, width_m) * 0.5 for width_m, _, _ in specs)
    left_groups: list[list[Vec3]] = [[] for _ in specs]
    right_groups: list[list[Vec3]] = [[] for _ in specs]

    for index, point in enumerate(points):
        previous_point = points[max(0, index - 1)]
        next_point = points[min(len(points) - 1, index + 1)]
        tangent_x, tangent_y = normalize2(
            next_point.x - previous_point.x,
            next_point.y - previous_point.y,
        )
        right_x = tangent_y
        right_y = -tangent_x
        for spec_index, half_width in enumerate(half_widths):
            height_m = specs[spec_index][2]
            left_groups[spec_index].append(
                Vec3(point.x - right_x * half_width, point.y - right_y * half_width, height_m)
            )
            right_groups[spec_index].append(
                Vec3(point.x + right_x * half_width, point.y + right_y * half_width, height_m)
            )

    return tuple(
        MeshStrip(tuple(left_groups[index]), tuple(right_groups[index]), color)
        for index, (_, color, _) in enumerate(specs)
    )


def cached_strips_from_centerline_width_specs(
    points: tuple[Vec3, ...],
    specs: PlannedPathStripSpecs,
) -> tuple[MeshStrip, ...]:
    key = (points, specs)
    cached = _PLANNED_PATH_STRIP_CACHE.get(key)
    if cached is not None:
        _PLANNED_PATH_STRIP_CACHE.move_to_end(key)
        return cached

    strips = strips_from_centerline_width_specs(points, specs)
    _PLANNED_PATH_STRIP_CACHE[key] = strips
    while len(_PLANNED_PATH_STRIP_CACHE) > PLANNED_PATH_STRIP_CACHE_LIMIT:
        _PLANNED_PATH_STRIP_CACHE.popitem(last=False)
    return strips


def lane_marking_strip_groups_from_segments(
    segments: tuple[tuple[Vec3, ...], ...],
    specs: tuple[tuple[int, Color, float], ...],
) -> tuple[tuple[MeshStrip, ...], ...]:
    if not segments or not specs:
        return tuple(() for _ in specs)

    half_widths = tuple(max(0.08, width_px * 0.022) * 0.5 for width_px, _, _ in specs)
    heights = tuple(height_m for _, _, height_m in specs)
    left_groups: list[list[Vec3]] = [[] for _ in specs]
    right_groups: list[list[Vec3]] = [[] for _ in specs]
    for segment in segments:
        append_lane_marking_segment_strip_groups(segment, half_widths, heights, left_groups, right_groups)

    return finish_lane_marking_strip_groups(left_groups, right_groups, specs)


def lane_marking_strip_groups_from_centerline(
    centerline: tuple[Vec3, ...],
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
) -> tuple[tuple[MeshStrip, ...], ...]:
    if len(centerline) < 2 or not specs:
        return tuple(() for _ in specs)

    half_widths = tuple(max(0.08, width_px * 0.022) * 0.5 for width_px, _, _ in specs)
    heights = tuple(height_m for _, _, height_m in specs)
    left_groups: list[list[Vec3]] = [[] for _ in specs]
    right_groups: list[list[Vec3]] = [[] for _ in specs]
    if style == "solid":
        append_lane_marking_segment_strip_groups(centerline, half_widths, heights, left_groups, right_groups)
        return finish_lane_marking_strip_groups(left_groups, right_groups, specs)

    dash_m = LANE_DASH_LENGTH_M
    gap_m = LANE_DASH_GAP_M
    cycle_m = dash_m + gap_m
    current_dash: list[Vec3] = []
    distance_m = 0.0
    dash_phase_m = lane_dash_phase_m(centerline, dash_m, gap_m)
    previous = centerline[0]
    eps = 0.0001

    for current in centerline[1:]:
        segment_dx = current.x - previous.x
        segment_dy = current.y - previous.y
        segment_dz = current.z - previous.z
        segment_m = math.sqrt(segment_dx * segment_dx + segment_dy * segment_dy + segment_dz * segment_dz)
        if segment_m <= 0.001:
            previous = current
            continue

        segment_start_m = distance_m
        segment_end_m = distance_m + segment_m
        cursor_m = segment_start_m
        cursor_point = previous

        while cursor_m < segment_end_m - eps:
            cycle_offset_m = lane_dash_cycle_offset(cursor_m, dash_phase_m, cycle_m, dash_m, eps)
            in_dash = cycle_offset_m < dash_m
            boundary_m = cursor_m + (dash_m - cycle_offset_m if in_dash else cycle_m - cycle_offset_m)
            next_m = min(segment_end_m, boundary_m)
            if next_m <= cursor_m + eps:
                next_m = segment_end_m

            next_point = lerp_vec3(previous, current, (next_m - segment_start_m) / segment_m)
            if in_dash:
                append_unique_point(current_dash, cursor_point)
                append_unique_point(current_dash, next_point)
                if boundary_m <= next_m + eps and len(current_dash) >= 2:
                    append_lane_marking_segment_strip_groups(
                        current_dash,
                        half_widths,
                        heights,
                        left_groups,
                        right_groups,
                    )
                    current_dash = []
            elif len(current_dash) >= 2:
                append_lane_marking_segment_strip_groups(
                    current_dash,
                    half_widths,
                    heights,
                    left_groups,
                    right_groups,
                )
                current_dash = []

            cursor_m = next_m
            cursor_point = next_point

        distance_m = segment_end_m
        previous = current

    if len(current_dash) >= 2:
        append_lane_marking_segment_strip_groups(current_dash, half_widths, heights, left_groups, right_groups)

    return finish_lane_marking_strip_groups(left_groups, right_groups, specs)


def append_lane_marking_segment_strip_groups(
    segment: tuple[Vec3, ...] | list[Vec3],
    half_widths: tuple[float, ...],
    heights: tuple[float, ...],
    left_groups: list[list[Vec3]],
    right_groups: list[list[Vec3]],
) -> None:
    point_count = len(segment)
    if point_count < 2:
        return
    vec3 = Vec3
    sqrt = math.sqrt
    group_specs = tuple(zip(half_widths, heights, left_groups, right_groups))
    last_index = point_count - 1
    for index, point in enumerate(segment):
        if index == 0:
            previous_point = segment[0]
            next_point = segment[1]
        elif index == last_index:
            previous_point = segment[last_index - 1]
            next_point = segment[last_index]
        else:
            previous_point = segment[index - 1]
            next_point = segment[index + 1]

        dx = next_point.x - previous_point.x
        dy = next_point.y - previous_point.y
        length = sqrt(dx * dx + dy * dy)
        if length <= 0.0001:
            right_x = 1.0
            right_y = -0.0
        else:
            inverse_length = 1.0 / length
            right_x = dy * inverse_length
            right_y = -dx * inverse_length

        point_x = point.x
        point_y = point.y
        first_point = index == 0
        for half_width, height_m, left_group, right_group in group_specs:
            left = vec3(point_x - right_x * half_width, point_y - right_y * half_width, height_m)
            right = vec3(point_x + right_x * half_width, point_y + right_y * half_width, height_m)
            if first_point and left_group:
                previous_right = right_group[-1]
                left_group.append(previous_right)
                right_group.append(previous_right)
                left_group.append(left)
                right_group.append(left)
            left_group.append(left)
            right_group.append(right)


def finish_lane_marking_strip_groups(
    left_groups: list[list[Vec3]],
    right_groups: list[list[Vec3]],
    specs: tuple[tuple[int, Color, float], ...],
) -> tuple[tuple[MeshStrip, ...], ...]:
    return tuple(
        (MeshStrip(tuple(left_groups[index]), tuple(right_groups[index]), color),)
        if len(left_groups[index]) >= 2 and len(right_groups[index]) >= 2
        else ()
        for index, (_, color, _) in enumerate(specs)
    )


def cached_or_merged_mesh_strip(strips: tuple[MeshStrip, ...]) -> MeshStrip | None:
    if len(strips) == 1:
        return strips[0]

    first = strips[0]
    key = tuple((id(strip.left), id(strip.right)) for strip in strips)
    cached = _MERGED_MESH_STRIP_CACHE.get(key)
    if cached is not None:
        cached_refs, merged_left, merged_right = cached
        if len(cached_refs) == len(strips) and all(
            left_ref is strip.left and right_ref is strip.right
            for (left_ref, right_ref), strip in zip(cached_refs, strips)
        ):
            _MERGED_MESH_STRIP_CACHE.move_to_end(key)
            return MeshStrip(
                left=merged_left,
                right=merged_right,
                color=first.color,
                x_offset_m=first.x_offset_m,
            )
        _MERGED_MESH_STRIP_CACHE.pop(key, None)

    left_group: list[Vec3] = []
    right_group: list[Vec3] = []
    for strip in strips:
        count = min(len(strip.left), len(strip.right))
        if count < 2:
            continue
        if left_group:
            previous_right = right_group[-1]
            next_left = strip.left[0]
            left_group.append(previous_right)
            right_group.append(previous_right)
            left_group.append(next_left)
            right_group.append(next_left)
        left_group.extend(strip.left[:count])
        right_group.extend(strip.right[:count])

    if len(left_group) < 2 or len(right_group) < 2:
        return None

    merged_left = tuple(left_group)
    merged_right = tuple(right_group)
    _MERGED_MESH_STRIP_CACHE[key] = (
        tuple((strip.left, strip.right) for strip in strips),
        merged_left,
        merged_right,
    )
    merged = MeshStrip(
        left=merged_left,
        right=merged_right,
        color=first.color,
        x_offset_m=first.x_offset_m,
    )
    while len(_MERGED_MESH_STRIP_CACHE) > MERGED_MESH_STRIP_CACHE_LIMIT:
        _MERGED_MESH_STRIP_CACHE.popitem(last=False)
    return merged


def merge_mesh_strips_by_style(strips: Iterable[MeshStrip]) -> tuple[MeshStrip, ...]:
    groups: OrderedDict[tuple[Color, float], list[MeshStrip]] = OrderedDict()
    for strip in strips:
        if min(len(strip.left), len(strip.right)) < 2:
            continue
        key = (strip.color, strip.x_offset_m)
        group = groups.get(key)
        if group is None:
            groups[key] = [strip]
        else:
            group.append(strip)

    merged: list[MeshStrip] = []
    for group in groups.values():
        if len(group) == 1:
            merged.append(group[0])
        else:
            group_tuple = tuple(group)
            merged_strip = cached_or_merged_mesh_strip(group_tuple)
            if merged_strip is None:
                merged.extend(group_tuple)
            else:
                merged.append(merged_strip)
    return tuple(merged)


def model_line_geometry_specs(
    specs: tuple[tuple[int, Color, float], ...],
) -> ModelLineStripGeometrySpecs:
    return tuple((width_px, height_m) for width_px, _, height_m in specs)


def model_line_placeholder_specs(
    specs: ModelLineStripGeometrySpecs,
) -> tuple[tuple[int, Color, float], ...]:
    return tuple((width_px, MODEL_LINE_STRIP_GROUP_CACHE_COLOR, height_m) for width_px, height_m in specs)


def model_line_cache_start_m(start_m: float) -> float:
    grid_m = MODEL_LINE_STRIP_GROUP_CACHE_GRID_M
    return math.floor(start_m / grid_m) * grid_m


def model_line_cache_end_m(end_m: float) -> float:
    grid_m = MODEL_LINE_STRIP_GROUP_CACHE_GRID_M
    return math.ceil(end_m / grid_m) * grid_m


def cache_grid_value(value: float, grid_m: float) -> float:
    return round(value / grid_m) * grid_m


def model_line_cache_point_key(model_points: tuple[ModelPathPoint, ...]) -> ModelLineStripPointKey:
    grid_scale = 1.0 / MODEL_LINE_STRIP_GROUP_CACHE_POINT_GRID_M
    round_value = round
    return tuple(
        (
            round_value(point.forward_m * grid_scale),
            round_value(point.lateral_m * grid_scale),
        )
        for point in model_points
    )


def model_line_render_points_and_key(
    model_points: tuple[ModelPathPoint, ...],
    point_limit: int,
) -> tuple[tuple[ModelPathPoint, ...], ModelLineStripPointKey]:
    cache_key = (id(model_points), int(point_limit))
    cached = _MODEL_LINE_RENDER_POINT_KEY_CACHE.get(cache_key)
    if cached is not None:
        cached_model_points, render_points, point_key = cached
        if cached_model_points is model_points:
            _MODEL_LINE_RENDER_POINT_KEY_CACHE.move_to_end(cache_key)
            return render_points, point_key
        _MODEL_LINE_RENDER_POINT_KEY_CACHE.pop(cache_key, None)

    point_count = len(model_points)
    if point_limit <= 0 or point_count <= point_limit:
        render_points = model_points
    else:
        last_index = point_count - 1
        selected: list[ModelPathPoint] = []
        previous_index = -1
        for output_index in range(point_limit):
            index = round(output_index * last_index / (point_limit - 1))
            if index == previous_index:
                continue
            selected.append(model_points[index])
            previous_index = index
        render_points = tuple(selected)

    point_key = model_line_cache_point_key(render_points)
    _MODEL_LINE_RENDER_POINT_KEY_CACHE[cache_key] = (model_points, render_points, point_key)
    while len(_MODEL_LINE_RENDER_POINT_KEY_CACHE) > MODEL_LINE_RENDER_POINT_KEY_CACHE_LIMIT:
        _MODEL_LINE_RENDER_POINT_KEY_CACHE.popitem(last=False)
    return render_points, point_key


def model_line_points_for_render(model_points: tuple[ModelPathPoint, ...]) -> tuple[ModelPathPoint, ...]:
    render_points, _ = model_line_render_points_and_key(model_points, MODEL_LINE_RENDER_POINT_LIMIT)
    return render_points


def cached_model_line_strip_groups(
    model_points: tuple[ModelPathPoint, ...],
    start_m: float,
    end_m: float,
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
    extend_before_model: bool,
    profile_add: ProfileAdd | None = None,
    profile_prefix: str = "scene.model_line",
) -> ModelLineStripGroups:
    cache_start_m = model_line_cache_start_m(start_m)
    cache_end_m = model_line_cache_end_m(end_m)
    geometry_specs = model_line_geometry_specs(specs)
    profile_stage = profile_scene_start(profile_add)
    render_points, point_key = model_line_render_points_and_key(model_points, MODEL_LINE_RENDER_POINT_LIMIT)
    profile_scene_add(profile_add, f"{profile_prefix}.key", profile_stage)
    key = (
        point_key,
        cache_start_m,
        cache_end_m,
        style,
        extend_before_model,
        geometry_specs,
    )
    cached = _MODEL_LINE_STRIP_GROUP_CACHE.get(key)
    if cached is not None:
        _MODEL_LINE_STRIP_GROUP_CACHE.move_to_end(key)
        profile_scene_add_elapsed(profile_add, f"{profile_prefix}.hit", 0.0)
        return cached

    profile_scene_add_elapsed(profile_add, f"{profile_prefix}.miss", 0.0)
    profile_stage = profile_scene_start(profile_add)
    centerline = model_line_centerline(render_points, cache_start_m, cache_end_m, 0.0)
    profile_scene_add(profile_add, f"{profile_prefix}.centerline", profile_stage)
    if len(centerline) < 2:
        groups: ModelLineStripGroups = None if extend_before_model else tuple(() for _ in geometry_specs)
    else:
        if extend_before_model:
            profile_stage = profile_scene_start(profile_add)
            centerline = extend_model_centerline_rearward(centerline, cache_start_m)
            profile_scene_add(profile_add, f"{profile_prefix}.extend", profile_stage)
        profile_stage = profile_scene_start(profile_add)
        groups = lane_marking_strip_groups_from_centerline(
            centerline,
            model_line_placeholder_specs(geometry_specs),
            style,
        )
        profile_scene_add(profile_add, f"{profile_prefix}.strips", profile_stage)

    _MODEL_LINE_STRIP_GROUP_CACHE[key] = groups
    while len(_MODEL_LINE_STRIP_GROUP_CACHE) > MODEL_LINE_STRIP_GROUP_CACHE_LIMIT:
        _MODEL_LINE_STRIP_GROUP_CACHE.popitem(last=False)
    return groups


def cached_lane_offset_strip_groups(
    offset: float,
    steering: float,
    lane_width_m: float,
    start_m: float,
    end_m: float,
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
) -> tuple[tuple[MeshStrip, ...], ...]:
    cache_offset = cache_grid_value(offset, LANE_OFFSET_STRIP_CACHE_OFFSET_GRID)
    cache_steering = cache_grid_value(steering, LANE_OFFSET_STRIP_CACHE_STEERING_GRID)
    cache_lane_width_m = max(
        0.1,
        cache_grid_value(lane_width_m, LANE_OFFSET_STRIP_CACHE_LANE_WIDTH_GRID_M),
    )
    cache_start_m = model_line_cache_start_m(start_m)
    cache_end_m = model_line_cache_end_m(end_m)
    geometry_specs = model_line_geometry_specs(specs)
    key = (
        cache_offset,
        cache_steering,
        cache_lane_width_m,
        cache_start_m,
        cache_end_m,
        style,
        geometry_specs,
    )
    cached = _LANE_OFFSET_STRIP_CACHE.get(key)
    if cached is not None:
        _LANE_OFFSET_STRIP_CACHE.move_to_end(key)
        return cached

    placeholder_specs = model_line_placeholder_specs(geometry_specs)
    if style == "solid":
        centerline = lane_centerline(
            cache_offset,
            cache_steering,
            cache_lane_width_m,
            cache_start_m,
            cache_end_m,
            STATIC_LINE_STEPS,
            0.0,
        )
        groups = lane_marking_strip_groups_from_segments((centerline,), placeholder_specs)
    else:
        segments: list[tuple[Vec3, ...]] = []
        dash_m = LANE_DASH_LENGTH_M
        cycle_m = dash_m + LANE_DASH_GAP_M
        cursor = dashed_lane_start_cursor_m(cache_start_m, dash_m, LANE_DASH_GAP_M)
        while cursor < cache_end_m:
            dash_start = max(cursor, cache_start_m)
            dash_end = min(cursor + dash_m, cache_end_m)
            if dash_end > dash_start + 0.001:
                segments.append(
                    lane_centerline(
                        cache_offset,
                        cache_steering,
                        cache_lane_width_m,
                        dash_start,
                        dash_end,
                        6,
                        0.0,
                    )
                )
            cursor += cycle_m
        groups = lane_marking_strip_groups_from_segments(tuple(segments), placeholder_specs)

    _LANE_OFFSET_STRIP_CACHE[key] = groups
    while len(_LANE_OFFSET_STRIP_CACHE) > LANE_OFFSET_STRIP_CACHE_LIMIT:
        _LANE_OFFSET_STRIP_CACHE.popitem(last=False)
    return groups


def lane_offset_strip_groups(
    offset: float,
    steering: float,
    lane_width_m: float,
    start_m: float,
    end_m: float,
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
) -> tuple[tuple[MeshStrip, ...], ...]:
    groups = cached_lane_offset_strip_groups(
        offset,
        steering,
        lane_width_m,
        start_m,
        end_m,
        specs,
        style,
    )
    return style_mesh_strip_groups(groups, specs, 0.0)


def style_mesh_strip_groups(
    groups: tuple[tuple[MeshStrip, ...], ...],
    specs: tuple[tuple[int, Color, float], ...],
    shift_x_m: float,
) -> tuple[tuple[MeshStrip, ...], ...]:
    return style_mesh_strip_groups_by_shift(groups, specs, tuple(shift_x_m for _ in specs))


def style_mesh_strip_groups_by_shift(
    groups: MeshStripGroups,
    specs: tuple[tuple[int, Color, float], ...],
    shift_x_m_by_group: tuple[float, ...],
) -> MeshStripGroups:
    cache_key = (id(groups), specs, shift_x_m_by_group)
    cached = _STYLE_MESH_STRIP_GROUP_CACHE.get(cache_key)
    if cached is not None:
        cached_groups, styled_groups = cached
        if cached_groups is groups:
            _STYLE_MESH_STRIP_GROUP_CACHE.move_to_end(cache_key)
            return styled_groups
        _STYLE_MESH_STRIP_GROUP_CACHE.pop(cache_key, None)

    styled_groups: list[tuple[MeshStrip, ...]] = []
    for group_index, group in enumerate(groups):
        color = specs[group_index][1]
        shift_x_m = shift_x_m_by_group[group_index] if group_index < len(shift_x_m_by_group) else 0.0
        has_shift = abs(shift_x_m) > 0.0001
        styled_group: list[MeshStrip] = []
        for strip in group:
            if not has_shift and strip.color == color:
                styled_group.append(strip)
            else:
                styled_group.append(
                    MeshStrip(
                        left=strip.left,
                        right=strip.right,
                        color=color,
                        x_offset_m=strip.x_offset_m + shift_x_m,
                    )
                )
        styled_groups.append(tuple(styled_group))
    result = tuple(styled_groups)
    _STYLE_MESH_STRIP_GROUP_CACHE[cache_key] = (groups, result)
    while len(_STYLE_MESH_STRIP_GROUP_CACHE) > STYLE_MESH_STRIP_GROUP_CACHE_LIMIT:
        _STYLE_MESH_STRIP_GROUP_CACHE.popitem(last=False)
    return result


def model_line_strip_groups(
    model_points: tuple[ModelPathPoint, ...],
    lateral_shift_m: float,
    start_m: float,
    end_m: float,
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
    extend_before_model: bool,
    profile_add: ProfileAdd | None = None,
    profile_prefix: str = "scene.model_line",
) -> ModelLineStripGroups:
    groups = cached_model_line_strip_groups(
        model_points,
        start_m,
        end_m,
        specs,
        style,
        extend_before_model,
        profile_add,
        profile_prefix,
    )
    if groups is None:
        return None
    profile_stage = profile_scene_start(profile_add)
    styled_groups = style_mesh_strip_groups(groups, specs, lateral_shift_m)
    profile_scene_add(profile_add, f"{profile_prefix}.style", profile_stage)
    return styled_groups


def planned_path_lane_offset(state: ClusterUiState, forward_m: float) -> float:
    start_offset = 0.0
    target_offset = 0.0
    if state.lane_change is not None:
        start_offset = clamp(state.ego_lane_offset, -1.25, 1.25)
        target_offset = state.highlight_lane_offset if state.highlight_lane_offset is not None else 0.0

    if state.lane_change is None:
        return 0.0

    blend = smoothstep(
        (forward_m - PATH_LANE_CHANGE_CURVE_START_M)
        / (PATH_LANE_CHANGE_CURVE_END_M - PATH_LANE_CHANGE_CURVE_START_M)
    )
    return start_offset + (target_offset - start_offset) * blend


def planned_path_end_m(state: ClusterUiState, blockers: tuple[PathBlocker, ...]) -> float:
    end_m = PATH_END_M
    for blocker in blockers:
        if blocker.forward_m <= PATH_START_M:
            continue
        path_offset = planned_path_lane_offset(state, blocker.forward_m)
        if abs(path_offset - blocker.offset) > PATH_BLOCKER_LANE_TOLERANCE:
            continue
        stop_m = blocker.forward_m - blocker.length_m * 0.5 - PATH_BLOCKER_CLEARANCE_M
        end_m = min(end_m, max(PATH_START_M + 0.6, stop_m))
    return end_m


def model_path_lateral_at_forward(state: ClusterUiState, relative_forward_m: float) -> float | None:
    if not state.model_path or relative_forward_m < 0.0:
        return None

    previous = state.model_path[0]
    if relative_forward_m <= previous.forward_m:
        return previous.lateral_m

    for point in state.model_path[1:]:
        if relative_forward_m <= point.forward_m:
            span = max(0.001, point.forward_m - previous.forward_m)
            amount = clamp((relative_forward_m - previous.forward_m) / span, 0.0, 1.0)
            return previous.lateral_m + (point.lateral_m - previous.lateral_m) * amount
        previous = point
    return None


def model_path_world_x(state: ClusterUiState, lane_width_m: float, forward_m: float) -> float | None:
    lateral_m = model_path_lateral_at_forward(state, scene_data_relative_forward_m(forward_m))
    if lateral_m is None:
        return None
    ego_offset = clamp(state.ego_lane_offset, -1.25, 1.25)
    ego_x_m = road_world_x(ego_offset, EGO_FORWARD_M, state.steering, lane_width_m)
    return ego_x_m + lateral_m


def model_path_end_m(state: ClusterUiState, lane_width_m: float, blockers: tuple[PathBlocker, ...]) -> float | None:
    if len(state.model_path) < 2:
        return None
    last_forward_m = state.model_path[-1].forward_m
    end_m = min(PATH_END_M, data_scene_forward_m(last_forward_m))
    if end_m <= PATH_START_M + 0.6:
        return None

    for blocker in blockers:
        if blocker.forward_m <= PATH_START_M:
            continue
        path_x_m = model_path_world_x(state, lane_width_m, blocker.forward_m)
        if path_x_m is None:
            continue
        blocker_x_m = road_world_x(blocker.offset, blocker.forward_m, state.steering, lane_width_m)
        if abs(path_x_m - blocker_x_m) > PATH_BLOCKER_LANE_TOLERANCE * lane_width_m:
            continue
        stop_m = blocker.forward_m - blocker.length_m * 0.5 - PATH_BLOCKER_CLEARANCE_M
        end_m = min(end_m, max(PATH_START_M + 0.6, stop_m))
    return end_m


def model_path_centerline(
    state: ClusterUiState,
    lane_width_m: float,
    blockers: tuple[PathBlocker, ...],
) -> tuple[Vec3, ...]:
    end_m = model_path_end_m(state, lane_width_m, blockers)
    if end_m is None:
        return ()
    relative_start_m = max(0.0, scene_data_relative_forward_m(PATH_START_M))
    relative_end_m = max(relative_start_m, scene_data_relative_forward_m(end_m))
    model_points = tuple(
        point
        for point in state.model_path
        if relative_start_m <= point.forward_m <= relative_end_m
    )
    if len(model_points) < 2:
        steps = max(4, int(PLANNED_PATH_FALLBACK_STEPS * (end_m - PATH_START_M) / (PATH_END_M - PATH_START_M)))
        sample_points = sample_range(PATH_START_M, end_m, steps)
        points: list[Vec3] = []
        for forward_m in sample_points:
            x_m = model_path_world_x(state, lane_width_m, forward_m)
            if x_m is not None:
                points.append(Vec3(x_m, forward_m, PATH_HEIGHT_M))
        return tuple(points) if len(points) >= 2 else ()
    else:
        ego_offset = clamp(state.ego_lane_offset, -1.25, 1.25)
        ego_x_m = road_world_x(ego_offset, EGO_FORWARD_M, state.steering, lane_width_m)
        points = [
            Vec3(ego_x_m + point.lateral_m, data_scene_forward_m(point.forward_m), PATH_HEIGHT_M)
            for point in model_points
        ]
    return tuple(points) if len(points) >= 2 else ()


def planned_path_strips(
    state: ClusterUiState,
    lane_width_m: float,
    blockers: tuple[PathBlocker, ...],
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
    profile_add: ProfileAdd | None = None,
) -> tuple[MeshStrip, ...]:
    profile_stage = profile_scene_start(profile_add)
    points = model_path_centerline(state, lane_width_m, blockers)
    model_driven = bool(points)
    if not points:
        end_m = planned_path_end_m(state, blockers)
        steps = max(4, int(PLANNED_PATH_FALLBACK_STEPS * (end_m - PATH_START_M) / (PATH_END_M - PATH_START_M)))
        centerline: list[Vec3] = []
        for forward_m in sample_range(PATH_START_M, end_m, steps):
            lane_offset = planned_path_lane_offset(state, forward_m)
            centerline.append(
                Vec3(
                    road_world_x(lane_offset, forward_m, state.steering, lane_width_m),
                    forward_m,
                    PATH_HEIGHT_M,
                )
            )
        points = tuple(centerline)
    profile_scene_add(profile_add, "scene.build.planned_path.centerline", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    path_specs: list[tuple[float, Color, float]] = [(0.86, theme.path_shadow, PATH_SHADOW_LAYER_M)]
    if model_driven:
        uncertainty_width = model_path_uncertainty_width(state)
        if uncertainty_width is not None:
            path_specs.append((uncertainty_width, theme.path_uncertainty, PATH_UNCERTAINTY_LAYER_M))
    path_specs.append((0.46, theme.path_body, PATH_BODY_LAYER_M))
    path_specs.append((0.16, theme.path_highlight, PATH_HIGHLIGHT_LAYER_M))
    strips = list(cached_strips_from_centerline_width_specs(points, tuple(path_specs)))
    profile_scene_add(profile_add, "scene.build.planned_path.strips", profile_stage)

    if model_driven:
        highlight_strip = strips.pop() if strips else None
        profile_stage = profile_scene_start(profile_add)
        strips.extend(model_path_metric_strips(state, points))
        profile_scene_add(profile_add, "scene.build.planned_path.metrics", profile_stage)
        if highlight_strip is not None:
            strips.append(highlight_strip)
    profile_stage = profile_scene_start(profile_add)
    strips.extend(follow_distance_marker_strips(state, points, lane_width_m))
    profile_scene_add(profile_add, "scene.build.planned_path.follow_distance", profile_stage)
    return tuple(strips)


def follow_distance_marker_strips(
    state: ClusterUiState,
    points: tuple[Vec3, ...],
    lane_width_m: float,
) -> tuple[MeshStrip, ...]:
    distance_m = state.longitudinal_desired_distance_m
    if distance_m is None or distance_m <= 0.0 or len(points) < 2:
        return ()
    forward_m = render_scene_forward_m(distance_m)
    if forward_m < points[0].y or forward_m > points[-1].y:
        return ()
    center_x_m = centerline_x_at_forward(points, forward_m)
    if center_x_m is None:
        return ()
    half_width_m = lane_width_m * 0.5

    def marker_strip(half_forward_m: float, extra_width_m: float, height_m: float, color: Color) -> MeshStrip:
        left_x_m = center_x_m - half_width_m - extra_width_m
        right_x_m = center_x_m + half_width_m + extra_width_m
        near_m = forward_m - half_forward_m
        far_m = forward_m + half_forward_m
        return MeshStrip(
            left=(
                Vec3(left_x_m, near_m, height_m),
                Vec3(left_x_m, far_m, height_m),
            ),
            right=(
                Vec3(right_x_m, near_m, height_m),
                Vec3(right_x_m, far_m, height_m),
            ),
            color=color,
        )

    return (
        marker_strip(
            FOLLOW_DISTANCE_MARKER_BACKING_FORWARD_M,
            FOLLOW_DISTANCE_MARKER_BACKING_EXTRA_WIDTH_M,
            FOLLOW_DISTANCE_MARKER_BACKING_LAYER_M,
            FOLLOW_DISTANCE_MARKER_BACKING_COLOR,
        ),
        marker_strip(
            FOLLOW_DISTANCE_MARKER_BODY_FORWARD_M,
            0.0,
            FOLLOW_DISTANCE_MARKER_BODY_LAYER_M,
            FOLLOW_DISTANCE_MARKER_BODY_COLOR,
        ),
    )


def centerline_x_at_forward(points: tuple[Vec3, ...], forward_m: float) -> float | None:
    if not points:
        return None
    previous = points[0]
    if forward_m <= previous.y:
        return previous.x
    for point in points[1:]:
        if forward_m <= point.y:
            span = max(0.001, point.y - previous.y)
            amount = clamp((forward_m - previous.y) / span, 0.0, 1.0)
            return previous.x + (point.x - previous.x) * amount
        previous = point
    return points[-1].x


def model_path_uncertainty_width(state: ClusterUiState) -> float | None:
    std_values = [point.lateral_std_m for point in state.model_path if point.lateral_std_m is not None]
    if not std_values:
        return None
    average_std = sum(std_values[:16]) / min(len(std_values), 16)
    return clamp(0.68 + average_std * 0.42, 0.72, 1.85)


def model_path_metric_strips(state: ClusterUiState, points: tuple[Vec3, ...]) -> tuple[MeshStrip, ...]:
    if len(state.model_path) < 2 or len(points) < 2:
        return ()
    strips: list[MeshStrip] = []
    metric_count = min(len(state.model_path), len(points))
    segment_count = min(metric_count - 1, MODEL_PATH_METRIC_SEGMENT_LIMIT)
    if segment_count <= 0:
        return ()
    source_segment_count = metric_count - 1
    for segment_index in range(segment_count):
        index = round(segment_index * source_segment_count / max(1, segment_count - 1))
        index = min(index, len(points) - 2)
        model_index = min(
            len(state.model_path) - 1,
            round(index * (len(state.model_path) - 1) / max(1, metric_count - 1)),
        )
        accel = state.model_path[model_index].accel_mps2
        if accel is None:
            continue
        color = path_metric_color(accel)
        segment = (
            Vec3(points[index].x, points[index].y, PATH_METRIC_LAYER_M),
            Vec3(points[index + 1].x, points[index + 1].y, PATH_METRIC_LAYER_M),
        )
        strips.append(strip_from_centerline(segment, 0.24, color))
    return tuple(strips)


def path_metric_color(accel_mps2: float) -> Color:
    if accel_mps2 <= -2.4:
        return RED[0], RED[1], RED[2], 210
    if accel_mps2 <= -0.7:
        return AMBER[0], AMBER[1], AMBER[2], 190
    if accel_mps2 >= 0.7:
        return 18, 184, 108, 170
    return 70, 152, 255, 145


def radar_points_for_display(state: ClusterUiState) -> tuple[RadarPoint, ...]:
    if state.radar_display_mode == CLUSTER_RADAR_DISPLAY_DETAIL:
        return state.radar_points
    return merged_radar_points(state.radar_points, state)


def merged_radar_points(points: tuple[RadarPoint, ...], state: ClusterUiState) -> tuple[RadarPoint, ...]:
    if len(points) < 2:
        return points
    groups: list[list[RadarPoint]] = []
    centroids: list[RadarPoint] = []
    for point in sorted(points, key=lambda item: (item.longitudinal_m, item.lateral_m, item.label)):
        match_index = next(
            (
                index
                for index, centroid in enumerate(centroids)
                if radar_points_are_mergeable(point, centroid, state)
            ),
            None,
        )
        if match_index is None:
            groups.append([point])
            centroids.append(point)
            continue
        groups[match_index].append(point)
        centroids[match_index] = merged_radar_point(groups[match_index], state)
    merged = tuple(centroid if len(group) > 1 else group[0] for centroid, group in zip(centroids, groups))
    return tuple(sorted(merged, key=lambda item: (item.longitudinal_m, abs(item.lateral_m), item.label)))


def radar_points_are_mergeable(left: RadarPoint, right: RadarPoint, state: ClusterUiState) -> bool:
    distance_m = max(abs(left.longitudinal_m), abs(right.longitudinal_m))
    longitudinal_tolerance = max(
        RADAR_POINT_MERGE_BASE_LONGITUDINAL_M,
        min(RADAR_POINT_MERGE_MAX_LONGITUDINAL_M, distance_m * 0.018),
    )
    if abs(left.longitudinal_m - right.longitudinal_m) > longitudinal_tolerance:
        return False
    if abs(left.lateral_m - right.lateral_m) > RADAR_POINT_MERGE_LATERAL_M:
        return False
    left_speed = radar_point_absolute_speed_kph(left, state)
    right_speed = radar_point_absolute_speed_kph(right, state)
    if (
        left_speed is not None
        and right_speed is not None
        and abs(left_speed - right_speed) > RADAR_POINT_MERGE_SPEED_KPH
    ):
        return False
    return True


def merged_radar_point(points: list[RadarPoint], state: ClusterUiState) -> RadarPoint:
    first = points[0]
    label = first.label if len(points) == 1 else f"{first.label}+{len(points) - 1}"
    source = first.source if all(point.source == first.source for point in points) else "merged"
    return RadarPoint(
        label=label,
        longitudinal_m=average_float(point.longitudinal_m for point in points),
        lateral_m=average_float(point.lateral_m for point in points),
        source=source,
        relative_speed_mps=average_optional_float(point.relative_speed_mps for point in points),
        absolute_speed_kph=average_optional_float(radar_point_absolute_speed_kph(point, state) for point in points),
        lateral_speed_mps=average_optional_float(point.lateral_speed_mps for point in points),
        relative_accel_mps2=average_optional_float(point.relative_accel_mps2 for point in points),
        probability=average_optional_float(point.probability for point in points),
        valid=max_optional_int(point.valid for point in points),
        valid_count=max_optional_int(point.valid_count for point in points),
        in_my_lane=max_optional_int(point.in_my_lane for point in points),
        motion_consistent=merged_radar_motion_consistent(point.motion_consistent for point in points),
        promotion_held=any(point.promotion_held for point in points),
    )


def average_float(values: Iterable[float]) -> float:
    numbers = [float(value) for value in values]
    return sum(numbers) / max(1, len(numbers))


def average_optional_float(values: Iterable[float | None]) -> float | None:
    numbers = [float(value) for value in values if value is not None]
    if not numbers:
        return None
    return sum(numbers) / len(numbers)


def max_optional_int(values: Iterable[int | None]) -> int | None:
    numbers = [int(value) for value in values if value is not None]
    return max(numbers) if numbers else None


def merged_radar_motion_consistent(values: Iterable[bool | None]) -> bool | None:
    has_true = False
    has_known = False
    for value in values:
        if value is None:
            continue
        has_known = True
        if not value:
            return False
        has_true = True
    if not has_known:
        return None
    return has_true


def radar_point_markers(
    state: ClusterUiState,
    lane_width_m: float,
    vehicle_points: tuple[RadarPoint, ...] = (),
    min_forward_m: float = ROAD_NEAR_M,
    max_forward_m: float = ROAD_FAR_M + 30.0,
    x_offset_m: float = 0.0,
) -> tuple[RadarPointMarker, ...]:
    markers: list[RadarPointMarker] = []
    for point in state.radar_points:
        if radar_point_hidden_by_vehicle_box(point, vehicle_points, state):
            continue
        forward_m = render_scene_forward_m(point.longitudinal_m)
        if forward_m < min_forward_m or forward_m > max_forward_m:
            continue
        color = radar_point_color(point)
        absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
        markers.append(
            RadarPointMarker(
                center=Vec3(
                    clamp(point.lateral_m, -lane_width_m * 3.0, lane_width_m * 3.0) + x_offset_m,
                    forward_m,
                    0.20,
                ),
                radius_m=radar_point_radius(point),
                color=color,
                label=point.label,
                longitudinal_m=point.longitudinal_m,
                lateral_m=point.lateral_m,
                relative_speed_mps=point.relative_speed_mps,
                absolute_speed_kph=absolute_speed_kph,
                lateral_speed_mps=point.lateral_speed_mps,
                relative_accel_mps2=point.relative_accel_mps2,
                probability=point.probability,
                valid=point.valid,
                in_my_lane=point.in_my_lane,
            )
        )
    return tuple(markers)


def radar_point_hidden_by_vehicle_box(
    point: RadarPoint,
    vehicle_points: tuple[RadarPoint, ...],
    state: ClusterUiState,
) -> bool:
    if state.radar_display_mode == CLUSTER_RADAR_DISPLAY_DETAIL:
        return any(point.label == vehicle_point.label for vehicle_point in vehicle_points)
    return any(radar_points_same_vehicle(point, vehicle_point) for vehicle_point in vehicle_points)


def radar_vehicle_points(state: ClusterUiState, lane_width_m: float) -> tuple[RadarPoint, ...]:
    selected: list[RadarPoint] = []
    candidates = sorted(
        (
            point
            for point in state.radar_points
            if radar_point_is_vehicle_candidate(point, state, lane_width_m)
        ),
        key=lambda point: (
            0 if radar_point_matches_detected_vehicle(point, state) else 1,
            point.longitudinal_m,
            abs(point.lateral_m),
        ),
    )
    if state.radar_display_mode == CLUSTER_RADAR_DISPLAY_DETAIL:
        return tuple(
            sorted(
                candidates,
                key=lambda point: (point.longitudinal_m, abs(point.lateral_m), point.label),
            )
        )
    for point in candidates:
        if any(radar_points_same_vehicle(point, existing) for existing in selected):
            continue
        selected.append(point)
    selected.sort(key=lambda point: point.longitudinal_m)
    return tuple(selected)


def detected_vehicles_with_merged_radar(
    vehicles: tuple[DetectedVehicle, ...],
    radar_points: tuple[RadarPoint, ...],
    state: ClusterUiState,
) -> tuple[DetectedVehicle, ...]:
    if not vehicles or not radar_points:
        return vehicles
    merged: list[DetectedVehicle] = []
    used_radar_labels: set[str] = set()
    for vehicle in vehicles:
        point = radar_merge_point_for_vehicle(
            vehicle,
            tuple(point for point in radar_points if point.label not in used_radar_labels),
            state,
        )
        if point is None:
            merged.append(vehicle)
            continue
        used_radar_labels.add(point.label)
        absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
        merged.append(
            replace(
                vehicle,
                source=f"{vehicle.source}{RADAR_MERGED_SOURCE_TAG}{point.label}",
                relative_speed_mps=vehicle.relative_speed_mps
                if vehicle.relative_speed_mps is not None
                else point.relative_speed_mps,
                absolute_speed_kph=vehicle.absolute_speed_kph
                if vehicle.absolute_speed_kph is not None
                else absolute_speed_kph,
                acceleration_mps2=vehicle.acceleration_mps2
                if vehicle.acceleration_mps2 is not None
                else point.relative_accel_mps2,
                ttc_s=vehicle.ttc_s
                if vehicle.ttc_s is not None
                else ttc_from_relative_speed(vehicle.longitudinal_m, point.relative_speed_mps),
            )
        )
    return tuple(merged)


def detected_vehicles_for_display(
    vehicles: tuple[DetectedVehicle, ...],
    state: ClusterUiState,
) -> tuple[DetectedVehicle, ...]:
    if state.radar_display_mode == CLUSTER_RADAR_DISPLAY_DETAIL or len(vehicles) < 2:
        return vehicles
    selected: list[DetectedVehicle] = []
    for vehicle in sorted(vehicles, key=detected_vehicle_display_priority):
        match_index = next(
            (
                index
                for index, existing in enumerate(selected)
                if detected_vehicles_same_front_vehicle(vehicle, existing)
            ),
            None,
        )
        if match_index is None:
            selected.append(vehicle)
        else:
            selected[match_index] = merge_detected_vehicle_for_display(selected[match_index], vehicle)
    return tuple(sorted(selected, key=lambda vehicle: vehicle.longitudinal_m))


def detected_vehicle_display_priority(vehicle: DetectedVehicle) -> tuple[int, float, float]:
    if vehicle.source == "radarState":
        source_priority = 0
    elif vehicle.primary:
        source_priority = 1
    elif vehicle.source.startswith("modelV2"):
        source_priority = 2
    elif vehicle.source == "carState" or vehicle.source.startswith("CAN 0x"):
        source_priority = 3
    else:
        source_priority = 4
    return source_priority, vehicle.longitudinal_m, -vehicle.probability


def detected_vehicles_same_front_vehicle(left: DetectedVehicle, right: DetectedVehicle) -> bool:
    if not (detected_vehicle_is_front_merge_candidate(left) and detected_vehicle_is_front_merge_candidate(right)):
        return False
    if detected_vehicle_base_source(left) == detected_vehicle_base_source(right):
        return False
    distance_m = max(left.longitudinal_m, right.longitudinal_m)
    longitudinal_tolerance = max(
        RADAR_FRONT_DETECTED_MERGE_LONGITUDINAL_MIN_M,
        min(RADAR_FRONT_DETECTED_MERGE_LONGITUDINAL_MAX_M, distance_m * 0.20),
    )
    return (
        abs(left.longitudinal_m - right.longitudinal_m) <= longitudinal_tolerance
        and abs(left.lateral_m - right.lateral_m) <= RADAR_FRONT_DETECTED_MERGE_LATERAL_M
    )


def detected_vehicle_is_front_merge_candidate(vehicle: DetectedVehicle) -> bool:
    if vehicle.longitudinal_m <= 0.0 or vehicle.label in CORNER_RADAR_LABELS:
        return False
    return (
        detected_vehicle_is_front_lead(vehicle)
        or vehicle.source.startswith("modelV2")
        or vehicle.primary
    )


def detected_vehicle_base_source(vehicle: DetectedVehicle) -> str:
    return vehicle_source_base(vehicle.source)


def vehicle_source_base(source: str) -> str:
    return source.split(RADAR_MERGED_SOURCE_TAG, 1)[0]


def vehicle_source_is_adas(source: str) -> bool:
    base_source = vehicle_source_base(source)
    return base_source == "carState" or base_source in ("CAN 0x162", "CAN 0x1ea")


def vehicle_source_is_camera(source: str) -> bool:
    return vehicle_source_base(source).startswith("camera")


def vehicle_source_is_front_radar(source: str) -> bool:
    return vehicle_source_base(source) == "radarState"


def vehicle_source_is_radar_track(source: str) -> bool:
    return source in ("radarPoint", "liveTracks") or RADAR_MERGED_SOURCE_TAG in source


def merge_detected_vehicle_for_display(base: DetectedVehicle, other: DetectedVehicle) -> DetectedVehicle:
    return replace(
        base,
        source=merged_detected_vehicle_source(base.source, other.source),
        probability=max(base.probability, other.probability),
        relative_speed_mps=base.relative_speed_mps if base.relative_speed_mps is not None else other.relative_speed_mps,
        absolute_speed_kph=base.absolute_speed_kph if base.absolute_speed_kph is not None else other.absolute_speed_kph,
        acceleration_mps2=base.acceleration_mps2 if base.acceleration_mps2 is not None else other.acceleration_mps2,
        cut_in=base.cut_in or other.cut_in,
        primary=base.primary or other.primary,
        ttc_s=base.ttc_s if base.ttc_s is not None else other.ttc_s,
        x_std_m=base.x_std_m if base.x_std_m is not None else other.x_std_m,
        y_std_m=base.y_std_m if base.y_std_m is not None else other.y_std_m,
    )


def merged_detected_vehicle_source(base_source: str, other_source: str) -> str:
    if vehicle_source_is_adas(base_source):
        return base_source
    if vehicle_source_is_adas(other_source):
        return other_source
    if vehicle_source_is_front_radar(base_source):
        return base_source
    if vehicle_source_is_front_radar(other_source):
        return other_source
    if vehicle_source_is_radar_track(base_source):
        return base_source
    if vehicle_source_is_radar_track(other_source):
        return other_source
    return base_source


def radar_merge_point_for_vehicle(
    vehicle: DetectedVehicle,
    radar_points: tuple[RadarPoint, ...],
    state: ClusterUiState,
) -> RadarPoint | None:
    if not detected_vehicle_needs_radar_merge(vehicle):
        return None
    candidates = tuple(
        point
        for point in radar_points
        if radar_point_can_fill_vehicle_speed(point, state) and radar_point_can_merge_with_vehicle(point, vehicle)
    )
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda point: (
            abs(point.longitudinal_m - vehicle.longitudinal_m),
            abs(point.lateral_m - vehicle.lateral_m),
        ),
    )


def detected_vehicle_needs_radar_merge(vehicle: DetectedVehicle) -> bool:
    if vehicle.source.startswith("modelV2") and vehicle.longitudinal_m > 0.0:
        return True
    return (
        vehicle.label in CORNER_RADAR_LABELS
        and vehicle.absolute_speed_kph is None
        and (vehicle.source == "carState" or vehicle.source.startswith("CAN 0x"))
    )


def radar_point_can_fill_vehicle_speed(point: RadarPoint, state: ClusterUiState) -> bool:
    return radar_point_absolute_speed_kph(point, state) is not None or point.relative_speed_mps is not None


def radar_point_can_merge_with_vehicle(point: RadarPoint, vehicle: DetectedVehicle) -> bool:
    if vehicle.source.startswith("modelV2"):
        return radar_point_close_to_detected_vehicle(point, vehicle)
    return radar_point_close_to_vehicle(point, vehicle)


def radar_point_close_to_vehicle(point: RadarPoint, vehicle: DetectedVehicle) -> bool:
    longitudinal_tolerance = max(
        RADAR_MERGE_LONGITUDINAL_MIN_M,
        min(RADAR_MERGE_LONGITUDINAL_MAX_M, vehicle.longitudinal_m * 0.08),
    )
    return (
        abs(point.longitudinal_m - vehicle.longitudinal_m) <= longitudinal_tolerance
        and abs(point.lateral_m - vehicle.lateral_m) <= RADAR_MERGE_LATERAL_M
    )


def merged_radar_point_label(vehicle: DetectedVehicle) -> str | None:
    if RADAR_MERGED_SOURCE_TAG not in vehicle.source:
        return None
    return vehicle.source.rsplit(RADAR_MERGED_SOURCE_TAG, 1)[1] or None


def ttc_from_relative_speed(longitudinal_m: float, relative_speed_mps: float | None) -> float | None:
    if relative_speed_mps is None or relative_speed_mps >= -0.15 or longitudinal_m <= 0.0:
        return None
    return min(99.9, longitudinal_m / max(0.15, -relative_speed_mps))


def radar_points_same_vehicle(left: RadarPoint, right: RadarPoint) -> bool:
    return (
        abs(left.longitudinal_m - right.longitudinal_m) <= RADAR_VEHICLE_DEDUP_LONGITUDINAL_M
        and abs(left.lateral_m - right.lateral_m) <= RADAR_VEHICLE_DEDUP_LATERAL_M
    )


def radar_vehicle_box(
    point: RadarPoint,
    state: ClusterUiState,
    lane_width_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> VehicleBox:
    confidence = radar_vehicle_confidence(point)
    alpha = int(92 + 163 * confidence)
    body_color = vehicle_color_for_source("radarPoint", theme, state.radar_source_color_mode)
    forward_m = render_scene_forward_m(point.longitudinal_m)
    center_x_m = clamp(point.lateral_m, -lane_width_m * 3.0, lane_width_m * 3.0)
    return VehicleBox(
        center=Vec3(center_x_m, forward_m, VEHICLE_HEIGHT_M * 0.5),
        right_x=1.0,
        right_y=0.0,
        forward_x=0.0,
        forward_y=1.0,
        width_m=VEHICLE_WIDTH_M,
        length_m=VEHICLE_LENGTH_M,
        height_m=VEHICLE_HEIGHT_M,
        body_color=rgba(body_color, alpha),
        side_color=rgba(darken(body_color, 0.20), alpha),
        rear_color=rgba(darken(body_color, 0.28), alpha),
        top_highlight=rgba(lighten(body_color, 0.16), min(235, alpha)),
        outline_color=rgba(darken(body_color, 0.42), min(235, alpha)),
        confidence=confidence,
        label=point.label,
        source="radarPoint",
        longitudinal_m=point.longitudinal_m,
        relative_speed_mps=point.relative_speed_mps,
        absolute_speed_kph=radar_point_absolute_speed_kph(point, state),
        acceleration_mps2=point.relative_accel_mps2,
        annotate=False,
    )


def radar_point_is_vehicle_candidate(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> bool:
    if not 2.5 <= point.longitudinal_m <= RADAR_VEHICLE_MAX_DISTANCE_M:
        return False
    if abs(point.lateral_m) > lane_width_m * RADAR_VEHICLE_MAX_LATERAL_LANES:
        return False
    if radar_point_is_confirmed_vehicle_source(point):
        return True
    if point.valid_count is not None:
        if point.valid_count < RADAR_VEHICLE_MIN_VALID_COUNT:
            return False
        if point.motion_consistent is not True:
            return False
    if point.promotion_held:
        return True
    outside_road_edge_m = radar_point_road_edge_outside_distance_m(point, state, lane_width_m)
    stable_edge_vehicle = (
        radar_point_has_stable_edge_vehicle_motion(point, state, lane_width_m)
        and radar_point_is_near_or_outside_road_edge(point, state, lane_width_m, outside_road_edge_m)
    )
    if (
        point.probability is not None
        and point.probability < 0.20
        and not point.in_my_lane
        and not stable_edge_vehicle
    ):
        return False
    keep_across_road_edge = radar_point_should_keep_across_road_edge(
        point,
        state,
        lane_width_m,
        outside_road_edge_m,
    )
    if (
        outside_road_edge_m is not None
        and outside_road_edge_m > RADAR_ROAD_EDGE_OUTSIDE_MARGIN_M
        and not keep_across_road_edge
    ):
        return False
    if radar_point_matches_detected_vehicle(point, state):
        return True
    if radar_point_has_vehicle_estimate(point, state, lane_width_m):
        return True
    if radar_point_is_stationary_object(point, state):
        return False
    if radar_point_is_side_static_reflection(point, state, lane_width_m):
        return False
    if radar_point_matches_static_road_edge(point, state, lane_width_m) and not keep_across_road_edge:
        return False
    if radar_point_is_moving_raw_vehicle(point, state, lane_width_m):
        return True
    return False


def radar_point_is_confirmed_vehicle_source(point: RadarPoint) -> bool:
    source = point.source.lower()
    return "0x162" in source or "0x1ea" in source


def radar_point_source_is_radar_track(point: RadarPoint) -> bool:
    return point.source == "liveTracks"


def radar_point_has_vehicle_estimate(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> bool:
    if radar_point_matches_detected_vehicle(point, state):
        return True
    if point.in_my_lane is not None and point.in_my_lane > 0:
        return abs(point.lateral_m) <= lane_width_m * RADAR_PROBABLE_VEHICLE_LATERAL_LANES
    if point.probability is not None and point.probability >= RADAR_VEHICLE_MIN_PROBABILITY:
        return abs(point.lateral_m) <= lane_width_m * RADAR_PROBABLE_VEHICLE_LATERAL_LANES
    return False


def radar_point_is_moving_raw_vehicle(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> bool:
    absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
    if absolute_speed_kph is None or absolute_speed_kph < RADAR_RAW_MOVING_SPEED_KPH:
        return False
    valid_count = point.valid_count if point.valid_count is not None else RADAR_RAW_CENTER_MIN_VALID_COUNT
    lateral_lanes = abs(point.lateral_m) / max(0.1, lane_width_m)
    if lateral_lanes <= RADAR_CENTER_RAW_LATERAL_LANES:
        return valid_count >= RADAR_RAW_CENTER_MIN_VALID_COUNT
    if lateral_lanes <= RADAR_ADJACENT_RAW_LATERAL_LANES:
        return valid_count >= RADAR_RAW_ADJACENT_MIN_VALID_COUNT
    if lateral_lanes <= RADAR_OUTER_RAW_LATERAL_LANES:
        return valid_count >= RADAR_RAW_OUTER_MIN_VALID_COUNT
    return False


def radar_point_should_keep_across_road_edge(
    point: RadarPoint,
    state: ClusterUiState,
    lane_width_m: float,
    outside_road_edge_m: float | None,
) -> bool:
    stable_edge_vehicle = radar_point_has_stable_edge_vehicle_motion(point, state, lane_width_m)
    if outside_road_edge_m is not None and outside_road_edge_m > RADAR_ROAD_EDGE_KEEP_OUTSIDE_MARGIN_M:
        return (
            outside_road_edge_m <= RADAR_ROAD_EDGE_STABLE_VEHICLE_OUTSIDE_MARGIN_M
            and stable_edge_vehicle
        )
    valid_count = point.valid_count if point.valid_count is not None else 0
    if valid_count < RADAR_ROAD_EDGE_KEEP_MIN_VALID_COUNT:
        return False
    absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
    if absolute_speed_kph is None or absolute_speed_kph < RADAR_ROAD_EDGE_KEEP_SPEED_KPH:
        return False
    if not radar_point_is_near_or_outside_road_edge(point, state, lane_width_m, outside_road_edge_m):
        return False
    return (
        stable_edge_vehicle
        or radar_point_has_vehicle_estimate(point, state, lane_width_m)
        or radar_point_is_moving_raw_vehicle(point, state, lane_width_m)
    )


def radar_point_has_stable_edge_vehicle_motion(
    point: RadarPoint,
    state: ClusterUiState,
    lane_width_m: float,
) -> bool:
    valid_count = point.valid_count if point.valid_count is not None else 0
    if valid_count < RADAR_ROAD_EDGE_STABLE_VEHICLE_MIN_VALID_COUNT:
        return False
    absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
    if absolute_speed_kph is None or absolute_speed_kph < RADAR_ROAD_EDGE_KEEP_SPEED_KPH:
        return False
    if (
        point.relative_accel_mps2 is not None
        and abs(point.relative_accel_mps2) > RADAR_ROAD_EDGE_STABLE_VEHICLE_MAX_ACCEL_MPS2
    ):
        return False
    return abs(point.lateral_m) <= lane_width_m * RADAR_VEHICLE_MAX_LATERAL_LANES


def radar_point_is_near_or_outside_road_edge(
    point: RadarPoint,
    state: ClusterUiState,
    lane_width_m: float,
    outside_road_edge_m: float | None,
) -> bool:
    if outside_road_edge_m is not None and outside_road_edge_m > 0.0:
        return True
    edge_distance_m = radar_point_road_edge_distance_m(point, state, lane_width_m)
    return edge_distance_m is not None and edge_distance_m <= RADAR_ROAD_EDGE_STATIONARY_CLEARANCE_M


def radar_point_matches_detected_vehicle(point: RadarPoint, state: ClusterUiState) -> bool:
    for vehicle in state.detected_vehicles:
        if radar_point_close_to_detected_vehicle(point, vehicle):
            return True
    return False


def radar_point_hidden_by_detected_vehicle(
    point: RadarPoint,
    vehicles: tuple[DetectedVehicle, ...],
    state: ClusterUiState,
) -> bool:
    if state.radar_display_mode == CLUSTER_RADAR_DISPLAY_DETAIL:
        return False
    for vehicle in vehicles:
        if radar_point_close_to_detected_vehicle(point, vehicle):
            return True
    return False


def radar_point_close_to_detected_vehicle(point: RadarPoint, vehicle: DetectedVehicle) -> bool:
    if detected_vehicle_is_front_lead(vehicle):
        longitudinal_tolerance = max(
            RADAR_FRONT_DETECTED_MERGE_LONGITUDINAL_MIN_M,
            min(
                RADAR_FRONT_DETECTED_MERGE_LONGITUDINAL_MAX_M,
                max(point.longitudinal_m, vehicle.longitudinal_m) * 0.20,
            ),
        )
        lateral_tolerance = RADAR_FRONT_DETECTED_MERGE_LATERAL_M
    else:
        longitudinal_tolerance = max(4.0, min(8.0, point.longitudinal_m * 0.08))
        lateral_tolerance = 1.35
    return (
        abs(point.longitudinal_m - vehicle.longitudinal_m) <= longitudinal_tolerance
        and abs(point.lateral_m - vehicle.lateral_m) <= lateral_tolerance
    )


def detected_vehicle_is_front_lead(vehicle: DetectedVehicle) -> bool:
    return vehicle.source == "radarState" or vehicle.label in ("TARGET", "TARGET2")


def radar_point_absolute_speed_kph(point: RadarPoint, state: ClusterUiState) -> float | None:
    if point.absolute_speed_kph is not None:
        return point.absolute_speed_kph
    if point.relative_speed_mps is None:
        return None
    return max(0.0, state.speed_kph + point.relative_speed_mps * 3.6)


def radar_point_is_stationary_object(point: RadarPoint, state: ClusterUiState) -> bool:
    absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
    return absolute_speed_kph is not None and absolute_speed_kph <= RADAR_STATIC_OBJECT_SPEED_KPH


def radar_point_is_side_static_reflection(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> bool:
    if state.speed_kph < RADAR_EGO_MOVING_SPEED_KPH:
        return False
    if point.in_my_lane is not None and point.in_my_lane > 0:
        return False
    if point.probability is not None and point.probability >= RADAR_VEHICLE_MIN_PROBABILITY:
        return False
    if abs(point.lateral_m) <= lane_width_m * RADAR_SIDE_STATIC_LATERAL_LANES:
        return False
    if abs(point.relative_speed_mps or 0.0) > RADAR_STATIC_OBJECT_SPEED_MPS:
        return False
    lateral_lanes = abs(point.lateral_m) / max(0.1, lane_width_m)
    valid_count = point.valid_count if point.valid_count is not None else 0
    if lateral_lanes <= RADAR_ADJACENT_RAW_LATERAL_LANES and valid_count >= RADAR_RAW_ADJACENT_MIN_VALID_COUNT:
        return False
    if lateral_lanes <= RADAR_OUTER_RAW_LATERAL_LANES and valid_count >= RADAR_RAW_OUTER_MIN_VALID_COUNT:
        return False
    return True


def radar_point_matches_static_road_edge(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> bool:
    edge_distance = radar_point_road_edge_distance_m(point, state, lane_width_m)
    if edge_distance is None:
        return False
    if edge_distance <= RADAR_ROAD_EDGE_HARD_CLEARANCE_M:
        return True
    rel_speed = abs(point.relative_speed_mps or 0.0)
    absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
    absolute_static = absolute_speed_kph is not None and absolute_speed_kph <= RADAR_STATIC_OBJECT_SPEED_KPH
    relative_static = rel_speed <= RADAR_STATIC_OBJECT_SPEED_MPS
    return edge_distance <= RADAR_ROAD_EDGE_STATIONARY_CLEARANCE_M and (absolute_static or relative_static)


def radar_point_is_outside_road_edges(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> bool:
    outside_m = radar_point_road_edge_outside_distance_m(point, state, lane_width_m)
    return outside_m is not None and outside_m > RADAR_ROAD_EDGE_OUTSIDE_MARGIN_M


def radar_point_road_edge_outside_distance_m(
    point: RadarPoint,
    state: ClusterUiState,
    lane_width_m: float,
) -> float | None:
    left_edge_m = road_edge_lateral_at(
        state.left_road_edge_points,
        state.left_road_edge_lateral_shift_m,
        state.left_road_edge_offset,
        point.longitudinal_m,
        lane_width_m,
    )
    right_edge_m = road_edge_lateral_at(
        state.right_road_edge_points,
        state.right_road_edge_lateral_shift_m,
        state.right_road_edge_offset,
        point.longitudinal_m,
        lane_width_m,
    )
    if left_edge_m is not None and right_edge_m is not None and left_edge_m >= right_edge_m:
        return None
    outside_m = 0.0
    if left_edge_m is not None and point.lateral_m < left_edge_m:
        outside_m = max(outside_m, left_edge_m - point.lateral_m)
    if right_edge_m is not None and point.lateral_m > right_edge_m:
        outside_m = max(outside_m, point.lateral_m - right_edge_m)
    return outside_m


def road_edge_lateral_at(
    edge_points: tuple[ModelPathPoint, ...],
    lateral_shift_m: float,
    edge_offset: float | None,
    forward_m: float,
    lane_width_m: float,
) -> float | None:
    edge_lateral = model_line_lateral_at(edge_points, forward_m, lateral_shift_m)
    if edge_lateral is not None:
        return edge_lateral
    if edge_offset is not None:
        return edge_offset * lane_width_m
    return None



def radar_point_road_edge_distance_m(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> float | None:
    distances: list[float] = []
    for edge_points, lateral_shift_m in (
        (state.left_road_edge_points, state.left_road_edge_lateral_shift_m),
        (state.right_road_edge_points, state.right_road_edge_lateral_shift_m),
    ):
        edge_lateral = model_line_lateral_at(edge_points, point.longitudinal_m, lateral_shift_m)
        if edge_lateral is not None:
            distances.append(abs(point.lateral_m - edge_lateral))
    for edge_offset in (state.left_road_edge_offset, state.right_road_edge_offset):
        if edge_offset is not None:
            distances.append(abs(point.lateral_m - edge_offset * lane_width_m))
    return min(distances) if distances else None


def model_line_lateral_at(
    points: tuple[ModelPathPoint, ...],
    forward_m: float,
    lateral_shift_m: float = 0.0,
) -> float | None:
    if not points:
        return None
    ordered = points
    if forward_m <= ordered[0].forward_m:
        return ordered[0].lateral_m + lateral_shift_m
    for left, right in zip(ordered, ordered[1:]):
        if left.forward_m <= forward_m <= right.forward_m:
            span = max(0.001, right.forward_m - left.forward_m)
            amount = clamp((forward_m - left.forward_m) / span, 0.0, 1.0)
            return left.lateral_m + (right.lateral_m - left.lateral_m) * amount + lateral_shift_m
    return ordered[-1].lateral_m + lateral_shift_m


def radar_vehicle_confidence(point: RadarPoint) -> float:
    if point.probability is not None:
        return clamp(0.58 + point.probability * 0.38, 0.58, 0.96)
    if point.valid_count is not None:
        return clamp(0.56 + min(point.valid_count, 120) / 120.0 * 0.36, 0.56, 0.92)
    return 0.72


def radar_point_color(point: RadarPoint) -> Color:
    if point.valid is not None and point.valid <= 0:
        return 116, 126, 136, 150
    if point.longitudinal_m < 12.0 and abs(point.lateral_m) < 1.6:
        return RED[0], RED[1], RED[2], 232
    if point.probability is not None and point.probability < 0.25:
        return 116, 126, 136, 150
    if radar_point_source_is_radar_track(point):
        return AMBER[0], AMBER[1], AMBER[2], 226
    if point.in_my_lane is not None and point.in_my_lane > 0:
        return BLUE[0], BLUE[1], BLUE[2], 226
    if point.relative_speed_mps is not None and point.relative_speed_mps < -2.5:
        return AMBER[0], AMBER[1], AMBER[2], 226
    return 34, 150, 255, 208


def radar_point_radius(point: RadarPoint) -> float:
    probability = point.probability if point.probability is not None else 0.72
    return clamp(0.105 + 0.07 * probability, 0.095, 0.19)


def vehicle_box(
    offset: float,
    forward_m: float,
    steering: float,
    lane_width_m: float,
    color: tuple[int, int, int],
    camera_active: bool,
    target_offset: float | None = None,
    confidence: float = 1.0,
    label: str = "",
    source: str = "",
    longitudinal_m: float | None = None,
    relative_speed_mps: float | None = None,
    absolute_speed_kph: float | None = None,
    acceleration_mps2: float | None = None,
    ttc_s: float | None = None,
    cut_in: bool = False,
    primary: bool = False,
    annotate: bool = False,
    x_offset_m: float = 0.0,
) -> VehicleBox:
    confidence = clamp(confidence, 0.0, 1.0)
    alpha = int(92 + 163 * confidence)
    body_color = color
    center_x_m = road_world_x(offset, forward_m, steering, lane_width_m) + x_offset_m
    right_x, right_y, forward_x, forward_y = vehicle_heading(
        offset,
        forward_m,
        steering,
        lane_width_m,
        target_offset,
    )
    width_m = VEHICLE_WIDTH_M
    length_m = VEHICLE_LENGTH_M
    height_m = VEHICLE_HEIGHT_M
    actual_longitudinal_m = (
        scene_data_relative_forward_m(forward_m)
        if longitudinal_m is None
        else longitudinal_m
    )

    return VehicleBox(
        center=Vec3(center_x_m, forward_m, height_m * 0.5),
        right_x=right_x,
        right_y=right_y,
        forward_x=forward_x,
        forward_y=forward_y,
        width_m=width_m,
        length_m=length_m,
        height_m=height_m,
        body_color=rgba(body_color, alpha),
        side_color=rgba(darken(body_color, 0.20), alpha),
        rear_color=rgba(darken(body_color, 0.28), alpha),
        top_highlight=rgba(lighten(body_color, 0.16), min(235, alpha)),
        outline_color=rgba(darken(body_color, 0.42), min(235, alpha)),
        confidence=confidence,
        label=label,
        source=source,
        longitudinal_m=actual_longitudinal_m,
        relative_speed_mps=relative_speed_mps,
        absolute_speed_kph=absolute_speed_kph,
        acceleration_mps2=acceleration_mps2,
        ttc_s=ttc_s,
        cut_in=cut_in,
        primary=primary,
        annotate=annotate,
    )


def ego_anchor_x_m(state: ClusterUiState, lane_width_m: float) -> float:
    ego_offset = clamp(state.ego_lane_offset, -1.25, 1.25)
    return road_world_x(ego_offset, EGO_FORWARD_M, state.steering, lane_width_m)


def scene_camera(state: ClusterUiState, lane_width_m: float, anchor_x_m: float = 0.0) -> CameraSpec:
    ego_x_m = ego_anchor_x_m(state, lane_width_m) - anchor_x_m
    ego_y_m = EGO_FORWARD_M
    if state.camera_view_mode == CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM:
        drive_camera = CameraSpec(
            position=Vec3(*DRIVE_CAMERA_EGO_BOTTOM_POSITION_M),
            target=Vec3(*DRIVE_CAMERA_EGO_BOTTOM_TARGET_M),
            fovy_deg=44.0,
        )
    else:
        drive_camera = CameraSpec(
            position=Vec3(0.0, -16.0 + DRIVE_CAMERA_FORWARD_SHIFT_M, 6.00),
            target=Vec3(0.0, 7.0 + DRIVE_CAMERA_FORWARD_SHIFT_M, -0.20),
            fovy_deg=44.0,
        )

    if not state.surround_view_active:
        return drive_camera

    yaw_rad = math.radians(clamp(state.surround_yaw_deg, -SURROUND_MAX_YAW_DEG, SURROUND_MAX_YAW_DEG))
    orbit_forward_x = math.sin(yaw_rad)
    orbit_forward_y = math.cos(yaw_rad)
    pitch_deg = clamp(state.surround_pitch_deg, -SURROUND_MAX_PITCH_DEG, SURROUND_MAX_PITCH_DEG)
    camera_height_m = SURROUND_CAMERA_HEIGHT_M + pitch_deg * 0.035
    target_forward_m = SURROUND_TARGET_FORWARD_M + pitch_deg * 0.12

    orbit_camera = CameraSpec(
        position=Vec3(
            ego_x_m - orbit_forward_x * SURROUND_CAMERA_DISTANCE_M,
            ego_y_m - orbit_forward_y * SURROUND_CAMERA_DISTANCE_M,
            camera_height_m,
        ),
        target=Vec3(
            ego_x_m + orbit_forward_x * target_forward_m,
            ego_y_m + orbit_forward_y * target_forward_m,
            SURROUND_TARGET_HEIGHT_M,
        ),
        fovy_deg=40.0,
    )
    orbit_amount = smoothstep(
        max(
            abs(state.surround_yaw_deg) / SURROUND_MAX_YAW_DEG,
            abs(state.surround_pitch_deg) / SURROUND_MAX_PITCH_DEG,
        )
    )
    return blend_camera(drive_camera, orbit_camera, orbit_amount)


def blend_vec3(start: Vec3, end: Vec3, amount: float) -> Vec3:
    return Vec3(
        start.x + (end.x - start.x) * amount,
        start.y + (end.y - start.y) * amount,
        start.z + (end.z - start.z) * amount,
    )


def blend_camera(start: CameraSpec, end: CameraSpec, amount: float) -> CameraSpec:
    amount = clamp(amount, 0.0, 1.0)
    return CameraSpec(
        position=blend_vec3(start.position, end.position, amount),
        target=blend_vec3(start.target, end.target, amount),
        fovy_deg=start.fovy_deg + (end.fovy_deg - start.fovy_deg) * amount,
    )


def translate_vec3_x(point: Vec3, shift_x_m: float) -> Vec3:
    return Vec3(point.x + shift_x_m, point.y, point.z)


def translate_vehicle_box_x(vehicle: VehicleBox, shift_x_m: float) -> VehicleBox:
    if abs(shift_x_m) <= 0.0001:
        return vehicle
    return VehicleBox(
        center=translate_vec3_x(vehicle.center, shift_x_m),
        right_x=vehicle.right_x,
        right_y=vehicle.right_y,
        forward_x=vehicle.forward_x,
        forward_y=vehicle.forward_y,
        width_m=vehicle.width_m,
        length_m=vehicle.length_m,
        height_m=vehicle.height_m,
        body_color=vehicle.body_color,
        side_color=vehicle.side_color,
        rear_color=vehicle.rear_color,
        top_highlight=vehicle.top_highlight,
        outline_color=vehicle.outline_color,
        confidence=vehicle.confidence,
        label=vehicle.label,
        source=vehicle.source,
        longitudinal_m=vehicle.longitudinal_m,
        relative_speed_mps=vehicle.relative_speed_mps,
        absolute_speed_kph=vehicle.absolute_speed_kph,
        acceleration_mps2=vehicle.acceleration_mps2,
        ttc_s=vehicle.ttc_s,
        cut_in=vehicle.cut_in,
        primary=vehicle.primary,
        annotate=vehicle.annotate,
    )


def translate_radar_marker_x(marker: RadarPointMarker, shift_x_m: float) -> RadarPointMarker:
    if abs(shift_x_m) <= 0.0001:
        return marker
    return RadarPointMarker(
        center=translate_vec3_x(marker.center, shift_x_m),
        radius_m=marker.radius_m,
        color=marker.color,
        label=marker.label,
        longitudinal_m=marker.longitudinal_m,
        lateral_m=marker.lateral_m,
        relative_speed_mps=marker.relative_speed_mps,
        absolute_speed_kph=marker.absolute_speed_kph,
        lateral_speed_mps=marker.lateral_speed_mps,
        relative_accel_mps2=marker.relative_accel_mps2,
        probability=marker.probability,
        valid=marker.valid,
        in_my_lane=marker.in_my_lane,
    )


def road_surface_offsets(state: ClusterUiState, route_mode: bool) -> tuple[float, float]:
    road_shift = state.road_view_lane_position if route_mode else 0.0
    left = road_shift - 0.92
    right = road_shift + 0.92
    if state.lane_change == "left":
        left = min(left, road_shift - 1.55)
    elif state.lane_change == "right":
        right = max(right, road_shift + 1.55)
    if state.extra_left_lane_visible and state.lane_change == "left":
        left = min(road_shift - 1.55, state.left_road_edge_offset if state.left_road_edge_offset is not None else road_shift - 1.9)
    elif route_mode and state.left_road_edge_offset is not None:
        left = min(left, max(state.left_road_edge_offset, -1.25))
    if state.extra_right_lane_visible and state.lane_change == "right":
        right = max(road_shift + 1.55, state.right_road_edge_offset if state.right_road_edge_offset is not None else road_shift + 1.9)
    elif route_mode and state.right_road_edge_offset is not None:
        right = max(right, min(state.right_road_edge_offset, 1.25))
    return clamp(left, -2.8, -0.68), clamp(right, 0.68, 2.8)


def road_edge_color(
    distance_m: float | None,
    confidence: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> Color:
    confidence = clamp(confidence, 0.0, 1.0)
    if distance_m is not None and distance_m < 0.85:
        base = RED
        alpha = 165 + int(80 * confidence)
    elif distance_m is not None and distance_m < 1.35:
        base = AMBER
        alpha = 145 + int(80 * confidence)
    else:
        base = theme.road_edge
        alpha = 150 + int(70 * confidence)
    return base[0], base[1], base[2], int(clamp(alpha, 120, 245))


def road_edge_3d_layers(
    color: Color,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> tuple[RoadEdgeLayer, ...]:
    base_rgb = color[:3]
    alpha = color[3]
    shadow_alpha = max(theme.road_edge_backing[3], int(alpha * 0.58))
    return (
        (
            22,
            rgba_with_alpha(darken(base_rgb, 0.58), shadow_alpha),
            ROAD_EDGE_SHADOW_HEIGHT_M,
            ROAD_EDGE_OUTSIDE_SHADOW_OFFSET_M,
        ),
        (
            14,
            rgba_with_alpha(darken(base_rgb, 0.30), alpha * 0.88),
            ROAD_EDGE_BODY_HEIGHT_M,
            ROAD_EDGE_BODY_OFFSET_M,
        ),
        (
            8,
            color,
            ROAD_EDGE_HEIGHT_M,
            0.0,
        ),
        (
            3,
            rgba_with_alpha(lighten(base_rgb, 0.38), alpha + 26),
            ROAD_EDGE_CREST_HEIGHT_M,
            ROAD_EDGE_CREST_OFFSET_M,
        ),
    )


def road_edge_layer_specs(layers: tuple[RoadEdgeLayer, ...]) -> tuple[tuple[int, Color, float], ...]:
    return tuple((width_px, color, height_m) for width_px, color, height_m, _ in layers)


def road_edge_layer_geometry_specs(layers: tuple[RoadEdgeLayer, ...]) -> RoadEdgeOffsetLayerGeometrySpecs:
    return tuple((width_px, height_m, lateral_offset_m) for width_px, _, height_m, lateral_offset_m in layers)


def road_edge_model_points_for_render(model_points: tuple[ModelPathPoint, ...]) -> tuple[ModelPathPoint, ...]:
    render_points, _ = model_line_render_points_and_key(model_points, ROAD_EDGE_MODEL_POINT_LIMIT)
    return render_points


def cached_road_edge_offset_strip_groups(
    offset: float,
    steering: float,
    lane_width_m: float,
    side: float,
    start_m: float,
    end_m: float,
    layers: tuple[RoadEdgeLayer, ...],
) -> RoadEdgeOffsetStripGroups:
    cache_offset = cache_grid_value(offset, ROAD_EDGE_OFFSET_STRIP_CACHE_OFFSET_GRID)
    cache_steering = cache_grid_value(steering, ROAD_EDGE_OFFSET_STRIP_CACHE_STEERING_GRID)
    cache_lane_width_m = max(
        0.1,
        cache_grid_value(lane_width_m, ROAD_EDGE_OFFSET_STRIP_CACHE_LANE_WIDTH_GRID_M),
    )
    cache_side = -1.0 if side < 0.0 else 1.0
    cache_start_m = model_line_cache_start_m(start_m)
    cache_end_m = model_line_cache_end_m(end_m)
    geometry_specs = road_edge_layer_geometry_specs(layers)
    key = (
        cache_offset,
        cache_steering,
        cache_lane_width_m,
        cache_side,
        cache_start_m,
        cache_end_m,
        geometry_specs,
    )
    cached = _ROAD_EDGE_OFFSET_STRIP_CACHE.get(key)
    if cached is not None:
        _ROAD_EDGE_OFFSET_STRIP_CACHE.move_to_end(key)
        return cached

    strip_groups: list[tuple[MeshStrip, ...]] = []
    for width_px, height_m, lateral_offset_m in geometry_specs:
        layer_offset = cache_offset + cache_side * lateral_offset_m / cache_lane_width_m
        centerline = lane_centerline(
            layer_offset,
            cache_steering,
            cache_lane_width_m,
            cache_start_m,
            cache_end_m,
            ROAD_EDGE_OFFSET_STEPS,
            0.0,
        )
        (layer_strips,) = lane_marking_strip_groups_from_segments(
            (centerline,),
            ((width_px, MODEL_LINE_STRIP_GROUP_CACHE_COLOR, height_m),),
        )
        strip_groups.append(layer_strips)

    groups = tuple(strip_groups)
    _ROAD_EDGE_OFFSET_STRIP_CACHE[key] = groups
    while len(_ROAD_EDGE_OFFSET_STRIP_CACHE) > ROAD_EDGE_OFFSET_STRIP_CACHE_LIMIT:
        _ROAD_EDGE_OFFSET_STRIP_CACHE.popitem(last=False)
    return groups


def road_edge_model_strips(
    model_points: tuple[ModelPathPoint, ...],
    lateral_shift_m: float,
    color: Color,
    side: float,
    start_m: float,
    end_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
    profile_add: ProfileAdd | None = None,
) -> tuple[MeshStrip, ...]:
    layers = road_edge_3d_layers(color, theme)
    specs = road_edge_layer_specs(layers)
    render_points = (
        model_points if ROAD_EDGE_MODEL_POINT_LIMIT <= 0
        else road_edge_model_points_for_render(model_points)
    )
    groups = cached_model_line_strip_groups(
        render_points,
        start_m,
        end_m,
        specs,
        "solid",
        True,
        profile_add,
        "scene.road_model",
    )
    if groups is None:
        return ()
    shifts = tuple(lateral_shift_m + side * lateral_offset_m for _, _, _, lateral_offset_m in layers)
    profile_stage = profile_scene_start(profile_add)
    styled_strips = tuple(
        strip
        for group in style_mesh_strip_groups_by_shift(groups, specs, shifts)
        for strip in group
    )
    profile_scene_add(profile_add, "scene.road_model.style", profile_stage)
    return styled_strips


def road_edge_offset_strips(
    offset: float,
    steering: float,
    lane_width_m: float,
    color: Color,
    side: float,
    start_m: float,
    end_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> tuple[MeshStrip, ...]:
    layers = road_edge_3d_layers(color, theme)
    groups = cached_road_edge_offset_strip_groups(
        offset,
        steering,
        lane_width_m,
        side,
        start_m,
        end_m,
        layers,
    )
    specs = road_edge_layer_specs(layers)
    styled_groups = style_mesh_strip_groups_by_shift(groups, specs, (0.0,) * len(specs))
    return tuple(
        strip
        for group in styled_groups
        for strip in group
    )


def vehicle_color_for_source(
    source: str,
    theme: ClusterTheme,
    source_color_mode: int,
) -> tuple[int, int, int]:
    if source_color_mode != CLUSTER_RADAR_SOURCE_COLOR_BY_SOURCE:
        return theme.default_vehicle
    if vehicle_source_is_adas(source):
        return GREEN
    if vehicle_source_is_front_radar(source):
        return RED
    if vehicle_source_is_radar_track(source):
        return AMBER
    if vehicle_source_is_camera(source):
        return BLUE_SOFT
    if source.startswith("modelV2"):
        return BLUE
    return theme.default_vehicle


def vehicle_color_for_detection(
    vehicle: DetectedVehicle,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
    source_color_mode: int = 0,
) -> tuple[int, int, int]:
    return vehicle_color_for_source(vehicle.source, theme, source_color_mode)


def vehicle_blocks_path(vehicle: DetectedVehicle) -> bool:
    if vehicle.longitudinal_m <= 0.0:
        return False
    if vehicle.source.startswith("modelV2") and vehicle.probability < 0.35:
        return False
    return True


def vehicle_badge_has_special_info(vehicle: DetectedVehicle) -> bool:
    if vehicle.cut_in:
        return True
    if vehicle.ttc_s is not None and vehicle.ttc_s < VEHICLE_BADGE_TTC_S:
        return True
    return vehicle.acceleration_mps2 is not None and abs(vehicle.acceleration_mps2) > VEHICLE_BADGE_ACCEL_MPS2


def road_edge_strips(
    state: ClusterUiState,
    route_mode: bool,
    lane_width_m: float,
    road_start_m: float,
    road_end_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
    profile_add: ProfileAdd | None = None,
) -> tuple[MeshStrip, ...]:
    def default_road_edge_strips() -> tuple[MeshStrip, ...]:
        default_color = road_edge_color(None, 1.0, theme)
        left_offset, right_offset = road_surface_offsets(state, route_mode)
        return (
            *road_edge_offset_strips(
                left_offset,
                state.steering,
                lane_width_m,
                default_color,
                -1.0,
                road_start_m,
                road_end_m,
                theme,
            ),
            *road_edge_offset_strips(
                right_offset,
                state.steering,
                lane_width_m,
                default_color,
                1.0,
                road_start_m,
                road_end_m,
                theme,
            ),
        )

    if not route_mode:
        return default_road_edge_strips()

    strips: list[MeshStrip] = []
    if state.left_road_edge_offset is not None or state.left_road_edge_points:
        left_color = road_edge_color(state.left_road_edge_distance_m, state.left_road_edge_confidence, theme)
        if state.left_road_edge_points:
            strips.extend(
                road_edge_model_strips(
                    state.left_road_edge_points,
                    state.left_road_edge_lateral_shift_m,
                    left_color,
                    -1.0,
                    road_start_m,
                    road_end_m,
                    theme,
                )
            )
        elif state.left_road_edge_offset is not None:
            strips.extend(
                road_edge_offset_strips(
                    clamp(state.left_road_edge_offset, -2.8, -0.68),
                    state.steering,
                    lane_width_m,
                    left_color,
                    -1.0,
                    road_start_m,
                    road_end_m,
                    theme,
                )
            )
    if state.right_road_edge_offset is not None or state.right_road_edge_points:
        right_color = road_edge_color(state.right_road_edge_distance_m, state.right_road_edge_confidence, theme)
        if state.right_road_edge_points:
            strips.extend(
                road_edge_model_strips(
                    state.right_road_edge_points,
                    state.right_road_edge_lateral_shift_m,
                    right_color,
                    1.0,
                    road_start_m,
                    road_end_m,
                    theme,
                    profile_add,
                )
            )
        elif state.right_road_edge_offset is not None:
            strips.extend(
                road_edge_offset_strips(
                    clamp(state.right_road_edge_offset, 0.68, 2.8),
                    state.steering,
                    lane_width_m,
                    right_color,
                    1.0,
                    road_start_m,
                    road_end_m,
                    theme,
                )
            )
    return tuple(strips)


def profile_scene_start(profile_add: ProfileAdd | None) -> float:
    return time.perf_counter() if profile_add is not None else 0.0


def profile_scene_add(profile_add: ProfileAdd | None, name: str, start_time: float) -> None:
    if profile_add is not None:
        profile_add(name, (time.perf_counter() - start_time) * 1000.0)


def profile_scene_add_elapsed(profile_add: ProfileAdd | None, name: str, elapsed_ms: float) -> None:
    if profile_add is not None:
        profile_add(name, elapsed_ms)


def lane_highlight_color(route_mode: bool) -> Color:
    alpha = LANE_HIGHLIGHT_ROUTE_ALPHA if route_mode else LANE_HIGHLIGHT_ALPHA
    return LANE_HIGHLIGHT_COLOR[0], LANE_HIGHLIGHT_COLOR[1], LANE_HIGHLIGHT_COLOR[2], alpha


def bsd_lane_marking_offsets(state: ClusterUiState) -> tuple[float, ...]:
    offsets: list[float] = []
    if state.left_blindspot:
        lane_center_offset = (
            state.highlight_lane_offset
            if state.highlight_lane == "left" and state.highlight_lane_offset is not None
            else -1.0
        )
        offsets.append(lane_center_offset + 0.5)
    if state.right_blindspot:
        lane_center_offset = (
            state.highlight_lane_offset
            if state.highlight_lane == "right" and state.highlight_lane_offset is not None
            else 1.0
        )
        offsets.append(lane_center_offset - 0.5)
    return tuple(offsets)


def lane_marking_color_for_state(
    marking: LaneMarking,
    bsd_marking_offsets: tuple[float, ...],
) -> tuple[int, int, int]:
    if any(abs(marking.offset - offset) <= BSD_LANE_MARKING_MATCH_TOLERANCE for offset in bsd_marking_offsets):
        return RED
    return marking.color


def data_geometry_mode_for_state(state: ClusterUiState) -> bool:
    return (
        state.route_overlay is not None
        or bool(state.detected_vehicles)
        or bool(state.radar_points)
        or state.left_road_edge_offset is not None
        or state.right_road_edge_offset is not None
        or bool(state.left_road_edge_points)
        or bool(state.right_road_edge_points)
        or any(marking.model_points for marking in state.lanes)
    )


def build_cluster_scene(
    state: ClusterUiState,
    profile_add: ProfileAdd | None = None,
    highlight_lane_lit: bool = True,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> ClusterScene:
    profile_stage = profile_scene_start(profile_add)
    lane_width_m = max(2.4, min(4.6, state.lane_width_m or DEFAULT_LANE_WIDTH_M))
    display_radar_points = radar_points_for_display(state)
    if display_radar_points is not state.radar_points:
        state = replace(state, radar_points=display_radar_points)
    anchor_x_m = ego_anchor_x_m(state, lane_width_m)
    scene_shift_x_m = -anchor_x_m
    relative_scene_x_offset_m = -scene_shift_x_m
    camera = scene_camera(state, lane_width_m, anchor_x_m)
    camera_active = state.surround_view_active
    selected_radar_vehicle_points = radar_vehicle_points(state, lane_width_m)
    selected_radar_vehicle_boxes = tuple(
        radar_vehicle_box(point, state, lane_width_m, theme)
        for point in selected_radar_vehicle_points
    )
    route_mode = data_geometry_mode_for_state(state)
    road_start_m = (
        SURROUND_ROAD_REAR_M if state.surround_view_active
        else DRIVE_VIEW_ROAD_START_M
    )
    road_end_m = (
        SURROUND_ROAD_FRONT_M if state.surround_view_active
        else ROAD_FAR_M
    )
    road_steps = ROAD_STEPS_SURROUND if camera_active else ROAD_STEPS_MODEL if route_mode else ROAD_STEPS_SIM
    if (state.detected_vehicles or selected_radar_vehicle_boxes) and not camera_active:
        nearest_detected_y = min(
            (render_scene_forward_m(vehicle.longitudinal_m) for vehicle in state.detected_vehicles),
            default=ROAD_FAR_M,
        )
        nearest_radar_y = min((vehicle.center.y for vehicle in selected_radar_vehicle_boxes), default=ROAD_FAR_M)
        nearest_detected_y = min(nearest_detected_y, nearest_radar_y)
        candidate_road_start_m = max(-35.0, nearest_detected_y - DRIVE_VIEW_REAR_ROAD_MARGIN_M)
        road_start_m = min(road_start_m, max(DRIVE_VIEW_ROAD_START_M, candidate_road_start_m))
    profile_scene_add(profile_add, "scene.build.setup", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    highlight_lanes: list[MeshStrip] = []
    if state.highlight_lane_offset is not None and highlight_lane_lit:
        highlight_strip = lane_floor_strip(
            state,
            state.highlight_lane_offset,
            lane_highlight_color(route_mode),
            lane_width_m,
            road_start_m,
            road_end_m,
            road_steps,
            route_mode,
            0.006,
        )
        if highlight_strip is not None:
            highlight_lanes.append(highlight_strip)
    profile_scene_add(profile_add, "scene.build.highlight_lanes", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    lane_strips: list[MeshStrip] = []
    lane_model_ms = 0.0
    lane_offset_ms = 0.0
    lane_collect_ms = 0.0
    bsd_marking_offsets = bsd_lane_marking_offsets(state)
    for marking in state.lanes:
        if not marking.visible:
            continue
        marking_specs = (
            (
                marking.width + LANE_MARKING_BORDER_EXTRA_WIDTH_PX,
                theme.lane_marking_border,
                LANE_MARKING_SHADOW_HEIGHT_M,
            ),
            (
                marking.width,
                rgba(lane_marking_color_for_state(marking, bsd_marking_offsets)),
                LANE_MARKING_HEIGHT_M,
            ),
        )
        strip_groups: tuple[tuple[MeshStrip, ...], ...] | None = None
        if marking.model_points:
            profile_step = profile_scene_start(profile_add)
            strip_groups = model_line_strip_groups(
                marking.model_points,
                marking.model_lateral_shift_m,
                road_start_m,
                road_end_m,
                marking_specs,
                marking.style,
                True,
                profile_add,
                "scene.lane_model",
            )
            if profile_add is not None:
                lane_model_ms += (time.perf_counter() - profile_step) * 1000.0
        if strip_groups is None:
            profile_step = profile_scene_start(profile_add)
            strip_groups = lane_offset_strip_groups(
                marking.offset,
                state.steering,
                lane_width_m,
                road_start_m,
                road_end_m,
                marking_specs,
                marking.style,
            )
            if profile_add is not None:
                lane_offset_ms += (time.perf_counter() - profile_step) * 1000.0
        profile_step = profile_scene_start(profile_add)
        backing_strips, foreground_strips = strip_groups
        lane_strips.extend(backing_strips)
        lane_strips.extend(foreground_strips)
        if profile_add is not None:
            lane_collect_ms += (time.perf_counter() - profile_step) * 1000.0
    profile_scene_add_elapsed(profile_add, "scene.build.lane_markings.model", lane_model_ms)
    profile_scene_add_elapsed(profile_add, "scene.build.lane_markings.offset", lane_offset_ms)
    profile_scene_add_elapsed(profile_add, "scene.build.lane_markings.collect", lane_collect_ms)
    profile_merge = profile_scene_start(profile_add)
    lane_markings = merge_mesh_strips_by_style(lane_strips)
    profile_scene_add(profile_add, "scene.build.lane_markings.merge", profile_merge)
    profile_scene_add(profile_add, "scene.build.lane_markings", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    ego_offset = clamp(state.ego_lane_offset, -1.25, 1.25)
    target_offset = state.highlight_lane_offset if state.lane_change_phase == "changing" else None
    ego_vehicle = vehicle_box(
        ego_offset,
        EGO_VEHICLE_CENTER_FORWARD_M,
        state.steering,
        lane_width_m,
        EGO,
        camera_active,
        target_offset,
    )
    merged_radar_labels = frozenset[str]()
    if route_mode:
        if state.radar_display_mode == CLUSTER_RADAR_DISPLAY_DETAIL:
            merged_detected_vehicles = state.detected_vehicles
        else:
            merged_detected_vehicles = detected_vehicles_with_merged_radar(
                state.detected_vehicles,
                state.radar_points,
                state,
            )
            merged_detected_vehicles = detected_vehicles_for_display(merged_detected_vehicles, state)
        render_detected_vehicles = merged_detected_vehicles
        merged_radar_labels = frozenset(
            label
            for label in (merged_radar_point_label(vehicle) for vehicle in render_detected_vehicles)
            if label is not None
        )
        detected_vehicle_boxes = tuple(
            vehicle_box(
                clamp(detected.lateral_m / lane_width_m, -2.2, 2.2),
                render_scene_forward_m(detected.longitudinal_m),
                state.steering,
                lane_width_m,
                vehicle_color_for_detection(detected, theme, state.radar_source_color_mode),
                camera_active,
                confidence=detected.probability,
                label=detected.label,
                source=detected.source,
                longitudinal_m=detected.longitudinal_m,
                relative_speed_mps=detected.relative_speed_mps,
                absolute_speed_kph=detected.absolute_speed_kph
                if detected.absolute_speed_kph is not None
                else (
                    max(0.0, state.speed_kph + detected.relative_speed_mps * 3.6)
                    if detected.relative_speed_mps is not None
                    else None
                ),
                acceleration_mps2=detected.acceleration_mps2,
                ttc_s=detected.ttc_s,
                cut_in=detected.cut_in,
                primary=detected.primary,
                annotate=vehicle_badge_has_special_info(detected),
                x_offset_m=relative_scene_x_offset_m,
            )
            for detected in render_detected_vehicles
        )
        blocking_detected_vehicles = tuple(
            detected for detected in state.detected_vehicles if vehicle_blocks_path(detected)
        )
        detected_blockers = tuple(
            PathBlocker(
                clamp(detected.lateral_m / lane_width_m, -2.2, 2.2),
                render_scene_forward_m(detected.longitudinal_m),
                VEHICLE_LENGTH_M,
            )
            for detected in blocking_detected_vehicles
        )
        visible_radar_vehicle_pairs = tuple(
            (point, box)
            for point, box in zip(selected_radar_vehicle_points, selected_radar_vehicle_boxes)
            if point.label not in merged_radar_labels
            and not radar_point_hidden_by_detected_vehicle(point, render_detected_vehicles, state)
        )
        visible_radar_vehicle_points = tuple(point for point, _ in visible_radar_vehicle_pairs)
        visible_radar_vehicle_boxes_raw = tuple(box for _, box in visible_radar_vehicle_pairs)
        radar_blockers = tuple(
            PathBlocker(
                clamp(vehicle.center.x / lane_width_m, -2.2, 2.2),
                vehicle.center.y,
                vehicle.length_m,
            )
            for vehicle in visible_radar_vehicle_boxes_raw
        )
        visible_radar_vehicle_boxes = tuple(
            vehicle_box_with_x_offset(vehicle, relative_scene_x_offset_m)
            for vehicle in visible_radar_vehicle_boxes_raw
        )
        blockers = (*detected_blockers, *radar_blockers)
        vehicles = (ego_vehicle, *detected_vehicle_boxes, *visible_radar_vehicle_boxes)
    else:
        blockers = ()
        vehicles = (ego_vehicle,)
    profile_scene_add(profile_add, "scene.build.vehicles", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    road_surface = MeshStrip((), (), rgba(theme.road))
    profile_scene_add(profile_add, "scene.build.road_surface", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    profile_geometry = profile_scene_start(profile_add)
    road_edges_raw = road_edge_strips(state, route_mode, lane_width_m, road_start_m, road_end_m, theme, profile_add)
    profile_scene_add(profile_add, "scene.build.road_edges.geometry", profile_geometry)
    profile_merge = profile_scene_start(profile_add)
    road_edges = merge_mesh_strips_by_style(road_edges_raw)
    profile_scene_add(profile_add, "scene.build.road_edges.merge", profile_merge)
    profile_scene_add(profile_add, "scene.build.road_edges", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    planned_path = planned_path_strips(state, lane_width_m, blockers, theme, profile_add)
    profile_scene_add(profile_add, "scene.build.planned_path", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    hidden_merged_radar_points = tuple(point for point in state.radar_points if point.label in merged_radar_labels)
    radar_points = radar_point_markers(
        state,
        lane_width_m,
        (*selected_radar_vehicle_points, *hidden_merged_radar_points),
        min_forward_m=road_start_m,
        max_forward_m=road_end_m if camera_active else ROAD_FAR_M + 30.0,
        x_offset_m=relative_scene_x_offset_m,
    )
    profile_scene_add(profile_add, "scene.build.radar_points", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    scene = ClusterScene(
        camera=camera,
        scene_shift_x_m=scene_shift_x_m,
        road_surface=road_surface,
        road_edges=road_edges,
        highlight_lanes=tuple(highlight_lanes),
        lane_markings=lane_markings,
        planned_path=tuple(planned_path),
        radar_points=tuple(radar_points),
        vehicles=tuple(vehicles),
    )
    profile_scene_add(profile_add, "scene.build.pack", profile_stage)
    return scene
