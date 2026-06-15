from __future__ import annotations

import math
import time
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import dataclass, replace

from cluster_config import (
    AMBER,
    BLUE,
    ClusterTheme,
    DEFAULT_LANE_WIDTH_M,
    EGO,
    EGO_FORWARD_M,
    GREEN,
    LIGHT_CLUSTER_THEME,
    PATH_END_M,
    PATH_HEIGHT_M,
    PATH_LANE_CHANGE_CURVE_END_M,
    PATH_LANE_CHANGE_CURVE_START_M,
    PATH_START_M,
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
ProfileAdd = Callable[[str, float], None]
PATH_BLOCKER_CLEARANCE_M = 1.25
PATH_BLOCKER_LANE_TOLERANCE = 0.42
RADAR_VEHICLE_MIN_VALID_COUNT = 11
RADAR_VEHICLE_MAX_DISTANCE_M = 150.0
RADAR_VEHICLE_MAX_LATERAL_LANES = 2.75
RADAR_ROAD_EDGE_HARD_CLEARANCE_M = 0.55
RADAR_ROAD_EDGE_STATIONARY_CLEARANCE_M = 1.05
RADAR_STATIC_OBJECT_SPEED_MPS = 1.25
RADAR_STATIC_OBJECT_SPEED_KPH = 8.0
RADAR_SIDE_STATIC_LATERAL_LANES = 0.58
RADAR_EGO_MOVING_SPEED_KPH = 10.0
RADAR_CENTER_RAW_LATERAL_LANES = 0.72
RADAR_ADJACENT_RAW_LATERAL_LANES = 1.45
RADAR_OUTER_RAW_LATERAL_LANES = 2.65
RADAR_RAW_MOVING_SPEED_KPH = 8.0
RADAR_RAW_CENTER_MIN_VALID_COUNT = 16
RADAR_RAW_ADJACENT_MIN_VALID_COUNT = 24
RADAR_RAW_OUTER_MIN_VALID_COUNT = 35
RADAR_PROBABLE_VEHICLE_LATERAL_LANES = 2.75
RADAR_VEHICLE_MIN_PROBABILITY = 0.35
RADAR_VEHICLE_DEDUP_LONGITUDINAL_M = 7.0
RADAR_VEHICLE_DEDUP_LATERAL_M = 1.6
RADAR_MERGE_LONGITUDINAL_MIN_M = 3.0
RADAR_MERGE_LONGITUDINAL_MAX_M = 7.0
RADAR_MERGE_LATERAL_M = 1.35
RADAR_MERGED_SOURCE_TAG = "+radar:"
CORNER_RADAR_LABELS = frozenset(("LF", "RF", "LR", "RR"))
REAR_CORNER_RADAR_LABELS = frozenset(("LR", "RR"))
VEHICLE_BADGE_TTC_S = 9.9
VEHICLE_BADGE_ACCEL_MPS2 = 1.0
MODEL_LINE_STRIP_GROUP_CACHE_LIMIT = 48
ROAD_STEPS_SURROUND = 96
ROAD_STEPS_MODEL = 48
ROAD_STEPS_SIM = 64
STATIC_LINE_STEPS = 56
PLANNED_PATH_FALLBACK_STEPS = 32
MODEL_PATH_METRIC_SEGMENT_LIMIT = 14
LANE_MARKING_SHADOW_HEIGHT_M = 0.026
LANE_MARKING_HEIGHT_M = 0.044
LANE_MARKING_BORDER_EXTRA_WIDTH_PX = 3
LANE_MARKING_BORDER_COLOR = LIGHT_CLUSTER_THEME.lane_marking_border
ROAD_EDGE_HEIGHT_M = 0.034
ROAD_EDGE_SHADOW_HEIGHT_M = 0.028
ROAD_EDGE_BACKING_COLOR = LIGHT_CLUSTER_THEME.road_edge_backing
PATH_SHADOW_LAYER_M = 0.024
PATH_UNCERTAINTY_LAYER_M = PATH_HEIGHT_M + 0.002
PATH_BODY_LAYER_M = PATH_HEIGHT_M + 0.046
PATH_METRIC_LAYER_M = PATH_HEIGHT_M + 0.066
PATH_HIGHLIGHT_LAYER_M = PATH_HEIGHT_M + 0.088
LANE_HIGHLIGHT_COLOR = (64, 148, 255)
LANE_HIGHLIGHT_ALPHA = 220
LANE_HIGHLIGHT_ROUTE_ALPHA = 170
BSD_LANE_MARKING_MATCH_TOLERANCE = 0.45


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
class RearVehicleIndicator:
    center: Vec3
    anchor: Vec3
    label: str
    lane_side: str
    longitudinal_m: float
    lateral_m: float
    source: str = ""


@dataclass(frozen=True, slots=True)
class PathBlocker:
    offset: float
    forward_m: float
    length_m: float


ModelLineStripGroups = tuple[tuple[MeshStrip, ...], ...] | None
ModelLineStripCacheKey = tuple[int, float, float, str, bool, tuple[tuple[int, Color, float], ...]]
_MODEL_LINE_STRIP_GROUP_CACHE: OrderedDict[
    ModelLineStripCacheKey,
    tuple[tuple[ModelPathPoint, ...], ModelLineStripGroups],
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
    rear_indicators: tuple[RearVehicleIndicator, ...] = ()


def rgba(color: tuple[int, int, int], alpha: int = 255) -> Color:
    return color[0], color[1], color[2], alpha


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
    visible_points: list[ModelPathPoint] = []
    for point in model_points:
        forward_m = data_scene_forward_m(point.forward_m)
        if start_m <= forward_m <= end_m:
            visible_points.append(point)
    return tuple(
        Vec3(point.lateral_m + lateral_shift_m, data_scene_forward_m(point.forward_m), height_m)
        for point in visible_points
    )


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


def dashed_centerline_segments(
    centerline: tuple[Vec3, ...],
    dash_m: float = 5.2,
    gap_m: float = 4.2,
) -> tuple[tuple[Vec3, ...], ...]:
    if len(centerline) < 2:
        return ()

    cycle_m = dash_m + gap_m
    segments: list[tuple[Vec3, ...]] = []
    current_dash: list[Vec3] = []
    distance_m = 0.0
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
            cycle_offset_m = cursor_m % cycle_m
            if cycle_offset_m < eps or abs(cycle_offset_m - cycle_m) < eps:
                cycle_offset_m = 0.0
            elif abs(cycle_offset_m - dash_m) < eps:
                cycle_offset_m = dash_m
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
    dash_m = 5.2
    gap_m = 4.2
    cursor = start_m
    while cursor < end_m:
        dash_end = min(cursor + dash_m, end_m)
        segment = lane_centerline(marking.offset, steering, lane_width_m, cursor, dash_end, 6, 0.0)
        if len(segment) >= 2:
            segments.append(segment)
        cursor += dash_m + gap_m
    return tuple(segments)


def strips_from_centerline_specs(
    points: tuple[Vec3, ...],
    specs: tuple[tuple[int, Color, float], ...],
) -> tuple[MeshStrip, ...]:
    if len(points) < 2:
        return ()

    half_widths = tuple(max(0.08, width_px * 0.022) * 0.5 for width_px, _, _ in specs)
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


def lane_marking_strip_groups_from_segments(
    segments: tuple[tuple[Vec3, ...], ...],
    specs: tuple[tuple[int, Color, float], ...],
) -> tuple[tuple[MeshStrip, ...], ...]:
    if not segments or not specs:
        return tuple(() for _ in specs)

    grouped: list[list[MeshStrip]] = [[] for _ in specs]
    for segment in segments:
        for spec_index, strip in enumerate(strips_from_centerline_specs(segment, specs)):
            grouped[spec_index].append(strip)
    return tuple(tuple(group) for group in grouped)


def cached_model_line_strip_groups(
    model_points: tuple[ModelPathPoint, ...],
    start_m: float,
    end_m: float,
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
    extend_before_model: bool,
) -> ModelLineStripGroups:
    key = (
        id(model_points),
        start_m,
        end_m,
        style,
        extend_before_model,
        specs,
    )
    cached = _MODEL_LINE_STRIP_GROUP_CACHE.get(key)
    if cached is not None and cached[0] is model_points:
        _MODEL_LINE_STRIP_GROUP_CACHE.move_to_end(key)
        return cached[1]
    if cached is not None:
        del _MODEL_LINE_STRIP_GROUP_CACHE[key]

    centerline = model_line_centerline(model_points, start_m, end_m, 0.0)
    if len(centerline) < 2:
        groups: ModelLineStripGroups = None if extend_before_model else tuple(() for _ in specs)
    else:
        if extend_before_model:
            centerline = extend_model_centerline_rearward(centerline, start_m)
        segments = (centerline,) if style == "solid" else dashed_centerline_segments(centerline)
        groups = lane_marking_strip_groups_from_segments(segments, specs)

    _MODEL_LINE_STRIP_GROUP_CACHE[key] = (model_points, groups)
    while len(_MODEL_LINE_STRIP_GROUP_CACHE) > MODEL_LINE_STRIP_GROUP_CACHE_LIMIT:
        _MODEL_LINE_STRIP_GROUP_CACHE.popitem(last=False)
    return groups


def translate_mesh_strip_groups_x(
    groups: tuple[tuple[MeshStrip, ...], ...],
    shift_x_m: float,
) -> tuple[tuple[MeshStrip, ...], ...]:
    if abs(shift_x_m) <= 0.0001:
        return groups
    return tuple(
        tuple(translate_mesh_strip_x(strip, shift_x_m) for strip in group)
        for group in groups
    )


def model_line_strip_groups(
    model_points: tuple[ModelPathPoint, ...],
    lateral_shift_m: float,
    start_m: float,
    end_m: float,
    specs: tuple[tuple[int, Color, float], ...],
    style: str,
    extend_before_model: bool,
) -> ModelLineStripGroups:
    groups = cached_model_line_strip_groups(
        model_points,
        start_m,
        end_m,
        specs,
        style,
        extend_before_model,
    )
    if groups is None:
        return None
    return translate_mesh_strip_groups_x(groups, lateral_shift_m)


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
) -> tuple[MeshStrip, ...]:
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
    path_specs: list[tuple[float, Color, float]] = [(0.86, theme.path_shadow, PATH_SHADOW_LAYER_M)]
    if model_driven:
        uncertainty_width = model_path_uncertainty_width(state)
        if uncertainty_width is not None:
            path_specs.append((uncertainty_width, theme.path_uncertainty, PATH_UNCERTAINTY_LAYER_M))
    path_specs.append((0.46, theme.path_body, PATH_BODY_LAYER_M))
    path_specs.append((0.16, theme.path_highlight, PATH_HIGHLIGHT_LAYER_M))
    strips = list(strips_from_centerline_width_specs(points, tuple(path_specs)))
    if model_driven:
        highlight_strip = strips.pop() if strips else None
        strips.extend(model_path_metric_strips(state, points))
        if highlight_strip is not None:
            strips.append(highlight_strip)
    return tuple(strips)


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


def radar_point_markers(
    state: ClusterUiState,
    lane_width_m: float,
    vehicle_points: tuple[RadarPoint, ...] = (),
    min_forward_m: float = ROAD_NEAR_M,
    max_forward_m: float = ROAD_FAR_M + 30.0,
) -> tuple[RadarPointMarker, ...]:
    markers: list[RadarPointMarker] = []
    for point in state.radar_points:
        if any(radar_points_same_vehicle(point, vehicle_point) for vehicle_point in vehicle_points):
            continue
        forward_m = data_scene_forward_m(point.longitudinal_m)
        if forward_m < min_forward_m or forward_m > max_forward_m:
            continue
        color = radar_point_color(point)
        absolute_speed_kph = radar_point_absolute_speed_kph(point, state)
        markers.append(
            RadarPointMarker(
                center=Vec3(
                    clamp(point.lateral_m, -lane_width_m * 3.0, lane_width_m * 3.0),
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
        if radar_point_can_fill_vehicle_speed(point, state) and radar_point_close_to_vehicle(point, vehicle)
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
    return (
        vehicle.label in CORNER_RADAR_LABELS
        and vehicle.absolute_speed_kph is None
        and (vehicle.source == "carState" or vehicle.source.startswith("CAN 0x"))
    )


def radar_point_can_fill_vehicle_speed(point: RadarPoint, state: ClusterUiState) -> bool:
    return radar_point_absolute_speed_kph(point, state) is not None or point.relative_speed_mps is not None


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


def radar_vehicle_box(point: RadarPoint, state: ClusterUiState, lane_width_m: float) -> VehicleBox:
    confidence = radar_vehicle_confidence(point)
    alpha = int(92 + 163 * confidence)
    body_color = GREEN
    forward_m = data_scene_forward_m(point.longitudinal_m)
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
    if point.valid_count is not None and point.valid_count < RADAR_VEHICLE_MIN_VALID_COUNT:
        return False
    if point.probability is not None and point.probability < 0.20 and not point.in_my_lane:
        return False
    if radar_point_has_vehicle_estimate(point, state, lane_width_m):
        return True
    if radar_point_is_stationary_object(point, state):
        return False
    if radar_point_is_side_static_reflection(point, state, lane_width_m):
        return False
    if radar_point_matches_static_road_edge(point, state, lane_width_m):
        return False
    if radar_point_is_moving_raw_vehicle(point, state, lane_width_m):
        return True
    return False


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


def radar_point_matches_detected_vehicle(point: RadarPoint, state: ClusterUiState) -> bool:
    for vehicle in state.detected_vehicles:
        longitudinal_tolerance = max(4.0, min(8.0, point.longitudinal_m * 0.08))
        if abs(point.longitudinal_m - vehicle.longitudinal_m) > longitudinal_tolerance:
            continue
        if abs(point.lateral_m - vehicle.lateral_m) <= 1.35:
            return True
    return False


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
    if point.in_my_lane is not None and point.in_my_lane > 0:
        return BLUE[0], BLUE[1], BLUE[2], 226
    if point.probability is not None and point.probability < 0.25:
        return 116, 126, 136, 150
    if point.relative_speed_mps is not None and point.relative_speed_mps < -2.5:
        return AMBER[0], AMBER[1], AMBER[2], 226
    return 34, 150, 255, 208


def radar_point_radius(point: RadarPoint) -> float:
    probability = point.probability if point.probability is not None else 0.72
    return clamp(0.105 + 0.07 * probability, 0.095, 0.19)


def rear_vehicle_indicators(
    vehicles: tuple[DetectedVehicle, ...],
    state: ClusterUiState,
    lane_width_m: float,
) -> tuple[RearVehicleIndicator, ...]:
    selected: dict[str, DetectedVehicle] = {}
    for vehicle in vehicles:
        if vehicle.label not in REAR_CORNER_RADAR_LABELS or vehicle.longitudinal_m >= -0.2:
            continue
        side = "left" if vehicle.label == "LR" or vehicle.lateral_m < 0.0 else "right"
        existing = selected.get(side)
        if existing is None or abs(vehicle.longitudinal_m) < abs(existing.longitudinal_m):
            selected[side] = vehicle

    indicators: list[RearVehicleIndicator] = []
    for side in ("left", "right"):
        vehicle = selected.get(side)
        if vehicle is None:
            continue
        forward_m = data_scene_forward_m(vehicle.longitudinal_m)
        anchor_forward_m = EGO_FORWARD_M + 2.3
        offset = clamp(vehicle.lateral_m / lane_width_m, -2.2, 2.2)
        indicators.append(
            RearVehicleIndicator(
                center=Vec3(
                    road_world_x(offset, forward_m, state.steering, lane_width_m),
                    forward_m,
                    VEHICLE_HEIGHT_M * 0.5,
                ),
                anchor=Vec3(
                    road_world_x(offset, anchor_forward_m, state.steering, lane_width_m),
                    anchor_forward_m,
                    0.22,
                ),
                label=vehicle.label,
                lane_side=side,
                longitudinal_m=vehicle.longitudinal_m,
                lateral_m=vehicle.lateral_m,
                source=vehicle.source,
            )
        )
    return tuple(indicators)


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
    relative_speed_mps: float | None = None,
    absolute_speed_kph: float | None = None,
    acceleration_mps2: float | None = None,
    ttc_s: float | None = None,
    cut_in: bool = False,
    primary: bool = False,
    annotate: bool = False,
) -> VehicleBox:
    confidence = clamp(confidence, 0.0, 1.0)
    alpha = int(92 + 163 * confidence)
    body_color = color
    center_x_m = road_world_x(offset, forward_m, steering, lane_width_m)
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

    drive_camera = CameraSpec(
        position=Vec3(0.0, -8.80, 5.20),
        target=Vec3(0.0, 22.0, 0.18),
        fovy_deg=31.0,
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


def translate_mesh_strip_x(strip: MeshStrip, shift_x_m: float) -> MeshStrip:
    if abs(shift_x_m) <= 0.0001:
        return strip
    return MeshStrip(
        left=strip.left,
        right=strip.right,
        color=strip.color,
        x_offset_m=strip.x_offset_m + shift_x_m,
    )


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


def translate_rear_indicator_x(indicator: RearVehicleIndicator, shift_x_m: float) -> RearVehicleIndicator:
    if abs(shift_x_m) <= 0.0001:
        return indicator
    return RearVehicleIndicator(
        center=translate_vec3_x(indicator.center, shift_x_m),
        anchor=translate_vec3_x(indicator.anchor, shift_x_m),
        label=indicator.label,
        lane_side=indicator.lane_side,
        longitudinal_m=indicator.longitudinal_m,
        lateral_m=indicator.lateral_m,
        source=indicator.source,
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


def road_edge_model_strips(
    model_points: tuple[ModelPathPoint, ...],
    lateral_shift_m: float,
    color: Color,
    start_m: float,
    end_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> tuple[MeshStrip, ...]:
    groups = model_line_strip_groups(
        model_points,
        lateral_shift_m,
        start_m,
        end_m,
        (
            (12, theme.road_edge_backing, ROAD_EDGE_SHADOW_HEIGHT_M),
            (7, color, ROAD_EDGE_HEIGHT_M),
        ),
        "solid",
        True,
    )
    if groups is None:
        return ()
    backing, foreground = groups
    return (*backing, *foreground)


def road_edge_offset_strips(
    offset: float,
    steering: float,
    lane_width_m: float,
    color: Color,
    start_m: float,
    end_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> tuple[MeshStrip, ...]:
    centerline = lane_centerline(offset, steering, lane_width_m, start_m, end_m, STATIC_LINE_STEPS, 0.0)
    backing, foreground = lane_marking_strip_groups_from_segments(
        (centerline,),
        (
            (12, theme.road_edge_backing, ROAD_EDGE_SHADOW_HEIGHT_M),
            (7, color, ROAD_EDGE_HEIGHT_M),
        ),
    )
    return (*backing, *foreground)


def vehicle_color_for_detection(
    vehicle: DetectedVehicle,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
) -> tuple[int, int, int]:
    if RADAR_MERGED_SOURCE_TAG in vehicle.source:
        return BLUE
    if vehicle.cut_in:
        return AMBER
    if vehicle.primary:
        return theme.primary_vehicle
    if vehicle.source.startswith("modelV2"):
        return theme.model_vehicle
    return theme.default_vehicle


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


def detected_vehicle_has_visible_box(vehicle: DetectedVehicle, camera_active: bool) -> bool:
    return not (
        vehicle.label in REAR_CORNER_RADAR_LABELS
        and vehicle.longitudinal_m < -0.2
        and not camera_active
    )


def road_edge_strips(
    state: ClusterUiState,
    route_mode: bool,
    lane_width_m: float,
    road_start_m: float,
    road_end_m: float,
    theme: ClusterTheme = LIGHT_CLUSTER_THEME,
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
                road_start_m,
                road_end_m,
                theme,
            ),
            *road_edge_offset_strips(
                right_offset,
                state.steering,
                lane_width_m,
                default_color,
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
                    road_start_m,
                    road_end_m,
                    theme,
                )
            )
        elif state.right_road_edge_offset is not None:
            strips.extend(
                road_edge_offset_strips(
                    clamp(state.right_road_edge_offset, 0.68, 2.8),
                    state.steering,
                    lane_width_m,
                    right_color,
                    road_start_m,
                    road_end_m,
                    theme,
                )
            )
    return tuple(strips) if strips else default_road_edge_strips()


def profile_scene_start(profile_add: ProfileAdd | None) -> float:
    return time.perf_counter() if profile_add is not None else 0.0


def profile_scene_add(profile_add: ProfileAdd | None, name: str, start_time: float) -> None:
    if profile_add is not None:
        profile_add(name, (time.perf_counter() - start_time) * 1000.0)


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
    anchor_x_m = ego_anchor_x_m(state, lane_width_m)
    scene_shift_x_m = -anchor_x_m
    camera = scene_camera(state, lane_width_m, anchor_x_m)
    camera_active = state.surround_view_active
    selected_radar_vehicle_points = radar_vehicle_points(state, lane_width_m)
    selected_radar_vehicle_boxes = tuple(
        radar_vehicle_box(point, state, lane_width_m)
        for point in selected_radar_vehicle_points
    )
    route_mode = data_geometry_mode_for_state(state)
    road_start_m = (
        SURROUND_ROAD_REAR_M if state.surround_view_active
        else ROAD_NEAR_M
    )
    road_end_m = (
        SURROUND_ROAD_FRONT_M if state.surround_view_active
        else ROAD_FAR_M
    )
    road_steps = ROAD_STEPS_SURROUND if camera_active else ROAD_STEPS_MODEL if route_mode else ROAD_STEPS_SIM
    if (state.detected_vehicles or selected_radar_vehicle_boxes) and not camera_active:
        nearest_detected_y = min(
            (data_scene_forward_m(vehicle.longitudinal_m) for vehicle in state.detected_vehicles),
            default=ROAD_FAR_M,
        )
        nearest_radar_y = min((vehicle.center.y for vehicle in selected_radar_vehicle_boxes), default=ROAD_FAR_M)
        nearest_detected_y = min(nearest_detected_y, nearest_radar_y)
        road_start_m = min(road_start_m, max(-35.0, nearest_detected_y - 8.0))
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
            strip_groups = model_line_strip_groups(
                marking.model_points,
                marking.model_lateral_shift_m,
                road_start_m,
                road_end_m,
                marking_specs,
                marking.style,
                True,
            )
        if strip_groups is None:
            marking_segments = lane_marking_segments_for_marking(
                marking,
                state.steering,
                lane_width_m,
                road_start_m,
                road_end_m,
                extend_before_model=True,
            )
            strip_groups = lane_marking_strip_groups_from_segments(marking_segments, marking_specs)
        backing_strips, foreground_strips = strip_groups
        lane_strips.extend(backing_strips)
        lane_strips.extend(foreground_strips)
    profile_scene_add(profile_add, "scene.build.lane_markings", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    ego_offset = clamp(state.ego_lane_offset, -1.25, 1.25)
    target_offset = state.highlight_lane_offset if state.lane_change_phase == "changing" else None
    ego_vehicle = vehicle_box(ego_offset, EGO_FORWARD_M, state.steering, lane_width_m, EGO, camera_active, target_offset)
    merged_radar_labels = frozenset[str]()
    if route_mode:
        merged_detected_vehicles = detected_vehicles_with_merged_radar(
            state.detected_vehicles,
            state.radar_points,
            state,
        )
        render_detected_vehicles = tuple(
            detected
            for detected in merged_detected_vehicles
            if detected_vehicle_has_visible_box(detected, camera_active)
        )
        merged_radar_labels = frozenset(
            label
            for label in (merged_radar_point_label(vehicle) for vehicle in render_detected_vehicles)
            if label is not None
        )
        detected_vehicle_boxes = tuple(
            vehicle_box(
                clamp(detected.lateral_m / lane_width_m, -2.2, 2.2),
                data_scene_forward_m(detected.longitudinal_m),
                state.steering,
                lane_width_m,
                vehicle_color_for_detection(detected, theme),
                camera_active,
                confidence=detected.probability,
                label=detected.label,
                source=detected.source,
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
            )
            for detected in render_detected_vehicles
        )
        blocking_detected_vehicles = tuple(
            detected for detected in state.detected_vehicles if vehicle_blocks_path(detected)
        )
        detected_blockers = tuple(
            PathBlocker(
                clamp(detected.lateral_m / lane_width_m, -2.2, 2.2),
                data_scene_forward_m(detected.longitudinal_m),
                VEHICLE_LENGTH_M,
            )
            for detected in blocking_detected_vehicles
        )
        visible_radar_vehicle_pairs = tuple(
            (point, box)
            for point, box in zip(selected_radar_vehicle_points, selected_radar_vehicle_boxes)
            if point.label not in merged_radar_labels
        )
        visible_radar_vehicle_points = tuple(point for point, _ in visible_radar_vehicle_pairs)
        visible_radar_vehicle_boxes = tuple(box for _, box in visible_radar_vehicle_pairs)
        radar_blockers = tuple(
            PathBlocker(
                clamp(vehicle.center.x / lane_width_m, -2.2, 2.2),
                vehicle.center.y,
                vehicle.length_m,
            )
            for vehicle in visible_radar_vehicle_boxes
        )
        blockers = (*detected_blockers, *radar_blockers)
        vehicles = (ego_vehicle, *detected_vehicle_boxes, *visible_radar_vehicle_boxes)
    else:
        blockers = ()
        vehicles = (ego_vehicle,)
    rear_indicators = rear_vehicle_indicators(state.detected_vehicles, state, lane_width_m) if route_mode else ()
    profile_scene_add(profile_add, "scene.build.vehicles", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    road_surface = MeshStrip((), (), rgba(theme.road))
    profile_scene_add(profile_add, "scene.build.road_surface", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    road_edges = road_edge_strips(state, route_mode, lane_width_m, road_start_m, road_end_m, theme)
    profile_scene_add(profile_add, "scene.build.road_edges", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    planned_path = planned_path_strips(state, lane_width_m, blockers, theme)
    profile_scene_add(profile_add, "scene.build.planned_path", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    hidden_merged_radar_points = tuple(point for point in state.radar_points if point.label in merged_radar_labels)
    radar_points = radar_point_markers(
        state,
        lane_width_m,
        (*selected_radar_vehicle_points, *hidden_merged_radar_points),
        min_forward_m=road_start_m if camera_active else ROAD_NEAR_M,
        max_forward_m=road_end_m if camera_active else ROAD_FAR_M + 30.0,
    )
    profile_scene_add(profile_add, "scene.build.radar_points", profile_stage)

    profile_stage = profile_scene_start(profile_add)
    scene = ClusterScene(
        camera=camera,
        scene_shift_x_m=scene_shift_x_m,
        road_surface=road_surface,
        road_edges=tuple(road_edges),
        highlight_lanes=tuple(highlight_lanes),
        lane_markings=tuple(lane_strips),
        planned_path=tuple(planned_path),
        radar_points=tuple(radar_points),
        vehicles=tuple(vehicles),
        rear_indicators=tuple(rear_indicators),
    )
    profile_scene_add(profile_add, "scene.build.pack", profile_stage)
    return scene
