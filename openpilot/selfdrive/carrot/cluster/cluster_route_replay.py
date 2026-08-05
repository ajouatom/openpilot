from __future__ import annotations

import bz2
import io
import math
import multiprocessing
import os
import queue
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import traceback
import numpy as np
from bisect import bisect_right
from dataclasses import dataclass, replace
from functools import cache
from pathlib import Path
from typing import Any

from cluster_config import (
    BLUE,
    BLUE_SOFT,
    DEFAULT_LANE_WIDTH_M,
    MAX_ACCEL_MPS2,
    MAX_SPEED_KPH,
    MAX_STEERING_ANGLE_DEG,
    MODEL_DIRECT_LANE_RECENTER_SECONDS,
    RADAR_TO_CAMERA_M,
    ROAD_CURVE_M_PER_M2,
    WHITE,
    YELLOW,
)
from cluster_models import (
    ClusterAlert,
    ClusterUiState,
    CruiseDisplayState,
    DetectedVehicle,
    LaneMarking,
    ModelPathPoint,
    ModelRiskPoint,
    RadarPoint,
    RADAR_MOVING_VEHICLE_MIN_SPEED_KPH,
    RouteOverlay,
    TpmsInfo,
    TripReportState,
    radar_position_is_zero,
)
from cluster_trip_report import TripReportTracker
from cluster_utils import clamp, smoothstep
from openpilot.selfdrive.controls.lib.cutin_helpers import (
    associate_cutin_tracks,
    combine_cutin_future_projection,
    CORNER_CUTIN_MAX_DREL_M,
    cutin_confirmation_frames,
    cutin_min_track_age_frames,
    cutin_entry_rejection_reason,
    cutin_tuning_from_sensitivity,
    effective_cutin_inward_speed,
    hold_side_corner_front_matches,
    FRONT_CUTIN_MAX_ABS_YREL_M,
    FRONT_CUTIN_MAX_DREL_M,
    is_cutin_track_discontinuous,
    FRONT_CUTIN_MIN_CONFIRM_S,
    FRONT_CUTIN_MIN_DREL_M,
    is_corner_confirmed_near_cutin,
    is_side_corner_object,
    is_corner_track_id,
    is_corner_radar_source,
    is_stable_corner_track_id,
    STABLE_CORNER_TRACK_ID_START,
    is_fast_cutin_entry,
    match_side_corner_to_front_tracks,
    new_cutin_position_history,
    update_cutin_confirmation,
    update_lane_relative_motion,
)


DBC_SIGNAL_RE = re.compile(
    r'^\s*SG_ (\w+) : (\d+)\|(\d+)@([01])([+-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\)'
)
ROUTE_SCHEMA_CACHE_NAME = "carrotpilot_cluster_capnp_v1"
LOG_FILENAMES = {
    "qlog": "qlog.zst",
    "rlog": "rlog.zst",
}
NUMBERED_FOLDER_RE = re.compile(r"^(.*?)(\d+)$")


@dataclass(frozen=True)
class DbcSignalSpec:
    start: int
    length: int
    byte_order: str
    signed: bool
    factor: float
    offset: float


MODEL_LEAD_MIN_PROB = 0.50
RADAR_POINT_STALE_S = 0.25
CORNER_DETECTION_STALE_S = 0.8
RADAR_MIN_LONGITUDINAL_M = 0.0
RADAR_FRONT_MAX_LONGITUDINAL_M = 180.0
CORNER_RADAR_REAR_MIN_LONGITUDINAL_M = -180.0
CCNC_CORNER_RADAR_ADDRESS = 0x162
ADRV_CORNER_RADAR_ADDRESS = 0x1EA
CORNER_OBJECT_TRACK_ID_OFFSET = 200
CORNER_OBJECT_TRACK_COUNT = 20
CORNER_OBJECT_180_TRACK_ID_OFFSET = 240
CORNER_OBJECT_180_TRACK_COUNT = 10
CORNER_OBJECT_430_TRACK_ID_OFFSET = 300
CORNER_OBJECT_430_TRACK_COUNT = 112
CORNER_OBJECT_SOURCE = "cornerRadar"
RAW_CORNER_RADAR_BUS = 1
RAW_CORNER_235_START_ADDR = 0x235
RAW_CORNER_235_END_ADDR = 0x248
RAW_CORNER_180_START_ADDR = 0x180
RAW_CORNER_180_END_ADDR = 0x184
RAW_CORNER_OBJECT_MAX_X_M = 180.0
RAW_CORNER_OBJECT_MAX_ABS_Y_M = 12.0
RAW_CORNER_TRACK_STALE_S = 0.55
RAW_CORNER_TRACK_MAX_MATCH_X_M = 7.0
RAW_CORNER_TRACK_MAX_MATCH_Y_M = 3.2
RAW_CORNER_TRACK_MAX_COST = 5.5
RAW_CORNER_TRACK_POSITION_ALPHA = 0.62
RAW_CORNER_TRACK_VELOCITY_ALPHA = 0.45
RAW_CORNER_TRACK_DISPLAY_MIN_HITS = 4
RAW_CORNER_TRACK_DISPLAY_OUTER_ABS_Y_M = 8.0
RAW_CORNER_TRACK_DISPLAY_OUTER_MIN_HITS = 12
REPLAY_CUTIN_RAW_OBJECT_MAX_AGE_S = 0.15
REPLAY_CUTIN_RAW_OBJECT_MAX_DREL_M = 1.5
REPLAY_CUTIN_RAW_OBJECT_MAX_YREL_M = 0.75
REPLAY_CUTIN_RAW_OBJECT_MAX_VREL_MPS = 2.5
ROUTE_CORNER_SOURCE_STABLE = "stable"
ROUTE_CORNER_SOURCE_RAW = "raw"
ROUTE_CORNER_SOURCE_LIVE = "live"
ROUTE_CORNER_SOURCE_CHOICES = (
    ROUTE_CORNER_SOURCE_STABLE,
    ROUTE_CORNER_SOURCE_RAW,
    ROUTE_CORNER_SOURCE_LIVE,
)
CORNER_YAW_COMP_MAX_DREL = 50.0
CORNER_YAW_COMP_MAX_YAW_RATE = 0.35
CORNER_YAW_COMP_MAX_YVREL_CORRECTION = 1.5
CORNER_DISPLAY_CENTERING_DEFAULT_M = 0.0
CORNER_DISPLAY_CENTERING_SIDE_LANE_MIN_M = 1.8
CORNER_DISPLAY_CENTERING_SIDE_LANE_WIDTH_RATIO = 0.55
CORNER_RADAR_OBJECTS_235_EXT_FLAG = 2 ** 12
CORNER_RADAR_OBJECTS_180_EXT_FLAG = 2 ** 13
CORNER_RADAR_OBJECTS_EXT_FLAGS = CORNER_RADAR_OBJECTS_235_EXT_FLAG | CORNER_RADAR_OBJECTS_180_EXT_FLAG
HYUNDAI_CAMERA_CAN_BUS_MOD = 2
CORNER_RADAR_DBC_MESSAGES = {
    CCNC_CORNER_RADAR_ADDRESS: "CCNC_0x162",
    ADRV_CORNER_RADAR_ADDRESS: "ADRV_0x1ea",
}
CORNER_RADAR_DBC_SIGNAL_NAMES = frozenset(
    {
        "LANE_CHANGING",
        "LF_DETECT",
        "LF_DETECT_DISTANCE",
        "LF_DETECT_LATERAL",
        "RF_DETECT",
        "RF_DETECT_DISTANCE",
        "RF_DETECT_LATERAL",
        "LR_DETECT",
        "LR_DETECT_DISTANCE",
        "LR_DETECT_LATERAL",
        "RR_DETECT",
        "RR_DETECT_DISTANCE",
        "RR_DETECT_LATERAL",
    }
)
ROUTE_REPLAY_MIN_BUFFER_FILES = 2
ROUTE_REPLAY_START_BUFFER_FILES = 1
ROUTE_REPLAY_READAHEAD_S = 5.0
ROUTE_REPLAY_PRELOAD_READY_AHEAD_S = 20.0
ROUTE_REPLAY_RETAIN_BEHIND_S = 1.0
ROUTE_REPLAY_PRELOAD_NICE = 10
ROUTE_VIDEO_FPS = 10.0
ROUTE_VIDEO_DECODE_WIDTH = 388
ROUTE_VIDEO_DECODE_HEIGHT = 244
ROUTE_VIDEO_SEEK_RESTART_FRAMES = 45
ROUTE_VIDEO_FFMPEG_ENV = "CLUSTER_ROUTE_FFMPEG"
ROUTE_SHOW_RECORDED_CUTINS_ENV = "CLUSTER_ROUTE_SHOW_RECORDED_CUTINS"
ROUTE_FRONT_RADAR_ONLY_ENV = "CLUSTER_ROUTE_FRONT_RADAR_ONLY"
ROUTE_CUTIN_RADAR_SOURCE_ENV = "CLUSTER_ROUTE_CUTIN_RADAR_SOURCE"
ROUTE_CUTIN_SENSITIVITY_ENV = "CLUSTER_ROUTE_CUTIN_SENSITIVITY"
ROUTE_CUTIN_RADAR_SOURCE_CORNER = "corner"
ROUTE_CUTIN_RADAR_SOURCE_FRONT = "front"
REPLAY_CUTIN_DT = 0.05
RECORDED_CUTIN_REPLAY_HOLD_S = 0.08
REPLAY_CUTIN_STICKY_FRAMES = int(0.7 / REPLAY_CUTIN_DT)
REPLAY_CUTIN_OUTPUT_HOLD_FRAMES = max(1, int(round(0.5 / REPLAY_CUTIN_DT)))
REPLAY_CUTIN_OUTPUT_HOLD_DREL_M = 3.0
REPLAY_CUTIN_OUTPUT_HOLD_YREL_M = 1.0
REPLAY_CUTIN_OUTPUT_HOLD_VREL_MPS = 2.0
REPLAY_CUTIN_KEEP_FUTURE_IN_LANE_PROB = 0.12
REPLAY_CUTIN_KEEP_MAX_DPATH_FUTURE = 1.6
REPLAY_CUTIN_KEEP_MAX_MOVING_AWAY = 0.3
REPLAY_CUTIN_YAW_GAIN = 0.6
NAV_SPEED_LIMIT_HOLD_SECONDS = 10.0
ROAD_EDGE_VEHICLE_OUTSIDE_MARGIN_M = 0.25
NEAR_ROAD_EDGE_VEHICLE_BLOCK_DISTANCE_M = 10.0
LANE_CHANGE_REINDEX_PEAK_THRESHOLD = 0.22
LANE_CHANGE_REINDEX_RESET_THRESHOLD = -0.08
CONTINUOUS_LANE_CHANGE_REBASE_PROGRESS = 0.12
LANE_CHANGE_MODEL_DIRECT_ONLY = True
ROUTE_REPLAY_USE_LANE_CHANGE_ANIMATION = True
LANE_LINE_PROBABILITY_MIN = 0.4
MODEL_DIRECT_LANE_SETTLE_MIN_PROGRESS = 0.65
LONGITUDINAL_PERSONALITY_GAPS = {
    "aggressive": 1,
    "standard": 2,
    "relaxed": 3,
    "morerelaxed": 4,
}


@dataclass(frozen=True, slots=True)
class RawCornerObject:
    t: float
    group: str
    address: int
    slot: int
    quality: int
    age: int
    object_id: int
    object_class: int
    width: float
    x: float
    y: float
    vx: float
    vy: float
    ax: float


@dataclass(slots=True)
class StableCornerTrack:
    track_id: int
    object_id: int
    group: str
    slot: int
    last_t: float
    x: float
    y: float
    vx: float
    vy: float
    ax: float
    quality: int
    age: int
    hits: int = 1
    lead_filter_t: float | None = None
    lead_filter_count: int = 0
    v_lead_filtered: float = 0.0
    v_lead_filtered_last: float = 0.0
    a_lead_filtered: float = 0.0
    j_lead_filtered: float = 0.0


@dataclass(frozen=True, slots=True)
class ReconstructedLiveTrack:
    trackId: int
    dRel: float
    yRel: float
    vRel: float
    aRel: float
    yvRel: float
    vLead: float
    measured: bool
    radarSource: str
    aLead: float = 0.0
    jLead: float = 0.0


class StableCornerObjectTracker:
    def __init__(self) -> None:
        self.tracks: dict[int, StableCornerTrack] = {}
        self.next_track_id = 0

    def update(self, obj: RawCornerObject) -> None:
        self._expire(obj.t)
        match = self._match(obj)
        if match is None:
            track_id = self.next_track_id
            self.next_track_id += 1
            self.tracks[track_id] = StableCornerTrack(
                track_id=track_id,
                object_id=obj.object_id,
                group=obj.group,
                slot=obj.slot,
                last_t=obj.t,
                x=obj.x,
                y=obj.y,
                vx=obj.vx,
                vy=obj.vy,
                ax=obj.ax,
                quality=obj.quality,
                age=obj.age,
            )
            return

        dt = max(0.0, obj.t - match.last_t)
        predicted_x = match.x + match.vx * dt
        predicted_y = match.y + match.vy * dt
        pos_alpha = RAW_CORNER_TRACK_POSITION_ALPHA
        vel_alpha = RAW_CORNER_TRACK_VELOCITY_ALPHA
        match.object_id = obj.object_id
        match.group = obj.group
        match.slot = obj.slot
        match.last_t = obj.t
        match.x = predicted_x * (1.0 - pos_alpha) + obj.x * pos_alpha
        match.y = predicted_y * (1.0 - pos_alpha) + obj.y * pos_alpha
        match.vx = match.vx * (1.0 - vel_alpha) + obj.vx * vel_alpha
        match.vy = match.vy * (1.0 - vel_alpha) + obj.vy * vel_alpha
        match.ax = obj.ax
        match.quality = obj.quality
        match.age = obj.age
        match.hits += 1

    def _visible_tracks_at(
        self,
        t: float,
        max_measurement_age_s: float | None = None,
    ) -> tuple[tuple[StableCornerTrack, float, float], ...]:
        self._expire(t)
        visible_tracks: list[tuple[StableCornerTrack, float, float]] = []
        for track in self.tracks.values():
            if track.hits < RAW_CORNER_TRACK_DISPLAY_MIN_HITS:
                continue
            dt = max(0.0, t - track.last_t)
            if max_measurement_age_s is not None and dt > max_measurement_age_s:
                continue
            x = track.x + track.vx * dt
            y = track.y + track.vy * dt
            if abs(y) > RAW_CORNER_TRACK_DISPLAY_OUTER_ABS_Y_M and track.hits < RAW_CORNER_TRACK_DISPLAY_OUTER_MIN_HITS:
                continue
            if not RADAR_MIN_LONGITUDINAL_M <= x <= RADAR_FRONT_MAX_LONGITUDINAL_M:
                continue
            if abs(y) > RAW_CORNER_OBJECT_MAX_ABS_Y_M:
                continue
            visible_tracks.append((track, x, y))
        return tuple(visible_tracks)

    def live_tracks_at(
        self,
        t: float,
        v_ego: float,
        max_measurement_age_s: float | None = None,
    ) -> tuple[ReconstructedLiveTrack, ...]:
        live_tracks: list[ReconstructedLiveTrack] = []
        for track, x, y in self._visible_tracks_at(t, max_measurement_age_s):
            v_lead = v_ego + track.vx
            a_lead, j_lead = self._update_lead_dynamics(track, t, v_lead)
            live_tracks.append(ReconstructedLiveTrack(
                trackId=STABLE_CORNER_TRACK_ID_START + track.track_id,
                dRel=x,
                yRel=y,
                vRel=track.vx,
                aRel=track.ax,
                yvRel=track.vy,
                vLead=v_lead,
                measured=True,
                radarSource="corner235" if track.group == "235" else "corner180",
                aLead=a_lead,
                jLead=j_lead,
            ))
        return tuple(live_tracks)

    @staticmethod
    def _update_lead_dynamics(track: StableCornerTrack, t: float, v_lead: float) -> tuple[float, float]:
        # Mirrors the RadarInterface MyTrack filter for raw-CAN replay. The
        # six-sample warm-up keeps reconstructed corner points aligned with
        # the values emitted by a running device.
        if track.lead_filter_t is None or t <= track.lead_filter_t or t - track.lead_filter_t > 0.25:
            track.lead_filter_t = t
            track.lead_filter_count = 1
            track.v_lead_filtered = v_lead
            track.v_lead_filtered_last = v_lead
            track.a_lead_filtered = 0.0
            track.j_lead_filtered = 0.0
            return 0.0, 0.0

        dt = max(0.01, min(0.10, t - track.lead_filter_t))
        track.lead_filter_t = t
        v_alpha = dt / (0.1 + dt)
        a_alpha = dt / (0.15 + dt)
        j_alpha = dt / (0.4 + dt)
        track.v_lead_filtered += v_alpha * (v_lead - track.v_lead_filtered)
        pseudo_stop = abs(track.v_lead_filtered) < 0.3 and abs(v_lead - track.v_lead_filtered) < 0.05
        a_raw = (track.v_lead_filtered - track.v_lead_filtered_last) / dt
        track.v_lead_filtered_last = track.v_lead_filtered
        if abs(a_raw - track.a_lead_filtered) > 3.0:
            track.lead_filter_count = 0
        a_input = 0.0 if pseudo_stop else clamp(a_raw, -10.0, 5.0)
        previous_a_lead = track.a_lead_filtered
        track.a_lead_filtered += a_alpha * (a_input - track.a_lead_filtered)
        j_input = (track.a_lead_filtered - previous_a_lead) / dt if track.lead_filter_count > 2 else 0.0
        track.j_lead_filtered += j_alpha * (j_input - track.j_lead_filtered)
        track.lead_filter_count += 1
        if track.lead_filter_count < 6:
            return 0.0, 0.0
        return track.a_lead_filtered, track.j_lead_filtered

    def points_at(self, t: float, ego_speed_kph: float) -> tuple[RadarPoint, ...]:
        visible_tracks = self._visible_tracks_at(t)

        id_counts: dict[int, int] = {}
        for track, _x, _y in visible_tracks:
            id_counts[track.object_id] = id_counts.get(track.object_id, 0) + 1

        points: list[RadarPoint] = []
        for track, x, y in visible_tracks:
            label = (
                f"CO{track.object_id:03d}"
                if id_counts.get(track.object_id, 0) == 1
                else f"CO{track.object_id:03d}_{track.track_id:02d}"
            )
            points.append(
                RadarPoint(
                    label=label,
                    longitudinal_m=x,
                    lateral_m=renderer_lateral_from_openpilot_yrel(y),
                    source=CORNER_OBJECT_SOURCE,
                    relative_speed_mps=track.vx,
                    absolute_speed_kph=ego_speed_kph + track.vx * 3.6,
                    lateral_speed_mps=renderer_lateral_from_openpilot_yrel(track.vy),
                    relative_accel_mps2=track.ax,
                    probability=clamp(0.35 + min(track.quality, 80) / 100.0, 0.35, 0.92),
                    valid=1,
                    valid_count=track.hits,
                )
            )
        return sorted_radar_points(points)

    def _match(self, obj: RawCornerObject) -> StableCornerTrack | None:
        same_id_best: StableCornerTrack | None = None
        same_id_best_cost = RAW_CORNER_TRACK_MAX_COST
        fallback_best: StableCornerTrack | None = None
        fallback_best_cost = RAW_CORNER_TRACK_MAX_COST
        for track in self.tracks.values():
            dt = max(0.0, obj.t - track.last_t)
            if dt > RAW_CORNER_TRACK_STALE_S:
                continue
            predicted_x = track.x + track.vx * dt
            predicted_y = track.y + track.vy * dt
            dx = abs(obj.x - predicted_x)
            dy = abs(obj.y - predicted_y)
            if dx > RAW_CORNER_TRACK_MAX_MATCH_X_M or dy > RAW_CORNER_TRACK_MAX_MATCH_Y_M:
                continue
            dv = abs(obj.vx - track.vx)
            age_penalty = 0.0 if obj.age >= track.age or track.age == 255 else 0.8
            cost = dx * 0.55 + dy * 1.05 + dv * 0.12 + age_penalty
            if obj.object_id == track.object_id:
                if cost < same_id_best_cost:
                    same_id_best_cost = cost
                    same_id_best = track
            elif cost + 3.5 < fallback_best_cost:
                fallback_best_cost = cost + 3.5
                fallback_best = track
        return same_id_best if same_id_best is not None else fallback_best

    def _expire(self, t: float) -> None:
        stale = [
            track_id
            for track_id, track in self.tracks.items()
            if t - track.last_t > RAW_CORNER_TRACK_STALE_S
        ]
        for track_id in stale:
            self.tracks.pop(track_id, None)


@dataclass(frozen=True, slots=True)
class RouteReplayFrame:
    t: float
    speed_kph: float
    accel_mps2: float
    steering: float
    steering_angle_deg: float | None
    speed_limit_kph: int | None
    speed_limit_source: str | None
    cruise_kph: int | None
    cruise_display_state: CruiseDisplayState
    gear_text: str | None
    cruise_gap: int | None
    lfa_active: bool | None
    active_lane_line: bool | None
    left_signal: bool
    right_signal: bool
    left_blindspot: bool
    right_blindspot: bool
    lane_width_m: float
    lane_center_offset_m: float | None
    left_lane_offset: float
    right_lane_offset: float
    left_lane_visible: bool
    right_lane_visible: bool
    extra_left_lane_visible: bool
    extra_right_lane_visible: bool
    left_road_edge_offset: float | None
    right_road_edge_offset: float | None
    left_lane_style: str
    right_lane_style: str
    left_lane_color: tuple[int, int, int] | None
    right_lane_color: tuple[int, int, int] | None
    road_curvature: float | None
    road_curvature_source: str
    lane_position_source: str
    model_lane_lines: tuple[tuple[ModelPathPoint, ...], ...]
    model_road_edges: tuple[tuple[ModelPathPoint, ...], ...]
    model_path: tuple[ModelPathPoint, ...]
    model_path_source: str
    lane_change_source: str
    lane_change: str | None
    lane_change_phase: str
    lane_change_progress: float
    lane_change_recenter_start_progress: float
    lane_change_continuation: bool
    throttle: float
    brake: float
    detected_vehicles: tuple[DetectedVehicle, ...]
    radar_points: tuple[RadarPoint, ...] = ()
    corner_radar_supported: bool = False
    tpms: TpmsInfo = TpmsInfo()
    ev_mode_valid: bool = False
    ev_mode_active: bool = False
    display_speed_kph: float | None = None
    traffic_state: int = 0
    driving_mode: int | None = None
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
    cutin_debug_text: str = "NEW CUTIN: waiting"
    recorded_cutin_active: bool = False
    recorded_cutin_sound: bool = False
    trip_report: TripReportState | None = None
    cpu_usage_percent: float | None = None
    cpu_temp_c: float | None = None
    memory_used_percent: float | None = None
    disk_used_percent: float | None = None
    alert: ClusterAlert | None = None


@dataclass
class ReplayCutinTrack:
    track_id: int
    cnt: int = 0
    cut_in_count: int = 0
    cut_in_start_abs_dpath: float = 0.0
    measured: bool = False
    d_rel: float = 0.0
    y_rel: float = 0.0
    v_rel: float = 0.0
    v_lead: float = 0.0
    yv_rel: float = 0.0
    d_path: float = 0.0
    d_path_future: float = 0.0
    in_lane_prob: float = 0.0
    in_lane_prob_future: float = 0.0
    d_path_rate: float = 0.0
    inward_speed: float = 0.0
    path_d_path: float = 0.0
    path_d_path_future: float = 0.0
    path_in_lane_prob: float = 0.0
    path_in_lane_prob_future: float = 0.0
    path_d_path_rate: float = 0.0
    path_inward_speed: float = 0.0
    path_y_std: float = float("inf")
    radar_inward_speed: float = 0.0
    side_corner_confirmed_count: int = 0
    rejection_reason: str = "waiting"
    position_history: Any = None
    path_position_history: Any = None

    def __post_init__(self) -> None:
        if self.position_history is None:
            self.position_history = new_cutin_position_history(REPLAY_CUTIN_DT)
        if self.path_position_history is None:
            self.path_position_history = new_cutin_position_history(REPLAY_CUTIN_DT)


@dataclass(frozen=True, slots=True)
class RouteVideoSegment:
    index: int | None
    path: Path
    start_t: float
    end_t: float


@dataclass(frozen=True, slots=True)
class RouteVideoFrame:
    rgba: bytes
    width: int
    height: int
    frame_id: str


@dataclass(slots=True)
class RouteReplayChunk:
    index: int
    path: Path
    frames: list[RouteReplayFrame]
    start_t: float
    end_t: float


@dataclass(slots=True)
class RouteReplayParsedFile:
    index: int
    path: Path
    frames: list[RouteReplayFrame]


@dataclass(slots=True)
class RouteReplayWorkerResult:
    generation: int
    index: int
    path: str
    frames: list[RouteReplayFrame] | None = None
    error: str | None = None


def normalize_route_frames(frames: list[RouteReplayFrame], first_t: float) -> list[RouteReplayFrame]:
    normalized = [replace(frame, t=frame.t - first_t) for frame in frames]
    normalized.sort(key=lambda frame: frame.t)
    return normalized


class RouteLogPreloadWorker:
    def __init__(self, corner_source: str = ROUTE_CORNER_SOURCE_LIVE) -> None:
        self.corner_source = corner_source
        self._requests: Any | None = None
        self._results: Any | None = None
        self._worker: Any | None = None
        self._start()

    def request(self, generation: int, file_index: int, file_path: Path) -> None:
        if self._requests is None:
            self._start()
        if self._requests is None:
            raise RuntimeError("route preload worker is not available")
        self._requests.put(("parse", generation, file_index, str(file_path)))

    def receive(self, block: bool) -> RouteReplayWorkerResult | None:
        if self._results is None:
            return None
        while True:
            try:
                if block:
                    return self._results.get(timeout=0.1)
                return self._results.get_nowait()
            except queue.Empty:
                if not block:
                    return None
                worker = self._worker
                if worker is not None and not worker.is_alive():
                    raise RuntimeError("route preload worker exited")

    def restart(self) -> None:
        self.close()
        self._start()

    def close(self) -> None:
        requests = self._requests
        worker = self._worker
        if worker is not None and worker.is_alive() and requests is not None:
            try:
                requests.put(("stop", 0, 0, ""), block=False)
            except Exception:
                pass
            worker.join(timeout=0.5)
            if worker.is_alive():
                terminate = getattr(worker, "terminate", None)
                if callable(terminate):
                    terminate()
                    worker.join(timeout=0.5)
        self._requests = None
        self._results = None
        self._worker = None

    def _start(self) -> None:
        if sys.platform != "win32":
            try:
                context = multiprocessing.get_context("fork")
                self._requests = context.Queue(maxsize=2)
                self._results = context.Queue(maxsize=2)
                self._worker = context.Process(
                    target=route_log_preload_worker,
                    args=(self._requests, self._results, self.corner_source, True),
                    name="route-log-preload",
                    daemon=True,
                )
                self._worker.start()
                return
            except Exception:
                self._requests = None
                self._results = None
                self._worker = None

        self._requests = queue.Queue(maxsize=2)
        self._results = queue.Queue()
        self._worker = threading.Thread(
            target=route_log_preload_worker,
            args=(self._requests, self._results, self.corner_source, False),
            name="route-log-preload",
            daemon=True,
        )
        self._worker.start()


def route_log_preload_worker(
    requests: Any,
    results: Any,
    corner_source: str = ROUTE_CORNER_SOURCE_LIVE,
    low_priority: bool = False,
) -> None:
    if low_priority:
        try:
            os.nice(ROUTE_REPLAY_PRELOAD_NICE)
        except OSError:
            pass
    log_schema = load_openpilot_log_schema()
    parser = RouteLogParser(corner_source, recompute_cutins=False)
    first_t: float | None = None
    while True:
        command, generation, file_index, file_path_text = requests.get()
        if command == "stop":
            return
        if command != "parse":
            continue
        try:
            file_path = Path(file_path_text)
            frames = parser.parse_file(file_path, log_schema)
            if frames:
                if first_t is None:
                    first_t = min(frame.t for frame in frames)
                frames = normalize_route_frames(frames, first_t)
            results.put(RouteReplayWorkerResult(generation, file_index, file_path_text, frames=frames))
        except BaseException:
            results.put(
                RouteReplayWorkerResult(
                    generation,
                    file_index,
                    file_path_text,
                    error=traceback.format_exc(),
                )
            )


class RouteReplaySource:
    def __init__(
        self,
        source_files: list[Path],
        corner_yaw_comp_gain: float = 0.0,
        corner_source: str = ROUTE_CORNER_SOURCE_LIVE,
        corner_lateral_offset_m: float = CORNER_DISPLAY_CENTERING_DEFAULT_M,
    ) -> None:
        if not source_files:
            raise RuntimeError("route contains no log files")
        self.corner_yaw_comp_gain = corner_yaw_comp_gain
        self.corner_lateral_offset_m = corner_lateral_offset_m
        self.corner_source = route_corner_source_or_default(corner_source)
        self.source_files = source_files
        self.frames: list[RouteReplayFrame] = []
        self.times: list[float] = []
        self.duration = 0.0
        self.video_segments: list[RouteVideoSegment] = []
        self._video_reader = RouteVideoFrameReader(self.video_segments)
        self._preload_worker = RouteLogPreloadWorker(self.corner_source)
        self._next_file_index = 0
        self._loaded_chunks: list[RouteReplayChunk] = []
        self._loaded_file_count = 0
        self._end_of_route = False
        self._preload_active = False
        self._preload_file_index: int | None = None
        self._preload_file_path: Path | None = None
        self._preload_generation = 0
        self._ensure_loaded(0.0)
        if not self.frames:
            raise RuntimeError("route contains no carState frames")

    @classmethod
    def load(
        cls,
        route_path: Path,
        log_kind: str = "qlog",
        start_segment: int | None = None,
        max_segments: int | None = None,
        corner_yaw_comp_gain: float = 0.0,
        corner_source: str = ROUTE_CORNER_SOURCE_LIVE,
        corner_lateral_offset_m: float = CORNER_DISPLAY_CENTERING_DEFAULT_M,
    ) -> RouteReplaySource:
        files = discover_route_logs(route_path, log_kind, start_segment, max_segments)
        if not files:
            raise RuntimeError(f"no {LOG_FILENAMES[log_kind]} files found under {route_path}")

        return cls(files, corner_yaw_comp_gain, corner_source, corner_lateral_offset_m)

    def is_finished(self, playback_seconds: float, loop: bool = False) -> bool:
        if not loop:
            self._ensure_loaded(playback_seconds)
        return not loop and playback_seconds > self.duration

    def state_at(
        self,
        playback_seconds: float,
        loop: bool = False,
        include_overlay: bool = False,
    ) -> ClusterUiState:
        if loop and self._end_of_route and self.duration > 0.0:
            playback_seconds %= self.duration
        self._ensure_loaded(playback_seconds)
        if self.duration <= 0.0:
            state = self._frame_to_state(self.frames[0])
            return self._with_overlay(state, self.frames[0], 0.0, loop) if include_overlay else state
        if not loop or self._end_of_route:
            playback_seconds = clamp(playback_seconds, 0.0, self.duration)

        right_index = bisect_right(self.times, playback_seconds)
        if right_index <= 0:
            state = self._frame_to_state(self.frames[0])
            return self._with_overlay(state, self.frames[0], playback_seconds, loop) if include_overlay else state
        if right_index >= len(self.frames):
            state = self._frame_to_state(self.frames[-1])
            return self._with_overlay(state, self.frames[-1], playback_seconds, loop) if include_overlay else state

        left = self.frames[right_index - 1]
        right = self.frames[right_index]
        span = max(0.001, right.t - left.t)
        amount = clamp((playback_seconds - left.t) / span, 0.0, 1.0)
        frame = blend_frames(left, right, amount)
        state = self._frame_to_state(frame)
        return self._with_overlay(state, frame, playback_seconds, loop) if include_overlay else state

    def close(self) -> None:
        self._preload_generation += 1
        self._preload_active = False
        self._preload_file_index = None
        self._preload_file_path = None
        self._preload_worker.close()
        if self._video_reader is not None:
            self._video_reader.close()

    def status_text(self, playback_seconds: float, loop: bool = False) -> str:
        shown_time = (
            playback_seconds % self.duration
            if loop and self._end_of_route and self.duration > 0.0
            else playback_seconds
        )
        shown_time = clamp(shown_time, 0.0, self.duration)
        frame = self._status_frame_at(shown_time)
        radar_count = len(frame.radar_points) if frame is not None else 0
        detected_count = len(frame.detected_vehicles) if frame is not None else 0
        file_count = len(self.source_files)
        return (
            f"route t={shown_time:6.1f}/{self.duration:6.1f}s "
            f"files={self._loaded_file_count}/{file_count} "
            f"radar={radar_count} "
            f"detected={detected_count}"
            f" corner={self.corner_source}"
            f"{f' yawHead={self.corner_yaw_comp_gain:+.2f}' if self.corner_yaw_comp_gain else ''}"
            f"{f' yCenter={self.corner_lateral_offset_m:.2f}m' if self.corner_lateral_offset_m else ''}"
        )

    def _frame_to_state(self, frame: RouteReplayFrame) -> ClusterUiState:
        state = frame_to_state(frame)
        if self.corner_yaw_comp_gain == 0.0 and self.corner_lateral_offset_m == 0.0:
            return state
        radar_points = tuple(
            replay_corner_adjusted_point(
                point,
                frame.vision_yaw_rate_rps,
                self.corner_yaw_comp_gain,
                self.corner_lateral_offset_m,
                frame.lane_width_m,
            )
            for point in state.radar_points
        )
        return replace(state, radar_points=radar_points)

    @property
    def loaded_file_count(self) -> int:
        return self._loaded_file_count

    def log_path_at(self, playback_seconds: float) -> Path:
        for chunk in self._loaded_chunks:
            if chunk.start_t - 0.5 <= playback_seconds <= chunk.end_t + 0.5:
                return chunk.path
        if self._loaded_chunks:
            nearest = min(
                self._loaded_chunks,
                key=lambda chunk: min(abs(playback_seconds - chunk.start_t), abs(playback_seconds - chunk.end_t)),
            )
            return nearest.path
        return self.source_files[0]

    def log_folder_at(self, playback_seconds: float) -> Path:
        return self.log_path_at(playback_seconds).parent

    def log_offset_at(self, playback_seconds: float) -> float:
        log_path = self.log_path_at(playback_seconds)
        chunk = next((item for item in self._loaded_chunks if item.path == log_path), None)
        if chunk is None:
            return 0.0
        return clamp(playback_seconds - chunk.start_t, 0.0, max(0.0, chunk.end_t - chunk.start_t))

    def _status_frame_at(self, playback_seconds: float) -> RouteReplayFrame | None:
        if not self.frames:
            return None
        if playback_seconds <= self.times[0]:
            return self.frames[0]
        right_index = bisect_right(self.times, playback_seconds)
        if right_index >= len(self.frames):
            return self.frames[-1]
        left = self.frames[right_index - 1]
        right = self.frames[right_index]
        span = max(0.001, right.t - left.t)
        amount = clamp((playback_seconds - left.t) / span, 0.0, 1.0)
        return blend_frames(left, right, amount)

    def _ensure_loaded(self, playback_seconds: float) -> None:
        if self.frames and playback_seconds < self.frames[0].t and not self._end_of_route:
            self._reset_stream()

        self._finish_preload_if_ready(playback_seconds)

        while not self._end_of_route and (
            not self.frames
            or len(self._loaded_chunks) < ROUTE_REPLAY_START_BUFFER_FILES
            or playback_seconds >= self.duration - ROUTE_REPLAY_READAHEAD_S
        ):
            if not self._load_next_file():
                break

        self._trim_loaded_chunks(playback_seconds)
        self._start_preload()

    def _finish_preload_if_ready(self, playback_seconds: float) -> bool:
        if not self._preload_active or not self.frames:
            return False
        if playback_seconds < self.duration - ROUTE_REPLAY_PRELOAD_READY_AHEAD_S:
            return False
        return self._finish_preload(block=False)

    def _load_next_file(self) -> bool:
        if self._preload_active:
            return self._finish_preload(block=True)

        if self._next_file_index >= len(self.source_files):
            self._end_of_route = True
            return False

        file_index = self._next_file_index
        file_path = self.source_files[file_index]
        self._next_file_index += 1
        self._request_preload(file_index, file_path)
        return self._finish_preload(block=True)

    def _start_preload(self) -> None:
        if (
            self._end_of_route
            or self._preload_active
            or self._next_file_index >= len(self.source_files)
        ):
            return

        file_index = self._next_file_index
        file_path = self.source_files[file_index]
        self._next_file_index += 1
        self._request_preload(file_index, file_path)

    def _request_preload(self, file_index: int, file_path: Path) -> None:
        self._preload_active = True
        self._preload_file_index = file_index
        self._preload_file_path = file_path
        self._preload_worker.request(self._preload_generation, file_index, file_path)

    def _finish_preload(self, block: bool) -> bool:
        if not self._preload_active:
            return False

        result = self._preload_worker.receive(block=block)
        if result is None:
            return False

        file_path = self._preload_file_path
        self._preload_active = False
        self._preload_file_index = None
        self._preload_file_path = None

        if result.generation != self._preload_generation:
            return False
        if result.error is not None:
            raise RuntimeError(f"failed to preload route log {file_path}:\n{result.error}")
        if result.frames is None:
            return False
        return self._append_parsed_file(RouteReplayParsedFile(result.index, Path(result.path), result.frames))

    def _stop_preload(self, wait: bool) -> None:
        self._preload_generation += 1
        if self._preload_active and wait:
            self._finish_preload(block=True)
        self._preload_active = False
        self._preload_file_index = None
        self._preload_file_path = None
        self._preload_worker.restart()

    def _append_parsed_file(self, parsed_file: RouteReplayParsedFile) -> bool:
        file_index = parsed_file.index
        file_path = parsed_file.path
        parsed_frames = parsed_file.frames
        self._loaded_file_count = max(self._loaded_file_count, self._next_file_index)
        if not parsed_frames:
            return True

        chunk = RouteReplayChunk(
            index=file_index,
            path=file_path,
            frames=parsed_frames,
            start_t=parsed_frames[0].t,
            end_t=parsed_frames[-1].t,
        )
        self._loaded_chunks.append(chunk)
        self._append_video_segment(file_path, chunk)
        self._rebuild_frame_index()
        return True

    def _append_video_segment(self, file_path: Path, chunk: RouteReplayChunk) -> None:
        video_path = file_path.parent / "qcamera.ts"
        if not video_path.exists():
            return
        self.video_segments.append(
            RouteVideoSegment(
                index=segment_index(file_path),
                path=video_path,
                start_t=chunk.start_t,
                end_t=chunk.end_t,
            )
        )

    def _trim_loaded_chunks(self, playback_seconds: float) -> None:
        removed = False
        while (
            len(self._loaded_chunks) > ROUTE_REPLAY_MIN_BUFFER_FILES
            and self._loaded_chunks[0].end_t < playback_seconds - ROUTE_REPLAY_RETAIN_BEHIND_S
        ):
            self._loaded_chunks.pop(0)
            removed = True
        if removed:
            self._rebuild_frame_index()

    def _rebuild_frame_index(self) -> None:
        self.frames = [
            frame
            for chunk in self._loaded_chunks
            for frame in chunk.frames
        ]
        self.frames.sort(key=lambda frame: frame.t)
        self.times = [frame.t for frame in self.frames]
        if self.frames:
            self.duration = max(self.duration, self.frames[-1].t)

    def _reset_stream(self) -> None:
        self._stop_preload(wait=True)
        if self._video_reader is not None:
            self._video_reader.close()
        self.frames = []
        self.times = []
        self.duration = 0.0
        self.video_segments = []
        self._video_reader = RouteVideoFrameReader(self.video_segments)
        self._next_file_index = 0
        self._loaded_chunks = []
        self._loaded_file_count = 0
        self._end_of_route = False

    def _with_overlay(
        self,
        state: ClusterUiState,
        frame: RouteReplayFrame,
        playback_seconds: float,
        loop: bool,
    ) -> ClusterUiState:
        overlay = self._route_overlay(frame, state, playback_seconds, loop)
        return replace(state, route_overlay=overlay)

    def _route_overlay(
        self,
        frame: RouteReplayFrame,
        state: ClusterUiState,
        playback_seconds: float,
        loop: bool,
    ) -> RouteOverlay:
        shown_time = playback_seconds % self.duration if loop and self.duration > 0.0 else playback_seconds
        shown_time = clamp(shown_time, 0.0, self.duration)
        segment = route_video_segment_at(self.video_segments, shown_time)
        segment_label = "--" if segment is None or segment.index is None else str(segment.index)
        video_frame = self._video_reader.frame_at(shown_time) if self._video_reader is not None else None
        signal_text = ("L" if frame.left_signal else "-") + ("R" if frame.right_signal else "-")
        lane_offset_text = "--" if frame.lane_center_offset_m is None else f"{frame.lane_center_offset_m:+.2f}m"
        limit_source = frame.speed_limit_source or "-"
        limit_text = "--" if frame.speed_limit_kph is None else f"{frame.speed_limit_kph:d}:{limit_source}"
        cruise_text = "--" if frame.cruise_kph is None else f"{frame.cruise_kph:d}"
        curve_text = "--" if frame.road_curvature is None else f"{frame.road_curvature:+.5f}"
        detected_text = detected_vehicle_summary(frame.detected_vehicles)
        radar_text = radar_point_summary(frame.radar_points)
        lane_change_text = "idle" if frame.lane_change is None else f"{frame.lane_change}:{frame.lane_change_progress:.2f}"
        plan_speed_text = "--" if frame.planned_speed_kph is None else f"{frame.planned_speed_kph:.0f}"
        plan_accel_text = "--" if frame.planned_accel_mps2 is None else f"{frame.planned_accel_mps2:+.1f}"
        turn_speed_text = "--" if frame.model_turn_speed_kph is None else f"{frame.model_turn_speed_kph:.0f}"
        engaged_text = "--" if frame.engaged_prob is None else f"{frame.engaged_prob:.0%}"
        lead_ttc_text = nearest_ttc_summary(frame.detected_vehicles)
        vision_text = "--" if frame.vision_speed_mps is None else f"{frame.vision_speed_mps * 3.6:.1f}kph"
        frame_drop_text = "--" if frame.frame_drop_perc is None else f"{frame.frame_drop_perc:.1f}%"
        model_time_text = "--" if frame.model_execution_time_ms is None else f"{frame.model_execution_time_ms:.0f}ms"
        confidence_text = frame.model_confidence or "--"
        availability_text = (
            ("L" if frame.lane_change_available_left else "-")
            + ("R" if frame.lane_change_available_right else "-")
        )
        gear_text = frame.gear_text or "--"
        gap_text = "--" if frame.cruise_gap is None else f"{frame.cruise_gap:d}"
        data_lines = (
            f"device cutin {'YES' if frame.recorded_cutin_active else 'NO'}   "
            f"sound {'PROMPT' if frame.recorded_cutin_sound else '--'}",
            f"t {shown_time:6.1f}/{self.duration:6.1f}s   seg {segment_label}",
            f"vEgo {state.speed_kph:5.1f} km/h   aEgo {state.accel_mps2:+.2f} m/s2",
            f"steer {frame.steering_angle_deg or 0.0:+.1f} deg   limit {limit_text}   cruise {cruise_text}",
            f"gear {gear_text}   gap {gap_text}   signals {signal_text}",
            f"curve {curve_text}   plan {plan_speed_text}kph {plan_accel_text}m/s2",
            f"lane {frame.lane_width_m:.2f}m center {lane_offset_text}   src {frame.lane_position_source}",
            f"lc {lane_change_text} avail {availability_text} p{frame.lane_change_prob:.2f}",
            f"model {confidence_text} eng {engaged_text} risk {frame.disengage_risk:.2f} hb {frame.hard_brake_risk:.2f}",
            f"turn {turn_speed_text}kph ttc {lead_ttc_text} stop {int(frame.should_stop)} drop {frame_drop_text} exec {model_time_text}",
            f"vision {vision_text} yaw {frame.vision_yaw_rate_rps or 0.0:+.3f}   detected {detected_text}",
            f"radar points {radar_text}",
        )

        if video_frame is None:
            status = self._video_reader.status_text() if self._video_reader is not None else "qcamera unavailable"
            return RouteOverlay(video_status=status, cutin_status=frame.cutin_debug_text, data_lines=data_lines)
        return RouteOverlay(
            video_rgba=video_frame.rgba,
            video_width=video_frame.width,
            video_height=video_frame.height,
            video_frame_id=video_frame.frame_id,
            cutin_status=frame.cutin_debug_text,
            data_lines=data_lines,
        )


def route_video_ffmpeg_path() -> str | None:
    env_path = os.environ.get(ROUTE_VIDEO_FFMPEG_ENV, "").strip()
    if env_path:
        resolved = shutil.which(env_path) or env_path
        if Path(resolved).exists() or shutil.which(resolved):
            return resolved

    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is not None:
        return ffmpeg

    try:
        import imageio_ffmpeg  # type: ignore

        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


class RouteVideoFrameReader:
    def __init__(self, segments: list[RouteVideoSegment]) -> None:
        self.segments = segments
        self._ffmpeg = route_video_ffmpeg_path()
        self._process: subprocess.Popen[bytes] | None = None
        self._segment_key: tuple[int | None, str, float, float] | None = None
        self._frame_index = -1
        self._last_frame: RouteVideoFrame | None = None
        self._status = "qcamera waiting"

    def frame_at(self, playback_seconds: float) -> RouteVideoFrame | None:
        segment = route_video_segment_at(self.segments, playback_seconds)
        if segment is None:
            self._close_process()
            self._status = "qcamera missing"
            return None
        if not segment.path.exists():
            self._close_process()
            self._status = "qcamera file missing"
            return None
        if self._ffmpeg is None:
            self._status = "qcamera ffmpeg missing"
            return None

        local_time_s = clamp(playback_seconds - segment.start_t, 0.0, max(0.0, segment.end_t - segment.start_t))
        target_frame = max(0, int(local_time_s * ROUTE_VIDEO_FPS))
        segment_key = self._key_for_segment(segment)
        needs_restart = (
            self._process is None
            or self._segment_key != segment_key
            or target_frame < self._frame_index
            or target_frame - self._frame_index > ROUTE_VIDEO_SEEK_RESTART_FRAMES
        )
        if needs_restart and not self._open_segment(segment, target_frame):
            return self._last_frame

        while self._frame_index < target_frame:
            if not self._read_next_frame():
                return self._last_frame
        self._status = ""
        return self._last_frame

    def status_text(self) -> str:
        return self._status or "qcamera unavailable"

    def close(self) -> None:
        self._close_process()

    def _open_segment(self, segment: RouteVideoSegment, start_frame: int) -> bool:
        self._close_process()
        if self._ffmpeg is None:
            self._status = "qcamera ffmpeg missing"
            return False
        seek_s = max(0.0, start_frame / ROUTE_VIDEO_FPS)
        command = [
            self._ffmpeg,
            "-hide_banner",
            "-loglevel",
            "error",
            "-threads",
            "1",
            "-i",
            str(segment.path),
            "-ss",
            f"{seek_s:.3f}",
            "-an",
            "-sn",
            "-vf",
            f"fps={ROUTE_VIDEO_FPS:g},scale={ROUTE_VIDEO_DECODE_WIDTH}:{ROUTE_VIDEO_DECODE_HEIGHT}",
            "-pix_fmt",
            "rgba",
            "-f",
            "rawvideo",
            "pipe:1",
        ]
        try:
            self._process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
        except OSError as exc:
            self._status = f"qcamera ffmpeg failed: {exc}"
            self._process = None
            return False
        self._segment_key = self._key_for_segment(segment)
        self._frame_index = start_frame - 1
        self._last_frame = None
        self._status = "qcamera starting"
        return True

    def _read_next_frame(self) -> bool:
        process = self._process
        if process is None or process.stdout is None:
            self._status = "qcamera unavailable"
            return False
        frame_size = ROUTE_VIDEO_DECODE_WIDTH * ROUTE_VIDEO_DECODE_HEIGHT * 4
        data = self._read_exact(process.stdout, frame_size)
        if data is None:
            self._status = "qcamera ended"
            self._close_process()
            return False
        self._frame_index += 1
        frame_id = f"{self._segment_key}:{self._frame_index}"
        self._last_frame = RouteVideoFrame(
            rgba=data,
            width=ROUTE_VIDEO_DECODE_WIDTH,
            height=ROUTE_VIDEO_DECODE_HEIGHT,
            frame_id=frame_id,
        )
        return True

    @staticmethod
    def _read_exact(stream: Any, size: int) -> bytes | None:
        chunks: list[bytes] = []
        remaining = size
        while remaining > 0:
            chunk = stream.read(remaining)
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return chunks[0] if len(chunks) == 1 else b"".join(chunks)

    @staticmethod
    def _key_for_segment(segment: RouteVideoSegment) -> tuple[int | None, str, float, float]:
        return segment.index, str(segment.path), segment.start_t, segment.end_t

    def _close_process(self) -> None:
        process = self._process
        self._process = None
        self._segment_key = None
        self._frame_index = -1
        if process is None:
            return
        if process.stdout is not None:
            try:
                process.stdout.close()
            except OSError:
                pass
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=0.3)
            except subprocess.TimeoutExpired:
                process.kill()
                try:
                    process.wait(timeout=0.3)
                except subprocess.TimeoutExpired:
                    pass


class RouteLogParser:
    def __init__(
        self,
        corner_source: str = ROUTE_CORNER_SOURCE_LIVE,
        reconstruct_corner_live_tracks: bool = False,
        cutin_radar_source: str | None = None,
        recompute_cutins: bool = True,
    ) -> None:
        self.corner_source = route_corner_source_or_default(corner_source)
        self.reconstruct_corner_live_tracks = reconstruct_corner_live_tracks
        # Normal route playback shows the radarState emitted on the device.
        # Focused validation tools can still opt into offline recomputation.
        self.show_recorded_cutins = True
        self.recompute_cutins = recompute_cutins
        self.front_radar_only = os.environ.get(ROUTE_FRONT_RADAR_ONLY_ENV) == "1"
        self.cutin_radar_source = cutin_radar_source or os.environ.get(
            ROUTE_CUTIN_RADAR_SOURCE_ENV, ROUTE_CUTIN_RADAR_SOURCE_CORNER
        )
        if self.cutin_radar_source not in (ROUTE_CUTIN_RADAR_SOURCE_CORNER, ROUTE_CUTIN_RADAR_SOURCE_FRONT):
            self.cutin_radar_source = ROUTE_CUTIN_RADAR_SOURCE_CORNER
        try:
            self.cutin_sensitivity = clamp(float(os.environ.get(ROUTE_CUTIN_SENSITIVITY_ENV, "50")), 0.0, 100.0)
        except ValueError:
            self.cutin_sensitivity = 50.0
        self.cutin_tuning = cutin_tuning_from_sensitivity(self.cutin_sensitivity)
        self.cutin_confirm_frames = max(1, int(round(self.cutin_tuning["confirm_s"] / REPLAY_CUTIN_DT)))
        if self.cutin_radar_source == ROUTE_CUTIN_RADAR_SOURCE_FRONT:
            self.cutin_confirm_frames = max(
                self.cutin_confirm_frames,
                int(round(FRONT_CUTIN_MIN_CONFIRM_S / REPLAY_CUTIN_DT)),
            )
        self.cutin_min_track_age = max(1, int(round(self.cutin_tuning["min_track_age_s"] / REPLAY_CUTIN_DT)))
        self.cutin_lane_xs: tuple[float, ...] = ()
        self.cutin_left_ys: tuple[float, ...] = ()
        self.cutin_right_ys: tuple[float, ...] = ()
        self.cutin_path_xs: tuple[float, ...] = ()
        self.cutin_path_ys: tuple[float, ...] = ()
        self.cutin_path_y_stds: tuple[float, ...] = ()
        self.cutin_yaw_rate = 0.0
        self.cutin_tracks: dict[int, ReplayCutinTrack] = {}
        self.cutin_side_corner_front_matches: dict[int, int] = {}
        self.cutin_side_corner_front_match_misses: dict[int, int] = {}
        self.cutin_corner_object_ids: dict[tuple[str, int], tuple[int, int]] = {}
        self.next_cutin_corner_track_id = STABLE_CORNER_TRACK_ID_START
        self.cutin_detections: tuple[DetectedVehicle, ...] = ()
        self.cutin_detection_t = -999.0
        self.cutin_output_hold_count = 0
        self.cutin_output_hold_reference: tuple[float, float, float] | None = None
        self.cutin_debug_text = f"NEW CUTIN {self.cutin_radar_source.upper()} S{self.cutin_sensitivity:.0f}: waiting"
        self.recorded_cutin_ids: set[int] = set()
        self.recorded_cutin_sound = False
        self.recorded_cutin_t = -999.0
        self.recorded_cutin_sound_t = -999.0
        self.recorded_cutin_detections: tuple[DetectedVehicle, ...] = ()
        self.lead_one_status = False
        self.lead_one_d_rel = 0.0
        self.lead_one_v_rel = 0.0
        self.lead_one_radar = False
        self.lead_one_track_id = -1
        self.speed_limit_kph: int | None = None
        self.speed_limit_source: str | None = None
        self.nav_speed_limit_kph: int | None = None
        self.nav_speed_limit_t = -999.0
        self.cruise_kph: int | None = None
        self.cruise_gap: int | None = None
        self.lfa_active: bool | None = None
        self.active_lane_line: bool | None = None
        self.selfdrive_enabled: bool | None = None
        self.controls_enabled: bool | None = None
        self.alert: ClusterAlert | None = None
        self.wheelbase_m = 2.7
        self.steer_ratio = 15.0
        self.trip_report_tracker = TripReportTracker()
        self.lane_width_m = DEFAULT_LANE_WIDTH_M
        self.left_lane_y_m: float | None = None
        self.right_lane_y_m: float | None = None
        self.outer_left_lane_y_m: float | None = None
        self.outer_right_lane_y_m: float | None = None
        self.left_road_edge_y_m: float | None = None
        self.right_road_edge_y_m: float | None = None
        self.left_lane_prob = 1.0
        self.right_lane_prob = 1.0
        self.outer_left_lane_prob = 0.0
        self.outer_right_lane_prob = 0.0
        self.left_road_edge_confidence = 0.0
        self.right_road_edge_confidence = 0.0
        self.left_lane_style = "solid"
        self.right_lane_style = "solid"
        self.left_lane_color: tuple[int, int, int] | None = None
        self.right_lane_color: tuple[int, int, int] | None = None
        self.lane_position_source = "default"
        self.model_curvature_m_inv: float | None = None
        self.model_curvature_source = "steeringAngleDeg"
        self.controls_curvature_m_inv: float | None = None
        self.controls_curvature_source = "steeringAngleDeg"
        self.model_lane_lines: tuple[tuple[ModelPathPoint, ...], ...] = ()
        self.model_road_edges: tuple[tuple[ModelPathPoint, ...], ...] = ()
        self.model_path: tuple[ModelPathPoint, ...] = ()
        self.model_path_source = "none"
        self.model_detections: tuple[DetectedVehicle, ...] = ()
        self.model_detection_t = -999.0
        self.planned_speed_kph: float | None = None
        self.planned_accel_mps2: float | None = None
        self.model_action_curvature_m_inv: float | None = None
        self.should_stop = False
        self.model_confidence: str | None = None
        self.model_turn_speed_kph: float | None = None
        self.engaged_prob: float | None = None
        self.desire_state: tuple[float, ...] = ()
        self.desire_prediction: tuple[tuple[float, ...], ...] = ()
        self.risk_points: tuple[ModelRiskPoint, ...] = ()
        self.brake_disengage_risk = 0.0
        self.gas_disengage_risk = 0.0
        self.steer_override_risk = 0.0
        self.hard_brake_risk = 0.0
        self.gas_press_prob = 0.0
        self.brake_press_prob = 0.0
        self.disengage_risk = 0.0
        self.hard_brake_predicted = False
        self.frame_age: int | None = None
        self.frame_drop_perc: float | None = None
        self.model_execution_time_ms: float | None = None
        self.vision_speed_mps: float | None = None
        self.vision_yaw_rate_rps: float | None = None
        self.vision_speed_std_mps: float | None = None
        self.vision_yaw_rate_std_rps: float | None = None
        self.camera_device_type: str | None = None
        self.camera_sensor: str | None = None
        self.cpu_usage_percent: float | None = None
        self.cpu_temp_c: float | None = None
        self.memory_used_percent: float | None = None
        self.disk_used_percent: float | None = None
        self.camera_calibration_euler: tuple[float, float, float] | None = None
        self.road_transform_trans: tuple[float, float, float] | None = None
        self.road_transform_std: tuple[float, float, float] | None = None
        self.camera_odometry_valid: bool | None = None
        self.longitudinal_plan_source: str | None = None
        self.longitudinal_plan_speeds_kph: tuple[float, ...] = ()
        self.longitudinal_plan_accels_mps2: tuple[float, ...] = ()
        self.longitudinal_plan_jerks_mps3: tuple[float, ...] = ()
        self.longitudinal_plan_fcw = False
        self.longitudinal_plan_should_stop = False
        self.longitudinal_plan_allow_throttle: bool | None = None
        self.longitudinal_plan_allow_brake: bool | None = None
        self.traffic_state = 0
        self.driving_mode: int | None = None
        self.longitudinal_t_follow_s: float | None = None
        self.longitudinal_desired_distance_m: float | None = None
        self.longitudinal_v_target_kph: float | None = None
        self.longitudinal_jerk_target_mps3: float | None = None
        self.lateral_plan_valid: bool | None = None
        self.lateral_plan_use_lane_lines: bool | None = None
        self.lateral_plan_solver_cost: float | None = None
        self.lateral_plan_debug_text: str | None = None
        self.lateral_plan_curvatures: tuple[float, ...] = ()
        self.lateral_plan_curvature_rates: tuple[float, ...] = ()
        self.lane_change_available_left: bool | None = None
        self.lane_change_available_right: bool | None = None
        self.left_lane_width_m: float | None = None
        self.right_lane_width_m: float | None = None
        self.left_road_edge_distance_m: float | None = None
        self.right_road_edge_distance_m: float | None = None
        self.model_lane_change_seen = False
        self.lane_change_source = "none"
        self.lane_change_ll_prob = 1.0
        self.lane_change_desire_left_prob = 0.0
        self.lane_change_desire_right_prob = 0.0
        self.lane_change_state = "off"
        self.lane_change_direction = "none"
        self.lane_change_started_t: float | None = None
        self.active_lane_change_direction: str | None = None
        self.lane_change_last_progress = 0.0
        self.lane_change_recenter_direction: str | None = None
        self.lane_change_recenter_started_t: float | None = None
        self.lane_change_recenter_start_progress = 1.0
        self.lane_change_continuation_active = False
        self.lane_change_previous_state = "off"
        self.lane_change_peak_directional_observed_offset = 0.0
        self.ccnc_corner_detections: dict[str, DetectedVehicle] = {}
        self.ccnc_corner_message_t = -999.0
        self.adrv_corner_detections: dict[str, DetectedVehicle] = {}
        self.adrv_corner_message_t = -999.0
        self.adrv_lane_changing = 0
        self.adrv_lane_changing_t = -999.0
        self.corner_radar_supported = False
        self.car_brand = ""
        self.corner_radar_tracks_seen = False
        self.raw_corner_tracker = StableCornerObjectTracker()
        self.raw_corner_objects: dict[tuple[str, int], RawCornerObject] = {}
        self.raw_corner_track_t = -999.0
        self.live_track_radar_points: dict[str, RadarPoint] = {}
        self.live_track_radar_t = -999.0
        self.radar_detections: tuple[DetectedVehicle, ...] = ()
        self.radar_detection_t = -999.0
        self.current_speed_kph = 0.0
        self.v_ego_cluster_seen = False

    def parse_file(self, file_path: Path, log_schema: Any) -> list[RouteReplayFrame]:
        frames: list[RouteReplayFrame] = []
        data = read_log_bytes(file_path)
        if not self.car_brand:
            for event in log_schema.Event.read_multiple_bytes(data):
                if safe_which(event) == "carParams":
                    self._update_car_params(event.carParams)
                    break
        for event in log_schema.Event.read_multiple_bytes(data):
            event_type = safe_which(event)
            if event_type is None:
                continue
            event_t = float(getattr(event, "logMonoTime", 0)) / 1_000_000_000.0
            if event_type == "carState":
                frames.append(self._frame_from_car_state(event.carState, event_t))
            elif event_type == "drivingModelData":
                self._update_driving_model(event.drivingModelData)
            elif event_type == "modelV2":
                self._update_model_v2(event.modelV2, event_t)
            elif event_type == "lateralPlan":
                self._update_lateral_plan(event.lateralPlan)
            elif event_type in ("navInstructionCarrot", "navInstruction"):
                self._update_nav_instruction(getattr(event, event_type), event_t)
            elif event_type == "longitudinalPlan":
                self._update_longitudinal_plan(
                    event.longitudinalPlan,
                    bool(safe_get(event, "valid", True)),
                )
            elif event_type == "controlsState":
                self._update_controls_state(event.controlsState)
            elif event_type == "selfdriveState":
                self._update_selfdrive_state(event.selfdriveState, event_t)
            elif event_type == "carControl":
                self._update_car_control(event.carControl)
            elif event_type == "deviceState":
                self._update_device_state(event.deviceState)
            elif event_type == "roadCameraState":
                self._update_road_camera_state(event.roadCameraState)
            elif event_type == "cameraOdometry":
                self._update_camera_odometry(event.cameraOdometry, bool(safe_get(event, "valid", True)))
            elif event_type == "livePose":
                self._update_live_pose(event.livePose, event_t)
            elif event_type == "liveCalibration":
                self._update_live_calibration(event.liveCalibration, bool(safe_get(event, "valid", True)))
            elif event_type == "carParams":
                self._update_car_params(event.carParams)
            elif event_type == "radarState":
                self._update_radar_state(event.radarState, event_t)
            elif event_type == "liveTracks":
                self._update_live_tracks(event.liveTracks, event_t)
            elif event_type in ("can", "sendcan"):
                self._update_can_detections(getattr(event, event_type), event_t, event_type)

        return frames

    def _frame_from_car_state(self, car_state: Any, event_t: float) -> RouteReplayFrame:
        speed_mps = max(0.0, safe_float(car_state, "vEgo", 0.0))
        speed_kph = clamp(speed_mps * 3.6, 0.0, MAX_SPEED_KPH)
        self.current_speed_kph = speed_kph
        display_speed_kph = self._display_speed_kph_from_car_state(car_state, speed_mps)
        accel_mps2 = clamp(safe_float(car_state, "aEgo", 0.0), -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2)
        steering_angle_deg = safe_optional_float(car_state, "steeringAngleDeg")
        road_curvature, road_curvature_source = self._current_road_curvature()
        if road_curvature is not None:
            steering = scene_steering_from_curvature(road_curvature)
        else:
            steering = 0.0 if steering_angle_deg is None else clamp(
                steering_angle_deg / MAX_STEERING_ANGLE_DEG,
                -1.0,
                1.0,
            )

        self.cruise_kph = self._cruise_kph_from_car_state(car_state)
        cruise_display_state = self._cruise_display_state_from_car_state(car_state, self.cruise_kph)
        gear_text = self._gear_text_from_car_state(car_state)
        car_cruise_gap = self._cruise_gap_from_car_state(car_state)
        if car_cruise_gap is not None:
            self.cruise_gap = car_cruise_gap
        cruise_gap = car_cruise_gap if car_cruise_gap is not None else self.cruise_gap

        car_speed_limit_kph = self._speed_limit_kph_from_car_state(car_state)
        self._expire_nav_speed_limit(event_t)
        if car_speed_limit_kph is not None:
            self.speed_limit_kph = car_speed_limit_kph
            self.speed_limit_source = "v"
        elif self.nav_speed_limit_kph is not None:
            self.speed_limit_kph = self.nav_speed_limit_kph
            self.speed_limit_source = "n"
        else:
            self.speed_limit_kph = None
            self.speed_limit_source = None

        self._update_lane_styles_from_car_state(car_state)
        lane_values = self._lane_values()
        left_signal = bool(safe_get(car_state, "leftBlinker", False))
        right_signal = bool(safe_get(car_state, "rightBlinker", False))
        left_blindspot = bool(safe_get(car_state, "leftBlindspot", False))
        right_blindspot = bool(safe_get(car_state, "rightBlindspot", False))
        observed_ego_lane_offset = 0.0
        if lane_values["center"] is not None:
            observed_ego_lane_offset = clamp(-lane_values["center"] / lane_values["width"], -1.25, 1.25)
        (
            lane_change,
            lane_change_phase,
            lane_change_progress,
            lane_change_recenter_start_progress,
            lane_change_continuation,
        ) = self._lane_change_values(
            event_t,
            left_signal,
            right_signal,
            observed_ego_lane_offset,
        )
        detected_vehicles = self._detected_vehicles_from_current_state(
            car_state,
            event_t,
            lane_values,
            lane_change,
            lane_change_phase,
        )
        if self.recompute_cutins and event_t - self.cutin_detection_t < 0.15:
            detected_vehicles = tuple((*detected_vehicles, *self.cutin_detections))
        radar_points = self._radar_points_from_current_state(event_t)
        tpms = tpms_info_from_car_state(car_state)
        ev_mode_valid = bool(safe_get(car_state, "evModeValid", False))
        ev_mode_active = ev_mode_valid and bool(safe_get(car_state, "evModeActive", False))
        selfdrive_enabled = (
            self.selfdrive_enabled
            if self.selfdrive_enabled is not None
            else bool(self.controls_enabled)
        )
        trip_report = self.trip_report_tracker.update(
            event_t,
            speed_mps,
            accel_mps2,
            steering_angle_deg,
            selfdrive_enabled,
            self.wheelbase_m,
            self.steer_ratio,
        )

        return RouteReplayFrame(
            t=event_t,
            speed_kph=speed_kph,
            accel_mps2=accel_mps2,
            steering=steering,
            steering_angle_deg=steering_angle_deg,
            speed_limit_kph=self.speed_limit_kph,
            speed_limit_source=self.speed_limit_source,
            cruise_kph=self.cruise_kph,
            cruise_display_state=cruise_display_state,
            gear_text=gear_text,
            cruise_gap=cruise_gap,
            lfa_active=self.lfa_active,
            active_lane_line=self.active_lane_line,
            left_signal=left_signal,
            right_signal=right_signal,
            left_blindspot=left_blindspot,
            right_blindspot=right_blindspot,
            lane_width_m=lane_values["width"],
            lane_center_offset_m=lane_values["center"],
            left_lane_offset=lane_values["left_offset"],
            right_lane_offset=lane_values["right_offset"],
            left_lane_visible=lane_values["left_visible"],
            right_lane_visible=lane_values["right_visible"],
            extra_left_lane_visible=lane_values["extra_left_visible"],
            extra_right_lane_visible=lane_values["extra_right_visible"],
            left_road_edge_offset=lane_values["left_road_edge_offset"],
            right_road_edge_offset=lane_values["right_road_edge_offset"],
            left_lane_style=self.left_lane_style,
            right_lane_style=self.right_lane_style,
            left_lane_color=self.left_lane_color,
            right_lane_color=self.right_lane_color,
            road_curvature=road_curvature,
            road_curvature_source=road_curvature_source,
            lane_position_source=self.lane_position_source,
            model_lane_lines=self.model_lane_lines,
            model_road_edges=self.model_road_edges,
            model_path=self.model_path,
            model_path_source=self.model_path_source,
            lane_change_source=self.lane_change_source,
            lane_change=lane_change,
            lane_change_phase=lane_change_phase,
            lane_change_progress=lane_change_progress,
            lane_change_recenter_start_progress=lane_change_recenter_start_progress,
            lane_change_continuation=lane_change_continuation,
            throttle=clamp(safe_float(car_state, "gas", 0.0), 0.0, 1.0),
            brake=clamp(safe_float(car_state, "brake", 0.0), 0.0, 1.0),
            detected_vehicles=detected_vehicles,
            radar_points=radar_points,
            corner_radar_supported=self.corner_radar_active_for_display(),
            tpms=tpms,
            ev_mode_valid=ev_mode_valid,
            ev_mode_active=ev_mode_active,
            display_speed_kph=display_speed_kph,
            traffic_state=self.traffic_state,
            driving_mode=self.driving_mode,
            planned_speed_kph=self.planned_speed_kph,
            planned_accel_mps2=self.planned_accel_mps2,
            planned_curvature_m_inv=self.model_action_curvature_m_inv,
            should_stop=self.should_stop,
            model_confidence=self.model_confidence,
            model_turn_speed_kph=self.model_turn_speed_kph,
            engaged_prob=self.engaged_prob,
            desire_state=self.desire_state,
            desire_prediction=self.desire_prediction,
            risk_points=self.risk_points,
            brake_disengage_risk=self.brake_disengage_risk,
            gas_disengage_risk=self.gas_disengage_risk,
            steer_override_risk=self.steer_override_risk,
            hard_brake_risk=self.hard_brake_risk,
            gas_press_prob=self.gas_press_prob,
            brake_press_prob=self.brake_press_prob,
            disengage_risk=self.disengage_risk,
            hard_brake_predicted=self.hard_brake_predicted,
            lane_change_available_left=self.lane_change_available_left,
            lane_change_available_right=self.lane_change_available_right,
            lane_change_prob=self.lane_change_ll_prob,
            left_lane_width_m=self.left_lane_width_m,
            right_lane_width_m=self.right_lane_width_m,
            left_road_edge_distance_m=self.left_road_edge_distance_m,
            right_road_edge_distance_m=self.right_road_edge_distance_m,
            left_road_edge_confidence=self.left_road_edge_confidence,
            right_road_edge_confidence=self.right_road_edge_confidence,
            frame_age=self.frame_age,
            frame_drop_perc=self.frame_drop_perc,
            model_execution_time_ms=self.model_execution_time_ms,
            vision_speed_mps=self.vision_speed_mps,
            vision_yaw_rate_rps=self.vision_yaw_rate_rps,
            vision_speed_std_mps=self.vision_speed_std_mps,
            vision_yaw_rate_std_rps=self.vision_yaw_rate_std_rps,
            camera_device_type=self.camera_device_type,
            camera_sensor=self.camera_sensor,
            cpu_usage_percent=self.cpu_usage_percent,
            cpu_temp_c=self.cpu_temp_c,
            memory_used_percent=self.memory_used_percent,
            disk_used_percent=self.disk_used_percent,
            camera_calibration_euler=self.camera_calibration_euler,
            road_transform_trans=self.road_transform_trans,
            road_transform_std=self.road_transform_std,
            camera_odometry_valid=self.camera_odometry_valid,
            longitudinal_plan_source=self.longitudinal_plan_source,
            longitudinal_plan_speeds_kph=self.longitudinal_plan_speeds_kph,
            longitudinal_plan_accels_mps2=self.longitudinal_plan_accels_mps2,
            longitudinal_plan_jerks_mps3=self.longitudinal_plan_jerks_mps3,
            longitudinal_plan_fcw=self.longitudinal_plan_fcw,
            longitudinal_plan_should_stop=self.longitudinal_plan_should_stop,
            longitudinal_plan_allow_throttle=self.longitudinal_plan_allow_throttle,
            longitudinal_plan_allow_brake=self.longitudinal_plan_allow_brake,
            longitudinal_t_follow_s=self.longitudinal_t_follow_s,
            longitudinal_desired_distance_m=self.longitudinal_desired_distance_m,
            longitudinal_v_target_kph=self.longitudinal_v_target_kph,
            longitudinal_jerk_target_mps3=self.longitudinal_jerk_target_mps3,
            lateral_plan_valid=self.lateral_plan_valid,
            lateral_plan_use_lane_lines=self.lateral_plan_use_lane_lines,
            lateral_plan_solver_cost=self.lateral_plan_solver_cost,
            lateral_plan_debug_text=self.lateral_plan_debug_text,
            lateral_plan_curvatures=self.lateral_plan_curvatures,
            lateral_plan_curvature_rates=self.lateral_plan_curvature_rates,
            cutin_debug_text=self.cutin_debug_text,
            recorded_cutin_active=event_t - self.recorded_cutin_t <= RECORDED_CUTIN_REPLAY_HOLD_S,
            recorded_cutin_sound=event_t - self.recorded_cutin_sound_t <= RECORDED_CUTIN_REPLAY_HOLD_S,
            trip_report=trip_report,
            alert=self.alert,
        )

    def _display_speed_kph_from_car_state(self, car_state: Any, fallback_speed_mps: float) -> float:
        v_ego_cluster = safe_float(car_state, "vEgoCluster", 0.0)
        self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
        display_speed_mps = v_ego_cluster if self.v_ego_cluster_seen else fallback_speed_mps
        return clamp(max(0.0, display_speed_mps) * 3.6, 0.0, MAX_SPEED_KPH)

    def _update_driving_model(self, model: Any) -> None:
        lane_meta = safe_get(model, "laneLineMeta")
        if lane_meta is not None:
            left_y = safe_optional_float(lane_meta, "leftY")
            right_y = safe_optional_float(lane_meta, "rightY")
            if left_y is not None and right_y is not None and right_y > left_y:
                self.left_lane_y_m = left_y
                self.right_lane_y_m = right_y
                self.lane_width_m = clamp(right_y - left_y, 2.4, 4.6)
                self.lane_position_source = "drivingModelData"
            self.left_lane_prob = clamp(safe_float(lane_meta, "leftProb", self.left_lane_prob), 0.0, 1.0)
            self.right_lane_prob = clamp(safe_float(lane_meta, "rightProb", self.right_lane_prob), 0.0, 1.0)

        action = safe_get(model, "action")
        if action is not None:
            self._update_model_action(action)

        meta = safe_get(model, "meta")
        if meta is not None and not self.model_lane_change_seen:
            self._update_lane_change_meta(meta, "drivingModelData")

    def _update_model_v2(self, model: Any, event_t: float) -> None:
        frame_age = safe_optional_int(model, "frameAge")
        self.frame_age = frame_age if frame_age is not None else self.frame_age
        frame_drop_perc = safe_optional_float(model, "frameDropPerc")
        self.frame_drop_perc = clamp(frame_drop_perc, 0.0, 100.0) if frame_drop_perc is not None else self.frame_drop_perc
        model_execution_time = safe_optional_float(model, "modelExecutionTime")
        self.model_execution_time_ms = (
            model_execution_time * 1000.0
            if model_execution_time is not None and model_execution_time < 10.0
            else model_execution_time
        )

        lane_lines = safe_get(model, "laneLines")
        lane_probs = safe_get(model, "laneLineProbs")
        if lane_lines is not None:
            self.model_lane_lines = tuple(model_line_points(lane_lines[index]) for index in range(len(lane_lines)))
        if lane_lines is not None and len(lane_lines) >= 3:
            self.cutin_lane_xs = tuple(float(value) for value in safe_get(lane_lines[1], "x", ()))
            self.cutin_left_ys = tuple(float(value) for value in safe_get(lane_lines[1], "y", ()))
            self.cutin_right_ys = tuple(float(value) for value in safe_get(lane_lines[2], "y", ()))
            left_y = first_list_value(safe_get(lane_lines[1], "y"))
            right_y = first_list_value(safe_get(lane_lines[2], "y"))
            if left_y is not None and right_y is not None and right_y > left_y:
                self.left_lane_y_m = left_y
                self.right_lane_y_m = right_y
                self.lane_width_m = clamp(right_y - left_y, 2.4, 4.6)
                self.lane_position_source = "modelV2"
        if lane_lines is not None and len(lane_lines) >= 4:
            self.outer_left_lane_y_m = first_list_value(safe_get(lane_lines[0], "y"))
            self.outer_right_lane_y_m = first_list_value(safe_get(lane_lines[3], "y"))
        if lane_probs is not None and len(lane_probs) >= 3:
            self.left_lane_prob = clamp(finite_float(lane_probs[1]) or 0.0, 0.0, 1.0)
            self.right_lane_prob = clamp(finite_float(lane_probs[2]) or 0.0, 0.0, 1.0)
        if lane_probs is not None and len(lane_probs) >= 4:
            self.outer_left_lane_prob = clamp(finite_float(lane_probs[0]) or 0.0, 0.0, 1.0)
            self.outer_right_lane_prob = clamp(finite_float(lane_probs[3]) or 0.0, 0.0, 1.0)

        road_edges = safe_get(model, "roadEdges")
        road_edge_stds = safe_get(model, "roadEdgeStds")
        if road_edges is not None:
            self.model_road_edges = tuple(model_line_points(road_edges[index]) for index in range(min(len(road_edges), 2)))
        if road_edges is not None and len(road_edges) >= 2:
            self.left_road_edge_y_m = first_list_value(safe_get(road_edges[0], "y"))
            self.right_road_edge_y_m = first_list_value(safe_get(road_edges[1], "y"))
        if road_edge_stds is not None and len(road_edge_stds) >= 2:
            self.left_road_edge_confidence = road_edge_confidence_from_std(finite_float(road_edge_stds[0]))
            self.right_road_edge_confidence = road_edge_confidence_from_std(finite_float(road_edge_stds[1]))

        model_path = model_path_points_from_model_v2(model)
        if model_path:
            self.model_path = model_path
            self.model_path_source = "modelV2.position"
        position = safe_get(model, "position")
        if position is not None:
            path_xs = tuple(float(value) for value in safe_get(position, "x", ()))
            path_ys = tuple(float(value) for value in safe_get(position, "y", ()))
            path_y_stds = tuple(float(value) for value in safe_get(position, "yStd", ()))
            if len(path_xs) >= 2 and len(path_ys) == len(path_xs):
                self.cutin_path_xs = path_xs
                self.cutin_path_ys = path_ys
                self.cutin_path_y_stds = path_y_stds if len(path_y_stds) == len(path_xs) else ()

        action = safe_get(model, "action")
        if action is not None:
            self._update_model_action(action)
        self.model_detections = model_lead_detections_from_model_v2(model)
        self.model_detection_t = event_t

        meta = safe_get(model, "meta")
        if meta is not None:
            self._update_lane_change_meta(meta, "modelV2")
            self._update_model_lane_change_values(meta)
            self._update_model_meta_values(meta, model)

    def _update_lateral_plan(self, lateral_plan: Any) -> None:
        lane_width = safe_optional_float(lateral_plan, "laneWidth")
        if lane_width is not None and lane_width > 0.0:
            self.lane_width_m = clamp(lane_width, 2.4, 4.6)

        if not self.model_lane_change_seen:
            self.lane_change_state = enum_text(safe_get(lateral_plan, "laneChangeState", "off"))
            self.lane_change_direction = enum_text(safe_get(lateral_plan, "laneChangeDirection", "none"))
            self.lane_change_source = "lateralPlan"
        curvature = first_list_value(safe_get(lateral_plan, "curvatures"))
        if curvature is not None and abs(curvature) < 0.05:
            self.model_curvature_m_inv = curvature
            self.model_curvature_source = "lateralPlan"
        self.lateral_plan_valid = bool(safe_get(lateral_plan, "mpcSolutionValid", self.lateral_plan_valid))
        self.lateral_plan_use_lane_lines = bool(safe_get(lateral_plan, "useLaneLines", self.lateral_plan_use_lane_lines))
        solver_cost = safe_optional_float(lateral_plan, "solverCost")
        if solver_cost is not None and solver_cost >= 0.0:
            self.lateral_plan_solver_cost = min(solver_cost, 1_000_000.0)
        debug_text = safe_get(lateral_plan, "latDebugText")
        if debug_text:
            self.lateral_plan_debug_text = str(debug_text)[:64]
        self.lateral_plan_curvatures = numeric_tuple(safe_get(lateral_plan, "curvatures"), minimum=-0.08, maximum=0.08)
        self.lateral_plan_curvature_rates = numeric_tuple(
            safe_get(lateral_plan, "curvatureRates"),
            minimum=-0.08,
            maximum=0.08,
        )

    def _update_nav_instruction(self, nav_instruction: Any, event_t: float) -> None:
        nav_speed_limit_kph = self._speed_limit_kph_from_nav_instruction(nav_instruction)
        if nav_speed_limit_kph is None:
            self._expire_nav_speed_limit(event_t)
            return
        self.nav_speed_limit_kph = nav_speed_limit_kph
        self.nav_speed_limit_t = event_t

    def _expire_nav_speed_limit(self, event_t: float) -> None:
        if (
            self.nav_speed_limit_kph is not None
            and event_t - self.nav_speed_limit_t > NAV_SPEED_LIMIT_HOLD_SECONDS
        ):
            self.nav_speed_limit_kph = None
            self.nav_speed_limit_t = -999.0

    def _update_longitudinal_plan(self, longitudinal_plan: Any, valid: bool = True) -> None:
        self.longitudinal_plan_source = enum_text(
            safe_get(longitudinal_plan, "longitudinalPlanSource", self.longitudinal_plan_source or "")
        ) or self.longitudinal_plan_source
        self.longitudinal_plan_speeds_kph = tuple(
            value * 3.6 for value in numeric_tuple(safe_get(longitudinal_plan, "speeds"), minimum=0.0, maximum=90.0)
        )
        self.longitudinal_plan_accels_mps2 = numeric_tuple(
            safe_get(longitudinal_plan, "accels"),
            minimum=-MAX_ACCEL_MPS2,
            maximum=MAX_ACCEL_MPS2,
        )
        self.longitudinal_plan_jerks_mps3 = numeric_tuple(
            safe_get(longitudinal_plan, "jerks"),
            minimum=-12.0,
            maximum=12.0,
        )
        self.longitudinal_plan_fcw = bool(safe_get(longitudinal_plan, "fcw", self.longitudinal_plan_fcw))
        self.longitudinal_plan_should_stop = bool(safe_get(longitudinal_plan, "shouldStop", self.longitudinal_plan_should_stop))
        self.longitudinal_plan_allow_throttle = bool(
            safe_get(longitudinal_plan, "allowThrottle", self.longitudinal_plan_allow_throttle)
        )
        self.longitudinal_plan_allow_brake = bool(
            safe_get(longitudinal_plan, "allowBrake", self.longitudinal_plan_allow_brake)
        )
        traffic_state = safe_optional_int(longitudinal_plan, "trafficState")
        if traffic_state in (0, 1, 2):
            self.traffic_state = traffic_state
        driving_mode = safe_optional_int(longitudinal_plan, "myDrivingMode")
        self.driving_mode = driving_mode if valid and driving_mode in (1, 2, 3, 4) else None
        t_follow = safe_optional_float(longitudinal_plan, "tFollow")
        if t_follow is not None and 0.0 <= t_follow <= 5.0:
            self.longitudinal_t_follow_s = t_follow
        desired_distance = safe_optional_float(longitudinal_plan, "desiredDistance")
        if desired_distance is not None and 0.0 <= desired_distance <= 250.0:
            self.longitudinal_desired_distance_m = desired_distance
        v_target_now = safe_optional_float(longitudinal_plan, "vTargetNow")
        if v_target_now is not None and 0.0 <= v_target_now <= 90.0:
            self.longitudinal_v_target_kph = v_target_now * 3.6
        jerk_target = safe_optional_float(longitudinal_plan, "jTargetNow")
        if jerk_target is not None and abs(jerk_target) <= 12.0:
            self.longitudinal_jerk_target_mps3 = jerk_target

    def _update_camera_odometry(self, camera_odometry: Any, valid: bool) -> None:
        self.camera_odometry_valid = valid
        trans = safe_get(camera_odometry, "trans")
        rot = safe_get(camera_odometry, "rot")
        trans_std = safe_get(camera_odometry, "transStd")
        rot_std = safe_get(camera_odometry, "rotStd")
        if trans is not None and len(trans) >= 1:
            speed = finite_float(trans[0])
            if speed is not None and abs(speed) < 90.0:
                self.vision_speed_mps = speed
        if rot is not None and len(rot) >= 3:
            yaw_rate = finite_float(rot[2])
            if yaw_rate is not None and abs(yaw_rate) < 2.0:
                self.vision_yaw_rate_rps = yaw_rate
        if trans_std is not None and len(trans_std) >= 1:
            speed_std = finite_float(trans_std[0])
            if speed_std is not None:
                self.vision_speed_std_mps = clamp(speed_std, 0.0, 20.0)
        if rot_std is not None and len(rot_std) >= 3:
            yaw_std = finite_float(rot_std[2])
            if yaw_std is not None:
                self.vision_yaw_rate_std_rps = clamp(yaw_std, 0.0, 2.0)
        if self.camera_calibration_euler is None:
            self.camera_calibration_euler = three_float_tuple(safe_get(camera_odometry, "wideFromDeviceEuler"))
        # cameraOdometry's road height is a noisy per-frame vision estimate. Use
        # it only as an initial fallback; liveCalibration supplies the stable
        # installation height and must not be overwritten every model frame.
        if self.road_transform_trans is None:
            self.road_transform_trans = three_float_tuple(safe_get(camera_odometry, "roadTransformTrans"))
        self.road_transform_std = three_float_tuple(safe_get(camera_odometry, "roadTransformTransStd"))

    def _update_live_calibration(self, live_calibration: Any, valid: bool) -> None:
        if not valid:
            return
        rpy_calib = three_float_tuple(safe_get(live_calibration, "rpyCalib"))
        if rpy_calib is not None:
            self.camera_calibration_euler = rpy_calib
        height_values = safe_get(live_calibration, "height")
        height_m = numeric_tuple(height_values, limit=1, minimum=0.5, maximum=3.0)
        if height_m:
            self.road_transform_trans = (0.0, 0.0, height_m[0])

    def _update_controls_state(self, controls_state: Any) -> None:
        enabled = safe_get(controls_state, "enabled", None)
        if enabled is not None:
            self.controls_enabled = bool(enabled)

        active_lane_line = safe_get(controls_state, "activeLaneLine", None)
        if active_lane_line is not None:
            self.active_lane_line = bool(active_lane_line)

        desired_curvature = safe_optional_float(controls_state, "desiredCurvature")
        if desired_curvature is not None and abs(desired_curvature) < 0.05:
            self.controls_curvature_m_inv = desired_curvature
            self.controls_curvature_source = "controlsState.desired"
            return
        curvature = safe_optional_float(controls_state, "curvature")
        if curvature is not None and abs(curvature) < 0.05:
            self.controls_curvature_m_inv = curvature
            self.controls_curvature_source = "controlsState"

    def _update_selfdrive_state(self, selfdrive_state: Any, event_t: float) -> None:
        enabled = safe_get(selfdrive_state, "enabled", None)
        if enabled is not None:
            self.selfdrive_enabled = bool(enabled)
        alert_size = safe_enum_int(
            safe_get(selfdrive_state, "alertSize", 0),
            ("none", "small", "mid", "full"),
        )
        if alert_size > 0:
            self.alert = ClusterAlert(
                text1=str(safe_get(selfdrive_state, "alertText1", "") or ""),
                text2=str(safe_get(selfdrive_state, "alertText2", "") or ""),
                size=alert_size,
                status=safe_enum_int(
                    safe_get(selfdrive_state, "alertStatus", 0),
                    ("normal", "userprompt", "critical"),
                ),
                alert_type=str(safe_get(selfdrive_state, "alertType", "") or ""),
            )
        else:
            self.alert = None
        alert_sound = str(safe_get(selfdrive_state, "alertSound", "none") or "none").lower()
        alert_type = str(safe_get(selfdrive_state, "alertType", "") or "").lower()
        self.recorded_cutin_sound = bool(
            self.recorded_cutin_ids
            and (
                (alert_sound == "prompt" and alert_type.startswith("audioprompt/"))
                or (alert_sound == "radarcutin" and alert_type.startswith("radarcutin/"))
            )
        )
        if self.recorded_cutin_sound:
            self.recorded_cutin_sound_t = event_t
        cruise_gap = self._cruise_gap_from_personality(safe_get(selfdrive_state, "personality"))
        if cruise_gap is not None:
            self.cruise_gap = cruise_gap

    def _update_car_control(self, car_control: Any) -> None:
        lat_active = safe_get(car_control, "latActive", None)
        if lat_active is not None:
            self.lfa_active = bool(lat_active)

    def _update_device_state(self, device_state: Any) -> None:
        device_type = safe_get(device_state, "deviceType", None)
        if device_type is not None:
            self.camera_device_type = str(device_type).strip().lower()
        cpu_usage = numeric_tuple(
            safe_get(device_state, "cpuUsagePercent"),
            limit=64,
            minimum=0.0,
            maximum=100.0,
        )
        if cpu_usage:
            self.cpu_usage_percent = sum(cpu_usage) / len(cpu_usage)
        cpu_temps = numeric_tuple(
            safe_get(device_state, "cpuTempC"),
            limit=64,
            minimum=-50.0,
            maximum=200.0,
        )
        if cpu_temps:
            self.cpu_temp_c = max(cpu_temps)
        memory_used_percent = safe_optional_float(device_state, "memoryUsagePercent")
        if memory_used_percent is not None and 0.0 <= memory_used_percent <= 100.0:
            self.memory_used_percent = memory_used_percent
        free_space_percent = safe_optional_float(device_state, "freeSpacePercent")
        if free_space_percent is not None and 0.0 <= free_space_percent <= 100.0:
            self.disk_used_percent = 100.0 - free_space_percent

    def _update_road_camera_state(self, camera_state: Any) -> None:
        sensor = safe_get(camera_state, "sensor", None)
        if sensor is not None:
            self.camera_sensor = str(sensor).strip().lower()

    def _update_car_params(self, car_params: Any) -> None:
        self.car_brand = str(safe_get(car_params, "brand", "") or "").lower()
        wheelbase_m = safe_optional_float(car_params, "wheelbase")
        if wheelbase_m is not None and 1.5 <= wheelbase_m <= 5.0:
            self.wheelbase_m = wheelbase_m
        steer_ratio = safe_optional_float(car_params, "steerRatio")
        if steer_ratio is not None and 1.0 <= steer_ratio <= 40.0:
            self.steer_ratio = steer_ratio
        ext_flags = safe_optional_int(car_params, "extFlags")
        if not self.front_radar_only and ext_flags is not None and (ext_flags & CORNER_RADAR_OBJECTS_EXT_FLAGS):
            self.corner_radar_supported = True

    def _update_live_pose(self, live_pose: Any, _event_t: float) -> None:
        self._update_cutin_live_pose(live_pose)

    def corner_radar_active_for_display(self) -> bool:
        return not self.front_radar_only and (self.corner_radar_supported or self.corner_radar_tracks_seen)

    def _update_radar_state(self, radar_state: Any, event_t: float) -> None:
        detections: list[DetectedVehicle] = []
        primary_track_ids: set[int] = set()
        lead_one = safe_get(radar_state, "leadOne")
        self.lead_one_status = bool(lead_one is not None and safe_get(lead_one, "status", False))
        self.lead_one_d_rel = safe_float(lead_one, "dRel", 0.0) if lead_one is not None else 0.0
        self.lead_one_v_rel = safe_float(lead_one, "vRel", 0.0) if lead_one is not None else 0.0
        self.lead_one_radar = bool(lead_one is not None and safe_get(lead_one, "radar", False))
        lead_one_track_id = safe_optional_int(lead_one, "radarTrackId") if lead_one is not None else None
        self.lead_one_track_id = lead_one_track_id if lead_one_track_id is not None else -1
        recorded_cutin_leads = tuple(safe_get(radar_state, "leadsCutIn", ()) or ())
        self.recorded_cutin_ids = {
            track_id
            for lead in recorded_cutin_leads
            if bool(safe_get(lead, "status", False))
            for track_id in [safe_optional_int(lead, "radarTrackId")]
            if track_id is not None
        }
        for label, lead_name in (("L1", "leadOne"), ("L2", "leadTwo")):
            lead = safe_get(radar_state, lead_name)
            if lead is None or not bool(safe_get(lead, "status", False)):
                continue
            d_rel = safe_float(lead, "dRel", 0.0)
            if not RADAR_MIN_LONGITUDINAL_M <= d_rel <= RADAR_FRONT_MAX_LONGITUDINAL_M:
                continue
            # openpilot yRel is left-positive; this renderer uses right-positive x.
            lateral_m = -safe_float(lead, "yRel", 0.0)
            if radar_position_is_zero(d_rel, lateral_m):
                continue
            relative_speed_mps = safe_optional_float(lead, "vRel")
            lead_speed_mps = safe_optional_float(lead, "vLead")
            lateral_speed_mps = safe_optional_float(lead, "vLat")
            if lateral_speed_mps is not None:
                lateral_speed_mps = -lateral_speed_mps
            track_id = safe_optional_int(lead, "radarTrackId")
            if self.front_radar_only and self.car_brand == "hyundai" and radar_track_id_is_corner_object(track_id):
                continue
            if track_id is not None:
                primary_track_ids.add(track_id)
            cut_in = (
                self.show_recorded_cutins
                and lead_name == "leadTwo"
                and track_id is not None
                and track_id in self.recorded_cutin_ids
            )
            absolute_speed_kph = (
                lead_speed_mps * 3.6
                if lead_speed_mps is not None
                else (
                    self.current_speed_kph + relative_speed_mps * 3.6
                    if relative_speed_mps is not None
                    else None
                )
            )
            detections.append(
                DetectedVehicle(
                    label="L2 CUT-IN" if cut_in else label,
                    longitudinal_m=d_rel,
                    lateral_m=clamp(lateral_m, -8.0, 8.0),
                    source="radarState",
                    relative_speed_mps=relative_speed_mps,
                    absolute_speed_kph=absolute_speed_kph,
                    lateral_speed_mps=lateral_speed_mps,
                    acceleration_mps2=safe_optional_float(lead, "aLeadK"),
                    cut_in=cut_in,
                    primary=True,
                    ttc_s=ttc_from_relative_speed(d_rel, relative_speed_mps),
                    radar_track_id=track_id,
                )
            )
        for lead in recorded_cutin_leads:
            if not bool(safe_get(lead, "status", False)):
                continue
            d_rel = safe_float(lead, "dRel", 0.0)
            if not RADAR_MIN_LONGITUDINAL_M <= d_rel <= RADAR_FRONT_MAX_LONGITUDINAL_M:
                continue
            track_id = safe_optional_int(lead, "radarTrackId")
            if track_id is not None and track_id in primary_track_ids:
                continue
            lateral_m = -safe_float(lead, "yRel", 0.0)
            if radar_position_is_zero(d_rel, lateral_m):
                continue
            relative_speed_mps = safe_optional_float(lead, "vRel")
            lead_speed_mps = safe_optional_float(lead, "vLead")
            lateral_speed_mps = safe_optional_float(lead, "vLat")
            if lateral_speed_mps is not None:
                lateral_speed_mps = -lateral_speed_mps
            absolute_speed_kph = (
                lead_speed_mps * 3.6
                if lead_speed_mps is not None
                else (
                    self.current_speed_kph + relative_speed_mps * 3.6
                    if relative_speed_mps is not None
                    else None
                )
            )
            detections.append(
                DetectedVehicle(
                    label="CUT-IN",
                    longitudinal_m=d_rel,
                    lateral_m=clamp(lateral_m, -8.0, 8.0),
                    source="radarState",
                    relative_speed_mps=relative_speed_mps,
                    absolute_speed_kph=absolute_speed_kph,
                    lateral_speed_mps=lateral_speed_mps,
                    acceleration_mps2=safe_optional_float(lead, "aLeadK"),
                    cut_in=True,
                    primary=True,
                    ttc_s=ttc_from_relative_speed(d_rel, relative_speed_mps),
                    radar_track_id=track_id,
                )
            )
        self.radar_detections = tuple(detections)
        self.radar_detection_t = event_t
        if not self.recompute_cutins:
            active_leads = tuple(
                lead for lead in recorded_cutin_leads
                if bool(safe_get(lead, "status", False))
            )
            if active_leads:
                current_cutins = tuple(detection for detection in detections if detection.cut_in)
                if current_cutins:
                    self.recorded_cutin_detections = current_cutins
                self.recorded_cutin_t = event_t
                lead = active_leads[0]
                track_id = safe_optional_int(lead, "radarTrackId")
                self.cutin_debug_text = (
                    f"LOG CUT-IN: YES | id{track_id if track_id is not None else -1} "
                    f"x {safe_float(lead, 'dRel', 0.0):.1f}m "
                    f"y {safe_float(lead, 'yRel', 0.0):+.1f}m"
                )
            else:
                held = event_t - self.recorded_cutin_t <= RECORDED_CUTIN_REPLAY_HOLD_S
                if held and self.recorded_cutin_detections:
                    held_ids = {detection.radar_track_id for detection in self.recorded_cutin_detections}
                    self.radar_detections = tuple(
                        detection for detection in self.radar_detections
                        if detection.radar_track_id not in held_ids
                    ) + self.recorded_cutin_detections
                    self.cutin_debug_text = "LOG CUT-IN: YES | recorded event hold"
                else:
                    self.recorded_cutin_detections = ()
                    self.cutin_debug_text = "LOG CUT-IN: NO"

    def _update_can_detections(self, can_messages: Any, event_t: float, source_service: str = "can") -> None:
        if self.front_radar_only:
            return
        for can_message in can_messages:
            address = int(safe_get(can_message, "address", -1))
            bus = int(safe_get(can_message, "src", -1))
            if source_service == "can" and bus >= 0x80:
                continue
            data = bytes(safe_get(can_message, "dat", b""))
            if source_service == "can" and bus == RAW_CORNER_RADAR_BUS:
                for obj in decode_raw_corner_objects(event_t, address, data):
                    raw_key = (obj.group, obj.slot)
                    if raw_corner_object_is_valid(obj):
                        self.raw_corner_objects[raw_key] = obj
                        self.raw_corner_tracker.update(obj)
                        self.raw_corner_track_t = event_t
                        self.corner_radar_tracks_seen = True
                    else:
                        self.raw_corner_objects.pop(raw_key, None)
            if address not in (CCNC_CORNER_RADAR_ADDRESS, ADRV_CORNER_RADAR_ADDRESS):
                continue
            if source_service == "sendcan" or not is_hyundai_camera_can_bus(bus):
                continue
            if len(data) < 24:
                continue
            corner_values = decode_hyundai_canfd_dbc_message(address, data)
            parsed = parse_corner_radar_message(address, data, corner_values)
            if address == ADRV_CORNER_RADAR_ADDRESS:
                lane_changing = int(corner_values.get("LANE_CHANGING", 0.0))
                self.adrv_corner_detections = parsed
                self.adrv_corner_message_t = event_t
                self.adrv_lane_changing = lane_changing
                self.adrv_lane_changing_t = event_t
            else:
                self.ccnc_corner_detections = parsed
                self.ccnc_corner_message_t = event_t

    def _update_live_tracks(self, live_tracks: Any, event_t: float) -> None:
        tracks = tuple(safe_get(live_tracks, "points", ()) or ())
        if self.reconstruct_corner_live_tracks:
            tracks = merge_recorded_and_reconstructed_tracks(
                tracks,
                self.raw_corner_tracker.live_tracks_at(event_t, self.current_speed_kph / 3.6),
                raw_corner_only=True,
            )
        if self.recompute_cutins:
            cutin_input = ReconstructedLiveTracks(tracks) if self.reconstruct_corner_live_tracks else live_tracks
            self._update_offline_cutin(cutin_input, event_t)
        points: dict[str, RadarPoint] = {}
        for index, track in enumerate(tracks):
            point = live_track_to_radar_point(
                track,
                index,
                self.current_speed_kph,
                allow_legacy_corner_ids=self.car_brand == "hyundai",
            )
            if point is not None and not (self.front_radar_only and point.source == CORNER_OBJECT_SOURCE):
                points[point.label] = point
                if point.source == CORNER_OBJECT_SOURCE:
                    self.corner_radar_tracks_seen = True
        self.live_track_radar_points = points
        self.live_track_radar_t = event_t

    def _update_cutin_live_pose(self, live_pose: Any) -> None:
        angular_velocity = safe_get(live_pose, "angularVelocityDevice")
        valid = bool(angular_velocity is not None and safe_get(angular_velocity, "valid", False))
        inputs_ok = bool(safe_get(live_pose, "inputsOK", False))
        sensors_ok = bool(safe_get(live_pose, "sensorsOK", False))
        if not (valid and inputs_ok and sensors_ok):
            return
        raw_yaw_rate = clamp(safe_float(angular_velocity, "z", 0.0), -0.35, 0.35)
        alpha = REPLAY_CUTIN_DT / (0.20 + REPLAY_CUTIN_DT)
        self.cutin_yaw_rate = (1.0 - alpha) * self.cutin_yaw_rate + alpha * raw_yaw_rate

    def _update_offline_cutin(self, live_tracks: Any, event_t: float) -> None:
        if self.front_radar_only and self.cutin_radar_source != ROUTE_CUTIN_RADAR_SOURCE_FRONT:
            self.cutin_tracks.clear()
            self.cutin_detections = ()
            self.cutin_output_hold_count = 0
            self.cutin_output_hold_reference = None
            self.cutin_debug_text = "NEW CUTIN: disabled | front radar only"
            return
        if not self._cutin_lane_geometry_available():
            self.cutin_detections = ()
            self.cutin_output_hold_count = 0
            self.cutin_output_hold_reference = None
            self.cutin_debug_text = (
                f"NEW CUTIN {self.cutin_radar_source.upper()} S{self.cutin_sensitivity:.0f}: waiting for laneLines"
            )
            return

        points = tuple(safe_get(live_tracks, "points", ()) or ())
        if self.cutin_radar_source == ROUTE_CUTIN_RADAR_SOURCE_CORNER:
            current_side_matches = self._side_corner_front_matches(points, event_t)
            available_front_ids = {
                int(safe_get(point, "trackId", -1))
                for point in points
                if bool(safe_get(point, "measured", False)) and not self._is_corner_live_track(point)
            }
            (
                self.cutin_side_corner_front_matches,
                self.cutin_side_corner_front_match_misses,
            ) = hold_side_corner_front_matches(
                current_side_matches,
                self.cutin_side_corner_front_matches,
                self.cutin_side_corner_front_match_misses,
                available_front_ids,
            )
        else:
            self.cutin_side_corner_front_matches = {}
            self.cutin_side_corner_front_match_misses = {}
        point_by_id = self._cutin_points_by_stable_id(points, event_t)
        previous_positions = {
            track_id: (track.d_rel, track.y_rel, track.v_rel)
            for track_id, track in self.cutin_tracks.items()
            if track.measured
        }
        current_positions = {
            track_id: (
                safe_float(point, "dRel", 0.0),
                safe_float(point, "yRel", 0.0),
                safe_float(point, "vRel", 0.0),
            )
            for track_id, point in point_by_id.items()
            if self._is_cutin_live_track(point) and bool(safe_get(point, "measured", False))
        }
        associations = associate_cutin_tracks(previous_positions, current_positions)
        previous_tracks = {
            track_id: ReplayCutinTrack(
                track_id=track.track_id,
                cnt=track.cnt,
                cut_in_count=track.cut_in_count,
                cut_in_start_abs_dpath=track.cut_in_start_abs_dpath,
                measured=track.measured,
                d_rel=track.d_rel,
                y_rel=track.y_rel,
                v_rel=track.v_rel,
                v_lead=track.v_lead,
                yv_rel=track.yv_rel,
                d_path=track.d_path,
                d_path_future=track.d_path_future,
                in_lane_prob=track.in_lane_prob,
                in_lane_prob_future=track.in_lane_prob_future,
                d_path_rate=track.d_path_rate,
                inward_speed=track.inward_speed,
                path_d_path=track.path_d_path,
                path_d_path_future=track.path_d_path_future,
                path_in_lane_prob=track.path_in_lane_prob,
                path_in_lane_prob_future=track.path_in_lane_prob_future,
                path_d_path_rate=track.path_d_path_rate,
                path_inward_speed=track.path_inward_speed,
                path_y_std=track.path_y_std,
                radar_inward_speed=track.radar_inward_speed,
                side_corner_confirmed_count=track.side_corner_confirmed_count,
                position_history=track.position_history.copy(),
                path_position_history=track.path_position_history.copy(),
            )
            for track_id, track in self.cutin_tracks.items()
        }
        valid_ids = {
            track_id for track_id, point in point_by_id.items()
            if self._is_cutin_live_track(point)
        }
        for track_id in tuple(self.cutin_tracks):
            if track_id not in valid_ids:
                self.cutin_tracks.pop(track_id, None)

        front_points = [
            point for track_id, point in point_by_id.items()
            if not self._is_corner_live_track(point) and bool(safe_get(point, "measured", False))
        ]
        diagnostics: list[ReplayCutinTrack] = []
        detections: list[DetectedVehicle] = []
        lane_line_available = self.left_lane_prob > 0.5 and self.right_lane_prob > 0.5

        for track_id, point in point_by_id.items():
            if not self._is_cutin_live_track(point):
                continue
            track = self.cutin_tracks.setdefault(track_id, ReplayCutinTrack(track_id))
            source_id = associations.get(track_id)
            if source_id is not None and source_id != track_id:
                source = previous_tracks[source_id]
                track.measured = source.measured
                track.d_rel = source.d_rel
                track.y_rel = source.y_rel
                track.v_rel = source.v_rel
                track.v_lead = source.v_lead
                track.cnt = source.cnt
                track.cut_in_count = source.cut_in_count
                track.cut_in_start_abs_dpath = source.cut_in_start_abs_dpath
                track.position_history.clear()
                track.position_history.extend(source.position_history)
                track.path_d_path = source.path_d_path
                track.path_d_path_future = source.path_d_path_future
                track.path_in_lane_prob = source.path_in_lane_prob
                track.path_in_lane_prob_future = source.path_in_lane_prob_future
                track.path_d_path_rate = source.path_d_path_rate
                track.path_inward_speed = source.path_inward_speed
                track.path_y_std = source.path_y_std
                track.radar_inward_speed = source.radar_inward_speed
                track.side_corner_confirmed_count = source.side_corner_confirmed_count
                track.path_position_history.clear()
                track.path_position_history.extend(source.path_position_history)
            prev_measured = track.measured
            prev_d_rel = track.d_rel
            prev_y_rel = track.y_rel
            prev_v_lead = track.v_lead
            track.measured = bool(safe_get(point, "measured", False))
            track.d_rel = safe_float(point, "dRel", 0.0)
            track.y_rel = safe_float(point, "yRel", 0.0)
            track.v_rel = safe_float(point, "vRel", 0.0)
            track.v_lead = safe_float(point, "vLead", 0.0)
            track.yv_rel = safe_float(point, "yvRel", 0.0)
            discontinuous = is_cutin_track_discontinuous(
                prev_measured,
                prev_d_rel,
                prev_y_rel,
                prev_v_lead,
                track.d_rel,
                track.y_rel,
                track.v_lead,
            )
            if not track.measured:
                track.cnt = 0
                track.cut_in_count = 0
                track.cut_in_start_abs_dpath = 0.0
                track.side_corner_confirmed_count = 0
                track.path_position_history.clear()
            elif discontinuous:
                track.cut_in_count = 0
                track.cut_in_start_abs_dpath = 0.0
                track.side_corner_confirmed_count = 0
                track.path_position_history.clear()

            side_corner_confirmed = track_id in self.cutin_side_corner_front_matches
            if track.measured and side_corner_confirmed:
                track.side_corner_confirmed_count += 1
            elif not side_corner_confirmed:
                track.side_corner_confirmed_count = 0

            v_corr = clamp(self.cutin_yaw_rate * track.y_rel * REPLAY_CUTIN_YAW_GAIN, -0.6, 0.6)
            yv_corr = clamp(
                -self.cutin_yaw_rate * clamp(track.d_rel, 0.0, 50.0) * REPLAY_CUTIN_YAW_GAIN,
                -1.5,
                1.5,
            )
            future_d_rel = track.d_rel + (track.v_rel + v_corr) * self.cutin_tuning["horizon_s"]
            future_y_rel = track.y_rel + (track.yv_rel + yv_corr) * self.cutin_tuning["horizon_s"]
            track.d_path, track.in_lane_prob = self._cutin_dpath(track.d_rel, track.y_rel)
            track.d_path_future, track.in_lane_prob_future = self._cutin_dpath(future_d_rel, future_y_rel)
            track.radar_inward_speed = max(
                0.0, -math.copysign(1.0, track.d_path) * (track.yv_rel + yv_corr)
            )
            track.d_path_rate, track.inward_speed = update_lane_relative_motion(
                track.position_history,
                track.d_rel,
                track.y_rel,
                self.cutin_lane_xs,
                self.cutin_left_ys,
                self.cutin_right_ys,
                track.measured,
                discontinuous,
                REPLAY_CUTIN_DT,
            )
            lane_half_width = self._cutin_lane_half_width(track.d_rel)
            track.d_path_future, track.in_lane_prob_future = combine_cutin_future_projection(
                track.d_path,
                track.d_path_rate,
                self.cutin_tuning["horizon_s"],
                lane_half_width,
                track.d_path_future,
                track.in_lane_prob_future,
                track.radar_inward_speed,
            )
            track.inward_speed = effective_cutin_inward_speed(
                track.d_rel,
                self.current_speed_kph / 3.6,
                track.inward_speed,
                track.d_path,
                track.d_path_future,
                self.cutin_tuning["horizon_s"],
            )
            (
                track.path_d_path,
                track.path_in_lane_prob,
                track.path_y_std,
            ) = self._cutin_path_dpath(track.d_rel, track.y_rel)
            (
                track.path_d_path_future,
                track.path_in_lane_prob_future,
                _,
            ) = self._cutin_path_dpath(future_d_rel, future_y_rel)
            if self._cutin_path_geometry_available():
                track.path_d_path_rate, track.path_inward_speed = update_lane_relative_motion(
                    track.path_position_history,
                    track.d_rel,
                    track.y_rel,
                    self.cutin_path_xs,
                    self.cutin_path_ys,
                    self.cutin_path_ys,
                    track.measured,
                    discontinuous,
                    REPLAY_CUTIN_DT,
                )
                track.path_d_path_future, track.path_in_lane_prob_future = combine_cutin_future_projection(
                    track.path_d_path,
                    track.path_d_path_rate,
                    self.cutin_tuning["horizon_s"],
                    lane_half_width,
                    track.path_d_path_future,
                    track.path_in_lane_prob_future,
                    max(0.0, -math.copysign(1.0, track.path_d_path) * (track.yv_rel + yv_corr)),
                )
                track.path_inward_speed = effective_cutin_inward_speed(
                    track.d_rel,
                    self.current_speed_kph / 3.6,
                    track.path_inward_speed,
                    track.path_d_path,
                    track.path_d_path_future,
                    self.cutin_tuning["horizon_s"],
                )
            else:
                track.path_position_history.clear()
                track.path_d_path_rate = 0.0
                track.path_inward_speed = 0.0
            if side_corner_confirmed:
                track.radar_inward_speed = max(
                    0.0, -math.copysign(1.0, track.path_d_path) * (track.yv_rel + yv_corr)
                )
            track.cnt += 1

            matching_front = any(
                abs(track.d_rel - safe_float(front, "dRel", 999.0)) < 3.0
                and abs(track.v_rel - safe_float(front, "vRel", 999.0)) < 2.0
                for front in front_points
            )
            closer = not self.lead_one_status or track.d_rel + 1.0 < self.lead_one_d_rel
            matches_lead_one = (
                matching_front
                and self.lead_one_status
                and self.lead_one_radar
                and not self._track_id_is_corner_live(point_by_id, self.lead_one_track_id)
                and abs(track.d_rel - self.lead_one_d_rel) < 3.0
                and abs(track.v_rel - self.lead_one_v_rel) < 2.0
            )
            closer_or_matching = closer or matches_lead_one
            special_near_cutin = (
                side_corner_confirmed
                and is_corner_confirmed_near_cutin(
                    confirmed_frames=track.side_corner_confirmed_count,
                    d_rel=track.d_rel,
                    v_lead=track.v_lead,
                    d_path=track.path_d_path,
                    d_path_future=track.path_d_path_future,
                    inward_speed=track.path_inward_speed,
                    radar_inward_speed=track.radar_inward_speed,
                    path_y_std=track.path_y_std,
                )
            )
            if side_corner_confirmed:
                entry_rejection_reason = None if special_near_cutin else "side-corner"
            else:
                entry_rejection_reason = cutin_entry_rejection_reason(
                    enabled=self.cutin_sensitivity > 0.0,
                    lane_line_available=lane_line_available,
                    corner_track=True,
                    closer_or_matching=closer_or_matching,
                    track_count=track.cnt,
                    min_track_age=cutin_min_track_age_frames(
                        self.cutin_min_track_age,
                        track.d_rel,
                        track.inward_speed,
                        self.current_speed_kph / 3.6,
                    ),
                    d_rel=track.d_rel,
                    v_lead=track.v_lead,
                    d_path=track.d_path,
                    d_path_future=track.d_path_future,
                    in_lane_prob=track.in_lane_prob,
                    in_lane_prob_future=track.in_lane_prob_future,
                    inward_speed=track.inward_speed,
                    tuning=self.cutin_tuning,
                    fast_lane_entry=is_fast_cutin_entry(
                        track.d_rel,
                        self.current_speed_kph / 3.6,
                        track.d_path,
                        lane_half_width,
                        track.inward_speed,
                        track.radar_inward_speed,
                        v_rel=track.v_rel,
                    ),
                    radar_inward_speed=track.radar_inward_speed,
                    max_d_rel=(
                        CORNER_CUTIN_MAX_DREL_M if self._is_corner_live_track(point) else None
                    ),
                )
            track.rejection_reason = entry_rejection_reason or "enter"
            entering = track.rejection_reason == "enter"
            moving_away = abs(track.d_path_future) - abs(track.d_path)
            keep = (
                track.cut_in_count > 0
                and closer_or_matching
                and 0.8 < track.d_rel < 55.0
                and track.v_lead > 2.0
                and moving_away <= REPLAY_CUTIN_KEEP_MAX_MOVING_AWAY
                and (
                    track.in_lane_prob_future > REPLAY_CUTIN_KEEP_FUTURE_IN_LANE_PROB
                    or abs(track.d_path_future) < REPLAY_CUTIN_KEEP_MAX_DPATH_FUTURE
                )
            )
            if side_corner_confirmed and track.cut_in_count > 0:
                moving_away_path = abs(track.path_d_path_future) - abs(track.path_d_path)
                keep = (
                    track.side_corner_confirmed_count > 0
                    and 0.8 < track.d_rel < 8.0
                    and track.v_lead > 0.0
                    and track.path_y_std <= 0.8
                    and moving_away_path <= REPLAY_CUTIN_KEEP_MAX_MOVING_AWAY
                    and (
                        track.path_in_lane_prob_future > REPLAY_CUTIN_KEEP_FUTURE_IN_LANE_PROB
                        or abs(track.path_d_path_future) < REPLAY_CUTIN_KEEP_MAX_DPATH_FUTURE
                    )
                )
            if (self.cutin_radar_source == ROUTE_CUTIN_RADAR_SOURCE_FRONT or side_corner_confirmed) and keep:
                entering = True
                if track.rejection_reason != "enter":
                    track.rejection_reason = "continue"
            confirmation_d_path = track.path_d_path if side_corner_confirmed else track.d_path
            confirmation_inward_speed = track.path_inward_speed if side_corner_confirmed else track.inward_speed
            confirm_frames = cutin_confirmation_frames(
                self.cutin_confirm_frames,
                track.d_rel,
                confirmation_inward_speed,
                self.current_speed_kph / 3.6,
            )
            track.cut_in_count, track.cut_in_start_abs_dpath = update_cutin_confirmation(
                track.cut_in_count,
                track.cut_in_start_abs_dpath,
                confirmation_d_path,
                track.d_rel,
                entering,
                keep,
                confirm_frames,
                REPLAY_CUTIN_STICKY_FRAMES,
                self.cutin_tuning["enter_min_progress"],
                self.current_speed_kph / 3.6,
            )

            if track.measured:
                diagnostics.append(track)
            if track.cut_in_count >= confirm_frames:
                detections.append(
                    DetectedVehicle(
                        label="NEW CUT-IN",
                        longitudinal_m=track.d_rel,
                        lateral_m=clamp(-track.y_rel, -8.0, 8.0),
                        source="cutinReplay",
                        relative_speed_mps=track.v_rel,
                        absolute_speed_kph=track.v_lead * 3.6,
                        lateral_speed_mps=-track.yv_rel,
                        cut_in=True,
                        primary=True,
                        radar_track_id=track.track_id,
                    )
                )

        if detections:
            nearest = min(detections, key=lambda detection: detection.longitudinal_m)
            self.cutin_output_hold_reference = (
                nearest.longitudinal_m,
                -nearest.lateral_m,
                nearest.relative_speed_mps or 0.0,
            )
            self.cutin_output_hold_count = REPLAY_CUTIN_OUTPUT_HOLD_FRAMES
        elif self.cutin_output_hold_count > 0 and self.cutin_output_hold_reference is not None:
            d_rel, y_rel, v_rel = self.cutin_output_hold_reference
            matches = [
                track for track in diagnostics
                if abs(track.d_rel - d_rel) <= REPLAY_CUTIN_OUTPUT_HOLD_DREL_M
                and abs(track.y_rel - y_rel) <= REPLAY_CUTIN_OUTPUT_HOLD_YREL_M
                and abs(track.v_rel - v_rel) <= REPLAY_CUTIN_OUTPUT_HOLD_VREL_MPS
            ]
            if matches:
                track = min(
                    matches,
                    key=lambda candidate: (
                        abs(candidate.d_rel - d_rel)
                        + abs(candidate.y_rel - y_rel)
                        + 0.5 * abs(candidate.v_rel - v_rel)
                    ),
                )
                detections.append(
                    DetectedVehicle(
                        label="NEW CUT-IN",
                        longitudinal_m=track.d_rel,
                        lateral_m=clamp(-track.y_rel, -8.0, 8.0),
                        source="cutinReplayHold",
                        relative_speed_mps=track.v_rel,
                        absolute_speed_kph=track.v_lead * 3.6,
                        lateral_speed_mps=-track.yv_rel,
                        cut_in=True,
                        primary=True,
                        radar_track_id=track.track_id,
                    )
                )
                self.cutin_output_hold_reference = (track.d_rel, track.y_rel, track.v_rel)
                self.cutin_output_hold_count -= 1
                if self.cutin_output_hold_count == 0:
                    self.cutin_output_hold_reference = None
            else:
                self.cutin_output_hold_count = 0
                self.cutin_output_hold_reference = None

        self.cutin_detections = tuple(detections)
        self.cutin_detection_t = event_t
        self.cutin_debug_text = self._cutin_debug_summary(diagnostics, detections)

    def _is_corner_live_track(self, point: Any) -> bool:
        source = safe_get(point, "radarSource", "frontRadar")
        if is_corner_radar_source(source):
            return True
        track_id = safe_optional_int(point, "trackId")
        return str(source) == "frontRadar" and self.car_brand == "hyundai" and radar_track_id_is_corner_object(track_id)

    def _side_corner_front_matches(self, points: tuple[Any, ...], event_t: float) -> dict[int, int]:
        corner_tracks: dict[int, tuple[float, float, float]] = {}
        for obj in self.raw_corner_objects.values():
            if not 0.0 <= event_t - obj.t <= REPLAY_CUTIN_RAW_OBJECT_MAX_AGE_S:
                continue
            if not raw_corner_object_is_valid(obj):
                continue
            corner_tracks[self._stable_cutin_corner_track_id(obj)] = (obj.x, obj.y, obj.vx)

        front_tracks = {
            int(safe_get(point, "trackId", -1)): (
                safe_float(point, "dRel", 0.0),
                safe_float(point, "yRel", 0.0),
                safe_float(point, "vRel", 0.0),
            )
            for point in points
            if bool(safe_get(point, "measured", False))
            and not self._is_corner_live_track(point)
            and str(safe_get(point, "radarSource", "frontRadar")) != "scc"
            and not (self.car_brand == "hyundai" and safe_optional_int(point, "trackId") == 0)
        }
        return match_side_corner_to_front_tracks(corner_tracks, front_tracks)

    def _cutin_points_by_stable_id(self, points: tuple[Any, ...], event_t: float) -> dict[int, Any]:
        if self.cutin_radar_source != ROUTE_CUTIN_RADAR_SOURCE_CORNER:
            return {int(safe_get(point, "trackId", -1)): point for point in points}

        point_by_id: dict[int, Any] = {}
        for point in points:
            track_id = int(safe_get(point, "trackId", -1))
            obj = self._raw_corner_object_for_live_track(point, event_t)
            if obj is not None:
                track_id = self._stable_cutin_corner_track_id(obj)
            point_by_id[track_id] = point
        return point_by_id

    def _raw_corner_object_for_live_track(self, point: Any, event_t: float) -> RawCornerObject | None:
        track_id = safe_optional_int(point, "trackId")
        if track_id is None:
            return None
        if CORNER_OBJECT_180_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_TRACK_COUNT:
            raw_key = ("180", track_id - CORNER_OBJECT_180_TRACK_ID_OFFSET)
        elif CORNER_OBJECT_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_TRACK_ID_OFFSET + CORNER_OBJECT_TRACK_COUNT:
            raw_key = ("235", track_id - CORNER_OBJECT_TRACK_ID_OFFSET)
        else:
            return None

        obj = self.raw_corner_objects.get(raw_key)
        if obj is None or not 0.0 <= event_t - obj.t <= REPLAY_CUTIN_RAW_OBJECT_MAX_AGE_S:
            return None
        if (abs(safe_float(point, "dRel", 0.0) - obj.x) > REPLAY_CUTIN_RAW_OBJECT_MAX_DREL_M
                or abs(safe_float(point, "yRel", 0.0) - obj.y) > REPLAY_CUTIN_RAW_OBJECT_MAX_YREL_M
                or abs(safe_float(point, "vRel", 0.0) - obj.vx) > REPLAY_CUTIN_RAW_OBJECT_MAX_VREL_MPS):
            return None
        return obj

    def _stable_cutin_corner_track_id(self, obj: RawCornerObject) -> int:
        key = (obj.group, obj.object_id)
        previous = self.cutin_corner_object_ids.get(key)
        if previous is None or obj.age < previous[1]:
            track_id = self.next_cutin_corner_track_id
            self.next_cutin_corner_track_id += 1
        else:
            track_id = previous[0]
        self.cutin_corner_object_ids[key] = (track_id, obj.age)
        return track_id

    def _is_front_cutin_track(self, point: Any) -> bool:
        if self._is_corner_live_track(point):
            return False
        source = str(safe_get(point, "radarSource", "frontRadar"))
        track_id = safe_optional_int(point, "trackId")
        if source == "scc" or (self.car_brand == "hyundai" and track_id == 0):
            return False
        d_rel = safe_float(point, "dRel", 0.0)
        y_rel = safe_float(point, "yRel", 0.0)
        return (
            FRONT_CUTIN_MIN_DREL_M <= d_rel <= min(FRONT_CUTIN_MAX_DREL_M, self.cutin_tuning["enter_max_x"])
            and abs(y_rel) <= FRONT_CUTIN_MAX_ABS_YREL_M
        )

    def _is_cutin_live_track(self, point: Any) -> bool:
        if self.cutin_radar_source == ROUTE_CUTIN_RADAR_SOURCE_FRONT:
            return self._is_front_cutin_track(point)
        track_id = safe_optional_int(point, "trackId")
        return self._is_corner_live_track(point) or (
            track_id is not None and track_id in self.cutin_side_corner_front_matches
        )

    def _track_id_is_corner_live(self, points: dict[int, Any], track_id: int) -> bool:
        point = points.get(track_id)
        if point is None:
            point = next((candidate for candidate in points.values() if safe_optional_int(candidate, "trackId") == track_id), None)
        return point is not None and self._is_corner_live_track(point)

    def _cutin_lane_geometry_available(self) -> bool:
        size = len(self.cutin_lane_xs)
        return size >= 2 and len(self.cutin_left_ys) == size and len(self.cutin_right_ys) == size

    def _cutin_dpath(self, d_rel: float, y_rel: float) -> tuple[float, float]:
        center_y = (
            np.interp(d_rel, self.cutin_lane_xs, self.cutin_left_ys)
            + np.interp(d_rel, self.cutin_lane_xs, self.cutin_right_ys)
        ) / 2.0
        left_y = np.interp(d_rel, self.cutin_lane_xs, self.cutin_left_ys)
        right_y = np.interp(d_rel, self.cutin_lane_xs, self.cutin_right_ys)
        lane_half_width = max(0.1, abs(right_y - left_y) / 2.0)
        d_path = float(y_rel + center_y)
        return d_path, max(0.0, 1.0 - abs(d_path) / lane_half_width)

    def _cutin_lane_half_width(self, d_rel: float) -> float:
        left_y = np.interp(d_rel, self.cutin_lane_xs, self.cutin_left_ys)
        right_y = np.interp(d_rel, self.cutin_lane_xs, self.cutin_right_ys)
        return max(0.1, abs(right_y - left_y) / 2.0)

    def _cutin_path_geometry_available(self) -> bool:
        return (
            len(self.cutin_path_xs) >= 2
            and len(self.cutin_path_ys) == len(self.cutin_path_xs)
        )

    def _cutin_path_dpath(self, d_rel: float, y_rel: float) -> tuple[float, float, float]:
        if not self._cutin_path_geometry_available():
            d_path, in_lane_prob = self._cutin_dpath(d_rel, y_rel)
            return d_path, in_lane_prob, float("inf")
        path_y = float(np.interp(d_rel, self.cutin_path_xs, self.cutin_path_ys))
        d_path = float(y_rel + path_y)
        lane_half_width = self._cutin_lane_half_width(d_rel)
        in_lane_prob = max(0.0, 1.0 - abs(d_path) / lane_half_width)
        path_y_std = (
            float(np.interp(d_rel, self.cutin_path_xs, self.cutin_path_y_stds))
            if len(self.cutin_path_y_stds) == len(self.cutin_path_xs)
            else float("inf")
        )
        return d_path, in_lane_prob, path_y_std

    def _cutin_debug_summary(
        self,
        diagnostics: list[ReplayCutinTrack],
        detections: list[DetectedVehicle],
    ) -> str:
        source = self.cutin_radar_source.upper()
        prefix = f"NEW CUTIN {source} S{self.cutin_sensitivity:.0f}: {'YES' if detections else 'NO'}"
        if not diagnostics:
            return prefix + f" | no {self.cutin_radar_source} tracks"
        reason_priority = {
            "enter": 0,
            "continue": 1,
            "future-lane": 1,
            "prob-gain": 2,
            "center-gain": 3,
            "lane-motion": 4,
            "track-age": 5,
            "already-center": 6,
            "behind-lead": 7,
            "range-speed": 8,
        }
        track = min(
            diagnostics,
            key=lambda item: (
                -item.cut_in_count,
                reason_priority.get(item.rejection_reason, 9),
                -item.inward_speed,
                item.d_rel,
            ),
        )
        threshold = self.cutin_tuning["enter_min_inward_speed"]
        confirm_frames = cutin_confirmation_frames(
            self.cutin_confirm_frames,
            track.d_rel,
            track.inward_speed,
            self.current_speed_kph / 3.6,
        )
        reason = track.rejection_reason
        if reason in ("enter", "continue") and track.cut_in_count < confirm_frames:
            reason = f"confirm {track.cut_in_count}/{confirm_frames}"
        return (
            f"{prefix} | id{track.track_id} x {track.d_rel:.1f}m dP {track.d_path:+.2f}m "
            f"in {track.inward_speed:.2f}/{threshold:.2f}mps {reason}"
        )

    def _radar_points_from_current_state(self, event_t: float) -> tuple[RadarPoint, ...]:
        if self.corner_source == ROUTE_CORNER_SOURCE_STABLE:
            stable_corner_points = self.raw_corner_tracker.points_at(event_t, self.current_speed_kph)
            if stable_corner_points:
                return stable_corner_points
        elif self.corner_source == ROUTE_CORNER_SOURCE_RAW:
            raw_corner_points = self._raw_corner_points_from_current_state(event_t)
            if raw_corner_points:
                return raw_corner_points

        live_tracks_fresh = event_t - self.live_track_radar_t < RADAR_POINT_STALE_S
        if live_tracks_fresh:
            corner_points = sorted_radar_points(
                point for point in self.live_track_radar_points.values() if point.source == CORNER_OBJECT_SOURCE
            )
            if corner_points:
                return corner_points

        if live_tracks_fresh:
            if self.corner_radar_active_for_display():
                return ()
            return sorted_radar_points(self.live_track_radar_points.values())
        return ()

    def _raw_corner_points_from_current_state(self, event_t: float) -> tuple[RadarPoint, ...]:
        points: list[RadarPoint] = []
        for key, obj in tuple(self.raw_corner_objects.items()):
            if event_t - obj.t > RAW_CORNER_TRACK_STALE_S:
                self.raw_corner_objects.pop(key, None)
                continue
            points.append(raw_corner_object_to_radar_point(obj, self.current_speed_kph))
        return sorted_radar_points(points)

    def _detected_vehicles_from_current_state(
        self,
        car_state: Any,
        event_t: float,
        lane_values: dict[str, Any],
        lane_change: str | None,
        lane_change_phase: str,
    ) -> tuple[DetectedVehicle, ...]:
        detections: list[DetectedVehicle] = []
        if event_t - self.model_detection_t < 0.8:
            detections.extend(self.model_detections)

        car_state_detections = () if self.front_radar_only else car_state_corner_detections(car_state)
        car_state_corner_labels = {vehicle.label for vehicle in car_state_detections}
        left_blindspot = bool(safe_get(car_state, "leftBlindspot", False))
        right_blindspot = bool(safe_get(car_state, "rightBlindspot", False))
        for vehicle in car_state_detections:
            if vehicle_is_blocked_by_near_road_edge(vehicle, lane_values):
                continue
            if (
                not vehicle_is_confirmed_corner_radar(vehicle)
                and not vehicle_is_inside_road_edges(vehicle, lane_values)
            ):
                continue
            if not has_nearby_vehicle(detections, vehicle, longitudinal_tolerance=3.0, lateral_tolerance=1.1):
                detections.append(vehicle)

        corner_detections = None if self.front_radar_only else self._corner_detections_for_current_state(
            event_t,
            lane_change,
            lane_change_phase,
        )
        if corner_detections is not None:
            for vehicle in corner_detections:
                if vehicle.label == "LR" and not left_blindspot:
                    continue
                if vehicle.label == "RR" and not right_blindspot:
                    continue
                if vehicle.label in car_state_corner_labels:
                    continue
                if vehicle_is_blocked_by_near_road_edge(vehicle, lane_values):
                    continue
                if (
                    not vehicle_is_confirmed_corner_radar(vehicle)
                    and not vehicle_is_inside_road_edges(vehicle, lane_values)
                ):
                    continue
                if not has_nearby_vehicle(detections, vehicle, longitudinal_tolerance=3.0, lateral_tolerance=1.1):
                    detections.append(vehicle)

        if event_t - self.radar_detection_t < 0.8:
            for vehicle in self.radar_detections:
                if vehicle.source == "radarState" and vehicle.cut_in:
                    detections.append(vehicle)
                    continue
                if vehicle_is_blocked_by_near_road_edge(vehicle, lane_values):
                    continue
                if (
                    not vehicle_is_inside_road_edges(vehicle, lane_values)
                    and not radar_vehicle_has_meaningful_motion(vehicle)
                ):
                    continue
                if vehicle.source == "radarState" or not has_nearby_vehicle(
                    detections,
                    vehicle,
                    longitudinal_tolerance=4.0,
                    lateral_tolerance=1.4,
                ):
                    detections.append(vehicle)

        return tuple(sorted(detections, key=lambda vehicle: vehicle.longitudinal_m))

    def _corner_detections_for_current_state(
        self,
        event_t: float,
        lane_change: str | None,
        lane_change_phase: str,
    ) -> tuple[DetectedVehicle, ...] | None:
        adrv_fresh = event_t - self.adrv_corner_message_t < CORNER_DETECTION_STALE_S
        ccnc_fresh = event_t - self.ccnc_corner_message_t < CORNER_DETECTION_STALE_S

        if adrv_fresh:
            return tuple(self.adrv_corner_detections.values())
        if ccnc_fresh:
            return tuple(self.ccnc_corner_detections.values())
        return None

    def _current_road_curvature(self) -> tuple[float | None, str]:
        if self.model_curvature_m_inv is not None:
            return self.model_curvature_m_inv, self.model_curvature_source
        if self.controls_curvature_m_inv is not None:
            return self.controls_curvature_m_inv, self.controls_curvature_source
        return None, "steeringAngleDeg"

    def _update_lane_change_meta(self, meta: Any, source: str) -> None:
        state = enum_text(safe_get(meta, "laneChangeState", self.lane_change_state))
        direction = enum_text(safe_get(meta, "laneChangeDirection", self.lane_change_direction))
        self.lane_change_state = state
        self.lane_change_direction = direction
        self.lane_change_source = source
        if source == "modelV2":
            self.model_lane_change_seen = True

    def _update_model_lane_change_values(self, meta: Any) -> None:
        self.lane_change_ll_prob = clamp(safe_float(meta, "laneChangeProb", self.lane_change_ll_prob), 0.0, 1.0)
        desire_state = safe_get(meta, "desireState")
        if desire_state is None or len(desire_state) <= 4:
            return
        left_prob = finite_float(desire_state[3])
        right_prob = finite_float(desire_state[4])
        if left_prob is not None:
            self.lane_change_desire_left_prob = clamp(left_prob, 0.0, 1.0)
        if right_prob is not None:
            self.lane_change_desire_right_prob = clamp(right_prob, 0.0, 1.0)

    def _update_model_action(self, action: Any) -> None:
        desired_velocity = safe_optional_float(action, "desiredVelocity")
        if desired_velocity is not None and 0.0 <= desired_velocity < 70.0:
            self.planned_speed_kph = desired_velocity * 3.6

        desired_accel = safe_optional_float(action, "desiredAcceleration")
        if desired_accel is not None:
            self.planned_accel_mps2 = clamp(desired_accel, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2)

        desired_curvature = safe_optional_float(action, "desiredCurvature")
        if desired_curvature is not None and abs(desired_curvature) < 0.05:
            self.model_action_curvature_m_inv = desired_curvature

        self.should_stop = bool(safe_get(action, "shouldStop", self.should_stop))

    def _update_model_meta_values(self, meta: Any, model: Any) -> None:
        self.model_confidence = enum_text(safe_get(model, "confidence", self.model_confidence))
        engaged_prob = safe_optional_float(meta, "engagedProb")
        if engaged_prob is not None:
            self.engaged_prob = clamp(engaged_prob, 0.0, 1.0)
        self.hard_brake_predicted = bool(safe_get(meta, "hardBrakePredicted", self.hard_brake_predicted))
        self.desire_state = numeric_tuple(safe_get(meta, "desireState"), limit=8, minimum=0.0, maximum=1.0)
        self.desire_prediction = desire_prediction_matrix(safe_get(meta, "desirePrediction"))
        self.risk_points = risk_points_from_meta(meta)
        self.brake_disengage_risk = list_max(safe_get(safe_get(meta, "disengagePredictions"), "brakeDisengageProbs"))
        self.gas_disengage_risk = list_max(safe_get(safe_get(meta, "disengagePredictions"), "gasDisengageProbs"))
        self.steer_override_risk = list_max(safe_get(safe_get(meta, "disengagePredictions"), "steerOverrideProbs"))
        self.hard_brake_risk = max(
            list_max(safe_get(safe_get(meta, "disengagePredictions"), "brake3MetersPerSecondSquaredProbs")),
            list_max(safe_get(safe_get(meta, "disengagePredictions"), "brake4MetersPerSecondSquaredProbs")),
            list_max(safe_get(safe_get(meta, "disengagePredictions"), "brake5MetersPerSecondSquaredProbs")),
        )
        self.gas_press_prob = list_max(safe_get(safe_get(meta, "disengagePredictions"), "gasPressProbs"))
        self.brake_press_prob = list_max(safe_get(safe_get(meta, "disengagePredictions"), "brakePressProbs"))
        self.disengage_risk = disengage_risk_from_meta(meta)
        self.lane_change_available_left = bool(safe_get(meta, "laneChangeAvailableLeft", False))
        self.lane_change_available_right = bool(safe_get(meta, "laneChangeAvailableRight", False))
        model_turn_speed = safe_optional_float(meta, "modelTurnSpeed")
        if model_turn_speed is not None and 0.0 < model_turn_speed < 90.0:
            self.model_turn_speed_kph = model_turn_speed * 3.6

        left_width = safe_optional_float(meta, "laneWidthLeft")
        right_width = safe_optional_float(meta, "laneWidthRight")
        self.left_lane_width_m = clamp(left_width, 0.0, 6.0) if left_width is not None else None
        self.right_lane_width_m = clamp(right_width, 0.0, 6.0) if right_width is not None else None

        left_distance = safe_optional_float(meta, "distanceToRoadEdgeLeft")
        right_distance = safe_optional_float(meta, "distanceToRoadEdgeRight")
        self.left_road_edge_distance_m = clamp(left_distance, 0.0, 20.0) if left_distance is not None else None
        self.right_road_edge_distance_m = clamp(right_distance, 0.0, 20.0) if right_distance is not None else None

    def _cruise_kph_from_car_state(self, car_state: Any) -> int | None:
        cruise_state = safe_get(car_state, "cruiseState")
        if cruise_state is not None and safe_get(cruise_state, "available", True) is False:
            return None

        for name in ("vCruiseCluster", "vCruise"):
            v_cruise = safe_float(car_state, name, 0.0)
            if 0.0 < v_cruise < 250.0:
                return int(round(v_cruise))

        if cruise_state is not None:
            speed_cluster_mps = safe_float(cruise_state, "speedCluster", 0.0)
            if 0.1 < speed_cluster_mps < 70.0:
                return int(round(speed_cluster_mps * 3.6))

            speed_mps = safe_float(cruise_state, "speed", 0.0)
            if 0.1 < speed_mps < 70.0:
                return int(round(speed_mps * 3.6))
        return None

    def _cruise_display_state_from_car_state(
        self,
        car_state: Any,
        cruise_kph: int | None,
    ) -> CruiseDisplayState:
        if cruise_kph is None:
            return "off"

        if self.selfdrive_enabled is not None:
            return "engaged" if self.selfdrive_enabled else "off"

        if self.controls_enabled is not None:
            return "engaged" if self.controls_enabled else "off"

        cruise_state = safe_get(car_state, "cruiseState")
        if cruise_state is not None and bool(safe_get(cruise_state, "enabled", False)):
            return "engaged"
        return "paused"

    def _speed_limit_kph_from_car_state(self, car_state: Any) -> int | None:
        speed_limit = safe_float(car_state, "speedLimit", 0.0)
        if speed_limit <= 0.0:
            return None
        return int(round(speed_limit))

    def _speed_limit_kph_from_nav_instruction(self, nav_instruction: Any) -> int | None:
        speed_limit = safe_float(nav_instruction, "speedLimit", 0.0)
        if speed_limit <= 0.1:
            return None
        rounded = int(round(speed_limit))
        integer_like = abs(speed_limit - rounded) < 0.05
        kph_like = speed_limit >= 45.0 or (speed_limit >= 30.0 and integer_like and rounded % 5 == 0)
        if kph_like:
            return rounded
        return int(round(speed_limit * 3.6))

    def _gear_text_from_car_state(self, car_state: Any) -> str | None:
        gear = safe_get(car_state, "gearShifter")
        if gear is None:
            return None
        gear_name = str(gear).split(".")[-1].strip().lower()
        if not gear_name:
            return None
        if "drive" in gear_name:
            gear_step = safe_optional_int(car_state, "gearStep")
            if gear_step is not None and 1 <= gear_step <= 8:
                return str(gear_step)
            return "D"
        if "park" in gear_name:
            return "P"
        if "reverse" in gear_name:
            return "R"
        if "neutral" in gear_name:
            return "N"
        if "sport" in gear_name:
            return "S"
        if "low" in gear_name:
            return "L"
        if "brake" in gear_name:
            return "B"
        if "eco" in gear_name:
            return "E"
        if "unknown" in gear_name:
            return "U"
        return "M"

    def _cruise_gap_from_car_state(self, car_state: Any) -> int | None:
        cruise_gap = safe_optional_int(car_state, "pcmCruiseGap")
        if cruise_gap is None or not 1 <= cruise_gap <= 4:
            return None
        return cruise_gap

    def _cruise_gap_from_personality(self, personality: Any) -> int | None:
        for value in (safe_get(personality, "raw"), personality):
            try:
                personality_index = int(value)
            except (TypeError, ValueError):
                continue
            if 0 <= personality_index <= 3:
                return personality_index + 1

        personality_name = str(personality).split(".")[-1].strip().replace("_", "").replace(" ", "").lower()
        return LONGITUDINAL_PERSONALITY_GAPS.get(personality_name)

    def _update_lane_styles_from_car_state(self, car_state: Any) -> None:
        left_code = safe_optional_int(car_state, "leftLaneLine")
        right_code = safe_optional_int(car_state, "rightLaneLine")
        if left_code is not None:
            self.left_lane_style = lane_style_from_code(left_code)
            self.left_lane_color = lane_color_from_code(left_code)
            if left_code < 0:
                self.left_lane_prob = 0.0
        if right_code is not None:
            self.right_lane_style = lane_style_from_code(right_code)
            self.right_lane_color = lane_color_from_code(right_code)
            if right_code < 0:
                self.right_lane_prob = 0.0

    def _lane_values(self) -> dict[str, Any]:
        width = clamp(self.lane_width_m, 2.4, 4.6)
        left_y = self.left_lane_y_m
        right_y = self.right_lane_y_m
        if left_y is None or right_y is None or right_y <= left_y:
            center_m: float | None = None
            return {
                "width": width,
                "center": center_m,
                "left_offset": -0.5,
                "right_offset": 0.5,
                "left_visible": self.left_lane_prob >= LANE_LINE_PROBABILITY_MIN,
                "right_visible": self.right_lane_prob >= LANE_LINE_PROBABILITY_MIN,
                "extra_left_visible": False,
                "extra_right_visible": False,
                "left_road_edge_offset": None,
                "right_road_edge_offset": None,
            }

        center_m = (left_y + right_y) * 0.5
        outer_left_offset = lane_offset_from_y(self.outer_left_lane_y_m, center_m, width)
        outer_right_offset = lane_offset_from_y(self.outer_right_lane_y_m, center_m, width)
        left_edge_offset = lane_offset_from_y(self.left_road_edge_y_m, center_m, width)
        right_edge_offset = lane_offset_from_y(self.right_road_edge_y_m, center_m, width)
        left_edge_visible = (
            left_edge_offset is not None
            and left_edge_offset < -0.68
        )
        right_edge_visible = (
            right_edge_offset is not None
            and right_edge_offset > 0.68
        )
        left_edge_bound = clamp(left_edge_offset, -2.8, -0.68) if left_edge_visible else None
        right_edge_bound = clamp(right_edge_offset, 0.68, 2.8) if right_edge_visible else None
        if left_edge_bound is not None and right_edge_bound is not None and left_edge_bound >= right_edge_bound:
            left_edge_bound = None
            right_edge_bound = None
        left_offset = clamp((left_y - center_m) / width, -0.75, -0.25)
        right_offset = clamp((right_y - center_m) / width, 0.25, 0.75)
        extra_left_visible = (
            outer_left_offset is not None
            and outer_left_offset < -0.78
            and self.outer_left_lane_prob >= LANE_LINE_PROBABILITY_MIN
            and lane_offset_inside_road_edges(outer_left_offset, left_edge_bound, right_edge_bound)
        )
        extra_right_visible = (
            outer_right_offset is not None
            and outer_right_offset > 0.78
            and self.outer_right_lane_prob >= LANE_LINE_PROBABILITY_MIN
            and lane_offset_inside_road_edges(outer_right_offset, left_edge_bound, right_edge_bound)
        )
        return {
            "width": width,
            "center": center_m,
            "left_offset": left_offset,
            "right_offset": right_offset,
            "left_visible": self.left_lane_prob >= LANE_LINE_PROBABILITY_MIN and lane_offset_inside_road_edges(left_offset, left_edge_bound, right_edge_bound),
            "right_visible": self.right_lane_prob >= LANE_LINE_PROBABILITY_MIN and lane_offset_inside_road_edges(right_offset, left_edge_bound, right_edge_bound),
            "extra_left_visible": extra_left_visible,
            "extra_right_visible": extra_right_visible,
            "left_road_edge_offset": left_edge_bound,
            "right_road_edge_offset": right_edge_bound,
        }

    def _lane_change_values(
        self,
        event_t: float,
        left_signal: bool,
        right_signal: bool,
        observed_ego_lane_offset: float,
    ) -> tuple[str | None, str, float, float, bool]:
        if LANE_CHANGE_MODEL_DIRECT_ONLY:
            return self._model_direct_lane_change_values(event_t)

        def remember(result: tuple[str | None, str, float, float, bool]) -> tuple[str | None, str, float, float, bool]:
            self.lane_change_previous_state = self.lane_change_state
            return result

        if self.model_lane_change_seen and self.lane_change_state == "off":
            if self.active_lane_change_direction is not None and self.lane_change_last_progress > 0.65:
                self.lane_change_recenter_direction = self.active_lane_change_direction
                self.lane_change_recenter_started_t = event_t
                self.lane_change_recenter_start_progress = clamp(self.lane_change_last_progress, 0.0, 1.0)
            self.lane_change_started_t = None
            self.active_lane_change_direction = None
            self.lane_change_continuation_active = False
            self.lane_change_peak_directional_observed_offset = 0.0
            recenter_values = self._lane_change_recenter_values(event_t)
            if recenter_values is not None:
                return remember(recenter_values)
            self.lane_change_last_progress = 0.0
            self.lane_change_recenter_start_progress = 1.0
            return remember((None, "idle", 0.0, 1.0, False))

        direction = self.lane_change_direction if self.lane_change_direction in ("left", "right") else None
        if direction is None and self.lane_change_state != "off":
            if left_signal and not right_signal:
                direction = "left"
            elif right_signal and not left_signal:
                direction = "right"
        active = direction is not None and self.lane_change_state != "off"
        if not active:
            if left_signal and not right_signal:
                direction = "left"
                active = True
            elif right_signal and not left_signal:
                direction = "right"
                active = True
        if not active:
            self.lane_change_started_t = None
            self.active_lane_change_direction = None
            self.lane_change_recenter_start_progress = 1.0
            self.lane_change_continuation_active = False
            self.lane_change_peak_directional_observed_offset = 0.0
            return remember((None, "idle", 0.0, 1.0, False))

        direction_sign = -1.0 if direction == "left" else 1.0
        directional_observed_offset = direction_sign * observed_ego_lane_offset
        model_reindexed_current_lane = (
            self.active_lane_change_direction == direction
            and self.lane_change_last_progress > 0.65
            and self.lane_change_peak_directional_observed_offset > LANE_CHANGE_REINDEX_PEAK_THRESHOLD
            and directional_observed_offset < LANE_CHANGE_REINDEX_RESET_THRESHOLD
        )
        same_direction_continuation = (
            (
                self.active_lane_change_direction == direction
                and self.lane_change_previous_state == "laneChangeFinishing"
                and self.lane_change_state == "laneChangeStarting"
                and self.lane_change_last_progress > 0.65
            )
            or (
                self.lane_change_recenter_direction == direction
                and self.lane_change_recenter_started_t is not None
            )
            or model_reindexed_current_lane
        )
        if (
            self.lane_change_started_t is None
            or self.active_lane_change_direction != direction
            or same_direction_continuation
        ):
            self.lane_change_started_t = event_t
            self.active_lane_change_direction = direction
            self.lane_change_recenter_direction = None
            self.lane_change_recenter_started_t = None
            self.lane_change_recenter_start_progress = 1.0
            self.lane_change_continuation_active = same_direction_continuation
            self.lane_change_peak_directional_observed_offset = max(0.0, directional_observed_offset)
        else:
            self.lane_change_peak_directional_observed_offset = max(
                self.lane_change_peak_directional_observed_offset,
                directional_observed_offset,
            )

        elapsed = max(0.0, event_t - self.lane_change_started_t)
        if self.lane_change_state == "preLaneChange":
            self.lane_change_last_progress = 0.0
            return remember((direction, "preparing", 0.0, 1.0, self.lane_change_continuation_active))

        if (
            self.lane_change_continuation_active
            and self.lane_change_source == "modelV2"
            and self.lane_change_state == "laneChangeStarting"
        ):
            model_progress = clamp(elapsed / 3.2, 0.0, 0.78)
        else:
            model_progress = self._model_lane_change_progress(direction, elapsed)
        if model_progress is not None:
            self.lane_change_last_progress = model_progress
            return remember(
                (
                    direction,
                    "changing",
                    model_progress,
                    1.0,
                    self.lane_change_continuation_active,
                )
            )

        if self.lane_change_state == "preLaneChange":
            progress = 0.0
        elif self.lane_change_state == "laneChangeFinishing":
            progress = clamp(0.55 + elapsed / 3.2, 0.55, 1.0)
        else:
            progress = clamp(elapsed / 3.2, 0.04, 0.92)
        self.lane_change_last_progress = progress
        return remember((direction, "changing", progress, 1.0, self.lane_change_continuation_active))

    def _lane_change_recenter_values(self, event_t: float) -> tuple[str, str, float, float, bool] | None:
        if self.lane_change_recenter_direction is None or self.lane_change_recenter_started_t is None:
            return None
        elapsed = max(0.0, event_t - self.lane_change_recenter_started_t)
        progress = clamp(elapsed / MODEL_DIRECT_LANE_RECENTER_SECONDS, 0.0, 1.0)
        if progress >= 1.0:
            self.lane_change_recenter_direction = None
            self.lane_change_recenter_started_t = None
            self.lane_change_recenter_start_progress = 1.0
            return None
        return (
            self.lane_change_recenter_direction,
            "recentering",
            progress,
            self.lane_change_recenter_start_progress,
            False,
        )

    def _model_lane_change_progress(self, direction: str, elapsed: float) -> float | None:
        if self.lane_change_source != "modelV2":
            return None
        if self.lane_change_state == "laneChangeFinishing":
            return clamp(0.78 + 0.22 * self.lane_change_ll_prob, 0.78, 1.0)
        if self.lane_change_state != "laneChangeStarting":
            return None

        desire_prob = (
            self.lane_change_desire_left_prob
            if direction == "left"
            else self.lane_change_desire_right_prob
        )
        lane_line_fade = 1.0 - self.lane_change_ll_prob
        fade_progress = 0.18 * lane_line_fade
        timer_progress = clamp(elapsed / 5.6, 0.0, 0.74)
        if desire_prob > 0.02:
            desire_progress = 0.20 + 0.56 * (1.0 - desire_prob)
        else:
            desire_progress = 0.0
        return clamp(max(fade_progress, timer_progress, desire_progress), 0.0, 0.78)

    def _model_direct_lane_change_values(self, event_t: float) -> tuple[str | None, str, float, float, bool]:
        self.lane_change_started_t = None
        self.lane_change_continuation_active = False
        self.lane_change_previous_state = self.lane_change_state
        self.lane_change_peak_directional_observed_offset = 0.0

        if not self.model_lane_change_seen:
            self._clear_model_direct_lane_change_state()
            return None, "idle", 0.0, 1.0, False

        direction = self.lane_change_direction if self.lane_change_direction in ("left", "right") else None
        if direction is None or self.lane_change_state == "off":
            recenter_values = self._model_direct_recenter_values(event_t)
            if recenter_values is not None:
                return recenter_values
            self._clear_model_direct_lane_change_state()
            return None, "idle", 0.0, 1.0, False

        self.lane_change_recenter_direction = None
        self.lane_change_recenter_started_t = None
        self.lane_change_recenter_start_progress = 1.0
        self.active_lane_change_direction = direction

        if self.lane_change_state == "preLaneChange":
            self.lane_change_last_progress = 0.0
            return direction, "preparing", 0.0, 1.0, False

        progress = self._model_direct_lane_change_value(direction)
        self.lane_change_last_progress = progress
        return direction, "changing", progress, 1.0, False

    def _model_direct_lane_change_value(self, direction: str) -> float:
        if self.lane_change_state == "laneChangeStarting":
            # The model desire for the active lane change fades out as the
            # maneuver completes, so visual position uses the complementary
            # value without timer or recenter synthesis.
            if direction == "left":
                return 1.0 - self.lane_change_desire_left_prob
            return 1.0 - self.lane_change_desire_right_prob
        if self.lane_change_state == "laneChangeFinishing":
            return 1.0
        return 0.0

    def _model_direct_recenter_values(self, event_t: float) -> tuple[str, str, float, float, bool] | None:
        if (
            self.lane_change_recenter_direction is None
            and self.active_lane_change_direction is not None
            and self.lane_change_last_progress >= MODEL_DIRECT_LANE_SETTLE_MIN_PROGRESS
        ):
            self.lane_change_recenter_direction = self.active_lane_change_direction
            self.lane_change_recenter_started_t = event_t
            self.lane_change_recenter_start_progress = clamp(self.lane_change_last_progress, 0.0, 1.0)
            self.active_lane_change_direction = None
            self.lane_change_last_progress = 0.0

        recenter_values = self._lane_change_recenter_values(event_t)
        if recenter_values is not None:
            return recenter_values
        return None

    def _clear_model_direct_lane_change_state(self) -> None:
        self.active_lane_change_direction = None
        self.lane_change_last_progress = 0.0
        self.lane_change_recenter_direction = None
        self.lane_change_recenter_started_t = None
        self.lane_change_recenter_start_progress = 1.0


def frame_to_state(frame: RouteReplayFrame) -> ClusterUiState:
    lane_width_m = clamp(frame.lane_width_m, 2.4, 4.6)
    observed_ego_lane_offset = 0.0
    if frame.lane_center_offset_m is not None:
        observed_ego_lane_offset = clamp(-frame.lane_center_offset_m / lane_width_m, -1.25, 1.25)

    (
        ego_lane_offset,
        road_view_lane_position,
        lane_grid_offset,
        highlight_lane_offset,
        use_animated_lane_grid,
    ) = route_lane_animation_values(frame, observed_ego_lane_offset)
    left_road_edge_offset = shifted_optional_offset(
        frame.left_road_edge_offset,
        lane_grid_offset if use_animated_lane_grid else 0.0,
    )
    right_road_edge_offset = shifted_optional_offset(
        frame.right_road_edge_offset,
        lane_grid_offset if use_animated_lane_grid else 0.0,
    )
    left_road_edge_points = (
        model_line_at(frame.model_road_edges, 0)
        if frame.left_road_edge_offset is not None
        else ()
    )
    left_road_edge_lateral_shift_m = model_line_lateral_shift(
        left_road_edge_points,
        frame,
        left_road_edge_offset,
        lane_grid_offset,
        use_animated_lane_grid,
    )
    right_road_edge_points = (
        model_line_at(frame.model_road_edges, 1)
        if frame.right_road_edge_offset is not None
        else ()
    )
    right_road_edge_lateral_shift_m = model_line_lateral_shift(
        right_road_edge_points,
        frame,
        right_road_edge_offset,
        lane_grid_offset,
        use_animated_lane_grid,
    )

    return ClusterUiState(
        speed_kph=frame.speed_kph,
        accel_mps2=frame.accel_mps2,
        steering=frame.steering,
        speed_limit_kph=frame.speed_limit_kph,
        speed_limit_source=frame.speed_limit_source,
        cruise_kph=frame.cruise_kph,
        cruise_display_state=frame.cruise_display_state,
        gear_text=frame.gear_text,
        cruise_gap=frame.cruise_gap,
        lfa_active=frame.lfa_active,
        active_lane_line=frame.active_lane_line,
        left_signal=frame.left_signal,
        right_signal=frame.right_signal,
        left_blindspot=frame.left_blindspot,
        right_blindspot=frame.right_blindspot,
        lane_change=frame.lane_change,
        lane_change_phase=frame.lane_change_phase,
        lane_change_progress=frame.lane_change_progress,
        highlight_lane=frame.lane_change,
        highlight_lane_offset=highlight_lane_offset,
        ego_lane_offset=ego_lane_offset,
        road_view_lane_position=road_view_lane_position,
        camera_lane_center_offset_m=frame.lane_center_offset_m,
        lane_width_m=lane_width_m,
        steering_angle_deg=frame.steering_angle_deg,
        surround_yaw_deg=0.0,
        surround_pitch_deg=0.0,
        surround_view_active=False,
        lanes=lanes_for_frame(frame, lane_grid_offset, use_animated_lane_grid),
        extra_left_lane_visible=frame.extra_left_lane_visible,
        extra_right_lane_visible=frame.extra_right_lane_visible,
        left_road_edge_offset=left_road_edge_offset,
        right_road_edge_offset=right_road_edge_offset,
        left_road_edge_points=left_road_edge_points,
        right_road_edge_points=right_road_edge_points,
        left_road_edge_lateral_shift_m=left_road_edge_lateral_shift_m,
        right_road_edge_lateral_shift_m=right_road_edge_lateral_shift_m,
        throttle=frame.throttle,
        brake=frame.brake,
        model_path=frame.model_path,
        detected_vehicles=frame.detected_vehicles,
        radar_points=frame.radar_points,
        corner_radar_supported=frame.corner_radar_supported,
        tpms=frame.tpms,
        ev_mode_valid=frame.ev_mode_valid,
        ev_mode_active=frame.ev_mode_valid and frame.ev_mode_active,
        planned_speed_kph=frame.planned_speed_kph,
        planned_accel_mps2=frame.planned_accel_mps2,
        planned_curvature_m_inv=frame.planned_curvature_m_inv,
        should_stop=frame.should_stop,
        model_confidence=frame.model_confidence,
        model_turn_speed_kph=frame.model_turn_speed_kph,
        engaged_prob=frame.engaged_prob,
        desire_state=frame.desire_state,
        desire_prediction=frame.desire_prediction,
        risk_points=frame.risk_points,
        brake_disengage_risk=frame.brake_disengage_risk,
        gas_disengage_risk=frame.gas_disengage_risk,
        steer_override_risk=frame.steer_override_risk,
        hard_brake_risk=frame.hard_brake_risk,
        gas_press_prob=frame.gas_press_prob,
        brake_press_prob=frame.brake_press_prob,
        disengage_risk=frame.disengage_risk,
        hard_brake_predicted=frame.hard_brake_predicted,
        lane_change_available_left=frame.lane_change_available_left,
        lane_change_available_right=frame.lane_change_available_right,
        lane_change_prob=frame.lane_change_prob,
        left_lane_width_m=frame.left_lane_width_m,
        right_lane_width_m=frame.right_lane_width_m,
        left_road_edge_distance_m=frame.left_road_edge_distance_m,
        right_road_edge_distance_m=frame.right_road_edge_distance_m,
        left_road_edge_confidence=frame.left_road_edge_confidence,
        right_road_edge_confidence=frame.right_road_edge_confidence,
        frame_age=frame.frame_age,
        frame_drop_perc=frame.frame_drop_perc,
        model_execution_time_ms=frame.model_execution_time_ms,
        vision_speed_mps=frame.vision_speed_mps,
        vision_yaw_rate_rps=frame.vision_yaw_rate_rps,
        vision_speed_std_mps=frame.vision_speed_std_mps,
        vision_yaw_rate_std_rps=frame.vision_yaw_rate_std_rps,
        camera_device_type=frame.camera_device_type,
        camera_sensor=frame.camera_sensor,
        camera_calibration_euler=frame.camera_calibration_euler,
        road_transform_trans=frame.road_transform_trans,
        road_transform_std=frame.road_transform_std,
        camera_odometry_valid=frame.camera_odometry_valid,
        longitudinal_plan_source=frame.longitudinal_plan_source,
        longitudinal_plan_speeds_kph=frame.longitudinal_plan_speeds_kph,
        longitudinal_plan_accels_mps2=frame.longitudinal_plan_accels_mps2,
        longitudinal_plan_jerks_mps3=frame.longitudinal_plan_jerks_mps3,
        longitudinal_plan_fcw=frame.longitudinal_plan_fcw,
        longitudinal_plan_should_stop=frame.longitudinal_plan_should_stop,
        longitudinal_plan_allow_throttle=frame.longitudinal_plan_allow_throttle,
        longitudinal_plan_allow_brake=frame.longitudinal_plan_allow_brake,
        longitudinal_t_follow_s=frame.longitudinal_t_follow_s,
        longitudinal_desired_distance_m=frame.longitudinal_desired_distance_m,
        longitudinal_v_target_kph=frame.longitudinal_v_target_kph,
        longitudinal_jerk_target_mps3=frame.longitudinal_jerk_target_mps3,
        lateral_plan_valid=frame.lateral_plan_valid,
        lateral_plan_use_lane_lines=frame.lateral_plan_use_lane_lines,
        lateral_plan_solver_cost=frame.lateral_plan_solver_cost,
        lateral_plan_debug_text=frame.lateral_plan_debug_text,
        lateral_plan_curvatures=frame.lateral_plan_curvatures,
        lateral_plan_curvature_rates=frame.lateral_plan_curvature_rates,
        display_speed_kph=frame.display_speed_kph,
        traffic_state=frame.traffic_state,
        driving_mode=frame.driving_mode,
        trip_report=frame.trip_report,
        cpu_usage_percent=frame.cpu_usage_percent,
        cpu_temp_c=frame.cpu_temp_c,
        memory_used_percent=frame.memory_used_percent,
        disk_used_percent=frame.disk_used_percent,
        recorded_cutin_active=frame.recorded_cutin_active,
        recorded_cutin_sound=frame.recorded_cutin_sound,
        alert=frame.alert,
    )


def route_lane_animation_values(
    frame: RouteReplayFrame,
    observed_ego_lane_offset: float,
) -> tuple[float, float, float, float | None, bool]:
    if frame.lane_change not in ("left", "right"):
        return observed_ego_lane_offset, 0.0, 0.0, None, False

    direction_sign = -1.0 if frame.lane_change == "left" else 1.0
    highlight_lane_offset: float | None = direction_sign
    if not ROUTE_REPLAY_USE_LANE_CHANGE_ANIMATION:
        signal_matches_lane_change = (
            (frame.lane_change == "left" and frame.left_signal)
            or (frame.lane_change == "right" and frame.right_signal)
        )
        highlight_lane_offset = direction_sign if signal_matches_lane_change else None
        return observed_ego_lane_offset, 0.0, 0.0, highlight_lane_offset, False

    if frame.lane_change_phase == "preparing":
        return 0.0, 0.0, 0.0, highlight_lane_offset, True

    if frame.lane_change_phase == "changing":
        if LANE_CHANGE_MODEL_DIRECT_ONLY:
            ego_lane_offset = direction_sign * clamp(frame.lane_change_progress, 0.0, 1.0)
            return ego_lane_offset, 0.0, 0.0, highlight_lane_offset, True

        lane_grid_offset = 0.0
        if frame.lane_change_continuation:
            rebase_progress = clamp(
                frame.lane_change_progress / CONTINUOUS_LANE_CHANGE_REBASE_PROGRESS,
                0.0,
                1.0,
            )
            rebase_blend = smoothstep(rebase_progress)
            lane_grid_offset = -direction_sign * (1.0 - rebase_blend)
            change_progress = clamp(
                (
                    frame.lane_change_progress
                    - CONTINUOUS_LANE_CHANGE_REBASE_PROGRESS
                )
                / (1.0 - CONTINUOUS_LANE_CHANGE_REBASE_PROGRESS),
                0.0,
                1.0,
            )
            ego_lane_offset = direction_sign * smoothstep(change_progress)
        else:
            ego_lane_offset = direction_sign * smoothstep(frame.lane_change_progress)
        return ego_lane_offset, lane_grid_offset, lane_grid_offset, highlight_lane_offset, True

    if frame.lane_change_phase == "recentering":
        recenter_blend = smoothstep(frame.lane_change_progress)
        start_ego_offset = direction_sign * smoothstep(frame.lane_change_recenter_start_progress)
        lane_grid_offset = -direction_sign * recenter_blend
        ego_lane_offset = start_ego_offset * (1.0 - recenter_blend) + observed_ego_lane_offset * recenter_blend
        return ego_lane_offset, lane_grid_offset, lane_grid_offset, None, True

    return observed_ego_lane_offset, 0.0, 0.0, None, False


def shifted_optional_offset(offset: float | None, shift: float) -> float | None:
    return None if offset is None else offset + shift


def blend_frames(left: RouteReplayFrame, right: RouteReplayFrame, amount: float) -> RouteReplayFrame:
    def lerp(a: float, b: float) -> float:
        return a + (b - a) * amount

    def lerp_optional(a: float | None, b: float | None) -> float | None:
        if a is None:
            return b
        if b is None:
            return a
        return lerp(a, b)

    discrete = left if amount < 0.5 else right
    if LANE_CHANGE_MODEL_DIRECT_ONLY:
        lane_change_progress = discrete.lane_change_progress
    elif (
        left.lane_change == right.lane_change
        and left.lane_change_phase == right.lane_change_phase
        and left.lane_change_continuation == right.lane_change_continuation
        and right.lane_change_progress >= left.lane_change_progress
    ):
        lane_change_progress = lerp(left.lane_change_progress, right.lane_change_progress)
    else:
        lane_change_progress = discrete.lane_change_progress
    return RouteReplayFrame(
        t=lerp(left.t, right.t),
        speed_kph=lerp(left.speed_kph, right.speed_kph),
        display_speed_kph=lerp_optional(left.display_speed_kph, right.display_speed_kph),
        accel_mps2=lerp(left.accel_mps2, right.accel_mps2),
        steering=lerp(left.steering, right.steering),
        steering_angle_deg=lerp_optional(left.steering_angle_deg, right.steering_angle_deg),
        speed_limit_kph=discrete.speed_limit_kph,
        speed_limit_source=discrete.speed_limit_source,
        cruise_kph=discrete.cruise_kph,
        cruise_display_state=discrete.cruise_display_state,
        gear_text=discrete.gear_text,
        cruise_gap=discrete.cruise_gap,
        traffic_state=discrete.traffic_state,
        driving_mode=discrete.driving_mode,
        lfa_active=discrete.lfa_active,
        active_lane_line=discrete.active_lane_line,
        left_signal=discrete.left_signal,
        right_signal=discrete.right_signal,
        left_blindspot=discrete.left_blindspot,
        right_blindspot=discrete.right_blindspot,
        lane_width_m=lerp(left.lane_width_m, right.lane_width_m),
        lane_center_offset_m=lerp_optional(left.lane_center_offset_m, right.lane_center_offset_m),
        left_lane_offset=lerp(left.left_lane_offset, right.left_lane_offset),
        right_lane_offset=lerp(left.right_lane_offset, right.right_lane_offset),
        left_lane_visible=discrete.left_lane_visible,
        right_lane_visible=discrete.right_lane_visible,
        extra_left_lane_visible=discrete.extra_left_lane_visible,
        extra_right_lane_visible=discrete.extra_right_lane_visible,
        left_road_edge_offset=lerp_optional(left.left_road_edge_offset, right.left_road_edge_offset),
        right_road_edge_offset=lerp_optional(left.right_road_edge_offset, right.right_road_edge_offset),
        left_lane_style=discrete.left_lane_style,
        right_lane_style=discrete.right_lane_style,
        left_lane_color=discrete.left_lane_color,
        right_lane_color=discrete.right_lane_color,
        road_curvature=lerp_optional(left.road_curvature, right.road_curvature),
        road_curvature_source=discrete.road_curvature_source,
        lane_position_source=discrete.lane_position_source,
        model_lane_lines=discrete.model_lane_lines,
        model_road_edges=discrete.model_road_edges,
        model_path=discrete.model_path,
        model_path_source=discrete.model_path_source,
        lane_change_source=discrete.lane_change_source,
        lane_change=discrete.lane_change,
        lane_change_phase=discrete.lane_change_phase,
        lane_change_progress=lane_change_progress,
        lane_change_recenter_start_progress=discrete.lane_change_recenter_start_progress,
        lane_change_continuation=discrete.lane_change_continuation,
        throttle=lerp(left.throttle, right.throttle),
        brake=lerp(left.brake, right.brake),
        detected_vehicles=discrete.detected_vehicles,
        radar_points=discrete.radar_points,
        corner_radar_supported=discrete.corner_radar_supported,
        tpms=discrete.tpms,
        ev_mode_valid=discrete.ev_mode_valid,
        ev_mode_active=discrete.ev_mode_valid and discrete.ev_mode_active,
        planned_speed_kph=lerp_optional(left.planned_speed_kph, right.planned_speed_kph),
        planned_accel_mps2=lerp_optional(left.planned_accel_mps2, right.planned_accel_mps2),
        planned_curvature_m_inv=lerp_optional(left.planned_curvature_m_inv, right.planned_curvature_m_inv),
        should_stop=discrete.should_stop,
        model_confidence=discrete.model_confidence,
        model_turn_speed_kph=lerp_optional(left.model_turn_speed_kph, right.model_turn_speed_kph),
        engaged_prob=lerp_optional(left.engaged_prob, right.engaged_prob),
        desire_state=discrete.desire_state,
        desire_prediction=discrete.desire_prediction,
        risk_points=discrete.risk_points,
        brake_disengage_risk=lerp(left.brake_disengage_risk, right.brake_disengage_risk),
        gas_disengage_risk=lerp(left.gas_disengage_risk, right.gas_disengage_risk),
        steer_override_risk=lerp(left.steer_override_risk, right.steer_override_risk),
        hard_brake_risk=lerp(left.hard_brake_risk, right.hard_brake_risk),
        gas_press_prob=lerp(left.gas_press_prob, right.gas_press_prob),
        brake_press_prob=lerp(left.brake_press_prob, right.brake_press_prob),
        disengage_risk=lerp(left.disengage_risk, right.disengage_risk),
        hard_brake_predicted=discrete.hard_brake_predicted,
        lane_change_available_left=discrete.lane_change_available_left,
        lane_change_available_right=discrete.lane_change_available_right,
        lane_change_prob=lerp(left.lane_change_prob, right.lane_change_prob),
        left_lane_width_m=lerp_optional(left.left_lane_width_m, right.left_lane_width_m),
        right_lane_width_m=lerp_optional(left.right_lane_width_m, right.right_lane_width_m),
        left_road_edge_distance_m=lerp_optional(left.left_road_edge_distance_m, right.left_road_edge_distance_m),
        right_road_edge_distance_m=lerp_optional(left.right_road_edge_distance_m, right.right_road_edge_distance_m),
        left_road_edge_confidence=lerp(left.left_road_edge_confidence, right.left_road_edge_confidence),
        right_road_edge_confidence=lerp(left.right_road_edge_confidence, right.right_road_edge_confidence),
        frame_age=discrete.frame_age,
        frame_drop_perc=lerp_optional(left.frame_drop_perc, right.frame_drop_perc),
        model_execution_time_ms=lerp_optional(left.model_execution_time_ms, right.model_execution_time_ms),
        vision_speed_mps=lerp_optional(left.vision_speed_mps, right.vision_speed_mps),
        vision_yaw_rate_rps=lerp_optional(left.vision_yaw_rate_rps, right.vision_yaw_rate_rps),
        vision_speed_std_mps=lerp_optional(left.vision_speed_std_mps, right.vision_speed_std_mps),
        vision_yaw_rate_std_rps=lerp_optional(left.vision_yaw_rate_std_rps, right.vision_yaw_rate_std_rps),
        camera_device_type=discrete.camera_device_type,
        camera_sensor=discrete.camera_sensor,
        cpu_usage_percent=lerp_optional(left.cpu_usage_percent, right.cpu_usage_percent),
        cpu_temp_c=lerp_optional(left.cpu_temp_c, right.cpu_temp_c),
        memory_used_percent=lerp_optional(left.memory_used_percent, right.memory_used_percent),
        disk_used_percent=lerp_optional(left.disk_used_percent, right.disk_used_percent),
        camera_calibration_euler=discrete.camera_calibration_euler,
        road_transform_trans=discrete.road_transform_trans,
        road_transform_std=discrete.road_transform_std,
        camera_odometry_valid=discrete.camera_odometry_valid,
        longitudinal_plan_source=discrete.longitudinal_plan_source,
        longitudinal_plan_speeds_kph=discrete.longitudinal_plan_speeds_kph,
        longitudinal_plan_accels_mps2=discrete.longitudinal_plan_accels_mps2,
        longitudinal_plan_jerks_mps3=discrete.longitudinal_plan_jerks_mps3,
        longitudinal_plan_fcw=discrete.longitudinal_plan_fcw,
        longitudinal_plan_should_stop=discrete.longitudinal_plan_should_stop,
        longitudinal_plan_allow_throttle=discrete.longitudinal_plan_allow_throttle,
        longitudinal_plan_allow_brake=discrete.longitudinal_plan_allow_brake,
        longitudinal_t_follow_s=lerp_optional(left.longitudinal_t_follow_s, right.longitudinal_t_follow_s),
        longitudinal_desired_distance_m=lerp_optional(left.longitudinal_desired_distance_m, right.longitudinal_desired_distance_m),
        longitudinal_v_target_kph=lerp_optional(left.longitudinal_v_target_kph, right.longitudinal_v_target_kph),
        longitudinal_jerk_target_mps3=lerp_optional(left.longitudinal_jerk_target_mps3, right.longitudinal_jerk_target_mps3),
        lateral_plan_valid=discrete.lateral_plan_valid,
        lateral_plan_use_lane_lines=discrete.lateral_plan_use_lane_lines,
        lateral_plan_solver_cost=lerp_optional(left.lateral_plan_solver_cost, right.lateral_plan_solver_cost),
        lateral_plan_debug_text=discrete.lateral_plan_debug_text,
        lateral_plan_curvatures=discrete.lateral_plan_curvatures,
        lateral_plan_curvature_rates=discrete.lateral_plan_curvature_rates,
        cutin_debug_text=discrete.cutin_debug_text,
        recorded_cutin_active=discrete.recorded_cutin_active,
        recorded_cutin_sound=discrete.recorded_cutin_sound,
        trip_report=discrete.trip_report,
        alert=discrete.alert,
    )


def lanes_for_frame(
    frame: RouteReplayFrame,
    lane_grid_offset: float = 0.0,
    use_animated_lane_grid: bool = False,
) -> tuple[LaneMarking, ...]:
    left_inner_color = frame.left_lane_color or BLUE
    right_inner_color = frame.right_lane_color or BLUE
    left_outer_color = WHITE
    right_outer_color = WHITE
    if frame.lane_change == "left":
        left_outer_color = BLUE_SOFT
    elif frame.lane_change == "right":
        right_outer_color = BLUE_SOFT

    if use_animated_lane_grid:
        left_inner = lane_grid_offset - 0.5
        right_inner = lane_grid_offset + 0.5
    else:
        left_inner = frame.left_lane_offset + lane_grid_offset
        right_inner = frame.right_lane_offset + lane_grid_offset

    # Model lane probabilities intentionally fade during a lane change. The
    # animated grid owns visual continuity for that interval, so its current
    # lane boundaries must remain visible while the grid moves.
    left_inner_visible = frame.left_lane_visible or use_animated_lane_grid
    right_inner_visible = frame.right_lane_visible or use_animated_lane_grid
    road_edge_shift = lane_grid_offset if use_animated_lane_grid else 0.0
    left_road_edge_offset = shifted_optional_offset(frame.left_road_edge_offset, road_edge_shift)
    right_road_edge_offset = shifted_optional_offset(frame.right_road_edge_offset, road_edge_shift)

    markings: list[LaneMarking] = []
    for index, points in enumerate(frame.model_lane_lines):
        if not points:
            continue
        if index == 1:
            offset = left_inner
            color = left_inner_color
            style = frame.left_lane_style
            visible = left_inner_visible
            width = 7
        elif index == 2:
            offset = right_inner
            color = right_inner_color
            style = frame.right_lane_style
            visible = right_inner_visible
            width = 7
        elif index == 0:
            offset = model_lane_offset_for_index(
                index,
                points,
                frame,
                left_inner,
                right_inner,
                lane_grid_offset,
            )
            color = model_lane_color_for_index(index, frame.lane_change)
            style = model_lane_style_for_index(index)
            visible = frame.extra_left_lane_visible or (
                use_animated_lane_grid and frame.lane_change == "left"
            )
            width = 5
        elif index == 3:
            offset = model_lane_offset_for_index(
                index,
                points,
                frame,
                left_inner,
                right_inner,
                lane_grid_offset,
            )
            color = model_lane_color_for_index(index, frame.lane_change)
            style = model_lane_style_for_index(index)
            visible = frame.extra_right_lane_visible or (
                use_animated_lane_grid and frame.lane_change == "right"
            )
            width = 5
        else:
            offset = model_lane_offset_for_index(
                index,
                points,
                frame,
                left_inner,
                right_inner,
                lane_grid_offset,
            )
            color = model_lane_color_for_index(index, frame.lane_change)
            style = model_lane_style_for_index(index)
            visible = True
            width = 5
        visible = visible and lane_offset_inside_road_edges(
            offset,
            left_road_edge_offset,
            right_road_edge_offset,
        )
        markings.append(
            LaneMarking(
                offset,
                color,
                style,
                visible=visible,
                width=width,
                model_points=points,
                model_lateral_shift_m=model_line_lateral_shift(
                    points,
                    frame,
                    offset,
                    lane_grid_offset,
                    use_animated_lane_grid,
                ),
            )
        )
    if markings:
        return tuple(markings)

    if use_animated_lane_grid and frame.lane_change == "left":
        left_outer = left_inner - 1.0
        left_outer_points = model_line_at(frame.model_lane_lines, 0)
        markings.append(
            LaneMarking(
                left_outer,
                left_outer_color,
                "solid",
                visible=(frame.extra_left_lane_visible or use_animated_lane_grid) and lane_offset_inside_road_edges(
                    left_outer,
                    left_road_edge_offset,
                    right_road_edge_offset,
                ),
                width=5,
                model_points=left_outer_points,
                model_lateral_shift_m=model_line_lateral_shift(
                    left_outer_points,
                    frame,
                    left_outer,
                    lane_grid_offset,
                    use_animated_lane_grid,
                ),
            )
        )
    left_inner_points = model_line_at(frame.model_lane_lines, 1)
    markings.append(
        LaneMarking(
            left_inner,
            left_inner_color,
            frame.left_lane_style,
            visible=left_inner_visible and lane_offset_inside_road_edges(
                left_inner,
                left_road_edge_offset,
                right_road_edge_offset,
            ),
            width=7,
            model_points=left_inner_points,
            model_lateral_shift_m=model_line_lateral_shift(
                left_inner_points,
                frame,
                left_inner,
                lane_grid_offset,
                use_animated_lane_grid,
            ),
        )
    )
    right_inner_points = model_line_at(frame.model_lane_lines, 2)
    markings.append(
        LaneMarking(
            right_inner,
            right_inner_color,
            frame.right_lane_style,
            visible=right_inner_visible and lane_offset_inside_road_edges(
                right_inner,
                left_road_edge_offset,
                right_road_edge_offset,
            ),
            width=7,
            model_points=right_inner_points,
            model_lateral_shift_m=model_line_lateral_shift(
                right_inner_points,
                frame,
                right_inner,
                lane_grid_offset,
                use_animated_lane_grid,
            ),
        )
    )
    if use_animated_lane_grid and frame.lane_change == "right":
        right_outer = right_inner + 1.0
        right_outer_points = model_line_at(frame.model_lane_lines, 3)
        markings.append(
            LaneMarking(
                right_outer,
                right_outer_color,
                "dashed",
                visible=(frame.extra_right_lane_visible or use_animated_lane_grid) and lane_offset_inside_road_edges(
                    right_outer,
                    left_road_edge_offset,
                    right_road_edge_offset,
                ),
                width=5,
                model_points=right_outer_points,
                model_lateral_shift_m=model_line_lateral_shift(
                    right_outer_points,
                    frame,
                    right_outer,
                    lane_grid_offset,
                    use_animated_lane_grid,
                ),
            )
        )
    return tuple(markings)


def model_lane_offset_for_index(
    index: int,
    points: tuple[ModelPathPoint, ...],
    frame: RouteReplayFrame,
    left_inner: float,
    right_inner: float,
    lane_grid_offset: float,
) -> float:
    center_m = frame.lane_center_offset_m
    if center_m is not None and points:
        offset = lane_offset_from_y(points[0].lateral_m, center_m, frame.lane_width_m)
        if offset is not None:
            return offset + lane_grid_offset
    if index < 1:
        return left_inner - (1 - index)
    if index > 2:
        return right_inner + (index - 2)
    return left_inner if index == 1 else right_inner


def model_lane_color_for_index(index: int, lane_change: str | None) -> tuple[int, int, int]:
    if index in (1, 2):
        return BLUE
    if index == 0 and lane_change == "left":
        return BLUE_SOFT
    if index == 3 and lane_change == "right":
        return BLUE_SOFT
    return WHITE


def model_lane_style_for_index(index: int) -> str:
    if index < 2:
        return "solid"
    return "dashed"


def model_line_at(
    lines: tuple[tuple[ModelPathPoint, ...], ...],
    index: int,
) -> tuple[ModelPathPoint, ...]:
    if index < 0 or index >= len(lines):
        return ()
    return lines[index]


def model_line_lateral_shift(
    points: tuple[ModelPathPoint, ...],
    frame: RouteReplayFrame,
    baseline_offset: float | None,
    lane_grid_offset: float,
    use_animated_lane_grid: bool,
) -> float:
    if not points:
        return 0.0
    lane_width_m = max(0.1, frame.lane_width_m)
    if use_animated_lane_grid and baseline_offset is not None:
        origin_lateral_m = points[0].lateral_m
        base_lateral_m = baseline_offset * lane_width_m
        return base_lateral_m - origin_lateral_m

    center_m = frame.lane_center_offset_m or 0.0
    shift_m = lane_grid_offset * lane_width_m
    return -center_m + shift_m


def model_line_points(line: Any) -> tuple[ModelPathPoint, ...]:
    xs = safe_get(line, "x")
    ys = safe_get(line, "y")
    if xs is None or ys is None:
        return ()

    count = min(len(xs), len(ys))
    points: list[ModelPathPoint] = []
    previous_forward_m = -1.0
    for index in range(count):
        forward_m = finite_float(xs[index])
        lateral_m = finite_float(ys[index])
        if forward_m is None or lateral_m is None:
            continue
        if not 0.0 <= forward_m <= 160.0:
            continue
        if abs(lateral_m) > 24.0:
            continue
        if forward_m <= previous_forward_m + 0.01:
            continue
        points.append(ModelPathPoint(forward_m=forward_m, lateral_m=lateral_m))
        previous_forward_m = forward_m
    return tuple(points)


def model_path_points_from_model_v2(model: Any) -> tuple[ModelPathPoint, ...]:
    position = safe_get(model, "position")
    if position is None:
        return ()
    xs = safe_get(position, "x")
    ys = safe_get(position, "y")
    if xs is None or ys is None:
        return ()
    y_stds = safe_get(position, "yStd")
    velocity = safe_get(model, "velocity")
    acceleration = safe_get(model, "acceleration")
    orientation = safe_get(model, "orientation")
    orientation_rate = safe_get(model, "orientationRate")
    speeds = safe_get(velocity, "x") if velocity is not None else None
    accels = safe_get(acceleration, "x") if acceleration is not None else None
    orientations = safe_get(orientation, "z") if orientation is not None else None
    orientation_rates = safe_get(orientation_rate, "z") if orientation_rate is not None else None

    count = min(len(xs), len(ys))
    points: list[ModelPathPoint] = []
    previous_forward_m = -1.0
    for index in range(count):
        forward_m = finite_float(xs[index])
        lateral_m = finite_float(ys[index])
        if forward_m is None or lateral_m is None:
            continue
        if not 0.0 <= forward_m <= 140.0:
            continue
        if abs(lateral_m) > 18.0:
            continue
        if forward_m <= previous_forward_m + 0.01:
            continue
        points.append(
            ModelPathPoint(
                forward_m=forward_m,
                lateral_m=lateral_m,
                lateral_std_m=list_value(y_stds, index),
                speed_mps=list_value(speeds, index),
                accel_mps2=list_value(accels, index),
                orientation_rad=list_value(orientations, index),
                orientation_rate_rps=list_value(orientation_rates, index),
            )
        )
        previous_forward_m = forward_m
    return tuple(points)


def model_lead_detections_from_model_v2(model: Any) -> tuple[DetectedVehicle, ...]:
    leads = safe_get(model, "leadsV3")
    if leads is None:
        return ()

    model_velocity = safe_get(model, "velocity")
    model_speed_mps = first_list_value(safe_get(model_velocity, "x")) if model_velocity is not None else None
    detections: list[DetectedVehicle] = []
    for index, lead in enumerate(leads):
        probability = clamp(safe_float(lead, "prob", 0.0), 0.0, 1.0)
        if probability < MODEL_LEAD_MIN_PROB:
            continue
        x_m = first_list_value(safe_get(lead, "x"))
        y_m = first_list_value(safe_get(lead, "y"))
        if x_m is None or y_m is None:
            continue
        longitudinal_m = x_m - RADAR_TO_CAMERA_M
        if not 0.2 < longitudinal_m < 180.0 or abs(y_m) > 8.0:
            continue

        lead_speed_mps = first_list_value(safe_get(lead, "v"))
        relative_speed_mps = (
            lead_speed_mps - model_speed_mps
            if lead_speed_mps is not None and model_speed_mps is not None
            else None
        )
        acceleration_mps2 = first_list_value(safe_get(lead, "a"))
        x_std_m = first_list_value(safe_get(lead, "xStd"))
        y_std_m = first_list_value(safe_get(lead, "yStd"))
        detections.append(
            DetectedVehicle(
                label=f"M{index + 1}",
                longitudinal_m=longitudinal_m,
                lateral_m=clamp(y_m, -8.0, 8.0),
                source="modelV2.leadsV3",
                probability=probability,
                relative_speed_mps=relative_speed_mps,
                absolute_speed_kph=max(0.0, lead_speed_mps * 3.6) if lead_speed_mps is not None else None,
                acceleration_mps2=acceleration_mps2,
                primary=index == 0,
                ttc_s=ttc_from_relative_speed(longitudinal_m, relative_speed_mps),
                x_std_m=x_std_m,
                y_std_m=y_std_m,
            )
        )
    return tuple(detections)


def ttc_from_relative_speed(longitudinal_m: float, relative_speed_mps: float | None) -> float | None:
    if longitudinal_m <= 0.0 or relative_speed_mps is None or relative_speed_mps >= -0.15:
        return None
    ttc_s = longitudinal_m / max(0.15, -relative_speed_mps)
    return clamp(ttc_s, 0.0, 99.0)


def lane_offset_from_y(y_m: float | None, center_m: float, lane_width_m: float) -> float | None:
    if y_m is None:
        return None
    return (y_m - center_m) / max(0.1, lane_width_m)


def lane_offset_inside_road_edges(
    offset: float | None,
    left_road_edge_offset: float | None,
    right_road_edge_offset: float | None,
) -> bool:
    if offset is None:
        return True
    if left_road_edge_offset is not None and offset < left_road_edge_offset:
        return False
    if right_road_edge_offset is not None and offset > right_road_edge_offset:
        return False
    return True


def road_edge_confidence_from_std(std: float | None) -> float:
    if std is None:
        return 0.0
    return clamp(1.0 - std / 2.0, 0.0, 1.0)


def disengage_risk_from_meta(meta: Any) -> float:
    predictions = safe_get(meta, "disengagePredictions")
    if predictions is None:
        return 0.0
    values = (
        list_max(safe_get(predictions, "brakeDisengageProbs")),
        list_max(safe_get(predictions, "gasDisengageProbs")),
        list_max(safe_get(predictions, "steerOverrideProbs")),
        list_max(safe_get(predictions, "brake3MetersPerSecondSquaredProbs")),
        list_max(safe_get(predictions, "brake4MetersPerSecondSquaredProbs")),
        list_max(safe_get(predictions, "brake5MetersPerSecondSquaredProbs")),
    )
    return clamp(max(values), 0.0, 1.0)


def list_max(values: Any) -> float:
    if values is None:
        return 0.0
    maximum = 0.0
    for value in values:
        parsed = finite_float(value)
        if parsed is not None:
            maximum = max(maximum, parsed)
    return maximum


def car_state_corner_detections(car_state: Any) -> tuple[DetectedVehicle, ...]:
    left_blindspot = bool(safe_get(car_state, "leftBlindspot", False))
    right_blindspot = bool(safe_get(car_state, "rightBlindspot", False))
    pairs = (
        ("LF", "leftLongDist", "leftLatDist", -1.0, 1.0),
        ("RF", "rightLongDist", "rightLatDist", 1.0, 1.0),
        ("LR", "leftRearLongDist", "leftRearLatDist", -1.0, -1.0),
        ("RR", "rightRearLongDist", "rightRearLatDist", 1.0, -1.0),
    )
    detections: list[DetectedVehicle] = []
    for label, distance_name, lateral_name, side, forward_sign in pairs:
        if label == "LR" and not left_blindspot:
            continue
        if label == "RR" and not right_blindspot:
            continue
        distance_m = safe_float(car_state, distance_name, 0.0)
        if not 0.2 < distance_m < 180.0:
            continue
        longitudinal_m = forward_sign * distance_m
        lateral_mag = normalized_lateral_m(safe_float(car_state, lateral_name, 0.0))
        detections.append(
            DetectedVehicle(
                label=label,
                longitudinal_m=longitudinal_m,
                lateral_m=side * lateral_mag,
                source="carState",
            )
        )
    return tuple(detections)


def parse_corner_radar_message(
    address: int,
    data: bytes,
    values: dict[str, float] | None = None,
) -> dict[str, DetectedVehicle]:
    if values is None:
        values = decode_hyundai_canfd_dbc_message(address, data)
    detections: dict[str, DetectedVehicle] = {}
    for label, forward_sign in (("LF", 1.0), ("RF", 1.0), ("LR", -1.0), ("RR", -1.0)):
        detect = values.get(f"{label}_DETECT", 0.0)
        distance_m = values.get(f"{label}_DETECT_DISTANCE", 0.0)
        if detect == 0 or not 0.2 < distance_m < 180.0:
            continue
        longitudinal_m = forward_sign * distance_m
        if forward_sign < 0.0:
            if not CORNER_RADAR_REAR_MIN_LONGITUDINAL_M <= longitudinal_m <= -0.2:
                continue
        elif not RADAR_MIN_LONGITUDINAL_M <= longitudinal_m <= RADAR_FRONT_MAX_LONGITUDINAL_M:
            continue
        lateral_mag = normalized_lateral_m(values.get(f"{label}_DETECT_LATERAL", 0.0))
        side = -1.0 if label.startswith("L") else 1.0
        detections[label] = DetectedVehicle(
            label=label,
            longitudinal_m=longitudinal_m,
            lateral_m=side * lateral_mag,
            source=f"CAN 0x{address:x}",
        )
    return detections


@cache
def hyundai_canfd_corner_dbc_signals() -> dict[int, dict[str, DbcSignalSpec]]:
    openpilot_root = find_openpilot_root_for_schema(Path(__file__).resolve().parent)
    dbc_path = find_opendbc_file(
        openpilot_root,
        "opendbc",
        "dbc",
        "generator",
        "hyundai",
        "hyundai_canfd.dbc",
    )
    signals: dict[int, dict[str, DbcSignalSpec]] = {address: {} for address in CORNER_RADAR_DBC_MESSAGES}
    if dbc_path is None:
        return signals
    current_address: int | None = None
    with open(dbc_path, encoding="utf-8") as dbc_file:
        for line in dbc_file:
            if line.startswith("BO_ "):
                parts = line.split()
                current_address = int(parts[1], 0) if len(parts) > 1 else None
                continue
            if current_address not in signals:
                continue
            match = DBC_SIGNAL_RE.match(line)
            if match is None:
                continue
            name, start, length, endian, sign, factor, offset = match.groups()
            if name not in CORNER_RADAR_DBC_SIGNAL_NAMES:
                continue
            signals[current_address][name] = DbcSignalSpec(
                start=int(start),
                length=int(length),
                byte_order="le" if endian == "1" else "be",
                signed=sign == "-",
                factor=float(factor),
                offset=float(offset),
            )
    return signals


def decode_hyundai_canfd_dbc_message(address: int, data: bytes) -> dict[str, float]:
    if address not in CORNER_RADAR_DBC_MESSAGES:
        return {}
    values: dict[str, float] = {}
    for name, signal in hyundai_canfd_corner_dbc_signals().get(address, {}).items():
        raw_value = dbc_unsigned(data, signal.start, signal.length, signal.byte_order)
        if signal.signed:
            raw_value -= ((raw_value >> (signal.length - 1)) & 0x1) * (1 << signal.length)
        values[name] = raw_value * signal.factor + signal.offset
    return values


def is_hyundai_camera_can_bus(bus: int) -> bool:
    return bus >= 0 and bus % 4 == HYUNDAI_CAMERA_CAN_BUS_MOD


def renderer_lateral_from_openpilot_yrel(y_rel: float) -> float:
    # openpilot radar/model UI projects radar points as -yRel; this renderer stores x as right-positive.
    return -y_rel


def live_track_to_radar_point(
    track: Any,
    index: int,
    ego_speed_kph: float,
    allow_legacy_corner_ids: bool = False,
) -> RadarPoint | None:
    d_rel = safe_optional_float(track, "dRel")
    if d_rel is None or not RADAR_MIN_LONGITUDINAL_M <= d_rel <= RADAR_FRONT_MAX_LONGITUDINAL_M:
        return None
    y_rel = safe_float(track, "yRel", 0.0)
    lateral_m = renderer_lateral_from_openpilot_yrel(y_rel)
    if radar_position_is_zero(d_rel, lateral_m):
        return None
    if not -12.0 <= lateral_m <= 12.0:
        return None
    track_id = safe_optional_int(track, "trackId")
    radar_source = safe_get(track, "radarSource", "frontRadar")
    is_corner_object = is_corner_radar_source(radar_source) or (
        allow_legacy_corner_ids
        and str(radar_source) == "frontRadar"
        and radar_track_id_is_corner_object(track_id)
    )
    label = (
        corner_track_label(track_id, str(radar_source))
        if is_corner_object
        else (f"T{track_id}" if track_id is not None else f"T{index:03d}")
    )
    rel_speed_mps = safe_optional_float(track, "vRel")
    lead_speed_mps = safe_optional_float(track, "vLead")
    lat_speed_mps = safe_optional_float(track, "yvRel")
    if lat_speed_mps is not None:
        lat_speed_mps = renderer_lateral_from_openpilot_yrel(lat_speed_mps)
    absolute_speed_kph = None
    if lead_speed_mps is not None:
        absolute_speed_kph = lead_speed_mps * 3.6
    elif rel_speed_mps is not None:
        absolute_speed_kph = ego_speed_kph + rel_speed_mps * 3.6
    measured = bool(safe_get(track, "measured", True))
    return RadarPoint(
        label=label,
        longitudinal_m=d_rel,
        lateral_m=lateral_m,
        source=CORNER_OBJECT_SOURCE if is_corner_object else "liveTracks",
        relative_speed_mps=rel_speed_mps,
        absolute_speed_kph=absolute_speed_kph,
        lateral_speed_mps=lat_speed_mps,
        relative_accel_mps2=safe_optional_float(track, "aRel"),
        probability=0.72 if measured else 0.38,
        valid=1 if measured else 0,
    )


def radar_track_id_is_corner_object(track_id: int | None) -> bool:
    return track_id is not None and (is_corner_track_id(track_id) or is_stable_corner_track_id(track_id))


def corner_track_label(track_id: int, radar_source: str = "") -> str:
    if is_stable_corner_track_id(track_id):
        group = "180" if radar_source == "corner180" else "430" if radar_source == "corner430" else "235"
        return f"CR{group}_T{track_id}"
    if CORNER_OBJECT_180_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_TRACK_COUNT:
        return f"CR180_{track_id - CORNER_OBJECT_180_TRACK_ID_OFFSET:02d}"
    if CORNER_OBJECT_430_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_430_TRACK_ID_OFFSET + CORNER_OBJECT_430_TRACK_COUNT:
        return f"CR430_{track_id - CORNER_OBJECT_430_TRACK_ID_OFFSET:03d}"
    return f"CR{track_id - CORNER_OBJECT_TRACK_ID_OFFSET:02d}"


def sorted_radar_points(points: Any) -> tuple[RadarPoint, ...]:
    filtered = [
        point
        for point in points
        if -12.0 <= point.lateral_m <= 12.0
        and RADAR_MIN_LONGITUDINAL_M <= point.longitudinal_m <= RADAR_FRONT_MAX_LONGITUDINAL_M
        and not radar_position_is_zero(point.longitudinal_m, point.lateral_m)
    ]
    filtered.sort(key=lambda point: (point.longitudinal_m, abs(point.lateral_m), point.label))
    return tuple(filtered)


def replay_corner_yaw_compensated_point(point: RadarPoint, yaw_rate_rps: float | None, gain: float) -> RadarPoint:
    if (
        gain == 0.0
        or not replay_corner_point_uses_display_centering(point)
        or yaw_rate_rps is None
        or not math.isfinite(yaw_rate_rps)
        or abs(yaw_rate_rps) >= CORNER_YAW_COMP_MAX_YAW_RATE
    ):
        return point

    yaw_d_rel = clamp(point.longitudinal_m, 0.0, CORNER_YAW_COMP_MAX_DREL)
    lateral_speed_corr = clamp(
        yaw_rate_rps * yaw_d_rel * gain,
        -CORNER_YAW_COMP_MAX_YVREL_CORRECTION,
        CORNER_YAW_COMP_MAX_YVREL_CORRECTION,
    )
    return replace(
        point,
        lateral_speed_mps=(
            point.lateral_speed_mps + lateral_speed_corr
            if point.lateral_speed_mps is not None
            else None
        ),
    )


def replay_corner_adjusted_point(
    point: RadarPoint,
    yaw_rate_rps: float | None,
    yaw_gain: float,
    lateral_offset_m: float,
    lane_width_m: float,
) -> RadarPoint:
    adjusted = replay_corner_yaw_compensated_point(point, yaw_rate_rps, yaw_gain)
    if not replay_corner_point_uses_display_centering(adjusted):
        return adjusted
    side_lane_min_m = max(
        CORNER_DISPLAY_CENTERING_SIDE_LANE_MIN_M,
        clamp(lane_width_m, 2.4, 4.6) * CORNER_DISPLAY_CENTERING_SIDE_LANE_WIDTH_RATIO,
    )
    if lateral_offset_m == 0.0 or abs(adjusted.lateral_m) < side_lane_min_m:
        return adjusted
    return replace(adjusted, lateral_m=adjusted.lateral_m - math.copysign(abs(lateral_offset_m), adjusted.lateral_m))


def replay_corner_point_uses_display_centering(point: RadarPoint) -> bool:
    return (
        point.source == CORNER_OBJECT_SOURCE
        and (point.label.startswith("CO") or point.label.startswith("CR"))
    )


def normalized_lateral_m(value: float) -> float:
    if 0.4 <= value <= 6.0:
        return value
    return 3.0


def scene_steering_from_curvature(curvature_m_inv: float) -> float:
    return clamp(curvature_m_inv / (2.0 * ROAD_CURVE_M_PER_M2), -1.0, 1.0)


def dbc_unsigned(data: bytes, start: int, length: int, byte_order: str) -> int:
    if byte_order == "le":
        return (int.from_bytes(data, "little") >> start) & ((1 << length) - 1)
    value = 0
    bit = start
    for _ in range(length):
        if bit < 0 or bit // 8 >= len(data):
            return value
        value = (value << 1) | ((data[bit // 8] >> (bit % 8)) & 1)
        bit = bit + 15 if bit % 8 == 0 else bit - 1
    return value


def dbc_signed(data: bytes, start: int, length: int, byte_order: str) -> int:
    value = dbc_unsigned(data, start, length, byte_order)
    sign_bit = 1 << (length - 1)
    return value - (1 << length) if value & sign_bit else value


def decode_raw_corner_objects(t: float, address: int, data: bytes) -> tuple[RawCornerObject, ...]:
    if len(data) != 32:
        return ()
    if RAW_CORNER_235_START_ADDR <= address <= RAW_CORNER_235_END_ADDR:
        return (
            decode_raw_corner_object_at(
                t,
                "235",
                address,
                address - RAW_CORNER_235_START_ADDR,
                data,
                24,
            ),
        )
    if RAW_CORNER_180_START_ADDR <= address <= RAW_CORNER_180_END_ADDR:
        base_slot = (address - RAW_CORNER_180_START_ADDR) * 2
        return (
            decode_raw_corner_object_at(t, "180", address, base_slot, data, 24),
            decode_raw_corner_object_at(t, "180", address, base_slot + 1, data, 152),
        )
    return ()


def decode_raw_corner_object_at(
    t: float,
    group: str,
    address: int,
    slot: int,
    data: bytes,
    base: int,
) -> RawCornerObject:
    width_bits = 8 if group == "235" else 7
    width_factor = 0.01 if group == "235" else 0.05
    class_bits = 4 if group == "235" else 3
    return RawCornerObject(
        t=t,
        group=group,
        address=address,
        slot=slot,
        quality=dbc_unsigned(data, base + 0, 7, "le"),
        age=dbc_unsigned(data, base + 8, 8, "le"),
        object_id=dbc_unsigned(data, base + 20, 7, "le"),
        object_class=dbc_unsigned(data, base + 36, class_bits, "le"),
        width=dbc_unsigned(data, base + 28, width_bits, "le") * width_factor,
        x=dbc_unsigned(data, base + 40, 13, "le") * 0.05,
        y=dbc_unsigned(data, base + 54, 12, "le") * 0.05 - 102.4,
        vx=dbc_unsigned(data, base + 67, 12, "le") * 0.05 - 100.0,
        vy=dbc_unsigned(data, base + 80, 10, "le") * 0.05 - 25.0,
        ax=dbc_signed(data, base + 91, 9, "le") * 0.05,
    )


def raw_corner_object_is_valid(obj: RawCornerObject) -> bool:
    if obj.quality < 1:
        return False
    if not 0.0 <= obj.x <= RAW_CORNER_OBJECT_MAX_X_M:
        return False
    if obj.x <= 0.2 and not is_side_corner_object(obj.x, obj.y):
        return False
    if abs(obj.y) > RAW_CORNER_OBJECT_MAX_ABS_Y_M:
        return False
    return not (obj.vx <= -99.0 and obj.x < 0.5)


def raw_corner_object_to_radar_point(obj: RawCornerObject, ego_speed_kph: float) -> RadarPoint:
    group_prefix = "R" if obj.group == "235" else "R180_"
    return RadarPoint(
        label=f"{group_prefix}{obj.slot:02d}",
        longitudinal_m=obj.x,
        lateral_m=renderer_lateral_from_openpilot_yrel(obj.y),
        source=CORNER_OBJECT_SOURCE,
        relative_speed_mps=obj.vx,
        absolute_speed_kph=ego_speed_kph + obj.vx * 3.6,
        lateral_speed_mps=renderer_lateral_from_openpilot_yrel(obj.vy),
        relative_accel_mps2=obj.ax,
        probability=clamp(0.35 + min(obj.quality, 80) / 100.0, 0.35, 0.92),
        valid=1,
        valid_count=obj.age,
    )


@dataclass(frozen=True, slots=True)
class ReconstructedLiveTracks:
    points: tuple[Any, ...]


def merge_recorded_and_reconstructed_tracks(
    recorded: tuple[Any, ...],
    reconstructed: tuple[ReconstructedLiveTrack, ...],
    prefer_reconstructed_corner: bool = False,
    raw_corner_only: bool = False,
) -> tuple[Any, ...]:
    recorded_groups: set[str] = set()
    for point in recorded:
        source = str(safe_get(point, "radarSource", "frontRadar"))
        track_id = int(safe_get(point, "trackId", -1))
        if source == "corner235" or CORNER_OBJECT_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_TRACK_ID_OFFSET + CORNER_OBJECT_TRACK_COUNT:
            recorded_groups.add("corner235")
        if source == "corner180" or CORNER_OBJECT_180_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_TRACK_COUNT:
            recorded_groups.add("corner180")
    if raw_corner_only:
        recorded = tuple(
            point for point in recorded
            if not point_is_corner_group(point, "corner235") and not point_is_corner_group(point, "corner180")
        )
        recorded_groups.clear()
    elif prefer_reconstructed_corner:
        reconstructed_groups = {point.radarSource for point in reconstructed}
        recorded = tuple(
            point for point in recorded
            if not (
                ("corner235" in reconstructed_groups and point_is_corner_group(point, "corner235"))
                or ("corner180" in reconstructed_groups and point_is_corner_group(point, "corner180"))
            )
        )
        recorded_groups -= reconstructed_groups
    added = tuple(point for point in reconstructed if point.radarSource not in recorded_groups)
    return recorded + added


def point_is_corner_group(point: Any, group: str) -> bool:
    source = str(safe_get(point, "radarSource", "frontRadar"))
    track_id = int(safe_get(point, "trackId", -1))
    if group == "corner235":
        return source == group or CORNER_OBJECT_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_TRACK_ID_OFFSET + CORNER_OBJECT_TRACK_COUNT
    return source == group or CORNER_OBJECT_180_TRACK_ID_OFFSET <= track_id < CORNER_OBJECT_180_TRACK_ID_OFFSET + CORNER_OBJECT_180_TRACK_COUNT


def route_corner_source_or_default(source: str | None) -> str:
    return source if source in ROUTE_CORNER_SOURCE_CHOICES else ROUTE_CORNER_SOURCE_LIVE


def has_nearby_vehicle(
    vehicles: list[DetectedVehicle],
    candidate: DetectedVehicle,
    longitudinal_tolerance: float,
    lateral_tolerance: float,
) -> bool:
    return any(
        abs(vehicle.longitudinal_m - candidate.longitudinal_m) <= longitudinal_tolerance
        and abs(vehicle.lateral_m - candidate.lateral_m) <= lateral_tolerance
        for vehicle in vehicles
    )


def road_edge_lateral_bounds_m(lane_values: dict[str, Any]) -> tuple[float | None, float | None]:
    center_m = lane_values.get("center")
    if center_m is None:
        return None, None
    width_m = max(0.1, float(lane_values.get("width", DEFAULT_LANE_WIDTH_M)))

    def edge_lateral(offset_name: str) -> float | None:
        offset = lane_values.get(offset_name)
        if offset is None:
            return None
        return float(center_m) + float(offset) * width_m

    left_edge_m = edge_lateral("left_road_edge_offset")
    right_edge_m = edge_lateral("right_road_edge_offset")
    if left_edge_m is not None and right_edge_m is not None and left_edge_m >= right_edge_m:
        return None, None
    return left_edge_m, right_edge_m


def vehicle_is_inside_road_edges(vehicle: DetectedVehicle, lane_values: dict[str, Any]) -> bool:
    left_edge_m, right_edge_m = road_edge_lateral_bounds_m(lane_values)
    if left_edge_m is not None and vehicle.lateral_m < left_edge_m - ROAD_EDGE_VEHICLE_OUTSIDE_MARGIN_M:
        return False
    if right_edge_m is not None and vehicle.lateral_m > right_edge_m + ROAD_EDGE_VEHICLE_OUTSIDE_MARGIN_M:
        return False
    return True


def vehicle_is_blocked_by_near_road_edge(vehicle: DetectedVehicle, lane_values: dict[str, Any]) -> bool:
    return (
        abs(vehicle.longitudinal_m) <= NEAR_ROAD_EDGE_VEHICLE_BLOCK_DISTANCE_M
        and not vehicle_is_inside_road_edges(vehicle, lane_values)
    )


def radar_vehicle_has_meaningful_motion(vehicle: DetectedVehicle) -> bool:
    return (
        vehicle.source == "radarState"
        and vehicle.absolute_speed_kph is not None
        and abs(vehicle.absolute_speed_kph) >= RADAR_MOVING_VEHICLE_MIN_SPEED_KPH
    )


def vehicle_is_confirmed_corner_radar(vehicle: DetectedVehicle) -> bool:
    if vehicle.label not in ("LF", "RF", "LR", "RR"):
        return False
    source = vehicle.source.lower()
    return source == "carstate" or source in ("can 0x162", "can 0x1ea")


def detected_vehicle_summary(vehicles: tuple[DetectedVehicle, ...]) -> str:
    if not vehicles:
        return "none"
    parts = []
    for vehicle in vehicles[:4]:
        rel = "" if vehicle.relative_speed_mps is None else f" {vehicle.relative_speed_mps:+.1f}mps"
        prob = "" if vehicle.probability >= 0.995 else f" p{vehicle.probability:.0%}"
        cut_in = " cut" if vehicle.cut_in else ""
        parts.append(f"{vehicle.label} {vehicle.longitudinal_m:+.0f}/{vehicle.lateral_m:+.1f}{rel}{prob}{cut_in}")
    if len(vehicles) > 4:
        parts.append(f"+{len(vehicles) - 4}")
    return " ".join(parts)


def nearest_ttc_summary(vehicles: tuple[DetectedVehicle, ...]) -> str:
    ttcs = [vehicle.ttc_s for vehicle in vehicles if vehicle.ttc_s is not None]
    if not ttcs:
        return "--"
    return f"{min(ttcs):.1f}s"


def radar_point_summary(points: tuple[RadarPoint, ...]) -> str:
    if not points:
        return "none"
    nearest = min(points, key=lambda point: max(0.0, point.longitudinal_m))
    rel = "" if nearest.relative_speed_mps is None else f" v{nearest.relative_speed_mps:+.1f}"
    prob = "" if nearest.probability is None else f" p{nearest.probability:.0%}"
    return f"{len(points)} nearest {nearest.label} {nearest.longitudinal_m:.0f}/{nearest.lateral_m:+.1f}{rel}{prob}"


def discover_route_logs(
    route_path: Path,
    log_kind: str,
    start_segment: int | None,
    max_segments: int | None,
) -> list[Path]:
    if log_kind not in LOG_FILENAMES:
        raise RuntimeError(f"unsupported route log kind: {log_kind}")

    route_path = route_path.resolve()
    filename = LOG_FILENAMES[log_kind]
    if route_path.is_file():
        return [route_path]

    search_root, effective_start_segment, route_id_filter = route_search_spec(route_path, start_segment)
    if not search_root.exists():
        raise RuntimeError(f"route path does not exist: {route_path}")

    files = sorted(search_root.rglob(filename), key=route_sort_key)
    if route_id_filter is not None:
        files = [
            path
            for path in files
            if segment_route_id(path) == route_id_filter
        ]
    if effective_start_segment is not None:
        files = [
            path
            for path in files
            if segment_index(path) is not None and segment_index(path) >= effective_start_segment
        ]
    if max_segments is not None:
        files = files[:max_segments]
    return files


def adjacent_route_log_path(
    log_path: Path,
    direction: int,
    log_kind: str = "rlog",
) -> Path | None:
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    if log_kind not in LOG_FILENAMES:
        raise RuntimeError(f"unsupported route log kind: {log_kind}")

    folder = log_path.resolve().parent if log_path.is_file() else log_path.resolve()
    match = NUMBERED_FOLDER_RE.fullmatch(folder.name)
    if match is None:
        return None

    prefix, digits = match.groups()
    next_index = int(digits) + direction
    if next_index < 0:
        return None
    next_name = f"{prefix}{next_index:0{len(digits)}d}"
    next_folder = folder.parent / next_name
    filenames = (LOG_FILENAMES[log_kind], f"{log_kind}.bz2")
    for filename in filenames:
        candidate = next_folder / filename
        if candidate.is_file():
            return candidate
    return None


def route_search_spec(route_path: Path, start_segment: int | None) -> tuple[Path, int | None, str | None]:
    segment = segment_index_from_name(route_path.name)
    route_id = segment_route_id_from_name(route_path.name)
    if route_path.exists():
        if segment is None:
            return route_path, start_segment, None
        return route_path.parent, start_segment if start_segment is not None else segment, route_id
    if segment is not None:
        return route_path.parent, start_segment if start_segment is not None else segment, route_id
    return route_path.parent, start_segment, route_path.name


def route_sort_key(path: Path) -> tuple[str, int, str]:
    parent = path.parent.name
    route = segment_route_id(path) or path.parent.parent.name
    index = segment_index(path)
    return route, index if index is not None else 10**9, parent


def route_video_segment_at(
    segments: list[RouteVideoSegment],
    playback_seconds: float,
) -> RouteVideoSegment | None:
    if not segments:
        return None
    starts = [segment.start_t for segment in segments]
    index = bisect_right(starts, playback_seconds) - 1
    if index < 0:
        return segments[0]
    segment = segments[min(index, len(segments) - 1)]
    if playback_seconds <= segment.end_t + 0.5:
        return segment
    if index + 1 < len(segments):
        return segments[index + 1]
    return segment


def segment_index(path: Path) -> int | None:
    return segment_index_from_name(path.parent.name)


def segment_route_id(path: Path) -> str | None:
    return segment_route_id_from_name(path.parent.name)


def segment_route_id_from_name(name: str) -> str | None:
    if segment_index_from_name(name) is None:
        return None
    return name.rsplit("--", 1)[0]


def segment_index_from_name(name: str) -> int | None:
    try:
        suffix = name.rsplit("--", 1)[1]
    except (IndexError, ValueError):
        return None
    return int(suffix) if suffix.isdigit() else None


_LOG_SCHEMA: Any | None = None


def load_openpilot_log_schema() -> Any:
    global _LOG_SCHEMA
    if _LOG_SCHEMA is not None:
        return _LOG_SCHEMA

    if sys.platform != "win32":
        try:
            from openpilot.cereal import log as capnp_log

            _LOG_SCHEMA = capnp_log
            return _LOG_SCHEMA
        except Exception:
            pass

    try:
        import capnp
    except ModuleNotFoundError as exc:
        raise RuntimeError("pycapnp is required to read openpilot route logs") from exc

    schema_dir = prepare_schema_copy()
    _LOG_SCHEMA = capnp.load(str(schema_dir / "log.capnp"), imports=[str(schema_dir)])
    return _LOG_SCHEMA


def prepare_schema_copy() -> Path:
    openpilot_root = find_openpilot_root_for_schema(Path(__file__).resolve().parent)
    cereal_root = openpilot_root / "cereal"
    car_schema = find_opendbc_file(openpilot_root, "opendbc", "car", "car.capnp")
    if car_schema is None:
        raise RuntimeError(f"openpilot car schema not found near: {openpilot_root}")

    schema_dir = Path(tempfile.gettempdir()) / ROUTE_SCHEMA_CACHE_NAME
    include_dir = schema_dir / "include"
    include_dir.mkdir(parents=True, exist_ok=True)
    for name in ("log.capnp", "custom.capnp", "deprecated.capnp"):
        shutil.copyfile(cereal_root / name, schema_dir / name)
    shutil.copyfile(car_schema, schema_dir / "car.capnp")
    shutil.copyfile(cereal_root / "include" / "c++.capnp", include_dir / "c++.capnp")
    return schema_dir


def find_openpilot_root_for_schema(start: Path) -> Path:
    for path in (start, *start.parents):
        if (path / "cereal").exists():
            return path
        nested = path / "openpilot"
        if (nested / "cereal").exists():
            return nested
    return start / "openpilot"


def find_opendbc_file(openpilot_root: Path, *relative_parts: str) -> Path | None:
    for root in (openpilot_root, openpilot_root.parent, *openpilot_root.parents):
        candidate = root / "opendbc_repo" / Path(*relative_parts)
        if candidate.exists():
            return candidate
    return None


def read_log_bytes(path: Path) -> bytes:
    data = path.read_bytes()
    if path.suffix == ".bz2" or data.startswith(b"BZh"):
        return bz2.decompress(data)
    if path.suffix == ".zst" or data.startswith(b"\x28\xb5\x2f\xfd"):
        try:
            import zstandard as zstd
        except ModuleNotFoundError as exc:
            raise RuntimeError("zstandard is required to read compressed route logs") from exc
        with zstd.ZstdDecompressor().stream_reader(io.BytesIO(data)) as reader:
            return reader.read()
    return data


def safe_which(event: Any) -> str | None:
    try:
        return event.which()
    except Exception:
        return None


def safe_get(obj: Any, name: str, default: Any = None) -> Any:
    try:
        return getattr(obj, name)
    except Exception:
        return default


def safe_enum_int(value: Any, names: tuple[str, ...], default: int = 0) -> int:
    raw = safe_get(value, "raw", value)
    try:
        parsed = int(raw)
    except (TypeError, ValueError):
        normalized = str(raw or "").replace("_", "").lower()
        try:
            parsed = names.index(normalized)
        except ValueError:
            return default
    return parsed if 0 <= parsed < len(names) else default


def safe_float(obj: Any, name: str, default: float) -> float:
    value = safe_get(obj, name, default)
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(value):
        return default
    return value


def finite_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def safe_optional_float(obj: Any, name: str) -> float | None:
    value = safe_float(obj, name, math.nan)
    return None if math.isnan(value) else value


def tpms_info_from_car_state(car_state: Any) -> TpmsInfo:
    tpms = safe_get(car_state, "tpms")

    def pressure(name: str) -> float | None:
        value = safe_optional_float(tpms, name) if tpms is not None else None
        return value if value is not None and 5.0 <= value <= 60.0 else None

    return TpmsInfo(
        fl=pressure("fl"),
        fr=pressure("fr"),
        rl=pressure("rl"),
        rr=pressure("rr"),
    )


def safe_optional_int(obj: Any, name: str) -> int | None:
    value = safe_get(obj, name)
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def list_value(values: Any, index: int) -> float | None:
    if values is None or index < 0 or index >= len(values):
        return None
    return finite_float(values[index])


def first_list_value(values: Any) -> float | None:
    if values is None or len(values) == 0:
        return None
    try:
        return float(values[0])
    except (TypeError, ValueError):
        return None


def enum_text(value: Any) -> str:
    return str(value)


def numeric_tuple(
    values: Any,
    limit: int | None = None,
    minimum: float | None = None,
    maximum: float | None = None,
) -> tuple[float, ...]:
    if values is None:
        return ()
    parsed: list[float] = []
    for index, value in enumerate(values):
        if limit is not None and index >= limit:
            break
        number = finite_float(value)
        if number is None:
            continue
        if minimum is not None:
            number = max(minimum, number)
        if maximum is not None:
            number = min(maximum, number)
        parsed.append(number)
    return tuple(parsed)


def three_float_tuple(values: Any) -> tuple[float, float, float] | None:
    parsed = numeric_tuple(values, limit=3)
    if len(parsed) < 3:
        return None
    return parsed[0], parsed[1], parsed[2]


def desire_prediction_matrix(values: Any) -> tuple[tuple[float, ...], ...]:
    flat = numeric_tuple(values, limit=32, minimum=0.0, maximum=1.0)
    if len(flat) < 8:
        return ()
    rows: list[tuple[float, ...]] = []
    for start in range(0, len(flat), 8):
        row = flat[start : start + 8]
        if len(row) == 8:
            rows.append(row)
    return tuple(rows[:4])


def risk_points_from_meta(meta: Any) -> tuple[ModelRiskPoint, ...]:
    predictions = safe_get(meta, "disengagePredictions")
    if predictions is None:
        return ()
    times = numeric_tuple(safe_get(predictions, "t"), limit=8, minimum=0.0, maximum=30.0)
    if not times:
        times = (2.0, 4.0, 6.0, 8.0, 10.0)
    fields = {
        "brake_disengage": numeric_tuple(safe_get(predictions, "brakeDisengageProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "gas_disengage": numeric_tuple(safe_get(predictions, "gasDisengageProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "steer_override": numeric_tuple(safe_get(predictions, "steerOverrideProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "hard_brake_3": numeric_tuple(safe_get(predictions, "brake3MetersPerSecondSquaredProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "hard_brake_4": numeric_tuple(safe_get(predictions, "brake4MetersPerSecondSquaredProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "hard_brake_5": numeric_tuple(safe_get(predictions, "brake5MetersPerSecondSquaredProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "gas_press": numeric_tuple(safe_get(predictions, "gasPressProbs"), limit=len(times), minimum=0.0, maximum=1.0),
        "brake_press": numeric_tuple(safe_get(predictions, "brakePressProbs"), limit=len(times), minimum=0.0, maximum=1.0),
    }
    points: list[ModelRiskPoint] = []
    for index, t_s in enumerate(times):
        points.append(
            ModelRiskPoint(
                t_s=t_s,
                brake_disengage=tuple_value(fields["brake_disengage"], index),
                gas_disengage=tuple_value(fields["gas_disengage"], index),
                steer_override=tuple_value(fields["steer_override"], index),
                hard_brake_3=tuple_value(fields["hard_brake_3"], index),
                hard_brake_4=tuple_value(fields["hard_brake_4"], index),
                hard_brake_5=tuple_value(fields["hard_brake_5"], index),
                gas_press=tuple_value(fields["gas_press"], index),
                brake_press=tuple_value(fields["brake_press"], index),
            )
        )
    return tuple(points)


def tuple_value(values: tuple[float, ...], index: int) -> float:
    return values[index] if 0 <= index < len(values) else 0.0


def lane_style_from_code(code: int) -> str:
    if code < 0:
        return "solid"
    return "dashed" if code % 10 == 0 else "solid"


def lane_color_from_code(code: int) -> tuple[int, int, int] | None:
    if code < 10:
        return None
    color_code = code // 10
    if color_code == 1:
        return WHITE
    if color_code == 2:
        return YELLOW
    return None
