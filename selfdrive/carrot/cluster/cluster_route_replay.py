from __future__ import annotations

import bz2
import io
import math
import multiprocessing
import os
import queue
import shutil
import subprocess
import sys
import tempfile
import threading
import traceback
from bisect import bisect_right
from dataclasses import dataclass, replace
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
    ROAD_CURVE_M_PER_M2,
    WHITE,
)
from cluster_models import (
    ClusterUiState,
    CruiseDisplayState,
    DetectedVehicle,
    LaneMarking,
    ModelPathPoint,
    ModelRiskPoint,
    RadarPoint,
    RouteOverlay,
)
from cluster_utils import clamp, smoothstep


ROUTE_SCHEMA_CACHE_NAME = "carrotpilot_cluster_capnp_v1"
LOG_FILENAMES = {
    "qlog": "qlog.zst",
    "rlog": "rlog.zst",
}
RADAR_TO_CAMERA_M = 1.52
MODEL_LEAD_MIN_PROB = 0.08
RADAR_POINT_STALE_S = 0.12
CORNER_DETECTION_STALE_S = 0.8
RADAR_MIN_LONGITUDINAL_M = 0.0
RADAR_FRONT_MAX_LONGITUDINAL_M = 180.0
CORNER_RADAR_REAR_MIN_LONGITUDINAL_M = -180.0
CCNC_CORNER_RADAR_ADDRESS = 0x162
ADRV_CORNER_RADAR_ADDRESS = 0x1EA
ROUTE_REPLAY_MIN_BUFFER_FILES = 2
ROUTE_REPLAY_START_BUFFER_FILES = 1
ROUTE_REPLAY_READAHEAD_S = 5.0
ROUTE_REPLAY_RETAIN_BEHIND_S = 1.0
ROUTE_REPLAY_PRELOAD_NICE = 10
ROUTE_VIDEO_FPS = 20.0
ROUTE_VIDEO_DECODE_WIDTH = 388
ROUTE_VIDEO_DECODE_HEIGHT = 244
ROUTE_VIDEO_SEEK_RESTART_FRAMES = 45
NAV_SPEED_LIMIT_HOLD_SECONDS = 10.0
ROAD_EDGE_VEHICLE_OUTSIDE_MARGIN_M = 0.25
LANE_CHANGE_REINDEX_PEAK_THRESHOLD = 0.22
LANE_CHANGE_REINDEX_RESET_THRESHOLD = -0.08
CONTINUOUS_LANE_CHANGE_REBASE_PROGRESS = 0.12
LANE_CHANGE_MODEL_DIRECT_ONLY = True
MODEL_DIRECT_LANE_SETTLE_MIN_PROGRESS = 0.65
LONGITUDINAL_PERSONALITY_GAPS = {
    "aggressive": 1,
    "standard": 2,
    "relaxed": 3,
    "morerelaxed": 4,
}


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
    display_speed_kph: float | None = None
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
    def __init__(self) -> None:
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
                    args=(self._requests, self._results, True),
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
            args=(self._requests, self._results, False),
            name="route-log-preload",
            daemon=True,
        )
        self._worker.start()


def route_log_preload_worker(requests: Any, results: Any, low_priority: bool = False) -> None:
    if low_priority:
        try:
            os.nice(ROUTE_REPLAY_PRELOAD_NICE)
        except OSError:
            pass
    log_schema = load_openpilot_log_schema()
    parser = RouteLogParser()
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
    ) -> None:
        if not source_files:
            raise RuntimeError("route contains no log files")
        self.source_files = source_files
        self.frames: list[RouteReplayFrame] = []
        self.times: list[float] = []
        self.duration = 0.0
        self.video_segments: list[RouteVideoSegment] = []
        self._video_reader = RouteVideoFrameReader(self.video_segments)
        self._preload_worker = RouteLogPreloadWorker()
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
    ) -> RouteReplaySource:
        files = discover_route_logs(route_path, log_kind, start_segment, max_segments)
        if not files:
            raise RuntimeError(f"no {LOG_FILENAMES[log_kind]} files found under {route_path}")

        return cls(files)

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
            state = frame_to_state(self.frames[0])
            return self._with_overlay(state, self.frames[0], 0.0, loop) if include_overlay else state
        if not loop or self._end_of_route:
            playback_seconds = clamp(playback_seconds, 0.0, self.duration)

        right_index = bisect_right(self.times, playback_seconds)
        if right_index <= 0:
            state = frame_to_state(self.frames[0])
            return self._with_overlay(state, self.frames[0], playback_seconds, loop) if include_overlay else state
        if right_index >= len(self.frames):
            state = frame_to_state(self.frames[-1])
            return self._with_overlay(state, self.frames[-1], playback_seconds, loop) if include_overlay else state

        left = self.frames[right_index - 1]
        right = self.frames[right_index]
        span = max(0.001, right.t - left.t)
        amount = clamp((playback_seconds - left.t) / span, 0.0, 1.0)
        frame = blend_frames(left, right, amount)
        state = frame_to_state(frame)
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
        )

    @property
    def loaded_file_count(self) -> int:
        return self._loaded_file_count

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

        while not self._end_of_route and (
            not self.frames
            or len(self._loaded_chunks) < ROUTE_REPLAY_START_BUFFER_FILES
            or playback_seconds >= self.duration - ROUTE_REPLAY_READAHEAD_S
        ):
            if not self._load_next_file():
                break

        self._trim_loaded_chunks(playback_seconds)
        self._start_preload()

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
            return RouteOverlay(video_status=status, data_lines=data_lines)
        return RouteOverlay(
            video_rgba=video_frame.rgba,
            video_width=video_frame.width,
            video_height=video_frame.height,
            video_frame_id=video_frame.frame_id,
            data_lines=data_lines,
        )


class RouteVideoFrameReader:
    def __init__(self, segments: list[RouteVideoSegment]) -> None:
        self.segments = segments
        self._ffmpeg = shutil.which("ffmpeg")
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
            "-ss",
            f"{seek_s:.3f}",
            "-i",
            str(segment.path),
            "-an",
            "-sn",
            "-vf",
            f"scale={ROUTE_VIDEO_DECODE_WIDTH}:{ROUTE_VIDEO_DECODE_HEIGHT}",
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
    def __init__(self) -> None:
        self.speed_limit_kph: int | None = None
        self.speed_limit_source: str | None = None
        self.nav_speed_limit_kph: int | None = None
        self.nav_speed_limit_t = -999.0
        self.cruise_kph: int | None = None
        self.cruise_gap: int | None = None
        self.lfa_active: bool | None = None
        self.controls_enabled: bool | None = None
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
        self.hyundai_canfd_radar_points: dict[str, RadarPoint] = {}
        self.hyundai_canfd_radar_history: dict[str, tuple[float, float]] = {}
        self.hyundai_canfd_radar_t = -999.0
        self.live_track_radar_points: dict[str, RadarPoint] = {}
        self.live_track_radar_t = -999.0
        self.radar_detections: tuple[DetectedVehicle, ...] = ()
        self.radar_detection_t = -999.0
        self.current_speed_kph = 0.0
        self.v_ego_cluster_seen = False

    def parse_file(self, file_path: Path, log_schema: Any) -> list[RouteReplayFrame]:
        frames: list[RouteReplayFrame] = []
        data = read_log_bytes(file_path)
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
                self._update_longitudinal_plan(event.longitudinalPlan)
            elif event_type == "controlsState":
                self._update_controls_state(event.controlsState)
            elif event_type == "selfdriveState":
                self._update_selfdrive_state(event.selfdriveState)
            elif event_type == "carControl":
                self._update_car_control(event.carControl)
            elif event_type == "cameraOdometry":
                self._update_camera_odometry(event.cameraOdometry, bool(safe_get(event, "valid", True)))
            elif event_type == "radarState":
                self._update_radar_state(event.radarState, event_t)
            elif event_type == "liveTracks":
                self._update_live_tracks(event.liveTracks, event_t)
            elif event_type in ("can", "sendcan"):
                self._update_can_detections(getattr(event, event_type), event_t)

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
        radar_points = self._radar_points_from_current_state(event_t)

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
            display_speed_kph=display_speed_kph,
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

    def _update_longitudinal_plan(self, longitudinal_plan: Any) -> None:
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
        self.camera_calibration_euler = three_float_tuple(safe_get(camera_odometry, "wideFromDeviceEuler"))
        self.road_transform_trans = three_float_tuple(safe_get(camera_odometry, "roadTransformTrans"))
        self.road_transform_std = three_float_tuple(safe_get(camera_odometry, "roadTransformTransStd"))

    def _update_controls_state(self, controls_state: Any) -> None:
        enabled = safe_get(controls_state, "enabled", None)
        if enabled is not None:
            self.controls_enabled = bool(enabled)

        desired_curvature = safe_optional_float(controls_state, "desiredCurvature")
        if desired_curvature is not None and abs(desired_curvature) < 0.05:
            self.controls_curvature_m_inv = desired_curvature
            self.controls_curvature_source = "controlsState.desired"
            return
        curvature = safe_optional_float(controls_state, "curvature")
        if curvature is not None and abs(curvature) < 0.05:
            self.controls_curvature_m_inv = curvature
            self.controls_curvature_source = "controlsState"

    def _update_selfdrive_state(self, selfdrive_state: Any) -> None:
        cruise_gap = self._cruise_gap_from_personality(safe_get(selfdrive_state, "personality"))
        if cruise_gap is not None:
            self.cruise_gap = cruise_gap

    def _update_car_control(self, car_control: Any) -> None:
        lat_active = safe_get(car_control, "latActive", None)
        if lat_active is not None:
            self.lfa_active = bool(lat_active)

    def _update_radar_state(self, radar_state: Any, event_t: float) -> None:
        detections: list[DetectedVehicle] = []
        for label, lead_name in (("TARGET", "leadOne"), ("TARGET2", "leadTwo")):
            lead = safe_get(radar_state, lead_name)
            if lead is None or not bool(safe_get(lead, "status", False)):
                continue
            d_rel = safe_float(lead, "dRel", 0.0)
            if not RADAR_MIN_LONGITUDINAL_M <= d_rel <= RADAR_FRONT_MAX_LONGITUDINAL_M:
                continue
            # openpilot yRel is left-positive; this renderer uses right-positive x.
            lateral_m = -safe_float(lead, "yRel", 0.0)
            relative_speed_mps = safe_optional_float(lead, "vRel")
            lead_speed_mps = safe_optional_float(lead, "vLead")
            absolute_speed_kph = (
                max(0.0, lead_speed_mps * 3.6)
                if lead_speed_mps is not None
                else (
                    max(0.0, self.current_speed_kph + relative_speed_mps * 3.6)
                    if relative_speed_mps is not None
                    else None
                )
            )
            detections.append(
                DetectedVehicle(
                    label=label,
                    longitudinal_m=d_rel,
                    lateral_m=clamp(lateral_m, -8.0, 8.0),
                    source="radarState",
                    relative_speed_mps=relative_speed_mps,
                    absolute_speed_kph=absolute_speed_kph,
                    acceleration_mps2=safe_optional_float(lead, "aLeadK"),
                    ttc_s=ttc_from_relative_speed(d_rel, relative_speed_mps),
                )
            )
        self.radar_detections = tuple(detections)
        self.radar_detection_t = event_t

    def _update_can_detections(self, can_messages: Any, event_t: float) -> None:
        for can_message in can_messages:
            address = int(safe_get(can_message, "address", -1))
            data = bytes(safe_get(can_message, "dat", b""))
            if is_hyundai_canfd_radar_address(address):
                labels = hyundai_canfd_radar_labels_for_address(address)
                radar_points = parse_hyundai_canfd_radar_message(address, data)
                valid_labels = {point.label for point in radar_points}
                for label in labels:
                    self.hyundai_canfd_radar_points.pop(label, None)
                    if label not in valid_labels:
                        self.hyundai_canfd_radar_history.pop(label, None)
                for point in radar_points:
                    self.hyundai_canfd_radar_points[point.label] = self._radar_point_with_absolute_speed(point, event_t)
                self.hyundai_canfd_radar_t = event_t
                continue
            if address not in (CCNC_CORNER_RADAR_ADDRESS, ADRV_CORNER_RADAR_ADDRESS):
                continue
            if len(data) < 24:
                continue
            parsed = parse_corner_radar_message(address, data)
            if address == ADRV_CORNER_RADAR_ADDRESS:
                self.adrv_corner_detections = parsed
                self.adrv_corner_message_t = event_t
                self.adrv_lane_changing = dbc_unsigned(data, 45, 3, "be")
                self.adrv_lane_changing_t = event_t
            else:
                self.ccnc_corner_detections = parsed
                self.ccnc_corner_message_t = event_t

    def _update_live_tracks(self, live_tracks: Any, event_t: float) -> None:
        points: dict[str, RadarPoint] = {}
        tracks = safe_get(live_tracks, "points", ())
        if tracks is None:
            tracks = ()
        for index, track in enumerate(tracks):
            point = live_track_to_radar_point(track, index, self.current_speed_kph)
            if point is not None:
                points[point.label] = point
        self.live_track_radar_points = points
        self.live_track_radar_t = event_t

    def _radar_points_from_current_state(self, event_t: float) -> tuple[RadarPoint, ...]:
        points: list[RadarPoint] = []
        if event_t - self.hyundai_canfd_radar_t < RADAR_POINT_STALE_S:
            points.extend(self.hyundai_canfd_radar_points.values())
        elif event_t - self.live_track_radar_t < RADAR_POINT_STALE_S:
            points.extend(self.live_track_radar_points.values())
        return sorted_radar_points(points)

    def _radar_point_with_absolute_speed(self, point: RadarPoint, event_t: float) -> RadarPoint:
        signal_speed_kph = (
            None
            if point.relative_speed_mps is None
            else max(0.0, self.current_speed_kph + point.relative_speed_mps * 3.6)
        )
        observed_speed_kph = None
        previous = self.hyundai_canfd_radar_history.get(point.label)
        if previous is not None:
            previous_distance_m, previous_t = previous
            dt = event_t - previous_t
            if 0.02 <= dt <= 0.45:
                observed_relative_mps = (point.longitudinal_m - previous_distance_m) / dt
                observed_speed_kph = max(0.0, self.current_speed_kph + observed_relative_mps * 3.6)
                if observed_speed_kph > MAX_SPEED_KPH * 1.8:
                    observed_speed_kph = None
        self.hyundai_canfd_radar_history[point.label] = (point.longitudinal_m, event_t)
        absolute_speed_kph = observed_speed_kph if observed_speed_kph is not None else signal_speed_kph
        return replace(point, absolute_speed_kph=absolute_speed_kph)

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

        corner_detections = self._corner_detections_for_current_state(
            event_t,
            lane_change,
            lane_change_phase,
        )
        if corner_detections is not None:
            for vehicle in corner_detections:
                if not vehicle_is_inside_road_edges(vehicle, lane_values):
                    continue
                if not has_nearby_vehicle(detections, vehicle, longitudinal_tolerance=3.0, lateral_tolerance=1.1):
                    detections.append(vehicle)
        else:
            for vehicle in car_state_corner_detections(car_state):
                if not vehicle_is_inside_road_edges(vehicle, lane_values):
                    continue
                if not has_nearby_vehicle(detections, vehicle, longitudinal_tolerance=3.0, lateral_tolerance=1.1):
                    detections.append(vehicle)

        if event_t - self.radar_detection_t < 0.8:
            for vehicle in self.radar_detections:
                if not vehicle_is_inside_road_edges(vehicle, lane_values):
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
        adrv_lane_change_fresh = event_t - self.adrv_lane_changing_t < CORNER_DETECTION_STALE_S
        lane_change_active = (
            lane_change in ("left", "right")
            and lane_change_phase in ("preparing", "changing", "recentering")
        )
        adrv_lane_change_active = adrv_lane_change_fresh and self.adrv_lane_changing != 0

        if adrv_fresh and (lane_change_active or adrv_lane_change_active):
            return tuple(self.adrv_corner_detections.values())
        if ccnc_fresh:
            return tuple(self.ccnc_corner_detections.values())
        if adrv_fresh:
            return tuple(self.adrv_corner_detections.values())
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

        if self.controls_enabled is not None:
            return "engaged" if self.controls_enabled else "paused"

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
            if left_code < 0:
                self.left_lane_prob = 0.0
        if right_code is not None:
            self.right_lane_style = lane_style_from_code(right_code)
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
                "left_visible": True,
                "right_visible": True,
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
        extra_left_visible = (
            outer_left_offset is not None
            and outer_left_offset < -0.78
            and self.outer_left_lane_prob > 0.35
        )
        extra_right_visible = (
            outer_right_offset is not None
            and outer_right_offset > 0.78
            and self.outer_right_lane_prob > 0.35
        )
        left_edge_visible = (
            left_edge_offset is not None
            and left_edge_offset < -0.68
            and self.left_road_edge_confidence > 0.15
            and (extra_left_visible or left_edge_offset > -1.25)
        )
        right_edge_visible = (
            right_edge_offset is not None
            and right_edge_offset > 0.68
            and self.right_road_edge_confidence > 0.15
            and (extra_right_visible or right_edge_offset < 1.25)
        )
        return {
            "width": width,
            "center": center_m,
            "left_offset": clamp((left_y - center_m) / width, -0.75, -0.25),
            "right_offset": clamp((right_y - center_m) / width, 0.25, 0.75),
            "left_visible": self.left_lane_prob > 0.22,
            "right_visible": self.right_lane_prob > 0.22,
            "extra_left_visible": extra_left_visible,
            "extra_right_visible": extra_right_visible,
            "left_road_edge_offset": clamp(left_edge_offset, -2.8, -0.68) if left_edge_visible else None,
            "right_road_edge_offset": clamp(right_edge_offset, 0.68, 2.8) if right_edge_visible else None,
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
    left_road_edge_points = model_line_at(frame.model_road_edges, 0)
    left_road_edge_lateral_shift_m = model_line_lateral_shift(
        left_road_edge_points,
        frame,
        left_road_edge_offset,
        lane_grid_offset,
        use_animated_lane_grid,
    )
    right_road_edge_points = model_line_at(frame.model_road_edges, 1)
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
    )


def route_lane_animation_values(
    frame: RouteReplayFrame,
    observed_ego_lane_offset: float,
) -> tuple[float, float, float, float | None, bool]:
    if frame.lane_change not in ("left", "right"):
        return observed_ego_lane_offset, 0.0, 0.0, None, False

    direction_sign = -1.0 if frame.lane_change == "left" else 1.0
    highlight_lane_offset: float | None = direction_sign
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
        lfa_active=discrete.lfa_active,
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
    )


def lanes_for_frame(
    frame: RouteReplayFrame,
    lane_grid_offset: float = 0.0,
    use_animated_lane_grid: bool = False,
) -> tuple[LaneMarking, ...]:
    left_inner_color = BLUE
    right_inner_color = BLUE
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

    force_lane_change_lanes = use_animated_lane_grid and frame.lane_change in ("left", "right")
    left_inner_visible = frame.left_lane_visible or force_lane_change_lanes
    right_inner_visible = frame.right_lane_visible or force_lane_change_lanes

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
                visible=True,
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
            visible=left_inner_visible,
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
            visible=right_inner_visible,
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
                visible=True,
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
        cut_in = model_lead_is_cut_in(lead)
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
                cut_in=cut_in,
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


def model_lead_is_cut_in(lead: Any) -> bool:
    ys = safe_get(lead, "y")
    xs = safe_get(lead, "x")
    if ys is None or xs is None or len(ys) < 2 or len(xs) < 2:
        return False
    y0 = finite_float(ys[0])
    if y0 is None or abs(y0) < 0.85:
        return False
    for index in range(1, min(len(ys), len(xs))):
        future_y = finite_float(ys[index])
        future_x = finite_float(xs[index])
        if future_y is None or future_x is None or future_x - RADAR_TO_CAMERA_M > 75.0:
            continue
        if abs(future_y) < 0.70 or abs(future_y) < abs(y0) - 0.55:
            return True
    return False


def lane_offset_from_y(y_m: float | None, center_m: float, lane_width_m: float) -> float | None:
    if y_m is None:
        return None
    return (y_m - center_m) / max(0.1, lane_width_m)


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
    pairs = (
        ("LF", "leftLongDist", "leftLatDist", -1.0),
        ("RF", "rightLongDist", "rightLatDist", 1.0),
    )
    detections: list[DetectedVehicle] = []
    for label, distance_name, lateral_name, side in pairs:
        distance_m = safe_float(car_state, distance_name, 0.0)
        if not 0.2 < distance_m < 180.0:
            continue
        lateral_mag = normalized_lateral_m(safe_float(car_state, lateral_name, 0.0))
        detections.append(
            DetectedVehicle(
                label=label,
                longitudinal_m=distance_m,
                lateral_m=side * lateral_mag,
                source="carState",
            )
        )
    return tuple(detections)


def corner_radar_specs(address: int) -> dict[str, tuple[str, int, int, str, int, int, str, int, int, float]]:
    if address == 0x162:
        return {
            "LF": ("le", 112, 5, "le", 117, 11, "le", 128, 7, 1.0),
            "RF": ("le", 136, 5, "le", 141, 11, "le", 152, 7, 1.0),
            "LR": ("le", 163, 5, "be", 175, 8, "le", 176, 7, -1.0),
            "RR": ("le", 192, 5, "le", 197, 8, "le", 205, 7, -1.0),
        }
    return {
        "LF": ("be", 74, 3, "le", 46, 11, "be", 70, 7, 1.0),
        "RF": ("be", 98, 3, "le", 75, 11, "be", 94, 7, 1.0),
        "LR": ("be", 162, 3, "le", 139, 8, "le", 152, 6, -1.0),
        "RR": ("be", 186, 3, "le", 163, 8, "le", 172, 6, -1.0),
    }


def parse_corner_radar_message(address: int, data: bytes) -> dict[str, DetectedVehicle]:
    specs = corner_radar_specs(address)
    detections: dict[str, DetectedVehicle] = {}
    for label, spec in specs.items():
        det_order, det_start, det_len, dist_order, dist_start, dist_len, lat_order, lat_start, lat_len, forward_sign = spec
        detect = dbc_unsigned(data, det_start, det_len, det_order)
        distance_m = dbc_unsigned(data, dist_start, dist_len, dist_order) * 0.1
        if detect == 0 or not 0.2 < distance_m < 180.0:
            continue
        longitudinal_m = forward_sign * distance_m
        if forward_sign < 0.0:
            if not CORNER_RADAR_REAR_MIN_LONGITUDINAL_M <= longitudinal_m <= -0.2:
                continue
        elif not RADAR_MIN_LONGITUDINAL_M <= longitudinal_m <= RADAR_FRONT_MAX_LONGITUDINAL_M:
            continue
        lateral_mag = normalized_lateral_m(dbc_unsigned(data, lat_start, lat_len, lat_order) * 0.1)
        side = -1.0 if label.endswith("F") and label.startswith("L") else 1.0
        if label.startswith("L"):
            side = -1.0
        elif label.startswith("R"):
            side = 1.0
        detections[label] = DetectedVehicle(
            label=label,
            longitudinal_m=longitudinal_m,
            lateral_m=side * lateral_mag,
            source=f"CAN 0x{address:x}",
        )
    return detections


def parse_hyundai_canfd_radar_message(address: int, data: bytes) -> tuple[RadarPoint, ...]:
    if 0x210 <= address <= 0x21F:
        return tuple(
            point
            for point in (
                parse_hyundai_canfd_radar_slot(address, data, 1),
                parse_hyundai_canfd_radar_slot(address, data, 2),
            )
            if point is not None
        )
    if 0x3A5 <= address <= 0x3C4:
        point = parse_hyundai_canfd_radar_point_3a5(address, data)
        return () if point is None else (point,)
    return ()


def is_hyundai_canfd_radar_address(address: int) -> bool:
    return 0x210 <= address <= 0x21F or 0x3A5 <= address <= 0x3C4


def hyundai_canfd_radar_labels_for_address(address: int) -> tuple[str, ...]:
    if 0x210 <= address <= 0x21F:
        index = (address - 0x210) * 2
        return (f"R{index:02d}", f"R{index + 1:02d}")
    if 0x3A5 <= address <= 0x3C4:
        return (f"P{address - 0x3A5:02d}",)
    return ()


def parse_hyundai_canfd_radar_slot(address: int, data: bytes, slot: int) -> RadarPoint | None:
    if len(data) < 32:
        return None
    base = 0 if slot == 1 else 128
    valid_count = dbc_unsigned(data, base + 47, 8, "be")
    if valid_count <= 10:
        return None
    long_dist_m = dbc_unsigned(data, base + 64, 12, "le") * 0.05
    raw_lat_dist_m = dbc_signed(data, base + 76, 12, "le") * 0.05
    rel_speed_mps = dbc_signed(data, base + 88, 14, "le") * 0.01
    raw_lat_speed_mps = dbc_signed(data, base + 104, 13, "le") * 0.01
    rel_accel_mps2 = dbc_signed(data, base + 118, 10, "le") * 0.05
    lat_dist_m = renderer_lateral_from_openpilot_yrel(raw_lat_dist_m)
    lat_speed_mps = renderer_lateral_from_openpilot_yrel(raw_lat_speed_mps)
    if not -10.0 <= lat_dist_m <= 10.0 or not 2.5 <= long_dist_m <= 180.0:
        return None
    index = (address - 0x210) * 2 + (slot - 1)
    return RadarPoint(
        label=f"R{index:02d}",
        longitudinal_m=long_dist_m,
        lateral_m=lat_dist_m,
        source=f"CAN-FD 0x{address:x}.{slot}",
        relative_speed_mps=rel_speed_mps,
        lateral_speed_mps=lat_speed_mps,
        relative_accel_mps2=rel_accel_mps2,
        valid_count=valid_count,
    )


def parse_hyundai_canfd_radar_point_3a5(address: int, data: bytes) -> RadarPoint | None:
    if len(data) < 24:
        return None
    valid = dbc_unsigned(data, 25, 2, "be")
    valid2 = dbc_unsigned(data, 28, 2, "be")
    probability = dbc_unsigned(data, 30, 10, "le") / 1023.0
    valid_count = dbc_unsigned(data, 47, 8, "be")
    if valid_count <= 10:
        return None
    long_dist_m = dbc_unsigned(data, 63, 13, "le") * 0.05
    raw_lat_dist_m = dbc_signed(data, 76, 12, "le") * 0.05
    rel_speed_mps = dbc_signed(data, 88, 14, "le") * 0.01
    in_my_lane = dbc_unsigned(data, 103, 2, "be")
    raw_lat_speed_mps = dbc_signed(data, 104, 13, "le") * 0.01
    rel_accel_mps2 = dbc_signed(data, 118, 10, "le") * 0.05
    lat_dist_m = renderer_lateral_from_openpilot_yrel(raw_lat_dist_m)
    lat_speed_mps = renderer_lateral_from_openpilot_yrel(raw_lat_speed_mps)
    if not -10.0 <= lat_dist_m <= 10.0 or not 2.5 <= long_dist_m <= 180.0:
        return None
    index = address - 0x3A5
    return RadarPoint(
        label=f"P{index:02d}",
        longitudinal_m=long_dist_m,
        lateral_m=lat_dist_m,
        source=f"CAN-FD 0x{address:x}",
        relative_speed_mps=rel_speed_mps,
        lateral_speed_mps=lat_speed_mps,
        relative_accel_mps2=rel_accel_mps2,
        probability=clamp(probability, 0.0, 1.0),
        valid=valid or valid2,
        valid_count=valid_count,
        in_my_lane=in_my_lane,
    )


def renderer_lateral_from_openpilot_yrel(y_rel: float) -> float:
    # openpilot radar/model UI projects radar points as -yRel; this renderer stores x as right-positive.
    return -y_rel


def live_track_to_radar_point(track: Any, index: int, ego_speed_kph: float) -> RadarPoint | None:
    d_rel = safe_optional_float(track, "dRel")
    if d_rel is None or not RADAR_MIN_LONGITUDINAL_M <= d_rel <= RADAR_FRONT_MAX_LONGITUDINAL_M:
        return None
    y_rel = safe_float(track, "yRel", 0.0)
    lateral_m = renderer_lateral_from_openpilot_yrel(y_rel)
    if not -12.0 <= lateral_m <= 12.0:
        return None
    track_id = safe_optional_int(track, "trackId")
    label = f"T{track_id}" if track_id is not None else f"T{index:03d}"
    rel_speed_mps = safe_optional_float(track, "vRel")
    lead_speed_mps = safe_optional_float(track, "vLead")
    absolute_speed_kph = None
    if lead_speed_mps is not None:
        absolute_speed_kph = max(0.0, lead_speed_mps * 3.6)
    elif rel_speed_mps is not None:
        absolute_speed_kph = max(0.0, ego_speed_kph + rel_speed_mps * 3.6)
    lat_speed_mps = safe_optional_float(track, "yvRel")
    if lat_speed_mps is not None:
        lat_speed_mps = renderer_lateral_from_openpilot_yrel(lat_speed_mps)
    measured = bool(safe_get(track, "measured", True))
    return RadarPoint(
        label=label,
        longitudinal_m=d_rel,
        lateral_m=lateral_m,
        source="liveTracks",
        relative_speed_mps=rel_speed_mps,
        absolute_speed_kph=absolute_speed_kph,
        lateral_speed_mps=lat_speed_mps,
        relative_accel_mps2=safe_optional_float(track, "aRel"),
        probability=0.72 if measured else 0.38,
        valid=1 if measured else 0,
    )


def sorted_radar_points(points: Any) -> tuple[RadarPoint, ...]:
    filtered = [
        point
        for point in points
        if -12.0 <= point.lateral_m <= 12.0
        and RADAR_MIN_LONGITUDINAL_M <= point.longitudinal_m <= RADAR_FRONT_MAX_LONGITUDINAL_M
    ]
    filtered.sort(key=lambda point: (point.longitudinal_m, abs(point.lateral_m), point.label))
    return tuple(filtered)


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
            from cereal import log as capnp_log

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
    car_schema = openpilot_root / "opendbc_repo" / "opendbc" / "car" / "car.capnp"
    if not car_schema.exists():
        raise RuntimeError(f"openpilot car schema not found: {car_schema}")

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
        if (path / "cereal").exists() and (path / "opendbc_repo").exists():
            return path
        nested = path / "openpilot"
        if (nested / "cereal").exists() and (nested / "opendbc_repo").exists():
            return nested
    return start / "openpilot"


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
