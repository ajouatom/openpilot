from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

from cluster_config import BLUE, DEFAULT_LANE_WIDTH_M
from cluster_models import ClusterUiState, LaneMarking
from cluster_route_replay import RouteLogParser, frame_to_state
from cluster_utils import clamp


def find_openpilot_root(start: Path) -> Path | None:
    for path in (start, *start.parents):
        if (path / "cereal").exists() and (path / "selfdrive").exists():
            return path
        nested = path / "openpilot"
        if (nested / "cereal").exists() and (nested / "selfdrive").exists():
            return nested
    return None


OPENPILOT_ROOT = find_openpilot_root(Path(__file__).resolve().parent)
if OPENPILOT_ROOT is not None:
    sys.path.insert(0, str(OPENPILOT_ROOT))


LIVE_SERVICES_BASE = (
    "carState",
    "modelV2",
    "radarState",
    "liveTracks",
    "longitudinalPlan",
    "lateralPlan",
    "controlsState",
    "cameraOdometry",
    "drivingModelData",
    "navInstruction",
    "navInstructionCarrot",
)
LIVE_CAN_SERVICES = ("can",)


class OpenpilotLiveSource:
    def __init__(self, include_can: bool = True, timeout_ms: int = 0) -> None:
        try:
            import cereal.messaging as messaging
        except Exception as exc:
            raise RuntimeError(
                "Openpilot live input requires cereal.messaging. Run from an openpilot environment "
                "or use --input route/random for local checks."
            ) from exc

        self.messaging: Any = messaging
        self.services = list(LIVE_SERVICES_BASE + (LIVE_CAN_SERVICES if include_can else ()))
        self.sm = messaging.SubMaster(self.services)
        self.parser = RouteLogParser()
        self.timeout_ms = max(0, int(timeout_ms))
        self.last_state: ClusterUiState | None = None
        self.start_t = time.monotonic()
        self.frames = 0

    def update(self) -> ClusterUiState:
        self.sm.update(self.timeout_ms)
        self._update_current_speed()

        for service in self.services:
            if not self._service_updated(service):
                continue
            event_t = self._service_time(service)
            self._apply_service_update(service, event_t)

        if self._service_alive("carState"):
            event_t = self._service_time("carState")
            frame = self.parser._frame_from_car_state(self.sm["carState"], event_t)
            self.last_state = frame_to_state(frame)
            self.frames += 1
            return self.last_state

        self.last_state = standby_state()
        return self.last_state

    def status_text(self) -> str:
        alive = sum(1 for service in self.services if self._service_alive(service))
        updated = sum(1 for service in self.services if self._service_updated(service))
        can_status = "can" if "can" in self.services else "no-can"
        age = time.monotonic() - self.start_t
        fps = self.frames / age if age > 0.1 else 0.0
        radar_count = len(self.last_state.radar_points) if self.last_state is not None else 0
        detected_count = len(self.last_state.detected_vehicles) if self.last_state is not None else 0
        return (
            f"live {can_status} alive={alive}/{len(self.services)} upd={updated} state={fps:.1f}Hz "
            f"radar={radar_count} detected={detected_count}"
        )

    def close(self) -> None:
        return None

    def _apply_service_update(self, service: str, event_t: float) -> None:
        data = self.sm[service]
        if service == "drivingModelData":
            self.parser._update_driving_model(data)
        elif service == "modelV2":
            self.parser._update_model_v2(data, event_t)
        elif service == "lateralPlan":
            self.parser._update_lateral_plan(data)
        elif service in ("navInstruction", "navInstructionCarrot"):
            self.parser._update_nav_instruction(data)
        elif service == "longitudinalPlan":
            self.parser._update_longitudinal_plan(data)
        elif service == "controlsState":
            self.parser._update_controls_state(data)
        elif service == "cameraOdometry":
            self.parser._update_camera_odometry(data, self._service_valid(service))
        elif service == "radarState":
            self.parser._update_radar_state(data, event_t)
        elif service == "liveTracks":
            self.parser._update_live_tracks(data, event_t)
        elif service == "can":
            self.parser._update_can_detections(data, event_t)

    def _update_current_speed(self) -> None:
        if not self._service_alive("carState"):
            return
        try:
            self.parser.current_speed_kph = clamp(float(self.sm["carState"].vEgo) * 3.6, 0.0, 140.0)
        except Exception:
            return

    def _service_time(self, service: str) -> float:
        try:
            mono_time = self.sm.logMonoTime.get(service, 0)
        except AttributeError:
            mono_time = 0
        return float(mono_time) / 1_000_000_000.0 if mono_time else time.monotonic()

    def _service_alive(self, service: str) -> bool:
        try:
            return bool(self.sm.alive.get(service, False))
        except AttributeError:
            return False

    def _service_updated(self, service: str) -> bool:
        try:
            return bool(self.sm.updated.get(service, False))
        except AttributeError:
            return False

    def _service_valid(self, service: str) -> bool:
        try:
            return bool(self.sm.valid.get(service, True))
        except AttributeError:
            return True


def standby_state() -> ClusterUiState:
    return ClusterUiState(
        speed_kph=0.0,
        accel_mps2=0.0,
        steering=0.0,
        speed_limit_kph=None,
        cruise_kph=None,
        cruise_display_state="off",
        left_signal=False,
        right_signal=False,
        left_blindspot=False,
        right_blindspot=False,
        lane_change=None,
        lane_change_phase="idle",
        lane_change_progress=0.0,
        highlight_lane=None,
        highlight_lane_offset=None,
        ego_lane_offset=0.0,
        road_view_lane_position=0.0,
        camera_lane_center_offset_m=None,
        lane_width_m=DEFAULT_LANE_WIDTH_M,
        steering_angle_deg=None,
        surround_yaw_deg=0.0,
        surround_pitch_deg=0.0,
        surround_view_active=False,
        lanes=(
            LaneMarking(-0.5, BLUE, "solid", width=7),
            LaneMarking(0.5, BLUE, "solid", width=7),
        ),
    )
