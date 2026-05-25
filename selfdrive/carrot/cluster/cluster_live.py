from __future__ import annotations

from dataclasses import replace
import math
import sys
import time
from pathlib import Path
from typing import Any

from cluster_config import BLUE, DEFAULT_LANE_WIDTH_M
from cluster_models import ClusterUiState, LaneMarking, LiveDebugInfo
from cluster_route_replay import RouteLogParser, frame_to_state, safe_get, safe_optional_float
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
    "liveDelay",
    "liveParameters",
    "liveTorqueParameters",
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
        self.params: Any | None = None
        self._next_debug_param_read_t = 0.0
        self._custom_steer_ratio: float | None = None
        self._steer_actuator_delay_param_s: float | None = None
        try:
            from openpilot.common.params import Params

            self.params = Params()
        except Exception:
            pass

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
            self.last_state = replace(frame_to_state(frame), live_debug=self._live_debug_info())
            self.frames += 1
            return self.last_state

        self.last_state = replace(standby_state(), live_debug=self._live_debug_info())
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

    def _live_debug_info(self) -> LiveDebugInfo | None:
        self._refresh_debug_params()

        live_delay_calibration_percent = None
        live_delay_lateral_s = None
        if self._service_alive("liveDelay"):
            live_delay = self.sm["liveDelay"]
            live_delay_calibration_percent = safe_optional_float(live_delay, "calPerc")
            live_delay_lateral_s = safe_optional_float(live_delay, "lateralDelay")
        steer_actuator_delay_s = self._effective_steer_actuator_delay(live_delay_lateral_s)

        live_torque_calibration_percent = None
        live_torque_valid = None
        live_torque_lat_accel_factor = None
        live_torque_friction = None
        if self._service_alive("liveTorqueParameters"):
            live_torque = self.sm["liveTorqueParameters"]
            live_torque_calibration_percent = safe_optional_float(live_torque, "calPerc")
            live_torque_valid = bool(safe_get(live_torque, "liveValid", False))
            live_torque_lat_accel_factor = safe_optional_float(live_torque, "latAccelFactorFiltered")
            if live_torque_lat_accel_factor is None:
                live_torque_lat_accel_factor = safe_optional_float(live_torque, "latAccelFactor")
            live_torque_friction = safe_optional_float(live_torque, "frictionCoefficientFiltered")
            if live_torque_friction is None:
                live_torque_friction = safe_optional_float(live_torque, "frictionCoefficient")

        live_steer_ratio = None
        if self._service_alive("liveParameters"):
            live_steer_ratio = safe_optional_float(self.sm["liveParameters"], "steerRatio")

        info = LiveDebugInfo(
            live_delay_calibration_percent=live_delay_calibration_percent,
            live_delay_lateral_s=live_delay_lateral_s,
            live_torque_calibration_percent=live_torque_calibration_percent,
            live_torque_valid=live_torque_valid,
            live_torque_lat_accel_factor=live_torque_lat_accel_factor,
            live_torque_friction=live_torque_friction,
            live_steer_ratio=live_steer_ratio,
            custom_steer_ratio=self._custom_steer_ratio,
            steer_actuator_delay_s=steer_actuator_delay_s,
        )
        values = (
            info.live_delay_calibration_percent,
            info.live_delay_lateral_s,
            info.live_torque_calibration_percent,
            info.live_torque_valid,
            info.live_torque_lat_accel_factor,
            info.live_torque_friction,
            info.live_steer_ratio,
            info.custom_steer_ratio,
            info.steer_actuator_delay_s,
        )
        return info if any(value is not None for value in values) else None

    def _refresh_debug_params(self) -> None:
        now = time.monotonic()
        if now < self._next_debug_param_read_t:
            return
        self._next_debug_param_read_t = now + 1.0
        if self.params is None:
            return
        self._custom_steer_ratio = self._finite_param_float("CustomSR", 0.1)
        self._steer_actuator_delay_param_s = self._finite_param_float("SteerActuatorDelay", 0.01)

    def _effective_steer_actuator_delay(self, live_delay_lateral_s: float | None) -> float | None:
        if self._steer_actuator_delay_param_s is not None and self._steer_actuator_delay_param_s > 0.0:
            return self._steer_actuator_delay_param_s
        if live_delay_lateral_s is not None:
            return live_delay_lateral_s
        return self._steer_actuator_delay_param_s

    def _finite_param_float(self, key: str, scale: float) -> float | None:
        if self.params is None:
            return None
        try:
            value = float(self.params.get_float(key)) * scale
        except Exception:
            return None
        return value if math.isfinite(value) else None

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
