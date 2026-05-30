from __future__ import annotations

from dataclasses import replace
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

from cluster_config import BLUE, DEFAULT_LANE_WIDTH_M, SHOW_PLOT_MODE_PARAM
from cluster_models import ClusterUiState, DebugPlotSnapshot, LaneMarking, LiveDebugInfo
from cluster_route_replay import RouteLogParser, finite_float, frame_to_state, safe_get, safe_optional_float
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
    "selfdriveState",
    "carControl",
    "deviceState",
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
        try:
            from cereal import log

            self.log: Any | None = log
        except Exception:
            self.log = None
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
        self._cached_live_debug: LiveDebugInfo | None = None
        self._show_plot_mode = 0
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
            self.last_state = self._with_debug_state(frame_to_state(frame))
            self.frames += 1
            return self.last_state

        self.last_state = self._with_debug_state(standby_state())
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

    def screen_brightness_percent(self) -> int | None:
        if not self._service_alive("deviceState"):
            return None
        try:
            value = float(self.sm["deviceState"].screenBrightnessPercent)
        except Exception:
            return None
        if not math.isfinite(value):
            return None
        return int(round(clamp(value, 0.0, 100.0)))

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
            self.parser._update_nav_instruction(data, event_t)
        elif service == "longitudinalPlan":
            self.parser._update_longitudinal_plan(data)
        elif service == "controlsState":
            self.parser._update_controls_state(data)
        elif service == "selfdriveState":
            self.parser._update_selfdrive_state(data)
        elif service == "carControl":
            self.parser._update_car_control(data)
        elif service == "cameraOdometry":
            self.parser._update_camera_odometry(data, self._service_valid(service))
        elif service == "radarState":
            self.parser._update_radar_state(data, event_t)
        elif service == "liveTracks":
            self.parser._update_live_tracks(data, event_t)
        elif service == "can":
            self.parser._update_can_detections(data, event_t)

    def _with_debug_state(self, state: ClusterUiState) -> ClusterUiState:
        return replace(state, live_debug=self._live_debug_info(), debug_plot=self._debug_plot_snapshot())

    def _live_debug_info(self) -> LiveDebugInfo | None:
        self._refresh_debug_params()
        cached = self._cached_live_debug

        live_delay_calibration_percent = cached.live_delay_calibration_percent if cached is not None else None
        live_delay_lateral_s = cached.live_delay_lateral_s if cached is not None else None
        if self._service_alive("liveDelay"):
            live_delay = self.sm["liveDelay"]
            live_delay_calibration_percent = self._first_present(
                safe_optional_float(live_delay, "calPerc"),
                live_delay_calibration_percent,
            )
            live_delay_lateral_s = self._first_present(
                safe_optional_float(live_delay, "lateralDelay"),
                live_delay_lateral_s,
            )
        steer_actuator_delay_s = self._effective_steer_actuator_delay(live_delay_lateral_s)

        live_torque_calibration_percent = cached.live_torque_calibration_percent if cached is not None else None
        live_torque_valid = cached.live_torque_valid if cached is not None else None
        live_torque_lat_accel_factor = cached.live_torque_lat_accel_factor if cached is not None else None
        live_torque_friction = cached.live_torque_friction if cached is not None else None
        if self._service_alive("liveTorqueParameters"):
            live_torque = self.sm["liveTorqueParameters"]
            live_torque_calibration_percent = self._first_present(
                safe_optional_float(live_torque, "calPerc"),
                live_torque_calibration_percent,
            )
            live_valid = safe_get(live_torque, "liveValid")
            live_torque_valid = bool(live_valid) if live_valid is not None else live_torque_valid
            live_torque_lat_accel_factor = self._first_present(
                safe_optional_float(live_torque, "latAccelFactorFiltered"),
                live_torque_lat_accel_factor,
            )
            if live_torque_lat_accel_factor is None:
                live_torque_lat_accel_factor = safe_optional_float(live_torque, "latAccelFactor")
            live_torque_friction = self._first_present(
                safe_optional_float(live_torque, "frictionCoefficientFiltered"),
                live_torque_friction,
            )
            if live_torque_friction is None:
                live_torque_friction = safe_optional_float(live_torque, "frictionCoefficient")

        live_steer_ratio = cached.live_steer_ratio if cached is not None else None
        if self._service_alive("liveParameters"):
            live_steer_ratio = self._first_present(
                safe_optional_float(self.sm["liveParameters"], "steerRatio"),
                live_steer_ratio,
            )

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
        self._show_plot_mode = self._param_int(SHOW_PLOT_MODE_PARAM, 0)
        self._cached_live_debug = self._read_cached_live_debug()

    def _debug_plot_snapshot(self) -> DebugPlotSnapshot | None:
        self._refresh_debug_params()
        mode = self._show_plot_mode
        if mode <= 0:
            return None
        values, title = self._make_debug_plot_data(mode)
        return DebugPlotSnapshot(mode=mode, title=title, values=values)

    def _make_debug_plot_data(self, show_plot_mode: int) -> tuple[tuple[float, float, float], str]:
        car_state = self._service_data("carState")
        long_plan = self._service_data("longitudinalPlan")
        car_control = self._service_data("carControl")
        controls_state = self._service_data("controlsState")
        model = self._service_data("modelV2")
        radar = self._service_data("radarState")
        live_params = self._service_data("liveParameters")

        a_ego = self._finite_attr(car_state, "aEgo")
        v_ego = self._finite_attr(car_state, "vEgo")
        accel_target = self._finite_index(safe_get(long_plan, "accels"), 0)
        speed_target = self._finite_index(safe_get(long_plan, "speeds"), 0)
        accel_out = self._finite_path(car_control, "actuators.accel")

        if show_plot_mode == 1:
            return (a_ego, accel_target, accel_out), "1.Accel (Y:a_ego, G:a_target, O:a_out)"

        if show_plot_mode == 2:
            return (speed_target, v_ego, a_ego), "2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)"

        if show_plot_mode == 3:
            position = safe_get(model, "position")
            velocity = safe_get(model, "velocity")
            return (
                self._finite_index(safe_get(position, "x"), 32),
                self._finite_index(safe_get(velocity, "x"), 32),
                self._finite_index(safe_get(velocity, "x"), 0),
            ), "3.Model(Y:pos_32, G:vel_32, O:vel_0)"

        if show_plot_mode == 4:
            lead = safe_get(radar, "leadOne")
            return (
                accel_target,
                self._finite_attr(lead, "aLeadK"),
                self._finite_attr(lead, "vRel"),
            ), "4.Lead(Y:accel, G:a_leadK, O:v_rel)"

        if show_plot_mode == 5:
            lead = safe_get(radar, "leadOne")
            return (
                a_ego,
                self._finite_attr(lead, "aLead"),
                self._finite_attr(lead, "jLead"),
            ), "5.Lead(Y:a_ego, G:a_lead, O:j_lead)"

        if show_plot_mode == 6:
            torque_state = self._path_value(controls_state, "lateralControlState.torqueState")
            return (
                self._finite_attr(torque_state, "actualLateralAccel") * 10.0,
                self._finite_attr(torque_state, "desiredLateralAccel") * 10.0,
                self._finite_attr(torque_state, "output") * 10.0,
            ), "6.Steer(Y:actual, G:desire, O:output) *10"

        if show_plot_mode == 7:
            return (
                self._finite_attr(car_state, "steeringAngleDeg"),
                self._finite_path(car_control, "actuators.steeringAngleDeg"),
                self._finite_attr(live_params, "angleOffsetDeg") * 10.0,
            ), "7.SteerA(Y:Actual, G:Target, O:Offset*10)"

        if show_plot_mode == 8:
            curvature = self._finite_path(car_control, "actuators.curvature") * 10000.0
            return (curvature, curvature, curvature), "8.Curvature(*10000)"

        return (0.0, 0.0, 0.0), "no data"

    def _read_cached_live_debug(self) -> LiveDebugInfo | None:
        if self.params is None:
            return None

        live_delay = self._event_service_from_param("LiveDelay", "liveDelay")
        live_torque = self._event_service_from_param("LiveTorqueParameters", "liveTorqueParameters")
        live_parameters = self._event_service_from_param("LiveParametersV2", "liveParameters")
        live_steer_ratio = None
        if live_parameters is not None:
            live_steer_ratio = safe_optional_float(live_parameters, "steerRatio")
        if live_steer_ratio is None:
            live_steer_ratio = self._legacy_live_parameters_steer_ratio()

        live_torque_valid = None
        if live_torque is not None:
            live_valid = safe_get(live_torque, "liveValid")
            live_torque_valid = bool(live_valid) if live_valid is not None else None

        info = LiveDebugInfo(
            live_delay_calibration_percent=safe_optional_float(live_delay, "calPerc") if live_delay is not None else None,
            live_delay_lateral_s=safe_optional_float(live_delay, "lateralDelay") if live_delay is not None else None,
            live_torque_calibration_percent=(
                safe_optional_float(live_torque, "calPerc") if live_torque is not None else None
            ),
            live_torque_valid=live_torque_valid,
            live_torque_lat_accel_factor=(
                safe_optional_float(live_torque, "latAccelFactorFiltered") if live_torque is not None else None
            ),
            live_torque_friction=(
                safe_optional_float(live_torque, "frictionCoefficientFiltered") if live_torque is not None else None
            ),
            live_steer_ratio=live_steer_ratio,
        )
        values = (
            info.live_delay_calibration_percent,
            info.live_delay_lateral_s,
            info.live_torque_calibration_percent,
            info.live_torque_valid,
            info.live_torque_lat_accel_factor,
            info.live_torque_friction,
            info.live_steer_ratio,
        )
        return info if any(value is not None for value in values) else None

    def _event_service_from_param(self, param_key: str, service_name: str) -> Any | None:
        if self.params is None or self.log is None:
            return None
        try:
            data = self.params.get(param_key)
        except Exception:
            return None
        if not data:
            return None
        try:
            event = self.messaging.log_from_bytes(data, self.log.Event)
            return safe_get(event, service_name)
        except Exception:
            return None

    def _legacy_live_parameters_steer_ratio(self) -> float | None:
        if self.params is None:
            return None
        try:
            data = self.params.get("LiveParameters")
        except Exception:
            return None
        if not data:
            return None
        if isinstance(data, dict):
            return finite_float(data.get("steerRatio"))
        try:
            if isinstance(data, (bytes, bytearray)):
                data = data.decode("utf-8")
            parsed = json.loads(data)
        except Exception:
            return None
        if not isinstance(parsed, dict):
            return None
        return finite_float(parsed.get("steerRatio"))

    def _effective_steer_actuator_delay(self, live_delay_lateral_s: float | None) -> float | None:
        if self._steer_actuator_delay_param_s is not None and self._steer_actuator_delay_param_s > 0.0:
            return self._steer_actuator_delay_param_s
        if live_delay_lateral_s is not None:
            return live_delay_lateral_s
        return self._steer_actuator_delay_param_s

    @staticmethod
    def _first_present(value: Any | None, fallback: Any | None) -> Any | None:
        return value if value is not None else fallback

    def _finite_param_float(self, key: str, scale: float) -> float | None:
        if self.params is None:
            return None
        try:
            value = float(self.params.get_float(key)) * scale
        except Exception:
            return None
        return value if math.isfinite(value) else None

    def _param_int(self, key: str, default: int) -> int:
        if self.params is None:
            return default
        try:
            value = int(self.params.get_int(key))
        except Exception:
            return default
        return value if value >= 0 else default

    def _service_data(self, service: str) -> Any | None:
        try:
            return self.sm[service]
        except Exception:
            return None

    @staticmethod
    def _path_value(obj: Any | None, path: str) -> Any | None:
        current = obj
        for name in path.split("."):
            if current is None:
                return None
            current = safe_get(current, name)
        return current

    def _finite_path(self, obj: Any | None, path: str, default: float = 0.0) -> float:
        return self._finite_value(self._path_value(obj, path), default)

    def _finite_attr(self, obj: Any | None, name: str, default: float = 0.0) -> float:
        return self._finite_value(safe_get(obj, name), default)

    def _finite_index(self, values: Any | None, index: int, default: float = 0.0) -> float:
        if values is None or index < 0:
            return default
        try:
            value = values[index]
        except Exception:
            return default
        return self._finite_value(value, default)

    @staticmethod
    def _finite_value(value: Any | None, default: float = 0.0) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return default
        return parsed if math.isfinite(parsed) else default

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
