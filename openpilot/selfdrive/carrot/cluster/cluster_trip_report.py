from __future__ import annotations

import math
from typing import Any

from cluster_models import TripReportState, TripTracePoint


EARTH_RADIUS_M = 6_378_137.0
TRIP_MOVING_SPEED_MPS = 0.3
TRIP_MAX_DT_S = 0.5
TRIP_TRACE_MIN_PERIOD_S = 0.4
TRIP_TRACE_MIN_DISTANCE_M = 3.0
TRIP_TRACE_MAX_POINTS = 512
TRIP_TRACE_MIN_RADIUS_M = 250.0
TRIP_TRACE_MAX_RADIUS_M = 1_000.0
TRIP_TRACE_RADIUS_QUANTUM_M = 10.0
TRIP_HARD_ACCEL_MPS2 = 2.5
TRIP_HARD_BRAKE_MPS2 = -3.0
TRIP_HARD_CORNER_MPS2 = 3.0
TRIP_CORNER_MIN_SPEED_MPS = 5.5
TRIP_EVENT_MIN_GAP_S = 1.5
TRIP_LIVE_POSE_MAX_AGE_S = 1.0
TRIP_GPS_MAX_AGE_S = 1.5
TRIP_GPS_MAX_ACCURACY_M = 50.0
TRIP_TRACE_SHIFT_BATCH_M = 0.5


def _finite(value: Any, default: float | None = None) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if math.isfinite(parsed) else default


def _value(obj: Any, name: str, default: Any = None) -> Any:
    if obj is None:
        return default
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class TripReportTracker:
    """Low-cost trip statistics and a bounded dead-reckoned trace."""

    def __init__(self) -> None:
        self._last_t: float | None = None
        self._duration_s = 0.0
        self._moving_time_s = 0.0
        self._distance_m = 0.0
        self._auto_distance_m = 0.0
        self._max_speed_kph = 0.0
        self._max_accel_mps2 = 0.0
        self._max_decel_mps2 = 0.0
        self._hard_accel_count = 0
        self._hard_brake_count = 0
        self._hard_corner_count = 0
        self._hard_accel_active = False
        self._hard_brake_active = False
        self._hard_corner_active = False
        self._last_hard_accel_t = -math.inf
        self._last_hard_brake_t = -math.inf
        self._last_hard_corner_t = -math.inf

        self._x_m = 0.0
        self._y_m = 0.0
        self._heading_rad = 0.0
        self._heading_initialized = False
        self._heading_source = "steering"
        self._live_pose_yaw_rad: float | None = None
        self._live_pose_heading_offset_rad: float | None = None
        self._live_pose_t = -math.inf

        self._gps_origin_lat_deg: float | None = None
        self._gps_origin_lon_deg: float | None = None
        self._gps_origin_x_m = 0.0
        self._gps_origin_y_m = 0.0
        self._gps_x_m: float | None = None
        self._gps_y_m: float | None = None
        self._gps_bearing_rad: float | None = None
        self._gps_bearing_t = -math.inf
        self._gps_accuracy_m = TRIP_GPS_MAX_ACCURACY_M
        self._gps_t = -math.inf
        self._gps_applied_t = -math.inf
        self._gps_corrected = False

        self._trace_points: list[TripTracePoint] = []
        self._trace_snapshot: tuple[TripTracePoint, ...] = ()
        self._trace_snapshot_generation = -1
        self._trace_generation = 0
        self._last_trace_t = -math.inf
        self._last_trace_x_m = 0.0
        self._last_trace_y_m = 0.0
        self._pending_trace_shift_x_m = 0.0
        self._pending_trace_shift_y_m = 0.0
        self._trace_radius_m = TRIP_TRACE_MIN_RADIUS_M
        self._last_snapshot = TripReportState()

    def update_live_pose(self, live_pose: Any, event_t: float) -> None:
        orientation = _value(live_pose, "orientationNED")
        valid = bool(_value(orientation, "valid", False))
        inputs_ok = bool(_value(live_pose, "inputsOK", False))
        sensors_ok = bool(_value(live_pose, "sensorsOK", False))
        yaw_rad = _finite(_value(orientation, "z"))
        yaw_std = _finite(_value(orientation, "zStd"), 0.0)
        if not (valid and inputs_ok and sensors_ok and yaw_rad is not None and yaw_std is not None):
            return
        if yaw_std > 0.5:
            return
        self._live_pose_yaw_rad = _wrap_angle(yaw_rad)
        self._live_pose_t = event_t

    def update_gps(self, gps: Any, event_t: float) -> None:
        has_fix = bool(_value(gps, "hasFix", False))
        try:
            flags = int(_value(gps, "flags", 0))
        except (TypeError, ValueError):
            flags = 0
        if not has_fix and flags <= 0:
            return

        latitude = _finite(_value(gps, "latitude"))
        longitude = _finite(_value(gps, "longitude"))
        accuracy = _finite(_value(gps, "horizontalAccuracy"), TRIP_GPS_MAX_ACCURACY_M)
        if (
            latitude is None
            or longitude is None
            or accuracy is None
            or abs(latitude) > 90.0
            or abs(longitude) > 180.0
            or accuracy > TRIP_GPS_MAX_ACCURACY_M
        ):
            return

        if self._gps_origin_lat_deg is None or self._gps_origin_lon_deg is None:
            self._gps_origin_lat_deg = latitude
            self._gps_origin_lon_deg = longitude
            self._gps_origin_x_m = self._x_m
            self._gps_origin_y_m = self._y_m

        lat0_rad = math.radians(self._gps_origin_lat_deg)
        self._gps_x_m = self._gps_origin_x_m + math.radians(longitude - self._gps_origin_lon_deg) * math.cos(lat0_rad) * EARTH_RADIUS_M
        self._gps_y_m = self._gps_origin_y_m + math.radians(latitude - self._gps_origin_lat_deg) * EARTH_RADIUS_M
        self._gps_accuracy_m = max(1.0, accuracy)
        self._gps_t = event_t

        bearing_deg = _finite(_value(gps, "bearingDeg"))
        bearing_accuracy_deg = _finite(_value(gps, "bearingAccuracyDeg"), 180.0)
        speed_mps = _finite(_value(gps, "speed"), 0.0) or 0.0
        if (
            bearing_deg is not None
            and bearing_accuracy_deg is not None
            and speed_mps >= 2.0
            and bearing_accuracy_deg <= 20.0
        ):
            self._gps_bearing_rad = _wrap_angle(math.radians(bearing_deg))
            self._gps_bearing_t = event_t

    def update(
        self,
        event_t: float,
        speed_mps: float,
        accel_mps2: float,
        steering_angle_deg: float | None,
        enabled: bool,
        wheelbase_m: float,
        steer_ratio: float,
    ) -> TripReportState:
        if not math.isfinite(event_t):
            return self._last_snapshot
        speed_mps = max(0.0, speed_mps if math.isfinite(speed_mps) else 0.0)
        accel_mps2 = accel_mps2 if math.isfinite(accel_mps2) else 0.0

        if self._last_t is None:
            self._last_t = event_t
            self._apply_heading_observations(event_t, speed_mps)
            self._apply_gps_position_correction(event_t)
            self._append_trace_point(event_t, speed_mps * 3.6, force=True)
            self._last_snapshot = self._snapshot()
            return self._last_snapshot

        raw_dt = event_t - self._last_t
        if raw_dt <= 0.0:
            return self._last_snapshot
        self._last_t = event_t
        dt = min(raw_dt, TRIP_MAX_DT_S)
        self._duration_s += dt

        pose_fresh = self._apply_heading_observations(event_t, speed_mps)
        road_wheel_angle_rad = 0.0
        if steering_angle_deg is not None and math.isfinite(steering_angle_deg):
            road_wheel_angle_rad = math.radians(steering_angle_deg) / max(1.0, steer_ratio)
        curvature_m_inv = math.tan(road_wheel_angle_rad) / max(1.5, wheelbase_m)
        if not pose_fresh:
            self._heading_rad = _wrap_angle(self._heading_rad - curvature_m_inv * speed_mps * dt)
            self._heading_source = "steering"

        distance_step_m = speed_mps * dt
        self._x_m += math.sin(self._heading_rad) * distance_step_m
        self._y_m += math.cos(self._heading_rad) * distance_step_m
        self._apply_gps_position_correction(event_t)

        self._distance_m += distance_step_m
        if speed_mps >= TRIP_MOVING_SPEED_MPS:
            self._moving_time_s += dt
            if enabled:
                self._auto_distance_m += distance_step_m
        self._max_speed_kph = max(self._max_speed_kph, speed_mps * 3.6)
        self._max_accel_mps2 = max(self._max_accel_mps2, accel_mps2)
        self._max_decel_mps2 = min(self._max_decel_mps2, accel_mps2)
        self._update_events(event_t, speed_mps, accel_mps2, curvature_m_inv)
        self._append_trace_point(event_t, speed_mps * 3.6)
        self._last_snapshot = self._snapshot()
        return self._last_snapshot

    def _apply_heading_observations(self, event_t: float, speed_mps: float) -> bool:
        pose_fresh = self._live_pose_yaw_rad is not None and event_t - self._live_pose_t <= TRIP_LIVE_POSE_MAX_AGE_S
        gps_heading_fresh = (
            self._gps_bearing_rad is not None
            and speed_mps >= 2.0
            and event_t - self._gps_bearing_t <= TRIP_GPS_MAX_AGE_S
        )
        if pose_fresh:
            if self._live_pose_heading_offset_rad is None and gps_heading_fresh:
                self._live_pose_heading_offset_rad = _wrap_angle(self._gps_bearing_rad - self._live_pose_yaw_rad)
                if self._heading_initialized:
                    rotation_rad = _wrap_angle(self._gps_bearing_rad - self._heading_rad)
                    self._rotate_trace_frame(rotation_rad)
                    self._heading_rad = self._gps_bearing_rad
                self._gps_corrected = True

            pose_heading_rad = self._live_pose_yaw_rad
            if self._live_pose_heading_offset_rad is not None:
                pose_heading_rad = _wrap_angle(pose_heading_rad + self._live_pose_heading_offset_rad)
            if not self._heading_initialized:
                self._heading_rad = pose_heading_rad
                self._heading_initialized = True
            else:
                yaw_error = _wrap_angle(pose_heading_rad - self._heading_rad)
                if abs(yaw_error) <= math.radians(75.0):
                    self._heading_rad = _wrap_angle(self._heading_rad + yaw_error * 0.45)
            self._heading_source = "livePose"
            return True

        if gps_heading_fresh:
            if not self._heading_initialized:
                self._heading_rad = self._gps_bearing_rad
                self._heading_initialized = True
            else:
                error = _wrap_angle(self._gps_bearing_rad - self._heading_rad)
                self._heading_rad = _wrap_angle(self._heading_rad + error * 0.12)
            self._heading_source = "gps"
        return False

    def _apply_gps_position_correction(self, event_t: float) -> None:
        if (
            self._gps_x_m is None
            or self._gps_y_m is None
            or self._gps_t <= self._gps_applied_t
            or event_t - self._gps_t > TRIP_GPS_MAX_AGE_S
        ):
            return
        self._gps_applied_t = self._gps_t
        error_x = self._gps_x_m - self._x_m
        error_y = self._gps_y_m - self._y_m
        error_m = math.hypot(error_x, error_y)
        plausible_error_m = max(35.0, self._gps_accuracy_m * 5.0)
        if error_m > plausible_error_m:
            return
        alpha = 0.18 if error_m < 8.0 else 0.08
        correction_x_m = error_x * alpha
        correction_y_m = error_y * alpha
        self._x_m += correction_x_m
        self._y_m += correction_y_m
        self._translate_trace_frame(correction_x_m, correction_y_m)
        self._gps_corrected = True

    def _translate_trace_frame(self, delta_x_m: float, delta_y_m: float) -> None:
        if abs(delta_x_m) <= 1e-9 and abs(delta_y_m) <= 1e-9:
            return
        self._last_trace_x_m += delta_x_m
        self._last_trace_y_m += delta_y_m
        self._pending_trace_shift_x_m += delta_x_m
        self._pending_trace_shift_y_m += delta_y_m
        if math.hypot(self._pending_trace_shift_x_m, self._pending_trace_shift_y_m) >= TRIP_TRACE_SHIFT_BATCH_M:
            self._flush_pending_trace_shift()

    def _flush_pending_trace_shift(self) -> None:
        delta_x_m = self._pending_trace_shift_x_m
        delta_y_m = self._pending_trace_shift_y_m
        if abs(delta_x_m) <= 1e-9 and abs(delta_y_m) <= 1e-9:
            return
        if self._trace_points:
            self._trace_points = [
                TripTracePoint(point.x_m + delta_x_m, point.y_m + delta_y_m, point.speed_kph)
                for point in self._trace_points
            ]
            self._trace_generation += 1
        self._pending_trace_shift_x_m = 0.0
        self._pending_trace_shift_y_m = 0.0

    def _rotate_trace_frame(self, rotation_rad: float) -> None:
        if abs(rotation_rad) <= 1e-9:
            return
        self._flush_pending_trace_shift()
        sin_rotation = math.sin(rotation_rad)
        cos_rotation = math.cos(rotation_rad)

        def rotate_point(x_m: float, y_m: float) -> tuple[float, float]:
            relative_x_m = x_m - self._x_m
            relative_y_m = y_m - self._y_m
            return (
                self._x_m + cos_rotation * relative_x_m + sin_rotation * relative_y_m,
                self._y_m - sin_rotation * relative_x_m + cos_rotation * relative_y_m,
            )

        if self._trace_points:
            self._trace_points = [
                TripTracePoint(*rotate_point(point.x_m, point.y_m), point.speed_kph)
                for point in self._trace_points
            ]
            self._trace_generation += 1
        self._last_trace_x_m, self._last_trace_y_m = rotate_point(
            self._last_trace_x_m,
            self._last_trace_y_m,
        )

    def _update_events(
        self,
        event_t: float,
        speed_mps: float,
        accel_mps2: float,
        curvature_m_inv: float,
    ) -> None:
        self._hard_accel_active, self._hard_accel_count, self._last_hard_accel_t = self._event_state(
            event_t,
            accel_mps2 >= TRIP_HARD_ACCEL_MPS2,
            accel_mps2 >= TRIP_HARD_ACCEL_MPS2 * 0.8,
            self._hard_accel_active,
            self._hard_accel_count,
            self._last_hard_accel_t,
        )
        self._hard_brake_active, self._hard_brake_count, self._last_hard_brake_t = self._event_state(
            event_t,
            accel_mps2 <= TRIP_HARD_BRAKE_MPS2,
            accel_mps2 <= TRIP_HARD_BRAKE_MPS2 * 0.8,
            self._hard_brake_active,
            self._hard_brake_count,
            self._last_hard_brake_t,
        )
        lateral_accel = abs(speed_mps * speed_mps * curvature_m_inv)
        hard_corner = speed_mps >= TRIP_CORNER_MIN_SPEED_MPS and lateral_accel >= TRIP_HARD_CORNER_MPS2
        corner_hold = speed_mps >= TRIP_CORNER_MIN_SPEED_MPS and lateral_accel >= TRIP_HARD_CORNER_MPS2 * 0.8
        self._hard_corner_active, self._hard_corner_count, self._last_hard_corner_t = self._event_state(
            event_t,
            hard_corner,
            corner_hold,
            self._hard_corner_active,
            self._hard_corner_count,
            self._last_hard_corner_t,
        )

    @staticmethod
    def _event_state(
        event_t: float,
        triggered: bool,
        held: bool,
        active: bool,
        count: int,
        last_event_t: float,
    ) -> tuple[bool, int, float]:
        if triggered and not active and event_t - last_event_t >= TRIP_EVENT_MIN_GAP_S:
            return True, count + 1, event_t
        if not held:
            active = False
        return active, count, last_event_t

    def _append_trace_point(self, event_t: float, speed_kph: float, force: bool = False) -> None:
        moved_m = math.hypot(self._x_m - self._last_trace_x_m, self._y_m - self._last_trace_y_m)
        if not force and (
            event_t - self._last_trace_t < TRIP_TRACE_MIN_PERIOD_S
            or moved_m < TRIP_TRACE_MIN_DISTANCE_M
        ):
            return
        self._flush_pending_trace_shift()
        self._trace_points.append(TripTracePoint(self._x_m, self._y_m, max(0.0, speed_kph)))
        self._last_trace_t = event_t
        self._last_trace_x_m = self._x_m
        self._last_trace_y_m = self._y_m
        self._trace_generation += 1

        keep_from = 0
        for index, point in enumerate(self._trace_points[:-1]):
            if math.hypot(point.x_m - self._x_m, point.y_m - self._y_m) > TRIP_TRACE_MAX_RADIUS_M:
                keep_from = index + 1
        if keep_from > 0:
            self._trace_points = self._trace_points[keep_from:]

        if len(self._trace_points) > TRIP_TRACE_MAX_POINTS:
            reduced = self._trace_points[::2]
            if reduced[-1] is not self._trace_points[-1]:
                reduced.append(self._trace_points[-1])
            self._trace_points = reduced

        extent_m = max(
            (math.hypot(point.x_m - self._x_m, point.y_m - self._y_m) for point in self._trace_points),
            default=0.0,
        )
        required_m = min(TRIP_TRACE_MAX_RADIUS_M, max(TRIP_TRACE_MIN_RADIUS_M, extent_m * 1.12))
        radius_m = min(
            TRIP_TRACE_MAX_RADIUS_M,
            math.ceil(required_m / TRIP_TRACE_RADIUS_QUANTUM_M) * TRIP_TRACE_RADIUS_QUANTUM_M,
        )
        self._trace_radius_m = max(self._trace_radius_m, radius_m)

    def _snapshot(self) -> TripReportState:
        if self._trace_snapshot_generation != self._trace_generation:
            self._trace_snapshot = tuple(self._trace_points)
            self._trace_snapshot_generation = self._trace_generation
        average_speed_kph = (
            self._distance_m / self._moving_time_s * 3.6
            if self._moving_time_s > 0.0
            else 0.0
        )
        auto_ratio_percent = (
            self._auto_distance_m / self._distance_m * 100.0
            if self._distance_m > 0.0
            else 0.0
        )
        return TripReportState(
            duration_s=self._duration_s,
            moving_time_s=self._moving_time_s,
            distance_m=self._distance_m,
            average_speed_kph=average_speed_kph,
            max_speed_kph=self._max_speed_kph,
            max_accel_mps2=self._max_accel_mps2,
            max_decel_mps2=self._max_decel_mps2,
            auto_ratio_percent=auto_ratio_percent,
            hard_accel_count=self._hard_accel_count,
            hard_brake_count=self._hard_brake_count,
            hard_corner_count=self._hard_corner_count,
            current_x_m=self._x_m,
            current_y_m=self._y_m,
            trace_radius_m=self._trace_radius_m,
            trace_points=self._trace_snapshot,
            trace_generation=self._trace_generation,
            heading_source=self._heading_source,
            gps_corrected=self._gps_corrected,
        )
