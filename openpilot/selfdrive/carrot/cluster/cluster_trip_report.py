from __future__ import annotations

import math

from cluster_models import TripReportState


TRIP_MOVING_SPEED_MPS = 0.3
TRIP_MAX_DT_S = 0.5
TRIP_HARD_ACCEL_MPS2 = 2.5
TRIP_HARD_BRAKE_MPS2 = -3.0
TRIP_HARD_CORNER_MPS2 = 3.0
TRIP_CORNER_MIN_SPEED_MPS = 5.5
TRIP_EVENT_MIN_GAP_S = 1.5


class TripReportTracker:
    """Low-cost trip statistics for the external HUD report."""

    def __init__(self) -> None:
        self._onroad: bool | None = None
        self.reset()

    def set_onroad(self, onroad: bool) -> TripReportState:
        onroad = bool(onroad)
        if onroad and self._onroad is False:
            self.reset()
        self._onroad = onroad
        return self._last_snapshot

    def reset(self) -> None:
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
        self._last_snapshot = TripReportState()

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
        if self._onroad is False:
            return self._last_snapshot
        if not math.isfinite(event_t):
            return self._last_snapshot
        speed_mps = max(0.0, speed_mps if math.isfinite(speed_mps) else 0.0)
        accel_mps2 = accel_mps2 if math.isfinite(accel_mps2) else 0.0

        if self._last_t is None:
            self._last_t = event_t
            self._last_snapshot = self._snapshot()
            return self._last_snapshot

        raw_dt = event_t - self._last_t
        if raw_dt <= 0.0:
            return self._last_snapshot
        self._last_t = event_t
        dt = min(raw_dt, TRIP_MAX_DT_S)
        self._duration_s += dt

        road_wheel_angle_rad = 0.0
        if steering_angle_deg is not None and math.isfinite(steering_angle_deg):
            road_wheel_angle_rad = math.radians(steering_angle_deg) / max(1.0, steer_ratio)
        curvature_m_inv = math.tan(road_wheel_angle_rad) / max(1.5, wheelbase_m)

        distance_step_m = speed_mps * dt
        self._distance_m += distance_step_m
        if speed_mps >= TRIP_MOVING_SPEED_MPS:
            self._moving_time_s += dt
            if enabled:
                self._auto_distance_m += distance_step_m
        self._max_speed_kph = max(self._max_speed_kph, speed_mps * 3.6)
        self._max_accel_mps2 = max(self._max_accel_mps2, accel_mps2)
        self._max_decel_mps2 = min(self._max_decel_mps2, accel_mps2)
        self._update_events(event_t, speed_mps, accel_mps2, curvature_m_inv)
        self._last_snapshot = self._snapshot()
        return self._last_snapshot

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

    def _snapshot(self) -> TripReportState:
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
        )
