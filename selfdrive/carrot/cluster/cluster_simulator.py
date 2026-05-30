from __future__ import annotations

import math

from cluster_config import (
    BLUE,
    BLUE_SOFT,
    COAST_DECEL_MPS2,
    CONTROLLER_ACCEL_MPS2,
    CONTROLLER_BRAKE_MPS2,
    DEFAULT_LANE_WIDTH_M,
    DRAG_DECEL_PER_MPS,
    LANE_CHANGE_MAX_SECONDS,
    LANE_CHANGE_MIN_SECONDS,
    MAX_ACCEL_MPS2,
    MAX_SPEED_KPH,
    MAX_STEERING_ANGLE_DEG,
    MODEL_DIRECT_LANE_RECENTER_SECONDS,
    SURROUND_MAX_PITCH_DEG,
    SURROUND_MAX_YAW_DEG,
    SURROUND_VIEW_SMOOTH_SECONDS,
    TURN_SIGNAL_SECONDS,
)
from cluster_models import ClusterUiState, LaneMarking, SimulatorInput
from cluster_utils import clamp, smoothstep


class ClusterSimulator:
    def __init__(self) -> None:
        self.elapsed = 0.0
        self.speed_kph = 0.0
        self.accel_mps2 = 0.0
        self.steering = 0.0
        self.left_signal_until = -999.0
        self.right_signal_until = -999.0
        self.lane_change_direction: str | None = None
        self.lane_change_phase = "idle"
        self.lane_change_elapsed = 0.0
        self.lane_change_progress = 0.0
        self.lane_change_recenter_start_progress = 1.0
        self.active_lane_position = 0.0
        self.ego_lane_position = 0.0
        self.view_lane_position = 0.0
        self.target_lane_position = 0.0
        self.lane_width_m = DEFAULT_LANE_WIDTH_M
        self.camera_lane_center_offset_m: float | None = None
        self.steering_angle_deg: float | None = None
        self.surround_yaw_deg = 0.0
        self.surround_pitch_deg = 0.0
        self.surround_view_active = False

    def update(self, command: SimulatorInput, dt: float) -> ClusterUiState:
        dt = clamp(dt, 0.001, 0.25)
        self.elapsed += dt
        self._update_steering(command)
        self._update_motion(command, dt)
        self._update_signals(command)
        self._update_lane_change(dt)
        self._apply_camera_lane_model(command)
        self._update_surround_view(command, dt)

        speed_limit_kph = self._speed_limit_for_current_road()
        cruise_kph = min(int(speed_limit_kph + 12), int(MAX_SPEED_KPH))
        lanes = self._lanes_for_current_state()
        left_signal = self.elapsed < self.left_signal_until
        right_signal = self.elapsed < self.right_signal_until
        highlight_active = self.lane_change_direction is not None and self.lane_change_phase in ("preparing", "changing")
        highlight_lane = self.lane_change_direction if highlight_active else None
        highlight_lane_offset = (
            self.target_lane_position - self.view_lane_position
            if highlight_active
            else None
        )

        return ClusterUiState(
            speed_kph=self.speed_kph,
            accel_mps2=self.accel_mps2,
            steering=self.steering,
            speed_limit_kph=speed_limit_kph,
            speed_limit_source="sim",
            cruise_kph=cruise_kph,
            cruise_display_state="engaged",
            gear_text="D",
            cruise_gap=3,
            lfa_active=True,
            left_signal=left_signal,
            right_signal=right_signal,
            left_blindspot=False,
            right_blindspot=False,
            lane_change=self.lane_change_direction,
            lane_change_phase=self.lane_change_phase,
            lane_change_progress=self.lane_change_progress,
            highlight_lane=highlight_lane,
            highlight_lane_offset=highlight_lane_offset,
            ego_lane_offset=self.ego_lane_position - self.view_lane_position,
            road_view_lane_position=self.view_lane_position,
            camera_lane_center_offset_m=self.camera_lane_center_offset_m,
            lane_width_m=self.lane_width_m,
            steering_angle_deg=self.steering_angle_deg,
            surround_yaw_deg=self.surround_yaw_deg,
            surround_pitch_deg=self.surround_pitch_deg,
            surround_view_active=self.surround_view_active,
            lanes=lanes,
            throttle=command.throttle,
            brake=command.brake,
        )

    def _update_steering(self, command: SimulatorInput) -> None:
        if command.steering_angle_deg is None:
            target_steering = clamp(command.steering, -1.0, 1.0)
            self.steering_angle_deg = target_steering * MAX_STEERING_ANGLE_DEG
        else:
            self.steering_angle_deg = clamp(
                command.steering_angle_deg,
                -MAX_STEERING_ANGLE_DEG,
                MAX_STEERING_ANGLE_DEG,
            )
            target_steering = self.steering_angle_deg / MAX_STEERING_ANGLE_DEG

        self.steering = self.steering * 0.72 + target_steering * 0.28

    def _update_motion(self, command: SimulatorInput, dt: float) -> None:
        throttle = clamp(command.throttle, 0.0, 1.0)
        brake = clamp(command.brake, 0.0, 1.0)
        speed_mps = self.speed_kph / 3.6
        accel_mps2 = (
            throttle * CONTROLLER_ACCEL_MPS2
            - brake * CONTROLLER_BRAKE_MPS2
            - speed_mps * DRAG_DECEL_PER_MPS
        )
        if throttle == 0.0 and brake == 0.0 and speed_mps > 0.0:
            accel_mps2 -= COAST_DECEL_MPS2

        next_speed_kph = self.speed_kph + accel_mps2 * dt * 3.6
        if next_speed_kph <= 0.0:
            next_speed_kph = 0.0
            accel_mps2 = max(0.0, accel_mps2)

        self.speed_kph = clamp(next_speed_kph, 0.0, MAX_SPEED_KPH)
        self.accel_mps2 = clamp(accel_mps2, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2)

    def _update_signals(self, command: SimulatorInput) -> None:
        if command.left_signal_requested:
            self._start_signal("left")
        if command.right_signal_requested:
            self._start_signal("right")

    def _start_signal(self, direction: str) -> None:
        if direction == "left":
            self.left_signal_until = self.elapsed + TURN_SIGNAL_SECONDS
            self.right_signal_until = min(self.right_signal_until, self.elapsed)
        else:
            self.right_signal_until = self.elapsed + TURN_SIGNAL_SECONDS
            self.left_signal_until = min(self.left_signal_until, self.elapsed)

        self.lane_change_direction = direction
        self.lane_change_phase = "changing"
        self.lane_change_elapsed = 0.0
        self.lane_change_progress = 0.0
        self.lane_change_recenter_start_progress = 1.0
        direction_sign = -1.0 if direction == "left" else 1.0
        self.target_lane_position = self.active_lane_position + direction_sign

    def _update_lane_change(self, dt: float) -> None:
        if self.lane_change_phase == "idle":
            self.lane_change_progress = 0.0
            self.ego_lane_position += (self.active_lane_position - self.ego_lane_position) * min(1.0, dt / 0.8)
            self.view_lane_position += (self.active_lane_position - self.view_lane_position) * min(1.0, dt / 0.8)
            return

        if self.lane_change_phase == "changing":
            self.lane_change_elapsed += dt
            self.lane_change_progress = clamp(
                self.lane_change_elapsed / self._lane_change_duration_seconds(),
                0.0,
                1.0,
            )
            direction_sign = self._lane_change_direction_sign()
            self.ego_lane_position = (
                self.active_lane_position
                + direction_sign * self.lane_change_progress
            )

            if self.lane_change_progress >= 1.0:
                self.active_lane_position = self.target_lane_position
                self.ego_lane_position = self.target_lane_position
                self.lane_change_phase = "recentering"
                self.lane_change_elapsed = 0.0
                self.lane_change_recenter_start_progress = 1.0
            return

        if self.lane_change_phase == "recentering":
            self.lane_change_elapsed += dt
            self.lane_change_progress = clamp(
                self.lane_change_elapsed / MODEL_DIRECT_LANE_RECENTER_SECONDS,
                0.0,
                1.0,
            )
            direction_sign = self._lane_change_direction_sign()
            recenter_blend = smoothstep(self.lane_change_progress)
            start_offset = direction_sign * smoothstep(self.lane_change_recenter_start_progress)
            self.view_lane_position = self.active_lane_position - start_offset * (1.0 - recenter_blend)
            self.ego_lane_position = self.active_lane_position
            if self.lane_change_progress >= 1.0:
                self.view_lane_position = self.active_lane_position
                self.lane_change_phase = "idle"
                self.lane_change_progress = 0.0
                self.lane_change_recenter_start_progress = 1.0
                self.lane_change_direction = None

    def _lane_change_duration_seconds(self) -> float:
        return clamp(
            LANE_CHANGE_MAX_SECONDS - self.speed_kph * 0.032,
            LANE_CHANGE_MIN_SECONDS,
            LANE_CHANGE_MAX_SECONDS,
        )

    def _lane_change_direction_sign(self) -> float:
        return -1.0 if self.lane_change_direction == "left" else 1.0

    def _apply_camera_lane_model(self, command: SimulatorInput) -> None:
        self.lane_width_m = max(2.4, min(4.6, command.camera_lane_width_m))
        self.camera_lane_center_offset_m = command.camera_lane_center_offset_m
        if command.camera_lane_center_offset_m is None:
            return

        camera_lane_offset = clamp(
            command.camera_lane_center_offset_m / self.lane_width_m,
            -1.4,
            1.4,
        )
        observed_ego_position = self.active_lane_position - camera_lane_offset
        if self.lane_change_direction is not None:
            low = min(self.active_lane_position, self.target_lane_position) - 0.25
            high = max(self.active_lane_position, self.target_lane_position) + 0.25
            observed_ego_position = clamp(observed_ego_position, low, high)

        self.ego_lane_position = self.ego_lane_position * 0.45 + observed_ego_position * 0.55
        if self.lane_change_phase == "changing" and self.target_lane_position != self.active_lane_position:
            measured_progress = abs(
                (self.ego_lane_position - self.active_lane_position)
                / (self.target_lane_position - self.active_lane_position)
            )
            self.lane_change_progress = max(self.lane_change_progress, clamp(measured_progress, 0.0, 1.0))
            if self.lane_change_progress >= 0.98:
                self.active_lane_position = self.target_lane_position
                self.ego_lane_position = self.target_lane_position
                self.lane_change_phase = "recentering"
                self.lane_change_elapsed = 0.0
                self.lane_change_recenter_start_progress = clamp(self.lane_change_progress, 0.0, 1.0)

    def _update_surround_view(self, command: SimulatorInput, dt: float) -> None:
        target_yaw = clamp(command.surround_yaw_deg, -SURROUND_MAX_YAW_DEG, SURROUND_MAX_YAW_DEG)
        target_pitch = clamp(command.surround_pitch_deg, -SURROUND_MAX_PITCH_DEG, SURROUND_MAX_PITCH_DEG)
        if not command.surround_view_active:
            target_yaw = 0.0
            target_pitch = 0.0

        alpha = min(1.0, dt / SURROUND_VIEW_SMOOTH_SECONDS)
        self.surround_yaw_deg += (target_yaw - self.surround_yaw_deg) * alpha
        self.surround_pitch_deg += (target_pitch - self.surround_pitch_deg) * alpha
        self.surround_view_active = (
            command.surround_view_active
            or abs(self.surround_yaw_deg) > 0.4
            or abs(self.surround_pitch_deg) > 0.4
        )

    def _speed_limit_for_current_road(self) -> int:
        if self.speed_kph < 55.0:
            return 50
        if self.speed_kph < 95.0:
            return 80
        return 100

    def _lanes_for_current_state(self) -> tuple[LaneMarking, ...]:
        lane_center = self.active_lane_position
        left_inner = BLUE
        right_inner = BLUE
        markings: list[LaneMarking] = []
        if self.lane_change_direction == "left":
            left_inner = BLUE
            markings.append(
                LaneMarking(lane_center - 1.5 - self.view_lane_position, BLUE_SOFT, "solid", width=5)
            )
        elif self.lane_change_direction == "right":
            right_inner = BLUE

        markings.extend(
            (
                LaneMarking(lane_center - 0.5 - self.view_lane_position, left_inner, "solid", width=7),
                LaneMarking(lane_center + 0.5 - self.view_lane_position, right_inner, "solid", width=7),
            )
        )
        if self.lane_change_direction == "right":
            markings.append(
                LaneMarking(lane_center + 1.5 - self.view_lane_position, BLUE_SOFT, "dashed", width=5)
            )
        return tuple(markings)


class RandomInputSource:
    def __init__(self) -> None:
        self.elapsed = 0.0
        self.next_signal_at = 4.0
        self.next_signal_left = True

    def update(self, dt: float) -> SimulatorInput:
        self.elapsed += dt
        throttle = 0.28 + 0.26 * math.sin(self.elapsed * 0.37)
        brake = 0.0
        if math.sin(self.elapsed * 0.21) < -0.72:
            throttle = 0.0
            brake = 0.34 + 0.28 * abs(math.sin(self.elapsed * 0.78))

        signal_left = False
        signal_right = False
        if self.elapsed >= self.next_signal_at:
            signal_left = self.next_signal_left
            signal_right = not self.next_signal_left
            self.next_signal_left = not self.next_signal_left
            self.next_signal_at = self.elapsed + 8.0

        return SimulatorInput(
            throttle=clamp(throttle, 0.0, 1.0),
            brake=clamp(brake, 0.0, 1.0),
            steering=0.38 * math.sin(self.elapsed * 0.42),
            left_signal_requested=signal_left,
            right_signal_requested=signal_right,
        )
