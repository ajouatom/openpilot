from __future__ import annotations

import time
from typing import Any

from cluster_config import SURROUND_MAX_PITCH_DEG, SURROUND_MAX_YAW_DEG
from cluster_models import SimulatorInput


TRIGGER_DEADZONE = 0.03
STEERING_DEADZONE = 0.06
VIEW_ROTATION_DEADZONE = 0.08
GAMEPAD_WARMUP_SECONDS = 0.6
LEFT_SIGNAL_BUTTONS = (4, 9, 13)
RIGHT_SIGNAL_BUTTONS = (5, 10, 14)


def normalize_signed_axis(axis_value: float | int) -> float:
    value = float(axis_value)
    if value < -1.0 or value > 1.0:
        value = value / 32767.0 if value >= 0 else value / 32768.0
    return max(-1.0, min(1.0, value))


def normalize_trigger_axis(axis_value: float | int) -> float:
    value = float(axis_value)
    if value < -1.0 or value > 1.0:
        if value < 0.0:
            value = (value + 32768.0) / 65535.0
        else:
            value = value / 32767.0
    elif value < 0.0:
        value = (value + 1.0) * 0.5

    if value < TRIGGER_DEADZONE:
        return 0.0
    return max(0.0, min(1.0, value))


def normalize_stick(axis_value: float) -> float:
    if abs(axis_value) < STEERING_DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, axis_value))


def normalize_view_axis(axis_value: float) -> float:
    if abs(axis_value) < VIEW_ROTATION_DEADZONE:
        return 0.0
    return max(-1.0, min(1.0, axis_value))


class DualSenseSimulator:
    def __init__(self, controller_index: int):
        import pygame

        self.pygame: Any = pygame
        self.controller: Any | None = None
        self.joystick: Any | None = None
        self.using_controller_api = False

        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count == 0:
            pygame.quit()
            raise SystemExit("No gamepad found. Use --input random to run without a controller.")

        try:
            from pygame._sdl2 import controller as sdl_controller

            sdl_controller.init()
            if sdl_controller.is_controller(controller_index):
                self.controller = sdl_controller.Controller(controller_index)
                self.using_controller_api = True
        except Exception:
            self.controller = None
            self.using_controller_api = False

        if self.controller is None:
            self.joystick = pygame.joystick.Joystick(controller_index)
            self.joystick.init()

        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0
        self.view_rotate_x = 0.0
        self.view_rotate_y = 0.0
        self.left_signal_requested = False
        self.right_signal_requested = False
        self.warmup_until = time.perf_counter() + GAMEPAD_WARMUP_SECONDS

    def close(self) -> None:
        if self.controller is not None:
            self.controller.quit()
        if self.joystick is not None:
            self.joystick.quit()
        self.pygame.quit()

    def _button_down(self, button_indexes: tuple[int, ...]) -> bool:
        if self.using_controller_api:
            return self._controller_button_down(button_indexes)
        return any(index < self.joystick.get_numbuttons() and self.joystick.get_button(index) for index in button_indexes)

    def _controller_button_down(self, button_indexes: tuple[int, ...]) -> bool:
        return any(self.controller.get_button(index) for index in button_indexes)

    def _hat_left_down(self) -> bool:
        if self.joystick is None or self.joystick.get_numhats() == 0:
            return False
        return self.joystick.get_hat(0)[0] < 0

    def _hat_right_down(self) -> bool:
        if self.joystick is None or self.joystick.get_numhats() == 0:
            return False
        return self.joystick.get_hat(0)[0] > 0

    def _read_motion(self) -> tuple[float, float, float]:
        if self.using_controller_api:
            steering = normalize_stick(
                normalize_signed_axis(self.controller.get_axis(self.pygame.CONTROLLER_AXIS_LEFTX))
            )
            brake = normalize_trigger_axis(
                self.controller.get_axis(self.pygame.CONTROLLER_AXIS_TRIGGERLEFT)
            )
            throttle = normalize_trigger_axis(
                self.controller.get_axis(self.pygame.CONTROLLER_AXIS_TRIGGERRIGHT)
            )
            return throttle, brake, steering

        steering = normalize_stick(
            normalize_signed_axis(self.joystick.get_axis(0) if self.joystick.get_numaxes() > 0 else 0.0)
        )
        brake = normalize_trigger_axis(
            self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else 0.0
        )
        throttle = normalize_trigger_axis(
            self.joystick.get_axis(5) if self.joystick.get_numaxes() > 5 else 0.0
        )
        return throttle, brake, steering

    def _read_view_rotation(self) -> tuple[float, float]:
        if self.using_controller_api:
            x_axis = normalize_view_axis(
                normalize_signed_axis(self.controller.get_axis(self.pygame.CONTROLLER_AXIS_RIGHTX))
            )
            y_axis = normalize_view_axis(
                normalize_signed_axis(self.controller.get_axis(self.pygame.CONTROLLER_AXIS_RIGHTY))
            )
            return x_axis, y_axis

        if self.joystick is None:
            return 0.0, 0.0
        x_axis = normalize_view_axis(
            normalize_signed_axis(self.joystick.get_axis(2) if self.joystick.get_numaxes() > 2 else 0.0)
        )
        y_axis = normalize_view_axis(
            normalize_signed_axis(self.joystick.get_axis(3) if self.joystick.get_numaxes() > 3 else 0.0)
        )
        return x_axis, y_axis

    def _read_signal_buttons(self) -> tuple[bool, bool]:
        if self.using_controller_api:
            left = any(
                self.controller.get_button(button)
                for button in (
                    self.pygame.CONTROLLER_BUTTON_DPAD_LEFT,
                    self.pygame.CONTROLLER_BUTTON_LEFTSHOULDER,
                )
            )
            right = any(
                self.controller.get_button(button)
                for button in (
                    self.pygame.CONTROLLER_BUTTON_DPAD_RIGHT,
                    self.pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,
                )
            )
            return left, right

        left = self._button_down(LEFT_SIGNAL_BUTTONS) or self._hat_left_down()
        right = self._button_down(RIGHT_SIGNAL_BUTTONS) or self._hat_right_down()
        return left, right

    def read_input(self) -> SimulatorInput:
        self.pygame.event.pump()
        self.throttle, self.brake, self.steering = self._read_motion()
        self.view_rotate_x, self.view_rotate_y = self._read_view_rotation()
        self.left_signal_requested, self.right_signal_requested = self._read_signal_buttons()
        if time.perf_counter() < self.warmup_until:
            self.throttle = 0.0
            self.brake = 0.0
            self.steering = 0.0
            self.view_rotate_x = 0.0
            self.view_rotate_y = 0.0
            self.left_signal_requested = False
            self.right_signal_requested = False
        return SimulatorInput(
            throttle=self.throttle,
            brake=self.brake,
            steering=self.steering,
            surround_yaw_deg=self.view_rotate_x * SURROUND_MAX_YAW_DEG,
            surround_pitch_deg=-self.view_rotate_y * SURROUND_MAX_PITCH_DEG,
            surround_view_active=self.view_rotate_x != 0.0 or self.view_rotate_y != 0.0,
            left_signal_requested=self.left_signal_requested,
            right_signal_requested=self.right_signal_requested,
        )

    def status_text(self) -> str:
        left = "L" if self.left_signal_requested else "-"
        right = "R" if self.right_signal_requested else "-"
        return (
            f"R2={self.throttle:.2f} L2={self.brake:.2f} "
            f"LSX={self.steering:+.2f} RS={self.view_rotate_x:+.2f},{self.view_rotate_y:+.2f} "
            f"SIG={left}{right}"
        )
