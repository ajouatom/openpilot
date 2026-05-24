from __future__ import annotations

import argparse
from dataclasses import replace
import gc
import math
import random
import threading
import time
from pathlib import Path
from typing import Any

from cluster_config import CLUSTER_THEME_PARAM, normalize_cluster_theme_mode
from cluster_live import OpenpilotLiveSource
from cluster_route_replay import RouteReplaySource
from cluster_usb_display import TuringUsbDisplay
from cluster_ui import (
    DESIGN_HEIGHT,
    DESIGN_WIDTH,
    MAX_SPEED_KPH,
    SURROUND_MAX_PITCH_DEG,
    SURROUND_MAX_YAW_DEG,
    ClusterSimulator,
    ClusterUiRenderer,
    RandomInputSource,
    RouteOverlay,
    SimulatorInput,
)

DEFAULT_FPS = 0.0
TRIGGER_DEADZONE = 0.03
STEERING_DEADZONE = 0.06
VIEW_ROTATION_DEADZONE = 0.08
GAMEPAD_WARMUP_SECONDS = 0.6
LEFT_SIGNAL_BUTTONS = (4, 9, 13)
RIGHT_SIGNAL_BUTTONS = (5, 10, 14)
THEME_PARAM_POLL_SECONDS = 1.0


class ProfileReporter:
    def __init__(self, enabled: bool, interval_s: float) -> None:
        self.enabled = enabled
        self.interval_s = max(0.2, interval_s)
        self.samples: dict[str, list[float]] = {}
        self.last_report_time = time.perf_counter()
        self.report_frames = 0

    def add(self, name: str, milliseconds: float) -> None:
        if not self.enabled:
            return
        self.samples.setdefault(name, []).append(milliseconds)

    def add_elapsed(self, name: str, start_time: float) -> None:
        if self.enabled:
            self.add(name, (time.perf_counter() - start_time) * 1000.0)

    def add_samples(self, samples: tuple[tuple[str, float], ...]) -> None:
        if not self.enabled:
            return
        for name, milliseconds in samples:
            self.add(name, milliseconds)

    def frame_done(self) -> None:
        if self.enabled:
            self.report_frames += 1

    def maybe_report(self, now: float) -> None:
        if not self.enabled or now - self.last_report_time < self.interval_s:
            return

        elapsed = max(0.001, now - self.last_report_time)
        print(f"PROFILE {self.report_frames} frames / {elapsed:.1f}s", flush=True)
        ordered = sorted(
            self.samples.items(),
            key=lambda item: sum(item[1]) / max(1, len(item[1])),
            reverse=True,
        )
        for name, values in ordered:
            if not values:
                continue
            average = sum(values) / len(values)
            print(
                f"  {name:<42} avg={average:7.2f}ms "
                f"max={max(values):7.2f}ms last={values[-1]:7.2f}ms n={len(values)}",
                flush=True,
            )
        self.samples.clear()
        self.report_frames = 0
        self.last_report_time = now


class GcProfileHook:
    def __init__(self, profile: ProfileReporter) -> None:
        self.profile = profile
        self._starts: dict[int, float] = {}

    def __call__(self, phase: str, info: dict[str, Any]) -> None:
        generation = int(info.get("generation", -1))
        if phase == "start":
            self._starts[generation] = time.perf_counter()
            return
        if phase != "stop":
            return
        start_time = self._starts.pop(generation, None)
        if start_time is not None:
            self.profile.add(f"gc.gen{generation}", (time.perf_counter() - start_time) * 1000.0)


class ClusterThemeParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> str:
        if self._params is None:
            return "auto"
        try:
            return normalize_cluster_theme_mode(self._params.get_int(CLUSTER_THEME_PARAM))
        except Exception:
            return "auto"


def freeze_gc_after_init(profile: ProfileReporter) -> None:
    freeze = getattr(gc, "freeze", None)
    if freeze is None:
        return

    profile_stage = time.perf_counter()
    gc.collect(2)
    profile.add_elapsed("gc.freeze_init.collect", profile_stage)

    profile_stage = time.perf_counter()
    freeze()
    profile.add_elapsed("gc.freeze_init.freeze", profile_stage)


class AsyncJpegUsbPipeline:
    def __init__(self, usb_display: TuringUsbDisplay) -> None:
        self.usb_display = usb_display
        self._condition = threading.Condition()
        self._pending_rgba: tuple[bytes, int, int] | None = None
        self._closing = False
        self._error: BaseException | None = None
        self._samples: list[tuple[str, float]] = []
        self._thread = threading.Thread(target=self._run, name="cluster-usb-jpeg", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def submit_rgba(self, rgba: bytes, width: int, height: int) -> None:
        self.check_error()
        with self._condition:
            self._pending_rgba = (rgba, width, height)
            self._condition.notify()

    def profile_samples(self) -> tuple[tuple[str, float], ...]:
        with self._condition:
            samples = tuple(self._samples)
            self._samples.clear()
        return samples

    def check_error(self) -> None:
        if self._error is not None:
            raise RuntimeError("asynchronous USB JPEG pipeline failed") from self._error

    def close(self) -> None:
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join(timeout=3.0)

    def _add_sample(self, name: str, start_time: float) -> None:
        milliseconds = (time.perf_counter() - start_time) * 1000.0
        with self._condition:
            self._samples.append((name, milliseconds))

    def _add_samples(self, samples: tuple[tuple[str, float], ...]) -> None:
        if not samples:
            return
        with self._condition:
            self._samples.extend(samples)

    def _take_pending(self) -> tuple[bytes, int, int] | None:
        with self._condition:
            while self._pending_rgba is None and not self._closing:
                self._condition.wait(timeout=0.1)
            if self._pending_rgba is None:
                return None
            pending = self._pending_rgba
            self._pending_rgba = None
            return pending

    def _run(self) -> None:
        while True:
            pending = self._take_pending()
            if pending is None:
                return
            rgba, width, height = pending
            try:
                self.usb_display.clear_profile_samples()
                profile_stage = time.perf_counter()
                jpeg = self.usb_display.encode_jpeg(rgba, width, height)
                self._add_sample("usb_async.encode_jpeg", profile_stage)

                profile_stage = time.perf_counter()
                self.usb_display.send_jpeg(jpeg)
                self._add_sample("usb_async.send_jpeg", profile_stage)
                self._add_samples(self.usb_display.profile_samples())
                self.usb_display.clear_profile_samples()
            except BaseException as exc:
                self._error = exc
                return


def next_simulated_values(speed_kph: float, dt: float) -> tuple[float, float, float]:
    accel_mps2 = 1.8 * math.sin(time.monotonic() * 0.8) + random.uniform(-0.35, 0.35)
    next_speed = max(0.0, min(MAX_SPEED_KPH, speed_kph + accel_mps2 * dt * 3.6))
    steering = 0.42 * math.sin(time.monotonic() * 0.55)
    return next_speed, accel_mps2, steering


def route_overlay_for_mode(overlay: RouteOverlay | None, mode: str) -> RouteOverlay | None:
    if overlay is None or mode == "off":
        return None
    if mode == "compact":
        return replace(overlay, data_lines=overlay.data_lines[:4])
    return overlay


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


def run_demo(
    duration_seconds: float | None,
    target_fps: float,
    input_mode: str,
    output_mode: str,
    controller_index: int,
    width: int | None,
    height: int | None,
    usb_brightness: int,
    usb_display_fps: int,
    usb_codec: str,
    usb_jpeg_quality: int,
    usb_jpeg_encoder: str,
    usb_fast_write: bool,
    usb_wait_frame_ack: bool,
    usb_async: bool,
    usb_frame_drain_attempts: int,
    usb_frame_drain_timeout_ms: int,
    usb_fast_drain_attempts: int,
    usb_fast_drain_timeout_ms: int,
    route_path: Path,
    route_log: str,
    route_overlay_mode: str,
    route_loop: bool,
    route_replay_speed: float,
    route_start_segment: int | None,
    route_max_segments: int | None,
    live_include_can: bool,
    live_timeout_ms: int,
    profile_render: bool,
    profile_interval_s: float,
    render_msaa: bool,
    gc_freeze_init: bool,
    theme_mode: str | None,
) -> None:
    profile = ProfileReporter(profile_render, profile_interval_s)
    gc_hook = GcProfileHook(profile) if profile_render else None
    if gc_hook is not None:
        gc.callbacks.append(gc_hook)
    usb_display: TuringUsbDisplay | None = None
    usb_pipeline: AsyncJpegUsbPipeline | None = None
    if output_mode in ("usb", "both"):
        usb_display = TuringUsbDisplay(
            brightness=usb_brightness,
            display_fps=usb_display_fps,
            jpeg_quality=usb_jpeg_quality,
            jpeg_encoder=usb_jpeg_encoder,
            fast_write=usb_fast_write,
            wait_for_frame_ack=usb_wait_frame_ack,
            frame_drain_attempts=usb_frame_drain_attempts,
            frame_drain_timeout_ms=usb_frame_drain_timeout_ms,
            fast_frame_drain_attempts=usb_fast_drain_attempts,
            fast_frame_drain_timeout_ms=usb_fast_drain_timeout_ms,
        )
        usb_display.set_profile_enabled(profile_render)
        profile_stage = time.perf_counter()
        usb_display.open()
        profile.add_elapsed("usb.open", profile_stage)
        profile.add_samples(usb_display.profile_samples())
        usb_display.clear_profile_samples()
        if usb_async and usb_codec == "jpeg":
            usb_pipeline = AsyncJpegUsbPipeline(usb_display)
            usb_pipeline.start()

    frame_width = width or (usb_display.landscape_width if usb_display is not None else DESIGN_WIDTH)
    frame_height = height or (usb_display.landscape_height if usb_display is not None else DESIGN_HEIGHT)
    theme_override = normalize_cluster_theme_mode(theme_mode) if theme_mode is not None else None
    theme_param_reader = ClusterThemeParamReader() if theme_override is None else None
    active_theme_mode = theme_override or (theme_param_reader.read() if theme_param_reader is not None else "auto")
    renderer = ClusterUiRenderer(
        frame_width,
        frame_height,
        target_fps=max(0, int(round(target_fps))),
        msaa_4x=render_msaa,
        theme_mode=active_theme_mode,
    )
    renderer.set_profile_enabled(profile_render)
    simulator = ClusterSimulator() if input_mode in ("random", "gamepad") else None
    controller = DualSenseSimulator(controller_index) if input_mode == "gamepad" else None
    random_input = RandomInputSource() if input_mode == "random" else None
    live_source = OpenpilotLiveSource(include_can=live_include_can, timeout_ms=live_timeout_ms) if input_mode == "live" else None
    route_source = None
    if input_mode == "route":
        profile_stage = time.perf_counter()
        route_source = RouteReplaySource.load(route_path, route_log, route_start_segment, route_max_segments)
        profile.add_elapsed("source.route_load_initial", profile_stage)
    if route_source is not None:
        print(
            f"Loaded route replay buffer: {len(route_source.frames)} frames, "
            f"{route_source.duration:.1f}s from "
            f"{route_source.loaded_file_count}/{len(route_source.source_files)} {route_log} files"
        )
    start_time = time.perf_counter()
    last_frame_time = start_time
    last_report_time = start_time
    next_theme_param_read = start_time
    report_frames = 0
    frame_interval = 1.0 / target_fps if target_fps > 0 else 0.0

    try:
        renderer.open(hidden=output_mode == "usb")
        profile.add_samples(renderer.profile_samples())
        renderer.clear_profile_samples()
        if gc_freeze_init:
            freeze_gc_after_init(profile)
        while True:
            frame_start_time = time.perf_counter()
            renderer.clear_profile_samples()
            if usb_display is not None and usb_pipeline is None:
                usb_display.clear_profile_samples()
            if usb_pipeline is not None:
                usb_pipeline.check_error()
                profile.add_samples(usb_pipeline.profile_samples())
            if output_mode in ("window", "both") and renderer.should_close():
                break

            now = time.perf_counter()
            if theme_override is None and now >= next_theme_param_read:
                next_theme_mode = theme_param_reader.read() if theme_param_reader is not None else "auto"
                if next_theme_mode != renderer.theme_mode:
                    renderer.set_theme_mode(next_theme_mode)
                next_theme_param_read = now + THEME_PARAM_POLL_SECONDS
            if duration_seconds is not None and now - start_time >= duration_seconds:
                break

            dt = max(0.001, now - last_frame_time)
            last_frame_time = now
            if live_source is not None:
                profile_stage = time.perf_counter()
                state = live_source.update()
                state = replace(state, center_clock_text=time.strftime("%H:%M:%S"))
                source_status = live_source.status_text()
                profile.add_elapsed("source.live_update", profile_stage)
            elif route_source is not None:
                profile_stage = time.perf_counter()
                playback_seconds = (now - start_time) * route_replay_speed
                if route_source.is_finished(playback_seconds, route_loop):
                    break
                state = route_source.state_at(
                    playback_seconds,
                    route_loop,
                    include_overlay=route_overlay_mode != "off",
                )
                state = replace(state, route_overlay=route_overlay_for_mode(state.route_overlay, route_overlay_mode))
                source_status = route_source.status_text(playback_seconds, route_loop)
                profile.add_elapsed("source.route_update", profile_stage)
            elif controller is None:
                profile_stage = time.perf_counter()
                command = random_input.update(dt) if random_input is not None else SimulatorInput()
                source_status = (
                    f"random R2={command.throttle:.2f} "
                    f"L2={command.brake:.2f} LSX={command.steering:+.2f}"
                )
                if simulator is None:
                    raise RuntimeError("simulator is not available for random input")
                state = simulator.update(command, dt)
                profile.add_elapsed("source.random_update", profile_stage)
            else:
                profile_stage = time.perf_counter()
                command = controller.read_input()
                source_status = controller.status_text()
                if simulator is None:
                    raise RuntimeError("simulator is not available for gamepad input")
                state = simulator.update(command, dt)
                profile.add_elapsed("source.gamepad_update", profile_stage)

            if output_mode in ("window", "both"):
                profile_stage = time.perf_counter()
                renderer.render_frame(state)
                profile.add_elapsed("main.window_render_total", profile_stage)
            if usb_display is not None:
                if usb_codec == "jpeg":
                    profile_stage = time.perf_counter()
                    rgba, image_width, image_height = renderer.render_to_rgba_bytes(
                        state,
                        portrait_upload=True,
                    )
                    profile.add_elapsed("main.usb.render_rgba_total", profile_stage)

                    if usb_pipeline is not None:
                        profile_stage = time.perf_counter()
                        usb_pipeline.submit_rgba(rgba, image_width, image_height)
                        profile.add_elapsed("main.usb_async.submit_rgba", profile_stage)
                    else:
                        profile_stage = time.perf_counter()
                        jpeg = usb_display.encode_jpeg(rgba, image_width, image_height)
                        profile.add_elapsed("main.usb.encode_jpeg", profile_stage)

                        profile_stage = time.perf_counter()
                        usb_display.send_jpeg(jpeg)
                        profile.add_elapsed("main.usb.send_jpeg", profile_stage)
                else:
                    profile_stage = time.perf_counter()
                    png = renderer.render_to_png_bytes(state, portrait_upload=True)
                    profile.add_elapsed("main.usb.render_png_total", profile_stage)
                    profile_stage = time.perf_counter()
                    usb_display.send_png(png)
                    profile.add_elapsed("main.usb.send_png", profile_stage)
                if usb_pipeline is not None:
                    profile.add_samples(usb_pipeline.profile_samples())
                else:
                    profile.add_samples(usb_display.profile_samples())
            profile.add_samples(renderer.profile_samples())
            report_frames += 1
            profile.add_elapsed("main.frame_active", frame_start_time)

            if frame_interval > 0.0:
                elapsed = time.perf_counter() - frame_start_time
                remaining = frame_interval - elapsed
                if remaining > 0.0:
                    profile_stage = time.perf_counter()
                    time.sleep(remaining)
                    profile.add_elapsed("main.sleep", profile_stage)

            now = time.perf_counter()
            profile.add_elapsed("main.frame_total", frame_start_time)
            profile.frame_done()
            profile.maybe_report(now)
            if now - last_report_time >= 2.0:
                actual_fps = report_frames / (now - last_report_time)
                lane_status = state.lane_change or (
                    "keep" if state.lane_change_phase == "idle" else state.lane_change_phase
                )
                print(
                    f"Refresh {actual_fps:.1f} Hz | "
                    f"speed={state.speed_kph:5.1f} km/h "
                    f"accel={state.accel_mps2:+.2f} m/s^2 "
                    f"limit={state.speed_limit_kph} "
                    f"lane={lane_status}:{state.lane_change_progress:.2f} "
                    f"ego_offset={state.ego_lane_offset:+.2f} | "
                    f"output={output_mode}/{usb_codec if usb_display else 'screen'}"
                    f"{'-fast' if usb_display and usb_fast_write else ''} "
                    f"{'async ' if usb_pipeline is not None else ''}"
                    f"theme={renderer.theme_mode} "
                    f"view_yaw={state.surround_yaw_deg:+.0f} "
                    f"{source_status}"
                )
                report_frames = 0
                last_report_time = now
    finally:
        if gc_hook is not None:
            try:
                gc.callbacks.remove(gc_hook)
            except ValueError:
                pass
        if usb_pipeline is not None:
            usb_pipeline.close()
        if controller is not None:
            controller.close()
        if route_source is not None:
            route_source.close()
        if live_source is not None:
            live_source.close()
        renderer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fps",
        type=float,
        default=DEFAULT_FPS,
        help="Target refresh rate. Use 0 for uncapped/as-fast-as-possible. Default: 0.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Run for this many seconds. Omit to refresh until the window is closed.",
    )
    parser.add_argument(
        "--input",
        choices=("random", "gamepad", "route", "live"),
        default="random",
        help="Input source. Use --input live for live openpilot cereal data, or route to replay logs.",
    )
    parser.add_argument(
        "--output",
        choices=("usb", "window", "both"),
        default="usb",
        help="Render target. usb sends frames to the TURZX USB display. Default: usb.",
    )
    parser.add_argument(
        "--controller-index",
        type=int,
        default=0,
        help="pygame joystick index for the DualSense controller.",
    )
    parser.add_argument("--width", type=int, default=None)
    parser.add_argument("--height", type=int, default=None)
    parser.add_argument("--usb-brightness", type=int, default=80)
    parser.add_argument(
        "--usb-display-fps",
        type=int,
        default=0,
        help="Optional TURZX display frame-rate command. Default 0 skips it because some units do not ACK it.",
    )
    parser.add_argument("--usb-codec", choices=("jpeg", "png"), default="jpeg")
    parser.add_argument("--usb-jpeg-quality", type=int, default=68)
    parser.add_argument(
        "--usb-jpeg-encoder",
        choices=("auto", "pillow", "turbojpeg"),
        default="auto",
        help="JPEG encoder for USB output. auto tries turbojpeg first and falls back to Pillow.",
    )
    parser.add_argument(
        "--usb-fast",
        action="store_true",
        help="Use short pre-write USB input drain before frame uploads. Useful only after no-ACK USB output is stable.",
    )
    parser.add_argument(
        "--usb-wait-frame-ack",
        action="store_true",
        help="Wait for a TURZX response after each frame upload. Default skips ACK because some units never reply.",
    )
    parser.add_argument(
        "--usb-async",
        action="store_true",
        help="Encode and send JPEG USB frames on a background thread to overlap transport with the next render.",
    )
    parser.add_argument(
        "--usb-frame-drain-attempts",
        type=int,
        default=2,
        help="IN endpoint drain read attempts before normal no-ACK frame uploads. Default: 2.",
    )
    parser.add_argument(
        "--usb-frame-drain-timeout-ms",
        type=int,
        default=2,
        help="Per-read IN endpoint drain timeout before normal no-ACK frame uploads. Default: 2.",
    )
    parser.add_argument(
        "--usb-fast-drain-attempts",
        type=int,
        default=3,
        help="IN endpoint drain read attempts before --usb-fast no-ACK frame uploads. Default: 3.",
    )
    parser.add_argument(
        "--usb-fast-drain-timeout-ms",
        type=int,
        default=2,
        help="Per-read IN endpoint drain timeout before --usb-fast no-ACK frame uploads. Default: 2.",
    )
    parser.add_argument(
        "--route",
        type=Path,
        default=Path("route"),
        help="Route directory or log file to replay when --input route is selected.",
    )
    parser.add_argument(
        "--route-log",
        choices=("qlog", "rlog"),
        default="rlog",
        help="Route log type to read. rlog has full corner radar data; qlog is faster but downsampled.",
    )
    parser.add_argument(
        "--route-overlay",
        choices=("compact", "full", "off"),
        default="compact",
        help="Route replay debug overlay. Default compact shows the replay camera/data panel; use off for performance tests.",
    )
    parser.add_argument(
        "--theme",
        choices=("auto", "dark", "light"),
        default=None,
        help=f"HUD theme override. Default reads {CLUSTER_THEME_PARAM}: 0 auto, 1 dark, 2 light.",
    )
    parser.add_argument(
        "--route-loop",
        action="store_true",
        help="Loop route replay instead of stopping at the end.",
    )
    parser.add_argument(
        "--route-replay-speed",
        type=float,
        default=1.0,
        help="Route playback speed multiplier.",
    )
    parser.add_argument(
        "--route-start-segment",
        type=int,
        default=None,
        help="First segment index to replay.",
    )
    parser.add_argument(
        "--route-max-segments",
        type=int,
        default=None,
        help="Maximum number of route segments to replay.",
    )
    parser.add_argument(
        "--live-no-can",
        action="store_true",
        help="Disable live CAN subscription. This keeps radarState/modelV2/liveTracks data but skips direct raw CAN-FD parsing.",
    )
    parser.add_argument(
        "--live-timeout-ms",
        type=int,
        default=0,
        help="SubMaster update timeout for --input live. Default 0 keeps rendering responsive.",
    )
    parser.add_argument(
        "--profile-render",
        action="store_true",
        help="Log render, GPU readback, USB encode/send, and input source timings.",
    )
    parser.add_argument(
        "--profile-interval",
        type=float,
        default=2.0,
        help="Seconds between --profile-render timing summaries. Default: 2.0.",
    )
    parser.add_argument(
        "--render-msaa",
        action="store_true",
        help="Enable raylib 4x MSAA config hint. Default off for maximum SD845 throughput.",
    )
    parser.add_argument(
        "--no-gc-freeze",
        action="store_true",
        help="Disable post-init gc.freeze(). Default enabled to avoid long gen2 pauses during USB rendering.",
    )
    args = parser.parse_args()
    if args.fps < 0:
        parser.error("--fps must be 0 or greater")
    if (args.width is not None and args.width <= 0) or (args.height is not None and args.height <= 0):
        parser.error("--width and --height must be greater than 0")
    if not 0 <= args.usb_brightness <= 100:
        parser.error("--usb-brightness must be between 0 and 100")
    if not 0 <= args.usb_display_fps <= 255:
        parser.error("--usb-display-fps must be between 0 and 255")
    if not 1 <= args.usb_jpeg_quality <= 95:
        parser.error("--usb-jpeg-quality must be between 1 and 95")
    if args.usb_async and args.usb_codec != "jpeg":
        parser.error("--usb-async only supports --usb-codec jpeg")
    if args.usb_frame_drain_attempts < 0 or args.usb_fast_drain_attempts < 0:
        parser.error("USB drain attempts must be 0 or greater")
    if args.usb_frame_drain_timeout_ms < 0 or args.usb_fast_drain_timeout_ms < 0:
        parser.error("USB drain timeouts must be 0 or greater")
    if args.input == "route" and args.route_replay_speed <= 0:
        parser.error("--route-replay-speed must be greater than 0")
    if args.route_start_segment is not None and args.route_start_segment < 0:
        parser.error("--route-start-segment must be 0 or greater")
    if args.route_max_segments is not None and args.route_max_segments <= 0:
        parser.error("--route-max-segments must be greater than 0")
    if args.profile_interval <= 0:
        parser.error("--profile-interval must be greater than 0")
    return args


def main() -> None:
    args = parse_args()
    fps_text = "uncapped" if args.fps == 0 else f"{args.fps:.1f} Hz"
    size_text = (
        f"{args.width or 'device'}x{args.height or 'device'}"
        if args.output in ("usb", "both")
        else f"{args.width or DESIGN_WIDTH}x{args.height or DESIGN_HEIGHT}"
    )
    print(
        f"Refreshing native raylib cluster UI at {fps_text} "
        f"input={args.input} output={args.output}: {size_text}"
    )
    try:
        run_demo(
            args.duration,
            args.fps,
            args.input,
            args.output,
            args.controller_index,
            args.width,
            args.height,
            args.usb_brightness,
            args.usb_display_fps,
            args.usb_codec,
            args.usb_jpeg_quality,
            args.usb_jpeg_encoder,
            args.usb_fast,
            args.usb_wait_frame_ack,
            args.usb_async,
            args.usb_frame_drain_attempts,
            args.usb_frame_drain_timeout_ms,
            args.usb_fast_drain_attempts,
            args.usb_fast_drain_timeout_ms,
            args.route,
            args.route_log,
            args.route_overlay,
            args.route_loop,
            args.route_replay_speed,
            args.route_start_segment,
            args.route_max_segments,
            not args.live_no_can,
            args.live_timeout_ms,
            args.profile_render,
            args.profile_interval,
            args.render_msaa,
            not args.no_gc_freeze,
            args.theme,
        )
    except KeyboardInterrupt:
        print("\nStopped.")
    except RuntimeError as exc:
        raise SystemExit(f"Error: {exc}") from exc


if __name__ == "__main__":
    main()
