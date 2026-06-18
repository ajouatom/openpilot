from __future__ import annotations

import argparse
import gc
from dataclasses import replace
import signal
import sys
import threading
import time
from pathlib import Path

from cluster_config import (
    CLUSTER_BRIGHTNESS_PARAM,
    CLUSTER_CAMERA_VIEW_MODE_PARAM,
    CLUSTER_ENCODER_AUTO,
    CLUSTER_ENCODER_HARDWARE,
    CLUSTER_ENCODER_JPEG,
    CLUSTER_ENCODER_PARAM,
    CLUSTER_ENCODER_SOFTWARE,
    CLUSTER_CORE_MODE_PARAM,
    CLUSTER_HUD_DEBUG_PARAM,
    CLUSTER_HUD_PARAM,
    CLUSTER_LIVE_FPS_PARAM,
    CLUSTER_PRIORITY_PARAM,
    CLUSTER_RADAR_DISPLAY_PARAM,
    CLUSTER_RADAR_INFO_PARAM,
    CLUSTER_RADAR_SOURCE_COLOR_PARAM,
    CLUSTER_SCREEN_MODE_DEBUG,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
    CLUSTER_SCREEN_MODE_NAVI_DEBUG,
    CLUSTER_SCREEN_MODE_PARAM,
    CLUSTER_THEME_PARAM,
    DESIGN_HEIGHT,
    DESIGN_WIDTH,
    normalize_cluster_brightness_percent,
    normalize_cluster_camera_view_mode,
    normalize_cluster_core_mode,
    normalize_cluster_encoder_mode,
    normalize_cluster_live_fps,
    normalize_cluster_priority,
    normalize_cluster_radar_display_mode,
    normalize_cluster_radar_info_mode,
    normalize_cluster_radar_source_color_mode,
    normalize_cluster_screen_mode,
    normalize_cluster_theme_mode,
)
from cluster_gamepad import DualSenseSimulator
from cluster_git_status import GitBranchStatusProvider
from cluster_h264_pipeline import (
    DEFAULT_H264_DEVICE,
    DEFAULT_H264_ENCODER_ALIGN,
    DEFAULT_H264_FFMPEG,
    DEFAULT_H264_FFMPEG_ENCODER,
    DEFAULT_H264_LIBRARY,
    DEFAULT_H264_RATE_CONTROL,
    DEFAULT_H264_SLICE_MAX_BYTES,
    H264UsbPipeline,
    NATIVE_RATE_CONTROLS,
)
from cluster_live import OpenpilotLiveSource
from cluster_models import RouteOverlay, SimulatorInput
from cluster_profile import GcProfileHook, ProfileReporter, freeze_gc_after_init
from cluster_renderer import ClusterUiRenderer
from cluster_route_replay import RouteReplaySource
from cluster_simulator import ClusterSimulator, RandomInputSource
from cluster_system_monitor import ClusterProcessCoreUsageSampler
from cluster_usb_display import TuringUsbDisplay, product_id_for_hud_mode
from cluster_usb_pipeline import AsyncJpegUsbPipeline

DEFAULT_FPS = 0.0
DEFAULT_USB_BRIGHTNESS = 80
DEFAULT_H264_BITRATE = "auto"
DEFAULT_H264_GOP = 1
H264_AUTO_BITRATE_BITS_PER_FPS = 234_000
H264_AUTO_BITRATE_MIN_BPS = 1_000_000
H264_AUTO_BITRATE_MAX_BPS = 7_000_000
DEFAULT_H264_DIMENSION_ALIGN = 1
THEME_PARAM_POLL_SECONDS = 1.0
FPS_PARAM_POLL_SECONDS = 1.0
BRIGHTNESS_PARAM_POLL_SECONDS = 1.0
SCREEN_MODE_PARAM_POLL_SECONDS = 1.0
CAMERA_VIEW_PARAM_POLL_SECONDS = 1.0
RADAR_PARAM_POLL_SECONDS = 1.0
HUD_MODE_PARAM_POLL_SECONDS = 1.0


def live_debug_panel_enabled(screen_mode: int) -> bool:
    return screen_mode == CLUSTER_SCREEN_MODE_DEBUG


def live_debug_plot_enabled(screen_mode: int) -> bool:
    return screen_mode in (CLUSTER_SCREEN_MODE_DEBUG_GRAPH, CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT)


def live_navi_debug_enabled(screen_mode: int) -> bool:
    return screen_mode == CLUSTER_SCREEN_MODE_NAVI_DEBUG


def resolved_usb_display_fps(
    requested_fps: int | None,
    usb_codec: str,
    target_fps: float,
    h264_fps: int,
) -> int:
    if requested_fps is not None:
        return int(requested_fps)
    if usb_codec != "h264":
        return 0
    source_fps = target_fps if target_fps > 0 else float(h264_fps)
    return int(max(1, min(255, round(source_fps))))


def resolved_usb_h264_bitrate(requested_bitrate: str, target_fps: float, h264_fps: int) -> str:
    text = requested_bitrate.strip()
    if text.lower() != "auto":
        return text
    source_fps = int(max(1, round(target_fps if target_fps > 0 else float(h264_fps))))
    bitrate_bps = source_fps * H264_AUTO_BITRATE_BITS_PER_FPS
    bitrate_bps = int(max(H264_AUTO_BITRATE_MIN_BPS, min(H264_AUTO_BITRATE_MAX_BPS, bitrate_bps)))
    if bitrate_bps % 1_000_000 == 0:
        return f"{bitrate_bps // 1_000_000}M"
    if bitrate_bps % 1_000 == 0:
        return f"{bitrate_bps // 1_000}k"
    return str(bitrate_bps)


def resolved_h264_encoder_fps(target_fps: float, h264_fps: int) -> int:
    return max(1, int(round(target_fps if target_fps > 0 else h264_fps)))


def option_present(argv: list[str], option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in argv)


def apply_cluster_encoder_param(args: argparse.Namespace) -> str:
    if args.output not in ("usb", "both"):
        return "disabled"
    if args.usb_codec_from_cli:
        return "--usb-codec"

    encoder_mode = ClusterHudEncoderParamReader().read()
    if encoder_mode is None:
        return "default"
    encoder_mode = normalize_cluster_encoder_mode(encoder_mode)

    if encoder_mode == CLUSTER_ENCODER_JPEG:
        args.usb_codec = "jpeg"
    elif encoder_mode in (CLUSTER_ENCODER_AUTO, CLUSTER_ENCODER_HARDWARE, CLUSTER_ENCODER_SOFTWARE):
        args.usb_codec = "h264"
        if not args.usb_h264_backend_from_cli:
            if encoder_mode == CLUSTER_ENCODER_SOFTWARE:
                args.usb_h264_backend = "ffmpeg"
            elif encoder_mode == CLUSTER_ENCODER_AUTO:
                args.usb_h264_backend = "native"
            else:
                args.usb_h264_backend = "native"
        if args.usb_h264_backend == "ffmpeg" and not args.usb_h264_ffmpeg_encoder_from_cli:
            args.usb_h264_ffmpeg_encoder = "libx264"
    return CLUSTER_ENCODER_PARAM


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


class ClusterLiveFpsParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> float:
        if self._params is None:
            return 0.0
        try:
            return normalize_cluster_live_fps(self._params.get_int(CLUSTER_LIVE_FPS_PARAM))
        except Exception:
            return 0.0


class ClusterHudBrightnessParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return 0
        try:
            return normalize_cluster_brightness_percent(self._params.get_int(CLUSTER_BRIGHTNESS_PARAM))
        except Exception:
            return 0


class ClusterScreenModeParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return 0
        try:
            return normalize_cluster_screen_mode(self._params.get_int(CLUSTER_SCREEN_MODE_PARAM))
        except Exception:
            return 0


class ClusterCameraViewModeParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return 0
        try:
            return normalize_cluster_camera_view_mode(self._params.get_int(CLUSTER_CAMERA_VIEW_MODE_PARAM))
        except Exception:
            return 0


class ClusterRadarInfoParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return 4
        try:
            value = self._params.get(CLUSTER_RADAR_INFO_PARAM)
            if value is None:
                return 4
            if isinstance(value, bytes):
                value = value.decode("utf-8", "ignore")
            return normalize_cluster_radar_info_mode(value)
        except Exception:
            return 4


class ClusterRadarDisplayParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return 0
        try:
            value = self._params.get(CLUSTER_RADAR_DISPLAY_PARAM)
            if value is None:
                return 0
            if isinstance(value, bytes):
                value = value.decode("utf-8", "ignore")
            return normalize_cluster_radar_display_mode(value)
        except Exception:
            return 0


class ClusterRadarSourceColorParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return 0
        try:
            value = self._params.get(CLUSTER_RADAR_SOURCE_COLOR_PARAM)
            if value is None:
                return 0
            if isinstance(value, bytes):
                value = value.decode("utf-8", "ignore")
            return normalize_cluster_radar_source_color_mode(value)
        except Exception:
            return 0


class ClusterHudModeParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int | None:
        if self._params is None:
            return None
        try:
            return int(self._params.get_int(CLUSTER_HUD_PARAM))
        except Exception:
            return None


class ClusterHudOutputGateParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read_mode(self) -> int:
        if self._params is None:
            return 0
        try:
            return max(0, min(3, int(self._params.get_int(CLUSTER_HUD_DEBUG_PARAM))))
        except Exception:
            return 0

    def allowed(self) -> bool:
        if self._params is None:
            return True
        try:
            return self.read_mode() >= 1 or bool(self._params.get_bool("IsOnroad"))
        except Exception:
            return False


class ClusterHudEncoderParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int | None:
        if self._params is None:
            return None
        try:
            return normalize_cluster_encoder_mode(self._params.get_int(CLUSTER_ENCODER_PARAM))
        except Exception:
            return None


class ClusterHudCoreModeParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int | None:
        if self._params is None:
            return None
        try:
            return normalize_cluster_core_mode(self._params.get_int(CLUSTER_CORE_MODE_PARAM))
        except Exception:
            return None


class ClusterHudPriorityParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int | None:
        if self._params is None:
            return None
        try:
            return normalize_cluster_priority(self._params.get_int(CLUSTER_PRIORITY_PARAM))
        except Exception:
            return None


def route_overlay_for_mode(overlay: RouteOverlay | None, mode: str) -> RouteOverlay | None:
    if overlay is None or mode == "off":
        return None
    if mode == "compact":
        return replace(overlay, data_lines=overlay.data_lines[:4])
    return overlay


def resolved_usb_brightness(
    setting: int,
    live_source: OpenpilotLiveSource | None,
    *,
    auto_enabled: bool,
) -> int:
    normalized = normalize_cluster_brightness_percent(setting)
    if normalized > 0 or not auto_enabled:
        return normalized

    if live_source is not None:
        auto_brightness = live_source.screen_brightness_percent()
        if auto_brightness is not None:
            return normalize_cluster_brightness_percent(auto_brightness)

    return DEFAULT_USB_BRIGHTNESS


def build_rgba_color_test_pattern(width: int, height: int) -> bytearray:
    half_width = max(1, width // 2)
    half_height = max(1, height // 2)
    red = bytes((255, 0, 0, 255))
    green = bytes((0, 255, 0, 255))
    blue = bytes((0, 0, 255, 255))
    white = bytes((255, 255, 255, 255))
    top_row = red * half_width + green * (width - half_width)
    bottom_row = blue * half_width + white * (width - half_width)
    return bytearray(top_row * half_height + bottom_row * (height - half_height))


def align_dimension(value: int, alignment: int) -> int:
    if alignment <= 1:
        return value
    return ((value + alignment - 1) // alignment) * alignment


def run_demo(
    duration_seconds: float | None,
    target_fps: float,
    live_fps_param_reader: ClusterLiveFpsParamReader | None,
    input_mode: str,
    output_mode: str,
    controller_index: int,
    width: int | None,
    height: int | None,
    usb_brightness: int,
    usb_brightness_param_reader: ClusterHudBrightnessParamReader | None,
    usb_display_fps: int,
    usb_display_fps_auto: bool,
    usb_codec: str,
    usb_jpeg_quality: int,
    usb_jpeg_encoder: str,
    usb_fast_write: bool,
    usb_wait_frame_ack: bool,
    usb_async: bool,
    usb_h264_bitrate: str,
    usb_h264_fps: int,
    usb_h264_gop: int,
    usb_h264_backend: str,
    usb_h264_library: str,
    usb_h264_ffmpeg: str,
    usb_h264_ffmpeg_encoder: str,
    usb_h264_device: str,
    usb_h264_input_format: str,
    usb_h264_slice_max_bytes: int,
    usb_h264_rate_control: str,
    usb_h264_realtime_priority: bool,
    usb_h264_orientation: str,
    usb_h264_align: int,
    usb_h264_encoder_align: int,
    usb_h264_chunk_size: int,
    usb_h264_wait_ack: bool,
    usb_h264_soft_ack: bool,
    usb_h264_dump: str,
    usb_h264_debug: bool,
    usb_h264_diagnose_interval_s: float,
    usb_h264_test_pattern: bool,
    usb_h264_test_pattern_nv12: bool,
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
    cluster_core_usage_enabled: bool,
    cluster_core_usage_debug: bool,
    profile_render: bool,
    profile_interval_s: float,
    gc_freeze_init: bool,
    theme_mode: str | None,
    hud_mode_watch: int | None,
    hud_encoder_watch: int | None,
    hud_core_mode_watch: int | None,
    hud_priority_watch: int | None,
) -> None:
    if hud_core_mode_watch is not None:
        hud_core_mode_watch = normalize_cluster_core_mode(hud_core_mode_watch)
    if hud_priority_watch is not None:
        hud_priority_watch = normalize_cluster_priority(hud_priority_watch)
    profile = ProfileReporter(profile_render, profile_interval_s)
    gc_hook = GcProfileHook(profile) if profile_render else None
    if gc_hook is not None:
        gc.callbacks.append(gc_hook)
    stop_requested = False
    previous_sigterm_handler = None
    signal_installed = False

    def request_stop(signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True
        print(f"Received signal {signum}; shutting down cluster HUD", flush=True)

    if threading.current_thread() is threading.main_thread():
        previous_sigterm_handler = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM, request_stop)
        signal_installed = True

    usb_display: TuringUsbDisplay | None = None
    usb_pipeline: AsyncJpegUsbPipeline | None = None
    h264_pipeline: H264UsbPipeline | None = None
    active_brightness_setting = normalize_cluster_brightness_percent(usb_brightness)
    usb_brightness_auto_enabled = usb_brightness_param_reader is not None
    initial_usb_brightness = resolved_usb_brightness(
        active_brightness_setting,
        None,
        auto_enabled=usb_brightness_auto_enabled,
    )
    if output_mode in ("usb", "both"):
        usb_display = TuringUsbDisplay(
            brightness=initial_usb_brightness,
            display_fps=usb_display_fps,
            jpeg_quality=usb_jpeg_quality,
            jpeg_encoder=usb_jpeg_encoder,
            fast_write=usb_fast_write,
            wait_for_frame_ack=usb_wait_frame_ack,
            frame_drain_attempts=usb_frame_drain_attempts,
            frame_drain_timeout_ms=usb_frame_drain_timeout_ms,
            fast_frame_drain_attempts=usb_fast_drain_attempts,
            fast_frame_drain_timeout_ms=usb_fast_drain_timeout_ms,
            expected_product_id=(
                product_id_for_hud_mode(hud_mode_watch) if hud_mode_watch is not None else None
            ),
        )
        usb_display.set_profile_enabled(profile_render)
        profile_stage = time.perf_counter()
        try:
            usb_display.open()
        except Exception:
            usb_display.close()
            raise
        profile.add_elapsed("usb.open", profile_stage)
        profile.add_samples(usb_display.profile_samples())
        usb_display.clear_profile_samples()
        if usb_async and usb_codec == "jpeg":
            usb_pipeline = AsyncJpegUsbPipeline(usb_display)
            usb_pipeline.start()

    frame_width = width or (usb_display.landscape_width if usb_display is not None else DESIGN_WIDTH)
    frame_height = height or (usb_display.landscape_height if usb_display is not None else DESIGN_HEIGHT)
    if usb_codec == "h264":
        aligned_width = align_dimension(frame_width, usb_h264_align)
        aligned_height = align_dimension(frame_height, usb_h264_align)
        if aligned_width != frame_width or aligned_height != frame_height:
            print(
                f"H264 USB output aligned render size from "
                f"{frame_width}x{frame_height} to {aligned_width}x{aligned_height} "
                f"(alignment={usb_h264_align})",
                flush=True,
            )
            frame_width = aligned_width
            frame_height = aligned_height
    h264_portrait_upload = usb_h264_orientation == "portrait"
    h264_width = frame_height if h264_portrait_upload else frame_width
    h264_height = frame_width if h264_portrait_upload else frame_height
    if usb_codec == "h264" and ((h264_width % 2) != 0 or (h264_height % 2) != 0):
        raise RuntimeError(
            f"H264 USB output requires even encoder dimensions, got {h264_width}x{h264_height}"
        )
    theme_override = normalize_cluster_theme_mode(theme_mode) if theme_mode is not None else None
    theme_param_reader = ClusterThemeParamReader() if theme_override is None else None
    active_theme_mode = theme_override or (theme_param_reader.read() if theme_param_reader is not None else "auto")
    screen_mode_param_reader = ClusterScreenModeParamReader()
    active_screen_mode = screen_mode_param_reader.read()
    camera_view_param_reader = ClusterCameraViewModeParamReader()
    active_camera_view_mode = camera_view_param_reader.read()
    radar_info_param_reader = ClusterRadarInfoParamReader()
    active_radar_info_mode = radar_info_param_reader.read()
    radar_display_param_reader = ClusterRadarDisplayParamReader()
    active_radar_display_mode = radar_display_param_reader.read()
    radar_source_color_param_reader = ClusterRadarSourceColorParamReader()
    active_radar_source_color_mode = radar_source_color_param_reader.read()
    hud_mode_param_reader = ClusterHudModeParamReader() if hud_mode_watch is not None else None
    hud_encoder_param_reader = ClusterHudEncoderParamReader() if hud_encoder_watch is not None else None
    hud_core_mode_param_reader = ClusterHudCoreModeParamReader() if hud_core_mode_watch is not None else None
    hud_priority_param_reader = ClusterHudPriorityParamReader() if hud_priority_watch is not None else None
    hud_debug_param_reader = ClusterHudOutputGateParamReader() if hud_mode_watch is not None or input_mode == "live" else None
    hud_output_gate_param_reader = hud_debug_param_reader if hud_mode_watch is not None else None
    active_hud_debug_mode = hud_debug_param_reader.read_mode() if hud_debug_param_reader is not None else 0
    renderer = ClusterUiRenderer(
        frame_width,
        frame_height,
        target_fps=max(0, int(round(target_fps))),
        theme_mode=active_theme_mode,
        screen_mode=active_screen_mode,
    )
    print(f"{CLUSTER_SCREEN_MODE_PARAM} initial: {active_screen_mode}", flush=True)
    print(f"{CLUSTER_CAMERA_VIEW_MODE_PARAM} initial: {active_camera_view_mode}", flush=True)
    print(
        f"{CLUSTER_RADAR_INFO_PARAM} initial: {active_radar_info_mode} "
        f"{CLUSTER_RADAR_DISPLAY_PARAM} initial: {active_radar_display_mode} "
        f"{CLUSTER_RADAR_SOURCE_COLOR_PARAM} initial: {active_radar_source_color_mode}",
        flush=True,
    )
    renderer.set_profile_enabled(profile_render)
    git_status_provider = GitBranchStatusProvider(Path(__file__).resolve().parent)
    cluster_core_usage_sampler = (
        ClusterProcessCoreUsageSampler(debug=cluster_core_usage_debug)
        if cluster_core_usage_enabled
        else None
    )
    simulator = ClusterSimulator() if input_mode in ("random", "gamepad") else None
    controller = DualSenseSimulator(controller_index) if input_mode == "gamepad" else None
    random_input = RandomInputSource() if input_mode == "random" else None
    live_source = OpenpilotLiveSource(include_can=live_include_can, timeout_ms=live_timeout_ms) if input_mode == "live" else None
    if live_source is not None:
        live_source.set_profile_enabled(profile_render)
        live_source.set_hud_debug_mode(active_hud_debug_mode)
        live_source.set_debug_panels_enabled(
            live_debug=live_debug_panel_enabled(active_screen_mode),
            debug_plot=live_debug_plot_enabled(active_screen_mode),
            navi_debug=live_navi_debug_enabled(active_screen_mode),
        )
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
    next_fps_param_read = start_time + FPS_PARAM_POLL_SECONDS
    next_brightness_param_read = start_time
    next_screen_mode_param_read = start_time
    next_camera_view_param_read = start_time
    next_radar_param_read = start_time
    next_hud_mode_param_read = start_time
    report_frames = 0
    display_actual_fps: float | None = None
    frame_interval = 1.0 / target_fps if target_fps > 0 else 0.0
    h264_test_pattern_rgba: bytearray | None = None
    h264_test_pattern_nv12: bytearray | None = None
    h264_render_nv12_buffer: bytearray | None = None
    h264_render_nv12_layout: tuple[int, int, int, int, int, int, bool] | None = None

    if hud_output_gate_param_reader is not None and not hud_output_gate_param_reader.allowed():
        print(
            f"{CLUSTER_HUD_DEBUG_PARAM}=0 and IsOnroad=0; "
            "cluster HUD output remains off",
            flush=True,
        )
        return

    try:
        renderer.open(hidden=output_mode == "usb")
        profile.add_samples(renderer.profile_samples())
        renderer.clear_profile_samples()
        if usb_display is not None and usb_codec == "h264":
            h264_encoder_fps = resolved_h264_encoder_fps(target_fps, usb_h264_fps)
            h264_pipeline = H264UsbPipeline(
                usb_display,
                h264_width,
                h264_height,
                usb_h264_encoder_align,
                h264_encoder_fps,
                usb_h264_bitrate,
                usb_h264_gop,
                usb_h264_backend,
                usb_h264_library,
                usb_h264_ffmpeg,
                usb_h264_ffmpeg_encoder,
                usb_h264_device,
                usb_h264_input_format,
                usb_h264_slice_max_bytes,
                usb_h264_rate_control,
                usb_h264_realtime_priority,
                usb_h264_chunk_size,
                usb_h264_wait_ack,
                usb_h264_soft_ack,
                usb_h264_dump,
                usb_h264_debug,
                usb_h264_diagnose_interval_s,
            )
            profile_stage = time.perf_counter()
            h264_pipeline.start()
            profile.add_elapsed("usb_h264.start", profile_stage)
            profile.add_samples(usb_display.profile_samples())
            usb_display.clear_profile_samples()
            if h264_pipeline.backend_name == "native":
                h264_render_nv12_layout = h264_pipeline.native_nv12_render_layout()
                stride, y_scanlines, uv_scanlines, uv_offset, input_bytes, render_bytes, active_submit = h264_render_nv12_layout
                print(
                    f"Using H264 native NV12 render path: "
                    f"{h264_pipeline.encoder_width}x{h264_pipeline.encoder_height} "
                    f"stride={stride} scanlines={y_scanlines}/{uv_scanlines} "
                    f"uv_offset={uv_offset} bytes={input_bytes} render_bytes={render_bytes} "
                    f"active_submit={'on' if active_submit else 'off'} flip_x=on",
                    flush=True,
                )
            if usb_h264_test_pattern:
                if h264_pipeline.backend_name == "native":
                    raise RuntimeError(
                        "--usb-h264-test-pattern is only supported by --usb-h264-backend ffmpeg; "
                        "use --usb-h264-test-pattern-nv12 for native H264"
                    )
                h264_test_pattern_rgba = build_rgba_color_test_pattern(
                    h264_pipeline.width,
                    h264_pipeline.height,
                )
                print(
                    f"Using H264 RGBA color test pattern: "
                    f"{h264_pipeline.width}x{h264_pipeline.height} "
                    f"orientation={usb_h264_orientation}",
                    flush=True,
                )
            if usb_h264_test_pattern_nv12:
                if h264_pipeline.backend_name != "native":
                    raise RuntimeError("--usb-h264-test-pattern-nv12 requires the native H264 backend")
                h264_test_pattern_nv12 = h264_pipeline.build_nv12_color_test_pattern()
                print(
                    f"Using H264 native NV12 color test pattern: "
                    f"{h264_pipeline.encoder_width}x{h264_pipeline.encoder_height} "
                    f"bytes={len(h264_test_pattern_nv12)} orientation={usb_h264_orientation}",
                    flush=True,
                )
        if gc_freeze_init:
            freeze_gc_after_init(profile)
        while True:
            if stop_requested:
                break
            frame_start_time = time.perf_counter()
            renderer.clear_profile_samples()
            if usb_display is not None and usb_pipeline is None:
                usb_display.clear_profile_samples()
            if usb_pipeline is not None:
                usb_pipeline.check_error()
                profile.add_samples(usb_pipeline.profile_samples())
            if h264_pipeline is not None:
                h264_pipeline.check_error()
                profile.add_samples(h264_pipeline.profile_samples())
            if output_mode in ("window", "both") and renderer.should_close():
                break

            now = time.perf_counter()
            if theme_override is None and now >= next_theme_param_read:
                next_theme_mode = theme_param_reader.read() if theme_param_reader is not None else "auto"
                if next_theme_mode != renderer.theme_mode:
                    renderer.set_theme_mode(next_theme_mode)
                next_theme_param_read = now + THEME_PARAM_POLL_SECONDS
            if now >= next_screen_mode_param_read:
                next_screen_mode = screen_mode_param_reader.read()
                if next_screen_mode != renderer.screen_mode:
                    print(
                        f"{CLUSTER_SCREEN_MODE_PARAM} updated: {renderer.screen_mode} -> {next_screen_mode}",
                        flush=True,
                    )
                    renderer.set_screen_mode(next_screen_mode)
                    if live_source is not None:
                        live_source.set_debug_panels_enabled(
                            live_debug=live_debug_panel_enabled(next_screen_mode),
                            debug_plot=live_debug_plot_enabled(next_screen_mode),
                            navi_debug=live_navi_debug_enabled(next_screen_mode),
                        )
                next_screen_mode_param_read = now + SCREEN_MODE_PARAM_POLL_SECONDS
            if now >= next_camera_view_param_read:
                next_camera_view_mode = camera_view_param_reader.read()
                if next_camera_view_mode != active_camera_view_mode:
                    print(
                        f"{CLUSTER_CAMERA_VIEW_MODE_PARAM} updated: "
                        f"{active_camera_view_mode} -> {next_camera_view_mode}",
                        flush=True,
                    )
                    active_camera_view_mode = next_camera_view_mode
                next_camera_view_param_read = now + CAMERA_VIEW_PARAM_POLL_SECONDS
            if now >= next_radar_param_read:
                next_radar_info_mode = radar_info_param_reader.read()
                if next_radar_info_mode != active_radar_info_mode:
                    print(
                        f"{CLUSTER_RADAR_INFO_PARAM} updated: "
                        f"{active_radar_info_mode} -> {next_radar_info_mode}",
                        flush=True,
                    )
                    active_radar_info_mode = next_radar_info_mode
                next_radar_display_mode = radar_display_param_reader.read()
                if next_radar_display_mode != active_radar_display_mode:
                    print(
                        f"{CLUSTER_RADAR_DISPLAY_PARAM} updated: "
                        f"{active_radar_display_mode} -> {next_radar_display_mode}",
                        flush=True,
                    )
                    active_radar_display_mode = next_radar_display_mode
                next_radar_source_color_mode = radar_source_color_param_reader.read()
                if next_radar_source_color_mode != active_radar_source_color_mode:
                    print(
                        f"{CLUSTER_RADAR_SOURCE_COLOR_PARAM} updated: "
                        f"{active_radar_source_color_mode} -> {next_radar_source_color_mode}",
                        flush=True,
                    )
                    active_radar_source_color_mode = next_radar_source_color_mode
                next_radar_param_read = now + RADAR_PARAM_POLL_SECONDS
            if (
                now >= next_hud_mode_param_read
                and (
                    hud_mode_param_reader is not None
                    or hud_encoder_param_reader is not None
                    or hud_core_mode_param_reader is not None
                    or hud_priority_param_reader is not None
                    or hud_debug_param_reader is not None
                    or hud_output_gate_param_reader is not None
                )
            ):
                next_hud_mode = hud_mode_param_reader.read() if hud_mode_param_reader is not None else None
                if hud_mode_param_reader is not None and next_hud_mode is not None and next_hud_mode != hud_mode_watch:
                    print(
                        f"{CLUSTER_HUD_PARAM} changed from {hud_mode_watch} to {next_hud_mode}; exiting",
                        flush=True,
                    )
                    break
                next_hud_encoder = hud_encoder_param_reader.read() if hud_encoder_param_reader is not None else None
                if next_hud_encoder is not None and next_hud_encoder != hud_encoder_watch:
                    print(
                        f"{CLUSTER_ENCODER_PARAM} changed from {hud_encoder_watch} to {next_hud_encoder}; exiting",
                        flush=True,
                    )
                    break
                next_hud_core_mode = hud_core_mode_param_reader.read() if hud_core_mode_param_reader is not None else None
                if next_hud_core_mode is not None and next_hud_core_mode != hud_core_mode_watch:
                    print(
                        f"{CLUSTER_CORE_MODE_PARAM} changed from "
                        f"{hud_core_mode_watch} to {next_hud_core_mode}; exiting for restart",
                        flush=True,
                    )
                    break
                next_hud_priority = hud_priority_param_reader.read() if hud_priority_param_reader is not None else None
                if next_hud_priority is not None and next_hud_priority != hud_priority_watch:
                    print(
                        f"{CLUSTER_PRIORITY_PARAM} changed from "
                        f"{hud_priority_watch} to {next_hud_priority}; exiting for restart",
                        flush=True,
                    )
                    break
                if hud_debug_param_reader is not None:
                    next_hud_debug_mode = hud_debug_param_reader.read_mode()
                    if next_hud_debug_mode != active_hud_debug_mode:
                        print(
                            f"{CLUSTER_HUD_DEBUG_PARAM} updated: "
                            f"{active_hud_debug_mode} -> {next_hud_debug_mode}",
                            flush=True,
                        )
                        active_hud_debug_mode = next_hud_debug_mode
                        if live_source is not None:
                            live_source.set_hud_debug_mode(active_hud_debug_mode)
                if hud_output_gate_param_reader is not None and not hud_output_gate_param_reader.allowed():
                    print(
                        f"{CLUSTER_HUD_DEBUG_PARAM}=0 and IsOnroad=0; "
                        "exiting to turn off cluster HUD output",
                        flush=True,
                    )
                    break
                next_hud_mode_param_read = now + HUD_MODE_PARAM_POLL_SECONDS
            if live_fps_param_reader is not None and now >= next_fps_param_read:
                next_target_fps = live_fps_param_reader.read()
                if next_target_fps != target_fps:
                    next_h264_encoder_fps = resolved_h264_encoder_fps(next_target_fps, usb_h264_fps)
                    if h264_pipeline is not None and next_h264_encoder_fps != h264_pipeline.fps:
                        print(
                            f"{CLUSTER_LIVE_FPS_PARAM} changed H264 encoder FPS "
                            f"from {h264_pipeline.fps} to {next_h264_encoder_fps}; exiting for restart",
                            flush=True,
                        )
                        break
                    target_fps = next_target_fps
                    frame_interval = 1.0 / target_fps if target_fps > 0 else 0.0
                    renderer.set_target_fps(max(0, int(round(target_fps))))
                    fps_text = "uncapped" if target_fps == 0 else f"{target_fps:.1f} Hz"
                    print(f"{CLUSTER_LIVE_FPS_PARAM} updated: {fps_text}", flush=True)
                    if usb_display is not None and usb_display_fps_auto:
                        next_display_fps = resolved_usb_display_fps(
                            None,
                            usb_codec,
                            target_fps,
                            usb_h264_fps,
                        )
                        if usb_display.set_display_fps(next_display_fps):
                            print(f"TURZX display FPS updated: {next_display_fps}", flush=True)
                next_fps_param_read = now + FPS_PARAM_POLL_SECONDS
            if duration_seconds is not None and now - start_time >= duration_seconds:
                break

            dt = max(0.001, now - last_frame_time)
            last_frame_time = now
            if dt < 1.0:
                instant_fps = 1.0 / dt
                display_actual_fps = (
                    instant_fps
                    if display_actual_fps is None
                    else display_actual_fps * 0.85 + instant_fps * 0.15
                )
            source_status: str | None = None
            center_clock_text: str | None = None
            if live_source is not None:
                profile_stage = time.perf_counter()
                state = live_source.update()
                center_clock_text = time.strftime("%H:%M:%S")
                profile.add_samples(live_source.profile_samples())
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

            if live_source is None:
                center_clock_text = state.center_clock_text

            cluster_core_usage_text = None
            if cluster_core_usage_sampler is not None:
                profile_stage = time.perf_counter()
                cluster_core_usage_text = cluster_core_usage_sampler.sample_text(now)
                profile.add_elapsed("main.cluster_core_usage_sample", profile_stage)
            state = replace(
                state,
                radar_info_mode=active_radar_info_mode,
                radar_display_mode=active_radar_display_mode,
                radar_source_color_mode=active_radar_source_color_mode,
                camera_view_mode=active_camera_view_mode,
                center_clock_text=center_clock_text,
                git_status=git_status_provider.status(),
                actual_fps=display_actual_fps,
                cluster_core_usage_text=cluster_core_usage_text,
            )
            brightness_now = time.perf_counter()
            if usb_display is not None and brightness_now >= next_brightness_param_read:
                if usb_brightness_param_reader is not None:
                    next_brightness_setting = usb_brightness_param_reader.read()
                    if next_brightness_setting != active_brightness_setting:
                        active_brightness_setting = next_brightness_setting
                        brightness_text = (
                            "auto"
                            if active_brightness_setting == 0
                            else f"{active_brightness_setting}%"
                        )
                        print(f"{CLUSTER_BRIGHTNESS_PARAM} updated: {brightness_text}", flush=True)
                next_usb_brightness = resolved_usb_brightness(
                    active_brightness_setting,
                    live_source,
                    auto_enabled=usb_brightness_auto_enabled,
                )
                usb_display.set_brightness(next_usb_brightness)
                next_brightness_param_read = brightness_now + BRIGHTNESS_PARAM_POLL_SECONDS

            if output_mode in ("window", "both"):
                profile_stage = time.perf_counter()
                renderer.render_frame(state)
                profile.add_elapsed("main.window_render_total", profile_stage)
            if usb_display is not None:
                if usb_codec == "jpeg":
                    if usb_pipeline is not None:
                        profile_stage = time.perf_counter()
                        usb_pipeline.wait_for_capacity()
                        profile.add_elapsed("main.usb_async.wait_capacity", profile_stage)
                        profile.add_samples(usb_pipeline.profile_samples())

                        profile_stage = time.perf_counter()
                        rgba, image_width, image_height = renderer.render_to_rgba_bytes(
                            state,
                            portrait_upload=True,
                        )
                        profile.add_elapsed("main.usb.render_rgba_total", profile_stage)

                        profile_stage = time.perf_counter()
                        usb_pipeline.submit_rgba(rgba, image_width, image_height)
                        profile.add_elapsed("main.usb_async.submit_rgba", profile_stage)
                    else:
                        profile_stage = time.perf_counter()
                        with renderer.render_to_rgba_buffer(state, portrait_upload=True) as (
                            rgba,
                            image_width,
                            image_height,
                        ):
                            profile.add_elapsed("main.usb.render_rgba_total", profile_stage)

                            profile_stage = time.perf_counter()
                            jpeg = usb_display.encode_jpeg(rgba, image_width, image_height)
                            profile.add_elapsed("main.usb.encode_jpeg", profile_stage)

                        profile_stage = time.perf_counter()
                        usb_display.send_jpeg(jpeg)
                        profile.add_elapsed("main.usb.send_jpeg", profile_stage)
                elif usb_codec == "h264":
                    if h264_pipeline is None:
                        raise RuntimeError("H264 USB pipeline is not available")
                    if h264_test_pattern_rgba is None and h264_test_pattern_nv12 is None:
                        if h264_pipeline.backend_name == "native":
                            if h264_render_nv12_layout is None:
                                raise RuntimeError("H264 GPU NV12 render path is missing the native layout")
                            stride, y_scanlines, uv_scanlines, uv_offset, input_bytes, render_bytes, _ = h264_render_nv12_layout
                            profile_stage = time.perf_counter()
                            with renderer.render_to_nv12_buffer(
                                state,
                                h264_pipeline.encoder_width,
                                h264_pipeline.encoder_height,
                                stride,
                                y_scanlines,
                                uv_scanlines,
                                uv_offset,
                                render_bytes,
                                h264_render_nv12_buffer,
                                flip_x=True,
                            ) as h264_render_nv12_frame:
                                profile.add_elapsed("main.usb.render_nv12_total", profile_stage)
                                if isinstance(h264_render_nv12_frame, bytearray):
                                    h264_render_nv12_buffer = h264_render_nv12_frame

                                profile_stage = time.perf_counter()
                                h264_pipeline.submit_nv12(
                                    h264_render_nv12_frame,
                                    h264_pipeline.encoder_width,
                                    h264_pipeline.encoder_height,
                                )
                                profile.add_elapsed("main.usb_h264.submit_nv12", profile_stage)
                        else:
                            profile_stage = time.perf_counter()
                            with renderer.render_to_rgba_buffer(
                                state,
                                portrait_upload=h264_portrait_upload,
                                output_width=h264_pipeline.encoder_width if h264_portrait_upload else None,
                                output_height=h264_pipeline.encoder_height if h264_portrait_upload else None,
                            ) as (
                                rgba,
                                image_width,
                                image_height,
                            ):
                                profile.add_elapsed("main.usb.render_rgba_total", profile_stage)

                                profile_stage = time.perf_counter()
                                h264_pipeline.submit_rgba(rgba, image_width, image_height)
                                profile.add_elapsed("main.usb_h264.submit_rgba", profile_stage)
                    else:
                        profile_stage = time.perf_counter()
                        if h264_test_pattern_nv12 is not None:
                            h264_pipeline.submit_nv12(
                                h264_test_pattern_nv12,
                                h264_pipeline.encoder_width,
                                h264_pipeline.encoder_height,
                            )
                        else:
                            h264_pipeline.submit_rgba(
                                h264_test_pattern_rgba,
                                h264_pipeline.width,
                                h264_pipeline.height,
                            )
                        profile.add_elapsed("main.usb_h264.submit_test_pattern", profile_stage)
                else:
                    profile_stage = time.perf_counter()
                    png = renderer.render_to_png_bytes(state, portrait_upload=True)
                    profile.add_elapsed("main.usb.render_png_total", profile_stage)
                    profile_stage = time.perf_counter()
                    usb_display.send_png(png)
                    profile.add_elapsed("main.usb.send_png", profile_stage)
                if usb_pipeline is not None:
                    profile.add_samples(usb_pipeline.profile_samples())
                elif h264_pipeline is not None:
                    profile.add_samples(h264_pipeline.profile_samples())
                    profile.add_samples(usb_display.profile_samples())
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
            should_print_status = now - last_report_time >= 2.0
            if should_print_status and source_status is None and live_source is not None:
                source_status = live_source.status_text()
                profile.add_samples(live_source.profile_samples())
            profile.maybe_report(now)
            if should_print_status:
                actual_fps = report_frames / (now - last_report_time)
                lane_status = state.lane_change or (
                    "keep" if state.lane_change_phase == "idle" else state.lane_change_phase
                )
                print(
                    f"Refresh {actual_fps:.1f} Hz | "
                    f"speed={state.speed_kph:5.1f} km/h "
                    f"accel={state.accel_mps2:+.2f} m/s^2 "
                    f"limit={state.speed_limit_kph}:{state.speed_limit_source or '-'} "
                    f"gear={state.gear_text or '-'} gap={state.cruise_gap or '-'} "
                    f"lane={lane_status}:{state.lane_change_progress:.2f} "
                    f"ego_offset={state.ego_lane_offset:+.2f} | "
                    f"output={output_mode}/{usb_codec if usb_display else 'screen'}"
                    f"{'-fast' if usb_display and usb_fast_write else ''} "
                    f"{'async ' if usb_pipeline is not None else ''}"
                    f"theme={renderer.theme_mode} "
                    f"cam={state.camera_view_mode} "
                    f"view_yaw={state.surround_yaw_deg:+.0f} "
                    f"{source_status}"
                )
                report_frames = 0
                last_report_time = now
    finally:
        if signal_installed:
            signal.signal(signal.SIGTERM, previous_sigterm_handler)
        if gc_hook is not None:
            try:
                gc.callbacks.remove(gc_hook)
            except ValueError:
                pass
        if h264_pipeline is not None:
            h264_pipeline.close()
        if usb_pipeline is not None:
            usb_pipeline.close()
        if controller is not None:
            controller.close()
        if route_source is not None:
            route_source.close()
        if live_source is not None:
            live_source.close()
        if usb_display is not None:
            usb_display.close()
        renderer.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--fps",
        type=float,
        default=None,
        help=(
            "Target refresh rate. Use 0 for uncapped/as-fast-as-possible. "
            f"When omitted, CLI runs read {CLUSTER_LIVE_FPS_PARAM}; mode 0 keeps the default cap behavior."
        ),
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
    parser.add_argument(
        "--usb-brightness",
        type=int,
        default=None,
        help=(
            "Manual TURZX brightness 0-100. When omitted, live USB mode reads "
            f"{CLUSTER_BRIGHTNESS_PARAM}: 0 auto, 1-100 manual."
        ),
    )
    parser.add_argument(
        "--usb-display-fps",
        type=int,
        default=None,
        help=(
            "Optional TURZX display frame-rate command. Default auto matches H264 output FPS "
            "and skips it for JPEG/PNG; use 0 to skip."
        ),
    )
    parser.add_argument(
        "--usb-codec",
        choices=("jpeg", "png", "h264"),
        default="jpeg",
        help=f"USB output codec. When omitted, USB CLI runs read {CLUSTER_ENCODER_PARAM}.",
    )
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
        "--usb-h264-bitrate",
        default=DEFAULT_H264_BITRATE,
        help=(
            "Target H264 bitrate for --usb-codec h264. Default auto uses about 234k per FPS "
            "bounded to 1M-7M; 30 FPS resolves to 7M."
        ),
    )
    parser.add_argument(
        "--usb-h264-fps",
        type=int,
        default=30,
        help="H264 encoder input FPS. Also caps non-live H264 USB runs when --fps is omitted. Default: 30.",
    )
    parser.add_argument(
        "--usb-h264-gop",
        type=int,
        default=DEFAULT_H264_GOP,
        help="H264 keyframe interval in frames. Default: %(default)s.",
    )
    parser.add_argument(
        "--usb-h264-backend",
        choices=("auto", "native", "ffmpeg"),
        default="native",
        help=(
            "H264 encoder backend. Default native uses the Qualcomm hardware encoder; "
            "ffmpeg/libx264 remains available as a known-good comparison path."
        ),
    )
    parser.add_argument(
        "--usb-h264-library",
        default=str(DEFAULT_H264_LIBRARY),
        help=(
            "Native hardware H264 shared library for --usb-codec h264. "
            f"Default: {DEFAULT_H264_LIBRARY}."
        ),
    )
    parser.add_argument(
        "--usb-h264-ffmpeg",
        default=DEFAULT_H264_FFMPEG,
        help=f"ffmpeg executable path/name for --usb-h264-backend ffmpeg. Default: {DEFAULT_H264_FFMPEG}.",
    )
    parser.add_argument(
        "--usb-h264-ffmpeg-encoder",
        default=DEFAULT_H264_FFMPEG_ENCODER,
        help=(
            "ffmpeg H264 encoder for --usb-h264-backend ffmpeg. "
            "Default libx264; auto prefers h264_v4l2m2m, then h264_omx, then libx264."
        ),
    )
    parser.add_argument(
        "--usb-h264-device",
        default=DEFAULT_H264_DEVICE,
        help=f"V4L2 hardware encoder device path. Default: {DEFAULT_H264_DEVICE}.",
    )
    parser.add_argument(
        "--usb-h264-input-format",
        choices=("auto", "nv12"),
        default="nv12",
        help=(
            "Hardware encoder input format. Default nv12 follows the existing loggerd V4L2 path."
        ),
    )
    parser.add_argument(
        "--usb-h264-slice-max-bytes",
        type=int,
        default=DEFAULT_H264_SLICE_MAX_BYTES,
        help=(
            "Hardware V4L2 multi-slice max bytes. Lower values produce smaller H264 NALs; "
            "0 disables multi-slice. Default: %(default)s."
        ),
    )
    parser.add_argument(
        "--usb-h264-rate-control",
        choices=tuple(NATIVE_RATE_CONTROLS.keys()),
        default=DEFAULT_H264_RATE_CONTROL,
        help=(
            "Hardware V4L2 rate-control mode for native H264. "
            "Default: %(default)s."
        ),
    )
    parser.add_argument(
        "--usb-h264-realtime-priority",
        action="store_true",
        help="Request realtime priority from the native V4L2 encoder.",
    )
    parser.add_argument(
        "--usb-h264-orientation",
        choices=("landscape", "portrait"),
        default="portrait",
        help=(
            "H264 bitstream geometry. portrait matches the JPEG/PNG rotated upload path; "
            "landscape encodes the rendered 1920x480 frame directly."
        ),
    )
    parser.add_argument(
        "--usb-h264-align",
        type=int,
        default=DEFAULT_H264_DIMENSION_ALIGN,
        help=(
            "Round H264 render/encoder dimensions up to this multiple. "
            "Default 1 preserves the panel's exact reported size."
        ),
    )
    parser.add_argument(
        "--usb-h264-encoder-align",
        type=int,
        default=DEFAULT_H264_ENCODER_ALIGN,
        help=(
            "Align native hardware encoder input dimensions without changing the rendered display size. "
            "Default %(default)s pads 462-wide TURZX frames to a macroblock-safe 464-wide encoder input."
        ),
    )
    parser.add_argument(
        "--usb-h264-chunk-size",
        type=int,
        default=0,
        help="Override TURZX H264 chunk size in bytes. Default 0 negotiates with the device.",
    )
    parser.add_argument(
        "--usb-h264-no-ack",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--usb-h264-wait-ack",
        action="store_true",
        help="Wait for a TURZX response after each H264 chunk and fail on timeout.",
    )
    parser.add_argument(
        "--usb-h264-soft-ack",
        action="store_true",
        help="Wait for TURZX H264 responses, but continue when the panel times out like the vendor video sender.",
    )
    parser.add_argument(
        "--usb-h264-dump",
        default="",
        help="Write the outgoing H264 bytestream to this path for ffprobe/ffplay comparison.",
    )
    parser.add_argument(
        "--usb-h264-debug",
        action="store_true",
        help="Print hardware encoder command/stderr and first H264 chunk sizes/headers for USB H264 debugging.",
    )
    parser.add_argument(
        "--usb-h264-diagnose-interval",
        type=float,
        default=0.0,
        help=(
            "Print compact H264 unit/chunk/USB timing summaries every N seconds; "
            "0 disables the periodic diagnostic log."
        ),
    )
    parser.add_argument(
        "--usb-h264-test-pattern",
        action="store_true",
        help="Feed a red/green/blue/white RGBA quadrant pattern into the ffmpeg H264 path.",
    )
    parser.add_argument(
        "--usb-h264-test-pattern-nv12",
        action="store_true",
        help="Feed a native-aligned red/green/blue/white NV12 quadrant pattern into the native H264 path.",
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
        "--cluster-hud-mode",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--cluster-hud-encoder",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--cluster-hud-core-mode",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--cluster-hud-priority",
        type=int,
        default=None,
        help=argparse.SUPPRESS,
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
        help=(
            "Disable live CAN/sendcan subscriptions. This keeps radarState/modelV2/liveTracks data "
            "but skips direct raw CAN parsing for camera-bus ADAS corner detections."
        ),
    )
    parser.add_argument(
        "--live-timeout-ms",
        type=int,
        default=0,
        help="SubMaster update timeout for --input live. Default 0 keeps rendering responsive.",
    )
    parser.add_argument(
        "--no-cluster-core-usage",
        action="store_true",
        help="Disable the lower-right cluster process per-core CPU overlay.",
    )
    parser.add_argument(
        "--cluster-core-usage-debug",
        action="store_true",
        help="Print live cluster process per-core CPU sampler scan cost and per-process CPU details.",
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
        "--no-gc-freeze",
        action="store_true",
        help="Disable post-init gc.freeze(). Default enabled to avoid long gen2 pauses during USB rendering.",
    )
    raw_args = sys.argv[1:]
    args = parser.parse_args()
    args.fps_from_cli = args.fps is not None
    args.usb_codec_from_cli = option_present(raw_args, "--usb-codec")
    args.usb_h264_backend_from_cli = option_present(raw_args, "--usb-h264-backend")
    args.usb_h264_ffmpeg_encoder_from_cli = option_present(raw_args, "--usb-h264-ffmpeg-encoder")
    if args.fps is None:
        args.fps = DEFAULT_FPS
    if args.fps < 0:
        parser.error("--fps must be 0 or greater")
    if (args.width is not None and args.width <= 0) or (args.height is not None and args.height <= 0):
        parser.error("--width and --height must be greater than 0")
    args.usb_brightness_from_cli = args.usb_brightness is not None
    if args.usb_brightness is not None and not 0 <= args.usb_brightness <= 100:
        parser.error("--usb-brightness must be between 0 and 100")
    if args.usb_display_fps is not None and not 0 <= args.usb_display_fps <= 255:
        parser.error("--usb-display-fps must be between 0 and 255")
    if not 1 <= args.usb_jpeg_quality <= 95:
        parser.error("--usb-jpeg-quality must be between 1 and 95")
    if args.usb_async and args.usb_codec != "jpeg":
        parser.error("--usb-async only supports --usb-codec jpeg")
    if args.usb_h264_fps <= 0:
        parser.error("--usb-h264-fps must be greater than 0")
    if args.usb_h264_gop <= 0:
        parser.error("--usb-h264-gop must be greater than 0")
    if args.usb_h264_chunk_size < 0:
        parser.error("--usb-h264-chunk-size must be 0 or greater")
    if args.usb_h264_slice_max_bytes < 0:
        parser.error("--usb-h264-slice-max-bytes must be 0 or greater")
    if args.usb_h264_align <= 0:
        parser.error("--usb-h264-align must be greater than 0")
    if args.usb_h264_encoder_align <= 0:
        parser.error("--usb-h264-encoder-align must be greater than 0")
    if args.usb_h264_wait_ack and args.usb_h264_no_ack:
        parser.error("--usb-h264-wait-ack and --usb-h264-no-ack cannot be used together")
    if args.usb_h264_soft_ack and args.usb_h264_no_ack:
        parser.error("--usb-h264-soft-ack and --usb-h264-no-ack cannot be used together")
    if not args.usb_h264_bitrate.strip():
        parser.error("--usb-h264-bitrate must not be empty")
    if args.usb_h264_diagnose_interval < 0:
        parser.error("--usb-h264-diagnose-interval must be 0 or greater")
    if args.usb_h264_test_pattern and args.usb_h264_test_pattern_nv12:
        parser.error("--usb-h264-test-pattern and --usb-h264-test-pattern-nv12 cannot be used together")
    if args.usb_h264_test_pattern_nv12 and args.usb_h264_backend == "ffmpeg":
        parser.error("--usb-h264-test-pattern-nv12 requires --usb-h264-backend native or auto")
    if not args.usb_h264_library:
        parser.error("--usb-h264-library must not be empty")
    if not args.usb_h264_ffmpeg:
        parser.error("--usb-h264-ffmpeg must not be empty")
    if not args.usb_h264_ffmpeg_encoder:
        parser.error("--usb-h264-ffmpeg-encoder must not be empty")
    if not args.usb_h264_device:
        parser.error("--usb-h264-device must not be empty")
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


def main(*, exit_on_error: bool = True) -> None:
    args = parse_args()
    encoder_source = apply_cluster_encoder_param(args)
    if args.usb_async and args.usb_codec != "jpeg":
        raise SystemExit("--usb-async only supports --usb-codec jpeg")
    target_fps = args.fps
    fps_source = "--fps" if args.fps_from_cli else "default"
    live_fps_param_reader = None
    if not args.fps_from_cli:
        fps_param_reader = ClusterLiveFpsParamReader()
        param_fps = fps_param_reader.read()
        if args.input == "live" or param_fps > 0:
            live_fps_param_reader = fps_param_reader
            target_fps = param_fps
            fps_source = CLUSTER_LIVE_FPS_PARAM
    if (
        args.output in ("usb", "both")
        and args.usb_codec == "h264"
        and not args.fps_from_cli
        and live_fps_param_reader is None
    ):
        target_fps = float(args.usb_h264_fps)
        fps_source = "--usb-h264-fps"
    usb_output_enabled = args.output in ("usb", "both")
    usb_display_fps = (
        resolved_usb_display_fps(
            args.usb_display_fps,
            args.usb_codec,
            target_fps,
            args.usb_h264_fps,
        )
        if usb_output_enabled
        else 0
    )
    usb_display_fps_auto = usb_output_enabled and args.usb_display_fps is None and args.usb_codec == "h264"
    usb_h264_bitrate = resolved_usb_h264_bitrate(args.usb_h264_bitrate, target_fps, args.usb_h264_fps)
    usb_h264_bitrate_auto = args.usb_h264_bitrate.strip().lower() == "auto"
    brightness_param_reader = None
    if args.usb_brightness_from_cli:
        usb_brightness = normalize_cluster_brightness_percent(args.usb_brightness)
        brightness_source = "--usb-brightness"
    else:
        brightness_param_reader = ClusterHudBrightnessParamReader()
        usb_brightness = brightness_param_reader.read()
        brightness_source = CLUSTER_BRIGHTNESS_PARAM
    fps_text = "uncapped" if target_fps == 0 else f"{target_fps:.1f} Hz"
    display_fps_text = (
        f"auto->{usb_display_fps}"
        if usb_display_fps_auto
        else ("off" if usb_display_fps == 0 else str(usb_display_fps))
    )
    h264_bitrate_text = ""
    if args.usb_codec == "h264":
        bitrate_text = f"auto->{usb_h264_bitrate}" if usb_h264_bitrate_auto else usb_h264_bitrate
        h264_bitrate_text = f" h264_bitrate={bitrate_text}"
        if args.usb_h264_rate_control != DEFAULT_H264_RATE_CONTROL:
            h264_bitrate_text += f" h264_rc={args.usb_h264_rate_control}"
        if args.usb_h264_realtime_priority:
            h264_bitrate_text += " h264_realtime=on"
        if args.usb_h264_diagnose_interval > 0:
            h264_bitrate_text += f" h264_diag={args.usb_h264_diagnose_interval:g}s"
    brightness_text = "auto" if brightness_param_reader is not None and usb_brightness == 0 else f"{usb_brightness}%"
    size_text = (
        f"{args.width or 'device'}x{args.height or 'device'}"
        if args.output in ("usb", "both")
        else f"{args.width or DESIGN_WIDTH}x{args.height or DESIGN_HEIGHT}"
    )
    print(
        f"Refreshing native raylib cluster UI at {fps_text} "
        f"input={args.input} output={args.output}: {size_text} "
        f"usb_codec={args.usb_codec} encoder_source={encoder_source}{h264_bitrate_text} "
        f"fps_source={fps_source} display_fps={display_fps_text} "
        f"brightness={brightness_text} brightness_source={brightness_source}"
    )
    try:
        run_demo(
            args.duration,
            target_fps,
            live_fps_param_reader,
            args.input,
            args.output,
            args.controller_index,
            args.width,
            args.height,
            usb_brightness,
            brightness_param_reader,
            usb_display_fps,
            usb_display_fps_auto,
            args.usb_codec,
            args.usb_jpeg_quality,
            args.usb_jpeg_encoder,
            args.usb_fast,
            args.usb_wait_frame_ack,
            args.usb_async,
            usb_h264_bitrate,
            args.usb_h264_fps,
            args.usb_h264_gop,
            args.usb_h264_backend,
            args.usb_h264_library,
            args.usb_h264_ffmpeg,
            args.usb_h264_ffmpeg_encoder,
            args.usb_h264_device,
            args.usb_h264_input_format,
            args.usb_h264_slice_max_bytes,
            args.usb_h264_rate_control,
            args.usb_h264_realtime_priority,
            args.usb_h264_orientation,
            args.usb_h264_align,
            args.usb_h264_encoder_align,
            args.usb_h264_chunk_size,
            args.usb_h264_wait_ack or args.usb_h264_soft_ack,
            args.usb_h264_soft_ack,
            args.usb_h264_dump,
            args.usb_h264_debug,
            args.usb_h264_diagnose_interval,
            args.usb_h264_test_pattern,
            args.usb_h264_test_pattern_nv12,
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
            not args.no_cluster_core_usage,
            args.cluster_core_usage_debug,
            args.profile_render,
            args.profile_interval,
            not args.no_gc_freeze,
            args.theme,
            args.cluster_hud_mode,
            args.cluster_hud_encoder,
            args.cluster_hud_core_mode,
            args.cluster_hud_priority,
        )
    except KeyboardInterrupt:
        print("\nStopped.")
    except RuntimeError as exc:
        if not exit_on_error:
            raise
        raise SystemExit(f"Error: {exc}") from exc


if __name__ == "__main__":
    main()
