from __future__ import annotations

import argparse
import gc
import os
from dataclasses import replace
import signal
import sys
import threading
import time
from pathlib import Path

from cluster_config import (
    CLUSTER_BRIGHTNESS_PARAM,
    CLUSTER_CAMERA_VIEW_MODE_PARAM,
    CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    CLUSTER_ENCODER_AUTO,
    CLUSTER_ENCODER_HARDWARE,
    CLUSTER_ENCODER_JPEG,
    CLUSTER_ENCODER_PARAM,
    CLUSTER_ENCODER_SOFTWARE,
    CLUSTER_HUD_MIRROR_PARAM,
    CLUSTER_CORE_MODE_PARAM,
    CLUSTER_HUD_DEBUG_PARAM,
    CLUSTER_HUD_PARAM,
    CLUSTER_LIVE_FPS_PARAM,
    CLUSTER_ORIENTATION_PARAM,
    CLUSTER_PANEL_LAYOUT_DRIVING_LEFT,
    CLUSTER_PANEL_LAYOUT_PARAM,
    CLUSTER_PRIORITY_PARAM,
    CLUSTER_RADAR_DISPLAY_PARAM,
    CLUSTER_RADAR_INFO_PARAM,
    CLUSTER_RADAR_SOURCE_COLOR_PARAM,
    CLUSTER_SCREEN_MODE_DEFAULT,
    CLUSTER_SCREEN_MODE_DEBUG,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
    CLUSTER_SCREEN_MODE_NAVI,
    CLUSTER_SCREEN_MODE_PARAM,
    CLUSTER_THEME_PARAM,
    DESIGN_HEIGHT,
    DESIGN_WIDTH,
    normalize_cluster_brightness_percent,
    cluster_camera_view_is_road_camera,
    normalize_cluster_camera_view_mode,
    normalize_cluster_core_mode,
    normalize_cluster_encoder_mode,
    normalize_cluster_live_fps,
    normalize_cluster_panel_layout,
    normalize_cluster_priority,
    normalize_cluster_radar_display_mode,
    normalize_cluster_radar_info_mode,
    normalize_cluster_radar_source_color_mode,
    normalize_cluster_screen_mode,
    normalize_cluster_theme_mode,
    resolved_usb_h264_bitrate,
)
from cluster_gamepad import DualSenseSimulator
from cluster_git_status import GitBranchStatusProvider
from cluster_gles_dmabuf import DirectNv12DmabufError
from cluster_gles_readback import DirectNv12ReadbackError
from cluster_display import CLUSTER_LANGUAGE_KO, normalize_cluster_language, normalize_metric_setting
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
from cluster_navi_overlay import merge_navi_overlay_state
from cluster_profile import GcProfileHook, ProfileReporter, freeze_gc_after_init
from cluster_renderer import ClusterUiRenderer
from cluster_route_replay import (
    ROUTE_CORNER_SOURCE_CHOICES,
    ROUTE_CORNER_SOURCE_LIVE,
    ROUTE_FRONT_RADAR_ONLY_ENV,
    RouteReplaySource,
    adjacent_route_log_path,
)
from cluster_simulator import ClusterSimulator, RandomInputSource
from cluster_system_monitor import ClusterProcessCoreUsageSampler, NetworkAddressProvider
from cluster_usb_display import TuringUsbDisplay, find_supported_usb_product, product_id_for_hud_mode
from cluster_usb_pipeline import AsyncJpegUsbPipeline
from openpilot.selfdrive.controls.lib.cutin_alert import CutinAlertCandidate, CutinAlertTracker

DEFAULT_FPS = 0.0
DEFAULT_USB_BRIGHTNESS = 80
DEFAULT_H264_BITRATE = "auto"
DEFAULT_H264_GOP = 1
DEFAULT_H264_DIMENSION_ALIGN = 1
THEME_PARAM_POLL_SECONDS = 1.0
DISPLAY_PREF_PARAM_POLL_SECONDS = 1.0
CLOCK_VISIBILITY_PARAM_POLL_SECONDS = 1.0
FPS_PARAM_POLL_SECONDS = 1.0
BRIGHTNESS_PARAM_POLL_SECONDS = 0.1
SCREEN_MODE_PARAM_POLL_SECONDS = 1.0
CAMERA_VIEW_PARAM_POLL_SECONDS = 1.0
PANEL_LAYOUT_PARAM_POLL_SECONDS = 1.0
RADAR_PARAM_POLL_SECONDS = 1.0
HUD_MODE_PARAM_POLL_SECONDS = 1.0
HUD_MIRROR_PARAM_POLL_SECONDS = 1.0
HUD_OUTPUT_GATE_PARAM_POLL_SECONDS = 0.1

try:
    from openpilot.system.hardware import TICI
except Exception:
    TICI = False


def live_debug_panel_enabled(screen_mode: int) -> bool:
    return screen_mode == CLUSTER_SCREEN_MODE_DEBUG


def live_debug_plot_enabled(screen_mode: int) -> bool:
    return screen_mode in (CLUSTER_SCREEN_MODE_DEBUG_GRAPH, CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT)


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


def resolved_h264_encoder_fps(target_fps: float, h264_fps: int) -> int:
    return max(1, int(round(target_fps if target_fps > 0 else h264_fps)))


def option_present(argv: list[str], option: str) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in argv)


def route_log_kind_for_path(path: Path, fallback: str) -> str:
    name = path.name.lower()
    if name.startswith("qlog."):
        return "qlog"
    if name.startswith("rlog."):
        return "rlog"
    return fallback


def route_state_cutin_candidates(state: object) -> tuple[CutinAlertCandidate, ...]:
    detected_vehicles = getattr(state, "detected_vehicles", ()) or ()
    candidates = tuple(
        CutinAlertCandidate(
            int(-1 if getattr(vehicle, "radar_track_id", None) is None else getattr(vehicle, "radar_track_id")),
            float(getattr(vehicle, "longitudinal_m", 0.0) or 0.0),
            float(getattr(vehicle, "lateral_m", 0.0) or 0.0),
            float(getattr(vehicle, "relative_speed_mps", 0.0) or 0.0),
        )
        for vehicle in detected_vehicles
        if bool(getattr(vehicle, "cut_in", False))
        and str(getattr(vehicle, "source", "")) == "radarState"
    )
    if candidates:
        return candidates
    if bool(getattr(state, "recorded_cutin_active", False)):
        return (CutinAlertCandidate(-2, 0.0, 0.0, 0.0),)
    return ()


def route_state_has_cutin(state: object) -> bool:
    return bool(route_state_cutin_candidates(state))


def route_state_recorded_cutin_sound_candidates(state: object) -> tuple[CutinAlertCandidate, ...]:
    if not bool(getattr(state, "recorded_cutin_sound", False)):
        return ()
    return (CutinAlertCandidate(-2, 0.0, 0.0, 0.0),)


def play_cutin_alert() -> None:
    def play() -> None:
        if sys.platform == "win32":
            try:
                import winsound

                # Beep does not depend on the user's Windows sound theme.
                winsound.Beep(1400, 140)
                winsound.Beep(1900, 180)
                return
            except (ImportError, RuntimeError):
                pass
        print("\a", end="", flush=True)

    threading.Thread(target=play, name="cutin-alert", daemon=True).start()


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


class ClusterDisplayPreferencesParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> tuple[str, bool]:
        if self._params is None:
            return CLUSTER_LANGUAGE_KO, True
        try:
            language = normalize_cluster_language(
                self._params.get("LanguageSetting", return_default=True),
                default=CLUSTER_LANGUAGE_KO,
            )
            is_metric = normalize_metric_setting(
                self._params.get("IsMetric", return_default=True),
                default=True,
            )
            return language, is_metric
        except Exception:
            return CLUSTER_LANGUAGE_KO, True


class ClusterClockVisibilityParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> bool:
        if self._params is None:
            return True
        try:
            return self._params.get_int("ShowDateTime") in (1, 2)
        except Exception:
            return True


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


class ClusterHudOrientationParamReader:
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
            orientation = int(self._params.get_int(CLUSTER_ORIENTATION_PARAM))
            return orientation if orientation in (0, 1, 2, 3) else 0
        except Exception:
            return 0


class ClusterHudMirrorParamReader:
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
            return max(0, min(3, self._params.get_int(CLUSTER_HUD_MIRROR_PARAM)))
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


class ClusterPanelLayoutParamReader:
    def __init__(self) -> None:
        self._params = None
        try:
            from openpilot.common.params import Params

            self._params = Params()
        except Exception:
            pass

    def read(self) -> int:
        if self._params is None:
            return CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
        try:
            return normalize_cluster_panel_layout(self._params.get_int(CLUSTER_PANEL_LAYOUT_PARAM))
        except Exception:
            return CLUSTER_PANEL_LAYOUT_DRIVING_LEFT


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


def route_overlay_for_mode(
    overlay: RouteOverlay | None,
    mode: str,
    *,
    keep_video: bool = False,
) -> RouteOverlay | None:
    if overlay is None or mode == "off":
        if overlay is not None and keep_video:
            return replace(overlay, panel_visible=False, data_lines=())
        return None
    if mode == "compact":
        return replace(overlay, panel_visible=True, data_lines=overlay.data_lines[:4])
    return replace(overlay, panel_visible=True)


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
    navi_host: str,
    navi_port: int,
    navi_advertise_ip: str | None,
    navi_beacon_enabled: bool,
    navi_map_theme: str,
    navi_overlay_enabled: bool,
    navi_publish_cereal: bool,
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
    route_corner_source: str,
    route_overlay_mode: str,
    route_tools_mode: str,
    camera_view_mode: int | None,
    panel_layout: str | int | None,
    route_loop: bool,
    route_pause_on_cutin: bool,
    route_replay_speed: float,
    route_start_time_s: float,
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
    screen_mode: str | None,
    hud_mode_watch: int | None,
    hud_encoder_watch: int | None,
    hud_core_mode_watch: int | None,
    hud_priority_watch: int | None,
    language: str | None,
    is_metric: bool | None,
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
    usb_orientation_param_reader = ClusterHudOrientationParamReader()
    initial_orientation = usb_orientation_param_reader.read()
    active_orientation = initial_orientation if initial_orientation in (0, 2) else 0

    hud_mirror_param_reader = ClusterHudMirrorParamReader()
    active_hud_mirror_mode = hud_mirror_param_reader.read()

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
        # H.264 startup command 13 carries this state; do not add another
        # mandatory sync transaction immediately after USB initialization.
        usb_display.set_orientation(active_orientation)
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
    display_pref_param_reader = (
        ClusterDisplayPreferencesParamReader()
        if language is None or is_metric is None
        else None
    )
    param_language, param_is_metric = (
        display_pref_param_reader.read()
        if display_pref_param_reader is not None
        else (CLUSTER_LANGUAGE_KO, True)
    )
    active_language = (
        normalize_cluster_language(language, default=CLUSTER_LANGUAGE_KO)
        if language is not None
        else param_language
    )
    active_is_metric = bool(is_metric) if is_metric is not None else param_is_metric
    clock_visibility_param_reader = ClusterClockVisibilityParamReader()
    active_clock_visible = clock_visibility_param_reader.read()
    screen_mode_override = normalize_cluster_screen_mode(screen_mode) if screen_mode is not None else None
    screen_mode_param_reader = (
        ClusterScreenModeParamReader()
        if input_mode != "navi" and screen_mode_override is None
        else None
    )
    if input_mode == "navi":
        active_screen_mode = CLUSTER_SCREEN_MODE_NAVI
    elif screen_mode_override is not None:
        active_screen_mode = screen_mode_override
    elif screen_mode_param_reader is not None:
        active_screen_mode = screen_mode_param_reader.read()
    else:
        active_screen_mode = CLUSTER_SCREEN_MODE_DEFAULT
    camera_view_override = (
        normalize_cluster_camera_view_mode(camera_view_mode)
        if camera_view_mode is not None
        else None
    )
    camera_view_param_reader = ClusterCameraViewModeParamReader() if camera_view_override is None else None
    active_camera_view_mode = (
        camera_view_override
        if camera_view_override is not None
        else camera_view_param_reader.read()
    )
    panel_layout_override = (
        normalize_cluster_panel_layout(panel_layout)
        if panel_layout is not None
        else None
    )
    panel_layout_param_reader = ClusterPanelLayoutParamReader() if panel_layout_override is None else None
    active_panel_layout = (
        panel_layout_override
        if panel_layout_override is not None
        else panel_layout_param_reader.read()
    )
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
        panel_layout=active_panel_layout,
        language=active_language,
        is_metric=active_is_metric,
    )
    print(
        f"Display preferences initial: language={active_language} units={'metric' if active_is_metric else 'imperial'}",
        flush=True,
    )
    print(f"ShowDateTime external HUD clock: {'on' if active_clock_visible else 'off'}", flush=True)
    print(f"{CLUSTER_SCREEN_MODE_PARAM} initial: {active_screen_mode}", flush=True)
    print(f"{CLUSTER_CAMERA_VIEW_MODE_PARAM} initial: {active_camera_view_mode}", flush=True)
    print(f"{CLUSTER_PANEL_LAYOUT_PARAM} initial: {active_panel_layout}", flush=True)
    print(
        f"{CLUSTER_RADAR_INFO_PARAM} initial: {active_radar_info_mode} "
        f"{CLUSTER_RADAR_DISPLAY_PARAM} initial: {active_radar_display_mode} "
        f"{CLUSTER_RADAR_SOURCE_COLOR_PARAM} initial: {active_radar_source_color_mode}",
        flush=True,
    )
    renderer.set_profile_enabled(profile_render)
    git_status_provider = GitBranchStatusProvider(Path(__file__).resolve().parent)
    network_address_provider = NetworkAddressProvider()
    cluster_core_usage_sampler = (
        ClusterProcessCoreUsageSampler(debug=cluster_core_usage_debug)
        if cluster_core_usage_enabled
        else None
    )
    simulator = ClusterSimulator() if input_mode in ("random", "gamepad") else None
    controller = DualSenseSimulator(controller_index) if input_mode == "gamepad" else None
    random_input = RandomInputSource() if input_mode == "random" else None
    live_source = OpenpilotLiveSource(include_can=live_include_can, timeout_ms=live_timeout_ms) if input_mode == "live" else None
    navi_source = None
    if input_mode == "navi" or navi_overlay_enabled:
        from cluster_navi_source import NaviSimulatorSource

        navi_source = NaviSimulatorSource(
            host=navi_host,
            port=navi_port,
            advertise_ip=navi_advertise_ip,
            beacon_enabled=navi_beacon_enabled,
            publish_cereal=navi_publish_cereal,
            map_theme=navi_map_theme,
        )
    if live_source is not None:
        live_source.set_profile_enabled(profile_render)
        live_source.set_hud_debug_mode(active_hud_debug_mode)
        live_source.set_debug_panels_enabled(
            live_debug=live_debug_panel_enabled(active_screen_mode),
            debug_plot=live_debug_plot_enabled(active_screen_mode),
            navi_debug=False,
        )
    route_source = None
    if input_mode == "route":
        profile_stage = time.perf_counter()
        try:
            route_source = RouteReplaySource.load(
                route_path,
                route_log,
                route_start_segment,
                route_max_segments,
                0.0,
                route_corner_source,
                0.0,
            )
        except Exception:
            if navi_source is not None:
                navi_source.close()
            if live_source is not None:
                live_source.close()
            raise
        profile.add_elapsed("source.route_load_initial", profile_stage)
    if route_source is not None:
        print(
            f"Loaded route replay buffer: {len(route_source.frames)} frames, "
            f"{route_source.duration:.1f}s from "
            f"{route_source.loaded_file_count}/{len(route_source.source_files)} {route_log} files"
        )
    route_tools_window = None
    start_time = time.perf_counter()
    route_wall_base_time = start_time
    route_playback_base_s = min(max(0.0, route_start_time_s), route_source.duration) if route_source is not None else 0.0
    route_paused = False
    route_pause_toggled_down = False
    route_active_corner_lateral_offset_m = 0.0
    active_route_log_kind = route_log
    active_route_overlay_mode = route_overlay_mode
    route_next_retry_time = 0.0
    route_end_waiting = False
    route_cutin_alert_tracker = CutinAlertTracker()
    route_options = {
        "front_radar_only": os.environ.get(ROUTE_FRONT_RADAR_ONLY_ENV) == "1",
        "route_loop": route_loop,
        "pause_on_cutin": route_pause_on_cutin,
        "show_route_overlay": active_route_overlay_mode != "off",
        "road_camera": cluster_camera_view_is_road_camera(active_camera_view_mode),
    }
    last_frame_time = start_time
    last_report_time = start_time
    next_theme_param_read = start_time
    next_display_pref_param_read = start_time
    next_clock_visibility_param_read = start_time
    next_fps_param_read = start_time + FPS_PARAM_POLL_SECONDS
    next_brightness_param_read = start_time
    next_screen_mode_param_read = start_time
    next_camera_view_param_read = start_time
    next_panel_layout_param_read = start_time
    next_radar_param_read = start_time
    next_hud_mode_param_read = start_time
    next_hud_output_gate_param_read = start_time
    next_hud_mirror_param_read = start_time + HUD_MIRROR_PARAM_POLL_SECONDS
    report_frames = 0
    display_actual_fps: float | None = None
    frame_interval = 1.0 / target_fps if target_fps > 0 else 0.0
    h264_test_pattern_rgba: bytearray | None = None
    h264_test_pattern_nv12: bytearray | None = None
    h264_render_nv12_buffer: bytearray | None = None
    h264_render_nv12_layout: tuple[int, int, int, int, int, int, bool] | None = None
    h264_direct_nv12_input = False
    h264_async_nv12_input = False
    h264_dmabuf_nv12_input = False

    def switch_route_source(
        new_path: Path,
        switch_time: float,
        resume_s: float = 0.0,
        paused: bool = False,
    ) -> bool:
        nonlocal route_source
        nonlocal route_path, active_route_log_kind
        nonlocal route_playback_base_s, route_wall_base_time, route_paused
        nonlocal route_next_retry_time, route_end_waiting

        new_path = new_path.resolve()
        new_log_kind = route_log_kind_for_path(new_path, active_route_log_kind)
        try:
            new_source = RouteReplaySource.load(
                new_path,
                new_log_kind,
                None,
                1,
                0.0,
                route_corner_source,
                route_active_corner_lateral_offset_m,
            )
        except Exception as exc:
            print(f"Failed to open replay log {new_path}: {exc}", flush=True)
            return False

        old_source = route_source
        route_source = new_source
        route_path = new_path
        active_route_log_kind = new_log_kind
        route_playback_base_s = min(max(0.0, resume_s), new_source.duration)
        route_wall_base_time = switch_time
        route_paused = paused
        route_next_retry_time = 0.0
        route_end_waiting = False
        route_cutin_alert_tracker.reset()
        if old_source is not None:
            old_source.close()
        print(
            f"Playing replay log: {new_source.source_files[0].parent} "
            f"({new_source.duration:.1f}s)",
            flush=True,
        )
        return True

    try:
        if hud_output_gate_param_reader is not None and not hud_output_gate_param_reader.allowed():
            print(
                f"{CLUSTER_HUD_DEBUG_PARAM}=0 and IsOnroad=0; "
                "cluster HUD output remains off",
                flush=True,
            )
            return

        if route_source is not None and route_tools_mode == "separate":
            from cluster_replay_tools import RouteReplayToolsWindow

            route_tools_window = RouteReplayToolsWindow()
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
                h264_direct_nv12_input = (
                    h264_pipeline.native_direct_input_available()
                    and renderer.direct_nv12_readback_available()
                )
                h264_dmabuf_nv12_input = (
                    h264_pipeline.native_dmabuf_input_available()
                    and renderer.nv12_dmabuf_output_available()
                )
                h264_async_nv12_input = (
                    h264_direct_nv12_input
                    and renderer.async_nv12_readback_available()
                )
                print(
                    f"Using H264 native NV12 render path: "
                    f"{h264_pipeline.encoder_width}x{h264_pipeline.encoder_height} "
                    f"stride={stride} scanlines={y_scanlines}/{uv_scanlines} "
                    f"uv_offset={uv_offset} bytes={input_bytes} render_bytes={render_bytes} "
                    f"active_submit={'on' if active_submit else 'off'} "
                    f"dmabuf_output={'on' if h264_dmabuf_nv12_input else 'off'} "
                    f"direct_ion={'on' if h264_direct_nv12_input else 'off'} "
                    f"async_pbo={'on' if h264_async_nv12_input else 'off'} flip_x=on",
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

            if (
                hud_output_gate_param_reader is not None
                and now >= next_hud_output_gate_param_read
            ):
                if not hud_output_gate_param_reader.allowed():
                    print(
                        f"{CLUSTER_HUD_DEBUG_PARAM}=0 and IsOnroad=0; "
                        "exiting to turn off cluster HUD output",
                        flush=True,
                    )
                    break
                next_hud_output_gate_param_read = now + HUD_OUTPUT_GATE_PARAM_POLL_SECONDS

            if now >= next_hud_mirror_param_read:
                next_hud_mirror_mode = hud_mirror_param_reader.read()
                if next_hud_mirror_mode != active_hud_mirror_mode:
                    print(
                        f"{CLUSTER_HUD_MIRROR_PARAM} updated: "
                        f"{active_hud_mirror_mode} -> {next_hud_mirror_mode}",
                        flush=True,
                    )
                    active_hud_mirror_mode = next_hud_mirror_mode
                next_hud_mirror_param_read = now + HUD_MIRROR_PARAM_POLL_SECONDS

            if theme_override is None and now >= next_theme_param_read:
                next_theme_mode = theme_param_reader.read() if theme_param_reader is not None else "auto"
                if next_theme_mode != renderer.theme_mode:
                    renderer.set_theme_mode(next_theme_mode)
                next_theme_param_read = now + THEME_PARAM_POLL_SECONDS
            if display_pref_param_reader is not None and now >= next_display_pref_param_read:
                param_language, param_is_metric = display_pref_param_reader.read()
                next_language = (
                    normalize_cluster_language(language, default=CLUSTER_LANGUAGE_KO)
                    if language is not None
                    else param_language
                )
                next_is_metric = bool(is_metric) if is_metric is not None else param_is_metric
                if next_language != renderer.language or next_is_metric != renderer.is_metric:
                    old_units = "metric" if renderer.is_metric else "imperial"
                    next_units = "metric" if next_is_metric else "imperial"
                    print(
                        f"Display preferences updated: {renderer.language}/{old_units} -> {next_language}/{next_units}",
                        flush=True,
                    )
                    renderer.set_display_preferences(next_language, next_is_metric)
                next_display_pref_param_read = now + DISPLAY_PREF_PARAM_POLL_SECONDS
            if now >= next_clock_visibility_param_read:
                next_clock_visible = clock_visibility_param_reader.read()
                if next_clock_visible != active_clock_visible:
                    old_clock_state = "on" if active_clock_visible else "off"
                    next_clock_state = "on" if next_clock_visible else "off"
                    print(
                        f"ShowDateTime external HUD clock: {old_clock_state} -> {next_clock_state}",
                        flush=True,
                    )
                    active_clock_visible = next_clock_visible
                next_clock_visibility_param_read = now + CLOCK_VISIBILITY_PARAM_POLL_SECONDS
            if screen_mode_param_reader is not None and now >= next_screen_mode_param_read:
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
                            navi_debug=False,
                        )
                next_screen_mode_param_read = now + SCREEN_MODE_PARAM_POLL_SECONDS
            if now >= next_camera_view_param_read:
                if camera_view_param_reader is not None:
                    next_camera_view_mode = camera_view_param_reader.read()
                    if next_camera_view_mode != active_camera_view_mode:
                        print(
                            f"{CLUSTER_CAMERA_VIEW_MODE_PARAM} updated: "
                            f"{active_camera_view_mode} -> {next_camera_view_mode}",
                            flush=True,
                        )
                        active_camera_view_mode = next_camera_view_mode
                next_camera_view_param_read = now + CAMERA_VIEW_PARAM_POLL_SECONDS
            if panel_layout_param_reader is not None and now >= next_panel_layout_param_read:
                next_panel_layout = panel_layout_param_reader.read()
                if next_panel_layout != renderer.panel_layout:
                    print(
                        f"{CLUSTER_PANEL_LAYOUT_PARAM} updated: "
                        f"{renderer.panel_layout} -> {next_panel_layout}",
                        flush=True,
                    )
                    renderer.set_panel_layout(next_panel_layout)
                next_panel_layout_param_read = now + PANEL_LAYOUT_PARAM_POLL_SECONDS
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
                if (
                    hud_output_gate_param_reader is not None
                    and live_source.onroad_state() is False
                    and hud_output_gate_param_reader.read_mode() == 0
                ):
                    print(
                        f"{CLUSTER_HUD_DEBUG_PARAM}=0 and deviceState.started=0; "
                        "exiting to turn off cluster HUD output",
                        flush=True,
                    )
                    break
                center_clock_text = time.strftime("%H:%M:%S") if active_clock_visible else None
                profile.add_samples(live_source.profile_samples())
                profile.add_elapsed("source.live_update", profile_stage)
            elif input_mode == "navi" and navi_source is not None:
                profile_stage = time.perf_counter()
                state = navi_source.update()
                source_status = navi_source.status_text()
                profile.add_elapsed("source.navi_update", profile_stage)
            elif route_source is not None:
                profile_stage = time.perf_counter()
                playback_seconds = (
                    route_playback_base_s
                    if route_paused
                    else route_playback_base_s + (now - route_wall_base_time) * route_replay_speed
                )
                action = None
                if route_tools_window is not None:
                    action = route_tools_window.poll()
                    if action is not None:
                        reload_parser_options = False
                        for option_name, option_value in action.option_updates:
                            if option_name not in route_options:
                                continue
                            route_options[option_name] = option_value
                            if option_name == "front_radar_only":
                                os.environ[ROUTE_FRONT_RADAR_ONLY_ENV] = "1" if option_value else "0"
                                reload_parser_options = True
                            elif option_name == "route_loop":
                                route_loop = option_value
                                if route_loop and route_end_waiting:
                                    route_end_waiting = False
                                    route_paused = False
                                    route_playback_base_s = 0.0
                                    route_wall_base_time = now
                                    playback_seconds = 0.0
                            elif option_name == "pause_on_cutin":
                                if not option_value:
                                    route_cutin_alert_tracker.reset()
                            elif option_name == "show_route_overlay":
                                active_route_overlay_mode = "compact" if option_value else "off"
                            elif option_name == "road_camera":
                                active_camera_view_mode = CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA if option_value else 0

                        if action.toggle_pause:
                            route_paused = not route_paused
                            route_playback_base_s = playback_seconds
                            route_wall_base_time = now
                        if action.seek_s is not None:
                            route_playback_base_s = action.seek_s
                            route_wall_base_time = now
                            playback_seconds = action.seek_s
                            route_paused = True
                            route_end_waiting = False
                            route_cutin_alert_tracker.reset()

                        requested_path = Path(action.open_path) if action.open_path is not None else None
                        if requested_path is None and (action.previous_log or action.next_log):
                            direction = -1 if action.previous_log else 1
                            requested_path = adjacent_route_log_path(
                                route_source.log_path_at(playback_seconds),
                                direction,
                                active_route_log_kind,
                            )
                            if requested_path is None:
                                direction_name = "previous" if direction < 0 else "next"
                                print(f"No {direction_name} numbered replay log found", flush=True)

                        if requested_path is not None:
                            if switch_route_source(requested_path, now):
                                playback_seconds = 0.0
                        elif reload_parser_options:
                            current_path = route_source.log_path_at(playback_seconds)
                            current_offset = route_source.log_offset_at(playback_seconds)
                            if switch_route_source(current_path, now, current_offset, route_paused):
                                playback_seconds = route_playback_base_s

                        if action.closed:
                            route_tools_window.close()
                            route_tools_window = None
                    elif not route_tools_window.is_alive:
                        route_tools_window.close()
                        route_tools_window = None

                if route_tools_mode == "overlay" and output_mode in ("window", "both"):
                    seek_s, next_corner_lateral_offset_m, _control_active = renderer.route_replay_control_input(
                        playback_seconds,
                        route_source.duration,
                        route_active_corner_lateral_offset_m,
                    )
                    mouse_down = renderer.route_replay_mouse_down()
                    if mouse_down and not _control_active and not route_pause_toggled_down:
                        route_paused = not route_paused
                        route_playback_base_s = playback_seconds
                        route_wall_base_time = now
                    route_pause_toggled_down = mouse_down
                    if seek_s is not None:
                        route_playback_base_s = seek_s
                        route_wall_base_time = now
                        playback_seconds = seek_s
                        route_paused = True
                        route_end_waiting = False
                        route_cutin_alert_tracker.reset()
                    if next_corner_lateral_offset_m != route_active_corner_lateral_offset_m:
                        route_active_corner_lateral_offset_m = next_corner_lateral_offset_m
                        route_source.corner_lateral_offset_m = route_active_corner_lateral_offset_m
                        route_paused = True
                        route_playback_base_s = playback_seconds
                        route_wall_base_time = now
                elif output_mode in ("window", "both"):
                    mouse_down = renderer.route_replay_mouse_down()
                    if mouse_down and not route_pause_toggled_down:
                        route_paused = not route_paused
                        route_playback_base_s = playback_seconds
                        route_wall_base_time = now
                    route_pause_toggled_down = mouse_down

                if route_end_waiting or route_source.is_finished(playback_seconds, route_loop):
                    route_paused = True
                    route_playback_base_s = route_source.duration
                    route_wall_base_time = now
                    playback_seconds = route_source.duration
                    route_end_waiting = True
                    if now >= route_next_retry_time:
                        route_next_retry_time = now + 1.0
                        next_path = adjacent_route_log_path(
                            route_source.log_path_at(playback_seconds),
                            1,
                            active_route_log_kind,
                        )
                        if next_path is not None and switch_route_source(next_path, now):
                            playback_seconds = 0.0

                route_source.corner_lateral_offset_m = route_active_corner_lateral_offset_m
                keep_camera_video = cluster_camera_view_is_road_camera(active_camera_view_mode)
                state = route_source.state_at(
                    playback_seconds,
                    route_loop,
                    include_overlay=active_route_overlay_mode != "off" or keep_camera_video,
                    camera_view_mode=active_camera_view_mode,
                )
                state = replace(
                    state,
                    route_overlay=route_overlay_for_mode(
                        state.route_overlay,
                        active_route_overlay_mode,
                        keep_video=keep_camera_video,
                    ),
                )
                if route_options["pause_on_cutin"]:
                    if route_cutin_alert_tracker.update(route_state_recorded_cutin_sound_candidates(state)):
                        route_paused = True
                        route_playback_base_s = playback_seconds
                        route_wall_base_time = now
                        play_cutin_alert()
                        print(
                            f"LOG CUT-IN at {playback_seconds:.2f}s; replay paused",
                            flush=True,
                        )
                else:
                    route_cutin_alert_tracker.reset()
                source_status = route_source.status_text(playback_seconds, route_loop)
                if route_end_waiting:
                    source_status += " | waiting for next log"
                route_debug_overlay = state.route_overlay
                if route_tools_window is not None:
                    route_tools_window.update(
                        playback_seconds,
                        route_source.duration,
                        route_paused,
                        source_status,
                        route_debug_overlay,
                        str(route_source.log_folder_at(playback_seconds)),
                        route_options,
                    )
                if route_tools_mode != "overlay" and route_debug_overlay is not None:
                    state = replace(
                        state,
                        route_overlay=replace(route_debug_overlay, panel_visible=False),
                    )
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

            if navi_overlay_enabled and navi_source is not None:
                profile_stage = time.perf_counter()
                navi_overlay_state = navi_source.update()
                state = merge_navi_overlay_state(state, navi_overlay_state)
                navi_status = navi_source.status_text()
                source_status = f"{source_status} | {navi_status}" if source_status else navi_status
                profile.add_elapsed("source.navi_overlay_update", profile_stage)

            if live_source is None:
                center_clock_text = state.center_clock_text if active_clock_visible else None

            cluster_core_usage_text = None
            if cluster_core_usage_sampler is not None:
                profile_stage = time.perf_counter()
                cluster_core_usage_text = cluster_core_usage_sampler.sample_text(now)
                profile.add_elapsed("main.cluster_core_usage_sample", profile_stage)
            network_address = network_address_provider.address(now)
            state = replace(
                state,
                radar_info_mode=active_radar_info_mode,
                radar_display_mode=active_radar_display_mode,
                radar_source_color_mode=active_radar_source_color_mode,
                camera_view_mode=active_camera_view_mode,
                center_clock_text=center_clock_text,
                git_status=git_status_provider.status(),
                network_address=network_address,
                network_connected=network_address is not None,
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
                next_orientation = usb_orientation_param_reader.read()
                if next_orientation in (0, 2) and next_orientation != active_orientation:
                    if h264_pipeline is not None and hud_mode_watch is not None:
                        print(
                            f"{CLUSTER_ORIENTATION_PARAM} changed from "
                            f"{active_orientation} to {next_orientation}; exiting for H264 restart",
                            flush=True,
                        )
                        break
                    if usb_display.set_orientation(next_orientation):
                        active_orientation = next_orientation
                        print(f"{CLUSTER_ORIENTATION_PARAM} updated: {active_orientation}", flush=True)
                next_brightness_param_read = brightness_now + BRIGHTNESS_PARAM_POLL_SECONDS

            if output_mode in ("window", "both"):
                profile_stage = time.perf_counter()
                if route_source is not None and route_tools_mode == "overlay":
                    renderer.render_route_replay_frame(
                        state,
                        playback_seconds,
                        route_source.duration,
                        route_active_corner_lateral_offset_m,
                        route_paused,
                    )
                else:
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
                            native_frame_handled = False
                            if h264_dmabuf_nv12_input:
                                try:
                                    profile_stage = time.perf_counter()
                                    with h264_pipeline.native_nv12_input_buffer() as direct_input:
                                        profile.add_elapsed("main.usb_h264.acquire_dmabuf", profile_stage)
                                        if direct_input.dmabuf_fd < 0:
                                            raise DirectNv12DmabufError(
                                                "native H264 input lease did not expose a DMA-BUF fd"
                                            )

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
                                            flip_x=not bool(active_hud_mirror_mode & 1),
                                            destination_dmabuf_fd=direct_input.dmabuf_fd,
                                        ):
                                            pass
                                        profile.add_elapsed("main.usb.render_nv12_dmabuf_total", profile_stage)

                                        profile_stage = time.perf_counter()
                                        h264_pipeline.submit_native_nv12_dmabuf_input(direct_input)
                                        profile.add_elapsed("main.usb_h264.submit_dmabuf", profile_stage)
                                    native_frame_handled = True
                                except DirectNv12DmabufError as exc:
                                    h264_dmabuf_nv12_input = False
                                    renderer.disable_nv12_dmabuf_output()
                                    print(
                                        f"H264 DMA-BUF output unavailable; using asynchronous PBO readback: {exc}",
                                        flush=True,
                                    )

                            if not native_frame_handled and h264_async_nv12_input:
                                try:
                                    if renderer.async_nv12_readback_ready():
                                        profile_stage = time.perf_counter()
                                        with h264_pipeline.native_nv12_input_buffer() as direct_input:
                                            profile.add_elapsed("main.usb_h264.acquire_pbo", profile_stage)

                                            profile_stage = time.perf_counter()
                                            if not renderer.copy_async_nv12_readback(
                                                direct_input.address,
                                                direct_input.size,
                                            ):
                                                raise DirectNv12ReadbackError(
                                                    "ready NV12 PBO was unavailable during copy"
                                                )
                                            profile.add_elapsed("main.usb_h264.copy_pbo", profile_stage)

                                            profile_stage = time.perf_counter()
                                            h264_pipeline.submit_native_nv12_input(direct_input)
                                            profile.add_elapsed("main.usb_h264.submit_pbo", profile_stage)

                                    if renderer.async_nv12_readback_can_enqueue():
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
                                            flip_x=not bool(active_hud_mirror_mode & 1),
                                            async_readback=True,
                                        ):
                                            pass
                                        profile.add_elapsed("main.usb.render_nv12_pbo_total", profile_stage)
                                    else:
                                        profile.add("main.usb_h264.pbo_ring_full_drop", 0.0)
                                    native_frame_handled = True
                                except DirectNv12ReadbackError as exc:
                                    h264_async_nv12_input = False
                                    renderer.disable_async_nv12_readback()
                                    print(
                                        f"H264 asynchronous PBO readback unavailable; using synchronous direct ION: {exc}",
                                        flush=True,
                                    )

                            if not native_frame_handled and h264_direct_nv12_input:
                                try:
                                    profile_stage = time.perf_counter()
                                    with h264_pipeline.native_nv12_input_buffer() as direct_input:
                                        profile.add_elapsed("main.usb_h264.acquire_direct", profile_stage)
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
                                            flip_x=not bool(active_hud_mirror_mode & 1),
                                            destination_address=direct_input.address,
                                            destination_size=direct_input.size,
                                        ):
                                            pass
                                        profile.add_elapsed("main.usb.render_nv12_total", profile_stage)

                                        profile_stage = time.perf_counter()
                                        h264_pipeline.submit_native_nv12_input(direct_input)
                                        profile.add_elapsed("main.usb_h264.submit_direct", profile_stage)
                                    native_frame_handled = True
                                except DirectNv12ReadbackError as exc:
                                    h264_direct_nv12_input = False
                                    h264_async_nv12_input = False
                                    renderer.disable_direct_nv12_readback()
                                    print(
                                        f"H264 direct ION readback unavailable; using staged NV12 fallback: {exc}",
                                        flush=True,
                                    )

                            if not native_frame_handled:
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
                                    flip_x=not bool(active_hud_mirror_mode & 1),
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
        network_address_provider.close()
        if cluster_core_usage_sampler is not None:
            cluster_core_usage_sampler.close()
        renderer.release_nv12_dmabuf_output()
        if h264_pipeline is not None:
            h264_pipeline.close()
        if usb_pipeline is not None:
            usb_pipeline.close()
        if controller is not None:
            controller.close()
        if route_tools_window is not None:
            route_tools_window.close()
        if route_source is not None:
            route_source.close()
        if live_source is not None:
            live_source.close()
        if navi_source is not None:
            navi_source.close()
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
        choices=("random", "gamepad", "route", "live", "navi"),
        default="random",
        help=(
            "Input source. Use navi for the embedded TCP 7714 Carrot receiver, "
            "live for openpilot cereal data, or route to replay logs."
        ),
    )
    parser.add_argument("--navi-host", default="0.0.0.0", help="Bind host for --input navi. Default: 0.0.0.0.")
    parser.add_argument("--navi-port", type=int, default=7714, help="TCP port for --input navi. Default: 7714.")
    parser.add_argument(
        "--navi-advertise-ip",
        default=None,
        help="Address broadcast to the Android app for --input navi. Default: detected LAN address.",
    )
    parser.add_argument(
        "--navi-no-beacon",
        action="store_true",
        help="Disable UDP 7705 discovery broadcast in --input navi mode.",
    )
    parser.add_argument(
        "--navi-map-theme",
        choices=("dark", "auto", "light"),
        default="dark",
        help="Theme requested for the smartphone-rendered map. Default: dark.",
    )
    parser.add_argument(
        "--navi-overlay",
        action="store_true",
        help="Receive live Carrot navigation while keeping the selected route/live vehicle input.",
    )
    parser.add_argument(
        "--navi-publish-cereal",
        action="store_true",
        help=argparse.SUPPRESS,
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
            "Target H264 bitrate for --usb-codec h264. Default auto preserves the 7M@30 FPS "
            "per-frame budget through 60 FPS; 60 FPS resolves to 14M."
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
        "--route-corner-source",
        choices=ROUTE_CORNER_SOURCE_CHOICES,
        default=ROUTE_CORNER_SOURCE_LIVE,
        help=" ".join((
            "Corner-radar replay source. live uses recorded liveTracks;",
            "stable reconstructs physical tracks from raw CAN; raw displays untracked CAN slots.",
        )),
    )
    parser.add_argument(
        "--route-overlay",
        choices=("compact", "full", "off"),
        default="compact",
        help="Route replay debug overlay. Default compact shows the replay camera/data panel; use off for performance tests.",
    )
    parser.add_argument(
        "--route-tools",
        choices=("overlay", "separate", "off"),
        default="overlay",
        help="Replay seek/debug UI placement. separate keeps the cluster frame clean in a second PC window.",
    )
    parser.add_argument(
        "--camera-view-mode",
        type=int,
        choices=(0, 1, 2, 3, 4),
        default=None,
        help=f"Camera view override. Default reads {CLUSTER_CAMERA_VIEW_MODE_PARAM}; 2 is narrow, 3 is wide, and 4 switches by speed.",
    )
    parser.add_argument(
        "--panel-layout",
        default=None,
        help=(
            "Panel layout override: driving-left (default) or driving-right. "
            f"Default reads {CLUSTER_PANEL_LAYOUT_PARAM}."
        ),
    )
    parser.add_argument(
        "--theme",
        choices=("auto", "dark", "light"),
        default=None,
        help=f"HUD theme override. Default reads {CLUSTER_THEME_PARAM}: 0 auto, 1 dark, 2 light.",
    )
    parser.add_argument(
        "--language",
        choices=("ko", "en"),
        default=None,
        help="Cluster text language override. Default follows LanguageSetting.",
    )
    unit_group = parser.add_mutually_exclusive_group()
    unit_group.add_argument(
        "--metric",
        dest="is_metric",
        action="store_true",
        help="Show speeds and distances in km/h, m, and km.",
    )
    unit_group.add_argument(
        "--imperial",
        dest="is_metric",
        action="store_false",
        help="Show speeds and distances in mph, ft, and mi.",
    )
    parser.set_defaults(is_metric=None)
    parser.add_argument(
        "--screen-mode",
        default=None,
        help=f"HUD screen override: 3d-fullscreen, default, debug-system, trip-report, or navi; default reads {CLUSTER_SCREEN_MODE_PARAM}.",
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
        "--route-pause-on-cutin",
        action="store_true",
        help="Play an alert and pause route replay on the first frame of any cut-in detection.",
    )
    parser.add_argument(
        "--route-replay-speed",
        type=float,
        default=1.0,
        help="Route playback speed multiplier.",
    )
    parser.add_argument(
        "--route-start-time",
        type=float,
        default=0.0,
        help="Initial route playback position in seconds.",
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
            "Keep live CAN/sendcan subscriptions disabled. This is the default for the in-car HUD."
        ),
    )
    parser.add_argument(
        "--live-include-can",
        action="store_true",
        help=(
            "Enable live CAN/sendcan subscriptions for PC/debug runs. This adds raw CAN parsing load."
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
    if not 1 <= args.navi_port <= 65535:
        parser.error("--navi-port must be between 1 and 65535")
    if args.navi_publish_cereal and args.input != "navi" and not args.navi_overlay:
        parser.error("--navi-publish-cereal requires --input navi or --navi-overlay")
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
    if args.route_start_time < 0.0:
        parser.error("--route-start-time must be 0 or greater")
    if args.route_max_segments is not None and args.route_max_segments <= 0:
        parser.error("--route-max-segments must be greater than 0")
    if args.profile_interval <= 0:
        parser.error("--profile-interval must be greater than 0")
    return args


def main(*, exit_on_error: bool = True) -> None:
    args = parse_args()
    if args.output in ("usb", "both") and not TICI:
        expected_product_id = product_id_for_hud_mode(args.cluster_hud_mode) if args.cluster_hud_mode is not None else None
        if find_supported_usb_product(expected_product_id) is None:
            print(
                "TURZX USB cluster display not found on PC; rendering to window only.",
                flush=True,
            )
            args.output = "window"
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
        f"navi_overlay={'on' if args.navi_overlay else 'off'} "
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
            args.navi_host,
            args.navi_port,
            args.navi_advertise_ip,
            not args.navi_no_beacon,
            args.navi_map_theme,
            args.navi_overlay,
            args.navi_publish_cereal,
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
            args.route_corner_source,
            args.route_overlay,
            args.route_tools,
            args.camera_view_mode,
            args.panel_layout,
            args.route_loop,
            args.route_pause_on_cutin,
            args.route_replay_speed,
            args.route_start_time,
            args.route_start_segment,
            args.route_max_segments,
            bool(args.live_include_can and not args.live_no_can),
            args.live_timeout_ms,
            not args.no_cluster_core_usage,
            args.cluster_core_usage_debug,
            args.profile_render,
            args.profile_interval,
            not args.no_gc_freeze,
            args.theme,
            args.screen_mode,
            args.cluster_hud_mode,
            args.cluster_hud_encoder,
            args.cluster_hud_core_mode,
            args.cluster_hud_priority,
            args.language,
            args.is_metric,
        )
    except KeyboardInterrupt:
        print("\nStopped.")
    except RuntimeError as exc:
        if not exit_on_error:
            raise
        raise SystemExit(f"Error: {exc}") from exc


if __name__ == "__main__":
    main()
