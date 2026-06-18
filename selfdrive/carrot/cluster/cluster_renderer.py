from __future__ import annotations

from collections import OrderedDict
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from functools import lru_cache
import base64
import math
import os
import time
from pathlib import Path

import pyray as rl

from cluster_config import (
    AMBER,
    BLUE,
    BLUE_SOFT,
    CLUSTER_RADAR_INFO_ALL_SPEED,
    CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
    CLUSTER_RADAR_INFO_NONE,
    CLUSTER_RADAR_INFO_VEHICLE_SPEED,
    CLUSTER_RADAR_INFO_VEHICLE_SPEED_DISTANCE,
    CLUSTER_RADAR_SOURCE_COLOR_BY_SOURCE,
    CLUSTER_SCREEN_MODE_DEBUG,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
    CLUSTER_SCREEN_MODE_NAVI_DEBUG,
    CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
    ClusterTheme,
    DESIGN_HEIGHT,
    DESIGN_WIDTH,
    EGO_FORWARD_M,
    GREEN,
    MAX_ACCEL_MPS2,
    MAX_SPEED_KPH,
    RED,
    TEXT,
    WHITE,
    current_cluster_theme,
    normalize_cluster_screen_mode,
    normalize_cluster_theme_mode,
)
from cluster_models import (
    ClusterUiState,
    DebugPlotSnapshot,
    GitBranchStatus,
    LiveDebugInfo,
    NaviDebugInfo,
    NaviGuidanceImage,
    NaviTrafficLightInfo,
    RouteOverlay,
)
from cluster_scene import (
    ClusterScene,
    MeshStrip,
    RADAR_STATIC_OBJECT_SPEED_KPH,
    RadarPointMarker,
    Vec3,
    VehicleBox,
    build_cluster_scene,
)
from cluster_system_monitor import SystemStats, SystemStatsSampler
from cluster_utils import blink_visible, clamp


CLUSTER_DIR = Path(__file__).resolve().parent
SELFDRIVE_DIR = CLUSTER_DIR.parents[1]
OPENPILOT_FONT_DIR = SELFDRIVE_DIR / "assets" / "fonts"
OPENPILOT_ADDON_FONT_DIR = SELFDRIVE_DIR / "assets" / "addon" / "font"
KAIGEN_GOTHIC_KR_BOLD_FONT_PATH = OPENPILOT_FONT_DIR / "KaiGenGothicKR-Bold.ttf"
JETBRAINS_MONO_FONT_PATH = OPENPILOT_FONT_DIR / "JetBrainsMono-Medium.ttf"
VEHICLE_MODEL_PATH = CLUSTER_DIR / "assets" / "models" / "cybertruck" / "cybertruck_cluster.obj"
FOLLOW_VEHICLE_ICON_PATH = SELFDRIVE_DIR / "assets" / "icons_mici" / "carrot_cruse_gap_trimmed.png"
LFA_ICON_PATH = SELFDRIVE_DIR / "assets" / "icons_mici" / "carrot_wheel_org.png"
ACCEL_TEXT_WIDTH_SAMPLES = ("+00.00", "-00.00")
TURN_SIGNAL_LEFT_CENTER_X = 610
TURN_SIGNAL_RIGHT_CENTER_X = 1310
TURN_SIGNAL_CENTER_Y = 72
TURN_SIGNAL_HEAD_HALF_HEIGHT = 38
TURN_SIGNAL_MID_CENTER_X = (TURN_SIGNAL_LEFT_CENTER_X + TURN_SIGNAL_RIGHT_CENTER_X) * 0.5
DRIVE_STATUS_BASE_BOX_SIZE = 46.0
DRIVE_STATUS_ROW_HEIGHT = TURN_SIGNAL_HEAD_HALF_HEIGHT * 2.0
DRIVE_STATUS_SCALE = DRIVE_STATUS_ROW_HEIGHT / DRIVE_STATUS_BASE_BOX_SIZE
GEAR_STATUS_CENTER_X = TURN_SIGNAL_LEFT_CENTER_X + 102
GEAR_STATUS_CENTER_Y = TURN_SIGNAL_CENTER_Y
GEAR_STATUS_BOX_SIZE = DRIVE_STATUS_ROW_HEIGHT * 0.82
GEAR_STATUS_FONT_SIZE = 34.0 * DRIVE_STATUS_SCALE * 0.82
GEAR_STATUS_OUTLINE_WIDTH = 2.0 * DRIVE_STATUS_SCALE
FOLLOW_STATUS_CENTER_X = GEAR_STATUS_CENTER_X + 132
FOLLOW_STATUS_W = 160
FOLLOW_STATUS_H = 42.0 * DRIVE_STATUS_SCALE
FOLLOW_STATUS_GAP_BARS = 4
FOLLOW_GAP_ACTIVE = (187, 61, 145, 255)
FOLLOW_GAP_INACTIVE = (118, 122, 128, 150)
FOLLOW_GAP_BAR_W = 5.4
FOLLOW_GAP_BAR_H = 7.7
FOLLOW_GAP_BAR_R = 1.3
FOLLOW_GAP_BAR_SCALE = 1.75 * DRIVE_STATUS_SCALE
FOLLOW_GAP_BAR_STEP_X = 6.3
FOLLOW_GAP_ICON_ASPECT = 44.0 / 27.5
FOLLOW_GAP_ICON_H = 32.0 * DRIVE_STATUS_SCALE
FOLLOW_GAP_ICON_W = FOLLOW_GAP_ICON_H * FOLLOW_GAP_ICON_ASPECT
TOP_CRUISE_CENTER_X = FOLLOW_STATUS_CENTER_X + 202
TOP_CRUISE_FONT_SIZE = 27.0 * DRIVE_STATUS_SCALE
TOP_CRUISE_UNIT_FONT_SIZE = TOP_CRUISE_FONT_SIZE
LFA_STATUS_CENTER_X = TOP_CRUISE_CENTER_X + 142
LFA_STATUS_ICON_SIZE = 28.0 * DRIVE_STATUS_SCALE
TOP_ICON_SIZE = 34.0 * DRIVE_STATUS_SCALE
DRIVE_STATUS_BOX_RADIUS = 8.0 * DRIVE_STATUS_SCALE
SPEED_VALUE_CENTER_X = 260
SPEED_VALUE_CENTER_Y = 230
SPEED_LIMIT_SIGN_CENTER_X = 460
SPEED_LIMIT_SIGN_CENTER_Y = TURN_SIGNAL_CENTER_Y
SPEED_LIMIT_SIGN_RADIUS = 56.0
SPEED_LIMIT_SOURCE_LABELS = {
    "vehicle": "v",
    "car": "v",
    "v": "v",
    "nav": "n",
    "navigation": "n",
    "n": "n",
    "model": "m",
    "m": "m",
    "vision": "vis",
    "vis": "vis",
    "sim": "sim",
}
SYSTEM_PANEL_X = 1416
SYSTEM_PANEL_Y = 118
SYSTEM_PANEL_W = 476
NAVI_TRAFFIC_PANEL_RIGHT = TURN_SIGNAL_RIGHT_CENTER_X + 96
NAVI_TRAFFIC_PANEL_H = 90
NAVI_TRAFFIC_PANEL_Y = TURN_SIGNAL_CENTER_Y + TURN_SIGNAL_HEAD_HALF_HEIGHT + 10
NAVI_TRAFFIC_SIGNAL_SIZE = 58.0
NAVI_TRAFFIC_SIGNAL_GAP = 10.0
NAVI_TRAFFIC_TEXT_GAP = 14.0
NAVI_TRAFFIC_PANEL_PAD_X = 16.0
NAVI_TRAFFIC_BG_LIGHT = (62, 68, 81)
NAVI_TRAFFIC_BG_DARK = (18, 21, 27)
NAVI_TRAFFIC_BG_OUTLINE = (238, 241, 246)
NAVI_TRAFFIC_OFF_LIGHT = (40, 43, 51)
NAVI_TRAFFIC_OFF_DARK = (36, 39, 47)
NAVI_TRAFFIC_OFF_ARROW = (58, 61, 70)
NAVI_TRAFFIC_RED = (255, 111, 111)
NAVI_TRAFFIC_GREEN = (103, 255, 78)
NAVI_GUIDANCE_IMAGE_X = SYSTEM_PANEL_X + 24
NAVI_GUIDANCE_IMAGE_Y = SYSTEM_PANEL_Y + 210
NAVI_GUIDANCE_IMAGE_W = SYSTEM_PANEL_W - 48
NAVI_GUIDANCE_IMAGE_H = 270
SYSTEM_STATS_REFRESH_SECONDS = 1.0
TEXT_MEASURE_CACHE_LIMIT = 1024
TRIANGLE_STRIP_POINT_CACHE_LIMIT = 256
DEBUG_PLOT_MAX_SAMPLES = 360
DEBUG_PLOT_SAMPLE_SECONDS = 0.05
DEBUG_PLOT_MARGIN = 18.0
DEBUG_PLOT_FULL_X = 500.0
DEBUG_PLOT_FULL_Y = DEBUG_PLOT_MARGIN
DEBUG_PLOT_FULL_W = 1392.0
DEBUG_PLOT_FULL_H = DESIGN_HEIGHT - DEBUG_PLOT_MARGIN * 2.0
DEBUG_PLOT_RIGHT_X = SYSTEM_PANEL_X
DEBUG_PLOT_RIGHT_Y = DEBUG_PLOT_MARGIN
DEBUG_PLOT_RIGHT_W = SYSTEM_PANEL_W
DEBUG_PLOT_RIGHT_H = DESIGN_HEIGHT - DEBUG_PLOT_MARGIN * 2.0
GIT_STATUS_MARGIN = 2
GIT_STATUS_DOT_RADIUS = 7
GIT_STATUS_DOT_TEXT_GAP = 6
GIT_STATUS_MAX_TEXT_W = 610
FPS_STATUS_MARGIN = 4
FPS_STATUS_DOT_RADIUS = 7
FPS_STATUS_DOT_TEXT_GAP = 6
FPS_STATUS_MAX_TEXT_W = 220
CLUSTER_CORE_USAGE_MARGIN = 2
CLUSTER_CORE_USAGE_MAX_TEXT_W = 760
RADAR_LABEL_DISTANCE_FONT_SIZE = 16
RADAR_LABEL_SPEED_FONT_SIZE = 14
VEHICLE_BADGE_DISTANCE_FONT_SIZE = 17
VEHICLE_BADGE_SPEED_FONT_SIZE = 15
RADAR_LABEL_ANCHOR_Z_OFFSET_M = 0.30
VEHICLE_BADGE_ANCHOR_Z_OFFSET_M = 0.32
WORLD_LABEL_NEAR_M = 18.0
WORLD_LABEL_FAR_M = 180.0
WORLD_LABEL_MIN_SCALE = 0.56
WORLD_LABEL_TEXTURE_CACHE_LIMIT = 512
WORLD_LABEL_TEXTURE_SIZE_GRID = 0.25
WORLD_LABEL_TEXTURE_PADDING_PX = 4
VEHICLE_MATERIAL_COLORS: dict[str, tuple[int, int, int, int]] = {
    "body": (156, 166, 172, 255),
    "wheel": (18, 20, 22, 255),
    "besi_roda": (36, 38, 42, 255),
    "light": (184, 222, 255, 255),
    "stop_light": (226, 34, 28, 255),
    "riting": (255, 146, 20, 255),
    "Material": (136, 142, 148, 255),
    "Material.002": (68, 72, 78, 255),
    "Material.003": (18, 20, 22, 255),
    "Material.004": (18, 20, 22, 255),
    "Material.005": (18, 20, 22, 255),
    "Material.006": (18, 20, 22, 255),
}
DEFAULT_VEHICLE_MATERIAL_COLOR = (142, 150, 156, 255)
NV12_PACK_VERTEX_SHADER = """
attribute vec3 vertexPosition;
attribute vec2 vertexTexCoord;
attribute vec4 vertexColor;

varying vec2 fragTexCoord;
varying vec4 fragColor;

uniform mat4 mvp;

void main() {
    fragTexCoord = vertexTexCoord;
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""
NV12_PACK_FRAGMENT_SHADER = """
#ifdef GL_ES
precision mediump float;
#endif

varying vec2 fragTexCoord;
varying vec4 fragColor;

uniform sampler2D texture0;
uniform vec2 srcSize;
uniform vec2 packedSize;
uniform int plane;
uniform int flipX;

const float Y_PAD = 0.062745;
const float UV_PAD = 0.501961;

vec3 sampleRgb(float x, float y) {
    if (flipX != 0) {
        // The portrait upload transform maps screen horizontal correction to source Y.
        y = srcSize.y - 1.0 - y;
    }
    vec2 clamped = clamp(vec2(x, y), vec2(0.0), srcSize - vec2(1.0));
    return texture2D(texture0, (clamped + vec2(0.5)) / srcSize).rgb;
}

float y601(vec3 rgb) {
    return clamp(0.062745 + 0.256788 * rgb.r + 0.504129 * rgb.g + 0.097906 * rgb.b, 0.0, 1.0);
}

float u601(vec3 rgb) {
    return clamp(0.501961 - 0.148223 * rgb.r - 0.290993 * rgb.g + 0.439216 * rgb.b, 0.0, 1.0);
}

float v601(vec3 rgb) {
    return clamp(0.501961 + 0.439216 * rgb.r - 0.367788 * rgb.g - 0.071427 * rgb.b, 0.0, 1.0);
}

vec3 sample2x2(float x, float y) {
    return (
        sampleRgb(x, y) +
        sampleRgb(x + 1.0, y) +
        sampleRgb(x, y + 1.0) +
        sampleRgb(x + 1.0, y + 1.0)
    ) * 0.25;
}

float packedY(float x, float y) {
    if (x >= srcSize.x || y >= srcSize.y) {
        return Y_PAD;
    }
    return y601(sampleRgb(x, y));
}

vec2 packedUV(float x, float y) {
    if (x >= srcSize.x || y >= srcSize.y) {
        return vec2(UV_PAD, UV_PAD);
    }
    vec3 rgb = sample2x2(x, y);
    return vec2(u601(rgb), v601(rgb));
}

void main() {
    vec2 packedCoord = min(floor(fragTexCoord * packedSize), packedSize - vec2(1.0));
    float baseX = packedCoord.x * 4.0;
    if (plane == 0) {
        float y = packedCoord.y;
        gl_FragColor = vec4(
            packedY(baseX, y),
            packedY(baseX + 1.0, y),
            packedY(baseX + 2.0, y),
            packedY(baseX + 3.0, y)
        );
    } else {
        float y = packedCoord.y * 2.0;
        vec2 left = packedUV(baseX, y);
        vec2 right = packedUV(baseX + 2.0, y);
        gl_FragColor = vec4(left.x, left.y, right.x, right.y);
    }
}
"""


@dataclass(slots=True)
class CachedTextTexture:
    texture: object
    text_width: float
    text_height: float
    texture_width: int
    texture_height: int
    padding_px: float


@lru_cache(maxsize=256)
def _cached_rl_color(r: int, g: int, b: int, a: int) -> rl.Color:
    return rl.Color(r, g, b, a)


def rgba_key(color: tuple[int, int, int] | tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    if len(color) == 4:
        r, g, b, a = color
    else:
        r, g, b = color
        a = 255
    return int(r), int(g), int(b), int(a)


def rl_color(color: tuple[int, int, int] | tuple[int, int, int, int], alpha: int | None = None) -> rl.Color:
    r, g, b, a = rgba_key(color)
    if alpha is not None:
        a = alpha
    return _cached_rl_color(int(r), int(g), int(b), int(a))


def radar_point_distance_label(point: RadarPointMarker) -> str:
    return f"{point.longitudinal_m:.0f} m"


def radar_point_speed_label(point: RadarPointMarker) -> str:
    if point.absolute_speed_kph is None:
        return ""
    return f"{point.absolute_speed_kph:.0f} km/h"


def vehicle_distance_label(vehicle: VehicleBox) -> str:
    if vehicle.absolute_speed_kph is not None and vehicle.absolute_speed_kph <= RADAR_STATIC_OBJECT_SPEED_KPH:
        return ""
    return f"{vehicle_distance_m(vehicle):.0f} m"


def vehicle_distance_m(vehicle: VehicleBox) -> float:
    if vehicle.longitudinal_m is not None:
        return vehicle.longitudinal_m
    return vehicle.center.y - EGO_FORWARD_M


def vehicle_speed_label(vehicle: VehicleBox) -> str:
    if vehicle.absolute_speed_kph is None:
        return ""
    return f"{vehicle.absolute_speed_kph:.0f} km/h"


def radar_info_shows_vehicle(mode: int) -> bool:
    return mode in (
        CLUSTER_RADAR_INFO_VEHICLE_SPEED,
        CLUSTER_RADAR_INFO_VEHICLE_SPEED_DISTANCE,
        CLUSTER_RADAR_INFO_ALL_SPEED,
        CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
    )


def radar_info_shows_radar_points(mode: int) -> bool:
    return mode in (
        CLUSTER_RADAR_INFO_ALL_SPEED,
        CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
    )


def radar_info_shows_speed(mode: int) -> bool:
    return mode != CLUSTER_RADAR_INFO_NONE


def radar_info_shows_distance(mode: int) -> bool:
    return mode in (
        CLUSTER_RADAR_INFO_VEHICLE_SPEED_DISTANCE,
        CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
    )


def vehicle_metric_color(vehicle: VehicleBox, theme: ClusterTheme, source_color_mode: int) -> tuple[int, int, int]:
    if source_color_mode != CLUSTER_RADAR_SOURCE_COLOR_BY_SOURCE:
        return theme.world_label_text
    if vehicle_source_is_adas(vehicle.source):
        return GREEN
    if vehicle_source_is_front_radar(vehicle.source):
        return RED
    if vehicle_source_is_radar_track(vehicle.source):
        return AMBER
    if vehicle_source_is_camera(vehicle.source):
        return BLUE_SOFT
    if vehicle.source.startswith("modelV2"):
        return BLUE
    return theme.world_label_text


def vehicle_source_base(source: str) -> str:
    return source.split("+radar:", 1)[0]


def vehicle_source_is_adas(source: str) -> bool:
    base_source = vehicle_source_base(source)
    return base_source == "carState" or base_source in ("CAN 0x162", "CAN 0x1ea")


def vehicle_source_is_camera(source: str) -> bool:
    return vehicle_source_base(source).startswith("camera")


def vehicle_source_is_front_radar(source: str) -> bool:
    return vehicle_source_base(source) == "radarState"


def vehicle_source_is_radar_track(source: str) -> bool:
    return source in ("radarPoint", "liveTracks") or "+radar:" in source


def speed_limit_source_label(source: str | None) -> str:
    if source is None:
        return ""
    normalized = source.strip().lower()
    if not normalized:
        return ""
    return SPEED_LIMIT_SOURCE_LABELS.get(normalized, normalized[:3])


def world_label_scale(distance_m: float) -> float:
    far_amount = clamp((abs(distance_m) - WORLD_LABEL_NEAR_M) / (WORLD_LABEL_FAR_M - WORLD_LABEL_NEAR_M), 0.0, 1.0)
    return 1.0 - far_amount * (1.0 - WORLD_LABEL_MIN_SCALE)


def vec3(point: Vec3) -> rl.Vector3:
    return rl.Vector3(point.x, point.y, point.z)


def rectangles_overlap(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> bool:
    lx, ly, lw, lh = left
    rx, ry, rw, rh = right
    return lx < rx + rw and lx + lw > rx and ly < ry + rh and ly + lh > ry


def camera_forward(camera) -> tuple[float, float, float] | None:
    dx = float(camera.target.x - camera.position.x)
    dy = float(camera.target.y - camera.position.y)
    dz = float(camera.target.z - camera.position.z)
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0001 or not all(math.isfinite(value) for value in (dx, dy, dz, length)):
        return None
    return dx / length, dy / length, dz / length


def camera_depth_m(point, camera) -> float | None:
    forward = camera_forward(camera)
    if forward is None:
        return None
    px = float(point.x - camera.position.x)
    py = float(point.y - camera.position.y)
    pz = float(point.z - camera.position.z)
    if not all(math.isfinite(value) for value in (px, py, pz)):
        return None
    fx, fy, fz = forward
    return px * fx + py * fy + pz * fz


def world_to_screen_label_anchor(point, camera, width: int, height: int):
    depth_m = camera_depth_m(point, camera)
    if depth_m is None or depth_m <= 0.05:
        return None
    screen = rl.get_world_to_screen_ex(point, camera, width, height)
    if not math.isfinite(screen.x) or not math.isfinite(screen.y):
        return None
    return screen


def label_rect_inside_bounds(
    rect: tuple[float, float, float, float],
    bounds: tuple[float, float, float, float],
) -> bool:
    x, y, width, height = rect
    left, top, right, bottom = bounds
    values = (x, y, width, height, left, top, right, bottom)
    if not all(math.isfinite(value) for value in values):
        return False
    return x >= left and y >= top and x + width <= right and y + height <= bottom


class ClusterUiRenderer:
    def __init__(
        self,
        width: int = DESIGN_WIDTH,
        height: int = DESIGN_HEIGHT,
        title: str = "carrotpilot cluster",
        target_fps: int = 0,
        theme_mode: str = "auto",
        screen_mode: int = 0,
    ) -> None:
        self.width = width
        self.height = height
        self.title = title
        self.target_fps = target_fps
        self.theme_mode = normalize_cluster_theme_mode(theme_mode)
        self.screen_mode = normalize_cluster_screen_mode(screen_mode)
        self._theme = current_cluster_theme(self.theme_mode)
        self.hidden = False
        self._window_open = False
        self._font = None
        self._owns_font = False
        self._accel_text_width = 0.0
        self._capture_target = None
        self._portrait_upload_target = None
        self._portrait_upload_target_size: tuple[int, int] | None = None
        self._nv12_pack_y_target = None
        self._nv12_pack_y_size: tuple[int, int] | None = None
        self._nv12_pack_uv_target = None
        self._nv12_pack_uv_size: tuple[int, int] | None = None
        self._nv12_pack_full_target = None
        self._nv12_pack_full_size: tuple[int, int] | None = None
        self._nv12_pack_shader = None
        self._nv12_pack_shader_locations: dict[str, int] = {}
        self._vehicle_model = None
        self._vehicle_model_load_attempted = False
        self._follow_vehicle_texture = None
        self._lfa_texture = None
        self._lfa_active_texture = None
        self._navi_guidance_texture = None
        self._navi_guidance_hash = ""
        self._navi_guidance_size: tuple[int, int] | None = None
        self._route_video_texture = None
        self._route_video_size: tuple[int, int] | None = None
        self._route_video_frame_id: str | None = None
        self._left_turn_signal_started_at: float | None = None
        self._right_turn_signal_started_at: float | None = None
        self._triangle_strip_point_cache: OrderedDict[
            tuple[int, int],
            tuple[tuple[Vec3, ...], tuple[Vec3, ...], object, int],
        ] = OrderedDict()
        self._world_label_texture_cache: OrderedDict[
            tuple[int, str, float, float, tuple[int, int, int, int]],
            CachedTextTexture,
        ] = OrderedDict()
        self._world_label_texture_cache_enabled = os.environ.get("CLUSTER_WORLD_LABEL_TEXTURE_CACHE", "0") == "1"
        self._text_measure_cache: dict[tuple[int, str, float, float], tuple[float, float]] = {}
        self._system_stats = SystemStatsSampler(SYSTEM_STATS_REFRESH_SECONDS)
        self._debug_plot_mode_prev = -1
        self._debug_plot_size = 0
        self._debug_plot_index = -1
        self._debug_plot_values = [[0.0] * DEBUG_PLOT_MAX_SAMPLES for _ in range(3)]
        self._debug_plot_min = -2.0
        self._debug_plot_max = 2.0
        self._debug_plot_last_sample_time: float | None = None
        self.profile_enabled = os.environ.get("CLUSTER_PROFILE_RENDER") == "1"
        self._profile_samples: list[tuple[str, float]] = []

    def set_profile_enabled(self, enabled: bool) -> None:
        self.profile_enabled = enabled

    def set_theme_mode(self, theme_mode: str) -> None:
        self.theme_mode = normalize_cluster_theme_mode(theme_mode)
        self._theme = current_cluster_theme(self.theme_mode)

    def set_screen_mode(self, screen_mode: int) -> None:
        self.screen_mode = normalize_cluster_screen_mode(screen_mode)

    def set_target_fps(self, target_fps: int) -> None:
        self.target_fps = max(0, int(target_fps))
        if self._window_open:
            profile_stage = self._profile_start()
            rl.set_target_fps(self.target_fps)
            self._profile_add("renderer.set_target_fps", profile_stage)

    def _current_theme(self) -> ClusterTheme:
        self._theme = current_cluster_theme(self.theme_mode)
        return self._theme

    def clear_profile_samples(self) -> None:
        self._profile_samples.clear()

    def profile_samples(self) -> list[tuple[str, float]]:
        return self._profile_samples

    def _profile_start(self) -> float:
        return time.perf_counter() if self.profile_enabled else 0.0

    def _profile_add(self, name: str, start_time: float) -> None:
        if self.profile_enabled:
            self._profile_samples.append((name, (time.perf_counter() - start_time) * 1000.0))

    def _profile_add_elapsed(self, name: str, elapsed_ms: float) -> None:
        if self.profile_enabled:
            self._profile_samples.append((name, elapsed_ms))

    def open(self, hidden: bool = False) -> None:
        if self._window_open:
            return
        profile_total = self._profile_start()
        self.hidden = hidden
        rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
        flags = 0
        if hidden:
            flags |= rl.ConfigFlags.FLAG_WINDOW_HIDDEN
        if flags:
            rl.set_config_flags(flags)
        profile_stage = self._profile_start()
        rl.init_window(self.width, self.height, self.title)
        self._profile_add("renderer.open.init_window", profile_stage)
        if self.target_fps > 0:
            profile_stage = self._profile_start()
            rl.set_target_fps(self.target_fps)
            self._profile_add("renderer.open.set_target_fps", profile_stage)
        profile_stage = self._profile_start()
        self._font = self._load_font()
        self._profile_add("renderer.open.load_font", profile_stage)
        profile_stage = self._profile_start()
        self._load_vehicle_model()
        self._profile_add("renderer.open.load_vehicle_model", profile_stage)
        profile_stage = self._profile_start()
        self._load_follow_vehicle_texture()
        self._profile_add("renderer.open.load_follow_vehicle_texture", profile_stage)
        profile_stage = self._profile_start()
        self._load_drive_status_textures()
        self._profile_add("renderer.open.load_drive_status_textures", profile_stage)
        self._window_open = True
        self._profile_add("renderer.open.total", profile_total)

    def close(self) -> None:
        if not self._window_open:
            return
        if self._capture_target is not None:
            rl.unload_render_texture(self._capture_target)
            self._capture_target = None
        if self._portrait_upload_target is not None:
            rl.unload_render_texture(self._portrait_upload_target)
            self._portrait_upload_target = None
            self._portrait_upload_target_size = None
        if self._nv12_pack_y_target is not None:
            rl.unload_render_texture(self._nv12_pack_y_target)
            self._nv12_pack_y_target = None
            self._nv12_pack_y_size = None
        if self._nv12_pack_uv_target is not None:
            rl.unload_render_texture(self._nv12_pack_uv_target)
            self._nv12_pack_uv_target = None
            self._nv12_pack_uv_size = None
        if self._nv12_pack_full_target is not None:
            rl.unload_render_texture(self._nv12_pack_full_target)
            self._nv12_pack_full_target = None
            self._nv12_pack_full_size = None
        if self._nv12_pack_shader is not None:
            rl.unload_shader(self._nv12_pack_shader)
            self._nv12_pack_shader = None
            self._nv12_pack_shader_locations = {}
        for cached_text in self._world_label_texture_cache.values():
            rl.unload_texture(cached_text.texture)
        self._world_label_texture_cache.clear()
        if self._route_video_texture is not None:
            rl.unload_texture(self._route_video_texture)
            self._route_video_texture = None
        if self._follow_vehicle_texture is not None:
            rl.unload_texture(self._follow_vehicle_texture)
            self._follow_vehicle_texture = None
        if self._lfa_texture is not None:
            rl.unload_texture(self._lfa_texture)
            self._lfa_texture = None
        if self._lfa_active_texture is not None:
            rl.unload_texture(self._lfa_active_texture)
            self._lfa_active_texture = None
        if self._navi_guidance_texture is not None:
            rl.unload_texture(self._navi_guidance_texture)
            self._navi_guidance_texture = None
            self._navi_guidance_hash = ""
            self._navi_guidance_size = None
        if self._owns_font and self._font is not None:
            rl.unload_font(self._font)
        self._font = None
        self._owns_font = False
        self._accel_text_width = 0.0
        if self._vehicle_model is not None:
            rl.unload_model(self._vehicle_model)
            self._vehicle_model = None
        self._vehicle_model_load_attempted = False
        self._route_video_size = None
        self._route_video_frame_id = None
        rl.close_window()
        self._window_open = False

    def should_close(self) -> bool:
        return bool(self._window_open and rl.window_should_close())

    def render_frame(self, state: ClusterUiState) -> None:
        self.open()
        profile_stage = self._profile_start()
        rl.begin_drawing()
        self._profile_add("render_frame.begin_drawing", profile_stage)
        profile_stage = self._profile_start()
        self.render(state)
        self._profile_add("render_frame.render", profile_stage)
        profile_stage = self._profile_start()
        rl.end_drawing()
        self._profile_add("render_frame.end_drawing", profile_stage)

    def render(self, state: ClusterUiState, signal_lights: tuple[bool, bool] | None = None) -> None:
        """Draw one frame into the currently active raylib render target."""
        if signal_lights is None:
            signal_lights = self._turn_signal_lights(state)
        profile_stage = self._profile_start()
        if self.screen_mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH:
            self._clear_world()
        else:
            self._render_world(state, signal_lights)
        self._profile_add("render.world", profile_stage)
        profile_stage = self._profile_start()
        self._draw_hud(state, signal_lights)
        self._profile_add("render.hud", profile_stage)

    def _clear_world(self) -> None:
        theme = self._current_theme()
        profile_stage = self._profile_start()
        rl.clear_background(rl_color(theme.bg))
        self._profile_add("render_world.clear_background", profile_stage)

    def _render_world(self, state: ClusterUiState, signal_lights: tuple[bool, bool] | None = None) -> None:
        if signal_lights is None:
            signal_lights = self._turn_signal_lights(state)
        theme = self._current_theme()
        profile_stage = self._profile_start()
        scene = build_cluster_scene(
            state,
            self._profile_add_elapsed if self.profile_enabled else None,
            highlight_lane_lit=self._highlight_lane_lit(state, signal_lights),
            theme=theme,
        )
        self._profile_add("render_world.build_scene", profile_stage)
        profile_stage = self._profile_start()
        rl.clear_background(rl_color(theme.bg))
        self._profile_add("render_world.clear_background", profile_stage)
        profile_stage = self._profile_start()
        self._draw_scene(scene, state)
        self._profile_add("render_world.draw_scene", profile_stage)

    def render_to_file(self, state: ClusterUiState, output_path: str | Path) -> None:
        image = self._render_to_image(state)
        try:
            rl.export_image(image, str(output_path))
        finally:
            rl.unload_image(image)

    def render_to_png_bytes(self, state: ClusterUiState, portrait_upload: bool = False) -> bytes:
        profile_stage = self._profile_start()
        image = self._render_to_image(state, portrait_upload=portrait_upload)
        self._profile_add("render_to_png.render_to_image", profile_stage)
        try:
            size = rl.ffi.new("int *")
            profile_stage = self._profile_start()
            data = rl.export_image_to_memory(image, ".png", size)
            self._profile_add("render_to_png.export_png", profile_stage)
            try:
                if size[0] <= 0:
                    raise RuntimeError("raylib failed to encode frame as PNG")
                return bytes(rl.ffi.buffer(data, size[0]))
            finally:
                rl.mem_free(data)
        finally:
            profile_stage = self._profile_start()
            rl.unload_image(image)
            self._profile_add("render_to_png.unload_image", profile_stage)

    def render_to_rgba_bytes(
        self,
        state: ClusterUiState,
        portrait_upload: bool = False,
        output_width: int | None = None,
        output_height: int | None = None,
    ) -> tuple[bytes, int, int]:
        with self.render_to_rgba_buffer(
            state,
            portrait_upload=portrait_upload,
            output_width=output_width,
            output_height=output_height,
        ) as (
            rgba_buffer,
            image_width,
            image_height,
        ):
            profile_stage = self._profile_start()
            rgba = bytes(rgba_buffer)
            self._profile_add("render_to_rgba.copy_bytes", profile_stage)
            return rgba, image_width, image_height

    @contextmanager
    def render_to_rgba_buffer(
        self,
        state: ClusterUiState,
        portrait_upload: bool = False,
        output_width: int | None = None,
        output_height: int | None = None,
    ) -> Iterator[tuple[object, int, int]]:
        profile_stage = self._profile_start()
        image = self._render_to_image(
            state,
            portrait_upload=portrait_upload,
            output_width=output_width,
            output_height=output_height,
        )
        self._profile_add("render_to_rgba.render_to_image", profile_stage)

        try:
            if image.format != rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
                profile_stage = self._profile_start()
                rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
                self._profile_add("render_to_rgba.image_format", profile_stage)

            byte_count = image.width * image.height * 4
            profile_stage = self._profile_start()
            rgba_buffer = rl.ffi.buffer(image.data, byte_count)
            self._profile_add("render_to_rgba.buffer_view", profile_stage)
            yield rgba_buffer, image.width, image.height
        finally:
            profile_stage = self._profile_start()
            rl.unload_image(image)
            self._profile_add("render_to_rgba.unload_image", profile_stage)

    @contextmanager
    def render_to_nv12_buffer(
        self,
        state: ClusterUiState,
        output_width: int,
        output_height: int,
        stride: int,
        y_scanlines: int,
        uv_scanlines: int,
        uv_offset: int,
        byte_count: int,
        buffer: bytearray | None = None,
        flip_x: bool = False,
    ) -> Iterator[object]:
        self.open(hidden=self.hidden)
        output_width = int(output_width)
        output_height = int(output_height)
        stride = int(stride)
        y_scanlines = int(y_scanlines)
        uv_scanlines = int(uv_scanlines)
        uv_offset = int(uv_offset)
        byte_count = int(byte_count)
        if output_width <= 0 or output_height <= 0 or stride <= 0 or byte_count <= 0:
            raise RuntimeError("NV12 render target layout is invalid")
        if stride < output_width or y_scanlines < output_height or uv_scanlines < (output_height + 1) // 2:
            raise RuntimeError("NV12 render target layout is smaller than the rendered frame")
        if uv_offset < stride * y_scanlines or byte_count < uv_offset + stride * uv_scanlines:
            raise RuntimeError("NV12 render target byte layout is inconsistent")

        profile_stage = self._profile_start()
        target = self._get_capture_target()
        self._profile_add("render_to_nv12.get_capture_target", profile_stage)

        profile_stage = self._profile_start()
        rl.begin_texture_mode(target)
        self.render(state)
        rl.end_texture_mode()
        self._profile_add("render_to_nv12.draw_to_target", profile_stage)

        profile_stage = self._profile_start()
        upload_target = self._get_portrait_upload_target(output_width, output_height)
        self._profile_add("render_to_nv12.get_portrait_upload_target", profile_stage)

        profile_stage = self._profile_start()
        rl.begin_texture_mode(upload_target)
        rl.clear_background(rl_color(self._current_theme().bg))
        source = rl.Rectangle(
            0.0,
            0.0,
            float(target.texture.width),
            float(target.texture.height),
        )
        dest = rl.Rectangle(
            0.0,
            float(self.width),
            float(self.width),
            float(self.height),
        )
        rl.draw_texture_pro(
            target.texture,
            source,
            dest,
            rl.Vector2(0.0, 0.0),
            -90.0,
            rl_color(WHITE),
        )
        rl.end_texture_mode()
        self._profile_add("render_to_nv12.gpu_upload_transform", profile_stage)

        pack_direct_input = stride % 4 == 0 and byte_count % stride == 0 and uv_offset % stride == 0
        if pack_direct_input:
            full_pack_w = stride // 4
            full_pack_h = byte_count // stride
            tail_pack_h = max(0, full_pack_h - y_scanlines - uv_scanlines)
            uv_pack_y = tail_pack_h
            y_pack_y = tail_pack_h + uv_scanlines

            profile_stage = self._profile_start()
            full_target = self._get_nv12_pack_target("full", full_pack_w, full_pack_h)
            self._profile_add("render_to_nv12.get_pack_targets", profile_stage)

            profile_stage = self._profile_start()
            self._render_nv12_pack_plane(
                upload_target.texture,
                full_target,
                output_width,
                output_height,
                0,
                flip_x,
                packed_width=full_pack_w,
                packed_height=y_scanlines,
                dest_y=y_pack_y,
                clear_target=True,
                clear_color=(128, 128, 128, 128),
            )
            self._profile_add("render_to_nv12.pack_y_shader", profile_stage)

            profile_stage = self._profile_start()
            self._render_nv12_pack_plane(
                upload_target.texture,
                full_target,
                output_width,
                output_height,
                1,
                flip_x,
                packed_width=full_pack_w,
                packed_height=uv_scanlines,
                dest_y=uv_pack_y,
                clear_target=False,
            )
            self._profile_add("render_to_nv12.pack_uv_shader", profile_stage)

            profile_stage = self._profile_start()
            image = rl.load_image_from_texture(full_target.texture)
            self._profile_add("render_to_nv12.readback_packed", profile_stage)

            try:
                if image.format != rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
                    profile_stage = self._profile_start()
                    rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
                    self._profile_add("render_to_nv12.packed_image_format", profile_stage)

                profile_stage = self._profile_start()
                nv12_buffer = rl.ffi.buffer(image.data, byte_count)
                self._profile_add("render_to_nv12.buffer_view", profile_stage)
                yield nv12_buffer
            finally:
                profile_stage = self._profile_start()
                rl.unload_image(image)
                self._profile_add("render_to_nv12.unload_image", profile_stage)
            return

        pack_full_stride = stride % 4 == 0
        if pack_full_stride:
            y_pack_w = stride // 4
            y_pack_h = y_scanlines
            uv_pack_w = stride // 4
            uv_pack_h = uv_scanlines
        else:
            y_pack_w = (output_width + 3) // 4
            y_pack_h = output_height
            uv_pack_w = (output_width + 3) // 4
            uv_pack_h = (output_height + 1) // 2
        profile_stage = self._profile_start()
        y_target = self._get_nv12_pack_target("y", y_pack_w, y_pack_h)
        uv_target = self._get_nv12_pack_target("uv", uv_pack_w, uv_pack_h)
        self._profile_add("render_to_nv12.get_pack_targets", profile_stage)

        profile_stage = self._profile_start()
        self._render_nv12_pack_plane(upload_target.texture, y_target, output_width, output_height, 0, flip_x)
        self._profile_add("render_to_nv12.pack_y_shader", profile_stage)

        profile_stage = self._profile_start()
        y_image = rl.load_image_from_texture(y_target.texture)
        self._profile_add("render_to_nv12.readback_y", profile_stage)

        profile_stage = self._profile_start()
        self._render_nv12_pack_plane(upload_target.texture, uv_target, output_width, output_height, 1, flip_x)
        self._profile_add("render_to_nv12.pack_uv_shader", profile_stage)

        profile_stage = self._profile_start()
        uv_image = rl.load_image_from_texture(uv_target.texture)
        self._profile_add("render_to_nv12.readback_uv", profile_stage)

        try:
            if y_image.format != rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
                profile_stage = self._profile_start()
                rl.image_format(y_image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
                self._profile_add("render_to_nv12.y_image_format", profile_stage)
            if uv_image.format != rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
                profile_stage = self._profile_start()
                rl.image_format(uv_image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)
                self._profile_add("render_to_nv12.uv_image_format", profile_stage)

            if buffer is None or len(buffer) != byte_count:
                buffer = bytearray(byte_count)
                buffer[:min(uv_offset, byte_count)] = b"\x10" * min(uv_offset, byte_count)
                if uv_offset < byte_count:
                    buffer[uv_offset:] = b"\x80" * (byte_count - uv_offset)

            y_row_bytes = y_pack_w * 4
            uv_row_bytes = uv_pack_w * 4
            y_data = rl.ffi.buffer(y_image.data, y_row_bytes * y_pack_h)
            uv_data = rl.ffi.buffer(uv_image.data, uv_row_bytes * uv_pack_h)

            if pack_full_stride:
                y_plane_bytes = stride * y_scanlines
                uv_plane_bytes = stride * uv_scanlines
                profile_stage = self._profile_start()
                buffer[:y_plane_bytes] = y_data[:y_plane_bytes]
                self._profile_add("render_to_nv12.copy_y", profile_stage)

                profile_stage = self._profile_start()
                buffer[uv_offset:uv_offset + uv_plane_bytes] = uv_data[:uv_plane_bytes]
                self._profile_add("render_to_nv12.copy_uv", profile_stage)
            else:
                profile_stage = self._profile_start()
                for row in range(output_height):
                    src_start = row * y_row_bytes
                    dst_start = row * stride
                    buffer[dst_start:dst_start + output_width] = y_data[src_start:src_start + output_width]
                self._profile_add("render_to_nv12.copy_y", profile_stage)

                profile_stage = self._profile_start()
                for row in range(uv_pack_h):
                    src_start = row * uv_row_bytes
                    dst_start = uv_offset + row * stride
                    buffer[dst_start:dst_start + output_width] = uv_data[src_start:src_start + output_width]
                self._profile_add("render_to_nv12.copy_uv", profile_stage)
            yield buffer
        finally:
            profile_stage = self._profile_start()
            rl.unload_image(y_image)
            rl.unload_image(uv_image)
            self._profile_add("render_to_nv12.unload_images", profile_stage)

    def _render_to_image(
        self,
        state: ClusterUiState,
        portrait_upload: bool = False,
        output_width: int | None = None,
        output_height: int | None = None,
    ):
        self.open(hidden=self.hidden)
        profile_stage = self._profile_start()
        target = self._get_capture_target()
        self._profile_add("render_to_image.get_capture_target", profile_stage)

        profile_stage = self._profile_start()
        rl.begin_texture_mode(target)
        self.render(state)
        rl.end_texture_mode()
        self._profile_add("render_to_image.draw_to_target", profile_stage)

        if portrait_upload:
            profile_stage = self._profile_start()
            upload_target = self._get_portrait_upload_target(output_width, output_height)
            self._profile_add("render_to_image.get_portrait_upload_target", profile_stage)

            profile_stage = self._profile_start()
            rl.begin_texture_mode(upload_target)
            rl.clear_background(rl_color(self._current_theme().bg))
            source = rl.Rectangle(
                0.0,
                0.0,
                float(target.texture.width),
                float(target.texture.height),
            )
            dest = rl.Rectangle(
                0.0,
                float(self.width),
                float(self.width),
                float(self.height),
            )
            origin = rl.Vector2(0.0, 0.0)
            rl.draw_texture_pro(
                target.texture,
                source,
                dest,
                origin,
                -90.0,
                rl_color(WHITE),
            )
            rl.end_texture_mode()
            self._profile_add("render_to_image.gpu_upload_transform", profile_stage)

            profile_stage = self._profile_start()
            image = rl.load_image_from_texture(upload_target.texture)
            self._profile_add("render_to_image.readback_upload_texture", profile_stage)
        else:
            profile_stage = self._profile_start()
            image = rl.load_image_from_texture(target.texture)
            self._profile_add("render_to_image.readback_texture", profile_stage)

            profile_stage = self._profile_start()
            rl.image_flip_vertical(image)
            self._profile_add("render_to_image.flip_vertical", profile_stage)

        return image

    def _get_capture_target(self):
        if self._capture_target is None:
            profile_stage = self._profile_start()
            self._capture_target = rl.load_render_texture(self.width, self.height)
            self._profile_add("render_target.alloc_capture", profile_stage)
            profile_stage = self._profile_start()
            rl.set_texture_filter(self._capture_target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            self._profile_add("render_target.filter_capture", profile_stage)
        return self._capture_target

    def _get_portrait_upload_target(self, width: int | None = None, height: int | None = None):
        target_width = int(width or self.height)
        target_height = int(height or self.width)
        target_size = (target_width, target_height)
        if self._portrait_upload_target is not None and self._portrait_upload_target_size != target_size:
            rl.unload_render_texture(self._portrait_upload_target)
            self._portrait_upload_target = None
            self._portrait_upload_target_size = None
        if self._portrait_upload_target is None:
            profile_stage = self._profile_start()
            self._portrait_upload_target = rl.load_render_texture(target_width, target_height)
            self._portrait_upload_target_size = target_size
            self._profile_add("render_target.alloc_portrait_upload", profile_stage)
            profile_stage = self._profile_start()
            rl.set_texture_filter(self._portrait_upload_target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            self._profile_add("render_target.filter_portrait_upload", profile_stage)
        return self._portrait_upload_target

    def _get_nv12_pack_target(self, plane: str, width: int, height: int):
        target_size = (int(width), int(height))
        if plane == "y":
            current = self._nv12_pack_y_target
            current_size = self._nv12_pack_y_size
        elif plane == "uv":
            current = self._nv12_pack_uv_target
            current_size = self._nv12_pack_uv_size
        elif plane == "full":
            current = self._nv12_pack_full_target
            current_size = self._nv12_pack_full_size
        else:
            raise RuntimeError(f"unknown NV12 pack plane: {plane}")

        if current is not None and current_size != target_size:
            rl.unload_render_texture(current)
            current = None
            current_size = None
        if current is None:
            profile_stage = self._profile_start()
            current = rl.load_render_texture(target_size[0], target_size[1])
            self._profile_add(f"render_target.alloc_nv12_{plane}", profile_stage)
            profile_stage = self._profile_start()
            rl.set_texture_filter(current.texture, rl.TextureFilter.TEXTURE_FILTER_POINT)
            self._profile_add(f"render_target.filter_nv12_{plane}", profile_stage)
            current_size = target_size

        if plane == "y":
            self._nv12_pack_y_target = current
            self._nv12_pack_y_size = current_size
        elif plane == "uv":
            self._nv12_pack_uv_target = current
            self._nv12_pack_uv_size = current_size
        else:
            self._nv12_pack_full_target = current
            self._nv12_pack_full_size = current_size
        return current

    def _get_nv12_pack_shader(self):
        if self._nv12_pack_shader is None:
            profile_stage = self._profile_start()
            self._nv12_pack_shader = rl.load_shader_from_memory(NV12_PACK_VERTEX_SHADER, NV12_PACK_FRAGMENT_SHADER)
            self._profile_add("render_to_nv12.load_pack_shader", profile_stage)
            if not rl.is_shader_valid(self._nv12_pack_shader):
                raise RuntimeError("failed to load NV12 pack shader")
            self._nv12_pack_shader_locations = {
                "srcSize": rl.get_shader_location(self._nv12_pack_shader, "srcSize"),
                "packedSize": rl.get_shader_location(self._nv12_pack_shader, "packedSize"),
                "plane": rl.get_shader_location(self._nv12_pack_shader, "plane"),
                "flipX": rl.get_shader_location(self._nv12_pack_shader, "flipX"),
            }
        return self._nv12_pack_shader

    def _render_nv12_pack_plane(
        self,
        source_texture,
        target,
        source_width: int,
        source_height: int,
        plane: int,
        flip_x: bool,
        packed_width: int | None = None,
        packed_height: int | None = None,
        dest_y: int = 0,
        clear_target: bool = True,
        clear_color: tuple[int, int, int, int] = (0, 0, 0, 0),
    ) -> None:
        shader = self._get_nv12_pack_shader()
        locations = self._nv12_pack_shader_locations
        pack_width = int(packed_width) if packed_width is not None else int(target.texture.width)
        pack_height = int(packed_height) if packed_height is not None else int(target.texture.height)
        src_size = rl.ffi.new("float[]", [float(source_width), float(source_height)])
        packed_size = rl.ffi.new("float[]", [float(pack_width), float(pack_height)])
        plane_value = rl.ffi.new("int[]", [int(plane)])
        flip_x_value = rl.ffi.new("int[]", [1 if flip_x else 0])
        rl.set_shader_value(shader, locations["srcSize"], src_size, rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2)
        rl.set_shader_value(shader, locations["packedSize"], packed_size, rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2)
        rl.set_shader_value(shader, locations["plane"], plane_value, rl.ShaderUniformDataType.SHADER_UNIFORM_INT)
        rl.set_shader_value(shader, locations["flipX"], flip_x_value, rl.ShaderUniformDataType.SHADER_UNIFORM_INT)

        rl.begin_texture_mode(target)
        if clear_target:
            rl.clear_background(rl_color(clear_color))
        rl.begin_shader_mode(shader)
        rl.rl_set_blend_factors(rl.RL_ONE, rl.RL_ZERO, rl.RL_FUNC_ADD)
        rl.begin_blend_mode(rl.BlendMode.BLEND_CUSTOM)
        try:
            rl.draw_texture_pro(
                source_texture,
                rl.Rectangle(0.0, 0.0, float(source_width), float(source_height)),
                rl.Rectangle(0.0, float(dest_y), float(pack_width), float(pack_height)),
                rl.Vector2(0.0, 0.0),
                0.0,
                rl_color(WHITE),
            )
        finally:
            rl.end_blend_mode()
            rl.end_shader_mode()
            rl.end_texture_mode()

    def _load_font(self):
        for candidate in self._font_candidates():
            if candidate.exists():
                try:
                    font = rl.load_font_ex(str(candidate), 160, None, 0)
                    if font.texture.id > 0:
                        rl.gen_texture_mipmaps(font.texture)
                        rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_TRILINEAR)
                        self._owns_font = True
                        return font
                except Exception as exc:
                    print(f"Cluster font load failed for {candidate}: {exc}")
        self._owns_font = False
        return rl.get_font_default()

    def _font_candidates(self) -> list[Path]:
        return [
            KAIGEN_GOTHIC_KR_BOLD_FONT_PATH,
            OPENPILOT_ADDON_FONT_DIR / "KaiGenGothicKR-Bold.ttf",
            JETBRAINS_MONO_FONT_PATH,
            OPENPILOT_FONT_DIR / "JetBrainsMono-Bold.ttf",
            Path("/data/openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf"),
            Path("/data/openpilot/selfdrive/assets/addon/font/KaiGenGothicKR-Bold.ttf"),
            Path("/usr/share/fonts/truetype/jetbrains-mono/JetBrainsMono-Medium.ttf"),
            Path("/usr/share/fonts/TTF/JetBrainsMono-Medium.ttf"),
            Path("/usr/local/share/fonts/JetBrainsMono-Medium.ttf"),
        ]

    def _load_vehicle_model(self) -> None:
        if self._vehicle_model_load_attempted:
            return
        self._vehicle_model_load_attempted = True
        if not VEHICLE_MODEL_PATH.exists():
            return
        try:
            profile_stage = self._profile_start()
            mesh = self._load_obj_mesh(VEHICLE_MODEL_PATH)
            self._profile_add("vehicle_model.parse_obj", profile_stage)
            profile_stage = self._profile_start()
            rl.upload_mesh(rl.ffi.addressof(mesh), False)
            self._profile_add("vehicle_model.upload_mesh", profile_stage)
            profile_stage = self._profile_start()
            model = rl.load_model_from_mesh(mesh)
            self._profile_add("vehicle_model.load_from_mesh", profile_stage)
            if not rl.is_model_valid(model):
                rl.unload_model(model)
                return
            self._vehicle_model = model
        except Exception as exc:
            print(f"Cybertruck vehicle model load failed: {exc}")
            self._vehicle_model = None

    def _load_follow_vehicle_texture(self) -> None:
        if self._follow_vehicle_texture is not None:
            return
        self._follow_vehicle_texture = self._load_icon_texture(FOLLOW_VEHICLE_ICON_PATH, "Follow gap vehicle")

    def _load_drive_status_textures(self) -> None:
        if self._lfa_texture is None:
            self._lfa_texture = self._load_icon_texture(LFA_ICON_PATH, "LFA")
        if self._lfa_active_texture is None:
            self._lfa_active_texture = self._load_lfa_active_texture()

    def _load_icon_texture(self, path: Path, label: str):
        if not path.exists():
            return None
        try:
            texture = rl.load_texture(str(path))
            if texture.id <= 0:
                return None
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            return texture
        except Exception as exc:
            print(f"{label} icon load failed: {exc}")
            return None

    def _load_lfa_active_texture(self):
        if not LFA_ICON_PATH.exists():
            return None
        image = None
        try:
            image = rl.load_image(str(LFA_ICON_PATH))
            if not rl.is_image_valid(image):
                return None
            if image.format != rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
                rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)

            data = rl.ffi.cast("unsigned char *", image.data)
            byte_count = image.width * image.height * 4
            green_r, green_g, green_b = GREEN
            for offset in range(0, byte_count, 4):
                alpha = int(data[offset + 3])
                if alpha == 0:
                    continue
                red = int(data[offset])
                green = int(data[offset + 1])
                blue = int(data[offset + 2])
                if red >= 220 and green >= 220 and blue >= 220:
                    data[offset] = green_r
                    data[offset + 1] = green_g
                    data[offset + 2] = green_b

            texture = rl.load_texture_from_image(image)
            if texture.id <= 0:
                return None
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            return texture
        except Exception as exc:
            print(f"LFA active icon load failed: {exc}")
            return None
        finally:
            if image is not None and rl.is_image_valid(image):
                rl.unload_image(image)

    def _load_obj_mesh(self, path: Path):
        vertices: list[tuple[float, float, float]] = []
        normals: list[tuple[float, float, float]] = []
        mesh_vertices: list[float] = []
        mesh_normals: list[float] = []
        mesh_colors: list[int] = []
        material_color = DEFAULT_VEHICLE_MATERIAL_COLOR

        def resolve_index(index_text: str, count: int) -> int:
            index = int(index_text)
            if index < 0:
                index = count + index + 1
            return index - 1

        def parse_face_token(token: str) -> tuple[int, int | None]:
            parts = token.split("/")
            vertex_index = resolve_index(parts[0], len(vertices))
            normal_index = None
            if len(parts) >= 3 and parts[2]:
                normal_index = resolve_index(parts[2], len(normals))
            return vertex_index, normal_index

        def face_normal(points: tuple[tuple[float, float, float], ...]) -> tuple[float, float, float]:
            ax, ay, az = points[0]
            bx, by, bz = points[1]
            cx, cy, cz = points[2]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            if length <= 0.000001:
                return 0.0, 0.0, 1.0
            return nx / length, ny / length, nz / length

        for raw in path.read_text(encoding="utf-8", errors="ignore").splitlines():
            parts = raw.split()
            if not parts or parts[0].startswith("#"):
                continue
            tag = parts[0]
            if tag == "v" and len(parts) >= 4:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif tag == "vn" and len(parts) >= 4:
                normals.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif tag == "usemtl" and len(parts) >= 2:
                material_color = VEHICLE_MATERIAL_COLORS.get(parts[1], DEFAULT_VEHICLE_MATERIAL_COLOR)
            elif tag == "f" and len(parts) >= 4:
                face = [parse_face_token(token) for token in parts[1:]]
                for index in range(1, len(face) - 1):
                    triangle = (face[0], face[index], face[index + 1])
                    points = tuple(vertices[vertex_index] for vertex_index, _ in triangle)
                    fallback_normal = face_normal(points)
                    for vertex_index, normal_index in triangle:
                        vertex = vertices[vertex_index]
                        normal = normals[normal_index] if normal_index is not None else fallback_normal
                        mesh_vertices.extend(vertex)
                        mesh_normals.extend(normal)
                        mesh_colors.extend(material_color)

        vertex_count = len(mesh_vertices) // 3
        if vertex_count < 3 or vertex_count % 3 != 0:
            raise RuntimeError(f"invalid vehicle mesh vertex count: {vertex_count}")

        mesh = rl.Mesh()
        mesh.vertexCount = vertex_count
        mesh.triangleCount = vertex_count // 3
        mesh.vertices = self._alloc_float_array(mesh_vertices)
        mesh.normals = self._alloc_float_array(mesh_normals)
        mesh.colors = self._alloc_uchar_array(mesh_colors)
        return mesh

    def _alloc_float_array(self, values: list[float]):
        data = rl.ffi.cast("float *", rl.mem_alloc(len(values) * rl.ffi.sizeof("float")))
        for index, value in enumerate(values):
            data[index] = value
        return data

    def _alloc_uchar_array(self, values: list[int]):
        data = rl.ffi.cast("unsigned char *", rl.mem_alloc(len(values) * rl.ffi.sizeof("unsigned char")))
        for index, value in enumerate(values):
            data[index] = int(value)
        return data

    def _draw_scene(self, scene: ClusterScene, state: ClusterUiState) -> None:
        camera = rl.Camera3D(
            vec3(scene.camera.position),
            vec3(scene.camera.target),
            rl.Vector3(0.0, 0.0, 1.0),
            scene.camera.fovy_deg,
            rl.CameraProjection.CAMERA_PERSPECTIVE,
        )
        profile_stage = self._profile_start()
        rl.begin_mode_3d(camera)
        self._profile_add("draw_scene.begin_mode_3d", profile_stage)
        rl.rl_push_matrix()
        if abs(scene.scene_shift_x_m) > 0.0001:
            rl.rl_translatef(scene.scene_shift_x_m, 0.0, 0.0)
        try:
            profile_stage = self._profile_start()
            for strip in scene.highlight_lanes:
                self._draw_strip(strip)
            self._profile_add("draw_scene.highlight_lanes", profile_stage)
            profile_stage = self._profile_start()
            for strip in scene.road_edges:
                self._draw_strip(strip)
            self._profile_add("draw_scene.road_edges", profile_stage)
            profile_stage = self._profile_start()
            for strip in scene.lane_markings:
                self._draw_strip(strip)
            self._profile_add("draw_scene.lane_markings", profile_stage)
            profile_stage = self._profile_start()
            for strip in scene.planned_path:
                self._draw_strip(strip)
            self._profile_add("draw_scene.planned_path", profile_stage)
            profile_stage = self._profile_start()
            for point in scene.radar_points:
                self._draw_radar_point(point)
            self._profile_add("draw_scene.radar_points", profile_stage)
            profile_stage = self._profile_start()
            for vehicle in scene.vehicles:
                self._draw_vehicle(vehicle)
            self._profile_add("draw_scene.vehicles", profile_stage)
        finally:
            rl.rl_pop_matrix()
        profile_stage = self._profile_start()
        rl.end_mode_3d()
        self._profile_add("draw_scene.end_mode_3d", profile_stage)
        profile_stage = self._profile_start()
        self._draw_radar_point_labels(
            scene.radar_points,
            camera,
            scene.scene_shift_x_m,
            state.radar_info_mode,
        )
        self._profile_add("draw_scene.radar_labels", profile_stage)
        profile_stage = self._profile_start()
        self._draw_vehicle_badges(
            scene.vehicles,
            camera,
            scene.scene_shift_x_m,
            state.radar_info_mode,
            state.radar_source_color_mode,
        )
        self._profile_add("draw_scene.vehicle_badges", profile_stage)

    def _draw_strip(self, strip: MeshStrip) -> None:
        count = min(len(strip.left), len(strip.right))
        if count < 2:
            return

        color = rl_color(strip.color)
        x_offset_m = strip.x_offset_m

        if hasattr(rl, "draw_triangle_strip_3d"):
            points, point_count = self._triangle_strip_points_for(strip, count)
            point_ptr = rl.ffi.cast("struct Vector3 *", points)
            if x_offset_m != 0.0:
                rl.rl_push_matrix()
                try:
                    rl.rl_translatef(x_offset_m, 0.0, 0.0)
                    rl.draw_triangle_strip_3d(point_ptr, point_count, color)
                finally:
                    rl.rl_pop_matrix()
            else:
                rl.draw_triangle_strip_3d(point_ptr, point_count, color)
            return

        for index in range(count - 1):
            left = strip.left[index]
            right = strip.right[index]
            next_left = strip.left[index + 1]
            next_right = strip.right[index + 1]
            left_near = rl.Vector3(left.x + x_offset_m, left.y, left.z)
            right_near = rl.Vector3(right.x + x_offset_m, right.y, right.z)
            left_far = rl.Vector3(next_left.x + x_offset_m, next_left.y, next_left.z)
            right_far = rl.Vector3(next_right.x + x_offset_m, next_right.y, next_right.z)
            rl.draw_triangle_3d(left_near, right_near, right_far, color)
            rl.draw_triangle_3d(left_near, right_far, left_far, color)

    def _triangle_strip_points_for(self, strip: MeshStrip, count: int):
        key = (id(strip.left), id(strip.right))
        cached = self._triangle_strip_point_cache.get(key)
        if cached is not None:
            left_ref, right_ref, points, point_count = cached
            if left_ref is strip.left and right_ref is strip.right:
                self._triangle_strip_point_cache.move_to_end(key)
                return points, point_count

        point_count = count * 2
        points = rl.ffi.new("struct Vector3[]", point_count)
        for index in range(count):
            left = strip.left[index]
            right = strip.right[index]

            points[index * 2].x = left.x
            points[index * 2].y = left.y
            points[index * 2].z = left.z

            points[index * 2 + 1].x = right.x
            points[index * 2 + 1].y = right.y
            points[index * 2 + 1].z = right.z

        self._triangle_strip_point_cache[key] = (
            strip.left,
            strip.right,
            points,
            point_count,
        )
        while len(self._triangle_strip_point_cache) > TRIANGLE_STRIP_POINT_CACHE_LIMIT:
            self._triangle_strip_point_cache.popitem(last=False)
        return points, point_count

    def _draw_vehicle(self, vehicle: VehicleBox) -> None:
        source_marker = vehicle.source.startswith("modelV2") or vehicle.source in ("radarState", "radarPoint")
        use_model = (
            self._vehicle_model is not None
            and not source_marker
            and (not vehicle.source or vehicle.primary or vehicle.cut_in)
        )
        if use_model:
            self._draw_vehicle_shadow(vehicle)
            self._draw_vehicle_model(vehicle)
            return
        if vehicle.source and (source_marker or (not vehicle.primary and not vehicle.cut_in)):
            self._draw_vehicle_marker(vehicle)
            return
        self._draw_vehicle_box(vehicle)

    def _draw_vehicle_marker(self, vehicle: VehicleBox) -> None:
        alpha = int(80 + 150 * clamp(vehicle.confidence, 0.0, 1.0))
        marker_center = rl.Vector3(vehicle.center.x, vehicle.center.y, vehicle.height_m * 0.32)
        marker_size = rl.Vector3(
            max(0.55, vehicle.width_m * 0.68),
            max(1.05, vehicle.length_m * 0.64),
            max(0.42, vehicle.height_m * 0.45),
        )
        rl.draw_cube_v(marker_center, marker_size, rl_color(vehicle.body_color, alpha))

    def _draw_radar_point(self, point: RadarPointMarker) -> None:
        side_m = max(0.16, point.radius_m * 1.75)
        height_m = max(0.12, point.radius_m * 1.15)
        marker_center = rl.Vector3(point.center.x, point.center.y, point.center.z)
        marker_size = rl.Vector3(side_m, side_m, height_m)
        rl.draw_cube_v(marker_center, marker_size, rl_color(point.color))

    def _draw_radar_point_labels(
        self,
        points: tuple[RadarPointMarker, ...],
        camera,
        scene_shift_x_m: float = 0.0,
        radar_info_mode: int = CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
    ) -> None:
        if not radar_info_shows_radar_points(radar_info_mode):
            return
        theme = self._current_theme()
        profile_enabled = self.profile_enabled
        profile_stage = self._profile_start()
        ordered = sorted(
            points,
            key=lambda point: (point.longitudinal_m, abs(point.lateral_m), point.label),
            reverse=True,
        )
        self._profile_add("draw_scene.radar_labels.sort", profile_stage)

        project_ms = 0.0
        layout_ms = 0.0
        text_ms = 0.0

        def draw_label_text(label, x, y, size, color) -> None:
            nonlocal text_ms
            if profile_enabled:
                text_stage = time.perf_counter()
                self._draw_world_label_text(label, x, y, size, color, anchor="center")
                text_ms += (time.perf_counter() - text_stage) * 1000.0
                return
            self._draw_world_label_text(label, x, y, size, color, anchor="center")

        for point in ordered:
            anchor = rl.Vector3(
                point.center.x + scene_shift_x_m,
                point.center.y,
                point.center.z + RADAR_LABEL_ANCHOR_Z_OFFSET_M,
            )
            if profile_enabled:
                project_stage = time.perf_counter()
            screen = world_to_screen_label_anchor(anchor, camera, self.width, self.height)
            if profile_enabled:
                project_ms += (time.perf_counter() - project_stage) * 1000.0
            if screen is None:
                continue
            if profile_enabled:
                layout_stage = time.perf_counter()
            distance = radar_point_distance_label(point) if radar_info_shows_distance(radar_info_mode) else ""
            speed = radar_point_speed_label(point) if radar_info_shows_speed(radar_info_mode) else ""
            if not distance and not speed:
                if profile_enabled:
                    layout_ms += (time.perf_counter() - layout_stage) * 1000.0
                continue
            scale = world_label_scale(point.longitudinal_m)
            distance_size = max(9.0, RADAR_LABEL_DISTANCE_FONT_SIZE * scale)
            speed_size = max(8.0, RADAR_LABEL_SPEED_FONT_SIZE * scale)
            shadow_offset = max(1.0, 1.2 * scale)
            gap = max(2.0, 4.0 * scale)
            if speed and distance:
                speed_y = screen.y - speed_size * 0.5
                distance_y = speed_y - (speed_size + distance_size) * 0.5 - gap
            elif speed:
                speed_y = screen.y - speed_size * 0.5
                distance_y = 0.0
            else:
                distance_y = screen.y - distance_size * 0.5
            center_x = screen.x
            shadow = theme.world_label_shadow
            text = theme.world_label_text
            if profile_enabled:
                layout_ms += (time.perf_counter() - layout_stage) * 1000.0
            if distance:
                draw_label_text(
                    distance,
                    center_x + shadow_offset,
                    distance_y + shadow_offset,
                    distance_size,
                    shadow,
                )
                draw_label_text(
                    distance,
                    center_x,
                    distance_y,
                    distance_size,
                    text,
                )
            if speed:
                draw_label_text(
                    speed,
                    center_x + shadow_offset,
                    speed_y + shadow_offset,
                    speed_size,
                    shadow,
                )
                draw_label_text(
                    speed,
                    center_x,
                    speed_y,
                    speed_size,
                    text,
                )
        self._profile_add_elapsed("draw_scene.radar_labels.project", project_ms)
        self._profile_add_elapsed("draw_scene.radar_labels.layout", layout_ms)
        self._profile_add_elapsed("draw_scene.radar_labels.text", text_ms)

    def _draw_vehicle_shadow(self, vehicle: VehicleBox) -> None:
        half_width = vehicle.width_m * 0.5
        half_length = vehicle.length_m * 0.5

        def corner(local_x: float, local_y: float, z: float) -> Vec3:
            return Vec3(
                vehicle.center.x + vehicle.right_x * local_x + vehicle.forward_x * local_y,
                vehicle.center.y + vehicle.right_y * local_x + vehicle.forward_y * local_y,
                z,
            )

        shadow = (
            corner(-half_width * 1.12, -half_length * 1.08, 0.018),
            corner(half_width * 1.12, -half_length * 1.08, 0.018),
            corner(half_width * 1.12, half_length * 1.08, 0.018),
            corner(-half_width * 1.12, half_length * 1.08, 0.018),
        )
        self._draw_quad(
            shadow[0],
            shadow[1],
            shadow[2],
            shadow[3],
            (0, 0, 0, int(18 + 34 * clamp(vehicle.confidence, 0.0, 1.0))),
        )

    def _draw_vehicle_model(self, vehicle: VehicleBox) -> None:
        if self._vehicle_model is None:
            return
        yaw_deg = math.degrees(math.atan2(-vehicle.forward_x, vehicle.forward_y))
        position = rl.Vector3(vehicle.center.x, vehicle.center.y, 0.035)
        rotation_axis = rl.Vector3(0.0, 0.0, 1.0)
        scale = rl.Vector3(vehicle.width_m, vehicle.length_m, vehicle.height_m)
        try:
            rl.rl_disable_backface_culling()
            alpha = int(92 + 163 * clamp(vehicle.confidence, 0.0, 1.0))
            tint = rl_color(vehicle.body_color) if vehicle.source == "radarPoint" else rl_color(WHITE, alpha)
            rl.draw_model_ex(self._vehicle_model, position, rotation_axis, yaw_deg, scale, tint)
        finally:
            rl.rl_enable_backface_culling()

    def _draw_vehicle_badges(
        self,
        vehicles: tuple[VehicleBox, ...],
        camera,
        scene_shift_x_m: float = 0.0,
        radar_info_mode: int = CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
        radar_source_color_mode: int = 0,
    ) -> None:
        if not radar_info_shows_vehicle(radar_info_mode):
            return
        theme = self._current_theme()
        profile_enabled = self.profile_enabled
        profile_stage = self._profile_start()
        ordered = sorted(
            (vehicle for vehicle in vehicles if vehicle.label),
            key=lambda vehicle: (
                0 if vehicle.primary else 1 if vehicle.cut_in else 2,
                max(0.0, vehicle.center.y - EGO_FORWARD_M),
                -vehicle.confidence,
            ),
        )
        self._profile_add("draw_scene.vehicle_badges.sort", profile_stage)

        project_ms = 0.0
        layout_ms = 0.0
        text_ms = 0.0

        def draw_label_text(label, x, y, size, color) -> None:
            nonlocal text_ms
            if profile_enabled:
                text_stage = time.perf_counter()
                self._draw_world_label_text(label, x, y, size, color, anchor="center")
                text_ms += (time.perf_counter() - text_stage) * 1000.0
                return
            self._draw_world_label_text(label, x, y, size, color, anchor="center")

        for vehicle in ordered:
            anchor = rl.Vector3(
                vehicle.center.x + scene_shift_x_m,
                vehicle.center.y,
                vehicle.height_m + VEHICLE_BADGE_ANCHOR_Z_OFFSET_M,
            )
            if profile_enabled:
                project_stage = time.perf_counter()
            screen = world_to_screen_label_anchor(anchor, camera, self.width, self.height)
            if profile_enabled:
                project_ms += (time.perf_counter() - project_stage) * 1000.0
            if screen is None:
                continue

            if profile_enabled:
                layout_stage = time.perf_counter()
            distance = vehicle_distance_label(vehicle) if radar_info_shows_distance(radar_info_mode) else ""
            speed = vehicle_speed_label(vehicle) if radar_info_shows_speed(radar_info_mode) else ""
            if not distance and not speed:
                if profile_enabled:
                    layout_ms += (time.perf_counter() - layout_stage) * 1000.0
                continue
            distance_m = vehicle_distance_m(vehicle)
            scale = world_label_scale(distance_m)
            distance_size = max(9.0, VEHICLE_BADGE_DISTANCE_FONT_SIZE * scale)
            speed_size = max(8.0, VEHICLE_BADGE_SPEED_FONT_SIZE * scale)
            shadow_offset = max(1.0, 1.2 * scale)
            gap = max(2.0, 4.0 * scale)
            if speed and distance:
                speed_y = screen.y - speed_size * 0.5
                distance_y = speed_y - (speed_size + distance_size) * 0.5 - gap
            elif speed:
                speed_y = screen.y - speed_size * 0.5
                distance_y = 0.0
            else:
                distance_y = screen.y - distance_size * 0.5
            center_x = screen.x
            shadow = theme.world_label_shadow
            text_color = vehicle_metric_color(vehicle, theme, radar_source_color_mode)
            if profile_enabled:
                layout_ms += (time.perf_counter() - layout_stage) * 1000.0
            if distance:
                draw_label_text(
                    distance,
                    center_x + shadow_offset,
                    distance_y + shadow_offset,
                    distance_size,
                    shadow,
                )
                draw_label_text(
                    distance,
                    center_x,
                    distance_y,
                    distance_size,
                    text_color,
                )
            if speed:
                draw_label_text(
                    speed,
                    center_x + shadow_offset,
                    speed_y + shadow_offset,
                    speed_size,
                    shadow,
                )
                draw_label_text(
                    speed,
                    center_x,
                    speed_y,
                    speed_size,
                    text_color,
                )
        self._profile_add_elapsed("draw_scene.vehicle_badges.project", project_ms)
        self._profile_add_elapsed("draw_scene.vehicle_badges.layout", layout_ms)
        self._profile_add_elapsed("draw_scene.vehicle_badges.text", text_ms)

    def _world_label_bounds(
        self,
        left: float,
        top: float,
        right: float,
        bottom: float,
    ) -> tuple[float, float, float, float]:
        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        return (
            left * sx,
            top * sy,
            self.width - right * sx,
            self.height - bottom * sy,
        )

    def _draw_vehicle_box(self, vehicle: VehicleBox) -> None:
        half_width = vehicle.width_m * 0.5
        half_length = vehicle.length_m * 0.5
        z0 = 0.035
        z1 = vehicle.height_m + z0

        def corner(local_x: float, local_y: float, z: float) -> Vec3:
            return Vec3(
                vehicle.center.x + vehicle.right_x * local_x + vehicle.forward_x * local_y,
                vehicle.center.y + vehicle.right_y * local_x + vehicle.forward_y * local_y,
                z,
            )

        base = (
            corner(-half_width, -half_length, z0),
            corner(half_width, -half_length, z0),
            corner(half_width, half_length, z0),
            corner(-half_width, half_length, z0),
        )
        top = (
            corner(-half_width, -half_length, z1),
            corner(half_width, -half_length, z1),
            corner(half_width, half_length, z1),
            corner(-half_width, half_length, z1),
        )
        self._draw_vehicle_shadow(vehicle)
        self._draw_quad(base[0], base[1], top[1], top[0], vehicle.rear_color)
        self._draw_quad(base[1], base[2], top[2], top[1], vehicle.side_color)
        self._draw_quad(base[2], base[3], top[3], top[2], vehicle.body_color)
        self._draw_quad(base[3], base[0], top[0], top[3], vehicle.side_color)
        self._draw_quad(top[0], top[1], top[2], top[3], vehicle.body_color)

        inset = 0.22
        highlight = tuple(
            Vec3(
                point.x + (vehicle.center.x - point.x) * inset,
                point.y + (vehicle.center.y - point.y) * inset,
                point.z + 0.006,
            )
            for point in top
        )
        self._draw_quad(highlight[0], highlight[1], highlight[2], highlight[3], vehicle.top_highlight)

        outline = rl_color(vehicle.outline_color)
        edge_points = base + top
        edges = (
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        )
        for start, end in edges:
            rl.draw_line_3d(vec3(edge_points[start]), vec3(edge_points[end]), outline)

    def _draw_quad(
        self,
        p0: Vec3,
        p1: Vec3,
        p2: Vec3,
        p3: Vec3,
        color: tuple[int, int, int, int],
    ) -> None:
        draw_color = rl_color(color)
        rl.draw_triangle_3d(vec3(p0), vec3(p1), vec3(p2), draw_color)
        rl.draw_triangle_3d(vec3(p0), vec3(p2), vec3(p3), draw_color)

    def _draw_hud(self, state: ClusterUiState, signal_lights: tuple[bool, bool] | None = None) -> None:
        if signal_lights is None:
            signal_lights = self._turn_signal_lights(state)
        left_signal_lit, right_signal_lit = signal_lights
        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        profile_stage = self._profile_start()
        rl.rl_push_matrix()
        rl.rl_scalef(sx, sy, 1.0)
        self._profile_add("hud.push_scale", profile_stage)
        try:
            screen_mode = self.screen_mode
            navi_active = state.navi_debug is not None
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH:
                profile_stage = self._profile_start()
                self._draw_speed_block(state)
                self._profile_add("hud.speed_block", profile_stage)
                profile_stage = self._profile_start()
                self._draw_accel_block(state)
                self._profile_add("hud.accel_block", profile_stage)
                profile_stage = self._profile_start()
                self._draw_debug_plot(
                    state.debug_plot,
                    DEBUG_PLOT_FULL_X,
                    DEBUG_PLOT_FULL_Y,
                    DEBUG_PLOT_FULL_W,
                    DEBUG_PLOT_FULL_H,
                )
                self._profile_add("hud.debug_plot_full", profile_stage)
                return

            profile_stage = self._profile_start()
            self._draw_speed_block(state)
            self._profile_add("hud.speed_block", profile_stage)
            profile_stage = self._profile_start()
            self._draw_accel_block(state)
            self._profile_add("hud.accel_block", profile_stage)
            profile_stage = self._profile_start()
            self._draw_turn_signal("left", left_signal_lit, show_inactive=state.debug_ui_visible)
            self._profile_add("hud.turn_signal_left", profile_stage)
            profile_stage = self._profile_start()
            self._draw_drive_status(state)
            self._profile_add("hud.drive_status", profile_stage)
            profile_stage = self._profile_start()
            self._draw_turn_signal("right", right_signal_lit, show_inactive=state.debug_ui_visible)
            self._profile_add("hud.turn_signal_right", profile_stage)
            if navi_active:
                profile_stage = self._profile_start()
                self._draw_navi_traffic_light_panel(state.navi_debug)
                self._profile_add("hud.navi_traffic", profile_stage)
            profile_stage = self._profile_start()
            self._draw_center_clock(state)
            self._profile_add("hud.center_clock", profile_stage)
            profile_stage = self._profile_start()
            self._draw_actual_fps(state.actual_fps)
            self._profile_add("hud.actual_fps", profile_stage)
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG:
                profile_stage = self._profile_start()
                self._draw_live_debug_panel(state)
                self._profile_add("hud.live_debug", profile_stage)
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG_SYSTEM:
                profile_stage = self._profile_start()
                self._draw_system_stats_panel(state)
                self._profile_add("hud.system_stats", profile_stage)
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT:
                profile_stage = self._profile_start()
                self._draw_debug_plot(
                    state.debug_plot,
                    DEBUG_PLOT_RIGHT_X,
                    DEBUG_PLOT_RIGHT_Y,
                    DEBUG_PLOT_RIGHT_W,
                    DEBUG_PLOT_RIGHT_H,
                )
                self._profile_add("hud.debug_plot_right", profile_stage)
            if screen_mode == CLUSTER_SCREEN_MODE_NAVI_DEBUG or navi_active:
                profile_stage = self._profile_start()
                self._draw_navi_debug_panel(state.navi_debug)
                self._profile_add("hud.navi_debug", profile_stage)
            if screen_mode not in (
                CLUSTER_SCREEN_MODE_DEBUG,
                CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
                CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
                CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
                CLUSTER_SCREEN_MODE_NAVI_DEBUG,
            ) and not navi_active:
                profile_stage = self._profile_start()
                self._draw_route_overlay(state.route_overlay)
                self._profile_add("hud.route_overlay", profile_stage)
            profile_stage = self._profile_start()
            self._draw_git_status(state.git_status)
            self._profile_add("hud.git_status", profile_stage)
            profile_stage = self._profile_start()
            self._draw_cluster_core_usage(state.cluster_core_usage_text)
            self._profile_add("hud.cluster_core_usage", profile_stage)
        finally:
            profile_stage = self._profile_start()
            rl.rl_pop_matrix()
            self._profile_add("hud.pop_matrix", profile_stage)

    def _draw_center_clock(self, state: ClusterUiState) -> None:
        if not state.center_clock_text:
            return

        theme = self._current_theme()
        text = state.center_clock_text
        x = SYSTEM_PANEL_X + SYSTEM_PANEL_W * 0.5
        y = 58
        size = 54
        spacing = max(1.0, size * 0.02)
        text_width, text_height = self._measure_text(text, size, spacing)

        pad_x = 28
        pad_y = 14
        rect = rl.Rectangle(
            x - text_width * 0.5 - pad_x,
            y - text_height * 0.5 - pad_y,
            text_width + pad_x * 2,
            text_height + pad_y * 2,
        )

        rl.draw_rectangle_rounded(rect, 0.28, 12, rl_color(theme.clock_bg))
        rl.draw_rectangle_rounded_lines_ex(rect, 0.28, 12, 2.0, rl_color(theme.clock_outline))
        self._draw_text(text, x, y, size, theme.clock_text, anchor="center")

    def _draw_debug_plot(
        self,
        plot: DebugPlotSnapshot | None,
        panel_x: float,
        panel_y: float,
        panel_w: float,
        panel_h: float,
    ) -> None:
        if plot is None or plot.mode <= 0:
            if self._debug_plot_mode_prev != 0:
                self._clear_debug_plot(0)
            self._draw_debug_plot_panel("SHOW PLOT MODE 0", None, panel_x, panel_y, panel_w, panel_h)
            return

        if plot.mode != self._debug_plot_mode_prev:
            self._clear_debug_plot(plot.mode)

        now = time.perf_counter()
        if self._debug_plot_last_sample_time is None or now - self._debug_plot_last_sample_time >= DEBUG_PLOT_SAMPLE_SECONDS:
            self._append_debug_plot_values(plot.values)
            self._debug_plot_last_sample_time = now

        self._draw_debug_plot_panel(plot.title, plot, panel_x, panel_y, panel_w, panel_h)

    def _clear_debug_plot(self, mode: int) -> None:
        self._debug_plot_mode_prev = mode
        self._debug_plot_size = 0
        self._debug_plot_index = -1
        self._debug_plot_values = [[0.0] * DEBUG_PLOT_MAX_SAMPLES for _ in range(3)]
        self._debug_plot_min = -2.0
        self._debug_plot_max = 2.0
        self._debug_plot_last_sample_time = None

    def _append_debug_plot_values(self, values: tuple[float, float, float]) -> None:
        self._debug_plot_index = (self._debug_plot_index + 1) % DEBUG_PLOT_MAX_SAMPLES
        if self._debug_plot_size < DEBUG_PLOT_MAX_SAMPLES:
            self._debug_plot_size += 1

        for index, value in enumerate(values):
            self._debug_plot_values[index][self._debug_plot_index] = value if math.isfinite(value) else 0.0

        self._update_debug_plot_bounds()

    def _update_debug_plot_bounds(self) -> None:
        if self._debug_plot_size <= 0:
            self._debug_plot_min = -2.0
            self._debug_plot_max = 2.0
            return

        minimum = float("inf")
        maximum = float("-inf")
        for series_index in range(3):
            for offset in range(self._debug_plot_size):
                value = self._debug_plot_value(series_index, offset)
                minimum = min(minimum, value)
                maximum = max(maximum, value)

        if minimum == float("inf") or maximum == float("-inf"):
            minimum = -2.0
            maximum = 2.0
        if minimum > -2.0:
            minimum = -2.0
        if maximum < 2.0:
            maximum = 2.0
        if maximum - minimum < 0.001:
            minimum -= 1.0
            maximum += 1.0
        self._debug_plot_min = minimum
        self._debug_plot_max = maximum

    def _debug_plot_value(self, series_index: int, oldest_offset: int) -> float:
        oldest_index = (self._debug_plot_index - self._debug_plot_size + 1) % DEBUG_PLOT_MAX_SAMPLES
        return self._debug_plot_values[series_index][(oldest_index + oldest_offset) % DEBUG_PLOT_MAX_SAMPLES]

    def _draw_debug_plot_panel(
        self,
        title: str,
        plot: DebugPlotSnapshot | None,
        panel_x: float,
        panel_y: float,
        panel_w: float,
        panel_h: float,
    ) -> None:
        theme = self._current_theme()
        compact = panel_w < 700.0
        pad = 18.0 if compact else 24.0
        title_y = panel_y + 30.0
        plot_x = panel_x + pad
        plot_y = panel_y + (74.0 if compact else 70.0)
        plot_w = panel_w - pad * 2.0
        plot_h = panel_h - (100.0 if compact else 96.0)
        plot_bottom = plot_y + plot_h

        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 18, theme.route_panel_bg, theme.faint, 2)
        title_size = 18 if compact else 22
        title_max_w = panel_w - pad * 2.0 - (120.0 if compact else 190.0)
        title = self._ellipsize_text(title, title_size, title_max_w)
        self._draw_text(title, panel_x + pad, title_y, title_size, theme.text)
        self._draw_text(
            f"min {self._debug_plot_min:.2f}  max {self._debug_plot_max:.2f}",
            panel_x + panel_w - pad,
            title_y,
            13 if compact else 17,
            theme.muted,
            anchor="right",
        )

        grid_color = rl_color(theme.faint, 110)
        axis_color = rl_color(theme.muted, 160)
        plot_rect = rl.Rectangle(plot_x, plot_y, plot_w, plot_h)
        rl.draw_rectangle_rec(plot_rect, rl_color((0, 0, 0), 52 if theme.is_dark else 30))
        rl.draw_rectangle_lines_ex(plot_rect, 2.0, rl_color(theme.faint))
        for index in range(1, 6):
            x = plot_x + plot_w * index / 6.0
            rl.draw_line_ex(rl.Vector2(x, plot_y), rl.Vector2(x, plot_bottom), 1.0, grid_color)
        for index in range(1, 4):
            y = plot_y + plot_h * index / 4.0
            rl.draw_line_ex(rl.Vector2(plot_x, y), rl.Vector2(plot_x + plot_w, y), 1.0, grid_color)

        value_range = self._debug_plot_max - self._debug_plot_min
        if self._debug_plot_min < 0.0 < self._debug_plot_max and value_range > 0.001:
            zero_y = plot_bottom - (0.0 - self._debug_plot_min) / value_range * plot_h
            rl.draw_line_ex(rl.Vector2(plot_x, zero_y), rl.Vector2(plot_x + plot_w, zero_y), 2.0, axis_color)

        if plot is None or self._debug_plot_size < 2:
            self._draw_text("no plot data", plot_x + plot_w * 0.5, plot_y + plot_h * 0.5, 22, theme.muted, anchor="center")
            return

        colors = (
            (255, 220, 0),
            GREEN,
            (255, 165, 0),
        )
        for series_index, color in enumerate(colors):
            self._draw_debug_plot_series(series_index, plot_x, plot_y, plot_w, plot_h, color)

    def _draw_debug_plot_series(
        self,
        series_index: int,
        plot_x: float,
        plot_y: float,
        plot_w: float,
        plot_h: float,
        color: tuple[int, int, int],
    ) -> None:
        value_range = max(0.001, self._debug_plot_max - self._debug_plot_min)
        previous: rl.Vector2 | None = None
        latest: rl.Vector2 | None = None
        latest_value = 0.0
        count = self._debug_plot_size
        dx = plot_w / max(1, count - 1)
        for offset in range(count):
            value = self._debug_plot_value(series_index, offset)
            x = plot_x + dx * offset
            y = plot_y + plot_h - (value - self._debug_plot_min) / value_range * plot_h
            point = rl.Vector2(x, y)
            if previous is not None:
                rl.draw_line_ex(previous, point, 3.0, rl_color(color))
            previous = point
            latest = point
            latest_value = value

        if latest is None:
            return
        label = f"{latest_value:.2f}"
        label_size = 18.0
        label_x = min(plot_x + plot_w - 4.0, latest.x + 42.0)
        label_y = clamp(latest.y + (24.0 if series_index > 0 else 0.0), plot_y + 12.0, plot_y + plot_h - 12.0)
        self._draw_text(label, label_x, label_y, label_size, color, anchor="right")

    def _draw_navi_debug_panel(self, info: NaviDebugInfo | None) -> None:
        theme = self._current_theme()
        panel_x = SYSTEM_PANEL_X
        panel_y = SYSTEM_PANEL_Y
        panel_w = SYSTEM_PANEL_W
        panel_h = min(DESIGN_HEIGHT - SYSTEM_PANEL_Y - 18.0, 520.0)
        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 18, theme.route_panel_bg, theme.faint, 2)

        if info is None:
            self._draw_text("NAVI receiver", panel_x + 24, panel_y + 34, 24, theme.text)
            self._draw_text("waiting for data", panel_x + 24, panel_y + 76, 22, theme.muted)
            self._draw_navi_guidance_image_box(None)
            return

        severity = info.severity.lower()
        title_color = {
            "stop": RED,
            "go": GREEN,
            "warning": RED,
            "caution": AMBER,
        }.get(severity, theme.text)
        title = self._ellipsize_text(info.title, 24, panel_w - 48)
        self._draw_text(title, panel_x + 24, panel_y + 34, 24, title_color)

        if severity not in ("normal", ""):
            badge_w = 92.0
            badge_h = 28.0
            badge_x = panel_x + panel_w - badge_w - 24.0
            badge_y = panel_y + 23.0
            self._rounded_rect(badge_x, badge_y, badge_w, badge_h, 10, title_color, None, 0.0)
            self._draw_text(severity.upper(), badge_x + badge_w * 0.5, badge_y + 6.0, 15, WHITE, anchor="center")

        y = panel_y + 82.0
        line_size = 19
        max_w = panel_w - 48.0
        lines = info.lines or ("no navi event",)
        for index, line in enumerate(lines[:4]):
            text = self._ellipsize_text(str(line), line_size, max_w)
            color = theme.text if index < 4 else theme.muted
            self._draw_text(text, panel_x + 24, y, line_size, color)
            y += 31.0
        self._draw_navi_guidance_image_box(info.guidance_image)

    def _draw_navi_traffic_light_panel(self, info: NaviDebugInfo | None) -> None:
        theme = self._current_theme()
        y = NAVI_TRAFFIC_PANEL_Y
        h = NAVI_TRAFFIC_PANEL_H
        traffic = info.traffic_light if info is not None else None
        bg = NAVI_TRAFFIC_BG_DARK if theme.is_dark else NAVI_TRAFFIC_BG_LIGHT
        off = NAVI_TRAFFIC_OFF_DARK if theme.is_dark else NAVI_TRAFFIC_OFF_LIGHT

        red_s = traffic.red_s if traffic is not None else None
        straight_s = traffic.straight_s if traffic is not None else None
        left_s = traffic.left_s if traffic is not None else None
        right_s = traffic.right_s if traffic is not None else None
        uturn_s = traffic.uturn_s if traffic is not None else None
        red_on = self._navi_signal_active(traffic, "red", red_s)
        straight_on = self._navi_signal_active(traffic, "straight", straight_s)
        left_on = self._navi_signal_active(traffic, "left", left_s)
        right_on = self._navi_signal_active(traffic, "right", right_s)
        uturn_on = self._navi_signal_active(traffic, "uturn", uturn_s)
        use_uturn_slot = (uturn_on or uturn_s is not None) and not (left_on or left_s is not None)

        primary_seconds, primary_red = self._navi_primary_signal_seconds(traffic)
        remain_text = "--" if primary_seconds is None else str(primary_seconds)
        remain_color = NAVI_TRAFFIC_RED if primary_red else NAVI_TRAFFIC_GREEN
        remain_size = 54.0
        remain_width, _ = self._measure_text(remain_text, remain_size, max(1.0, remain_size * 0.02))
        show_right = right_on or right_s is not None
        slot_count = 4 if show_right else 3
        slot_span = slot_count * NAVI_TRAFFIC_SIGNAL_SIZE + (slot_count - 1) * NAVI_TRAFFIC_SIGNAL_GAP
        content_w = slot_span + NAVI_TRAFFIC_TEXT_GAP + remain_width
        w = max(270.0, content_w + NAVI_TRAFFIC_PANEL_PAD_X * 2.0)
        x = NAVI_TRAFFIC_PANEL_RIGHT - w
        shadow = (0, 0, 0, 92)
        outline = theme.faint if theme.is_dark else NAVI_TRAFFIC_BG_OUTLINE
        self._rounded_rect(x + 5.0, y + 7.0, w, h, 20.0, shadow, None, 0.0)
        self._rounded_rect(x, y, w, h, 20.0, bg, outline, 3.0)

        slot_y = y + h * 0.5
        slot_x = x + NAVI_TRAFFIC_PANEL_PAD_X + NAVI_TRAFFIC_SIGNAL_SIZE * 0.5
        step = NAVI_TRAFFIC_SIGNAL_SIZE + NAVI_TRAFFIC_SIGNAL_GAP
        self._draw_navi_red_signal(slot_x, slot_y, red_on, off)
        self._draw_navi_turn_signal_slot(
            slot_x + step,
            slot_y,
            active=uturn_on if use_uturn_slot else left_on,
            uturn=use_uturn_slot,
            off=off,
        )
        self._draw_navi_green_signal(slot_x + step * 2.0, slot_y, straight_on, off)
        if show_right:
            self._draw_navi_turn_signal_slot(slot_x + step * 3.0, slot_y, active=right_on, right=True, off=off)

        remain_x = slot_x + NAVI_TRAFFIC_SIGNAL_SIZE * 0.5 + step * (slot_count - 1) + NAVI_TRAFFIC_TEXT_GAP
        self._draw_text_with_stroke(
            remain_text,
            remain_x,
            slot_y,
            remain_size,
            remain_color,
            (0, 0, 0),
            4,
            anchor="left",
        )

    @staticmethod
    def _navi_signal_active(traffic: NaviTrafficLightInfo | None, name: str, seconds: int | None) -> bool:
        if traffic is None:
            return False
        flag = getattr(traffic, f"{name}_on", None)
        if flag is not None:
            return bool(flag)
        return seconds is not None

    def _navi_primary_signal_seconds(self, traffic: NaviTrafficLightInfo | None) -> tuple[int | None, bool]:
        if traffic is None:
            return None, False
        ordered = (
            ("red", traffic.red_s, True),
            ("left", traffic.left_s, False),
            ("uturn", traffic.uturn_s, False),
            ("straight", traffic.straight_s, False),
            ("right", traffic.right_s, False),
        )
        for name, seconds, is_red in ordered:
            if self._navi_signal_active(traffic, name, seconds):
                return seconds, is_red
        for _, seconds, is_red in ordered:
            if seconds is not None:
                return seconds, is_red
        return None, False

    def _draw_navi_red_signal(self, cx: float, cy: float, active: bool, off: tuple[int, int, int]) -> None:
        radius = NAVI_TRAFFIC_SIGNAL_SIZE * (12.5 / 26.0 if active else 0.5)
        color = NAVI_TRAFFIC_RED if active else off
        rl.draw_circle_v(rl.Vector2(cx, cy), radius, rl_color(color))
        if active:
            self._draw_navi_circle_stroke(cx, cy, radius, max(1.6, NAVI_TRAFFIC_SIGNAL_SIZE / 26.0), (0, 0, 0))

    def _draw_navi_green_signal(self, cx: float, cy: float, active: bool, off: tuple[int, int, int]) -> None:
        radius = NAVI_TRAFFIC_SIGNAL_SIZE * (12.5 / 26.0 if active else 0.5)
        color = NAVI_TRAFFIC_GREEN if active else off
        rl.draw_circle_v(rl.Vector2(cx, cy), radius, rl_color(color))
        if active:
            self._draw_navi_circle_stroke(cx, cy, radius, max(1.6, NAVI_TRAFFIC_SIGNAL_SIZE / 26.0), (0, 0, 0))

    def _draw_navi_turn_signal_slot(
        self,
        cx: float,
        cy: float,
        *,
        active: bool,
        uturn: bool = False,
        right: bool = False,
        off: tuple[int, int, int],
    ) -> None:
        radius = NAVI_TRAFFIC_SIGNAL_SIZE * 0.5
        active_bg = (0, 0, 0) if self._current_theme().is_dark else (34, 34, 34)
        rl.draw_circle_v(rl.Vector2(cx, cy), radius, rl_color(active_bg if active else off))
        color = NAVI_TRAFFIC_GREEN if active else NAVI_TRAFFIC_OFF_ARROW
        if uturn:
            self._draw_navi_uturn_arrow(cx, cy, color)
        else:
            self._draw_navi_horizontal_arrow(cx, cy, color, right=right)

    def _draw_navi_circle_stroke(
        self,
        cx: float,
        cy: float,
        radius: float,
        stroke_width: float,
        color: tuple[int, int, int],
    ) -> None:
        rl.draw_ring(
            rl.Vector2(cx, cy),
            max(0.0, radius - stroke_width),
            radius,
            0.0,
            360.0,
            48,
            rl_color(color),
        )

    @staticmethod
    def _navi_slot_point(cx: float, cy: float, x: float, y: float) -> "rl.Vector2":
        scale = NAVI_TRAFFIC_SIGNAL_SIZE / 26.0
        return rl.Vector2(cx + (x - 13.0) * scale, cy + (y - 13.0) * scale)

    @staticmethod
    def _draw_navi_round_line(start: "rl.Vector2", end: "rl.Vector2", width: float, color: tuple[int, int, int]) -> None:
        draw_color = rl_color(color)
        rl.draw_line_ex(start, end, width, draw_color)
        cap_radius = width * 0.5
        rl.draw_circle_v(start, cap_radius, draw_color)
        rl.draw_circle_v(end, cap_radius, draw_color)

    def _draw_navi_horizontal_arrow(self, cx: float, cy: float, color: tuple[int, int, int], *, right: bool) -> None:
        def point(x: float, y: float) -> "rl.Vector2":
            return self._navi_slot_point(cx, cy, 26.0 - x if right else x, y)

        width = max(3.0, 3.0 * NAVI_TRAFFIC_SIGNAL_SIZE / 26.0)
        self._draw_navi_round_line(point(8.5, 13.0), point(20.5, 13.0), width, color)
        tip = point(6.418, 13.0)
        top = point(12.532, 7.0)
        bottom = point(12.532, 19.0)
        self._draw_navi_round_line(top, tip, width, color)
        self._draw_navi_round_line(tip, bottom, width, color)

    def _draw_navi_uturn_arrow(self, cx: float, cy: float, color: tuple[int, int, int]) -> None:
        scale = NAVI_TRAFFIC_SIGNAL_SIZE / 26.0
        width = max(3.0, 3.0 * scale)
        rl.draw_ring(
            rl.Vector2(cx, cy - 3.2 * scale),
            4.7 * scale,
            4.7 * scale + width,
            190.0,
            360.0,
            28,
            rl_color(color),
        )
        self._draw_navi_round_line(
            self._navi_slot_point(cx, cy, 17.8, 10.0),
            self._navi_slot_point(cx, cy, 17.8, 18.6),
            width,
            color,
        )
        tip = self._navi_slot_point(cx, cy, 8.0, 18.8)
        top = self._navi_slot_point(cx, cy, 4.9, 12.7)
        bottom = self._navi_slot_point(cx, cy, 12.6, 15.0)
        rl.draw_triangle(top, tip, bottom, rl_color(color))

    def _draw_navi_guidance_image_box(self, image: NaviGuidanceImage | None) -> None:
        theme = self._current_theme()
        box_x = NAVI_GUIDANCE_IMAGE_X
        box_y = NAVI_GUIDANCE_IMAGE_Y
        box_w = NAVI_GUIDANCE_IMAGE_W
        box_h = NAVI_GUIDANCE_IMAGE_H
        self._rounded_rect(box_x, box_y, box_w, box_h, 14, theme.panel_bg, theme.faint, 2)
        self._draw_text("3D GUIDE", box_x + 16, box_y + 18, 15, theme.muted)
        texture = self._navi_guidance_texture_for(image)
        if texture is None:
            return
        texture_w = float(texture.width)
        texture_h = float(texture.height)
        if texture_w <= 0.0 or texture_h <= 0.0:
            return
        inner_x = box_x + 14.0
        inner_y = box_y + 40.0
        inner_w = box_w - 28.0
        inner_h = box_h - 54.0
        scale = min(inner_w / texture_w, inner_h / texture_h)
        draw_w = texture_w * scale
        draw_h = texture_h * scale
        dest = rl.Rectangle(inner_x + (inner_w - draw_w) * 0.5, inner_y + (inner_h - draw_h) * 0.5, draw_w, draw_h)
        source = rl.Rectangle(0.0, 0.0, texture_w, texture_h)
        rl.draw_texture_pro(texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))

    def _navi_guidance_texture_for(self, image: NaviGuidanceImage | None):
        image_hash = image.image_hash if image is not None else ""
        image_base64 = image.image_base64 if image is not None else ""
        if not image_hash and image_base64:
            image_hash = str(hash(image_base64))
        if not image_base64:
            if self._navi_guidance_texture is not None:
                rl.unload_texture(self._navi_guidance_texture)
                self._navi_guidance_texture = None
                self._navi_guidance_hash = ""
                self._navi_guidance_size = None
            return None
        if self._navi_guidance_texture is not None and image_hash == self._navi_guidance_hash:
            return self._navi_guidance_texture
        if self._navi_guidance_texture is not None:
            rl.unload_texture(self._navi_guidance_texture)
            self._navi_guidance_texture = None
            self._navi_guidance_hash = ""
            self._navi_guidance_size = None
        try:
            payload = image_base64.split(",", 1)[1] if "," in image_base64[:64] else image_base64
            image_bytes = base64.b64decode(payload, validate=False)
        except Exception:
            return None
        extension = ".jpg" if image is not None and "jpeg" in image.image_mime.lower() else ".png"
        loaded_image = None
        try:
            loaded_image = rl.load_image_from_memory(extension, image_bytes, len(image_bytes))
            if not rl.is_image_valid(loaded_image):
                return None
            texture = rl.load_texture_from_image(loaded_image)
            if not rl.is_texture_valid(texture):
                rl.unload_texture(texture)
                return None
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            self._navi_guidance_texture = texture
            self._navi_guidance_hash = image_hash
            self._navi_guidance_size = (int(texture.width), int(texture.height))
            return self._navi_guidance_texture
        except Exception:
            return None
        finally:
            if loaded_image is not None and rl.is_image_valid(loaded_image):
                rl.unload_image(loaded_image)

    def _draw_system_stats_panel(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        stats = self._system_stats.sample()
        cpu_count = len(stats.cpu_core_percents)
        columns = 2 if cpu_count <= 8 else 4
        rows = max(1, math.ceil(max(1, cpu_count) / columns))
        core_row_h = 30.0 if columns == 2 else 24.0
        header_h = 122.0
        panel_h = min(DESIGN_HEIGHT - SYSTEM_PANEL_Y - 18.0, header_h + rows * core_row_h + 18.0)
        core_area_h = max(24.0, panel_h - header_h - 14.0)
        core_row_h = min(core_row_h, core_area_h / rows)

        panel_x = SYSTEM_PANEL_X
        panel_y = SYSTEM_PANEL_Y
        panel_w = SYSTEM_PANEL_W
        pad_x = 24.0
        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 18, theme.route_panel_bg, theme.faint, 2)
        self._draw_text("SYSTEM", panel_x + pad_x, panel_y + 28, 18, theme.muted)

        mem_percent = stats.memory_used_percent
        mem_color = self._system_metric_color(mem_percent)
        self._draw_text("MEM", panel_x + pad_x, panel_y + 62, 17, theme.muted)
        self._draw_text(
            self._memory_text(stats),
            panel_x + 86,
            panel_y + 62,
            17,
            theme.text if stats.memory_used_bytes is not None else theme.muted,
        )
        self._draw_text(
            self._percent_text(mem_percent),
            panel_x + panel_w - pad_x,
            panel_y + 62,
            17,
            mem_color,
            anchor="right",
        )
        self._draw_percent_bar(panel_x + pad_x, panel_y + 80, panel_w - pad_x * 2, 12, mem_percent, mem_color)

        cpu_header_y = panel_y + 104
        self._draw_text("CPU CORE %", panel_x + pad_x, cpu_header_y, 15, theme.muted)
        if cpu_count == 0:
            self._draw_text("unavailable", panel_x + panel_w - pad_x, cpu_header_y, 15, theme.muted, anchor="right")
            return

        core_start_y = panel_y + header_h
        gap_x = 18.0 if columns == 2 else 10.0
        cell_w = (panel_w - pad_x * 2 - gap_x * (columns - 1)) / columns
        for index, percent in enumerate(stats.cpu_core_percents):
            row = index // columns
            column = index % columns
            cell_x = panel_x + pad_x + column * (cell_w + gap_x)
            line_y = core_start_y + row * core_row_h
            color = self._system_metric_color(percent)
            text_size = 15 if columns == 2 else 12
            self._draw_text(f"C{index}", cell_x, line_y + 8, text_size, theme.muted)
            self._draw_text(self._percent_text(percent), cell_x + cell_w, line_y + 8, text_size, color, anchor="right")
            self._draw_percent_bar(cell_x, line_y + 19, cell_w, 6, percent, color)

    def _draw_live_debug_panel(self, state: ClusterUiState) -> None:
        sections = self._live_debug_sections(state)
        if not sections:
            return

        theme = self._current_theme()
        panel_x = SYSTEM_PANEL_X
        panel_y = SYSTEM_PANEL_Y
        panel_w = SYSTEM_PANEL_W
        pad_x = 24.0
        header_h = 54.0
        section_title_h = 20.0
        row_h = 24.0
        section_gap = 10.0
        content_h = sum(section_title_h + len(rows) * row_h for _, rows in sections)
        content_h += max(0, len(sections) - 1) * section_gap
        panel_h = min(DESIGN_HEIGHT - SYSTEM_PANEL_Y - 18.0, header_h + content_h + 18.0)
        max_y = panel_y + panel_h - 18.0

        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 18, theme.route_panel_bg, theme.faint, 2)
        self._draw_text("LIVE DEBUG", panel_x + pad_x, panel_y + 28, 18, theme.muted)

        y = panel_y + header_h
        label_x = panel_x + pad_x
        value_x = panel_x + panel_w - pad_x
        label_w = 168.0
        value_max_w = panel_w - pad_x * 2 - label_w - 12.0
        for section_index, (section_title, rows) in enumerate(sections):
            if section_index > 0:
                line_y = y - section_gap * 0.45
                rl.draw_line_ex(
                    rl.Vector2(panel_x + pad_x, line_y),
                    rl.Vector2(panel_x + panel_w - pad_x, line_y),
                    1.0,
                    rl_color(theme.faint),
                )
            if y + section_title_h * 0.5 > max_y:
                break
            self._draw_text(section_title, label_x, y + 8.0, 15, theme.muted)
            y += section_title_h
            for label, value in rows:
                if y + row_h * 0.5 > max_y:
                    break
                self._draw_text(label, label_x, y + 8.0, 17, theme.muted)
                value = self._ellipsize_text(value, 17, value_max_w)
                self._draw_text(value, value_x, y + 8.0, 17, theme.text, anchor="right")
                y += row_h
            y += section_gap

    def _live_debug_sections(self, state: ClusterUiState) -> tuple[tuple[str, tuple[tuple[str, str], ...]], ...]:
        sections: list[tuple[str, tuple[tuple[str, str], ...]]] = []
        live_debug = state.live_debug
        if live_debug is not None:
            if live_debug.live_delay_calibration_percent is not None or live_debug.live_delay_lateral_s is not None:
                sections.append(
                    (
                        "LIVE DELAY",
                        (
                            (
                                "CAL / LAT",
                                f"{self._optional_percent_text(live_debug.live_delay_calibration_percent)} / "
                                f"{self._optional_seconds_text(live_debug.live_delay_lateral_s, 2)}",
                            ),
                        ),
                    )
                )
            if (
                live_debug.live_torque_calibration_percent is not None
                or live_debug.live_torque_valid is not None
                or live_debug.live_torque_lat_accel_factor is not None
                or live_debug.live_torque_friction is not None
            ):
                live_valid = "--" if live_debug.live_torque_valid is None else "ON" if live_debug.live_torque_valid else "OFF"
                sections.append(
                    (
                        "LIVE TORQUE",
                        (
                            (
                                "STATE",
                                f"{live_valid} / {self._optional_percent_text(live_debug.live_torque_calibration_percent)}",
                            ),
                            (
                                "FACT / FRIC",
                                f"{self._optional_float_text(live_debug.live_torque_lat_accel_factor, 2)} / "
                                f"{self._optional_float_text(live_debug.live_torque_friction, 2)}",
                            ),
                        ),
                    )
                )
            if (
                live_debug.live_steer_ratio is not None
                or live_debug.custom_steer_ratio is not None
                or live_debug.steer_actuator_delay_s is not None
            ):
                sections.append(
                    (
                        "STEERING",
                        (
                            (
                                "SR LIVE / CUSTOM",
                                f"{self._optional_float_text(live_debug.live_steer_ratio, 1)} / "
                                f"{self._optional_float_text(live_debug.custom_steer_ratio, 1)}",
                            ),
                            ("SAD", self._optional_seconds_text(live_debug.steer_actuator_delay_s, 2)),
                        ),
                    )
                )
        if state.lateral_plan_debug_text:
            sections.append(
                (
                    "LATERAL PLAN",
                    (("DEBUG", str(state.lateral_plan_debug_text)),),
                )
            )
        return tuple(sections)

    @staticmethod
    def _optional_percent_text(value: float | None) -> str:
        if value is None or not math.isfinite(value):
            return "--%"
        return f"{value:.0f}%"

    @staticmethod
    def _optional_float_text(value: float | None, digits: int) -> str:
        if value is None or not math.isfinite(value):
            return "--"
        return f"{value:.{digits}f}"

    def _optional_seconds_text(self, value: float | None, digits: int) -> str:
        text = self._optional_float_text(value, digits)
        return text if text == "--" else f"{text} s"

    def _draw_percent_bar(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        percent: float | None,
        fill: tuple[int, int, int],
    ) -> None:
        theme = self._current_theme()
        self._rounded_rect(x, y, width, height, height * 0.5, theme.gauge_bg)
        if percent is None:
            return
        fill_ratio = clamp(percent, 0.0, 100.0) / 100.0
        if fill_ratio <= 0.0:
            return
        fill_width = max(2.0, width * fill_ratio)
        self._rounded_rect(x, y, fill_width, height, height * 0.5, fill)

    @staticmethod
    def _memory_text(stats: SystemStats) -> str:
        if stats.memory_used_bytes is None or stats.memory_total_bytes is None:
            return "--/-- GB"
        used_gib = stats.memory_used_bytes / (1024.0 ** 3)
        total_gib = stats.memory_total_bytes / (1024.0 ** 3)
        return f"{used_gib:.1f}/{total_gib:.1f} GB"

    @staticmethod
    def _percent_text(percent: float | None) -> str:
        if percent is None:
            return "--%"
        return f"{clamp(percent, 0.0, 100.0):3.0f}%"

    def _system_metric_color(self, percent: float | None) -> tuple[int, int, int]:
        theme = self._current_theme()
        if percent is None:
            return theme.muted
        if percent >= 85.0:
            return RED
        if percent >= 60.0:
            return AMBER
        return BLUE

    def _draw_route_overlay(self, overlay: RouteOverlay | None) -> None:
        if overlay is None:
            return
        theme = self._current_theme()
        panel_x = 1416
        panel_y = 34
        panel_w = 476
        video_h = 244
        data_y = 300
        profile_stage = self._profile_start()
        self._rounded_rect(panel_x, panel_y, panel_w, 410, 18, theme.route_panel_bg, theme.faint, 2)
        self._profile_add("route_overlay.panel", profile_stage)
        profile_stage = self._profile_start()
        self._draw_route_video(overlay, panel_x + 10, panel_y + 10, panel_w - 20, video_h)
        self._profile_add("route_overlay.video", profile_stage)
        profile_stage = self._profile_start()
        self._draw_route_data(overlay, panel_x + 18, data_y, panel_w - 36)
        self._profile_add("route_overlay.data", profile_stage)

    def _draw_route_video(self, overlay: RouteOverlay, x: float, y: float, width: float, height: float) -> None:
        theme = self._current_theme()
        video_rect = rl.Rectangle(x, y, width, height)
        profile_stage = self._profile_start()
        rl.draw_rectangle_rounded(video_rect, 0.04, 10, rl_color(theme.route_video_bg))
        self._profile_add("route_video.background", profile_stage)
        if overlay.video_rgba is None or overlay.video_width <= 0 or overlay.video_height <= 0:
            status = overlay.video_status or "qcamera unavailable"
            profile_stage = self._profile_start()
            self._draw_text(status, x + width * 0.5, y + height * 0.5, 20, theme.route_video_status, anchor="center")
            self._profile_add("route_video.status_text", profile_stage)
            return

        profile_stage = self._profile_start()
        texture = self._route_video_texture_for_overlay(overlay)
        self._profile_add("route_video.texture_for_overlay", profile_stage)
        if texture is None:
            return
        source = rl.Rectangle(0.0, 0.0, float(overlay.video_width), float(overlay.video_height))
        scale = min(width / overlay.video_width, height / overlay.video_height)
        draw_w = overlay.video_width * scale
        draw_h = overlay.video_height * scale
        dest = rl.Rectangle(x + (width - draw_w) * 0.5, y + (height - draw_h) * 0.5, draw_w, draw_h)
        profile_stage = self._profile_start()
        rl.draw_texture_pro(texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))
        self._profile_add("route_video.draw_texture", profile_stage)

    def _route_video_texture_for_overlay(self, overlay: RouteOverlay):
        size = (overlay.video_width, overlay.video_height)
        if self._route_video_texture is None or self._route_video_size != size:
            if self._route_video_texture is not None:
                rl.unload_texture(self._route_video_texture)
            profile_stage = self._profile_start()
            image = rl.gen_image_color(overlay.video_width, overlay.video_height, rl_color((0, 0, 0)))
            self._route_video_texture = rl.load_texture_from_image(image)
            rl.unload_image(image)
            self._profile_add("route_video.alloc_texture", profile_stage)
            profile_stage = self._profile_start()
            rl.set_texture_filter(self._route_video_texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            self._profile_add("route_video.filter_texture", profile_stage)
            self._route_video_size = size
            self._route_video_frame_id = None

        if overlay.video_frame_id != self._route_video_frame_id:
            expected = overlay.video_width * overlay.video_height * 4
            if len(overlay.video_rgba or b"") != expected:
                return self._route_video_texture
            profile_stage = self._profile_start()
            pixels = rl.ffi.new("unsigned char[]", overlay.video_rgba)
            self._profile_add("route_video.copy_pixels", profile_stage)
            profile_stage = self._profile_start()
            rl.update_texture(self._route_video_texture, pixels)
            self._profile_add("route_video.update_texture", profile_stage)
            self._route_video_frame_id = overlay.video_frame_id
        return self._route_video_texture

    def _draw_route_data(self, overlay: RouteOverlay, x: float, y: float, width: float) -> None:
        theme = self._current_theme()
        self._draw_text("ROUTE DATA", x, y, 16, theme.muted)
        for index, line in enumerate(overlay.data_lines[:10]):
            self._draw_text(line, x, y + 22 + index * 14, 12, theme.text)

    def _draw_git_status(self, status: GitBranchStatus | None) -> None:
        if status is None:
            return

        theme = self._current_theme()
        color = self._git_status_color(status, theme)
        text = status.branch if not status.detail else f"{status.branch} ({status.detail})"
        text_size = 20
        text = self._ellipsize_text(text, text_size, GIT_STATUS_MAX_TEXT_W)
        spacing = max(1.0, text_size * 0.02)
        _, text_height = self._measure_text(text, text_size, spacing)
        row_h = max(text_height, GIT_STATUS_DOT_RADIUS * 2)
        center_y = DESIGN_HEIGHT - GIT_STATUS_MARGIN - row_h * 0.5
        dot_center_x = GIT_STATUS_MARGIN + GIT_STATUS_DOT_RADIUS
        text_x = GIT_STATUS_MARGIN + GIT_STATUS_DOT_RADIUS * 2 + GIT_STATUS_DOT_TEXT_GAP
        rl.draw_circle_v(rl.Vector2(dot_center_x, center_y), GIT_STATUS_DOT_RADIUS, rl_color(color))
        self._draw_text(text, text_x, center_y, text_size, color)

    def _draw_actual_fps(self, actual_fps: float | None) -> None:
        if actual_fps is None or not math.isfinite(actual_fps):
            return

        theme = self._current_theme()
        color = theme.muted
        text = f"FPS {actual_fps:.1f} Hz"
        text_size = 20
        text = self._ellipsize_text(text, text_size, FPS_STATUS_MAX_TEXT_W)
        spacing = max(1.0, text_size * 0.02)
        text_width, text_height = self._measure_text(text, text_size, spacing)
        row_h = max(text_height, FPS_STATUS_DOT_RADIUS * 2)
        center_y = FPS_STATUS_MARGIN + row_h * 0.5
        text_x = DESIGN_WIDTH - FPS_STATUS_MARGIN
        dot_center_x = text_x - text_width - FPS_STATUS_DOT_TEXT_GAP - FPS_STATUS_DOT_RADIUS
        rl.draw_circle_v(rl.Vector2(dot_center_x, center_y), FPS_STATUS_DOT_RADIUS, rl_color(GREEN))
        self._draw_text(text, text_x, center_y, text_size, color, anchor="right")

    def _draw_cluster_core_usage(self, text: str | None) -> None:
        if not text:
            return

        theme = self._current_theme()
        text_size = 18
        text = self._ellipsize_text(text, text_size, CLUSTER_CORE_USAGE_MAX_TEXT_W)
        spacing = max(1.0, text_size * 0.02)
        _, text_height = self._measure_text(text, text_size, spacing)
        x = DESIGN_WIDTH - CLUSTER_CORE_USAGE_MARGIN
        y = DESIGN_HEIGHT - CLUSTER_CORE_USAGE_MARGIN - text_height * 0.5
        self._draw_text(text, x, y, text_size, theme.muted, anchor="right")

    @staticmethod
    def _git_status_color(status: GitBranchStatus, theme: ClusterTheme) -> tuple[int, int, int]:
        if status.state == "ok":
            return GREEN
        if status.state == "pull":
            return AMBER
        if status.state == "missing":
            return RED
        return theme.muted

    def _draw_drive_status(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        gear_text = (state.gear_text or "").strip().upper()
        if (
            not state.debug_ui_visible
            and not gear_text
            and state.cruise_gap is None
            and not self._cruise_set_visible(state)
            and state.lfa_active is None
        ):
            return

        bottom_y = self._drive_status_bottom_y(state)
        gear_display = gear_text[:2] if gear_text else "-"
        gear_color = GREEN if gear_text and gear_text != "U" else theme.muted
        self._draw_drive_status_box(
            gear_display,
            GEAR_STATUS_CENTER_X,
            bottom_y - GEAR_STATUS_BOX_SIZE * 0.5,
            GEAR_STATUS_BOX_SIZE,
            GEAR_STATUS_FONT_SIZE,
            gear_color,
        )

        self._draw_follow_gap_status(state, bottom_y)
        self._draw_top_cruise_set(state, bottom_y)
        self._draw_lfa_status_icon(state, bottom_y)

    def _drive_status_bottom_y(self, state: ClusterUiState) -> float:
        speed_text = self._cruise_set_speed_text(state)
        speed_spacing = max(1.0, TOP_CRUISE_FONT_SIZE * 0.02)
        unit_spacing = max(1.0, TOP_CRUISE_UNIT_FONT_SIZE * 0.02)
        _, speed_h = self._measure_text(speed_text, TOP_CRUISE_FONT_SIZE, speed_spacing)
        _, unit_h = self._measure_text("km/h", TOP_CRUISE_UNIT_FONT_SIZE, unit_spacing)
        row_h = max(
            GEAR_STATUS_BOX_SIZE,
            FOLLOW_GAP_ICON_H,
            LFA_STATUS_ICON_SIZE,
            speed_h,
            unit_h,
        )
        return SPEED_LIMIT_SIGN_CENTER_Y - SPEED_LIMIT_SIGN_RADIUS + row_h

    def _draw_drive_status_box(
        self,
        text: str,
        center_x: float,
        center_y: float,
        box_size: float,
        font_size: float,
        text_color: tuple[int, int, int],
    ) -> None:
        box_x = center_x - box_size * 0.5
        box_y = center_y - box_size * 0.5
        rect = rl.Rectangle(box_x, box_y, box_size, box_size)
        roundness = max(0.0, min(1.0, DRIVE_STATUS_BOX_RADIUS / max(1.0, box_size)))
        rl.draw_rectangle_rounded_lines_ex(rect, roundness, 12, GEAR_STATUS_OUTLINE_WIDTH, rl_color(text_color))
        self._draw_text(
            text,
            center_x,
            center_y + 1,
            font_size,
            text_color,
            anchor="center",
        )

    def _draw_follow_gap_status(self, state: ClusterUiState, bottom_y: float) -> None:
        x = FOLLOW_STATUS_CENTER_X - FOLLOW_STATUS_W * 0.5

        gap_count = 0 if state.cruise_gap is None else int(clamp(float(state.cruise_gap), 1.0, float(FOLLOW_STATUS_GAP_BARS)))
        bar_w = FOLLOW_GAP_BAR_W * FOLLOW_GAP_BAR_SCALE
        bar_h = FOLLOW_GAP_BAR_H * FOLLOW_GAP_BAR_SCALE
        bar_r = FOLLOW_GAP_BAR_R * FOLLOW_GAP_BAR_SCALE
        bar_step = FOLLOW_GAP_BAR_STEP_X * FOLLOW_GAP_BAR_SCALE
        bars_total_w = bar_w + bar_step * (FOLLOW_STATUS_GAP_BARS - 1)
        icon_x = x + FOLLOW_STATUS_W - FOLLOW_GAP_ICON_W
        icon_y = bottom_y - FOLLOW_GAP_ICON_H
        bar_x = icon_x - bars_total_w - 3.0
        bar_y = bottom_y - bar_h
        for index in range(FOLLOW_STATUS_GAP_BARS):
            active = index >= FOLLOW_STATUS_GAP_BARS - gap_count
            self._rounded_rect(
                bar_x + index * bar_step,
                bar_y,
                bar_w,
                bar_h,
                bar_r,
                FOLLOW_GAP_ACTIVE if active else FOLLOW_GAP_INACTIVE,
                None,
                0.0,
            )

        self._draw_follow_vehicle_icon(icon_x, icon_y)

    def _draw_follow_vehicle_icon(self, x: float, y: float) -> None:
        texture = self._follow_vehicle_texture
        if texture is None:
            theme = self._current_theme()
            car_x = x + FOLLOW_GAP_ICON_W * 0.5
            car_y = y + FOLLOW_GAP_ICON_H * 0.5
            self._rounded_rect(car_x - 16, car_y - 8, 32, 16, 5.0, theme.muted, None, 0.0)
            self._rounded_rect(car_x - 7, car_y - 14, 15, 8, 4.0, theme.muted, None, 0.0)
            return

        source = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
        dest = rl.Rectangle(x, y, FOLLOW_GAP_ICON_W, FOLLOW_GAP_ICON_H)
        rl.draw_texture_pro(texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))

    def _draw_bottom_aligned_texture_icon(
        self,
        texture,
        center_x: float,
        bottom_y: float,
        width: float,
        height: float,
        tint: tuple[int, int, int] | tuple[int, int, int, int],
        alpha: int | None = None,
        rotation_deg: float = 0.0,
    ) -> bool:
        if texture is None:
            return False
        source = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
        dest = rl.Rectangle(center_x, bottom_y - height * 0.5, width, height)
        origin = rl.Vector2(width * 0.5, height * 0.5)
        rl.draw_texture_pro(texture, source, dest, origin, rotation_deg, rl_color(tint, alpha))
        return True

    def _draw_top_cruise_set(self, state: ClusterUiState, bottom_y: float) -> None:
        theme = self._current_theme()
        speed_text = self._cruise_set_speed_text(state)
        speed_color = self._cruise_set_color(state, theme)
        unit_color = speed_color
        speed_spacing = max(1.0, TOP_CRUISE_FONT_SIZE * 0.02)
        unit_spacing = max(1.0, TOP_CRUISE_UNIT_FONT_SIZE * 0.02)
        speed_w, _ = self._measure_text(speed_text, TOP_CRUISE_FONT_SIZE, speed_spacing)
        unit_w, _ = self._measure_text("km/h", TOP_CRUISE_UNIT_FONT_SIZE, unit_spacing)
        unit_gap = 5.0
        total_w = speed_w + unit_w + unit_gap
        start_x = TOP_CRUISE_CENTER_X - total_w * 0.5
        _, speed_h = self._measure_text(speed_text, TOP_CRUISE_FONT_SIZE, speed_spacing)
        _, unit_h = self._measure_text("km/h", TOP_CRUISE_UNIT_FONT_SIZE, unit_spacing)
        text_center_y = bottom_y - max(speed_h, unit_h) * 0.5
        self._draw_text(speed_text, start_x, text_center_y, TOP_CRUISE_FONT_SIZE, speed_color)
        self._draw_text("km/h", start_x + speed_w + unit_gap, text_center_y, TOP_CRUISE_UNIT_FONT_SIZE, unit_color)

    def _draw_lfa_status_icon(self, state: ClusterUiState, bottom_y: float) -> None:
        theme = self._current_theme()
        active = bool(state.lfa_active)
        texture = self._lfa_active_texture if active and self._lfa_active_texture is not None else self._lfa_texture
        tint = WHITE if active else theme.muted
        alpha = 255 if active else 190
        rotation_deg = -float(state.steering_angle_deg or 0.0)
        if self._draw_bottom_aligned_texture_icon(
            texture,
            LFA_STATUS_CENTER_X,
            bottom_y,
            LFA_STATUS_ICON_SIZE,
            LFA_STATUS_ICON_SIZE,
            tint,
            alpha,
            rotation_deg,
        ):
            return

        outline = GREEN if active else theme.muted
        fill_alpha = 46 if active else 26
        center = rl.Vector2(LFA_STATUS_CENTER_X, bottom_y - LFA_STATUS_ICON_SIZE * 0.5)
        scale = TOP_ICON_SIZE / 34.0
        rl.draw_circle_v(center, TOP_ICON_SIZE * 0.5, rl_color(outline, fill_alpha))
        rl.draw_circle_lines(int(center.x), int(center.y), TOP_ICON_SIZE * 0.5, rl_color(outline, 210))
        rl.draw_circle_lines(int(center.x), int(center.y + 1), TOP_ICON_SIZE * 0.26, rl_color(outline, 210))
        rl.draw_line_ex(
            rl.Vector2(center.x - 7 * scale, center.y + 5 * scale),
            rl.Vector2(center.x + 7 * scale, center.y + 5 * scale),
            2.2 * scale,
            rl_color(outline, 210),
        )
        rl.draw_line_ex(
            rl.Vector2(center.x, center.y + 5 * scale),
            rl.Vector2(center.x, center.y + 12 * scale),
            2.2 * scale,
            rl_color(outline, 210),
        )

    def _draw_speed_block(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        display_speed_kph = state.display_speed_kph if state.display_speed_kph is not None else state.speed_kph
        speed_value = int(round(clamp(display_speed_kph, 0.0, MAX_SPEED_KPH)))
        self._draw_text(str(speed_value), SPEED_VALUE_CENTER_X, SPEED_VALUE_CENTER_Y, 156, theme.text, anchor="center")

        if state.speed_limit_kph is not None or state.navi_debug is not None:
            center = rl.Vector2(SPEED_LIMIT_SIGN_CENTER_X, SPEED_LIMIT_SIGN_CENTER_Y)
            rl.draw_circle_v(center, SPEED_LIMIT_SIGN_RADIUS, rl_color(RED))
            rl.draw_circle_v(center, 47, rl_color(WHITE))
            limit_text = "--" if state.speed_limit_kph is None else str(state.speed_limit_kph)
            self._draw_text(
                limit_text,
                SPEED_LIMIT_SIGN_CENTER_X,
                SPEED_LIMIT_SIGN_CENTER_Y - 12,
                42,
                TEXT,
                anchor="center",
            )
            source_label = speed_limit_source_label(state.speed_limit_source) if state.speed_limit_kph is not None else ""
            if source_label:
                self._draw_text(
                    source_label,
                    SPEED_LIMIT_SIGN_CENTER_X,
                    SPEED_LIMIT_SIGN_CENTER_Y + 31,
                    17,
                    TEXT,
                    anchor="center",
                )

    @staticmethod
    def _cruise_set_visible(state: ClusterUiState) -> bool:
        return state.cruise_kph is not None and state.cruise_display_state != "off"

    @staticmethod
    def _cruise_set_speed_text(state: ClusterUiState) -> str:
        if state.cruise_display_state == "off" or state.cruise_kph is None:
            return "---"
        return str(int(round(state.cruise_kph)))

    @staticmethod
    def _cruise_set_color(state: ClusterUiState, theme: ClusterTheme) -> tuple[int, int, int]:
        if state.cruise_display_state == "off" or state.cruise_kph is None:
            return theme.muted
        if state.cruise_display_state == "paused":
            return theme.muted
        if state.speed_limit_kph is not None and state.cruise_kph == state.speed_limit_kph:
            return GREEN
        return BLUE

    def _draw_accel_block(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        top = 80
        bottom = 400
        center = (top + bottom) // 2
        gauge_width = 56
        accel_value = 0.0 if abs(state.accel_mps2) < 0.005 else state.accel_mps2
        accel_text = f"{accel_value:+05.2f}"
        accel_text_x = 20
        accel_text_size = 38
        text_spacing = max(1.0, accel_text_size * 0.02)
        if self._accel_text_width <= 0.0:
            self._accel_text_width = max(
                self._measure_text(text, accel_text_size, text_spacing)[0]
                for text in ACCEL_TEXT_WIDTH_SAMPLES
            )
        text_width = self._accel_text_width
        gauge_center_x = accel_text_x + text_width * 0.5
        gauge_x = gauge_center_x - gauge_width * 0.5
        fill_x = gauge_x + 8
        fill_width = 40
        self._rounded_rect(gauge_x, top, gauge_width, bottom - top, 18, theme.gauge_bg, theme.faint, 2)
        rl.draw_line_ex(
            rl.Vector2(gauge_x, center),
            rl.Vector2(gauge_x + gauge_width, center),
            3,
            rl_color(theme.gauge_midline),
        )
        value = clamp(state.accel_mps2, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2)
        fill_color = GREEN if value > 0 else RED if value < 0 else theme.muted
        if value != 0.0:
            fill_height = int(abs(value) / MAX_ACCEL_MPS2 * ((bottom - top) / 2 - 8))
            if value > 0:
                self._rounded_rect(fill_x, center - fill_height, fill_width, fill_height, 13, fill_color)
            else:
                self._rounded_rect(fill_x, center, fill_width, fill_height, 13, fill_color)
        self._draw_text(accel_text, accel_text_x, 48, accel_text_size, fill_color)
        self._draw_text("m/s^2", gauge_center_x, 424, 21, theme.muted, anchor="center")

    def _turn_signal_lights(self, state: ClusterUiState) -> tuple[bool, bool]:
        now = time.perf_counter()
        return (
            self._turn_signal_lit("left", state.left_signal, now),
            self._turn_signal_lit("right", state.right_signal, now),
        )

    @staticmethod
    def _highlight_lane_lit(state: ClusterUiState, signal_lights: tuple[bool, bool]) -> bool:
        left_signal_lit, right_signal_lit = signal_lights
        if state.highlight_lane == "left":
            return left_signal_lit
        if state.highlight_lane == "right":
            return right_signal_lit
        if state.left_signal != state.right_signal:
            return left_signal_lit if state.left_signal else right_signal_lit
        return True

    def _turn_signal_lit(self, side: str, active: bool, now: float | None = None) -> bool:
        if not active:
            if side == "left":
                self._left_turn_signal_started_at = None
            else:
                self._right_turn_signal_started_at = None
            return False

        if now is None:
            now = time.perf_counter()
        if side == "left":
            if self._left_turn_signal_started_at is None:
                self._left_turn_signal_started_at = now
            started_at = self._left_turn_signal_started_at
        else:
            if self._right_turn_signal_started_at is None:
                self._right_turn_signal_started_at = now
            started_at = self._right_turn_signal_started_at
        return blink_visible(now, started_at, float("inf"))

    def _draw_turn_signal(self, side: str, lit: bool, show_inactive: bool = False) -> None:
        if not lit and not show_inactive:
            return

        theme = self._current_theme()
        cx = TURN_SIGNAL_LEFT_CENTER_X if side == "left" else TURN_SIGNAL_RIGHT_CENTER_X
        cy = TURN_SIGNAL_CENTER_Y
        direction = -1 if side == "left" else 1
        fill = GREEN if lit else (*theme.muted, 42)
        outline = (8, 118, 65) if lit else (*theme.muted, 150)
        tail_back = -36
        tail_front = 12
        tail_half_height = 16
        head_tip_x = 60
        head_half_height = TURN_SIGNAL_HEAD_HALF_HEIGHT

        def point(local_x: float, local_y: float) -> rl.Vector2:
            return rl.Vector2(cx + direction * local_x, cy + local_y)

        tail_rect = rl.Rectangle(
            cx + direction * tail_back,
            cy - tail_half_height,
            direction * (tail_front - tail_back),
            tail_half_height * 2,
        )
        if tail_rect.width < 0:
            tail_rect.x += tail_rect.width
            tail_rect.width = -tail_rect.width

        head_top = point(tail_front, -head_half_height)
        head_tip = point(head_tip_x, 0)
        head_bottom = point(tail_front, head_half_height)
        if direction < 0:
            head_vertices = (head_top, head_tip, head_bottom)
        else:
            head_vertices = (head_top, head_bottom, head_tip)

        rl.draw_rectangle_rec(tail_rect, rl_color(fill))
        rl.draw_triangle(*head_vertices, rl_color(fill))

        outline_points = [
            point(tail_back, -tail_half_height),
            point(tail_front, -tail_half_height),
            head_top,
            head_tip,
            head_bottom,
            point(tail_front, tail_half_height),
            point(tail_back, tail_half_height),
        ]
        line_color = rl_color(outline)
        for index, start in enumerate(outline_points):
            end = outline_points[(index + 1) % len(outline_points)]
            rl.draw_line_ex(start, end, 3, line_color)

    def _rounded_rect(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        radius: float,
        fill: tuple[int, int, int],
        outline: tuple[int, int, int] | None = None,
        outline_width: float = 1.0,
    ) -> None:
        rect = rl.Rectangle(x, y, width, height)
        roundness = max(0.0, min(1.0, radius / max(1.0, min(width, height))))
        rl.draw_rectangle_rounded(rect, roundness, 12, rl_color(fill))
        if outline is not None and outline_width > 0:
            rl.draw_rectangle_rounded_lines_ex(rect, roundness, 12, outline_width, rl_color(outline))

    def _draw_text(
        self,
        text: str,
        x: float,
        y: float,
        size: float,
        color: tuple[int, int, int],
        anchor: str = "left",
    ) -> None:
        spacing = max(1.0, size * 0.02)
        text_width, text_height = self._measure_text(text, size, spacing)
        draw_x = x
        draw_y = y
        if anchor == "center":
            draw_x = x - text_width * 0.5
            draw_y = y - text_height * 0.5
        elif anchor == "left":
            draw_y = y - text_height * 0.5
        elif anchor == "right":
            draw_x = x - text_width
            draw_y = y - text_height * 0.5
        rl.draw_text_ex(self._font, text, rl.Vector2(draw_x, draw_y), size, spacing, rl_color(color))

    def _draw_text_with_stroke(
        self,
        text: str,
        x: float,
        y: float,
        size: float,
        color: tuple[int, int, int],
        stroke_color: tuple[int, int, int],
        stroke_width: int,
        anchor: str = "left",
    ) -> None:
        if stroke_width > 0:
            for dx, dy in (
                (-stroke_width, 0),
                (stroke_width, 0),
                (0, -stroke_width),
                (0, stroke_width),
                (-stroke_width, -stroke_width),
                (stroke_width, -stroke_width),
                (-stroke_width, stroke_width),
                (stroke_width, stroke_width),
            ):
                self._draw_text(text, x + dx, y + dy, size, stroke_color, anchor)
        self._draw_text(text, x, y, size, color, anchor)

    def _draw_world_label_text(
        self,
        text: str,
        x: float,
        y: float,
        size: float,
        color: tuple[int, int, int] | tuple[int, int, int, int],
        anchor: str = "left",
    ) -> None:
        if not self._world_label_texture_cache_enabled:
            self._draw_text(text, x, y, size, color, anchor)
            return

        cached_text = self._world_label_texture(text, size, color)
        if cached_text is None:
            self._draw_text(text, x, y, size, color, anchor)
            return

        draw_x = x
        draw_y = y
        if anchor == "center":
            draw_x = x - cached_text.text_width * 0.5
            draw_y = y - cached_text.text_height * 0.5
        elif anchor == "left":
            draw_y = y - cached_text.text_height * 0.5
        elif anchor == "right":
            draw_x = x - cached_text.text_width
            draw_y = y - cached_text.text_height * 0.5
        draw_x -= cached_text.padding_px
        draw_y -= cached_text.padding_px

        source = rl.Rectangle(
            0.0,
            0.0,
            float(cached_text.texture_width),
            float(cached_text.texture_height),
        )
        dest = rl.Rectangle(
            draw_x,
            draw_y,
            float(cached_text.texture_width),
            float(cached_text.texture_height),
        )
        rl.draw_texture_pro(
            cached_text.texture,
            source,
            dest,
            rl.Vector2(0.0, 0.0),
            0.0,
            rl_color(WHITE),
        )

    def _world_label_texture(
        self,
        text: str,
        size: float,
        color: tuple[int, int, int] | tuple[int, int, int, int],
    ) -> CachedTextTexture | None:
        if self._font is None:
            self._font = rl.get_font_default()
        render_size = max(
            1.0,
            round(float(size) / WORLD_LABEL_TEXTURE_SIZE_GRID) * WORLD_LABEL_TEXTURE_SIZE_GRID,
        )
        spacing = max(1.0, render_size * 0.02)
        color_key = rgba_key(color)
        cache_key = (id(self._font), text, render_size, spacing, color_key)
        cached_text = self._world_label_texture_cache.get(cache_key)
        if cached_text is not None:
            self._world_label_texture_cache.move_to_end(cache_key)
            return cached_text

        profile_stage = self._profile_start()
        text_width, text_height = self._measure_text(text, render_size, spacing)
        padding_px = float(WORLD_LABEL_TEXTURE_PADDING_PX)
        texture_width = max(1, int(math.ceil(text_width + padding_px * 2.0)))
        texture_height = max(1, int(math.ceil(text_height + padding_px * 2.0)))
        image = None
        texture = None
        try:
            image = rl.gen_image_color(texture_width, texture_height, rl_color((0, 0, 0, 0)))
            rl.image_draw_text_ex(
                image,
                self._font,
                text,
                rl.Vector2(padding_px, padding_px),
                render_size,
                spacing,
                rl_color(color_key),
            )
            texture = rl.load_texture_from_image(image)
            if hasattr(rl, "is_texture_valid") and not rl.is_texture_valid(texture):
                rl.unload_texture(texture)
                return None
        except Exception:
            if texture is not None:
                try:
                    rl.unload_texture(texture)
                except Exception:
                    pass
            return None
        finally:
            if image is not None:
                rl.unload_image(image)

        cached_text = CachedTextTexture(
            texture=texture,
            text_width=text_width,
            text_height=text_height,
            texture_width=texture_width,
            texture_height=texture_height,
            padding_px=padding_px,
        )
        self._world_label_texture_cache[cache_key] = cached_text
        while len(self._world_label_texture_cache) > WORLD_LABEL_TEXTURE_CACHE_LIMIT:
            _, old_text = self._world_label_texture_cache.popitem(last=False)
            rl.unload_texture(old_text.texture)
        self._profile_add("world_label_texture_cache.miss", profile_stage)
        return cached_text

    def _measure_text(self, text: str, size: float, spacing: float | None = None) -> tuple[float, float]:
        if self._font is None:
            self._font = rl.get_font_default()
        measure_spacing = max(1.0, size * 0.02) if spacing is None else spacing
        key = (id(self._font), text, float(size), float(measure_spacing))
        measured = self._text_measure_cache.get(key)
        if measured is not None:
            return measured
        if len(self._text_measure_cache) >= TEXT_MEASURE_CACHE_LIMIT:
            self._text_measure_cache.clear()
        text_size = rl.measure_text_ex(self._font, text, size, measure_spacing)
        measured = (float(text_size.x), float(text_size.y))
        self._text_measure_cache[key] = measured
        return measured

    def _ellipsize_text(self, text: str, size: float, max_width: float) -> str:
        spacing = max(1.0, size * 0.02)
        if self._measure_text(text, size, spacing)[0] <= max_width:
            return text
        ellipsis = "..."
        low = 0
        high = len(text)
        while low < high:
            mid = (low + high + 1) // 2
            candidate = text[:mid] + ellipsis
            if self._measure_text(candidate, size, spacing)[0] <= max_width:
                low = mid
            else:
                high = mid - 1
        return text[:low] + ellipsis
