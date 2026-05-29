from __future__ import annotations

import time
from dataclasses import dataclass

DESIGN_WIDTH = 1920
DESIGN_HEIGHT = 480

Color3 = tuple[int, int, int]
Color4 = tuple[int, int, int, int]


@dataclass(frozen=True, slots=True)
class ClusterTheme:
    name: str
    is_dark: bool
    bg: Color3
    panel_bg: Color3
    text: Color3
    muted: Color3
    faint: Color3
    road: Color3
    road_edge: Color3
    lane_marking_border: Color4
    road_edge_backing: Color4
    path_shadow: Color4
    path_uncertainty: Color4
    path_body: Color4
    path_highlight: Color4
    world_label_shadow: Color3
    world_label_text: Color3
    clock_bg: Color4
    clock_outline: Color4
    clock_text: Color3
    gauge_bg: Color3
    gauge_midline: Color3
    inactive_signal_fill: Color4
    inactive_signal_outline: Color3
    route_panel_bg: Color3
    route_video_bg: Color3
    route_video_status: Color3
    primary_vehicle: Color3
    model_vehicle: Color3
    default_vehicle: Color3


CLUSTER_THEME_AUTO = 0
CLUSTER_THEME_DARK = 1
CLUSTER_THEME_LIGHT = 2
CLUSTER_HUD_PARAM = "ClusterHud"
CLUSTER_BRIGHTNESS_PARAM = "ClusterHudBrightness"
CLUSTER_THEME_PARAM = "ClusterHudTheme"
CLUSTER_LIVE_FPS_PARAM = "ClusterHudLiveFps"
CLUSTER_SCREEN_MODE_DEFAULT = 0
CLUSTER_SCREEN_MODE_DEBUG = 1
CLUSTER_SCREEN_MODE_DEBUG_SYSTEM = 2
CLUSTER_SCREEN_MODE_DEBUG_GRAPH = 3
CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT = 4
CLUSTER_SCREEN_MODE_PARAM = "ClusterHudScreenMode"
SHOW_PLOT_MODE_PARAM = "ShowPlotMode"
AUTO_DARK_START_HOUR = 18
AUTO_LIGHT_START_HOUR = 6

LIGHT_CLUSTER_THEME = ClusterTheme(
    name="light",
    is_dark=False,
    bg=(244, 246, 248),
    panel_bg=(250, 251, 252),
    text=(20, 24, 28),
    muted=(104, 112, 120),
    faint=(214, 220, 226),
    road=(218, 222, 226),
    road_edge=(108, 122, 138),
    lane_marking_border=(54, 62, 70, 205),
    road_edge_backing=(72, 82, 92, 118),
    path_shadow=(56, 72, 88, 70),
    path_uncertainty=(112, 169, 255, 74),
    path_body=(34, 126, 255, 220),
    path_highlight=(222, 239, 255, 238),
    world_label_shadow=(245, 248, 252),
    world_label_text=(8, 10, 12),
    clock_bg=(8, 10, 12, 150),
    clock_outline=(255, 255, 255, 72),
    clock_text=(255, 255, 255),
    gauge_bg=(232, 236, 240),
    gauge_midline=(88, 96, 104),
    inactive_signal_fill=(195, 202, 209, 92),
    inactive_signal_outline=(168, 176, 184),
    route_panel_bg=(248, 250, 252),
    route_video_bg=(18, 20, 22),
    route_video_status=(212, 218, 224),
    primary_vehicle=(50, 66, 82),
    model_vehicle=(88, 100, 112),
    default_vehicle=(70, 78, 88),
)

DARK_CLUSTER_THEME = ClusterTheme(
    name="dark",
    is_dark=True,
    bg=(7, 10, 14),
    panel_bg=(18, 23, 29),
    text=(238, 242, 247),
    muted=(150, 160, 172),
    faint=(66, 76, 88),
    road=(30, 36, 43),
    road_edge=(118, 138, 158),
    lane_marking_border=(3, 6, 10, 205),
    road_edge_backing=(4, 8, 12, 196),
    path_shadow=(0, 0, 0, 110),
    path_uncertainty=(92, 154, 255, 82),
    path_body=(48, 146, 255, 230),
    path_highlight=(220, 238, 255, 245),
    world_label_shadow=(0, 0, 0),
    world_label_text=(238, 242, 247),
    clock_bg=(0, 0, 0, 172),
    clock_outline=(238, 242, 247, 72),
    clock_text=(255, 255, 255),
    gauge_bg=(18, 23, 29),
    gauge_midline=(98, 112, 128),
    inactive_signal_fill=(74, 86, 100, 92),
    inactive_signal_outline=(94, 108, 124),
    route_panel_bg=(16, 20, 25),
    route_video_bg=(5, 8, 12),
    route_video_status=(184, 194, 206),
    primary_vehicle=(92, 112, 134),
    model_vehicle=(108, 122, 138),
    default_vehicle=(84, 96, 110),
)


def normalize_cluster_theme_mode(value: object) -> str:
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("auto", "dark", "light"):
            return normalized
        try:
            value = int(normalized)
        except ValueError:
            return "auto"
    if value == CLUSTER_THEME_DARK:
        return "dark"
    if value == CLUSTER_THEME_LIGHT:
        return "light"
    return "auto"


def normalize_cluster_live_fps(value: object) -> float:
    if isinstance(value, str):
        normalized = value.strip()
        try:
            value = int(normalized)
        except ValueError:
            return 0.0
    try:
        mode = int(value)
    except (TypeError, ValueError):
        return 0.0
    if mode == 1:
        return 10.0
    if mode == 2:
        return 20.0
    if mode == 3:
        return 30.0
    return 0.0


def normalize_cluster_brightness_percent(value: object) -> int:
    if isinstance(value, str):
        normalized = value.strip()
        try:
            value = int(normalized)
        except ValueError:
            return 0
    try:
        brightness = int(value)
    except (TypeError, ValueError):
        return 0
    return min(100, max(0, brightness))


def normalize_cluster_screen_mode(value: object) -> int:
    if isinstance(value, str):
        normalized = value.strip().lower()
        aliases = {
            "default": CLUSTER_SCREEN_MODE_DEFAULT,
            "debug": CLUSTER_SCREEN_MODE_DEBUG,
            "system": CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
            "debug-system": CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
            "debug_system": CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
            "graph": CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
            "graph-full": CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
            "graph_full": CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
            "graph-right": CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
            "graph_right": CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
            "debug-graph": CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
            "debug_graph": CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
            "debug-graph-right": CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
            "debug_graph_right": CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
        }
        if normalized in aliases:
            return aliases[normalized]
        try:
            value = int(normalized)
        except ValueError:
            return CLUSTER_SCREEN_MODE_DEFAULT
    try:
        mode = int(value)
    except (TypeError, ValueError):
        return CLUSTER_SCREEN_MODE_DEFAULT
    if mode in (
        CLUSTER_SCREEN_MODE_DEFAULT,
        CLUSTER_SCREEN_MODE_DEBUG,
        CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
        CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
        CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
    ):
        return mode
    return CLUSTER_SCREEN_MODE_DEFAULT


def current_cluster_theme(mode: object = "auto", now: float | None = None) -> ClusterTheme:
    normalized = normalize_cluster_theme_mode(mode)
    if normalized == "dark":
        return DARK_CLUSTER_THEME
    if normalized == "light":
        return LIGHT_CLUSTER_THEME

    local_hour = time.localtime(now).tm_hour if now is not None else time.localtime().tm_hour
    if local_hour >= AUTO_DARK_START_HOUR or local_hour < AUTO_LIGHT_START_HOUR:
        return DARK_CLUSTER_THEME
    return LIGHT_CLUSTER_THEME


BG = LIGHT_CLUSTER_THEME.bg
PANEL_BG = LIGHT_CLUSTER_THEME.panel_bg
TEXT = LIGHT_CLUSTER_THEME.text
MUTED = LIGHT_CLUSTER_THEME.muted
FAINT = LIGHT_CLUSTER_THEME.faint
ROAD = LIGHT_CLUSTER_THEME.road
ROAD_EDGE = LIGHT_CLUSTER_THEME.road_edge
WHITE = (255, 255, 255)
BLUE = (38, 132, 255)
BLUE_SOFT = (168, 207, 255)
GREEN = (20, 188, 104)
AMBER = (244, 172, 54)
RED = (222, 72, 64)
EGO = (32, 89, 179)
CAR_DARK = LIGHT_CLUSTER_THEME.default_vehicle

MAX_SPEED_KPH = 140.0
MAX_ACCEL_MPS2 = 5.0
CONTROLLER_ACCEL_MPS2 = 3.2
CONTROLLER_BRAKE_MPS2 = 5.0
COAST_DECEL_MPS2 = 0.18
DRAG_DECEL_PER_MPS = 0.012
LANE_CHANGE_SECONDS = 4.2
LANE_CHANGE_MIN_SECONDS = 2.2
LANE_CHANGE_MAX_SECONDS = 4.8
LANE_RECENTER_SECONDS = 1.35
MODEL_DIRECT_LANE_RECENTER_SECONDS = 0.85
DEFAULT_LANE_WIDTH_M = 3.6
MAX_STEERING_ANGLE_DEG = 45.0
TURN_SIGNAL_SECONDS = 5.4
TURN_SIGNAL_BLINK_PERIOD_SECONDS = 1.0
TURN_SIGNAL_BLINK_ON_SECONDS = TURN_SIGNAL_BLINK_PERIOD_SECONDS * 0.5

CAMERA_CENTER_X = 1050.0
CAMERA_HORIZON_Y = 30.0
CAMERA_HEIGHT_M = 1.45
CAMERA_FOCAL_X = 240.0
CAMERA_FOCAL_Y = 1050.0
ROAD_NEAR_M = 0.75
ROAD_FAR_M = 90.0
ROAD_CURVE_M_PER_M2 = 0.0042
EGO_FORWARD_M = 4.18
PATH_START_M = 6.70
PATH_END_M = 72.0
PATH_HEIGHT_M = 0.10
PATH_LANE_CHANGE_CURVE_START_M = 6.70
PATH_LANE_CHANGE_CURVE_END_M = 15.50
SURROUND_MAX_YAW_DEG = 180.0
SURROUND_MAX_PITCH_DEG = 18.0
SURROUND_VIEW_SMOOTH_SECONDS = 0.16
SURROUND_CAMERA_DISTANCE_M = 6.3
SURROUND_CAMERA_HEIGHT_M = 2.65
SURROUND_TARGET_FORWARD_M = 7.6
SURROUND_TARGET_HEIGHT_M = 0.25
SURROUND_CENTER_Y = 265.0
SURROUND_FOCAL_X = 315.0
SURROUND_FOCAL_Y = 355.0
SURROUND_ROAD_REAR_M = -70.0
SURROUND_ROAD_FRONT_M = 115.0
SURROUND_ROAD_STEPS = 96
SURROUND_ROAD_NEAR_DEPTH_M = 0.75
VEHICLE_WIDTH_M = 1.82
VEHICLE_LENGTH_M = 4.35
VEHICLE_SURROUND_WIDTH_M = 1.05
VEHICLE_SURROUND_LENGTH_M = 1.85
VEHICLE_HEIGHT_M = 1.35
VEHICLE_SURROUND_HEIGHT_MULTIPLIER = 3.0
VEHICLE_LANE_CHANGE_SLOPE = 0.0
VEHICLE_AA_SCALE = 3
VEHICLE_CORNER_RADIUS_PX = 7.5
