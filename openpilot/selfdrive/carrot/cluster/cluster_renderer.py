from __future__ import annotations

from collections import OrderedDict
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, replace
from functools import lru_cache
import base64
import math
import os
import time
from pathlib import Path

import numpy as np
import pyray as rl

from openpilot.common.transformations.camera import DEVICE_CAMERAS, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.carrot.deceleration_source import navigation_status_presentation

from cluster_gles_dmabuf import DirectNv12DmabufError, create_tici_nv12_dmabuf_pool
from cluster_gles_readback import DirectNv12ReadbackError, create_tici_direct_readback
from cluster_display import (
    CLUSTER_LANGUAGE_KO,
    cluster_text,
    display_speed,
    format_navi_distance,
    format_radar_distance,
    format_trip_distance,
    normalize_cluster_language,
    speed_unit,
)
from cluster_config import (
    AMBER,
    BLUE,
    BLUE_SOFT,
    CLUSTER_PANEL_LAYOUT_DRIVING_LEFT,
    CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
    CLUSTER_RADAR_INFO_ALL_SPEED,
    CLUSTER_RADAR_INFO_ALL_SPEED_DISTANCE,
    CLUSTER_RADAR_INFO_NONE,
    CLUSTER_RADAR_INFO_VEHICLE_SPEED,
    CLUSTER_RADAR_INFO_VEHICLE_SPEED_DISTANCE,
    CLUSTER_RADAR_SOURCE_COLOR_BY_SOURCE,
    CLUSTER_SCREEN_MODE_DEBUG,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
    CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
    CLUSTER_SCREEN_MODE_NAVI,
    CLUSTER_SCREEN_MODE_TRIP_REPORT,
    CLUSTER_SCREEN_MODE_DEFAULT,
    CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
    ClusterTheme,
    DESIGN_HEIGHT,
    DESIGN_WIDTH,
    EGO_FORWARD_M,
    GREEN,
    MAX_ACCEL_MPS2,
    MAX_SPEED_KPH,
    RADAR_TO_CAMERA_M,
    RED,
    TEXT,
    VEHICLE_NAVI,
    VEHICLE_LENGTH_M,
    WHITE,
    cluster_camera_view_is_road_camera,
    cluster_camera_view_prefers_wide,
    cluster_wide_camera_zoom_factor,
    current_cluster_theme,
    normalize_cluster_screen_mode,
    normalize_cluster_panel_layout,
    normalize_cluster_theme_mode,
)
from cluster_models import (
    ClusterAlert,
    ClusterUiState,
    DebugPlotSnapshot,
    GitBranchStatus,
    NaviDebugInfo,
    NaviDashboardState,
    NaviGuidanceImage,
    NaviMediaFrame,
    NaviLaneInfo,
    NaviLiveState,
    NaviTrafficLightInfo,
    RouteOverlay,
    TripReportState,
)
from cluster_scene import (
    ClusterScene,
    MeshStrip,
    RADAR_STATIC_OBJECT_SPEED_KPH,
    RadarPointMarker,
    Vec3,
    VehicleBox,
    build_cluster_scene,
    cluster_scene_state_key,
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
TPMS_CAR_ICON_PATH = CLUSTER_DIR / "assets" / "images" / "tpms_toy_car.png"
SPEED_BG_PATH = SELFDRIVE_DIR / "assets" / "images" / "speed_bg.png"
TRAFFIC_RED_ICON_PATH = SELFDRIVE_DIR / "assets" / "images" / "traffic_red.png"
TRAFFIC_GREEN_ICON_PATH = SELFDRIVE_DIR / "assets" / "images" / "traffic_green.png"
FOLLOW_VEHICLE_ICON_PATH = SELFDRIVE_DIR / "assets" / "icons_mici" / "carrot_cruse_gap_trimmed.png"
LFA_ICON_PATH = SELFDRIVE_DIR / "assets" / "icons_mici" / "carrot_wheel_org.png"
LFA_LANE_ICON_PATH = SELFDRIVE_DIR / "assets" / "icons_mici" / "carrot_wheel_lane.png"
WIFI_ICON_PATH = SELFDRIVE_DIR / "assets" / "icons_mici" / "settings" / "network" / "wifi_strength_full.png"
ROUTE_CONTROL_PANEL_X = 340.0
ROUTE_CONTROL_PANEL_Y = DESIGN_HEIGHT - 74.0
ROUTE_CONTROL_PANEL_W = 1040.0
ROUTE_CONTROL_PANEL_H = 34.0
ROUTE_CONTROL_SEEK_Y = ROUTE_CONTROL_PANEL_Y + 18.0
ROUTE_CONTROL_BAR_X = ROUTE_CONTROL_PANEL_X + 142.0
ROUTE_CONTROL_BAR_W = ROUTE_CONTROL_PANEL_W - 284.0
# Keep the road camera strictly left of the navigation panel. These constants are
# shared so future panel layout changes cannot reintroduce an overlap.
NAVI_LIVE_PANEL_RIGHT = DESIGN_WIDTH - 4
NAVI_LIVE_PANEL_W = 792
NAVI_LIVE_PANEL_X = NAVI_LIVE_PANEL_RIGHT - NAVI_LIVE_PANEL_W
TRIP_REPORT_PANEL_X = NAVI_LIVE_PANEL_X
TRIP_REPORT_PANEL_Y = 1.0
TRIP_REPORT_PANEL_W = NAVI_LIVE_PANEL_W
TRIP_REPORT_PANEL_H = DESIGN_HEIGHT - 2.0
TRIP_REPORT_CACHE_REFRESH_SECONDS = 1.0
CAMERA_BACKGROUND_X = 0.0
CAMERA_BACKGROUND_Y = 0.0
CAMERA_BACKGROUND_W = NAVI_LIVE_PANEL_X
CAMERA_BACKGROUND_H = DESIGN_HEIGHT
CAMERA_BACKGROUND_ALPHA = 220
CAMERA_BACKGROUND_VIGNETTE_ALPHA = 32
CLUSTER_ALERT_SIZE_SMALL = 1
CLUSTER_ALERT_SIZE_MID = 2
CLUSTER_ALERT_SIZE_FULL = 3
CLUSTER_ALERT_TEXT_COLORS = {
    0: (255, 255, 255, 255),
    1: (255, 174, 82, 255),
    2: (255, 82, 96, 255),
}
CLUSTER_ALERT_TEXT_STROKE = (0, 0, 0, 255)
CLUSTER_SYNTHETIC_ALERT_TEXT_KEYS = {
    "clusterSelfdriveStartup": ("openpilot_unavailable", "waiting_to_start"),
    "clusterSelfdriveTimeout": ("take_control_immediately", "system_unresponsive"),
    "clusterSelfdriveReboot": ("system_unresponsive", "reboot_device"),
}
# 0.5 is a centered cover crop; values toward 1.0 retain more of the road
# camera's lower edge. Keep this shared with the projected overlay transform.
CAMERA_BACKGROUND_VERTICAL_BIAS = 0.75
CAMERA_OVERLAY_VEHICLE_ROAD_HEIGHT_M = 0.025
CAMERA_OVERLAY_MIN_DEPTH_M = 0.5
CAMERA_OVERLAY_FRAME_MIN_SIZE_PX = 24.0
CAMERA_OVERLAY_FRAME_MAX_WIDTH_RATIO = 0.22
CAMERA_OVERLAY_FRAME_MAX_HEIGHT_RATIO = 0.45
CAMERA_OVERLAY_FRAME_MAX_ASPECT = 2.4
CAMERA_OVERLAY_FRAME_EDGE_PAD_PX = 3.0
CAMERA_OVERLAY_FRAME_ROUND_SEGMENTS = 4
CAMERA_OVERLAY_DEFAULT_CAMERA = DEVICE_CAMERAS["tici", "ar0231"].fcam
CAMERA_OVERLAY_DEFAULT_HEIGHT_M = 1.22
CAMERA_OVERLAY_Z_OFFSET_DEFAULT_M = 0.00
CAMERA_OVERLAY_Z_OFFSET_MIN_M = -0.80
CAMERA_OVERLAY_Z_OFFSET_MAX_M = 0.40
CAMERA_OVERLAY_Z_OFFSET_STEP_M = 0.05
CAMERA_OVERLAY_TUNE_PANEL_Y = ROUTE_CONTROL_PANEL_Y - 38.0
CAMERA_OVERLAY_PITCH_OFFSET_DEFAULT_DEG = 0.0
CAMERA_OVERLAY_PITCH_OFFSET_MIN_DEG = -3.0
CAMERA_OVERLAY_PITCH_OFFSET_MAX_DEG = 3.0
CAMERA_OVERLAY_PITCH_OFFSET_STEP_DEG = 0.2
CAMERA_OVERLAY_PITCH_TUNE_PANEL_Y = ROUTE_CONTROL_PANEL_Y - 72.0
CORNER_RADAR_COIN_HEIGHT_M = 0.10
CORNER_RADAR_COIN_SLICES = 28
CORNER_RADAR_COIN_FILL = (52, 210, 230)
CORNER_RADAR_COIN_SIDE = (20, 116, 132)
CORNER_RADAR_COIN_RING = (185, 248, 255)
CORNER_RADAR_COIN_LEAD_ONE_FILL = (255, 170, 36)
CORNER_RADAR_COIN_LEAD_ONE_SIDE = (138, 82, 18)
CORNER_RADAR_COIN_LEAD_ONE_RING = (255, 190, 54)
CORNER_RADAR_COIN_LEAD_TWO_FILL = (255, 218, 64)
CORNER_RADAR_COIN_LEAD_TWO_SIDE = (142, 96, 16)
CORNER_RADAR_COIN_LEAD_TWO_RING = (255, 242, 132)
CORNER_RADAR_COIN_CUTIN_FILL = (198, 78, 238)
CORNER_RADAR_COIN_CUTIN_SIDE = (94, 34, 122)
CORNER_RADAR_COIN_CUTIN_RING = (238, 174, 255)
CORNER_RADAR_COIN_SLOWER_FILL = (54, 136, 255)
CORNER_RADAR_COIN_SLOWER_SIDE = (24, 65, 132)
CORNER_RADAR_COIN_SLOWER_RING = (146, 198, 255)
CORNER_RADAR_COIN_STOPPED_FILL = (132, 140, 148)
CORNER_RADAR_COIN_STOPPED_SIDE = (68, 74, 80)
CORNER_RADAR_COIN_STOPPED_RING = (205, 212, 218)
CORNER_RADAR_COIN_STOPPED_MAX_SPEED_KPH = 3.0
TPMS_LOW_PRESSURE_PSI = 31.0
TURN_SIGNAL_LEFT_CENTER_X = 610
TURN_SIGNAL_RIGHT_CENTER_X = 1310
TURN_SIGNAL_CENTER_Y = 72
TURN_SIGNAL_HEAD_HALF_HEIGHT = 38
TURN_SIGNAL_MID_CENTER_X = (TURN_SIGNAL_LEFT_CENTER_X + TURN_SIGNAL_RIGHT_CENTER_X) * 0.5
LANE_TURN_SIGNAL_LEFT_CENTER_X = 760
LANE_TURN_SIGNAL_RIGHT_CENTER_X = 1160
LANE_TURN_SIGNAL_CENTER_Y = 350
DRIVE_STATUS_BASE_BOX_SIZE = 46.0
DRIVE_STATUS_ROW_HEIGHT = TURN_SIGNAL_HEAD_HALF_HEIGHT * 2.0
DRIVE_STATUS_SCALE = DRIVE_STATUS_ROW_HEIGHT / DRIVE_STATUS_BASE_BOX_SIZE
GEAR_STATUS_CENTER_X = 900
GEAR_STATUS_CENTER_Y = TURN_SIGNAL_CENTER_Y
GEAR_STATUS_BOX_SIZE = DRIVE_STATUS_ROW_HEIGHT * 0.82
GEAR_STATUS_FONT_SIZE = 34.0 * DRIVE_STATUS_SCALE * 0.82
GEAR_STATUS_OUTLINE_WIDTH = 2.0 * DRIVE_STATUS_SCALE
FOLLOW_STATUS_CENTER_X = 1020
FOLLOW_STATUS_W = 160
FOLLOW_STATUS_H = 42.0 * DRIVE_STATUS_SCALE
FOLLOW_STATUS_GAP_BARS = 4
FOLLOW_GAP_ACTIVE = (103, 255, 78, 255)
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
WIFI_STATUS_CENTER_X = 160
WIFI_STATUS_ICON_SIZE = 48.0
EGPU_STATUS_CENTER_X = 245.0
EGPU_STATUS_W = 76.0
EGPU_STATUS_H = 34.0
EGPU_STATUS_FONT_SIZE = 19.0
NAV_STATUS_CENTER_X = WIFI_STATUS_CENTER_X - 22.0
NAV_STATUS_CENTER_Y = 99.0
NAV_STATUS_FONT_SIZE = 22.0
LFA_STATUS_CENTER_X = 70
LFA_STATUS_ICON_SIZE = 34.0 * DRIVE_STATUS_SCALE
LFA_LANE_ICON_WIDTH_SCALE = 2.0
LFA_LANE_ICON_TOP_OFFSET = 3.0
TOP_STATUS_CENTER_Y = 55.0
TOP_CLOCK_CENTER_X = 315.0
TOP_ICON_SIZE = 34.0 * DRIVE_STATUS_SCALE
DRIVE_STATUS_BOX_RADIUS = 8.0 * DRIVE_STATUS_SCALE
SPEED_HUD_SCALE = 0.8
SPEED_PANEL_X = 18
SPEED_PANEL_Y = 295
SPEED_PANEL_W = 380 * SPEED_HUD_SCALE
SPEED_PANEL_H = 142 * SPEED_HUD_SCALE
SPEED_VALUE_CENTER_X = SPEED_PANEL_X + 84 * SPEED_HUD_SCALE
SPEED_VALUE_CENTER_Y = SPEED_PANEL_Y + 70 * SPEED_HUD_SCALE
SPEED_VALUE_FONT_SIZE = 112 * SPEED_HUD_SCALE
CRUISE_SPEED_CENTER_Y = SPEED_PANEL_Y + 55 * SPEED_HUD_SCALE
CRUISE_SET_SPEED_CENTER_X = SPEED_PANEL_X + 289 * SPEED_HUD_SCALE
CRUISE_SET_SPEED_FONT_SIZE = 58 * SPEED_HUD_SCALE
CRUISE_OVERRIDE_SPEED_CENTER_X = SPEED_PANEL_X + 392 * SPEED_HUD_SCALE
CRUISE_OVERRIDE_SPEED_CENTER_Y = SPEED_PANEL_Y + 30 * SPEED_HUD_SCALE
CRUISE_OVERRIDE_LABEL_CENTER_Y = SPEED_PANEL_Y - 4 * SPEED_HUD_SCALE
CRUISE_OVERRIDE_LABEL_FONT_SIZE = 25 * SPEED_HUD_SCALE
CRUISE_OVERRIDE_SPEED_FONT_SIZE = 52 * SPEED_HUD_SCALE
# Center the gap number in the hollow of speed_bg.png's right curl.
SPEED_GAP_CENTER_X = SPEED_PANEL_X + SPEED_PANEL_W * 0.92
SPEED_GAP_CENTER_Y = SPEED_PANEL_Y + SPEED_PANEL_H * 0.835
SPEED_GAP_FONT_SIZE = 28.0
SPEED_GEAR_CENTER_X = SPEED_PANEL_X + SPEED_PANEL_W + 34.0
SPEED_GEAR_W = 45.0
SPEED_GEAR_H = 58.0
SPEED_GEAR_CENTER_Y = SPEED_PANEL_Y + SPEED_PANEL_H - SPEED_GEAR_H * 0.5
SPEED_GEAR_FONT_SIZE = 48.0
# A compact EV telltale fits between three-digit vehicle and cruise-set speeds.
SPEED_EV_CENTER_X = 181.0
SPEED_EV_CENTER_Y = SPEED_VALUE_CENTER_Y
SPEED_EV_FONT_SIZE = 28.0
SPEED_MODEL_TRAFFIC_CENTER_X = SPEED_VALUE_CENTER_X - 38.0
SPEED_MODEL_TRAFFIC_CENTER_Y = SPEED_PANEL_Y - 9.0
SPEED_MODEL_TRAFFIC_ICON_SIZE = 34.0
SPEED_DRIVING_MODE_GAP = 5.0
SPEED_DRIVING_MODE_BASE_CENTER_OFFSET_X = SPEED_DRIVING_MODE_GAP + 36.0
SPEED_DRIVING_MODE_CENTER_X = (
    SPEED_MODEL_TRAFFIC_CENTER_X
    + SPEED_MODEL_TRAFFIC_ICON_SIZE * 0.5
    + SPEED_DRIVING_MODE_BASE_CENTER_OFFSET_X
)
SPEED_DRIVING_MODE_CENTER_Y = SPEED_MODEL_TRAFFIC_CENTER_Y
SPEED_DRIVING_MODE_FONT_SIZE = 27.0
SPEED_DRIVING_MODE_STYLES = {
    1: ("driving_mode_eco", (0, 255, 0, 200)),
    2: ("driving_mode_safe", (255, 165, 0, 200)),
    3: ("driving_mode_normal", (255, 255, 255, 200)),
    4: ("driving_mode_sport", (255, 0, 0, 200)),
}
SIDE_GAUGE_TOP = 88
SIDE_GAUGE_BOTTOM = 186
SIDE_GAUGE_LOWER_TOP = 248
SIDE_GAUGE_LOWER_BOTTOM = 346
SIDE_GAUGE_WIDTH = 62
SIDE_GAUGE_VALUE_Y = 64
SIDE_GAUGE_LABEL_OFFSET = 15
SIDE_GAUGE_LEFT_CENTER_X = 968
SIDE_GAUGE_COLUMN_GAP = 78
SIDE_GAUGE_OUTLINE = (240, 244, 248, 190)
SPEED_LIMIT_SIGN_CENTER_X = 80
SPEED_LIMIT_SIGN_CENTER_Y = 160
SPEED_LIMIT_SIGN_RADIUS = 42.0
SPEED_LIMIT_SIGN_RING_WIDTH = 6.0
CRUISE_OVERRIDE_APPLY_COLOR = (184, 112, 24)
SYSTEM_PANEL_X = 1416
SYSTEM_PANEL_Y = 118
SYSTEM_PANEL_W = 476
NAVI_LIVE_PANEL_Y = 1
NAVI_LIVE_PANEL_H = DESIGN_HEIGHT - 2
NAVI_WORLD_VIEW_SHIFT_X = (DESIGN_WIDTH - NAVI_LIVE_PANEL_X) * 0.5
# Preserve each right-side widget's distance from the driving-view edge when
# expanding the 1124 px driving region to the full 1920 px display.
FULLSCREEN_3D_SIDE_WIDGET_OFFSET_X = DESIGN_WIDTH - CAMERA_BACKGROUND_W
FULLSCREEN_3D_CLOCK_CENTER_X = DESIGN_WIDTH * 0.5
FULLSCREEN_3D_TRAFFIC_PANEL_RIGHT = DESIGN_WIDTH - 28.0
NAVI_MAP_BACKGROUND = (0, 0, 0, 255)
TPMS_STATUS_CENTER_X = NAVI_LIVE_PANEL_X - 62.0
TPMS_STATUS_VALUE_CENTER_Y = 429.5
TPMS_STATUS_CAR_CENTER_Y = 429.5
TPMS_STATUS_COLUMN_OFFSET = 29.5
TPMS_STATUS_ROW_OFFSET = 24.0
TPMS_STATUS_CAR_W = 38.0
TPMS_STATUS_CAR_H = 72.0
TPMS_STATUS_WHEEL_W = 32.0
TPMS_STATUS_WHEEL_H = 30.0
TPMS_STATUS_ICON_W = 104.0
TPMS_STATUS_ICON_H = 78.0
FULLSCREEN_3D_CORE_USAGE_RIGHT_X = (
    TPMS_STATUS_CENTER_X
    + FULLSCREEN_3D_SIDE_WIDGET_OFFSET_X
    - TPMS_STATUS_ICON_W * 0.5
    - 18.0
)
TPMS_STATUS_FONT_SIZE = 21.0
NAVI_LIVE_ICON_X = NAVI_LIVE_PANEL_X + 72
NAVI_LIVE_ICON_Y = NAVI_LIVE_PANEL_Y + 99
NAVI_LIVE_ICON_SIZE = 78.0
NAVI_LIVE_CONTENT_X = NAVI_LIVE_PANEL_X + 136
NAVI_LIVE_CONTENT_W = NAVI_LIVE_PANEL_W - 158
NAVI_LIVE_NEXT_Y = NAVI_LIVE_PANEL_Y + 184
NAVI_LIVE_LANE_Y = NAVI_LIVE_PANEL_Y + 258
NAVI_LIVE_FOOTER_Y = NAVI_LIVE_PANEL_Y + 328
NAVI_LIVE_GUIDANCE_MEDIA_SCALE = 1.2
NAVI_TURN_LEFT_TYPES = frozenset((7, 12, 16, 17, 44, 75, 76, 102, 105, 112, 115, 118, 1000, 1002, 1006))
NAVI_TURN_RIGHT_TYPES = frozenset((6, 13, 19, 43, 73, 74, 101, 104, 111, 114, 117, 123, 124, 1001, 1003, 1007))
NAVI_TURN_ROUNDABOUT_TYPES = frozenset(range(131, 143))
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
NAVI_MODE_LEFT_W = 520.0
NAVI_MODE_MAP_X = NAVI_MODE_LEFT_W
NAVI_MODE_MAP_W = 880.0
NAVI_MODE_RIGHT_X = NAVI_MODE_MAP_X + NAVI_MODE_MAP_W
NAVI_MODE_RIGHT_W = DESIGN_WIDTH - NAVI_MODE_RIGHT_X
KOREAN_FONT_BASE_SIZE = 32
SYSTEM_STATS_REFRESH_SECONDS = 1.0
TEXT_MEASURE_CACHE_LIMIT = 1024
STROKED_TEXT_TEXTURE_CACHE_LIMIT = 384
STROKED_TEXT_TEXTURE_PADDING_PX = 2
TRIANGLE_STRIP_POINT_CACHE_LIMIT = 256
DEBUG_PLOT_MAX_SAMPLES = 360
DEBUG_PLOT_SAMPLE_SECONDS = 0.05
DEBUG_PLOT_MARGIN = 18.0
DEBUG_PLOT_FULL_X = 500.0
DEBUG_PLOT_FULL_Y = DEBUG_PLOT_MARGIN
DEBUG_PLOT_FULL_W = 1392.0
DEBUG_PLOT_FULL_H = DESIGN_HEIGHT - DEBUG_PLOT_MARGIN * 2.0
DEBUG_PLOT_RIGHT_X = NAVI_LIVE_PANEL_X
DEBUG_PLOT_RIGHT_Y = NAVI_LIVE_PANEL_Y
DEBUG_PLOT_RIGHT_W = NAVI_LIVE_PANEL_W
DEBUG_PLOT_RIGHT_H = NAVI_LIVE_PANEL_H
GIT_STATUS_MARGIN = 2
GIT_STATUS_BOTTOM_MARGIN = 12
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
NAVI_YUV420_FRAGMENT_SHADER = """
#ifdef GL_ES
precision mediump float;
#endif

varying vec2 fragTexCoord;

uniform sampler2D texture0;
uniform sampler2D texture1;
uniform sampler2D texture2;
uniform vec2 uvScaleU;
uniform vec2 uvScaleV;

void main() {
    float y = 1.164383 * (texture2D(texture0, fragTexCoord).r - 0.062745);
    float u = texture2D(texture1, fragTexCoord * uvScaleU).r - 0.501961;
    float v = texture2D(texture2, fragTexCoord * uvScaleV).r - 0.501961;
    vec3 rgb = vec3(
        y + 1.596027 * v,
        y - 0.391762 * u - 0.812968 * v,
        y + 2.017232 * u
    );
    gl_FragColor = vec4(clamp(rgb, 0.0, 1.0), 1.0);
}
"""
NAVI_EXTERNAL_VERTEX_SHADER = """
#version 300 es
precision mediump float;
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec4 vertexColor;
uniform mat4 mvp;
out vec2 fragTexCoord;
void main() {
    fragTexCoord = vertexTexCoord;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""
NAVI_EXTERNAL_FRAGMENT_SHADER = """
#version 300 es
#extension GL_OES_EGL_image_external_essl3 : enable
precision mediump float;
in vec2 fragTexCoord;
uniform samplerExternalOES texture0;
out vec4 fragColor;
void main() {
    fragColor = texture(texture0, fragTexCoord);
}
"""
NAVI_EGL_IMAGE_CACHE_LIMIT = 10


@dataclass(slots=True)
class CachedTextTexture:
    texture: object
    text_width: float
    text_height: float
    texture_width: int
    texture_height: int
    padding_px: float


StrokedTextTextureCacheKey = tuple[
    int,
    str,
    float,
    float,
    tuple[int, int, int, int],
    tuple[int, int, int, int],
    int,
]


@dataclass(frozen=True, slots=True)
class PendingStrokedTextTexture:
    font: object
    text: str
    render_size: float
    spacing: float
    fill_color: tuple[int, int, int, int]
    stroke_color: tuple[int, int, int, int]
    stroke_width: int


@dataclass(slots=True)
class CachedNaviMediaTexture:
    sequence: int
    size: tuple[int, int]
    mime: str
    texture: object
    u_texture: object | None = None
    v_texture: object | None = None
    uv_scale_u: object | None = None
    uv_scale_v: object | None = None
    hardware_token: tuple[int, int] | None = None


@dataclass(frozen=True, slots=True)
class CameraOverlayProjection:
    dest: rl.Rectangle
    source: rl.Rectangle
    video_dest: rl.Rectangle
    camera_width: float
    camera_height: float
    focal_length: float
    zoom: float
    video_tx: float
    video_ty: float
    view_from_road: tuple[tuple[float, float, float], ...]
    camera_height_m: float
    wide_camera: bool = False


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


def radar_point_distance_label(point: RadarPointMarker, is_metric: bool = True) -> str:
    if point.absolute_speed_kph is not None and abs(point.absolute_speed_kph) <= RADAR_STATIC_OBJECT_SPEED_KPH:
        return ""
    return format_radar_distance(point.longitudinal_m, is_metric)


def radar_point_speed_label(point: RadarPointMarker, is_metric: bool = True) -> str:
    if point.absolute_speed_kph is None:
        return ""
    if abs(point.absolute_speed_kph) <= RADAR_STATIC_OBJECT_SPEED_KPH:
        return ""
    return f"{display_speed(point.absolute_speed_kph, is_metric):.0f} {speed_unit(is_metric)}"


def vehicle_distance_label(vehicle: VehicleBox, is_metric: bool = True) -> str:
    if (
        vehicle.absolute_speed_kph is not None
        and abs(vehicle.absolute_speed_kph) <= RADAR_STATIC_OBJECT_SPEED_KPH
        and not vehicle.primary
        and not vehicle.cut_in
    ):
        return ""
    distance = format_radar_distance(vehicle_distance_m(vehicle), is_metric)
    if vehicle.cut_in:
        return f"CUT-IN {distance}"
    if (vehicle.primary or vehicle.cut_in) and vehicle.label:
        label = vehicle.label.upper()
        if label.startswith("L1") or label.startswith("L2"):
            return distance
        return f"{vehicle.label} {distance}"
    return distance


def vehicle_distance_m(vehicle: VehicleBox) -> float:
    if vehicle.longitudinal_m is not None:
        return vehicle.longitudinal_m
    return vehicle.center.y - EGO_FORWARD_M


def vehicle_speed_label(vehicle: VehicleBox, is_metric: bool = True) -> str:
    if vehicle.absolute_speed_kph is None:
        return ""
    if abs(vehicle.absolute_speed_kph) <= RADAR_STATIC_OBJECT_SPEED_KPH:
        return ""
    return f"{display_speed(vehicle.absolute_speed_kph, is_metric):.0f} {speed_unit(is_metric)}"


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


def camera_overlay_vehicle_coin_colors(
    vehicle: VehicleBox,
    lead_one: bool,
    lead_two: bool,
) -> tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]:
    if vehicle.cut_in or "CUT-IN" in vehicle.label.upper():
        return CORNER_RADAR_COIN_CUTIN_FILL, CORNER_RADAR_COIN_CUTIN_SIDE, CORNER_RADAR_COIN_CUTIN_RING
    if lead_one:
        return CORNER_RADAR_COIN_LEAD_ONE_FILL, CORNER_RADAR_COIN_LEAD_ONE_SIDE, CORNER_RADAR_COIN_LEAD_ONE_RING
    if lead_two:
        return CORNER_RADAR_COIN_LEAD_TWO_FILL, CORNER_RADAR_COIN_LEAD_TWO_SIDE, CORNER_RADAR_COIN_LEAD_TWO_RING
    if camera_overlay_vehicle_is_stopped(vehicle):
        return CORNER_RADAR_COIN_STOPPED_FILL, CORNER_RADAR_COIN_STOPPED_SIDE, CORNER_RADAR_COIN_STOPPED_RING
    if vehicle.absolute_speed_kph is not None and vehicle.absolute_speed_kph < 0.0:
        return RED, (116, 28, 32), RED
    if vehicle.relative_speed_mps is not None and vehicle.relative_speed_mps < 0.0:
        return CORNER_RADAR_COIN_SLOWER_FILL, CORNER_RADAR_COIN_SLOWER_SIDE, CORNER_RADAR_COIN_SLOWER_RING
    return CORNER_RADAR_COIN_FILL, CORNER_RADAR_COIN_SIDE, CORNER_RADAR_COIN_RING


def camera_overlay_vehicle_is_stopped(vehicle: VehicleBox) -> bool:
    return (
        vehicle.absolute_speed_kph is not None
        and abs(vehicle.absolute_speed_kph) <= CORNER_RADAR_COIN_STOPPED_MAX_SPEED_KPH
    )


def vehicle_metric_color(vehicle: VehicleBox, theme: ClusterTheme, source_color_mode: int) -> tuple[int, int, int]:
    if vehicle.cut_in:
        return AMBER
    if vehicle.primary:
        return theme.primary_vehicle
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
    return source in ("radarPoint", "liveTracks", "cornerRadar") or "+radar:" in source


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
    # Class defaults also keep lightweight object.__new__ render probes on the
    # original Korean/metric presentation without running the GPU-heavy init.
    language = CLUSTER_LANGUAGE_KO
    is_metric = True
    panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_LEFT

    def __init__(
        self,
        width: int = DESIGN_WIDTH,
        height: int = DESIGN_HEIGHT,
        title: str = "carrotpilot cluster",
        target_fps: int = 0,
        theme_mode: str = "auto",
        screen_mode: int = 0,
        panel_layout: int = CLUSTER_PANEL_LAYOUT_DRIVING_LEFT,
        language: str = CLUSTER_LANGUAGE_KO,
        is_metric: bool = True,
    ) -> None:
        self.width = width
        self.height = height
        self.title = title
        self.target_fps = target_fps
        self.theme_mode = normalize_cluster_theme_mode(theme_mode)
        self.screen_mode = normalize_cluster_screen_mode(screen_mode)
        self.panel_layout = normalize_cluster_panel_layout(panel_layout)
        self.language = normalize_cluster_language(language, default=CLUSTER_LANGUAGE_KO)
        self.is_metric = bool(is_metric)
        self._theme = current_cluster_theme(self.theme_mode)
        self.hidden = False
        self._window_open = False
        self._font = None
        self._owns_font = False
        self._korean_font = None
        self._owns_korean_font = False
        self._capture_target = None
        self._trip_report_target = None
        self._trip_report_cache_key: tuple[object, ...] | None = None
        self._trip_report_cache_valid = False
        self._trip_report_cache_visible = False
        self._trip_report_cache_next_refresh = 0.0
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
        self._direct_nv12_readback = None
        self._direct_nv12_readback_checked = False
        self._direct_nv12_readback_disabled = False
        self._nv12_dmabuf_pool = None
        self._nv12_dmabuf_pool_checked = False
        self._nv12_dmabuf_pool_disabled = False
        self._vehicle_model = None
        self._vehicle_model_load_attempted = False
        self._scene_cache_key: tuple[object, ...] | None = None
        self._scene_cache: ClusterScene | None = None
        self._speed_bg_texture = None
        self._traffic_red_texture = None
        self._traffic_green_texture = None
        self._follow_vehicle_texture = None
        self._lfa_texture = None
        self._lfa_active_texture = None
        self._lfa_lane_texture = None
        self._wifi_texture = None
        self._tpms_car_texture = None
        self._navi_guidance_texture = None
        self._navi_guidance_hash = ""
        self._navi_guidance_size: tuple[int, int] | None = None
        self._navi_media_textures: dict[str, CachedNaviMediaTexture] = {}
        self._navi_yuv_shader = None
        self._navi_yuv_shader_locations: dict[str, int] = {}
        self._navi_external_shader = None
        self._navi_egl_images: OrderedDict[tuple[int, int], object] = OrderedDict()
        self._route_video_texture = None
        self._route_video_size: tuple[int, int] | None = None
        self._route_video_frame_id: str | None = None
        self._live_road_camera = None
        self._live_road_camera_failed = False
        self._camera_overlay_wide = False
        self._left_turn_signal_started_at: float | None = None
        self._right_turn_signal_started_at: float | None = None
        self._hazard_signal_started_at: float | None = None
        self._lane_highlight_side: str | None = None
        self._lane_highlight_started_at: float | None = None
        self._triangle_strip_point_cache: OrderedDict[
            tuple[int, int],
            tuple[tuple[Vec3, ...], tuple[Vec3, ...], object, int],
        ] = OrderedDict()
        self._camera_overlay_strip_points = None
        self._camera_overlay_strip_point_capacity = 0
        self._camera_overlay_strip_pair_visibility = bytearray()
        self._raw_draw_triangle_strip_2d = getattr(getattr(rl, "rl", None), "DrawTriangleStrip", None)
        self._world_label_texture_cache: OrderedDict[
            tuple[int, str, float, float, tuple[int, int, int, int]],
            CachedTextTexture,
        ] = OrderedDict()
        self._world_label_texture_cache_enabled = os.environ.get("CLUSTER_WORLD_LABEL_TEXTURE_CACHE", "0") == "1"
        self._stroked_text_texture_cache: OrderedDict[StrokedTextTextureCacheKey, CachedTextTexture] = OrderedDict()
        self._pending_stroked_text_textures: OrderedDict[
            StrokedTextTextureCacheKey,
            PendingStrokedTextTexture,
        ] = OrderedDict()
        self._stroked_text_texture_cache_enabled = os.environ.get("CLUSTER_STROKED_TEXT_TEXTURE_CACHE", "1") != "0"
        self._raw_draw_text_ex = getattr(getattr(rl, "rl", None), "DrawTextEx", None)
        self._raw_stroked_text_enabled = (
            os.environ.get("CLUSTER_RAW_STROKED_TEXT", "1") != "0"
            and self._raw_draw_text_ex is not None
        )
        self._text_measure_cache: dict[tuple[int, str, float, float], tuple[float, float]] = {}
        self._system_stats = SystemStatsSampler(SYSTEM_STATS_REFRESH_SECONDS)
        self._debug_plot_mode_prev = -1
        self._debug_plot_size = 0
        self._debug_plot_index = -1
        self._debug_plot_values = [[0.0] * DEBUG_PLOT_MAX_SAMPLES for _ in range(3)]
        self._debug_plot_min = -2.0
        self._debug_plot_max = 2.0
        self._debug_plot_last_sample_time: float | None = None
        self.camera_overlay_z_offset_m = CAMERA_OVERLAY_Z_OFFSET_DEFAULT_M
        self.camera_overlay_pitch_offset_deg = CAMERA_OVERLAY_PITCH_OFFSET_DEFAULT_DEG
        self.route_camera_tuning_visible = os.environ.get("CLUSTER_ROUTE_CAMERA_TUNING") == "1"
        self.profile_enabled = os.environ.get("CLUSTER_PROFILE_RENDER") == "1"
        self._profile_samples: list[tuple[str, float]] = []

    def set_profile_enabled(self, enabled: bool) -> None:
        self.profile_enabled = enabled

    def set_theme_mode(self, theme_mode: str) -> None:
        self.theme_mode = normalize_cluster_theme_mode(theme_mode)
        self._theme = current_cluster_theme(self.theme_mode)
        self._invalidate_trip_report_cache()

    def set_screen_mode(self, screen_mode: int) -> None:
        self.screen_mode = normalize_cluster_screen_mode(screen_mode)
        self._invalidate_trip_report_cache()

    def set_panel_layout(self, panel_layout: int) -> None:
        self.panel_layout = normalize_cluster_panel_layout(panel_layout)
        self._invalidate_trip_report_cache()

    def set_display_preferences(self, language: str, is_metric: bool) -> None:
        self.language = normalize_cluster_language(language, default=CLUSTER_LANGUAGE_KO)
        self.is_metric = bool(is_metric)
        self._invalidate_trip_report_cache()

    def _invalidate_trip_report_cache(self) -> None:
        self._trip_report_cache_valid = False
        self._trip_report_cache_next_refresh = 0.0

    def _text(self, key: str) -> str:
        return cluster_text(self.language, key)

    def _panel_swap_active(self) -> bool:
        if self.panel_layout != CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT:
            return False
        return getattr(self, "screen_mode", CLUSTER_SCREEN_MODE_DEFAULT) not in (
            CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
            CLUSTER_SCREEN_MODE_NAVI,
        )

    def _driving_panel_offset_design_x(self) -> float:
        return NAVI_LIVE_PANEL_W if self._panel_swap_active() else 0.0

    def _information_panel_offset_design_x(self) -> float:
        return -NAVI_LIVE_PANEL_X if self._panel_swap_active() else 0.0

    def _information_panel_x(self, x: float) -> float:
        return x + self._information_panel_offset_design_x()

    def _driving_hud_offset_design_x(self, screen_mode: int) -> float:
        if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D:
            return 0.0
        return self._driving_panel_offset_design_x()

    @staticmethod
    def _tpms_offset_design_x(screen_mode: int) -> float:
        return FULLSCREEN_3D_SIDE_WIDGET_OFFSET_X if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D else 0.0

    def _side_gauge_offset_design_x(self, screen_mode: int) -> float:
        if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D:
            return FULLSCREEN_3D_SIDE_WIDGET_OFFSET_X
        return 0.0

    @staticmethod
    def _center_clock_x(screen_mode: int) -> float:
        return FULLSCREEN_3D_CLOCK_CENTER_X if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D else TOP_CLOCK_CENTER_X

    def _traffic_panel_right(self, screen_mode: int) -> float:
        if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D:
            return FULLSCREEN_3D_TRAFFIC_PANEL_RIGHT
        right = NAVI_TRAFFIC_PANEL_RIGHT
        if self._panel_swap_active():
            right -= NAVI_WORLD_VIEW_SHIFT_X
        return right

    def _core_usage_right_x(self, screen_mode: int) -> float:
        if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D:
            return FULLSCREEN_3D_CORE_USAGE_RIGHT_X
        return DESIGN_WIDTH + self._information_panel_offset_design_x()

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

    def direct_nv12_readback_available(self) -> bool:
        if self._direct_nv12_readback_disabled:
            return False
        if not self._direct_nv12_readback_checked:
            self._direct_nv12_readback_checked = True
            try:
                self._direct_nv12_readback = create_tici_direct_readback()
            except DirectNv12ReadbackError:
                self._direct_nv12_readback_disabled = True
        return self._direct_nv12_readback is not None

    def nv12_dmabuf_output_available(self) -> bool:
        if self._nv12_dmabuf_pool_disabled:
            return False
        if not self._nv12_dmabuf_pool_checked:
            self._nv12_dmabuf_pool_checked = True
            try:
                self._nv12_dmabuf_pool = create_tici_nv12_dmabuf_pool()
            except DirectNv12DmabufError:
                self._nv12_dmabuf_pool_disabled = True
        return self._nv12_dmabuf_pool is not None

    def async_nv12_readback_available(self) -> bool:
        if not self.direct_nv12_readback_available():
            return False
        return bool(self._direct_nv12_readback.async_supported)

    def async_nv12_readback_can_enqueue(self) -> bool:
        readback = self._direct_nv12_readback
        return readback is not None and readback.async_can_enqueue()

    def async_nv12_readback_ready(self) -> bool:
        readback = self._direct_nv12_readback
        return readback is not None and readback.async_ready()

    def copy_async_nv12_readback(self, destination_address: int, destination_size: int) -> bool:
        readback = self._direct_nv12_readback
        if readback is None:
            raise DirectNv12ReadbackError("TICI GLES asynchronous NV12 readback is not available")
        profile_stage = self._profile_start()
        copied = readback.copy_ready(destination_address, destination_size)
        self._profile_add("render_to_nv12.readback_pbo_copy", profile_stage)
        return copied

    def disable_async_nv12_readback(self) -> None:
        if self._direct_nv12_readback is not None:
            self._direct_nv12_readback.disable_async()

    def disable_direct_nv12_readback(self) -> None:
        if self._direct_nv12_readback is not None:
            self._direct_nv12_readback.close()
        self._direct_nv12_readback = None
        self._direct_nv12_readback_disabled = True

    def disable_nv12_dmabuf_output(self) -> None:
        self.release_nv12_dmabuf_output()
        self._nv12_dmabuf_pool_disabled = True

    def release_nv12_dmabuf_output(self) -> None:
        if self._nv12_dmabuf_pool is not None:
            self._nv12_dmabuf_pool.close()
        self._nv12_dmabuf_pool = None
        self._nv12_dmabuf_pool_checked = False

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
        self._korean_font = self._load_korean_font()
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
        self._system_stats.close()
        if not self._window_open:
            return
        if self._direct_nv12_readback is not None:
            self._direct_nv12_readback.close()
            self._direct_nv12_readback = None
            self._direct_nv12_readback_checked = False
        self.release_nv12_dmabuf_output()
        if self._capture_target is not None:
            rl.unload_render_texture(self._capture_target)
            self._capture_target = None
        if self._trip_report_target is not None:
            rl.unload_render_texture(self._trip_report_target)
            self._trip_report_target = None
            self._trip_report_cache_key = None
            self._trip_report_cache_valid = False
            self._trip_report_cache_visible = False
            self._trip_report_cache_next_refresh = 0.0
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
        for cached_text in self._stroked_text_texture_cache.values():
            rl.unload_texture(cached_text.texture)
        self._stroked_text_texture_cache.clear()
        self._pending_stroked_text_textures.clear()
        if self._route_video_texture is not None:
            rl.unload_texture(self._route_video_texture)
            self._route_video_texture = None
        self._close_live_road_camera()
        if self._speed_bg_texture is not None:
            rl.unload_texture(self._speed_bg_texture)
            self._speed_bg_texture = None
        if self._traffic_red_texture is not None:
            rl.unload_texture(self._traffic_red_texture)
            self._traffic_red_texture = None
        if self._traffic_green_texture is not None:
            rl.unload_texture(self._traffic_green_texture)
            self._traffic_green_texture = None
        if self._follow_vehicle_texture is not None:
            rl.unload_texture(self._follow_vehicle_texture)
            self._follow_vehicle_texture = None
        if self._lfa_texture is not None:
            rl.unload_texture(self._lfa_texture)
            self._lfa_texture = None
        if self._lfa_active_texture is not None:
            rl.unload_texture(self._lfa_active_texture)
            self._lfa_active_texture = None
        if self._lfa_lane_texture is not None:
            rl.unload_texture(self._lfa_lane_texture)
            self._lfa_lane_texture = None
        if self._wifi_texture is not None:
            rl.unload_texture(self._wifi_texture)
            self._wifi_texture = None
        if self._tpms_car_texture is not None:
            rl.unload_texture(self._tpms_car_texture)
            self._tpms_car_texture = None
        if self._navi_guidance_texture is not None:
            rl.unload_texture(self._navi_guidance_texture)
            self._navi_guidance_texture = None
            self._navi_guidance_hash = ""
            self._navi_guidance_size = None
        for cached_media in self._navi_media_textures.values():
            self._unload_navi_media_texture(cached_media)
        self._navi_media_textures.clear()
        if self._navi_egl_images:
            from openpilot.system.ui.lib.egl import destroy_egl_image

            for egl_image in self._navi_egl_images.values():
                destroy_egl_image(egl_image)
            self._navi_egl_images.clear()
        if self._navi_external_shader is not None:
            rl.unload_shader(self._navi_external_shader)
            self._navi_external_shader = None
        if self._navi_yuv_shader is not None:
            rl.unload_shader(self._navi_yuv_shader)
            self._navi_yuv_shader = None
            self._navi_yuv_shader_locations = {}
        if self._owns_font and self._font is not None:
            rl.unload_font(self._font)
        if self._owns_korean_font and self._korean_font is not None:
            rl.unload_font(self._korean_font)
        self._font = None
        self._owns_font = False
        self._korean_font = None
        self._owns_korean_font = False
        if self._vehicle_model is not None:
            rl.unload_model(self._vehicle_model)
            self._vehicle_model = None
        self._vehicle_model_load_attempted = False
        self._scene_cache_key = None
        self._scene_cache = None
        self._camera_overlay_strip_points = None
        self._camera_overlay_strip_point_capacity = 0
        self._camera_overlay_strip_pair_visibility = bytearray()
        self._route_video_size = None
        self._route_video_frame_id = None
        rl.close_window()
        self._window_open = False

    def should_close(self) -> bool:
        return bool(self._window_open and rl.window_should_close())

    def render_frame(self, state: ClusterUiState) -> None:
        self.open()
        self._prepare_trip_report_cache(state)
        profile_stage = self._profile_start()
        rl.begin_drawing()
        self._profile_add("render_frame.begin_drawing", profile_stage)
        profile_stage = self._profile_start()
        self.render(state)
        self._profile_add("render_frame.render", profile_stage)
        profile_stage = self._profile_start()
        rl.end_drawing()
        self._profile_add("render_frame.end_drawing", profile_stage)
        self._flush_pending_stroked_text_textures()

    def render_route_replay_frame(
        self,
        state: ClusterUiState,
        playback_s: float,
        duration_s: float,
        corner_lateral_offset_m: float,
        paused: bool = False,
    ) -> None:
        self.open()
        self._prepare_trip_report_cache(state)
        profile_stage = self._profile_start()
        rl.begin_drawing()
        self._profile_add("render_route_frame.begin_drawing", profile_stage)
        try:
            profile_stage = self._profile_start()
            self.render(state)
            self._profile_add("render_route_frame.render", profile_stage)
            profile_stage = self._profile_start()
            self._draw_route_replay_controls(
                playback_s,
                duration_s,
                corner_lateral_offset_m,
                paused,
                self.route_camera_tuning_visible
                and cluster_camera_view_is_road_camera(state.camera_view_mode),
            )
            self._profile_add("render_route_frame.controls", profile_stage)
        finally:
            profile_stage = self._profile_start()
            rl.end_drawing()
            self._profile_add("render_route_frame.end_drawing", profile_stage)
            self._flush_pending_stroked_text_textures()

    def route_replay_control_input(
        self,
        playback_s: float,
        duration_s: float,
        corner_lateral_offset_m: float,
    ) -> tuple[float | None, float, bool]:
        if not self._window_open or duration_s <= 0.0:
            return None, corner_lateral_offset_m, False

        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        mouse = rl.get_mouse_position()
        mx = float(mouse.x) / max(0.001, sx)
        my = float(mouse.y) / max(0.001, sy)
        control_active = self._camera_overlay_tuning_input(mx, my) if self.route_camera_tuning_visible else False
        if not rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT):
            return None, corner_lateral_offset_m, control_active

        seek_rect = rl.Rectangle(ROUTE_CONTROL_BAR_X, ROUTE_CONTROL_SEEK_Y - 8.0, ROUTE_CONTROL_BAR_W, 16.0)
        if self._point_in_rect(mx, my, seek_rect):
            ratio = clamp((mx - ROUTE_CONTROL_BAR_X) / max(1.0, ROUTE_CONTROL_BAR_W), 0.0, 1.0)
            return ratio * duration_s, corner_lateral_offset_m, True
        return None, corner_lateral_offset_m, control_active

    def _camera_overlay_tuning_input(self, mx: float, my: float) -> bool:
        left_key = getattr(rl, "KEY_LEFT_BRACKET", 91)
        right_key = getattr(rl, "KEY_RIGHT_BRACKET", 93)
        pitch_down_key = getattr(rl, "KEY_SEMICOLON", 59)
        pitch_up_key = getattr(rl, "KEY_APOSTROPHE", 39)
        reset_key = getattr(rl, "KEY_ZERO", 48)
        changed = False
        if rl.is_key_pressed(left_key):
            self.camera_overlay_z_offset_m -= CAMERA_OVERLAY_Z_OFFSET_STEP_M
            changed = True
        if rl.is_key_pressed(right_key):
            self.camera_overlay_z_offset_m += CAMERA_OVERLAY_Z_OFFSET_STEP_M
            changed = True
        if rl.is_key_pressed(pitch_down_key):
            self.camera_overlay_pitch_offset_deg -= CAMERA_OVERLAY_PITCH_OFFSET_STEP_DEG
            changed = True
        if rl.is_key_pressed(pitch_up_key):
            self.camera_overlay_pitch_offset_deg += CAMERA_OVERLAY_PITCH_OFFSET_STEP_DEG
            changed = True
        if rl.is_key_pressed(reset_key):
            self.camera_overlay_z_offset_m = CAMERA_OVERLAY_Z_OFFSET_DEFAULT_M
            self.camera_overlay_pitch_offset_deg = CAMERA_OVERLAY_PITCH_OFFSET_DEFAULT_DEG
            changed = True

        z_tune_rect = rl.Rectangle(ROUTE_CONTROL_BAR_X, CAMERA_OVERLAY_TUNE_PANEL_Y - 8.0, ROUTE_CONTROL_BAR_W, 16.0)
        pitch_tune_rect = rl.Rectangle(ROUTE_CONTROL_BAR_X, CAMERA_OVERLAY_PITCH_TUNE_PANEL_Y - 8.0, ROUTE_CONTROL_BAR_W, 16.0)
        mouse_z_tuning = rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT) and self._point_in_rect(mx, my, z_tune_rect)
        mouse_pitch_tuning = rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT) and self._point_in_rect(mx, my, pitch_tune_rect)
        if mouse_z_tuning:
            ratio = clamp((mx - ROUTE_CONTROL_BAR_X) / max(1.0, ROUTE_CONTROL_BAR_W), 0.0, 1.0)
            self.camera_overlay_z_offset_m = (
                CAMERA_OVERLAY_Z_OFFSET_MIN_M
                + ratio * (CAMERA_OVERLAY_Z_OFFSET_MAX_M - CAMERA_OVERLAY_Z_OFFSET_MIN_M)
            )
            changed = True
        if mouse_pitch_tuning:
            ratio = clamp((mx - ROUTE_CONTROL_BAR_X) / max(1.0, ROUTE_CONTROL_BAR_W), 0.0, 1.0)
            self.camera_overlay_pitch_offset_deg = (
                CAMERA_OVERLAY_PITCH_OFFSET_MIN_DEG
                + ratio * (CAMERA_OVERLAY_PITCH_OFFSET_MAX_DEG - CAMERA_OVERLAY_PITCH_OFFSET_MIN_DEG)
            )
            changed = True

        if changed:
            self.camera_overlay_z_offset_m = clamp(
                self.camera_overlay_z_offset_m,
                CAMERA_OVERLAY_Z_OFFSET_MIN_M,
                CAMERA_OVERLAY_Z_OFFSET_MAX_M,
            )
            self.camera_overlay_pitch_offset_deg = clamp(
                self.camera_overlay_pitch_offset_deg,
                CAMERA_OVERLAY_PITCH_OFFSET_MIN_DEG,
                CAMERA_OVERLAY_PITCH_OFFSET_MAX_DEG,
            )
        return changed or mouse_z_tuning or mouse_pitch_tuning

    def route_replay_mouse_down(self) -> bool:
        if not self._window_open:
            return False
        return bool(rl.is_mouse_button_down(rl.MOUSE_BUTTON_LEFT))

    @staticmethod
    def _point_in_rect(x: float, y: float, rect: "rl.Rectangle") -> bool:
        return rect.x <= x <= rect.x + rect.width and rect.y <= y <= rect.y + rect.height

    def render(self, state: ClusterUiState, signal_lights: tuple[bool, bool] | None = None) -> None:
        """Draw one frame into the currently active raylib render target."""
        if signal_lights is None:
            signal_lights = self._turn_signal_lights(state)
        profile_stage = self._profile_start()
        if self.screen_mode in (CLUSTER_SCREEN_MODE_DEBUG_GRAPH, CLUSTER_SCREEN_MODE_NAVI):
            self._clear_world()
        else:
            self._render_world(state, signal_lights)
        self._profile_add("render.world", profile_stage)
        profile_stage = self._profile_start()
        self._draw_hud(state, signal_lights)
        self._profile_add("render.hud", profile_stage)
        profile_stage = self._profile_start()
        self._draw_alert_overlay(getattr(state, "alert", None))
        self._profile_add("render.alert", profile_stage)

    def _prepare_trip_report_cache(self, state: ClusterUiState, now: float | None = None) -> None:
        """Refresh the expensive trip-report panel outside the active frame target."""
        if self._effective_screen_mode(state) != CLUSTER_SCREEN_MODE_TRIP_REPORT:
            self._trip_report_cache_visible = False
            return

        refresh_time = time.monotonic() if now is None else float(now)
        theme = self._current_theme()
        cache_key = (
            theme,
            self.language,
            self.is_metric,
            self.panel_layout,
            int(self.width),
            int(self.height),
        )
        needs_refresh = (
            not self._trip_report_cache_visible
            or not self._trip_report_cache_valid
            or self._trip_report_target is None
            or self._trip_report_cache_key != cache_key
            or refresh_time >= self._trip_report_cache_next_refresh
        )
        if needs_refresh:
            profile_stage = self._profile_start()
            self._refresh_trip_report_cache(state)
            self._trip_report_cache_key = cache_key
            self._trip_report_cache_valid = True
            self._trip_report_cache_next_refresh = refresh_time + TRIP_REPORT_CACHE_REFRESH_SECONDS
            self._profile_add("trip_report_cache.refresh", profile_stage)
        self._trip_report_cache_visible = True

    def _refresh_trip_report_cache(self, state: ClusterUiState) -> None:
        target_width = int(round(TRIP_REPORT_PANEL_W))
        target_height = int(round(TRIP_REPORT_PANEL_H))
        if self._trip_report_target is None:
            self._trip_report_target = rl.load_render_texture(target_width, target_height)
            rl.set_texture_filter(
                self._trip_report_target.texture,
                rl.TextureFilter.TEXTURE_FILTER_BILINEAR,
            )

        panel_x = self._information_panel_x(TRIP_REPORT_PANEL_X)
        rl.begin_texture_mode(self._trip_report_target)
        try:
            rl.clear_background(rl_color((0, 0, 0, 0)))
            rl.rl_push_matrix()
            rl.rl_translatef(-panel_x, -TRIP_REPORT_PANEL_Y, 0.0)
            try:
                self._draw_trip_report_panel_contents(state)
            finally:
                rl.rl_pop_matrix()
        finally:
            rl.end_texture_mode()

    def _clear_world(self) -> None:
        theme = self._current_theme()
        profile_stage = self._profile_start()
        rl.clear_background(rl_color(theme.bg))
        self._profile_add("render_world.clear_background", profile_stage)

    def _render_world(self, state: ClusterUiState, signal_lights: tuple[bool, bool] | None = None) -> None:
        if signal_lights is None:
            signal_lights = self._turn_signal_lights(state)
        if not cluster_camera_view_is_road_camera(state.camera_view_mode) or not state.onroad:
            self._close_live_road_camera()
        theme = self._current_theme()
        profile_stage = self._profile_start()
        scene = self._scene_for_state(state, self._highlight_lane_lit(state, signal_lights), theme)
        self._profile_add("render_world.build_scene", profile_stage)
        profile_stage = self._profile_start()
        rl.clear_background(rl_color(theme.bg))
        self._profile_add("render_world.clear_background", profile_stage)
        if cluster_camera_view_is_road_camera(state.camera_view_mode):
            self._camera_overlay_wide = self._select_camera_overlay_stream(state)
            projection = self._camera_overlay_projection(state)
            profile_stage = self._profile_start()
            self._draw_camera_background(state, projection)
            self._profile_add("render_world.camera_background", profile_stage)
            profile_stage = self._profile_start()
            self._draw_camera_projected_overlay(scene, state, projection)
            self._profile_add("render_world.camera_projected_overlay", profile_stage)
        else:
            self._camera_overlay_wide = False
            profile_stage = self._profile_start()
            self._draw_scene(scene, state)
            self._profile_add("render_world.draw_scene", profile_stage)

    def _scene_for_state(
        self,
        state: ClusterUiState,
        highlight_lane_lit: bool,
        theme: ClusterTheme,
    ) -> ClusterScene:
        cache_key = (*cluster_scene_state_key(state), highlight_lane_lit, theme)
        if self._scene_cache is not None and cache_key == self._scene_cache_key:
            return self._scene_cache
        scene = build_cluster_scene(
            state,
            self._profile_add_elapsed if self.profile_enabled else None,
            highlight_lane_lit=highlight_lane_lit,
            theme=theme,
        )
        self._scene_cache_key = cache_key
        self._scene_cache = scene
        return scene

    def _draw_camera_background(
        self,
        state: ClusterUiState,
        projection: CameraOverlayProjection | None = None,
    ) -> None:
        if not cluster_camera_view_is_road_camera(state.camera_view_mode):
            return
        overlay = state.route_overlay
        if projection is None:
            projection = self._camera_overlay_projection(state)
        if projection is None:
            return
        texture = None
        if overlay is not None and overlay.video_rgba is not None:
            texture = self._route_video_texture_for_overlay(overlay)
        rl.begin_scissor_mode(
            int(round(projection.dest.x)),
            int(round(projection.dest.y)),
            int(round(projection.dest.width)),
            int(round(projection.dest.height)),
        )
        try:
            drew_camera = False
            if texture is not None:
                rl.draw_texture_pro(
                    texture,
                    projection.source,
                    projection.video_dest,
                    rl.Vector2(0.0, 0.0),
                    0.0,
                    rl_color(WHITE, CAMERA_BACKGROUND_ALPHA),
                )
                drew_camera = True
            else:
                live_camera = self._live_road_camera_view() if state.onroad else None
                if live_camera is not None:
                    try:
                        drew_camera = live_camera.draw(projection.video_dest)
                    except Exception as exc:
                        print(f"Cluster live road camera failed: {exc}", flush=True)
                        self._close_live_road_camera()
            if drew_camera:
                rl.draw_rectangle_rec(projection.dest, rl_color((0, 0, 0), CAMERA_BACKGROUND_VIGNETTE_ALPHA))
        finally:
            rl.end_scissor_mode()

    def _live_road_camera_view(self):
        live_camera = getattr(self, "_live_road_camera", None)
        if live_camera is not None:
            return live_camera
        if getattr(self, "_live_road_camera_failed", False) or os.name != "posix":
            return None
        try:
            from cluster_live_camera import LiveRoadCamera

            self._live_road_camera = LiveRoadCamera()
        except Exception as exc:
            print(f"Cluster live road camera disabled: {exc}", flush=True)
            self._live_road_camera_failed = True
        return self._live_road_camera

    def _select_camera_overlay_stream(self, state: ClusterUiState) -> bool:
        overlay = getattr(state, "route_overlay", None)
        if overlay is not None and overlay.video_rgba is not None:
            return overlay.camera_stream == "wide"
        live_camera = self._live_road_camera_view() if state.onroad else None
        if live_camera is None:
            return False
        prefer_wide = cluster_camera_view_prefers_wide(
            state.camera_view_mode,
            state.speed_kph,
            live_camera.is_wide,
        )
        try:
            return bool(live_camera.select_stream(prefer_wide))
        except Exception as exc:
            print(f"Cluster live camera stream selection failed: {exc}", flush=True)
            self._close_live_road_camera()
            return False

    def _close_live_road_camera(self) -> None:
        live_camera = getattr(self, "_live_road_camera", None)
        if live_camera is not None:
            live_camera.close()
            self._live_road_camera = None

    def _camera_overlay_content_rect(self) -> rl.Rectangle:
        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        return rl.Rectangle(
            (CAMERA_BACKGROUND_X + self._driving_panel_offset_design_x()) * sx,
            CAMERA_BACKGROUND_Y * sy,
            CAMERA_BACKGROUND_W * sx,
            CAMERA_BACKGROUND_H * sy,
        )

    @staticmethod
    def _camera_overlay_camera(state: ClusterUiState, wide_camera: bool = False):
        device_type = (state.camera_device_type or "").strip().lower()
        sensor = (state.camera_sensor or "").strip().lower()
        for key in ((device_type, sensor), ("unknown", sensor), (device_type, "unknown")):
            device_camera = DEVICE_CAMERAS.get(key)
            if device_camera is not None:
                camera = device_camera.ecam if wide_camera else device_camera.fcam
                return camera if camera.width > 0 and camera.height > 0 else device_camera.fcam
        if sensor:
            for (_, known_sensor), device_camera in DEVICE_CAMERAS.items():
                if known_sensor == sensor:
                    camera = device_camera.ecam if wide_camera else device_camera.fcam
                    return camera if camera.width > 0 and camera.height > 0 else device_camera.fcam
        default_device_camera = DEVICE_CAMERAS["tici", "ar0231"]
        return default_device_camera.ecam if wide_camera else CAMERA_OVERLAY_DEFAULT_CAMERA

    def _camera_overlay_projection(self, state: ClusterUiState) -> CameraOverlayProjection | None:
        overlay = state.route_overlay
        dest = self._camera_overlay_content_rect()
        roll = pitch = yaw = 0.0
        if state.camera_calibration_euler is not None and len(state.camera_calibration_euler) >= 3:
            roll, pitch, yaw = state.camera_calibration_euler
        pitch += math.radians(self.camera_overlay_pitch_offset_deg)

        device_from_road = rot_from_euler([roll, pitch, yaw]).dot(np.diag([1.0, -1.0, -1.0]))
        wide_camera = bool(getattr(self, "_camera_overlay_wide", False))
        if wide_camera and state.wide_camera_from_device_euler is not None:
            wide_from_device = rot_from_euler(state.wide_camera_from_device_euler)
            view_from_road = view_frame_from_device_frame.dot(wide_from_device).dot(device_from_road)
        else:
            view_from_road = view_frame_from_device_frame.dot(device_from_road)
        camera = self._camera_overlay_camera(state, wide_camera)
        source_width = (
            float(overlay.video_width)
            if overlay is not None and overlay.video_width > 0
            else float(camera.width)
        )
        source_height = (
            float(overlay.video_height)
            if overlay is not None and overlay.video_height > 0
            else float(camera.height)
        )
        intrinsic = camera.intrinsics
        calib_transform = intrinsic @ view_from_road
        kep = calib_transform @ np.array([1000.0, 0.0, 0.0])
        zoom = max(dest.width / float(camera.width), dest.height / float(camera.height))
        if wide_camera:
            zoom *= cluster_wide_camera_zoom_factor(state.speed_kph)
        cx = float(intrinsic[0, 2])
        cy = float(intrinsic[1, 2])
        max_x_offset = max(0.0, cx * zoom - dest.width * 0.5)
        max_y_offset = max(0.0, cy * zoom - dest.height * 0.5)
        x_offset = 0.0
        y_offset = 0.0
        if abs(float(kep[2])) > 1e-6:
            x_offset = clamp((float(kep[0]) / float(kep[2]) - cx) * zoom, -max_x_offset, max_x_offset)
            y_offset = clamp((float(kep[1]) / float(kep[2]) - cy) * zoom, -max_y_offset, max_y_offset)
        video_tx = (dest.width * 0.5 + dest.x - x_offset) - cx * zoom
        rendered_height = float(camera.height) * zoom
        vertical_crop = max(0.0, rendered_height - dest.height)
        video_ty = dest.y - y_offset - vertical_crop * CAMERA_BACKGROUND_VERTICAL_BIAS
        video_dest = rl.Rectangle(
            video_tx,
            video_ty,
            float(camera.width) * zoom,
            rendered_height,
        )
        camera_height_m = CAMERA_OVERLAY_DEFAULT_HEIGHT_M
        if state.road_transform_trans is not None and len(state.road_transform_trans) >= 3:
            height_m = float(state.road_transform_trans[2])
            if math.isfinite(height_m):
                camera_height_m = clamp(height_m, 0.5, 3.0)
        return CameraOverlayProjection(
            dest=dest,
            source=rl.Rectangle(0.0, 0.0, source_width, source_height),
            video_dest=video_dest,
            camera_width=float(camera.width),
            camera_height=float(camera.height),
            focal_length=float(camera.focal_length),
            zoom=zoom,
            video_tx=video_tx,
            video_ty=video_ty,
            view_from_road=tuple(tuple(float(value) for value in row) for row in view_from_road),
            camera_height_m=camera_height_m,
            wide_camera=wide_camera,
        )

    def _camera_overlay_screen_xy(
        self,
        point: Vec3,
        projection: CameraOverlayProjection,
        scene_shift_x_m: float = 0.0,
    ) -> tuple[float, float] | None:
        # Scene geometry is shifted forward for the synthetic 3D camera. Camera
        # projection uses the physical road coordinate whose origin is the ego.
        road_forward = float(point.y) - EGO_FORWARD_M
        road_left = -float(point.x + scene_shift_x_m)
        road_up = float(point.z) + self.camera_overlay_z_offset_m
        if road_forward <= CAMERA_OVERLAY_MIN_DEPTH_M:
            return None

        row_x, row_y, row_z = projection.view_from_road
        view_x = row_x[0] * road_forward + row_x[1] * road_left + row_x[2] * road_up
        view_y = (
            row_y[0] * road_forward
            + row_y[1] * road_left
            + row_y[2] * road_up
            + projection.camera_height_m
        )
        view_z = row_z[0] * road_forward + row_z[1] * road_left + row_z[2] * road_up
        if view_z <= CAMERA_OVERLAY_MIN_DEPTH_M or not (
            math.isfinite(view_x) and math.isfinite(view_y) and math.isfinite(view_z)
        ):
            return None

        camera_x = projection.focal_length * view_x / view_z + projection.camera_width * 0.5
        camera_y = projection.focal_length * view_y / view_z + projection.camera_height * 0.5
        screen_x = projection.zoom * camera_x + projection.video_tx
        screen_y = projection.zoom * camera_y + projection.video_ty
        clip_margin = 60.0
        if (
            screen_x < projection.dest.x - clip_margin
            or screen_x > projection.dest.x + projection.dest.width + clip_margin
            or screen_y < projection.dest.y - clip_margin
            or screen_y > projection.dest.y + projection.dest.height + clip_margin
        ):
            return None
        return screen_x, screen_y

    def _draw_camera_projected_overlay(
        self,
        scene: ClusterScene,
        state: ClusterUiState,
        projection: CameraOverlayProjection | None = None,
    ) -> None:
        if projection is None:
            projection = self._camera_overlay_projection(state)
        if projection is None:
            return

        rl.begin_scissor_mode(
            int(round(projection.dest.x)),
            int(round(projection.dest.y)),
            int(round(projection.dest.width)),
            int(round(projection.dest.height)),
        )
        try:
            profile_stage = self._profile_start()
            for strip in scene.highlight_lanes:
                self._draw_camera_overlay_strip(strip, projection, scene.scene_shift_x_m)
            for strip in scene.road_edges:
                self._draw_camera_overlay_strip(strip, projection, scene.scene_shift_x_m)
            for strip in scene.lane_markings:
                self._draw_camera_overlay_strip(strip, projection, scene.scene_shift_x_m)
            for strip in scene.planned_path:
                self._draw_camera_overlay_strip(strip, projection, scene.scene_shift_x_m)
            self._profile_add("render_world.camera_projected_overlay.strips", profile_stage)
            profile_stage = self._profile_start()
            for point in scene.radar_points:
                self._draw_camera_overlay_radar_point(point, projection, scene.scene_shift_x_m, state.radar_info_mode)
            self._profile_add("render_world.camera_projected_overlay.radar", profile_stage)
            profile_stage = self._profile_start()
            for vehicle in sorted(scene.vehicles, key=self._camera_overlay_vehicle_draw_key):
                self._draw_camera_overlay_vehicle(vehicle, projection, scene.scene_shift_x_m, state.radar_info_mode)
            self._profile_add("render_world.camera_projected_overlay.vehicles", profile_stage)
        finally:
            rl.end_scissor_mode()

    def _draw_camera_overlay_strip(
        self,
        strip: MeshStrip,
        projection: CameraOverlayProjection,
        scene_shift_x_m: float,
    ) -> None:
        count = min(len(strip.left), len(strip.right))
        if count < 2:
            return
        color = rl_color(strip.color)
        points = self._camera_overlay_strip_point_buffer(count * 2)
        pair_visible = self._camera_overlay_strip_visibility_buffer(count)
        x_offset_m = scene_shift_x_m + strip.x_offset_m
        project_point = self._camera_overlay_screen_xy
        left_points = strip.left
        right_points = strip.right
        for index in range(count):
            pair_visible[index] = 0
            left = project_point(left_points[index], projection, x_offset_m)
            right = project_point(right_points[index], projection, x_offset_m)
            if left is None or right is None:
                continue
            left_point = points[index * 2]
            left_point.x, left_point.y = left
            right_point = points[index * 2 + 1]
            right_point.x, right_point.y = right
            pair_visible[index] = 1

        draw_triangle_strip = getattr(self, "_raw_draw_triangle_strip_2d", None)
        if draw_triangle_strip is not None:
            point_ptr = rl.ffi.cast("struct Vector2 *", points)
            index = 0
            while index < count:
                # Split at rejected endpoint pairs so batching cannot bridge a
                # gap that the legacy per-segment clipping left empty.
                while index < count and not pair_visible[index]:
                    index += 1
                start = index
                while index < count and pair_visible[index]:
                    index += 1
                pair_count = index - start
                if pair_count >= 2:
                    # L0,R0,L1,R1 expands to the same two triangles and
                    # winding as the legacy DrawTriangle pair below.
                    draw_triangle_strip(point_ptr + start * 2, pair_count * 2, color)
            return

        for index in range(count - 1):
            if not pair_visible[index] or not pair_visible[index + 1]:
                continue
            left0 = points[index * 2]
            right0 = points[index * 2 + 1]
            left1 = points[(index + 1) * 2]
            right1 = points[(index + 1) * 2 + 1]
            rl.draw_triangle(left0, right0, right1, color)
            rl.draw_triangle(left0, right1, left1, color)

    def _camera_overlay_strip_point_buffer(self, point_count: int):
        capacity = getattr(self, "_camera_overlay_strip_point_capacity", 0)
        points = getattr(self, "_camera_overlay_strip_points", None)
        if points is not None and capacity >= point_count:
            return points

        capacity = 1 << max(0, point_count - 1).bit_length()
        points = rl.ffi.new("struct Vector2[]", capacity)
        self._camera_overlay_strip_points = points
        self._camera_overlay_strip_point_capacity = capacity
        return points

    def _camera_overlay_strip_visibility_buffer(self, pair_count: int) -> bytearray:
        pair_visible = getattr(self, "_camera_overlay_strip_pair_visibility", None)
        if pair_visible is None:
            pair_visible = bytearray()
        if len(pair_visible) < pair_count:
            capacity = 1 << max(0, pair_count - 1).bit_length()
            pair_visible.extend(bytes(capacity - len(pair_visible)))
        self._camera_overlay_strip_pair_visibility = pair_visible
        return pair_visible

    def _draw_camera_overlay_vehicle(
        self,
        vehicle: VehicleBox,
        projection: CameraOverlayProjection,
        scene_shift_x_m: float,
        radar_info_mode: int,
    ) -> None:
        self._draw_camera_overlay_vehicle_frame(vehicle, projection, scene_shift_x_m, radar_info_mode)

    @staticmethod
    def _camera_overlay_vehicle_draw_key(vehicle: VehicleBox) -> tuple[int, float]:
        label = vehicle.label.upper()
        if vehicle.primary or label.startswith("L1"):
            priority = 5
        elif vehicle.cut_in or label.startswith("L2") or "CUT-IN" in label:
            priority = 4
        elif vehicle.source == "cornerRadar":
            priority = 3
        else:
            priority = 2
        return priority, -vehicle_distance_m(vehicle)

    def _draw_camera_overlay_vehicle_frame(
        self,
        vehicle: VehicleBox,
        projection: CameraOverlayProjection,
        scene_shift_x_m: float,
        radar_info_mode: int,
    ) -> None:
        center_y_m = vehicle.center.y + RADAR_TO_CAMERA_M + VEHICLE_LENGTH_M
        base_z = CAMERA_OVERLAY_VEHICLE_ROAD_HEIGHT_M
        center = self._camera_overlay_screen_xy(
            Vec3(vehicle.center.x, center_y_m, base_z),
            projection,
            scene_shift_x_m,
        )
        if center is None:
            return

        label = vehicle.label.upper()
        lead_two = label.startswith("L2") or "CUT-IN" in label
        lead_one = label.startswith("L1") or (vehicle.primary and not lead_two)
        emphasized = vehicle.primary or vehicle.cut_in
        half_width_m = max(0.6, vehicle.width_m * 0.58)
        frame_height_m = max(0.8, vehicle.height_m * 1.12)
        left_base = self._camera_overlay_screen_xy(
            Vec3(
                vehicle.center.x - half_width_m,
                center_y_m,
                base_z,
            ),
            projection,
            scene_shift_x_m,
        )
        right_base = self._camera_overlay_screen_xy(
            Vec3(
                vehicle.center.x + half_width_m,
                center_y_m,
                base_z,
            ),
            projection,
            scene_shift_x_m,
        )
        left_top = self._camera_overlay_screen_xy(
            Vec3(
                vehicle.center.x - half_width_m,
                center_y_m,
                base_z + frame_height_m,
            ),
            projection,
            scene_shift_x_m,
        )
        right_top = self._camera_overlay_screen_xy(
            Vec3(
                vehicle.center.x + half_width_m,
                center_y_m,
                base_z + frame_height_m,
            ),
            projection,
            scene_shift_x_m,
        )
        if left_base is None or right_base is None or left_top is None or right_top is None:
            return

        min_x = min(left_base[0], right_base[0], left_top[0], right_top[0])
        max_x = max(left_base[0], right_base[0], left_top[0], right_top[0])
        min_y = min(left_base[1], right_base[1], left_top[1], right_top[1])
        max_y = max(left_base[1], right_base[1], left_top[1], right_top[1])
        width = max_x - min_x
        height = max_y - min_y
        if not (math.isfinite(width) and math.isfinite(height)) or width <= 0.0 or height <= 0.0:
            return
        pad_x = max(4.0, width * 0.08)
        pad_y = max(4.0, height * 0.08)
        frame_width = clamp(
            width + pad_x * 2.0,
            CAMERA_OVERLAY_FRAME_MIN_SIZE_PX,
            projection.dest.width * CAMERA_OVERLAY_FRAME_MAX_WIDTH_RATIO,
        )
        frame_height = clamp(
            height + pad_y * 2.0,
            CAMERA_OVERLAY_FRAME_MIN_SIZE_PX,
            projection.dest.height * CAMERA_OVERLAY_FRAME_MAX_HEIGHT_RATIO,
        )
        if frame_width > frame_height * CAMERA_OVERLAY_FRAME_MAX_ASPECT:
            frame_width = frame_height * CAMERA_OVERLAY_FRAME_MAX_ASPECT
        elif frame_height > frame_width * CAMERA_OVERLAY_FRAME_MAX_ASPECT:
            frame_height = frame_width * CAMERA_OVERLAY_FRAME_MAX_ASPECT
        frame_center_x = (min_x + max_x) * 0.5
        frame_center_y = (min_y + max_y) * 0.5
        available_half_width = min(
            frame_center_x - projection.dest.x - CAMERA_OVERLAY_FRAME_EDGE_PAD_PX,
            projection.dest.x + projection.dest.width - frame_center_x - CAMERA_OVERLAY_FRAME_EDGE_PAD_PX,
        )
        available_half_height = min(
            frame_center_y - projection.dest.y - CAMERA_OVERLAY_FRAME_EDGE_PAD_PX,
            projection.dest.y + projection.dest.height - frame_center_y - CAMERA_OVERLAY_FRAME_EDGE_PAD_PX,
        )
        minimum_half_size = CAMERA_OVERLAY_FRAME_MIN_SIZE_PX * 0.5
        if available_half_width < minimum_half_size or available_half_height < minimum_half_size:
            return
        frame_width = min(frame_width, available_half_width * 2.0)
        frame_height = min(frame_height, available_half_height * 2.0)
        marker = rl.Rectangle(
            frame_center_x - frame_width * 0.5,
            frame_center_y - frame_height * 0.5,
            frame_width,
            frame_height,
        )

        confidence = clamp(vehicle.confidence, 0.0, 1.0)
        _fill_base, _side_base, ring_base = camera_overlay_vehicle_coin_colors(vehicle, lead_one, lead_two)
        ring_alpha = int(180 + 65 * confidence)
        rl.draw_rectangle_rounded_lines_ex(
            marker,
            0.20,
            CAMERA_OVERLAY_FRAME_ROUND_SEGMENTS,
            3.5 if emphasized else 3.0,
            rl_color(ring_base, ring_alpha),
        )

        label = self._vehicle_overlay_label(vehicle, radar_info_mode)
        if label:
            label_color = (*ring_base[:3], 255 if vehicle.primary or vehicle.cut_in else 230)
            label_y = marker.y - 12.0
            if lead_one:
                label_y = marker.y - 18.0
            elif lead_two:
                label_y = marker.y + marker.height + 16.0
            self._draw_world_label_text(
                label,
                marker.x + marker.width * 0.5,
                max(14.0, label_y),
                17 if emphasized else 15,
                label_color,
                anchor="center",
            )

    def _vehicle_overlay_label(self, vehicle: VehicleBox, radar_info_mode: int) -> str:
        parts: list[str] = []
        stopped = camera_overlay_vehicle_is_stopped(vehicle)
        if vehicle.label and (vehicle.primary or vehicle.cut_in):
            label = vehicle.label.upper()
            if not (label.startswith("L1") or label.startswith("L2") or "CUT-IN" in label):
                parts.append(vehicle.label)
        if (stopped or radar_info_shows_distance(radar_info_mode)) and vehicle.longitudinal_m is not None:
            parts.append(format_radar_distance(vehicle.longitudinal_m, self.is_metric))
        if not stopped and radar_info_shows_speed(radar_info_mode) and vehicle.absolute_speed_kph is not None:
            parts.append(f"{display_speed(vehicle.absolute_speed_kph, self.is_metric):.0f} {speed_unit(self.is_metric)}")
        return " ".join(parts)

    def _draw_camera_overlay_radar_point(
        self,
        point: RadarPointMarker,
        projection: CameraOverlayProjection,
        scene_shift_x_m: float,
        radar_info_mode: int,
    ) -> None:
        camera_point = Vec3(point.center.x, point.center.y + RADAR_TO_CAMERA_M, point.center.z)
        screen = self._camera_overlay_screen_xy(camera_point, projection, scene_shift_x_m)
        if screen is None:
            return
        radius = max(2.5, min(5.0, 40.0 / max(6.0, abs(point.longitudinal_m))))
        marker = rl.Rectangle(
            screen[0] - radius,
            screen[1] - radius,
            radius * 2.0,
            radius * 2.0,
        )
        rl.draw_rectangle_rec(marker, rl_color(point.color, 225))
        if not radar_info_shows_radar_points(radar_info_mode):
            return
        label_parts = []
        if radar_info_shows_distance(radar_info_mode):
            label_parts.append(format_radar_distance(point.longitudinal_m, self.is_metric))
        if point.absolute_speed_kph is not None:
            label_parts.append(f"{display_speed(point.absolute_speed_kph, self.is_metric):.0f} {speed_unit(self.is_metric)}")
        if label_parts:
            self._draw_world_label_text(
                " ".join(label_parts),
                screen[0],
                screen[1] - 16.0,
                15,
                (*WHITE[:3], 220),
                anchor="center",
            )

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
        destination_address: int | None = None,
        destination_size: int = 0,
        async_readback: bool = False,
        destination_dmabuf_fd: int | None = None,
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

        self._prepare_trip_report_cache(state)
        profile_stage = self._profile_start()
        target = self._get_capture_target()
        self._profile_add("render_to_nv12.get_capture_target", profile_stage)

        profile_stage = self._profile_start()
        rl.begin_texture_mode(target)
        self.render(state)
        rl.end_texture_mode()
        self._profile_add("render_to_nv12.draw_to_target", profile_stage)
        self._flush_pending_stroked_text_textures()

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
        if (destination_address is not None or async_readback or destination_dmabuf_fd is not None) and not pack_direct_input:
            raise DirectNv12ReadbackError("direct NV12 readback requires a four-byte packed Venus layout")
        if destination_address is not None and async_readback:
            raise DirectNv12ReadbackError("asynchronous NV12 readback cannot use a direct destination")
        if destination_dmabuf_fd is not None and (destination_address is not None or async_readback):
            raise DirectNv12DmabufError("encoder DMA-BUF output cannot use a readback destination")
        if pack_direct_input:
            full_pack_w = stride // 4
            full_pack_h = byte_count // stride
            tail_pack_h = max(0, full_pack_h - y_scanlines - uv_scanlines)
            uv_pack_y = tail_pack_h
            y_pack_y = tail_pack_h + uv_scanlines

            profile_stage = self._profile_start()
            if destination_dmabuf_fd is None:
                full_target = self._get_nv12_pack_target("full", full_pack_w, full_pack_h)
            else:
                dmabuf_pool = self._nv12_dmabuf_pool
                if dmabuf_pool is None:
                    raise DirectNv12DmabufError("encoder DMA-BUF render targets are unavailable")
                full_target = dmabuf_pool.target_for(destination_dmabuf_fd, stride, byte_count)
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

            if destination_dmabuf_fd is not None:
                profile_stage = self._profile_start()
                dmabuf_pool.wait_for_gpu()
                self._profile_add("render_to_nv12.dmabuf_fence_wait", profile_stage)
                yield destination_dmabuf_fd
                return

            if async_readback:
                readback = self._direct_nv12_readback
                if readback is None or not readback.async_supported:
                    raise DirectNv12ReadbackError("TICI GLES asynchronous NV12 readback is not available")
                profile_stage = self._profile_start()
                enqueued = readback.enqueue_rgba(
                    full_target.id,
                    full_target.texture.width,
                    full_target.texture.height,
                )
                self._profile_add("render_to_nv12.readback_pbo_enqueue", profile_stage)
                yield enqueued
                return

            if destination_address is not None:
                readback = self._direct_nv12_readback
                if readback is None:
                    raise DirectNv12ReadbackError("TICI GLES direct NV12 readback is not available")
                profile_stage = self._profile_start()
                readback.read_rgba(
                    full_target.id,
                    full_target.texture.width,
                    full_target.texture.height,
                    destination_address,
                    destination_size,
                )
                self._profile_add("render_to_nv12.readback_direct_ion", profile_stage)
                yield destination_address
                return

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
        self._prepare_trip_report_cache(state)
        profile_stage = self._profile_start()
        target = self._get_capture_target()
        self._profile_add("render_to_image.get_capture_target", profile_stage)

        profile_stage = self._profile_start()
        rl.begin_texture_mode(target)
        self.render(state)
        rl.end_texture_mode()
        self._profile_add("render_to_image.draw_to_target", profile_stage)
        self._flush_pending_stroked_text_textures()

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
        glyphs = None
        glyph_count = 0
        base_size = 160
        for candidate in self._font_candidates():
            if candidate.exists():
                try:
                    # Some TTFs have glyph bounds slightly taller than the requested
                    # size, which makes raylib print harmless FONT warnings at startup.
                    rl.set_trace_log_level(rl.TraceLogLevel.LOG_ERROR)
                    font = rl.load_font_ex(str(candidate), base_size, glyphs, glyph_count)
                    rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
                    if font.texture.id > 0:
                        rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
                        self._owns_font = True
                        return font
                except Exception as exc:
                    rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
                    print(f"Cluster font load failed for {candidate}: {exc}")
        self._owns_font = False
        return rl.get_font_default()

    def _load_korean_font(self):
        codepoints = self._navi_font_codepoints()
        glyph_buffer = rl.ffi.new("int[]", codepoints)
        glyphs = rl.ffi.cast("int *", glyph_buffer)
        for candidate in self._font_candidates():
            if not candidate.exists():
                continue
            try:
                rl.set_trace_log_level(rl.TraceLogLevel.LOG_ERROR)
                font = rl.load_font_ex(str(candidate), KOREAN_FONT_BASE_SIZE, glyphs, len(codepoints))
                rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
                if font.texture.id > 0:
                    rl.set_texture_filter(font.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
                    self._owns_korean_font = True
                    return font
            except Exception as exc:
                rl.set_trace_log_level(rl.TraceLogLevel.LOG_WARNING)
                print(f"Cluster Korean font load failed for {candidate}: {exc}")
        self._owns_korean_font = False
        return None

    def _font_candidates(self) -> list[Path]:
        return [
            KAIGEN_GOTHIC_KR_BOLD_FONT_PATH,
            OPENPILOT_ADDON_FONT_DIR / "KaiGenGothicKR-Bold.ttf",
            Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts" / "malgun.ttf",
            JETBRAINS_MONO_FONT_PATH,
            OPENPILOT_FONT_DIR / "JetBrainsMono-Bold.ttf",
            Path("/data/openpilot/openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf"),
            Path("/usr/share/fonts/truetype/jetbrains-mono/JetBrainsMono-Medium.ttf"),
            Path("/usr/share/fonts/TTF/JetBrainsMono-Medium.ttf"),
            Path("/usr/local/share/fonts/JetBrainsMono-Medium.ttf"),
        ]

    @staticmethod
    @lru_cache(maxsize=1)
    def _navi_font_codepoints() -> tuple[int, ...]:
        return (
            *range(0x20, 0x0250),
            *range(0x2000, 0x2070),
            0x20A9,
            *range(0x3131, 0x3190),
            *range(0xAC00, 0xD7A4),
        )

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
        if self._speed_bg_texture is None:
            self._speed_bg_texture = self._load_icon_texture(SPEED_BG_PATH, "Speed background")
        if self._traffic_red_texture is None:
            self._traffic_red_texture = self._load_alpha_cropped_icon_texture(TRAFFIC_RED_ICON_PATH, "Red traffic light")
        if self._traffic_green_texture is None:
            self._traffic_green_texture = self._load_alpha_cropped_icon_texture(
                TRAFFIC_GREEN_ICON_PATH,
                "Green traffic light",
            )
        if self._lfa_texture is None:
            self._lfa_texture = self._load_icon_texture(LFA_ICON_PATH, "LFA")
        if self._lfa_active_texture is None:
            self._lfa_active_texture = self._load_lfa_active_texture()
        if self._lfa_lane_texture is None:
            self._lfa_lane_texture = self._load_icon_texture(LFA_LANE_ICON_PATH, "LFA lane mode")
        if self._wifi_texture is None:
            self._wifi_texture = self._load_icon_texture(WIFI_ICON_PATH, "Wi-Fi")
        if self._tpms_car_texture is None:
            self._tpms_car_texture = self._load_icon_texture(TPMS_CAR_ICON_PATH, "TPMS toy car")

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

    def _load_alpha_cropped_icon_texture(self, path: Path, label: str):
        if not path.exists():
            return None
        image = None
        try:
            image = rl.load_image(str(path))
            if not rl.is_image_valid(image):
                return None
            if image.format != rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8:
                rl.image_format(image, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8A8)

            data = rl.ffi.cast("unsigned char *", image.data)
            min_x, min_y = image.width, image.height
            max_x = max_y = -1
            for y in range(image.height):
                for x in range(image.width):
                    if int(data[(y * image.width + x) * 4 + 3]) <= 8:
                        continue
                    min_x = min(min_x, x)
                    min_y = min(min_y, y)
                    max_x = max(max_x, x)
                    max_y = max(max_y, y)
            if max_x < min_x or max_y < min_y:
                return None

            padding = 2
            min_x = max(0, min_x - padding)
            min_y = max(0, min_y - padding)
            max_x = min(image.width - 1, max_x + padding)
            max_y = min(image.height - 1, max_y + padding)
            rl.image_crop(image, rl.Rectangle(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1))

            texture = rl.load_texture_from_image(image)
            if texture.id <= 0:
                return None
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            return texture
        except Exception as exc:
            print(f"{label} icon load failed: {exc}")
            return None
        finally:
            if image is not None and rl.is_image_valid(image):
                rl.unload_image(image)

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
        view_shift_x = self._world_view_shift_x(state)
        driving_offset_x = 0.0
        if abs(view_shift_x) > 0.001:
            driving_offset_x = self._driving_panel_offset_design_x() * self.width / DESIGN_WIDTH
        scene_offset_x = driving_offset_x - view_shift_x
        if abs(scene_offset_x) > 0.001:
            rl.rl_viewport(int(round(scene_offset_x)), 0, self.width, self.height)
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
        if abs(scene_offset_x) > 0.001:
            rl.rl_viewport(0, 0, self.width, self.height)
            rl.rl_push_matrix()
            rl.rl_translatef(scene_offset_x, 0.0, 0.0)
        try:
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
        finally:
            if abs(scene_offset_x) > 0.001:
                rl.rl_pop_matrix()

    def _world_view_shift_x(self, state: ClusterUiState) -> float:
        screen_mode = self._effective_screen_mode(state)
        if cluster_camera_view_is_road_camera(state.camera_view_mode):
            return 0.0
        if screen_mode in (
            CLUSTER_SCREEN_MODE_DEBUG,
            CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
            CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
            CLUSTER_SCREEN_MODE_TRIP_REPORT,
        ):
            return NAVI_WORLD_VIEW_SHIFT_X * self.width / DESIGN_WIDTH
        if screen_mode != CLUSTER_SCREEN_MODE_DEFAULT:
            return 0.0
        navi_panel_visible = bool(
            self._navi_live_panel_visible(state.navi_live)
            or self._navi_map_frame_present(state.navi_dashboard)
            or state.navi_dashboard is not None
        )
        if not navi_panel_visible:
            return 0.0
        return NAVI_WORLD_VIEW_SHIFT_X * self.width / DESIGN_WIDTH

    def _world_view_shift_design_x(self, state: ClusterUiState) -> float:
        if self.width <= 0:
            return 0.0
        return self._world_view_shift_x(state) * DESIGN_WIDTH / self.width

    def _turn_signal_center_x_offset(self, state: ClusterUiState, side: str) -> float:
        signal_center_x = (
            LANE_TURN_SIGNAL_LEFT_CENTER_X if side == "left" else LANE_TURN_SIGNAL_RIGHT_CENTER_X
        )
        if self._effective_screen_mode(state) == CLUSTER_SCREEN_MODE_FULLSCREEN_3D:
            return 0.0
        if cluster_camera_view_is_road_camera(state.camera_view_mode):
            camera_signal_center_x = CAMERA_BACKGROUND_X + signal_center_x * CAMERA_BACKGROUND_W / DESIGN_WIDTH
            return camera_signal_center_x - signal_center_x
        view_shift_x = self._world_view_shift_design_x(state)
        if self._panel_swap_active() and abs(view_shift_x) <= 0.001:
            view_shift_x = NAVI_WORLD_VIEW_SHIFT_X
        return -view_shift_x

    def _draw_tpms_status(self, state: ClusterUiState) -> None:
        tpms = state.tpms
        pressures = (tpms.fl, tpms.fr, tpms.rl, tpms.rr)
        if not any(value is not None for value in pressures):
            return

        if not self._draw_tpms_car_icon():
            self._draw_tpms_car_fallback()

        badge_positions = (
            (
                TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
                tpms.fl,
            ),
            (
                TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
                tpms.fr,
            ),
            (
                TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
                tpms.rl,
            ),
            (
                TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
                tpms.rr,
            ),
        )
        for center_x, center_y, pressure in badge_positions:
            self._draw_compact_tpms_value(pressure, center_x, center_y)

    def _draw_tpms_car_icon(self) -> bool:
        texture = getattr(self, "_tpms_car_texture", None)
        if texture is None:
            return False
        source = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
        destination = rl.Rectangle(
            TPMS_STATUS_CENTER_X - TPMS_STATUS_ICON_W * 0.5,
            TPMS_STATUS_CAR_CENTER_Y - TPMS_STATUS_ICON_H * 0.5,
            TPMS_STATUS_ICON_W,
            TPMS_STATUS_ICON_H,
        )
        rl.draw_texture_pro(
            texture,
            source,
            destination,
            rl.Vector2(0.0, 0.0),
            0.0,
            rl_color(WHITE),
        )
        return True

    def _draw_tpms_car_fallback(self) -> None:
        theme = self._current_theme()
        if theme.is_dark:
            body_fill = (0, 0, 0, 255)
            body_outline = (214, 224, 231, 237)
            glass_fill = (70, 190, 224, 220)
        else:
            body_fill = (245, 248, 252, 255)
            body_outline = (112, 124, 134, 237)
            glass_fill = (75, 185, 216, 220)
        wheel_fill = (5, 9, 12, 247)
        wheel_outline = (180, 192, 201, 235)

        car_x = TPMS_STATUS_CENTER_X - TPMS_STATUS_CAR_W * 0.5
        car_y = TPMS_STATUS_CAR_CENTER_Y - TPMS_STATUS_CAR_H * 0.5
        wheel_centers = (
            (
                TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
            ),
            (
                TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y - TPMS_STATUS_ROW_OFFSET,
            ),
            (
                TPMS_STATUS_CENTER_X - TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
            ),
            (
                TPMS_STATUS_CENTER_X + TPMS_STATUS_COLUMN_OFFSET,
                TPMS_STATUS_VALUE_CENTER_Y + TPMS_STATUS_ROW_OFFSET,
            ),
        )
        for wheel_center_x, wheel_center_y in wheel_centers:
            self._rounded_rect(
                wheel_center_x - TPMS_STATUS_WHEEL_W * 0.5,
                wheel_center_y - TPMS_STATUS_WHEEL_H * 0.5,
                TPMS_STATUS_WHEEL_W,
                TPMS_STATUS_WHEEL_H,
                7.0,
                wheel_fill,
                wheel_outline,
                1.4,
            )

        self._rounded_rect(
            car_x,
            car_y,
            TPMS_STATUS_CAR_W,
            TPMS_STATUS_CAR_H,
            6.5,
            body_fill,
            body_outline,
            2.0,
        )
        self._rounded_rect(
            car_x + 6.0,
            car_y + 10.0,
            TPMS_STATUS_CAR_W - 12.0,
            21.0,
            5.0,
            glass_fill,
            body_outline,
            1.2,
        )
        rl.draw_line_ex(
            rl.Vector2(car_x + 6.0, car_y + 56.0),
            rl.Vector2(car_x + TPMS_STATUS_CAR_W - 6.0, car_y + 56.0),
            1.5,
            rl_color(body_outline),
        )

    def _draw_compact_tpms_value(self, pressure: float | None, center_x: float, center_y: float) -> None:
        low = pressure is not None and pressure < TPMS_LOW_PRESSURE_PSI
        text = "--" if pressure is None else f"{pressure:.0f}"
        color = RED if low else WHITE
        if pressure is None:
            color = (170, 180, 188, 255)
        self._draw_text_with_stroke(
            text,
            center_x,
            center_y,
            TPMS_STATUS_FONT_SIZE,
            color,
            (0, 0, 0, 255),
            2,
            anchor="center",
        )

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
        source_marker = vehicle.source.startswith("modelV2") or vehicle.source in ("radarState", "radarPoint", "cornerRadar")
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

        def with_alpha(color: tuple[int, int, int] | tuple[int, int, int, int]) -> tuple[int, int, int, int]:
            return color[0], color[1], color[2], alpha

        marker_height_m = max(0.42, vehicle.height_m * 0.45)
        marker_base_z = max(0.0, vehicle.center.z - vehicle.height_m * 0.5)

        marker_vehicle = replace(
            vehicle,
            center=Vec3(vehicle.center.x, vehicle.center.y, marker_base_z + marker_height_m * 0.5),
            width_m=max(0.55, vehicle.width_m * 0.68),
            length_m=max(1.05, vehicle.length_m * 0.64),
            height_m=marker_height_m,
            body_color=with_alpha(vehicle.body_color),
            side_color=with_alpha(vehicle.side_color),
            rear_color=with_alpha(vehicle.rear_color),
            top_highlight=with_alpha(vehicle.top_highlight),
            outline_color=with_alpha(vehicle.outline_color),
        )
        self._draw_vehicle_box(marker_vehicle)

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
            distance = (
                radar_point_distance_label(point, self.is_metric)
                if radar_info_shows_distance(radar_info_mode)
                else ""
            )
            speed = (
                radar_point_speed_label(point, self.is_metric)
                if radar_info_shows_speed(radar_info_mode)
                else ""
            )
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
        show_vehicle_info = radar_info_shows_vehicle(radar_info_mode)
        if not show_vehicle_info and not any(vehicle.primary or vehicle.cut_in for vehicle in vehicles):
            return
        theme = self._current_theme()
        profile_enabled = self.profile_enabled
        profile_stage = self._profile_start()
        ordered = sorted(
            (
                vehicle
                for vehicle in vehicles
                if vehicle.label and (show_vehicle_info or vehicle.primary or vehicle.cut_in)
            ),
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
            show_important_label = vehicle.primary or vehicle.cut_in
            distance = (
                vehicle_distance_label(vehicle, self.is_metric)
                if radar_info_shows_distance(radar_info_mode) or show_important_label
                else ""
            )
            speed = (
                vehicle_speed_label(vehicle, self.is_metric)
                if radar_info_shows_speed(radar_info_mode)
                else ""
            )
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
        if vehicle.primary or vehicle.cut_in:
            self._draw_selected_vehicle_outline(vehicle, corner, half_width, half_length, z0, z1)

    def _draw_selected_vehicle_outline(
        self,
        vehicle: VehicleBox,
        corner,
        half_width: float,
        half_length: float,
        z0: float,
        z1: float,
    ) -> None:
        outline_color = AMBER if vehicle.cut_in else WHITE
        outline = rl_color(outline_color, 255)
        halo_width = half_width + 0.12
        halo_length = half_length + 0.16
        halo_z0 = z0 + 0.015
        halo_z1 = z1 + 0.065
        halo_base = (
            corner(-halo_width, -halo_length, halo_z0),
            corner(halo_width, -halo_length, halo_z0),
            corner(halo_width, halo_length, halo_z0),
            corner(-halo_width, halo_length, halo_z0),
        )
        halo_top = (
            corner(-halo_width, -halo_length, halo_z1),
            corner(halo_width, -halo_length, halo_z1),
            corner(halo_width, halo_length, halo_z1),
            corner(-halo_width, halo_length, halo_z1),
        )
        halo_edges = (
            (halo_top[0], halo_top[1]),
            (halo_top[1], halo_top[2]),
            (halo_top[2], halo_top[3]),
            (halo_top[3], halo_top[0]),
            (halo_base[0], halo_base[1]),
            (halo_base[1], halo_base[2]),
            (halo_base[2], halo_base[3]),
            (halo_base[3], halo_base[0]),
            (halo_base[0], halo_top[0]),
            (halo_base[1], halo_top[1]),
            (halo_base[2], halo_top[2]),
            (halo_base[3], halo_top[3]),
        )
        for start, end in halo_edges:
            rl.draw_line_3d(vec3(start), vec3(end), outline)

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
            screen_mode = self._effective_screen_mode(state)
            if screen_mode == CLUSTER_SCREEN_MODE_NAVI:
                self._draw_navi_dashboard(state)
                self._draw_status_footer(state)
                return
            navi_debug_active = state.navi_debug is not None
            navi_live_active = (
                self._navi_live_panel_visible(state.navi_live)
                or self._navi_map_frame_present(state.navi_dashboard)
                or (screen_mode == CLUSTER_SCREEN_MODE_DEFAULT and state.navi_dashboard is not None)
            )
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH:
                profile_stage = self._profile_start()
                self._draw_speed_block(state)
                self._profile_add("hud.speed_block", profile_stage)
                profile_stage = self._profile_start()
                self._draw_accel_block(state)
                self._profile_add("hud.accel_block", profile_stage)
                self._draw_steering_output_block(state)
                profile_stage = self._profile_start()
                self._draw_debug_plot(
                    state.debug_plot,
                    DEBUG_PLOT_FULL_X,
                    DEBUG_PLOT_FULL_Y,
                    DEBUG_PLOT_FULL_W,
                    DEBUG_PLOT_FULL_H,
                )
                self._profile_add("hud.debug_plot_full", profile_stage)
                self._draw_status_footer(state)
                return

            self._draw_driving_hud_content(
                state,
                screen_mode,
                left_signal_lit,
                right_signal_lit,
            )
            if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D:
                self._draw_status_footer(state)
                return
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG_SYSTEM:
                profile_stage = self._profile_start()
                self._draw_system_dashboard_panel(state)
                self._profile_add("hud.system_dashboard", profile_stage)
                self._draw_status_footer(state)
                return
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG:
                profile_stage = self._profile_start()
                self._draw_live_debug_panel(state)
                self._profile_add("hud.live_debug", profile_stage)
            if screen_mode == CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT:
                profile_stage = self._profile_start()
                self._draw_debug_plot(
                    state.debug_plot,
                    self._information_panel_x(DEBUG_PLOT_RIGHT_X),
                    DEBUG_PLOT_RIGHT_Y,
                    DEBUG_PLOT_RIGHT_W,
                    DEBUG_PLOT_RIGHT_H,
                )
                self._profile_add("hud.debug_plot_right", profile_stage)
            if screen_mode == CLUSTER_SCREEN_MODE_TRIP_REPORT:
                profile_stage = self._profile_start()
                self._draw_trip_report_panel(state)
                self._profile_add("hud.trip_report", profile_stage)
            elif navi_debug_active:
                profile_stage = self._profile_start()
                self._draw_navi_debug_panel(state.navi_debug)
                self._profile_add("hud.navi_debug", profile_stage)
            elif screen_mode == CLUSTER_SCREEN_MODE_DEFAULT and navi_live_active:
                profile_stage = self._profile_start()
                self._draw_navi_live_panel(state)
                self._profile_add("hud.navi_live", profile_stage)
            if screen_mode not in (
                CLUSTER_SCREEN_MODE_DEBUG,
                CLUSTER_SCREEN_MODE_DEBUG_GRAPH,
                CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
                CLUSTER_SCREEN_MODE_TRIP_REPORT,
            ) and not navi_debug_active and not navi_live_active:
                profile_stage = self._profile_start()
                self._draw_route_overlay(state.route_overlay)
                self._profile_add("hud.route_overlay", profile_stage)
            self._draw_status_footer(
                state,
                include_core_usage=screen_mode != CLUSTER_SCREEN_MODE_TRIP_REPORT,
            )
        finally:
            profile_stage = self._profile_start()
            rl.rl_pop_matrix()
            self._profile_add("hud.pop_matrix", profile_stage)

    def _draw_alert_overlay(self, alert: ClusterAlert | None) -> None:
        if alert is None or alert.size <= 0 or not (alert.text1 or alert.text2):
            return

        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        rl.rl_push_matrix()
        rl.rl_scalef(sx, sy, 1.0)
        try:
            panel_x = self._driving_panel_offset_design_x()
            panel_w = CAMERA_BACKGROUND_W
            panel_center_x = panel_x + panel_w * 0.5

            if alert.size >= CLUSTER_ALERT_SIZE_FULL:
                max_text_w = panel_w - 100.0
                title_size = 56.0
                detail_size = 32.0
            elif alert.size == CLUSTER_ALERT_SIZE_MID:
                max_text_w = panel_w - 128.0
                title_size = 48.0
                detail_size = 28.0
            else:
                max_text_w = panel_w - 200.0
                title_size = 42.0
                detail_size = 24.0

            title, detail = self._cluster_alert_text(alert)
            if not title and detail:
                title, detail = detail, ""
            title, title_size = self._fit_alert_text(title, title_size, max_text_w, 30.0)
            detail, detail_size = self._fit_alert_text(detail, detail_size, max_text_w, 20.0)
            center_y = DESIGN_HEIGHT * 0.5
            title_color = CLUSTER_ALERT_TEXT_COLORS.get(alert.status, CLUSTER_ALERT_TEXT_COLORS[0])
            if detail:
                title_y = center_y - (22.0 if alert.size == CLUSTER_ALERT_SIZE_SMALL else 30.0)
                detail_y = center_y + (27.0 if alert.size == CLUSTER_ALERT_SIZE_SMALL else 36.0)
                self._draw_text_with_stroke(
                    title, panel_center_x, title_y, title_size, title_color,
                    CLUSTER_ALERT_TEXT_STROKE, 4, anchor="center", cache=True,
                )
                self._draw_text_with_stroke(
                    detail, panel_center_x, detail_y, detail_size, WHITE,
                    CLUSTER_ALERT_TEXT_STROKE, 3, anchor="center", cache=True,
                )
            else:
                self._draw_text_with_stroke(
                    title, panel_center_x, center_y, title_size, title_color,
                    CLUSTER_ALERT_TEXT_STROKE, 4, anchor="center", cache=True,
                )
        finally:
            rl.rl_pop_matrix()

    def _cluster_alert_text(self, alert: ClusterAlert) -> tuple[str, str]:
        keys = CLUSTER_SYNTHETIC_ALERT_TEXT_KEYS.get(alert.alert_type)
        if keys is not None:
            return self._text(keys[0]), self._text(keys[1])
        return " ".join(alert.text1.split()), " ".join(alert.text2.split())

    def _fit_alert_text(
        self,
        text: str,
        preferred_size: float,
        max_width: float,
        minimum_size: float,
    ) -> tuple[str, float]:
        if not text:
            return "", preferred_size
        size = preferred_size
        while size > minimum_size and self._measure_text(text, size)[0] > max_width:
            size -= 2.0
        return self._ellipsize_text(text, size, max_width), size

    def _draw_driving_hud_content(
        self,
        state: ClusterUiState,
        screen_mode: int,
        left_signal_lit: bool,
        right_signal_lit: bool,
    ) -> None:
        offset_x = self._driving_hud_offset_design_x(screen_mode)
        tpms_offset_x = self._tpms_offset_design_x(screen_mode)
        side_gauge_offset_x = self._side_gauge_offset_design_x(screen_mode)
        rl.rl_push_matrix()
        if abs(offset_x) > 0.001:
            rl.rl_translatef(offset_x, 0.0, 0.0)
        try:
            profile_stage = self._profile_start()
            self._draw_speed_block(state, tpms_offset_x=tpms_offset_x)
            self._profile_add("hud.speed_block", profile_stage)
            side_gauges_translated = abs(side_gauge_offset_x) > 0.001
            if side_gauges_translated:
                rl.rl_push_matrix()
                rl.rl_translatef(side_gauge_offset_x, 0.0, 0.0)
            try:
                profile_stage = self._profile_start()
                self._draw_accel_block(state)
                self._profile_add("hud.accel_block", profile_stage)
                self._draw_steering_output_block(state)
            finally:
                if side_gauges_translated:
                    rl.rl_pop_matrix()
            profile_stage = self._profile_start()
            self._draw_turn_signal(
                "left",
                left_signal_lit,
                show_inactive=state.debug_ui_visible,
                center_x_offset=self._turn_signal_center_x_offset(state, "left"),
            )
            self._profile_add("hud.turn_signal_left", profile_stage)
            profile_stage = self._profile_start()
            self._draw_drive_status(state)
            self._profile_add("hud.drive_status", profile_stage)
            profile_stage = self._profile_start()
            self._draw_turn_signal(
                "right",
                right_signal_lit,
                show_inactive=state.debug_ui_visible,
                center_x_offset=self._turn_signal_center_x_offset(state, "right"),
            )
            self._profile_add("hud.turn_signal_right", profile_stage)
            traffic_drawn_in_map = screen_mode == CLUSTER_SCREEN_MODE_DEFAULT and self._navi_map_frame_present(
                state.navi_dashboard
            )
            dashboard = state.navi_dashboard
            traffic_frame = next(
                (frame for frame in dashboard.media if frame.key == "image:traffic_signal"),
                None,
            ) if dashboard is not None and dashboard.connected else None
            if (
                traffic_frame is not None
                and not traffic_drawn_in_map
                and screen_mode != CLUSTER_SCREEN_MODE_TRIP_REPORT
            ):
                profile_stage = self._profile_start()
                traffic_panel_right = self._traffic_panel_right(screen_mode)
                self._draw_navi_media(
                    traffic_frame,
                    rl.Rectangle(traffic_panel_right - 230.0, NAVI_TRAFFIC_PANEL_Y, 230.0, 98.0),
                    align_x=1.0,
                    align_y=0.0,
                )
                self._profile_add("hud.navi_traffic", profile_stage)
            profile_stage = self._profile_start()
            self._draw_center_clock(state, center_x=self._center_clock_x(screen_mode))
            self._profile_add("hud.center_clock", profile_stage)
        finally:
            rl.rl_pop_matrix()

    def _effective_screen_mode(self, state: ClusterUiState) -> int:
        requested_screen_mode = getattr(self, "screen_mode", CLUSTER_SCREEN_MODE_DEFAULT)
        if (
            requested_screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D
            and cluster_camera_view_is_road_camera(getattr(state, "camera_view_mode", 0))
        ):
            requested_screen_mode = CLUSTER_SCREEN_MODE_DEFAULT
        if requested_screen_mode != CLUSTER_SCREEN_MODE_DEFAULT:
            return requested_screen_mode
        gear_text = str(getattr(state, "gear_text", "") or "").strip().upper()
        if getattr(state, "onroad", False) and gear_text == "P":
            # Surface the completed trip as soon as the driver parks, even
            # when an active navigation session would otherwise own the panel.
            return CLUSTER_SCREEN_MODE_TRIP_REPORT
        dashboard = getattr(state, "navi_dashboard", None)
        dashboard_connected = bool(
            dashboard is not None and dashboard.connected
        )
        navigation_received = bool(
            getattr(state, "external_nav_active", False)
            or dashboard_connected
            or self._navi_live_panel_visible(getattr(state, "navi_live", None))
        )
        return (
            CLUSTER_SCREEN_MODE_DEFAULT
            if navigation_received
            else CLUSTER_SCREEN_MODE_TRIP_REPORT
        )

    def _draw_status_footer(
        self,
        state: ClusterUiState,
        *,
        include_core_usage: bool = True,
    ) -> None:
        profile_stage = self._profile_start()
        screen_mode = self._effective_screen_mode(state)
        offset_x = self._driving_hud_offset_design_x(screen_mode)
        if abs(offset_x) > 0.001:
            rl.rl_push_matrix()
            try:
                rl.rl_translatef(offset_x, 0.0, 0.0)
                self._draw_git_status(state.git_status, state.network_address, state.actual_fps)
            finally:
                rl.rl_pop_matrix()
        else:
            self._draw_git_status(state.git_status, state.network_address, state.actual_fps)
        self._profile_add("hud.git_status", profile_stage)
        if include_core_usage:
            profile_stage = self._profile_start()
            self._draw_cluster_core_usage(
                state.cluster_core_usage_text,
                right_x=self._core_usage_right_x(screen_mode),
            )
            self._profile_add("hud.cluster_core_usage", profile_stage)

    def _draw_route_replay_controls(
        self,
        playback_s: float,
        duration_s: float,
        corner_lateral_offset_m: float,
        paused: bool,
        camera_overlay_tuning_visible: bool = False,
    ) -> None:
        sx = self.width / DESIGN_WIDTH
        sy = self.height / DESIGN_HEIGHT
        rl.rl_push_matrix()
        rl.rl_scalef(sx, sy, 1.0)
        try:
            theme = self._current_theme()
            panel = rl.Rectangle(
                ROUTE_CONTROL_PANEL_X,
                ROUTE_CONTROL_PANEL_Y,
                ROUTE_CONTROL_PANEL_W,
                ROUTE_CONTROL_PANEL_H,
            )
            rl.draw_rectangle_rounded(panel, 0.20, 12, rl_color((2, 5, 10, 188)))
            rl.draw_rectangle_rounded_lines_ex(panel, 0.20, 12, 1.4, rl_color((255, 255, 255, 54)))

            duration_s = max(0.001, duration_s)
            playback_s = clamp(playback_s, 0.0, duration_s)
            seek_ratio = playback_s / duration_s
            self._draw_route_slider(
                "seek",
                f"{self._format_time(playback_s)} / {self._format_time(duration_s)}{' PAUSED' if paused else ''}",
                seek_ratio,
                ROUTE_CONTROL_SEEK_Y,
                BLUE_SOFT,
                theme.text,
            )
            if camera_overlay_tuning_visible:
                pitch_ratio = (
                    (self.camera_overlay_pitch_offset_deg - CAMERA_OVERLAY_PITCH_OFFSET_MIN_DEG)
                    / max(0.001, CAMERA_OVERLAY_PITCH_OFFSET_MAX_DEG - CAMERA_OVERLAY_PITCH_OFFSET_MIN_DEG)
                )
                self._draw_route_slider(
                    "pitch",
                    f"{self.camera_overlay_pitch_offset_deg:+.1f} deg   ; / '",
                    pitch_ratio,
                    CAMERA_OVERLAY_PITCH_TUNE_PANEL_Y,
                    RED,
                    theme.text,
                )
                z_ratio = (
                    (self.camera_overlay_z_offset_m - CAMERA_OVERLAY_Z_OFFSET_MIN_M)
                    / max(0.001, CAMERA_OVERLAY_Z_OFFSET_MAX_M - CAMERA_OVERLAY_Z_OFFSET_MIN_M)
                )
                self._draw_route_slider(
                    "cam z",
                    f"{self.camera_overlay_z_offset_m:+.2f} m   [ / ]",
                    z_ratio,
                    CAMERA_OVERLAY_TUNE_PANEL_Y,
                    AMBER,
                    theme.text,
                )
        finally:
            rl.rl_pop_matrix()

    def _draw_route_slider(
        self,
        label: str,
        value: str,
        ratio: float,
        center_y: float,
        fill_color: tuple[int, int, int],
        text_color: tuple[int, int, int],
    ) -> None:
        ratio = clamp(ratio, 0.0, 1.0)
        self._draw_text(label, ROUTE_CONTROL_PANEL_X + 26.0, center_y - 9.0, 16, text_color)
        self._draw_text(value, ROUTE_CONTROL_PANEL_X + ROUTE_CONTROL_PANEL_W - 26.0, center_y - 9.0, 16, text_color, anchor="right")
        bar_bg = rl.Rectangle(ROUTE_CONTROL_BAR_X, center_y - 3.0, ROUTE_CONTROL_BAR_W, 6.0)
        bar_fill = rl.Rectangle(ROUTE_CONTROL_BAR_X, center_y - 3.0, ROUTE_CONTROL_BAR_W * ratio, 6.0)
        knob_x = ROUTE_CONTROL_BAR_X + ROUTE_CONTROL_BAR_W * ratio
        rl.draw_rectangle_rounded(bar_bg, 1.0, 8, rl_color((255, 255, 255, 50)))
        rl.draw_rectangle_rounded(bar_fill, 1.0, 8, rl_color(fill_color, 205))
        rl.draw_circle_v(rl.Vector2(knob_x, center_y), 8.5, rl_color(fill_color, 235))
        rl.draw_circle_lines(int(round(knob_x)), int(round(center_y)), 9.5, rl_color((255, 255, 255, 150)))

    @staticmethod
    def _format_time(seconds: float) -> str:
        total = max(0, int(round(seconds)))
        minutes = total // 60
        secs = total % 60
        return f"{minutes:d}:{secs:02d}"

    def _draw_center_clock(self, state: ClusterUiState, *, center_x: float = TOP_CLOCK_CENTER_X) -> None:
        if state.center_clock_text:
            self._draw_text_with_stroke(
                state.center_clock_text,
                center_x,
                TOP_STATUS_CENTER_Y,
                48,
                WHITE,
                (10, 13, 16),
                2,
                anchor="center",
            )
        navi_connected = bool(state.navi_dashboard is not None and state.navi_dashboard.connected)
        navi_status = navigation_status_presentation(
            getattr(state, "vehicle_navi_available", False),
            state.external_nav_active or navi_connected,
        )
        if navi_status is not None:
            navi_label, navi_color_mode = navi_status
            self._draw_text_with_stroke(
                navi_label,
                NAV_STATUS_CENTER_X,
                NAV_STATUS_CENTER_Y,
                NAV_STATUS_FONT_SIZE,
                VEHICLE_NAVI if navi_color_mode == 3 else AMBER,
                (10, 13, 16),
                2,
                anchor="center",
                cache=True,
            )

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

    def _draw_navi_dashboard(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        dashboard = state.navi_dashboard
        navi = state.navi_live
        rl.draw_rectangle(0, 0, int(NAVI_MODE_LEFT_W), DESIGN_HEIGHT, rl_color(theme.panel_bg))
        rl.draw_rectangle(
            int(NAVI_MODE_RIGHT_X),
            0,
            int(NAVI_MODE_RIGHT_W),
            DESIGN_HEIGHT,
            rl_color(theme.panel_bg),
        )
        rl.draw_line_ex(
            rl.Vector2(NAVI_MODE_LEFT_W, 0.0),
            rl.Vector2(NAVI_MODE_LEFT_W, DESIGN_HEIGHT),
            2.0,
            rl_color(theme.faint),
        )
        rl.draw_line_ex(
            rl.Vector2(NAVI_MODE_RIGHT_X, 0.0),
            rl.Vector2(NAVI_MODE_RIGHT_X, DESIGN_HEIGHT),
            2.0,
            rl_color(theme.faint),
        )

        media = {frame.key: frame for frame in dashboard.media} if dashboard is not None else {}
        map_rect = rl.Rectangle(NAVI_MODE_MAP_X, 0.0, NAVI_MODE_MAP_W, DESIGN_HEIGHT)
        map_frame = media.get("render:map_main")
        rl.draw_rectangle_rec(map_rect, rl_color(NAVI_MAP_BACKGROUND))
        if not self._draw_navi_media(map_frame, map_rect, cover=False):
            self._draw_navi_route_fallback(navi, map_rect)
        if dashboard is not None and dashboard.map_stream_stalled:
            self._draw_navi_map_stalled(map_rect, dashboard.map_frame_age_ms)

        self._draw_navi_map_media(media)
        self._draw_navi_left_band(state, navi, media)
        self._draw_navi_right_band(navi, dashboard, media)

    def _draw_navi_left_band(
        self,
        state: ClusterUiState,
        navi: NaviLiveState | None,
        media: dict[str, NaviMediaFrame],
    ) -> None:
        theme = self._current_theme()
        full_rect = rl.Rectangle(10.0, 8.0, 500.0, 88.0)
        compact_rect = rl.Rectangle(10.0, 104.0, 330.0, 145.0)
        next_rect = rl.Rectangle(350.0, 104.0, 160.0, 72.0)
        drew_full = self._draw_navi_media(media.get("image:tbt_current_full"), full_rect)
        drew_compact = self._draw_navi_media(media.get("image:tbt_current_compact"), compact_rect)
        drew_next = self._draw_navi_media(media.get("image:tbt_next"), next_rect)

        current = navi.current if navi is not None else None
        if current is not None and not (drew_full or drew_compact):
            self._draw_navi_turn_icon(current.turn_type, 72.0, 78.0, 92.0)
            self._draw_text(self._format_navi_distance(current.distance_m), 136.0, 30.0, 42.0, theme.text)
            current_text = current.main_text or current.road_name or current.near_direction
            self._draw_text(
                self._ellipsize_text(current_text, 26.0, 350.0),
                136.0,
                82.0,
                26.0,
                theme.text,
            )
        next_guidance = navi.next if navi is not None else None
        if next_guidance is not None and not drew_next:
            self._draw_navi_turn_icon(next_guidance.turn_type, 386.0, 143.0, 54.0)
            self._draw_text(
                self._format_navi_distance(next_guidance.distance_m),
                422.0,
                119.0,
                20.0,
                theme.text,
            )

        displayed_speed = display_speed(state.speed_kph, self.is_metric)
        road_limit = state.speed_limit_kph
        road_name = ""
        if navi is not None:
            if navi.vehicle is not None:
                road_name = navi.vehicle.road_name
            if road_limit is None and navi.speed is not None:
                road_limit = navi.speed.road_limit_kph
        self._draw_text(f"{displayed_speed:.0f}", 32.0, 284.0, 82.0, theme.text)
        self._draw_text(speed_unit(self.is_metric), 143.0, 332.0, 18.0, theme.muted)
        if road_limit is not None:
            rl.draw_circle_v(rl.Vector2(218.0, 325.0), 36.0, rl_color(WHITE))
            rl.draw_ring(rl.Vector2(218.0, 325.0), 30.0, 36.0, 0.0, 360.0, 48, rl_color(RED))
            self._draw_text(
                str(int(round(display_speed(road_limit, self.is_metric)))),
                218.0,
                308.0,
                27.0,
                (20, 24, 28),
                anchor="center",
            )
        if road_name:
            self._draw_text(
                self._ellipsize_text(road_name, 22.0, 480.0),
                20.0,
                389.0,
                22.0,
                theme.text,
            )
        if navi is not None and navi.route is not None:
            route = navi.route
            route_text = self._format_navi_distance(route.remaining_distance_m)
            if route.remaining_time_s > 0:
                route_text += f"  {max(1, round(route.remaining_time_s / 60.0))} min"
            self._draw_text(route_text, 20.0, 431.0, 24.0, BLUE_SOFT)

    def _draw_navi_map_media(self, media: dict[str, NaviMediaFrame]) -> None:
        expanded = media.get("image:crossroad_expanded")
        minimized = media.get("image:crossroad_minimized")
        if expanded is not None and expanded.present:
            self._draw_navi_media(expanded, rl.Rectangle(NAVI_MODE_MAP_X + 55.0, 20.0, 770.0, 430.0))
        elif minimized is not None and minimized.present:
            self._draw_navi_media(
                minimized,
                rl.Rectangle(NAVI_MODE_MAP_X + NAVI_MODE_MAP_W - 260.0, 48.0, 250.0, 410.0),
            )

        self._draw_navi_media(
            media.get("image:lane_bottom"),
            rl.Rectangle(NAVI_MODE_MAP_X + 80.0, 298.0, NAVI_MODE_MAP_W - 160.0, 174.0),
        )
        self._draw_navi_media(
            media.get("image:traffic_signal"),
            rl.Rectangle(NAVI_MODE_MAP_X + 16.0, 18.0, 230.0, 98.0),
        )

        center_x = NAVI_MODE_MAP_X + NAVI_MODE_MAP_W * 0.5
        self._draw_navi_media(
            media.get("image:center_tbt_icon"),
            rl.Rectangle(center_x - 112.0, 104.0, 92.0, 92.0),
        )
        self._draw_navi_media(
            media.get("image:center_tbt_text"),
            rl.Rectangle(center_x - 12.0, 108.0, 220.0, 78.0),
        )
        self._draw_navi_media(
            media.get("image:center_tbt_fee"),
            rl.Rectangle(center_x - 90.0, 188.0, 180.0, 58.0),
        )

    def _draw_navi_right_band(
        self,
        navi: NaviLiveState | None,
        dashboard: NaviDashboardState | None,
        media: dict[str, NaviMediaFrame],
    ) -> None:
        theme = self._current_theme()
        x = NAVI_MODE_RIGHT_X + 18.0
        width = NAVI_MODE_RIGHT_W - 36.0
        current = navi.current if navi is not None else None
        next_guidance = navi.next if navi is not None else None
        self._draw_text(self._text("navigation"), x, 14.0, 16.0, theme.muted)
        if current is not None:
            current_text = current.main_text or current.road_name or current.near_direction
            self._draw_text(
                self._ellipsize_text(current_text, 27.0, width - 120.0),
                x,
                42.0,
                27.0,
                theme.text,
            )
            self._draw_text(
                self._format_navi_distance(current.distance_m),
                NAVI_MODE_RIGHT_X + NAVI_MODE_RIGHT_W - 18.0,
                42.0,
                27.0,
                BLUE_SOFT,
                anchor="right",
            )
        if next_guidance is not None:
            next_text = next_guidance.main_text or next_guidance.road_name or next_guidance.near_direction
            next_line = f"{self._text('next')} {self._format_navi_distance(next_guidance.distance_m)}  {next_text}"
            self._draw_text(self._ellipsize_text(next_line, 18.0, width), x, 82.0, 18.0, theme.muted)

        y = 116.0
        safety_frames = [
            media.get("image:safety_primary"),
            media.get("image:safety_secondary"),
            media.get("image:safety_section"),
        ]
        active_safety = [frame for frame in safety_frames if frame is not None and frame.present]
        if active_safety:
            slot_h = min(82.0, 176.0 / len(active_safety))
            for frame in active_safety:
                self._draw_navi_media(frame, rl.Rectangle(x, y, width, slot_h - 4.0))
                y += slot_h
        elif navi is not None and navi.speed is not None:
            speed = navi.speed
            sdi_values = (
                ("SDI", speed.sdi_type, speed.sdi_speed_limit_kph, speed.sdi_distance_m),
                ("SDI 2", speed.secondary_sdi_type, speed.secondary_sdi_speed_limit_kph,
                 speed.secondary_sdi_distance_m),
            )
            for label, sdi_type, speed_limit_kph, distance_m in sdi_values:
                if sdi_type is None:
                    continue
                sdi = label
                if speed_limit_kph is not None:
                    sdi += f" {display_speed(speed_limit_kph, self.is_metric):.0f} {speed_unit(self.is_metric)}"
                if distance_m is not None:
                    sdi += f"  {self._format_navi_distance(distance_m)}"
                self._draw_text(sdi, x, y + 6.0, 24.0, AMBER)
                y += 40.0
            if speed.section_active:
                section = self._text("section")
                if speed.section_average_kph is not None:
                    section += f" {self._text('average')} {display_speed(speed.section_average_kph, self.is_metric):.0f}"
                if speed.section_remaining_distance_m is not None:
                    section += f"  {self._format_navi_distance(speed.section_remaining_distance_m)}"
                self._draw_text(self._ellipsize_text(section, 20.0, width), x, y + 4.0, 20.0, theme.text)
                y += 34.0

        route_y = max(292.0, y + 8.0)
        if navi is not None and navi.route is not None:
            route = navi.route
            total = max(1, route.total_distance_m)
            progress = clamp(route.moved_distance_m / total, 0.0, 1.0)
            self._draw_text(self._text("route"), x, route_y, 15.0, theme.muted)
            route_value = self._format_navi_distance(route.remaining_distance_m)
            if route.remaining_time_s > 0:
                route_value += f" / {max(1, round(route.remaining_time_s / 60.0))} min"
            self._draw_text(route_value, x + width, route_y - 2.0, 20.0, theme.text, anchor="right")
            bar_y = route_y + 28.0
            rl.draw_rectangle_rounded(rl.Rectangle(x, bar_y, width, 8.0), 1.0, 8, rl_color(theme.faint))
            if progress > 0.0:
                rl.draw_rectangle_rounded(
                    rl.Rectangle(x, bar_y, max(8.0, width * progress), 8.0),
                    1.0,
                    8,
                    rl_color(BLUE_SOFT),
                )

        status_y = 358.0
        if dashboard is None:
            self._draw_text(f"TCP 7714 {self._text('waiting')}", x, status_y, 18.0, AMBER)
            return
        status_color = GREEN if dashboard.connected and dashboard.error is None else RED if dashboard.error else AMBER
        status_text = self._text("connected") if dashboard.connected else self._text("waiting")
        self._draw_text(status_text, x, status_y, 18.0, status_color)
        self._draw_text(
            f"{dashboard.app_version or '-'}  rev {dashboard.manifest_revision}  rx {dashboard.received_count}",
            x + width,
            status_y,
            15.0,
            theme.muted,
            anchor="right",
        )
        self._draw_text(self._ellipsize_text(dashboard.app_status, 14.0, width), x, status_y + 27.0, 14.0, theme.text)
        self._draw_text(self._ellipsize_text(dashboard.camera_status, 14.0, width), x, status_y + 49.0, 14.0, theme.text)
        self._draw_text(
            self._ellipsize_text(dashboard.composition_status, 14.0, width),
            x,
            status_y + 71.0,
            14.0,
            theme.text,
        )
        present = sum(1 for item in dashboard.items if item.present)
        age = "-" if dashboard.last_received_age_ms is None else f"{dashboard.last_received_age_ms} ms"
        footer = f"STREAMS {present}/28 VALUE  |  AGE {age}  |  {dashboard.peer}"
        self._draw_text(self._ellipsize_text(footer, 13.0, width), x, 458.0, 13.0, theme.muted)
        if dashboard.error:
            self._draw_text(self._ellipsize_text(dashboard.error, 13.0, width), x, 436.0, 13.0, RED)

    def _draw_navi_route_fallback(self, navi: NaviLiveState | None, rect: rl.Rectangle) -> None:
        theme = self._current_theme()
        rl.draw_rectangle_rec(rect, rl_color((9, 13, 18)))
        route = navi.route if navi is not None else None
        points = route.polyline if route is not None else ()
        if len(points) >= 2:
            latitudes = [point[0] for point in points]
            longitudes = [point[1] for point in points]
            min_lat, max_lat = min(latitudes), max(latitudes)
            min_lon, max_lon = min(longitudes), max(longitudes)
            lat_span = max(0.00001, max_lat - min_lat)
            lon_span = max(0.00001, max_lon - min_lon)

            def project(latitude: float, longitude: float) -> rl.Vector2:
                px = rect.x + 42.0 + (longitude - min_lon) / lon_span * (rect.width - 84.0)
                py = rect.y + rect.height - 42.0 - (latitude - min_lat) / lat_span * (rect.height - 84.0)
                return rl.Vector2(px, py)

            previous = project(*points[0])
            for point in points[1:]:
                current = project(*point)
                rl.draw_line_ex(previous, current, 7.0, rl_color((21, 58, 96)))
                rl.draw_line_ex(previous, current, 3.0, rl_color(BLUE_SOFT))
                previous = current
            if navi is not None and navi.vehicle is not None:
                vehicle = project(navi.vehicle.latitude, navi.vehicle.longitude)
                rl.draw_circle_v(vehicle, 11.0, rl_color(WHITE))
                rl.draw_circle_v(vehicle, 7.0, rl_color(BLUE))
        self._draw_text("MAP STREAM WAITING", rect.x + rect.width * 0.5, 224.0, 18.0, theme.muted, anchor="center")

    def _draw_navi_media(
        self,
        frame: NaviMediaFrame | None,
        rect: rl.Rectangle,
        *,
        cover: bool = False,
        align_x: float = 0.5,
        align_y: float = 0.5,
    ) -> bool:
        texture = self._navi_media_texture_for(frame)
        cached = self._navi_media_textures.get(frame.key) if frame is not None else None
        if texture is None or cached is None or cached.size[0] <= 0 or cached.size[1] <= 0:
            return False
        source_w = float(cached.size[0])
        source_h = float(cached.size[1])
        scale = max(rect.width / source_w, rect.height / source_h) if cover else min(
            rect.width / source_w,
            rect.height / source_h,
        )
        draw_w = source_w * scale
        draw_h = source_h * scale
        source = rl.Rectangle(0.0, 0.0, source_w, source_h)
        dest = rl.Rectangle(
            rect.x + (rect.width - draw_w) * clamp(align_x, 0.0, 1.0),
            rect.y + (rect.height - draw_h) * clamp(align_y, 0.0, 1.0),
            draw_w,
            draw_h,
        )
        self._draw_cached_navi_media(cached, source, dest)
        return True

    def _navi_media_fitted_size(
        self,
        frame: NaviMediaFrame | None,
        rect: rl.Rectangle,
        *,
        cover: bool = False,
    ) -> tuple[float, float] | None:
        texture = self._navi_media_texture_for(frame)
        cached = self._navi_media_textures.get(frame.key) if frame is not None else None
        if texture is None or cached is None or cached.size[0] <= 0 or cached.size[1] <= 0:
            return None
        source_w = float(cached.size[0])
        source_h = float(cached.size[1])
        scale = max(rect.width / source_w, rect.height / source_h) if cover else min(
            rect.width / source_w,
            rect.height / source_h,
        )
        return source_w * scale, source_h * scale

    def _draw_cached_navi_media(
        self,
        cached: CachedNaviMediaTexture,
        source: rl.Rectangle,
        dest: rl.Rectangle,
    ) -> None:
        if cached.mime == "video/nv12-dmabuf":
            token = cached.hardware_token
            egl_image = self._navi_egl_images.get(token) if token is not None else None
            if egl_image is None:
                return
            from openpilot.system.ui.lib.egl import bind_egl_image_to_texture

            shader = self._get_navi_external_shader()
            rl.begin_shader_mode(shader)
            try:
                # GL_TEXTURE_EXTERNAL_OES is texture-unit state and raylib only
                # restores GL_TEXTURE_2D during a batch draw. Rebind on every
                # draw so the road camera cannot remain selected on texture0.
                bind_egl_image_to_texture(cached.texture.id, egl_image)
                rl.draw_texture_pro(cached.texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))
            finally:
                rl.end_shader_mode()
            return
        if cached.mime != "image/yuv420p":
            rl.draw_texture_pro(cached.texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))
            return
        if cached.u_texture is None or cached.v_texture is None:
            return
        shader = self._get_navi_yuv_shader()
        locations = self._navi_yuv_shader_locations
        rl.begin_shader_mode(shader)
        try:
            rl.set_shader_value_texture(shader, locations["texture1"], cached.u_texture)
            rl.set_shader_value_texture(shader, locations["texture2"], cached.v_texture)
            rl.set_shader_value(
                shader,
                locations["uvScaleU"],
                cached.uv_scale_u,
                rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2,
            )
            rl.set_shader_value(
                shader,
                locations["uvScaleV"],
                cached.uv_scale_v,
                rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2,
            )
            rl.draw_texture_pro(cached.texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))
        finally:
            rl.end_shader_mode()

    def _get_navi_external_shader(self):
        if self._navi_external_shader is None:
            self._navi_external_shader = rl.load_shader_from_memory(
                NAVI_EXTERNAL_VERTEX_SHADER,
                NAVI_EXTERNAL_FRAGMENT_SHADER,
            )
            if not rl.is_shader_valid(self._navi_external_shader):
                rl.unload_shader(self._navi_external_shader)
                self._navi_external_shader = None
                raise RuntimeError("failed to load Navi external NV12 shader")
        return self._navi_external_shader

    def _get_navi_yuv_shader(self):
        if self._navi_yuv_shader is None:
            self._navi_yuv_shader = rl.load_shader_from_memory(
                NV12_PACK_VERTEX_SHADER,
                NAVI_YUV420_FRAGMENT_SHADER,
            )
            if not rl.is_shader_valid(self._navi_yuv_shader):
                rl.unload_shader(self._navi_yuv_shader)
                self._navi_yuv_shader = None
                raise RuntimeError("failed to load Navi YUV420 shader")
            self._navi_yuv_shader_locations = {
                name: rl.get_shader_location(self._navi_yuv_shader, name)
                for name in ("texture1", "texture2", "uvScaleU", "uvScaleV")
            }
        return self._navi_yuv_shader

    @staticmethod
    def _load_navi_plane_texture(width: int, height: int, texture_filter):
        image = rl.Image(None, int(width), int(height), 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_GRAYSCALE)
        texture = rl.load_texture_from_image(image)
        if not rl.is_texture_valid(texture):
            raise RuntimeError(f"failed to allocate Navi YUV plane texture {width}x{height}")
        rl.set_texture_filter(texture, texture_filter)
        return texture

    @staticmethod
    def _unload_navi_media_texture(cached: CachedNaviMediaTexture) -> None:
        for texture in (cached.texture, cached.u_texture, cached.v_texture):
            if texture is not None:
                rl.unload_texture(texture)

    def _evict_navi_egl_images(self) -> None:
        if len(self._navi_egl_images) <= NAVI_EGL_IMAGE_CACHE_LIMIT:
            return
        from openpilot.system.ui.lib.egl import destroy_egl_image

        active_tokens = {
            cached.hardware_token
            for cached in self._navi_media_textures.values()
            if cached.hardware_token is not None
        }
        for token in tuple(self._navi_egl_images):
            if len(self._navi_egl_images) <= NAVI_EGL_IMAGE_CACHE_LIMIT:
                break
            if token in active_tokens:
                continue
            destroy_egl_image(self._navi_egl_images.pop(token))

    def _discard_stale_navi_egl_generation(self, generation: int) -> None:
        stale_tokens = tuple(token for token in self._navi_egl_images if token[0] != generation)
        if not stale_tokens:
            return
        from openpilot.system.ui.lib.egl import destroy_egl_image

        for token in stale_tokens:
            destroy_egl_image(self._navi_egl_images.pop(token))

    def _upload_navi_hardware_buffer(
        self,
        frame: NaviMediaFrame,
        cached: CachedNaviMediaTexture | None,
    ):
        hardware_buffer = frame.hardware_buffer
        if hardware_buffer is None:
            return cached.texture if cached is not None else None
        width = int(frame.width)
        height = int(frame.height)
        stride = int(getattr(hardware_buffer, "stride", 0))
        uv_offset = int(getattr(hardware_buffer, "uv_offset", 0))
        fd = int(getattr(hardware_buffer, "fd", -1))
        token = tuple(getattr(hardware_buffer, "token", ()))
        if (
            width <= 0
            or height <= 0
            or stride < width
            or uv_offset < stride * height
            or fd < 0
            or len(token) != 2
        ):
            return cached.texture if cached is not None else None

        profile_stage = self._profile_start()
        try:
            from openpilot.system.ui.lib.egl import bind_egl_image_to_texture, create_egl_image, init_egl

            if not init_egl():
                raise RuntimeError("EGL DMA-BUF import is unavailable")
            self._discard_stale_navi_egl_generation(int(token[0]))
            egl_image = self._navi_egl_images.get(token)
            if egl_image is None:
                egl_image = create_egl_image(width, height, stride, fd, uv_offset)
                if egl_image is None:
                    raise RuntimeError("EGL rejected the decoded NV12 DMA-BUF")
                self._navi_egl_images[token] = egl_image
            else:
                self._navi_egl_images.move_to_end(token)

            reuse = bool(
                cached is not None
                and cached.mime == frame.mime
                and cached.size == (width, height)
                and cached.u_texture is None
                and cached.v_texture is None
            )
            texture = cached.texture if reuse else None
            if texture is None:
                image = rl.gen_image_color(1, 1, rl.BLACK)
                try:
                    texture = rl.load_texture_from_image(image)
                finally:
                    rl.unload_image(image)
                if not rl.is_texture_valid(texture):
                    raise RuntimeError("failed to allocate Navi external texture")
                rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            texture.width = width
            texture.height = height
            bind_egl_image_to_texture(texture.id, egl_image)
        except Exception as exc:
            if "texture" in locals() and texture is not None and (cached is None or texture is not cached.texture):
                rl.unload_texture(texture)
            mark_failed = getattr(hardware_buffer, "mark_egl_import_failed", None)
            if mark_failed is not None:
                mark_failed(str(exc))
            print(
                f"Navi hardware H264 EGL import failed token={token}; requesting PyAV fallback: {exc}",
                flush=True,
            )
            return cached.texture if cached is not None else None

        replacement = CachedNaviMediaTexture(
            sequence=frame.sequence,
            size=(width, height),
            mime=frame.mime,
            texture=texture,
            hardware_token=(int(token[0]), int(token[1])),
        )
        if cached is not None and cached.texture is not texture:
            self._unload_navi_media_texture(cached)
        self._navi_media_textures[frame.key] = replacement
        self._evict_navi_egl_images()
        self._profile_add("navi_media.bind_nv12_dmabuf", profile_stage)
        return texture

    def _upload_navi_yuv420p(
        self,
        frame: NaviMediaFrame,
        cached: CachedNaviMediaTexture | None,
    ):
        if frame.plane_data is None or frame.plane_strides is None:
            return cached.texture if cached is not None else None
        width = int(frame.width)
        height = int(frame.height)
        if width <= 0 or height <= 0 or (width & 1) != 0 or (height & 1) != 0:
            return cached.texture if cached is not None else None
        y_stride, u_stride, v_stride = (int(value) for value in frame.plane_strides)
        chroma_width = width // 2
        chroma_height = height // 2
        if y_stride < width or u_stride < chroma_width or v_stride < chroma_width:
            return cached.texture if cached is not None else None
        plane_shapes = (
            (y_stride, height),
            (u_stride, chroma_height),
            (v_stride, chroma_height),
        )
        if any(
            len(data) < stride * plane_height
            for data, (stride, plane_height) in zip(frame.plane_data, plane_shapes, strict=True)
        ):
            return cached.texture if cached is not None else None

        reuse = bool(
            cached is not None
            and cached.mime == frame.mime
            and cached.size == (width, height)
            and cached.texture.width == y_stride
            and cached.texture.height == height
            and cached.u_texture is not None
            and cached.u_texture.width == u_stride
            and cached.v_texture is not None
            and cached.v_texture.width == v_stride
        )
        replacement = cached
        if not reuse:
            created: list[object] = []
            try:
                y_texture = self._load_navi_plane_texture(
                    y_stride,
                    height,
                    rl.TextureFilter.TEXTURE_FILTER_BILINEAR,
                )
                created.append(y_texture)
                u_texture = self._load_navi_plane_texture(
                    u_stride,
                    chroma_height,
                    rl.TextureFilter.TEXTURE_FILTER_POINT,
                )
                created.append(u_texture)
                v_texture = self._load_navi_plane_texture(
                    v_stride,
                    chroma_height,
                    rl.TextureFilter.TEXTURE_FILTER_POINT,
                )
                created.append(v_texture)
                replacement = CachedNaviMediaTexture(
                    sequence=frame.sequence,
                    size=(width, height),
                    mime=frame.mime,
                    texture=y_texture,
                    u_texture=u_texture,
                    v_texture=v_texture,
                    uv_scale_u=rl.ffi.new("float[]", [y_stride / (2.0 * u_stride), 1.0]),
                    uv_scale_v=rl.ffi.new("float[]", [y_stride / (2.0 * v_stride), 1.0]),
                )
            except Exception:
                for texture in created:
                    rl.unload_texture(texture)
                return cached.texture if cached is not None else None

        if replacement is None or replacement.u_texture is None or replacement.v_texture is None:
            return cached.texture if cached is not None else None
        profile_stage = self._profile_start()
        try:
            for texture, plane in zip(
                (replacement.texture, replacement.u_texture, replacement.v_texture),
                frame.plane_data,
                strict=True,
            ):
                pixels = rl.ffi.cast("void *", rl.ffi.from_buffer("const unsigned char[]", plane))
                rl.update_texture(texture, pixels)
        except Exception:
            if replacement is not cached:
                self._unload_navi_media_texture(replacement)
            return cached.texture if cached is not None else None
        self._profile_add("navi_media.upload_yuv420p", profile_stage)

        replacement.sequence = frame.sequence
        if replacement is not cached:
            if cached is not None:
                self._unload_navi_media_texture(cached)
            self._navi_media_textures[frame.key] = replacement
        return replacement.texture

    def _navi_media_texture_for(self, frame: NaviMediaFrame | None):
        if frame is None:
            return None
        cached = self._navi_media_textures.get(frame.key)
        has_payload = frame.hardware_buffer is not None or frame.data is not None or (
            frame.mime == "image/yuv420p" and frame.plane_data is not None and frame.plane_strides is not None
        )
        if not frame.present or not has_payload:
            if cached is not None:
                self._unload_navi_media_texture(cached)
                self._navi_media_textures.pop(frame.key, None)
            return None
        if cached is not None and cached.sequence == frame.sequence and cached.mime == frame.mime:
            return cached.texture
        if frame.mime == "video/nv12-dmabuf":
            return self._upload_navi_hardware_buffer(frame, cached)
        if frame.mime == "image/yuv420p":
            return self._upload_navi_yuv420p(frame, cached)
        size = (frame.width, frame.height)
        if frame.mime == "image/rgba":
            if len(frame.data) != frame.width * frame.height * 4:
                return cached.texture if cached is not None else None
            if cached is None or cached.mime != frame.mime or cached.size != size:
                if cached is not None:
                    self._unload_navi_media_texture(cached)
                image = rl.gen_image_color(frame.width, frame.height, rl_color((0, 0, 0)))
                texture = rl.load_texture_from_image(image)
                rl.unload_image(image)
                if not rl.is_texture_valid(texture):
                    return None
                rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            else:
                texture = cached.texture
            pixels = rl.ffi.cast("void *", rl.ffi.from_buffer("const unsigned char[]", frame.data))
            rl.update_texture(texture, pixels)
            self._navi_media_textures[frame.key] = CachedNaviMediaTexture(
                frame.sequence,
                size,
                frame.mime,
                texture,
            )
            return texture

        extension = ".jpg" if "jpeg" in frame.mime else ".png"
        loaded_image = None
        try:
            loaded_image = rl.load_image_from_memory(extension, frame.data, len(frame.data))
            if not rl.is_image_valid(loaded_image):
                return cached.texture if cached is not None else None
            texture = rl.load_texture_from_image(loaded_image)
            if not rl.is_texture_valid(texture):
                rl.unload_texture(texture)
                return cached.texture if cached is not None else None
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
            if cached is not None:
                self._unload_navi_media_texture(cached)
            self._navi_media_textures[frame.key] = CachedNaviMediaTexture(
                frame.sequence,
                (int(texture.width), int(texture.height)),
                frame.mime,
                texture,
            )
            return texture
        except Exception:
            return cached.texture if cached is not None else None
        finally:
            if loaded_image is not None and rl.is_image_valid(loaded_image):
                rl.unload_image(loaded_image)

    @staticmethod
    def _navi_live_panel_visible(navi: NaviLiveState | None) -> bool:
        if navi is None:
            return False
        return bool(
            navi.current is not None
            or navi.next is not None
            or navi.lane_current is not None
            or navi.route is not None
            or (navi.crossroad is not None and navi.crossroad.visible)
            or (
                navi.speed is not None
                and (navi.speed.sdi_type is not None or navi.speed.section_active)
            )
            or (navi.status is not None and navi.status.guidance_active)
        )

    @staticmethod
    def _navi_map_frame_present(dashboard: NaviDashboardState | None) -> bool:
        if dashboard is None or not dashboard.connected or dashboard.map_stream_stalled:
            return False
        return any(frame.key == "render:map_main" and frame.present for frame in dashboard.media)

    def _draw_navi_map_stalled(self, rect: rl.Rectangle, age_ms: int | None) -> None:
        rl.draw_rectangle_rec(rect, rl_color(NAVI_MAP_BACKGROUND))
        center_x = rect.x + rect.width * 0.5
        center_y = rect.y + rect.height * 0.5
        self._draw_text_with_stroke(
            "MAP STREAM STALLED",
            center_x,
            center_y - 18.0,
            24.0,
            AMBER,
            (5, 9, 12),
            2,
            anchor="center",
        )
        age_text = "WAITING FOR NEXT FRAME"
        if age_ms is not None:
            age_text = f"NO MAP FRAME FOR {age_ms / 1000.0:.1f} s"
        self._draw_text_with_stroke(
            age_text,
            center_x,
            center_y + 18.0,
            16.0,
            WHITE,
            (5, 9, 12),
            1,
            anchor="center",
        )

    def _draw_navi_live_panel(self, state: ClusterUiState) -> None:
        navi = state.navi_live
        theme = self._current_theme()
        x = self._information_panel_x(NAVI_LIVE_PANEL_X)
        y = NAVI_LIVE_PANEL_Y
        w = NAVI_LIVE_PANEL_W
        h = NAVI_LIVE_PANEL_H

        dashboard = state.navi_dashboard
        if dashboard is not None and not dashboard.connected:
            self._draw_system_stats_panel(
                state,
                panel_x=x,
                panel_y=y,
                panel_w=w,
                panel_h=h,
                status_text="NAVI DISCONNECTED",
            )
            return
        if dashboard is not None and dashboard.map_stream_stalled:
            self._rounded_rect(x, y, w, h, 8.0, NAVI_MAP_BACKGROUND, theme.faint, 2.0)
            self._draw_navi_map_stalled(
                rl.Rectangle(x + 3.0, y + 3.0, w - 6.0, h - 6.0),
                dashboard.map_frame_age_ms,
            )
            self._rounded_rect(x, y, w, h, 8.0, (0, 0, 0, 0), theme.faint, 2.0)
            return

        media = {
            frame.key: frame
            for frame in dashboard.media
        } if dashboard is not None else {}
        map_frame = media.get("render:map_main")
        panel_bg = NAVI_MAP_BACKGROUND if map_frame is not None and map_frame.present else theme.route_panel_bg
        self._rounded_rect(x, y, w, h, 8.0, panel_bg, theme.faint, 2.0)
        map_rect = rl.Rectangle(x + 3.0, y + 3.0, w - 6.0, h - 6.0)
        if map_frame is not None and map_frame.present:
            rl.draw_rectangle_rec(map_rect, rl_color(NAVI_MAP_BACKGROUND))
            rl.begin_scissor_mode(
                int(round(map_rect.x)),
                int(round(map_rect.y)),
                int(round(map_rect.width)),
                int(round(map_rect.height)),
            )
            try:
                self._draw_navi_media(map_frame, map_rect, cover=True)
            finally:
                rl.end_scissor_mode()
            self._rounded_rect(x, y, w, h, 8.0, (0, 0, 0, 0), theme.faint, 2.0)
            self._draw_navi_crossroad_box(media.get("image:crossroad_expanded"), x, y, h)

            current_frame = media.get("image:tbt_current_compact")
            if current_frame is None or not current_frame.present:
                current_frame = media.get("image:tbt_current_full")
            current_rect = rl.Rectangle(
                x + 12.0,
                y + 12.0,
                310.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE,
                116.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE,
            )
            current_size = self._navi_media_fitted_size(current_frame, current_rect)
            self._draw_navi_media(
                current_frame,
                current_rect,
                align_x=0.0,
                align_y=0.0,
            )
            # The compact TBT bitmap includes transparent padding at its bottom.
            next_y = current_rect.y + max(
                0.0,
                (current_size[1] if current_size is not None else 0.0)
                - 24.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE,
            )
            self._draw_navi_media(
                media.get("image:tbt_next"),
                rl.Rectangle(
                    x + 12.0,
                    next_y,
                    190.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE,
                    68.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE,
                ),
                align_x=0.0,
                align_y=0.0,
            )

            map_center_x = x + w * 0.5
            center_tbt_icon_rect = rl.Rectangle(map_center_x - 55.0, y + 76.0, 110.0, 110.0)
            self._draw_navi_media(
                media.get("image:center_tbt_icon"),
                center_tbt_icon_rect,
            )
            self._draw_navi_media(
                media.get("image:center_tbt_text"),
                rl.Rectangle(
                    map_center_x - 110.0,
                    center_tbt_icon_rect.y + center_tbt_icon_rect.height + 2.0,
                    220.0,
                    78.0,
                ),
            )
            traffic_rect = rl.Rectangle(x + w - 242.0, y + 12.0, 230.0, 98.0)
            self._draw_navi_media(
                media.get("image:traffic_signal"),
                traffic_rect,
                align_x=1.0,
                align_y=0.0,
            )

            safety_frame = next(
                (
                    frame
                    for key in ("image:safety_primary", "image:safety_secondary", "image:safety_section")
                    for frame in (media.get(key),)
                    if frame is not None and frame.present
                ),
                None,
            )
            self._draw_navi_media(
                safety_frame,
                rl.Rectangle(x + 10.0, y + h * 0.62 - 50.0, 110.0, 100.0),
                align_x=0.0,
            )
            lane_width = 226.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE
            lane_height = 67.0 * NAVI_LIVE_GUIDANCE_MEDIA_SCALE
            lane_rect = rl.Rectangle(
                x + (w - lane_width) * 0.5,
                y + h - lane_height - 6.0,
                lane_width,
                lane_height,
            )
            lane_frame = media.get("image:lane_bottom")
            lane_size = self._navi_media_fitted_size(lane_frame, lane_rect)
            self._draw_navi_media(
                lane_frame,
                lane_rect,
            )

            lane_left = lane_rect.x
            lane_right = lane_rect.x + lane_rect.width
            if lane_size is not None:
                lane_left += (lane_rect.width - lane_size[0]) * 0.5
                lane_right = lane_left + lane_size[0]

            if navi is not None and navi.route is not None:
                route = navi.route
                left_x = x + 14.0
                left_width = lane_left - left_x - 14.0
                distance = self._format_navi_distance(route.remaining_distance_m)
                minutes = max(1, round(route.remaining_time_s / 60.0))
                arrival = time.strftime("%H:%M", time.localtime(time.time() + max(0, route.remaining_time_s)))
                self._draw_text_with_stroke(
                    self._ellipsize_text(distance, 25.0, left_width),
                    left_x,
                    y + h - 48.0,
                    25.0,
                    WHITE,
                    (10, 13, 16),
                    2,
                )
                self._draw_text_with_stroke(
                    self._ellipsize_text(f"{arrival}  {minutes} min", 22.0, left_width),
                    left_x,
                    y + h - 19.0,
                    22.0,
                    WHITE,
                    (10, 13, 16),
                    2,
                )

            road_name = navi.vehicle.road_name if navi is not None and navi.vehicle is not None else ""
            right_x = x + w - 14.0
            right_width = right_x - lane_right - 14.0
            if road_name and right_width >= 70.0:
                self._draw_text_with_stroke(
                    self._ellipsize_text(road_name, 20.0, right_width),
                    right_x,
                    y + h - 25.0,
                    20.0,
                    WHITE,
                    (10, 13, 16),
                    2,
                    anchor="right",
                    cache=True,
                )
            return

        if navi is None:
            return

        current = navi.current
        if current is not None:
            self._draw_navi_turn_icon(current.turn_type, x + 72.0, NAVI_LIVE_ICON_Y, NAVI_LIVE_ICON_SIZE)
            distance_text = self._format_navi_distance(current.distance_m)
            self._draw_text(distance_text, x + 136.0, y + 26.0, 38.0, theme.text)
            main_text = current.main_text or current.road_name or current.near_direction
            self._draw_text(
                self._ellipsize_text(main_text, 25.0, NAVI_LIVE_CONTENT_W),
                x + 136.0,
                y + 75.0,
                25.0,
                theme.text,
            )
            detail = current.road_name if current.road_name and current.road_name != main_text else current.near_direction
            if detail:
                self._draw_text(
                    self._ellipsize_text(detail, 18.0, NAVI_LIVE_CONTENT_W),
                    x + 136.0,
                    y + 112.0,
                    18.0,
                    theme.muted,
                )
        elif navi.vehicle is not None and navi.vehicle.road_name:
            road_name = self._ellipsize_text(navi.vehicle.road_name, 28.0, w - 48.0)
            self._draw_text(road_name, x + 24.0, y + 34.0, 28.0, theme.text)

        next_guidance = navi.next
        if next_guidance is not None:
            rl.draw_line_ex(
                rl.Vector2(x + 20.0, NAVI_LIVE_NEXT_Y - 12.0),
                rl.Vector2(x + w - 20.0, NAVI_LIVE_NEXT_Y - 12.0),
                1.5,
                rl_color(theme.faint),
            )
            self._draw_navi_turn_icon(next_guidance.turn_type, x + 46.0, NAVI_LIVE_NEXT_Y + 22.0, 44.0)
            self._draw_text(
                self._format_navi_distance(next_guidance.distance_m),
                x + 82.0,
                NAVI_LIVE_NEXT_Y - 1.0,
                24.0,
                theme.text,
            )
            next_text = next_guidance.main_text or next_guidance.road_name or next_guidance.near_direction
            self._draw_text(
                self._ellipsize_text(next_text, 18.0, w - 180.0),
                x + 180.0,
                NAVI_LIVE_NEXT_Y + 5.0,
                18.0,
                theme.muted,
            )

        lane = navi.lane_current
        if lane is None and navi.lane_ahead:
            lane = navi.lane_ahead[0]
        if lane is not None and lane.visible:
            self._draw_navi_lane_strip(lane)

        footer_parts: list[str] = []
        if navi.route is not None and navi.route.remaining_distance_m > 0:
            route_text = self._format_navi_distance(navi.route.remaining_distance_m)
            if navi.route.remaining_time_s > 0:
                route_text += f" / {max(1, round(navi.route.remaining_time_s / 60.0))} min"
            footer_parts.append(route_text)
        if navi.speed is not None and navi.speed.sdi_type is not None:
            sdi_text = "SDI"
            if navi.speed.sdi_speed_limit_kph:
                sdi_text += f" {display_speed(navi.speed.sdi_speed_limit_kph, self.is_metric):.0f}"
            if navi.speed.sdi_distance_m is not None:
                sdi_text += f" / {self._format_navi_distance(navi.speed.sdi_distance_m)}"
            footer_parts.append(sdi_text)
        if navi.speed is not None and navi.speed.secondary_sdi_type is not None:
            secondary_sdi_text = "SDI2"
            if navi.speed.secondary_sdi_speed_limit_kph:
                secondary_sdi_text += (
                    f" {display_speed(navi.speed.secondary_sdi_speed_limit_kph, self.is_metric):.0f}"
                )
            if navi.speed.secondary_sdi_distance_m is not None:
                secondary_sdi_text += f" / {self._format_navi_distance(navi.speed.secondary_sdi_distance_m)}"
            footer_parts.append(secondary_sdi_text)
        if navi.crossroad is not None and navi.crossroad.visible:
            footer_parts.append(f"JCT {self._format_navi_distance(navi.crossroad.distance_m)}")
        if navi.status is not None and navi.status.off_route:
            footer_parts.append(self._text("off_route"))
        if footer_parts:
            footer = self._ellipsize_text("   ".join(footer_parts), 18.0, w - 40.0)
            self._draw_text(footer, x + 20.0, NAVI_LIVE_FOOTER_Y, 18.0, theme.muted)

    def _draw_navi_crossroad_box(
        self,
        frame: NaviMediaFrame | None,
        map_left: float,
        map_top: float,
        map_height: float,
    ) -> None:
        if frame is None or not frame.present or frame.width <= 0 or frame.height <= 0:
            return
        theme = self._current_theme()
        box_h = map_height * 0.70
        aspect = frame.width / max(1.0, float(frame.height))
        if self._panel_swap_active():
            box_x = map_left + NAVI_LIVE_PANEL_W
            available_w = DESIGN_WIDTH - box_x - 4.0
        else:
            available_w = map_left - 830.0
            box_x = map_left
        box_w = clamp(box_h * aspect, 320.0, max(320.0, available_w))
        if not self._panel_swap_active():
            box_x -= box_w
        box = rl.Rectangle(box_x, map_top, box_w, box_h)
        self._rounded_rect(box.x, box.y, box.width, box.height, 8.0, theme.route_panel_bg, theme.faint, 2.0)
        image_rect = rl.Rectangle(box.x + 4.0, box.y + 4.0, box.width - 8.0, box.height - 8.0)
        rl.begin_scissor_mode(
            int(round(image_rect.x)),
            int(round(image_rect.y)),
            int(round(image_rect.width)),
            int(round(image_rect.height)),
        )
        try:
            self._draw_navi_media(frame, image_rect, cover=True)
        finally:
            rl.end_scissor_mode()
        self._rounded_rect(box.x, box.y, box.width, box.height, 8.0, (0, 0, 0, 0), theme.faint, 2.0)

    def _draw_navi_lane_strip(self, lane: NaviLaneInfo) -> None:
        theme = self._current_theme()
        count = min(8, max(lane.count, len(lane.available), len(lane.turn_info)))
        if count <= 0:
            return
        x = self._information_panel_x(NAVI_LIVE_PANEL_X) + 22.0
        available_w = NAVI_LIVE_PANEL_W - 44.0
        gap = 6.0
        cell_w = min(46.0, (available_w - gap * (count - 1)) / count)
        total_w = cell_w * count + gap * (count - 1)
        x += (available_w - total_w) * 0.5
        y = NAVI_LIVE_LANE_Y
        current_index = lane.current_lane - 1 if 1 <= lane.current_lane <= count else lane.current_lane
        for index in range(count):
            available = index < len(lane.available) and lane.available[index] > 0
            active = index == current_index
            fill = (35, 92, 58, 230) if available else theme.panel_bg
            outline = GREEN if active else (102, 194, 132) if available else theme.faint
            self._rounded_rect(x, y, cell_w, 46.0, 4.0, fill, outline, 3.0 if active else 1.5)
            center_x = x + cell_w * 0.5
            arrow_color = GREEN if available else theme.muted
            self._draw_navi_round_line(
                rl.Vector2(center_x, y + 33.0),
                rl.Vector2(center_x, y + 14.0),
                3.0,
                arrow_color,
            )
            rl.draw_triangle(
                rl.Vector2(center_x, y + 8.0),
                rl.Vector2(center_x - 5.0, y + 16.0),
                rl.Vector2(center_x + 5.0, y + 16.0),
                rl_color(arrow_color),
            )
            x += cell_w + gap

    def _draw_navi_turn_icon(self, turn_type: int, cx: float, cy: float, size: float) -> None:
        theme = self._current_theme()
        radius = size * 0.5
        rl.draw_circle_v(rl.Vector2(cx, cy), radius, rl_color((24, 88, 148, 235)))
        rl.draw_ring(
            rl.Vector2(cx, cy),
            max(0.0, radius - 2.0),
            radius,
            0.0,
            360.0,
            48,
            rl_color(BLUE_SOFT),
        )
        color = WHITE if theme.is_dark else (246, 249, 252)
        width = max(3.0, size * 0.075)

        def point(dx: float, dy: float) -> rl.Vector2:
            return rl.Vector2(cx + dx * size, cy + dy * size)

        if turn_type in (153, 154, 249):
            self._draw_text("TG", cx, cy - size * 0.16, size * 0.34, color, anchor="center")
            return
        if turn_type == 14:
            rl.draw_ring(
                rl.Vector2(cx, cy - size * 0.04),
                size * 0.16,
                size * 0.16 + width,
                190.0,
                360.0,
                30,
                rl_color(color),
            )
            self._draw_navi_round_line(point(0.20, -0.04), point(0.20, 0.28), width, color)
            rl.draw_triangle(point(-0.24, 0.23), point(-0.34, 0.08), point(-0.12, 0.13), rl_color(color))
            return
        if turn_type in NAVI_TURN_ROUNDABOUT_TYPES:
            rl.draw_ring(
                rl.Vector2(cx, cy),
                size * 0.16,
                size * 0.16 + width,
                25.0,
                330.0,
                36,
                rl_color(color),
            )
            rl.draw_triangle(point(0.29, -0.20), point(0.11, -0.22), point(0.23, -0.05), rl_color(color))
            return

        direction = -1.0 if turn_type in NAVI_TURN_LEFT_TYPES else 1.0 if turn_type in NAVI_TURN_RIGHT_TYPES else 0.0
        if direction == 0.0:
            self._draw_navi_round_line(point(0.0, 0.28), point(0.0, -0.24), width, color)
            rl.draw_triangle(point(0.0, -0.35), point(-0.11, -0.19), point(0.11, -0.19), rl_color(color))
            return
        self._draw_navi_round_line(point(0.0, 0.28), point(0.0, -0.03), width, color)
        self._draw_navi_round_line(point(0.0, -0.03), point(direction * 0.27, -0.03), width, color)
        rl.draw_triangle(
            point(direction * 0.36, -0.03),
            point(direction * 0.22, -0.15),
            point(direction * 0.22, 0.09),
            rl_color(color),
        )

    def _format_navi_distance(self, distance_m: int | float) -> str:
        return format_navi_distance(distance_m, self.is_metric)

    def _draw_navi_debug_panel(self, info: NaviDebugInfo | None) -> None:
        theme = self._current_theme()
        panel_x = self._information_panel_x(SYSTEM_PANEL_X)
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

    def _draw_navi_traffic_light_panel(
        self,
        traffic: NaviTrafficLightInfo,
        *,
        panel_right: float = NAVI_TRAFFIC_PANEL_RIGHT,
        panel_y: float = NAVI_TRAFFIC_PANEL_Y,
    ) -> None:
        theme = self._current_theme()
        y = panel_y
        h = NAVI_TRAFFIC_PANEL_H
        bg = NAVI_TRAFFIC_BG_DARK if theme.is_dark else NAVI_TRAFFIC_BG_LIGHT
        off = NAVI_TRAFFIC_OFF_DARK if theme.is_dark else NAVI_TRAFFIC_OFF_LIGHT

        elapsed_s = 0
        if traffic.meta is not None:
            elapsed_s = max(0, int(time.monotonic() - traffic.meta.received_mono_s))

        def countdown(seconds: int | None) -> int | None:
            return None if seconds is None else max(0, seconds - elapsed_s)

        red_s = countdown(traffic.red_s)
        straight_s = countdown(traffic.straight_s)
        left_s = countdown(traffic.left_s)
        right_s = countdown(traffic.right_s)
        uturn_s = countdown(traffic.uturn_s)
        red_on = self._navi_signal_active(traffic, "red", red_s)
        straight_on = self._navi_signal_active(traffic, "straight", straight_s)
        left_on = self._navi_signal_active(traffic, "left", left_s)
        right_on = self._navi_signal_active(traffic, "right", right_s)
        uturn_on = self._navi_signal_active(traffic, "uturn", uturn_s)
        use_uturn_slot = (uturn_on or uturn_s is not None) and not (left_on or left_s is not None)

        primary_seconds, primary_red = self._navi_primary_signal_seconds(
            traffic,
            red_s=red_s,
            straight_s=straight_s,
            left_s=left_s,
            right_s=right_s,
            uturn_s=uturn_s,
        )
        remain_text = "--" if primary_seconds is None else str(primary_seconds)
        remain_color = NAVI_TRAFFIC_RED if primary_red else NAVI_TRAFFIC_GREEN
        remain_size = 54.0
        remain_width, _ = self._measure_text(remain_text, remain_size, max(1.0, remain_size * 0.02))
        show_right = right_on or right_s is not None
        slot_count = 4 if show_right else 3
        slot_span = slot_count * NAVI_TRAFFIC_SIGNAL_SIZE + (slot_count - 1) * NAVI_TRAFFIC_SIGNAL_GAP
        content_w = slot_span + NAVI_TRAFFIC_TEXT_GAP + remain_width
        w = max(270.0, content_w + NAVI_TRAFFIC_PANEL_PAD_X * 2.0)
        x = panel_right - w
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

    def _navi_primary_signal_seconds(
        self,
        traffic: NaviTrafficLightInfo | None,
        *,
        red_s: int | None = None,
        straight_s: int | None = None,
        left_s: int | None = None,
        right_s: int | None = None,
        uturn_s: int | None = None,
    ) -> tuple[int | None, bool]:
        if traffic is None:
            return None, False
        ordered = (
            ("red", traffic.red_s if red_s is None else red_s, True),
            ("left", traffic.left_s if left_s is None else left_s, False),
            ("uturn", traffic.uturn_s if uturn_s is None else uturn_s, False),
            ("straight", traffic.straight_s if straight_s is None else straight_s, False),
            ("right", traffic.right_s if right_s is None else right_s, False),
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
        box_x = self._information_panel_x(NAVI_GUIDANCE_IMAGE_X)
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

    def _draw_system_dashboard_panel(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        stats = self._system_stats.sample()
        panel_x = self._information_panel_x(NAVI_LIVE_PANEL_X)
        panel_y = NAVI_LIVE_PANEL_Y
        panel_w = NAVI_LIVE_PANEL_W
        panel_h = NAVI_LIVE_PANEL_H
        card_y = panel_y + 8.0
        card_h = panel_h - 16.0
        detail_x = panel_x + 16.0
        detail_w = 474.0
        health_x = detail_x + detail_w + 10.0
        health_w = panel_x + panel_w - 16.0 - health_x

        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 12.0, theme.panel_bg, theme.faint, 1.5)
        self._draw_system_stats_panel(
            state,
            panel_x=detail_x,
            panel_y=card_y,
            panel_w=detail_w,
            panel_h=card_h,
            show_runtime_info=True,
            title_text="DETAIL",
            stats=stats,
        )
        self._draw_system_health_card(
            state,
            stats,
            health_x,
            card_y,
            health_w,
            card_h,
        )

    def _draw_system_stats_panel(
        self,
        state: ClusterUiState,
        *,
        panel_x: float | None = None,
        panel_y: float = SYSTEM_PANEL_Y,
        panel_w: float = SYSTEM_PANEL_W,
        panel_h: float | None = None,
        status_text: str | None = None,
        show_runtime_info: bool = False,
        title_text: str = "SYSTEM",
        stats: SystemStats | None = None,
    ) -> None:
        if panel_x is None:
            panel_x = self._information_panel_x(SYSTEM_PANEL_X)
        theme = self._current_theme()
        if stats is None:
            stats = self._system_stats.sample()
        disconnected = status_text is not None
        detailed = disconnected or show_runtime_info
        cpu_count = len(stats.cpu_core_percents)
        columns = 2 if cpu_count <= 8 else 4
        rows = max(1, math.ceil(max(1, cpu_count) / columns))
        core_row_h = 30.0 if columns == 2 else 24.0
        header_h = 216.0 if detailed else 122.0
        if panel_h is None:
            panel_h = min(DESIGN_HEIGHT - panel_y - 18.0, header_h + rows * core_row_h + 18.0)
        core_area_h = max(24.0, panel_h - header_h - 14.0)
        core_row_h = min(core_row_h, core_area_h / rows)

        pad_x = 24.0
        panel_bg = NAVI_MAP_BACKGROUND if disconnected else theme.route_panel_bg
        panel_outline = (82, 92, 100, 255) if disconnected else theme.faint
        text_color = WHITE if disconnected else theme.text
        muted_color = (154, 164, 172, 255) if disconnected else theme.muted
        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 18, panel_bg, panel_outline, 2)
        self._draw_text(title_text, panel_x + pad_x, panel_y + 28, 18, muted_color)
        if status_text:
            self._draw_text(
                status_text,
                panel_x + panel_w - pad_x,
                panel_y + 28,
                18,
                AMBER,
                anchor="right",
            )

        if detailed:
            fps_text = "-- Hz"
            if state.actual_fps is not None and math.isfinite(state.actual_fps):
                fps_text = f"{state.actual_fps:.1f} Hz"
            camera_text = "STANDBY"
            if state.onroad:
                camera_text = (
                    "ROAD CAMERA"
                    if cluster_camera_view_is_road_camera(state.camera_view_mode)
                    else "ONROAD"
                )
            info_rows = (
                ("NETWORK", state.network_address or "unavailable"),
                ("DISPLAY", f"{self.width} x {self.height}  /  {fps_text}"),
                ("CAMERA", camera_text),
            )
            for index, (label, value) in enumerate(info_rows):
                line_y = panel_y + 66.0 + index * 30.0
                self._draw_text(label, panel_x + pad_x, line_y, 16, muted_color)
                self._draw_text(value, panel_x + panel_w - pad_x, line_y, 17, text_color, anchor="right")

        mem_percent = stats.memory_used_percent
        mem_color = self._system_metric_color(mem_percent)
        mem_text_y = panel_y + (164.0 if detailed else 62.0)
        mem_bar_y = panel_y + (182.0 if detailed else 80.0)
        self._draw_text("MEM", panel_x + pad_x, mem_text_y, 17, muted_color)
        self._draw_text(
            self._memory_text(stats),
            panel_x + 86,
            mem_text_y,
            17,
            text_color if stats.memory_used_bytes is not None else muted_color,
        )
        self._draw_text(
            self._percent_text(mem_percent),
            panel_x + panel_w - pad_x,
            mem_text_y,
            17,
            mem_color,
            anchor="right",
        )
        self._draw_percent_bar(panel_x + pad_x, mem_bar_y, panel_w - pad_x * 2, 12, mem_percent, mem_color)

        cpu_header_y = panel_y + (208.0 if detailed else 104.0)
        self._draw_text("CPU CORE %", panel_x + pad_x, cpu_header_y, 15, muted_color)
        if cpu_count == 0:
            self._draw_text("unavailable", panel_x + panel_w - pad_x, cpu_header_y, 15, muted_color, anchor="right")
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
            self._draw_text(f"C{index}", cell_x, line_y + 8, text_size, muted_color)
            self._draw_text(self._percent_text(percent), cell_x + cell_w, line_y + 8, text_size, color, anchor="right")
            self._draw_percent_bar(cell_x, line_y + 19, cell_w, 6, percent, color)

    def _draw_trip_report_panel(self, state: ClusterUiState) -> None:
        target = getattr(self, "_trip_report_target", None)
        if not getattr(self, "_trip_report_cache_valid", False) or target is None:
            self._draw_trip_report_panel_contents(state)
            return

        panel_x = self._information_panel_x(TRIP_REPORT_PANEL_X)
        source = rl.Rectangle(
            0.0,
            0.0,
            float(target.texture.width),
            -float(target.texture.height),
        )
        destination = rl.Rectangle(
            panel_x,
            TRIP_REPORT_PANEL_Y,
            TRIP_REPORT_PANEL_W,
            TRIP_REPORT_PANEL_H,
        )
        rl.draw_texture_pro(
            target.texture,
            source,
            destination,
            rl.Vector2(0.0, 0.0),
            0.0,
            rl_color(WHITE),
        )

    def _draw_trip_report_panel_contents(self, state: ClusterUiState) -> None:
        report = state.trip_report or TripReportState()
        stats = self._system_stats.sample()
        theme = self._current_theme()
        panel_x = self._information_panel_x(TRIP_REPORT_PANEL_X)
        panel_y = TRIP_REPORT_PANEL_Y
        panel_w = TRIP_REPORT_PANEL_W
        panel_h = TRIP_REPORT_PANEL_H
        panel_bg = theme.panel_bg
        card_bg = theme.route_panel_bg
        card_outline = theme.faint
        text_color = theme.text
        muted = theme.muted

        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 12.0, panel_bg, card_outline, 1.5)

        summary_x = panel_x + 16.0
        summary_y = panel_y + 8.0
        summary_w = 474.0
        summary_h = panel_h - 16.0
        system_x = summary_x + summary_w + 10.0
        system_y = summary_y
        system_w = panel_x + panel_w - 16.0 - system_x
        system_h = summary_h
        self._rounded_rect(summary_x, summary_y, summary_w, summary_h, 10.0, card_bg, card_outline, 1.2)

        self._draw_text(self._text("trip_summary"), summary_x + 18.0, summary_y + 31.0, 22.0, muted)
        self._draw_text(self._text("time"), summary_x + 18.0, summary_y + 82.0, 19.0, muted)
        self._draw_text(
            self._trip_format_time(report.duration_s),
            summary_x + summary_w - 18.0,
            summary_y + 82.0,
            35.0,
            (255, 177, 105),
            anchor="right",
        )
        rl.draw_line_ex(
            rl.Vector2(summary_x + 18.0, summary_y + 114.0),
            rl.Vector2(summary_x + summary_w - 18.0, summary_y + 114.0),
            1.0,
            rl_color(card_outline),
        )
        metric_w = (summary_w - 54.0) * 0.5
        metric_left = summary_x + 18.0
        metric_right = metric_left + metric_w + 18.0
        report_speed_unit = speed_unit(self.is_metric)
        self._draw_trip_metric(
            metric_left,
            summary_y + 138.0,
            self._text("distance"),
            self._trip_distance_text(report.distance_m),
            metric_w,
            text_color=text_color,
            muted_color=muted,
        )
        self._draw_trip_metric(
            metric_right,
            summary_y + 138.0,
            self._text("average_speed"),
            f"{display_speed(report.average_speed_kph, self.is_metric):.1f}",
            metric_w,
            report_speed_unit,
            text_color=text_color,
            muted_color=muted,
        )
        self._draw_trip_metric(
            metric_left,
            summary_y + 218.0,
            self._text("max_speed"),
            f"{display_speed(report.max_speed_kph, self.is_metric):.1f}",
            metric_w,
            report_speed_unit,
            text_color=text_color,
            muted_color=muted,
        )
        self._draw_trip_metric(
            metric_right,
            summary_y + 218.0,
            self._text("auto_drive"),
            f"{report.auto_ratio_percent:.0f}",
            metric_w,
            "%",
            text_color=text_color,
            muted_color=muted,
        )
        self._draw_trip_metric(
            metric_left,
            summary_y + 298.0,
            self._text("max_accel"),
            f"{report.max_accel_mps2:+.2f}",
            metric_w,
            "m/s²",
            text_color=text_color,
            muted_color=muted,
        )
        self._draw_trip_metric(
            metric_right,
            summary_y + 298.0,
            self._text("max_decel"),
            f"{report.max_decel_mps2:+.2f}",
            metric_w,
            "m/s²",
            text_color=text_color,
            muted_color=muted,
        )

        event_y = summary_y + 390.0
        event_w = (summary_w - 56.0) / 3.0
        event_label_size = 16.0 if self.language == CLUSTER_LANGUAGE_KO else 13.0
        for index, (label, count, color) in enumerate((
            (self._text("hard_accel"), report.hard_accel_count, AMBER),
            (self._text("hard_brake"), report.hard_brake_count, RED),
            (self._text("hard_corner"), report.hard_corner_count, BLUE_SOFT),
        )):
            event_x = summary_x + 18.0 + index * (event_w + 10.0)
            self._rounded_rect(event_x, event_y, event_w, 47.0, 7.0, theme.panel_bg, color, 1.2)
            self._draw_text(label, event_x + 9.0, event_y + 23.5, event_label_size, muted)
            self._draw_text(str(count), event_x + event_w - 9.0, event_y + 23.5, 23.0, color, anchor="right")

        self._draw_system_health_card(
            state,
            stats,
            system_x,
            system_y,
            system_w,
            system_h,
            panel_bg=card_bg,
            panel_outline=card_outline,
            text_color=text_color,
            muted_color=muted,
            target_fill=theme.panel_bg,
        )

    def _draw_system_health_card(
        self,
        state: ClusterUiState,
        stats: SystemStats,
        panel_x: float,
        panel_y: float,
        panel_w: float,
        panel_h: float,
        *,
        panel_bg: tuple[int, ...] | None = None,
        panel_outline: tuple[int, ...] | None = None,
        text_color: tuple[int, ...] | None = None,
        muted_color: tuple[int, ...] | None = None,
        target_fill: tuple[int, ...] | None = None,
    ) -> None:
        theme = self._current_theme() if any(
            value is None
            for value in (panel_bg, panel_outline, text_color, muted_color, target_fill)
        ) else None
        if panel_bg is None:
            assert theme is not None
            panel_bg = theme.route_panel_bg
        if panel_outline is None:
            assert theme is not None
            panel_outline = theme.faint
        if text_color is None:
            assert theme is not None
            text_color = theme.text
        if muted_color is None:
            assert theme is not None
            muted_color = theme.muted
        if target_fill is None:
            assert theme is not None
            target_fill = theme.panel_bg

        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 10.0, panel_bg, panel_outline, 1.2)
        self._draw_text(self._text("system"), panel_x + 18.0, panel_y + 31.0, 22.0, muted_color)
        cpu_percent = state.cpu_usage_percent if state.cpu_usage_percent is not None else stats.cpu_used_percent
        memory_percent = (
            state.memory_used_percent
            if state.memory_used_percent is not None
            else stats.memory_used_percent
        )
        disk_percent = (
            state.disk_used_percent
            if state.disk_used_percent is not None
            else stats.disk_used_percent
        )
        system_metrics = (
            ("CPU", self._percent_text(cpu_percent).strip(), cpu_percent, self._system_metric_color(cpu_percent)),
            (
                "TEMP",
                "--°C" if state.cpu_temp_c is None else f"{state.cpu_temp_c:.0f}°C",
                state.cpu_temp_c,
                self._trip_temp_color(state.cpu_temp_c, muted_color),
            ),
            ("MEM", self._percent_text(memory_percent).strip(), memory_percent, self._system_metric_color(memory_percent)),
            ("DISK", self._percent_text(disk_percent).strip(), disk_percent, self._system_metric_color(disk_percent)),
        )
        gauge_pad_x = 16.0
        gauge_gap_x = 8.0
        gauge_w = (panel_w - gauge_pad_x * 2.0 - gauge_gap_x) * 0.5
        gauge_centers_y = (panel_y + 107.0, panel_y + 224.0)
        for index, (label, value, percent, color) in enumerate(system_metrics):
            column = index % 2
            row = index // 2
            center_x = panel_x + gauge_pad_x + gauge_w * 0.5 + column * (gauge_w + gauge_gap_x)
            self._draw_system_gauge(
                center_x,
                gauge_centers_y[row],
                label,
                value,
                percent,
                color,
                muted_color,
                panel_outline,
            )

        rl.draw_line_ex(
            rl.Vector2(panel_x + 18.0, panel_y + 282.0),
            rl.Vector2(panel_x + panel_w - 18.0, panel_y + 282.0),
            1.0,
            rl_color(panel_outline),
        )
        self._draw_text(self._text("device_angle"), panel_x + 18.0, panel_y + 308.0, 18.0, muted_color)
        pitch_deg: float | None = None
        yaw_deg: float | None = None
        calibration = state.camera_calibration_euler
        if calibration is not None and len(calibration) >= 3:
            candidate_pitch = math.degrees(calibration[1])
            candidate_yaw = math.degrees(calibration[2])
            if math.isfinite(candidate_pitch) and math.isfinite(candidate_yaw):
                pitch_deg = candidate_pitch
                yaw_deg = candidate_yaw
        self._draw_device_angle_indicator(
            panel_x + 14.0,
            panel_y + 325.0,
            panel_w - 28.0,
            117.0,
            pitch_deg,
            yaw_deg,
            muted_color,
            panel_outline,
            text_color=text_color,
            target_fill=target_fill,
        )

    def _draw_system_gauge(
        self,
        center_x: float,
        center_y: float,
        label: str,
        value_text: str,
        percent: float | None,
        value_color: tuple[int, int, int],
        muted: tuple[int, int, int],
        track_color: tuple[int, int, int],
    ) -> None:
        radius = 41.0
        stroke_width = 5.0
        start_angle = 135.0
        sweep_angle = 270.0
        segments = 24
        rl.draw_ring(
            rl.Vector2(center_x, center_y),
            radius - stroke_width,
            radius,
            start_angle,
            start_angle + sweep_angle,
            segments,
            rl_color(track_color),
        )
        if percent is not None and math.isfinite(percent):
            ratio = clamp(percent, 0.0, 100.0) / 100.0
            if ratio > 0.0:
                end_angle = start_angle + sweep_angle * ratio
                progress_color = BLUE_SOFT
                rl.draw_ring(
                    rl.Vector2(center_x, center_y),
                    radius - stroke_width,
                    radius,
                    start_angle,
                    end_angle,
                    max(4, int(round(segments * ratio))),
                    rl_color(progress_color),
                )
                cap_radius = stroke_width * 0.5
                for angle in (start_angle, end_angle):
                    angle_rad = math.radians(angle)
                    cap_center = rl.Vector2(
                        center_x + (radius - cap_radius) * math.cos(angle_rad),
                        center_y + (radius - cap_radius) * math.sin(angle_rad),
                    )
                    rl.draw_circle_v(cap_center, cap_radius, rl_color(progress_color))

        self._draw_text(label, center_x, center_y - 13.0, 13.0, muted, anchor="center")
        self._draw_text(value_text, center_x, center_y + 9.0, 23.0, value_color, anchor="center")

    @staticmethod
    def _device_angle_target_offset(
        pitch_deg: float,
        yaw_deg: float,
        radius: float,
        max_angle_deg: float = 6.0,
    ) -> tuple[float, float]:
        usable_radius = radius * 0.68
        scale = usable_radius / max_angle_deg
        # Match the device settings convention: positive pitch points down and
        # positive yaw points left.
        return (
            -clamp(yaw_deg, -max_angle_deg, max_angle_deg) * scale,
            clamp(pitch_deg, -max_angle_deg, max_angle_deg) * scale,
        )

    def _draw_device_angle_indicator(
        self,
        x: float,
        y: float,
        width: float,
        height: float,
        pitch_deg: float | None,
        yaw_deg: float | None,
        muted: tuple[int, int, int],
        outline: tuple[int, int, int],
        *,
        text_color: tuple[int, ...] = WHITE,
        target_fill: tuple[int, ...] = (11, 18, 26, 235),
    ) -> None:
        center_x = x + 52.0
        center_y = y + height * 0.5
        radius = 31.0
        rl.draw_circle_v(rl.Vector2(center_x, center_y), radius, rl_color(target_fill))
        rl.draw_ring(
            rl.Vector2(center_x, center_y),
            radius - 1.2,
            radius,
            0.0,
            360.0,
            24,
            rl_color(outline),
        )
        rl.draw_line_ex(
            rl.Vector2(center_x - radius + 6.0, center_y),
            rl.Vector2(center_x + radius - 6.0, center_y),
            1.0,
            rl_color(outline),
        )
        rl.draw_line_ex(
            rl.Vector2(center_x, center_y - radius + 6.0),
            rl.Vector2(center_x, center_y + radius - 6.0),
            1.0,
            rl_color(outline),
        )
        self._draw_text("P-", center_x, center_y - radius - 7.0, 9.0, muted, anchor="center")
        self._draw_text("P+", center_x, center_y + radius + 7.0, 9.0, muted, anchor="center")
        self._draw_text("Y+", center_x - radius - 8.0, center_y, 9.0, muted, anchor="center")
        self._draw_text("Y-", center_x + radius + 8.0, center_y, 9.0, muted, anchor="center")
        rl.draw_circle_v(rl.Vector2(center_x, center_y), 2.5, rl_color(muted))

        valid_angles = (
            pitch_deg is not None
            and yaw_deg is not None
            and math.isfinite(pitch_deg)
            and math.isfinite(yaw_deg)
        )
        if valid_angles:
            offset_x, offset_y = self._device_angle_target_offset(pitch_deg, yaw_deg, radius)
            marker_x = center_x + offset_x
            marker_y = center_y + offset_y
            rl.draw_line_ex(
                rl.Vector2(center_x, center_y),
                rl.Vector2(marker_x, marker_y),
                2.5,
                rl_color(BLUE_SOFT),
            )
            rl.draw_circle_v(rl.Vector2(marker_x, marker_y), 6.0, rl_color(BLUE_SOFT))
            rl.draw_circle_v(rl.Vector2(marker_x, marker_y), 2.2, rl_color(target_fill))

        value_x = x + width * 0.45
        pitch_text = "P  --°" if pitch_deg is None else f"P {pitch_deg:+.1f}°"
        yaw_text = "Y  --°" if yaw_deg is None else f"Y {yaw_deg:+.1f}°"
        self._draw_text(pitch_text, value_x, y + 36.0, 22.0, text_color)
        self._draw_text(yaw_text, value_x, y + 69.0, 22.0, text_color)

    def _draw_trip_metric(
        self,
        x: float,
        y: float,
        label: str,
        value: str,
        width: float,
        unit: str = "",
        *,
        text_color: tuple[int, ...] | None = None,
        muted_color: tuple[int, ...] | None = None,
    ) -> None:
        theme = self._current_theme() if text_color is None or muted_color is None else None
        if text_color is None:
            assert theme is not None
            text_color = theme.text
        if muted_color is None:
            assert theme is not None
            muted_color = theme.muted
        self._draw_text(label, x, y, 18.0, muted_color)
        self._draw_text(value, x, y + 32.0, 29.0, text_color)
        if unit:
            self._draw_text(unit, x + width, y + 32.0, 16.0, muted_color, anchor="right")

    @staticmethod
    def _trip_temp_color(
        temp_c: float | None,
        unavailable_color: tuple[int, ...],
    ) -> tuple[int, ...]:
        if temp_c is None or not math.isfinite(temp_c):
            return unavailable_color
        if temp_c >= 90.0:
            return RED
        if temp_c >= 75.0:
            return AMBER
        return GREEN

    @staticmethod
    def _trip_format_time(duration_s: float) -> str:
        total_seconds = max(0, int(round(duration_s)))
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

    def _trip_distance_text(self, distance_m: float) -> str:
        return format_trip_distance(distance_m, self.is_metric)

    def _draw_live_debug_panel(self, state: ClusterUiState) -> None:
        sections = self._live_debug_sections(state)
        theme = self._current_theme()
        panel_x = self._information_panel_x(NAVI_LIVE_PANEL_X)
        panel_y = NAVI_LIVE_PANEL_Y
        panel_w = NAVI_LIVE_PANEL_W
        panel_h = NAVI_LIVE_PANEL_H
        card_pad_x = 16.0
        card_pad_y = 8.0
        card_gap = 10.0
        card_w = (panel_w - card_pad_x * 2.0 - card_gap) * 0.5
        card_h = (panel_h - card_pad_y * 2.0 - card_gap) * 0.5

        self._rounded_rect(panel_x, panel_y, panel_w, panel_h, 12.0, theme.panel_bg, theme.faint, 1.5)
        for index, (section_title, rows) in enumerate(sections):
            column = index % 2
            row = index // 2
            card_x = panel_x + card_pad_x + column * (card_w + card_gap)
            card_y = panel_y + card_pad_y + row * (card_h + card_gap)
            content_x = card_x + 18.0
            value_x = card_x + card_w - 18.0
            value_max_w = card_w - 210.0

            self._rounded_rect(card_x, card_y, card_w, card_h, 12.0, theme.route_panel_bg, theme.faint, 1.2)
            self._draw_text(section_title, content_x, card_y + 30.0, 18.0, theme.muted)
            for row_index, (label, value) in enumerate(rows):
                text_y = card_y + 78.0 + row_index * 38.0
                self._draw_text(label, content_x, text_y, 17.0, theme.muted)
                value = self._ellipsize_text(value, 17.0, value_max_w)
                self._draw_text(value, value_x, text_y, 17.0, theme.text, anchor="right")

    def _live_debug_sections(self, state: ClusterUiState) -> tuple[tuple[str, tuple[tuple[str, str], ...]], ...]:
        live_debug = state.live_debug
        live_valid = "--"
        if live_debug is not None and live_debug.live_torque_valid is not None:
            live_valid = "ON" if live_debug.live_torque_valid else "OFF"
        lateral_plan_text = str(state.lateral_plan_debug_text) if state.lateral_plan_debug_text else "--"
        return (
            (
                "LIVE DELAY",
                (
                    (
                        "CAL / LAT",
                        f"{self._optional_percent_text(live_debug.live_delay_calibration_percent if live_debug else None)} / "
                        f"{self._optional_seconds_text(live_debug.live_delay_lateral_s if live_debug else None, 2)}",
                    ),
                ),
            ),
            (
                "LIVE TORQUE",
                (
                    (
                        "STATE",
                        f"{live_valid} / "
                        f"{self._optional_percent_text(live_debug.live_torque_calibration_percent if live_debug else None)}",
                    ),
                    (
                        "FACT / FRIC",
                        f"{self._optional_float_text(live_debug.live_torque_lat_accel_factor if live_debug else None, 2)} / "
                        f"{self._optional_float_text(live_debug.live_torque_friction if live_debug else None, 2)}",
                    ),
                ),
            ),
            (
                "STEERING",
                (
                    (
                        "SR LIVE / CUSTOM",
                        f"{self._optional_float_text(live_debug.live_steer_ratio if live_debug else None, 1)} / "
                        f"{self._optional_float_text(live_debug.custom_steer_ratio if live_debug else None, 1)}",
                    ),
                    (
                        "SAD",
                        self._optional_seconds_text(live_debug.steer_actuator_delay_s if live_debug else None, 2),
                    ),
                ),
            ),
            (
                "LATERAL PLAN",
                (("DEBUG", lateral_plan_text),),
            ),
        )

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
        if overlay is None or not overlay.panel_visible:
            return
        theme = self._current_theme()
        panel_x = self._information_panel_x(SYSTEM_PANEL_X)
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
        self._draw_text("CURRENT CODE CUT-IN", x, y, 16, theme.muted)
        status_parts = (overlay.cutin_status or "NEW CUTIN: waiting").split(" | ", 1)
        status_color = RED if ": YES" in status_parts[0] else GREEN
        self._draw_text(status_parts[0], x, y + 22, 15, status_color)
        if len(status_parts) > 1:
            self._draw_text(status_parts[1], x, y + 41, 12, theme.text)
        for index, line in enumerate(overlay.data_lines[:6]):
            self._draw_text(line, x, y + 61 + index * 13, 11, theme.text)

    def _draw_git_status(
        self,
        status: GitBranchStatus | None,
        network_address: str | None = None,
        actual_fps: float | None = None,
    ) -> None:
        if status is None and network_address is None:
            return

        theme = self._current_theme()
        text_size = 20
        spacing = max(1.0, text_size * 0.02)
        _, text_height = self._measure_text("0", text_size, spacing)
        row_h = max(text_height, GIT_STATUS_DOT_RADIUS * 2)
        center_y = DESIGN_HEIGHT - GIT_STATUS_BOTTOM_MARGIN - row_h * 0.5
        text_x = GIT_STATUS_MARGIN + GIT_STATUS_DOT_RADIUS * 2 + GIT_STATUS_DOT_TEXT_GAP
        if status is not None:
            color = self._git_status_color(status, theme)
            text = status.branch if not status.detail else f"{status.branch} ({status.detail})"
            text = self._ellipsize_text(text, text_size, GIT_STATUS_MAX_TEXT_W)
            dot_center_x = GIT_STATUS_MARGIN + GIT_STATUS_DOT_RADIUS
            rl.draw_circle_v(rl.Vector2(dot_center_x, center_y), GIT_STATUS_DOT_RADIUS, rl_color(color))
            self._draw_text_with_stroke(text, text_x, center_y, text_size, WHITE, (5, 9, 12), 2, cache=True)
        if network_address:
            network_text = self._ellipsize_text(network_address, text_size, GIT_STATUS_MAX_TEXT_W)
            network_y = center_y - row_h - 4.0
            self._draw_text_with_stroke(
                network_text,
                text_x,
                network_y,
                text_size,
                WHITE,
                (5, 9, 12),
                2,
                cache=True,
            )
            network_width, _ = self._measure_text(network_text, text_size, spacing)
            self._draw_actual_fps(actual_fps, text_x + network_width + 18.0, network_y)

    def _draw_actual_fps(self, actual_fps: float | None, x: float, center_y: float) -> None:
        if actual_fps is None or not math.isfinite(actual_fps):
            return

        text = f"FPS {actual_fps:.1f} Hz"
        text_size = 20
        text = self._ellipsize_text(text, text_size, FPS_STATUS_MAX_TEXT_W)
        dot_center_x = x + FPS_STATUS_DOT_RADIUS
        text_x = x + FPS_STATUS_DOT_RADIUS * 2 + FPS_STATUS_DOT_TEXT_GAP
        rl.draw_circle_v(rl.Vector2(dot_center_x, center_y), FPS_STATUS_DOT_RADIUS, rl_color(GREEN))
        self._draw_text_with_stroke(text, text_x, center_y, text_size, WHITE, (5, 9, 12), 2)

    def _draw_cluster_core_usage(self, text: str | None, *, right_x: float = DESIGN_WIDTH) -> None:
        if not text:
            return

        theme = self._current_theme()
        text_size = 18
        text = self._ellipsize_text(text, text_size, CLUSTER_CORE_USAGE_MAX_TEXT_W)
        spacing = max(1.0, text_size * 0.02)
        _, text_height = self._measure_text(text, text_size, spacing)
        x = right_x - CLUSTER_CORE_USAGE_MARGIN
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
        gear_text = (state.gear_text or "").strip().upper()
        if (
            not state.debug_ui_visible
            and not gear_text
            and state.cruise_gap is None
            and not self._cruise_set_visible(state)
            and state.lfa_active is None
            and not state.egpu_active
        ):
            return

        self._draw_network_status(state, TOP_STATUS_CENTER_Y + WIFI_STATUS_ICON_SIZE * 0.5)
        self._draw_egpu_status(state)
        self._draw_lfa_status_icon(state, TOP_STATUS_CENTER_Y + LFA_STATUS_ICON_SIZE * 0.5)

    def _draw_egpu_status(self, state: ClusterUiState) -> None:
        if not state.egpu_active:
            return
        rect = rl.Rectangle(
            EGPU_STATUS_CENTER_X - EGPU_STATUS_W * 0.5,
            TOP_STATUS_CENTER_Y - EGPU_STATUS_H * 0.5,
            EGPU_STATUS_W,
            EGPU_STATUS_H,
        )
        rl.draw_rectangle_rounded(rect, 0.35, 8, rl_color((0, 0, 0), 150))
        rl.draw_rectangle_rounded_lines_ex(rect, 0.35, 8, 2.0, rl_color(GREEN))
        self._draw_text("eGPU", EGPU_STATUS_CENTER_X, TOP_STATUS_CENTER_Y + 1.0,
                        EGPU_STATUS_FONT_SIZE, GREEN, anchor="center")

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
        bar_h = FOLLOW_GAP_BAR_H * FOLLOW_GAP_BAR_SCALE * 2.0
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

    def _draw_network_status(self, state: ClusterUiState, bottom_y: float) -> None:
        theme = self._current_theme()
        tint = WHITE if state.network_connected else theme.muted
        alpha = 255 if state.network_connected else 130
        self._draw_outlined_bottom_aligned_texture_icon(
            self._wifi_texture,
            WIFI_STATUS_CENTER_X,
            bottom_y,
            WIFI_STATUS_ICON_SIZE,
            WIFI_STATUS_ICON_SIZE,
            tint,
            alpha,
        )

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

    def _draw_outlined_bottom_aligned_texture_icon(
        self,
        texture,
        center_x: float,
        bottom_y: float,
        width: float,
        height: float,
        tint: tuple[int, int, int] | tuple[int, int, int, int],
        alpha: int,
        rotation_deg: float = 0.0,
    ) -> bool:
        if texture is None:
            return False
        outline_offset = 2.0
        outline_alpha = max(150, min(245, alpha))
        for dx, dy in (
            (-outline_offset, 0.0),
            (outline_offset, 0.0),
            (0.0, -outline_offset),
            (0.0, outline_offset),
            (-outline_offset, -outline_offset),
            (outline_offset, -outline_offset),
            (-outline_offset, outline_offset),
            (outline_offset, outline_offset),
        ):
            self._draw_bottom_aligned_texture_icon(
                texture,
                center_x + dx,
                bottom_y + dy,
                width,
                height,
                (5, 9, 12),
                outline_alpha,
                rotation_deg,
            )
        return self._draw_bottom_aligned_texture_icon(
            texture,
            center_x,
            bottom_y,
            width,
            height,
            tint,
            alpha,
            rotation_deg,
        )

    def _draw_lfa_status_icon(self, state: ClusterUiState, bottom_y: float) -> None:
        theme = self._current_theme()
        active = bool(state.lfa_active)
        texture = self._lfa_active_texture if active and self._lfa_active_texture is not None else self._lfa_texture
        tint = WHITE if active else theme.muted
        alpha = 255 if active else 190
        rotation_deg = -float(state.steering_angle_deg or 0.0)
        drew_wheel = self._draw_bottom_aligned_texture_icon(
            texture,
            LFA_STATUS_CENTER_X,
            bottom_y,
            LFA_STATUS_ICON_SIZE,
            LFA_STATUS_ICON_SIZE,
            tint,
            alpha,
            rotation_deg,
        )
        if state.active_lane_line:
            self._draw_bottom_aligned_texture_icon(
                self._lfa_lane_texture,
                LFA_STATUS_CENTER_X,
                bottom_y - LFA_LANE_ICON_TOP_OFFSET,
                LFA_STATUS_ICON_SIZE * LFA_LANE_ICON_WIDTH_SCALE,
                LFA_STATUS_ICON_SIZE,
                GREEN if active else theme.muted,
                alpha,
            )
        if drew_wheel:
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

    def _draw_speed_block(self, state: ClusterUiState, *, tpms_offset_x: float = 0.0) -> None:
        theme = self._current_theme()
        display_speed_kph = state.display_speed_kph if state.display_speed_kph is not None else state.speed_kph
        speed_value = int(round(display_speed(clamp(display_speed_kph, 0.0, MAX_SPEED_KPH), self.is_metric)))
        speed_text = str(speed_value)
        speed_font_size = SPEED_VALUE_FONT_SIZE if len(speed_text) <= 2 else SPEED_VALUE_FONT_SIZE * 0.86
        self._draw_speed_panel_bg()
        self._draw_text_with_stroke(
            speed_text,
            SPEED_VALUE_CENTER_X,
            SPEED_VALUE_CENTER_Y,
            speed_font_size,
            WHITE,
            (0, 0, 0),
            3,
            anchor="center",
        )

        cruise_color = self._cruise_set_color(state, theme)
        cruise_text = self._cruise_set_speed_text(state)
        self._draw_text_with_stroke(
            cruise_text,
            CRUISE_SET_SPEED_CENTER_X,
            CRUISE_SPEED_CENTER_Y,
            CRUISE_SET_SPEED_FONT_SIZE,
            cruise_color,
            (0, 0, 0),
            2,
            anchor="center",
        )
        self._draw_driving_mode_indicator(state)
        self._draw_model_traffic_state(state.traffic_state)
        self._draw_cruise_gap_badge(state.cruise_gap)
        self._draw_speed_gear_badge(state)
        self._draw_ev_mode_indicator(state)
        tpms_translated = abs(tpms_offset_x) > 0.001
        if tpms_translated:
            rl.rl_push_matrix()
            rl.rl_translatef(tpms_offset_x, 0.0, 0.0)
        try:
            self._draw_tpms_status(state)
        finally:
            if tpms_translated:
                rl.rl_pop_matrix()

        if self._cruise_set_visible(state) and state.cruise_override_kph is not None:
            override_color = (
                GREEN
                if state.cruise_override_color_mode == 1
                else AMBER
                if state.cruise_override_color_mode == 4
                else VEHICLE_NAVI
                if state.cruise_override_color_mode == 3
                else CRUISE_OVERRIDE_APPLY_COLOR
                if state.cruise_override_color_mode == 2
                else theme.text
            )
            override_text = str(int(round(display_speed(state.cruise_override_kph, self.is_metric))))
            override_label = state.cruise_override_label or ""
            override_label_font_size = CRUISE_OVERRIDE_LABEL_FONT_SIZE * min(
                1.0,
                8.0 / max(8, len(override_label)),
            )
            self._draw_text_with_stroke(
                override_label,
                CRUISE_OVERRIDE_SPEED_CENTER_X,
                CRUISE_OVERRIDE_LABEL_CENTER_Y,
                override_label_font_size,
                override_color,
                (0, 0, 0),
                2,
                anchor="center",
            )
            self._draw_text_with_stroke(
                override_text,
                CRUISE_OVERRIDE_SPEED_CENTER_X,
                CRUISE_OVERRIDE_SPEED_CENTER_Y,
                CRUISE_OVERRIDE_SPEED_FONT_SIZE,
                override_color,
                (0, 0, 0),
                2,
                anchor="center",
            )

        if state.speed_limit_kph is not None or state.navi_debug is not None:
            center = rl.Vector2(SPEED_LIMIT_SIGN_CENTER_X, SPEED_LIMIT_SIGN_CENTER_Y)
            rl.draw_circle_v(center, SPEED_LIMIT_SIGN_RADIUS, rl_color(RED))
            rl.draw_circle_v(center, SPEED_LIMIT_SIGN_RADIUS - SPEED_LIMIT_SIGN_RING_WIDTH, rl_color(WHITE))
            limit_text = (
                "--"
                if state.speed_limit_kph is None
                else str(int(round(display_speed(state.speed_limit_kph, self.is_metric))))
            )
            limit_font_size = 42 if len(limit_text) <= 2 else 36
            self._draw_text(
                limit_text,
                SPEED_LIMIT_SIGN_CENTER_X,
                SPEED_LIMIT_SIGN_CENTER_Y,
                limit_font_size,
                TEXT,
                anchor="center",
            )

    def _draw_driving_mode_indicator(self, state: ClusterUiState) -> None:
        style = SPEED_DRIVING_MODE_STYLES.get(state.driving_mode)
        if style is None:
            return
        text_key, color = style
        text = self._text(text_key)
        font_size = SPEED_DRIVING_MODE_FONT_SIZE if self.language == CLUSTER_LANGUAGE_KO else 22.0
        self._draw_text_with_stroke(
            text,
            SPEED_DRIVING_MODE_CENTER_X,
            SPEED_DRIVING_MODE_CENTER_Y,
            font_size,
            color,
            (5, 9, 12),
            2,
            anchor="center",
            cache=True,
        )

    def _draw_model_traffic_state(self, traffic_state: int) -> None:
        if traffic_state not in (1, 2):
            return
        red_light = traffic_state == 1
        texture = self._traffic_red_texture if red_light else self._traffic_green_texture
        if texture is None:
            return
        size = SPEED_MODEL_TRAFFIC_ICON_SIZE
        source = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
        destination = rl.Rectangle(
            SPEED_MODEL_TRAFFIC_CENTER_X - size * 0.5,
            SPEED_MODEL_TRAFFIC_CENTER_Y - size * 0.5,
            size,
            size,
        )
        rl.draw_texture_pro(texture, source, destination, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE))

    def _draw_cruise_gap_badge(self, cruise_gap: int | None) -> None:
        if cruise_gap is None:
            return
        gap = int(clamp(float(cruise_gap), 1.0, 4.0))
        self._draw_text_with_stroke(
            str(gap),
            SPEED_GAP_CENTER_X,
            SPEED_GAP_CENTER_Y,
            SPEED_GAP_FONT_SIZE,
            WHITE,
            (5, 9, 12),
            2,
            anchor="center",
            cache=True,
        )

    def _draw_speed_gear_badge(self, state: ClusterUiState) -> None:
        gear = (state.gear_text or "").strip().upper()[:2]
        if not gear:
            return
        theme = self._current_theme()
        color = WHITE if gear != "U" else theme.muted
        outline = color if cluster_camera_view_is_road_camera(state.camera_view_mode) else (5, 9, 12, 245)
        rect = rl.Rectangle(
            SPEED_GEAR_CENTER_X - SPEED_GEAR_W * 0.5,
            SPEED_GEAR_CENTER_Y - SPEED_GEAR_H * 0.5,
            SPEED_GEAR_W,
            SPEED_GEAR_H,
        )
        self._rounded_rect(rect.x, rect.y, rect.width, rect.height, 8.0, (0, 0, 0, 0), outline, 3.0)
        self._draw_text_with_stroke(
            gear,
            SPEED_GEAR_CENTER_X,
            SPEED_GEAR_CENTER_Y,
            SPEED_GEAR_FONT_SIZE,
            color,
            (5, 9, 12),
            2,
            anchor="center",
            cache=True,
        )

    def _draw_ev_mode_indicator(
        self,
        state: ClusterUiState,
        center_x: float = SPEED_EV_CENTER_X,
        center_y: float = SPEED_EV_CENTER_Y,
        font_size: float = SPEED_EV_FONT_SIZE,
    ) -> None:
        if not (state.ev_mode_valid and state.ev_mode_active):
            return
        self._draw_text_with_stroke(
            "EV",
            center_x,
            center_y,
            font_size,
            GREEN,
            (5, 9, 12),
            2,
            anchor="center",
            cache=True,
        )

    def _draw_speed_panel_bg(self) -> None:
        texture = self._speed_bg_texture
        if texture is None:
            return
        source = rl.Rectangle(0.0, 0.0, float(texture.width), float(texture.height))
        dest = rl.Rectangle(SPEED_PANEL_X, SPEED_PANEL_Y, SPEED_PANEL_W, SPEED_PANEL_H)
        rl.draw_texture_pro(texture, source, dest, rl.Vector2(0.0, 0.0), 0.0, rl_color(WHITE, 235))

    @staticmethod
    def _cruise_set_visible(state: ClusterUiState) -> bool:
        return state.cruise_kph is not None and state.cruise_display_state != "off"

    def _cruise_set_speed_text(self, state: ClusterUiState) -> str:
        if state.cruise_display_state == "off" or state.cruise_kph is None:
            return "--"
        return str(int(round(display_speed(state.cruise_kph, self.is_metric))))

    @staticmethod
    def _cruise_set_color(state: ClusterUiState, theme: ClusterTheme) -> tuple[int, int, int]:
        if state.cruise_display_state == "off" or state.cruise_kph is None:
            return theme.muted
        if state.cruise_display_state == "paused":
            return theme.muted
        return GREEN

    def _draw_accel_block(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        top = SIDE_GAUGE_TOP
        bottom = SIDE_GAUGE_BOTTOM
        center = (top + bottom) // 2
        gauge_width = SIDE_GAUGE_WIDTH
        accel_value = 0.0 if abs(state.accel_mps2) < 0.005 else state.accel_mps2
        accel_text = f"{accel_value:+05.2f}"
        accel_text_size = 17
        gauge_center_x = SIDE_GAUGE_LEFT_CENTER_X
        gauge_x = gauge_center_x - gauge_width * 0.5
        fill_x = gauge_x + 8
        fill_width = gauge_width - 16
        outline = self._side_gauge_outline(state)
        self._rounded_rect(gauge_x, top, gauge_width, bottom - top, 18, (0, 0, 0, 0), outline, 2)
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
        self._draw_text_with_stroke(
            accel_text,
            gauge_center_x,
            SIDE_GAUGE_VALUE_Y,
            accel_text_size,
            fill_color,
            (5, 9, 12),
            2,
            anchor="center",
            cache=False,
        )
        self._draw_text_with_stroke(
            "accel",
            gauge_center_x,
            bottom + SIDE_GAUGE_LABEL_OFFSET,
            17,
            WHITE,
            (5, 9, 12),
            2,
            anchor="center",
            cache=True,
        )

        energy_value = state.fuel_gauge
        if energy_value is not None:
            self._draw_level_gauge(
                gauge_center_x,
                SIDE_GAUGE_LOWER_TOP,
                SIDE_GAUGE_LOWER_BOTTOM,
                gauge_width,
                clamp(energy_value, 0.0, 1.0),
                state.energy_gauge_label,
                GREEN if energy_value > 0.15 else RED,
                outline,
            )

    def _draw_steering_output_block(self, state: ClusterUiState) -> None:
        theme = self._current_theme()
        gauge_center_x = SIDE_GAUGE_LEFT_CENTER_X + SIDE_GAUGE_COLUMN_GAP
        top = SIDE_GAUGE_TOP
        bottom = SIDE_GAUGE_BOTTOM
        gauge_width = SIDE_GAUGE_WIDTH
        outline = self._side_gauge_outline(state)
        if (
            state.steering_output is not None
            and state.steering_output_normalized is not None
            and state.steering_output_kind is not None
        ):
            value = float(state.steering_output)
            output_kind = state.steering_output_kind
            normalized = clamp(float(state.steering_output_normalized), -1.0, 1.0)
            if output_kind == "angle":
                value_text = f"{value:+.0f}%"
            else:
                value_text = f"{normalized * 100.0:+.0f}%" if abs(value) > 999.0 else f"{value:+.0f}"
            color = BLUE if normalized > 0 else AMBER if normalized < 0 else theme.muted
            self._draw_bipolar_gauge(gauge_center_x, top, bottom, gauge_width, normalized, color, outline)
            self._draw_text_with_stroke(
                value_text,
                gauge_center_x,
                SIDE_GAUGE_VALUE_Y,
                17,
                color,
                (5, 9, 12),
                2,
                anchor="center",
                cache=False,
            )
            self._draw_text_with_stroke(
                "steer",
                gauge_center_x,
                bottom + SIDE_GAUGE_LABEL_OFFSET,
                17,
                WHITE,
                (5, 9, 12),
                2,
                anchor="center",
                cache=True,
            )

        urea_value = state.urea_gauge
        if urea_value is not None:
            self._draw_level_gauge(
                gauge_center_x,
                SIDE_GAUGE_LOWER_TOP,
                SIDE_GAUGE_LOWER_BOTTOM,
                gauge_width,
                clamp(urea_value, 0.0, 1.0),
                "DEF",
                AMBER if urea_value <= 0.15 else BLUE,
                outline,
            )

    def _side_gauge_outline(self, state: ClusterUiState) -> tuple[int, int, int, int]:
        if self._current_theme().is_dark or cluster_camera_view_is_road_camera(state.camera_view_mode):
            return SIDE_GAUGE_OUTLINE
        return (5, 9, 12, 235)

    def _draw_bipolar_gauge(
        self,
        center_x: float,
        top: float,
        bottom: float,
        width: float,
        value: float,
        color: tuple[int, int, int],
        outline: tuple[int, int, int, int],
    ) -> None:
        theme = self._current_theme()
        center_y = (top + bottom) * 0.5
        gauge_x = center_x - width * 0.5
        fill_x = gauge_x + 8
        fill_width = width - 16
        self._rounded_rect(gauge_x, top, width, bottom - top, 18, (0, 0, 0, 0), outline, 2)
        rl.draw_line_ex(rl.Vector2(gauge_x, center_y), rl.Vector2(gauge_x + width, center_y), 3, rl_color(theme.gauge_midline))
        fill_height = abs(value) * ((bottom - top) * 0.5 - 8)
        if fill_height <= 0.0:
            return
        fill_y = center_y - fill_height if value > 0.0 else center_y
        self._rounded_rect(fill_x, fill_y, fill_width, fill_height, 13, color)

    def _draw_level_gauge(
        self,
        center_x: float,
        top: float,
        bottom: float,
        width: float,
        value: float,
        label: str,
        color: tuple[int, int, int],
        outline: tuple[int, int, int, int],
    ) -> None:
        gauge_x = center_x - width * 0.5
        self._rounded_rect(gauge_x, top, width, bottom - top, 18, (0, 0, 0, 0), outline, 2)
        fill_height = value * (bottom - top - 16)
        if fill_height > 0.0:
            self._rounded_rect(gauge_x + 8, bottom - 8 - fill_height, width - 16, fill_height, 13, color)
        self._draw_text_with_stroke(
            f"{value * 100:.0f}%", center_x, top - 16, 17, color, (5, 9, 12), 2, anchor="center"
        )
        self._draw_text_with_stroke(
            label,
            center_x,
            bottom + SIDE_GAUGE_LABEL_OFFSET,
            17,
            WHITE,
            (5, 9, 12),
            2,
            anchor="center",
            cache=True,
        )

    def _turn_signal_lights(self, state: ClusterUiState) -> tuple[bool, bool]:
        now = time.perf_counter()
        if state.left_signal and state.right_signal:
            if self._hazard_signal_started_at is None:
                self._hazard_signal_started_at = now
                self._left_turn_signal_started_at = None
                self._right_turn_signal_started_at = None
            lit = blink_visible(now, self._hazard_signal_started_at, float("inf"))
            return lit, lit

        self._hazard_signal_started_at = None
        return (
            self._turn_signal_lit("left", state.left_signal, now),
            self._turn_signal_lit("right", state.right_signal, now),
        )

    def _highlight_lane_lit(self, state: ClusterUiState, signal_lights: tuple[bool, bool]) -> bool:
        left_signal_lit, right_signal_lit = signal_lights
        side = state.highlight_lane if state.highlight_lane in ("left", "right") else None
        if side is not None and state.highlight_lane_offset is not None:
            now = time.perf_counter()
            if side != self._lane_highlight_side or self._lane_highlight_started_at is None:
                self._lane_highlight_side = side
                self._lane_highlight_started_at = now

            signal_active = state.left_signal if side == "left" else state.right_signal
            signal_lit = left_signal_lit if side == "left" else right_signal_lit
            signal_started_at = (
                self._hazard_signal_started_at
                if state.left_signal and state.right_signal
                else self._left_turn_signal_started_at
                if side == "left"
                else self._right_turn_signal_started_at
            )
            if signal_started_at is not None:
                self._lane_highlight_started_at = signal_started_at
            if signal_active:
                return signal_lit
            return blink_visible(now, self._lane_highlight_started_at, float("inf"))

        self._lane_highlight_side = None
        self._lane_highlight_started_at = None
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

    def _draw_turn_signal(
        self,
        side: str,
        lit: bool,
        show_inactive: bool = False,
        center_x_offset: float = 0.0,
    ) -> None:
        if not lit and not show_inactive:
            return

        theme = self._current_theme()
        cx = (
            LANE_TURN_SIGNAL_LEFT_CENTER_X if side == "left" else LANE_TURN_SIGNAL_RIGHT_CENTER_X
        ) + center_x_offset
        cy = LANE_TURN_SIGNAL_CENTER_Y
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
        if not text:
            return
        font = self._font_for_text(text)
        spacing = max(1.0, size * 0.02)
        text_width, text_height = self._measure_text(text, size, spacing, font)
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
        rl.draw_text_ex(font, text, rl.Vector2(draw_x, draw_y), size, spacing, rl_color(color))

    def _font_for_text(self, text: str):
        if self._font is None:
            self._font = rl.get_font_default()
        if self._korean_font is not None and any(ord(char) > 0x7F for char in text):
            return self._korean_font
        return self._font

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
        cache: bool = False,
    ) -> None:
        if not text:
            return
        if cache and stroke_width > 0 and self._stroked_text_texture_cache_enabled:
            cached_text = self._stroked_text_texture(
                text,
                size,
                color,
                stroke_color,
                stroke_width,
            )
            if cached_text is not None:
                self._draw_cached_text_texture(cached_text, x, y, anchor)
                return
        if stroke_width > 0 and self._draw_stroked_text_raw(
            text,
            x,
            y,
            size,
            color,
            stroke_color,
            stroke_width,
            anchor,
        ):
            return
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

    def _draw_stroked_text_raw(
        self,
        text: str,
        x: float,
        y: float,
        size: float,
        color: tuple[int, int, int],
        stroke_color: tuple[int, int, int],
        stroke_width: int,
        anchor: str,
    ) -> bool:
        draw_text_ex = getattr(self, "_raw_draw_text_ex", None)
        if not getattr(self, "_raw_stroked_text_enabled", False) or draw_text_ex is None:
            return False

        font = self._font_for_text(text)
        spacing = max(1.0, size * 0.02)
        text_width, text_height = self._measure_text(text, size, spacing, font)
        encoded_text = text.encode("utf-8")
        position = rl.Vector2(0.0, 0.0)
        fill = rl_color(color)
        stroke = rl_color(stroke_color)

        def draw_at(draw_x: float, draw_y: float, draw_color: rl.Color) -> None:
            if anchor == "center":
                draw_x -= text_width * 0.5
                draw_y -= text_height * 0.5
            elif anchor == "left":
                draw_y -= text_height * 0.5
            elif anchor == "right":
                draw_x -= text_width
                draw_y -= text_height * 0.5
            position.x = draw_x
            position.y = draw_y
            draw_text_ex(font, encoded_text, position, size, spacing, draw_color)

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
            draw_at(x + dx, y + dy, stroke)
        draw_at(x, y, fill)
        return True

    def _draw_cached_text_texture(
        self,
        cached_text: CachedTextTexture,
        x: float,
        y: float,
        anchor: str,
    ) -> None:
        draw_x = x
        draw_y = y
        if anchor == "center":
            draw_x -= cached_text.text_width * 0.5
            draw_y -= cached_text.text_height * 0.5
        elif anchor == "left":
            draw_y -= cached_text.text_height * 0.5
        elif anchor == "right":
            draw_x -= cached_text.text_width
            draw_y -= cached_text.text_height * 0.5
        draw_x -= cached_text.padding_px
        draw_y -= cached_text.padding_px
        rl.draw_texture_pro(
            cached_text.texture,
            rl.Rectangle(
                0.0,
                0.0,
                float(cached_text.texture_width),
                float(cached_text.texture_height),
            ),
            rl.Rectangle(
                draw_x,
                draw_y,
                float(cached_text.texture_width),
                float(cached_text.texture_height),
            ),
            rl.Vector2(0.0, 0.0),
            0.0,
            rl_color(WHITE),
        )

    def _stroked_text_texture(
        self,
        text: str,
        size: float,
        color: tuple[int, int, int] | tuple[int, int, int, int],
        stroke_color: tuple[int, int, int] | tuple[int, int, int, int],
        stroke_width: int,
    ) -> CachedTextTexture | None:
        font = self._font_for_text(text)
        render_size = float(size)
        spacing = max(1.0, render_size * 0.02)
        fill_key = rgba_key(color)
        stroke_key = rgba_key(stroke_color)
        cache_key = (
            id(font),
            text,
            render_size,
            spacing,
            fill_key,
            stroke_key,
            int(stroke_width),
        )
        cached_text = self._stroked_text_texture_cache.get(cache_key)
        if cached_text is not None:
            self._stroked_text_texture_cache.move_to_end(cache_key)
            return cached_text

        # Texture upload inside an active target corrupts the miss frame on GLES.
        self._pending_stroked_text_textures.setdefault(
            cache_key,
            PendingStrokedTextTexture(
                font=font,
                text=text,
                render_size=render_size,
                spacing=spacing,
                fill_color=fill_key,
                stroke_color=stroke_key,
                stroke_width=int(stroke_width),
            ),
        )
        return None

    def _flush_pending_stroked_text_textures(self) -> None:
        if not self._pending_stroked_text_textures:
            return
        if not self._stroked_text_texture_cache_enabled:
            self._pending_stroked_text_textures.clear()
            return

        profile_stage = self._profile_start()
        pending_textures = self._pending_stroked_text_textures
        self._pending_stroked_text_textures = OrderedDict()
        for cache_key, pending in pending_textures.items():
            if cache_key in self._stroked_text_texture_cache:
                continue
            cached_text = self._create_stroked_text_texture(pending)
            if cached_text is None:
                continue
            self._stroked_text_texture_cache[cache_key] = cached_text
            while len(self._stroked_text_texture_cache) > STROKED_TEXT_TEXTURE_CACHE_LIMIT:
                _, old_text = self._stroked_text_texture_cache.popitem(last=False)
                rl.unload_texture(old_text.texture)
        self._profile_add("stroked_text_texture_cache.flush", profile_stage)

    def _create_stroked_text_texture(
        self,
        pending: PendingStrokedTextTexture,
    ) -> CachedTextTexture | None:
        text_width, text_height = self._measure_text(
            pending.text,
            pending.render_size,
            pending.spacing,
            pending.font,
        )
        padding_px = float(max(0, pending.stroke_width) + STROKED_TEXT_TEXTURE_PADDING_PX)
        texture_width = max(1, int(math.ceil(text_width + padding_px * 2.0)))
        texture_height = max(1, int(math.ceil(text_height + padding_px * 2.0)))
        image = None
        texture = None
        try:
            image = rl.gen_image_color(texture_width, texture_height, rl_color((0, 0, 0, 0)))
            for dx, dy in (
                (-pending.stroke_width, 0),
                (pending.stroke_width, 0),
                (0, -pending.stroke_width),
                (0, pending.stroke_width),
                (-pending.stroke_width, -pending.stroke_width),
                (pending.stroke_width, -pending.stroke_width),
                (-pending.stroke_width, pending.stroke_width),
                (pending.stroke_width, pending.stroke_width),
            ):
                rl.image_draw_text_ex(
                    image,
                    pending.font,
                    pending.text,
                    rl.Vector2(padding_px + dx, padding_px + dy),
                    pending.render_size,
                    pending.spacing,
                    rl_color(pending.stroke_color),
                )
            rl.image_draw_text_ex(
                image,
                pending.font,
                pending.text,
                rl.Vector2(padding_px, padding_px),
                pending.render_size,
                pending.spacing,
                rl_color(pending.fill_color),
            )
            texture = rl.load_texture_from_image(image)
            if hasattr(rl, "is_texture_valid") and not rl.is_texture_valid(texture):
                rl.unload_texture(texture)
                return None
            rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
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
        return cached_text

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
        font = self._font_for_text(text)
        render_size = max(
            1.0,
            round(float(size) / WORLD_LABEL_TEXTURE_SIZE_GRID) * WORLD_LABEL_TEXTURE_SIZE_GRID,
        )
        spacing = max(1.0, render_size * 0.02)
        color_key = rgba_key(color)
        cache_key = (id(font), text, render_size, spacing, color_key)
        cached_text = self._world_label_texture_cache.get(cache_key)
        if cached_text is not None:
            self._world_label_texture_cache.move_to_end(cache_key)
            return cached_text

        profile_stage = self._profile_start()
        text_width, text_height = self._measure_text(text, render_size, spacing, font)
        padding_px = float(WORLD_LABEL_TEXTURE_PADDING_PX)
        texture_width = max(1, int(math.ceil(text_width + padding_px * 2.0)))
        texture_height = max(1, int(math.ceil(text_height + padding_px * 2.0)))
        image = None
        texture = None
        try:
            image = rl.gen_image_color(texture_width, texture_height, rl_color((0, 0, 0, 0)))
            rl.image_draw_text_ex(
                image,
                font,
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

    def _measure_text(
        self,
        text: str,
        size: float,
        spacing: float | None = None,
        font=None,
    ) -> tuple[float, float]:
        if font is None:
            font = self._font_for_text(text)
        measure_spacing = max(1.0, size * 0.02) if spacing is None else spacing
        key = (id(font), text, float(size), float(measure_spacing))
        measured = self._text_measure_cache.get(key)
        if measured is not None:
            return measured
        if len(self._text_measure_cache) >= TEXT_MEASURE_CACHE_LIMIT:
            self._text_measure_cache.clear()
        text_size = rl.measure_text_ex(font, text, size, measure_spacing)
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
