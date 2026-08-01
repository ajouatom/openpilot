from pathlib import Path
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import (
  CLUSTER_PANEL_LAYOUT_DRIVING_LEFT,
  CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT,
  CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
  CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
  CLUSTER_SCREEN_MODE_TRIP_REPORT,
  normalize_cluster_live_fps,
  normalize_cluster_panel_layout,
  normalize_cluster_screen_mode,
  resolved_usb_h264_bitrate,
)


def test_cluster_live_fps_modes_remain_independent_from_map_fps():
  assert [normalize_cluster_live_fps(mode) for mode in range(7)] == [
    0.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0,
  ]
  assert normalize_cluster_live_fps(7) == 0.0
  assert normalize_cluster_live_fps("invalid") == 0.0


def test_cluster_h264_auto_bitrate_preserves_per_frame_budget_through_60_fps():
  expected = {
    10: "2333k",
    20: "4667k",
    30: "7M",
    40: "9333k",
    50: "11667k",
    60: "14M",
  }
  assert {fps: resolved_usb_h264_bitrate("auto", fps, 30) for fps in expected} == expected
  assert resolved_usb_h264_bitrate("auto", 0, 30) == "7M"
  assert resolved_usb_h264_bitrate("12M", 60, 30) == "12M"


def test_cluster_screen_mode_five_selects_trip_report():
  assert normalize_cluster_screen_mode(5) == CLUSTER_SCREEN_MODE_TRIP_REPORT
  assert normalize_cluster_screen_mode("trip-report") == CLUSTER_SCREEN_MODE_TRIP_REPORT
  assert normalize_cluster_screen_mode("navi-debug") == 0


def test_cluster_screen_mode_two_selects_debug_system():
  assert CLUSTER_SCREEN_MODE_DEBUG_SYSTEM == 2
  assert normalize_cluster_screen_mode(2) == CLUSTER_SCREEN_MODE_DEBUG_SYSTEM
  assert normalize_cluster_screen_mode("system") == CLUSTER_SCREEN_MODE_DEBUG_SYSTEM
  assert normalize_cluster_screen_mode("debug_system") == CLUSTER_SCREEN_MODE_DEBUG_SYSTEM


def test_cluster_screen_mode_minus_one_selects_fullscreen_3d():
  assert CLUSTER_SCREEN_MODE_FULLSCREEN_3D == -1
  assert normalize_cluster_screen_mode(-1) == CLUSTER_SCREEN_MODE_FULLSCREEN_3D
  assert normalize_cluster_screen_mode("3d-fullscreen") == CLUSTER_SCREEN_MODE_FULLSCREEN_3D
  assert normalize_cluster_screen_mode("fullscreen_3d") == CLUSTER_SCREEN_MODE_FULLSCREEN_3D


def test_cluster_panel_layout_accepts_named_and_numeric_sides():
  assert normalize_cluster_panel_layout(0) == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
  assert normalize_cluster_panel_layout("driving-left") == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
  assert normalize_cluster_panel_layout(1) == CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  assert normalize_cluster_panel_layout("driving-right") == CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  assert normalize_cluster_panel_layout("swap") == CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  assert normalize_cluster_panel_layout("invalid") == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
