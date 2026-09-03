"""Guards the fields the settings UI now reads straight from the catalogue.

Display units and control kinds used to live in hardcoded tables inside
web/js/pages/setting.js, so adding a parameter meant editing two places and
forgetting one was silent. They are declared per parameter now; these tests
keep the declarations well formed.
"""
import json
from pathlib import Path

import pytest

SETTINGS_PATH = Path(__file__).resolve().parents[3] / "carrot_settings.json"
PARAMS_KEYS_PATH = Path(__file__).resolve().parents[4] / "common" / "params_keys.h"

# Mirrors SETTING_DISPLAY_UNIT_TYPES / SETTING_CONTROL_KINDS in setting.js.
KNOWN_DISPLAY_UNITS = {"raw", "speedKph", "distanceCm", "timeSec", "timeMin", "percent", "degree"}
KNOWN_CONTROL_KINDS = {"toggle", "segmented", "select", "slider"}
KNOWN_RISK_LEVELS = {"high", "medium"}


@pytest.fixture(scope="module")
def settings():
  with SETTINGS_PATH.open(encoding="utf-8") as f:
    return json.load(f)


@pytest.fixture(scope="module")
def params(settings):
  return settings["params"]


def test_the_catalogue_is_readable_and_populated(params):
  assert len(params) > 100
  assert all(isinstance(p.get("name"), str) and p["name"] for p in params)


def test_obsolete_lead_response_settings_are_removed(settings, params):
  removed = {"JLeadFactor3", "RadarReactionFactor"}
  by_name = {p["name"] for p in params}
  assert removed.isdisjoint(by_name)
  assert all(
    removed.isdisjoint(group.get("params", []))
    for category in settings["menu"]
    for section in category.get("groups", [])
    for group in section.get("groups", [])
  )
  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert all(name not in params_keys for name in removed)


def test_automatic_driving_mode_exposes_manual_normal_and_eco_choices(params):
  by_name = {p["name"]: p for p in params}
  automatic = by_name["MyDrivingModeAuto"]
  assert (automatic["min"], automatic["max"], automatic["default"]) == (0, 2, 0)
  assert "1:일반↔안전" in automatic["descr"]
  assert "2:에코↔안전" in automatic["descr"]


def test_longitudinal_comfort_settings_use_driver_facing_language(params):
  by_name = {p["name"]: p for p in params}

  lead_response = by_name["DynamicTFollow"]
  assert lead_response["default"] == 0
  assert lead_response["display_unit"] == "percent"
  assert "0%는 사용 안 함" in lead_response["descr"]

  lead_accel_response = by_name["LeadAccelResponse"]
  assert (lead_accel_response["min"], lead_accel_response["max"], lead_accel_response["default"]) == (0, 5, 0)
  assert lead_accel_response["control"] == "select"
  assert "차간거리 1단계" in lead_accel_response["descr"]
  assert "TFollowGap1" in lead_accel_response["descr"]
  assert "3단계는 일상 균형형" in lead_accel_response["descr"]
  assert "최대 0.2m/s²" in lead_accel_response["descr"]
  assert lead_accel_response["options"]["ko"][3] == "3 균형(추천)"
  assert lead_accel_response["options"]["ko"][-1] == "5 가속추종(시험)"

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"LeadAccelResponse", {PERSISTENT, INT, "0"}}' in params_keys

  lane_change = by_name["DynamicTFollowLC"]
  assert lane_change["default"] == 100
  assert "100%는 변화 없음" in lane_change["descr"]

  decel_margin = by_name["TFollowDecelBoost"]
  assert decel_margin["default"] == 50
  assert "목표 간격" in decel_margin["descr"]

  driving_mode = by_name["MyDrivingMode"]
  assert "ComfortBrake" not in driving_mode["descr"]
  assert "멀리서부터 천천히 감속" in driving_mode["descr"]


def test_longitudinal_pid_defaults_match_registry(params):
  by_name = {p["name"]: p for p in params}
  assert tuple(by_name[name]["default"] for name in (
    "LongTuningKpV", "LongTuningKiV", "LongTuningKf",
  )) == (100, 0, 100)

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  for name, default in (
    ("LongTuningKpV", 100),
    ("LongTuningKiV", 0),
    ("LongTuningKf", 100),
  ):
    assert f'{{"{name}", {{PERSISTENT, INT, "{default}"}}}}' in params_keys


def test_c3x_lite_hardware_setting_is_exposed(settings, params):
  by_name = {p["name"]: p for p in params}
  c3x_lite = by_name["HardwareC3xLite"]
  assert (c3x_lite["min"], c3x_lite["max"], c3x_lite["default"]) == (0, 1, 0)
  assert c3x_lite["control"] == "toggle"
  assert c3x_lite["risk"] == "high"

  vehicle = next(category for category in settings["menu"] if category["id"] == "VEHICLE")
  device_hardware = next(group for group in vehicle["groups"] if group["id"] == "VEH_DEVICE")
  assert device_hardware["params"] == ["HardwareC3xLite"]


def test_wide_camera_fallback_setting_is_exposed(settings, params):
  by_name = {p["name"]: p for p in params}
  use_wide_camera = by_name["UseWideCamera"]
  assert (use_wide_camera["min"], use_wide_camera["max"], use_wide_camera["default"]) == (0, 1, 1)
  assert use_wide_camera["control"] == "toggle"
  assert use_wide_camera["risk"] == "high"
  assert "재부팅" in use_wide_camera["descr"]
  assert "Reboot" in use_wide_camera["edescr"]

  system = next(category for category in settings["menu"] if category["id"] == "SYSTEM")
  camera = next(group for group in system["groups"] if group["id"] == "SYS_CAMERA")
  assert camera["params"] == ["UseWideCamera"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"UseWideCamera", {PERSISTENT, BOOL, "1"}}' in params_keys


def test_vehicle_navi_can_control_is_opt_in(settings, params):
  by_name = {p["name"]: p for p in params}
  control = by_name["VehicleNaviCanControl"]
  assert (control["min"], control["max"], control["default"]) == (0, 1, 0)
  assert control["control"] == "toggle"
  assert control["risk"] == "high"
  assert "PV5에서는 일반 과속카메라와 방지턱만 지원" in control["descr"]
  assert "average-speed zones are not yet supported" in control["edescr"]

  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  camera = next(group for group in speed["groups"] if group["id"] == "SPEED_CAMERA")
  assert "VehicleNaviCanControl" in camera["params"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"VehicleNaviCanControl", {PERSISTENT, BOOL, "0"}}' in params_keys


def test_vehicle_navi_school_zone_control_is_opt_in(settings, params):
  by_name = {p["name"]: p for p in params}
  control = by_name["VehicleNaviSchoolZoneControl"]
  assert (control["min"], control["max"], control["default"]) == (0, 1, 0)
  assert control["control"] == "toggle"
  assert control["risk"] == "high"
  assert "PV5에서는 아직 동작하지 않습니다" in control["descr"]
  assert "not yet supported on the PV5" in control["edescr"]

  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  camera = next(group for group in speed["groups"] if group["id"] == "SPEED_CAMERA")
  assert "VehicleNaviSchoolZoneControl" in camera["params"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"VehicleNaviSchoolZoneControl", {PERSISTENT, BOOL, "0"}}' in params_keys


def test_vehicle_navi_curve_control_settings_are_removed(settings):
  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  curve = next(group for group in speed["groups"] if group["id"] == "SPEED_CURVE")
  assert not any(name.startswith("VehicleNaviCurve") for name in curve["params"])
  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert "VehicleNaviCurve" not in params_keys


def test_tpms_position_setting_matches_device_support(params):
  by_name = {p["name"]: p for p in params}
  show_tpms = by_name["ShowTpms"]
  assert (show_tpms["min"], show_tpms["max"], show_tpms["default"]) == (0, 3, 1)
  assert "C4(MICI)는 해당 없음" in show_tpms["descr"]
  assert "Not applicable to C4 (MICI)" in show_tpms["edescr"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"ShowTpms", {PERSISTENT, INT, "1"}}' in params_keys


def test_external_hud_brightness_and_orientation_use_catalog_controls(settings, params):
  by_name = {p["name"]: p for p in params}
  brightness = by_name["ClusterHudBrightness"]
  assert (brightness["min"], brightness["max"], brightness["default"]) == (0, 100, 0)
  assert "live_update" not in brightness

  orientation = by_name["ClusterHudOrientation"]
  assert (orientation["min"], orientation["max"], orientation["default"]) == (0, 3, 0)
  assert orientation["control"] == "select"
  assert orientation["descr"].endswith("0: 0도\n1: X\n2: 180도\n3: X")
  assert orientation["options"]["en"] == [
    "0 degrees",
    "Unsupported",
    "180 degrees",
    "Unsupported",
  ]
  panel_layout = by_name["ClusterHudPanelLayout"]
  assert (panel_layout["min"], panel_layout["max"], panel_layout["default"]) == (0, 1, 0)
  assert panel_layout["control"] == "select"
  assert panel_layout["options"]["en"] == [
    "Driving left / info right",
    "Info left / driving right",
  ]
  screen_mode = by_name["ClusterHudScreenMode"]
  assert (screen_mode["min"], screen_mode["max"], screen_mode["default"]) == (-1, 5, 0)
  assert screen_mode["descr"].startswith("-1: 3D 전체화면\n0: 기본(내비/주행 리포트)")
  assert screen_mode["edescr"].startswith("-1: 3D fullscreen\n0: Default (navigation/driving report)")
  assert screen_mode["cdescr"].startswith("-1: 3D全屏\n0: 默认(导航/驾驶报告)")

  display = next(category for category in settings["menu"] if category["id"] == "DISPLAY")
  hud = next(group for group in display["groups"] if group["id"] == "DISP_HUD")
  basic = next(group for group in hud["groups"] if group["id"] == "HUD_BASIC")
  screen = next(group for group in hud["groups"] if group["id"] == "HUD_SCREEN")
  assert basic["params"][:3] == [
    "ClusterHud",
    "ClusterHudBrightness",
    "ClusterHudOrientation",
  ]
  assert screen["params"] == [
    "ClusterHudEncoder",
    "ClusterHudLiveFps",
    "ClusterHudScreenMode",
    "ClusterHudPanelLayout",
    "ClusterHudCameraViewMode",
  ]
  camera_view = by_name["ClusterHudCameraViewMode"]
  assert (camera_view["min"], camera_view["max"], camera_view["default"]) == (0, 4, 0)
  assert camera_view["control"] == "select"
  assert camera_view["options"]["ko"][2:] == ["일반 카메라", "광각 카메라", "속도 자동"]
  assert camera_view["options"]["en"][2:] == ["Narrow camera", "Wide camera", "Speed automatic"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"ClusterHudBrightness", {PERSISTENT, INT, "0"}}' in params_keys
  assert '{"ClusterHudOrientation", {PERSISTENT, INT, "0"}}' in params_keys
  assert '{"ClusterHudPanelLayout", {PERSISTENT, INT, "0"}}' in params_keys


def test_cluster_camera_preference_is_in_brightness_and_view(settings, params):
  by_name = {p["name"]: p for p in params}
  camera = by_name["ShowCameraWithCluster"]
  assert (camera["min"], camera["max"], camera["default"], camera["unit"]) == (0, 1, 0, 1)
  assert camera["descr"] == "0: 카메라 미표시(기본)\n1: 카메라 영상 표시"
  assert camera["edescr"] == "0: Hide camera (default)\n1: Show camera video"
  assert camera["cdescr"] == "0: 不显示摄像头（默认）\n1: 显示摄像头画面"

  display = next(category for category in settings["menu"] if category["id"] == "DISPLAY")
  brightness = next(group for group in display["groups"] if group["id"] == "DISP_BRIGHT")
  assert brightness["params"] == [
    "ShowCustomBrightness",
    "ShowModelView",
    "ShowCameraWithCluster",
  ]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"ShowCameraWithCluster", {PERSISTENT, INT, "0"}}' in params_keys


def test_carrot_radar_is_fixed_without_mode_or_sensitivity(settings, params):
  by_name = {p["name"]: p for p in params}
  for removed_name in (
    "RadarLeadModelMode",
    "RadarDPathMode",
    "RadarMotionMode",
    "CarrotRadarMode",
    "CarrotRadarCutInSensitivity",
  ):
    assert removed_name not in by_name
  vehicle = next(category for category in settings["menu"] if category["id"] == "VEHICLE")
  radar = next(group for group in vehicle["groups"] if group["id"] == "VEH_RADAR")
  assert radar["params"] == [
    "EnableRadarTracks",
    "EnableCornerRadar",
  ]


def test_parameter_names_are_unique(params):
  names = [p["name"] for p in params]
  assert len(names) == len(set(names))


def test_every_declared_display_unit_is_one_the_ui_knows(params):
  declared = {p["name"]: p["display_unit"] for p in params if "display_unit" in p}
  assert declared, "the migration should have left declarations behind"
  unknown = {name: unit for name, unit in declared.items() if unit not in KNOWN_DISPLAY_UNITS}
  assert unknown == {}


def test_every_declared_control_is_one_the_ui_knows(params):
  declared = {p["name"]: p["control"] for p in params if "control" in p}
  assert declared
  unknown = {name: kind for name, kind in declared.items() if kind not in KNOWN_CONTROL_KINDS}
  assert unknown == {}


# "raw" is the absence of a unit, so declaring it only adds noise.
def test_no_parameter_declares_the_empty_unit(params):
  assert [p["name"] for p in params if p.get("display_unit") == "raw"] == []


def test_a_percent_parameter_still_carries_its_unit(params):
  by_name = {p["name"]: p for p in params}
  # The parameter behind the cruise-speed incident; its unit must survive.
  assert by_name["ApplyModelSpeed"]["display_unit"] == "percent"
  assert by_name["TFollowDecelBoost"]["display_unit"] == "percent"


def test_steer_ratio_rate_has_safe_catalog_and_registry_defaults(params):
  by_name = {p["name"]: p for p in params}
  steer_ratio_rate = by_name["SteerRatioRate"]
  assert (steer_ratio_rate["min"], steer_ratio_rate["max"], steer_ratio_rate["default"]) == (30, 200, 100)
  assert steer_ratio_rate["display_unit"] == "percent"

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"SteerRatioRate", {PERSISTENT, INT, "100"}}' in params_keys


def test_control_overrides_land_on_parameters_that_need_them(params):
  by_name = {p["name"]: p for p in params}
  # Range inference would make these sliders, which is why they are pinned.
  assert by_name["ShowPathMode"]["control"] == "select"
  assert by_name["SoundLanguageSetting"]["control"] == "select"


def test_declared_options_cover_every_value_in_range(params):
  """Labels are indexed from `min`, so a short list silently blanks a choice."""
  declared = {p["name"]: p for p in params if "options" in p}
  assert declared

  for name, p in declared.items():
    span = p["max"] - p["min"] + 1
    for language, labels in p["options"].items():
      assert language in {"ko", "en", "zh"}, f"{name}: unexpected language {language}"
      assert isinstance(labels, list), f"{name}.{language} must be a list"
      assert len(labels) == span, f"{name}.{language} has {len(labels)} labels for {span} values"
      assert all(isinstance(label, str) and label for label in labels), f"{name}.{language} has an empty label"


def test_option_labels_exist_in_every_supported_language(params):
  for p in (p for p in params if "options" in p):
    assert set(p["options"]) == {"ko", "en", "zh"}, p["name"]


def test_every_declared_risk_is_one_the_ui_knows(params):
  declared = {p["name"]: p["risk"] for p in params if "risk" in p}
  unknown = {name: level for name, level in declared.items() if level not in KNOWN_RISK_LEVELS}
  assert unknown == {}


def test_the_incident_parameter_is_flagged_risky(params):
  # ApplyModelSpeed at -1 pins cruise to 5 km/h — the parameter this whole
  # effort started from. It stays marked so its warning badge shows.
  by_name = {p["name"]: p for p in params}
  assert by_name["ApplyModelSpeed"].get("risk") == "high"


def test_declared_numeric_bounds_stay_intact(params):
  """The migration rewrote the whole file, so the values it did not touch
  have to be exactly what they were."""
  by_name = {p["name"]: p for p in params}
  assert (by_name["ApplyModelSpeed"]["min"], by_name["ApplyModelSpeed"]["max"]) == (-120, 120)
  assert by_name["ApplyModelSpeed"]["default"] == 0
  assert by_name["TFollowDecelBoost"]["unit"] == 10
