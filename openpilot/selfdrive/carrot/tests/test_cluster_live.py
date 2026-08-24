from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_live import (
  LIVE_SERVICES_BASE,
  OpenpilotLiveSource,
  deceleration_source_presentation,
  deceleration_source_display_label,
  standby_state,
)
import cluster_live
from cluster_models import ClusterAlert


@pytest.mark.parametrize(
  ("source", "expected"),
  (
    ("cam", "NAVI"),
    ("hda", "vNAVI"),
    ("hda_section", "vNAVI"),
    ("hda_bump", "vNAVI"),
    ("school", "vNAVI"),
    ("route", "NAVI"),
    ("vturn", "turn:c"),
    ("model", "turn:c"),
    ("atc", "NAVI"),
    ("section", "NAVI"),
    ("longsource:c", "longsource:c"),
    ("custom-source", "custom-s"),
    (None, "apply"),
  ),
)
def test_deceleration_source_display_label(source, expected) -> None:
  assert deceleration_source_display_label(source) == expected


@pytest.mark.parametrize(
  ("desired_source", "expected_label", "expected_mode"),
  (
    ("cam", "NAVI", 4),
    ("hda", "vNAVI", 3),
    ("hda_section", "vNAVI", 3),
    ("hda_bump", "vNAVI", 3),
    ("school", "vNAVI", 3),
    ("route", "NAVI", 4),
    ("model", "turn:c", 2),
  ),
)
def test_live_deceleration_override_presents_navigation_origin(desired_source, expected_label, expected_mode) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  source._current_carrot_navi = lambda _now: None
  carrot_man = SimpleNamespace(activeCarrot=3, desiredSpeed=55.0, desiredSource=desired_source)
  source._service_data = lambda service: carrot_man if service == "carrotMan" else None
  source._service_alive = lambda _service: False
  state = replace(standby_state(), cruise_kph=100, cruise_display_state="engaged")

  decorated = source._with_live_hud_state(state)

  assert decorated.cruise_override_kph == 55.0
  assert decorated.cruise_override_label == expected_label
  assert decorated.cruise_override_color_mode == expected_mode


def test_live_hud_reads_active_egpu_param(monkeypatch) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  source._current_carrot_navi = lambda _now: None
  source.params = SimpleNamespace(get_bool=lambda key: key == "UsbGpuActive")
  source._service_data = lambda _service: None
  source._service_alive = lambda _service: False
  monkeypatch.setattr(cluster_live.time, "monotonic", lambda: 100.0)

  decorated = source._with_live_hud_state(standby_state())

  assert decorated.egpu_active


def test_vehicle_navigation_presentation_is_blue() -> None:
  assert deceleration_source_presentation("hda") == ("vNAVI", 3)
  assert deceleration_source_presentation("hda_section") == ("vNAVI", 3)
  assert deceleration_source_presentation("hda_bump") == ("vNAVI", 3)
  assert deceleration_source_presentation("school") == ("vNAVI", 3)


def test_vehicle_navigation_profile_displays_with_cruise_off() -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  source._current_carrot_navi = lambda _now: None
  carrot_man = SimpleNamespace(
    activeCarrot=0,
    desiredSpeed=250.0,
    desiredSource="none",
    vehicleNaviActive=True,
    vehicleNaviSpeed=105,
  )
  source._service_data = lambda service: carrot_man if service == "carrotMan" else None
  source._service_alive = lambda _service: False
  source._service_valid = lambda _service: False

  decorated = source._with_live_hud_state(standby_state())

  assert decorated.cruise_display_state == "off"
  assert decorated.cruise_override_kph == 105
  assert decorated.cruise_override_label == "vNAVI"
  assert decorated.cruise_override_color_mode == 3


def test_external_navigation_presentation_is_green() -> None:
  assert deceleration_source_presentation("cam") == ("NAVI", 4)
  assert deceleration_source_presentation("bump") == ("NAVI", 4)
  assert deceleration_source_presentation("route") == ("NAVI", 4)


@pytest.mark.parametrize(
  ("active_carrot", "desired_source", "guidance_active", "expected"),
  (
    (5, "hda_bump", False, False),
    (6, "school", False, False),
    (3, "hda", False, False),
    (3, "cam", False, True),
    (5, "bump", False, True),
    (5, "hda_bump", True, True),
  ),
)
def test_vehicle_navigation_does_not_replace_driving_report(
  active_carrot, desired_source, guidance_active, expected,
) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  navi_live = SimpleNamespace(current=object(), status=None, speed=None) if guidance_active else None
  source._current_carrot_navi = lambda _now: navi_live
  carrot_man = SimpleNamespace(activeCarrot=active_carrot, desiredSpeed=55.0, desiredSource=desired_source)
  source._service_data = lambda service: carrot_man if service == "carrotMan" else None
  source._service_alive = lambda _service: False
  source._service_valid = lambda _service: False

  decorated = source._with_live_hud_state(replace(standby_state(), cruise_kph=100, cruise_display_state="engaged"))

  assert decorated.external_nav_active is expected


@pytest.mark.parametrize(
  ("alive", "valid", "expected"),
  ((True, True, 2), (True, False, None), (False, True, None), (False, False, None)),
)
def test_live_driving_mode_requires_alive_valid_longitudinal_plan(alive, valid, expected) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  source._current_carrot_navi = lambda _now: None
  source._service_data = lambda _service: None
  source._service_alive = lambda service: alive if service == "longitudinalPlan" else False
  source._service_valid = lambda service: valid if service == "longitudinalPlan" else True

  decorated = source._with_live_hud_state(replace(standby_state(), driving_mode=2))

  assert decorated.driving_mode == expected


def test_live_cluster_avoids_unused_gps_subscriptions_after_trace_removal() -> None:
  assert "livePose" in LIVE_SERVICES_BASE
  assert "gpsLocationExternal" not in LIVE_SERVICES_BASE
  assert "gpsLocation" not in LIVE_SERVICES_BASE


def test_live_onroad_state_is_unknown_until_device_state_is_alive() -> None:
  source = object.__new__(OpenpilotLiveSource)
  device_state = SimpleNamespace(started=False)
  source._service_alive = lambda _service: False
  source._service_data = lambda _service: device_state
  assert source.onroad_state() is None

  source._service_alive = lambda service: service == "deviceState"
  assert source.onroad_state() is False
  device_state.started = True
  assert source.onroad_state() is True


def test_cached_calibration_is_loaded_for_installation_angle() -> None:
  calibration = SimpleNamespace(rpyCalib=(0.0, 0.02, -0.01))
  event = SimpleNamespace(valid=True, liveCalibration=calibration)
  calls = []
  source = object.__new__(OpenpilotLiveSource)
  source.params = SimpleNamespace(get=lambda key: b"calibration" if key == "CalibrationParams" else None)
  source.log = SimpleNamespace(Event=object())
  source.messaging = SimpleNamespace(log_from_bytes=lambda payload, schema: event)
  source.parser = SimpleNamespace(
    _update_live_calibration=lambda value, valid: calls.append((value, valid)),
  )

  source._load_cached_calibration()

  assert calls == [(calibration, True)]


def test_live_alert_reports_selfdrive_timeout_like_device_ui(monkeypatch) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._alert_onroad = True
  source._alert_onroad_started_t = 80.0
  source._selfdrive_seen_onroad = True
  source.sm = SimpleNamespace(recv_time={"selfdriveState": 94.0})
  source._service_updated = lambda _service: False
  source._service_data = lambda _service: SimpleNamespace(enabled=True)
  monkeypatch.setattr(cluster_live.time, "monotonic", lambda: 100.0)

  alert = source._live_cluster_alert(None, True)

  assert alert == ClusterAlert(
    "TAKE CONTROL IMMEDIATELY",
    "System Unresponsive",
    size=3,
    status=2,
    alert_type="clusterSelfdriveTimeout",
  )


def test_live_alert_reports_startup_wait_after_five_seconds(monkeypatch) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._alert_onroad = True
  source._alert_onroad_started_t = 90.0
  source._selfdrive_seen_onroad = False
  source.sm = SimpleNamespace(recv_time={})
  source._service_updated = lambda _service: False
  monkeypatch.setattr(cluster_live.time, "monotonic", lambda: 100.0)

  alert = source._live_cluster_alert(None, True)

  assert alert == ClusterAlert(
    "openpilot Unavailable",
    "Waiting to start",
    size=2,
    status=0,
    alert_type="clusterSelfdriveStartup",
  )
