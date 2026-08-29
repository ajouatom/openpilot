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
  live_route_parser,
  standby_state,
)
import cluster_live
from cluster_models import ClusterAlert


@pytest.mark.parametrize(
  ("source", "expected"),
  (
    ("cam", "cam"),
    ("hda", "cam"),
    ("hda_section", "section"),
    ("hda_bump", "bump"),
    ("school", "school"),
    ("route", "route"),
    ("vturn", "vturn"),
    ("model", "model"),
    ("atc", "turn"),
    ("section", "section"),
    ("longsource:c", "longsour"),
    ("custom-source", "custom-s"),
    (None, "apply"),
  ),
)
def test_deceleration_source_display_label(source, expected) -> None:
  assert deceleration_source_display_label(source) == expected


@pytest.mark.parametrize(
  ("desired_source", "expected_label", "expected_mode"),
  (
    ("cam", "cam", 4),
    ("hda", "cam", 3),
    ("hda_section", "section", 3),
    ("hda_bump", "bump", 3),
    ("school", "school", 3),
    ("route", "route", 4),
    ("model", "model", 2),
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


def test_vehicle_navigation_deceleration_presents_actual_reason() -> None:
  assert deceleration_source_presentation("hda") == ("cam", 3)
  assert deceleration_source_presentation("hda_section") == ("section", 3)
  assert deceleration_source_presentation("hda_bump") == ("bump", 3)
  assert deceleration_source_presentation("school") == ("school", 3)


def test_vehicle_navigation_availability_does_not_force_speed_with_cruise_off() -> None:
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
    vehicleNaviAvailable=True,
  )
  source._service_data = lambda service: carrot_man if service == "carrotMan" else None
  source._service_alive = lambda _service: False
  source._service_valid = lambda _service: False

  decorated = source._with_live_hud_state(standby_state())

  assert decorated.cruise_display_state == "off"
  assert decorated.cruise_override_kph is None
  assert decorated.cruise_override_label is None
  assert decorated.cruise_override_color_mode == 0
  assert decorated.vehicle_navi_available


def test_external_navigation_deceleration_presents_actual_reason() -> None:
  assert deceleration_source_presentation("cam") == ("cam", 4)
  assert deceleration_source_presentation("bump") == ("bump", 4)
  assert deceleration_source_presentation("route") == ("route", 4)


@pytest.mark.parametrize(
  ("active_carrot", "desired_source", "legacy_remote", "carrot_navi_connected", "expected"),
  (
    (5, "hda_bump", "", False, False),
    (6, "school", "", False, False),
    (3, "cam", "", False, False),
    (0, "none", "192.168.0.2", False, True),
    (5, "hda_bump", "192.168.0.2", False, True),
    (0, "none", "", True, True),
  ),
)
def test_external_navigation_connection_does_not_follow_speed_control_state(
  active_carrot, desired_source, legacy_remote, carrot_navi_connected, expected,
) -> None:
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  source._current_carrot_navi = lambda _now: None
  carrot_man = SimpleNamespace(
    activeCarrot=active_carrot,
    desiredSpeed=55.0,
    desiredSource=desired_source,
    remote=legacy_remote,
  )
  carrot_navi = SimpleNamespace(connected=carrot_navi_connected)
  source._service_data = lambda service: carrot_man if service == "carrotMan" else carrot_navi if service == "carrotNavi" else None
  source._service_alive = lambda service: carrot_navi_connected if service == "carrotNavi" else False
  source._service_valid = lambda service: carrot_navi_connected if service == "carrotNavi" else False

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


def test_live_cluster_uses_authoritative_recorded_cutin_without_recomputation() -> None:
  parser = live_route_parser()

  def fail_if_called(*_args) -> None:
    raise AssertionError("live cluster must not recompute CUT-IN from liveTracks")

  parser._update_offline_cutin = fail_if_called
  parser._update_live_tracks(SimpleNamespace(points=()), 1.0)

  assert not parser.recompute_cutins


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
