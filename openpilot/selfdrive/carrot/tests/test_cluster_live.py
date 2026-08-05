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
  deceleration_source_display_label,
  standby_state,
)
import cluster_live
from cluster_models import ClusterAlert


@pytest.mark.parametrize(
  ("source", "expected"),
  (
    ("cam", "cam:n"),
    ("hda", "cam:v"),
    ("route", "route:v"),
    ("vturn", "turn:c"),
    ("model", "turn:c"),
    ("atc", "turn:n"),
    ("section", "section:n"),
    ("longsource:c", "longsource:c"),
    ("custom-source", "custom-s"),
    (None, "apply"),
  ),
)
def test_deceleration_source_display_label(source, expected) -> None:
  assert deceleration_source_display_label(source) == expected


@pytest.mark.parametrize(
  ("desired_source", "expected_label"),
  (
    ("cam", "cam:n"),
    ("hda", "cam:v"),
    ("route", "route:v"),
    ("model", "turn:c"),
  ),
)
def test_live_deceleration_override_adds_source_origin(desired_source, expected_label) -> None:
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
  assert decorated.cruise_override_color_mode == 2


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
