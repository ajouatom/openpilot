from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_renderer
from cluster_display import (
  cluster_text,
  display_speed,
  format_navi_distance,
  format_radar_distance,
  format_trip_distance,
  normalize_cluster_language,
  normalize_metric_setting,
  speed_unit,
)
from cluster_renderer import ClusterUiRenderer
from main import ClusterDisplayPreferencesParamReader


@pytest.mark.parametrize(
  ("value", "expected"),
  (
    ("ko", "ko"),
    ("main_ko", "ko"),
    ("kr", "ko"),
    (b"en", "en"),
    ("main_en", "en"),
    ("zh-CHT", "en"),
    ("unknown", "en"),
  ),
)
def test_cluster_language_supports_korean_and_english_only(value, expected):
  assert normalize_cluster_language(value) == expected


def test_metric_and_imperial_display_conversions():
  assert normalize_metric_setting(None) is True
  assert normalize_metric_setting(b"1") is True
  assert normalize_metric_setting("0") is False
  assert display_speed(100.0, True) == 100.0
  assert display_speed(100.0, False) == pytest.approx(62.1371192)
  assert speed_unit(True) == "km/h"
  assert speed_unit(False) == "mph"
  assert format_radar_distance(100.0, True) == "100 m"
  assert format_radar_distance(100.0, False) == "328 ft"
  assert format_navi_distance(50.0, False) == "164 ft"
  assert format_navi_distance(1609.344, False) == "1.0 mi"
  assert format_trip_distance(1609.344, False) == "1.00 mi"


def test_cluster_text_falls_back_to_english_for_unsupported_languages():
  assert cluster_text("ko", "driving_report") == "주행리포트"
  assert cluster_text("en", "driving_report") == "DRIVING REPORT"
  assert cluster_text("zh-CHT", "driving_report") == "DRIVING REPORT"


def test_display_preference_reader_uses_param_defaults_and_values():
  class FakeParams:
    def __init__(self, values):
      self.values = values

    def get(self, name, return_default=False):
      assert return_default
      return self.values.get(name)

  reader = object.__new__(ClusterDisplayPreferencesParamReader)
  reader._params = FakeParams({"LanguageSetting": "main_ko", "IsMetric": b"0"})
  assert reader.read() == ("ko", False)

  reader._params = FakeParams({})
  assert reader.read() == ("ko", True)


def test_trip_report_draws_english_imperial_labels(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.language = "en"
  renderer.is_metric = False
  renderer._system_stats = SimpleNamespace(sample=lambda: SimpleNamespace(
    cpu_used_percent=20.0,
    memory_used_percent=30.0,
    disk_used_percent=40.0,
  ))
  text_draws = []
  metrics = []
  gauges = []
  angles = []
  renderer._rounded_rect = lambda *_args, **_kwargs: None
  renderer._draw_text = lambda *args, **kwargs: text_draws.append((args, kwargs))
  renderer._draw_trip_metric = lambda *args, **kwargs: metrics.append((args, kwargs))
  renderer._draw_system_gauge = lambda *args, **kwargs: gauges.append((args, kwargs))
  renderer._draw_device_angle_indicator = lambda *args, **kwargs: angles.append((args, kwargs))
  renderer._system_metric_color = lambda _value: (1, 2, 3)
  monkeypatch.setattr(cluster_renderer.rl, "draw_line_ex", lambda *_args, **_kwargs: None)

  report = SimpleNamespace(
    duration_s=65.0,
    distance_m=1609.344,
    average_speed_kph=100.0,
    max_speed_kph=120.0,
    auto_ratio_percent=95.0,
    max_accel_mps2=1.23,
    max_decel_mps2=-2.34,
    hard_accel_count=1,
    hard_brake_count=2,
    hard_corner_count=3,
  )
  state = SimpleNamespace(
    trip_report=report,
    cpu_usage_percent=25.0,
    memory_used_percent=35.0,
    disk_used_percent=45.0,
    cpu_temp_c=62.0,
    camera_calibration_euler=(0.0, 0.01, -0.02),
  )

  renderer._draw_trip_report_panel(state)

  drawn_text = {args[0] for args, _kwargs in text_draws}
  assert {"TRIP SUMMARY", "SYSTEM", "DEVICE ANGLE"} <= drawn_text
  assert "DRIVING REPORT" not in drawn_text
  assert {"HARD ACCEL", "HARD BRAKE", "HARD CORNER"} <= drawn_text
  assert metrics[0][0][2:4] == ("DISTANCE", "1.00 mi")
  assert metrics[1][0][2:4] == ("AVG SPEED", "62.1")
  assert metrics[1][0][5] == "mph"
  assert metrics[2][0][2:4] == ("MAX SPEED", "74.6")
  assert metrics[2][0][5] == "mph"
  assert [gauge[0][2] for gauge in gauges] == ["CPU", "TEMP", "MEM", "DISK"]
  assert angles[0][0][4:6] == pytest.approx((0.5729578, -1.1459156))


def test_device_angle_target_maps_pitch_and_yaw_to_expected_axes():
  radius = 30.0

  assert ClusterUiRenderer._device_angle_target_offset(3.0, 2.0, radius) == pytest.approx((-6.8, 10.2))
  assert ClusterUiRenderer._device_angle_target_offset(-3.0, -2.0, radius) == pytest.approx((6.8, -10.2))
  assert ClusterUiRenderer._device_angle_target_offset(20.0, -20.0, radius) == pytest.approx((20.4, 20.4))
