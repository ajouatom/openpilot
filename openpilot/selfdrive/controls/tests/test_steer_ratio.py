import math

import pytest

from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio


@pytest.mark.parametrize("invalid_rate", [-math.inf, math.nan, 0.0, 29.0, 201.0, math.inf])
def test_invalid_steer_ratio_rate_uses_full_live_ratio(invalid_rate):
  assert resolve_vehicle_model_steer_ratio(14.75, invalid_rate, 0.0, False) == pytest.approx(14.75)


@pytest.mark.parametrize(("rate", "expected"), [(30.0, 4.425), (80.0, 11.8), (100.0, 14.75), (200.0, 29.5)])
def test_valid_steer_ratio_rate_scales_live_ratio(rate, expected):
  assert resolve_vehicle_model_steer_ratio(14.75, rate, 0.0, False) == pytest.approx(expected)


def test_custom_steer_ratio_overrides_live_ratio_rate():
  assert resolve_vehicle_model_steer_ratio(14.75, 80.0, 150.0, False) == pytest.approx(15.0)


def test_vw_meb_always_uses_live_ratio():
  assert resolve_vehicle_model_steer_ratio(14.75, 0.0, 150.0, True) == pytest.approx(14.75)
