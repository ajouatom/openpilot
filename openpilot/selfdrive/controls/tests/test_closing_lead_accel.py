from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.closing_lead_accel import (
  CLOSING_LEAD_MAX_ACCEL_STEP,
  apply_closing_lead_accel_limit,
  get_closing_lead_accel_limit,
)


def lead(d_rel=80.0, v_lead=24.0, a_lead=0.0, status=True):
  return SimpleNamespace(status=status, dRel=d_rel, vLead=v_lead, aLeadK=a_lead)


def test_no_limit_without_a_closing_lead():
  assert get_closing_lead_accel_limit(lead(status=False), 27.0, 1.0) is None
  assert get_closing_lead_accel_limit(lead(v_lead=27.0), 27.0, 1.0) is None  # same speed
  assert get_closing_lead_accel_limit(lead(v_lead=26.7), 27.0, 1.0) is None  # 0.3 m/s: below threshold
  assert get_closing_lead_accel_limit(lead(d_rel=180.0), 27.0, 1.0) is None  # too far
  assert get_closing_lead_accel_limit(lead(v_lead=float("nan")), 27.0, 1.0) is None


def test_ceiling_fades_to_zero_with_closing_speed():
  # 2026-10-03 case: 96 km/h toward an 88 km/h lead (2.2 m/s closing) -> no acceleration.
  assert get_closing_lead_accel_limit(lead(v_lead=24.4), 26.7, 1.0) == 0.0
  half = get_closing_lead_accel_limit(lead(v_lead=25.75), 27.0, 1.0)  # 1.25 m/s closing
  assert half == pytest.approx(0.5)


def test_accelerating_lead_is_not_treated_as_slower():
  assert get_closing_lead_accel_limit(lead(v_lead=25.0, a_lead=1.0), 27.0, 1.0) is None


def test_ceiling_never_requests_braking_and_ramps_down():
  assert get_closing_lead_accel_limit(lead(v_lead=10.0), 27.0, 1.0) == 0.0
  # The ceiling falls from the current plan by one step at a time.
  assert apply_closing_lead_accel_limit(1.0, 0.8, 0.0) == pytest.approx(0.8 - CLOSING_LEAD_MAX_ACCEL_STEP)
  assert apply_closing_lead_accel_limit(1.0, 0.02, 0.0) == 0.0
  assert apply_closing_lead_accel_limit(1.0, -0.5, 0.0) == 0.0
  # It never raises the normal ceiling, and does nothing without a limit.
  assert apply_closing_lead_accel_limit(0.3, 0.8, 0.0) == 0.3
  assert apply_closing_lead_accel_limit(0.7, 0.2, None) == 0.7
