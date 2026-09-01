import pytest

from openpilot.selfdrive.controls.lib.longitudinal_preview import (
  DRIVING_MODE_ECO,
  DRIVING_MODE_HIGH,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_SAFE,
  apply_preview_target,
  clip_action_time,
  get_lead_preview_request,
  rate_limit_preview,
)


def request(mode, a_lead, j_lead=0.0):
  return get_lead_preview_request(
    mode,
    lead_status=True,
    a_lead=a_lead,
    j_lead=j_lead,
  )


def test_safe_preview_changes_sign_with_lead_acceleration():
  assert request(DRIVING_MODE_SAFE, 0.5).offset_s == pytest.approx(-0.10)
  assert request(DRIVING_MODE_SAFE, -0.5).offset_s == pytest.approx(0.30)


def test_eco_halves_acceleration_side_but_keeps_braking_response():
  assert request(DRIVING_MODE_ECO, 0.5).offset_s == pytest.approx(-0.06)
  assert request(DRIVING_MODE_ECO, -0.4).offset_s == pytest.approx(0.18)


def test_normal_does_not_preview_positive_lead_acceleration():
  assert request(DRIVING_MODE_NORMAL, 1.0).offset_s == 0.0
  assert request(DRIVING_MODE_NORMAL, -0.5).offset_s == pytest.approx(0.14)


def test_high_previews_stable_acceleration_forward():
  assert request(DRIVING_MODE_HIGH, 0.5).offset_s == pytest.approx(0.10)
  assert request(DRIVING_MODE_HIGH, -0.5).offset_s == pytest.approx(0.14)


@pytest.mark.parametrize(("mode", "preview_max"), [
  (DRIVING_MODE_SAFE, 0.60),
  (DRIVING_MODE_ECO, 0.45),
  (DRIVING_MODE_NORMAL, 0.25),
  (DRIVING_MODE_HIGH, 0.25),
])
def test_braking_preview_is_capped_by_mode(mode, preview_max):
  assert request(mode, -10.0).offset_s == pytest.approx(preview_max)


def test_jerk_is_converted_to_short_horizon_lead_acceleration():
  result = request(DRIVING_MODE_SAFE, 0.0, -1.0)
  assert result.lead_accel_signal == pytest.approx(-0.15)
  assert result.offset_s == pytest.approx(0.1125)


def test_preview_is_disabled_for_invalid_or_missing_lead():
  assert not get_lead_preview_request(
    DRIVING_MODE_SAFE, lead_status=False, a_lead=-1.0, j_lead=-1.0,
  ).active
  assert not get_lead_preview_request(
    DRIVING_MODE_SAFE, lead_status=True, a_lead=float("nan"), j_lead=0.0,
  ).active


def test_preview_rate_and_action_time_are_bounded():
  assert rate_limit_preview(0.60, 0.0) == pytest.approx(0.08)
  assert rate_limit_preview(0.0, 0.20) == pytest.approx(0.17)
  assert rate_limit_preview(-0.10, 0.0) == pytest.approx(-0.03)
  assert rate_limit_preview(-0.10, 0.03) == pytest.approx(0.0)
  assert clip_action_time(2.40, 1.0) == pytest.approx(2.50)
  assert clip_action_time(0.20, -1.0) == pytest.approx(0.05)


def test_zero_preview_preserves_configured_actuator_delay():
  assert clip_action_time(2.05, 0.0) == pytest.approx(2.05)


def test_comfort_modes_only_remove_acceleration_with_bounded_prebraking():
  assert apply_preview_target(0.20, 0.40, DRIVING_MODE_SAFE, -0.5) == pytest.approx(0.20)
  assert apply_preview_target(0.20, -0.20, DRIVING_MODE_SAFE, -0.5) == pytest.approx(0.02)
  assert apply_preview_target(0.02, -0.20, DRIVING_MODE_SAFE, -0.5) == pytest.approx(-0.05)
  assert apply_preview_target(0.08, -0.20, DRIVING_MODE_ECO, -0.5) == pytest.approx(-0.04)
  assert apply_preview_target(-0.50, -1.0, DRIVING_MODE_NORMAL, -0.5) == pytest.approx(-0.58)


def test_positive_lead_acceleration_never_requests_prebraking():
  assert apply_preview_target(0.08, -0.20, DRIVING_MODE_SAFE, 0.5) == pytest.approx(0.0)


def test_high_acceleration_preview_has_a_bounded_gain():
  assert apply_preview_target(0.20, 0.50, DRIVING_MODE_HIGH, 0.5) == pytest.approx(0.35)
  assert apply_preview_target(0.20, 0.10, DRIVING_MODE_HIGH, 0.5) == pytest.approx(0.20)
