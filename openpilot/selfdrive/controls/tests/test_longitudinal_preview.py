import pytest

from openpilot.selfdrive.controls.lib.longitudinal_preview import (
  DRIVING_MODE_ECO,
  DRIVING_MODE_HIGH,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_SAFE,
  apply_preview_target,
  clip_action_time,
  clip_preview_offset,
  get_lead_preview_request,
  rate_limit_preview,
)


def request(mode, a_lead, a_ego=0.0):
  return get_lead_preview_request(
    mode,
    lead_status=True,
    a_lead=a_lead,
    a_ego=a_ego,
  )


def test_safe_preview_changes_sign_with_lead_acceleration():
  assert request(DRIVING_MODE_SAFE, 0.5).offset_s == pytest.approx(-0.20)
  assert request(DRIVING_MODE_SAFE, -0.5).offset_s == pytest.approx(0.40)


def test_eco_delays_acceleration_side_but_keeps_braking_response():
  assert request(DRIVING_MODE_ECO, 0.5).offset_s == pytest.approx(-0.15)
  assert request(DRIVING_MODE_ECO, -0.4).offset_s == pytest.approx(0.30)


def test_normal_does_not_preview_positive_lead_acceleration():
  assert request(DRIVING_MODE_NORMAL, 1.0).offset_s == 0.0
  assert request(DRIVING_MODE_NORMAL, -0.5).offset_s == pytest.approx(0.40)


def test_high_previews_stable_acceleration_forward():
  assert request(DRIVING_MODE_HIGH, 0.5).offset_s == pytest.approx(0.10)
  assert request(DRIVING_MODE_HIGH, -0.5).offset_s == pytest.approx(0.40)


@pytest.mark.parametrize(("mode", "preview_max"), [
  (DRIVING_MODE_SAFE, 1.50),
  (DRIVING_MODE_ECO, 1.50),
  (DRIVING_MODE_NORMAL, 1.50),
  (DRIVING_MODE_HIGH, 1.50),
])
def test_braking_preview_is_capped_by_mode(mode, preview_max):
  assert request(mode, -10.0).offset_s == pytest.approx(preview_max)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_all_modes_use_relative_acceleration_for_lead_deceleration(mode):
  result = request(mode, 0.0, a_ego=0.8)
  assert result.lead_accel_signal == pytest.approx(-0.7)
  assert result.offset_s == pytest.approx(0.7)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_all_modes_release_preview_when_ego_matches_lead_deceleration(mode):
  result = request(mode, -0.5, a_ego=-0.5)
  assert result.lead_accel_signal == 0.0
  assert result.offset_s == 0.0


def test_preview_is_disabled_for_invalid_or_missing_lead():
  assert not get_lead_preview_request(
    DRIVING_MODE_SAFE, lead_status=False, a_lead=-1.0,
  ).active
  assert not get_lead_preview_request(
    DRIVING_MODE_SAFE, lead_status=True, a_lead=float("nan"),
  ).active
  assert not get_lead_preview_request(
    DRIVING_MODE_SAFE, lead_status=True, a_lead=-1.0, a_ego=float("nan"),
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


def test_negative_preview_cannot_cross_zero_actuator_delay():
  long_actuator_delay = 0.15
  base_action_t = long_actuator_delay + 0.05
  effective_preview = clip_preview_offset(base_action_t, -1.0)
  assert effective_preview == pytest.approx(-long_actuator_delay)
  assert long_actuator_delay + effective_preview == pytest.approx(0.0)


def test_preview_only_removes_acceleration_with_bounded_prebraking():
  assert apply_preview_target(0.20, 0.40, DRIVING_MODE_SAFE, -0.5) == pytest.approx(0.20)
  assert apply_preview_target(0.20, -0.20, DRIVING_MODE_SAFE, -0.5) == pytest.approx(-0.05)
  assert apply_preview_target(0.02, -0.20, DRIVING_MODE_SAFE, -0.5) == pytest.approx(-0.05)
  assert apply_preview_target(0.08, -0.20, DRIVING_MODE_ECO, -0.5) == pytest.approx(-0.04)
  assert apply_preview_target(-0.50, -1.0, DRIVING_MODE_NORMAL, -0.5) == pytest.approx(-0.58)


@pytest.mark.parametrize(("mode", "floor"), [
  (DRIVING_MODE_SAFE, -0.05),
  (DRIVING_MODE_ECO, -0.04),
  (DRIVING_MODE_NORMAL, -0.03),
  (DRIVING_MODE_HIGH, -0.03),
])
def test_all_modes_can_release_positive_acceleration_to_coast(mode, floor):
  assert apply_preview_target(0.70, 0.20, mode, -1.0) == pytest.approx(0.20)
  assert apply_preview_target(0.70, -0.20, mode, -1.0) == pytest.approx(floor)


def test_normal_reacts_to_speed_bump_lead_deceleration():
  request_normal = request(DRIVING_MODE_NORMAL, a_lead=-1.36, a_ego=0.70)
  assert request_normal.offset_s == pytest.approx(1.50)
  assert apply_preview_target(0.70, 0.22, DRIVING_MODE_NORMAL, request_normal.lead_accel_signal) == pytest.approx(0.22)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_future_accel_recovery_cannot_release_current_braking(mode):
  assert apply_preview_target(-0.60, -0.10, mode, -0.5, a_ego=-0.4) == pytest.approx(-0.60)


def test_positive_lead_acceleration_never_requests_prebraking():
  assert apply_preview_target(0.08, -0.20, DRIVING_MODE_SAFE, 0.5) == pytest.approx(0.0)


def test_high_acceleration_preview_has_a_bounded_gain():
  assert apply_preview_target(0.20, 0.50, DRIVING_MODE_HIGH, 0.5) == pytest.approx(0.35)
  assert apply_preview_target(0.20, 0.10, DRIVING_MODE_HIGH, 0.5) == pytest.approx(0.20)


def test_high_acceleration_preview_is_disabled_while_decelerating():
  assert apply_preview_target(-0.50, -0.10, DRIVING_MODE_HIGH, 0.5, a_ego=0.0) == pytest.approx(-0.50)
  assert apply_preview_target(0.20, 0.50, DRIVING_MODE_HIGH, 0.5, a_ego=-0.3) == pytest.approx(0.20)
