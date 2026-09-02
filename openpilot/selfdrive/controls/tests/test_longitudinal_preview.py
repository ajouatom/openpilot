import pytest

from openpilot.selfdrive.controls.lib.longitudinal_preview import (
  DRIVING_MODE_ECO,
  DRIVING_MODE_HIGH,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_SAFE,
  LEAD_ACCEL_RESPONSE_TUNING,
  apply_preview_target,
  clip_action_time,
  clip_preview_offset,
  get_lead_preview_request,
  lead_accel_response_allowed,
  rate_limit_accel_boost,
  rate_limit_preview,
)


def request(mode, a_lead, a_ego=0.0, **kwargs):
  return get_lead_preview_request(
    mode,
    lead_status=True,
    a_lead=a_lead,
    a_ego=a_ego,
    **kwargs,
  )


def test_positive_lead_acceleration_is_disabled_without_tf1_response():
  assert request(DRIVING_MODE_SAFE, 0.5).offset_s == 0.0
  assert request(DRIVING_MODE_ECO, 0.5).offset_s == 0.0
  assert request(DRIVING_MODE_NORMAL, 0.5).offset_s == 0.0
  assert request(DRIVING_MODE_HIGH, 0.5).offset_s == 0.0


def test_safe_preview_reacts_to_lead_deceleration():
  assert request(DRIVING_MODE_SAFE, -0.5).offset_s == pytest.approx(0.40)


def test_eco_keeps_braking_response():
  assert request(DRIVING_MODE_ECO, -0.4).offset_s == pytest.approx(0.30)


def test_normal_keeps_braking_response():
  assert request(DRIVING_MODE_NORMAL, -0.5).offset_s == pytest.approx(0.40)


def test_high_keeps_braking_response():
  assert request(DRIVING_MODE_HIGH, -0.5).offset_s == pytest.approx(0.40)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
@pytest.mark.parametrize("level", range(1, 6))
def test_tf1_acceleration_response_is_equal_in_every_drive_mode(mode, level):
  result = request(
    mode, 1.1,
    accel_response_level=level,
    accel_response_enabled=True,
    v_rel=0.0,
    gap_margin=0.5,
  )
  tuning = LEAD_ACCEL_RESPONSE_TUNING[level]
  assert result.accel_response_active
  assert result.offset_s == pytest.approx(min(1.0 * tuning.preview_factor, tuning.preview_max))


def test_response_levels_rise_monotonically_from_gentle_to_tracking():
  tunings = [LEAD_ACCEL_RESPONSE_TUNING[level] for level in range(1, 6)]
  assert [t.preview_max for t in tunings] == sorted(t.preview_max for t in tunings)
  assert [t.target_delta_max for t in tunings] == sorted(t.target_delta_max for t in tunings)
  assert [t.gap_margin_floor for t in tunings] == sorted(
    (t.gap_margin_floor for t in tunings), reverse=True,
  )


def test_only_level_five_adds_direct_alead_boost():
  for level in range(1, 5):
    result = request(DRIVING_MODE_NORMAL, 2.0, accel_response_level=level,
                     accel_response_enabled=True, v_rel=0.0, gap_margin=1.0)
    assert result.accel_boost_target == 0.0

  result = request(DRIVING_MODE_NORMAL, 2.0, accel_response_level=5,
                   accel_response_enabled=True, v_rel=0.0, gap_margin=1.0)
  assert result.accel_boost_target == pytest.approx(0.50)


def test_acceleration_response_requires_gap_and_relative_speed_margin():
  assert not lead_accel_response_allowed(1, v_rel=0.0, gap_margin=-0.01, lead_accel_signal=1.0, a_lead=1.0)
  assert lead_accel_response_allowed(3, v_rel=0.0, gap_margin=-2.00, lead_accel_signal=1.0, a_lead=1.0)
  assert not lead_accel_response_allowed(3, v_rel=0.0, gap_margin=-2.01, lead_accel_signal=1.0, a_lead=1.0)
  assert not lead_accel_response_allowed(5, v_rel=-0.21, gap_margin=1.0, lead_accel_signal=1.0, a_lead=1.0)
  assert not lead_accel_response_allowed(5, v_rel=-0.20, gap_margin=1.0, lead_accel_signal=0.3, a_lead=1.0)
  assert lead_accel_response_allowed(5, v_rel=-0.20, gap_margin=1.0, lead_accel_signal=0.4, a_lead=1.0)
  assert not lead_accel_response_allowed(1, v_rel=-0.01, gap_margin=1.0, lead_accel_signal=2.0, a_lead=2.0)
  assert lead_accel_response_allowed(1, v_rel=0.0, gap_margin=0.0, lead_accel_signal=0.1, a_lead=0.2)


def test_level_five_can_follow_from_inside_target_gap_when_lead_is_pulling_away():
  assert not lead_accel_response_allowed(4, v_rel=0.0, gap_margin=-5.0, lead_accel_signal=1.0, a_lead=1.0)
  assert lead_accel_response_allowed(5, v_rel=0.0, gap_margin=-5.0, lead_accel_signal=1.0, a_lead=1.0)
  assert not lead_accel_response_allowed(5, v_rel=-0.5, gap_margin=-5.0, lead_accel_signal=1.0, a_lead=1.0)


def test_level_five_requires_positive_raw_lead_acceleration():
  assert not lead_accel_response_allowed(5, v_rel=0.5, gap_margin=1.0, lead_accel_signal=0.5, a_lead=0.0)
  assert not request(
    DRIVING_MODE_NORMAL, 0.0, a_ego=-0.5,
    accel_response_level=5, accel_response_enabled=True,
    v_rel=0.5, gap_margin=1.0,
  ).accel_response_active


def test_level_five_keeps_tracking_while_lead_acceleration_remains_positive():
  result = request(
    DRIVING_MODE_NORMAL, 0.6, a_ego=0.6,
    accel_response_level=5, accel_response_enabled=True,
    v_rel=0.2, gap_margin=-5.0,
  )
  assert result.accel_response_active
  assert result.offset_s == pytest.approx(0.55)
  assert result.accel_boost_target == pytest.approx(0.30)


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


def test_level_five_boost_has_fast_attack_and_slower_release():
  tuning = LEAD_ACCEL_RESPONSE_TUNING[5]
  assert rate_limit_accel_boost(tuning.boost_max, 0.0, tuning.boost_attack_step) == pytest.approx(0.20)
  assert rate_limit_accel_boost(tuning.boost_max, 0.20, tuning.boost_attack_step) == pytest.approx(0.40)
  assert rate_limit_accel_boost(0.0, 0.40, tuning.boost_attack_step) == pytest.approx(0.36)
  assert rate_limit_accel_boost(0.0, 0.40, tuning.boost_attack_step, immediate_release=True) == 0.0


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
  assert apply_preview_target(0.08, -0.20, DRIVING_MODE_SAFE, 0.5) == pytest.approx(0.08)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_level_five_acceleration_target_is_mode_independent_and_bounded(mode):
  assert apply_preview_target(
    0.20, 0.80, mode, 0.5,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.50,
    accel_max=2.0,
    a_lead=1.0,
  ) == pytest.approx(1.00)


def test_acceleration_response_respects_vehicle_acceleration_limit():
  assert apply_preview_target(
    0.20, 0.80, DRIVING_MODE_NORMAL, 0.5,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.50,
    accel_max=0.50,
    a_lead=1.0,
  ) == pytest.approx(0.50)


def test_lower_acceleration_response_is_disabled_while_plan_or_vehicle_is_decelerating():
  kwargs = dict(accel_response_active=True, accel_response_level=4, accel_boost=0.0)
  assert apply_preview_target(-0.50, -0.10, DRIVING_MODE_HIGH, 0.5, a_ego=0.0, **kwargs) == pytest.approx(-0.50)
  assert apply_preview_target(0.20, 0.50, DRIVING_MODE_HIGH, 0.5, a_ego=-0.3, **kwargs) == pytest.approx(0.20)


def test_level_five_direct_boost_can_release_braking():
  assert apply_preview_target(
    -0.50, -0.80, DRIVING_MODE_HIGH, 0.5,
    a_ego=-0.3,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.20,
    a_lead=0.50,
  ) == pytest.approx(-0.30)


def test_level_five_braking_release_respects_acceleration_limit():
  assert apply_preview_target(
    -0.50, -0.80, DRIVING_MODE_HIGH, 0.5,
    a_ego=-0.3,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.20,
    accel_max=-0.40,
    a_lead=0.50,
  ) == pytest.approx(-0.40)


def test_level_five_direct_boost_applies_when_preview_trajectory_is_falling():
  assert apply_preview_target(
    0.20, 0.10, DRIVING_MODE_HIGH, 0.5,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.50,
    a_lead=1.0,
  ) == pytest.approx(0.70)


def test_level_five_acceleration_cannot_exceed_lead_by_more_than_small_overshoot():
  assert apply_preview_target(
    0.10, 1.20, DRIVING_MODE_HIGH, 0.5,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.50,
    accel_max=2.0,
    a_lead=0.40,
  ) == pytest.approx(0.60)
