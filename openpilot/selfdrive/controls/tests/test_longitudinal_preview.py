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
  get_cruise_accel_target,
  get_lead_preview_request,
  lead_accel_response_allowed,
  lead_accel_response_source_allowed,
  rate_limit_accel_boost,
  rate_limit_cruise_accel_target,
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
  assert [t.gap_error_gain for t in tunings] == sorted(t.gap_error_gain for t in tunings)
  assert [t.opening_speed_gain for t in tunings] == sorted(t.opening_speed_gain for t in tunings)
  assert [t.gap_recovery_max for t in tunings] == sorted(t.gap_recovery_max for t in tunings)
  assert [t.gap_margin_floor for t in tunings] == sorted(
    (t.gap_margin_floor for t in tunings), reverse=True,
  )


def test_cruise_source_response_starts_at_balanced_level():
  assert not lead_accel_response_source_allowed(2, 'cruise')
  assert lead_accel_response_source_allowed(3, 'cruise')
  assert lead_accel_response_source_allowed(4, 'cruise')
  assert lead_accel_response_source_allowed(5, 'cruise')
  assert lead_accel_response_source_allowed(1, 'lead0')
  assert not lead_accel_response_source_allowed(5, 'e2e')


@pytest.mark.parametrize("level", [3, 4, 5])
def test_cruise_source_uses_positive_raw_lead_acceleration(level):
  result = request(
    DRIVING_MODE_NORMAL, 0.6, a_ego=0.6,
    accel_response_level=level,
    accel_response_enabled=True,
    cruise_source_active=True,
    v_rel=0.4,
    gap_margin=1.0,
  )
  assert result.accel_response_active
  assert result.cruise_source_active


def test_gentle_levels_cannot_enable_cruise_source_response():
  result = request(
    DRIVING_MODE_NORMAL, 0.6,
    accel_response_level=2,
    accel_response_enabled=True,
    cruise_source_active=True,
    v_rel=0.4,
    gap_margin=1.0,
  )
  assert not result.accel_response_active
  assert not result.cruise_source_active


@pytest.mark.parametrize("level", [3, 4, 5])
def test_cruise_source_response_releases_when_lead_acceleration_ends_or_gap_will_close(level):
  assert not request(
    DRIVING_MODE_NORMAL, 0.10,
    accel_response_level=level,
    accel_response_enabled=True,
    cruise_source_active=True,
    v_rel=0.4,
    gap_margin=1.0,
  ).accel_response_active
  assert not request(
    DRIVING_MODE_NORMAL, 0.60, a_ego=2.0,
    accel_response_level=level,
    accel_response_enabled=True,
    cruise_source_active=True,
    v_rel=0.0,
    gap_margin=1.0,
  ).accel_response_active


def test_only_level_five_adds_direct_alead_boost():
  for level in range(1, 5):
    result = request(DRIVING_MODE_NORMAL, 2.0, accel_response_level=level,
                     accel_response_enabled=True, v_rel=0.0, gap_margin=1.0)
    assert result.accel_boost_target == 0.0

  result = request(DRIVING_MODE_NORMAL, 2.0, accel_response_level=5,
                   accel_response_enabled=True, v_rel=0.0, gap_margin=1.0)
  assert result.accel_boost_target == pytest.approx(0.75)


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
  assert result.offset_s == pytest.approx(0.65)
  assert result.accel_boost_target == pytest.approx(0.40)


def test_cruise_acceleration_levels_progressively_use_cruise_max_envelope():
  kwargs = dict(base_target=0.31, accel_max=2.33, a_lead=0.81, speed_error=59.3 / 3.6)
  targets = [
    get_cruise_accel_target(accel_response_level=level, **kwargs)
    for level in (3, 4, 5)
  ]
  assert targets == pytest.approx([0.86, 1.06, 1.26])


def test_cruise_acceleration_levels_use_opening_gap_for_distinct_recovery():
  kwargs = dict(
    base_target=0.70,
    accel_max=2.33,
    a_lead=0.81,
    speed_error=59.3 / 3.6,
    v_rel=0.80,
    gap_margin=6.0,
  )
  targets = [
    get_cruise_accel_target(accel_response_level=level, **kwargs)
    for level in (3, 4, 5)
  ]
  assert targets == pytest.approx([1.13, 1.60, 2.00])


def test_gap_recovery_does_not_reward_a_closing_or_short_gap():
  kwargs = dict(base_target=0.70, accel_max=2.33, a_lead=0.81, speed_error=59.3 / 3.6)
  baseline = get_cruise_accel_target(accel_response_level=5, **kwargs)
  assert get_cruise_accel_target(
    accel_response_level=5, v_rel=-0.1, gap_margin=-1.0, **kwargs,
  ) == pytest.approx(baseline)


def test_cruise_acceleration_target_needs_set_speed_headroom_and_positive_lead_accel():
  kwargs = dict(base_target=0.31, accel_max=2.33, accel_response_level=5)
  assert get_cruise_accel_target(a_lead=0.81, speed_error=1.0 / 3.6, **kwargs) == pytest.approx(0.31)
  assert get_cruise_accel_target(a_lead=0.10, speed_error=20.0 / 3.6, **kwargs) == pytest.approx(0.31)


def test_cruise_acceleration_target_respects_final_acceleration_limit():
  assert get_cruise_accel_target(
    0.20,
    accel_max=0.50,
    a_lead=2.0,
    speed_error=20.0 / 3.6,
    accel_response_level=5,
  ) == pytest.approx(0.50)


def test_cruise_acceleration_target_attacks_by_level_and_releases_immediately():
  assert rate_limit_cruise_accel_target(0.80, 0.30, 0.30, 0.08) == pytest.approx(0.38)
  assert rate_limit_cruise_accel_target(0.80, 0.30, 0.30, 0.20) == pytest.approx(0.50)
  assert rate_limit_cruise_accel_target(0.40, 0.70, 0.30, 0.20) == pytest.approx(0.40)


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
  assert rate_limit_accel_boost(tuning.boost_max, 0.0, tuning.boost_attack_step) == pytest.approx(0.30)
  assert rate_limit_accel_boost(tuning.boost_max, 0.30, tuning.boost_attack_step) == pytest.approx(0.60)
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
    accel_boost=0.75,
    accel_max=2.0,
    a_lead=1.0,
  ) == pytest.approx(1.45)


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


def test_level_five_acceleration_without_gap_recovery_uses_bounded_overshoot():
  assert apply_preview_target(
    0.10, 1.20, DRIVING_MODE_HIGH, 0.5,
    accel_response_active=True,
    accel_response_level=5,
    accel_boost=0.50,
    accel_max=2.0,
    a_lead=0.40,
  ) == pytest.approx(0.85)


@pytest.mark.parametrize(("level", "cruise_target", "expected"), [
  (3, 0.86, 0.86),
  (4, 1.06, 1.06),
  (5, 1.26, 1.26),
])
def test_cruise_source_target_is_applied_by_levels_three_to_five(level, cruise_target, expected):
  assert apply_preview_target(
    0.31, 0.40, DRIVING_MODE_NORMAL, 0.0,
    a_ego=0.31,
    accel_response_active=True,
    accel_response_level=level,
    accel_max=2.33,
    a_lead=0.81,
    cruise_source_active=True,
    cruise_accel_target=cruise_target,
  ) == pytest.approx(expected)


def test_cruise_source_floor_can_overcome_falling_preview_without_releasing_braking():
  assert apply_preview_target(
    0.30, 0.20, DRIVING_MODE_NORMAL, -0.20,
    a_ego=0.20,
    accel_response_active=True,
    accel_response_level=4,
    accel_max=1.50,
    a_lead=0.60,
    cruise_source_active=True,
    cruise_accel_target=0.70,
  ) == pytest.approx(0.70)
  assert apply_preview_target(
    -0.10, 0.20, DRIVING_MODE_NORMAL, 0.40,
    a_ego=-0.20,
    accel_response_active=True,
    accel_response_level=4,
    accel_max=1.50,
    a_lead=0.60,
    cruise_source_active=True,
    cruise_accel_target=0.70,
  ) == pytest.approx(-0.10)
