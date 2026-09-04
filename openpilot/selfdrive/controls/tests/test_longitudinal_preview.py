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
  get_lead_accel_mpc_request,
  get_lead_preview_request,
  lead_accel_response_allowed,
  lead_accel_response_source_allowed,
  rate_limit_preview,
)


def preview_request(mode, a_lead, *, a_ego=0.0, lead_status=True):
  return get_lead_preview_request(
    mode,
    lead_status=lead_status,
    a_lead=a_lead,
    a_ego=a_ego,
  )


def mpc_request(level, *, source='lead0', enabled=True, lead_status=True,
                a_lead=1.1, a_ego=0.0, v_rel=0.0, gap_margin=0.5,
                speed_error=20.0):
  return get_lead_accel_mpc_request(
    level,
    enabled=enabled,
    source=source,
    lead_status=lead_status,
    a_lead=a_lead,
    a_ego=a_ego,
    v_rel=v_rel,
    gap_margin=gap_margin,
    speed_error=speed_error,
  )


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_all_modes_keep_same_lead_deceleration_preview(mode):
  result = preview_request(mode, -0.5)
  assert result.active
  assert result.lead_accel_signal == pytest.approx(-0.4)
  assert result.offset_s == pytest.approx(0.4)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_positive_lead_acceleration_is_not_post_mpc_preview(mode):
  result = preview_request(mode, 1.1)
  assert result.active
  assert result.lead_accel_signal == pytest.approx(1.0)
  assert result.offset_s == 0.0


def test_preview_uses_relative_acceleration_and_releases_when_matched():
  assert preview_request(DRIVING_MODE_NORMAL, 0.0, a_ego=0.8).lead_accel_signal == pytest.approx(-0.7)
  matched = preview_request(DRIVING_MODE_NORMAL, -0.5, a_ego=-0.5)
  assert matched.lead_accel_signal == 0.0
  assert matched.offset_s == 0.0


def test_preview_is_disabled_for_invalid_or_missing_lead():
  assert not preview_request(DRIVING_MODE_SAFE, -1.0, lead_status=False).active
  assert not preview_request(DRIVING_MODE_SAFE, float('nan')).active
  assert not preview_request(DRIVING_MODE_SAFE, -1.0, a_ego=float('nan')).active


def test_response_costs_progress_from_gentle_to_maximum():
  tunings = [LEAD_ACCEL_RESPONSE_TUNING[level] for level in range(1, 6)]
  assert [t.a_change_cost_factor for t in tunings] == sorted(
    (t.a_change_cost_factor for t in tunings), reverse=True,
  )
  assert [t.jerk_cost_factor for t in tunings] == sorted(
    (t.jerk_cost_factor for t in tunings), reverse=True,
  )


@pytest.mark.parametrize(("level", "a_change_factor", "jerk_factor"), [
  (1, 0.85, 0.95),
  (2, 0.65, 0.80),
  (3, 0.40, 0.60),
  (4, 0.18, 0.35),
  (5, 0.05, 0.15),
])
def test_active_response_returns_mpc_cost_factors(level, a_change_factor, jerk_factor):
  result = mpc_request(level)
  assert result.active
  assert result.level == level
  assert result.a_change_cost_factor == pytest.approx(a_change_factor)
  assert result.jerk_cost_factor == pytest.approx(jerk_factor)


def test_cruise_source_response_starts_at_balanced_level():
  assert not lead_accel_response_source_allowed(2, 'cruise')
  assert lead_accel_response_source_allowed(3, 'cruise')
  assert lead_accel_response_source_allowed(4, 'cruise')
  assert lead_accel_response_source_allowed(5, 'cruise')
  assert lead_accel_response_source_allowed(1, 'lead0')
  assert not lead_accel_response_source_allowed(5, 'e2e')


@pytest.mark.parametrize("level", [3, 4, 5])
def test_cruise_source_uses_positive_raw_lead_acceleration(level):
  result = mpc_request(
    level,
    source='cruise',
    a_lead=0.6,
    a_ego=0.6,
    v_rel=0.4,
    gap_margin=1.0,
  )
  assert result.active


def test_gentle_levels_cannot_enable_cruise_source_response():
  assert not mpc_request(2, source='cruise', v_rel=0.4).active


@pytest.mark.parametrize("level", [3, 4, 5])
def test_cruise_response_releases_when_lead_acceleration_ends_or_gap_will_close(level):
  assert not mpc_request(level, source='cruise', a_lead=0.1, v_rel=0.4).active
  assert not mpc_request(
    level, source='cruise', a_lead=0.6, a_ego=2.0, v_rel=0.0,
  ).active


@pytest.mark.parametrize("level", range(1, 6))
def test_every_level_releases_when_raw_lead_acceleration_ends(level):
  assert not mpc_request(level, a_lead=0.1, a_ego=-0.5, v_rel=0.5).active


def test_cruise_response_requires_set_speed_headroom():
  assert not mpc_request(5, source='cruise', speed_error=1.0 / 3.6).active
  assert mpc_request(5, source='cruise', speed_error=1.01 / 3.6).active


def test_acceleration_response_requires_gap_and_relative_speed_margin():
  assert not lead_accel_response_allowed(1, v_rel=0.0, gap_margin=0.0,
                                         lead_accel_signal=1.0, a_lead=1.0)
  assert lead_accel_response_allowed(3, v_rel=0.0, gap_margin=0.01,
                                     lead_accel_signal=1.0, a_lead=1.0)
  assert not lead_accel_response_allowed(3, v_rel=0.0, gap_margin=-0.01,
                                         lead_accel_signal=1.0, a_lead=1.0)
  assert not lead_accel_response_allowed(5, v_rel=-0.21, gap_margin=1.0,
                                         lead_accel_signal=1.0, a_lead=1.0)
  assert lead_accel_response_allowed(5, v_rel=-0.20, gap_margin=1.0,
                                     lead_accel_signal=0.4, a_lead=1.0)


@pytest.mark.parametrize("level", range(1, 6))
def test_every_level_returns_to_normal_cost_at_configured_tf(level):
  assert mpc_request(level, v_rel=0.2, gap_margin=0.01).active
  assert not mpc_request(level, v_rel=0.2, gap_margin=0.0).active
  assert not mpc_request(level, v_rel=0.2, gap_margin=-0.01).active


def test_level_five_requires_positive_raw_lead_acceleration():
  assert not mpc_request(5, a_lead=0.0, a_ego=-0.5, v_rel=0.5).active


def test_inactive_request_restores_default_mpc_cost_factors():
  for result in (
    mpc_request(5, enabled=False),
    mpc_request(5, lead_status=False),
    mpc_request(0),
    mpc_request(5, source='e2e'),
  ):
    assert not result.active
    assert result.a_change_cost_factor == 1.0
    assert result.jerk_cost_factor == 1.0


def test_preview_rate_and_action_time_are_bounded():
  assert rate_limit_preview(0.60, 0.0) == pytest.approx(0.08)
  assert rate_limit_preview(0.0, 0.20) == pytest.approx(0.17)
  assert rate_limit_preview(-0.10, 0.0) == pytest.approx(-0.03)
  assert clip_action_time(2.40, 1.0) == pytest.approx(2.50)
  assert clip_action_time(0.20, -1.0) == pytest.approx(0.05)


def test_zero_preview_preserves_configured_actuator_delay():
  assert clip_action_time(2.05, 0.0) == pytest.approx(2.05)


def test_negative_preview_cannot_cross_zero_actuator_delay():
  long_actuator_delay = 0.15
  base_action_t = long_actuator_delay + 0.05
  effective_preview = clip_preview_offset(base_action_t, -1.0)
  assert effective_preview == pytest.approx(-long_actuator_delay)


def test_preview_only_removes_acceleration_with_bounded_prebraking():
  assert apply_preview_target(0.20, 0.40, DRIVING_MODE_SAFE, -0.5) == pytest.approx(0.20)
  assert apply_preview_target(0.20, -0.20, DRIVING_MODE_SAFE, -0.5) == pytest.approx(-0.05)
  assert apply_preview_target(0.08, -0.20, DRIVING_MODE_ECO, -0.5) == pytest.approx(-0.04)
  assert apply_preview_target(-0.50, -1.0, DRIVING_MODE_NORMAL, -0.5) == pytest.approx(-0.58)


@pytest.mark.parametrize("mode", [
  DRIVING_MODE_SAFE,
  DRIVING_MODE_ECO,
  DRIVING_MODE_NORMAL,
  DRIVING_MODE_HIGH,
])
def test_positive_signal_never_changes_mpc_acceleration_output(mode):
  assert apply_preview_target(0.20, 1.20, mode, 0.5) == pytest.approx(0.20)
  assert apply_preview_target(-0.50, 0.50, mode, 0.5) == pytest.approx(-0.50)
