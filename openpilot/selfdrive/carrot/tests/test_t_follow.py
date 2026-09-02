from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot.carrot_functions import (
  CarrotPlanner,
  DrivingMode,
  DrivingModeDetector,
  get_driving_mode_comfort_brake_factor,
)
from openpilot.selfdrive.carrot.t_follow import get_t_follow_mode_factor, get_t_follow_mode_max, ramp_t_follow


DT_MDL = 0.05


def _apply_for_seconds(current: float, target: float, seconds: float, decel_extra: float = 0.0) -> float:
  for _ in range(round(seconds / DT_MDL)):
    current = ramp_t_follow(target, current, decel_extra, DT_MDL)
  return current


def test_gap_one_to_four_is_felt_within_two_seconds():
  assert _apply_for_seconds(1.1, 1.6, 1.7) == pytest.approx(1.6)


def test_gap_increase_is_faster_while_already_decelerating():
  assert _apply_for_seconds(1.1, 1.6, 0.85, decel_extra=0.03) == pytest.approx(1.6)


def test_gap_reduction_remains_immediate():
  assert ramp_t_follow(1.1, 1.6, 0.0, DT_MDL) == pytest.approx(1.1)


def test_lane_change_response_scales_gap_and_jerk_for_first_1_5_seconds():
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.desireState = 1.0
  planner.desireStateCount = 1
  planner.dynamicTFollowLC = 0.8
  planner.jerk_factor = 0.7
  planner.t_follow_last = 1.3
  planner._tf_decel_extra = 0.0

  lead = SimpleNamespace(status=True, jLead=2.0)
  assert planner.dynamic_t_follow(1.3, lead, 0.0, 0.0) == pytest.approx(1.04)
  assert planner.jerk_factor_apply == pytest.approx(0.56)


def test_dynamic_lead_acceleration_response_closes_gap_more_quickly():
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.desireState = 0.0
  planner.desireStateCount = 0
  planner.dynamicTFollow = 0.2
  planner.dynamicTFollowLC = 1.0
  planner.jerk_factor = 0.7
  planner.t_follow_last = 1.3
  planner._tf_decel_extra = 0.0

  lead = SimpleNamespace(status=True, jLead=2.0)
  assert planner.dynamic_t_follow(1.3, lead, 0.0, 0.0) == pytest.approx(1.1)
  assert planner.jerk_factor_apply == pytest.approx(0.35)


@pytest.mark.parametrize(
  ("comfort_factor", "t_follow_factor"),
  (
    (0.9, 1.1),
    (0.8, 1.2),
    (1.0, 1.0),
  ),
)
def test_comfort_mode_reduction_increases_t_follow(comfort_factor, t_follow_factor):
  assert get_t_follow_mode_factor(comfort_factor) == pytest.approx(t_follow_factor)


def test_safe_mode_increase_is_not_clipped_at_the_configured_normal_max():
  assert get_t_follow_mode_max(1.6, 1.2, 0.0) == pytest.approx(1.92)


def test_mode_and_deceleration_gap_increase_preserve_global_cap():
  assert get_t_follow_mode_max(1.8, 1.2, 0.1) == pytest.approx(2.0)


def test_safe_comfort_brake_uses_a_modest_reduction_only():
  assert get_driving_mode_comfort_brake_factor(DrivingMode.Safe) == pytest.approx(0.9)
  assert get_driving_mode_comfort_brake_factor(DrivingMode.Eco) == pytest.approx(1.0)
  assert get_driving_mode_comfort_brake_factor(DrivingMode.Normal) == pytest.approx(1.0)
  assert get_driving_mode_comfort_brake_factor(DrivingMode.High) == pytest.approx(1.0)


@pytest.mark.parametrize(
  ("auto_mode", "congested", "expected"),
  (
    (1, False, DrivingMode.Normal),
    (1, True, DrivingMode.Safe),
    (2, False, DrivingMode.Eco),
    (2, True, DrivingMode.Safe),
  ),
)
def test_automatic_driving_mode_mapping(auto_mode, congested, expected):
  detector = DrivingModeDetector()
  detector.congested = congested
  assert detector.get_mode(auto_mode) == expected


def test_safe_t_follow_does_not_compound_during_repeated_deceleration():
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.enableSpeedTF = 0
  planner.tFollowGap1 = 0.5
  planner.tFollowGap2 = 0.6
  planner.tFollowGap3 = 0.8
  planner.tFollowGap4 = 1.2
  planner.tFollowDecelBoost = 0.0
  planner.myDrivingMode = DrivingMode.Safe
  planner.myTFollowFactor = 1.2
  planner._tf_decel_extra = 0.0
  planner._tf_applied = 0.72
  planner.t_follow_last = 0.72

  values = [
    planner.get_T_FOLLOW(log.LongitudinalPersonality.standard, v_ego=10.0, a_ego=-1.0)
    for _ in range(100)
  ]
  assert values == pytest.approx([0.72] * 100)


def _speed_tf_planner(lead_accel_response: int, applied_t_follow: float) -> CarrotPlanner:
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.enableSpeedTF = -3
  planner.tFollowGap1 = 0.4
  planner.tFollowGap2 = 0.6
  planner.tFollowGap3 = 0.8
  planner.tFollowGap4 = 1.2
  planner.tFollowDecelBoost = 0.0
  planner.leadAccelResponse = lead_accel_response
  planner.myDrivingMode = DrivingMode.Safe
  planner.myTFollowFactor = 1.2
  planner._tf_decel_extra = 0.0
  planner._tf_applied = applied_t_follow
  planner.t_follow_last = applied_t_follow
  return planner


def test_level_five_tf1_uses_configured_gap_one_despite_speed_table_and_safe_mode():
  planner = _speed_tf_planner(lead_accel_response=5, applied_t_follow=0.72)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=0.0,
    lead_status=True, lead_accel=0.5,
  ) == pytest.approx(0.4)


def test_lower_tf1_response_keeps_speed_table_and_safe_mode_gap():
  planner = _speed_tf_planner(lead_accel_response=4, applied_t_follow=0.72)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=0.0, lead_status=True,
  ) == pytest.approx(0.72)


def test_level_five_tf1_keeps_larger_gap_while_decelerating():
  planner = _speed_tf_planner(lead_accel_response=5, applied_t_follow=0.72)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=-1.0,
    lead_status=True, lead_accel=0.5,
  ) == pytest.approx(0.72)


@pytest.mark.parametrize("lead_accel", [0.0, -0.5])
def test_level_five_returns_to_normal_gap_when_lead_stops_accelerating(lead_accel):
  planner = _speed_tf_planner(lead_accel_response=5, applied_t_follow=0.72)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=0.0,
    lead_status=True, lead_accel=lead_accel,
  ) == pytest.approx(0.72)


def test_level_five_acceleration_end_uses_existing_gap_increase_ramp():
  planner = _speed_tf_planner(lead_accel_response=5, applied_t_follow=0.72)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=0.0,
    lead_status=True, lead_accel=0.5,
  ) == pytest.approx(0.4)
  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=0.0,
    lead_status=True, lead_accel=0.0,
  ) == pytest.approx(0.415)


def test_level_five_does_not_force_gap_one_for_other_personalities():
  planner = _speed_tf_planner(lead_accel_response=5, applied_t_follow=0.936)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.standard, v_ego=50.0 / 3.6, a_ego=0.0,
    lead_status=True, lead_accel=0.5,
  ) == pytest.approx(0.936)


def test_level_five_tf1_does_not_change_cruise_target_without_a_lead():
  planner = _speed_tf_planner(lead_accel_response=5, applied_t_follow=0.72)

  assert planner.get_T_FOLLOW(
    log.LongitudinalPersonality.aggressive, v_ego=50.0 / 3.6, a_ego=0.0,
    lead_status=False, lead_accel=0.5,
  ) == pytest.approx(0.72)
