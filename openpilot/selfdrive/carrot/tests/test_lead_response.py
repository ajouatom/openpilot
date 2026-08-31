from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lead_response import (
  LEAD_RESPONSE_BALANCED,
  LEAD_RESPONSE_CONFIRM_FRAMES,
  LEAD_RESPONSE_MIN_LEAD_SPEED,
  LEAD_RESPONSE_MIN_TRACK_FRAMES,
  LEAD_RESPONSE_SMOOTH,
  LEAD_RESPONSE_SYNC,
  auto_driving_mode_for_congestion,
  blend_lead_accel_reference,
  build_lead_accel_trajectory,
  build_lead_accel_reference,
  calculate_lead_braking_urgency,
  combine_braking_urgency_with_margin,
  combine_lead_accel_references,
  equal_lead_t_follow_adjustment,
  lead_closing_preview_weight,
  lead_obstacle_relevance,
  lead_response_confidence,
  lead_response_mode_for_driving_mode,
  lead_response_target_weight,
  lead_safety_jerk_factor,
  rate_limit_lead_response_weight,
  should_apply_lead_accel_reference,
)


TIME_INDICES = np.array((0.0, 0.2, 0.6, 1.2, 2.0))


def lead(**overrides):
  values = {
    "status": True,
    "radar": True,
    "dRel": 20.0,
    "vRel": 0.0,
    "aLead": -2.0,
    "aLeadK": -2.0,
    "aLeadTau": 0.5,
  }
  values.update(overrides)
  return SimpleNamespace(**values)


def reference(mode: int, **lead_overrides):
  return build_lead_accel_reference(
    lead(**lead_overrides),
    mode=mode,
    v_ego=15.0,
    v_cruise=30.0,
    desired_distance=15.0,
    previous_acceleration=0.0,
    time_indices=TIME_INDICES,
  )


def test_response_modes_preserve_order_without_delaying_onset() -> None:
  smooth = reference(LEAD_RESPONSE_SMOOTH)
  balanced = reference(LEAD_RESPONSE_BALANCED)
  sync = reference(LEAD_RESPONSE_SYNC)

  assert smooth is not None and balanced is not None and sync is not None
  assert 0.0 > smooth.acceleration[0] > balanced.acceleration[0] > sync.acceleration[0]
  assert abs(smooth.raw_acceleration[0]) < abs(balanced.raw_acceleration[0]) < abs(sync.raw_acceleration[0])


def test_existing_driving_modes_select_the_response_profile() -> None:
  assert lead_response_mode_for_driving_mode(1) == LEAD_RESPONSE_SMOOTH
  assert lead_response_mode_for_driving_mode(2) == LEAD_RESPONSE_BALANCED
  assert lead_response_mode_for_driving_mode(3) == LEAD_RESPONSE_SYNC
  assert lead_response_mode_for_driving_mode(4) == LEAD_RESPONSE_SYNC
  assert lead_response_mode_for_driving_mode(99) == LEAD_RESPONSE_BALANCED


def test_collision_urgency_removes_comfort_mode_mpc_inertia() -> None:
  assert lead_safety_jerk_factor(0.75, 0.0) == pytest.approx(0.75)
  assert lead_safety_jerk_factor(0.75, 0.3) == pytest.approx(0.75)
  assert 0.50 < lead_safety_jerk_factor(0.75, 0.8) < 0.75
  assert lead_safety_jerk_factor(0.75, 1.0) == pytest.approx(0.50)
  assert lead_safety_jerk_factor(0.70, 1.0) == pytest.approx(0.50)
  assert lead_safety_jerk_factor(0.50, 1.0) == pytest.approx(0.50)
  assert lead_safety_jerk_factor(0.25, 1.0) == pytest.approx(0.25)


@pytest.mark.parametrize(
  ("auto_mode", "calm_mode", "clear_mode"),
  ((1, 1, 2), (2, 1, 3), (3, 2, 3)),
)
def test_auto_driving_mode_pairs(auto_mode: int, calm_mode: int, clear_mode: int) -> None:
  assert auto_driving_mode_for_congestion(auto_mode, congested=True) == calm_mode
  assert auto_driving_mode_for_congestion(auto_mode, congested=False) == clear_mode


def test_surplus_distance_softens_smooth_more_than_sync() -> None:
  smooth_close = reference(LEAD_RESPONSE_SMOOTH, dRel=15.0)
  smooth_far = reference(LEAD_RESPONSE_SMOOTH, dRel=35.0)
  sync_close = reference(LEAD_RESPONSE_SYNC, dRel=15.0)
  sync_far = reference(LEAD_RESPONSE_SYNC, dRel=35.0)

  assert smooth_close is not None and smooth_far is not None
  assert sync_close is not None and sync_far is not None
  smooth_relief = abs(smooth_close.raw_acceleration[0]) - abs(smooth_far.raw_acceleration[0])
  sync_relief = abs(sync_close.raw_acceleration[0]) - abs(sync_far.raw_acceleration[0])
  assert smooth_relief > sync_relief


def test_short_ttc_blends_every_mode_toward_safety_response() -> None:
  smooth = reference(
    LEAD_RESPONSE_SMOOTH,
    dRel=8.0,
    vRel=-4.0,
  )
  sync = reference(
    LEAD_RESPONSE_SYNC,
    dRel=8.0,
    vRel=-4.0,
  )

  assert smooth is not None and sync is not None
  assert smooth.safety_blend == pytest.approx(1.0)
  assert smooth.effective_gain == pytest.approx(1.0)
  assert smooth.raw_acceleration[0] == pytest.approx(sync.raw_acceleration[0])


def test_full_urgency_makes_deceleration_mode_independent_with_surplus_gap() -> None:
  references = [
    build_lead_accel_reference(
      lead(dRel=25.0, vRel=0.0, aLead=-1.0, aLeadK=-1.0),
      mode=mode,
      v_ego=15.0,
      v_cruise=30.0,
      desired_distance=15.0,
      previous_acceleration=0.0,
      time_indices=TIME_INDICES,
      braking_urgency=1.0,
    )
    for mode in (LEAD_RESPONSE_SMOOTH, LEAD_RESPONSE_BALANCED, LEAD_RESPONSE_SYNC)
  ]

  assert all(reference is not None for reference in references)
  for reference in references[1:]:
    assert reference.raw_acceleration == pytest.approx(references[0].raw_acceleration)
    assert reference.acceleration == pytest.approx(references[0].acceleration)


def test_reference_is_rate_limited_from_previous_acceleration() -> None:
  smooth = reference(LEAD_RESPONSE_SMOOTH, aLead=-4.0, aLeadK=-4.0)
  balanced = reference(LEAD_RESPONSE_BALANCED, aLead=-4.0, aLeadK=-4.0)
  sync = reference(LEAD_RESPONSE_SYNC, aLead=-4.0, aLeadK=-4.0)

  assert smooth is not None and balanced is not None and sync is not None
  assert smooth.acceleration[0] == pytest.approx(-0.04)
  assert balanced.acceleration[0] == pytest.approx(-0.08)
  assert sync.acceleration[0] == pytest.approx(-0.14)


def test_fast_closing_lead_requests_deceleration_after_alead_recovers() -> None:
  stationary_acceleration = reference(
    LEAD_RESPONSE_BALANCED,
    dRel=8.0,
    vRel=-4.0,
    aLead=0.0,
    aLeadK=0.0,
  )

  assert stationary_acceleration is not None
  assert stationary_acceleration.raw_acceleration[0] < -1.0
  assert stationary_acceleration.acceleration[0] < 0.0


def test_closing_urgency_keeps_bounded_feedback_active_after_alead_recovers() -> None:
  common = {
    "mode": LEAD_RESPONSE_SMOOTH,
    "v_ego": 8.0,
    "v_cruise": 20.0,
    "desired_distance": 15.0,
    "previous_acceleration": 0.0,
    "time_indices": TIME_INDICES,
  }
  calm = build_lead_accel_reference(
    lead(dRel=10.0, vRel=-2.0, aLead=0.0, aLeadK=0.0),
    braking_urgency=0.0,
    **common,
  )
  urgent = build_lead_accel_reference(
    lead(dRel=10.0, vRel=-2.0, aLead=0.0, aLeadK=0.0),
    braking_urgency=1.0,
    **common,
  )

  assert calm is not None and urgent is not None
  assert calm.raw_acceleration[0] < 0.0
  assert urgent.raw_acceleration[0] < calm.raw_acceleration[0]
  assert urgent.acceleration[0] < 0.0


def test_e4f_smooth_approach_does_not_release_braking_on_positive_alead() -> None:
  # 00000e4f--6327e1ceba--2 at 50.29 s: the same lead was still being
  # approached at 6 m/s, but aLeadK briefly became positive and the old FF
  # changed from braking to acceleration.
  incident = build_lead_accel_reference(
    lead(dRel=43.94, vRel=-6.04, aLead=0.46, aLeadK=0.46),
    mode=LEAD_RESPONSE_SMOOTH,
    v_ego=14.87,
    v_cruise=25.0,
    desired_distance=43.36,
    previous_acceleration=-0.15,
    time_indices=TIME_INDICES,
  )

  assert incident is not None
  assert incident.raw_acceleration[0] <= -0.65
  assert incident.acceleration[0] < -0.15


def test_fast_closing_guard_is_drive_mode_independent() -> None:
  references = [
    build_lead_accel_reference(
      lead(dRel=43.94, vRel=-6.04, aLead=0.46, aLeadK=0.46),
      mode=mode,
      v_ego=14.87,
      v_cruise=25.0,
      desired_distance=43.36,
      previous_acceleration=-0.15,
      time_indices=TIME_INDICES,
    )
    for mode in (LEAD_RESPONSE_SMOOTH, LEAD_RESPONSE_BALANCED, LEAD_RESPONSE_SYNC)
  ]

  assert all(item is not None for item in references)
  for item in references[1:]:
    assert item.raw_acceleration == pytest.approx(references[0].raw_acceleration)
    assert item.acceleration == pytest.approx(references[0].acceleration)


def test_closing_preview_starts_before_desired_gap_is_consumed() -> None:
  # Same route at 48.04 s: 12.8 m of surplus remained, but at -6.14 m/s it
  # would be consumed in about 2.1 seconds.
  weight = lead_closing_preview_weight(58.22, -6.14, 45.40)
  assert weight > 0.95
  assert lead_response_target_weight(0.0, 0.5, weight) > 0.95


def test_distant_slowly_converging_lead_does_not_open_closing_preview() -> None:
  assert lead_closing_preview_weight(130.0, -3.0, 12.0) == pytest.approx(0.0)


def test_emergency_jerk_rate_requires_braking_urgency() -> None:
  common = {
    "mode": LEAD_RESPONSE_SMOOTH,
    "v_ego": 15.0,
    "v_cruise": 30.0,
    "desired_distance": 15.0,
    "previous_acceleration": 0.0,
    "time_indices": TIME_INDICES,
  }
  calm = build_lead_accel_reference(
    lead(dRel=8.0, vRel=-4.0, aLead=-4.0, aLeadK=-4.0),
    braking_urgency=0.0,
    **common,
  )
  urgent = build_lead_accel_reference(
    lead(dRel=8.0, vRel=-4.0, aLead=-4.0, aLeadK=-4.0),
    braking_urgency=1.0,
    **common,
  )

  assert calm is not None and urgent is not None
  assert calm.safety_blend == pytest.approx(1.0)
  assert calm.raw_acceleration[0] == pytest.approx(urgent.raw_acceleration[0])
  assert calm.acceleration[0] == pytest.approx(-0.06)
  assert urgent.acceleration[0] == pytest.approx(-0.175)


def test_non_radar_or_invalid_lead_has_no_feedforward_reference() -> None:
  assert build_lead_accel_reference(
    lead(radar=False),
    mode=LEAD_RESPONSE_BALANCED,
    v_ego=10.0,
    v_cruise=20.0,
    desired_distance=10.0,
    previous_acceleration=0.0,
    time_indices=TIME_INDICES,
  ) is None


def test_far_lead_deceleration_never_becomes_acceleration_request() -> None:
  smooth = reference(
    LEAD_RESPONSE_SMOOTH,
    dRel=60.0,
    vRel=0.0,
    aLead=-2.0,
    aLeadK=-2.0,
  )

  assert smooth is not None
  assert np.all(smooth.raw_acceleration <= 0.0)


def test_far_closing_lead_cannot_accelerate_past_cruise_speed() -> None:
  # Route c80e193a070a8bbe/00000bd0--5eb0330984--9 reproduced a
  # +0.55 m/s^2 raw reference here and accelerated from a 100 km/h set speed
  # to 115 km/h indicated while the slower lead was already being closed on.
  at_cruise = build_lead_accel_reference(
    lead(
      dRel=130.1,
      vRel=-1.37,
      aLead=0.13,
      aLeadK=0.13,
      aLeadTau=0.3,
    ),
    mode=LEAD_RESPONSE_SMOOTH,
    v_ego=100.0 / 3.6,
    v_cruise=100.0 / 3.6,
    desired_distance=44.8,
    previous_acceleration=0.41,
    time_indices=TIME_INDICES,
  )

  assert at_cruise is not None
  assert np.all(at_cruise.raw_acceleration <= 0.0)
  assert np.all(at_cruise.acceleration <= 0.0)


def test_positive_lead_response_tapers_with_cruise_speed_headroom() -> None:
  speed_headroom = 0.2
  near_cruise = build_lead_accel_reference(
    lead(
      dRel=60.0,
      vRel=1.0,
      aLead=1.0,
      aLeadK=1.0,
    ),
    mode=LEAD_RESPONSE_SYNC,
    v_ego=25.0,
    v_cruise=25.0 + speed_headroom,
    desired_distance=30.0,
    previous_acceleration=0.5,
    time_indices=TIME_INDICES,
  )

  assert near_cruise is not None
  assert np.max(near_cruise.raw_acceleration) <= speed_headroom
  assert np.max(near_cruise.acceleration) <= speed_headroom


def test_far_closing_lead_cannot_suppress_cruise_acceleration() -> None:
  # Route 00000e48--68efa63608--2 had a 100+ m lead while cruise was the
  # limiting MPC source. Its negative vRel reference suppressed launch accel.
  far_lead = build_lead_accel_reference(
    lead(dRel=130.0, vRel=-3.0, aLead=-0.2, aLeadK=-0.2),
    mode=LEAD_RESPONSE_BALANCED,
    v_ego=8.0,
    v_cruise=25.0,
    desired_distance=12.0,
    previous_acceleration=1.0,
    time_indices=TIME_INDICES,
  )

  assert far_lead is not None
  assert far_lead.raw_acceleration[0] < 0.0
  assert should_apply_lead_accel_reference(
    reset_state=False,
    mpc_mode="acc",
    source="cruise",
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
    lead_speed=20.0,
  )
  # Source is no longer a binary gate. A far lead instead gets zero continuous
  # obstacle relevance and therefore cannot replace cruise acceleration.
  weight = lead_obstacle_relevance(
    lead_obstacle=np.full_like(TIME_INDICES, 140.0),
    cruise_obstacle=np.full_like(TIME_INDICES, 20.0),
    v_ego=8.0,
    time_indices=TIME_INDICES,
  )
  assert weight == pytest.approx(0.0)
  blended = blend_lead_accel_reference(
    np.ones_like(TIME_INDICES), far_lead.acceleration, weight,
  )
  assert np.all(blended == 1.0)


def test_filtered_lead_deceleration_opens_early_preview_weight() -> None:
  assert lead_response_target_weight(0.0, -0.2) == pytest.approx(0.0)
  assert 0.0 < lead_response_target_weight(0.0, -0.6) < 1.0
  assert lead_response_target_weight(0.0, -1.0) == pytest.approx(1.0)
  assert lead_response_target_weight(0.7, 0.5) == pytest.approx(0.7)
  assert lead_response_target_weight(0.7, -1.0) == pytest.approx(1.0)


@pytest.mark.parametrize("invalid_acceleration", (np.nan, np.inf, -np.inf))
def test_nonfinite_lead_acceleration_cannot_open_preview(invalid_acceleration: float) -> None:
  assert lead_response_target_weight(0.0, invalid_acceleration) == pytest.approx(0.0)
  assert lead_response_target_weight(0.4, invalid_acceleration) == pytest.approx(0.4)


@pytest.mark.parametrize("mode", (LEAD_RESPONSE_SMOOTH, LEAD_RESPONSE_BALANCED, LEAD_RESPONSE_SYNC))
def test_far_confirmed_hard_deceleration_is_bounded_but_not_blocked(mode: int) -> None:
  previous = np.ones_like(TIME_INDICES)
  far_lead = build_lead_accel_reference(
    lead(dRel=130.0, vRel=0.0, aLead=-4.0, aLeadK=-4.0),
    mode=mode,
    v_ego=20.0,
    v_cruise=30.0,
    desired_distance=25.0,
    previous_acceleration=float(previous[0]),
    time_indices=TIME_INDICES,
  )

  assert far_lead is not None
  target_weight = lead_response_target_weight(0.0, -4.0)
  first_preview_weight = rate_limit_lead_response_weight(0.0, target_weight)
  effective_weight = lead_response_confidence(
    reset_state=False,
    mpc_mode="acc",
    stable_frames=LEAD_RESPONSE_MIN_TRACK_FRAMES,
    lead_speed=20.0,
  ) * first_preview_weight
  preview = blend_lead_accel_reference(previous, far_lead.acceleration, effective_weight)

  # The first confirmed frame already sheds acceleration, but confidence,
  # preview attack, and the selected profile's jerk bound prevent a step brake.
  assert 0.0 < preview[0] < previous[0]
  assert previous[0] - preview[0] < 0.02
  # Once confirmed, a real hard-decelerating lead is allowed to request
  # braking even before the diagnostic MPC source label changes.
  confirmed = blend_lead_accel_reference(previous, far_lead.acceleration, target_weight)
  assert np.any(confirmed < 0.0)


def test_lead_response_starts_on_second_frame_and_builds_confidence() -> None:
  common = {
    "reset_state": False,
    "mpc_mode": "acc",
    "source": "lead0",
    "lead_speed": LEAD_RESPONSE_MIN_LEAD_SPEED + 1.0,
  }
  assert not should_apply_lead_accel_reference(
    **common,
    stable_frames=LEAD_RESPONSE_MIN_TRACK_FRAMES - 1,
  )
  assert should_apply_lead_accel_reference(
    **common,
    stable_frames=LEAD_RESPONSE_MIN_TRACK_FRAMES,
  )
  assert lead_response_confidence(
    reset_state=False,
    mpc_mode="acc",
    stable_frames=LEAD_RESPONSE_MIN_TRACK_FRAMES,
    lead_speed=LEAD_RESPONSE_MIN_LEAD_SPEED + 1.0,
  ) == pytest.approx(0.2)
  assert lead_response_confidence(
    reset_state=False,
    mpc_mode="acc",
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
    lead_speed=LEAD_RESPONSE_MIN_LEAD_SPEED + 1.0,
  ) == pytest.approx(1.0)
  # lead/cruise is a continuous blend; the diagnostic source label cannot add
  # a 0.2-0.4 second delay.
  assert should_apply_lead_accel_reference(
    **(common | {"source": "cruise"}),
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
  )
  assert not should_apply_lead_accel_reference(
    **(common | {"reset_state": True}),
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
  )
  assert not should_apply_lead_accel_reference(
    **(common | {"mpc_mode": "blended"}),
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
  )


def test_stopped_lead_uses_standard_mpc_stop_trajectory() -> None:
  assert not should_apply_lead_accel_reference(
    reset_state=False,
    mpc_mode="acc",
    source="lead0",
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
    lead_speed=0.0,
  )
  assert not should_apply_lead_accel_reference(
    reset_state=False,
    mpc_mode="acc",
    source="lead0",
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
    lead_speed=LEAD_RESPONSE_MIN_LEAD_SPEED,
  )


def test_lead_jerk_is_integrated_before_adding_to_acceleration() -> None:
  time_indices = np.array((0.0, 0.2, 0.5))
  time_differences = np.diff(time_indices, prepend=(0.0,))
  trajectory = build_lead_accel_trajectory(
    acceleration=0.0,
    acceleration_tau=1.5,
    jerk=-1.0,
    time_indices=time_indices,
    time_differences=time_differences,
  )

  assert trajectory[0] == pytest.approx(0.0)
  assert -0.25 < trajectory[1] < -0.15
  assert trajectory[2] < trajectory[1]


def test_lead_relevance_previews_a_future_source_change() -> None:
  cruise = np.array((20.0, 22.0, 26.0, 32.0, 40.0))
  lead = np.array((22.0, 22.5, 23.0, 24.0, 27.0))

  weight = lead_obstacle_relevance(lead, cruise, 15.0, TIME_INDICES)

  assert lead[0] > cruise[0]
  assert weight > 0.5


def test_lead_relevance_is_continuous_around_equal_obstacles() -> None:
  cruise = np.full_like(TIME_INDICES, 30.0)
  slightly_ahead = lead_obstacle_relevance(
    cruise + 0.1, cruise, 15.0, TIME_INDICES,
  )
  slightly_limiting = lead_obstacle_relevance(
    cruise - 0.1, cruise, 15.0, TIME_INDICES,
  )

  assert 0.45 < slightly_ahead < 0.5
  assert 0.5 < slightly_limiting < 0.55


def test_lead_weight_attacks_faster_than_it_releases() -> None:
  assert rate_limit_lead_response_weight(0.0, 1.0) == pytest.approx(0.25)
  assert rate_limit_lead_response_weight(1.0, 0.0) == pytest.approx(0.8)


def test_early_deceleration_blends_more_than_early_acceleration() -> None:
  previous = np.zeros(3)
  braking = blend_lead_accel_reference(previous, -np.ones(3), 0.5)
  accelerating = blend_lead_accel_reference(previous, np.ones(3), 0.5)

  assert np.all(braking == pytest.approx(-0.5))
  assert np.all(accelerating == pytest.approx(0.25))


def test_lead_one_and_lead_two_are_equal_and_more_restrictive_wins() -> None:
  previous = np.zeros(3)
  lead_one = np.array((-0.2, -0.3, -0.4))
  lead_two = np.array((-0.5, -0.1, -0.6))

  combined_12 = combine_lead_accel_references(previous, [lead_one, lead_two])
  combined_21 = combine_lead_accel_references(previous, [lead_two, lead_one])

  assert np.array_equal(combined_12, np.minimum(lead_one, lead_two))
  assert np.array_equal(combined_12, combined_21)


def test_dynamic_t_follow_treats_lead_order_equally() -> None:
  lead_one_first = equal_lead_t_follow_adjustment([-2.0, 1.5], 0.5)
  lead_two_first = equal_lead_t_follow_adjustment([1.5, -2.0], 0.5)

  assert lead_one_first == pytest.approx(lead_two_first)
  assert lead_one_first > 0.0


def test_mpc_braking_urgency_treats_lead_order_equally() -> None:
  safe = (True, 60.0, -1.0)
  dangerous = (True, 12.0, -6.0)

  danger_as_one = calculate_lead_braking_urgency(20.0, (dangerous, safe))
  danger_as_two = calculate_lead_braking_urgency(20.0, (safe, dangerous))

  assert danger_as_one == pytest.approx(danger_as_two)
  assert danger_as_one > 0.8


def test_predicted_mpc_margin_can_raise_braking_urgency() -> None:
  assert combine_braking_urgency_with_margin(0.2, 1.0) == pytest.approx(0.2)
  assert combine_braking_urgency_with_margin(0.2, -1.5) == pytest.approx(0.5)
  assert combine_braking_urgency_with_margin(0.2, -3.0) == pytest.approx(1.0)
