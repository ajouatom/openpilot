from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lead_response import (
  LEAD_RESPONSE_BALANCED,
  LEAD_RESPONSE_CONFIRM_FRAMES,
  LEAD_RESPONSE_MIN_LEAD_SPEED,
  LEAD_RESPONSE_SMOOTH,
  LEAD_RESPONSE_SYNC,
  build_lead_accel_trajectory,
  build_lead_accel_reference,
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


def test_reference_is_rate_limited_from_previous_acceleration() -> None:
  smooth = reference(LEAD_RESPONSE_SMOOTH, aLead=-4.0, aLeadK=-4.0)
  balanced = reference(LEAD_RESPONSE_BALANCED, aLead=-4.0, aLeadK=-4.0)
  sync = reference(LEAD_RESPONSE_SYNC, aLead=-4.0, aLeadK=-4.0)

  assert smooth is not None and balanced is not None and sync is not None
  assert smooth.acceleration[0] == pytest.approx(-0.04)
  assert balanced.acceleration[0] == pytest.approx(-0.08)
  assert sync.acceleration[0] == pytest.approx(-0.14)


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
  assert not should_apply_lead_accel_reference(
    reset_state=False,
    mpc_mode="acc",
    source="cruise",
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES,
    lead_speed=20.0,
  )


def test_lead_response_requires_stable_active_acc_lead() -> None:
  common = {
    "reset_state": False,
    "mpc_mode": "acc",
    "source": "lead0",
    "lead_speed": LEAD_RESPONSE_MIN_LEAD_SPEED + 1.0,
  }
  assert not should_apply_lead_accel_reference(
    **common,
    stable_frames=LEAD_RESPONSE_CONFIRM_FRAMES - 1,
  )
  assert should_apply_lead_accel_reference(
    **common,
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
