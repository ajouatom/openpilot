from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lead_response import (
  LEAD_RESPONSE_BALANCED,
  LEAD_RESPONSE_SMOOTH,
  LEAD_RESPONSE_SYNC,
  build_lead_accel_trajectory,
  build_lead_accel_reference,
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
