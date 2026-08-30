"""Lead-acceleration reference generation for longitudinal MPC.

The reference starts early and small. Hard safety remains in the MPC obstacle
and danger constraints; this module only shapes how strongly normal following
tries to reproduce the physical lead vehicle's acceleration.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


LEAD_RESPONSE_SMOOTH = 0
LEAD_RESPONSE_BALANCED = 1
LEAD_RESPONSE_SYNC = 2


@dataclass(frozen=True)
class LeadResponseProfile:
  accel_gain: float
  far_decel_gain: float
  v_rel_gain: float
  gap_deficit_gain: float
  gap_surplus_gain: float
  attack_jerk: float
  release_jerk: float


@dataclass(frozen=True)
class LeadResponseReference:
  acceleration: np.ndarray
  raw_acceleration: np.ndarray
  effective_gain: float
  safety_blend: float
  gap_error: float


PROFILES = {
  LEAD_RESPONSE_SMOOTH: LeadResponseProfile(
    accel_gain=0.45,
    far_decel_gain=0.25,
    v_rel_gain=0.08,
    gap_deficit_gain=0.06,
    gap_surplus_gain=0.030,
    attack_jerk=0.8,
    release_jerk=0.6,
  ),
  LEAD_RESPONSE_BALANCED: LeadResponseProfile(
    accel_gain=0.72,
    far_decel_gain=0.55,
    v_rel_gain=0.14,
    gap_deficit_gain=0.09,
    gap_surplus_gain=0.020,
    attack_jerk=1.6,
    release_jerk=1.0,
  ),
  LEAD_RESPONSE_SYNC: LeadResponseProfile(
    accel_gain=1.00,
    far_decel_gain=0.90,
    v_rel_gain=0.20,
    gap_deficit_gain=0.12,
    gap_surplus_gain=0.010,
    attack_jerk=2.8,
    release_jerk=1.8,
  ),
}


def lead_response_profile(mode: int) -> LeadResponseProfile:
  return PROFILES.get(int(mode), PROFILES[LEAD_RESPONSE_BALANCED])


def _value(value: Any, name: str, default: float = 0.0) -> float:
  try:
    result = float(getattr(value, name))
  except (AttributeError, TypeError, ValueError):
    return float(default)
  return result if np.isfinite(result) else float(default)


def _rate_limit_reference(
  target: np.ndarray,
  initial_acceleration: float,
  time_indices: np.ndarray,
  attack_jerk: float,
  release_jerk: float,
) -> np.ndarray:
  limited = np.empty_like(target)
  previous = float(initial_acceleration)
  previous_time = 0.0
  for index, current_time in enumerate(time_indices):
    # Stage zero is still one planner cycle ahead of the previous solution.
    dt = max(0.05 if index == 0 else float(current_time - previous_time), 1e-3)
    requested = float(target[index])
    if requested < previous:
      current = max(requested, previous - attack_jerk * dt)
    else:
      current = min(requested, previous + release_jerk * dt)
    limited[index] = current
    previous = current
    previous_time = float(current_time)
  return limited


def build_lead_accel_trajectory(
  *,
  acceleration: float,
  acceleration_tau: float,
  jerk: float,
  time_indices: np.ndarray,
  time_differences: np.ndarray,
) -> np.ndarray:
  """Extrapolate lead acceleration while preserving jerk's physical units."""
  time_indices = np.asarray(time_indices)
  time_differences = np.asarray(time_differences)
  jerk_tau = float(np.interp(jerk, [-2.0, 0.0, 2.0], [0.2, 2.0, 0.1]))
  jerk_trajectory = jerk * np.exp(
    -jerk_tau * time_indices ** 2 / 2.0,
  )
  jerk_acceleration = np.cumsum(time_differences * jerk_trajectory)
  return np.clip(
    acceleration * np.exp(
      -acceleration_tau * time_indices ** 2 / 2.0,
    ) + jerk_acceleration,
    -10.0,
    5.0,
  )


def build_lead_accel_reference(
  lead: Any,
  *,
  mode: int,
  v_ego: float,
  desired_distance: float,
  previous_acceleration: float,
  time_indices: np.ndarray,
) -> LeadResponseReference | None:
  if (
    lead is None
    or not bool(getattr(lead, "status", False))
    or not bool(getattr(lead, "radar", False))
  ):
    return None

  profile = lead_response_profile(mode)
  d_rel = max(0.0, _value(lead, "dRel"))
  v_rel = float(np.clip(_value(lead, "vRel"), -8.0, 8.0))
  a_lead = float(np.clip(_value(lead, "aLeadK", _value(lead, "aLead")), -5.0, 3.0))
  a_lead_tau = float(np.clip(_value(lead, "aLeadTau", 1.5), 0.05, 3.0))
  gap_error = d_rel - max(0.0, float(desired_distance))
  surplus_time = max(0.0, gap_error) / max(float(v_ego), 5.0)

  # A smooth profile spends available distance instead of reproducing a wild
  # lead exactly. Sync retains almost all lead acceleration even when far back.
  far_scale = float(np.interp(
    surplus_time,
    [0.0, 1.2],
    [1.0, profile.far_decel_gain],
  ))
  base_gain = profile.accel_gain * (far_scale if a_lead < 0.0 else 1.0)

  closing_ttc = d_rel / max(-v_rel, 0.1) if v_rel < -0.1 else 99.0
  deficit_fraction = max(0.0, -gap_error) / max(float(desired_distance), 1.0)
  ttc_fraction = float(np.clip((4.0 - closing_ttc) / 2.5, 0.0, 1.0))
  safety_blend = float(np.clip(deficit_fraction + ttc_fraction, 0.0, 1.0))
  effective_gain = base_gain + safety_blend * (1.0 - base_gain)
  safety_profile = PROFILES[LEAD_RESPONSE_SYNC]
  effective_v_rel_gain = (
    profile.v_rel_gain
    + safety_blend * (safety_profile.v_rel_gain - profile.v_rel_gain)
  )
  effective_gap_gain = (
    profile.gap_deficit_gain
    + safety_blend * (
      safety_profile.gap_deficit_gain - profile.gap_deficit_gain
    )
  )

  lead_acceleration = (
    a_lead * np.exp(-a_lead_tau * np.asarray(time_indices) ** 2 / 2.0)
  )
  v_rel_feedback = effective_v_rel_gain * v_rel
  gap_deficit_feedback = effective_gap_gain * min(gap_error, 0.0)
  gap_surplus_feedback = profile.gap_surplus_gain * min(
    max(gap_error, 0.0), 20.0,
  )
  if a_lead < 0.0:
    # Surplus distance may soften braking, but it must never turn a newly
    # observed lead deceleration into an acceleration request.
    gap_surplus_feedback = np.minimum(
      gap_surplus_feedback,
      -0.8 * effective_gain * lead_acceleration,
    )
  raw_reference = np.clip(
    effective_gain * lead_acceleration
    + v_rel_feedback
    + gap_deficit_feedback
    + gap_surplus_feedback,
    -4.0,
    2.5,
  )

  attack_jerk = profile.attack_jerk + safety_blend * (3.5 - profile.attack_jerk)
  release_jerk = profile.release_jerk + safety_blend * (2.0 - profile.release_jerk)
  reference = _rate_limit_reference(
    raw_reference,
    previous_acceleration,
    np.asarray(time_indices),
    attack_jerk,
    release_jerk,
  )
  return LeadResponseReference(
    acceleration=reference,
    raw_acceleration=raw_reference,
    effective_gain=float(effective_gain),
    safety_blend=safety_blend,
    gap_error=float(gap_error),
  )


__all__ = (
  "LEAD_RESPONSE_BALANCED",
  "LEAD_RESPONSE_SMOOTH",
  "LEAD_RESPONSE_SYNC",
  "LeadResponseReference",
  "build_lead_accel_trajectory",
  "build_lead_accel_reference",
  "lead_response_profile",
)
