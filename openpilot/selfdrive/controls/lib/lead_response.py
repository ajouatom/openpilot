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
LEAD_RESPONSE_CRUISE_TAPER_TIME = 1.0
# The hard MPC obstacle is active on the first frame. Acceleration feed-forward
# starts on the second frame and reaches full confidence after 0.3 seconds.
LEAD_RESPONSE_MIN_TRACK_FRAMES = 2
LEAD_RESPONSE_CONFIRM_FRAMES = 6
LEAD_RESPONSE_BLEND_HORIZON = 1.5
LEAD_RESPONSE_WEIGHT_ATTACK = 0.25
LEAD_RESPONSE_WEIGHT_RELEASE = 0.20
# Near a stop, obstacle distance is the authoritative signal. Radar-derived
# acceleration is too noisy to improve the existing MPC stop trajectory.
LEAD_RESPONSE_MIN_LEAD_SPEED = 2.0
LEAD_SAFETY_JERK_FACTOR = 0.50


@dataclass(frozen=True)
class LeadResponseProfile:
  accel_gain: float
  far_decel_gain: float
  v_rel_gain: float
  gap_deficit_gain: float
  gap_surplus_gain: float
  tracking_feedback_limit: float
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
    # Smooth keeps normal control authority. Comfort comes from the lower
    # jerk rates and softer far-lead response, not from waiting until the MPC
    # has consumed the available following distance.
    accel_gain=0.70,
    far_decel_gain=0.35,
    v_rel_gain=0.10,
    gap_deficit_gain=0.07,
    gap_surplus_gain=0.025,
    tracking_feedback_limit=0.15,
    attack_jerk=0.8,
    release_jerk=0.8,
  ),
  LEAD_RESPONSE_BALANCED: LeadResponseProfile(
    accel_gain=0.72,
    far_decel_gain=0.55,
    v_rel_gain=0.14,
    gap_deficit_gain=0.09,
    gap_surplus_gain=0.020,
    tracking_feedback_limit=0.18,
    attack_jerk=1.6,
    release_jerk=1.0,
  ),
  LEAD_RESPONSE_SYNC: LeadResponseProfile(
    accel_gain=1.00,
    far_decel_gain=0.90,
    v_rel_gain=0.20,
    gap_deficit_gain=0.12,
    gap_surplus_gain=0.010,
    tracking_feedback_limit=0.25,
    attack_jerk=2.8,
    release_jerk=1.8,
  ),
}


def lead_response_profile(mode: int) -> LeadResponseProfile:
  return PROFILES.get(int(mode), PROFILES[LEAD_RESPONSE_BALANCED])


def lead_response_mode_for_driving_mode(driving_mode: int) -> int:
  """Use the existing drive mode as the single user-facing response control."""
  return {
    1: LEAD_RESPONSE_SMOOTH,    # Smooth
    2: LEAD_RESPONSE_BALANCED,  # Balanced
    3: LEAD_RESPONSE_SYNC,      # Sync
    4: LEAD_RESPONSE_SYNC,      # High: Sync + traffic-signal bypass
  }.get(int(driving_mode), LEAD_RESPONSE_BALANCED)


def lead_safety_jerk_factor(nominal_factor: float, braking_urgency: float) -> float:
  """Remove comfort-mode MPC inertia as collision urgency rises."""
  nominal_factor = max(float(nominal_factor), 0.0)
  urgency_blend = float(np.interp(
    np.clip(float(braking_urgency), 0.0, 1.0),
    [0.3, 1.0],
    [0.0, 1.0],
  ))
  safety_factor = min(nominal_factor, LEAD_SAFETY_JERK_FACTOR)
  return nominal_factor + urgency_blend * (safety_factor - nominal_factor)


def auto_driving_mode_for_congestion(auto_mode: int, congested: bool) -> int:
  """Select the configured calm/clear drive-mode pair."""
  calm_mode, clear_mode = {
    1: (1, 2),  # Smooth <-> Balanced
    2: (1, 3),  # Smooth <-> Sync
    3: (2, 3),  # Balanced <-> Sync
  }.get(int(auto_mode), (2, 2))
  return calm_mode if congested else clear_mode


def equal_lead_t_follow_adjustment(jerks: list[float], dynamic_factor: float) -> float:
  """Combine simultaneous lead adjustments without leadOne/leadTwo priority."""
  if not jerks or dynamic_factor <= 0.0:
    return 0.0
  adjustments = [
    float(np.interp(jerk, [-3.0, -0.5, 0.5, 2.0], [1.0, 0.0, 0.0, -1.0])) * dynamic_factor
    for jerk in jerks
  ]
  return float(max(adjustments))


def calculate_lead_braking_urgency(
  v_ego: float,
  leads: tuple[tuple[bool, float, float], ...],
) -> float:
  """Return equal, order-independent TTC/headway urgency for all MPC leads."""
  urgency = 0.0
  for visible, distance, relative_speed in leads:
    if not visible or not np.isfinite(distance) or not np.isfinite(relative_speed) or distance <= 0.0:
      continue
    closing_speed = max(0.0, -float(relative_speed))
    if closing_speed <= 0.1:
      continue
    ttc = float(distance) / closing_speed
    ttc_urgency = float(np.interp(ttc, [1.5, 5.0], [1.0, 0.0]))
    headway = float(distance) / max(float(v_ego), 1.0)
    closing_ratio = closing_speed / max(float(v_ego), 1.0)
    gap_urgency = float(np.interp(headway, [0.35, 0.9], [1.0, 0.0])) * min(1.0, closing_ratio * 2.0)
    urgency = max(urgency, ttc_urgency, gap_urgency)
  return float(np.clip(urgency, 0.0, 1.0))


def combine_braking_urgency_with_margin(lead_urgency: float, predicted_danger_margin: float) -> float:
  """Add the MPC's predicted obstacle margin to the direct closing urgency."""
  margin_urgency = float(np.interp(predicted_danger_margin, [-3.0, 0.0], [1.0, 0.0]))
  return float(np.clip(max(lead_urgency, margin_urgency), 0.0, 1.0))


def should_apply_lead_accel_reference(
  *,
  reset_state: bool,
  mpc_mode: str,
  source: str,
  stable_frames: int,
  lead_speed: float,
) -> bool:
  """Compatibility predicate for whether continuous lead response may start.

  ``source`` is intentionally not a gate. A stable moving lead may contribute a
  small preview response while cruise still limits the immediate MPC obstacle.
  """
  del source
  return lead_response_confidence(
    reset_state=reset_state,
    mpc_mode=mpc_mode,
    stable_frames=stable_frames,
    lead_speed=lead_speed,
  ) > 0.0


def lead_response_confidence(
  *,
  reset_state: bool,
  mpc_mode: str,
  stable_frames: int,
  lead_speed: float,
) -> float:
  if (
    not reset_state
    and mpc_mode == "acc"
    and np.isfinite(lead_speed)
    and lead_speed > LEAD_RESPONSE_MIN_LEAD_SPEED
    and stable_frames >= LEAD_RESPONSE_MIN_TRACK_FRAMES
  ):
    return float(np.interp(
      stable_frames,
      [LEAD_RESPONSE_MIN_TRACK_FRAMES - 1, LEAD_RESPONSE_CONFIRM_FRAMES],
      [0.0, 1.0],
    ))
  return 0.0


def lead_obstacle_relevance(
  lead_obstacle: np.ndarray,
  cruise_obstacle: np.ndarray,
  v_ego: float,
  time_indices: np.ndarray,
) -> float:
  """Return a continuous lead/cruise blend weight from predicted obstacles.

  Positive advantage means the lead is the tighter obstacle. Looking ahead is
  what permits a small response before the old instantaneous source switch.
  """
  lead_obstacle = np.asarray(lead_obstacle, dtype=float)
  cruise_obstacle = np.asarray(cruise_obstacle, dtype=float)
  time_indices = np.asarray(time_indices, dtype=float)
  count = min(len(lead_obstacle), len(cruise_obstacle), len(time_indices))
  if count == 0:
    return 0.0
  valid = (
    np.isfinite(lead_obstacle[:count])
    & np.isfinite(cruise_obstacle[:count])
    & np.isfinite(time_indices[:count])
    & (time_indices[:count] <= LEAD_RESPONSE_BLEND_HORIZON)
  )
  if not np.any(valid):
    return 0.0
  advantage = float(np.max(cruise_obstacle[:count][valid] - lead_obstacle[:count][valid]))
  blend_distance = float(np.clip(max(float(v_ego), 0.0) * 0.6, 3.0, 12.0))
  return float(np.interp(advantage, [-blend_distance, blend_distance], [0.0, 1.0]))


def rate_limit_lead_response_weight(previous: float, target: float) -> float:
  previous = float(np.clip(previous, 0.0, 1.0))
  target = float(np.clip(target, 0.0, 1.0))
  if target > previous:
    return min(target, previous + LEAD_RESPONSE_WEIGHT_ATTACK)
  return max(target, previous - LEAD_RESPONSE_WEIGHT_RELEASE)


def blend_lead_accel_reference(
  previous_acceleration: np.ndarray,
  lead_acceleration: np.ndarray,
  weight: float,
) -> np.ndarray:
  """Continuously blend a lead reference without preview acceleration surges."""
  previous_acceleration = np.asarray(previous_acceleration, dtype=float)
  lead_acceleration = np.asarray(lead_acceleration, dtype=float)
  weight = float(np.clip(weight, 0.0, 1.0))
  delta = lead_acceleration - previous_acceleration
  # Early negative response is useful; early positive response is not. Requiring
  # squared confidence for acceleration also prevents a far lead suppressing
  # or replacing the cruise target as source weights chatter near equality.
  applied_weight = np.where(delta <= 0.0, weight, weight * weight)
  return previous_acceleration + applied_weight * delta


def combine_lead_accel_references(
  previous_acceleration: np.ndarray,
  candidates: list[np.ndarray],
) -> np.ndarray:
  """Treat leadOne and leadTwo equally; the more restrictive reference wins."""
  if not candidates:
    return np.copy(previous_acceleration)
  return np.minimum.reduce([np.asarray(candidate, dtype=float) for candidate in candidates])


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
  v_cruise: float,
  desired_distance: float,
  previous_acceleration: float,
  time_indices: np.ndarray,
  braking_urgency: float = 0.0,
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
  urgency = float(np.clip(braking_urgency, 0.0, 1.0))
  control_blend = max(safety_blend, urgency)
  effective_gain = base_gain + control_blend * (1.0 - base_gain)
  safety_profile = PROFILES[LEAD_RESPONSE_SYNC]
  effective_v_rel_gain = (
    profile.v_rel_gain
    + control_blend * (safety_profile.v_rel_gain - profile.v_rel_gain)
  )
  effective_gap_gain = (
    profile.gap_deficit_gain
    + control_blend * (
      safety_profile.gap_deficit_gain - profile.gap_deficit_gain
    )
  )
  effective_gap_surplus_gain = (
    profile.gap_surplus_gain
    + control_blend * (
      safety_profile.gap_surplus_gain - profile.gap_surplus_gain
    )
  )
  effective_feedback_limit = (
    profile.tracking_feedback_limit
    + control_blend * (
      safety_profile.tracking_feedback_limit
      - profile.tracking_feedback_limit
    )
  )

  lead_acceleration = (
    a_lead * np.exp(-a_lead_tau * np.asarray(time_indices) ** 2 / 2.0)
  )
  v_rel_feedback = effective_v_rel_gain * v_rel
  gap_deficit_feedback = effective_gap_gain * min(gap_error, 0.0)
  gap_surplus_feedback = effective_gap_surplus_gain * min(
    max(gap_error, 0.0), 20.0,
  )
  if a_lead < 0.0:
    # Surplus distance may soften braking, but it must never turn a newly
    # observed lead deceleration into an acceleration request.
    gap_surplus_feedback = np.minimum(
      gap_surplus_feedback,
      -0.8 * effective_gain * lead_acceleration,
    )
  # Distance and relative speed are already represented by the hard MPC
  # obstacle. Keep only a bounded correction here. Normally it remains gated
  # by a lead acceleration event; verified closing urgency may open it even
  # after aLead has returned near zero, avoiding a brake-release-brake cycle.
  motion_activity = float(np.interp(abs(a_lead), [0.15, 0.75], [0.0, 1.0]))
  tracking_activity = max(motion_activity, urgency)
  tracking_feedback = tracking_activity * np.clip(
    v_rel_feedback + gap_deficit_feedback + gap_surplus_feedback,
    -effective_feedback_limit,
    effective_feedback_limit,
  )
  raw_reference = np.clip(
    effective_gain * lead_acceleration + tracking_feedback,
    -4.0,
    2.5,
  )

  # Lead response is acceleration feed-forward, not a second speed target.
  # Taper its positive contribution as cruise-speed headroom closes so a far
  # lead cannot pull the ego vehicle above the driver's set speed. Negative
  # references remain untouched for early braking and obstacle safety.
  cruise_headroom = max(0.0, float(v_cruise) - float(v_ego))
  positive_accel_ceiling = cruise_headroom / LEAD_RESPONSE_CRUISE_TAPER_TIME
  raw_reference = np.minimum(raw_reference, positive_accel_ceiling)

  # A modest gap deficit may increase how closely the reference matches the
  # lead, but it must not independently open an emergency jerk rate. Hard MPC
  # safety remains authoritative and exposes extra rate only through urgency.
  attack_jerk = profile.attack_jerk + urgency * (3.5 - profile.attack_jerk)
  release_jerk = profile.release_jerk + urgency * (2.0 - profile.release_jerk)
  reference = _rate_limit_reference(
    raw_reference,
    previous_acceleration,
    np.asarray(time_indices),
    attack_jerk,
    release_jerk,
  )
  # Do not let release rate limiting preserve a stale positive acceleration
  # after the cruise-speed headroom has disappeared.
  reference = np.minimum(reference, positive_accel_ceiling)
  return LeadResponseReference(
    acceleration=reference,
    raw_acceleration=raw_reference,
    effective_gain=float(effective_gain),
    safety_blend=safety_blend,
    gap_error=float(gap_error),
  )


__all__ = (
  "LEAD_SAFETY_JERK_FACTOR",
  "LEAD_RESPONSE_BALANCED",
  "LEAD_RESPONSE_BLEND_HORIZON",
  "LEAD_RESPONSE_CONFIRM_FRAMES",
  "LEAD_RESPONSE_CRUISE_TAPER_TIME",
  "LEAD_RESPONSE_MIN_LEAD_SPEED",
  "LEAD_RESPONSE_MIN_TRACK_FRAMES",
  "LEAD_RESPONSE_SMOOTH",
  "LEAD_RESPONSE_SYNC",
  "LeadResponseReference",
  "auto_driving_mode_for_congestion",
  "blend_lead_accel_reference",
  "build_lead_accel_trajectory",
  "build_lead_accel_reference",
  "calculate_lead_braking_urgency",
  "combine_braking_urgency_with_margin",
  "combine_lead_accel_references",
  "equal_lead_t_follow_adjustment",
  "lead_obstacle_relevance",
  "lead_response_confidence",
  "lead_response_mode_for_driving_mode",
  "lead_response_profile",
  "lead_safety_jerk_factor",
  "rate_limit_lead_response_weight",
  "should_apply_lead_accel_reference",
)
