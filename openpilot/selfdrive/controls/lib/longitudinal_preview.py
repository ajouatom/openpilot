"""Bounded lead deceleration preview and MPC lead-acceleration tuning."""

from __future__ import annotations

from dataclasses import dataclass
import math


DRIVING_MODE_ECO = 1
DRIVING_MODE_SAFE = 2
DRIVING_MODE_NORMAL = 3
DRIVING_MODE_HIGH = 4

LEAD_ACCEL_DEADBAND = 0.10
EGO_ACCEL_LIMIT = 3.0
PREVIEW_DECEL_ATTACK_STEP_S = 0.08
PREVIEW_RELEASE_STEP_S = 0.03
LEAD_ACCEL_RESPONSE_MIN = 0
LEAD_ACCEL_RESPONSE_MAX = 5
LEAD_ACCEL_CRUISE_RESPONSE_MIN = 3
LEAD_ACCEL_TF1_FORCE_MIN = 4
LEAD_ACCEL_MIN_TRACK_FRAMES = 3
CRUISE_SPEED_ERROR_DEADBAND = 1.0 / 3.6

# Preview is an offset from the calibrated actuator action time. These bounds
# affect only the existing early-deceleration preview. Positive lead response
# is solved inside MPC and never modifies its published targets afterward.
MIN_ACTION_TIME_S = 0.05
MAX_ACTION_TIME_S = 2.50


@dataclass(frozen=True)
class PreviewTuning:
  ego_accel_factor: float
  decel_factor: float
  decel_preview_max: float
  prebrake_floor: float
  predecel_delta_max: float | None
  active_decel_delta_max: float


@dataclass(frozen=True)
class LeadAccelResponseTuning:
  prediction_horizon: float
  closing_speed_floor: float
  a_change_cost_factor: float
  jerk_cost_factor: float


@dataclass(frozen=True)
class PreviewRequest:
  offset_s: float
  lead_accel_signal: float
  active: bool


@dataclass(frozen=True)
class LeadAccelMpcRequest:
  active: bool
  level: int = 0
  lead_accel_signal: float = 0.0
  a_change_cost_factor: float = 1.0
  jerk_cost_factor: float = 1.0


MODE_TUNING = {
  # Every mode uses the same relative-acceleration safety horizon for braking.
  # Mode character remains in the existing following-time, comfort-brake and
  # maximum-acceleration settings.
  DRIVING_MODE_SAFE: PreviewTuning(
    ego_accel_factor=1.0,
    decel_factor=1.00, decel_preview_max=1.50,
    prebrake_floor=-0.05, predecel_delta_max=None, active_decel_delta_max=0.10,
  ),
  DRIVING_MODE_ECO: PreviewTuning(
    ego_accel_factor=1.0,
    decel_factor=1.00, decel_preview_max=1.50,
    prebrake_floor=-0.04, predecel_delta_max=None, active_decel_delta_max=0.09,
  ),
  DRIVING_MODE_NORMAL: PreviewTuning(
    ego_accel_factor=1.0,
    decel_factor=1.00, decel_preview_max=1.50,
    prebrake_floor=-0.03, predecel_delta_max=None, active_decel_delta_max=0.08,
  ),
  DRIVING_MODE_HIGH: PreviewTuning(
    ego_accel_factor=1.0,
    decel_factor=1.00, decel_preview_max=1.50,
    prebrake_floor=-0.03, predecel_delta_max=None, active_decel_delta_max=0.08,
  ),
}


# Active ACC normally uses aChangeCost=200. Positive lead response reduces the
# acceleration-change and jerk costs inside MPC so that both vTargetNow and
# aTarget come from the same progressively stronger trajectory. The physical
# acceleration, lead-distance, danger-zone, turn and cut-in limits are not
# changed by these factors.
LEAD_ACCEL_RESPONSE_TUNING = {
  1: LeadAccelResponseTuning(0.00, 0.00, 0.85, 0.95),
  2: LeadAccelResponseTuning(0.10, -0.05, 0.65, 0.80),
  3: LeadAccelResponseTuning(0.25, -0.10, 0.40, 0.60),
  4: LeadAccelResponseTuning(0.40, -0.15, 0.18, 0.35),
  5: LeadAccelResponseTuning(0.50, -0.20, 0.05, 0.15),
}


def _mode_value(driving_mode) -> int:
  return int(getattr(driving_mode, "value", driving_mode))


def _deadzone(value: float, deadband: float) -> float:
  if value > deadband:
    return value - deadband
  if value < -deadband:
    return value + deadband
  return 0.0


def _lead_accel_signal(a_lead: float, a_ego: float, ego_accel_factor: float = 1.0) -> float:
  bounded_ego_accel = max(-EGO_ACCEL_LIMIT, min(EGO_ACCEL_LIMIT, float(a_ego)))
  return _deadzone(float(a_lead) - ego_accel_factor * bounded_ego_accel, LEAD_ACCEL_DEADBAND)


def clip_lead_accel_response_level(level: int) -> int:
  return max(LEAD_ACCEL_RESPONSE_MIN, min(LEAD_ACCEL_RESPONSE_MAX, int(level)))


def lead_accel_response_source_allowed(level: int, source: str) -> bool:
  response_level = clip_lead_accel_response_level(level)
  return source in ('lead0', 'lead1') or (
    source == 'cruise' and response_level >= LEAD_ACCEL_CRUISE_RESPONSE_MIN
  )


def lead_accel_response_allowed(level: int, *, v_rel: float, gap_margin: float,
                                lead_accel_signal: float, a_lead: float,
                                cruise_source_active: bool = False) -> bool:
  """Allow positive MPC response only while the lead is safely pulling away."""
  response_level = clip_lead_accel_response_level(level)
  tuning = LEAD_ACCEL_RESPONSE_TUNING.get(response_level)
  values = (v_rel, gap_margin, lead_accel_signal, a_lead)
  if tuning is None or not all(math.isfinite(value) for value in values):
    return False
  if cruise_source_active and response_level < LEAD_ACCEL_CRUISE_RESPONSE_MIN:
    return False

  # Level 5 is the explicit maximum-response test mode. Levels 3-5 use raw
  # positive lead acceleration with a cruise source because relative
  # acceleration may be near zero while an existing gap is opening.
  positive_lead_accel = a_lead > LEAD_ACCEL_DEADBAND
  # Cost reduction is only for catching back up to the configured TF. Once
  # that distance is reached, normal MPC costs resume and maintain the gap.
  if (gap_margin <= 0.0 or
      v_rel < tuning.closing_speed_floor or
      not positive_lead_accel or
      (response_level < LEAD_ACCEL_RESPONSE_MAX and not cruise_source_active and lead_accel_signal <= 0.0)):
    return False

  predicted_v_rel = float(v_rel) + float(lead_accel_signal) * tuning.prediction_horizon
  return predicted_v_rel >= 0.0


def get_lead_accel_mpc_request(
  level: int,
  *,
  enabled: bool,
  source: str,
  lead_status: bool,
  a_lead: float,
  a_ego: float,
  v_rel: float,
  gap_margin: float,
  speed_error: float = math.inf,
) -> LeadAccelMpcRequest:
  """Return MPC cost factors for a stable, positively accelerating lead."""
  response_level = clip_lead_accel_response_level(level)
  values = (a_lead, a_ego, v_rel, gap_margin)
  if (not enabled or not lead_status or
      not lead_accel_response_source_allowed(response_level, source) or
      not all(math.isfinite(value) for value in values)):
    return LeadAccelMpcRequest(False)

  lead_accel_signal = _lead_accel_signal(a_lead, a_ego)
  cruise_source_active = source == 'cruise'
  if cruise_source_active and (
    not math.isfinite(speed_error) or speed_error <= CRUISE_SPEED_ERROR_DEADBAND
  ):
    return LeadAccelMpcRequest(False, level=response_level, lead_accel_signal=lead_accel_signal)
  if not lead_accel_response_allowed(
    response_level,
    v_rel=v_rel,
    gap_margin=gap_margin,
    lead_accel_signal=lead_accel_signal,
    a_lead=a_lead,
    cruise_source_active=cruise_source_active,
  ):
    return LeadAccelMpcRequest(False, level=response_level, lead_accel_signal=lead_accel_signal)

  tuning = LEAD_ACCEL_RESPONSE_TUNING[response_level]
  return LeadAccelMpcRequest(
    True,
    level=response_level,
    lead_accel_signal=lead_accel_signal,
    a_change_cost_factor=tuning.a_change_cost_factor,
    jerk_cost_factor=tuning.jerk_cost_factor,
  )


def get_lead_preview_request(
  driving_mode,
  *,
  lead_status: bool,
  a_lead: float,
  a_ego: float = 0.0,
) -> PreviewRequest:
  """Map negative relative acceleration to an early-deceleration preview."""
  tuning = MODE_TUNING.get(_mode_value(driving_mode))
  if tuning is None or not lead_status or not all(math.isfinite(value) for value in (a_lead, a_ego)):
    return PreviewRequest(0.0, 0.0, False)

  lead_accel_signal = _lead_accel_signal(a_lead, a_ego, tuning.ego_accel_factor)
  offset_s = (
    min(-lead_accel_signal * tuning.decel_factor, tuning.decel_preview_max)
    if lead_accel_signal < 0.0 else 0.0
  )
  return PreviewRequest(float(offset_s), float(lead_accel_signal), True)


def rate_limit_preview(
  target_s: float,
  current_s: float,
  decel_attack_step_s: float = PREVIEW_DECEL_ATTACK_STEP_S,
  release_step_s: float = PREVIEW_RELEASE_STEP_S,
) -> float:
  """Advance braking preview quickly and release it progressively."""
  increasing_decel_preview = target_s > current_s and target_s > 0.0
  step = decel_attack_step_s if increasing_decel_preview else release_step_s
  step = max(0.0, float(step))
  return float(max(current_s - step, min(current_s + step, target_s)))


def clip_action_time(base_action_t: float, preview_s: float) -> float:
  return float(max(MIN_ACTION_TIME_S, min(MAX_ACTION_TIME_S, base_action_t + preview_s)))


def clip_preview_offset(base_action_t: float, preview_s: float) -> float:
  """Return the effective offset after enforcing the MPC action-time bounds."""
  return float(clip_action_time(base_action_t, preview_s) - float(base_action_t))


def apply_preview_target(
  base_target: float,
  preview_target: float,
  driving_mode,
  lead_accel_signal: float,
) -> float:
  """Apply only bounded early deceleration; never add acceleration post-MPC."""
  tuning = MODE_TUNING.get(_mode_value(driving_mode))
  base = float(base_target)
  if tuning is None or lead_accel_signal >= 0.0:
    return base

  candidate = min(float(preview_target), base)
  if base > 0.0:
    candidate = max(candidate, tuning.prebrake_floor)
    if tuning.predecel_delta_max is not None:
      candidate = max(candidate, base - tuning.predecel_delta_max)
  else:
    candidate = max(candidate, base - tuning.active_decel_delta_max)
  return float(candidate)
