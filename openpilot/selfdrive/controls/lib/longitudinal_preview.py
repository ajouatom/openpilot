"""Bounded drive-mode preview of the longitudinal MPC trajectory."""

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
CRUISE_SPEED_ERROR_DEADBAND = 1.0 / 3.6
CRUISE_SPEED_ERROR_ACCEL_GAIN = 0.80

# Preview is an offset from the calibrated actuator action time.  These caps
# keep the request inside the published 2.5 s MPC trajectory and prevent a
# large lead acceleration from scaling without bound. The preview caps below,
# rather than these trajectory bounds, limit how far a configured actuator
# delay can move.
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
  preview_factor: float
  preview_max: float
  target_delta_max: float
  prediction_horizon: float
  closing_speed_floor: float
  gap_margin_floor: float
  boost_gain: float = 0.0
  boost_max: float = 0.0
  boost_attack_step: float = 0.0
  lead_accel_overshoot_max: float = math.inf
  cruise_accel_fraction: float = 0.0
  cruise_accel_attack_step: float = 0.0


@dataclass(frozen=True)
class PreviewRequest:
  offset_s: float
  lead_accel_signal: float
  active: bool
  accel_response_active: bool = False
  accel_response_level: int = 0
  accel_boost_target: float = 0.0
  preview_attack_step: float = PREVIEW_DECEL_ATTACK_STEP_S
  boost_attack_step: float = 0.0
  cruise_source_active: bool = False


MODE_TUNING = {
  # Every mode uses the same relative-acceleration safety horizon for braking.
  # Mode character is kept on the acceleration side and by each mode's
  # existing following-time, comfort-brake and maximum-acceleration settings.
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


# Positive lead-acceleration response is intentionally independent of drive
# mode. It is available only while the driver has selected following-distance
# level 1; the mode-specific acceleration envelope remains the final limit.
LEAD_ACCEL_RESPONSE_TUNING = {
  1: LeadAccelResponseTuning(0.20, 0.08, 0.08, 0.00, 0.00, 0.00),
  2: LeadAccelResponseTuning(0.35, 0.15, 0.18, 0.10, -0.05, -0.75),
  # Levels 3 through 5 may recover an opening TF1 gap while cruise is the
  # MPC source. Their direct cruise targets progressively use more of the
  # final CruiseMaxVals envelope, after turn and cut-in limits are applied.
  3: LeadAccelResponseTuning(0.55, 0.28, 0.45, 0.20, -0.10, -2.00,
                             lead_accel_overshoot_max=0.10,
                             cruise_accel_fraction=0.55, cruise_accel_attack_step=0.08),
  4: LeadAccelResponseTuning(0.80, 0.45, 0.65, 0.35, -0.15, -3.00,
                             lead_accel_overshoot_max=0.15,
                             cruise_accel_fraction=0.80, cruise_accel_attack_step=0.12),
  # Level 5 deliberately follows positive lead acceleration and permits a
  # small acceleration overshoot. The lead-acceleration cap and the vehicle's
  # final acceleration limit keep that response bounded.
  5: LeadAccelResponseTuning(1.10, 0.70, 0.80, 0.50, -0.20, -math.inf,
                             boost_gain=0.60, boost_max=0.50, boost_attack_step=0.20,
                             lead_accel_overshoot_max=0.20,
                             cruise_accel_fraction=1.00, cruise_accel_attack_step=0.20),
}


def _mode_value(driving_mode) -> int:
  return int(getattr(driving_mode, "value", driving_mode))


def _deadzone(value: float, deadband: float) -> float:
  if value > deadband:
    return value - deadband
  if value < -deadband:
    return value + deadband
  return 0.0


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
  """Allow TF1 acceleration preview with a bounded closing-speed check."""
  response_level = clip_lead_accel_response_level(level)
  tuning = LEAD_ACCEL_RESPONSE_TUNING.get(response_level)
  if tuning is None or not all(math.isfinite(value) for value in (v_rel, gap_margin, lead_accel_signal, a_lead)):
    return False
  if cruise_source_active and response_level < LEAD_ACCEL_CRUISE_RESPONSE_MIN:
    return False
  # Level 5 is the driver's explicit close-following test mode. Levels 3-5
  # also use raw positive lead acceleration while cruise controls the MPC,
  # since relative acceleration can be zero even as an existing gap opens.
  inside_target_gap = gap_margin < tuning.gap_margin_floor
  positive_lead_accel = a_lead > LEAD_ACCEL_DEADBAND
  use_raw_lead_accel = response_level == LEAD_ACCEL_RESPONSE_MAX or cruise_source_active
  if (inside_target_gap or v_rel < tuning.closing_speed_floor or
      (not use_raw_lead_accel and lead_accel_signal <= 0.0) or
      (use_raw_lead_accel and not positive_lead_accel)):
    return False

  predicted_v_rel = float(v_rel) + float(lead_accel_signal) * tuning.prediction_horizon
  return predicted_v_rel >= 0.0


def get_lead_preview_request(
  driving_mode,
  *,
  lead_status: bool,
  a_lead: float,
  a_ego: float = 0.0,
  accel_response_level: int = 0,
  accel_response_enabled: bool = False,
  cruise_source_active: bool = False,
  v_rel: float = 0.0,
  gap_margin: float = 0.0,
) -> PreviewRequest:
  """Map mode-weighted relative acceleration to a signed preview request."""
  tuning = MODE_TUNING.get(_mode_value(driving_mode))
  if tuning is None or not lead_status or not all(math.isfinite(value) for value in (a_lead, a_ego)):
    return PreviewRequest(0.0, 0.0, False)

  # Keep the preview signal on measured relative acceleration. Radar jerk is a
  # useful trend diagnostic, but its longer causal fit can lag rapid changes and
  # must not make the action-time request oscillate.
  bounded_ego_accel = max(-EGO_ACCEL_LIMIT, min(EGO_ACCEL_LIMIT, float(a_ego)))
  lead_accel_signal = (
    float(a_lead)
    - tuning.ego_accel_factor * bounded_ego_accel
  )
  lead_accel_signal = _deadzone(lead_accel_signal, LEAD_ACCEL_DEADBAND)

  response_level = clip_lead_accel_response_level(accel_response_level)
  accel_response_active = False
  accel_boost_target = 0.0
  preview_attack_step = PREVIEW_DECEL_ATTACK_STEP_S
  boost_attack_step = 0.0

  raw_lead_response = (
    (response_level == LEAD_ACCEL_RESPONSE_MAX or
     (cruise_source_active and response_level >= LEAD_ACCEL_CRUISE_RESPONSE_MIN))
    and accel_response_enabled
    and lead_accel_response_allowed(
      response_level, v_rel=v_rel, gap_margin=gap_margin,
      lead_accel_signal=lead_accel_signal, a_lead=float(a_lead),
      cruise_source_active=cruise_source_active,
    )
  )
  if raw_lead_response:
    response_tuning = LEAD_ACCEL_RESPONSE_TUNING[response_level]
    response_signal = _deadzone(float(a_lead), LEAD_ACCEL_DEADBAND)
    offset_s = min(response_signal * response_tuning.preview_factor, response_tuning.preview_max)
    accel_boost_target = min(response_signal * response_tuning.boost_gain, response_tuning.boost_max)
    accel_response_active = True
    preview_attack_step = response_tuning.preview_max
    boost_attack_step = response_tuning.boost_attack_step
  elif lead_accel_signal < 0.0:
    offset_s = min(-lead_accel_signal * tuning.decel_factor, tuning.decel_preview_max)
  elif (lead_accel_signal > 0.0 and accel_response_enabled and
        lead_accel_response_allowed(response_level, v_rel=v_rel, gap_margin=gap_margin,
                                    lead_accel_signal=lead_accel_signal, a_lead=float(a_lead),
                                    cruise_source_active=cruise_source_active)):
    response_tuning = LEAD_ACCEL_RESPONSE_TUNING[response_level]
    offset_s = min(lead_accel_signal * response_tuning.preview_factor, response_tuning.preview_max)
    accel_boost_target = min(lead_accel_signal * response_tuning.boost_gain, response_tuning.boost_max)
    accel_response_active = True
    # Acceleration preview should take effect on the first valid planning
    # cycle. Its output delta and optional level-5 boost remain independently
    # bounded below.
    preview_attack_step = response_tuning.preview_max
    boost_attack_step = response_tuning.boost_attack_step
  else:
    offset_s = 0.0

  return PreviewRequest(
    float(offset_s), float(lead_accel_signal), True,
    accel_response_active=accel_response_active,
    accel_response_level=response_level,
    accel_boost_target=float(accel_boost_target),
    preview_attack_step=float(preview_attack_step),
    boost_attack_step=float(boost_attack_step),
    cruise_source_active=bool(cruise_source_active and accel_response_active),
  )


def get_cruise_accel_target(
  base_target: float,
  *,
  accel_max: float,
  a_lead: float,
  speed_error: float,
  accel_response_level: int,
) -> float:
  """Return a bounded cruise-source acceleration target for TF1 levels 3-5."""
  response_level = clip_lead_accel_response_level(accel_response_level)
  tuning = LEAD_ACCEL_RESPONSE_TUNING.get(response_level)
  values = (base_target, accel_max, a_lead, speed_error)
  if (tuning is None or response_level < LEAD_ACCEL_CRUISE_RESPONSE_MIN or
      not all(math.isfinite(value) for value in values) or
      a_lead <= LEAD_ACCEL_DEADBAND or speed_error <= CRUISE_SPEED_ERROR_DEADBAND):
    return float(base_target)

  base = float(base_target)
  final_accel_max = max(0.0, float(accel_max))
  cruise_envelope = final_accel_max * tuning.cruise_accel_fraction
  speed_error_envelope = (
    float(speed_error) - CRUISE_SPEED_ERROR_DEADBAND
  ) * CRUISE_SPEED_ERROR_ACCEL_GAIN
  lead_envelope = float(a_lead) + tuning.lead_accel_overshoot_max
  target = min(
    cruise_envelope,
    speed_error_envelope,
    lead_envelope,
    base + tuning.target_delta_max,
    final_accel_max,
  )
  return float(max(base, target))


def rate_limit_cruise_accel_target(target: float, current: float, base: float,
                                   attack_step: float) -> float:
  """Raise a cruise response progressively and release reductions immediately."""
  target = max(float(base), float(target))
  current = max(float(base), float(current))
  if target <= current:
    return target
  return float(min(target, current + max(0.0, float(attack_step))))


def rate_limit_preview(
  target_s: float,
  current_s: float,
  decel_attack_step_s: float = PREVIEW_DECEL_ATTACK_STEP_S,
  release_step_s: float = PREVIEW_RELEASE_STEP_S,
) -> float:
  """Advance braking preview quickly and release/acceleration-preview slowly."""
  increasing_decel_preview = target_s > current_s and target_s > 0.0
  step = decel_attack_step_s if increasing_decel_preview else release_step_s
  step = max(0.0, float(step))
  return float(max(current_s - step, min(current_s + step, target_s)))


def clip_action_time(base_action_t: float, preview_s: float) -> float:
  return float(max(MIN_ACTION_TIME_S, min(MAX_ACTION_TIME_S, base_action_t + preview_s)))


def clip_preview_offset(base_action_t: float, preview_s: float) -> float:
  """Return the effective offset after enforcing the MPC action-time bounds."""
  return float(clip_action_time(base_action_t, preview_s) - float(base_action_t))


def rate_limit_accel_boost(target: float, current: float, attack_step: float,
                           release_step: float = 0.04, immediate_release: bool = False) -> float:
  """Keep level-5 feed-forward quick without exposing raw aLead noise."""
  if immediate_release and target <= 0.0:
    return 0.0
  step = max(0.0, float(attack_step if target > current else release_step))
  return float(max(current - step, min(current + step, target)))


def apply_preview_target(
  base_target: float,
  preview_target: float,
  driving_mode,
  lead_accel_signal: float,
  a_ego: float = 0.0,
  accel_response_active: bool = False,
  accel_response_level: int = 0,
  accel_boost: float = 0.0,
  accel_max: float = math.inf,
  a_lead: float = 0.0,
  cruise_source_active: bool = False,
  cruise_accel_target: float = -math.inf,
) -> float:
  """Keep mode preview conservative and bound its acceleration delta."""
  tuning = MODE_TUNING.get(_mode_value(driving_mode))
  base = float(base_target)
  candidate = float(preview_target)
  ego_not_braking = math.isfinite(a_ego) and float(a_ego) >= -LEAD_ACCEL_DEADBAND

  if tuning is None:
    return base

  response_tuning = LEAD_ACCEL_RESPONSE_TUNING.get(clip_lead_accel_response_level(accel_response_level))
  response_level = clip_lead_accel_response_level(accel_response_level)
  forceful_response = response_level == LEAD_ACCEL_RESPONSE_MAX
  conservative_gates_pass = (
    base >= 0.0 and ego_not_braking and (candidate >= base or cruise_source_active)
  )
  raw_lead_response = forceful_response or (
    cruise_source_active and response_level >= LEAD_ACCEL_CRUISE_RESPONSE_MIN
  )
  positive_response = lead_accel_signal > 0.0 or (
    raw_lead_response and math.isfinite(a_lead) and float(a_lead) > LEAD_ACCEL_DEADBAND
  )
  if (accel_response_active and response_tuning is not None and positive_response and
      (forceful_response or conservative_gates_pass)):
    # Level 5's direct feed-forward remains useful even while the MPC
    # trajectory is still releasing braking. Lower levels retain the
    # current-acceleration gate; cruise levels 3-4 may replace a falling
    # preview with their independently bounded CruiseMax target.
    target = max(base, candidate) + max(0.0, float(accel_boost))
    if cruise_source_active and math.isfinite(cruise_accel_target):
      target = max(target, float(cruise_accel_target))
    if raw_lead_response:
      target = min(target, float(a_lead) + response_tuning.lead_accel_overshoot_max)
    target = min(target, base + response_tuning.target_delta_max, float(accel_max))
    return float(max(base, target))

  if lead_accel_signal > 0.0:
    return base

  # Every mode may remove acceleration for a negative relative-acceleration
  # signal. Pre-braking remains mode-bounded; a positive signal can reduce
  # acceleration only as far as coasting.
  candidate = min(candidate, base)
  if base > 0.0:
    prebrake_floor = tuning.prebrake_floor if lead_accel_signal < 0.0 else 0.0
    candidate = max(candidate, prebrake_floor)
    if tuning.predecel_delta_max is not None:
      candidate = max(candidate, base - tuning.predecel_delta_max)
  else:
    candidate = max(candidate, base - tuning.active_decel_delta_max)
  return float(candidate)
