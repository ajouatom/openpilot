"""Bounded drive-mode preview of the longitudinal MPC trajectory."""

from __future__ import annotations

from dataclasses import dataclass
import math


DRIVING_MODE_ECO = 1
DRIVING_MODE_SAFE = 2
DRIVING_MODE_NORMAL = 3
DRIVING_MODE_HIGH = 4

LEAD_JERK_LOOKAHEAD_S = 0.25
LEAD_ACCEL_DEADBAND = 0.10
LEAD_JERK_LIMIT = 3.0
PREVIEW_DECEL_ATTACK_STEP_S = 0.08
PREVIEW_RELEASE_STEP_S = 0.03

# Preview is an offset from the calibrated actuator action time.  These caps
# keep the request inside the published 2.5 s MPC trajectory and prevent a
# large lead acceleration from scaling without bound. The preview caps below,
# rather than these trajectory bounds, limit how far a configured actuator
# delay can move.
MIN_ACTION_TIME_S = 0.05
MAX_ACTION_TIME_S = 2.50

MAX_HIGH_ACCEL_TARGET_DELTA = 0.15


@dataclass(frozen=True)
class PreviewTuning:
  accel_factor: float
  decel_factor: float
  accel_preview_max: float
  decel_preview_max: float
  prebrake_floor: float
  predecel_delta_max: float
  active_decel_delta_max: float
  accelerate_early: bool = False


@dataclass(frozen=True)
class PreviewRequest:
  offset_s: float
  lead_accel_signal: float
  active: bool


MODE_TUNING = {
  # Braking uses a longer horizon than acceleration because nearby MPC nodes
  # remain close together at the start of a lead deceleration. Safe responds
  # earliest with a small pre-brake; Eco is moderate; Normal preserves
  # positive-lead acceleration timing and previews braking only.
  DRIVING_MODE_SAFE: PreviewTuning(
    accel_factor=0.50, decel_factor=0.75,
    accel_preview_max=0.10, decel_preview_max=0.60,
    prebrake_floor=-0.05, predecel_delta_max=0.18, active_decel_delta_max=0.10,
  ),
  DRIVING_MODE_ECO: PreviewTuning(
    accel_factor=0.25, decel_factor=0.60,
    accel_preview_max=0.06, decel_preview_max=0.45,
    prebrake_floor=-0.04, predecel_delta_max=0.16, active_decel_delta_max=0.09,
  ),
  DRIVING_MODE_NORMAL: PreviewTuning(
    accel_factor=0.0, decel_factor=0.35,
    accel_preview_max=0.0, decel_preview_max=0.25,
    prebrake_floor=-0.03, predecel_delta_max=0.15, active_decel_delta_max=0.08,
  ),
  # High keeps the normal safety-side preview and looks farther ahead during a
  # confirmed lead acceleration.  Its existing max-accel factor remains the
  # sustained acceleration envelope.
  DRIVING_MODE_HIGH: PreviewTuning(
    accel_factor=0.30, decel_factor=0.35,
    accel_preview_max=0.10, decel_preview_max=0.25,
    prebrake_floor=-0.03, predecel_delta_max=0.15, active_decel_delta_max=0.08,
    accelerate_early=True,
  ),
}


def _mode_value(driving_mode) -> int:
  return int(getattr(driving_mode, "value", driving_mode))


def _deadzone(value: float, deadband: float) -> float:
  if value > deadband:
    return value - deadband
  if value < -deadband:
    return value + deadband
  return 0.0


def get_lead_preview_request(
  driving_mode,
  *,
  lead_status: bool,
  a_lead: float,
  j_lead: float,
) -> PreviewRequest:
  """Map lead acceleration/jerk to a bounded signed preview-time request."""
  tuning = MODE_TUNING.get(_mode_value(driving_mode))
  if tuning is None or not lead_status or not (math.isfinite(a_lead) and math.isfinite(j_lead)):
    return PreviewRequest(0.0, 0.0, False)

  # Convert jerk to a short-horizon acceleration contribution before combining
  # the signals.  This avoids adding m/s^3 directly to m/s^2 and lets jerk only
  # advance the onset of an already bounded preview response.
  bounded_jerk = max(-LEAD_JERK_LIMIT, min(LEAD_JERK_LIMIT, float(j_lead)))
  lead_accel_signal = float(a_lead) + LEAD_JERK_LOOKAHEAD_S * bounded_jerk
  lead_accel_signal = _deadzone(lead_accel_signal, LEAD_ACCEL_DEADBAND)

  if lead_accel_signal < 0.0:
    offset_s = min(-lead_accel_signal * tuning.decel_factor, tuning.decel_preview_max)
  elif lead_accel_signal > 0.0 and tuning.accel_factor > 0.0:
    magnitude = min(lead_accel_signal * tuning.accel_factor, tuning.accel_preview_max)
    offset_s = magnitude if tuning.accelerate_early else -magnitude
  else:
    offset_s = 0.0

  return PreviewRequest(float(offset_s), float(lead_accel_signal), True)


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


def apply_preview_target(
  base_target: float,
  preview_target: float,
  driving_mode,
  lead_accel_signal: float,
) -> float:
  """Keep mode preview conservative and bound its acceleration delta."""
  mode = _mode_value(driving_mode)
  tuning = MODE_TUNING.get(mode)
  base = float(base_target)
  candidate = float(preview_target)

  if tuning is None:
    return base

  if mode == DRIVING_MODE_HIGH and lead_accel_signal > 0.0:
    # High may advance a rising acceleration trajectory, but never by more than
    # the bounded delta.  The normal MPC maximum acceleration is still obeyed.
    return float(max(base, min(candidate, base + MAX_HIGH_ACCEL_TARGET_DELTA)))

  # Safe, Eco and Normal preview may only remove acceleration. A confirmed
  # negative lead signal may request a small, mode-bounded pre-brake; positive
  # lead acceleration can reduce acceleration only as far as coasting.
  candidate = min(candidate, base)
  if base > 0.0:
    prebrake_floor = tuning.prebrake_floor if lead_accel_signal < 0.0 else 0.0
    candidate = max(candidate, prebrake_floor, base - tuning.predecel_delta_max)
  else:
    candidate = max(candidate, base - tuning.active_decel_delta_max)
  return float(candidate)
