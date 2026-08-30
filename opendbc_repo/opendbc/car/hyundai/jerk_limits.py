import numpy as np


CANFD_JERK_UPPER_MIN = 1.0
CANFD_JERK_LIMIT_MAX = 5.0
CANFD_JERK_ERROR_DEADBAND = 0.25
CANFD_JERK_ERROR_FULL_SCALE = 0.5
CANFD_JERK_RELEASE_THRESHOLD = 0.1
CANFD_JERK_LOWER_ACCEL_BP = [0.0, 0.8, 1.2, 1.5, 2.0, 2.5, 3.2]
CANFD_JERK_LOWER_LIMIT_V = [1.2, 1.2, 1.2, 1.7, 3.0, 3.3, 3.7]
CANFD_JERK_LOWER_MODE_CAPS = [1.8, 3.0, 5.0]


def calculate_lead_braking_urgency(v_ego: float, leads: tuple[tuple[bool, float, float], ...]) -> float:
  """Return equal, order-independent braking urgency for leadOne/leadTwo."""
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


def calculate_canfd_jerk_limits(accel: float, jerk: float, tracking_error: float = 0.0,
                                braking_urgency: float = 0.0, response_mode: int | None = None) -> tuple[float, float]:
  if response_mode is None:
    lower_cap = CANFD_JERK_LIMIT_MAX
  else:
    mode = int(np.clip(response_mode, 0, len(CANFD_JERK_LOWER_MODE_CAPS) - 1))
    normal_lower_cap = CANFD_JERK_LOWER_MODE_CAPS[mode]
    request_urgency = float(np.interp(max(0.0, -accel), [1.5, 4.0], [0.0, 1.0]))
    urgency = max(float(np.clip(braking_urgency, 0.0, 1.0)), request_urgency)
    lower_cap = normal_lower_cap + urgency * (CANFD_JERK_LIMIT_MAX - normal_lower_cap)

  # Do not slow the release from braking. Comfort modes shape deceleration
  # onset only; jerkUpper retains full historical authority to avoid a tail.
  jerk_u = np.clip(jerk * 2.0, CANFD_JERK_UPPER_MIN, CANFD_JERK_LIMIT_MAX)
  jerk_l_mpc = np.clip(-jerk * 4.0, 1.0, lower_cap)

  # Acceleration demand only assists after sustained measured under-deceleration.
  # A positive jerk releases the assist immediately, preventing braking tails.
  assist_ratio = 0.0
  if jerk <= CANFD_JERK_RELEASE_THRESHOLD:
    assist_ratio = np.clip((tracking_error - CANFD_JERK_ERROR_DEADBAND) / CANFD_JERK_ERROR_FULL_SCALE, 0.0, 1.0)

  decel_request = max(0.0, -accel)
  jerk_l_feedforward = np.interp(decel_request, CANFD_JERK_LOWER_ACCEL_BP, CANFD_JERK_LOWER_LIMIT_V)
  jerk_l_assist = 1.0 + assist_ratio * (jerk_l_feedforward - 1.0)
  jerk_l = np.clip(max(jerk_l_mpc, jerk_l_assist), 1.0, lower_cap)
  return float(jerk_u), float(jerk_l)


__all__ = (
  "CANFD_JERK_ERROR_DEADBAND",
  "CANFD_JERK_ERROR_FULL_SCALE",
  "CANFD_JERK_LIMIT_MAX",
  "CANFD_JERK_RELEASE_THRESHOLD",
  "CANFD_JERK_UPPER_MIN",
  "calculate_canfd_jerk_limits",
  "calculate_lead_braking_urgency",
)
