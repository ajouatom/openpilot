import numpy as np


CANFD_JERK_UPPER_MIN = 1.0
CANFD_JERK_LIMIT_MAX = 5.0
CANFD_JERK_ERROR_DEADBAND = 0.25
CANFD_JERK_ERROR_FULL_SCALE = 0.5
CANFD_JERK_RELEASE_THRESHOLD = 0.1
CANFD_JERK_LOWER_ACCEL_BP = [0.0, 0.8, 1.2, 1.5, 2.0, 2.5, 3.2]
CANFD_JERK_LOWER_LIMIT_V = [1.2, 1.2, 1.2, 1.7, 3.0, 3.3, 3.7]
def calculate_canfd_jerk_limits(accel: float, jerk: float, tracking_error: float = 0.0,
                                braking_urgency: float = 0.0) -> tuple[float, float]:
  urgency = float(np.clip(braking_urgency, 0.0, 1.0))

  # Comfort modes shape the command in MPC. The actuator layer retains the
  # historical jerk behavior and only adds safety authority from one scalar.
  jerk_u = np.clip(jerk * 2.0, CANFD_JERK_UPPER_MIN, CANFD_JERK_LIMIT_MAX)
  jerk_l_mpc = np.clip(-jerk * 4.0, 1.0, CANFD_JERK_LIMIT_MAX)
  jerk_l_risk = 1.0 + urgency * (CANFD_JERK_LIMIT_MAX - 1.0)

  # Acceleration demand only assists after sustained measured under-deceleration.
  # A positive jerk releases the assist immediately, preventing braking tails.
  assist_ratio = 0.0
  if jerk <= CANFD_JERK_RELEASE_THRESHOLD:
    assist_ratio = np.clip((tracking_error - CANFD_JERK_ERROR_DEADBAND) / CANFD_JERK_ERROR_FULL_SCALE, 0.0, 1.0)

  decel_request = max(0.0, -accel)
  jerk_l_feedforward = np.interp(decel_request, CANFD_JERK_LOWER_ACCEL_BP, CANFD_JERK_LOWER_LIMIT_V)
  jerk_l_assist = 1.0 + assist_ratio * (jerk_l_feedforward - 1.0)
  jerk_l = np.clip(max(jerk_l_mpc, jerk_l_assist, jerk_l_risk), 1.0, CANFD_JERK_LIMIT_MAX)
  return float(jerk_u), float(jerk_l)


__all__ = (
  "CANFD_JERK_ERROR_DEADBAND",
  "CANFD_JERK_ERROR_FULL_SCALE",
  "CANFD_JERK_LIMIT_MAX",
  "CANFD_JERK_RELEASE_THRESHOLD",
  "CANFD_JERK_UPPER_MIN",
  "calculate_canfd_jerk_limits",
)
