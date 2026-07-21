T_FOLLOW_RISE_RATE = 0.30  # seconds of time gap per second
T_FOLLOW_DECEL_RISE_RATE = 0.60
T_FOLLOW_DECEL_EXTRA_THRESHOLD = 0.02


def ramp_t_follow(target: float, current: float, decel_extra: float, dt: float) -> float:
  """Apply increases progressively while keeping gap reductions immediate."""
  if target <= current:
    return float(target)

  rise_rate = T_FOLLOW_DECEL_RISE_RATE if decel_extra > T_FOLLOW_DECEL_EXTRA_THRESHOLD else T_FOLLOW_RISE_RATE
  return float(min(target, current + rise_rate * dt))
