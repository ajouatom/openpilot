T_FOLLOW_RISE_RATE = 0.30  # seconds of time gap per second
T_FOLLOW_DECEL_RISE_RATE = 0.60
T_FOLLOW_DECEL_EXTRA_THRESHOLD = 0.02


def get_t_follow_mode_factor(accel_comfort_factor: float) -> float:
  """Invert a comfort-mode reduction into the intended following-time increase."""
  return float(2.0 - accel_comfort_factor)


def get_t_follow_mode_max(configured_max: float, mode_factor: float, decel_extra: float) -> float:
  """Let a comfort mode increase the configured gap while preserving the global cap."""
  return float(min(2.0, configured_max * max(1.0, mode_factor) + max(0.0, decel_extra)))


def ramp_t_follow(target: float, current: float, decel_extra: float, dt: float) -> float:
  """Apply increases progressively while keeping gap reductions immediate."""
  if target <= current:
    return float(target)

  rise_rate = T_FOLLOW_DECEL_RISE_RATE if decel_extra > T_FOLLOW_DECEL_EXTRA_THRESHOLD else T_FOLLOW_RISE_RATE
  return float(min(target, current + rise_rate * dt))
