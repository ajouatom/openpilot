T_FOLLOW_RISE_RATE = 0.30  # seconds of time gap per second
T_FOLLOW_DECEL_RISE_RATE = 0.60
T_FOLLOW_DECEL_EXTRA_THRESHOLD = 0.02


def get_t_follow_mode_factor(accel_comfort_factor: float) -> float:
  """Invert a comfort-mode reduction into the intended following-time increase."""
  return float(2.0 - accel_comfort_factor)


def get_speed_t_follow_factor(setting: int, speed_kph: float) -> float:
  """10 means unchanged; 20 means twice the selected TF at 100 km/h."""
  factor_at_100 = min(30, max(10, setting)) * 0.1
  return 1.0 + (factor_at_100 - 1.0) * max(0.0, speed_kph) / 100.0


def get_lead_response_for_gap(common: int, overrides, gap_index: int) -> int:
  gap_index = int(getattr(gap_index, 'raw', gap_index))
  value = overrides[gap_index]
  return int(min(5, max(0, common if value < 0 else value)))


def get_t_follow_mode_max(configured_max: float, mode_factor: float, decel_extra: float) -> float:
  """The caller supplies the speed-scaled maximum, including any held baseline."""
  return float(configured_max * max(1.0, mode_factor) + max(0.0, decel_extra))


def ramp_t_follow(target: float, current: float, decel_extra: float, dt: float) -> float:
  """Apply increases progressively while keeping gap reductions immediate."""
  if target <= current:
    return float(target)

  rise_rate = T_FOLLOW_DECEL_RISE_RATE if decel_extra > T_FOLLOW_DECEL_EXTRA_THRESHOLD else T_FOLLOW_RISE_RATE
  return float(min(target, current + rise_rate * dt))


MODE_T_FOLLOW_RELEASE_RATE = 0.05  # mode multiplier per second (Safe -> Normal in 4 s)


def ramp_mode_t_follow_factor(target: float, current: float, dt: float) -> float:
  """Release only the mode margin slowly; explicit driver gap changes still apply."""
  return float(max(target, current - MODE_T_FOLLOW_RELEASE_RATE * dt))
