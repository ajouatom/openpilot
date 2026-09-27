"""Stored and runtime lower bound for the planner's stop-speed threshold."""
import math


STOPPING_SPEED_MIN = 10  # hundredths of m/s
STOPPING_SPEED_DEFAULT = 50


def get_stopping_speed(params, *, blocking=False):
  try:
    stored = params.get_float("VEgoStopping")
  except (TypeError, ValueError, OverflowError):
    stored = float("nan")
  value = max(STOPPING_SPEED_MIN, stored) if math.isfinite(stored) else STOPPING_SPEED_DEFAULT
  if value != stored:
    # Startup repairs persistent values before settings are shown. During driving,
    # enforce the bound immediately and repair external writes without blocking.
    write = params.put_int if blocking else params.put_int_nonblocking
    write("VEgoStopping", int(value))
  return value * 0.01
