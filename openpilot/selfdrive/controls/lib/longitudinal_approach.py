"""Bounded closing-distance preferences; never modify physical MPC obstacles."""

import math

import numpy as np


# (closing-time allowance in seconds, maximum additional distance in metres).
# Disabled and maximum-response modes intentionally have no approach preference.
APPROACH_TUNING = {1: (3.0, 12.0), 2: (2.0, 8.0), 3: (1.0, 4.0), 4: (0.3, 1.0)}
CLOSING_DEADBAND = 0.2
ENTRY_TIME = 0.8
BRAKE_FADE_START = 0.8
BRAKE_FADE_END = 2.0


def approach_margin(level, ego_speed, lead_speed, distance, stop_distance):
  """Return a fresh, non-accumulating distance preference at each horizon node.

  The energy estimate is a comfort gate for a constant-speed lead, not a safety
  bound. Lead braking predictions and all original MPC constraints remain active.
  Fade the extra preference as required braking grows, so a newly acquired close
  stationary lead does not request a large additional gap on top of braking.
  """
  ego, lead, distance = np.broadcast_arrays(ego_speed, lead_speed, distance)
  zero = np.zeros_like(ego, dtype=float)
  if level not in APPROACH_TUNING or not math.isfinite(stop_distance) or stop_distance < 0.0:
    return zero
  finite = np.isfinite(ego) & np.isfinite(lead) & np.isfinite(distance) & (ego >= 0.0) & (lead >= 0.0)
  ego = np.where(finite, ego, 0.0)
  lead = np.where(finite, lead, 0.0)
  available = np.maximum(np.where(finite, distance, 0.0) - stop_distance, 0.0)
  horizon, cap = APPROACH_TUNING[level]
  closing = np.maximum(ego - lead - CLOSING_DEADBAND, 0.0)
  required_brake = np.maximum(ego**2 - lead**2, 0.0) / (2.0 * np.maximum(available, 0.1))
  # Urgency now suppresses the whole preference, even if the previous solution
  # predicts sufficient braking later. Never ask to recover a comfort gap late.
  if required_brake.size:
    required_brake = np.maximum(required_brake, required_brake.flat[0])
  urgency_scale = np.clip((BRAKE_FADE_END - required_brake) / (BRAKE_FADE_END - BRAKE_FADE_START), 0.0, 1.0)
  margin = np.minimum(np.minimum(horizon * closing, cap), 0.25 * available) * urgency_scale
  return np.where(finite, margin, zero)


class LeadApproachState:
  """Ramp acquisition independently for each selected lead; discard stale tracks."""

  def __init__(self):
    self.key = None
    self.strength = 0.0

  def update(self, level, track_id, enabled, dt):
    if not enabled or level not in APPROACH_TUNING or track_id < 0 or not math.isfinite(dt) or dt <= 0.0:
      self.key, self.strength = None, 0.0
      return 0.0
    key = (level, track_id)
    if key != self.key:
      self.strength = 0.0
    self.key = key
    self.strength = min(1.0, self.strength + dt / ENTRY_TIME)
    return self.strength


def approach_reference(obstacles, margins, ego_speed):
  """Shift only the existing soft distance reference, accounting for all sources.

  A positive yref asks for more clearance. The unmodified obstacle minimum still
  feeds the solver's danger constraint and source selection. Cruise/map targets
  receive no margin. This reuses the existing solver ABI and distance cost.
  """
  preferred = obstacles.copy()
  preferred[:, :2] -= margins
  shift = np.maximum(0.0, np.min(obstacles, axis=1) - np.min(preferred, axis=1))
  return shift / (np.maximum(ego_speed, 0.0) + 10.0)
