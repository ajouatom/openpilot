"""Retain bounded lead-gap headroom, then recover it with a first-order filter."""

import math

import numpy as np


RECOVERY_TAU = (5.0, 4.0, 3.0, 2.0, 1.0)
MAX_TOTAL_TF = 2.5
CAPTURE_FRACTION = 0.5
MIN_CAPTURE_SPEED = 1.0
STOPPED_SPEED = 0.3
FULL_RECOVERY_SPEED = 5.0
OPENING_SPEED = 0.2
OPENING_FILTER_TAU = 0.3
ENTRY_TIME = 0.8
MAX_CAPTURE_RISE = 0.5  # seconds of additional TF per second, after acquisition


class LeadGapState:
  def __init__(self):
    self.key = None
    self.extra_tf = 0.0
    self.strength = 0.0
    self.filtered_relative_speed = 0.0

  def update(self, *, level, track_id, enabled, dt, ego_speed, lead_speed, relative_speed,
             distance, desired_distance, base_tf):
    values = (dt, ego_speed, lead_speed, relative_speed, distance, desired_distance, base_tf)
    if (not enabled or level not in range(len(RECOVERY_TAU)) or track_id < 0
        or not all(map(math.isfinite, values)) or dt <= 0.0 or ego_speed < 0.0 or distance <= 0.0 or base_tf < 0.0):
      self.__init__()
      return 0.0

    cap = max(0.0, MAX_TOTAL_TF - base_tf)
    candidate = min(cap, CAPTURE_FRACTION * max(0.0, distance - desired_distance) / max(ego_speed, MIN_CAPTURE_SPEED))
    key = (level, track_id)
    acquired = key != self.key
    if acquired:
      self.key, self.extra_tf, self.strength = key, candidate, 0.0
      self.filtered_relative_speed = relative_speed

    # Use measured relative speed, not lead speed minus the MPC's planned ego
    # speed. Opening can continue through a lead acceleration lull or restart.
    self.filtered_relative_speed += dt / (OPENING_FILTER_TAU + dt) * (relative_speed - self.filtered_relative_speed)
    if not acquired:
      previous = self.extra_tf
      self.extra_tf /= 1.0 + dt * recovery_strength(lead_speed) / RECOVERY_TAU[level]
      if self.filtered_relative_speed > OPENING_SPEED:
        # Keep recovering even while opening. A large constant gap alone must
        # not replenish the allowance, and new headroom enters at a bounded rate.
        self.extra_tf = max(self.extra_tf, min(candidate, previous + MAX_CAPTURE_RISE * dt))

    self.extra_tf = min(cap, self.extra_tf)
    self.strength = min(1.0, self.strength + dt / ENTRY_TIME)
    return self.extra_tf * self.strength

  def margins(self, *, level, times, ego_speeds, lead_speeds, base_tf):
    if self.key is None or level not in range(len(RECOVERY_TAU)):
      return np.zeros_like(times)
    # Predict the same inexpensive first-order recovery over the MPC horizon.
    # At a stopped lead the TF stays fixed, but its distance term vanishes as
    # ego stops. Do not assume future opening will replenish the allowance.
    strength = recovery_strength(lead_speeds)
    remaining = np.cumprod(1.0 / (1.0 + np.diff(times, prepend=0.0) * strength / RECOVERY_TAU[level]))
    extra = min(self.extra_tf, max(0.0, MAX_TOTAL_TF - base_tf)) * self.strength
    return np.maximum(ego_speeds, 0.0) * extra * remaining


def recovery_strength(lead_speed):
  return np.clip((lead_speed - STOPPED_SPEED) / (FULL_RECOVERY_SPEED - STOPPED_SPEED), 0.0, 1.0)


def gap_reference(obstacles, margins, ego_speed):
  """Change the comfort reference only; preserve physical obstacles and limits."""
  preferred = obstacles.copy()
  preferred[:, :2] -= margins
  shift = np.maximum(0.0, np.min(obstacles, axis=1) - np.min(preferred, axis=1))
  return shift / (np.maximum(ego_speed, 0.0) + 10.0)
