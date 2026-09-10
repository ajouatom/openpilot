"""Retain bounded lead-gap headroom, then recover it with a first-order filter."""

import math

import numpy as np


RECOVERY_TAU = (5.0, 4.0, 3.0, 2.0, 1.0)
MAX_TOTAL_TF = 2.5
CAPTURE_FRACTION = 0.5
MIN_CAPTURE_SPEED = 1.0
STOPPED_SPEED = 0.3
FULL_RECOVERY_SPEED = 5.0
LAUNCH_ACCEL_END = 0.5
ENTRY_TIME = 0.8
MAX_CAPTURE_RISE = 0.5  # seconds of additional TF per second, after acquisition


class LeadGapState:
  def __init__(self):
    self.key = None
    self.extra_tf = 0.0
    self.strength = 0.0
    self.launching = False

  def update(self, *, level, track_id, enabled, dt, ego_speed, lead_speed, lead_accel,
             distance, desired_distance, base_tf):
    values = (dt, ego_speed, lead_speed, lead_accel, distance, desired_distance, base_tf)
    if (not enabled or level not in range(len(RECOVERY_TAU)) or track_id < 0
        or not all(map(math.isfinite, values)) or dt <= 0.0 or ego_speed < 0.0 or distance <= 0.0 or base_tf < 0.0):
      self.__init__()
      return 0.0

    cap = max(0.0, MAX_TOTAL_TF - base_tf)
    candidate = min(cap, CAPTURE_FRACTION * max(0.0, distance - desired_distance) / max(ego_speed, MIN_CAPTURE_SPEED))
    key = (level, track_id)
    acquired = key != self.key
    if acquired:
      self.key, self.extra_tf, self.strength, self.launching = key, candidate, 0.0, False

    # A stopped ego can acquire a lead long before departure. Collect launch
    # headroom immediately, rather than first applying it after acceleration ends.
    self.launching |= ego_speed <= STOPPED_SPEED and lead_speed > STOPPED_SPEED
    if self.launching:
      self.extra_tf = min(max(self.extra_tf, candidate), self.extra_tf + MAX_CAPTURE_RISE * dt)
      self.launching = ego_speed <= STOPPED_SPEED or (lead_accel > LAUNCH_ACCEL_END and lead_speed > ego_speed)
    elif not acquired:
      # Target zero in a FirstOrderFilter: no exp(), no per-frame re-capture.
      self.extra_tf /= 1.0 + dt * recovery_strength(lead_speed) / RECOVERY_TAU[level]

    self.extra_tf = min(cap, self.extra_tf)
    self.strength = min(1.0, self.strength + dt / ENTRY_TIME)
    return self.extra_tf * self.strength

  def margins(self, *, level, times, ego_speeds, lead_speeds, base_tf):
    if self.key is None or level not in range(len(RECOVERY_TAU)):
      return np.zeros_like(times)
    # Predict the same inexpensive first-order recovery over the MPC horizon.
    # At a stopped lead the TF stays fixed, but its distance term vanishes as
    # ego stops. A launch collection remains held within this cycle's horizon.
    strength = 0.0 if self.launching else recovery_strength(lead_speeds)
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
