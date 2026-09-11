"""Recover newly opened lead-gap headroom with continuous two-stage dynamics."""

from copy import copy
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
    self.recovery_tf = 0.0
    self.strength = 0.0
    self.filtered_relative_speed = 0.0
    self.candidate = 0.0
    self.distance = 0.0
    self.desired_distance = 0.0
    self.relative_speed = 0.0

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
      self.recovery_tf = candidate
      self.filtered_relative_speed = relative_speed

    # Use measured relative speed, not lead speed minus the MPC's planned ego
    # speed. Opening can continue through a lead acceleration lull or restart.
    self.filtered_relative_speed += -math.expm1(-dt / OPENING_FILTER_TAU) * (relative_speed - self.filtered_relative_speed)
    if not acquired:
      captured = 0.0
      if self.filtered_relative_speed > OPENING_SPEED:
        # Spend only newly observed headroom, never the same candidate again.
        # A changing braking-distance estimate or ego-speed denominator alone
        # is not an opening gap. Bound measurement jumps by relative motion.
        opened = min(max(0.0, distance - self.distance), max(0.0, relative_speed) * dt)
        captured = min(max(0.0, candidate - self.candidate),
                       CAPTURE_FRACTION * opened / max(ego_speed, MIN_CAPTURE_SPEED), MAX_CAPTURE_RISE * dt)
      self.recovery_tf, self.extra_tf = recover_headroom(
        self.recovery_tf, self.extra_tf, captured, dt, lead_speed, RECOVERY_TAU[level], cap)

    self.extra_tf = min(cap, self.extra_tf)
    self.recovery_tf = min(cap, self.recovery_tf)
    self.strength = min(1.0, self.strength + dt / ENTRY_TIME)
    self.candidate, self.distance = candidate, distance
    self.desired_distance, self.relative_speed = desired_distance, relative_speed
    return self.extra_tf * entry_weight(self.strength)

  def margins(self, *, level, times, ego_speeds, lead_speeds, base_tf,
              lead_distances=None, desired_distances=None):
    if self.key is None or level not in range(len(RECOVERY_TAU)):
      return np.zeros_like(times)
    # Roll a copy through the SAME capture, entry and recovery rules. Anchor
    # predicted changes to measurements, rather than replacing measured vRel
    # with the MPC's planned ego speed. Forecasting cannot mutate live state.
    relative_speeds = self.relative_speed + (lead_speeds - ego_speeds) - (lead_speeds[0] - ego_speeds[0])
    if lead_distances is None:
      lead_distances = self.distance + np.cumsum(np.diff(times, prepend=0.0) * relative_speeds)
    else:
      lead_distances = self.distance + lead_distances - lead_distances[0]
    if desired_distances is None:
      desired_distances = np.full_like(times, self.desired_distance)
    else:
      desired_distances = self.desired_distance + desired_distances - desired_distances[0]
    predicted = copy(self)
    margins = np.zeros_like(times)
    previous_time = 0.0
    for i, t in enumerate(times):
      dt = float(t - previous_time)
      if dt > 0.0:
        predicted.update(level=level, track_id=self.key[1], enabled=True, dt=dt,
                         ego_speed=float(ego_speeds[i]), lead_speed=float(lead_speeds[i]),
                         relative_speed=float(relative_speeds[i]), distance=max(1e-3, float(lead_distances[i])),
                         desired_distance=float(desired_distances[i]), base_tf=base_tf)
      margins[i] = max(0.0, ego_speeds[i]) * min(predicted.extra_tf, max(0.0, MAX_TOTAL_TF - base_tf)) * entry_weight(predicted.strength)
      previous_time = t
    return margins


def entry_weight(strength):
  """Zero entry slope at both ends of the existing acquisition interval."""
  return strength * strength * (3.0 - 2.0 * strength)


def recover_headroom(reservoir, extra, captured, dt, lead_speed, tau, cap):
  """Exact two-stage decay with constant capture rate over this interval.

  With no capture and initially equal states, extra = H*(1+k*t)*exp(-k*t).
  It starts with zero recovery slope. Both stages share the same speed clock,
  retaining headroom at a stopped lead. Two stages use k=2/tau so their mean
  recovery time remains tau instead of doubling the previous time scale.
  """
  k = 2.0 * float(recovery_strength(lead_speed)) / tau
  if k <= 0.0:
    return min(cap, reservoir + captured), extra
  z = k * dt
  decay = math.exp(-z)
  # Stable integration of the capture input, including near-zero lead speed.
  first = -math.expm1(-z) / z
  second = (1.0 - (1.0 + z) * decay) / z if z > 1e-4 else z / 2.0 - z*z / 3.0 + z*z*z / 8.0
  return (min(cap, max(0.0, reservoir * decay + captured * first)),
          min(cap, max(0.0, (extra + z * reservoir) * decay + captured * second)))


def recovery_strength(lead_speed):
  return min(1.0, max(0.0, (lead_speed - STOPPED_SPEED) / (FULL_RECOVERY_SPEED - STOPPED_SPEED)))


def gap_reference(obstacles, margins, ego_speed):
  """Change the comfort reference only; preserve physical obstacles and limits."""
  preferred = obstacles.copy()
  preferred[:, :2] -= margins
  shift = np.maximum(0.0, np.min(obstacles, axis=1) - np.min(preferred, axis=1))
  return shift / (np.maximum(ego_speed, 0.0) + 10.0)
