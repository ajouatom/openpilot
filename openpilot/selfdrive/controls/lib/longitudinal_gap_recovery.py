"""Hold opening-gap headroom, then recover it with continuous two-stage dynamics."""

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
      # An opening gap is deliberate comfort headroom, including when lead
      # acceleration has already eased. Follow its envelope rather than only
      # accepting candidate increments while simultaneously draining it.
      target = max(candidate, self.recovery_tf, self.extra_tf) if self.filtered_relative_speed > OPENING_SPEED else None
      self.recovery_tf, self.extra_tf = advance_headroom(
        self.recovery_tf, self.extra_tf, target, dt, lead_speed, RECOVERY_TAU[level], cap)

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
    relative_speed_offset = self.relative_speed - (lead_speeds[0] - ego_speeds[0])
    relative_speeds = lead_speeds - ego_speeds + relative_speed_offset
    if lead_distances is None:
      lead_distances = self.distance + np.cumsum(np.diff(times, prepend=0.0) * relative_speeds)
    else:
      lead_distances = self.distance + lead_distances - lead_distances[0] + times * relative_speed_offset
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


def advance_headroom(reservoir, extra, target, dt, lead_speed, tau, cap):
  """Charge/hold an absolute envelope, or release through two filter stages.

  The reservoir rises at a bounded rate while opening, never accumulating the
  same candidate repeatedly. The output always follows extra'=k*(reservoir-extra),
  including across charge/hold/release transitions. On release the reservoir
  decays too: equal initial states give extra=H*(1+k*t)*exp(-k*t).
  """
  reservoir, extra = min(cap, reservoir), min(cap, extra)
  k = 2.0 * float(recovery_strength(lead_speed)) / tau
  if target is not None:
    charge_time = min(dt, max(0.0, min(cap, target) - reservoir) / MAX_CAPTURE_RISE)
    if k > 0.0:
      following = -math.expm1(-k * charge_time)
      extra += (reservoir - extra) * following + MAX_CAPTURE_RISE * (charge_time - following / k)
    reservoir += MAX_CAPTURE_RISE * charge_time
    if k > 0.0:
      extra += (reservoir - extra) * -math.expm1(-k * (dt - charge_time))
    return reservoir, min(cap, max(0.0, extra))
  if k <= 0.0:
    return reservoir, extra
  z = k * dt
  decay = math.exp(-z)
  return reservoir * decay, (extra + z * reservoir) * decay


def recovery_strength(lead_speed):
  return min(1.0, max(0.0, (lead_speed - STOPPED_SPEED) / (FULL_RECOVERY_SPEED - STOPPED_SPEED)))


def gap_reference(obstacles, margins, ego_speed):
  """Change the comfort reference only; preserve physical obstacles and limits."""
  preferred = obstacles.copy()
  preferred[:, :2] -= margins
  shift = np.maximum(0.0, np.min(obstacles, axis=1) - np.min(preferred, axis=1))
  return shift / (np.maximum(ego_speed, 0.0) + 10.0)
