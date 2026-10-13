"""Bounded comfort slack; no braking-onset distance is retained."""

import math

import numpy as np


def comfort_path_has_reserve(ego_speeds, physical_gaps, danger_margins, times, stop_distance):
  near = times <= 3.0
  return bool(np.all(np.isfinite(danger_margins)) and np.all(np.isfinite(physical_gaps))
              and np.all(np.isfinite(ego_speeds))
              and np.min(danger_margins[near]) > 3.0
              and np.min(physical_gaps[near] - stop_distance - 0.7 * ego_speeds[near]) > 2.0)


class LeadComfortState:
  def __init__(self):
    self.key = None
    self.age = 0.0
    self.distance = 0.0
    self.relative_speed = 0.0
    self.relief = 0.0

  def update(self, *, enabled, track_id, dt, distance, relative_speed):
    values = (dt, distance, relative_speed)
    if (not enabled or track_id < 0 or not all(map(math.isfinite, values))
        or not 0.0 < dt <= 0.2 or distance <= 0.0):
      self.__init__()
      return
    if (self.key != track_id
        or abs(distance - self.distance - self.relative_speed * dt) > 3.0):
      self.__init__()
      self.key = track_id
    self.age += dt
    self.distance, self.relative_speed = distance, relative_speed

  def comfort_relief(self, *, dt, enabled, base_tf, ego_speeds, lead_speeds, lead_accel,
                     physical_gaps, danger_margins, times, stop_distance):
    """Spend at most 0.4 s of comfort TF, never modify the danger constraint.

    0.7 s is a lower comfort-reference bound, NOT a safety guarantee. Closing
    speed, observed clearance and the unchanged lead forecast must also leave
    reserve. New leads, stops and already constrained approaches get no relief.
    """
    v = float(ego_speeds[0])
    eligible = (enabled and self.age >= 1.0
                and base_tf > 0.7 and v > 5.0 and lead_speeds[0] > 2.0
                and lead_accel < -0.3 and -2.5 < self.relative_speed < 0.0
                and self.distance - stop_distance > v * (base_tf + 0.2)
                and comfort_path_has_reserve(ego_speeds, physical_gaps, danger_margins, times, stop_distance))
    # Hazard withdrawal is immediate. This is a preference, never a brake cap.
    self.relief = min(1.0, self.relief + dt / 0.5) if eligible else 0.0
    if not eligible:
      return np.zeros_like(ego_speeds)
    return self.relief * min(0.4, max(0.0, base_tf - 0.7)) * np.maximum(ego_speeds, 0.0)


def braking_gap_reference(obstacles, margins, ego_speeds):
  preferred = obstacles.copy()
  preferred[:, :2] -= margins
  # Signed shift allows comfort slack while retaining every other obstacle.
  return (np.min(obstacles, axis=1) - np.min(preferred, axis=1)) / (np.maximum(ego_speeds, 0.0) + 10.0)
