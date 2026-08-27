from __future__ import annotations

import numpy as np


TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0


def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """Allow steering angle to suppress only entry into a new signal stop."""
  return abs(float(steering_angle_deg)) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_obstacle_distance(stop_distance: float, cruise_obstacle_distance: float,
                                       distance_adjust: float, release_distance: float = 50.0) -> float:
  """Smoothly release the historical cruise-distance mask before a signal stop."""
  signal_obstacle = max(0.0, float(stop_distance) + float(distance_adjust))
  cruise_obstacle = max(0.0, float(cruise_obstacle_distance))
  release_distance = max(0.0, float(release_distance))

  # Historically, a signal obstacle between 50 m and the cruise safe distance
  # was replaced by the cruise obstacle, then exposed all at once at 50 m. Keep
  # that protection at first contact, but progressively expose the real signal
  # obstacle so braking can build before the 50 m boundary.
  if release_distance < signal_obstacle < cruise_obstacle:
    release = float(np.interp(signal_obstacle, [release_distance, cruise_obstacle], [1.0, 0.0]))
    return cruise_obstacle + release * (signal_obstacle - cruise_obstacle)
  return signal_obstacle
