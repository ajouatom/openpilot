from __future__ import annotations

import numpy as np


# Pull the virtual stop line toward the car at speed so the stationary-obstacle
# constraint starts the stop before the model endpoint becomes urgent. Preserve
# the historical high-speed correction, but fade it out inside 50 m so the final
# stopping position remains model-based.
TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH = (0.0, 100.0)
TRAFFIC_STOP_DISTANCE_RATIO = (1.0, 0.7)
TRAFFIC_STOP_DISTANCE_FADE_BP_M = (0.0, 50.0)
TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0


def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """Allow steering angle to suppress only entry into a new signal stop."""
  return abs(float(steering_angle_deg)) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_reference_speed(v_ego_kph: float, previous_reference_kph: float | None) -> float:
  """Latch the highest speed seen during a signal stop so its distance advance does not relax."""
  return max(0.0, float(v_ego_kph), float(previous_reference_kph or 0.0))


def get_virtual_traffic_stop_distance(model_distance: float, v_ego_kph: float) -> float:
  """Return the model stop distance with a bounded, near-line-fading advance."""
  model_distance = max(0.0, float(model_distance))
  v_ego_kph = max(0.0, float(v_ego_kph))

  distance_ratio = float(np.interp(
    v_ego_kph,
    TRAFFIC_STOP_DISTANCE_RATIO_SPEED_BP_KPH,
    TRAFFIC_STOP_DISTANCE_RATIO,
  ))
  applied_ratio = float(np.interp(
    model_distance,
    TRAFFIC_STOP_DISTANCE_FADE_BP_M,
    (1.0, distance_ratio),
  ))
  return max(0.0, model_distance * applied_ratio)


def get_traffic_stop_obstacle_distance(stop_distance: float, distance_adjust: float) -> float:
  """Apply the configured stop-line correction without placing an obstacle behind the ego."""
  return max(0.0, float(stop_distance) + float(distance_adjust))
