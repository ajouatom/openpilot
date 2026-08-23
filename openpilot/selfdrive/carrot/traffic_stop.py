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
TRAFFIC_STOP_SOFT_DECEL_MPS2 = 2.2
TRAFFIC_STOP_MAX_DECEL_MPS2 = 4.0
TRAFFIC_STOP_RESPONSE_TIME_S = 0.5
TRAFFIC_STOP_DISTANCE_UNCERTAINTY_M = 5.0
TRAFFIC_STOP_DECEL_SAFETY_BUFFER_MPS2 = 0.2


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


def get_traffic_stop_accel_floor(v_ego: float, raw_stop_distance: float, stop_distance: float) -> float:
  """Return a comfortable signal-stop accel floor that releases as stopping margin shrinks."""
  values = (v_ego, raw_stop_distance, stop_distance)
  if not all(np.isfinite(value) for value in values):
    return -TRAFFIC_STOP_MAX_DECEL_MPS2

  v_ego = max(0.0, float(v_ego))
  available_distance = (
    float(raw_stop_distance)
    - max(0.0, float(stop_distance))
    - v_ego * TRAFFIC_STOP_RESPONSE_TIME_S
    - TRAFFIC_STOP_DISTANCE_UNCERTAINTY_M
  )
  if available_distance <= 0.0:
    return -TRAFFIC_STOP_MAX_DECEL_MPS2

  required_decel = v_ego ** 2 / (2.0 * available_distance)
  allowed_decel = np.clip(
    max(TRAFFIC_STOP_SOFT_DECEL_MPS2, required_decel + TRAFFIC_STOP_DECEL_SAFETY_BUFFER_MPS2),
    TRAFFIC_STOP_SOFT_DECEL_MPS2,
    TRAFFIC_STOP_MAX_DECEL_MPS2,
  )
  return -float(allowed_decel)
