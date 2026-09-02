from __future__ import annotations

from collections import deque
from statistics import median

import numpy as np


TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG = 50.0
MODEL_LEAD_STOP_OFFSET_M = 2.0
MODEL_LEAD_STOP_CONFIRM_FRAMES = 5
MODEL_LEAD_STOP_PROBABILITY_MIN = 0.90
MODEL_LEAD_STOP_DISTANCE_MIN_M = 4.0
MODEL_LEAD_STOP_DISTANCE_MAX_M = 80.0
MODEL_LEAD_STOP_GAP_MIN_M = 0.0
MODEL_LEAD_STOP_GAP_MAX_M = 3.0
MODEL_LEAD_STOP_SPEED_MAX_MPS = 2.0
MODEL_LEAD_STOP_X_STD_MAX_M = 5.0
MODEL_LEAD_STOP_Y_STD_MAX_M = 0.75
MODEL_LEAD_STOP_V_STD_MAX_MPS = 1.5


class TrafficStopModelLeadMatcher:
  """Recognize when an E2E stop point belongs to a stationary model lead.

  The driving model normally places its ego stop endpoint about 2 m behind a
  queued vehicle.  Once that relationship is stable, expose the vehicle's
  approximate position as the MPC obstacle while keeping the normal configured
  following distance.  This is deliberately only an E2E obstacle correction;
  it never publishes or promotes a radar lead.
  """

  def __init__(self, confirm_frames: int = MODEL_LEAD_STOP_CONFIRM_FRAMES):
    self._confirm_frames = max(1, int(confirm_frames))
    self._lead_distances = deque(maxlen=self._confirm_frames)
    self._lead_velocities = deque(maxlen=self._confirm_frames)
    self._match_count = 0
    self._confirmed = False

  def _clear_pending(self) -> None:
    self._lead_distances.clear()
    self._lead_velocities.clear()
    self._match_count = 0

  def reset(self) -> None:
    self._clear_pending()
    self._confirmed = False

  def update(self, *, stop_active: bool, allow_confirmation: bool, active_lead: bool,
             stop_distance: float, lead_probability: float, lead_distance: float,
             lead_velocity: float, lead_x_std: float, lead_y_std: float,
             lead_v_std: float) -> float:
    if not stop_active or active_lead:
      self.reset()
      return 0.0

    if self._confirmed:
      return MODEL_LEAD_STOP_OFFSET_M

    values = (
      stop_distance, lead_probability, lead_distance, lead_velocity,
      lead_x_std, lead_y_std, lead_v_std,
    )
    if not allow_confirmation or not all(np.isfinite(value) for value in values):
      self._clear_pending()
      return 0.0

    self._lead_distances.append(float(lead_distance))
    self._lead_velocities.append(float(lead_velocity))
    filtered_distance = float(median(self._lead_distances))
    filtered_velocity = float(median(self._lead_velocities))
    endpoint_gap = filtered_distance - float(stop_distance)

    valid = (
      float(lead_probability) >= MODEL_LEAD_STOP_PROBABILITY_MIN
      and MODEL_LEAD_STOP_DISTANCE_MIN_M <= filtered_distance <= MODEL_LEAD_STOP_DISTANCE_MAX_M
      and MODEL_LEAD_STOP_GAP_MIN_M <= endpoint_gap <= MODEL_LEAD_STOP_GAP_MAX_M
      and abs(filtered_velocity) <= MODEL_LEAD_STOP_SPEED_MAX_MPS
      and 0.0 <= float(lead_x_std) <= MODEL_LEAD_STOP_X_STD_MAX_M
      and 0.0 <= float(lead_y_std) <= MODEL_LEAD_STOP_Y_STD_MAX_M
      and 0.0 <= float(lead_v_std) <= MODEL_LEAD_STOP_V_STD_MAX_MPS
    )
    self._match_count = self._match_count + 1 if valid else 0
    if not valid:
      self._lead_distances.clear()
      self._lead_velocities.clear()
    elif self._match_count >= self._confirm_frames:
      self._confirmed = True

    return MODEL_LEAD_STOP_OFFSET_M if self._confirmed else 0.0


def is_traffic_stop_entry_allowed(steering_angle_deg: float) -> bool:
  """Allow steering angle to suppress only entry into a new signal stop."""
  return abs(float(steering_angle_deg)) < TRAFFIC_STOP_ENTRY_STEERING_LIMIT_DEG


def get_traffic_stop_distance_adjust(configured_adjust: float, v_ego: float,
                                     model_lead_offset: float) -> float:
  """Select the obstacle offset without mixing signal and queued-car semantics."""
  model_lead_offset = float(model_lead_offset)
  if np.isfinite(model_lead_offset) and model_lead_offset > 0.0:
    return model_lead_offset
  return float(configured_adjust) if float(v_ego) > 0.1 else -2.0


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
