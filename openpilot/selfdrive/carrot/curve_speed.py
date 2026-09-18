"""Vision curve geometry and a distance-based approach-speed envelope.

Model velocity is used only to recover curvature from yaw rate. It is never
treated as a desired vehicle speed. All calculations use metres and seconds;
the result is converted to the cruise/display speed convention at the boundary.
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants


NO_LIMIT_KPH = 250.0
MIN_MODEL_SPEED = 3.0
MAX_MODEL_TIME = 6.0
MAX_PREVIEW_DISTANCE = 180.0
TARGET_LAT_ACCEL = 1.9
APPROACH_DECEL = 1.0
APPROACH_JERK = 0.8
RESPONSE_TIME = 1.0
RELEASE_HOLD = 0.35
GEOMETRY_RELEASE_WINDOW = 0.25
GEOMETRY_MIN_SPAN = 0.20
RELEASE_RATE_KPH = 7.2


@dataclass(frozen=True)
class CurveSpeed:
  approach_kph: float = NO_LIMIT_KPH
  curve_kph: float = NO_LIMIT_KPH
  distance: float = 0.0
  direction: float = 1.0


def curve_speed(model, v_ego, sensitivity=1.0, lower_limit_kph=30.0, *, speed_ratio=1.0, a_ego=0.0):
  """Find the tightest distance-adjusted speed constraint on a valid model path.

  A three-node median rejects isolated yaw spikes before taking any minimum.
  The time horizon bounds prediction age; the distance horizon bounds the road
  relevant to the vehicle, even when the model predicts a different speed.
  Invalid/slow nodes break the neighbourhood instead of bridging missing data.
  """
  if not all(math.isfinite(value) for value in (v_ego, sensitivity, lower_limit_kph, speed_ratio, a_ego)):
    return None
  if sensitivity <= 0.0 or v_ego < 0.0:
    return None
  ratio = speed_ratio if 0.5 < speed_ratio <= 1.2 else 1.0
  try:
    values = (model.position.x, model.position.y, model.position.z,
              model.velocity.x, model.orientationRate.z)
    if any(len(value) != ModelConstants.IDX_N for value in values):
      return None
    # Do not let a bad distant tail discard valid near geometry.
    end = int(np.searchsorted(ModelConstants.T_IDXS, MAX_MODEL_TIME, side="right"))
    position = np.asarray(values[:3], dtype=float)[:, :end]
    velocity, yaw_rate = np.asarray(values[3:], dtype=float)[:, :end]
  except (AttributeError, TypeError, ValueError):
    return None
  if not np.all(np.isfinite(position)):
    return None
  distance = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(position, axis=1), axis=0))]
  max_distance = min(MAX_PREVIEW_DISTANCE, max(30.0, v_ego * MAX_MODEL_TIME))
  valid = np.isfinite(velocity) & np.isfinite(yaw_rate) & (velocity >= MIN_MODEL_SPEED)
  curvature = np.divide(yaw_rate, velocity, out=np.zeros_like(yaw_rate), where=valid)
  # Include time to unwind acceleration and build gentle braking, as well as
  # controller/actuator response. The downstream planner still enforces jerk.
  response_time = RESPONSE_TIME + (max(0.0, a_ego) + APPROACH_DECEL) / (2.0 * APPROACH_JERK)
  response_distance = v_ego * response_time
  floor_ms = max(5.0, lower_limit_kph) * ratio / 3.6
  lateral_budget = TARGET_LAT_ACCEL / float(np.clip(sensitivity, 0.5, 3.0))
  best = CurveSpeed()
  found_valid = False
  for i in range(len(distance)):
    start = min(max(i - 1, 0), len(distance) - 3)
    neighbourhood = slice(start, start + 3)
    if distance[i] > max_distance or not np.all(valid[neighbourhood]):
      continue
    # A stationary path with a nonzero velocity prediction is not usable.
    if distance[start + 2] - distance[start] < 0.05:
      continue
    found_valid = True
    curve = float(np.median(curvature[neighbourhood]))
    if abs(curve) < 1e-6:
      continue
    curve_ms = max(floor_ms, math.sqrt(lateral_budget / abs(curve)))
    braking_distance = max(0.0, distance[i] - response_distance)
    approach_ms = math.sqrt(curve_ms**2 + 2.0 * APPROACH_DECEL * braking_distance)
    approach_kph = min(NO_LIMIT_KPH, approach_ms * 3.6 / ratio)
    if approach_kph < best.approach_kph:
      best = CurveSpeed(approach_kph, min(NO_LIMIT_KPH, curve_ms * 3.6 / ratio),
                        float(distance[i]), math.copysign(1.0, curve))
  return best if found_valid else None


class VisionCurveSpeed:
  """Tighten immediately; release against a short history of fresh geometry."""
  def __init__(self):
    self.speed = NO_LIMIT_KPH
    self.direction = 1.0
    self.last_time = None
    self.release_since = None
    self.geometry = deque()
    self.last_model_time = None

  def update(self, result, now, *, model_time=None):
    if not math.isfinite(now):
      return self.speed * self.direction
    dt = 0.0 if self.last_time is None else max(0.0, min(now - self.last_time, 0.2))
    self.last_time = now
    if result is not None and not math.isfinite(result.approach_kph):
      result = None
    if result is not None:
      # Re-reading one model frame is not independent confirmation of an exit.
      stamp = now if model_time is None else model_time
      if self.last_model_time is not None and stamp <= self.last_model_time:
        # Updated vehicle speed/acceleration can tighten the same model path.
        if result.approach_kph < self.speed:
          self.speed = result.approach_kph
          self.direction = result.direction
          self.geometry.clear()
        return self.speed * self.direction
      self.last_model_time = stamp
      if self.geometry and (now < self.geometry[-1][0] or now - self.geometry[-1][0] > GEOMETRY_MIN_SPAN):
        self.geometry.clear()
      self.geometry.append((now, result.approach_kph))
      while self.geometry and now - self.geometry[0][0] > GEOMETRY_RELEASE_WINDOW + 1e-9:
        self.geometry.popleft()
      target = result.approach_kph
      confirmed = len(self.geometry) >= 3 and now - self.geometry[0][0] >= GEOMETRY_MIN_SPAN - 1e-9
      if target <= self.speed:
        self.speed = target
        self.direction = result.direction
      elif confirmed:
        # Release the ceiling, not a commanded acceleration. The longitudinal
        # planner continues to enforce acceleration/jerk and other speed limits.
        self.speed = min(value for _, value in self.geometry)
      self.release_since = None
    else:
      # Missing/invalid geometry is not evidence that the road has straightened.
      self.geometry.clear()
      if self.release_since is None:
        self.release_since = now
      if now - self.release_since >= RELEASE_HOLD:
        self.speed = min(NO_LIMIT_KPH, self.speed + RELEASE_RATE_KPH * dt)
    return self.speed * self.direction
