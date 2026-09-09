import math

import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants


TURN_CURVATURE_LOOKAHEAD = 1.0
TURN_CURVATURE_MIN_SPEED = 3.0
TURN_ACCEL_LOOKAHEAD = 3.0


def get_future_curvature(model_msg, fallback_curvature, lookahead=TURN_CURVATURE_LOOKAHEAD):
  if (len(model_msg.orientationRate.z) != ModelConstants.IDX_N or
      len(model_msg.velocity.x) != ModelConstants.IDX_N):
    return fallback_curvature

  yaw_rate = float(np.interp(lookahead, ModelConstants.T_IDXS, model_msg.orientationRate.z))
  velocity = float(np.interp(lookahead, ModelConstants.T_IDXS, model_msg.velocity.x))
  if not (np.isfinite(yaw_rate) and np.isfinite(velocity)):
    return fallback_curvature
  return yaw_rate / max(abs(velocity), TURN_CURVATURE_MIN_SPEED)


def _preview_curvatures(model_msg, v_ego, accel_max):
  """Return road distances and curvature over the reliable near model horizon."""
  values = [model_msg.position.x, model_msg.position.y, model_msg.position.z,
            model_msg.velocity.x, model_msg.orientationRate.z]
  if any(len(value) != ModelConstants.IDX_N for value in values):
    return None
  values = np.asarray(values, dtype=float)
  # Invalid far-tail predictions must not discard otherwise valid near geometry.
  last = int(np.searchsorted(ModelConstants.T_IDXS, TURN_ACCEL_LOOKAHEAD))
  if not np.all(np.isfinite(values[:, :last + 1])):
    return None
  times = np.append(np.asarray(ModelConstants.T_IDXS[:last]), TURN_ACCEL_LOOKAHEAD)
  x, y, z, velocity, yaw_rate = [np.interp(times, ModelConstants.T_IDXS, value) for value in values]
  distance = np.r_[0., np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))]
  # Use distance, not model time: the model may travel more slowly than ACC.
  reachable = v_ego * TURN_ACCEL_LOOKAHEAD + 0.5 * accel_max * TURN_ACCEL_LOOKAHEAD**2
  valid = (distance <= reachable) & (velocity >= TURN_CURVATURE_MIN_SPEED)
  # Include the spatial boundary between model nodes as the car accelerates.
  if 0.0 < reachable < distance[-1]:
    boundary_velocity = float(np.interp(reachable, distance, velocity))
    if boundary_velocity >= TURN_CURVATURE_MIN_SPEED:
      boundary_curve = abs(float(np.interp(reachable, distance, yaw_rate))) / boundary_velocity
      return np.r_[distance[valid], reachable], np.r_[np.abs(yaw_rate[valid] / velocity[valid]), boundary_curve]
  if not np.any(valid):
    return None
  return distance[valid], np.abs(yaw_rate[valid] / velocity[valid])


def limit_accel_in_turns(v_ego, curvature, a_target, a_lat_max, safety_ratio=0.70, min_v=0.1,
                         *, model_msg=None, v_cruise=None, current_curvature=0.0):
  """Cap positive acceleration using current and upcoming combined acceleration.

  At each upcoming road distance s, a candidate constant acceleration a gives
  v(s)^2 = v_ego^2 + 2*a*s, capped at cruise speed (never below current speed).
  Require a^2 + (v(s)^2 * curvature(s))^2 <= (a_lat_max*safety_ratio)^2.
  Solving for a avoids the launch deadlock caused by assuming full acceleration
  first and then forbidding all acceleration because of its predicted speed.
  Also taper the normal acceleration allowance by the fraction of the squared
  lateral budget remaining. This reserves comfort margin before the circular
  combined-acceleration boundary becomes binding.
  Braking and jerk/actuator response remain the longitudinal planner's job.
  """
  if v_ego < min_v or a_lat_max <= 0.0 or a_target[1] <= 0.0:
    return list(a_target)

  total_accel = abs(a_lat_max) * safety_ratio
  curvatures = [abs(c) for c in (curvature, current_curvature) if math.isfinite(c)]
  lateral_accel = v_ego**2 * max(curvatures, default=0.0)
  maximum = min(a_target[1], math.sqrt(max(0.0, total_accel**2 - lateral_accel**2)))
  if model_msg is not None and total_accel > 0.0:
    maximum = min(maximum, a_target[1] * max(0.0, 1.0 - (lateral_accel / total_accel)**2))
  preview = _preview_curvatures(model_msg, v_ego, a_target[1]) if model_msg is not None else None
  if preview is None or maximum <= 0.0:
    return [a_target[0], maximum]

  distance, curve = preview
  cruise_squared = max(v_ego, v_cruise)**2 if v_cruise is not None and math.isfinite(v_cruise) else math.inf

  def feasible(accel):
    speed_squared = np.minimum(v_ego**2 + 2.0 * accel * distance, cruise_squared)
    lateral_squared = (speed_squared * curve)**2
    return bool(np.all((accel**2 + lateral_squared <= total_accel**2) &
                       (accel <= a_target[1] * (1.0 - lateral_squared / total_accel**2))))

  if not feasible(0.0):
    return [a_target[0], 0.0]
  if feasible(maximum):
    return [a_target[0], maximum]
  lower, upper = 0.0, maximum
  for _ in range(12):
    middle = (lower + upper) * 0.5
    if feasible(middle):
      lower = middle
    else:
      upper = middle
  return [a_target[0], lower]
