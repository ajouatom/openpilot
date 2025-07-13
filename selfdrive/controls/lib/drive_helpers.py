import numpy as np
from cereal import log
from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
import numpy as np

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2
MAX_CURVATURE_DELTA_FRAME = 0.03 #0.019 # about 3 degree / DT_CTRL 

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error

def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures, steer_actuator_delay, distances):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
    distances = [0.0] * CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  delay = max(0.01, steer_actuator_delay)

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  delayed_curvature_desired = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], curvatures)
  future_curvature_desired = np.interp(1.2, ModelConstants.T_IDXS[:CONTROL_N], curvatures)

  psi = np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)

  distance = max(np.interp(delay, ModelConstants.T_IDXS[:CONTROL_N], distances), 0.001)
  #average_curvature_desired = psi / (v_ego * delay)

  # curve -> straight or reverse curve
  if v_ego > 5 and abs(current_curvature_desired) > 0.002 and \
     (abs(future_curvature_desired) < 0.001 or np.sign(current_curvature_desired) != np.sign(future_curvature_desired)):
    psis_damping = 0.2
  else:
    psis_damping = 1.0
  #psi *= psis_damping


  average_curvature_desired = psi / distance
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = np.clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature

def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll):
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  max_lat_accel = MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  min_lat_accel = -MAX_LATERAL_ACCEL_NO_ROLL + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  
  new_curvature = np.clip(
    new_curvature,
    prev_curvature - MAX_CURVATURE_DELTA_FRAME,
    prev_curvature + MAX_CURVATURE_DELTA_FRAME
  )
  
  was_limited = limited_accel or limited_max_curv or (abs(new_curvature - prev_curvature) >= MAX_CURVATURE_DELTA_FRAME)

  return float(new_curvature), was_limited

def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = np.clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
    v_max = np.max(speeds)
  else:
    v_now = 0.0
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
    v_max = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop, v_target, v_max #v_target #v_now

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
