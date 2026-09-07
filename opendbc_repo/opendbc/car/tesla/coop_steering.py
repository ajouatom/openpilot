import math
import numpy as np
from collections import namedtuple
from dataclasses import replace

from opendbc.car import structs, rate_limit, DT_CTRL, apply_steer_angle_limits_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.tesla.values import CarControllerParams


DT_LAT_CTRL = DT_CTRL * CarControllerParams.STEER_STEP


class CoopSteeringCarControllerParams(CarControllerParams):
  ANGLE_LIMITS = replace(CarControllerParams.ANGLE_LIMITS, MAX_ANGLE_RATE=5)


STEERING_DEG_PHASE_LEAD_COEFF = 8.0

# angle override
STEER_OVERRIDE_MIN_TORQUE = 0.5 # Nm - based on typical steering bias + noise - used for the deadzone
STEER_OVERRIDE_MAX_TORQUE = 2.5 # Nm - typical torque before EPS disengages due to hands_on_level=3
STEER_OVERRIDE_TORQUE_RANGE = STEER_OVERRIDE_MAX_TORQUE - STEER_OVERRIDE_MIN_TORQUE
STEER_OVERRIDE_STANDSTILL_VEGO = 0.1 # m/s - below this speed holding-torque estimate collapses
STEER_OVERRIDE_MAX_LAT_ACCEL = 2.0 # m/s^2 - determines angle rate - speed dependent - similar to Tesla comfort steering mode
STEER_OVERRIDE_TARGET_ANGLE_MAX = CarControllerParams.ANGLE_LIMITS.STEER_ANGLE_MAX  # deg

# override angle ramp control
STEER_OVERRIDE_DELTA_GAIN_LIMIT = 125 # deg/s/Nm
STEER_OVERRIDE_DELTA_GAIN_LIMIT_CENTERING = CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / DT_LAT_CTRL / STEER_OVERRIDE_TORQUE_RANGE

# limit steering acceleration when engaging
STEER_RESUME_RATE_LIMIT_RAMP_RATE = 300 # deg/s^2

CoopSteeringData = namedtuple("CoopSteeringData",
                              ["steeringAngleDeg", "lat_active", "control_type"])


def get_steer_from_lat_accel(lat_accel, v_ego: float, VM: VehicleModel):
  curvature = lat_accel / (max(1, v_ego) ** 2)
  return math.degrees(VM.get_steer_from_curvature(curvature, v_ego, 0))


def apply_bounds(signal: float, limit: float) -> float:
  return float(np.clip(signal, -limit, limit))


def apply_deadzone(signal: float, deadzone: float) -> float:
  return signal - apply_bounds(signal, deadzone)


def get_override_torque_to_angle(vEgo: float, VM: VehicleModel, lat_accel: float) -> float:
  steer_from_lat_accel = apply_bounds(get_steer_from_lat_accel(lat_accel, vEgo, VM), STEER_OVERRIDE_TARGET_ANGLE_MAX)
  return steer_from_lat_accel / STEER_OVERRIDE_TORQUE_RANGE


def calc_override_angle_delta_limit(torque: float, gain_limit: float) -> float:
  delta_gain_limit_max = CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / DT_LAT_CTRL / STEER_OVERRIDE_TORQUE_RANGE
  return torque * min(gain_limit, delta_gain_limit_max) * DT_LAT_CTRL


class SteerRateLimiter:
  def __init__(self):
    self._last = 0.0

  def reset(self, angle: float) -> None:
    self._last = angle

  def update(self, angle: float, angle_delta_lim: float) -> float:
    angle_lim = rate_limit(angle, self._last, -angle_delta_lim, angle_delta_lim)
    self._last = angle_lim
    return angle_lim


class CoopSteeringCarController:
  def __init__(self):
    self.apply_angle_last = 0
    self.coop_apply_angle_sat_last = 0
    self.angle_override = 0
    self.resume_rate_limiter_delta = SteerRateLimiter()
    self.resume_rate_limiter = SteerRateLimiter()

  def reset_override_state(self, apply_angle: float) -> None:
    self.apply_angle_last = apply_angle
    self.angle_override = 0.0
    self.coop_apply_angle_sat_last = apply_angle

  def compute_override_targets(self, vEgo: float, steering_torque: float, VM: VehicleModel) -> tuple[float, float]:
    torque_to_angle = get_override_torque_to_angle(vEgo, VM, STEER_OVERRIDE_MAX_LAT_ACCEL)
    driver_torque_with_deadzone = apply_deadzone(steering_torque, STEER_OVERRIDE_MIN_TORQUE)
    neutral_torque = 0.0 if abs(vEgo) <= STEER_OVERRIDE_STANDSTILL_VEGO else self.angle_override / torque_to_angle
    return driver_torque_with_deadzone * torque_to_angle, driver_torque_with_deadzone - neutral_torque

  def override_slew_step(self, angle_override_target: float, override_torque: float) -> float:
    target_error = angle_override_target - self.angle_override
    slew_rate_away = calc_override_angle_delta_limit(abs(override_torque), STEER_OVERRIDE_DELTA_GAIN_LIMIT)
    slew_rate_center = calc_override_angle_delta_limit(abs(override_torque), STEER_OVERRIDE_DELTA_GAIN_LIMIT_CENTERING)
    down_step = slew_rate_center if self.angle_override > 0 else slew_rate_away
    up_step = slew_rate_center if self.angle_override < 0 else slew_rate_away
    return float(np.clip(target_error, -down_step, up_step))

  @staticmethod
  def adjust_slew_for_planner(slew_step: float, apply_angle_step: float, override_torque: float) -> float:
    direction = slew_step * apply_angle_step
    if direction > 0:
      return slew_step - apply_bounds(apply_angle_step, abs(slew_step))
    if direction < 0:
      return slew_step - abs(override_torque) / STEER_OVERRIDE_TORQUE_RANGE * apply_angle_step
    return slew_step

  @staticmethod
  def unwind_on_saturation(angle_override: float, sat_error: float) -> float:
    if angle_override * sat_error <= 0:
      return angle_override
    return angle_override - apply_bounds(sat_error, abs(angle_override))

  def resume_steer_desired_rate_limit(self, lat_active: bool, apply_angle: float, steering_angle: float) -> float:
    if not lat_active:
      self.resume_rate_limiter_delta.reset(0)
      self.resume_rate_limiter.reset(steering_angle)
      return steering_angle

    angle_rate_delta_lim = self.resume_rate_limiter_delta.update(CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE,
                                                         STEER_RESUME_RATE_LIMIT_RAMP_RATE * DT_LAT_CTRL**2)
    apply_angle_lim = self.resume_rate_limiter.update(apply_angle, angle_rate_delta_lim)
    return apply_angle_lim

  def update(self, apply_angle, lat_active, CS: structs.CarState, VM: VehicleModel) -> CoopSteeringData:
    # estimate real steering angle by adding rate to the tesla filtered angle
    steeringAngleDegPhaseLead = CS.out.steeringAngleDeg + CS.out.steeringRateDeg / STEERING_DEG_PHASE_LEAD_COEFF

    # avoid sudden rotation on engagement
    apply_angle = self.resume_steer_desired_rate_limit(lat_active, apply_angle, steeringAngleDegPhaseLead)

    if not lat_active:
      self.reset_override_state(apply_angle)
      return CoopSteeringData(apply_angle, lat_active, 1)

    apply_angle_step = apply_angle - self.apply_angle_last
    self.apply_angle_last = apply_angle
    angle_override_target, override_torque = self.compute_override_targets(CS.out.vEgo, CS.out.steeringTorque, VM)
    slew_step = self.adjust_slew_for_planner(self.override_slew_step(angle_override_target, override_torque), apply_angle_step, override_torque)
    self.angle_override += slew_step

    apply_angle += self.angle_override
    self.coop_apply_angle_sat_last = apply_steer_angle_limits_vm(apply_angle, self.coop_apply_angle_sat_last, CS.out.vEgoRaw,
                                                                   CS.out.steeringAngleDeg, lat_active, CoopSteeringCarControllerParams, VM)
    self.angle_override = self.unwind_on_saturation(self.angle_override, apply_angle - self.coop_apply_angle_sat_last)
    return CoopSteeringData(self.coop_apply_angle_sat_last, lat_active, 1)
