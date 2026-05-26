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
STEER_OVERRIDE_MIN_TORQUE = 0.5 # Nm - based on typical steering bias + noise
STEER_OVERRIDE_MAX_TORQUE = 2.5 # Nm max torque before EPS disengages
STEER_OVERRIDE_MAX_LAT_ACCEL = 1.5 # m/s^2 - determines angle rate - speed dependent - similar to Tesla comfort steering mode
STEER_OVERRIDE_LAT_ACCEL_GAIN_LIMIT = 10 # deg/Nm stability and smoothness for angle control

# angle ramping
STEER_OVERRIDE_MAX_LAT_JERK = 2.0 # m/s^3 - determines angle ramping rate - speed dependent
STEER_OVERRIDE_MAX_LAT_JERK_CENTERING = CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_LATERAL_JERK # m/s^3 - for low speed angle ramp down
# stability and smoothness for angle ramp control - at very low speeds this takes precedence over jerk settings
STEER_OVERRIDE_LAT_JERK_GAIN_LIMIT = 100 # deg/s/Nm
STEER_OVERRIDE_TORQUE_RANGE = STEER_OVERRIDE_MAX_TORQUE - STEER_OVERRIDE_MIN_TORQUE

# limit model acceleration when engaging
STEER_RESUME_RATE_LIMIT_RAMP_RATE = 500 # deg/s^2


CoopSteeringData = namedtuple("CoopSteeringData",
                              ["steeringAngleDeg", "lat_active", "control_type"])


def get_steer_from_lat_accel(lat_accel, v_ego: float, VM: VehicleModel):
  curvature = lat_accel / (max(1, v_ego) ** 2)
  return math.degrees(VM.get_steer_from_curvature(curvature, v_ego, 0))


def apply_bounds(signal: float, limit: float) -> float:
  return float(np.clip(signal, -limit, limit))


def apply_deadzone(signal: float, deadzone: float) -> float:
  return signal - apply_bounds(signal, deadzone)


def calc_override_angle_limited(torque: float, vEgo: float, VM: VehicleModel, lat_accel) -> float:
  torque_to_angle = get_steer_from_lat_accel(lat_accel, vEgo, VM) / STEER_OVERRIDE_TORQUE_RANGE
  gain_limit = STEER_OVERRIDE_LAT_ACCEL_GAIN_LIMIT
  override_angle_target = torque * min(torque_to_angle, gain_limit)
  return override_angle_target


def calc_override_angle_delta_limited(torque: float, vEgo: float, VM: VehicleModel, lat_jerk) -> float:
  # prevents windup in carcontroller rate limiter
  lat_jerk = min(lat_jerk, CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_LATERAL_JERK)

  torque_to_angle = get_steer_from_lat_accel(lat_jerk, vEgo, VM) / STEER_OVERRIDE_TORQUE_RANGE
  gain_limit = min(STEER_OVERRIDE_LAT_JERK_GAIN_LIMIT, CarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE / DT_CTRL / STEER_OVERRIDE_TORQUE_RANGE)
  override_angle_rate = torque * min(torque_to_angle, gain_limit)

  # prevent windup in angle rate limiter
  return apply_bounds(override_angle_rate * DT_LAT_CTRL, CoopSteeringCarControllerParams.ANGLE_LIMITS.MAX_ANGLE_RATE)


class SteerRateLimiter:
  def __init__(self):
    self._last = 0.0

  def reset(self, angle: float) -> None:
    self._last = angle

  def update(self, angle: float, angle_delta_lim: float) -> float:
    angle_lim = rate_limit(angle, self._last, -angle_delta_lim, angle_delta_lim)
    self._last = angle_lim
    return angle_lim


class SteerAccelLimiter:
  def __init__(self):
    self.delta_rl = SteerRateLimiter()
    self.angle_cmd = 0.0

  def reset(self, angle: float) -> None:
    self.delta_rl.reset(0)
    self.angle_cmd = angle

  def update(self, angle_target: float, max_rate: float, accel: float, decel: float, dt: float) -> float:
    if dt <= 0.0:
      return self.angle_cmd

    accel_delta = max(0.0, accel) * (dt * dt)
    decel_delta = max(0.0, decel) * (dt * dt)

    err = angle_target - self.angle_cmd
    err = apply_bounds(err, max(0.0, max_rate) * dt)

    if err * self.delta_rl._last < 0:
      delta = decel_delta
    else:
      delta = accel_delta

    # Handle large decel (enabled with inf value)
    if decel == np.inf and err * self.delta_rl._last < 0:
      self.delta_rl._last = 0
      angle_out = self.angle_cmd
    else:
      self.delta_rl._last = self.delta_rl.update(err, delta)
      if decel == np.inf:
        self.delta_rl._last = apply_bounds(self.delta_rl._last, abs(err))
      angle_out = self.angle_cmd + self.delta_rl._last

    self.angle_cmd = angle_out
    return angle_out


class CoopSteeringCarController:
  def __init__(self):
    self.coop_apply_angle_last = 0
    self.coop_apply_angle_last_sat = 0
    self.override_angle_accu = 0
    self.override_active_counter = 0
    self.resume_rate_limiter_delta = SteerRateLimiter()
    self.resume_rate_limiter = SteerRateLimiter()
    self.override_accel_rate_limiter = SteerAccelLimiter()

  def apply_override_angle_direct(self, lat_active: bool, driverTorque: float, vEgo: float, VM: VehicleModel) -> float:
    if not lat_active:
      return 0.0

    steering_torque_with_deadzone = apply_deadzone(driverTorque, STEER_OVERRIDE_MIN_TORQUE)
    angle_override = calc_override_angle_limited(steering_torque_with_deadzone, vEgo, VM, STEER_OVERRIDE_MAX_LAT_ACCEL)
    return angle_override

  def apply_override_angle_relative(self, lat_active: bool, driverTorque: float, vEgo: float,
                                    VM: VehicleModel, unwind_weight: float = 1.0) -> float:
    if not lat_active:
      self.override_angle_accu = 0
      return 0

    # unwind accumulator toward zero if the previous loop saturated
    unwind = (self.coop_apply_angle_last - self.coop_apply_angle_last_sat) * unwind_weight
    if self.override_angle_accu * unwind > 0:
      unwind = apply_bounds(unwind, abs(self.override_angle_accu))
      self.override_angle_accu -= unwind

    # torque biasing emulates the steering centering when released
    if self.override_angle_accu > 0 and abs(vEgo) > 0.1:
      torque_biased = driverTorque - STEER_OVERRIDE_MIN_TORQUE
    elif self.override_angle_accu < 0 and abs(vEgo) > 0.1:
      torque_biased = driverTorque + STEER_OVERRIDE_MIN_TORQUE
    else:
      torque_biased = apply_deadzone(driverTorque, STEER_OVERRIDE_MIN_TORQUE)

    # higher rate when centering
    angle_override_delta = calc_override_angle_delta_limited(torque_biased, vEgo, VM,
                          STEER_OVERRIDE_MAX_LAT_JERK if (torque_biased * self.override_angle_accu) > 0
                          else STEER_OVERRIDE_MAX_LAT_JERK_CENTERING)

    new_override_angle_accu = self.override_angle_accu + angle_override_delta
    # snap to 0 if sign changes and driver torque is in centering zone
    if (new_override_angle_accu * self.override_angle_accu) < 0 and abs(driverTorque) < STEER_OVERRIDE_MIN_TORQUE:
      new_override_angle_accu = 0

    self.override_angle_accu = new_override_angle_accu
    return self.override_angle_accu

  def apply_override_angle_combined(self, lat_active: bool, driverTorque: float, vEgo: float, VM: VehicleModel) -> float:
    if not lat_active:
      return 0

    # calculate capability of direct angle override (fully active above ~36kph)
    direct_override_capability = (calc_override_angle_limited(STEER_OVERRIDE_TORQUE_RANGE, vEgo, VM, STEER_OVERRIDE_MAX_LAT_ACCEL) /
                   get_steer_from_lat_accel(STEER_OVERRIDE_MAX_LAT_ACCEL, vEgo, VM))

    angle_override_direct = self.apply_override_angle_direct(lat_active, driverTorque, vEgo, VM)
    relative_weight = 1.0 - direct_override_capability
    angle_override_relative = self.apply_override_angle_relative(lat_active, driverTorque, vEgo, VM,
                                                                 unwind_weight=relative_weight)

    return angle_override_direct * direct_override_capability + angle_override_relative * relative_weight

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

    apply_angle += self.apply_override_angle_combined(lat_active, CS.out.steeringTorque, CS.out.vEgo, VM)

    # final rate limit - matching panda safety
    self.coop_apply_angle_last = apply_angle
    self.coop_apply_angle_last_sat = apply_steer_angle_limits_vm(apply_angle, self.coop_apply_angle_last_sat, CS.out.vEgoRaw,
                                                    CS.out.steeringAngleDeg, lat_active, CoopSteeringCarControllerParams, VM)

    return CoopSteeringData(self.coop_apply_angle_last_sat, lat_active, 1)
