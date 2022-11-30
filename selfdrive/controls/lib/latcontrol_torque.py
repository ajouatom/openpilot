import math

from cereal import log
from common.numpy_fast import interp
from selfdrive.controls.lib.latcontrol import LatControl
from selfdrive.controls.lib.pid import PIDController
from selfdrive.controls.lib.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from common.params import Params

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects we
# use a LOW_SPEED_FACTOR in the error. Additionally, there is
# friction in the steering wheel that needs to be overcome to
# move it at all, this is compensated for too.

LOW_SPEED_X = [0, 10, 20, 30]
LOW_SPEED_Y = [15, 13, 10, 5]


class LatControlTorque(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.torque_params = CP.lateralTuning.torque
    self.pid = PIDController(self.torque_params.kp, self.torque_params.ki,
                             k_f=self.torque_params.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.use_steering_angle = self.torque_params.useSteeringAngle
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.paramsCount = 0

    self.lateralTorqueCustom = Params().get_bool("LateralTorqueCustom")
    self.lateralTorqueAccelFactor = float(int(Params().get("LateralTorqueAccelFactor", encoding="utf8")))*0.001
    self.lateralTorqueFriction = float(int(Params().get("LateralTorqueFriction", encoding="utf8")))*0.001

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction

  def update_params(self):
    self.paramsCount += 1
    if self.paramsCount > 30:
      self.paramsCount = 0
      lateralTorqueKp = float(int(Params().get("LateralTorqueKp", encoding="utf8")))*0.01
      lateralTorqueKi = float(int(Params().get("LateralTorqueKi", encoding="utf8")))*0.01
      lateralTorqueKd = float(int(Params().get("LateralTorqueKd", encoding="utf8")))*0.01
      lateralTorqueKf = float(int(Params().get("LateralTorqueKf", encoding="utf8")))*0.01
      self.pid._k_p = [[0], [lateralTorqueKp]]
      self.pid._k_i = [[0], [lateralTorqueKi]]
      self.pid._k_d = [[0], [lateralTorqueKd]]
      self.pid.k_f = lateralTorqueKf
    elif self.paramsCount == 10:
      self.lateralTorqueCustom = Params().get_bool("LateralTorqueCustom")
      self.lateralTorqueAccelFactor = float(int(Params().get("LateralTorqueAccelFactor", encoding="utf8")))*0.001
      self.lateralTorqueFriction = float(int(Params().get("LateralTorqueFriction", encoding="utf8")))*0.001
      if self.lateralTorqueCustom:
        self.torque_params.latAccelFactor = self.lateralTorqueAccelFactor
        self.torque_params.friction = self.lateralTorqueFriction

  def update(self, active, CS, VM, params, last_actuators, steer_limited, desired_curvature, desired_curvature_rate, llk):
    self.update_params()
    pid_log = log.ControlsState.LateralTorqueState.new_message()

    if not active:
      output_torque = 0.0
      pid_log.active = False
      angle_steers_des = 0.
    else:
      if self.use_steering_angle:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      else:
        actual_curvature_vm = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        actual_curvature_llk = llk.angularVelocityCalibrated.value[2] / CS.vEgo
        actual_curvature = interp(CS.vEgo, [2.0, 5.0], [actual_curvature_vm, actual_curvature_llk])
        curvature_deadzone = 0.0
      desired_lateral_accel = desired_curvature * CS.vEgo ** 2

      # desired rate is the desired rate of change in the setpoint, not the absolute desired curvature
      # desired_lateral_jerk = desired_curvature_rate * CS.vEgo ** 2
      actual_lateral_accel = actual_curvature * CS.vEgo ** 2
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      low_speed_factor = interp(CS.vEgo, LOW_SPEED_X, LOW_SPEED_Y)**2
      setpoint = desired_lateral_accel + low_speed_factor * desired_curvature
      measurement = actual_lateral_accel + low_speed_factor * actual_curvature
      gravity_adjusted_lateral_accel = desired_lateral_accel - params.roll * ACCELERATION_DUE_TO_GRAVITY
      torque_from_setpoint = self.torque_from_lateral_accel(setpoint, self.torque_params, setpoint,
                                                     lateral_accel_deadzone, friction_compensation=False)
      torque_from_measurement = self.torque_from_lateral_accel(measurement, self.torque_params, measurement,
                                                     lateral_accel_deadzone, friction_compensation=False)
      pid_log.error = torque_from_setpoint - torque_from_measurement
      ff = self.torque_from_lateral_accel(gravity_adjusted_lateral_accel, self.torque_params,
                                          desired_lateral_accel - actual_lateral_accel,
                                          lateral_accel_deadzone, friction_compensation=True)

      freeze_integrator = steer_limited or CS.steeringPressed or CS.vEgo < 5
      output_torque = self.pid.update(pid_log.error,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

      #C2#pid_log.active = True
      #C2#pid_log.p = self.pid.p
      #C2#pid_log.i = self.pid.i
      #C2#pid_log.d = self.pid.d
      #C2#pid_log.f = self.pid.f
      #C2#pid_log.output = -output_torque
      #C2#pid_log.actualLateralAccel = actual_lateral_accel
      #C2#pid_log.desiredLateralAccel = desired_lateral_accel
      #C2#pid_log.saturated = self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited)
      self.latDebugText = 'latAccel={:1.3f},Friction={:1.3f}'.format(self.torque_params.latAccelFactor, self.torque_params.friction)
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll)) + params.angleOffsetDeg

    # TODO left is positive in this convention
    return -output_torque, angle_steers_des, pid_log
