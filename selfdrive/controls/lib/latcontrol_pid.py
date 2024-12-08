import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.params import Params


class LatControlPID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()
    # carrot
    self.frame = 0
    self.params = Params()
    self.error_last = 0.0

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, steer_limited, desired_curvature, desired_curvature_now, calibrated_pose):
    self.frame += 1
    if self.frame % 10 == 0:
      lateralTorqueKp = self.params.get_float("LateralTorqueKp")*0.01
      lateralTorqueKi = self.params.get_float("LateralTorqueKi")*0.01
      lateralTorqueKd = self.params.get_float("LateralTorqueKd")*0.01
      self.pid._k_p = [[0], [lateralTorqueKp]]
      self.pid._k_i = [[0], [lateralTorqueKi]]
      self.pid._k_d = [[0], [lateralTorqueKd]]
  
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des_no_offset_now = math.degrees(VM.get_steer_from_curvature(-desired_curvature_now, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset_now + params.angleOffsetDeg
    error = angle_steers_des - CS.steeringAngleDeg

    pid_log.steeringAngleDesiredDeg = angle_steers_des
    pid_log.angleError = error
    if not active:
      output_steer = 0.0
      pid_log.active = False
      self.pid.reset()
    else:
      # offset does not contribute to resistive torque
      steer_feedforward = self.get_steer_feedforward(angle_steers_des_no_offset, CS.vEgo)

      output_steer = self.pid.update(error, override=CS.steeringPressed,
                                     error_rate=error - self.error_last,
                                     feedforward=steer_feedforward, speed=CS.vEgo)
      pid_log.active = True
      pid_log.p = self.pid.p
      pid_log.i = self.pid.i
      pid_log.f = self.pid.f
      pid_log.output = output_steer
      pid_log.saturated = self._check_saturation(self.steer_max - abs(output_steer) < 1e-3, CS, steer_limited)

    self.error_last = error

    return output_steer, angle_steers_des, pid_log
