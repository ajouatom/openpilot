import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees


class LatControlAngle(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.sat_check_min_speed = 5.
    self.angle_steers_des = 0.0
    self.angle_steers_des_alpha = 0.1
    self.yStd = 0.0
    self.yStd_alpha = 0.1

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, llk, curvature_limited, model_data=None):  
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if model_data is not None:
      yStd = model_data.position.yStd[5]
    else:
      yStd = 0.0
    self.yStd = self.yStd * (1 - self.yStd_alpha) + yStd * self.yStd_alpha    

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

    self.angle_steers_des_alpha = np.interp(self.yStd, [0.3, 0.5], [1.0, 0.1])
    self.angle_steers_des = angle_steers_des * (1 - self.angle_steers_des_alpha) + self.angle_steers_des_alpha * self.angle_steers_des

    angle_control_saturated = abs(self.angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = self.angle_steers_des if not CS.steeringPressed else float(CS.steeringAngleDeg)
    return 0, float(self.angle_steers_des), angle_log
