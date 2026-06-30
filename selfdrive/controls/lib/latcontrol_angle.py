import math
import numpy as np

from cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

STEER_ANGLE_SATURATION_THRESHOLD = 3.0  # Degrees
ANGLE_TRIM_MAX_CURVATURE = 4.0e-4  # about one steering-wheel degree on HKG LFA2
ANGLE_TRIM_I = 0.10
ANGLE_TRIM_BUILDUP_BOOST = 2.5
ANGLE_TRIM_LEAK_RATE = 0.995
HYUNDAI_ANGLE_CONTROL_FLAG = 1 << 24


class LatControlAngle(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.sat_check_min_speed = 5.
    self.curvature_trim = 0.0
    self.curvature_trim_enabled = bool(CP.flags & HYUNDAI_ANGLE_CONTROL_FLAG)
    angle_control_level = int(np.clip(Params().get_int("AngleControlBlend"), 0, 100))
    self.angle_policy_blend = angle_control_level / 100.0 if self.curvature_trim_enabled else 0.0

  def reset(self):
    super().reset()
    self.curvature_trim = 0.0

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, CC, curvature_limited, model_data=None):  
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
      self.curvature_trim = 0.0
    else:
      angle_log.active = True
      corrected_curvature = float(desired_curvature)
      if not self.curvature_trim_enabled or self.angle_policy_blend <= 0.0 or CS.steeringPressed or CS.vEgo < 0.3:
        self.curvature_trim = 0.0
      else:
        actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
        curvature_error = float(desired_curvature) - actual_curvature
        buildup = curvature_error * float(desired_curvature) > 0.0 and abs(CS.steeringRateDeg) > 5.0
        trim_i = ANGLE_TRIM_I * (ANGLE_TRIM_BUILDUP_BOOST if buildup else 1.0)
        self.curvature_trim += curvature_error * trim_i * 0.01
        self.curvature_trim *= ANGLE_TRIM_LEAK_RATE
        self.curvature_trim = float(np.clip(self.curvature_trim, -ANGLE_TRIM_MAX_CURVATURE, ANGLE_TRIM_MAX_CURVATURE))
        corrected_curvature += self.curvature_trim * self.angle_policy_blend

      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-corrected_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg
      if CS.steeringPressed and self.angle_policy_blend > 0.0:
        angle_steers_des = float(np.interp(self.angle_policy_blend, [0.0, 1.0],
                                           [angle_steers_des, CS.steeringAngleDeg]))

    angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = float(angle_steers_des) if not CS.steeringPressed else float(CS.steeringAngleDeg)
    return 0, float(angle_steers_des), angle_log
