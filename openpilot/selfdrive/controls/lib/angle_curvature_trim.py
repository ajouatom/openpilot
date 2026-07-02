import math
import numpy as np


ANGLE_TRIM_MAX_CURVATURE = 4.0e-4
ANGLE_TRIM_I = 0.10
ANGLE_TRIM_BUILDUP_BOOST = 2.5
ANGLE_TRIM_LEAK_RATE = 0.995
HYUNDAI_ANGLE_CONTROL_FLAG = 1 << 24


class AngleCurvatureTrim:
  def __init__(self, car_flags):
    self.enabled = bool(car_flags & HYUNDAI_ANGLE_CONTROL_FLAG)
    self.curvature_trim = 0.0

  def reset(self):
    self.curvature_trim = 0.0

  def update(self, active, CS, VM, params, desired_curvature):
    if not active:
      self.reset()
      return float(CS.steeringAngleDeg)

    corrected_curvature = float(desired_curvature)
    if not self.enabled or CS.steeringPressed or CS.vEgo < 0.3:
      self.reset()
    else:
      actual_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
      curvature_error = float(desired_curvature) - actual_curvature
      buildup = curvature_error * float(desired_curvature) > 0.0 and abs(CS.steeringRateDeg) > 5.0
      trim_i = ANGLE_TRIM_I * (ANGLE_TRIM_BUILDUP_BOOST if buildup else 1.0)
      self.curvature_trim += curvature_error * trim_i * 0.01
      self.curvature_trim *= ANGLE_TRIM_LEAK_RATE
      self.curvature_trim = float(np.clip(self.curvature_trim, -ANGLE_TRIM_MAX_CURVATURE, ANGLE_TRIM_MAX_CURVATURE))
      corrected_curvature += self.curvature_trim

    angle_steers_des = math.degrees(VM.get_steer_from_curvature(-corrected_curvature, CS.vEgo, params.roll))
    angle_steers_des += params.angleOffsetDeg
    return float(CS.steeringAngleDeg) if CS.steeringPressed else float(angle_steers_des)
