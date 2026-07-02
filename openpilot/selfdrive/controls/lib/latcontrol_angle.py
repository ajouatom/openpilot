from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.angle_curvature_trim import AngleCurvatureTrim
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

STEER_ANGLE_SATURATION_THRESHOLD = 3.0  # Degrees


class LatControlAngle(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.sat_check_min_speed = 5.
    self.angle_trim = AngleCurvatureTrim(CP.flags)

  def reset(self):
    super().reset()
    self.angle_trim.reset()

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, CC, curvature_limited, model_data=None):  
    angle_log = log.ControlsState.LateralAngleState.new_message()

    angle_log.active = bool(active)
    angle_steers_des = self.angle_trim.update(active, CS, VM, params, desired_curvature)

    angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = float(angle_steers_des) if not CS.steeringPressed else float(CS.steeringAngleDeg)
    return 0, float(angle_steers_des), angle_log
