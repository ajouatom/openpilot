import math

from cereal import log
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees


class LatControlAngle(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.sat_check_min_speed = 5.
    self.use_neurodob = getattr(CI, "use_neurodob", False)
    self.neurodob_model = getattr(CI, "neurodob_model", None)
    self._prev_angle_error = 0.0

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, CC, curvature_limited, model_data=None):  
    angle_log = log.ControlsState.LateralAngleState.new_message()
    neurodob_delta = 0.0

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

      if self.use_neurodob and self.neurodob_model is not None:
        try:
          desired_lateral_accel = desired_curvature * CS.vEgo ** 2
          angle_error = angle_steers_des - CS.steeringAngleDeg
          angle_error_rate = (angle_error - self._prev_angle_error) / DT_CTRL
          desired_yaw_rate = desired_curvature * CS.vEgo
          yaw_rate_error = desired_yaw_rate - CS.yawRate
          state_features = [
            angle_error,
            angle_error_rate,
            yaw_rate_error,
            CS.steeringRateDeg,
            angle_steers_des,
            CS.vEgo,
            params.roll,
            desired_lateral_accel,
          ]
          neurodob_delta = self.neurodob_model.predict(state_features)
          angle_steers_des += neurodob_delta
          self._prev_angle_error = angle_error
        except Exception as e:
          print(f"NeuroDOB predict failed, disabling model: {e}")
          self.use_neurodob = False
          self.neurodob_model = None

    angle_control_saturated = abs(angle_steers_des - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.output = float(neurodob_delta)
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = float(angle_steers_des) if not CS.steeringPressed else float(CS.steeringAngleDeg)
    return 0, float(angle_steers_des), angle_log

  def reset(self):
    super().reset()
    self._prev_angle_error = 0.0
    if self.use_neurodob and self.neurodob_model is not None:
      self.neurodob_model.reset()
