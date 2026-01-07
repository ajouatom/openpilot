import math

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController
from openpilot.common.filter_simple import FirstOrderFilter

LaneChangeState = log.LaneChangeState


class LatControlPID(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.pid = PIDController((CP.lateralTuning.pid.kpBP, CP.lateralTuning.pid.kpV),
                             (CP.lateralTuning.pid.kiBP, CP.lateralTuning.pid.kiV),
                             k_f=CP.lateralTuning.pid.kf, pos_limit=self.steer_max, neg_limit=-self.steer_max)
    self.get_steer_feedforward = CI.get_steer_feedforward_function()

    # Steering pressed 감쇠 필터: 수동 조향 시 자동조향을 부드럽게 완화
    # 1.0 = 정상, 0.25 = steeringPressed 시 감쇠 (약간의 자동조향 감도 유지)
    # 차선변경 중일 때는 더 긴 시정수로 부드러운 전환 (nudge 모드 대응)
    self.steer_pressed_reduction_factor = FirstOrderFilter(1.0, 0.6, 0.01)  # 기본 0.6초 시정수
    self.steer_pressed_reduction_factor_lc = FirstOrderFilter(1.0, 0.9, 0.01)  # 차선변경 시 0.9초 시정수

  def reset(self):
    super().reset()
    self.pid.reset()

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, CC, curvature_limited, model_data=None):
    pid_log = log.ControlsState.LateralPIDState.new_message()
    pid_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    pid_log.steeringRateDeg = float(CS.steeringRateDeg)

    angle_steers_des_no_offset = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
    angle_steers_des = angle_steers_des_no_offset + params.angleOffsetDeg
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
                                     feedforward=steer_feedforward, speed=CS.vEgo)

      # steeringPressed 상태에서 자동조향을 느슨하게 하여 수동 조향을 부드럽게
      # 감쇠 계수: 1.0 (정상) -> 0.25 (steeringPressed, 약간의 자동조향 감도 유지)
      # 차선변경 중일 때는 더 부드러운 전환을 위해 긴 시정수 필터 사용 (nudge 모드 대응)
      is_lane_changing = False
      if model_data is not None and hasattr(model_data, 'meta') and hasattr(model_data.meta, 'laneChangeState'):
        lane_change_state = model_data.meta.laneChangeState
        # laneChangeStarting(2) 또는 laneChangeFinishing(3) 상태일 때
        is_lane_changing = lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)

      target_reduction = 0.25 if CS.steeringPressed else 1.0
      if is_lane_changing:
        # 차선변경 중일 때는 더 긴 시정수로 부드러운 전환 (nudge 모드에서 자동조향으로 전환 시)
        reduction_factor = self.steer_pressed_reduction_factor_lc.update(target_reduction)
      else:
        reduction_factor = self.steer_pressed_reduction_factor.update(target_reduction)
      output_steer = output_steer * reduction_factor

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(output_steer)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_steer) < 1e-3, CS, steer_limited_by_controls, curvature_limited))

    return output_steer, angle_steers_des, pid_log
