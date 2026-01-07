import math
import numpy as np

from cereal import log
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.filter_simple import FirstOrderFilter

LaneChangeState = log.LaneChangeState

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # Degrees


class LatControlAngle(LatControl):
  def __init__(self, CP, CI):
    super().__init__(CP, CI)
    self.sat_check_min_speed = 5.

    # Steering pressed 감쇠 필터: 수동 조향 시 자동조향을 부드럽게 완화
    # angle 제어에서는 desired angle을 현재 angle 쪽으로 부드럽게 조정
    # 차선변경 중일 때는 더 긴 시정수로 부드러운 전환 (nudge 모드 대응)
    self.steer_pressed_reduction_factor = FirstOrderFilter(1.0, 0.6, 0.01)  # 기본 0.6초 시정수
    self.steer_pressed_reduction_factor_lc = FirstOrderFilter(1.0, 0.9, 0.01)  # 차선변경 시 0.9초 시정수

  def update(self, active, CS, VM, params, steer_limited_by_controls, desired_curvature, CC, curvature_limited, model_data=None):
    angle_log = log.ControlsState.LateralAngleState.new_message()

    if not active:
      angle_log.active = False
      angle_steers_des = float(CS.steeringAngleDeg)
    else:
      angle_log.active = True
      angle_steers_des = math.degrees(VM.get_steer_from_curvature(-desired_curvature, CS.vEgo, params.roll))
      angle_steers_des += params.angleOffsetDeg

    # steeringPressed 상태에서 자동조향을 느슨하게 하여 수동 조향을 부드럽게
    # angle 제어에서는 desired angle을 현재 angle 쪽으로 부드럽게 조정
    # 차선변경 중일 때는 더 부드러운 전환을 위해 긴 시정수 필터 사용 (nudge 모드 대응)
    is_lane_changing = False
    if model_data is not None and hasattr(model_data, 'meta') and hasattr(model_data.meta, 'laneChangeState'):
      lane_change_state = model_data.meta.laneChangeState
      # laneChangeStarting(2) 또는 laneChangeFinishing(3) 상태일 때
      is_lane_changing = lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)

    if CS.steeringPressed:
      # steeringPressed일 때는 desired angle을 현재 angle 쪽으로 부드럽게 조정
      # 감쇠 계수: 1.0 (정상) -> 0.25 (steeringPressed, 약간의 자동조향 감도 유지)
      target_reduction = 0.25
    else:
      target_reduction = 1.0

    if is_lane_changing:
      # 차선변경 중일 때는 더 긴 시정수로 부드러운 전환 (nudge 모드에서 자동조향으로 전환 시)
      reduction_factor = self.steer_pressed_reduction_factor_lc.update(target_reduction)
    else:
      reduction_factor = self.steer_pressed_reduction_factor.update(target_reduction)

    # desired angle을 현재 angle과의 차이에 reduction_factor를 적용하여 부드럽게 조정
    angle_diff = angle_steers_des - CS.steeringAngleDeg
    angle_steers_des_adjusted = CS.steeringAngleDeg + angle_diff * reduction_factor

    angle_control_saturated = abs(angle_steers_des_adjusted - CS.steeringAngleDeg) > STEER_ANGLE_SATURATION_THRESHOLD
    angle_log.saturated = bool(self._check_saturation(angle_control_saturated, CS, False, curvature_limited))
    angle_log.steeringAngleDeg = float(CS.steeringAngleDeg)
    angle_log.steeringAngleDesiredDeg = float(angle_steers_des_adjusted) if not CS.steeringPressed else float(CS.steeringAngleDeg)
    return 0, float(angle_steers_des_adjusted), angle_log
