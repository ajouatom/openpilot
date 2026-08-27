import numpy as np
from openpilot.cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# ── 정차 마무리(소프트랜딩) ─────────────────────────────────────────
# 계획(aTarget)은 정지 직전 감속을 0 부근까지 풀어 부드럽게 안착하도록 수렴하는데,
# 종전 stopping 로직은 이를 무시하고 PID의 깊은 명령을 이어받아 stopAccel 방향으로
# 계속 조이기만 했다(하강 전용 램프). 결과: 정지 직전 감속이 되레 심화되는 '울컥' 정차.
# → 2단계로 분리: 바퀴가 멈추기 전(v>SETTLE)에는 계획 수준까지 브레이크를 완만히 풀어
#   소프트랜딩하고, 정지 후에만 홀드 압력(stopAccel)으로 조인다(정지 상태라 체감 없음).
# ※ openpilotLongitudinalControl=True(HyundaiCameraSCC<2) 구간에서만 동작.
STOP_SETTLE_SPEED = 0.15       # m/s   이 속도 미만이면 '바퀴 정지'로 보고 홀드 단계 전환
STOP_SOFT_MIN_DECEL = 0.30     # m/s^2 구르는 동안 유지할 최소 감속(크리프/경사 밀림 방지)
STOP_SOFT_RELEASE_JERK = 0.7   # m/s^3 소프트랜딩 시 브레이크 풀림(상승) 속도 제한
STOP_HOLD_FACTOR = 0.45        # 정지 후 stopAccel로 조이는 속도 배율(x stoppingDecelRate)


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill,
                             a_ego=0.0, stopping_accel=-0.5, radarState=None):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        stopping_accel = stopping_accel if stopping_accel < 0.0 else -0.5
        leadOne = getattr(radarState, "leadOne", None) if radarState is not None else None
        fcw_stop = bool(leadOne and getattr(leadOne, "status", False) and getattr(leadOne, "dRel", 10.0) < 4.0)
        if a_ego > stopping_accel or fcw_stop: # and v_ego < 1.0:
          long_control_state = LongCtrlState.stopping
        if long_control_state == LongCtrlState.starting:
          long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             k_f=CP.longitudinalTuning.kf, rate=1 / DT_CTRL)
    self.last_output_accel = 0.0


    self.params = Params()
    self.readParamCount = 0
    self.stopping_accel = 0
    self.j_lead = 0.0

    self.use_accel_pid = False
    if CP.brand == "toyota":
      self.use_accel_pid = True

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, long_plan, accel_limits, t_since_plan, radarState):

    soft_hold_active = CS.softHoldActive > 0
    a_target_ff = long_plan.aTarget
    v_target_now = long_plan.vTargetNow
    j_target_now = long_plan.jTargetNow
    should_stop = long_plan.shouldStop

    self.readParamCount += 1
    if self.readParamCount >= 100:
      self.readParamCount = 0
      self.stopping_accel = self.params.get_float("StoppingAccel") * 0.01
    elif self.readParamCount == 10:
      if len(self.CP.longitudinalTuning.kpBP) == 1 and len(self.CP.longitudinalTuning.kiBP)==1:
        longitudinalTuningKpV = self.params.get_float("LongTuningKpV") * 0.01
        longitudinalTuningKiV = self.params.get_float("LongTuningKiV") * 0.001
        self.pid._k_p = (self.CP.longitudinalTuning.kpBP, [longitudinalTuningKpV])
        self.pid._k_i = (self.CP.longitudinalTuning.kiBP, [longitudinalTuningKiV])
        self.pid.k_f = self.params.get_float("LongTuningKf") * 0.01


    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = min(accel_limits[1], max(1.0, a_target_ff + 0.35))

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill, CS.aEgo, self.stopping_accel, radarState)
    if active and soft_hold_active:
      self.long_control_state = LongCtrlState.stopping

    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel

      if soft_hold_active:
        output_accel = self.CP.stopAccel
      else:
        stopAccel = self.stopping_accel if self.stopping_accel < 0.0 else self.CP.stopAccel
        if CS.vEgo > STOP_SETTLE_SPEED:
          # 소프트랜딩: 계획이 풀라는 만큼(단, 최소 감속은 유지) 브레이크를 완만히 풀어 안착.
          # 계획이 더 깊은 감속을 요구하면(경사/정지점 초과 등) 그쪽으로 조여 따라간다.
          soft_target = float(np.clip(a_target_ff, stopAccel, -STOP_SOFT_MIN_DECEL))
          if output_accel < soft_target:
            output_accel = min(soft_target, output_accel + STOP_SOFT_RELEASE_JERK * DT_CTRL)
          else:
            output_accel = max(soft_target, output_accel - self.CP.stoppingDecelRate * DT_CTRL)
        elif output_accel > stopAccel:
          # 정지 완료: 홀드 압력(stopAccel)까지 조임(차량이 멈춰 있어 모션 체감 없음).
          # Brake Cushion: stopAccel에 가까워질수록 rate를 줄여 부드럽게 안착.
          accel_margin = max(output_accel - stopAccel, 0.01)
          cushion_factor = float(np.interp(accel_margin, [0.0, 0.3, 1.0], [0.15, 0.5, 1.0]))
          output_accel = min(output_accel, 0.0)
          output_accel -= self.CP.stoppingDecelRate * STOP_HOLD_FACTOR * cushion_factor * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      if self.use_accel_pid:
        error = a_target_ff - CS.aEgo
      else:
        error = v_target_now - CS.vEgo
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target_ff)

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel, a_target_ff, j_target_now
