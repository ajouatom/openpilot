import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill, a_ego, stopping_accel, radarState):
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
        leadOne = radarState.leadOne
        fcw_stop = leadOne.status and leadOne.dRel < 4.0
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

    # ── Coasting deadband + hysteresis ──────────────────────────────
    # 사람 주행의 "발 떼고 코스팅(회생제동)" 구간을 흉내내, 계획 가속도가 0 근처에서
    # 미세하게 +/- 진동(울렁거림)할 때 출력 가감속을 0으로 부드럽게 수렴시킨다.
    # LongCoastBand: 코스팅 진입 임계 가속도(0.01 m/s² 단위, 0=비활성).
    # 진입 임계의 2배를 벗어나야 코스팅을 빠져나가는 히스테리시스로 채터링을 방지.
    self.coast_band = 0.0       # m/s² (0=off)
    self.coasting = False
    self.COAST_EXIT_JERK = 1.2  # m/s³, 코스팅 진입 시 출력→0 수렴 속도

    self.use_accel_pid = False
    if CP.brand == "toyota":
      self.use_accel_pid = True

  def reset(self):
    self.pid.reset()
    self.coasting = False

  def _update_coast_state(self, a_target):
    """계획 가속도(a_target) 기준으로 코스팅 진입/이탈을 히스테리시스로 판정."""
    if self.coast_band <= 0.0:
      self.coasting = False
      return
    if self.coasting:
      # 진입 임계의 2배(이탈 임계)를 넘어서는 분명한 가감속 요구가 있을 때만 이탈
      if abs(a_target) > self.coast_band * 2.0:
        self.coasting = False
    else:
      if abs(a_target) < self.coast_band:
        self.coasting = True

  def _coast_output(self):
    """코스팅 중 출력 가감속을 0(무가감속=자연 회생제동)으로 부드럽게 램프."""
    step = self.COAST_EXIT_JERK * DT_CTRL
    oa = self.last_output_accel
    if oa > step:
      return oa - step
    if oa < -step:
      return oa + step
    return 0.0

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
      self.coast_band = self.params.get_float("LongCoastBand") * 0.01
    elif self.readParamCount == 10:
      if len(self.CP.longitudinalTuning.kpBP) == 1 and len(self.CP.longitudinalTuning.kiBP)==1:
        longitudinalTuningKpV = self.params.get_float("LongTuningKpV") * 0.01
        longitudinalTuningKiV = self.params.get_float("LongTuningKiV") * 0.001
        self.pid._k_p = (self.CP.longitudinalTuning.kpBP, [longitudinalTuningKpV])
        self.pid._k_i = (self.CP.longitudinalTuning.kiBP, [longitudinalTuningKiV])
        self.pid.k_f = self.params.get_float("LongTuningKf") * 0.01


    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

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

      stopAccel = self.stopping_accel if self.stopping_accel < 0.0 else self.CP.stopAccel
      if output_accel > stopAccel:
        # 저속일수록 감속 rate를 줄여서 정차 직전 꿀렁임 방지 (속도 비례 감속 한계 조절)
        speed_factor = float(np.interp(CS.vEgo, [0.0, 0.2, 0.5, 1.5], [0.05, 0.1, 0.3, 1.0]))
        # Brake Cushion: stopAccel에 가까워질수록 감속 rate를 더 줄여서 부드럽게 안착
        accel_margin = max(output_accel - stopAccel, 0.01)
        cushion_factor = float(np.interp(accel_margin, [0.0, 0.3, 1.0], [0.15, 0.5, 1.0]))
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * speed_factor * cushion_factor * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      if self.use_accel_pid:
        error = a_target_ff - CS.aEgo
      else:
        error = v_target_now - CS.vEgo
      # 코스팅 판정은 '계획 가속도(a_target_ff)'의 미세 진동을 기준으로 한다.
      self._update_coast_state(a_target_ff)
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target_ff,
                                     freeze_integrator=self.coasting)
      if self.coasting:
        # 코스팅 중에는 출력을 0으로 부드럽게 수렴 → 자연 회생제동/엔진브레이크 활용
        output_accel = self._coast_output()

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel, a_target_ff, j_target_now
