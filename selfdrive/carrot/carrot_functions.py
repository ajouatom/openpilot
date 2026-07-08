import time
from enum import Enum

from cereal import log
from openpilot.common.params import Params
import numpy as np
from openpilot.common.realtime import DT_MDL
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import MyMovingAverage
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.carrot.carrot_learning import CarrotLearner, DrivingStyleProfiler
from openpilot.common.swaglog import cloudlog

EventName = log.OnroadEvent.EventName
LaneChangeState = log.LaneChangeState

# 속도-가변 차간거리: 고정 stop_distance가 저속 time-gap을 부풀려 'time-gap 역전'이 생긴다
# (로그 f7: 5-15kph 3.4s vs 45kph+ 1.6s → 저속 과도하게 넓고 중고속은 좁음). 속도가 낮을수록
# t_follow를 약간 줄여(≤30 좁게) 저속 간격을 당기고, 높을수록 늘려(≥30 넓게) time-gap을
# 정상화한다(저속 좁게/고속 넓게 — 사용자 요청).
_SPDTF_BP    = [20.0, 32.0, 50.0, 80.0]   # 속도 보간점(km/h)
_SPDTF_DELTA = [-0.20, 0.0, 0.18, 0.28]   # 위 속도에서 t_follow 가감(초)
_SPDTF_MIN   = 0.55                        # 보정 후 t_follow 안전 하한(초)
# t_follow 감소(앞차 가속 등으로 간격 좁힐 때) 변화율 제한 — 즉시 스냅하면 '순간 가속'으로
# 느껴지므로 부드럽게. 증가(0.1)보다 빠르게 둬 추종 반응성은 유지. (스텝/호출당, *DT_MDL)
_TF_DECREASE_RATE = 1.5

# 정차 후 출발 catch-up: 완전정지→출발 구간에서 일시적으로 Gap1(최근접)으로 추종해
# 선행차를 민첩하게 따라잡고, 일정 속도 이상이면 원래 Gap으로 복귀한다. dynamic_t_follow의
# catch-up이 저속(<30)에서 꺼져 있어(사각지대) 이 구간을 보완. 거리(t_follow)에만 적용하고
# jerk/가속 특성은 원래 Gap을 유지하며, 복귀 시 apply_t_follow 증가율제한이 부드럽게 처리.
_LAUNCH_GAP_ARM_KPH    = 3.0    # 이 속도 미만(정차/near-stop)에서 Gap1 무장
_LAUNCH_GAP_REVERT_KPH = 40.0   # 이 속도 이상이면 원래 Gap으로 복귀

# 정차 후 출발 가속 부스트: 위 launch 구간에서 가속 상한을 HIGH 모드 수준으로 일시 확대해
# 캐치업 가속력을 확보한다(NORMAL 모드 CruiseMaxVals 상한에 막히는 문제 보완). 급발진 방지를
# 위해 배수를 S-커브(smoothstep)로 이징: 출발 초반(급발진 방지)·40km/h 복귀(툭 끊김 방지)
# 양 끝을 완만하게 하고 중속 구간에서 최대. 상한(ceiling)만 키우므로 catch-up이 필요할 때만
# MPC가 실제로 사용한다(정속·근접 추종에선 미사용).
_LAUNCH_ACCEL_GAIN     = 1.4    # 부스트 최대 배수(≈HIGH 모드 factor 상당)
_LAUNCH_EASE_IN_KPH    = 10.0   # 0→이 속도까지 S-커브로 부스트 상승(급발진 방지)
_LAUNCH_EASE_OUT_KPH   = 30.0   # 이 속도→REVERT까지 S-커브로 부스트 하강(복귀 부드럽게)

class XState(Enum):
  lead = 0
  cruise = 1
  e2eCruise = 2
  e2eStop = 3
  e2ePrepare = 4
  e2eStopped = 5

  def __str__(self):
    return self.name

class DrivingMode(Enum):
  Eco = 1
  Safe = 2
  Normal = 3
  High = 4

  def __str__(self):
    return self.name

class TrafficState(Enum):
  off = 0
  red = 1
  green = 2

  def __str__(self):
    return self.name

A_CRUISE_MAX_BP_CARROT = [0., 10 * CV.KPH_TO_MS, 40 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 110 * CV.KPH_TO_MS, 140 * CV.KPH_TO_MS]

class CarrotPlanner:
  def __init__(self):
    self.params = Params()
    self.params_count = 0
    self.frame = 0

    #self.log = ""

    #self.aChangeCost = 200
    #self.aChangeCostStart = 40
    #self.tFollowSpeedAdd = 0.0
    #self.tFollowSpeedAddM = 0.0
    #self.tFollowLeadCarSpeed = 0.0
    #self.tFollowLeadCarAccel = 0.0
    #self.lo_timer = 0
    #self.v_ego_prev = 0.0

    self.trafficState = TrafficState.off
    self.xStopFilter = MyMovingAverage(3)
    self.xStopFilter2 = MyMovingAverage(15)
    self.vFilter = MyMovingAverage(10)
    #self.t_follow_prev = self.get_T_FOLLOW()
    self.stop_distance = 6.0
    self.fakeCruiseDistance = 0.0
    self.xState = XState.cruise
    self.xStop = 0.0
    self.actual_stop_distance = 0.0
    #self.debugLongText = ""
    self.stopping_count = 0
    self.traffic_starting_count = 0
    self.user_stop_distance = -1

    self.t_follow_last = 1.5

    self.startSignCount = 0
    self.stopSignCount = 0

    self.stop_distance = 6.0
    self.trafficStopDistanceAdjust = 2.5 #params.get_float("TrafficStopDistanceAdjust") / 100.
    self.comfortBrake = 2.4
    self.comfort_brake = self.comfortBrake

    self.soft_hold_active = 0
    self.events = Events()
    self.myDrivingMode = DrivingMode(self.params.get_int("MyDrivingMode"))
    self.myDrivingMode_last = self.myDrivingMode
    self.myDrivingMode_disable_auto = False
    self.myEcoModeFactor = 0.9
    self.mySafeModeFactor = 0.8
    self.myHighModeFactor = 1.2
    self.drivingModeDetector = DrivingModeDetector()
    self.mySafeFactor = 1.0

    self.tFollowGap1 = 1.1
    self.tFollowGap2 = 1.3
    self.tFollowGap3 = 1.45
    self.tFollowGap4 = 1.6

    self.dynamicTFollow = 0.0
    self.dynamicTFollowLC = 0.0
    self.enableSpeedTF = 0
    self.tFollowDecelBoost = 0.0
    self.personality = 1
    self.launch_close_gap = False  # 정차 후 출발 catch-up: Gap1 일시 적용 상태

    self.cruiseMaxVals0 = 1.6
    self.cruiseMaxVals1 = 1.6
    self.cruiseMaxVals2 = 1.2
    self.cruiseMaxVals3 = 1.0
    self.cruiseMaxVals4 = 0.8
    self.cruiseMaxVals5 = 0.7
    self.cruiseMaxVals6 = 0.6

    self.aChangeCostStarting = 10.0

    self.trafficLightDetectMode = 2 # 0: None, 1:Stop, 2:Stop&Go
    self.trafficState_carrot = 0
    self.carrot_stay_stop = False

    self.eco_over_speed = 2
    self.eco_target_speed = 0
    
    self.autoNaviSpeedDecelRate = 1.5

    self.desireState = 0.0
    self.desireStateCount = 0
    self.jerk_factor = 1.0
    self.jerk_factor_apply = 1.0

    self.j_lead_factor = 0.0

    self.activeCarrot = 0
    self.xDistToTurn = 0
    self.atcType = ""
    self.atc_active = False
    self.tFollowSpeedFactor = 0.0 # 고속 주행 시 추가 차간 거리 가중치

    self._stop_x_rl = None
    self.last_event_time = 0.0
    self.learner = CarrotLearner()
    self.profiler = DrivingStyleProfiler()
    self.filtered_j_lead = 0.0
    self._v_ego_kph = 0.0

  def _params_update(self):
    self.frame += 1
    self.params_count += 1
    if self.params_count % 10 == 0:
      myDrivingMode = DrivingMode(self.params.get_int("MyDrivingMode"))
      if myDrivingMode != self.myDrivingMode_last:
        self.myDrivingMode_disable_auto = True
      self.myDrivingMode_last = myDrivingMode
      
      self.myDrivingModeAuto = self.params.get_int("MyDrivingModeAuto")
      if self.myDrivingModeAuto > 0 and not self.myDrivingMode_disable_auto:
        self.myDrivingMode = self.drivingModeDetector.get_mode()
      else:
        self.myDrivingMode = myDrivingMode

    if self.params_count == 10:
      self.myHighModeFactor = 1.2 #float(self.params.get_int("MyHighModeFactor")) / 100.
      self.trafficLightDetectMode = self.params.get_int("TrafficLightDetectMode") # 0: None, 1:Stop, 2:Stop&Go
    elif self.params_count == 20:
      self.tFollowGap1 = self.params.get_float("TFollowGap1") / 100.
      self.tFollowGap2 = self.params.get_float("TFollowGap2") / 100.
      self.tFollowGap3 = self.params.get_float("TFollowGap3") / 100.
      self.tFollowGap4 = self.params.get_float("TFollowGap4") / 100.
      self.tFollowSpeedFactor = self.params.get_float("TFollowSpeedFactor") / 100.
      self.dynamicTFollow = self.params.get_float("DynamicTFollow") / 100.
      self.dynamicTFollowLC = self.params.get_float("DynamicTFollowLC") / 100.
      self.enableSpeedTF = self.params.get_int("EnableSpeedTF")
      self.tFollowDecelBoost = self.params.get_float("TFollowDecelBoost") / 100.
    elif self.params_count == 30:
      self.cruiseMaxVals0 = self.params.get_float("CruiseMaxVals0") / 100.
      self.cruiseMaxVals1 = self.params.get_float("CruiseMaxVals1") / 100.
      self.cruiseMaxVals2 = self.params.get_float("CruiseMaxVals2") / 100.
      self.cruiseMaxVals3 = self.params.get_float("CruiseMaxVals3") / 100.
      self.cruiseMaxVals4 = self.params.get_float("CruiseMaxVals4") / 100.
      self.cruiseMaxVals5 = self.params.get_float("CruiseMaxVals5") / 100.
      self.cruiseMaxVals6 = self.params.get_float("CruiseMaxVals6") / 100.
    elif self.params_count == 40:
      self.stop_distance = self.params.get_float("StopDistanceCarrot") / 100.
      self.j_lead_factor = self.params.get_float("JLeadFactor3") / 100.
      self.eco_over_speed = self.params.get_int("CruiseEcoControl")
      self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
      self.aChangeCostStarting = self.params.get_float("AChangeCostStarting")
      self.trafficStopDistanceAdjust = self.params.get_float("TrafficStopDistanceAdjust") / 100.
    elif self.params_count >= 100:

      self.params_count = 0

  def _launch_accel_factor(self, v_ego):
    # 정차 후 출발 구간에서만 가속 상한 배수를 S-커브(smoothstep)로 이징해 반환.
    # 양 끝(출발 0, 복귀 REVERT)에서 1.0으로 수렴 → 급발진·복귀 툭 끊김 없이 매끄럽게.
    if not self.launch_close_gap:
      return 1.0
    v_kph = v_ego * CV.MS_TO_KPH
    if v_kph <= _LAUNCH_EASE_IN_KPH:
      s = v_kph / max(1.0, _LAUNCH_EASE_IN_KPH)                       # 0→1 (이징 인)
    elif v_kph >= _LAUNCH_EASE_OUT_KPH:
      span = max(1.0, _LAUNCH_GAP_REVERT_KPH - _LAUNCH_EASE_OUT_KPH)
      s = float(np.clip((_LAUNCH_GAP_REVERT_KPH - v_kph) / span, 0.0, 1.0))  # 1→0 (이징 아웃)
    else:
      s = 1.0                                                          # 중속: 최대 부스트
    w = s * s * (3.0 - 2.0 * s)                                        # smoothstep(양 끝 기울기 0)
    return 1.0 + (_LAUNCH_ACCEL_GAIN - 1.0) * w

  def get_carrot_accel(self, v_ego):
    cruiseMaxVals = [self.cruiseMaxVals0, self.cruiseMaxVals1, self.cruiseMaxVals2, self.cruiseMaxVals3, self.cruiseMaxVals4, self.cruiseMaxVals5, self.cruiseMaxVals6]
    factor = self.myHighModeFactor if self.myDrivingMode == DrivingMode.High else self.mySafeFactor
    accel = float(np.interp(v_ego, A_CRUISE_MAX_BP_CARROT, cruiseMaxVals) * factor)
    return accel * self._launch_accel_factor(v_ego)

  def _get_base_t_follow(self, personality, v_ego):
    if self.enableSpeedTF < 0:
      TF_SPEED_BPS = {
        -1: [0, 30, 60, 90],
        -2: [0, 40, 80, 120],
        -3: [0, 50, 100, 150],
      }

      v_kph = v_ego * CV.MS_TO_KPH
      bp = TF_SPEED_BPS.get(self.enableSpeedTF, [0, 30, 60, 90])

      tf_base = float(np.interp(
        v_kph,
        bp,
        [self.tFollowGap1, self.tFollowGap2, self.tFollowGap3, self.tFollowGap4]
      ))

      self.jerk_factor = float(np.interp(v_kph, bp, [1.0, 0.7, 0.5, 0.5]))

      if personality == log.LongitudinalPersonality.moreRelaxed:
        tf_base *= 2.0
      elif personality == log.LongitudinalPersonality.relaxed:
        tf_base *= 1.6
      elif personality == log.LongitudinalPersonality.standard:
        tf_base *= 1.3
      elif personality == log.LongitudinalPersonality.aggressive:
        tf_base *= 1.0
      else:
        raise NotImplementedError("Longitudinal personality not supported")

    else:
      if personality == log.LongitudinalPersonality.moreRelaxed:
        self.jerk_factor = 1.0
        tf_base = self.tFollowGap4
      elif personality == log.LongitudinalPersonality.relaxed:
        self.jerk_factor = 1.0
        tf_base = self.tFollowGap3
      elif personality == log.LongitudinalPersonality.standard:
        self.jerk_factor = 1.0 if self.myDrivingMode == DrivingMode.Safe else 0.7
        tf_base = self.tFollowGap2
      elif personality == log.LongitudinalPersonality.aggressive:
        self.jerk_factor = 1.0 if self.myDrivingMode == DrivingMode.Safe else 0.5
        tf_base = self.tFollowGap1
      else:
        raise NotImplementedError("Longitudinal personality not supported")

    return float(tf_base)


  def _apply_speed_t_follow_scale(self, tf_base, v_ego):
    tf_target = float(tf_base)

    # enableSpeedTF > 0:
    # 저속에서는 차간거리 축소, 고속으로 갈수록 원래값으로 복귀
    if self.enableSpeedTF > 0:
      reduce = self.enableSpeedTF * 0.01
      s = float(np.clip(v_ego * CV.MS_TO_KPH / 100.0, 0.0, 1.0))
      scale = (1.0 - reduce) + reduce * s
      tf_target *= scale

    # 고속 주행 시 추가 차간 거리 가중치 (TFollowSpeedFactor)
    # v_ego가 높을수록 차간 거리를 추가로 확보함
    if self.tFollowSpeedFactor > 0:
      speed_boost = float(np.clip((v_ego * CV.MS_TO_KPH - 60.0) / 100.0, 0.0, 1.0))
      tf_target += speed_boost * self.tFollowSpeedFactor

    return float(tf_target)


  def _apply_decel_hold_and_boost_t_follow(self, tf_target, a_ego):
    if not hasattr(self, "_tf_applied") or self._tf_applied <= 0.0:
      self._tf_applied = float(tf_target)

    DECEL_HOLD_A = -0.2  # m/s^2

    # 감속 중에는 t_follow 축소를 막음
    if a_ego <= DECEL_HOLD_A and tf_target < self._tf_applied:
      tf_held = float(self._tf_applied)
    else:
      tf_held = float(tf_target)

    # 감속 중에는 속도 감소로 실제 거리 여유가 줄 수 있으므로 약간 추가 확보
    # a_ego = -0.2 부근에서는 거의 0, 더 강한 감속일수록 boost 증가
    decel_boost = float(np.interp(a_ego, [-2.5, -1.0, -0.2, 0.0],
                                  [0.25, 0.12, 0.02, 0.0]))

    return float(tf_held + decel_boost * self.tFollowDecelBoost)


  def _clip_t_follow(self, t_follow):
    tf_min = float(min(self.tFollowGap1, self.tFollowGap2, self.tFollowGap3, self.tFollowGap4))
    tf_max = float(max(self.tFollowGap1, self.tFollowGap2, self.tFollowGap3, self.tFollowGap4))
    return float(np.clip(t_follow, max(0.3, tf_min), tf_max))

  def get_T_FOLLOW(self, personality=log.LongitudinalPersonality.standard, v_ego=0.0, a_ego=0.0):
    if self.launch_close_gap:
      personality = log.LongitudinalPersonality.aggressive  # 정차 후 출발: 최근접 Gap으로 catch-up
    tf_base = self._get_base_t_follow(personality, v_ego)
    tf_target = self._apply_speed_t_follow_scale(tf_base, v_ego)
    tf_adjusted = self._apply_decel_hold_and_boost_t_follow(tf_target, a_ego)
    tf_safe = float(tf_adjusted * self.mySafeFactor)
    tf_final = self._clip_t_follow(tf_safe)
    # 속도-가변 간격 보정 (clip 이후 적용 — clip 하한에 막히지 않게). 자체 안전 하한 유지.
    # 저속(≤30): t_follow 약간↓(간격 좁힘) / 고속(≥30): ↑(간격 넓힘) → time-gap 역전 정상화.
    v_kph = v_ego * CV.MS_TO_KPH
    tf_final = max(tf_final + float(np.interp(v_kph, _SPDTF_BP, _SPDTF_DELTA)), _SPDTF_MIN)
    self._tf_applied = float(tf_final)
    self._v_ego_kph = float(v_kph)   # dynamic_t_follow catch-up 속도 게이트용
    return self.apply_t_follow(tf_final)


  def _update_model_desire(self, sm):
    meta = sm['modelV2'].meta
    carState = sm['carState']

    if meta.laneChangeState == LaneChangeState.laneChangeStarting:
      self.desireState = meta.desireState[3] if carState.leftBlinker else meta.desireState[4]
      self.desireStateCount += 1
    else:
      self.desireState = 0.0
      self.desireStateCount = 0


  def dynamic_t_follow(self, t_follow, lead, desired_follow_distance, prev_a):
    self.jerk_factor_apply = self.jerk_factor

    # 차선변경 시작 후 1.5초 동안은 공격적으로
    if self.desireState > 0.9 and self.desireStateCount < int(1.5 / DT_MDL):
      dynamicTFollowLC = max(0.2, self.dynamicTFollowLC)
      t_follow *= dynamicTFollowLC
      self.jerk_factor_apply = self.jerk_factor * dynamicTFollowLC

    # 일반 lead follow:
    elif lead.status:
      if self.dynamicTFollow > 0.0:
        # lead.jLead 필터링을 통해 고주파 노이즈 제거
        self.filtered_j_lead = 0.9 * self.filtered_j_lead + 0.1 * lead.jLead
        # lead.jLead < 0 : 앞차가 감속 방향으로 변함 -> 차간거리 증가
        # lead.jLead > 0 : 앞차가 가속 방향으로 변함 -> 차간거리 감소
        t_follow += np.interp(self.filtered_j_lead, [-3.0, -0.5, 0.5, 2.0], [1.0, 0.0, 0.0, -1.0]) * self.dynamicTFollow
        t_follow = np.clip(t_follow, 0.3, 2.0)

      # 선행차가 '정속으로 멀어지는'(vRel>0, jLead~0) 경우엔 위 jLead 로직이 반응하지 않아
      # 중고속에서 재가속(catch-up)이 약했다(로그 0101: 40-70 +0.5, 70+ +0.2). 간격목표를
      # 속도비례로 살짝 좁혀 catch-up을 민첩하게. 저속(≤30, 이미 충분)·밀착(≤6m)엔 미적용.
      v_kph = getattr(self, "_v_ego_kph", 0.0)
      if lead.vRel > 0.5 and lead.dRel > 6.0:
        catchup = float(np.interp(v_kph, [30.0, 50.0, 90.0], [0.0, 0.15, 0.30]))
        catchup *= float(np.interp(lead.vRel, [0.5, 3.0], [0.0, 1.0]))
        t_follow = max(t_follow - catchup, 0.3)

      # Dynamic Jerk Control for early & gentle braking:
      # If lead deceleration is detected and we are not braking hard, increase jerk penalty (make it smoother/gentler).
      # Relax it back to normal jerk factor as distance error grows or ego deceleration increases.
      if lead.aLeadK < -0.5 or lead.jLead < -0.2:
        dist_err = desired_follow_distance - lead.dRel
        prev_a_scalar = float(prev_a[0]) if hasattr(prev_a, "__len__") else float(prev_a)
        scale_err = float(np.interp(dist_err, [0.0, 5.0], [1.8, 1.0]))
        scale_decel = float(np.interp(prev_a_scalar, [-1.5, -0.5], [1.0, 1.8]))
        jerk_scale = min(scale_err, scale_decel)
        self.jerk_factor_apply = self.jerk_factor * jerk_scale
      elif self.filtered_j_lead > 0.5 or lead.vRel > 1.0:
        # 선행차가 가속/정속으로 멀어짐: jerk penalty를 약간 낮춰 재가속을 민첩하게 (부분 복원).
        # jLead(가속 변화)뿐 아니라 vRel>1(정속 이탈)에도 적용 → 중고속 catch-up 보강.
        self.jerk_factor_apply = self.jerk_factor * 0.7

    return self.apply_t_follow(t_follow, 0.0)


  def apply_t_follow(self, t_follow, adjust_t_follow=0.0):
    # t_follow가 급격히 증가하면 목표거리도 급격히 증가하여 강한 감속을 유도할 수 있으므로
    # 증가 방향만 천천히 반영
    if t_follow > self.t_follow_last:
      t_follow = min(t_follow, self.t_follow_last + 0.1 * DT_MDL)
    elif t_follow < self.t_follow_last:
      # 감소(앞차 가속 등으로 간격 좁힐 때)도 즉시 스냅하지 않고 부드럽게 → 순간 가속 완화.
      t_follow = max(t_follow, self.t_follow_last - _TF_DECREASE_RATE * DT_MDL)

    self.t_follow_last = float(t_follow)
    return float(t_follow + adjust_t_follow)

  def update_stop_dist(self, stop_x):
    stop_x = self.xStopFilter.process(stop_x, median = True)
    stop_x = self.xStopFilter2.process(stop_x)
    return stop_x

  def check_model_stopping(self, v_cruise, v, v_ego, a_ego, model_x, y, d_rel):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    model_v = self.vFilter.process(v[-1])
    startSign = model_v > 5.0 or model_v > (v[0] + 2)

    if v_ego_kph < 1.0:
      stopSign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 82.0:
      stopSign = (model_x < d_rel - 3.0 and
                  model_x < np.interp(v[0] * 3.6, [60, 80], [120.0, 150]) and
                  ((model_v < 3.0) or (model_v < v[0] * 0.7)) and
                  abs(y[-1]) < 5.0)
      # 정상주행중 감속하는 경우(카메라 감속등), 오감지가 많음. 
      # 회생감속시:v_cruise=0에는 신호호감지하도록함.
      if v_cruise != 0 and (self.xState == XState.e2eCruise and a_ego < -1.0):
        stopSign = False
    else:
      stopSign = False

    # self.stopSignCount = (
    #   self.stopSignCount + 1
    #   if (
    #     stopSign
    #     and (
    #       model_x > get_safe_obstacle_distance(
    #         v_ego,
    #         t_follow=0,
    #         comfort_brake=COMFORT_BRAKE,
    #         stop_distance=-1.0,
    #       )
    #     )
    #   )
    #   else 0
    # )
    self.stopSignCount = self.stopSignCount + 1 if stopSign else 0
    self.startSignCount = self.startSignCount + 1 if startSign and not stopSign else 0

    if self.stopSignCount * DT_MDL > 0.0:
      self.trafficState = TrafficState.red
    elif self.startSignCount * DT_MDL > 0.2:
      self.trafficState = TrafficState.green
    else:
      self.trafficState = TrafficState.off

  def _update_carrot_man(self, sm, v_ego_kph, v_cruise_kph):
    atc_active = False
    if sm.alive['carrotMan']:
      carrot_man = sm['carrotMan']
      atc_turn_left = carrot_man.atcType in ["turn left", "atc left"]
      trigger_start = self.carrot_stay_stop = False
      if atc_turn_left or sm['carState'].leftBlinker:
        if self.trafficState_carrot == 1 and carrot_man.trafficState == 3: # red -> left triggered
          trigger_start = True
        elif carrot_man.trafficState in [1, 2]:
          self.carrot_stay_stop = True
      elif self.trafficState_carrot == 1 and carrot_man.trafficState == 2:  # red -> green triggered
        trigger_start = True
      else:
        trigger_start = False
      self.trafficState_carrot = carrot_man.trafficState

      if trigger_start:
        if self.soft_hold_active > 0:
          self.add_event(EventName.trafficSignChanged)
        elif self.xState in [XState.e2eStop, XState.e2eStopped]:
          self.xState = XState.e2eCruise
          self.traffic_starting_count = 10.0 / DT_MDL

      self.activeCarrot = carrot_man.activeCarrot
      self.xDistToTurn = carrot_man.xDistToTurn
      atc_active = self.activeCarrot > 1 and 0 < self.xDistToTurn < 100
      self.atcType = carrot_man.atcType

      v_cruise_kph = min(v_cruise_kph, carrot_man.desiredSpeed)

    return v_cruise_kph, atc_active

  def cruise_eco_control(self, v_ego_kph, v_cruise_kph):
    v_cruise_kph_apply = v_cruise_kph
    if self.eco_over_speed > 0:
      if self.eco_target_speed > 0:
        if self.eco_target_speed < v_cruise_kph:
          self.eco_target_speed = v_cruise_kph
        elif self.eco_target_speed > v_cruise_kph:
          self.eco_target_speed = 0
      elif self.eco_target_speed == 0 and v_ego_kph + 3 < v_cruise_kph and v_cruise_kph > 20.0:  # 주행중 속도가 떨어지면 다시 크루즈연비제어 시작.
        self.eco_target_speed = v_cruise_kph

      if self.eco_target_speed != 0:  ## 크루즈 연비 제어모드 작동중일때: 연비제어 종료지점
        if v_ego_kph > self.eco_target_speed: # 설정속도를 초과하면..
          self.eco_target_speed = 0
        else:
          v_cruise_kph_apply = self.eco_target_speed + self.eco_over_speed  # + 설정 속도로 설정함.
    else:
      self.eco_target_speed = 0

    return v_cruise_kph_apply

  def add_event(self, event_name):
    now = time.time()
    if now - self.last_event_time > 5.0:
      self.events.add(event_name)
      self.last_event_time = now

  def update(self, sm, v_cruise_kph, mode):
    self._params_update()
    self._update_model_desire(sm)

    self.events = Events()
    carstate = sm['carState']
    vCluRatio = carstate.vCluRatio
    #controlsState = sm['controlsState']
    radarstate = sm['radarState']
    model = sm['modelV2']

    #self.soft_hold_active = sm['carControl'].hudControl.softHoldActive # carrot 1
    self.soft_hold_active = sm['carState'].softHoldActive # carrot 2

    self.comfort_brake = self.comfortBrake

    v_ego = carstate.vEgo
    a_ego = carstate.aEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    v_ego_cluster = carstate.vEgoCluster
    v_ego_cluster_kph = v_ego_cluster * CV.MS_TO_KPH

    leadOne = radarstate.leadOne
    self.mySafeFactor = 1.0
    if self.myDrivingMode == DrivingMode.Eco: # eco
      self.mySafeFactor = self.myEcoModeFactor
    elif self.myDrivingMode == DrivingMode.Safe: #safe
      self.mySafeFactor = self.mySafeModeFactor


    self.drivingModeDetector.update_data(carstate, leadOne)

    v_cruise_kph = self.cruise_eco_control(v_ego_cluster_kph, v_cruise_kph)
    v_cruise_kph, atc_active = self._update_carrot_man(sm, v_ego_kph, v_cruise_kph)
    
    #if atc_active and not self.atc_active and self.xState not in [XState.e2eStop, XState.e2eStopped, XState.lead]:
    #  if self.atcType in ["turn left", "turn right", "atc left", "atc right"]:
    #    self.xState = XState.e2ePrepare
    self.atc_active = atc_active

    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if vCluRatio > 0.5:
      v_cruise *= vCluRatio

    x = model.position.x
    y = model.position.y
    v = model.velocity.x

    self.fakeCruiseDistance = 0.0
    lead_detected = radarstate.leadOne.status # & radarstate.leadOne.radar

    self.xStop = self.update_stop_dist(x[31])
    stop_model_x_raw = self.xStop
    if self._stop_x_rl is None:
      self._stop_x_rl = stop_model_x_raw
    else:
      max_close = v_ego * DT_MDL + 0.5
      if stop_model_x_raw > self._stop_x_rl:
        self._stop_x_rl = stop_model_x_raw
      else:
        self._stop_x_rl = max(self._stop_x_rl - max_close, stop_model_x_raw)

    stop_model_x = self._stop_x_rl
    stop_model_x_rl = self._stop_x_rl

    trafficState_last = self.trafficState
    #self.check_model_stopping(v, v_ego, self.xStop, y)
    self.check_model_stopping(v_cruise, v, v_ego, a_ego, x[-1], y, radarstate.leadOne.dRel if lead_detected else 1000)

    if self.myDrivingMode == DrivingMode.High or self.trafficLightDetectMode == 0:
      self.trafficState = TrafficState.off
    if abs(carstate.steeringAngleDeg) > 20:
      self.trafficState = TrafficState.off

    #self.update_user_control()
    if carstate.gasPressed or carstate.brakePressed:
      self.user_stop_distance = -1

    if self.soft_hold_active > 0:
      self.xState = XState.e2eStopped
      if trafficState_last in [TrafficState.off, TrafficState.red] and self.trafficState == TrafficState.green:
        self.add_event(EventName.trafficSignChanged)
    elif self.xState == XState.e2eStopped:
      if carstate.gasPressed:
        self.xState = XState.e2eCruise #XState.e2ePrepare
      elif lead_detected and (radarstate.leadOne.dRel - stop_model_x_raw) < 2.0:
        self.xState = XState.lead
      elif self.stopping_count == 0:
        if self.trafficState == TrafficState.green and not self.carrot_stay_stop and not carstate.leftBlinker and self.trafficLightDetectMode != 1:
          #self.xState = XState.e2ePrepare
          self.xState = XState.e2eCruise  # 실험모드를 거치지 않고 바로 출발.
          self.add_event(EventName.trafficSignGreen)
      self.stopping_count = max(0, self.stopping_count - 1)
      v_cruise = 0
    elif self.xState == XState.e2eStop:
      self.stopping_count = 0
      if carstate.gasPressed:  # Stop detecting traffic signal for 10 seconds
        #self.xState = XState.e2ePrepare
        self.xState = XState.e2eCruise
        self.traffic_starting_count = 10.0 / DT_MDL
      elif lead_detected and (radarstate.leadOne.dRel - stop_model_x_raw) < 2.0:
        self.xState = XState.lead
      else:
        if self.trafficState == TrafficState.green:
          self.add_event(EventName.trafficSignGreen)
          self.xState = XState.e2eCruise
        else:
          self.comfort_brake = self.comfortBrake * 0.9
          #self.comfort_brake = COMFORT_BRAKE
          self.trafficStopAdjustRatio = np.interp(v_ego_kph, [0, 100], [1.0, 0.7])
          stop_dist = stop_model_x_rl * np.interp(stop_model_x_rl, [0, 50], [1.0, self.trafficStopAdjustRatio])  ##�����Ÿ��� ���� �����Ÿ� ��������
          if stop_dist > 10.0: ### 10M�̻��϶���, self.actual_stop_distance�� ������Ʈ��.
            self.actual_stop_distance = stop_dist
          stop_model_x = 0
          self.fakeCruiseDistance = 0 if self.actual_stop_distance > 10.0 else 10.0
          if v_ego < 0.3:
            self.stopping_count = 0.5 / DT_MDL
            self.xState = XState.e2eStopped
    elif self.xState == XState.e2ePrepare:
      if lead_detected:
        self.xState = XState.lead
      elif self.atc_active:
        if carstate.gasPressed:
          self.xState = XState.e2eCruise
      elif v_ego_kph < 5.0 and self.trafficState != TrafficState.green:
        self.xState = XState.e2eStop
        self.actual_stop_distance = 5.0 #2.0
      elif v_ego_kph > 5.0: # and stop_model_x > 30.0:
        self.xState = XState.e2eCruise
    else: #XState.lead, XState.cruise, XState.e2eCruise
      self.traffic_starting_count = max(0, self.traffic_starting_count - 1)
      if lead_detected:
        self.xState = XState.lead
      elif self.trafficState == TrafficState.red and abs(carstate.steeringAngleDeg) < 30 and self.traffic_starting_count == 0:
        self.add_event(EventName.trafficStopping)
        self.xState = XState.e2eStop
        self.actual_stop_distance = stop_model_x_rl
      else:
        self.xState = XState.e2eCruise

    if self.trafficState in [TrafficState.off, TrafficState.green] or self.xState not in [XState.e2eStop, XState.e2eStopped]:
      stop_model_x = 1000.0

    if self.user_stop_distance >= 0:
      self.user_stop_distance = max(0, self.user_stop_distance - v_ego * DT_MDL)
      self.actual_stop_distance = self.user_stop_distance
      self.xState = XState.e2eStop if self.user_stop_distance > 0 else XState.e2eStopped

    if mode == 'acc':
      mode = 'blended' if self.xState in [XState.e2ePrepare] else 'acc'

    self.comfort_brake *= self.mySafeFactor
    # Low-Speed Comfort Brake Scaling
    v_ego_kph = v_ego * CV.MS_TO_KPH
    low_speed_factor = float(np.interp(v_ego_kph, [2.0, 10.0], [0.7, 1.0]))
    self.comfort_brake *= low_speed_factor
    self.actual_stop_distance = max(0, self.actual_stop_distance - (v_ego * DT_MDL))

    if stop_model_x == 1000.0: ##  e2eCruise, lead�ΰ��
      self.actual_stop_distance = 0.0
    elif self.actual_stop_distance > 0: ## e2eStop, e2eStopped�ΰ��..
      stop_model_x = 0.0

    stopping_active = self.xState not in [XState.e2eStop, XState.e2eStopped]
    if not stopping_active:
      self._stop_x_rl = stop_model_x_raw

    # self.debugLongText = (
    #   f"XState({str(self.xState)})," +
    #   f"stop_x={stop_x:.1f}," +
    #   f"stopDist={self.actual_stop_distance:.1f}," +
    #   f"Traffic={str(self.trafficState)}"
    # )
    #��ȣ�� �������� self.xState.value

    stop_dist =  stop_model_x + self.actual_stop_distance
    stop_dist = max(stop_dist, 0.0)

    stopping_active = (self.xState in [XState.e2eStop, XState.e2eStopped])
    if stopping_active and stop_dist < 300.0:
      stop_dist_soft = max(stop_dist - 1.0, 0.0)
      v_soft = float(np.sqrt(max(0.0, 2.0 * self.comfort_brake * stop_dist_soft)))
      v_cruise = min(v_cruise, v_soft)

    self.v_cruise = v_cruise
    self.stop_dist = stop_dist
    self.mode = mode
    #return v_cruise, stop_dist, mode

    # Auto-Tuner: 학습 데이터 수집
    from cereal import car
    gear_park = carstate.gearShifter == car.CarState.GearShifter.park
    engaged = sm.alive.get('selfdriveState', False) and sm['selfdriveState'].enabled

    # 정차 후 출발 catch-up: 완전정지(near-stop)에서 Gap1 무장 → 출발하며 선행차 민첩 추종,
    # 일정 속도 이상/선행차 없음/미인게이지 시 원래 Gap 복귀. (get_T_FOLLOW에서 소비)
    if not engaged or not lead_detected or v_ego_kph >= _LAUNCH_GAP_REVERT_KPH:
      self.launch_close_gap = False
    elif v_ego_kph < _LAUNCH_GAP_ARM_KPH:
      self.launch_close_gap = True

    # 현재 GAP 단계 파악 (Personality 기반)
    personality = sm['selfdriveState'].personality
    current_gap = 2  # default standard
    if personality == log.LongitudinalPersonality.moreRelaxed: current_gap = 4
    elif personality == log.LongitudinalPersonality.relaxed: current_gap = 3
    elif personality == log.LongitudinalPersonality.standard: current_gap = 2
    elif personality == log.LongitudinalPersonality.aggressive: current_gap = 1
    self.learner.set_current_gap(current_gap)

    # Auto-Tuner는 비핵심 학습 기능이므로, 여기서 예외가 나도 안전필수
    # 종방향 플래너(plannerd)가 죽지 않도록 반드시 격리한다.
    try:
      self.learner.update(v_ego_kph, carstate.gasPressed, engaged, gear_park,
                          steer_deg=carstate.steeringAngleDeg, steer_pressed=carstate.steeringPressed,
                          brake_pressed=carstate.brakePressed,
                          lead_drel=leadOne.dRel if leadOne.status else 0.0,
                          lead_v_kph=leadOne.vLead * CV.MS_TO_KPH if leadOne.status else 0.0,
                          a_ego=a_ego, lead_jlead=leadOne.jLead if leadOne.status else 0.0,
                          v_cruise_kph=v_cruise_kph,
                          gas_val=carstate.gas, brake_val=carstate.brake, sm=sm)

      # DSP: 수동 주행 성향 프로파일링 (오픈파일럿 미인게이지 상태에서만 수집)
      self.profiler.update(v_ego_kph, engaged, gear_park,
                           a_ego=a_ego, brake_pressed=carstate.brakePressed,
                           lead_drel=leadOne.dRel if leadOne.status else 0.0,
                           lead_v_kph=leadOne.vLead * CV.MS_TO_KPH if leadOne.status else 0.0)
    except Exception:
      cloudlog.exception("CarrotLearner/Profiler update failed")

    return v_cruise_kph

class DrivingModeDetector:
    def __init__(self):
        self.congested = False

        self.counter = 0
        self.enter_needed = 5
        self.exit_needed = 5

        self.distance_threshold = 12
        self.speed_threshold = 2
        self.accel_threshold = 1.5
        self.lead_speed_exit_threshold = 35

    def update_data(self, carstate, leadOne):
      my_speed = carstate.vEgo * CV.MS_TO_KPH
      my_accel = carstate.aEgo
      lead_speed = 0
      lead_accel = 0
      distance = 200
      if leadOne.status:
        lead_speed = leadOne.vLead * CV.MS_TO_KPH
        lead_accel = leadOne.aLead
        distance = leadOne.dRel

      # ---- 진입 조건(OR로 묶기) ----
      enter = (
          (distance <= self.distance_threshold and lead_speed <= self.speed_threshold) or
          (lead_speed < 5 and lead_accel < 0.2 and my_speed > 1.0 and distance < 200)
      )

      # ---- 탈출 조건(더 보수적으로) ----
      exit_ = (
          (lead_accel > self.accel_threshold) or
          (my_speed > self.lead_speed_exit_threshold) or
          (distance >= 200)
      )

      # ---- 디바운스 로직 ----
      if enter:
        self.counter += 1  
      elif exit_:
        self.counter -= 1

      if self.counter >= self.enter_needed:
        self.congested = True
        self.counter = self.enter_needed
      elif self.counter <= - self.exit_needed:
        self.congested = False
        self.counter = - self.exit_needed

    def get_mode(self):
        return DrivingMode.Safe if self.congested else DrivingMode.Normal
