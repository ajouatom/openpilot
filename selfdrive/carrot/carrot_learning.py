"""
CarrotLearning - Phase 1~4: Longitudinal + Lateral Learning
karpathy.md 원칙 준수: 최소 구현, 단일 목적.

[Phase 1] CruiseMaxVals0~6 (속도구간별 가속 강도)
  트리거: 크루즈 인게이지 중 gasPressed
  추천 발동: 구간당 누적 ≥ 10초

[Phase 2] SteerActuatorDelay / PathOffset / SteerRatioRate (조향 딜레이 / 중심선 편차 / SR 비율)
  트리거:
    - 직진 시 steeringAngleDeg 편차 → PathOffset 추천
    - 커브 진입 중 steeringPressed 비율 → SteerActuatorDelay 추천
    - 커브 진입 중 steeringPressed 비율 (고비율) → SteerRatioRate 추천 (SR 부족 시)
  추천 발동: 샘플 ≥ 200개 (약 20초 직진 데이터)

[Phase 3] JLeadFactor3 (제동 반응성)
  트리거: 선행차가 가깝고 느릴 때 brakePressed
  추천 발동: 수동 제동 횟수 ≥ 5회

[Phase 4] TFollowGap1~4 (차간 거리 설정)
  트리거: 크루즈 인게이지 중 선행차 있음 + gasPressed
         → 거리가 너무 넓어 운전자가 능동적으로 좁히려는 패턴
  추천: 현재 GAP 단계의 TFollowGap 값 감소 추천 (거리 좁히기)
  추천 발동: 인게이지 상태 선행차 추종 중 gas 개입 누적 ≥ 15초

저장: Params("CarrotLearningData") — JSON 문자열
팝업: gearShifter == park 시 CarrotLearningPopupReady = True
"""

import json
import math
import numpy as np
from openpilot.common.params import Params
from openpilot.common.conversions import Conversions as CV

# ── Phase 1 상수 ─────────────────────────────────────────────────────
_BP_KPH = [0, 10, 40, 60, 80, 110, 140]
_NUM_BANDS = len(_BP_KPH)
_ACCEL_KEYS = [f"CruiseMaxVals{i}" for i in range(_NUM_BANDS)]
_GAS_THRESHOLD_SEC = 10.0       # 구간당 누적 개입 시간 기준 (초)
_GAS_RECOMMEND_RATIO = 0.10     # 추천 증가 비율 (10%)
_GAS_REDUCE_RATIO = -0.07       # 추천 감소 비율 (-7%, 가속 과다 시)
_GAS_REDUCE_THRESHOLD_SEC = 5.0 # 가속 중 브레이크 개입 누적 기준 (초)

# ── Phase 2 상수 ─────────────────────────────────────────────────────
_STRAIGHT_DEG = 5.0             # 직진 판단 조향각 임계값 (도)
_CURVE_RATE_DEG_S = 10.0        # 커브 진입 판단: 조향각 변화율 임계값 (도/초)
_LATERAL_MIN_SAMPLES = 200      # PathOffset 추천을 위한 최소 직진 샘플 수
_LATERAL_MIN_CURVE = 20         # SteerActuatorDelay 추천을 위한 최소 커브 이벤트 수
_PATH_OFFSET_DEG_THRESHOLD = 1.5  # 평균 편차가 이 이상이면 PathOffset 추천 (도)
_PATH_OFFSET_DEG_PER_UNIT = 0.1   # 1 도 편차 ≈ 10 units PathOffset 변화 (실험값)
_CURVE_OVERRIDE_RATIO = 0.5     # 커브 진입의 50% 이상에서 override → SteerActuatorDelay 증가
_DELAY_STEP_UNIT = 10           # SteerActuatorDelay 한 번 추천 시 변화량 (UI 단위, +0.1s)
_SR_RATE_OVERRIDE_RATIO = 0.7   # 커브 진입의 70% 이상에서 override → SteerRatioRate 추가 추천
_SR_RATE_STEP_UNIT = 3          # SteerRatioRate 한 번 추천 시 변화량 (+3%)

# ── Phase 3 상수 ─────────────────────────────────────────────────────
_BRAKE_MIN_COUNT = 5            # 추천을 위한 최소 수동 브레이크 횟수
_JLEAD_STEP_UNIT = 20           # JLeadFactor3 한 번 추천 시 변화량 (강화: 10 -> 20)
_JLEAD_REDUCE_STEP = -7         # 제동 과다 시 변화량
_JLEAD_GAS_THRESHOLD_SEC = 5.0  # 제동 중 가속 개입 누적 기준 (초)

# ── Phase 5 상수 (DynamicTFollow / TFollowDecelBoost) ────────────────
# DynamicTFollow: 앞차 급감속(jLead↓) 중 브레이크 개입 횟수 기반
_DYN_TFOLLOW_BRAKE_MIN = 4      # 추천 발동 최소 이벤트 수
_DYN_JLEAD_THRESHOLD = -1.0     # 앞차 가감속도 (m/s^3) 이하이면 '급감속' 판정
_DYN_TFOLLOW_STEP = 5           # DynamicTFollow 한 번 추천 시 변화량 (+0.05)
_DYN_TFOLLOW_MAX = 100          # 최대값 (=1.0)
# TFollowDecelBoost: 내 차 감속 중 브레이크 개입 횟수 기반
_DECEL_BOOST_BRAKE_MIN = 4      # 추천 발동 최소 이벤트 수
_DECEL_A_THRESHOLD = -0.8       # 내 차 감속도 (m/s^2) 이하이면 '강한 감속' 판정
_DECEL_BOOST_STEP = 5           # TFollowDecelBoost 한 번 추천 시 변화량 (+0.05)
_DECEL_BOOST_MAX = 60           # 최대값

# ── Phase 4 상수 ─────────────────────────────────────────────────────
# TFollowGap1~4: Gap1(가장 좁음, 공격적), Gap4(가장 넓음, 여유)
# 각 Gap에 해당하는 Params key
_TFOLLOW_KEYS = ["TFollowGap1", "TFollowGap2", "TFollowGap3", "TFollowGap4"]
_TFOLLOW_NAMES = ["GAP1", "GAP2", "GAP3", "GAP4"]
_TFOLLOW_GAS_THRESHOLD_SEC = 15.0  # 선행차 추종 중 gas 누적 개입 시간 기준
_TFOLLOW_STEP_UNIT = -5            # 감소 추천 (-5 units = -0.05s)
_TFOLLOW_MIN_LEAD_DREL = 120.0     # 선행차 거리가 이보다 가까우면 '간격 좁히기' 패턴 아님
_TFOLLOW_MIN_V_KPH = 60.0          # 고속도로 주행 구간에서만 학습
_TFOLLOW_WIDEN_STEP = 5            # 증가 추천 (+0.05s)
_TFOLLOW_BRAKE_THRESHOLD_SEC = 10.0 # 거리 부족으로 인한 브레이크 누적 기준 (초)
_TFOLLOW_SPEED_FACTOR_STEP = 5     # 고속 보정치 증가 단위 (+0.05)
_AUTO_HUNTING_THRESHOLD = 0.8      # 자율 주행 중 가감속 변동(Hunting) 감지 임계치 (m/s^2)

# ── 공통 ─────────────────────────────────────────────────────────────
_DT = 0.1  # update() 호출 주기 (초)


def _speed_band(v_ego_kph: float) -> int:
  """속도에 해당하는 가장 가까운 하위 구간 인덱스 반환"""
  for i in range(_NUM_BANDS - 1, -1, -1):
    if v_ego_kph >= _BP_KPH[i]:
      return i
  return 0


class CarrotLearner:
  """
  CarrotPlanner.update()에서 매 프레임 호출됨.
  Phase 1: 가속 개입 데이터 누적 → CruiseMaxVals 추천
  Phase 2: 조향 편차/커브오버라이드 누적 → PathOffset / SteerActuatorDelay / SteerRatioRate 추천
  Phase 3: 수동 제동 누적 → JLeadFactor3 추천
  Phase 4: 선행차 추종 중 가속 개입 누적 → TFollowGap 감소 추천
  Parking 전환 시 추천값을 Params에 기록.
  """

  def __init__(self):
    self._params = Params()
    # Phase 1
    self._gas_acc = [0.0] * _NUM_BANDS
    self._gas_dec_acc = [0.0] * _NUM_BANDS  # 가속 중 수동 브레이크 개입
    self._gas_dec_auto_acc = [0.0] * _NUM_BANDS # 자율 주행 중 급가속 감지
    # Phase 2
    self._steer_acc = 0.0        # 직진 구간 조향각 누적합 (도)
    self._steer_count = 0        # 직진 샘플 수
    self._prev_steer_deg = 0.0   # 이전 프레임 조향각
    self._curve_entries = 0      # 커브 진입 이벤트 수
    self._curve_overrides = 0    # 커브 진입 중 steeringPressed 이벤트 수
    self._curve_overrides_understeer = 0     # 조향 부족 개입 횟수 (안쪽으로 더 꺾음)
    self._curve_overrides_inner_hugging = 0   # 안쪽 쏠림 개입 횟수 (바깥쪽으로 풀어줌)
    self._in_curve_entry = False # 커브 진입 상태 플래그 (중복 카운트 방지)
    # Phase 3
    self._brake_count = 0        # 수동 브레이크 개입 횟수
    self._brake_auto_count = 0   # 자율 주행 중 급제동 횟수
    self._jlead_gas_acc = 0.0    # 제동 중 수동 가속 개입
    self._prev_brake = False
    self._prev_auto_brake = False
    # Phase 4 (TFollowGap)
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4 # 수동 브레이크 개입
    self._tfollow_brake_auto_acc = [0.0] * 4 # 자율 주행 중 헌팅 감지
    self._tfollow_speed_brake_acc = 0.0      # 고속 주행 시 브레이크 개입 누적
    self._current_gap = 1  # 현재 활성화된 GAP 단계 (1~4)
    # Phase 5 (DynamicTFollow / TFollowDecelBoost)
    self._dyn_brake_count = 0    # 앞차 급감속 중 브레이크 개입 횟수
    self._decel_brake_count = 0  # 내 차 강한 감속 중 브레이크 개입 횟수

    # v3 Override Intensity & Dynamics Logging
    self._gas_max_accel = [0.0] * _NUM_BANDS
    self._gas_max_pedal = [0.0] * _NUM_BANDS
    self._brake_max_decel = 0.0
    self._brake_min_ttc = 999.0
    self._tfollow_min_gap = [999.0] * 4

    self._prev_gear_park = True  # 초기값(시동 시 P단 간주)
    self._has_driven = False     # 주행(D단/이동) 여부 플래그
    self._prev_a_ego = 0.0       # 이전 프레임 가속도
    self._accel_swing_count = 0  # 가감속 반전(Hunting) 카운트

    # ── 조향 방식 자동 감지 및 토크 학습 변수 ──────────────────────────
    self.is_angle_control = None
    self._torque_curve_entries = 0
    self._torque_curve_overrides = 0
    self._torque_curve_overrides_understeer = 0
    self._torque_curve_overrides_inner_hugging = 0
    self._torque_straight_entries = 0
    self._torque_straight_overrides = 0
    self._torque_error_count = 0


    # ── 주행 중 팝업 타이머 ─────────────────────────────────────────────
    # Trigger 1: 인게이지 30분 경과 → 즉시 팝업 (주행 중 포함)
    self._engaged_elapsed_sec = 0.0      # 인게이지 누적 시간
    self._popup_interval_sec = 1800.0    # 30분 = 1800초
    # Trigger 2: 5분 주기 추천 체크 → 정차 시 팝업
    self._check_elapsed_sec = 0.0        # 추천 체크 타이머
    self._check_interval_sec = 300.0     # 5분마다 체크
    self._pending_popup = False          # 정차 대기 팝업 플래그
    # 공통: 팝업 후 쿨다운 (중복 발동 방지)
    self._popup_cooldown_sec = 0.0       # 팝업 발동 후 5분 쿨다운

    # Phase 6 (Curve Decel Aggressiveness)
    self._curve_override_gas_sec = 0.0
    self._curve_override_brake_sec = 0.0
    self._curve_override_brake_count = 0
    self._curve_max_decel = 0.0
    self._curve_steer_error_sec = 0.0

    self._load()

  # ------------------------------------------------------------------
  # 공개 API
  # ------------------------------------------------------------------

  def _detect_steer_control_type(self):
    try:
      from cereal import car
      import cereal.messaging as messaging
      cp_bytes = self._params.get("CarParams")
      if cp_bytes is not None:
        cp = messaging.log_from_bytes(cp_bytes, car.CarParams)
        self.is_angle_control = (cp.steerControlType == car.CarParams.SteerControlType.angle)
        print(f"CarrotLearner: Detected steerControlType = {cp.steerControlType} (is_angle_control: {self.is_angle_control})")
      else:
        # CarParams가 아직 로드되지 않은 경우 다음 프레임에서 재시도하도록 None 유지
        pass
    except Exception as e:
      self.is_angle_control = False  # 에러 시 기본값 torque(False)로 롤백
      print(f"CarrotLearner: Error detecting steerControlType: {e}")

  def set_current_gap(self, gap: int):
    """현재 GAP 단계 설정 (1~4). CarrotPlanner에서 매 프레임 전달."""
    self._current_gap = max(1, min(4, gap))

  def update(self, v_ego_kph: float, gas_pressed: bool, engaged: bool, gear_park: bool,
             steer_deg: float = 0.0, steer_pressed: bool = False,
             brake_pressed: bool = False, lead_drel: float = 0.0, lead_v_kph: float = 0.0,
             a_ego: float = 0.0, lead_jlead: float = 0.0, v_cruise_kph: float = 0.0,
             gas_val: float = 0.0, brake_val: float = 0.0, sm=None):
    """
    매 프레임 호출.
    - Phase 1: engaged + gas_pressed → 속도구간 누적
    - Phase 2: engaged + 직진 → 조향 편차 누적
               engaged + 커브 진입 + steer_pressed → override 카운트
    - Phase 3: brake_pressed + 선행차 근접 → 브레이크 개입 카운트
    - Phase 4: engaged + 선행차 존재 + gas_pressed + 고속 → TFollowGap gas 누적
    - gear_park=True → 추천 계산 및 Params 저장
    """
    if not self._is_active():
      if self._params.get_bool("CarrotLearningPopupReady"):
        self._params.put_bool("CarrotLearningPopupReady", False)
      return

    # 조향 제어 방식 lazy loading 감지
    if self.is_angle_control is None:
      self._detect_steer_control_type()

    prev_brake = self._prev_brake


    # UI로부터 초기화(Clear) 신호가 오면 내부 메모리를 비움
    if self._params.get_bool("CarrotLearningClear"):
      self._clear_all_data()
      self._params.put_bool("CarrotLearningClear", False)

    # 주행 여부 판단 (오픈파일럿 인게이지가 단 한 번이라도 실행되었을 때만 유효 주행 세션으로 판단)
    if engaged:
      self._has_driven = True

    # ── Phase 1: 가속 개입 ────────────────────────────────────────────
    # 단순히 설정속도에 도달했는데 더 빨리 가고 싶어 밟는 경우는 제외 (설정속도 오버라이드)
    # 즉, 설정속도보다 충분히 낮은데도 가속이 답답할 때만 학습에 포함
    if engaged and gas_pressed and v_ego_kph >= 1.0:
      if v_ego_kph < (v_cruise_kph - 3.0):
        idx = _speed_band(v_ego_kph)
        self._gas_acc[idx] += _DT
        self._gas_max_accel[idx] = max(self._gas_max_accel[idx], a_ego)
        self._gas_max_pedal[idx] = max(self._gas_max_pedal[idx], gas_val)
    
    # 과속방지턱 감속 작동 중이거나 40km/h 미만 저속 구간 여부 판별 (A/B안 적용)
    is_speed_bump = False
    if sm is not None and sm.alive.get('carrotMan', False):
      try:
        carrot_man = sm['carrotMan']
        is_speed_bump = (carrot_man.xSpdType == 22 or carrot_man.activeCarrot == 5)
      except Exception:
        pass

    # 가속 과다 학습 방지: 가속 중인데 브레이크를 밟는 경우 OR 자율 주행 중 과도한 가속
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 가속 하향 패널티 수집 제외
    if engaged and v_ego_kph < (v_cruise_kph - 3.0) and not is_speed_bump and v_ego_kph >= 40.0:
      idx = _speed_band(v_ego_kph)
      if brake_pressed:
        if lead_drel == 0 or lead_drel > 120.0:
          self._gas_dec_acc[idx] += _DT
      elif not gas_pressed and a_ego > 1.5: # 자율 주행 중 급가속 감지
        self._gas_dec_auto_acc[idx] += _DT

    # ── Phase 2: 조향 패턴 (속도 20km/h 이상, 인게이지 상태) ──────────
    if engaged and v_ego_kph >= 20.0:
      steer_rate = abs(steer_deg - self._prev_steer_deg) / _DT

      desired_angle = 0.0
      steer_err = 0.0
      if sm is not None:
        try:
          if sm.alive.get('carControl', False):
            desired_angle = sm['carControl'].actuators.steeringAngleDeg
          elif sm.alive.get('controlsState', False):
            desired_angle = sm['controlsState'].steeringAngleDesired
          steer_err = desired_angle - steer_deg
        except Exception:
          pass

      if self.is_angle_control:
        # [A] 앵글 조향 차량 데이터 수집
        # 직진 구간 편차 수집 (override 없을 때만 순수 자동조향 편차)
        if abs(steer_deg) < _STRAIGHT_DEG and not steer_pressed:
          self._steer_acc += steer_deg
          self._steer_count += 1

        # 커브 진입 감지 (조향각 변화율이 임계값 초과)
        if steer_rate > _CURVE_RATE_DEG_S:
          if not self._in_curve_entry:
            self._curve_entries += 1
            if steer_pressed:
              self._curve_overrides += 1
              # 개입 방향 판정 (desired_angle * steer_err)
              # desired_angle과 steer_err의 부호가 같으면 Outer(풀어주기), 반대면 Inner(더 꺾기)
              if desired_angle * steer_err > 0:
                self._curve_overrides_inner_hugging += 1
              elif desired_angle * steer_err < 0:
                self._curve_overrides_understeer += 1
            self._in_curve_entry = True
        else:
          self._in_curve_entry = False
      else:
        # [B] 토크 조향 차량 데이터 수집
        # 1. 커브 구간 (조향각이 크고 속도가 있을 때)
        if v_ego_kph >= 40.0 and abs(steer_deg) >= 8.0:
          self._torque_curve_entries += 1
          if steer_pressed:
            self._torque_curve_overrides += 1
            # 개입 방향 판정
            if desired_angle * steer_err > 0:
              self._torque_curve_overrides_inner_hugging += 1
            elif desired_angle * steer_err < 0:
              self._torque_curve_overrides_understeer += 1
              
          if abs(steer_err) >= 1.5:
            self._torque_error_count += 1
        
        # 2. 완만한/직선 구간 (조향각이 작을 때 - Friction 용)
        elif v_ego_kph >= 30.0 and abs(steer_deg) < 4.0:
          self._torque_straight_entries += 1
          if steer_pressed:
            self._torque_straight_overrides += 1

    self._prev_steer_deg = steer_deg

    # Phase 3: 수동 제동 (선행차 근접 시) ────────────────────────
    is_auto_braking = False
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 제동 학습(JLeadFactor) 수집 제외
    if engaged and not gear_park and 0 < lead_drel < 100.0 and not is_speed_bump and v_ego_kph >= 40.0:
      # (1) 수동 제동 트리거
      if brake_pressed:
        if not self._prev_brake:
          self._brake_count += 1
        self._brake_max_decel = max(self._brake_max_decel, -a_ego)
        v_ego_ms = v_ego_kph / 3.6
        lead_v_ms = lead_v_kph / 3.6
        v_rel_ms = lead_v_ms - v_ego_ms
        if v_rel_ms < 0:
          ttc = lead_drel / -v_rel_ms
          self._brake_min_ttc = min(self._brake_min_ttc, ttc)
      # (2) 자율 주행 중 너무 늦게 급제동 발생 -> 제동 시점 앞당기기 필요
      elif not brake_pressed and a_ego < -1.7:
        is_auto_braking = True
        if not self._prev_auto_brake:
          self._brake_auto_count += 1
    
    self._prev_auto_brake = is_auto_braking
    
    # 제동 과다 학습 방지: 강한 제동 중 가속 페달을 밟는 경우 (불필요한 제동 억제)
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 가속 오버라이드 학습 수집 제외
    if engaged and gas_pressed and a_ego < -0.8 and not is_speed_bump and v_ego_kph >= 40.0:
      if 0 < lead_drel < 150.0:
        self._jlead_gas_acc += _DT

    # ── Phase 5: DynamicTFollow / TFollowDecelBoost ──────────────────
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 DynamicTFollow 학습 수집 제외
    if engaged and brake_pressed and not self._prev_brake and not is_speed_bump and v_ego_kph >= 40.0:
      # DynamicTFollow: 앞차 급감속 중 브레이크 개입
      if lead_jlead < _DYN_JLEAD_THRESHOLD and lead_drel < 150.0:
        self._dyn_brake_count += 1
      # TFollowDecelBoost: 내 차 강한 감속 중 브레이크 개입
      if a_ego < _DECEL_A_THRESHOLD:
        self._decel_brake_count += 1

    self._prev_brake = brake_pressed

    # ── Phase 4: TFollowGap (선행차 추종 중 가속 개입) ──────────────
    # 고속 크루즈 중 선행차가 충분히 멀리 있는데도 gas를 밟는다면
    # → 시스템이 설정된 거리보다 넓게 벌어져 있어 운전자가 좁히려는 것
    if (engaged and gas_pressed and v_ego_kph >= _TFOLLOW_MIN_V_KPH
        and lead_drel > _TFOLLOW_MIN_LEAD_DREL):
      gap_idx = self._current_gap - 1  # 0-indexed
      self._tfollow_gas_acc[gap_idx] += _DT
      v_ego_ms = v_ego_kph / 3.6
      if v_ego_ms > 1.0:
        time_gap = lead_drel / v_ego_ms
        self._tfollow_min_gap[gap_idx] = min(self._tfollow_min_gap[gap_idx], time_gap)

    # 거리 부족 학습 방지: 정속 추종 중 불안해서 브레이크를 밟는 경우 OR Hunting 감지
    if engaged and v_ego_kph >= 40.0 and 0 < lead_drel < 80.0:
      gap_idx = self._current_gap - 1
      if brake_pressed:
        if abs(v_ego_kph - lead_v_kph) < 5.0:
          self._tfollow_brake_acc[gap_idx] += _DT
      else:
        # 자율 주행 중 가감속 헌팅(Swing) 감지
        if (self._prev_a_ego > 0.3 and a_ego < -0.3) or (self._prev_a_ego < -0.3 and a_ego > 0.3):
          self._accel_swing_count += 1
          if self._accel_swing_count > 8:
            self._tfollow_brake_auto_acc[gap_idx] += _DT * 2.0
            self._accel_swing_count = 0
            
      # 고속 주행(80km/h 이상) 시 추가 거리 보정 학습
      if v_ego_kph >= 80.0 and brake_pressed:
        self._tfollow_speed_brake_acc += _DT

    # ── Phase 6: 가변 곡선 감속 학습 ──────────────────────────────────────────
    if engaged and sm is not None and sm.alive.get('modelV2', False) and v_ego_kph >= 20.0:
      modelData = sm['modelV2']
      if len(modelData.position.x) >= 3:
        x_pts = np.array(modelData.position.x)
        y_pts = np.array(modelData.position.y)
        n_points = len(x_pts)
        
        # Calculate maximum curvature along the predicted path
        max_c = 0.0
        for i in range(1, n_points - 1):
          x1, y1 = x_pts[i-1], y_pts[i-1]
          x2, y2 = x_pts[i], y_pts[i]
          x3, y3 = x_pts[i+1], y_pts[i+1]
          
          dx1, dy1 = x2 - x1, y2 - y1
          dx2, dy2 = x3 - x2, y3 - y2
          
          a_side = math.sqrt(dx1**2 + dy1**2)
          b_side = math.sqrt(dx2**2 + dy2**2)
          c_side = math.sqrt((x3 - x1)**2 + (y3 - y1)**2)
          
          cross = dx1 * dy2 - dy1 * dx2
          if a_side * b_side * c_side > 1e-6:
            curvature = (2.0 * abs(cross)) / (a_side * b_side * c_side)
            max_c = max(max_c, curvature)

        # Curve detected threshold: max_c > 0.0035 (radius < ~285m)
        is_curve = (max_c > 0.0035)
        no_lead = (lead_drel == 0.0 or lead_drel > 80.0) # no close lead car
        
        if is_curve and no_lead:
          if gas_pressed:
            self._curve_override_gas_sec += _DT
          elif brake_pressed:
            self._curve_override_brake_sec += _DT
            # Track peak deceleration in curve
            self._curve_max_decel = max(self._curve_max_decel, -a_ego)
            
            # Detect unique brake event in curve
            if not prev_brake:
              self._curve_override_brake_count += 1
          
          # Calculate steering tracking error (desired vs actual steering angle)
          try:
            desired_angle = 0.0
            if sm.alive.get('carControl', False):
              desired_angle = sm['carControl'].actuators.steeringAngleDeg
            elif sm.alive.get('controlsState', False):
              desired_angle = sm['controlsState'].steeringAngleDesired
            steer_err = desired_angle - steer_deg
            if abs(steer_err) >= 1.5:
              self._curve_steer_error_sec += _DT
          except Exception:
            pass

    # ── 주행 중 팝업 타이머 업데이트 ──────────────────────────────────
    if engaged and not gear_park:
      self._engaged_elapsed_sec += _DT
      self._check_elapsed_sec += _DT

    # 쿨다운 소모
    if self._popup_cooldown_sec > 0:
      self._popup_cooldown_sec -= _DT

    # [Trigger 2-체크] 5분마다 추천사항 확인 → pending 플래그 세팅
    if self._check_elapsed_sec >= self._check_interval_sec:
      self._check_elapsed_sec = 0.0
      if self._has_driven and self._popup_cooldown_sec <= 0:
        recs = self._calc_recommendations()
        if recs:
          self._pending_popup = True

    # [Trigger 2-발동] 정차 시 (v < 3 km/h) pending 팝업 표시
    if (self._pending_popup and not gear_park
        and self._has_driven and v_ego_kph < 3.0
        and self._popup_cooldown_sec <= 0):
      self._fire_popup(source="stop")
      self._pending_popup = False

    # [Trigger 1] 인게이지 30분 경과 → 즉시 팝업 (주행 중이라도)
    if (self._engaged_elapsed_sec >= self._popup_interval_sec
        and self._has_driven and self._popup_cooldown_sec <= 0):
      self._fire_popup(source="timer")
      self._engaged_elapsed_sec = 0.0
      self._pending_popup = False  # 30분 팝업이 발동하면 pending 취소

    # [Trigger 3] 주차 감지 (이전에 주차가 아니었고, 주행을 한 번이라도 한 경우에만 발동)
    if gear_park and not self._prev_gear_park and self._has_driven:
      self._on_parking()
      self._has_driven = False  # 팝업 후 플래그 초기화

    self._prev_gear_park = gear_park

  def _clear_all_data(self):
    """모든 누적 데이터를 0으로 초기화하고 DB에서도 삭제"""
    self._gas_acc = [0.0] * _NUM_BANDS
    self._gas_dec_acc = [0.0] * _NUM_BANDS
    self._gas_dec_auto_acc = [0.0] * _NUM_BANDS
    self._steer_acc = 0.0
    self._steer_count = 0
    self._curve_entries = 0
    self._curve_overrides = 0
    self._curve_overrides_understeer = 0
    self._curve_overrides_inner_hugging = 0
    self._torque_curve_entries = 0
    self._torque_curve_overrides = 0
    self._torque_curve_overrides_understeer = 0
    self._torque_curve_overrides_inner_hugging = 0
    self._torque_straight_entries = 0
    self._torque_straight_overrides = 0
    self._torque_error_count = 0
    self._brake_count = 0
    self._brake_auto_count = 0
    self._jlead_gas_acc = 0.0
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4
    self._tfollow_brake_auto_acc = [0.0] * 4
    self._tfollow_speed_brake_acc = 0.0
    self._dyn_brake_count = 0
    self._decel_brake_count = 0
    self._prev_brake = False
    self._prev_auto_brake = False

    # v3 Override Intensity & Dynamics variables reset
    self._gas_max_accel = [0.0] * _NUM_BANDS
    self._gas_max_pedal = [0.0] * _NUM_BANDS
    self._brake_max_decel = 0.0
    self._brake_min_ttc = 999.0
    self._tfollow_min_gap = [999.0] * 4

    # Phase 6 reset
    self._curve_override_gas_sec = 0.0
    self._curve_override_brake_sec = 0.0
    self._curve_override_brake_count = 0
    self._curve_max_decel = 0.0
    self._curve_steer_error_sec = 0.0

    self._params.remove("CarrotLearningData")
    self._params.remove("CarrotLearningRecommend")

  def is_active(self) -> bool:
    return self._is_active()

  # ------------------------------------------------------------------
  # 내부 메서드
  # ------------------------------------------------------------------

  def _is_active(self) -> bool:
    return self._params.get_int("CarrotLearningActive") == 1

  def _load(self):
    """이전 세션 누적 데이터 복원"""
    raw = self._params.get("CarrotLearningData")
    if not raw:
      return
    try:
      data = json.loads(raw)
      # Phase 1
      loaded = data.get("gas_acc", [0.0] * _NUM_BANDS)
      if len(loaded) == _NUM_BANDS:
        self._gas_acc = [float(x) for x in loaded]
      loaded_dec = data.get("gas_dec_acc", [0.0] * _NUM_BANDS)
      if len(loaded_dec) == _NUM_BANDS:
        self._gas_dec_acc = [float(x) for x in loaded_dec]
      loaded_auto = data.get("gas_dec_auto_acc", [0.0] * _NUM_BANDS)
      if len(loaded_auto) == _NUM_BANDS:
        self._gas_dec_auto_acc = [float(x) for x in loaded_auto]
      # Phase 2
      lat = data.get("lateral", {})
      self._steer_acc = float(lat.get("steer_acc", 0.0))
      self._steer_count = int(lat.get("steer_count", 0))
      self._curve_entries = int(lat.get("curve_entries", 0))
      self._curve_overrides = int(lat.get("curve_overrides", 0))
      self._curve_overrides_understeer = int(lat.get("curve_overrides_understeer", 0))
      self._curve_overrides_inner_hugging = int(lat.get("curve_overrides_inner_hugging", 0))
      self._torque_curve_entries = int(lat.get("torque_curve_entries", 0))
      self._torque_curve_overrides = int(lat.get("torque_curve_overrides", 0))
      self._torque_curve_overrides_understeer = int(lat.get("torque_curve_overrides_understeer", 0))
      self._torque_curve_overrides_inner_hugging = int(lat.get("torque_curve_overrides_inner_hugging", 0))
      self._torque_straight_entries = int(lat.get("torque_straight_entries", 0))
      self._torque_straight_overrides = int(lat.get("torque_straight_overrides", 0))
      self._torque_error_count = int(lat.get("torque_error_count", 0))
      # Phase 3
      lon = data.get("lon", {})
      self._brake_count = int(lon.get("brake_count", 0))
      self._jlead_gas_acc = float(lon.get("jlead_gas_acc", 0.0))
      # Phase 4
      loaded4 = data.get("tfollow_gas_acc", [0.0] * 4)
      if len(loaded4) == 4:
        self._tfollow_gas_acc = [float(x) for x in loaded4]
      loaded4_dec = data.get("tfollow_brake_acc", [0.0] * 4)
      if len(loaded4_dec) == 4:
        self._tfollow_brake_acc = [float(x) for x in loaded4_dec]
      loaded4_auto = data.get("tfollow_brake_auto_acc", [0.0] * 4)
      if len(loaded4_auto) == 4:
        self._tfollow_brake_auto_acc = [float(x) for x in loaded4_auto]
      self._tfollow_speed_brake_acc = float(data.get("tfollow_speed_brake_acc", 0.0))
      self._brake_auto_count = int(data.get("brake_auto_count", 0))
      # Phase 5
      p5 = data.get("phase5", {})
      self._dyn_brake_count = int(p5.get("dyn_brake_count", 0))
      self._decel_brake_count = int(p5.get("decel_brake_count", 0))
      # Phase 6
      p6 = data.get("phase6", {})
      self._curve_override_gas_sec = float(p6.get("curve_override_gas_sec", 0.0))
      self._curve_override_brake_sec = float(p6.get("curve_override_brake_sec", 0.0))
      self._curve_override_brake_count = int(p6.get("curve_override_brake_count", 0))
      self._curve_max_decel = float(p6.get("curve_max_decel", 0.0))
      self._curve_steer_error_sec = float(p6.get("curve_steer_error_sec", 0.0))

      # v3 Override Intensity & Dynamics Restore
      override = data.get("override_dynamics", {})
      loaded_gmax_a = override.get("gas_max_accel", [0.0] * _NUM_BANDS)
      if len(loaded_gmax_a) == _NUM_BANDS:
        self._gas_max_accel = [float(x) for x in loaded_gmax_a]
      loaded_gmax_p = override.get("gas_max_pedal", [0.0] * _NUM_BANDS)
      if len(loaded_gmax_p) == _NUM_BANDS:
        self._gas_max_pedal = [float(x) for x in loaded_gmax_p]
      self._brake_max_decel = float(override.get("brake_max_decel", 0.0))
      self._brake_min_ttc = float(override.get("brake_min_ttc", 999.0))
      loaded_tf_min = override.get("tfollow_min_gap", [999.0] * 4)
      if len(loaded_tf_min) == 4:
        self._tfollow_min_gap = [float(x) for x in loaded_tf_min]
    except Exception:
      pass  # 데이터 손상 시 기본값 유지

  def _save(self):
    """현재 누적 데이터를 Params에 저장"""
    data = {
      "gas_acc": self._gas_acc,
      "gas_dec_acc": self._gas_dec_acc,
      "lateral": {
        "steer_acc": self._steer_acc,
        "steer_count": self._steer_count,
        "curve_entries": self._curve_entries,
        "curve_overrides": self._curve_overrides,
        "curve_overrides_understeer": self._curve_overrides_understeer,
        "curve_overrides_inner_hugging": self._curve_overrides_inner_hugging,
        "torque_curve_entries": self._torque_curve_entries,
        "torque_curve_overrides": self._torque_curve_overrides,
        "torque_curve_overrides_understeer": self._torque_curve_overrides_understeer,
        "torque_curve_overrides_inner_hugging": self._torque_curve_overrides_inner_hugging,
        "torque_straight_entries": self._torque_straight_entries,
        "torque_straight_overrides": self._torque_straight_overrides,
        "torque_error_count": self._torque_error_count,
      },
      "lon": {
        "brake_count": self._brake_count,
        "jlead_gas_acc": self._jlead_gas_acc,
      },
      "tfollow_gas_acc": self._tfollow_gas_acc,
      "tfollow_brake_acc": self._tfollow_brake_acc,
      "tfollow_brake_auto_acc": self._tfollow_brake_auto_acc,
      "tfollow_speed_brake_acc": self._tfollow_speed_brake_acc,
      "gas_dec_auto_acc": self._gas_dec_auto_acc,
      "brake_auto_count": self._brake_auto_count,
      "phase5": {
        "dyn_brake_count": self._dyn_brake_count,
        "decel_brake_count": self._decel_brake_count,
      },
      "phase6": {
        "curve_override_gas_sec": self._curve_override_gas_sec,
        "curve_override_brake_sec": self._curve_override_brake_sec,
        "curve_override_brake_count": self._curve_override_brake_count,
        "curve_max_decel": self._curve_max_decel,
        "curve_steer_error_sec": self._curve_steer_error_sec,
      },
      "override_dynamics": {
        "gas_max_accel": self._gas_max_accel,
        "gas_max_pedal": self._gas_max_pedal,
        "brake_max_decel": self._brake_max_decel,
        "brake_min_ttc": self._brake_min_ttc,
        "tfollow_min_gap": self._tfollow_min_gap,
      },
    }
    self._params.put("CarrotLearningData", json.dumps(data).encode('utf8'))

  def _fire_popup(self, source: str = "parking"):
    """추천 계산 → Params 저장 → 팝업 신호.
    source: 'parking' | 'stop' | 'timer'
    """
    self._save()
    recommendations = self._calc_recommendations()
    if not recommendations:
      return
    self._params.put("CarrotLearningRecommend", json.dumps(recommendations).encode('utf8'))
    self._params.put("CarrotLearningPopupSource", source)
    self._params.put_bool("CarrotLearningPopupReady", True)
    self._popup_cooldown_sec = 300.0  # 5분 쿨다운 (중복 팝업 방지)

  def _on_parking(self):
    """주차 전환 시: _fire_popup 호출"""
    self._fire_popup(source="parking")
    self._engaged_elapsed_sec = 0.0
    self._pending_popup = False

  def _calc_recommendations(self) -> dict:
    """Phase 1~4 추천값 계산. 추천 없으면 빈 dict 반환."""
    result = {
      "가속 (Acceleration)": {},
      "조향 (Steering)": {},
      "주행 (Driving)": {},
      "거리 (Following Distance)": {},
      "동적제어 (Dynamic Control)": {},
    }

    # ── Phase 1: CruiseMaxVals ──────────────────────────────────────
    drive_mode = self._params.get_int("MyDrivingMode") # 1: ECO, 2: SAFE, 3: NORMAL, 4: HIGH
    for i, acc_sec in enumerate(self._gas_acc):
      key = _ACCEL_KEYS[i]
      current_raw = self._params.get_int(key)
      if current_raw <= 0: continue

      # 드라이브 모드별 동적 가속 제한 상한값 설정
      max_limit = 250
      if key == "CruiseMaxVals1":
        if drive_mode in (1, 2):    # ECO, SAFE
          max_limit = 180
        elif drive_mode == 3:       # NORMAL
          max_limit = 200
        elif drive_mode == 4:       # HIGH
          max_limit = 250
      elif key == "CruiseMaxVals2":
        if drive_mode in (1, 2):    # ECO, SAFE
          max_limit = 150
        elif drive_mode in (3, 4):  # NORMAL, HIGH
          max_limit = 160
      elif key == "CruiseMaxVals3":
        if drive_mode in (1, 2):    # ECO, SAFE
          max_limit = 110
        elif drive_mode in (3, 4):  # NORMAL, HIGH
          max_limit = 120

      total_dec = self._gas_dec_acc[i] + self._gas_dec_auto_acc[i]
      
      # [상호 억제 로직: Accel Penalty Discount]
      # 주행 중 브레이크 개입(수동+자동)이 많았다면, 가속이 굼떴던 느낌(gas help)보다
      # 차량이 거칠어 브레이크를 밟은 상황(instability)이 우선이므로 가속 누적 신호를 깎아줍니다.
      total_brake_events = self._brake_count + self._brake_auto_count
      dampened_acc_sec = max(0.0, acc_sec - (total_brake_events * 2.5)) # 브레이크 1회당 가속 누적 2.5초 감쇄

      # 자율 가감속 요동(Auto-Surging) 방지:
      # 운전자가 페달을 밟지 않았어도 자율 급가속과 자율 급감속이 동시에 잦은 경우, 가속 한계치를 최우선적으로 낮춥니다.
      is_auto_surging = (self._gas_dec_auto_acc[i] >= 3.0 and self._brake_auto_count >= 3)

      # 오버라이드 중 기록된 피크 가속도 (m/s^2)
      max_accel = self._gas_max_accel[i]
      current_accel_limit = current_raw / 100.0
      accel_deficit = max_accel - current_accel_limit

      recommended_raw = current_raw
      reason = ""
      sec = 0.0

      if current_raw > max_limit:
        # 현재 설정값이 동적 상한선보다 큰 경우 강제 상한 제한으로 하향 조치 트리거
        recommended_raw = max_limit
        reason = f"exceeds drive-mode limit ({max_limit})"
        sec = 0.0
      elif dampened_acc_sec >= _GAS_THRESHOLD_SEC and not is_auto_surging:
        # 피크 가속도 부족분에 비례하는 가변 증가율 적용 (최소 5%, 최대 25%)
        if accel_deficit > 0.05:
          dynamic_ratio = float(np.clip(accel_deficit / current_accel_limit * 0.8, 0.05, 0.25))
        else:
          dynamic_ratio = _GAS_RECOMMEND_RATIO # 기본 10%
        recommended_raw = min(max_limit, int(current_raw * (1.0 + dynamic_ratio)))
        reason = f"gas help (deficit {accel_deficit:.2f}m/s^2, ratio {dynamic_ratio*100:.1f}%)"
        sec = dampened_acc_sec
      elif total_dec >= _GAS_REDUCE_THRESHOLD_SEC or (total_brake_events >= 8 and current_raw > 100) or is_auto_surging:
        recommended_raw = max(50, int(current_raw * (1.0 + _GAS_REDUCE_RATIO)))
        recommended_raw = min(max_limit, recommended_raw)
        if is_auto_surging:
          reason = "excessive auto-surging penalty"
          sec = self._gas_dec_auto_acc[i] + self._brake_auto_count
        else:
          reason = "excessive braking penalty" if total_brake_events >= 8 else ("aggressive accel (auto)" if self._gas_dec_auto_acc[i] > self._gas_dec_acc[i] else "too aggressive (manual)")
          sec = max(total_dec, total_brake_events)
      else:
        continue

      if recommended_raw != current_raw:
        result["가속 (Acceleration)"][key] = {
          "current": current_raw,
          "recommended": recommended_raw,
          "band_kph": f"{_BP_KPH[i]}~{_BP_KPH[i+1] if i+1 < _NUM_BANDS else '∞'} km/h ({reason})",
          "acc_sec": round(sec, 1),
        }

    # ── Phase 2: 조향 패턴 추천 ─────────────────────────────────────
    if self.is_angle_control:
      # ── [A] 앵글 조향 차량 튜닝 ────────────────────────────────────
      # Phase 2a: PathOffset (직진 편차)
      if self._steer_count >= _LATERAL_MIN_SAMPLES:
        avg_deg = self._steer_acc / self._steer_count
        if abs(avg_deg) >= _PATH_OFFSET_DEG_THRESHOLD:
          current_offset = self._params.get_int("PathOffset")
          # 양수 avg_deg: 차가 우측으로 쏠림 → PathOffset 증가 (경로를 우측으로)
          delta = int(avg_deg / _PATH_OFFSET_DEG_PER_UNIT)
          recommended = int(np.clip(current_offset + delta, -150, 150))
          if recommended != current_offset:
            result["조향 (Steering)"]["PathOffset"] = {
              "current": current_offset,
              "recommended": recommended,
              "band_kph": "직진 주행 편차 보정",
              "avg_deg": round(avg_deg, 2),
            }

      # Phase 2b: SteerActuatorDelay & SteerRatioRate
      if self._curve_entries >= _LATERAL_MIN_CURVE:
        override_ratio = self._curve_overrides / self._curve_entries
        
        # 개입 방향 비율 산출
        understeer_ratio = 0.0
        inner_hugging_ratio = 0.0
        if self._curve_overrides > 0:
          understeer_ratio = self._curve_overrides_understeer / self._curve_overrides
          inner_hugging_ratio = self._curve_overrides_inner_hugging / self._curve_overrides

        # (1) SteerActuatorDelay
        current_delay = self._params.get_int("SteerActuatorDelay")
        recommended_delay = current_delay
        if override_ratio >= 0.30:
          if understeer_ratio >= 0.60:
            recommended_delay = min(300, current_delay + _DELAY_STEP_UNIT)
          elif inner_hugging_ratio >= 0.60:
            recommended_delay = max(50, current_delay - _DELAY_STEP_UNIT)
        elif override_ratio < 0.10: # 개입이 거의 없는 안정 상태 → 지연시간 소폭 감소로 최적점 수렴
          recommended_delay = max(50, current_delay - 10)
          
        if recommended_delay != current_delay:
          result["조향 (Steering)"]["SteerActuatorDelay"] = {
            "current": current_delay,
            "recommended": recommended_delay,
            "band_kph": "커브 진입 지연 보정 (조향 지연 상향)" if (override_ratio >= 0.30 and understeer_ratio >= 0.60) else ("커브 진입 안쪽 쏠림 보정 (조향 지연 하향)" if (override_ratio >= 0.30 and inner_hugging_ratio >= 0.60) else "커브 진입 안정화 감쇄"),
            "override_ratio": round(override_ratio * 100, 1),
          }

        # (2) SteerRatioRate
        current_sr_rate = self._params.get_int("SteerRatioRate")
        if current_sr_rate <= 0:
          current_sr_rate = 100  # 기본값 100%
        recommended_sr = current_sr_rate
        
        if override_ratio >= 0.40:
          if understeer_ratio >= 0.60:
            recommended_sr = min(150, current_sr_rate + _SR_RATE_STEP_UNIT)
          elif inner_hugging_ratio >= 0.60:
            recommended_sr = max(90, current_sr_rate - _SR_RATE_STEP_UNIT)
        elif override_ratio < 0.15: # 개입이 적은 경우 → 조향비 소폭 감소하여 더 완만하고 부드럽게
          recommended_sr = max(90, current_sr_rate - 2)
          
        if recommended_sr != current_sr_rate:
          result["조향 (Steering)"]["SteerRatioRate"] = {
            "current": current_sr_rate,
            "recommended": recommended_sr,
            "band_kph": "커브 강한 개입 대응 (조향 강화)" if (override_ratio >= 0.40 and understeer_ratio >= 0.60) else ("커브 안쪽 쏠림 개입 대응 (조향 감쇄)" if (override_ratio >= 0.40 and inner_hugging_ratio >= 0.60) else "부드러운 조향 감속 보정"),
            "override_ratio": round(override_ratio * 100, 1),
          }
    
    else:
      # ── [B] 토크 조향 차량 튜닝 ────────────────────────────────────
      # 1. SteerActuatorDelay
      if self._torque_curve_entries >= 100: # 최소 샘플 수 (약 10초)
        override_ratio = self._torque_curve_overrides / self._torque_curve_entries
        
        understeer_ratio = 0.0
        inner_hugging_ratio = 0.0
        if self._torque_curve_overrides > 0:
          understeer_ratio = self._torque_curve_overrides_understeer / self._torque_curve_overrides
          inner_hugging_ratio = self._torque_curve_overrides_inner_hugging / self._torque_curve_overrides

        current_delay = self._params.get_int("SteerActuatorDelay")
        recommended_delay = current_delay
        if override_ratio >= 0.30:
          if understeer_ratio >= 0.60:
            recommended_delay = min(300, current_delay + 10)
          elif inner_hugging_ratio >= 0.60:
            recommended_delay = max(50, current_delay - 10)
        elif override_ratio < 0.10:
          recommended_delay = max(50, current_delay - 10)
          
        if recommended_delay != current_delay:
          result["조향 (Steering)"]["SteerActuatorDelay"] = {
            "current": current_delay,
            "recommended": recommended_delay,
            "band_kph": "토크 커브 진입 지연 보정" if (override_ratio >= 0.30 and understeer_ratio >= 0.60) else ("토크 커브 안쪽 쏠림 보정" if (override_ratio >= 0.30 and inner_hugging_ratio >= 0.60) else "토크 커브 안정화 감쇄"),
            "override_ratio": round(override_ratio * 100, 1),
          }

        # 2. LateralTorqueAccelFactor & LateralTorqueKf (횡가속도 비례 피드포워드)
        current_factor = self._params.get_int("LateralTorqueAccelFactor")
        current_kf = self._params.get_int("LateralTorqueKf")
        
        recommended_factor = current_factor
        recommended_kf = current_kf
        
        if override_ratio >= 0.40: # 개입 비중이 높음 → 피드포워드 상향
          recommended_factor = min(4000, current_factor + 100)
          recommended_kf = min(200, current_kf + 3)
        elif override_ratio < 0.15: # 개입이 거의 없음 → 피드포워드 소폭 하향 수렴
          recommended_factor = max(1500, current_factor - 50)
          recommended_kf = max(50, current_kf - 1)
          
        if recommended_factor != current_factor:
          result["조향 (Steering)"]["LateralTorqueAccelFactor"] = {
            "current": current_factor,
            "recommended": recommended_factor,
            "band_kph": "토크 커브 선향력 상향" if override_ratio >= 0.40 else "토크 커브 선향력 안정화 감쇄",
            "override_ratio": round(override_ratio * 100, 1),
          }
        if recommended_kf != current_kf:
          result["조향 (Steering)"]["LateralTorqueKf"] = {
            "current": current_kf,
            "recommended": recommended_kf,
            "band_kph": "토크 커브 피드포워드 강화" if override_ratio >= 0.40 else "토크 피드포워드 안정화 감쇄",
            "override_ratio": round(override_ratio * 100, 1),
          }

      # 3. LateralTorqueFriction (직선 미세 지연 보정)
      if self._torque_straight_entries >= 200: # 직선/완만 구간 약 20초
        straight_override_ratio = self._torque_straight_overrides / self._torque_straight_entries
        current_friction = self._params.get_int("LateralTorqueFriction")
        recommended_friction = current_friction
        
        if straight_override_ratio >= 0.35: # 미세 보정 구간 개입 높음 → 마찰보상 상향
          recommended_friction = min(300, current_friction + 5)
        elif straight_override_ratio < 0.08: # 개입 없음 → 마찰보상 소폭 하향 수렴
          recommended_friction = max(10, current_friction - 2)
          
        if recommended_friction != current_friction:
          result["조향 (Steering)"]["LateralTorqueFriction"] = {
            "current": current_friction,
            "recommended": recommended_friction,
            "band_kph": "직선 미세 불감대 해소" if straight_override_ratio >= 0.35 else "미세 조향 마찰보상 최적화",
            "override_ratio": round(straight_override_ratio * 100, 1),
          }

      # 4. LateralTorqueKiV / LateralTorqueKpV (조향 누적 오차 피드백 보정)
      if self._torque_error_count >= 50: # 오차가 일정 횟수 이상 누적된 경우 (약 5초 누적)
        current_kiv = self._params.get_int("LateralTorqueKiV")
        current_kpv = self._params.get_int("LateralTorqueKpV")
        
        recommended_kiv = min(100, current_kiv + 1)
        recommended_kpv = min(300, current_kpv + 5)
        
        if recommended_kiv != current_kiv:
          result["조향 (Steering)"]["LateralTorqueKiV"] = {
            "current": current_kiv,
            "recommended": recommended_kiv,
            "band_kph": "조향 정상오차 쏠림 제어 (피드백)",
            "error_ticks": self._torque_error_count,
          }
        if recommended_kpv != current_kpv:
          result["조향 (Steering)"]["LateralTorqueKpV"] = {
            "current": current_kpv,
            "recommended": recommended_kpv,
            "band_kph": "조향 오차 복원 속도 상향 (피드백)",
            "error_ticks": self._torque_error_count,
          }

    # ── Phase 3: JLeadFactor3 (수동 제동) ───────────────────────────
    jlead_candidate = None
    total_brake = self._brake_count + self._brake_auto_count
    if total_brake >= _BRAKE_MIN_COUNT:
      current_jlead = self._params.get_int("JLeadFactor3")
      
      # TTC와 감속량을 반영한 동적 증가 계산
      ttc_factor = float(np.clip((4.5 - self._brake_min_ttc) / 2.0, 0.0, 1.0))
      decel_factor = float(np.clip((self._brake_max_decel - 0.8) / 1.0, 0.0, 1.0))
      dynamic_step = int(10 + 25 * max(ttc_factor, decel_factor))
      
      recommended = min(100, current_jlead + dynamic_step)
      if recommended != current_jlead:
        reason = "late braking (auto)" if self._brake_auto_count > self._brake_count else "approaching lead (manual)"
        jlead_candidate = {
          "current": current_jlead,
          "recommended": recommended,
          "band_kph": f"{reason} (TTC min {self._brake_min_ttc:.1f}s, step +{dynamic_step})",
          "brake_count": round(total_brake, 1),
          "_signal": total_brake,
        }
    elif self._jlead_gas_acc >= _JLEAD_GAS_THRESHOLD_SEC:
      current_jlead = self._params.get_int("JLeadFactor3")
      recommended = max(50, current_jlead + _JLEAD_REDUCE_STEP)
      if recommended != current_jlead:
        jlead_candidate = {
          "current": current_jlead,
          "recommended": recommended,
          "band_kph": "too aggressive (gas override)",
          "gas_sec": round(self._jlead_gas_acc, 1),
          "_signal": self._jlead_gas_acc,
        }

    # ── Phase 4: TFollowGap (선행차 추종 중 거리 좁히기 가속 개입) ──
    for i, gas_sec in enumerate(self._tfollow_gas_acc):
      key = _TFOLLOW_KEYS[i]
      name = _TFOLLOW_NAMES[i]
      current_val = self._params.get_int(key)
      if current_val <= 0: continue

      total_dec = self._tfollow_brake_acc[i] + self._tfollow_brake_auto_acc[i]
      
      recommended_val = current_val
      if gas_sec >= _TFOLLOW_GAS_THRESHOLD_SEC:
        # 실제 차간 거리 오차에 기반한 동적 감소 계산
        target_val = int(self._tfollow_min_gap[i] * 100)
        gap_diff = current_val - target_val
        if gap_diff > 10:
          dynamic_step = float(np.clip(int(gap_diff * 0.5), 5, 25))
        else:
          dynamic_step = 5
        recommended_val = max(70, current_val - int(dynamic_step))
        reason = f"too wide (gap diff {gap_diff*0.01:.2f}s, step -{int(dynamic_step)})"
        sec = gas_sec
      elif total_dec >= _TFOLLOW_BRAKE_THRESHOLD_SEC:
        # 제동 급박도에 비례한 차간 거리 동적 증가
        dynamic_step = int(5 + float(np.clip((4.0 - self._brake_min_ttc) * 5, 0, 10)))
        recommended_val = min(200, current_val + dynamic_step)
        reason = "hunting detected (auto)" if self._tfollow_brake_auto_acc[i] > self._tfollow_brake_acc[i] else f"too short (step +{dynamic_step})"
        sec = total_dec
      else:
        continue

      if recommended_val != current_val:
        result["거리 (Following Distance)"][key] = {
          "current": current_val,
          "recommended": recommended_val,
          "band_kph": f"highway ≥{_TFOLLOW_MIN_V_KPH:.0f}km/h ({name}, {reason})",
          "sec": round(sec, 1),
        }

    # ── Phase 6: TFollowSpeedFactor (고속 차간 거리 보정) ───────────
    if self._tfollow_speed_brake_acc >= _TFOLLOW_BRAKE_THRESHOLD_SEC:
      current_sf = self._params.get_int("TFollowSpeedFactor")
      recommended_sf = min(100, current_sf + _TFOLLOW_SPEED_FACTOR_STEP)
      if recommended_sf != current_sf:
        result["거리 (Following Distance)"]["TFollowSpeedFactor"] = {
          "current": current_sf,
          "recommended": recommended_sf,
          "band_kph": "high-speed safety (>80km/h)",
          "sec": round(self._tfollow_speed_brake_acc, 1),
        }

    # ── Phase 5a: DynamicTFollow (앞차 급감속 반응 민감도) ───────────
    dyn_candidate = None
    if self._dyn_brake_count >= _DYN_TFOLLOW_BRAKE_MIN:
      current_dyn = self._params.get_int("DynamicTFollow")
      recommended_dyn = min(_DYN_TFOLLOW_MAX, current_dyn + _DYN_TFOLLOW_STEP)
      if recommended_dyn != current_dyn:
        dyn_candidate = {
          "current": current_dyn,
          "recommended": recommended_dyn,
          "band_kph": "lead decel override",
          "brake_count": self._dyn_brake_count,
          "_signal": self._dyn_brake_count,
        }

    # ── Phase 5b: TFollowDecelBoost (내 차 감속 중 버퍼 확보) ────────
    boost_candidate = None
    if self._decel_brake_count >= _DECEL_BOOST_BRAKE_MIN:
      current_boost = self._params.get_int("TFollowDecelBoost")
      recommended_boost = min(_DECEL_BOOST_MAX, current_boost + _DECEL_BOOST_STEP)
      if recommended_boost != current_boost:
        boost_candidate = {
          "current": current_boost,
          "recommended": recommended_boost,
          "band_kph": "decel braking",
          "brake_count": self._decel_brake_count,
          "_signal": self._decel_brake_count,
        }

    # ── 브레이크 파라미터 충돌 방지: 동시 다중 적용 시 과보정 억제 ──
    # JLeadFactor3 / DynamicTFollow / TFollowDecelBoost 는 모두
    # '제동 여유 확대' 방향이므로 동시 적용 시 복합 효과로 과보수화 위험.
    # → 가장 강한 시그널 1개만 이번 세션에 추천하고, 나머지는 '다음 세션' 권고.
    brake_candidates = [
      ("JLeadFactor3",      "주행 (Driving)",         jlead_candidate),
      ("DynamicTFollow",    "동적제어 (Dynamic Control)", dyn_candidate),
      ("TFollowDecelBoost", "동적제어 (Dynamic Control)", boost_candidate),
    ]
    active = [(name, group, c) for name, group, c in brake_candidates if c is not None]

    if len(active) <= 1:
      # 충돌 없음: 그냥 모두 추가
      for name, group, c in active:
        entry = {k: v for k, v in c.items() if k != "_signal"}
        result[group][name] = entry
    else:
      # 2개 이상 동시 발동 → 시그널이 가장 강한 것 1개만 추천
      active_sorted = sorted(active, key=lambda x: x[2]["_signal"], reverse=True)
      winner_name, winner_group, winner_c = active_sorted[0]
      entry = {k: v for k, v in winner_c.items() if k != "_signal"}
      # 다음 세션에 재평가하도록 안내 메시지 추가
      deferred_names = [n for n, g, c in active_sorted[1:]]
      entry["band_kph"] = entry["band_kph"] + f" ※다음세션권고:{','.join(deferred_names)}"
      result[winner_group][winner_name] = entry

    # ── Phase 6: Curve Speed Aggressiveness ─────────────────────────
    key = "AutoCurveSpeedAggressiveness"
    current_raw = self._params.get_int(key)
    if current_raw <= 0:
      current_raw = 100

    recommended_raw = current_raw
    reason = ""
    sec = 0.0

    # Brake/steer overrides (Safety critical - takes priority)
    if self._curve_override_brake_count >= 3 or self._curve_override_brake_sec >= 5.0 or self._curve_steer_error_sec >= 3.0:
      recommended_raw = max(60, current_raw - 10)
      if self._curve_steer_error_sec >= 3.0:
        reason = f"steering tracking error (accumulated {self._curve_steer_error_sec:.1f}s)"
        sec = self._curve_steer_error_sec
      else:
        reason = f"brake overrides (count {self._curve_override_brake_count}, peak decel {self._curve_max_decel:.2f}m/s^2)"
        sec = self._curve_override_brake_sec
    elif self._curve_override_gas_sec >= 10.0:
      recommended_raw = min(130, current_raw + 10)
      reason = f"gas overrides (acc {self._curve_override_gas_sec:.1f}s)"
      sec = self._curve_override_gas_sec

    if recommended_raw != current_raw:
      if "곡선 (Curve)" not in result:
        result["곡선 (Curve)"] = {}
      result["곡선 (Curve)"][key] = {
        "current": current_raw,
        "recommended": recommended_raw,
        "reason": reason,
        "band_kph": "curve deceleration",
        "sec": sec,
      }

    return {k: v for k, v in result.items() if v}

  def apply_recommendations(self):
    """UI [적용] 버튼 클릭 시 호출. 추천 적용 + 데이터 초기화."""
    raw = self._params.get("CarrotLearningRecommend")
    if not raw:
      return
    try:
      recommendations = json.loads(raw)
    except Exception:
      return
    for group in recommendations:
      for key in recommendations[group]:
        info = recommendations[group][key]
        self._params.put_int(key, info["recommended"])

    # 데이터 초기화
    self._gas_acc = [0.0] * _NUM_BANDS
    self._gas_dec_acc = [0.0] * _NUM_BANDS
    self._gas_dec_auto_acc = [0.0] * _NUM_BANDS
    self._steer_acc = 0.0
    self._steer_count = 0
    self._curve_entries = 0
    self._curve_overrides = 0
    self._brake_count = 0
    self._brake_auto_count = 0
    self._jlead_gas_acc = 0.0
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4
    self._tfollow_brake_auto_acc = [0.0] * 4
    self._tfollow_speed_brake_acc = 0.0
    self._dyn_brake_count = 0
    self._decel_brake_count = 0
    self._prev_brake = False
    self._prev_auto_brake = False

    # v3 Override Intensity & Dynamics variables reset
    self._gas_max_accel = [0.0] * _NUM_BANDS
    self._gas_max_pedal = [0.0] * _NUM_BANDS
    self._brake_max_decel = 0.0
    self._brake_min_ttc = 999.0
    self._tfollow_min_gap = [999.0] * 4

    # Phase 6 reset
    self._curve_override_gas_sec = 0.0
    self._curve_override_brake_sec = 0.0
    self._curve_override_brake_count = 0
    self._curve_max_decel = 0.0
    self._curve_steer_error_sec = 0.0

    # 주행 중 팝업 타이머 리셋 (적용 후 재학습 시작)
    self._engaged_elapsed_sec = 0.0
    self._check_elapsed_sec = 0.0
    self._pending_popup = False
    self._popup_cooldown_sec = 0.0

    self._params.remove("CarrotLearningData")
    self._params.remove("CarrotLearningRecommend")
    self._params.remove("CarrotLearningPopupSource")
    self._params.put_bool("CarrotLearningPopupReady", False)


# ══════════════════════════════════════════════════════════════════════
# Driving Style Profiler (DSP)
# 수동 주행 데이터를 분석하여 오픈파일럿 종방향 파라미터 초기값을 추천.
# CarrotLearner(engaged 전용)와 상호 독립적으로 작동.
# ══════════════════════════════════════════════════════════════════════

# DSP 상수
_DSP_MIN_ACCEL_SAMPLES = 50       # 가속 프로파일 최소 샘플 수 (~5초)
_DSP_MIN_FOLLOW_SAMPLES = 30     # 차간 거리 최소 안정 추종 샘플 수 (~3초)
_DSP_MIN_BRAKE_EVENTS = 5        # 제동 시점 최소 이벤트 수
_DSP_MIN_DRIVE_TIME_SEC = 600.0  # 최소 수동 주행 시간 (10분)
_DSP_SAFETY_TF_MIN = 0.70        # TFollowGap 안전 하한선 (초)
_DSP_SAFETY_TF_MAX = 2.50        # TFollowGap 안전 상한선 (초)
_DSP_SAFETY_ACCEL_MAX = 250      # CruiseMaxVals 안전 상한 (2.50 m/s²)
_DSP_SAFETY_ACCEL_MIN = 50       # CruiseMaxVals 안전 하한 (0.50 m/s²)
_DSP_JLEAD_SAFETY_MULTIPLIER = 1.1  # 시스템 지연 보정 계수 (인간 0.2초 vs 시스템 0.5초)


class DrivingStyleProfiler:
  """수동 주행 성향 프로파일러 (Driving Style Profiler).

  오픈파일럿 미인게이지 상태(수동 운전)에서의 가속/제동/차간거리
  패턴을 수집하여 오픈파일럿 종방향 파라미터 초기값을 추천합니다.

  CarrotLearner와 역할 분담:
    - DSP: 초기값 설정 (Personalization, engaged=False)
    - CarrotLearner: 지속적 미세 조정 (Error Correction, engaged=True)
  """

  def __init__(self):
    self._params = Params()
    self._active = False  # DSP 활성화 여부 (프로파일링 미완료 시 활성)

    # ── 가속 프로파일 (속도 대역별) ──
    self._accel_samples = [0] * _NUM_BANDS        # 샘플 수
    self._accel_sum = [0.0] * _NUM_BANDS          # aEgo 합산
    self._accel_max = [0.0] * _NUM_BANDS          # 대역별 최대 가속도

    # ── 차간 거리 프로파일 (안정 추종 구간) ──
    self._follow_samples = 0                      # 안정 추종 샘플 수
    self._follow_time_sum = 0.0                    # 시간거리(dRel/vEgo) 합산
    self._follow_stable_streak = 0                 # 연속 안정 추종 프레임 수
    self._follow_speed_time_pairs = []             # (speed_kph, time_gap) 쌍 저장 (최대 500)

    # ── 제동 프로파일 ──
    self._brake_events = 0                         # 수동 제동 이벤트 수
    self._brake_drel_sum = 0.0                     # 제동 시점 dRel 합산
    self._brake_vrel_sum = 0.0                     # 제동 시점 vRel 합산

    # ── 세션 관리 ──
    self._manual_drive_time = 0.0                  # 수동 주행 누적 시간
    self._prev_brake = False
    self._has_manual_driven = False                # 수동 주행이 한 번이라도 있었는지

    self._load()

  def _is_active(self) -> bool:
    """DSP가 아직 프로파일링이 완료되지 않았을 때만 활성화."""
    return not self._params.get_bool("CarrotDSPComplete")

  def _load(self):
    """저장된 DSP 데이터 복원."""
    raw = self._params.get("CarrotDSPData")
    if not raw:
      return
    try:
      data = json.loads(raw)
      self._accel_samples = [int(x) for x in data.get("accel_samples", [0]*_NUM_BANDS)]
      self._accel_sum = [float(x) for x in data.get("accel_sum", [0.0]*_NUM_BANDS)]
      self._accel_max = [float(x) for x in data.get("accel_max", [0.0]*_NUM_BANDS)]
      self._follow_samples = int(data.get("follow_samples", 0))
      self._follow_time_sum = float(data.get("follow_time_sum", 0.0))
      self._follow_speed_time_pairs = data.get("follow_speed_time_pairs", [])
      self._brake_events = int(data.get("brake_events", 0))
      self._brake_drel_sum = float(data.get("brake_drel_sum", 0.0))
      self._brake_vrel_sum = float(data.get("brake_vrel_sum", 0.0))
      self._manual_drive_time = float(data.get("manual_drive_time", 0.0))
    except Exception:
      pass

  def _save(self):
    """DSP 데이터를 Params에 저장."""
    data = {
      "accel_samples": self._accel_samples,
      "accel_sum": self._accel_sum,
      "accel_max": self._accel_max,
      "follow_samples": self._follow_samples,
      "follow_time_sum": self._follow_time_sum,
      "follow_speed_time_pairs": self._follow_speed_time_pairs[-500:],  # 최대 500개 유지
      "brake_events": self._brake_events,
      "brake_drel_sum": self._brake_drel_sum,
      "brake_vrel_sum": self._brake_vrel_sum,
      "manual_drive_time": self._manual_drive_time,
    }
    self._params.put("CarrotDSPData", json.dumps(data).encode('utf8'))

  def update(self, v_ego_kph: float, engaged: bool, gear_park: bool,
             a_ego: float = 0.0, brake_pressed: bool = False,
             lead_drel: float = 0.0, lead_v_kph: float = 0.0):
    """매 프레임 호출 (carrot_functions.py에서).

    engaged=False(수동 주행) 상태에서만 데이터를 수집합니다.
    gear_park=True 시 추천을 계산합니다.
    """
    if not self._is_active():
      return

    # 주차 전환 시: 저장 → 추천 계산
    if gear_park:
      if self._has_manual_driven:
        self._save()
        self._calc_and_publish_recommendations()
        self._has_manual_driven = False
      return

    # ── 수동 주행 데이터만 수집 (오픈파일럿 미인게이지) ──
    if engaged:
      return  # 인게이지 상태에서는 CarrotLearner가 담당

    if v_ego_kph < 3.0:
      self._prev_brake = brake_pressed
      return  # 극저속은 의미 없음

    self._has_manual_driven = True
    self._manual_drive_time += 0.1  # ~DT_MDL (100ms)

    # ── (A) 가속 프로파일 수집 ──
    # 가속 페달을 밟고 있을 때(aEgo > 0.3)의 가속도를 속도 대역별로 기록
    if a_ego > 0.3:
      idx = _speed_band(v_ego_kph)
      self._accel_samples[idx] += 1
      self._accel_sum[idx] += a_ego
      self._accel_max[idx] = max(self._accel_max[idx], a_ego)

    # ── (B) 차간 거리 프로파일 수집 ──
    # 선행차가 존재하고, 안정적 추종 중(가/감속 없이 일정 거리 유지)일 때
    v_ego_ms = v_ego_kph / 3.6
    if lead_drel > 5.0 and v_ego_ms > 3.0 and abs(a_ego) < 0.5:
      time_gap = lead_drel / v_ego_ms
      if 0.5 < time_gap < 4.0:  # 비정상적 값 필터링
        self._follow_stable_streak += 1
        if self._follow_stable_streak >= 10:  # 1초 이상 안정 추종
          self._follow_samples += 1
          self._follow_time_sum += time_gap
          if len(self._follow_speed_time_pairs) < 500:
            self._follow_speed_time_pairs.append([round(v_ego_kph, 1), round(time_gap, 3)])
      else:
        self._follow_stable_streak = 0
    else:
      self._follow_stable_streak = 0

    # ── (C) 제동 프로파일 수집 ──
    # 선행차가 가까울 때 브레이크를 밟는 시점의 거리와 상대속도를 기록
    if brake_pressed and not self._prev_brake:
      if lead_drel > 0 and lead_drel < 100.0:
        self._brake_events += 1
        self._brake_drel_sum += lead_drel
        v_rel = v_ego_kph - lead_v_kph
        self._brake_vrel_sum += max(0, v_rel)

    self._prev_brake = brake_pressed

  def _calc_and_publish_recommendations(self):
    """수집된 수동 주행 데이터를 분석하여 초기 파라미터 추천을 생성."""
    if self._manual_drive_time < _DSP_MIN_DRIVE_TIME_SEC:
      return  # 최소 주행 시간 미달

    result = {}

    # ── (A) CruiseMaxVals 초기값 추천 ──
    accel_recs = {}
    for i in range(_NUM_BANDS):
      if self._accel_samples[i] < _DSP_MIN_ACCEL_SAMPLES:
        continue
      avg_accel = self._accel_sum[i] / self._accel_samples[i]
      max_accel = self._accel_max[i]
      # 운전자의 평균 가속도와 최대 가속도를 모두 고려하여 크루즈 최고 가속도 한계를 설정
      # OP의 최고 한계이므로 너무 답답하지 않게 평균값보다 훨씬 높게 설정합니다.
      derived_accel = max(avg_accel * 1.5, max_accel * 0.85)
      recommended_raw = int(np.clip(derived_accel * 100, _DSP_SAFETY_ACCEL_MIN, _DSP_SAFETY_ACCEL_MAX))
      current_raw = self._params.get_int(_ACCEL_KEYS[i])
      if current_raw <= 0:
        continue
      # 현재 값과 차이가 5% 이상일 때만 추천
      if abs(recommended_raw - current_raw) >= current_raw * 0.05:
        accel_recs[_ACCEL_KEYS[i]] = {
          "current": current_raw,
          "recommended": recommended_raw,
          "band_kph": f"{_BP_KPH[i]}~{_BP_KPH[i+1] if i+1 < _NUM_BANDS else '∞'} km/h",
          "avg_accel": round(avg_accel, 2),
          "max_accel": round(self._accel_max[i], 2),
          "samples": self._accel_samples[i],
        }
    if accel_recs:
      result["🚀 가속 초기값 (Accel Profile)"] = accel_recs

    # ── (B) TFollowGap 초기값 추천 ──
    follow_recs = {}
    if self._follow_samples >= _DSP_MIN_FOLLOW_SAMPLES:
      avg_time_gap = self._follow_time_sum / self._follow_samples
      # 기계가 유지하는 간격은 사람이 인지하는 것보다 멀게 느껴지므로 0.9를 곱해 보정
      adjusted_time_gap = avg_time_gap * 0.9
      # 안전 하한선 적용
      safe_time_gap = max(_DSP_SAFETY_TF_MIN, min(_DSP_SAFETY_TF_MAX, adjusted_time_gap))
      recommended_raw = int(safe_time_gap * 100)

      # 현재 GAP2 (Standard) 기준으로 비교
      current_gap2 = self._params.get_int("TFollowGap2")
      if current_gap2 > 0 and abs(recommended_raw - current_gap2) >= 5:
        follow_recs["TFollowGap2"] = {
          "current": current_gap2,
          "recommended": recommended_raw,
          "band_kph": "Standard GAP (수동 주행 평균 차간 시간)",
          "avg_time_gap": round(avg_time_gap, 2),
          "samples": self._follow_samples,
        }

      # 나머지 GAP 레벨도 비례 조정
      # GAP1 = 추천값 * 0.85, GAP3 = 추천값 * 1.12, GAP4 = 추천값 * 1.23
      gap_ratios = {"TFollowGap1": 0.85, "TFollowGap3": 1.12, "TFollowGap4": 1.23}
      for key, ratio in gap_ratios.items():
        derived = int(np.clip(safe_time_gap * ratio * 100, _DSP_SAFETY_TF_MIN * 100, _DSP_SAFETY_TF_MAX * 100))
        current = self._params.get_int(key)
        if current > 0 and abs(derived - current) >= 5:
          follow_recs[key] = {
            "current": current,
            "recommended": derived,
            "band_kph": f"{key} (비례 조정, ratio={ratio})",
            "avg_time_gap": round(safe_time_gap * ratio, 2),
            "samples": self._follow_samples,
          }

    if follow_recs:
      result["🛣️ 차간거리 초기값 (Follow Profile)"] = follow_recs

    # ── (C) JLeadFactor3 초기값 추천 ──
    if self._brake_events >= _DSP_BRAKE_EVENTS_MIN:
      avg_brake_drel = self._brake_drel_sum / self._brake_events
      # 운전자의 평균 제동 시작 거리 → JLeadFactor3 매핑
      # 거리가 먼 운전자 = 조기 제동 선호 = 높은 JLeadFactor3
      # 거리가 짧은 운전자 = 늦은 제동 선호 = 낮은 JLeadFactor3
      # 선형 매핑: dRel 15m → 50, dRel 60m → 120
      raw_factor = int(np.interp(avg_brake_drel, [15, 60], [50, 120]))
      # 시스템 지연 보정 적용 (1.1배)
      recommended_jlead = int(np.clip(raw_factor * _DSP_JLEAD_SAFETY_MULTIPLIER, 50, 200))
      current_jlead = self._params.get_int("JLeadFactor3")
      if current_jlead > 0 and abs(recommended_jlead - current_jlead) >= 10:
        result["🚙 제동 시점 초기값 (Brake Profile)"] = {
          "JLeadFactor3": {
            "current": current_jlead,
            "recommended": recommended_jlead,
            "band_kph": f"평균 제동 시작 거리: {avg_brake_drel:.1f}m (시스템 지연 보정 x{_DSP_JLEAD_SAFETY_MULTIPLIER})",
            "avg_brake_drel": round(avg_brake_drel, 1),
            "brake_events": self._brake_events,
          }
        }

    if not result:
      return  # 추천할 항목 없음

    # 추천 결과 저장 및 팝업 트리거
    self._params.put("CarrotDSPRecommend", json.dumps(result).encode('utf8'))
    self._params.put_bool("CarrotDSPPopupReady", True)

  def apply_recommendations(self):
    """UI [적용] 버튼 클릭 시 호출. 추천 적용 + 프로파일링 완료 마킹."""
    raw = self._params.get("CarrotDSPRecommend")
    if not raw:
      return
    try:
      recommendations = json.loads(raw)
    except Exception:
      return
    for group in recommendations:
      for key in recommendations[group]:
        info = recommendations[group][key]
        self._params.put_int(key, info["recommended"])

    # 프로파일링 완료 마킹 → 이후 DSP는 비활성화, CarrotLearner가 미세 조정
    self._params.put_bool("CarrotDSPComplete", True)
    self._params.remove("CarrotDSPData")
    self._params.remove("CarrotDSPRecommend")
    self._params.put_bool("CarrotDSPPopupReady", False)

  def get_profile_progress(self) -> dict:
    """현재 프로파일링 진행 상황을 반환 (UI 표시용)."""
    total_min = self._manual_drive_time / 60.0
    accel_ready = sum(1 for s in self._accel_samples if s >= _DSP_MIN_ACCEL_SAMPLES)
    return {
      "drive_time_min": round(total_min, 1),
      "accel_bands_ready": f"{accel_ready}/{_NUM_BANDS}",
      "follow_samples": self._follow_samples,
      "brake_events": self._brake_events,
      "is_ready": self._manual_drive_time >= _DSP_MIN_DRIVE_TIME_SEC,
    }


# _DSP_BRAKE_EVENTS_MIN: 별도 상수 정의 (상단의 Phase3 상수와 공유 불가)
_DSP_BRAKE_EVENTS_MIN = 5
