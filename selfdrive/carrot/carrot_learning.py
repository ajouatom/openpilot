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
_JLEAD_STEP_UNIT = 10           # JLeadFactor3 한 번 추천 시 변화량
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
    self._in_curve_entry = False # 커브 진입 상태 플래그 (중복 카운트 방지)
    # Phase 3
    self._brake_count = 0        # 수동 브레이크 개입 횟수
    self._brake_auto_count = 0   # 자율 주행 중 급제동 횟수
    self._jlead_gas_acc = 0.0    # 제동 중 수동 가속 개입
    self._prev_brake = False
    # Phase 4 (TFollowGap)
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4 # 수동 브레이크 개입
    self._tfollow_brake_auto_acc = [0.0] * 4 # 자율 주행 중 헌팅 감지
    self._current_gap = 1  # 현재 활성화된 GAP 단계 (1~4)
    # Phase 5 (DynamicTFollow / TFollowDecelBoost)
    self._dyn_brake_count = 0    # 앞차 급감속 중 브레이크 개입 횟수
    self._decel_brake_count = 0  # 내 차 강한 감속 중 브레이크 개입 횟수

    self._prev_gear_park = True  # 초기값(시동 시 P단 간주)
    self._has_driven = False     # 주행(D단/이동) 여부 플래그
    self._prev_a_ego = 0.0       # 이전 프레임 가속도
    self._accel_swing_count = 0  # 가감속 반전(Hunting) 카운트

    self._load()

  # ------------------------------------------------------------------
  # 공개 API
  # ------------------------------------------------------------------

  def set_current_gap(self, gap: int):
    """현재 GAP 단계 설정 (1~4). CarrotPlanner에서 매 프레임 전달."""
    self._current_gap = max(1, min(4, gap))

  def update(self, v_ego_kph: float, gas_pressed: bool, engaged: bool, gear_park: bool,
             steer_deg: float = 0.0, steer_pressed: bool = False,
             brake_pressed: bool = False, lead_drel: float = 0.0, lead_v_kph: float = 0.0,
             a_ego: float = 0.0, lead_jlead: float = 0.0, v_cruise_kph: float = 0.0):
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
      return

    # UI로부터 초기화(Clear) 신호가 오면 내부 메모리를 비움
    if self._params.get_bool("CarrotLearningClear"):
      self._clear_all_data()
      self._params.put_bool("CarrotLearningClear", False)

    # 주행 여부 판단 (인게이지 되거나 속도가 5km/h 이상이면 주행한 것으로 간주)
    if engaged or v_ego_kph >= 5.0:
      self._has_driven = True

    # ── Phase 1: 가속 개입 ────────────────────────────────────────────
    # 단순히 설정속도에 도달했는데 더 빨리 가고 싶어 밟는 경우는 제외 (설정속도 오버라이드)
    # 즉, 설정속도보다 충분히 낮은데도 가속이 답답할 때만 학습에 포함
    if engaged and gas_pressed and v_ego_kph >= 1.0:
      if v_ego_kph < (v_cruise_kph - 3.0):
        self._gas_acc[_speed_band(v_ego_kph)] += _DT
    
    # 가속 과다 학습 방지: 가속 중인데 브레이크를 밟는 경우 OR 자율 주행 중 과도한 가속
    if engaged and v_ego_kph < (v_cruise_kph - 3.0):
      idx = _speed_band(v_ego_kph)
      if brake_pressed:
        if lead_drel == 0 or lead_drel > 120.0:
          self._gas_dec_acc[idx] += _DT
      elif not gas_pressed and a_ego > 1.5: # 자율 주행 중 급가속 감지
        self._gas_dec_acc[idx] += _DT * 0.5

    # ── Phase 2: 조향 패턴 (속도 20km/h 이상, 인게이지 상태) ──────────
    if engaged and v_ego_kph >= 20.0:
      steer_rate = abs(steer_deg - self._prev_steer_deg) / _DT

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
          self._in_curve_entry = True
      else:
        self._in_curve_entry = False

    self._prev_steer_deg = steer_deg

    # Phase 3: 수동 제동 (선행차 근접 시) ────────────────────────
    if engaged and not gear_park and 0 < lead_drel < 100.0:
      # (1) 수동 제동 트리거
      if brake_pressed and not self._prev_brake:
        self._brake_count += 1
      # (2) 자율 주행 중 너무 늦게 급제동 발생 -> 제동 시점 앞당기기 필요
      elif not brake_pressed and a_ego < -1.7:
        self._brake_count += 0.3
    
    # 제동 과다 학습 방지: 강한 제동 중 가속 페달을 밟는 경우 (불필요한 제동 억제)
    if engaged and gas_pressed and a_ego < -0.8:
      if 0 < lead_drel < 150.0:
        self._jlead_gas_acc += _DT

    # ── Phase 5: DynamicTFollow / TFollowDecelBoost ──────────────────
    if brake_pressed and not self._prev_brake:
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
            self._tfollow_brake_acc[gap_idx] += _DT * 0.2
            self._accel_swing_count = 0

    # 주차 감지 (이전에 주차가 아니었고, 주행을 한 번이라도 한 경우에만 발동)
    if gear_park and not self._prev_gear_park and self._has_driven:
      self._on_parking()
      self._has_driven = False  # 팝업 후 플래그 초기화

    self._prev_gear_park = gear_park

  def _clear_all_data(self):
    """모든 누적 데이터를 0으로 초기화하고 DB에서도 삭제"""
    self._gas_acc = [0.0] * _NUM_BANDS
    self._steer_acc = 0.0
    self._steer_count = 0
    self._curve_entries = 0
    self._curve_overrides = 0
    self._brake_count = 0
    self._tfollow_gas_acc = [0.0] * 4
    self._dyn_brake_count = 0
    self._decel_brake_count = 0
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
      # Phase 5
      p5 = data.get("phase5", {})
      self._dyn_brake_count = int(p5.get("dyn_brake_count", 0))
      self._decel_brake_count = int(p5.get("decel_brake_count", 0))
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
      },
      "lon": {
        "brake_count": self._brake_count,
        "jlead_gas_acc": self._jlead_gas_acc,
      },
      "tfollow_gas_acc": self._tfollow_gas_acc,
      "tfollow_brake_acc": self._tfollow_brake_acc,
      "tfollow_brake_auto_acc": self._tfollow_brake_auto_acc,
      "gas_dec_auto_acc": self._gas_dec_auto_acc,
      "phase5": {
        "dyn_brake_count": self._dyn_brake_count,
        "decel_brake_count": self._decel_brake_count,
      },
    }
    self._params.put("CarrotLearningData", json.dumps(data).encode('utf8'))

  def _on_parking(self):
    """주차 전환 시: 저장 → 추천 계산 → 팝업 신호"""
    self._save()
    recommendations = self._calc_recommendations()
    if not recommendations:
      return
    self._params.put("CarrotLearningRecommend", json.dumps(recommendations).encode('utf8'))
    self._params.put_bool("CarrotLearningPopupReady", True)

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
    for i, acc_sec in enumerate(self._gas_acc):
      key = _ACCEL_KEYS[i]
      current_raw = self._params.get_int(key)
      if current_raw <= 0: continue

      total_dec = self._gas_dec_acc[i] + self._gas_dec_auto_acc[i]
      if acc_sec >= _GAS_THRESHOLD_SEC:
        recommended_raw = min(250, int(current_raw * (1.0 + _GAS_RECOMMEND_RATIO)))
        reason = "gas help (manual)"
        sec = acc_sec
      elif total_dec >= _GAS_REDUCE_THRESHOLD_SEC:
        recommended_raw = max(50, int(current_raw * (1.0 + _GAS_REDUCE_RATIO)))
        reason = "aggressive accel (auto)" if self._gas_dec_auto_acc[i] > self._gas_dec_acc[i] else "too aggressive (manual)"
        sec = total_dec
      else:
        continue

      if recommended_raw != current_raw:
        result["가속 (Acceleration)"][key] = {
          "current": current_raw,
          "recommended": recommended_raw,
          "band_kph": f"{_BP_KPH[i]}~{_BP_KPH[i+1] if i+1 < _NUM_BANDS else '∞'} km/h ({reason})",
          "acc_sec": round(sec, 1),
        }

    # ── Phase 2a: PathOffset (직진 편차) ────────────────────────────
    if self._steer_count >= _LATERAL_MIN_SAMPLES:
      avg_deg = self._steer_acc / self._steer_count
      if abs(avg_deg) >= _PATH_OFFSET_DEG_THRESHOLD:
        current_offset = self._params.get_int("PathOffset")
        # 양수 avg_deg: 차가 우측으로 쏠림 → PathOffset 증가 (경로를 우측으로)
        delta = int(avg_deg / _PATH_OFFSET_DEG_PER_UNIT)
        recommended = int(np.clip(current_offset + delta, -200, 200))
        if recommended != current_offset:
          result["조향 (Steering)"]["PathOffset"] = {
            "current": current_offset,
            "recommended": recommended,
            "band_kph": "straight driving",
            "avg_deg": round(avg_deg, 2),
          }

    # ── Phase 2b: SteerActuatorDelay (커브 진입 override 비율) ──────
    if self._curve_entries >= _LATERAL_MIN_CURVE:
      override_ratio = self._curve_overrides / self._curve_entries
      if override_ratio >= _CURVE_OVERRIDE_RATIO:
        current_delay = self._params.get_int("SteerActuatorDelay")
        recommended = min(300, current_delay + _DELAY_STEP_UNIT)
        if recommended != current_delay:
          result["조향 (Steering)"]["SteerActuatorDelay"] = {
            "current": current_delay,
            "recommended": recommended,
            "band_kph": "curve entry",
            "override_ratio": round(override_ratio * 100, 1),
          }

      # ── Phase 2c: SteerRatioRate (커브 override 비율이 매우 높을 때) ─
      # SteerActuatorDelay 증가로도 부족한 경우, SteerRatioRate(SR 배율)를 추가로 높임
      if override_ratio >= _SR_RATE_OVERRIDE_RATIO:
        current_sr_rate = self._params.get_int("SteerRatioRate")
        if current_sr_rate <= 0:
          current_sr_rate = 100  # 기본값 100%
        recommended_sr = min(150, current_sr_rate + _SR_RATE_STEP_UNIT)
        if recommended_sr != current_sr_rate:
          result["조향 (Steering)"]["SteerRatioRate"] = {
            "current": current_sr_rate,
            "recommended": recommended_sr,
            "band_kph": "curve entry (high override)",
            "override_ratio": round(override_ratio * 100, 1),
          }

    # ── Phase 3: JLeadFactor3 (수동 제동) ───────────────────────────
    jlead_candidate = None
    total_brake = self._brake_count + self._brake_auto_count
    if total_brake >= _BRAKE_MIN_COUNT:
      current_jlead = self._params.get_int("JLeadFactor3")
      recommended = min(200, current_jlead + _JLEAD_STEP_UNIT)
      if recommended != current_jlead:
        reason = "late braking (auto)" if self._brake_auto_count > self._brake_count else "approaching lead (manual)"
        jlead_candidate = {
          "current": current_jlead,
          "recommended": recommended,
          "band_kph": f"{reason}",
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
      if gas_sec >= _TFOLLOW_GAS_THRESHOLD_SEC:
        recommended_val = max(70, current_val + _TFOLLOW_STEP_UNIT)
        reason = "too wide (manual gas)"
        sec = gas_sec
      elif total_dec >= _TFOLLOW_BRAKE_THRESHOLD_SEC:
        recommended_val = min(200, current_val + _TFOLLOW_WIDEN_STEP)
        reason = "hunting detected (auto)" if self._tfollow_brake_auto_acc[i] > self._tfollow_brake_acc[i] else "too short (manual brake)"
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
    self._dyn_brake_count = 0
    self._decel_brake_count = 0
    self._params.remove("CarrotLearningData")
    self._params.remove("CarrotLearningRecommend")
    self._params.put_bool("CarrotLearningPopupReady", False)
