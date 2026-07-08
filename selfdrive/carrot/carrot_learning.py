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
from collections import deque
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
# 저속 밴드(0/1) 양방향 균형용 약한 하향 보정(decay)
# (저속대는 과가속 패널티가 ≥40km/h에서만 수집되어 단방향 상승하던 문제 보완)
_LOWBAND_DECAY_BANDS = (0, 1)   # 하향 보정을 적용할 저속 밴드 인덱스
_LOWBAND_DECAY_MIN_SEC = 60.0   # 해당 밴드를 이만큼 주행했을 때만 decay (오판 방지)
_LOWBAND_DECAY_GAS_DEADBAND = 1.0  # 가속요청 누적이 이 미만이면 '요청 없음'으로 간주
_LOWBAND_DECAY_STEP = 3         # 1회 decay 변화량 (기본값 방향으로 약하게)

# ── Phase 2 상수 ─────────────────────────────────────────────────────
_STRAIGHT_DEG = 5.0             # 직진 판단 조향각 임계값 (도)
_CURVE_RATE_DEG_S = 10.0        # (구) 커브 진입 판단: 조향각 변화율 임계값 (도/초)
_CURVE_DEG = 8.0               # 커브 구간 판단 조향각 임계값 (도). 이 이상이면 코너로 보고 매 틱 표본 수집
_LATERAL_MIN_SAMPLES = 200      # PathOffset 추천을 위한 최소 직진 샘플 수
_LATERAL_MIN_CURVE = 100        # SteerActuatorDelay 추천 최소 커브 표본 수 (per-tick, _DT=0.1 → 약 10초)
# PathOffset 신호: 차선중심 대비 횡편차(m). (구버전: 직진 평균 '조향각'을 신호로 썼으나
# PathOffset은 경로 횡위치만 옮길 뿐 조향각 평균(캠버/얼라인먼트로 결정)을 못 바꿔 신호가
# 수렴하지 않고 ±150(=±1.5m!)까지 폭주 → 차선 쏠림 유발. 차선중심 편차는 보정이 반영되면
# 0으로 수렴하는 자기제한 신호.)
_PATH_OFFSET_LC_MIN_M = 0.08      # 평균 차선중심 편차가 이 이상(m)이면 추천
_PATH_OFFSET_LC_GAIN = 0.5        # 편차(m)→units 변환 감쇠(0.5: 과보정 방지, 2~3사이클 수렴)
_PATH_OFFSET_STEP_MAX = 10        # 1회 추천 최대 변화량(units)
_PATH_OFFSET_ABS_MAX = 30         # 절대 상한(units, =0.30m). 구버전 ±150에서 대폭 축소
_CURVE_OVERRIDE_RATIO = 0.5     # 커브 진입의 50% 이상에서 override → SteerActuatorDelay 증가
_DELAY_STEP_UNIT = 10           # SteerActuatorDelay 한 번 추천 시 변화량 (UI 단위, +0.1s)
_SAD_LEARN_MIN = 15             # SteerActuatorDelay 학습 하한 (0.15s) — 과거 50은 너무 높았음
_SAD_LEARN_MAX = 60             # 학습 상한 (0.60s) — 과거 300(=3s)은 비현실적
_SAD_AUTO_BASELINE = 20         # 현재값 0(=liveDelay 자동)일 때 환산 기준 (0.20s)
_SR_RATE_OVERRIDE_RATIO = 0.7   # 커브 진입의 70% 이상에서 override → SteerRatioRate 추가 추천
_SR_RATE_STEP_UNIT = 3          # SteerRatioRate 한 번 추천 시 변화량 (+3%)

# ── 토크 피드백(KpV/KiV) 진동-우선 학습 상수 (고속 핑퐁/커브 탈출 진동 대응) ──
# 핵심: 추종오차의 '크기'가 아니라 '부호 패턴'으로 비례게인 방향을 정한다.
#   부호 반전(zero-crossing) 多 = 진동 → Kp 과다 → 하향 (발산 차단)
#   부호 유지 = 정상상태 lag → FF(Kf) 강화 우선, Kp는 보조만
_TORQUE_OSC_MIN = 8             # 커브 추종오차 부호반전(진동) 판정 최소 누적
_TORQUE_STEADY_MIN = 25         # 정상상태 lag(부호유지) 판정 최소 누적
_TORQUE_KPV_STEP = 5            # LateralTorqueKpV 변화량
_TORQUE_KPV_MIN = 30            # KpV 학습 하한 (default 100보다 낮게 — 고속 핑퐁 차량 수렴 허용)
_TORQUE_KPV_MAX = 200           # KpV 학습 상한
_TORQUE_KIV_MIN = 0             # KiV 학습 하한
_TORQUE_KIV_MAX = 50            # KiV 학습 상한
_TORQUE_KF_FF_PREF = 100        # Kf가 이 이상이면 'FF 충분' → 정상상태 lag을 Kp로 보조
_TORQUE_PINGPONG_MIN_KPH = 70.0 # 고속 직선 핑퐁 검출 최소 속도
_TORQUE_PINGPONG_DEG = 0.4      # 핑퐁 반전 판정 조향각 데드밴드 (도)
_TORQUE_PINGPONG_MIN_COUNT = 15 # 직선 핑퐁 판정 최소 반전 횟수

# ── Phase 3 상수 ─────────────────────────────────────────────────────
_BRAKE_MIN_COUNT = 6            # 추천을 위한 최소 수동 브레이크 횟수 (5 -> 6, 민감도 완화)
_JLEAD_LATE_TTC = 3.5          # 이 TTC(초) 미만의 제동만 '늦은 제동'으로 인정(정상 제동 누적 방지)
_JLEAD_PROACTIVE_TTC = 6.0     # 선제적 '교육용' 제동으로 인정할 TTC 상한 (3.5~6.0s)
_JLEAD_PROACTIVE_DECEL = 1.0   # 선제 제동이 '굼뜬 반응' 신호로 인정될 최소 감속도 (m/s^2)
_JLEAD_PROACTIVE_STEP = 5      # 선제 제동 1회 추천 시 약한 증가량 (정상 제동 과누적 방지)
_JLEAD_AUTO_TTC = 6.0          # 자율 제동을 '늦은 제동'으로 셀 TTC 상한 (이상이면 여유 감속으로 간주, 누적 제외)
_JLEAD_AUTO_PANIC_DECEL = -2.5 # TTC와 무관하게 늦은 제동으로 인정할 패닉 감속도 (m/s^2)
_JLEAD_STEP_UNIT = 20           # JLeadFactor3 한 번 추천 시 변화량 (강화: 10 -> 20)
_JLEAD_REDUCE_STEP = -7         # 제동 과다 시 변화량
_JLEAD_GAS_THRESHOLD_SEC = 5.0  # 제동 중 가속 개입 누적 기준 (초)
# JLeadFactor3 추천 상한(80→20). late-braking 신호로 무한 상향되어(runaway) 37까지 올라
# 선행차 감지 초기 지령 jerk 스파이크(급격한 onset)를 유발한 문제 대응. 고속 선제 제동은
# 별도 Lever A/C(HIGH_SPEED_*)가 담당하므로 이 상한을 낮춰도 고속 안전성은 유지된다.
_JLEAD_MAX_RECO = 20

# ── Phase 5 상수 (DynamicTFollow / TFollowDecelBoost) ────────────────
# DynamicTFollow: 앞차 급감속(jLead↓) 중 브레이크 개입 횟수 기반
_DYN_TFOLLOW_BRAKE_MIN = 6      # 추천 발동 최소 이벤트 수 (4 -> 6, 단방향 누적 완화)
_DYN_JLEAD_THRESHOLD = -1.0     # 앞차 가감속도 (m/s^3) 이하이면 '급감속' 판정
_DYN_TFOLLOW_STEP = 3           # DynamicTFollow 한 번 추천 시 변화량 (+0.03)
_DYN_TFOLLOW_MAX = 50           # 최대값 (=0.5)
# TFollowDecelBoost: 내 차 감속 중 브레이크 개입 횟수 기반
_DECEL_BOOST_BRAKE_MIN = 6      # 추천 발동 최소 이벤트 수 (4 -> 6, 단방향 누적 완화)
_DECEL_A_THRESHOLD = -0.8       # 내 차 감속도 (m/s^2) 이하이면 '강한 감속' 판정
_DECEL_BOOST_STEP = 3           # TFollowDecelBoost 한 번 추천 시 변화량 (+0.03)
_DECEL_BOOST_MAX = 40           # 최대값

# ── Phase 4 상수 ─────────────────────────────────────────────────────
# TFollowGap1~4: Gap1(가장 좁음, 공격적), Gap4(가장 넓음, 여유)
# 각 Gap에 해당하는 Params key
_TFOLLOW_KEYS = ["TFollowGap1", "TFollowGap2", "TFollowGap3", "TFollowGap4"]
_TFOLLOW_NAMES = ["GAP1", "GAP2", "GAP3", "GAP4"]
_TFOLLOW_GAS_THRESHOLD_SEC = 15.0  # 선행차 추종 중 gas 누적 개입 시간 기준
_TFOLLOW_GAP_DIFF_MIN = 10         # 축소 최소 근거: 운전자가 설정보다 ≥0.10s 더 바짝 따라간 경우만
_TFOLLOW_STEP_UNIT = -5            # 감소 추천 (-5 units = -0.05s)
_TFOLLOW_MAX_LEAD_DREL = 150.0     # 선행차 거리 인식 최대 한계선
_TFOLLOW_MIN_V_KPH = 40.0          # 학습을 개시할 최소 주행 속도 (60 -> 40)
_TFOLLOW_WIDEN_STEP = 5            # 증가 추천 (+0.05s)
_TFOLLOW_BRAKE_THRESHOLD_SEC = 10.0 # 거리 부족으로 인한 브레이크 누적 기준 (초)
_TFOLLOW_SPEED_FACTOR_THRESHOLD_SEC = 15.0 # 고속 보정 발동 기준 (10 -> 15, 단방향 누적 완화)
_TFOLLOW_SPEED_FACTOR_STEP = 5     # 고속 보정치 증가 단위 (+0.05)
_AUTO_HUNTING_THRESHOLD = 0.8      # 자율 주행 중 가감속 변동(Hunting) 감지 임계치 (m/s^2)

# ── Phase 7 상수 (정차/출발: StoppingAccel / VEgoStopping / StopDistanceCarrot) ──
_STOP_APPROACH_V_KPH = 25.0     # 이 속도 이하에서 자율 감속 중이면 '정차 접근'으로 판정
_STOP_DECEL_THRESHOLD = -0.3    # 정차 접근 판정 감속도 (m/s^2)
_STOP_FULLSTOP_V_KPH = 0.8      # 완전 정지 판정 속도 (km/h)
_STOP_MIN_EVENTS = 5            # 추천 발동 최소 정차 완료 횟수
_STOP_HARSH_JERK = 2.5          # 정차 직전 급격한 감속도 변화(저크) 임계값 (m/s^3)
_STOP_ACCEL_STEP = 5            # StoppingAccel 변화량 (UI, +0.05m/s^2)
_STOP_VEGO_STEP = 3             # VEgoStopping 변화량 (UI, +0.03m/s)
_STOP_DIST_STEP = 20            # StopDistanceCarrot 변화량 (UI, +0.20m)
_STOP_GAP_WIDE_M = 5.0          # 정지 시 선행차와 이 거리 이상이면 '너무 멀리 정지' (선행차에 더 가깝게: 6.0→5.0)
_STOP_GAP_NEAR_M = 3.0          # 정지 시 선행차와 이 거리 이하이면 '너무 가까이 정지' (하한 당김: 3.5→3.0)

# ── Phase 8 상수 (종방향 PID: LongTuningKpV / LongTuningKf / LongActuatorDelay) ──
_LONG_MIN_SAMPLES = 300         # 추천 발동 최소 샘플 수 (~30초 자율 가감속)
_LONG_ERR_THRESHOLD = 0.4       # 추종 오차(지령-실측 가속도) 유의 임계값 (m/s^2)
_LONG_LAG_RATIO = 0.30          # 둔감(lag) 비율 임계값 → Kf/Delay 상향
_LONG_OVERSHOOT_RATIO = 0.30    # 진동(overshoot) 비율 임계값 → KpV 하향
# lag는 '지령이 변하는 중(transient)'에서만 측정한다. 정상상태(정속)에서 남는 추종오차는
# 도로 경사(aEgo가 중력성분 포함)·게인 오프셋이라 '시간지연(lag)'이 아니며, 이를 lag로
# 오인하면 LongActuatorDelay/Kf를 잘못된 방향으로 무한 상향한다. (로그 분석으로 확인:
# 지령을 시간이동해도 오차가 안 줄고, 같은 코드라도 도로에 따라 오차가 크게 달라짐)
_LONG_CMD_CHANGE = 0.03         # 지령가속도가 1스텝(_DT)에 이만큼(m/s^2) 변하면 transient 시작
_LONG_TRANS_HOLD_STEPS = 8      # 지령 변화 후 transient로 유지할 스텝 수 (8*_DT=0.8s, 추종 정착)
_LONG_TRANS_MIN = 120           # transient lag_ratio를 신뢰할 최소 transient 표본 수
_LONG_KP_STEP = 5               # LongTuningKpV 변화량
_LONG_KF_STEP = 3               # LongTuningKf 변화량
_LONG_DELAY_STEP = 5            # LongActuatorDelay 변화량

# ── Phase 9 상수 (수동주행 기준분포 로거 → LongCoastBand 추천) ──────────
# 인게이지 '개입 카운팅'과 달리, 사람이 직접 운전하는 동안의 (상황 → 가감속/추종거리/
# 페달상태)를 통째로 누적하여 '사람이라면 어떻게 했을까'의 기준분포(정답)로 삼는다.
# 1차 적용: 무페달(코스팅) 구간의 자연 감속(회생제동/엔진브레이크)을 측정하여
# 종방향 코스팅 데드밴드(LongCoastBand)를 직접 보정 — 역문제가 없는 깨끗한 학습 대상.
_MANUAL_MIN_V_KPH = 20.0        # 수동주행 기준분포 수집 최소 속도 (정체 stop&go 잡음 배제)
_MANUAL_COAST_MIN_N = 300       # LongCoastBand 추천 발동 최소 코스팅 표본 (~30초)
_MANUAL_COAST_MIN_SEC = 60.0    # LongCoastBand 추천 발동 최소 누적 코스팅 시간
_MANUAL_COAST_GAIN = 0.25       # 측정 코스팅 감속 → 데드밴드 변환 계수(차의 코스트 권한 일부만 사용)

# ── 공통 ─────────────────────────────────────────────────────────────
_DT = 0.1  # update() 호출 주기 (초)


# ── 파라미터 스펙 레지스트리 (오픈파일럿 파라미터 문서1/2/3 기준) ──────
# 흩어져 있던 변환식·범위·방향성을 단일 출처로 코드화하여 부호 반전/
# 범위 이탈을 원천 차단한다. 신규 학습 항목 추가 시 여기에 1줄만 등록.
#   min/max     : UI 정수 기준 안전 범위 (문서 '범위' 컬럼)
#   default     : 공장 기본값 (문서 '기본값' 컬럼)
#   conv        : UI 정수 → 실제값 변환 계수 (참고용)
#   direction   : +1 = 값↑이 효과↑ / -1 = 값↑이 효과↓ (분모 등 역방향)
#   apply       : "lat" | "long" → CarrotTunerApplyLat/Long 필터 카테고리
_PARAM_SPEC = {
  # 종방향 PID / 액추에이터 (문서2)
  "LongTuningKpV":      {"min": 0,    "max": 150,  "default": 100, "conv": 0.01,  "direction": 1,  "apply": "long"},
  "LongTuningKiV":      {"min": 0,    "max": 2000, "default": 0,   "conv": 0.001, "direction": 1,  "apply": "long"},
  # max 200→120/30: lag 지표가 action_t(=LongActuatorDelay) 선행분만큼 cmd-actual 차이를
  # 부풀려 무한 상향(runaway, 20→90)시키고 그 결과 output이 궤적을 과도히 앞서 감지 순간
  # 급제동을 유발 → 상한을 실제 물리값 근방으로 제한.
  "LongTuningKf":       {"min": 0,    "max": 120,  "default": 100, "conv": 0.01,  "direction": 1,  "apply": "long"},
  "LongActuatorDelay":  {"min": 0,    "max": 30,   "default": 20,  "conv": 0.01,  "direction": 1,  "apply": "long"},
  # 코스팅 데드밴드(0=비활성). 값↑ = 더 넓은 무가감속 구간 → 코스팅(회생제동) 더 적극 사용
  "LongCoastBand":      {"min": 0,    "max": 40,   "default": 0,   "conv": 0.01,  "direction": 1,  "apply": "long"},
  # 커브 감속 (문서1/실제 knob: vturn_speed의 AutoCurveSpeedFactor)
  # 값↑ = 곡률을 더 민감하게 인식 → 커브 목표속도↓ → 더 일찍/충분히 감속
  # 안전 학습 밴드는 80~200 (params_keys.h 절대범위 50~300 내)
  "AutoCurveSpeedFactor": {"min": 80,  "max": 160,  "default": 120, "conv": 0.01,  "direction": 1,  "apply": "long"},
  # 정차 / 출발 (문서2)
  "StoppingAccel":      {"min": -100, "max": 0,    "default": 0,   "conv": 0.01,  "direction": 1,  "apply": "long"},
  "VEgoStopping":       {"min": 1,    "max": 100,  "default": 50,  "conv": 0.01,  "direction": 1,  "apply": "long"},
  "StopDistanceCarrot": {"min": 350,  "max": 1000, "default": 550, "conv": 0.01,  "direction": 1,  "apply": "long"},
  # 토크 조향 (문서2) — AccelFactor는 분모이므로 direction=-1
  "LateralTorqueAccelFactor": {"min": 1000, "max": 6000, "default": 2500, "conv": 0.001, "direction": -1, "apply": "lat"},
  "LateralTorqueKf":          {"min": 0,    "max": 200,  "default": 100,  "conv": 0.01,  "direction": 1,  "apply": "lat"},
}


def _clamp_spec(key: str, raw) -> int:
  """레지스트리 범위로 안전 클램프. 미등록 키는 원값(int)으로 반환."""
  spec = _PARAM_SPEC.get(key)
  if spec is None:
    return int(raw)
  return int(np.clip(raw, spec["min"], spec["max"]))


def _decay_toward(cur, target, step) -> int:
  """cur를 target 방향으로 최대 step만큼만 이동(넘어가지 않음). 양방향 균형 수렴용.
  '잘 달릴 때 게인을 0까지 무한 감쇄'하던 단방향 drift를 막고 기본값으로 수렴시킨다."""
  cur = int(cur)
  target = int(target)
  step = abs(int(step))
  if cur > target:
    return max(target, cur - step)
  if cur < target:
    return min(target, cur + step)
  return cur


# ── 공장 기본값 (params_keys.h 기준 단일 출처) ────────────────────────
# 오토튜너가 변경할 수 있는 모든 파라미터의 설치 시 기본값.
# '공장초기화(Factory Reset)' 시 이 값으로 일괄 복원한다.
_FACTORY_DEFAULTS = {
  "CruiseMaxVals0": 160, "CruiseMaxVals1": 200, "CruiseMaxVals2": 160,
  "CruiseMaxVals3": 130, "CruiseMaxVals4": 110, "CruiseMaxVals5": 95,
  "CruiseMaxVals6": 80,
  "JLeadFactor3": 0,
  "TFollowGap1": 110, "TFollowGap2": 120, "TFollowGap3": 140, "TFollowGap4": 160,
  "TFollowSpeedFactor": 0, "DynamicTFollow": 0, "TFollowDecelBoost": 10,
  "PathOffset": 0, "SteerActuatorDelay": 0, "SteerRatioRate": 100,
  "LateralTorqueAccelFactor": 2500, "LateralTorqueKf": 100,
  "LateralTorqueFriction": 100, "LateralTorqueKiV": 10, "LateralTorqueKpV": 100,
  "AutoCurveSpeedFactor": 120, "AutoCurveSpeedAggressiveness": 100,
  "StoppingAccel": 0, "VEgoStopping": 50, "StopDistanceCarrot": 550,
  "LongTuningKf": 100, "LongTuningKpV": 100, "LongActuatorDelay": 20,
  "LongCoastBand": 0,
}


# 추천 키 → 소속 Phase. '적용'된 Phase의 누적치만 선택적으로 리셋하기 위한 매핑.
# (과거: 적용 여부와 무관하게 전체 Phase를 리셋 → 느리게 쌓이는 조향 학습이
#  무관한 longitudinal 적용 시마다 초기화되어 문턱에 도달하지 못했음)
_KEY_RESET_PHASE = {
  **{k: 1 for k in _ACCEL_KEYS},
  "PathOffset": 2, "SteerActuatorDelay": 2, "SteerRatioRate": 2,
  "LateralTorqueAccelFactor": 2, "LateralTorqueKf": 2, "LateralTorqueFriction": 2,
  "LateralTorqueKiV": 2, "LateralTorqueKpV": 2,
  "JLeadFactor3": 3,
  **{k: 4 for k in _TFOLLOW_KEYS}, "TFollowSpeedFactor": 4,
  "DynamicTFollow": 5, "TFollowDecelBoost": 5,
  "AutoCurveSpeedFactor": 6,
  "StoppingAccel": 7, "VEgoStopping": 7, "StopDistanceCarrot": 7,
  "LongTuningKf": 8, "LongActuatorDelay": 8, "LongTuningKpV": 8,
  "LongCoastBand": 9,
}


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
    self._band_sec = [0.0] * _NUM_BANDS     # 밴드별 인게이지 주행시간 (저속 decay 판정용)
    # Phase 2
    self._steer_acc = 0.0        # 직진 구간 조향각 누적합 (도)
    self._steer_count = 0        # 직진 샘플 수
    self._lane_center_acc = 0.0  # 직진 구간 차선중심 y좌표 누적합 (m, 모델 프레임 — path.y와 동일 좌표계)
    self._lane_center_n = 0      # 차선중심 표본 수 (세션 한정, 비영속)
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
    self._speed_factor_sec = 0.0             # TFollowSpeedFactor decay용 고속 주행 누적 시간
    # Phase 5 (DynamicTFollow / TFollowDecelBoost)
    self._dyn_brake_count = 0    # 앞차 급감속 중 브레이크 개입 횟수
    self._decel_brake_count = 0  # 내 차 강한 감속 중 브레이크 개입 횟수
    self._dyn_sec = 0.0          # DynamicTFollow decay용 인게이지+선행차 급감속 없음 시간
    self._decel_sec = 0.0        # TFollowDecelBoost decay용 인게이지 감속 없음 시간
    self._jlead_sec = 0.0        # JLeadFactor3 decay용 인게이지+선행차 근접 주행 시간

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
    # 진동-우선 학습용: 추종오차 '부호 패턴' 추적
    self._torque_osc_count = 0          # 커브 추종오차 부호반전(진동) 누적
    self._torque_steady_count = 0       # 커브 추종오차 부호유지(정상상태 lag) 누적
    self._torque_straight_reversal = 0  # 고속 직선 조향각 반전(핑퐁) 누적
    self._prev_torque_err_sign = 0      # 직전 유의 추종오차 부호 (커브 진동 검출)
    self._prev_torque_straight_sign = 0 # 직전 직선 조향각 부호 (핑퐁 검출)


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
    self._curve_brake_min_v = 999.0
    self._curve_active_sec = 0.0   # 커브 주행 누적 시간(개입 무관). decay(편안한 커브 판정)용

    # Phase 7 (정차/출발: StoppingAccel / VEgoStopping / StopDistanceCarrot)
    self._stop_events = 0            # 완전 정지 완료 횟수 (denominator)
    self._stop_brake_sec = 0.0       # 정차 접근 중 운전자 추가 제동 누적 (정차가 약/늦음)
    self._stop_gas_sec = 0.0         # 정차 접근 중 운전자 가속 개입 누적 (정차가 강/이름)
    self._stop_harsh_count = 0       # 정차 직전 급격한 저크 발생 횟수 (거친 정지)
    self._stop_lead_gap_sum = 0.0    # 선행차 뒤 정지 시 최종 거리 합 (m)
    self._stop_lead_gap_count = 0    # 선행차 뒤 정지 표본 수
    self._stop_approaching = False   # 정차 접근 상태 플래그
    self._stop_max_jerk = 0.0        # 현재 정차 접근의 피크 저크 (transient)
    self._prev_v_kph = 0.0           # 이전 프레임 속도

    # Phase 8 (종방향 PID: LongTuningKpV / LongTuningKf / LongActuatorDelay)
    self._long_samples = 0           # 자율 가감속 추종 표본 수(전체)
    self._long_err_sum = 0.0         # |지령가속도 - 실측가속도| 누적(전체, 참고용)
    self._long_lag_count = 0         # 둔감 표본 수(전체, 참고용)
    self._long_overshoot_count = 0   # 진동(부호 반전) 표본 수
    self._prev_long_err = 0.0        # 이전 프레임 추종 오차
    self._prev_cmd_accel = 0.0       # 이전 프레임 지령가속도 (transient 감지용)
    self._long_trans_hold = 0        # transient 유지 카운터(스텝)
    self._cmd_hist = deque(maxlen=12)  # 지령가속도 이력(지연보상 비교용, 1.2s, 비영속)
    self._long_delay_units = 20      # LongActuatorDelay 캐시(UI units, 주기 갱신)
    self._long_trans_samples = 0     # transient(지령 변화 중) 표본 수 → lag 분모
    self._long_trans_lag = 0         # transient 중 lag(둔감) 표본 수
    self._long_trans_err_sum = 0.0   # transient 중 |추종오차| 누적

    # Phase 9 (수동주행 기준분포 로거 → LongCoastBand)
    # 모두 밴드별(_NUM_BANDS) 누적. 사람이 직접 운전하는 동안의 '정답' 분포.
    self._manual_coast_sec = [0.0] * _NUM_BANDS        # 무페달(코스팅) 주행 누적 시간
    self._manual_coast_decel_sum = [0.0] * _NUM_BANDS  # 코스팅 중 자연 감속 크기 합 (m/s², 양수)
    self._manual_coast_decel_n = [0] * _NUM_BANDS      # 코스팅 중 감속 표본 수
    self._manual_gas_accel_sum = [0.0] * _NUM_BANDS    # 가속페달 시 사람의 가속도 합 (m/s²)
    self._manual_gas_n = [0] * _NUM_BANDS              # 가속페달 표본 수
    self._manual_brake_decel_sum = [0.0] * _NUM_BANDS  # 브레이크 시 사람의 감속 크기 합 (m/s², 양수)
    self._manual_brake_n = [0] * _NUM_BANDS            # 브레이크 표본 수
    self._manual_gap_sum = 0.0       # 수동 추종 차간시간(time gap) 합 (s)
    self._manual_gap_n = 0           # 수동 추종 표본 수

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

  def _cur_or_default(self, key: str) -> int:
    """현재 파라미터값(get_int). 미설정(≤0)이면 레지스트리 기본값으로 대체.
    (추천 계산에서 반복되던 'get_int 후 ≤0이면 default' 패턴을 통일.)"""
    cur = self._params.get_int(key)
    if cur <= 0:
      cur = _PARAM_SPEC[key]["default"]
    return cur

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
    # 공장초기화 신호 (학습 활성 여부와 무관하게 항상 처리)
    # 신규 키가 params 바인딩에 아직 없을 수 있으므로 방어적으로 처리한다.
    # (여기서 예외가 나도 아래 학습 로직은 정상 진행되어야 함)
    try:
      if self._params.get_bool("CarrotTunerFactoryReset"):
        self._factory_reset()
        self._params.put_bool("CarrotTunerFactoryReset", False)
    except Exception:
      pass

    if not self._is_active():
      if self._params.get_bool("CarrotLearningPopupReady"):
        self._params.put_bool("CarrotLearningPopupReady", False)
      return

    # Apply 토글(LAT/LONG)이 꺼져 있으면 해당 카테고리의 학습(데이터 누적)도 멈춘다.
    # (과거: 토글은 추천 계산/적용만 막고 누적은 계속됨 → 끈 동안 쌓인 데이터가
    #  재활성화 시 한꺼번에 반영되던 문제. 이제 OFF면 누적 자체를 건너뛴다.)
    #   - apply_lat  : Phase 2(조향) 누적 게이트
    #   - apply_long : Phase 1/3/4/5/6/7/8/9(가속·제동·추종·곡선·정차·PID·수동분포) 게이트
    apply_lat = self._params.get_bool("CarrotTunerApplyLat") if self._params.get("CarrotTunerApplyLat") is not None else True
    apply_long = self._params.get_bool("CarrotTunerApplyLong") if self._params.get("CarrotTunerApplyLong") is not None else True

    # 조향 제어 방식 lazy loading 감지
    if self.is_angle_control is None:
      self._detect_steer_control_type()

    prev_brake = self._prev_brake

    # ── 학습 제외 조건 판정 ──────────────────────────
    left_prob = 1.0
    right_prob = 1.0
    left_blinker = False
    right_blinker = False

    if sm is not None:
      if 'modelV2' in sm.data and sm.alive.get('modelV2', False):
        try:
          model = sm['modelV2']
          if hasattr(model, 'laneLineProbs') and len(model.laneLineProbs) >= 4:
            left_prob = model.laneLineProbs[1]
            right_prob = model.laneLineProbs[2]
          elif hasattr(model, 'laneLineProbs') and len(model.laneLineProbs) >= 2:
            left_prob = model.laneLineProbs[0]
            right_prob = model.laneLineProbs[1]
        except Exception:
          pass

      if 'carState' in sm.data and sm.alive.get('carState', False):
        try:
          car_state = sm['carState']
          left_blinker = car_state.leftBlinker
          right_blinker = car_state.rightBlinker
        except Exception:
          pass

    poor_lanes = (left_prob < 0.5 and right_prob < 0.5)
    blinker_on = (left_blinker or right_blinker)
    extreme_acceleration = (a_ego > 2.2 or gas_val > 0.7)

    exclude_override = poor_lanes or blinker_on
    exclude_steer_learning = exclude_override
    exclude_gas_learning = extreme_acceleration or exclude_override
    exclude_brake_learning = exclude_override

    # UI로부터 초기화(Clear) 신호가 오면 내부 메모리를 비움
    if self._params.get_bool("CarrotLearningClear"):
      self._clear_all_data()
      self._params.put_bool("CarrotLearningClear", False)

    # 주행 여부 판단 (오픈파일럿 인게이지가 단 한 번이라도 실행되었을 때만 유효 주행 세션으로 판단)
    if engaged:
      self._has_driven = True

    # ── Phase 1: 가속 개입 ────────────────────────────────────────────
    # 밴드별 주행시간 누적 (저속 decay 판정용: 실제로 그 속도대를 달렸는지 확인)
    if apply_long and engaged and v_ego_kph >= 1.0:
      self._band_sec[_speed_band(v_ego_kph)] += _DT

    # 단순히 설정속도에 도달했는데 더 빨리 가고 싶어 밟는 경우는 제외 (설정속도 오버라이드)
    # 즉, 설정속도보다 충분히 낮은데도 가속이 답답할 때만 학습에 포함
    if apply_long and engaged and gas_pressed and v_ego_kph >= 1.0:
      # 선행차를 추종하며 간격을 좁히려는 가속(=차간거리 선호)은 '가속 능력 부족'이 아니므로
      # CruiseMaxVals(최대 가속도) 학습에서 제외한다. (선행차 거리·상대속도 동특성 반영)
      #   - 가까운 선행차(60m 이내)가 있고, 그 차가 나보다 크게 빠르지 않으면(멀어지지 않으면)
      #     운전자의 가속은 간격 좁히기/추종 목적 → 가속능력 부족 신호로 보지 않음.
      #   - 저속 정체(stop&go)에서 CruiseMaxVals0가 단방향 누적되던 주원인.
      lead_follow_gas = False
      if 0.0 < lead_drel < 60.0:
        v_rel_kph = lead_v_kph - v_ego_kph   # 양수면 선행차가 더 빠름(멀어짐)
        if v_rel_kph < 10.0:
          lead_follow_gas = True
      if v_ego_kph < (v_cruise_kph - 3.0) and not exclude_gas_learning and not lead_follow_gas:
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
    if apply_long and engaged and v_ego_kph < (v_cruise_kph - 3.0) and not is_speed_bump and v_ego_kph >= 40.0:
      idx = _speed_band(v_ego_kph)
      if brake_pressed:
        if (lead_drel == 0 or lead_drel > 120.0) and not exclude_brake_learning:
          self._gas_dec_acc[idx] += _DT
      elif not gas_pressed and a_ego > 1.5: # 자율 주행 중 급가속 감지
        self._gas_dec_auto_acc[idx] += _DT

    # ── Phase 2: 조향 패턴 (속도 20km/h 이상, 인게이지 상태) ──────────
    if apply_lat and engaged and v_ego_kph >= 20.0:
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
          # PathOffset용 차선중심 편차(m) 수집 — PathOffset이 실제로 바꾸는 신호(횡위치)
          try:
            if sm is not None and sm.alive.get('modelV2', False):
              md = sm['modelV2']
              ll, lp = md.laneLines, md.laneLineProbs
              if len(ll) >= 4 and len(lp) >= 4 and lp[1] > 0.5 and lp[2] > 0.5 \
                 and len(ll[1].y) > 0 and len(ll[2].y) > 0:
                self._lane_center_acc += (ll[1].y[0] + ll[2].y[0]) * 0.5
                self._lane_center_n += 1
          except Exception:
            pass

        # 커브 구간 감지: 조향각이 충분히 크고 속도가 있으면 코너로 보고 "매 틱" 표본 수집.
        # (구버전은 진입 순간 1틱에서 steer_pressed인 경우만 봐서, 코너 중간에 들어가는
        #  언더스티어 보정(바깥 쏠림 후 안쪽으로 꺾음)을 거의 못 잡았음 → 학습 안 됨)
        if v_ego_kph >= 30.0 and abs(steer_deg) >= _CURVE_DEG:
          self._curve_entries += 1
          if steer_pressed and not exclude_steer_learning:
            self._curve_overrides += 1
            # 개입 방향 판정 (desired_angle * steer_err)
            # desired_angle과 steer_err의 부호가 같으면 Outer(안쪽쏠림, 풀어주기), 반대면 Inner(언더스티어, 더 꺾기)
            if desired_angle * steer_err > 0:
              self._curve_overrides_inner_hugging += 1
            elif desired_angle * steer_err < 0:
              self._curve_overrides_understeer += 1
      else:
        # [B] 토크 조향 차량 데이터 수집
        # 1. 커브 구간 (조향각이 크고 속도가 있을 때)
        if v_ego_kph >= 40.0 and abs(steer_deg) >= 5.0:
          self._torque_curve_entries += 1
          if steer_pressed and not exclude_steer_learning:
            self._torque_curve_overrides += 1
            # 개입 방향 판정
            if desired_angle * steer_err > 0:
              self._torque_curve_overrides_inner_hugging += 1
            elif desired_angle * steer_err < 0:
              self._torque_curve_overrides_understeer += 1

          # 추종오차의 '크기'가 아니라 '부호 패턴'을 본다.
          # 부호 반전(zero-crossing) = 진동(Kp 과다) / 부호 유지 = 정상상태 lag.
          # (구버전은 |err|만 세서 진동이 만든 오차를 '게인 부족'으로 오인 → KpV 무한 상향 발산)
          if abs(steer_err) >= 1.5:
            self._torque_error_count += 1
            err_sign = 1 if steer_err > 0 else -1
            if self._prev_torque_err_sign != 0:
              if err_sign != self._prev_torque_err_sign:
                self._torque_osc_count += 1     # 부호 반전 → 진동
              else:
                self._torque_steady_count += 1  # 부호 유지 → 정상상태 lag
            self._prev_torque_err_sign = err_sign

        # 2. 완만한/직선 구간 (조향각이 작을 때 - Friction 용)
        elif v_ego_kph >= 30.0 and abs(steer_deg) < 5.0:
          self._torque_straight_entries += 1
          self._prev_torque_err_sign = 0  # 커브 종료 → 진동 부호 추적 초기화(구간 간 오검출 방지)
          if steer_pressed and not exclude_steer_learning:
            self._torque_straight_overrides += 1
          # 고속 직선 핑퐁(KpV 과다) 검출: 운전자 개입 없이 조향각이
          # 0 근처에서 반복 반전 → 순수 비례게인 과다 신호(소유주가 KpV로 잡던 현상).
          elif v_ego_kph >= _TORQUE_PINGPONG_MIN_KPH:
            s_sign = 0
            if steer_deg > _TORQUE_PINGPONG_DEG:
              s_sign = 1
            elif steer_deg < -_TORQUE_PINGPONG_DEG:
              s_sign = -1
            if s_sign != 0:
              if self._prev_torque_straight_sign != 0 and s_sign != self._prev_torque_straight_sign:
                self._torque_straight_reversal += 1
              self._prev_torque_straight_sign = s_sign

    self._prev_steer_deg = steer_deg

    # Phase 3: 수동 제동 (선행차 근접 시) ────────────────────────
    is_auto_braking = False
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 제동 학습(JLeadFactor) 수집 제외
    if apply_long and engaged and not gear_park and 0 < lead_drel < 100.0 and not is_speed_bump and v_ego_kph >= 40.0:
      # (1) 수동 제동 트리거
      if brake_pressed and not exclude_brake_learning:
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
      #  단, TTC(위급도)를 함께 본다. 잘 예측된 '여유 있는' 중간 감속(TTC 충분)은
      #  늦은 제동이 아니므로 제외 — onset jerk 완화로 생긴 정상 감속이 JLeadFactor3를
      #  끌어올려 다시 제동을 날카롭게 만드는 악순환을 차단한다.
      elif not brake_pressed and a_ego < -1.7:
        v_ego_ms = v_ego_kph / 3.6
        lead_v_ms = lead_v_kph / 3.6
        v_rel_ms = lead_v_ms - v_ego_ms
        ttc = lead_drel / -v_rel_ms if v_rel_ms < 0 else 999.0
        # 실제 '늦은 제동' 조건: TTC가 낮아 위급(접근)하거나, 패닉 수준 급제동
        if ttc < _JLEAD_AUTO_TTC or a_ego < _JLEAD_AUTO_PANIC_DECEL:
          is_auto_braking = True
          self._brake_min_ttc = min(self._brake_min_ttc, ttc)
          if not self._prev_auto_brake:
            self._brake_auto_count += 1
    
    self._prev_auto_brake = is_auto_braking
    
    # 제동 과다 학습 방지: 강한 제동 중 가속 페달을 밟는 경우 (불필요한 제동 억제)
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 가속 오버라이드 학습 수집 제외
    if apply_long and engaged and gas_pressed and a_ego < -0.8 and not is_speed_bump and v_ego_kph >= 40.0:
      if 0 < lead_drel < 150.0 and not exclude_gas_learning:
        self._jlead_gas_acc += _DT

    # ── Phase 5: DynamicTFollow / TFollowDecelBoost ──────────────────
    # A/B안 적용: 과속방지턱 통과 중이거나 40km/h 미만에서는 DynamicTFollow 학습 수집 제외
    if apply_long and engaged and brake_pressed and not self._prev_brake and not is_speed_bump and v_ego_kph >= 40.0 and not exclude_brake_learning:
      # DynamicTFollow: 앞차 급감속 중 브레이크 개입
      if lead_jlead < _DYN_JLEAD_THRESHOLD and lead_drel < 150.0:
        self._dyn_brake_count += 1
      # TFollowDecelBoost: 내 차 강한 감속 중 브레이크 개입
      if a_ego < _DECEL_A_THRESHOLD:
        self._decel_brake_count += 1

    # decay 누적 시간 업데이트
    if apply_long and engaged and not gear_park and not is_speed_bump and v_ego_kph >= 40.0:
      self._decel_sec += _DT
      if 0.0 < lead_drel < 100.0:
        self._jlead_sec += _DT
      if 0.0 < lead_drel < 150.0:
        self._dyn_sec += _DT
    if apply_long and engaged and v_ego_kph >= 80.0:
      self._speed_factor_sec += _DT

    self._prev_brake = brake_pressed

    # ── Phase 4: TFollowGap (선행차 추종 중 가속/감속 개입) ──────────
    # 선행차가 잡힌 상태(0 < lead_drel < _TFOLLOW_MAX_LEAD_DREL)에서
    # 일정 속도 이상 주행 중 페달 개입을 분석합니다.
    if apply_long and engaged and v_ego_kph >= _TFOLLOW_MIN_V_KPH and 0.0 < lead_drel < _TFOLLOW_MAX_LEAD_DREL:
      gap_idx = self._current_gap - 1  # 0-indexed
      
      # (1) 가속 페달 개입 시 -> 간격 좁히기 의도로 판단
      if gas_pressed and not exclude_gas_learning:
        self._tfollow_gas_acc[gap_idx] += _DT
        v_ego_ms = v_ego_kph / 3.6
        if v_ego_ms > 1.0:
          time_gap = lead_drel / v_ego_ms
          self._tfollow_min_gap[gap_idx] = min(self._tfollow_min_gap[gap_idx], time_gap)
      
      # (2) 브레이크 페달 개입 시 -> 차간 거리가 가까워지거나 불안해서 감속하려는 의도로 판단
      elif brake_pressed and not exclude_brake_learning:
        self._tfollow_brake_acc[gap_idx] += _DT
        
        # 고속 주행(80km/h 이상) 시 추가 고속 차간 거리 보정 학습 누적
        if v_ego_kph >= 80.0:
          self._tfollow_speed_brake_acc += _DT
      
      # (3) 페달 개입이 없을 때 자율 주행 중 가감속 헌팅(Swing) 감지
      else:
        if (self._prev_a_ego > 0.3 and a_ego < -0.3) or (self._prev_a_ego < -0.3 and a_ego > 0.3):
          self._accel_swing_count += 1
          if self._accel_swing_count > 8:
            self._tfollow_brake_auto_acc[gap_idx] += _DT * 2.0
            self._accel_swing_count = 0

    # ── Phase 6: 가변 곡선 감속 학습 ──────────────────────────────────────────
    if apply_long and engaged and sm is not None and sm.alive.get('modelV2', False) and v_ego_kph >= 20.0:
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
          self._curve_active_sec += _DT   # 커브 주행 누적(개입 무관) - decay 판정용
          if gas_pressed and not exclude_gas_learning:
            self._curve_override_gas_sec += _DT
          elif brake_pressed and not exclude_brake_learning:
            self._curve_override_brake_sec += _DT
            # Track peak deceleration in curve
            self._curve_max_decel = max(self._curve_max_decel, -a_ego)
            # 커브 제동 시 최저 속도(하한 도달 여부 판단용)
            self._curve_brake_min_v = min(self._curve_brake_min_v, v_ego_kph)

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

    # ── Phase 7: 정차/출발 데이터 수집 ───────────────────────────────
    # 자율 정차 접근(저속 + 자율 감속 + 가속페달 없음) 중 운전자 개입/정지 품질 수집
    if apply_long and engaged and not gear_park:
      approaching = (v_ego_kph <= _STOP_APPROACH_V_KPH and a_ego < _STOP_DECEL_THRESHOLD
                     and not gas_pressed)
      if approaching:
        self._stop_approaching = True

      if self._stop_approaching:
        # (a) 운전자 추가 제동 → 자율 정차가 약하거나 늦음
        if brake_pressed and not exclude_brake_learning:
          self._stop_brake_sec += _DT
        # (b) 운전자 가속 개입 → 자율 정차가 강하거나 이름
        elif gas_pressed and not exclude_gas_learning:
          self._stop_gas_sec += _DT
        # (c) 정차 직전 피크 저크(거친 정지) 추적
        if v_ego_kph < 8.0:
          jerk = abs(a_ego - self._prev_a_ego) / _DT
          self._stop_max_jerk = max(self._stop_max_jerk, jerk)
        # (d) 완전 정지 전이 감지 (직전 이동 → 정지)
        if v_ego_kph < _STOP_FULLSTOP_V_KPH and self._prev_v_kph >= _STOP_FULLSTOP_V_KPH:
          self._stop_events += 1
          if self._stop_max_jerk >= _STOP_HARSH_JERK:
            self._stop_harsh_count += 1
          if 0.0 < lead_drel < 80.0:
            self._stop_lead_gap_sum += lead_drel
            self._stop_lead_gap_count += 1
          self._stop_approaching = False
          self._stop_max_jerk = 0.0
    else:
      self._stop_approaching = False
      self._stop_max_jerk = 0.0

    # ── Phase 8: 종방향 PID 추종 데이터 수집 ─────────────────────────
    # 자율 가감속(운전자 페달 미개입) 중 지령가속도 대비 실측가속도 추종 오차 분석
    if (apply_long and engaged and not gear_park and not gas_pressed and not brake_pressed
        and v_ego_kph >= 20.0 and sm is not None and sm.alive.get('carControl', False)):
      try:
        cmd_accel = sm['carControl'].actuators.accel
        # 지연 보상 비교: 출력(cmd)은 계획을 action_t(=LongActuatorDelay+DT_MDL)만큼 앞서
        # 샘플한 값이므로, 현재 실측(a_ego)은 'action_t 전의 지령'과 비교해야 공정하다.
        # (구버전: cmd(t)-a_ego(t) → transient마다 선행분이 통째로 lag로 계산돼 Delay/Kf가
        #  무한 상향되는 양의 피드백(runaway 20→90) → 감지 순간 과제동 유발)
        if self._long_samples % 100 == 0:
          self._long_delay_units = max(0, self._params.get_int("LongActuatorDelay"))
        self._cmd_hist.append(float(cmd_accel))
        n_delay = min(len(self._cmd_hist) - 1,
                      int((self._long_delay_units * 0.01 + 0.05) / _DT + 0.5))
        long_err = self._cmd_hist[-1 - n_delay] - a_ego
        self._long_samples += 1
        self._long_err_sum += abs(long_err)
        if abs(long_err) >= _LONG_ERR_THRESHOLD:
          self._long_lag_count += 1   # 전체 lag(참고용)

        # 지령이 변하면 이후 잠시(HOLD)를 transient로 간주 — 실측이 따라붙는 정착 구간.
        if abs(cmd_accel - self._prev_cmd_accel) >= _LONG_CMD_CHANGE:
          self._long_trans_hold = _LONG_TRANS_HOLD_STEPS
        self._prev_cmd_accel = cmd_accel

        # 둔감(lag)은 transient(지령 변화 추종) 구간에서만 측정한다.
        # → 정속(정상상태)에서 남는 경사·게인 오프셋을 lag로 오인하지 않음.
        if self._long_trans_hold > 0:
          self._long_trans_samples += 1
          self._long_trans_err_sum += abs(long_err)
          if abs(long_err) >= _LONG_ERR_THRESHOLD:
            self._long_trans_lag += 1
          self._long_trans_hold -= 1

        # 진동(overshoot): 오차 부호가 반전하며 진폭이 큼 → 과반응
        if long_err * self._prev_long_err < 0 and abs(long_err) >= 0.3:
          self._long_overshoot_count += 1
        self._prev_long_err = long_err
      except Exception:
        pass
    else:
      # 수집 구간 단절(페달 개입/저속 등) → 이력 클리어로 stale 지연보상 비교 방지
      self._cmd_hist.clear()
      self._long_trans_hold = 0

    # ── Phase 9: 수동주행 기준분포 로거 ──────────────────────────────
    # openpilot 비인게이지(=사람이 직접 운전) 주행 중, 상황별 사람의 가감속·추종거리·
    # 페달상태를 통째로 누적한다. 핵심은 '무페달(코스팅)' 구간의 자연 감속을 측정해
    # 이 차/이 운전자의 회생제동 권한과 코스팅 선호를 직접 식별하는 것(역문제 없음).
    if apply_long and (not engaged) and not gear_park and v_ego_kph >= _MANUAL_MIN_V_KPH:
      band = _speed_band(v_ego_kph)
      if gas_pressed:
        # 사람이 선택한 가속(과도 가속은 학습 제외 기준 재사용)
        if not extreme_acceleration:
          self._manual_gas_accel_sum[band] += a_ego
          self._manual_gas_n[band] += 1
      elif brake_pressed:
        # 사람이 선택한 감속 크기(양수로 저장)
        self._manual_brake_decel_sum[band] += max(-a_ego, 0.0)
        self._manual_brake_n[band] += 1
      else:
        # 무페달 = 코스팅(자연 회생제동/엔진브레이크). 이 구간의 감속률을 측정.
        self._manual_coast_sec[band] += _DT
        if a_ego < 0.0:
          self._manual_coast_decel_sum[band] += -a_ego
          self._manual_coast_decel_n[band] += 1
      # 수동 추종 차간시간(time gap) 기준 분포 (선행차 존재 시)
      if 0.0 < lead_drel < _TFOLLOW_MAX_LEAD_DREL and v_ego_kph >= _TFOLLOW_MIN_V_KPH:
        v_ms = v_ego_kph / 3.6
        if v_ms > 1.0:
          self._manual_gap_sum += lead_drel / v_ms
          self._manual_gap_n += 1

    # 이전 프레임 상태 갱신 (Phase 4 swing 감지 + Phase 7 저크 계산 공용)
    self._prev_a_ego = a_ego
    self._prev_v_kph = v_ego_kph

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

  # ── Phase별 누적 데이터 리셋 헬퍼 (단일 출처) ──────────────────────
  # 과거 apply_recommendations()의 리셋 목록이 _clear_all_data()와 어긋나
  # understeer/inner_hugging·torque 카운터가 '적용' 후에도 남아, 분모(curve_overrides)만
  # 0이 되어 비율(understeer_ratio 등)이 오염되던 버그가 있었다.
  # 모든 리셋 경로를 아래 헬퍼로 일원화해 재발을 막는다.
  def _reset_phase1(self):
    """가속 (CruiseMaxVals0~6)"""
    self._gas_acc = [0.0] * _NUM_BANDS
    self._gas_dec_acc = [0.0] * _NUM_BANDS
    self._gas_dec_auto_acc = [0.0] * _NUM_BANDS
    self._band_sec = [0.0] * _NUM_BANDS
    self._gas_max_accel = [0.0] * _NUM_BANDS
    self._gas_max_pedal = [0.0] * _NUM_BANDS

  def _reset_phase2(self):
    """조향 (PathOffset/SteerActuatorDelay/SteerRatioRate + 토크 조향) — 방향 카운터 포함"""
    self._steer_acc = 0.0
    self._steer_count = 0
    self._lane_center_acc = 0.0
    self._lane_center_n = 0
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
    self._torque_osc_count = 0
    self._torque_steady_count = 0
    self._torque_straight_reversal = 0
    self._prev_torque_err_sign = 0
    self._prev_torque_straight_sign = 0

  def _reset_phase3(self):
    """JLeadFactor3 (수동/자율 제동)"""
    self._brake_count = 0
    self._brake_auto_count = 0
    self._jlead_gas_acc = 0.0
    self._jlead_sec = 0.0
    self._brake_max_decel = 0.0
    self._brake_min_ttc = 999.0
    self._prev_brake = False
    self._prev_auto_brake = False

  def _reset_phase4(self):
    """TFollowGap1~4 / TFollowSpeedFactor"""
    self._tfollow_gas_acc = [0.0] * 4
    self._tfollow_brake_acc = [0.0] * 4
    self._tfollow_brake_auto_acc = [0.0] * 4
    self._tfollow_speed_brake_acc = 0.0
    self._speed_factor_sec = 0.0
    self._tfollow_min_gap = [999.0] * 4

  def _reset_phase5(self):
    """DynamicTFollow / TFollowDecelBoost"""
    self._dyn_brake_count = 0
    self._decel_brake_count = 0
    self._dyn_sec = 0.0
    self._decel_sec = 0.0

  def _reset_phase6(self):
    """AutoCurveSpeedFactor (커브 감속)"""
    self._curve_override_gas_sec = 0.0
    self._curve_override_brake_sec = 0.0
    self._curve_override_brake_count = 0
    self._curve_max_decel = 0.0
    self._curve_steer_error_sec = 0.0
    self._curve_brake_min_v = 999.0
    self._curve_active_sec = 0.0

  def _reset_phase7(self):
    """정차/출발 (StoppingAccel/VEgoStopping/StopDistanceCarrot)"""
    self._stop_events = 0
    self._stop_brake_sec = 0.0
    self._stop_gas_sec = 0.0
    self._stop_harsh_count = 0
    self._stop_lead_gap_sum = 0.0
    self._stop_lead_gap_count = 0
    self._stop_approaching = False
    self._stop_max_jerk = 0.0

  def _reset_phase8(self):
    """종방향 PID (LongTuningKf/LongActuatorDelay/LongTuningKpV)"""
    self._long_samples = 0
    self._long_err_sum = 0.0
    self._long_lag_count = 0
    self._long_overshoot_count = 0
    self._prev_long_err = 0.0
    self._prev_cmd_accel = 0.0
    self._long_trans_hold = 0
    self._long_trans_samples = 0
    self._long_trans_lag = 0
    self._long_trans_err_sum = 0.0

  def _reset_phase9(self):
    """수동주행 기준분포 로거 (LongCoastBand)"""
    self._manual_coast_sec = [0.0] * _NUM_BANDS
    self._manual_coast_decel_sum = [0.0] * _NUM_BANDS
    self._manual_coast_decel_n = [0] * _NUM_BANDS
    self._manual_gas_accel_sum = [0.0] * _NUM_BANDS
    self._manual_gas_n = [0] * _NUM_BANDS
    self._manual_brake_decel_sum = [0.0] * _NUM_BANDS
    self._manual_brake_n = [0] * _NUM_BANDS
    self._manual_gap_sum = 0.0
    self._manual_gap_n = 0

  def _reset_all_phases(self):
    self._reset_phase1()
    self._reset_phase2()
    self._reset_phase3()
    self._reset_phase4()
    self._reset_phase5()
    self._reset_phase6()
    self._reset_phase7()
    self._reset_phase8()
    self._reset_phase9()

  def _clear_all_data(self):
    """모든 누적 데이터를 0으로 초기화하고 DB에서도 삭제"""
    self._reset_all_phases()
    self._params.remove("CarrotLearningData")
    self._params.remove("CarrotLearningRecommend")

  def _factory_reset(self):
    """공장초기화: 오토튜너가 변경할 수 있는 모든 파라미터를 설치 기본값으로
    일괄 복원하고, 누적 학습 데이터/추천도 모두 삭제한다.
    (오토튜닝 결과가 마음에 들지 않을 때 처음 상태로 되돌리는 용도)"""
    for key, val in _FACTORY_DEFAULTS.items():
      self._params.put_int(key, val)
    self._clear_all_data()
    self._params.remove("CarrotLearningRecommend")
    self._params.remove("CarrotLearningHistory")
    self._params.put_bool("CarrotLearningPopupReady", False)

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
      loaded_band = data.get("band_sec", [0.0] * _NUM_BANDS)
      if len(loaded_band) == _NUM_BANDS:
        self._band_sec = [float(x) for x in loaded_band]
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
      self._torque_osc_count = int(lat.get("torque_osc_count", 0))
      self._torque_steady_count = int(lat.get("torque_steady_count", 0))
      self._torque_straight_reversal = int(lat.get("torque_straight_reversal", 0))
      # Phase 3
      lon = data.get("lon", {})
      self._brake_count = int(lon.get("brake_count", 0))
      self._jlead_gas_acc = float(lon.get("jlead_gas_acc", 0.0))
      self._jlead_sec = float(lon.get("jlead_sec", 0.0))
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
      self._speed_factor_sec = float(data.get("speed_factor_sec", 0.0))
      self._brake_auto_count = int(data.get("brake_auto_count", 0))
      # Phase 5
      p5 = data.get("phase5", {})
      self._dyn_brake_count = int(p5.get("dyn_brake_count", 0))
      self._decel_brake_count = int(p5.get("decel_brake_count", 0))
      self._dyn_sec = float(p5.get("dyn_sec", 0.0))
      self._decel_sec = float(p5.get("decel_sec", 0.0))
      # Phase 6
      p6 = data.get("phase6", {})
      self._curve_override_gas_sec = float(p6.get("curve_override_gas_sec", 0.0))
      self._curve_override_brake_sec = float(p6.get("curve_override_brake_sec", 0.0))
      self._curve_override_brake_count = int(p6.get("curve_override_brake_count", 0))
      self._curve_max_decel = float(p6.get("curve_max_decel", 0.0))
      self._curve_steer_error_sec = float(p6.get("curve_steer_error_sec", 0.0))
      self._curve_brake_min_v = float(p6.get("curve_brake_min_v", 999.0))
      self._curve_active_sec = float(p6.get("curve_active_sec", 0.0))
      # Phase 7
      p7 = data.get("phase7", {})
      self._stop_events = int(p7.get("stop_events", 0))
      self._stop_brake_sec = float(p7.get("stop_brake_sec", 0.0))
      self._stop_gas_sec = float(p7.get("stop_gas_sec", 0.0))
      self._stop_harsh_count = int(p7.get("stop_harsh_count", 0))
      self._stop_lead_gap_sum = float(p7.get("stop_lead_gap_sum", 0.0))
      self._stop_lead_gap_count = int(p7.get("stop_lead_gap_count", 0))
      # Phase 8
      p8 = data.get("phase8", {})
      self._long_samples = int(p8.get("long_samples", 0))
      self._long_err_sum = float(p8.get("long_err_sum", 0.0))
      self._long_lag_count = int(p8.get("long_lag_count", 0))
      self._long_overshoot_count = int(p8.get("long_overshoot_count", 0))
      self._long_trans_samples = int(p8.get("long_trans_samples", 0))
      self._long_trans_lag = int(p8.get("long_trans_lag", 0))
      self._long_trans_err_sum = float(p8.get("long_trans_err_sum", 0.0))
      # Phase 9 (밴드별 리스트는 길이 검증 후 복원)
      p9 = data.get("phase9", {})
      def _load_band_list(key, cast, default):
        v = p9.get(key, None)
        if isinstance(v, list) and len(v) == _NUM_BANDS:
          return [cast(x) for x in v]
        return [default] * _NUM_BANDS
      self._manual_coast_sec = _load_band_list("manual_coast_sec", float, 0.0)
      self._manual_coast_decel_sum = _load_band_list("manual_coast_decel_sum", float, 0.0)
      self._manual_coast_decel_n = _load_band_list("manual_coast_decel_n", int, 0)
      self._manual_gas_accel_sum = _load_band_list("manual_gas_accel_sum", float, 0.0)
      self._manual_gas_n = _load_band_list("manual_gas_n", int, 0)
      self._manual_brake_decel_sum = _load_band_list("manual_brake_decel_sum", float, 0.0)
      self._manual_brake_n = _load_band_list("manual_brake_n", int, 0)
      self._manual_gap_sum = float(p9.get("manual_gap_sum", 0.0))
      self._manual_gap_n = int(p9.get("manual_gap_n", 0))

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
      "band_sec": self._band_sec,
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
        "torque_osc_count": self._torque_osc_count,
        "torque_steady_count": self._torque_steady_count,
        "torque_straight_reversal": self._torque_straight_reversal,
      },
      "lon": {
        "brake_count": self._brake_count,
        "jlead_gas_acc": self._jlead_gas_acc,
        "jlead_sec": self._jlead_sec,
      },
      "tfollow_gas_acc": self._tfollow_gas_acc,
      "tfollow_brake_acc": self._tfollow_brake_acc,
      "tfollow_brake_auto_acc": self._tfollow_brake_auto_acc,
      "tfollow_speed_brake_acc": self._tfollow_speed_brake_acc,
      "speed_factor_sec": self._speed_factor_sec,
      "gas_dec_auto_acc": self._gas_dec_auto_acc,
      "brake_auto_count": self._brake_auto_count,
      "phase5": {
        "dyn_brake_count": self._dyn_brake_count,
        "decel_brake_count": self._decel_brake_count,
        "dyn_sec": self._dyn_sec,
        "decel_sec": self._decel_sec,
      },
      "phase6": {
        "curve_override_gas_sec": self._curve_override_gas_sec,
        "curve_override_brake_sec": self._curve_override_brake_sec,
        "curve_override_brake_count": self._curve_override_brake_count,
        "curve_max_decel": self._curve_max_decel,
        "curve_steer_error_sec": self._curve_steer_error_sec,
        "curve_brake_min_v": self._curve_brake_min_v,
        "curve_active_sec": self._curve_active_sec,
      },
      "phase7": {
        "stop_events": self._stop_events,
        "stop_brake_sec": self._stop_brake_sec,
        "stop_gas_sec": self._stop_gas_sec,
        "stop_harsh_count": self._stop_harsh_count,
        "stop_lead_gap_sum": self._stop_lead_gap_sum,
        "stop_lead_gap_count": self._stop_lead_gap_count,
      },
      "phase8": {
        "long_samples": self._long_samples,
        "long_err_sum": self._long_err_sum,
        "long_lag_count": self._long_lag_count,
        "long_overshoot_count": self._long_overshoot_count,
        "long_trans_samples": self._long_trans_samples,
        "long_trans_lag": self._long_trans_lag,
        "long_trans_err_sum": self._long_trans_err_sum,
      },
      "phase9": {
        "manual_coast_sec": self._manual_coast_sec,
        "manual_coast_decel_sum": self._manual_coast_decel_sum,
        "manual_coast_decel_n": self._manual_coast_decel_n,
        "manual_gas_accel_sum": self._manual_gas_accel_sum,
        "manual_gas_n": self._manual_gas_n,
        "manual_brake_decel_sum": self._manual_brake_decel_sum,
        "manual_brake_n": self._manual_brake_n,
        "manual_gap_sum": self._manual_gap_sum,
        "manual_gap_n": self._manual_gap_n,
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
    apply_lat = self._params.get_bool("CarrotTunerApplyLat") if self._params.get("CarrotTunerApplyLat") is not None else True
    apply_long = self._params.get_bool("CarrotTunerApplyLong") if self._params.get("CarrotTunerApplyLong") is not None else True

    result = {
      "가속 (Acceleration)": {},
      "조향 (Steering)": {},
      "주행 (Driving)": {},
      "거리 (Following Distance)": {},
      "동적제어 (Dynamic Control)": {},
    }

    # ── Phase 1: CruiseMaxVals ──────────────────────────────────────
    drive_mode = self._params.get_int("MyDrivingMode") # 1: ECO, 2: SAFE, 3: NORMAL, 4: HIGH
    if apply_long:
      for i, acc_sec in enumerate(self._gas_acc):
        key = _ACCEL_KEYS[i]
        current_raw = self._params.get_int(key)
        if current_raw <= 0: continue

        # 드라이브 모드별 동적 가속 제한 상한값 설정 (더 조밀하고 안전하게 제한)
        max_limit = 200
        if key == "CruiseMaxVals0":   # 0~10 km/h
          max_limit = 220
        elif key == "CruiseMaxVals1": # 10~40 km/h
          if drive_mode in (1, 2):    # ECO, SAFE
            max_limit = 170
          elif drive_mode == 3:       # NORMAL
            max_limit = 190
          elif drive_mode == 4:       # HIGH
            max_limit = 220
        elif key == "CruiseMaxVals2": # 40~60 km/h
          if drive_mode in (1, 2):    # ECO, SAFE
            max_limit = 140
          elif drive_mode in (3, 4):  # NORMAL, HIGH
            max_limit = 150
        elif key == "CruiseMaxVals3": # 60~80 km/h
          if drive_mode in (1, 2):    # ECO, SAFE
            max_limit = 110
          elif drive_mode in (3, 4):  # NORMAL, HIGH
            max_limit = 120
        elif key == "CruiseMaxVals4": # 80~110 km/h
          max_limit = 100
        elif key == "CruiseMaxVals5": # 110~140 km/h
          max_limit = 80
        elif key == "CruiseMaxVals6": # 140~ km/h
          max_limit = 60

        total_dec = self._gas_dec_acc[i] + self._gas_dec_auto_acc[i]
        
        # [상호 억제 로직: Accel Penalty Discount]
        # 가속 중 '실제 제동'(gas_dec_acc — 선행차 추종 제동은 수집 단계에서 제외됨)이
        # 있었다면 가속이 굼떴다는 신호(gas help)를 그만큼 깎는다.
        # (과거: 선행차 추종 제동까지 포함한 brake_count로 깎아, 선행차 급감속 때문에
        #  밟은 브레이크가 가속 한계를 잠식하던 신호 오귀속 버그 → gas_dec_acc[i] 기반으로 분리)
        dampened_acc_sec = max(0.0, acc_sec - self._gas_dec_acc[i])

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
        elif total_dec >= _GAS_REDUCE_THRESHOLD_SEC or is_auto_surging:
          # 하향 신호는 가속-중-제동(gas_dec_acc, 선행차 제외) + 자율 급가속(gas_dec_auto_acc)
          # + 가감속 요동(auto-surging)만 사용. 선행차 추종 제동은 더 이상 가속을 깎지 않는다.
          recommended_raw = max(50, int(current_raw * (1.0 + _GAS_REDUCE_RATIO)))
          recommended_raw = min(max_limit, recommended_raw)
          if is_auto_surging:
            reason = "excessive auto-surging penalty"
            sec = self._gas_dec_auto_acc[i] + self._brake_auto_count
          else:
            reason = "aggressive accel (auto)" if self._gas_dec_auto_acc[i] > self._gas_dec_acc[i] else "too aggressive (manual)"
            sec = total_dec
        elif (i in _LOWBAND_DECAY_BANDS
              and self._band_sec[i] >= _LOWBAND_DECAY_MIN_SEC
              and acc_sec < _LOWBAND_DECAY_GAS_DEADBAND
              and current_raw > _FACTORY_DEFAULTS.get(key, current_raw)):
          # 저속 밴드(0/1) 양방향 균형: 그 속도대를 충분히 달렸는데 가속요청이 없으면
          # 기본값 쪽으로 약하게 하향(과거 단방향 상승 보완). 기본값 미만으로는 내리지 않음.
          default_raw = _FACTORY_DEFAULTS[key]
          recommended_raw = max(default_raw, current_raw - _LOWBAND_DECAY_STEP)
          reason = f"low-speed relax (no gas request, default {default_raw})"
          sec = round(self._band_sec[i], 1)
        else:
          continue

        if recommended_raw != current_raw:
          # Max Delta Cap: 1회 적용 시 최대 변동폭을 ±15로 제한
          delta = recommended_raw - current_raw
          if delta > 15:
            recommended_raw = current_raw + 15
          elif delta < -15:
            recommended_raw = current_raw - 15

          result["가속 (Acceleration)"][key] = {
            "current": current_raw,
            "recommended": recommended_raw,
            "band_kph": f"{_BP_KPH[i]}~{_BP_KPH[i+1] if i+1 < _NUM_BANDS else '∞'} km/h ({reason})",
            "acc_sec": round(sec, 1),
          }

    # ── Phase 2: 조향 패턴 추천 ─────────────────────────────────────
    if apply_lat and self.is_angle_control:
      # ── [A] 앵글 조향 차량 튜닝 ────────────────────────────────────
      # Phase 2a: PathOffset (차선중심 편차) — 신호를 조향각→차선중심 편차(m)로 교체.
      # 조향각 평균은 PathOffset으로 바뀌지 않아 수렴 불가(runaway ±150, 차선 쏠림 유발).
      if self._lane_center_n >= _LATERAL_MIN_SAMPLES:
        # avg_lc = 차선중심의 y좌표(모델 프레임, ego=0). path.y와 같은 좌표계이므로
        # 경로를 avg_lc 방향으로 그만큼 옮기면 차가 차선중심에 수렴한다(프레임 무관).
        avg_lc = self._lane_center_acc / self._lane_center_n
        if abs(avg_lc) >= _PATH_OFFSET_LC_MIN_M:
          current_offset = self._params.get_int("PathOffset")
          delta = int(np.clip(avg_lc * 100.0 * _PATH_OFFSET_LC_GAIN,
                              -_PATH_OFFSET_STEP_MAX, _PATH_OFFSET_STEP_MAX))
          recommended = int(np.clip(current_offset + delta,
                                    -_PATH_OFFSET_ABS_MAX, _PATH_OFFSET_ABS_MAX))
          if recommended != current_offset:
            result["조향 (Steering)"]["PathOffset"] = {
              "current": current_offset,
              "recommended": recommended,
              "band_kph": "차선중심 편차 보정",
              "avg_m": round(avg_lc, 2),
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
        # 0 = liveDelay(자동, ~0.2s). 학습 조정 시 기준값으로 환산.
        base_delay = current_delay if current_delay > 0 else _SAD_AUTO_BASELINE
        recommended_delay = current_delay
        if override_ratio >= 0.30:
          if understeer_ratio >= 0.60:
            recommended_delay = min(_SAD_LEARN_MAX, base_delay + _DELAY_STEP_UNIT)
          elif inner_hugging_ratio >= 0.60:
            recommended_delay = max(_SAD_LEARN_MIN, base_delay - _DELAY_STEP_UNIT)
        # 안정 상태(개입 적음)에서는 값을 변경하지 않는다.
        # (과거: max(50, current-10)로 인해 초기화 직후 안정 주행이면 무조건 50으로 튀던 버그)

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
        elif override_ratio < 0.15 and current_sr_rate > 100:
          # 개입이 적은 안정 상태 → 과거 '강화분'(>100)만 기본값(100) 쪽으로 소폭 환원.
          # (과거: 90까지 내려 기본값보다 약한 조향=언더스티어를 유발 → 커브 바깥 쏠림 악화.
          #  inner_hugging 보정으로 100 미만이 된 경우는 정당한 약화이므로 건드리지 않는다.)
          recommended_sr = max(100, current_sr_rate - 2)
          
        if recommended_sr != current_sr_rate:
          result["조향 (Steering)"]["SteerRatioRate"] = {
            "current": current_sr_rate,
            "recommended": recommended_sr,
            "band_kph": "커브 강한 개입 대응 (조향 강화)" if (override_ratio >= 0.40 and understeer_ratio >= 0.60) else ("커브 안쪽 쏠림 개입 대응 (조향 감쇄)" if (override_ratio >= 0.40 and inner_hugging_ratio >= 0.60) else "부드러운 조향 감속 보정"),
            "override_ratio": round(override_ratio * 100, 1),
          }
    
    elif apply_lat:
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
        # 0 = liveDelay(자동, ~0.2s). 학습 조정 시 기준값으로 환산.
        base_delay = current_delay if current_delay > 0 else _SAD_AUTO_BASELINE
        recommended_delay = current_delay
        if override_ratio >= 0.30:
          if understeer_ratio >= 0.60:
            recommended_delay = min(_SAD_LEARN_MAX, base_delay + _DELAY_STEP_UNIT)
          elif inner_hugging_ratio >= 0.60:
            recommended_delay = max(_SAD_LEARN_MIN, base_delay - _DELAY_STEP_UNIT)
        # 안정 상태(개입 적음)에서는 값을 변경하지 않는다(50으로 튀던 버그 제거).

        if recommended_delay != current_delay:
          result["조향 (Steering)"]["SteerActuatorDelay"] = {
            "current": current_delay,
            "recommended": recommended_delay,
            "band_kph": "토크 커브 진입 지연 보정" if (override_ratio >= 0.30 and understeer_ratio >= 0.60) else ("토크 커브 안쪽 쏠림 보정" if (override_ratio >= 0.30 and inner_hugging_ratio >= 0.60) else "토크 커브 안정화 감쇄"),
            "override_ratio": round(override_ratio * 100, 1),
          }

        # 2. LateralTorqueAccelFactor & LateralTorqueKf (횡가속도 비례 피드포워드)
        # ※ 방향성 주의(문서2): 토크 = 목표횡가속도 / AccelFactor + friction
        #    AccelFactor는 분모 → 값↑ = 토크↓ (조향 약화), 값↓ = 토크↑ (조향 강화)
        #    Kf는 피드포워드 게인 → 값↑ = 토크↑ (조향 강화)
        #    understeer(OP가 덜 꺾어 운전자가 더 꺾음) → 토크 강화: factor↓ + Kf↑
        #    inner_hugging(OP가 과하게 꺾어 운전자가 풀어줌) → 토크 약화: factor↑ + Kf↓
        current_factor = self._params.get_int("LateralTorqueAccelFactor")
        current_kf = self._params.get_int("LateralTorqueKf")

        recommended_factor = current_factor
        recommended_kf = current_kf
        steer_dir = ""

        if override_ratio >= 0.40 and understeer_ratio >= 0.60:
          # 조향 부족 → 토크 강화 (factor↓, Kf↑)
          recommended_factor = _clamp_spec("LateralTorqueAccelFactor", current_factor - 100)
          recommended_kf = _clamp_spec("LateralTorqueKf", current_kf + 3)
          steer_dir = "understeer"
        elif override_ratio >= 0.40 and inner_hugging_ratio >= 0.60:
          # 안쪽 쏠림 → 토크 약화 (factor↑, Kf↓)
          recommended_factor = _clamp_spec("LateralTorqueAccelFactor", current_factor + 100)
          recommended_kf = _clamp_spec("LateralTorqueKf", current_kf - 3)
          steer_dir = "inner_hugging"
        elif override_ratio < 0.15:
          # 개입이 거의 없는 안정 상태 → Kf를 0까지 무한 감쇄하지 않고
          # 공장기본값(100)쪽으로만 약하게 수렴(양방향 균형). FF를 0으로 깎아
          # Kp 의존(진동 영역)으로 끌고 가던 단방향 drift 제거.
          kf_default = _FACTORY_DEFAULTS["LateralTorqueKf"]
          recommended_kf = _clamp_spec("LateralTorqueKf", _decay_toward(current_kf, kf_default, 1))
          steer_dir = "stable"

        if recommended_factor != current_factor:
          result["조향 (Steering)"]["LateralTorqueAccelFactor"] = {
            "current": current_factor,
            "recommended": recommended_factor,
            "band_kph": "토크 커브 조향 강화 (선회력↑)" if steer_dir == "understeer" else "토크 커브 안쪽 쏠림 완화 (선회력↓)",
            "override_ratio": round(override_ratio * 100, 1),
          }
        if recommended_kf != current_kf:
          result["조향 (Steering)"]["LateralTorqueKf"] = {
            "current": current_kf,
            "recommended": recommended_kf,
            "band_kph": "토크 커브 피드포워드 강화" if steer_dir == "understeer" else ("토크 피드포워드 약화" if steer_dir == "inner_hugging" else "토크 피드포워드 안정화 감쇄"),
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

      # 4. LateralTorqueKiV / LateralTorqueKpV (피드백 게인) ─ 진동 vs 정상상태 lag 구분
      #    핵심 개선: 추종오차의 '크기'가 아니라 '부호 패턴'으로 비례게인 방향을 정한다.
      #    (구버전: error_count≥50이면 무조건 KpV 상향 → 진동이 만든 오차도 '게인부족'으로
      #     오인해 KpV를 계속 올림 → 고속 직선 핑퐁/커브 탈출 진동 = 발산 양의 피드백)
      #    - 진동(부호반전 多)·고속 직선 핑퐁 → Kp 과다 → KpV/KiV 하향(default 아래도 허용)
      #    - 정상상태 lag(부호 일관) → FF(Kf) 강화 우선, Kp는 Kf 충분할 때만 보조 상향
      #    - 추종 양호 → 공장기본값으로 약하게 수렴(양방향 균형)
      current_kiv = self._params.get_int("LateralTorqueKiV")
      current_kpv = self._params.get_int("LateralTorqueKpV")
      current_kf2 = self._params.get_int("LateralTorqueKf")  # 블록 독립적으로 재조회(스코프 안전)
      kiv_default = _FACTORY_DEFAULTS["LateralTorqueKiV"]   # 10
      kpv_default = _FACTORY_DEFAULTS["LateralTorqueKpV"]   # 100
      recommended_kiv = current_kiv
      recommended_kpv = current_kpv
      kp_dir = ""

      curve_osc_dominant = (self._torque_osc_count >= _TORQUE_OSC_MIN
                            and self._torque_osc_count > self._torque_steady_count)
      straight_pingpong = self._torque_straight_reversal >= _TORQUE_PINGPONG_MIN_COUNT

      if straight_pingpong or curve_osc_dominant:
        # 진동/핑퐁 → 비례·적분 게인 하향. default(100) 아래로도 수렴 허용(소유주 최적 KpV=70).
        recommended_kpv = max(_TORQUE_KPV_MIN, current_kpv - _TORQUE_KPV_STEP)
        recommended_kiv = max(_TORQUE_KIV_MIN, current_kiv - 1)
        kp_dir = "oscillation"
      elif self._torque_steady_count >= _TORQUE_STEADY_MIN and self._torque_osc_count < _TORQUE_OSC_MIN:
        # 정상상태 lag(부호 일관): Kf(FF) 강화는 위 understeer 로직이 담당.
        # Kf가 이미 충분(≥기준)할 때만 잔차 보정용으로 Kp를 소폭 상향.
        if current_kf2 >= _TORQUE_KF_FF_PREF:
          recommended_kpv = min(_TORQUE_KPV_MAX, current_kpv + _TORQUE_KPV_STEP)
          recommended_kiv = min(_TORQUE_KIV_MAX, current_kiv + 1)
        kp_dir = "steady_lag"
      elif self._torque_curve_entries >= 100 and self._torque_error_count < 20:
        # 충분한 커브 주행에도 추종오차가 적음(양호) → 기본값 쪽으로 약하게 수렴
        recommended_kpv = _decay_toward(current_kpv, kpv_default, 2)
        recommended_kiv = _decay_toward(current_kiv, kiv_default, 1)
        kp_dir = "good"

      _KP_MSG = {
        "oscillation": "고속 핑퐁/진동 감지 → 비례게인 하향",
        "steady_lag": "정상상태 추종지연 → 비례게인 보조 상향",
        "good": "조향 추종 양호 → 비례게인 기본값 수렴",
      }
      _KI_MSG = {
        "oscillation": "진동 억제 → 적분게인 하향",
        "steady_lag": "정상상태 오차 → 적분게인 보조 상향",
        "good": "조향 추종 양호 → 적분게인 기본값 수렴",
      }
      if recommended_kpv != current_kpv:
        result["조향 (Steering)"]["LateralTorqueKpV"] = {
          "current": current_kpv,
          "recommended": recommended_kpv,
          "band_kph": _KP_MSG.get(kp_dir, ""),
          "error_ticks": self._torque_error_count,
          "osc_ticks": self._torque_osc_count + self._torque_straight_reversal,
        }
      if recommended_kiv != current_kiv:
        result["조향 (Steering)"]["LateralTorqueKiV"] = {
          "current": current_kiv,
          "recommended": recommended_kiv,
          "band_kph": _KI_MSG.get(kp_dir, ""),
          "error_ticks": self._torque_error_count,
        }

    # ── Phase 3: JLeadFactor3 (수동 제동) ───────────────────────────
    jlead_candidate = None
    if apply_long:
      total_brake = self._brake_count + self._brake_auto_count
      # '늦은 제동'으로 판단되는 경우에만 상향. (정상적인 여유 제동의 단방향 누적 방지)
      #  - 자율 급제동(brake_auto_count): 시스템이 늦게 스스로 급제동 = 명백한 지연 신호
      #  - 수동 제동이라도 TTC가 충분히 낮을 때만(늦은 시점) 인정
      late_braking = (self._brake_auto_count >= 3) or (self._brake_min_ttc < _JLEAD_LATE_TTC)
      # 선제적 '교육용' 제동: 위험할 만큼 늦지는 않지만(TTC 3.5~6s) 선행차에 접근하며
      # 운전자가 직접, 약하지 않은 감속(≥1.0m/s^2)으로 반복 제동 = '시스템 반응이 굼뜨다'를
      # 가르치는 신호. (과거: TTC<3.5의 위험한 늦은 제동만 인정해 이 교육이 무시됐음)
      proactive_braking = (not late_braking
                           and _JLEAD_LATE_TTC <= self._brake_min_ttc < _JLEAD_PROACTIVE_TTC
                           and self._brake_max_decel >= _JLEAD_PROACTIVE_DECEL)
      if total_brake >= _BRAKE_MIN_COUNT and (late_braking or proactive_braking):
        current_jlead = self._params.get_int("JLeadFactor3")

        if late_braking:
          # TTC와 감속량을 반영한 동적 증가 계산
          ttc_factor = float(np.clip((4.5 - self._brake_min_ttc) / 2.0, 0.0, 1.0))
          decel_factor = float(np.clip((self._brake_max_decel - 0.8) / 1.0, 0.0, 1.0))
          dynamic_step = int(10 + 25 * max(ttc_factor, decel_factor))
          # Max Delta Cap: 1회당 변화폭을 최대 15로 제한
          dynamic_step = min(15, dynamic_step)
          reason = "late braking (auto)" if self._brake_auto_count > self._brake_count else "approaching lead (manual)"
        else:
          # 선제 교육 제동은 약하게(+5)만 반영 → 정상 여유 제동의 과도 누적 방지
          dynamic_step = _JLEAD_PROACTIVE_STEP
          reason = "proactive braking (manual)"

        recommended = min(_JLEAD_MAX_RECO, current_jlead + dynamic_step) # 상한 80 -> 20 (runaway 방지)
        if recommended != current_jlead:
          jlead_candidate = {
            "current": current_jlead,
            "recommended": recommended,
            "band_kph": f"{reason} (TTC min {self._brake_min_ttc:.1f}s, step +{dynamic_step})",
            "brake_count": round(total_brake, 1),
            "_signal": total_brake,
          }
      elif self._jlead_gas_acc >= _JLEAD_GAS_THRESHOLD_SEC:
        current_jlead = self._params.get_int("JLeadFactor3")
        recommended = max(0, current_jlead + _JLEAD_REDUCE_STEP)
        recommended = min(_JLEAD_MAX_RECO, recommended) # 상한 20 적용
        if recommended != current_jlead:
          jlead_candidate = {
            "current": current_jlead,
            "recommended": recommended,
            "band_kph": "too aggressive (gas override)",
            "gas_sec": round(self._jlead_gas_acc, 1),
            "_signal": self._jlead_gas_acc,
          }
      elif self._jlead_sec >= 180.0 and total_brake < 2 and self._jlead_gas_acc < 1.0:
        current_jlead = self._params.get_int("JLeadFactor3")
        default_jlead = _FACTORY_DEFAULTS["JLeadFactor3"] # 0
        if current_jlead > default_jlead:
          recommended = max(default_jlead, current_jlead - 5)
          if recommended != current_jlead:
            jlead_candidate = {
              "current": current_jlead,
              "recommended": recommended,
              "band_kph": "jlead decay (no late braking, default 0)",
              "sec": round(self._jlead_sec, 1),
              "_signal": 0.1,
            }

    # ── Phase 4: TFollowGap (선행차 추종 중 거리 좁히기 가속 개입) ──
    # EnableSpeedTF<0(속도 앵커 모드)에선 GAP1~4가 '버튼 단계'가 아니라 '속도 앵커'이므로
    # 현재 버튼 gap을 학습하는 이 로직이 의미상 맞지 않는다(예: 고속 추종은 gap3/4가 지배하는데
    # 현재 버튼 gap2만 축소됨). 이 경우 자동학습을 끄고 사용자가 앵커값을 직접 설정하도록 둔다.
    if apply_long and self._params.get_int("EnableSpeedTF") >= 0:
      for i, gas_sec in enumerate(self._tfollow_gas_acc):
        key = _TFOLLOW_KEYS[i]
        name = _TFOLLOW_NAMES[i]
        current_val = self._params.get_int(key)
        if current_val <= 0: continue

        total_dec = self._tfollow_brake_acc[i] + self._tfollow_brake_auto_acc[i]
        
        recommended_val = current_val
        if gas_sec >= _TFOLLOW_GAS_THRESHOLD_SEC:
          # 실제 차간 거리 오차에 기반한 동적 감소 계산.
          # 운전자가 현재 설정보다 실제로 더 바짝(gap_diff>MIN) 따라간 근거가 있을 때만 축소한다.
          # min_gap이 설정 이상(gap_diff<=MIN)이면 '더 좁히려는' 근거가 없으므로 미조정
          # (과거: 근거 없이도 매번 -5 축소 → 120으로 재설정해도 계속 줄어드는 폭주 버그).
          target_val = int(self._tfollow_min_gap[i] * 100)
          gap_diff = current_val - target_val
          if gap_diff <= _TFOLLOW_GAP_DIFF_MIN:
            continue
          dynamic_step = float(np.clip(int(gap_diff * 0.5), 5, 25))
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
    if apply_long and self._tfollow_speed_brake_acc >= _TFOLLOW_SPEED_FACTOR_THRESHOLD_SEC:
      current_sf = self._params.get_int("TFollowSpeedFactor")
      recommended_sf = min(100, current_sf + _TFOLLOW_SPEED_FACTOR_STEP)
      if recommended_sf != current_sf:
        result["거리 (Following Distance)"]["TFollowSpeedFactor"] = {
          "current": current_sf,
          "recommended": recommended_sf,
          "band_kph": "high-speed safety (>80km/h)",
          "sec": round(self._tfollow_speed_brake_acc, 1),
        }
    elif apply_long and self._speed_factor_sec >= 180.0 and self._tfollow_speed_brake_acc < 1.0:
      current_sf = self._params.get_int("TFollowSpeedFactor")
      default_sf = _FACTORY_DEFAULTS["TFollowSpeedFactor"]
      if current_sf > default_sf:
        recommended_sf = max(default_sf, current_sf - 5)
        if recommended_sf != current_sf:
          result["거리 (Following Distance)"]["TFollowSpeedFactor"] = {
            "current": current_sf,
            "recommended": recommended_sf,
            "band_kph": "high-speed decay (no braking, default 0)",
            "sec": round(self._speed_factor_sec, 1),
          }

    # ── Phase 5a: DynamicTFollow (앞차 급감속 반응 민감도) ───────────
    dyn_candidate = None
    if apply_long and self._dyn_brake_count >= _DYN_TFOLLOW_BRAKE_MIN:
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
    elif apply_long and self._dyn_sec >= 180.0 and self._dyn_brake_count < 1:
      current_dyn = self._params.get_int("DynamicTFollow")
      default_dyn = _FACTORY_DEFAULTS["DynamicTFollow"]
      if current_dyn > default_dyn:
        recommended_dyn = max(default_dyn, current_dyn - 3)
        if recommended_dyn != current_dyn:
          dyn_candidate = {
            "current": current_dyn,
            "recommended": recommended_dyn,
            "band_kph": "dyn decay (no lead decel, default 0)",
            "brake_count": self._dyn_brake_count,
            "_signal": 0.1,
          }

    # ── Phase 5b: TFollowDecelBoost (내 차 감속 중 버퍼 확보) ────────
    boost_candidate = None
    if apply_long and self._decel_brake_count >= _DECEL_BOOST_BRAKE_MIN:
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
    elif apply_long and self._decel_sec >= 180.0 and self._decel_brake_count < 1:
      current_boost = self._params.get_int("TFollowDecelBoost")
      default_boost = _FACTORY_DEFAULTS["TFollowDecelBoost"]
      if current_boost > default_boost:
        recommended_boost = max(default_boost, current_boost - 3)
        if recommended_boost != current_boost:
          boost_candidate = {
            "current": current_boost,
            "recommended": recommended_boost,
            "band_kph": "boost decay (no decel override, default 10)",
            "brake_count": self._decel_brake_count,
            "_signal": 0.1,
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

    # ── Phase 6: Curve Speed (AutoCurveSpeedFactor) ─────────────────
    # 실제 커브 감속 knob은 AutoCurveSpeedFactor (carrot_man.vturn_speed에서 사용).
    #   값↑ = 곡률을 더 민감하게 인식 → 커브 목표속도↓ → 커브 시작 전 더 일찍/충분히 감속.
    # (과거: 미사용 파라미터 AutoCurveSpeedAggressiveness를 조정해 실주행 효과가 없었음)
    key = "AutoCurveSpeedFactor"
    current_raw = self._cur_or_default(key)

    recommended_raw = current_raw
    reason = ""
    sec = 0.0

    # 하한(floor) 도달 여부: 커브에서 이미 하한속도까지 내려간 상태로 제동했다면
    # AutoCurveSpeedFactor를 올려도 목표속도가 더 낮아지지 못해(=floor 클램프) 효과가 없다.
    # → 이 경우 factor 상향 추천을 생략해 '헛도는 학습'(상한까지 무의미한 누적)을 방지.
    lower_limit = self._params.get_int("AutoCurveSpeedLowerLimit")
    if lower_limit <= 0:
      lower_limit = 30
    floor_bound = (self._curve_brake_min_v <= lower_limit + 3)

    has_brake_signal = (self._curve_override_brake_count >= 3 or self._curve_override_brake_sec >= 5.0)
    # (A) 조향 추적오차는 더 이상 곡선감속 상향에 쓰지 않는다.
    #     커브의 조향오차는 언더스티어/횡방향 튜닝 문제(SteerActuatorDelay/SteerRatioRate가 담당)이지
    #     '감속 부족'이 아니다. 과거엔 이걸 factor↑로 오인해 상한까지 런어웨이(과감속)했음.
    default_factor = _PARAM_SPEC["AutoCurveSpeedFactor"]["default"]   # 120
    # (B) 편안한 커브: 충분히 커브를 달렸는데 제동/가속 개입이 거의 없고 감속도도 낮음
    comfortable = (self._curve_active_sec >= 30.0
                   and not has_brake_signal
                   and self._curve_override_gas_sec < 5.0
                   and self._curve_max_decel < 1.5)

    # 커브에서 제동 개입 = 감속 부족 → Factor 상향(더 일찍 더 감속)
    if apply_long and has_brake_signal and not floor_bound:
      recommended_raw = _clamp_spec(key, current_raw + 10)
      reason = f"brake in curve (count {self._curve_override_brake_count}, peak decel {self._curve_max_decel:.2f}m/s^2)"
      sec = self._curve_override_brake_sec
    # floor-bound(이미 하한속도에서 제동)면 factor 무효 → 추천 생략(헛도는 학습 방지)
    # (C) 커브에서 가속 개입 = 과도한 감속 → Factor 하향(덜 감속). 임계 10s→5s로 완화.
    #     단, 커브 조향 추적오차(언더스티어)가 누적된 상태(≥5s)면 '더 빠른 커브 진입'은
    #     바깥 쏠림을 악화시키므로 하향을 보류한다(횡방향 튜닝이 먼저 해결되어야 함).
    elif apply_long and self._curve_override_gas_sec >= 5.0 and self._curve_steer_error_sec < 5.0:
      recommended_raw = _clamp_spec(key, current_raw - 10)
      reason = f"gas in curve (acc {self._curve_override_gas_sec:.1f}s)"
      sec = self._curve_override_gas_sec
    # (B) 개입 없이 편안히 통과 + factor가 기본값보다 높으면 → default 쪽으로 소폭 자동 회복(decay)
    elif apply_long and comfortable and current_raw > default_factor:
      recommended_raw = _clamp_spec(key, max(default_factor, current_raw - 5))
      reason = f"comfortable curves, decay toward default ({self._curve_active_sec:.0f}s)"
      sec = self._curve_active_sec

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

    # ── Phase 7: 정차/출발 (StoppingAccel / VEgoStopping / StopDistanceCarrot) ──
    if apply_long and self._stop_events >= _STOP_MIN_EVENTS:
      brake_per = self._stop_brake_sec / self._stop_events
      gas_per = self._stop_gas_sec / self._stop_events
      harsh_ratio = self._stop_harsh_count / self._stop_events

      # (1) StoppingAccel: 정차 접근 중 제동 vs 가속 개입 균형
      #     음수일수록 일찍·강하게 제동 시작 (문서2: 0=강한제동, 음수=일찍 약하게)
      cur_sa = self._params.get_int("StoppingAccel")
      rec_sa = cur_sa
      sa_reason = ""
      if brake_per >= 0.8 and brake_per > gas_per:
        rec_sa = _clamp_spec("StoppingAccel", cur_sa - _STOP_ACCEL_STEP)
        sa_reason = f"weak/late stop (brake {brake_per:.1f}s/stop)"
      elif gas_per >= 0.8 and gas_per > brake_per:
        rec_sa = _clamp_spec("StoppingAccel", cur_sa + _STOP_ACCEL_STEP)
        sa_reason = f"early/hard stop (gas {gas_per:.1f}s/stop)"
      if rec_sa != cur_sa:
        result["주행 (Driving)"]["StoppingAccel"] = {
          "current": cur_sa,
          "recommended": rec_sa,
          "band_kph": f"정차 제동 시점/강도 ({sa_reason})",
          "stop_events": self._stop_events,
        }

      # (2) VEgoStopping: 거친 정지 비율이 높으면 정지 판정 속도 상향 → 부드럽게 멈춤
      if harsh_ratio >= 0.5:
        cur_ve = self._cur_or_default("VEgoStopping")
        rec_ve = _clamp_spec("VEgoStopping", cur_ve + _STOP_VEGO_STEP)
        if rec_ve != cur_ve:
          result["주행 (Driving)"]["VEgoStopping"] = {
            "current": cur_ve,
            "recommended": rec_ve,
            "band_kph": f"거친 정지 완화 (harsh {harsh_ratio*100:.0f}%)",
            "stop_events": self._stop_events,
          }
      elif harsh_ratio <= 0.2:
        # 정지가 부드러움: 기본값 방향으로 약하게 감쇠(단방향 래칫 방지).
        # 근본 원인(예: Delay runaway 과제동)이 고쳐진 뒤 올라간 값이 눌러앉는 것을 막는다.
        cur_ve = self._cur_or_default("VEgoStopping")
        default_ve = _PARAM_SPEC["VEgoStopping"]["default"]
        if cur_ve > default_ve:
          rec_ve = _decay_toward(cur_ve, default_ve, 2)
          if rec_ve != cur_ve:
            result["주행 (Driving)"]["VEgoStopping"] = {
              "current": cur_ve,
              "recommended": rec_ve,
              "band_kph": f"정지 양호 → 기본값 감쇠 (harsh {harsh_ratio*100:.0f}%)",
              "stop_events": self._stop_events,
            }

      # (3) StopDistanceCarrot: 선행차 뒤 최종 정지 거리 보정
      if self._stop_lead_gap_count >= 3:
        avg_gap = self._stop_lead_gap_sum / self._stop_lead_gap_count
        cur_sd = self._cur_or_default("StopDistanceCarrot")
        rec_sd = cur_sd
        sd_reason = ""
        if avg_gap >= _STOP_GAP_WIDE_M and gas_per > brake_per:
          rec_sd = _clamp_spec("StopDistanceCarrot", cur_sd - _STOP_DIST_STEP)
          sd_reason = f"stops too far ({avg_gap:.1f}m)"
        elif avg_gap <= _STOP_GAP_NEAR_M or brake_per > gas_per:
          rec_sd = _clamp_spec("StopDistanceCarrot", cur_sd + _STOP_DIST_STEP)
          sd_reason = f"stops too close ({avg_gap:.1f}m)"
        if rec_sd != cur_sd:
          result["주행 (Driving)"]["StopDistanceCarrot"] = {
            "current": cur_sd,
            "recommended": rec_sd,
            "band_kph": f"정지 거리 보정 ({sd_reason})",
            "stop_events": self._stop_events,
          }

    # ── Phase 8: 종방향 PID (LongTuningKf / LongActuatorDelay / LongTuningKpV) ──
    if apply_long and self._long_samples >= _LONG_MIN_SAMPLES:
      # lag(둔감)는 transient(지령 변화 추종) 구간에서만 평가한다. transient 표본이
      # 충분치 않으면 0으로 둬서 Kf/Delay를 헛상향하지 않는다. (정속 경사 오프셋 배제)
      if self._long_trans_samples >= _LONG_TRANS_MIN:
        lag_ratio = self._long_trans_lag / self._long_trans_samples
        mean_abs_err = self._long_trans_err_sum / self._long_trans_samples
      else:
        lag_ratio = 0.0
        mean_abs_err = self._long_err_sum / self._long_samples
      overshoot_ratio = self._long_overshoot_count / self._long_samples

      if lag_ratio >= _LONG_LAG_RATIO and lag_ratio > overshoot_ratio:
        # 둔감(추종 지연) 우세 → 피드포워드/지연보정 상향 (선제 가감속)
        cur_kf = self._cur_or_default("LongTuningKf")
        rec_kf = _clamp_spec("LongTuningKf", cur_kf + _LONG_KF_STEP)
        if rec_kf != cur_kf:
          result["주행 (Driving)"]["LongTuningKf"] = {
            "current": cur_kf,
            "recommended": rec_kf,
            "band_kph": f"가감속 둔감 보정 (lag {lag_ratio*100:.0f}%, err {mean_abs_err:.2f})",
            "samples": self._long_samples,
          }
        cur_ld = self._cur_or_default("LongActuatorDelay")
        rec_ld = _clamp_spec("LongActuatorDelay", cur_ld + _LONG_DELAY_STEP)
        if rec_ld != cur_ld:
          result["주행 (Driving)"]["LongActuatorDelay"] = {
            "current": cur_ld,
            "recommended": rec_ld,
            "band_kph": f"가감속 선제 반영 (lag {lag_ratio*100:.0f}%)",
            "samples": self._long_samples,
          }
      elif overshoot_ratio >= _LONG_OVERSHOOT_RATIO and overshoot_ratio > lag_ratio:
        # 진동(과반응) 우세 → 비례게인 하향
        cur_kp = self._cur_or_default("LongTuningKpV")
        rec_kp = _clamp_spec("LongTuningKpV", cur_kp - _LONG_KP_STEP)
        if rec_kp != cur_kp:
          result["주행 (Driving)"]["LongTuningKpV"] = {
            "current": cur_kp,
            "recommended": rec_kp,
            "band_kph": f"가감속 진동 억제 (overshoot {overshoot_ratio*100:.0f}%)",
            "samples": self._long_samples,
          }
      elif lag_ratio < 0.15 and overshoot_ratio < 0.15:
        # 건강(추종 양호): Kf/Delay를 기본값 방향으로 약하게 감쇠 — 단방향 래칫 방지.
        # (0.15~0.30 사이는 히스테리시스 데드밴드로 무조치. _clamp_spec이 규격 밖으로
        #  올라가버린 과거 값(예: Delay 90)도 상한으로 즉시 회수한다.)
        for pkey, step in (("LongTuningKf", _LONG_KF_STEP), ("LongActuatorDelay", _LONG_DELAY_STEP)):
          cur = self._cur_or_default(pkey)
          rec = _clamp_spec(pkey, _decay_toward(cur, _PARAM_SPEC[pkey]["default"], step))
          if rec != cur:
            result["주행 (Driving)"][pkey] = {
              "current": cur,
              "recommended": rec,
              "band_kph": f"양호 → 기본값 감쇠 (lag {lag_ratio*100:.0f}%)",
              "samples": self._long_samples,
            }

    # ── Phase 9: 수동주행 코스팅 측정 → LongCoastBand 추천 ────────────
    # 사람이 무페달로 코스팅할 때의 자연 감속(회생제동/엔진브레이크)을 측정하여,
    # 종방향 코스팅 데드밴드를 차의 코스트 권한 일부 범위 내에서 직접 보정한다.
    if apply_long:
      coast_n = sum(self._manual_coast_decel_n)
      coast_sec = sum(self._manual_coast_sec)
      if coast_n >= _MANUAL_COAST_MIN_N and coast_sec >= _MANUAL_COAST_MIN_SEC:
        mean_coast_decel = sum(self._manual_coast_decel_sum) / coast_n  # m/s² (양수)
        # 데드밴드(m/s²) = 측정 코스팅 감속 × 게인, 안전범위로 클램프 후 cm/s² 정수화
        rec_band = _clamp_spec("LongCoastBand",
                               round(np.clip(mean_coast_decel * _MANUAL_COAST_GAIN, 0.15, 0.40) * 100))
        cur_band = self._params.get_int("LongCoastBand")
        # 5cm/s²(=0.05 m/s²) 이상 차이날 때만 추천(미세 변동 잡음 억제)
        if abs(rec_band - cur_band) >= 5:
          result["주행 (Driving)"]["LongCoastBand"] = {
            "current": cur_band,
            "recommended": rec_band,
            "band_kph": f"수동 코스팅 감속 {mean_coast_decel:.2f}m/s² 측정 → 코스팅(회생제동) 데드밴드 보정",
            "samples": coast_n,
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

    apply_lat = self._params.get_bool("CarrotTunerApplyLat") if self._params.get("CarrotTunerApplyLat") is not None else True
    apply_long = self._params.get_bool("CarrotTunerApplyLong") if self._params.get("CarrotTunerApplyLong") is not None else True

    lat_keys = {
      "PathOffset", "SteerActuatorDelay", "SteerRatioRate", 
      "LateralTorqueAccelFactor", "LateralTorqueKf", "LateralTorqueFriction", 
      "LateralTorqueKiV", "LateralTorqueKpV"
    }

    applied_changes = {}
    for group in recommendations:
      g_items = {}
      for key in recommendations[group]:
        info = recommendations[group][key]
        is_lat = key in lat_keys
        if is_lat and not apply_lat:
          continue
        if not is_lat and not apply_long:
          continue
        self._params.put_int(key, info["recommended"])
        g_items[key] = info
      if g_items:
        applied_changes[group] = g_items

    if applied_changes:
      import datetime
      timestamp_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
      history_id = datetime.datetime.now().strftime("%Y%m%d%H%M%S%f")
      
      new_entry = {
        "id": history_id,
        "timestamp": timestamp_str,
        "changes": applied_changes
      }
      
      history_raw = self._params.get("CarrotLearningHistory")
      history_arr = []
      if history_raw:
        try:
          history_arr = json.loads(history_raw)
          if not isinstance(history_arr, list):
            history_arr = []
        except Exception:
          history_arr = []
          
      history_arr.insert(0, new_entry)
      history_arr = history_arr[:50]
      self._params.put("CarrotLearningHistory", json.dumps(history_arr).encode('utf8'))


    # ── 적용된 Phase의 누적치만 선택적으로 리셋 ─────────────────────
    # 적용되지 않은 Phase(특히 느리게 쌓이는 조향)는 데이터를 보존해
    # 무관한 적용 때문에 학습 진척이 초기화되지 않도록 한다.
    # (제동 3종 충돌 방지로 '다음 세션 권고'된 항목의 근거 데이터도 자연히 이월됨)
    applied_phases = set()
    for group in applied_changes:
      for key in applied_changes[group]:
        ph = _KEY_RESET_PHASE.get(key)
        if ph is not None:
          applied_phases.add(ph)
    phase_reset = {
      1: self._reset_phase1, 2: self._reset_phase2, 3: self._reset_phase3,
      4: self._reset_phase4, 5: self._reset_phase5, 6: self._reset_phase6,
      7: self._reset_phase7, 8: self._reset_phase8, 9: self._reset_phase9,
    }
    for ph in applied_phases:
      phase_reset[ph]()

    # 주행 중 팝업 타이머 리셋 (적용 후 재학습 시작)
    self._engaged_elapsed_sec = 0.0
    self._check_elapsed_sec = 0.0
    self._pending_popup = False
    self._popup_cooldown_sec = 0.0

    # 보존된 Phase 데이터가 재부팅에도 살아남도록 즉시 저장(remove 대신 _save).
    self._save()
    self._params.remove("CarrotLearningRecommend")
    self._params.remove("CarrotLearningPopupSource")
    self._params.put_bool("CarrotLearningPopupReady", False)
    # 주의: 적용된 추천값이 다음 학습 라운드의 새 기준점이 됩니다.
    # (과거: 여기서 튜닝 파라미터를 공장초기값으로 원복하여 방금 적용한 추천을
    #  덮어쓰는 버그가 있었음 → 제거. 누적 데이터만 초기화하고 파라미터는 유지.)


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
