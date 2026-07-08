#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, N
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_speed_error, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params


LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -2.0 #-1.2
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.5
MIN_ALLOW_THROTTLE_SPEED = 2.5
RESET_DECEL_RAMP_TIME = 2.0

# ── Jerk ease-in (A안): 가감속 onset에서 jerk를 점증시켜 S-curve로 만든다 ──
# 일정 jerk 상한은 onset에서 jerk가 0→상한으로 '계단'처럼 튀어(=jounce 스파이크)
# 시작 jolt를 남긴다. 시작 직후 jerk를 점증시키면 가속도가 S자로 부드럽게 붙는다.
JERK_EASE_TIME = 0.4        # 새 maneuver 시작 후 jerk를 100%로 키우는 시간(s)
JERK_EASE_FLOOR = 0.3       # 시작 시 jerk 비율 하한(감속 onset 등)
# 가속은 선행차 추종 재가속(거리 좁히기)이 느리지 않게 시작 jerk를 더 높게 둔다.
# (감속보다 높은 floor → 초중반 가속력↑, 단 0에서 시작하는 ease 자체는 유지해 급가속감 방지)
JERK_EASE_FLOOR_ACCEL = 0.6
# 가속 ease-out: 가속을 마무리하며(현재 가속 중, 양의 가속을 0 근처로 줄이는 구간) 목표
# 차간거리에 살며시 도달하도록 부드러운 jerk를 쓴다(사용자 요청: 끝부분은 더 부드럽게).
ACCEL_EASEOUT_JERK = 1.2
# 고속 제동 안전 우회: 고속에서 선행차 접근 중이면 ease를 풀어(=즉응) 감지 초기부터
# 충분한 제동이 미리 들어가게 한다(고속 늦은 감지로 인한 충돌 우려 대응).
HIGH_SPEED_BRAKE_KPH = 70.0
HIGH_SPEED_BRAKE_TTC = 8.0  # 이 TTC(초) 이내로 접근 중이면 제동 ease 해제

# Lookup table for turns
_A_TOTAL_MAX_V = [2.4, 4.8] #[1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]
LAT_WEIGHT = 0.7


def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns_org(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  steer_abs = abs(angle_steers)
  if v_ego > 20 or (v_ego > 25 and steer_abs < 3.0):
    return a_target
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase) * LAT_WEIGHT
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]

def limit_accel_in_turns(v_ego, curvature, a_target, a_lat_max,
                         safety_ratio=0.70,   # 0.60~0.85 (작을수록 더 얌전)
                         min_v=0.1):
  """
  v_ego    : m/s
  curvature: 1/m  (sign 포함)
  a_target : [a_min, a_max] (m/s^2)
  a_lat_max: 허용 최대 횡가속 (m/s^2)

  safety_ratio:
    a_lat_max에 소프트 마진을 주는 비율.
    예) a_lat_max=4, safety_ratio=0.7 -> 실사용 한계 2.8로 계산.

  return   : [a_min, 제한된 a_max]
  """
  if v_ego < min_v or a_lat_max <= 0.0:
    return a_target

  a_lat_eff = abs(a_lat_max) * float(safety_ratio)

  # 횡가속
  a_y_abs = abs((v_ego * v_ego) * curvature)

  # 남은 종가속 여유 (원형 경계)
  if a_y_abs >= a_lat_eff:
    a_x_allowed = 0.0
  else:
    a_x_allowed = math.sqrt(a_lat_eff * a_lat_eff - a_y_abs * a_y_abs)

  # a_target = [min, max] 중 max만 제한
  return [a_target[0], min(a_target[1], a_x_allowed)]

class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self._jerk_ramp_t = 0.0   # jerk ease-in 경과시간(새 가감속 시작부터)
    self._jerk_dir = 0        # 직전 가감속 방향(+1 가속 / -1 감속 / 0)
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_v_target_now = 0.0
    self.output_j_target_now = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.vCluRatio = 1.0
    self.reset_decel_timer = 0
    self.reset_decel_start_a = 0.0
    
    self.v_cruise_kph = 0.0

    self.params = Params()

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  @staticmethod
  def _is_stopping(sm, carrot):
    """정지 의도(신호등 정지 xState==3 또는 모델 shouldStop) 여부."""
    try:
      return bool(carrot.xState.value == 3 or sm['modelV2'].action.shouldStop)
    except Exception:
      return False


  def _positive_jerk_limit(self, a_prev, v_ego_kph):
    """가속 방향(a_target>a_prev) jerk 상한."""
    if a_prev < 0.0:
      # 감속 해제(음수 가속도를 0으로 푸는 구간)는 크게 허용 — 브레이크는 신속히 뗀다.
      return 3.0
    # 가속 build-up: 저중속 재가속(추종 거리 좁히기)이 느리지 않게 jerk를 속도비례로 키우고,
    # 가속 전용 ease floor로 초중반 가속력을 확보(급가속감은 ease-in ramp로 방지).
    jerk_speed = float(np.interp(v_ego_kph, [0.0, 30.0, 80.0], [0.95, 1.5, 2.0]))
    jerk_accel = float(np.interp(a_prev, [0.0, 1.0], [1.0, 0.7]))
    ease_acc = float(np.clip(self._jerk_ramp_t / JERK_EASE_TIME, JERK_EASE_FLOOR_ACCEL, 1.0))
    return jerk_speed * jerk_accel * ease_acc

  def _negative_jerk_limit(self, a_target, a_prev, v_ego_kph, sm, carrot):
    """감속 방향(a_target<a_prev) jerk 상한. 상황별로 ease를 풀어 즉응 제동을 확보한다."""
    if a_prev > 0.1 and a_target > -0.4:
      # 가속 ease-out: 가속 중 목표 간격에 근접해 가속을 거두는 구간(실제 제동 아님)은
      # 부드러운 jerk로 살며시 마무리(끝부분 부드럽게 — 사용자 요청).
      return ACCEL_EASEOUT_JERK
    # 제동 build-up: 목표 감속이 깊을수록 한도를 키워(≈무제한) 안전 확보.
    #   -1.2(완만) 2.0 / -2.5 5.0 / -4.0(긴급) 12.0  [m/s^3]
    max_negative_jerk = float(np.interp(a_target, [-4.0, -2.5, -1.2], [12.0, 5.0, 2.0]))
    # 기본 ease(전환 후 점증)에서 시작, 아래 상황에선 ease를 풀어(→1.0) 즉응 제동.
    ease_dec = float(np.clip(self._jerk_ramp_t / JERK_EASE_TIME, JERK_EASE_FLOOR, 1.0))
    # (1) 깊은 감속은 '고속에서만' ease 해제 — 저속(≤25km/h)은 급브레이킹 완화 위해 ease 유지.
    deep = float(np.interp(a_target, [-3.0, -1.5], [1.0, 0.0])) * float(np.interp(v_ego_kph, [25.0, 50.0], [0.0, 1.0]))
    ease_dec = max(ease_dec, deep)
    # (2) 고속 + 선행차 접근(TTC<임계): 감지 초기부터 충분 제동(고속 늦은 감지 충돌 우려 대응).
    if v_ego_kph >= HIGH_SPEED_BRAKE_KPH:
      try:
        lead = sm['radarState'].leadOne
        if lead.status and lead.dRel > 0.0 and lead.vRel < 0.0 and (lead.dRel / -lead.vRel) < HIGH_SPEED_BRAKE_TTC:
          ease_dec = 1.0
      except Exception:
        pass
    # (3) 정지(신호등/모델) 접근: 선행차 없어도 즉응 제동으로 정지선 초과 방지.
    if self._is_stopping(sm, carrot):
      ease_dec = 1.0
    return max_negative_jerk * ease_dec

  def _apply_jerk_limits(self, a_target, a_prev, v_ego_kph, sm, carrot):
    """가감속 명령의 jerk(가속도 변화율)를 제한해 onset jolt를 줄인다(S-curve).

    maneuver phase(가속 +1 / 감속 -1)를 a_target 부호 + 데드밴드(±0.15)로 판정하고,
    가속↔감속 '전환'에서만 ease ramp를 재시작한다(지속 가감속에선 jerk가 100%까지 자람).
    """
    if a_target > 0.15:
      phase = 1
    elif a_target < -0.15:
      phase = -1
    else:
      phase = self._jerk_dir   # 데드밴드: 직전 phase 유지
    if phase != self._jerk_dir:
      self._jerk_ramp_t = 0.0
    self._jerk_dir = phase
    self._jerk_ramp_t += self.dt

    if a_target > a_prev:
      a_target = min(a_target, a_prev + self._positive_jerk_limit(a_prev, v_ego_kph) * self.dt)
    elif a_target < a_prev:
      a_target = max(a_target, a_prev - self._negative_jerk_limit(a_target, a_prev, v_ego_kph, sm, carrot) * self.dt)
    return a_target

  def update(self, sm, carrot):
    self.mpc.mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)

    self.v_cruise_kph = carrot.update(sm, v_cruise_kph, self.mpc.mode)
    self.mpc.mode = carrot.mode
    v_cruise = self.v_cruise_kph * CV.KPH_TO_MS

    vCluRatio = sm['carState'].vCluRatio
    if vCluRatio > 0.5:
      self.vCluRatio = vCluRatio
      v_cruise *= vCluRatio

    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized or carrot.soft_hold_active

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      #accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits = [A_CRUISE_MIN, carrot.get_carrot_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      #accel_limits_turns = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_limits, self.CP)
      a_lat_max = 3.0
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['controlsState'].desiredCurvature, accel_limits, a_lat_max)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])
      
      self.mpc.prev_a = np.full(N+1, self.a_desired) ## carrot

      self.reset_decel_timer = int(RESET_DECEL_RAMP_TIME / self.dt)
      self.reset_decel_start_a = self.a_desired

    elif self.reset_decel_timer > 0:
      ramp_steps = max(1, int(RESET_DECEL_RAMP_TIME / self.dt))
      t = float(np.clip(self.reset_decel_timer / ramp_steps, 0.0, 1.0))
      self.reset_decel_timer -= 1

      # 2초 동안 감속 하한을 현재 가속도 근처에서 원래 limit으로 천천히 복귀
      soft_min_accel = min(0.0, self.reset_decel_start_a - 0.05)
      ramped_min_accel = soft_min_accel * t + accel_limits_turns[0] * (1.0 - t)

      accel_limits_turns[0] = max(
        accel_limits_turns[0],
        min(0.0, ramped_min_accel)
      )

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = True #throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_limits_turns[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_limits_turns[1], clipped_accel_coast])
      accel_limits_turns[1] = min(accel_limits_turns[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality, jerk_factor = carrot.jerk_factor_apply, a_change_cost_starting = carrot.aChangeCostStarting)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(carrot, reset_state, sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill and not reset_state
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    a_target = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    v_ego_kph = v_ego * CV.MS_TO_KPH

    # jerk(가속도 변화율) 제한으로 a_target을 다듬는다.
    # (내리막 중력보상은 제거 — 내리막 크롤링 추종/재출발에서 MPC와 충돌해 과감속·stuck을
    #  반복 유발. 평지와 동일한 openpilot 기본 동작 유지. git 히스토리에 보존.)
    a_target = self._apply_jerk_limits(a_target, a_prev, v_ego_kph, sm, carrot)
    self.a_desired = a_target
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    longitudinalActuatorDelay = self.params.get_float("LongActuatorDelay")*0.01
    vEgoStopping = self.params.get_float("VEgoStopping") * 0.01
    action_t =  longitudinalActuatorDelay + DT_MDL

    output_a_target_mpc, output_should_stop_mpc, output_v_target_mpc, _ = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    output_v_target_now_e2e = sm['modelV2'].action.desiredVelocity

    if self.mpc.mode == 'acc':
      output_a_target = output_a_target_mpc
      output_v_target_now = output_v_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      output_v_target_now = min(output_v_target_mpc, output_v_target_now_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    #for idx in range(2):
    #  accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    #self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    #self.prev_accel_clip = accel_clip
    self.output_a_target = output_a_target
    self.output_v_target_now = output_v_target_now
    self.output_j_target_now = self.j_desired_trajectory[0]

  def publish(self, sm, pm, carrot):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.vTargetNow = float(self.output_v_target_now)
    longitudinalPlan.jTargetNow = float(self.output_j_target_now)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    longitudinalPlan.xState = carrot.xState.value
    longitudinalPlan.trafficState = carrot.trafficState.value
    longitudinalPlan.cruiseTarget = self.v_cruise_kph
    longitudinalPlan.tFollow = float(self.mpc.t_follow)
    longitudinalPlan.desiredDistance = float(self.mpc.desired_distance)
    longitudinalPlan.events = carrot.events.to_msg()
    longitudinalPlan.myDrivingMode = carrot.myDrivingMode.value

    pm.send('longitudinalPlan', plan_send)
