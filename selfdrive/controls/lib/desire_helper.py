from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params
from collections import deque

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}


def calculate_lane_width_frog(lane, current_lane, road_edge):
  lane_x, lane_y = np.array(lane.x), np.array(lane.y)
  edge_x, edge_y = np.array(road_edge.x), np.array(road_edge.y)
  current_x, current_y = np.array(current_lane.x), np.array(current_lane.y)

  lane_y_interp = np.interp(current_x, lane_x[lane_x.argsort()], lane_y[lane_x.argsort()])
  road_edge_y_interp = np.interp(current_x, edge_x[edge_x.argsort()], edge_y[edge_x.argsort()])

  distance_to_lane = np.mean(np.abs(current_y - lane_y_interp))
  distance_to_road_edge = np.mean(np.abs(current_y - road_edge_y_interp))

  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge


def calculate_lane_width(lane, lane_prob, current_lane, road_edge):
  # t ≈ 1초 앞 기준으로 차선/도로 가장자리까지의 거리 계산
  t = 1.0
  current_lane_y = np.interp(t, ModelConstants.T_IDXS, current_lane.y)
  lane_y = np.interp(t, ModelConstants.T_IDXS, lane.y)
  distance_to_lane = abs(current_lane_y - lane_y)

  road_edge_y = np.interp(t, ModelConstants.T_IDXS, road_edge.y)
  distance_to_road_edge = abs(current_lane_y - road_edge_y)
  distance_to_road_edge_far = abs(current_lane_y - np.interp(2.0, ModelConstants.T_IDXS, road_edge.y))

  lane_valid = lane_prob > 0.5
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, distance_to_road_edge_far, lane_valid


class ExistCounter:
  """
  존재 여부가 노이즈처럼 튀는 신호를 히스테리시스로 안정화해 주는 카운터.
  - true가 일정 시간 이상 지속되면 counter를 양수로 증가
  - false가 일정 시간 이상 지속되면 counter를 음수로 감소
  """
  def __init__(self):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(0.2 / DT_MDL)  # 약 0.2초 이상 지속 시 유효로 판단

  def update(self, exist_flag: bool):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0
      if self.true_count >= self.threshold:
        self.counter = max(self.counter + 1, 1)
    else:
      self.false_count += 1
      self.true_count = 0
      if self.false_count >= self.threshold:
        self.counter = min(self.counter - 1, -1)
    return self.true_count


class DesireHelper:
  def __init__(self):
    self.params = Params()
    self.frame = 0

    # Lane change / turn 상태
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_delay = 0.0
    self.maneuver_type = "none"  # "none" / "turn" / "lane_change"

    # Desire / turn 관련
    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.desire_disable_count = 0   # turn 후 일정 시간 동안 차선변경 금지
    self.turn_disable_count = 0     # steeringAngle 매우 클 때 turn 금지

    # Lane / road edge 상태
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_diff = 0.0
    self.lane_width_right_diff = 0.0
    self.distance_to_road_edge_left = 0.0
    self.distance_to_road_edge_right = 0.0
    self.distance_to_road_edge_left_far = 0.0
    self.distance_to_road_edge_right_far = 0.0

    self.lane_exist_left_count = ExistCounter()
    self.lane_exist_right_count = ExistCounter()
    self.lane_width_left_count = ExistCounter()
    self.lane_width_right_count = ExistCounter()
    self.road_edge_left_count = ExistCounter()
    self.road_edge_right_count = ExistCounter()

    self.available_left_lane = False
    self.available_right_lane = False
    self.available_left_edge = False
    self.available_right_edge = False

    self.lane_width_left_queue = deque(maxlen=int(1.0 / DT_MDL))
    self.lane_width_right_queue = deque(maxlen=int(1.0 / DT_MDL))

    self.lane_available_last = False
    self.edge_available_last = False
    self.lane_available_trigger = False
    self.lane_appeared = False
    self.lane_line_info = 0

    # Blinkers / ATC
    self.blinker_ignore = False
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.atc_type = ""
    self.atc_active = 0  # 0: 없음, 1: ATC 동작 중, 2: 운전자와 ATC 상충

    # Auto lane change 관련
    self.auto_lane_change_enable = False
    self.next_lane_change = False
    self.blindspot_detected_counter = 0

    # Keep pulse
    self.keep_pulse_timer = 0.0

    # 파라미터
    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneChangeDelay = 0.0
    self.modelTurnSpeedFactor = 0.0
    self.model_turn_speed = 0.0

    # 기타
    self.prev_desire_enabled = False
    self.desireLog = ""
    self.object_detected_count = 0

    # ★ 현재 차선 확률 (Ego 기준 좌/우)
    self.cur_left_prob = 1.0   # laneLineProbs[1]
    self.cur_right_prob = 1.0  # laneLineProbs[2]
    self.current_lane_missing = False
  # ─────────────────────────────────────────────
  #  Config / Model 관련
  # ─────────────────────────────────────────────

  def _update_params_periodic(self):
    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd = self.params.get_int("LaneChangeBsd")
      self.laneChangeDelay = self.params.get_float("LaneChangeDelay") * 0.1
      self.modelTurnSpeedFactor = self.params.get_float("ModelTurnSpeedFactor") * 0.1

  def _make_model_turn_speed(self, modeldata):
    if self.modelTurnSpeedFactor > 0:
      model_turn_speed = np.interp(self.modelTurnSpeedFactor,
                                   modeldata.velocity.t,
                                   modeldata.velocity.x) * CV.MS_TO_KPH * 1.2
      self.model_turn_speed = self.model_turn_speed * 0.9 + model_turn_speed * 0.1
    else:
      self.model_turn_speed = 200.0

  # ─────────────────────────────────────────────
  #  Lane / Edge 상태 계산
  # ─────────────────────────────────────────────

  def _check_lane_state(self, modeldata):
    # 왼쪽: laneLines[0] vs 현재차선 laneLines[1], roadEdges[0]
    lane_width_left, self.distance_to_road_edge_left, self.distance_to_road_edge_left_far, lane_prob_left = \
      calculate_lane_width(modeldata.laneLines[0], modeldata.laneLineProbs[0],
                           modeldata.laneLines[1], modeldata.roadEdges[0])

    # 오른쪽: laneLines[3] vs 현재차선 laneLines[2], roadEdges[1]
    lane_width_right, self.distance_to_road_edge_right, self.distance_to_road_edge_right_far, lane_prob_right = \
      calculate_lane_width(modeldata.laneLines[3], modeldata.laneLineProbs[3],
                           modeldata.laneLines[2], modeldata.roadEdges[1])

    # 차선 존재 카운터 업데이트
    self.lane_exist_left_count.update(lane_prob_left)
    self.lane_exist_right_count.update(lane_prob_right)

    # 1초 이동 평균 (노이즈 줄이기)
    self.lane_width_left_queue.append(lane_width_left)
    self.lane_width_right_queue.append(lane_width_right)

    self.lane_width_left = float(np.mean(self.lane_width_left_queue))
    self.lane_width_right = float(np.mean(self.lane_width_right_queue))

    self.lane_width_left_diff = self.lane_width_left_queue[-1] - self.lane_width_left_queue[0]
    self.lane_width_right_diff = self.lane_width_right_queue[-1] - self.lane_width_right_queue[0]

    # 유효 차선/엣지 판단
    min_lane_width = 2.5
    self.lane_width_left_count.update(self.lane_width_left > min_lane_width)
    self.lane_width_right_count.update(self.lane_width_right > min_lane_width)
    self.road_edge_left_count.update(self.distance_to_road_edge_left > min_lane_width)
    self.road_edge_right_count.update(self.distance_to_road_edge_right > min_lane_width)

    available_count = int(0.2 / DT_MDL)
    self.available_left_lane = self.lane_width_left_count.counter > available_count
    self.available_right_lane = self.lane_width_right_count.counter > available_count
    self.available_left_edge = self.road_edge_left_count.counter > available_count and self.distance_to_road_edge_left_far > min_lane_width
    self.available_right_edge = self.road_edge_right_count.counter > available_count and self.distance_to_road_edge_right_far > min_lane_width

    self.cur_left_prob = modeldata.laneLineProbs[1]
    self.cur_right_prob = modeldata.laneLineProbs[2]

  # ─────────────────────────────────────────────
  #  모델 내 desire 상태 (turn 예측 등)
  # ─────────────────────────────────────────────

  def _check_desire_state(self, modeldata, carstate, maneuver_type):
    desire_state = modeldata.meta.desireState
    # turnLeft + turnRight 확률
    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1

    #if self.turn_desire_state:
    #  self.desire_disable_count = int(2.0 / DT_MDL)
    #else:
    #  self.desire_disable_count = max(0, self.desire_disable_count - 1)

    # steeringAngle 너무 크면 turn 자체를 일정 시간 막기
    if abs(carstate.steeringAngleDeg) > 80:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  # ─────────────────────────────────────────────
  #  Blinkers / ATC 상태 업데이트
  # ─────────────────────────────────────────────

  def _update_driver_blinker(self, carstate):
    driver_blinker_state = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    driver_blinker_changed = driver_blinker_state != self.driver_blinker_state
    self.driver_blinker_state = driver_blinker_state

    driver_desire_enabled = driver_blinker_state in [BLINKER_LEFT, BLINKER_RIGHT]
    if self.laneChangeNeedTorque < 0:
      # 운전자 깜빡이를 무시하고 차선변경 안 하는 설정
      driver_desire_enabled = False

    return driver_blinker_state, driver_blinker_changed, driver_desire_enabled

  def _update_atc_blinker(self, carrotMan, v_ego, driver_blinker_state):
    """
    ATC에서 온 turn/lanechange 명령 기반 깜빡이 상태 갱신.
    """
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    # ATC 기반 자동 차선변경 유지 시간
    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif atc_type in ["turn left", "turn right"]:
      # 네비 turn 안내: 속도 조건을 턴 쪽으로 강제
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
    elif atc_type in ["fork left", "fork right", "atc left", "atc right"]:
      # 분기(lanechange에 가까움)
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ["fork left", "atc left"] else BLINKER_RIGHT
        self.atc_active = 1
    else:
      self.atc_active = 0

    # 운전자 깜빡이와 ATC 깜빡이가 충돌할 경우 ATC 무효화
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2

    atc_desire_enabled = atc_blinker_state in [BLINKER_LEFT, BLINKER_RIGHT]

    # blinker_ignore일 때는 깜빡이 신호를 잠시 무시
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      driver_blinker_state = BLINKER_NONE
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # ATC 타입이 바뀌었으면 이번 프레임은 무시 (안정화 목적)
    if self.atc_type != atc_type:
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

  # ─────────────────────────────────────────────
  #  Turn / LaneChange 모드 분류
  # ─────────────────────────────────────────────

  def _classify_maneuver_type(self, blinker_state, carstate, old_type):
    """
    깜빡이가 들어왔을 때 이번 조작이 turn인지 lane_change인지 분류.
    - 너무 복잡하게 가지 않고, 현재 속도/감속/차선상태/모델 turn 상태 기준으로 점수화.
    """
    if blinker_state == BLINKER_NONE:
      return "none"

    v_kph = carstate.vEgo * CV.MS_TO_KPH
    accel = carstate.aEgo

    # 깜빡이 방향에 따라 참조할 lane/edge 선택
    if blinker_state == BLINKER_LEFT:
      lane_exist_counter = self.lane_exist_left_count.counter
      lane_available = self.available_left_lane
      edge_available = self.available_left_edge
      lane_prob_side = self.cur_left_prob
      edge_dist = self.distance_to_road_edge_left_far
    else:
      lane_exist_counter = self.lane_exist_right_count.counter
      lane_available = self.available_right_lane
      edge_available = self.available_right_edge
      lane_prob_side = self.cur_right_prob
      edge_dist = self.distance_to_road_edge_right_far

    score_turn = 0

    if v_kph < 30.0:
      score_turn += 1
    elif v_kph < 40.0 and accel < -1.0:
      score_turn += 1

    # 차로가 없고, 로드에지도 여유없고..
    if v_kph < 40.0 and not lane_available and not edge_available:
      score_turn += 1

    # 차선이 잘 안 보이거나(교차로/삼거리 등)
    if v_kph < 40.0 and lane_exist_counter < int(0.5 / DT_MDL):
      score_turn += 1

    # steeringAngle이 크면 턴에 가깝다고 본다
    #if abs(carstate.steeringAngleDeg) > 45.0:
    #  score_turn += 1

    # 모델이 이미 turn을 예측 중이면 가중치
    if self.turn_desire_state:
      score_turn += 1

    # ATC가 turn 안내 중이면 가중치
    if self.atc_type in ["turn left", "turn right"]:
      score_turn += 2
    elif self.atc_type in ["fork left", "fork right", "atc left", "atc right"]:
      score_turn -= 2  # fork/atc는 lanechange 쪽에 더 가깝게

    # ★ road edge가 충분히 멀면(교차로/넓은 공간으로 판단) 턴 쪽으로 가산점
    edge_far = edge_dist > 4.0  # 튜닝 포인트 (4~6m 정도가 무난)
    #if edge_far:
    #  score_turn += 1
      
    current_lane_missing = lane_prob_side < 0.3
    self.current_lane_missing = current_lane_missing
    # 튜닝 포인트: score_turn 임계값
    if score_turn >= 2:
      #if current_lane_missing and edge_far:
      if edge_far:
        return "turn"
      else:
        return old_type
    else:
      return "lane_change"

  # ─────────────────────────────────────────────
  #  메인 업데이트 루틴
  # ─────────────────────────────────────────────

  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):
    self.frame += 1
    self._update_params_periodic()
    self._make_model_turn_speed(modeldata)

    # 카운터 감소
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay = max(0.0, self.lane_change_delay - DT_MDL)
    self.blindspot_detected_counter = max(0, self.blindspot_detected_counter - 1)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane / desire 상태 갱신
    self._check_lane_state(modeldata)
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    # 운전자 깜빡이
    driver_blinker_state, driver_blinker_changed, driver_desire_enabled = self._update_driver_blinker(carstate)

    # BSD 설정
    ignore_bsd = (self.laneChangeBsd < 0)
    block_lanechange_bsd = (self.laneChangeBsd == 1)

    # ATC 깜빡이
    atc_blinker_state, atc_desire_enabled = self._update_atc_blinker(carrotMan, v_ego, driver_blinker_state)

    # 최종 깜빡이/Desire enabled 판단
    desire_enabled = driver_desire_enabled or atc_desire_enabled
    blinker_state = driver_blinker_state if driver_desire_enabled else atc_blinker_state

    # lane_line_info (HUD용 등)
    lane_line_info = carstate.leftLaneLine if blinker_state == BLINKER_LEFT else carstate.rightLaneLine

    # BSD / 주변 차량 감지
    if desire_enabled:
      lane_exist_counter = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
      lane_available = self.available_left_lane if blinker_state == BLINKER_LEFT else self.available_right_lane
      edge_available = self.available_left_edge if blinker_state == BLINKER_LEFT else self.available_right_edge
      self.lane_appeared = self.lane_appeared or lane_exist_counter == int(0.2 / DT_MDL)

      radar = radarState.leadLeft if blinker_state == BLINKER_LEFT else radarState.leadRight
      side_object_dist = radar.dRel + radar.vLead * 4.0 if radar.status else 255
      object_detected = side_object_dist < v_ego * 3.0
      if object_detected:
        self.object_detected_count = max(1, self.object_detected_count + 1)
      else:
        self.object_detected_count = min(-1, self.object_detected_count - 1)

      lane_line_info_edge_detect = (lane_line_info % 10 in [0, 5] and self.lane_line_info not in [0, 5])
      self.lane_line_info = lane_line_info % 10
    else:
      lane_exist_counter = 0
      lane_available = True
      edge_available = True
      self.lane_appeared = False
      self.lane_available_trigger = False
      self.object_detected_count = 0
      lane_line_info_edge_detect = False
      self.lane_line_info = lane_line_info % 10

    # 차선/엣지 기반 lane change 가능 여부
    lane_change_available = (lane_available or edge_available) and lane_line_info < 20  # 20 미만이면 흰색 라인

    # lane_available 변화 & 폭 변화로 lane_available_trigger 계산
    self.lane_available_trigger = False
    if blinker_state == BLINKER_LEFT:
      lane_width_diff = self.lane_width_left_diff
      distance_to_road_edge = self.distance_to_road_edge_left
      lane_width_side = self.lane_width_left
    else:
      lane_width_diff = self.lane_width_right_diff
      distance_to_road_edge = self.distance_to_road_edge_right
      lane_width_side = self.lane_width_right

    if lane_width_diff > 0.8 and (lane_width_side < distance_to_road_edge):
      self.lane_available_trigger = True

    edge_availabled = (not self.edge_available_last and edge_available)
    side_object_detected = self.object_detected_count > -0.3 / DT_MDL
    self.lane_appeared = self.lane_appeared and distance_to_road_edge < 4.0

    # Auto lane change 트리거
    if self.carrot_lane_change_count > 0:
      auto_lane_change_blocked = False
      auto_lane_change_trigger = lane_change_available
    else:
      auto_lane_change_blocked = ((atc_blinker_state == BLINKER_LEFT) and (driver_blinker_state != BLINKER_LEFT))
      self.auto_lane_change_enable = self.auto_lane_change_enable and not auto_lane_change_blocked
      auto_lane_change_trigger = self.auto_lane_change_enable and edge_available and (self.lane_available_trigger or self.lane_appeared) and not side_object_detected
      self.desireLog = f"L:{self.auto_lane_change_enable},{auto_lane_change_blocked},E:{lane_available},{edge_available},A:{self.lane_available_trigger},{self.lane_appeared},{lane_width_diff:.1f},{lane_width_side:.1f},{distance_to_road_edge:.1f}={auto_lane_change_trigger}"

    # 메인 상태머신

    # 0) lateral 끊기거나 너무 오래 지속되면 리셋
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    # 1) turn 후 일정시간 동안은 아무 것도 하지 않음
    elif self.desire_disable_count > 0:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    else:
      # 깜빡이 켜져 있을 때, 이번 조작이 turn인지 lane_change인지 먼저 분류
      if desire_enabled:
        new_type = self._classify_maneuver_type(blinker_state, carstate, self.maneuver_type)
      else:
        new_type = "none"

      # ★ 1) 원래 lane_change였는데 새로 보니 turn 조건 + 차선 없음이면 → 강제 전환 허용
      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in [LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting]:
        # 차선변경 도중에도 조건 만족 시 턴으로 스위칭
        self.maneuver_type = "turn"
        self.lane_change_state = LaneChangeState.off  # FSM 리셋 후 turn 루트로
      # ★ 2) 그 외에는 off/pre 상태에서만 모드 변경
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ TURN 모드 처리 ─
      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires: # and not carstate.standstill:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0:
          self.turn_direction = TurnDirection.none
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
          # 호환성을 위해 lane_change_direction도 turn과 동일하게 세팅
          self.lane_change_direction = self.turn_direction

      # ─ Lane Change FSM 처리 ─
      else:
        self.turn_direction = TurnDirection.none

        # LaneChangeState.off
        if self.lane_change_state == LaneChangeState.off:
          if desire_enabled and not self.prev_desire_enabled and not below_lane_change_speed:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.lane_change_delay = self.laneChangeDelay

            # 맨 끝 차선이 아니면, ATC 자동 차선변경 비활성
            lane_exist_counter_side = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
            self.auto_lane_change_enable = False if lane_exist_counter_side > 0 or lane_change_available else True
            self.next_lane_change = False

        # LaneChangeState.preLaneChange
        elif self.lane_change_state == LaneChangeState.preLaneChange:
          self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right
          dir_map = {
            LaneChangeDirection.left: (carstate.steeringTorque > 0, carstate.leftBlindspot),
            LaneChangeDirection.right: (carstate.steeringTorque < 0, carstate.rightBlindspot),
          }
          torque_cond, blindspot_cond = dir_map.get(self.lane_change_direction, (False, False))
          torque_applied = carstate.steeringPressed and torque_cond
          blindspot_detected = blindspot_cond

          # 차선이 일정시간 이상 안보이면 자동차선변경 허용
          lane_exist_counter_side = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
          if not lane_available or lane_exist_counter_side < int(2.0 / DT_MDL):
            self.auto_lane_change_enable = True

          # BSD
          if blindspot_detected and not ignore_bsd:
            self.blindspot_detected_counter = int(1.5 / DT_MDL)

          if not desire_enabled or below_lane_change_speed:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          else:
            # 차선변경 시작 조건
            if (lane_change_available and self.lane_change_delay == 0) or lane_line_info_edge_detect:
              if self.blindspot_detected_counter > 0 and not ignore_bsd:
                if torque_applied and not block_lanechange_bsd:
                  self.lane_change_state = LaneChangeState.laneChangeStarting
              elif self.laneChangeNeedTorque > 0 or self.next_lane_change:
                if torque_applied:
                  self.lane_change_state = LaneChangeState.laneChangeStarting
              elif driver_desire_enabled:
                self.lane_change_state = LaneChangeState.laneChangeStarting
              elif torque_applied or auto_lane_change_trigger or lane_line_info_edge_detect:
                self.lane_change_state = LaneChangeState.laneChangeStarting

        # LaneChangeState.laneChangeStarting
        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          # 원래 차선라인을 0.5초 동안 서서히 fade-out
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing

        # LaneChangeState.laneChangeFinishing
        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          # 1초 동안 서서히 lane line 복귀
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              self.next_lane_change = True
            else:
              self.lane_change_state = LaneChangeState.off

    # lane_change_timer 관리
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.lane_available_last = lane_available
    self.edge_available_last = edge_available

    self.prev_desire_enabled = desire_enabled

    # 운전자가 반대 방향으로 강하게 조향하면 해당 차선변경/턴 취소
    steering_pressed_cancel = carstate.steeringPressed and \
                              ((carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
                               (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT))
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True

    # 최종 desire 결정
    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # keep pulse (LaneChangeState.preLaneChange에서 유지)
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
