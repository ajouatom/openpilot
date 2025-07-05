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
  t = 1.0 # 약 1초 앞의 차선.
  current_lane_y = np.interp(t, ModelConstants.T_IDXS, current_lane.y)
  lane_y = np.interp(t, ModelConstants.T_IDXS, lane.y)
  distance_to_lane = abs(current_lane_y - lane_y)
  #if lane_prob < 0.3:# 차선이 없으면 없는것으로 간주시킴.
  #  distance_to_lane = min(2.0, distance_to_lane)
  road_edge_y = np.interp(t, ModelConstants.T_IDXS, road_edge.y)
  distance_to_road_edge = abs(current_lane_y - road_edge_y)
  distance_to_road_edge_far = abs(current_lane_y - np.interp(2.0, ModelConstants.T_IDXS, road_edge.y))
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, distance_to_road_edge_far, lane_prob > 0.5

class ExistCounter:
  def __init__(self):
    self.counter = 0
    self.true_count = 0
    self.false_count = 0
    self.threshold = int(0.2 / DT_MDL)  # 노이즈를 무시하기 위한 임계값 설정

  def update(self, exist_flag):
    if exist_flag:
      self.true_count += 1
      self.false_count = 0  # false count 초기화
      if self.true_count >= self.threshold:
          self.counter = max(self.counter + 1, 1)
    else:
      self.false_count += 1
      self.true_count = 0  # true count 초기화
      if self.false_count >= self.threshold:
          self.counter = min(self.counter - 1, -1)

    return self.true_count

class DesireHelper:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_desire_enabled = False
    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.atc_active = 0
    self.desireLog = ""
    self.lane_width_left = 0
    self.lane_width_right = 0
    self.lane_width_left_diff = 0
    self.lane_width_right_diff = 0
    self.distance_to_road_edge_left = 0
    self.distance_to_road_edge_right = 0
    self.distance_to_road_edge_left_far = 0
    self.distance_to_road_edge_right_far = 0
    self.blinker_ignore = False

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
    self.lane_width_left_queue = deque(maxlen=int(1.0/DT_MDL))
    self.lane_width_right_queue = deque(maxlen=int(1.0/DT_MDL))

    self.lane_available_last = False
    self.edge_available_last = False
    self.object_detected_count = 0    

    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneChangeDelay = 0
    self.lane_change_delay = 0.0
    self.driver_blinker_state = BLINKER_NONE
    self.atc_type = ""

    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.carrot_blinker_state = BLINKER_NONE

    self.turn_desire_state = False
    self.desire_disable_count = 0
    self.blindspot_detected_counter = 0
    self.auto_lane_change_enable = False

  def check_lane_state(self, modeldata):
    lane_width_left, self.distance_to_road_edge_left, self.distance_to_road_edge_left_far, lane_prob_left = calculate_lane_width(modeldata.laneLines[0], modeldata.laneLineProbs[0],
                                                                                                 modeldata.laneLines[1], modeldata.roadEdges[0])
    lane_width_right, self.distance_to_road_edge_right, self.distance_to_road_edge_right_far, lane_prob_right = calculate_lane_width(modeldata.laneLines[3], modeldata.laneLineProbs[3],
                                                                                                    modeldata.laneLines[2], modeldata.roadEdges[1])
    self.lane_exist_left_count.update(lane_prob_left)
    self.lane_exist_right_count.update(lane_prob_right)
    
    self.lane_width_left_queue.append(lane_width_left)
    self.lane_width_right_queue.append(lane_width_right)
    self.lane_width_left = np.mean(self.lane_width_left_queue)
    self.lane_width_right = np.mean(self.lane_width_right_queue)
    self.lane_width_left_diff = self.lane_width_left_queue[-1] - self.lane_width_left_queue[0]
    self.lane_width_right_diff = self.lane_width_right_queue[-1] - self.lane_width_right_queue[0]
    
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

  def check_desire_state(self, modeldata):
    desire_state  = modeldata.meta.desireState
    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1
    if self.turn_desire_state:
      self.desire_disable_count = int(2.0/DT_MDL)
    else:
      self.desire_disable_count = max(0, self.desire_disable_count - 1)
    #print(f"desire_state = {desire_state}, turn_desire_state = {self.turn_desire_state}, disable_count = {self.desire_disable_count}")

  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):

    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd = self.params.get_int("LaneChangeBsd")
      self.laneChangeDelay = self.params.get_float("LaneChangeDelay") * 0.1
    self.frame += 1

    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay = max(0, self.lane_change_delay - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    ##### check lane state
    self.check_lane_state(modeldata)
    self.check_desire_state(modeldata)

    #### check driver's blinker state
    driver_blinker_state = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    driver_blinker_changed = driver_blinker_state != self.driver_blinker_state
    self.driver_blinker_state = driver_blinker_state
    driver_desire_enabled = driver_blinker_state in [BLINKER_LEFT, BLINKER_RIGHT]
    if self.laneChangeNeedTorque < 0: # 운전자가 깜박이 켜도 차선변경 안함.
      driver_desire_enabled = False

    ignore_bsd = True if self.laneChangeBsd < 0 else False
    block_lanechange_bsd = True if self.laneChangeBsd == 1 else False

    self.blindspot_detected_counter = max(0, self.blindspot_detected_counter - 1)

    ##### check ATC's blinker state
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE
    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      #print(f"Desire lanechange: {carrotMan.carrotArg}")
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
    elif atc_type in ["turn left", "turn right"]:
      if self.atc_active != 2:
        below_lane_change_speed = True
        self.lane_change_timer = 0.0
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
    elif atc_type in ["fork left", "fork right", "atc left", "atc right"]:
      if self.atc_active != 2:
        below_lane_change_speed = False
        atc_blinker_state = BLINKER_LEFT if atc_type in ["fork left", "atc left"] else BLINKER_RIGHT        
        self.atc_active = 1
    else:
      self.atc_active = 0
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2
    atc_desire_enabled = atc_blinker_state in [BLINKER_LEFT, BLINKER_RIGHT]

    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      driver_blinker_state = BLINKER_NONE
      atc_blinker_state = BLINKER_NONE
      driver_desire_enabled = False

    if self.atc_type != atc_type:
      atc_desire_enabled = False

    self.atc_type = atc_type

    desire_enabled = driver_desire_enabled or atc_desire_enabled
    blinker_state = driver_blinker_state if driver_desire_enabled else atc_blinker_state
    
    if desire_enabled:
      lane_exist_counter = self.lane_exist_left_count.counter if blinker_state == BLINKER_LEFT else self.lane_exist_right_count.counter
      lane_available = self.available_left_lane if blinker_state == BLINKER_LEFT else self.available_right_lane
      edge_available = self.available_left_edge if blinker_state == BLINKER_LEFT else self.available_right_edge
      lane_appeared = lane_exist_counter == int(0.2 / DT_MDL)

      radar = radarState.leadLeft if blinker_state == BLINKER_LEFT else radarState.leadRight
      side_object_dist = radar.dRel + radar.vLead * 4.0 if radar.status else 255
      object_detected = side_object_dist < v_ego * 3.0
      self.object_detected_count = max(1, self.object_detected_count + 1) if object_detected else min(-1, self.object_detected_count - 1)

    else:
      lane_exist_counter = 0
      lane_available = True
      edge_available = True
      lane_appeared = False
      self.object_detected_count = 0

    #lane_available_trigger = not self.lane_available_last and lane_available
    lane_change_available = lane_available or edge_available  
    lane_available_trigger = False
    lane_width_diff = self.lane_width_left_diff if atc_blinker_state == BLINKER_LEFT else self.lane_width_right_diff
    distance_to_road_edge = self.distance_to_road_edge_left if atc_blinker_state == BLINKER_LEFT else self.distance_to_road_edge_right
    lane_width_side = self.lane_width_left if atc_blinker_state == BLINKER_LEFT else self.lane_width_right
    if lane_width_diff > 0.8 and (lane_width_side < distance_to_road_edge):
      lane_available_trigger = True
    edge_availabled = not self.edge_available_last and edge_available
    side_object_detected = self.object_detected_count > -0.3 / DT_MDL
    lane_appeared = lane_appeared and distance_to_road_edge < 4.0


    if self.carrot_lane_change_count > 0:
      auto_lane_change_blocked = False
      auto_lane_change_trigger = lane_change_available
    else:
      auto_lane_change_blocked = ((atc_blinker_state == BLINKER_LEFT) and (driver_blinker_state != BLINKER_LEFT))
      #auto_lane_change_trigger = not auto_lane_change_blocked and edge_available and (lane_available_trigger or edge_availabled or lane_appeared) and not side_object_detected
      auto_lane_change_trigger = self.auto_lane_change_enable and not auto_lane_change_blocked and edge_available and (lane_available_trigger or lane_appeared) and not side_object_detected
      self.desireLog = f"L:{self.auto_lane_change_enable},{auto_lane_change_blocked},E:{lane_available},{edge_available},A:{lane_available_trigger},{lane_appeared},{lane_width_diff:.1f},{lane_width_side:.1f},{distance_to_road_edge:.1f}={auto_lane_change_trigger}"

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      #print("Desire canceled")
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
    elif desire_enabled and ((below_lane_change_speed and not carstate.standstill and self.enable_turn_desires) or self.turn_desire_state):
      #print("Desire Turning")
      self.lane_change_state = LaneChangeState.off
      self.turn_direction = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
      self.lane_change_direction = self.turn_direction #LaneChangeDirection.none
      desire_enabled = False
    elif self.desire_disable_count > 0: # Turn 후 일정시간 동안 차선변경 불가능
      #print("Desire after turning")
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
    else:
      #print(f"Desire LaneChange below={below_lane_change_speed}, lane_change_state={self.lane_change_state}, desire_enabled={desire_enabled},{self.prev_desire_enabled} ")
      self.turn_direction = TurnDirection.none
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and desire_enabled and not self.prev_desire_enabled and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_delay = self.laneChangeDelay

        # 맨끝차선이 아니면(측면에 차선이 있으면), ATC 자동작동 안함.
        #self.auto_lane_change_enable = False if lane_exist_counter > 0 else True
        self.auto_lane_change_enable = False if lane_exist_counter > 0 or lane_change_available else True
         

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          blinker_state == BLINKER_LEFT else LaneChangeDirection.right

        dir_map = {
            LaneChangeDirection.left:  (carstate.steeringTorque > 0, carstate.leftBlindspot),
            LaneChangeDirection.right: (carstate.steeringTorque < 0, carstate.rightBlindspot),
        }
        torque_cond, blindspot_cond = dir_map.get(self.lane_change_direction, (False, False))
        torque_applied = carstate.steeringPressed and torque_cond
        blindspot_detected = blindspot_cond

        if not lane_available or lane_exist_counter < int(2.0 / DT_MDL): #lane_exist_counter > int(0.2 / DT_MDL) and not lane_change_available:
          self.auto_lane_change_enable = True

        if blindspot_detected and not ignore_bsd:
          self.blindspot_detected_counter = int(1.5 / DT_MDL)
          # BSD검출시.. 아래 두줄로 자동차선변경 해제함.. 위험해서 자동차선변경기능은 안하는걸로...
          #self.lane_change_state = LaneChangeState.off
          #self.lane_change_direction = LaneChangeDirection.none
        if not desire_enabled or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        else:
          if lane_change_available and self.lane_change_delay == 0:
            if self.blindspot_detected_counter > 0 and not ignore_bsd:  # BSD검출시
              if torque_applied and not block_lanechange_bsd:
                self.lane_change_state = LaneChangeState.laneChangeStarting
            elif self.laneChangeNeedTorque > 0: # 조향토크필요
              if torque_applied:
                self.lane_change_state = LaneChangeState.laneChangeStarting
            elif driver_desire_enabled:
              self.lane_change_state = LaneChangeState.laneChangeStarting
            # ATC작동인경우 차선이 나타나거나 차선이 생기면 차선변경 시작
            # lane_appeared: 차선이 생기는건 안함.. 위험.
            elif torque_applied or auto_lane_change_trigger:
              self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if desire_enabled:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL


    self.lane_available_last = lane_available
    self.edge_available_last = edge_available

    self.prev_desire_enabled = desire_enabled
    steering_pressed = carstate.steeringPressed and \
                     ((carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT))
    if steering_pressed and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True

    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    #print(f"desire = {self.desire}")
    #self.desireLog = f"desire = {self.desire}"
    #self.desireLog = f"rlane={self.distance_to_road_edge_right:.1f},{self.distance_to_road_edge_right_far:.1f}"

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
