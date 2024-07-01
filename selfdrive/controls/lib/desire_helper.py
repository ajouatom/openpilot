from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL
import numpy as np
from openpilot.common.params import Params
from enum import Enum

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

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

class NooActive(Enum):
  inactive = 0
  active = 1
  no_new_lane_detected = 2
  new_lane_detected = 10
  road_edge_detected = 11
  new_lane_appeared = 12

  def __str__(self):
    return self.name


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
  index = 10 #약 1초 앞의 차선..
  distance_to_lane = abs(current_lane.y[index] - lane.y[index])
  #if lane_prob < 0.3: # 차선이 없으면 없는것으로 간주시킴.
  #  distance_to_lane = min(2.0, distance_to_lane)
  distance_to_road_edge = abs(current_lane.y[index] - road_edge.y[index]);
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, lane_prob > 0.5

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
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    self.turn_desires = True  #항상 True
    self.lane_detection = True #항상 True
    self.one_lane_change = True
    self.lane_change_delay = 0.0
    
    self.turn_direction = TurnDirection.none

    self.lane_change_completed = False
    self.turn_completed = False

    self.lane_change_wait_timer = 0
    self.lane_width_left = 0
    self.lane_width_right = 0
    self.distance_to_road_edge_left = 0
    self.distance_to_road_edge_right = 0

    self.lane_change_wait_timer = 0

    self.lane_available_prev = False
    self.edge_available_prev = False
    self.lane_exist_left_count = ExistCounter()
    self.lane_exist_right_count = ExistCounter()
    self.lane_width_left_count = ExistCounter()
    self.lane_width_right_count = ExistCounter()
    self.road_edge_left_count = ExistCounter()
    self.road_edge_right_count = ExistCounter()

    self.blinker_bypass = False

    self.available_left_lane = False
    self.available_right_lane = False
    self.available_left_edge = False
    self.available_right_edge = False

    self.object_detected_count = 0
    self._log_timer = 0
    self.debugText = ""
    self.noo_active = NooActive.inactive
    self.params = Params()
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoLaneChangeSpeed = self.params.get_int("AutoLaneChangeSpeed") / 3.6

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.debugText = ""
    else:
      self.debugText = log
      self._log_timer = int(2/DT_MDL) # 2s

  #def update_exist_count(self, counter, exist):
  #  return max(counter + 1, 1) if exist else min(counter - 1, -1)

  def update(self, carstate, modeldata, lateral_active, lane_change_prob, sm):
    self._add_log("")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.laneChangeNeedTorque = self.params.get_bool("LaneChangeNeedTorque")
    self.laneChangeLaneCheck = self.params.get_bool("LaneChangeLaneCheck")    # 0: No check, 1: Lane Only, 2: use Edge
    self.autoLaneChangeSpeed = self.params.get_int("AutoLaneChangeSpeed") / 3.6
    radarState = sm['radarState']
    self.leftSideObjectDist = 255
    self.rightSideObjectDist = 255
    if radarState.leadLeft.status:
      self.leftSideObjectDist = radarState.leadLeft.dRel + radarState.leadLeft.vLead * 4.0
    if radarState.leadRight.status:
      self.rightSideObjectDist = radarState.leadRight.dRel + radarState.leadRight.vLead * 4.0

    leftBlinkerExt = sm['longitudinalPlan'].leftBlinkerExt
    rightBlinkerExt = sm['longitudinalPlan'].rightBlinkerExt
    if leftBlinkerExt + rightBlinkerExt == 0:
      leftBlinkerExt = sm['controlsState'].leftBlinkerExt
      rightBlinkerExt = sm['controlsState'].rightBlinkerExt
    blinkerExtMode = int((leftBlinkerExt + rightBlinkerExt) / 20000)  ## 둘다 10000 or 20000이 + 되어 있으므로,, 10000이 아니라 20000으로 나누어야함.
    leftBlinkerExt %= 10000
    rightBlinkerExt %= 10000

    v_ego = carstate.vEgo
    leftBlinker = carstate.leftBlinker or leftBlinkerExt > 0
    rightBlinker = carstate.rightBlinker or rightBlinkerExt > 0
    one_blinker = leftBlinker != rightBlinker
    if not one_blinker:
      self.blinker_bypass = False
    one_blinker &= not self.blinker_bypass
    below_lane_change_speed = v_ego < self.autoLaneChangeSpeed if blinkerExtMode in [0,2] else v_ego < 5. * CV.KPH_TO_MS  ## carrot, when auto turn...

   
      # Calculate left and right lane widths
    self.lane_width_left, self.distance_to_road_edge_left, lane_exist_left = calculate_lane_width(modeldata.laneLines[0], modeldata.laneLineProbs[0], modeldata.laneLines[1], modeldata.roadEdges[0])
    self.lane_width_right, self.distance_to_road_edge_right, lane_exist_right = calculate_lane_width(modeldata.laneLines[3], modeldata.laneLineProbs[3], modeldata.laneLines[2], modeldata.roadEdges[1])

    self.lane_exist_left_count.update(lane_exist_left)
    self.lane_exist_right_count.update(lane_exist_right)
    self.lane_width_left_count.update(self.lane_width_left > 2.5)
    self.lane_width_right_count.update(self.lane_width_right > 2.5)
    self.road_edge_left_count.update(self.distance_to_road_edge_left > 2.5)
    self.road_edge_right_count.update(self.distance_to_road_edge_right > 2.5)

    #self.lane_exist_left_count = self.update_exist_count(self.lane_exist_left_count, lane_exist_left)
    #self.lane_exist_right_count = self.update_exist_count(self.lane_exist_right_count, lane_exist_right)
    #self.lane_width_left_count = self.update_exist_count(self.lane_width_left_count, self.lane_width_left > 2.5)
    #self.lane_width_right_count = self.update_exist_count(self.lane_width_right_count, self.lane_width_right > 2.5)
    #self.road_edge_left_count = self.update_exist_count(self.road_edge_left_count, self.distance_to_road_edge_left > 2.5)
    #self.road_edge_right_count = self.update_exist_count(self.road_edge_right_count, self.distance_to_road_edge_right > 2.5)

    available_count = int(0.2 / DT_MDL)
    self.available_left_lane = self.lane_width_left_count.counter > available_count
    self.available_right_lane = self.lane_width_right_count.counter > available_count
    self.available_left_edge = self.road_edge_left_count.counter > available_count
    self.available_right_edge = self.road_edge_right_count.counter > available_count


    # Calculate the desired lane width for nudgeless lane change with lane detection
    if not (self.lane_detection and one_blinker) or below_lane_change_speed:
      lane_available = True
      edge_available = False
      lane_appeared = False
    else:
      lane_available = self.available_left_lane if leftBlinker else self.available_right_lane
      edge_available = self.available_left_edge if leftBlinker else self.available_right_edge
      lane_appeared = self.lane_exist_left_count.counter == int(0.2 / DT_MDL) if leftBlinker else self.lane_exist_right_count.counter == int(0.2 / DT_MDL)

      if self.laneChangeLaneCheck == 0: # 차선이 항상 존재하는것으로 처리함.
        lane_available = True if edge_available else False
        edge_available = False
      elif self.laneChangeLaneCheck == 1: # 차선있을때만, edge는 항상작동안함.
        edge_available = False
      elif self.laneChangeLaneCheck == 2:  # 차선있을때, edge도 차선변경가능
        pass

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
    elif one_blinker and ((below_lane_change_speed and self.turn_desires and blinkerExtMode in [0]) or blinkerExtMode == 2):
      self.turn_direction = TurnDirection.turnLeft if leftBlinker else TurnDirection.turnRight
      # Set the "turn_completed" flag to prevent lane changes after completing a turn
      self.turn_completed = True
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self._add_log("Lane change turning.. {}".format("left" if leftBlinker else "right"))
    else:
      # TurnDirection.turnLeft / turnRight
      self.turn_direction = TurnDirection.none

      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_wait_timer = 0
        self.noo_active = NooActive.inactive
        self._add_log("Lane change prepare.. {}".format("left" if leftBlinker else "right"))

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        object_dist = v_ego * 3.0
        object_detected = ((self.leftSideObjectDist < object_dist and self.lane_change_direction == LaneChangeDirection.left) or
                           (self.rightSideObjectDist < object_dist and self.lane_change_direction == LaneChangeDirection.right))
        self.object_detected_count = max(1, self.object_detected_count + 1) if object_detected else min(-1, self.object_detected_count - 1)

        # Conduct a nudgeless lane change if all the conditions are true
        self.lane_change_wait_timer += DT_MDL

        ## nooHelper인경우 차선이 생기면 하면 됨. (수동개입이 없는경우에만)        
        if (not carstate.leftBlinker and not carstate.rightBlinker) and blinkerExtMode > 0: # Noo Helper #0: voice etc, 1:noo helper lanechange, 2: noo helper turn
          if self.autoTurnControl == 3 or leftBlinker:
            self.noo_active = NooActive.active
          elif lane_appeared: #start... 차선이 발견됨.
            self.noo_active = NooActive.new_lane_appeared
          elif not self.lane_available_prev and lane_available: # start... 차선이 생김 (로드경계가 멀어짐)
            self.noo_active = NooActive.new_lane_detected
          elif not self.edge_available_prev and edge_available: # start... 에지가 멀어짐. 
            self.noo_active = NooActive.road_edge_detected
          elif self.noo_active.value < 10 and self.lane_available_prev and lane_available: #차선이 계속있음.
            self.noo_active = NooActive.no_new_lane_detected
          #else: #if not edge_available: #에지가 가까움.
          #  self.noo_active = 4
        else:
          self.noo_active = NooActive.inactive

        if self.object_detected_count > -0.3 / DT_MDL:  # 0.5 sec
          self._add_log("Lane change object detected.. {:.1f}m".format(self.leftSideObjectDist if leftBlinker else self.rightSideObjectDist))
        elif not lane_available and self.noo_active.value < 10:
          self._add_log("Lane change no lane available")
        elif self.noo_active == NooActive.active or self.laneChangeNeedTorque:
          self._add_log("Lane change blocked. need torque")
        elif self.noo_active == NooActive.no_new_lane_detected:
          self._add_log("Lane change blocked. not end lane")
        elif self.lane_change_completed:
          self._add_log("Lane change need torque to start")
        elif self.lane_change_wait_timer < self.lane_change_delay:
          self._add_log("Lane change waiting timer. {:.1f}s".format(self.lane_change_wait_timer))
        elif blindspot_detected:
          self._add_log("Blindspot detected")
        else:
        #if not object_detected and not need_torque and lane_available and not self.lane_change_completed and self.lane_change_wait_timer >= self.lane_change_delay:          
          torque_applied = True
          self.lane_change_wait_timer = 0

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self._add_log("Lane change canceled.. {}".format("no blinker" if not one_blinker else "low speed"))
        elif torque_applied and not blindspot_detected:
          # Set the "lane_change_completed" flag to prevent any more lane changes if the toggle is on
          self.lane_change_completed = self.one_lane_change
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self._add_log("Lane change starting.. {}, noo={}".format("left" if leftBlinker else "right", str(self.noo_active)))

        #self._add_log("DesireLog.. Object {:.1f}:{:.1f}:{:.1f}".format(self.leftSideObjectDist, object_dist, self.rightSideObjectDist))
      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        #self._add_log("Lane change starting.. {}, {}".format("left" if leftBlinker else "right", str(self.noo_active)))
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        self._add_log("Lane change finishing.. ")
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self._add_log("Lane change continued.")
          else:
            self.lane_change_state = LaneChangeState.off
            self._add_log("Lane change finished.")

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker
    self.lane_available_prev = lane_available
    self.edge_available_prev = edge_available
    
    steering_pressed = carstate.steeringPressed and \
                     ((carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.left) or
                      (carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.right))
    if steering_pressed:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_bypass = True
      self._add_log("Lane change steering pressed.. canceled")
    
    # Reset the flags
    self.lane_change_completed &= one_blinker
    self.turn_completed &= one_blinker

    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
    elif not self.turn_completed:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    else:
      self.desire = log.Desire.none

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
