from cereal import log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_MDL

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
  index = 10 #�� 1�� ���� ����..
  distance_to_lane = abs(current_lane.y[index] - lane.y[index])
  #if lane_prob < 0.3: # ������ ������ ���°����� ���ֽ�Ŵ.
  #  distance_to_lane = min(2.0, distance_to_lane)
  distance_to_road_edge = abs(current_lane.y[index] - road_edge.y[index]);
  return min(distance_to_lane, distance_to_road_edge), distance_to_road_edge, lane_prob > 0.5
class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.atc_active = 0
    self.desireLog = ""
    self.lane_width_left = 0
    self.lane_width_right = 0
    self.distance_to_road_edge_left = 0
    self.distance_to_road_edge_right = 0
    self.blinker_bypass = False
    
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan):
    v_ego = carstate.vEgo
    #one_blinker = carstate.leftBlinker != carstate.rightBlinker
    leftBlinker = carstate.leftBlinker
    rightBlinker = carstate.rightBlinker
    one_blinker = leftBlinker != rightBlinker
    user_one_blinker = one_blinker
    
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not leftBlinker and not rightBlinker:
      atc_type = carrotMan.atcType
      if atc_type in ["turn left", "turn right"]:
        if self.atc_active != 2:
          below_lane_change_speed = True
          self.lane_change_timer = 0.0
          leftBlinker = True if atc_type == "turn left" else False
          rightBlinker = not leftBlinker          
          self.atc_active = 1
          self.blinker_bypass = False
      elif atc_type in ["fork left", "fork right"]:
        if self.atc_active != 2:
          leftBlinker = True if atc_type == "fork left" else False
          rightBlinker = not leftBlinker
          self.atc_active = 1
      else:
        self.atc_active = 0
      one_blinker = leftBlinker != rightBlinker
    if not one_blinker:
      self.blinker_bypass = False
    one_blinker &= not self.blinker_bypass
    self.lane_width_left, self.distance_to_road_edge_left, lane_exist_left = calculate_lane_width(modeldata.laneLines[0], modeldata.laneLineProbs[0], modeldata.laneLines[1], modeldata.roadEdges[0])
    self.lane_width_right, self.distance_to_road_edge_right, lane_exist_right = calculate_lane_width(modeldata.laneLines[3], modeldata.laneLineProbs[3], modeldata.laneLines[2], modeldata.roadEdges[1])

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
    elif one_blinker and below_lane_change_speed and not carstate.standstill and self.enable_turn_desires:
      self.lane_change_state = LaneChangeState.off
      self.turn_direction = TurnDirection.turnLeft if leftBlinker else TurnDirection.turnRight
      self.lane_change_direction = self.turn_direction #LaneChangeDirection.none
    else:
      self.turn_direction = TurnDirection.none     
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0

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

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif user_one_blinker and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
        elif torque_applied and not blindspot_detected:
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
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker
    steering_pressed = carstate.steeringPressed and \
                     ((carstate.steeringTorque < 0 and leftBlinker) or (carstate.steeringTorque > 0 and rightBlinker))
    if steering_pressed and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_bypass = True

    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    self.desireLog = f"desire = {self.desire}"
      
    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
