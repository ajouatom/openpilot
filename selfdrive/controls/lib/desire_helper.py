from cereal import log, car
from common.conversions import Conversions as CV
from common.realtime import DT_MDL
from common.numpy_fast import interp
from common.params import Params

LaneChangeState = log.LateralPlan.LaneChangeState
LaneChangeDirection = log.LateralPlan.LaneChangeDirection
EventName = car.CarEvent.EventName

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS # 30 * CV.MPH_TO_MS
TURN_CHANGE_SPEED_MAX = 30 * CV.KPH_TO_MS # 30 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
}
DESIRES_TURN = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.turnLeft,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.LateralPlan.Desire.none,
    LaneChangeState.preLaneChange: log.LateralPlan.Desire.none,
    LaneChangeState.laneChangeStarting: log.LateralPlan.Desire.turnRight,
    LaneChangeState.laneChangeFinishing: log.LateralPlan.Desire.none,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.lane_change_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.LateralPlan.Desire.none
    self.turnControlState = False
    self.paramsCount = 0
    self.autoTurnControl = int(Params().get("AutoTurnControl", encoding="utf8"))
    self.autoTurnSpeed = int(Params().get("AutoTurnSpeed", encoding="'utf8"))
    self.autoTurnTimeMax = int(Params().get("AutoTurnTimeMax", encoding="'utf8"))
    self.autoLaneChangeSpeed = int(Params().get("AutoLaneChangeSpeed", encoding="'utf8"))

    self.desireEvent = 0
    self.desireEvent_prev = 0
    self.waitTorqueApply = False


  def update(self, carstate, lateral_active, lane_change_prob, md, turn_prob):
    self.paramsCount += 1
    if self.paramsCount > 100:
      self.autoTurnControl = int(Params().get("AutoTurnControl", encoding="utf8"))
      self.autoTurnSpeed = int(Params().get("AutoTurnSpeed", encoding="'utf8"))
      self.autoTurnTimeMax = int(Params().get("AutoTurnTimeMax", encoding="'utf8"))
      self.autoLaneChangeSpeed = int(Params().get("AutoLaneChangeSpeed", encoding="'utf8"))
      self.paramsCount = 0

    v_ego = carstate.vEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    #로드엣지 읽기..
    left_road_edge = -md.roadEdges[0].y[0]
    right_road_edge = md.roadEdges[1].y[0]
    road_edge_detected = (((left_road_edge < 3.5) and carstate.leftBlinker) or ((right_road_edge < 3.5) and carstate.rightBlinker))

    #레인체인지 또는 자동턴 타임아웃
    laneChangeTimeMax = LANE_CHANGE_TIME_MAX if not self.turnControlState else self.autoTurnTimeMax

    #BSD읽기.
    blindspot_detected = ((carstate.leftBlindspot and carstate.leftBlinker) or(carstate.rightBlindspot and carstate.rightBlinker))

    #핸들토크읽기
    torque_applied = carstate.steeringPressed and \
                        ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                        (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

    steering_pressed = carstate.steeringPressed and \
                        ((carstate.steeringTorque > 0 and carstate.leftBlinker) or
                        (carstate.steeringTorque < 0 and carstate.rightBlinker))

    checkAutoTurnEnabled = self.autoTurnControl > 0
    checkAutoTurnSpeed = (v_ego_kph < self.autoTurnSpeed) and checkAutoTurnEnabled

    # Timeout검사
    if not lateral_active or self.lane_change_timer > laneChangeTimeMax: #LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      #self.lane_change_direction = LaneChangeDirection.none
      self.desireEvent = 0
    else:      
      # 0. 감지 및 결정 단계: LaneChangeState.off: 깜박이와 속도검사. 
      #   - 정지상태: 깜박이를 켜고 있음: 출발할때 검사해야함.
      #   - 저속: 차선변경속도이하: 턴할지, 차로변경할지 결정해야함.
      #   - 중속: 80키로이하: 
      #   - 고속
      if self.lane_change_state == LaneChangeState.off:
        self.desireEvent = 0
        self.desireEvent_prev = 0
        self.lane_change_direction = LaneChangeDirection.none
        self.turnControlState = False
        if one_blinker and (not self.prev_one_blinker or v_ego_kph < 4 or (checkAutoTurnEnabled and steering_pressed)):  ##깜박이가 켜진시점에 검사, 정지상태에서는 lat_active가 아님. 
          # 정지상태, 출발할때
          if v_ego_kph < 4.0:
            if self.autoTurnControl > 0: 
              self.turnControlState = True
              self.lane_change_state = LaneChangeState.preLaneChange
          # 저속
          elif v_ego_kph < self.autoLaneChangeSpeed:
            if road_edge_detected and checkAutoTurnEnabled:
              self.turnControlState = True
              self.lane_change_state = LaneChangeState.preLaneChange
            elif carstate.rightBlinker:
              if checkAutoTurnEnabled:
                if steering_pressed:
                  self.turnControlState = True
                self.lane_change_state = LaneChangeState.preLaneChange
            else: # 좌측저속: 차선검출:차선변경, 차선없음: 진행 (HW:차선검출루틴필요)
              if steering_pressed and checkAutoTurnEnabled:
                self.turnControlState = True
                self.lane_change_state = LaneChangeState.preLaneChange

          # 중속
          elif v_ego_kph < 80.0:
            # 브레이킹하면서 깜박이.... 좌: 좌회전확률이 높음(차로변경차선변경 또는 좌회전). , 우: 우회전확률이 높으나(우측차로변경차선 또는 우회전), 
            # 하지만... 차로변경차선이 있는지 없는지는 알수 없음... 
            # 따라서, 브레이크밟으면서 깜박이는 턴속도가 될때까지, 차선변경 턴은 하지 않도록 하자~
            if carstate.brakePressed:  
              #로드에지가 아닌경우
              if not road_edge_detected:
                if steering_pressed and checkAutoTurnEnabled:
                  self.turnControlState = True                
                  self.lane_change_state = LaneChangeState.preLaneChange
                elif carstate.rightBlinker:
                  self.lane_change_state = LaneChangeState.preLaneChange
                elif carstate.leftBlinker and steering_pressed:
                  self.lane_change_state = LaneChangeState.preLaneChange
              #우, 로드에지
              elif carstate.rightBlinker:
                if checkAutoTurnEnabled and (checkAutoTurnSpeed or steering_pressed):
                  self.turnControlState = True
                self.lane_change_state = LaneChangeState.preLaneChange
              #좌, 로드에지
              else:
                if checkAutoTurnEnabled and (checkAutoTurnSpeed or steering_pressed):
                  self.turnControlState = True
                self.lane_change_state = LaneChangeState.preLaneChange
            else:
              self.lane_change_state = LaneChangeState.preLaneChange
          # 고속: 무조건 차선변경
          else:
            self.lane_change_state = LaneChangeState.preLaneChange

          if self.lane_change_state == LaneChangeState.preLaneChange:
              self.lane_change_ll_prob = 1.0
              self.waitTorqueApply = False

      # 1. 대기단계: LaneChangeState.preLaneChange: 
      elif self.lane_change_state == LaneChangeState.preLaneChange:        
        self.desireEvent = 0
        self.lane_change_pulse_timer += DT_MDL
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        if self.turnControlState:
          if not one_blinker:
            self.lane_change_state = LaneChangeState.off
          elif v_ego_kph < 2.0:
            self.lane_change_pulse_timer = 0.0
          elif self.lane_change_pulse_timer > 0.2: # and not blindspot_detected:
            self.lane_change_state = LaneChangeState.laneChangeStarting
        else:
          # 깜박이가 꺼지거나, 속도가 줄어들면... 차선변경 중지.
          if not one_blinker: #속도가 줄어도 차선변경이 가능하게 함(시험), or v_ego_kph < self.autoLaneChangeSpeed:  
            self.lane_change_state = LaneChangeState.off
          elif self.lane_change_pulse_timer > 0.2:
            if blindspot_detected:
              self.desireEvent = EventName.laneChangeBlocked
              self.waitTorqueApply = True
            elif road_edge_detected: # BSD 또는 road_edge검출이 안되면 차선변경 시작.
              self.desireEvent = EventName.laneChangeRoadEdge
            elif not self.waitTorqueApply:
              self.lane_change_state = LaneChangeState.laneChangeStarting

          # BSD검출시 torque힘을 기다려야 함.
          if self.waitTorqueApply:
            if torque_applied:
              self.lane_change_state = LaneChangeState.laneChangeStarting
            elif self.desireEvent == 0:
              if self.desireEvent_prev > 0:
                self.desireEvent = self.desireEvent_prev
              else:
                self.desireEvent = EventName.preLaneChangeLeft if self.lane_change_direction == LaneChangeDirection.left else EventName.preLaneChangeRight
            self.desireEvent_prev = self.desireEvent

      # 2. LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.desireEvent = EventName.laneChange
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        ## 차선변경시 핸들을 방향으로 건들면... 턴으로 변경, 반대로 하면 취소..
        if not self.turnControlState:
          if steering_pressed and checkAutoTurnSpeed:  # 저속, 차선변경중 같은 방향 핸들토크.
            self.turnControlState = True
          elif carstate.steeringPressed:  # 차선변경중 핸들토크: 무시
            #self.lane_change_state = LaneChangeState.off
            pass
          elif self.lane_change_direction == LaneChangeDirection.right and road_edge_detected and checkAutoTurnSpeed: # 우측차선변경, 로드엣지, 턴속도, 턴~
            self.turnControlState = True
        elif steering_pressed: #턴중 핸들돌림... 무시
          pass
        elif carstate.steeringPressed: # 턴중 반대로 핸들돌림... off
          self.lane_change_state = LaneChangeState.off

        # 98% certainty
        if self.turnControlState:
            if turn_prob < 0.02 and self.lane_change_ll_prob < 0.01:
              self.lane_change_state = LaneChangeState.laneChangeFinishing
        else:
            if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
              self.lane_change_state = LaneChangeState.laneChangeFinishing

      # 3.LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        self.desireEvent = EventName.laneChange
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.turnControlState:
            if self.lane_change_ll_prob > 0.99:
              self.lane_change_direction = LaneChangeDirection.none
            if one_blinker and not carstate.steeringPressed: #깜박이 켜고 있으면.... 아직 턴하고 있는중... 이때 preLaneChange로 넘어가면 계속 턴하려고 함...
              pass
            else:
              self.lane_change_state = LaneChangeState.off
        else:
            if self.lane_change_ll_prob > 0.99:
              self.lane_change_direction = LaneChangeDirection.none
            if one_blinker:
              self.lane_change_state = LaneChangeState.preLaneChange
              self.waitTorqueApply = True
            else:
              self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.laneChangeFinishing, LaneChangeState.off):
      self.lane_change_pulse_timer = 0.0
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
      #self.lane_change_direction = LaneChangeDirection.none
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES_TURN[self.lane_change_direction][self.lane_change_state] if self.turnControlState else DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
      self.waitTorqueApply = False
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none



  def update_(self, carstate, lateral_active, lane_change_prob, md, turn_prob):
    self.paramsCount += 1
    if self.paramsCount > 100:
      self.autoTurnControl = int(Params().get("AutoTurnControl", encoding="utf8"))
      self.autoTurnSpeed = int(Params().get("AutoTurnSpeed", encoding="'utf8"))
      self.autoTurnTimeMax = int(Params().get("AutoTurnTimeMax", encoding="'utf8"))
      self.autoLaneChangeSpeed = int(Params().get("AutoLaneChangeSpeed", encoding="'utf8"))
      self.paramsCount = 0

    v_ego = carstate.vEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    #로드엣지 읽기..
    left_road_edge = -md.roadEdges[0].y[0]
    right_road_edge = md.roadEdges[1].y[0]
    road_edge_detected = (((left_road_edge < 3.5) and carstate.leftBlinker) or ((right_road_edge < 3.5) and carstate.rightBlinker))

    #레인체인지 또는 자동턴 타임아웃
    laneChangeTimeMax = LANE_CHANGE_TIME_MAX if not self.turnControlState else self.autoTurnTimeMax

    #BSD읽기.
    blindspot_detected = ((carstate.leftBlindspot and carstate.leftBlinker) or(carstate.rightBlindspot and carstate.rightBlinker))

    if not lateral_active or self.lane_change_timer > laneChangeTimeMax: #LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.desireEvent = 0
    else:
      self.lane_change_direction = LaneChangeDirection.none
      # LaneChangeState.off: 깜박이와 속도검사.      
      if self.lane_change_state == LaneChangeState.off:
        self.desireEvent = 0
        #자동턴 조건, 깜박이ON, 저속이거나, 자동턴속도&브레이크 밟힘 #and not BSD.
        if self.autoTurnControl>0 and one_blinker and (below_lane_change_speed or ((v_ego_kph < self.autoTurnSpeed) and carstate.brakePressed)):# and not blindspot_detected: 
           self.lane_change_state = LaneChangeState.preLaneChange
           self.lane_change_ll_prob = 1.0
           self.turnControlState = True
        #차선변경조건: 깜박이OFF->ON 그리고 차선변경속도 => 차선변경준비 and not BSD, (로드엣지는 preLaneChange에서 대기중 시작하도록 함)
        elif one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
          if blindspot_detected: 
            self.desireEvent = EventName.laneChangeBlocked
          else:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.turnControlState = False

      #if self.lane_change_state == LaneChangeState.off and one_blinker and (not self.prev_one_blinker or autoTurnControl>0) and not below_lane_change_speed:
      #  self.lane_change_state = LaneChangeState.preLaneChange
      #  self.lane_change_ll_prob = 1.0

      # LaneChangeState.preLaneChange: 
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        self.desireEvent = EventName.preLaneChangeLeft if LaneChangeDirection.left else EventName.preLaneChangeRight
        self.lane_change_pulse_timer += DT_MDL
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        #blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
        #                      (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        #road_edge_detected = (((left_road_edge < 3.5) and self.lane_change_direction == LaneChangeDirection.left) or
        #                      ((right_road_edge < 3.5) and self.lane_change_direction == LaneChangeDirection.right))

        if self.turnControlState:
          if not one_blinker:
            self.lane_change_state = LaneChangeState.off
          elif self.lane_change_pulse_timer > 0.1: # and not blindspot_detected:
            self.lane_change_state = LaneChangeState.laneChangeStarting
        else:
          # 깜박이가 꺼지거나, 속도가 줄어들면... 차선변경 중지.
          if not one_blinker or below_lane_change_speed:
            self.lane_change_state = LaneChangeState.off
          elif self.lane_change_pulse_timer > 0.1:
            if blindspot_detected or road_edge_detected: # BSD 또는 road_edge검출이 안되면 차선변경 시작.
              self.desireEvent = EventName.laneChangeBlocked
              self.lane_change_state = LaneChangeState.off
              self.desireEvent = 0
            else:
              self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.desireEvent = EventName.laneChange
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if self.turnControlState:
            if turn_prob < 0.02 and self.lane_change_ll_prob < 0.01:
              self.lane_change_state = LaneChangeState.laneChangeFinishing
        else:
            if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
              self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        self.desireEvent = EventName.laneChange
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.turnControlState:
            if self.lane_change_ll_prob > 0.99:
                self.lane_change_direction = LaneChangeDirection.none
            if one_blinker: #깜박이 켜고 있으면.... 아직 턴하고 있는중... 이때 preLaneChange로 넘어가면 계속 턴하려고 함...
                pass
            else:
                self.lane_change_state = LaneChangeState.off
        else:
            if self.lane_change_ll_prob > 0.99:
                self.lane_change_direction = LaneChangeDirection.none
            if one_blinker:
                self.lane_change_state = LaneChangeState.preLaneChange
            else:
                self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.laneChangeFinishing, LaneChangeState.off):
      self.lane_change_pulse_timer = 0.0
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
      self.lane_change_direction = LaneChangeDirection.none
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES_TURN[self.lane_change_direction][self.lane_change_state] if self.turnControlState else DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.LateralPlan.Desire.keepLeft, log.LateralPlan.Desire.keepRight):
        self.desire = log.LateralPlan.Desire.none
