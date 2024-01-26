from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, DT_CTRL
from enum import Enum
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.modeld.constants import ModelConstants
import numpy as np
from common.filter_simple import StreamingMovingAverage

class CarrotSpeedControllerParams:
  def __init__(self):
    self.params = Params()
    self.params_count = 0
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = 2
    self.autoTurnControl = Params().get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = Params().get_int("AutoTurnControlTurnEnd")
    self.autoTurnMapChange = Params().get_int("AutoTurnMapChange")
    self.autoTurnControlSpeedLaneChange = Params().get_int("AutoTurnControlSpeedLaneChange")
    self.autoTurnControlSpeedTurn = Params().get_int("AutoTurnControlSpeedTurn")

  def update(self):
    self.params_count += 1
    if self.params_count == 10:
      self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
      self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
      self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
      self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
      self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
      self.autoNaviSpeedCtrl = 2
    elif self.params_count == 40:
      self.autoTurnControl = Params().get_int("AutoTurnControl")
      self.autoTurnControlTurnEnd = Params().get_int("AutoTurnControlTurnEnd")
      self.autoTurnMapChange = Params().get_int("AutoTurnMapChange")
      self.autoTurnControlSpeedLaneChange = Params().get_int("AutoTurnControlSpeedLaneChange")
      self.autoTurnControlSpeedTurn = Params().get_int("AutoTurnControlSpeedTurn")
    elif self.params_count >= 50:
      self.params_count = 0

def decelerate_for_speed_camera(safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):
  if left_dist <= 0:
    return 250
  if left_dist <= safe_dist:
    return safe_speed
  temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
  dV = (-safe_speed + math.sqrt(temp)) * decel_rate
  apply_speed = min(250 , safe_speed + dV)
  min_speed = prev_apply_speed - (decel_rate * 1.2) * 2 * 0.01 #DT_CTRL
  apply_speed = max(apply_speed, min_speed)
  return apply_speed

class SpeedType(Enum):
  CAR = 1
  CAM = 2
  NAVI = 3
  TBT = 4
  TBT_NEXT = 5
  def __str__(self):
    return self.name


class SpeedCtrlData:
  def __init__(self, source, params):
    self.params = params
    self.source = source
    self.active = False
    self.remain_dist = 0
    self.safe_speed = 0   #kph
    self.apply_speed = 0  #kph

    self.turn = False
    self.direction = 0

  def reset(self):
    self.apply_speed = 250
    self.turn = False
    self.direction = 0

  def set(self, direction=0, turn=False):
    self.direction = direction
    self.turn = turn

  def update_distance(self, delta_dist):
    self.remain_dist = max(0, self.remain_dist - delta_dist)
    if self.remain_dist == 0:
      self.reset()

  def update(self, safe_dist, safe_speed, remain_dist, v_cruise_kph, updated):
    self.turn = False
    self.direction = 0
    if updated:
      self.remain_dist = remain_dist
    self.apply_speed = decelerate_for_speed_camera(self.safe_speed/3.6, safe_dist, v_cruise_kph/3.6, self.params.autoNaviSpeedDecelRate, self.remain_dist) * 3.6
    if 0 < self.apply_speed < 250:
      self.active = True

class CarrotSpeedController:
  def __init__(self):
    self.params = CarrotSpeedControllerParams();
    self.speed_data = {
      SpeedType.CAR: SpeedCtrlData("car", self.params),
      SpeedType.CAM: SpeedCtrlData("cam", self.params),
      SpeedType.NAVI: SpeedCtrlData("navi", self.params),
      SpeedType.TBT: SpeedCtrlData("tbt", self.params),
      SpeedType.TBT_NEXT: SpeedCtrlData("tbt_next", self.params)
    }
    self.distance_traveled = 0
    self.roadSpeed = 30
    self.activeAPM = 0
    self.turn_dist = 50
    self.turn_speed = 30
    self.laneChange_dist = 160
    self.laneChange_speed = 60
    self.nooHelperActivated = 0
    self.nooHelperActivateCount = 0
    self.blinkerExtMode = 0 # 0: Normal, 10000: voice
    self.rightBlinkerExtCount = 0
    self.leftBlinkerExtCount = 0
    self.debugText = ""
    self._log_timer = 0

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.debugText = ""
    else:
      self.debugText = log
      self._log_timer = 300

  def get_apply_speed(self):
    data = min(self.speed_data.values(), key=lambda data: data.apply_speed)
    return data.apply_speed, data.source, data.direction, data.turn, data.remain_dist
    #return min(self.speed_data.values(), key=lambda data: data.apply_speed)

  def _update_car(self, CS, controls, v_cruise_kph):
    ## from CAR
    speed_data = self.speed_data[SpeedType.CAR]
    if CS.speedLimit > 0 and CS.speedLimitDistance > 0:
      speed_data.update(self.params.autoNaviSpeedCtrlEnd * v_ego, CS.speedLimit, CS.speedLimitDistance, v_cruise_kph)
    else:
      speed_data.reset()
    speed_data.set()

  def _update_cam(self, CS, controls, v_cruise_kph):
    ## from SpeedCam
    msg = self.roadLimitSpeed = controls.sm['roadLimitSpeed']
    updated = controls.sm.updated['roadLimitSpeed']
    camType = int(msg.camType)
    xSignType = msg.xSignType
    safeSpeed = msg.xSpdLimit * self.params.autoNaviSpeedSafetyFactor
    speed_data = self.speed_data[SpeedType.CAM]
    if camType == 22 or xSignType == 22:
      speed_data.update(self.autoNaviSpeedBumpTime * v_ego, self.params.autoNaviSpeedBumpSpeed, msg.xSpdDist, v_cruise_kph)
    elif msg.xSpdLimit > 0 and msg.xSpdDist > 0:
      isSectionLimit = True if xSignType==165 or leftDist > 3000 or camType == 4 else False
      isSectionLimit = False if leftDist < 50 else isSectionLimit
      speed_data.update(self.autoNaviSpeedCtrlEnd * v_ego, self. safeSpeed, msg.xSpdDist, v_cruise_kph)
      if isSectionLimit and speed_data.apply_speed > safeSpeed:
        speed_data.apply_speed = safeSpeed
    else:
      speed_data.reset()
      return False

    speed_data.set()
    return True

  def _update_navi(self, CS, controls, v_cruise_kph):
    navInstruction = controls.sm['navInstruction']      
    nav_type = navInstruction.maneuverType;
    nav_modifier = navInstruction.maneuverModifier;
    nav_turn = False
    nav_speed_down = False
    direction = 0 #1:left, 2:right
    speed_data = self.speed_data[SpeedType.NAVI]
    nav_distance = speed_data.remain_dist
    if nav_type in ['turn', 'fork', 'off ramp']:
      if controls.sm.updated['navInstruction']:
        nav_distance = navInstruction.maneuverDistance;
      nav_turn = True if nav_type == 'turn' and nav_modifier in ['left', 'right'] else False
      direction = 1 if nav_modifier in ['slight left', 'left'] else 2 if nav_modifier in ['slight right', 'right'] else 0
      if nav_speed_down or nav_turn:
        speed_data.update(self.turn_dist, self.turn_speed, nav_distance, v_cruise_kph)
      elif direction > 0:
        speed_data.update(self.laneChange_dist, self.laneChange_speed, nav_distance, v_cruise_kph)
      speed_data.set(direction, nav_turn)
    else:
      speed_data.reset()
      return False
    return True

  def _update_tbt(self, CS, controls, v_cruise_kph):
    roadLimitSpeed = controls.sm['roadLimitSpeed']
    if roadLimitSpeed.xDistToTurn > 0 and roadLimitSpeed.xTurnInfo > 0:
      speed_data = self.speed_data[SpeedType.TBT]
      nav_distance = speed_data.remain_dist
      if controls.sm.updated['roadLimitSpeed']:
        nav_distance = roadLimitSpeed.xDistToTurn
      nav_type = roadLimitSpeed.xTurnInfo
      nav_turn = True if nav_type in [1,2] else False
      nav_speed_down = True if nav_turn or nav_type in [5, 6] else False
      direction = 1 if nav_type in [1,3] else 2 if nav_type in [2,4,43] else 0
      if nav_speed_down or nav_turn:
        speed_data.update(self.turn_dist, self.turn_speed, nav_distance, v_cruise_kph)
      elif direction > 0:
        speed_data.update(self.laneChange_dist, self.laneChange_speed, nav_distance, v_cruise_kph)
      speed_data.set(direction, nav_turn)
    else:
      speed_data.reset()
      return False
    return True

  def _update_tbt_next(self, CS, controls, v_cruise_kph):
    roadLimitSpeed = controls.sm['roadLimitSpeed']
    if roadLimitSpeed.xDistToTurn > 0 and roadLimitSpeed.xTurnInfo > 0:
      if roadLimitSpeed.xDistToTurnNext > 0 and roadLimitSpeed.xTurnInfoNext > 0:
        speed_data = self.speed_data[SpeedType.TBT_NEXT]
        nav_distance = speed_data.remain_dist
        if controls.sm.updated['roadLimitSpeed']:
          nav_distance = roadLimitSpeed.xDistToTurn + roadLimitSpeed.xDistToTurnNext

        nav_type = roadLimitSpeed.xTurnInfoNext
        nav_turn = True if nav_type in [1,2] else False
        nav_speed_down = True if nav_turn or nav_type in [5, 6] else False
        direction = 1 if nav_type in [1,3] else 2 if nav_type in [2,4,43] else 0
        if nav_speed_down or nav_turn:
          speed_data.update(self.turn_dist, self.turn_speed, nav_distance, v_cruise_kph)
        elif direction > 0:
          speed_data.update(self.laneChange_dist, self.laneChange_speed, nav_distance, v_cruise_kph)
        speed_data.set(direction, nav_turn)
    else:
      speed_data.reset()
      return False
    return True

  def update(self, CS, controls, v_cruise_kph):
    self.params.update()
    v_ego = CS.vEgoCluster
    # remain_distance
    delta_dist = controls.distance_traveled - self.distance_traveled
    self.distance_traveled = controls.distance_traveled
    for speed_data in self.speed_data.values():
      speed_data.update_distance(delta_dist)

    md = controls.sm['modelV2']
    distanceToRoadEdgeLeft = md.meta.distanceToRoadEdgeLeft
    distanceToRoadEdgeRight = md.meta.distanceToRoadEdgeRight
    roadLimitSpeed = controls.sm['roadLimitSpeed']
    roadcate = roadLimitSpeed.roadcate
    xNextRoadWidth = roadLimitSpeed.xNextRoadWidth
    if roadcate > 7 and (distanceToRoadEdgeLeft + distanceToRoadEdgeRight) > 5.5:
      roadcate = 5
    self.turn_dist = interp(xNextRoadWidth, [5, 10], [43, 60])
    self.turn_speed = interp(xNextRoadWidth, [5, 10], [self.params.autoTurnControlSpeedTurn, self.params.autoTurnControlSpeedTurn*1.5])
    self.laneChange_dist = interp(roadcate, [0, 1, 2, 7], [300, 280, 200, 160])
    self.laneChange_speed = interp(roadcate, [0, 1, 2, 7],
                                   [self.params.autoTurnControlSpeedLaneChange*1.5, self.params.autoTurnControlSpeedLaneChange*1.5,
                                    self.params.autoTurnControlSpeedLaneChange*1.2, self.params.autoTurnControlSpeedLaneChange])

    if not self._update_cam(CS, controls, v_cruise_kph):
      self._update_car(CS, controls, v_cruise_kph)

    if self.params.autoTurnControl > 0:
      if self._update_tbt(CS, controls, v_cruise_kph):
        self._update_tbt_next(CS, controls, v_cruise_kph)
      else:
        self._update_navi(CS, controls, v_cruise_kph)

    apply_speed, source, direction, turn, dist = self.get_apply_speed()
    # auto lanechange or turn
    self.noo_helper(v_ego, source, direction, turn, dist, apply_speed)

    return apply_speed

  def noo_helper(self, v_ego, source, direction, turn, dist, speed):
    self.rightBlinkerExtCount = max(self.rightBlinkerExtCount - 1, 0)
    self.leftBlinkerExtCount = max(self.leftBlinkerExtCount - 1, 0)
    if self.rightBlinkerExtCount + self.leftBlinkerExtCount <= 0:
      self.blinkerExtMode = 0

    if dist > 0:
      start_dist = interp(v_ego*3.6, [60, 120], [300, 1000])
      if self.nooHelperActivated == 0 and dist < start_dist:
        self.nooHelperActivated = 1
      if self.nooHelperActivated:
        self.nooHelperActivated = max(1, self.nooHelperActivated)
        self.nooHelperActivateCount = max(0, self.nooHelperActivateCount + 1)
        self._add_log("Auto Speed Down to {:.0f}km/h. {:.0f}m left.".format(speed, dist))
    else:
      self.nooHelperActivated = 0
      self.nooHelperActivateCount = min(0, self.nooHelperActivateCount - 1)

    if self.params.autoTurnMapChange > 0:
      if self.nooHelperActivateCount == 10:
        Params().put_nonblocking("CarrotDisplay", "3")
      elif self.nooHelperActivateCount == -10:
        Params().put_nonblocking("CarrotDisplay", "2")

    ## lanechange, turn : 300m left
    nav_turn = False
    nav_direction = 0
    if 5 < dist < 300 and direction != 0:
      if nav_turn:
        if dist < self.turn_dist:
          # start Turn
          nav_direction = direction
        elif dist < self.laneChange_dist:
          nav_turn = False
          nav_direction = direction
        else:
          nav_turn = False
          nav_direction = 0
      elif dist < self.laneChange_dist:
        nav_direction = direction
      else:
        nav_direction = 0
    else:
      nav_turn = False
      nav_direction = 0        

    #self.debugTextNoo = "N<{}>{:.0f}[{}],T{}[{}],L{:.0f}/{:.0f},T{:.0f}/{:.0f}".format(
    #  self.nooHelperActivated,
    #  dist, direction, nav_direction, nav_turn,
    #  self.laneChange_dist, self.laneChange_speed, self.turn_dist, self.turn_speed)

    if self.params.autoTurnControl in [1,2] and self.nooHelperActivated == 1:
      blinkerExtState = self.rightBlinkerExtCount + self.rightBlinkerExtCount
      if nav_direction == 1 and nav_turn: # 왼쪽차선변경은 위험하니 턴인경우만 하자, 하지만 지금은 안함.
        if CS.rightBlinker or (CS.steeringPressed and CS.steeringTorque < 0):
          self.nooHelperActivated = 2
        else:
          self.leftBlinkerExtCount = 10
          self.blinkerExtMode = 20000 if nav_turn else 10000
      elif nav_direction == 2:
        if CS.leftBlinker or (CS.steeringPressed and CS.steeringTorque > 0):
          self.nooHelperActivated = 2
        else:
          self.rightBlinkerExtCount = 10
          self.blinkerExtMode = 20000 if nav_turn else 10000
      if self.nooHelperActivated == 2:
        self._add_log("Automatic lanechange canceled(blinker or steering torque)")
        self.rightBlinkerExtCount = self.leftBlinkerExtCount = self.blinkerExtMode = 0

      if self.blinkerExtMode >= 10000:
        self._add_log("Automatic {} Started. {:.0f}m left".format("Turning" if self.blinkerExtMode >= 20000 else "Lanechanging", dist ))

      #TODO: make event..
      #if blinkerExtState <= 0 and self.rightBlinkerExtCount + self.rightBlinkerExtCount > 0:
      #  self._make_event(controls, EventName.audioTurn if nav_turn else EventName.audioLaneChange)


