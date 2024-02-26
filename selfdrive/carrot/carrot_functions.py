import math
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, DT_CTRL
from enum import Enum
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.modeld.constants import ModelConstants
import numpy as np
from common.filter_simple import StreamingMovingAverage
from abc import abstractmethod, ABC
from openpilot.selfdrive.frogpilot.functions.map_turn_speed_controller import MapTurnSpeedController
from openpilot.selfdrive.navd.helpers import Coordinate
import json

MIN_TARGET_V = 5    # m/s

def decelerate_for_speed_camera(safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):
  if left_dist <= safe_dist:
    return safe_speed
  temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
  dV = (-safe_speed + math.sqrt(temp)) * decel_rate
  apply_speed = min(250 , safe_speed + dV)
  min_speed = prev_apply_speed - (decel_rate * 1.8) * 2 * DT_MDL
  apply_speed = max(apply_speed, min_speed)
  return apply_speed

class CarrotBase(ABC):
  def __init__(self):
    self._log_timer = 0
    self._log_timeout = 300
    self.log = ""
    self.update_params()

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.log = ""
    else:
      self.log = log
      self._log_timer = 300

  def update(self, sm, v_cruise_kph):
    self._add_log("")
    return self._update(sm, v_cruise_kph)

  @abstractmethod
  def update_params(self):
    pass

  @abstractmethod
  def _update(self, sm, v_cruise_kph):
    return v_cruise_kph

MIN_CURVE_SPEED = 10. * CV.KPH_TO_MS
TARGET_LAT_A = 1.9  # m/s^2

class CarrotVisionTurn(CarrotBase):
  def __init__(self, params):
    self.params = params
    super().__init__()
    self._log_timeout = 20
    self.curvatureFilter = StreamingMovingAverage(5)

  def update_params(self):
    self.autoCurveSpeedCtrlUse = int(self.params.get("AutoCurveSpeedCtrlUse"))
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor")*0.01
    self.autoCurveSpeedAggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness")*0.01

  def _update(self, sm, v_cruise_kph):
    CS = sm['carState']
    ## turn speed
    self.turnSpeed, self.curveSpeed = self.turn_speed(CS, sm)
    if self.autoCurveSpeedCtrlUse > 0:
      v_cruise_kph = min(v_cruise_kph, self.turnSpeed)
    return v_cruise_kph

  def turn_speed(self, CS, sm):

    modelData = sm['modelV2']
    v_ego = CS.vEgo
    # Set the curve sensitivity
    orientation_rate = np.array(modelData.orientationRate.z) * self.autoCurveSpeedFactor
    velocity = np.array(modelData.velocity.x)

    # Get the maximum lat accel from the model
    max_index = np.argmax(np.abs(orientation_rate))
    curv_direction = np.sign(orientation_rate[max_index])
    max_pred_lat_acc = np.amax(np.abs(orientation_rate) * velocity)

    # Get the maximum curve based on the current velocity
    max_curve = max_pred_lat_acc / (v_ego**2)

    # Set the target lateral acceleration
    adjusted_target_lat_a = TARGET_LAT_A * self.autoCurveSpeedAggressiveness

    # Get the target velocity for the maximum curve
    turnSpeed = max((adjusted_target_lat_a / max_curve)**0.5  * 3.6, 15.0)
    return turnSpeed, turnSpeed * curv_direction


class CarrotMapTurnSpeed(CarrotBase):
  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory
    super().__init__()
    self.mtsc = MapTurnSpeedController()
    self.mtsc_target = 0  #MS
    self.mtsc_limit = 0
    self.road_curvature = 0
    self.limitSpeed = 0.0

  def update_params(self):
    self.enableOSM = self.params.get_int("EnableOSM")
    self.map_turn_speed_controller = self.params.get_bool("MTSCEnabled")
    self.mtsc_curvature_check = self.params.get_bool("MTSCCurvatureCheck")
    is_metric = True
    self.mtsc_limit = self.params.get_float("MTSCLimit") * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    if self.map_turn_speed_controller:
      self.params_memory.put_float_nonblocking("MapTargetLatA", 2 * (self.params.get_int("MTSCAggressiveness") / 100))

  def _update(self, sm, v_cruise_kph):
    CS = sm['carState']
    v_ego = CS.vEgoCluster
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    roadName = self.get_current_road_name()
    map_speed_limit = self.get_current_speed_limit()

    #map_speed_limit_next, map_speed_limit_dist_next = self.get_next_speed_limit_and_distance()

    #target_velocities = json.loads(self.params_memory.get("MapTargetVelocities"))
    #map_curvatures = json.loads(self.params_memory.get("MapCurvatures"))
    #print("vel={}, curv={}".format(target_velocities, map_curvatures))
    #log = "osm:[{}], speedLimit:{:.1f}, mapTargetVel:{:.1f},curvature:{:.4f}".format(roadName, map_speed_limit, target_velocities, map_curvatures)

    # Pfeiferj's Map Turn Speed Controller
    if self.map_turn_speed_controller and v_ego > MIN_TARGET_V:
      #mtsc_active = self.mtsc_target < v_cruise
      self.mtsc_target = self.mtsc.target_speed(v_ego, CS.aEgo)

      # MTSC failsafes
      #if self.mtsc_curvature_check and self.road_curvature < 1.0 and not mtsc_active:
      #  self.mtsc_target = v_cruise
      #if v_ego - self.mtsc_limit >= self.mtsc_target:
      #  self.mtsc_target = v_cruise
      #log = "MTSC speed = {:.1f}kmh".format(self.mtsc_target * 3.6)
      #self._add_log(log)
      v_cruise_kph = min(v_cruise_kph, max(self.mtsc_target * 3.6, self.mtsc_limit))

    log = "osm:{:.1f}:[{}], speedLimit:{:.1f}".format(
      self.mtsc_target*3.6, roadName, map_speed_limit)#, map_speed_limit_next, map_speed_limit_dist_next)
    self._add_log(log)

    if False: #controls.sm.updated['liveMapData']:
      osm = sm['liveMapData']
      log = "speedLimit={}/{},{}/{}/{:.0f},turn={}/{:.1f}/{}/{:.1f}".format(
        osm.speedLimitValid, osm.speedLimit,
        osm.speedLimitAheadValid, osm.speedLimitAhead, osm.speedLimitAheadDistance,
        osm.turnSpeedLimitValid, osm.turnSpeedLimit, osm.turnSpeedLimitSign, osm.turnSpeedLimitEndDistance)
      self._add_log(log)

      if self.enableOSM > 0:
        if osm.speedLimitAheadValid and osm.speedLimitAhead > 0 and osm.speedLimitAheadDistance < 50:
          self.limitSpeed = osm.speedLimitAhead
        elif osm.speedLimitValid and osm.speedLimit > 0:
          self.limitSpeed = osm.speedLimit

        if self.enableOSM > 1:
          if self.limitSpeed > 0:
            self.cruiseSpeedMax = self.limitSpeed * self.autoNaviSpeedSafetyFactor

        if self.enableOSM > 2:
          if osm.turnSpeedLimitValid and osm.turnSpeedLimit > 0:
            v_cruise_kph = min(v_cruise_kph, osm.turnSpeedLimit)
      
    return v_cruise_kph

  def get_current_speed_limit(self):
    speed_limit = self.params_memory.get("MapSpeedLimit", encoding='utf8')
    return float(speed_limit) if speed_limit else 0.0

  def get_current_road_name(self):
    current_road_name = self.params_memory.get("RoadName", encoding='utf8')
    return current_road_name if current_road_name else ""

  def getLastGpsCoord(self):
    try:
      position = json.loads(self.params_memory.get("LastGPSPosition"))
      lat = position["latitude"]
      lon = position["longitude"]
    except: return Coordinate(0.0, 0.0)
    return Coordinate(lat, lon)

  def get_next_speed_limit_and_distance(self):
    next_speed_limit_section_str = self.params_memory.get("NextMapSpeedLimit", encoding='utf8')
    next_speed_limit_section = json.loads(next_speed_limit_section_str) if next_speed_limit_section_str else {}
    next_speed_limit = next_speed_limit_section.get('speedlimit', 0.0)
    next_speed_limit_latitude = next_speed_limit_section.get('latitude')
    next_speed_limit_longitude = next_speed_limit_section.get('longitude')
    next_speed_limit_distance = 0

    if next_speed_limit_latitude and next_speed_limit_longitude:
      self.last_gps = self.getLastGpsCoord()

      next_speed_limit_coordinates = Coordinate(next_speed_limit_latitude, next_speed_limit_longitude)
      next_speed_limit_distance = (self.last_gps or Coordinate(0, 0)).distance_to(next_speed_limit_coordinates)

    return next_speed_limit, next_speed_limit_distance


class CarrotNaviHelper(CarrotBase):
  def __init__(self, params):
    self.params = params
    super().__init__()
    self.distance_traveled = 0.0
    self.nav_distance = 0  # for navInstruction
    self.naviDistance = 0
    self.naviSpeed = 0
    self.nooHelperActivated = 0
    self.nooHelperActivateCount = 0
    self.debugTextNoo = ""
    self.blinkerExtMode = 0 # 0: Normal, 10000: voice
    self.rightBlinkerExtCount = 0
    self.leftBlinkerExtCount = 0

  def update_params(self):
    self.autoTurnControlSpeedLaneChange = self.params.get_int("AutoTurnControlSpeedLaneChange")
    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01

  def _update(self, sm, v_cruise_kph):
    self.rightBlinkerExtCount = max(self.rightBlinkerExtCount - 1, 0)
    self.leftBlinkerExtCount = max(self.leftBlinkerExtCount - 1, 0)
    if self.rightBlinkerExtCount + self.leftBlinkerExtCount <= 0:
      self.blinkerExtMode = 0

    CS = sm['carState']
    v_ego = CS.vEgoCluster
    delta_dist = CS.vEgo * DT_MDL
    self.distance_traveled += delta_dist
    self.nav_distance = max(0, self.nav_distance - delta_dist)

    if self.autoTurnControl > 0:
      navInstruction = sm['navInstruction']      
      roadLimitSpeed = sm['roadLimitSpeed']
      md = sm['modelV2']
      distanceToRoadEdgeLeft = md.meta.distanceToRoadEdgeLeft
      distanceToRoadEdgeRight = md.meta.distanceToRoadEdgeRight

      nav_type = navInstruction.maneuverType;
      nav_modifier = navInstruction.maneuverModifier;
      nav_turn = False
      nav_speedDown = False
      direction = 0 #1:left, 2:right
      if nav_type in ['turn', 'fork', 'off ramp'] and roadLimitSpeed.xDistToTurn <= 0 and roadLimitSpeed.xTurnInfo < 0:
        if controls.sm.updated['navInstruction']:
          self.nav_distance = navInstruction.maneuverDistance;
        nav_turn = True if nav_type == 'turn' and nav_modifier in ['left', 'right'] else False
        direction = 1 if nav_modifier in ['slight left', 'left'] else 2 if nav_modifier in ['slight right', 'right'] else 0
      else:
        if sm.updated['roadLimitSpeed']:
          self.nav_distance = roadLimitSpeed.xDistToTurn
        nav_type = roadLimitSpeed.xTurnInfo
        nav_turn = True if nav_type in [1,2] else False
        nav_speedDown = True if nav_turn or nav_type in [5, 6] else False
        direction = 1 if nav_type in [1,3] else 2 if nav_type in [2,4,43] else 0

      roadcate = roadLimitSpeed.roadcate
      xNextRoadWidth = roadLimitSpeed.xNextRoadWidth
      if roadcate > 7 and (distanceToRoadEdgeLeft + distanceToRoadEdgeRight) > 5.5:
        roadcate = 5
      turn_dist = interp(xNextRoadWidth, [5, 10], [43, 60])
      turn_speed = self.autoTurnControlSpeedTurn #interp(xNextRoadWidth, [5, 10], [self.autoTurnControlSpeedTurn, self.autoTurnControlSpeedTurn*1.5])
      laneChange_dist = interp(roadcate, [0, 1, 2, 7], [300, 280, 200, 160])
      laneChange_speed = interp(roadcate, [0, 1, 2, 7], [self.autoTurnControlSpeedLaneChange*1.5, self.autoTurnControlSpeedLaneChange*1.5, self.autoTurnControlSpeedLaneChange*1.2, self.autoTurnControlSpeedLaneChange])

      self.naviDistance = 0
      self.naviSpeed = 0
      if nav_turn or nav_speedDown or direction != 0:
        if self.autoTurnControl in [2,3]:
          self.naviDistance = self.nav_distance
          self.naviSpeed = turn_speed if nav_turn or (nav_speedDown and nav_type in [5]) else laneChange_speed

      start_dist = interp(v_ego*3.6, [60, 110], [300, 1000])
      if 0 < self.naviDistance < start_dist:
        if self.nooHelperActivated == 0:
          self.nooHelperActivated = 1
        self.nooHelperActivated = max(1, self.nooHelperActivated)
        self.nooHelperActivateCount = max(0, self.nooHelperActivateCount + 1)
        self._add_log("Auto Speed Down to {:.0f}km/h. {:.0f}m left.".format(self.naviSpeed, self.naviDistance))
      else:
        self.nooHelperActivated = 0
        self.nooHelperActivateCount = min(0, self.nooHelperActivateCount - 1)

      if self.autoTurnMapChange > 0:
        if self.nooHelperActivateCount == 10:
          self.params.put_nonblocking("CarrotDisplay", "3")
        elif self.nooHelperActivateCount == - int(5/DT_CTRL):
          self.params.put_nonblocking("CarrotDisplay", "2")

      ## lanechange, turn : 300m left
      if 5 < self.nav_distance < 300 and direction != 0:
        if nav_turn:
          if self.nav_distance < turn_dist:
            # start Turn
            nav_direction = direction
          elif self.nav_distance < laneChange_dist:
            nav_turn = False
            nav_direction = direction
          else:
            nav_turn = False
            nav_direction = 0
        elif self.nav_distance < laneChange_dist:
          nav_direction = direction
        else:
          nav_direction = 0
      else:
        nav_turn = False
        nav_direction = 0        

      self.debugTextNoo = "N<{}>{:.0f}[{}],T{}[{}],L{:.0f}/{:.0f},T{:.0f}/{:.0f}".format(
        self.nooHelperActivated,
        self.nav_distance, direction, nav_direction, nav_turn,
        laneChange_dist, laneChange_speed, turn_dist, turn_speed)

      if self.autoTurnControl > 0 and self.nooHelperActivated == 1:
        blinkerExtState = self.rightBlinkerExtCount + self.rightBlinkerExtCount
        if nav_direction == 1: #여기서는 풀고... desire에서 막자.  and nav_turn: # 왼쪽차선변경은 위험하니 턴인경우만 하자, 하지만 지금은 안함.
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
          self._add_log("Automatic {} Started. {:.0f}m left".format("Turning" if self.blinkerExtMode >= 20000 else "Lanechanging", self.naviDistance ))

        #if blinkerExtState <= 0 and self.leftBlinkerExtCount + self.rightBlinkerExtCount > 0 and v_ego > 0.5:
        #  self._make_event(controls, EventName.audioTurn if nav_turn else EventName.audioLaneChange)

        apTbtDistance = self.naviDistance
        apTbtSpeed = self.naviSpeed
        if apTbtSpeed > 0 and apTbtDistance > 0:
          safeTbtDist = self.autoTurnControlTurnEnd * v_ego
          applyTbtSpeed = decelerate_for_speed_camera(apTbtSpeed/3.6, safeTbtDist, v_cruise_kph/3.6, self.autoNaviSpeedDecelRate, apTbtDistance) * 3.6
          if applyTbtSpeed < v_cruise_kph:
            v_cruise_kph = applyTbtSpeed

    else:
      self.naviDistance = 0
      self.naviSpeed = 0

    return v_cruise_kph

class CarrotNaviSpeedManager(CarrotBase):
  def __init__(self, params):
    self.params = params
    super().__init__()

    self.activeAPM = 0
    self.roadSpeed = 0

  def update_params(self):
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = self.params.get_int("AutoNaviSpeedCtrl")

  def _update(self, sm, v_cruise_kph):
    v_ego = sm['carState'].vEgoCluster
    CS = sm['carState']
    msg = self.roadLimitSpeed = sm['roadLimitSpeed']
    self.activeAPM = msg.active

    self.roadSpeed = clip(0, msg.roadLimitSpeed, 150.0)
    camType = int(msg.camType)
    xSignType = msg.xSignType

    isSpeedBump = False
    isSectionLimit = False
    safeSpeed = 0
    leftDist = 0
    speedLimitType = 0
    safeDist = 0
  
    
    if camType == 22 or xSignType == 22:
      safeSpeed = self.autoNaviSpeedBumpSpeed
      isSpeedBump = True

    if msg.xSpdLimit > 0 and msg.xSpdDist > 0:
      safeSpeed = msg.xSpdLimit if safeSpeed <= 0 else safeSpeed
      leftDist = msg.xSpdDist
      isSectionLimit = True if xSignType==165 or leftDist > 3000 or camType == 4 else False
      isSectionLimit = False if leftDist < 50 else isSectionLimit
      speedLimitType = 2 if not isSectionLimit else 3
    elif msg.camLimitSpeed > 0 and msg.camLimitSpeedLeftDist>0:
      safeSpeed = msg.camLimitSpeed
      leftDist = msg.camLimitSpeedLeftDist
      isSectionLimit = True if leftDist > 3000 or camType == 4 else False
      isSectionLimit = False if leftDist < 50 else isSectionLimit
      speedLimitType = 2 if not isSectionLimit else 3
    elif CS.speedLimit > 0 and CS.speedLimitDistance > 0 and self.autoNaviSpeedCtrl >= 1:
      safeSpeed = CS.speedLimit
      leftDist = CS.speedLimitDistance
      speedLimitType = 2 if leftDist > 1 else 3

    if isSpeedBump:
      speedLimitType = 1 
      safeDist = self.autoNaviSpeedBumpTime * v_ego
    elif safeSpeed>0 and leftDist>0:
      safeDist = self.autoNaviSpeedCtrlEnd * v_ego

    safeSpeed *= self.autoNaviSpeedSafetyFactor

    log = ""
    if leftDist > 0 and safeSpeed > 0 and safeDist > 0:
      applySpeed = decelerate_for_speed_camera(safeSpeed/3.6, safeDist, v_cruise_kph * CV.KPH_TO_MS, self.autoNaviSpeedDecelRate, leftDist) * CV.MS_TO_KPH
      if isSectionLimit and applySpeed > safeSpeed:
        applySpeed = safeSpeed
    else:
      applySpeed = 255


    if applySpeed < 200:
      log = "{},{:.1f}<{:.1f}/{:.1f},{:.1f} B{} A{:.1f}/{:.1f} N{:.1f}/{:.1f} C{:.1f}/{:.1f}".format(
                    msg.roadcate, applySpeed, safeSpeed, leftDist, safeDist,
                    1 if isSpeedBump else 0, 
                    msg.xSpdLimit, msg.xSpdDist,
                    msg.camLimitSpeed, msg.camLimitSpeedLeftDist,
                    CS.speedLimit, CS.speedLimitDistance)
      self._add_log(log)
    #self.debugText2 = log
    if speedLimitType == 2:
      self.activeAPM += 1000
    return min(v_cruise_kph, applySpeed) #, roadSpeed, leftDist, speedLimitType

class CarrotPlannerHelper:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.params_count = 0
    self.frame = 0

    self.vision_turn = CarrotVisionTurn(self.params)
    self.turnSpeed = 300
    self.curveSpeed = 300

    self.map_turn = CarrotMapTurnSpeed(self.params, self.params_memory)
    self.navi_helper = CarrotNaviHelper(self.params)
    self.navi_speed_manager = CarrotNaviSpeedManager(self.params)
    self.activeAPM = 0

    self.log = ""

  def _params_update(self):
    self.frame += 1
    self.params_count += 1
    if self.params_count == 10:
      self.vision_turn.update_params()
    elif self.params_count == 20:
      self.map_turn.update_params()
    elif self.params_count == 30:
      self.navi_helper.update_params()
    elif self.params_count == 40:
      self.navi_speed_manager.update_params()
    elif self.params_count >= 100:
      self.params_count = 0

  def update(self, sm, v_cruise_kph):
    vision_turn_kph = self.vision_turn.update(sm, v_cruise_kph)
    self.turnSpeed = self.vision_turn.turnSpeed
    self.curveSpeed = self.vision_turn.curveSpeed

    map_turn_kph = self.map_turn.update(sm, v_cruise_kph)
    self.limitSpeed = self.map_turn.limitSpeed
    #self.log = self.map_turn.log

    navi_helper_kph = self.navi_helper.update(sm, v_cruise_kph)
    self.leftBlinkerExt = self.navi_helper.leftBlinkerExtCount + self.navi_helper.blinkerExtMode
    self.rightBlinkerExt = self.navi_helper.rightBlinkerExtCount + self.navi_helper.blinkerExtMode

    navi_speed_manager_kph = self.navi_speed_manager.update(sm, v_cruise_kph)
    self.activeAPM = self.navi_speed_manager.activeAPM

    self.log = self.vision_turn.log
    if len(self.log):
      self.log += "|"
    self.log += self.map_turn.log
    if len(self.log):
      self.log += "|"
    self.log += self.navi_helper.log
    if len(self.log):
      self.log += "|"
    self.log += self.navi_speed_manager.log

    return min(vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph)














