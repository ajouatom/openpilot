import math
import numpy as np
import json

from cereal import car, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, DT_CTRL
from enum import Enum
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.modeld.constants import ModelConstants
from abc import abstractmethod, ABC
from openpilot.selfdrive.frogpilot.functions.map_turn_speed_controller import MapTurnSpeedController
from openpilot.selfdrive.navd.helpers import Coordinate
from openpilot.selfdrive.controls.neokii.navi_controller import SpeedLimiter
EventName = car.CarEvent.EventName

MIN_TARGET_V = 5    # m/s

def decelerate_for_speed_camera(safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):
  if left_dist <= safe_dist:
    return safe_speed
  temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
  dV = (-safe_speed + math.sqrt(temp)) * decel_rate
  apply_speed = min(250 , safe_speed + dV)
  min_speed = 0 # prev_apply_speed - (decel_rate * 1.8) * 2 * DT_MDL
  apply_speed = max(apply_speed, min_speed)
  return apply_speed

class CarrotBase(ABC):
  def __init__(self):
    self._log_timer = 0
    self._log_timeout = int(3/DT_MDL) # 3 seconds
    self.log = ""
    self.event = -1
    self.update_params()

  def _add_log(self, log, event=-1):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.log = ""
        self.event = -1
    else:
      self.log = log
      self.event = event
      self._log_timer = self._log_timeout

  def update(self, sm, v_cruise_kph, v_cruise_kph_prev):
    self._add_log("")
    return self._update(sm, v_cruise_kph, v_cruise_kph_prev)

  @abstractmethod
  def update_params(self):
    pass

  @abstractmethod
  def _update(self, sm, v_cruise_kph, v_cruise_kph_prev):
    return v_cruise_kph

MIN_CURVE_SPEED = 10. * CV.KPH_TO_MS
TARGET_LAT_A = 1.9  # m/s^2

class CarrotVisionTurn(CarrotBase):
  def __init__(self, params):
    self.params = params
    super().__init__()
    self._log_timeout = int(2/DT_MDL) # 2 seconds

  def update_params(self):
    self.autoCurveSpeedLowerLimit = int(self.params.get("AutoCurveSpeedLowerLimit"))
    self.autoCurveSpeedFactor = self.params.get_int("AutoCurveSpeedFactor")*0.01
    self.autoCurveSpeedAggressiveness = self.params.get_int("AutoCurveSpeedAggressiveness")*0.01

  def _update(self, sm, v_cruise_kph, v_cruise_kph_prev):
    CS = sm['carState']
    ## turn speed
    self.turnSpeed, self.curveSpeed = self.turn_speed(CS, sm)
    if self.turnSpeed < CS.vEgo * CV.MS_TO_KPH:
      self._add_log("Vision turn speed down {:.1f}kmh".format(self.turnSpeed)) #, EventName.speedDown)
    v_cruise_kph = min(v_cruise_kph, self.turnSpeed)
    return v_cruise_kph

  def turn_speed(self, CS, sm):

    modelData = sm['modelV2']
    v_ego = max(CS.vEgo, 0.1)
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
    turnSpeed = max(abs(adjusted_target_lat_a / max_curve)**0.5  * 3.6, self.autoCurveSpeedLowerLimit)
    return turnSpeed, turnSpeed * curv_direction


class CarrotMapTurnSpeed(CarrotBase):
  def __init__(self, params, params_memory):
    self.params = params
    self.params_memory = params_memory
    self.map_turn_aggressiveness = 100
    self.mtsc = MapTurnSpeedController()
    self.mtsc_target = 0  #MS
    self.road_curvature = 0
    self.limitSpeed = 0.0
    self.mtsc_canceled = 0
    super().__init__()

  def update_params(self):
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.MSLCEnabled = self.params.get_int("MSLCEnabled")
    self.map_turn_speed_controller = self.params.get_bool("MTSCEnabled")
    self.mtsc_curvature_check = self.params.get_bool("MTSCCurvatureCheck")
    map_turn_aggressiveness = self.params.get_int("MTSCAggressiveness")
    if self.map_turn_speed_controller and map_turn_aggressiveness != self.map_turn_aggressiveness:
      self.map_turn_aggressiveness = map_turn_aggressiveness
      self.params_memory.put_float_nonblocking("MapTargetLatA", 2 * (self.map_turn_aggressiveness / 100))

  def _update(self, sm, v_cruise_kph, v_cruise_kph_prev):
    CS = sm['carState']
    v_ego = CS.vEgoCluster
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_prev = v_cruise_kph_prev * CV.KPH_TO_MS

    roadName = self.get_current_road_name()
    map_speed_limit = self.get_current_speed_limit()

    map_speed_limit_next, map_speed_limit_dist_next = self.get_next_speed_limit_and_distance()

    #target_velocities = json.loads(self.params_memory.get("MapTargetVelocities"))
    #map_curvatures = json.loads(self.params_memory.get("MapCurvatures"))
    #print("vel={}, curv={}".format(target_velocities, map_curvatures))
    #log = "osm:[{}], speedLimit:{:.1f}, mapTargetVel:{:.1f},curvature:{:.4f}".format(roadName, map_speed_limit, target_velocities, map_curvatures)

    # Pfeiferj's Map Turn Speed Controller
    if self.map_turn_speed_controller and v_ego > MIN_TARGET_V:
      mtsc_active = self.mtsc_target < v_cruise_prev
      #self.mtsc_target = self.mtsc.target_speed(v_ego, CS.aEgo)
      self.mtsc_target = self.mtsc.target_speed(100.0, CS.aEgo)

      self.road_curvature = self.calculate_road_curvature(sm['modelV2'], v_ego)

      if CS.gasPressed:
        self.mtsc_canceled = 1

      if self.mtsc_target > v_cruise:
        self.mtsc_canceled = 0

      # MTSC failsafes
      if self.mtsc_curvature_check and self.road_curvature < 1.0 and not mtsc_active:
        self.mtsc_target = v_cruise
        #log = "map:{:.1f}:[{}] r_c = {:.4f}".format(self.mtsc_target*3.6, roadName, self.road_curvature)
        #self._add_log(log)
      else:
        #log = "MTSC speed = {:.1f}kmh".format(self.mtsc_target * 3.6)
        #self._add_log(log)
        #log = "map:{:.1f}:[{}], mapsl:{:.1f}, rc={:.3f}, next={:.0f}/{:.0f}".format(
        #  self.mtsc_target*3.6, roadName, map_speed_limit, self.road_curvature, map_speed_limit_next*3.6, map_speed_limit_dist_next)
        #self._add_log(log) #, EventName.speedDown if v_cruise_kph_apply < v_cruise_kph else -1)
        if self.mtsc_target > 0 and self.mtsc_canceled == 0:
          v_cruise_kph_apply = min(v_cruise_kph, max(self.mtsc_target * 3.6, 15.0))
          log = "map:{:.1f}:[{}], mapsl:{:.1f}".format(
            self.mtsc_target*3.6, roadName, map_speed_limit)#, map_speed_limit_next, map_speed_limit_dist_next)
          self._add_log(log) # 소리끔..시골길 시끄러워... , EventName.speedDown if v_cruise_kph_apply < v_cruise_kph else -1)
          v_cruise_kph = v_cruise_kph_apply

      if self.MSLCEnabled > 0:
        if map_speed_limit > 0:
          v_cruise_kph_apply = min(v_cruise_kph, map_speed_limit * 3.6 * self.autoNaviSpeedSafetyFactor)
          log = "mapsl:{:.1f}".format(map_speed_limit*3.6)#, map_speed_limit_next, map_speed_limit_dist_next)
          self._add_log(log, EventName.speedDown if v_cruise_kph_apply < v_cruise_kph else -1)
          v_cruise_kph = v_cruise_kph_apply

    return v_cruise_kph

  def calculate_road_curvature(self, modelData, v_ego):
    predicted_velocities = np.array(modelData.velocity.x)
    curvature_ratios = np.abs(np.array(modelData.acceleration.y)) / (predicted_velocities**2)
    return np.amax(curvature_ratios * (v_ego**2))

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
    self.nav_turn = False
    self.left_sec = 11

  def update_params(self):
    self.autoTurnControlSpeedLaneChange = self.params.get_int("AutoTurnControlSpeedLaneChange")
    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01

  def _update(self, sm, v_cruise_kph, v_cruise_kph_prev):
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
        if sm.updated['navInstruction']:
          self.nav_distance = navInstruction.maneuverDistance;
        #nav_turn = True if nav_type == 'turn' and nav_modifier in ['left', 'right', 'sharp left', 'sharp right'] else False
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
      laneChange_dist = interp(roadcate, [0, 1, 2, 7], [350, 300, 200, 160])
      laneChange_speed = interp(roadcate, [0, 1, 2, 7], [self.autoTurnControlSpeedLaneChange*2.0, self.autoTurnControlSpeedLaneChange*1.5, self.autoTurnControlSpeedLaneChange*1.2, self.autoTurnControlSpeedLaneChange])

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
        self.nav_turn = False
        self.nooHelperActivated = 0
        self.nooHelperActivateCount = min(0, self.nooHelperActivateCount - 1)

      if self.autoTurnMapChange > 1:
        if self.nooHelperActivateCount == 10:
          self.params.put_nonblocking("CarrotDisplay", "3")
        elif self.nooHelperActivateCount == - int(5/DT_CTRL):
          self.params.put_nonblocking("CarrotDisplay", "2")

      ## lanechange, turn : 300m left
      if 5 < self.nav_distance < 350 and direction != 0:
        if nav_turn:
          if self.nav_distance < turn_dist or self.nav_turn:
            # start Turn
            nav_direction = direction
          elif self.nav_distance < laneChange_dist:
            nav_direction = direction
            nav_turn = False
          else:
            nav_direction = 0
            nav_turn = False
        elif self.nav_distance < laneChange_dist:
          nav_direction = direction
        else:
          nav_direction = 0
        self.nav_turn = nav_turn

        road_width = interp(xNextRoadWidth, [5, 10], [10, 20])
        left_sec = int(max(self.nav_distance - v_ego - road_width, 1) / max(1, v_ego))
        if left_sec < self.left_sec:
          self.params.put_int_nonblocking("CarrotCountDownSec", left_sec)
          self.left_sec = left_sec
      else:
        self.nav_turn = False
        nav_direction = 0
        self.left_sec = 11

      self.debugTextNoo = "N<{}>{:.0f}[{}],T{}[{}],L{:.0f}/{:.0f},T{:.0f}/{:.0f}".format(
        self.nooHelperActivated,
        self.nav_distance, direction, nav_direction, self.nav_turn,
        laneChange_dist, laneChange_speed, turn_dist, turn_speed)

      if self.autoTurnControl > 0 and self.nooHelperActivated == 1:
        blinkerExtState = self.rightBlinkerExtCount + self.rightBlinkerExtCount
        if nav_direction == 1: #여기서는 풀고... desire에서 막자.  and self.nav_turn: # 왼쪽차선변경은 위험하니 턴인경우만 하자, 하지만 지금은 안함.
          if CS.rightBlinker or (CS.steeringPressed and CS.steeringTorque < 0):
            self.nooHelperActivated = 2
          else:
            self.leftBlinkerExtCount = 10
            self.blinkerExtMode = 20000 if self.nav_turn else 10000
        elif nav_direction == 2:
          if CS.leftBlinker or (CS.steeringPressed and CS.steeringTorque > 0):
            self.nooHelperActivated = 2
          else:
            self.rightBlinkerExtCount = 10
            self.blinkerExtMode = 20000 if self.nav_turn else 10000
        if self.nooHelperActivated == 2:
          self._add_log("Automatic lanechange canceled(blinker or steering torque)", EventName.audioLaneChange)
          self.rightBlinkerExtCount = self.leftBlinkerExtCount = self.blinkerExtMode = 0

        if self.blinkerExtMode >= 10000:
          turning = self.blinkerExtMode >= 20000
          self._add_log("Automatic {} Started. {:.0f}m left".format("Turning" if turning else "Lanechanging", self.naviDistance), EventName.audioTurn if turning else EventName.audioLaneChange )

        #if blinkerExtState <= 0 and self.leftBlinkerExtCount + self.rightBlinkerExtCount > 0 and v_ego > 0.5:
        #  self._make_event(controls, EventName.audioTurn if self.nav_turn else EventName.audioLaneChange)

        apTbtDistance = self.naviDistance
        apTbtSpeed = self.naviSpeed
        if apTbtSpeed > 0 and apTbtDistance > 0:
          safeTbtDist = self.autoTurnControlTurnEnd * v_ego
          applyTbtSpeed = decelerate_for_speed_camera(apTbtSpeed/3.6, safeTbtDist, v_ego, self.autoNaviSpeedDecelRate, apTbtDistance) * 3.6
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
    self.left_sec = 11
    self.params.put_int_nonblocking("CarrotCountDownSec", self.left_sec)

    self.test_count = 0

  def update_params(self):
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = self.params.get_int("AutoNaviSpeedCtrl")

  def _update(self, sm, v_cruise_kph, v_cruise_kph_prev):

    if self.autoNaviSpeedCtrl == 0:
      return v_cruise_kph

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

    if camType == 22 or xSignType == 22:
      safeSpeed = self.autoNaviSpeedBumpSpeed
      isSpeedBump = True

    if isSpeedBump:
      speedLimitType = 1 
      safeDist = self.autoNaviSpeedBumpTime * v_ego
    elif safeSpeed>0 and leftDist>0:
      safeDist = self.autoNaviSpeedCtrlEnd * v_ego

    safeSpeed *= self.autoNaviSpeedSafetyFactor

    log = ""
    if leftDist > 0 and safeSpeed > 0 and safeDist > 0:
      applySpeed = decelerate_for_speed_camera(safeSpeed/3.6, safeDist, v_ego, self.autoNaviSpeedDecelRate, leftDist) * CV.MS_TO_KPH
      if isSectionLimit and applySpeed > safeSpeed:
        applySpeed = safeSpeed
      left_sec = int(max(leftDist - v_ego, 1) / max(1, v_ego))
      if left_sec < self.left_sec:
        self.params.put_int_nonblocking("CarrotCountDownSec", left_sec)
        self.left_sec = left_sec
    else:
      applySpeed = 255
      if False:
        self.test_count += 1
        if self.test_count > 20:
          self.test_count = 0
          self.left_sec -= 1
          if self.left_sec < 0:
            self.left_sec = 11
          self.params.put_int_nonblocking("CarrotCountDownSec", self.left_sec)
        pass
      else:
        if self.left_sec != 11:
          self.params.put_int_nonblocking("CarrotCountDownSec", 11)
        self.left_sec = 11


    if applySpeed < 200:
      log = "{},{:.1f}<{:.1f}/{:.1f},{:.1f} B{} A{:.1f}/{:.1f} N{:.1f}/{:.1f} C{:.1f}/{:.1f}".format(
                    msg.roadcate, applySpeed, safeSpeed, leftDist, safeDist,
                    1 if isSpeedBump else 0, 
                    msg.xSpdLimit, msg.xSpdDist,
                    msg.camLimitSpeed, msg.camLimitSpeedLeftDist,
                    CS.speedLimit, CS.speedLimitDistance)
      self._add_log(log, EventName.speedDown if applySpeed < v_ego * CV.MS_TO_KPH else -1)
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

    self.v_cruise_kph = 255

    self.is_metric = self.params.get_bool("IsMetric")
    
    self.vision_turn = CarrotVisionTurn(self.params)
    self.turnSpeed = 300
    self.curveSpeed = 300

    self.map_turn = CarrotMapTurnSpeed(self.params, self.params_memory)
    self.navi_helper = CarrotNaviHelper(self.params)
    self.navi_speed_manager = CarrotNaviSpeedManager(self.params)
    self.activeAPM = 0

    self.gas_override_speed = 0

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
    self._params_update()
    enabled = sm['controlsState'].enabled
    if enabled:
      self.v_cruise_kph = min(v_cruise_kph, self.v_cruise_kph)
    else:
      self.v_cruise_kph = 255
    
    vision_turn_kph = self.vision_turn.update(sm, v_cruise_kph, self.v_cruise_kph)
    self.turnSpeed = self.vision_turn.turnSpeed
    self.curveSpeed = self.vision_turn.curveSpeed
    self.event = self.vision_turn.event

    map_turn_kph = self.map_turn.update(sm, v_cruise_kph, self.v_cruise_kph)
    self.limitSpeed = self.map_turn.limitSpeed
    #self.log = self.map_turn.log
    if self.map_turn.event >= 0:
      self.event = self.map_turn.event

    navi_helper_kph = self.navi_helper.update(sm, v_cruise_kph, self.v_cruise_kph)
    self.leftBlinkerExt = self.navi_helper.leftBlinkerExtCount + self.navi_helper.blinkerExtMode
    self.rightBlinkerExt = self.navi_helper.rightBlinkerExtCount + self.navi_helper.blinkerExtMode

    if self.navi_helper.event >= 0:
      self.event = self.navi_helper.event    

    navi_speed_manager_kph = self.navi_speed_manager.update(sm, v_cruise_kph, self.v_cruise_kph)
    self.activeAPM = self.navi_speed_manager.activeAPM
    if self.navi_speed_manager.event >= 0:
      self.event = self.navi_speed_manager.event

    #apply_limit_speed, road_limit_speed, left_dist, first_started, cam_type, max_speed_log = \
    #  SpeedLimiter.instance().get_max_speed(v_cruise_kph, self.is_metric)
    #nda_log = "nda_type={} | ".format(cam_type) if apply_limit_speed > 0 else ""
    #nda_speed_kph = apply_limit_speed  if apply_limit_speed > 0 else 255

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

    #self.log = "{} VT{:.1f}, MT{:.1f}, NH{:.1f}, SP{:.1f}".format(self.map_turn.log, vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph)
    #self.v_cruise_kph = min(vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph)

    values_and_names = [
        (vision_turn_kph, "vTurn"),
        (map_turn_kph, "mTurn"),
        (navi_helper_kph, "noo"),
        (navi_speed_manager_kph, "navi"),
        #(nda_speed_kph, "nda"),
    ]

    # min 함수를 사용하여 가장 작은 값을 가진 튜플 찾기
    self.v_cruise_kph, self.source = min(values_and_names, key=lambda x: x[0])
    if self.v_cruise_kph == v_cruise_kph:
      self.source = "none"
      self.gas_override_speed = 0
    else:
      if sm['carState'].gasPressed and self.source not in ["navi", "nda"]:
        self.gas_override_speed = sm['carState'].vEgoCluster * 3.6
      elif sm['carState'].brakePressed:
        self.gas_override_speed = 0
      self.log = self.log + "v{:.0f}:m{:.0f},n{:.0f},s{:.0f},g{:.0f}".format(vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph, self.gas_override_speed)
      self.v_cruise_kph = max(self.v_cruise_kph, self.gas_override_speed)
    return self.v_cruise_kph
