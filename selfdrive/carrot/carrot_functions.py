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


class CarrotBase(ABC):
  def __init__(self):
    self._log_timer = 0
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

## 국가법령정보센터: 도로설계기준
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15., 1./10.]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 10]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30, 20]
MIN_CURVE_SPEED = 10. * CV.KPH_TO_MS
TARGET_LAT_A = 1.9  # m/s^2

class CarrotVisionTurn(CarrotBase):
  def __init__(self, params):
    self.params = params
    super().__init__()
    self.curvatureFilter = StreamingMovingAverage(5)
    self.curveSpeed = 0.0 # turnSpeed with sign
    self.turnSpeed_prev = 300

  def update_params(self):
    self.autoCurveSpeedCtrlUse = int(self.params.get("AutoCurveSpeedCtrlUse"))
    self.autoCurveSpeedFactor = float(int(self.params.get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
    self.autoCurveSpeedFactorIn = float(int(self.params.get("AutoCurveSpeedFactorIn", encoding="utf8")))*0.01

  def _update(self, sm, v_cruise_kph):
    CS = sm['carState']
    ## turn speed
    self.turnSpeed = self.apilot_curve(CS, sm)
    #self.turnSpeed = self.turn_speed(CS, sm)
    if self.autoCurveSpeedCtrlUse > 0:
      v_cruise_kph = min(v_cruise_kph, self.turnSpeed)
    return v_cruise_kph

  def turn_speed(self, CS, sm):

    modelData = sm['modelV2']
    v_ego = CS.vEgo
    # Set the curve sensitivity
    orientation_rate = np.array(np.abs(modelData.orientationRate.z)) * self.autoCurveSpeedFactor
    velocity = np.array(modelData.velocity.x)

    # Get the maximum lat accel from the model
    max_pred_lat_acc = np.amax(orientation_rate * velocity)

    # Get the maximum curve based on the current velocity
    max_curve = max_pred_lat_acc / (v_ego**2)

    # Set the target lateral acceleration
    self.turn_aggressiveness = 1.0
    adjusted_target_lat_a = TARGET_LAT_A * self.turn_aggressiveness

    # Get the target velocity for the maximum curve
    return max((adjusted_target_lat_a / max_curve)**0.5  * 3.6, 15.0)

  def apilot_curve(self, CS, sm):
    if len(sm['modelV2'].orientationRate.z) != 33:
      return 300
    #self.road_curvature = self.calculate_road_curvature(sm['modelV2'], CS.vEgo)

    # 회전속도를 선속도 나누면 : 곡률이 됨. [20]은 약 4초앞의 곡률을 보고 커브를 계산함.
    #curvature = abs(sm['modelV2'].orientationRate.z[20] / clip(CS.vEgo, 0.1, 100.0))
    orientationRates = np.array(sm['modelV2'].orientationRate.z, dtype=np.float32)
    # 계산된 결과로, oritetationRates를 나누어 조금더 curvature값이 커지도록 함.
    speed = min(self.turnSpeed_prev / 3.6, clip(CS.vEgo, 0.5, 100.0))    
    #curvature = np.max(np.abs(orientationRates[12:])) / speed  # 12: 약1.4초 미래의 curvature를 계산함.
    #curvature = np.max(np.abs(orientationRates[12:20])) / speed  # 12: 약1.4~3.5초 미래의 curvature를 계산함.
    curvature = np.max(orientationRates[12:28]) / speed  
    curvature = self.curvatureFilter.process(curvature) * self.autoCurveSpeedFactor
    turnSpeed = 300
    if abs(curvature) > 0.0001:
      turnSpeed = interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
      turnSpeed = clip(turnSpeed, MIN_CURVE_SPEED, 255)
    else:
      turnSpeed = 300

    self.curveSpeed = turnSpeed * np.sign(curvature)

    #print("curvature={:.5f}, speed = {:.1f},{:.1f}".format(curvature, turnSpeed, self.curveSpeed))
    self.turnSpeed_prev = turnSpeed
    speed_diff = max(0, CS.vEgo*3.6 - turnSpeed)
    turnSpeed = turnSpeed - speed_diff * self.autoCurveSpeedFactorIn
    #controls.debugText2 = 'CURVE={:5.1f},curvature={:5.4f},mode={:3.1f}'.format(self.turnSpeed_prev, curvature, self.drivingModeIndex)
    return turnSpeed

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

    target_velocities = json.loads(self.params_memory.get("MapTargetVelocities"))
    map_curvatures = json.loads(self.params_memory.get("MapCurvatures"))
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

    else:
      self.naviDistance = 0
      self.naviSpeed = 0

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
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")

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
      applySpeed = self.decelerate_for_speed_camera(safeSpeed/3.6, safeDist, v_cruise_kph * CV.KPH_TO_MS, self.autoNaviSpeedDecelRate, leftDist) * CV.MS_TO_KPH
      if isSectionLimit and applySpeed > safeSpeed:
        applySpeed = safeSpeed
    else:
      applySpeed = 255

    apTbtDistance = self.naviDistance
    apTbtSpeed = self.naviSpeed
    if apTbtSpeed > 0 and apTbtDistance > 0:
      safeTbtDist = self.autoTurnControlTurnEnd * v_ego
      applyTbtSpeed = self.decelerate_for_speed_camera(apTbtSpeed/3.6, safeTbtDist, v_cruise_kph/3.6, self.autoNaviSpeedDecelRate, apTbtDistance) * 3.6
      if applyTbtSpeed < applySpeed:
        applySpeed = applyTbtSpeed
        safeSpeed = apTbtSpeed
        leftDist = apTbtDistance
        safeDist = safeTbtDist
        speedLimitType = 4

    #log = "{},{:.1f}<{:.1f}/{:.1f},{:.1f} B{} A{:.1f}/{:.1f} N{:.1f}/{:.1f} C{:.1f}/{:.1f} V{:.1f}/{:.1f} ".format(
    #              msg.roadcate, applySpeed, safeSpeed, leftDist, safeDist,
    #              1 if isSpeedBump else 0, 
    #              msg.xSpdLimit, msg.xSpdDist,
    #              msg.camLimitSpeed, msg.camLimitSpeedLeftDist,
    #              CS.speedLimit, CS.speedLimitDistance,
    #              apTbtSpeed, apTbtDistance)
    #self.debugText2 = log
    if speedLimitType == 2:
      self.activeAPM += 1000
    return min(v_cruise_kph, applySpeed) #, roadSpeed, leftDist, speedLimitType

  def decelerate_for_speed_camera(self, safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):
    if left_dist <= safe_dist:
      return safe_speed
    temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
    dV = (-safe_speed + math.sqrt(temp)) * decel_rate
    apply_speed = min(250 , safe_speed + dV)
    min_speed = prev_apply_speed - (decel_rate * 1.8) * 2 * DT_MDL
    apply_speed = max(apply_speed, min_speed)
    return apply_speed

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
    v_cruise_kph = self.vision_turn.update(sm, v_cruise_kph)
    self.turnSpeed = self.vision_turn.turnSpeed
    self.curveSpeed = self.vision_turn.curveSpeed

    v_cruise_kph = self.map_turn.update(sm, v_cruise_kph)
    self.limitSpeed = self.map_turn.limitSpeed
    self.log = self.map_turn.log

    self.navi_helper.update(sm, v_cruise_kph)
    self.leftBlinkerExt = self.navi_helper.leftBlinkerExtCount + self.navi_helper.blinkerExtMode
    self.rightBlinkerExt = self.navi_helper.rightBlinkerExtCount + self.navi_helper.blinkerExtMode

    self.navi_speed_manager.naviDistance = self.navi_helper.naviDistance
    self.navi_speed_manager.naviSpeed = self.navi_helper.naviSpeed
    v_cruise_kph = self.navi_speed_manager.update(sm, v_cruise_kph)
    self.activeAPM = self.navi_speed_manager.activeAPM

    return v_cruise_kph






















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


