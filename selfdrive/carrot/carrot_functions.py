from openpilot.common.params import Params
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_MDL
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import StreamingMovingAverage
from enum import Enum
import json

from openpilot.selfdrive.selfdrived.events import Events
from cereal import car, log
EventName = log.OnroadEvent.EventName

class XState(Enum):
  lead = 0
  cruise = 1
  e2eCruise = 2
  e2eStop = 3
  e2ePrepare = 4
  e2eStopped = 5

  def __str__(self):
    return self.name

class TrafficState(Enum):
  off = 0
  red = 1
  green = 2

  def __str__(self):
    return self.name

A_CRUISE_MAX_BP_CARROT = [0., 40 * CV.KPH_TO_MS, 60 * CV.KPH_TO_MS, 80 * CV.KPH_TO_MS, 110 * CV.KPH_TO_MS, 140 * CV.KPH_TO_MS]

class CarrotPlanner:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.params_count = 0
    self.frame = 0

    #self.log = ""

    #self.aChangeCost = 200
    #self.aChangeCostStart = 40
    #self.tFollowSpeedAdd = 0.0
    #self.tFollowSpeedAddM = 0.0
    #self.tFollowLeadCarSpeed = 0.0
    #self.tFollowLeadCarAccel = 0.0
    #self.lo_timer = 0 
    #self.v_ego_prev = 0.0

    self.trafficState = TrafficState.off
    self.xStopFilter = StreamingMovingAverage(3)
    self.xStopFilter2 = StreamingMovingAverage(15)
    self.vFilter = StreamingMovingAverage(10)
    #self.t_follow_prev = self.get_T_FOLLOW()
    self.stop_distance = 6.0
    self.fakeCruiseDistance = 0.0
    self.xState = XState.cruise
    self.xStop = 0.0
    self.actual_stop_distance = 0.0
    #self.debugLongText = ""
    self.stopping_count = 0
    self.traffic_starting_count = 0
    self.user_stop_distance = -1
    
    #self.t_follow = 0
    
    self.startSignCount = 0
    self.stopSignCount = 0
    
    self.myDrivingMode = self.params.get_int("MyDrivingMode")
    self.myEcoModeFactor = 0.9 #params.get_float("MyEcoModeFactor") / 100.
    self.mySafeModeFactor = 0.8 #params.get_float("MySafeModeFactor") / 100.
    self.mySafeFactor = 1.0
    if self.myDrivingMode == 1: # eco
      self.mySafeFactor = self.myEcoModeFactor
    elif self.myDrivingMode == 2: #safe
      self.mySafeFactor = self.mySafeModeFactor

    self.stop_distance = 6.0
    self.trafficStopDistanceAdjust = 1.5 #params.get_float("TrafficStopDistanceAdjust") / 100.
    self.comfortBrake = 2.4 #params.get_float("ComfortBrake") / 100.
    self.comfort_brake = self.comfortBrake

    self.soft_hold_active = 0
    self.events = Events()
    self.left_sec = -1
    self.myDrivingMode = 3
    self.myEcoModeFactor = 0.9
    self.mySafeModeFactor = 0.8
    self.myHighModeFactor = 1.2
    self.tFollowGap1 = 1.1
    self.tFollowGap2 = 1.3
    self.tFollowGap3 = 1.45
    self.tFollowGap4 = 1.6

    self.cruiseMaxVals1 = 1.6
    self.cruiseMaxVals2 = 1.2
    self.cruiseMaxVals3 = 1.0
    self.cruiseMaxVals4 = 0.8
    self.cruiseMaxVals5 = 0.7
    self.cruiseMaxVals6 = 0.6


    self.trafficState_carrot = 0

    self.eco_over_speed = 4
    self.eco_target_speed = 0

  def _params_update(self):
    self.frame += 1
    self.params_count += 1
    if self.params_count % 2 == 0:
      self.myDrivingMode = self.params.get_int("MyDrivingMode")
      self.mySafeFactor = 1.0
      if self.myDrivingMode == 1: # eco
        self.mySafeFactor = self.myEcoModeFactor
      elif self.myDrivingMode == 2: #safe
        self.mySafeFactor = self.mySafeModeFactor

    if self.params_count == 10:
      self.myHighModeFactor = 1.2 #float(self.params.get_int("MyHighModeFactor")) / 100.
    elif self.params_count == 20:
      self.tFollowGap1 = self.params.get_float("TFollowGap1") / 100.
      self.tFollowGap2 = self.params.get_float("TFollowGap2") / 100.
      self.tFollowGap3 = self.params.get_float("TFollowGap3") / 100.
      self.tFollowGap4 = self.params.get_float("TFollowGap4") / 100.
    elif self.params_count == 30:
      self.cruiseMaxVals1 = self.params.get_float("CruiseMaxVals1") / 100.
      self.cruiseMaxVals2 = self.params.get_float("CruiseMaxVals2") / 100.
      self.cruiseMaxVals3 = self.params.get_float("CruiseMaxVals3") / 100.
      self.cruiseMaxVals4 = self.params.get_float("CruiseMaxVals4") / 100.
      self.cruiseMaxVals5 = self.params.get_float("CruiseMaxVals5") / 100.
      self.cruiseMaxVals6 = self.params.get_float("CruiseMaxVals6") / 100.
    elif self.params_count == 40:
      self.stop_distance = self.params.get_float("StopDistanceCarrot") / 100.

    elif self.params_count >= 100:
      
      self.params_count = 0

  def get_carrot_accel(self, v_ego):
    cruiseMaxVals = [self.cruiseMaxVals1, self.cruiseMaxVals2, self.cruiseMaxVals3, self.cruiseMaxVals4, self.cruiseMaxVals5, self.cruiseMaxVals6]
    factor = self.myHighModeFactor if self.myDrivingMode == 4 else 1.0
    return interp(v_ego, A_CRUISE_MAX_BP_CARROT, cruiseMaxVals) * factor

  def get_T_FOLLOW(self, personality=log.LongitudinalPersonality.standard):
    if personality==log.LongitudinalPersonality.moreRelaxed:
      return self.tFollowGap4
    elif personality==log.LongitudinalPersonality.relaxed:
      return self.tFollowGap3
    elif personality==log.LongitudinalPersonality.standard:
      return self.tFollowGap2
    elif personality==log.LongitudinalPersonality.aggressive:
      return self.tFollowGap1
    else:
      raise NotImplementedError("Longitudinal personality not supported")

  def update_stop_dist(self, stop_x):
    stop_x = self.xStopFilter.process(stop_x, median = True)
    stop_x = self.xStopFilter2.process(stop_x)
    return stop_x


  def check_model_stopping(self, v, v_ego, model_x, y, d_rel):
    v_ego_kph = v_ego * CV.MS_TO_KPH
    model_v = self.vFilter.process(v[-1])
    startSign = model_v > 5.0 or model_v > (v[0]+2)

    if v_ego_kph < 1.0:
      stopSign = model_x < 20.0 and model_v < 10.0
    elif v_ego_kph < 82.0:
      stopSign = model_x < d_rel and model_x < interp(v[0], [60/3.6, 80/3.6], [120.0, 150]) and ((model_v < 3.0) or (model_v < v[0]*0.7))  and abs(y[-1]) < 5.0
    else:
      stopSign = False

    #self.stopSignCount = self.stopSignCount + 1 if (stopSign and (model_x > get_safe_obstacle_distance(v_ego, t_follow=0, comfort_brake=COMFORT_BRAKE, stop_distance=-1.0))) else 0
    self.stopSignCount = self.stopSignCount + 1 if stopSign else 0
    self.startSignCount = self.startSignCount + 1 if startSign and not stopSign else 0

    if self.stopSignCount * DT_MDL > 0.0:
      self.trafficState = TrafficState.red
    elif self.startSignCount * DT_MDL > 0.2:
      self.trafficState = TrafficState.green
    else:
      self.trafficState = TrafficState.off
  
  def _update_carrot_man(self, sm, v_ego_kph, v_cruise_kph):
    if sm.alive['carrotMan']:
      carrot_man = sm['carrotMan']
      if self.trafficState_carrot == 1 and carrot_man.trafficState == 2:
        if self.soft_hold_active > 0:
          self.events.add(EventName.trafficSignChanged)
        elif self.xState in [XState.e2eStop, XState.e2eStopped]:
          self.xState = XState.e2eCruise
          self.traffic_starting_count = 10.0 / DT_MDL
      self.trafficState_carrot = carrot_man.trafficState
      
      v_cruise_kph = min(v_cruise_kph, carrot_man.desiredSpeed)
      xSpdCountDown = carrot_man.xSpdCountDown if carrot_man.xSpdDist > 0 else 100
      xTurnCountDown = carrot_man.xTurnCountDown if carrot_man.xDistToTurn > 0 else 100
      left_sec = min(xSpdCountDown, xTurnCountDown)
      if left_sec != self.left_sec:
        max_left_sec = min(10, max(5, int(v_ego_kph/10)))
        if 1 <= left_sec <= max_left_sec:
          #self.events.add(getattr(EventName, f'audio{left_sec}'))
          self.params_memory.put_int_nonblocking("CarrotCountDownSec", left_sec)
        elif left_sec == 0 and self.left_sec == 1:
          #self.events.add(EventName.audio0)
          self.params_memory.put_int_nonblocking("CarrotCountDownSec", left_sec)
        self.left_sec = left_sec

    return v_cruise_kph

  def cruise_eco_control(self, v_ego_kph, v_cruise_kph):
    v_cruise_kph_apply = v_cruise_kph
    if self.eco_over_speed > 0:
      if self.eco_target_speed > 0:
        if self.eco_target_speed < v_cruise_kph:
          self.eco_target_speed = v_cruise_kph
        elif self.eco_target_speed > v_cruise_kph:
          self.eco_target_speed = 0
      elif self.eco_target_speed == 0 and v_ego_kph + 3 < v_cruise_kph and v_cruise_kph > 20.0:  # 주행중 속도가 떨어지면 다시 크루즈연비제어 시작.
        self.eco_target_speed = v_cruise_kph

      if self.eco_target_speed != 0:  ## 크루즈 연비 제어모드 작동중일때: 연비제어 종료지점
        if v_ego_kph > self.eco_target_speed: # 설정속도를 초과하면..
          self.eco_target_speed = 0
        else:
          v_cruise_kph_apply = self.eco_target_speed + self.eco_over_speed  # + 설정 속도로 설정함.
    else:
      self.eco_target_speed = 0

    return v_cruise_kph_apply

  def update(self, sm, v_cruise_kph):
    self._params_update()

    self.events = Events()
    carstate = sm['carState']
    vCluRatio = carstate.vCluRatio
    #controlsState = sm['controlsState']
    radarstate = sm['radarState']
    model = sm['modelV2']

    #self.soft_hold_active = sm['carControl'].hudControl.softHoldActive # carrot 1
    self.soft_hold_active = sm['carState'].softHoldActive # carrot 2

    self.comfort_brake = self.comfortBrake
	
    v_ego = carstate.vEgo
    v_ego_kph = v_ego * CV.MS_TO_KPH
    v_ego_cluster = carstate.vEgoCluster
    v_ego_cluster_kph = v_ego_cluster * CV.MS_TO_KPH

    v_cruise_kph = self.cruise_eco_control(v_ego_cluster_kph, v_cruise_kph)
    v_cruise_kph = self._update_carrot_man(sm, v_ego_kph, v_cruise_kph)

    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if vCluRatio > 0.5:
      v_cruise *= vCluRatio

    x = model.position.x
    y = model.position.y
    v = model.velocity.x

    self.fakeCruiseDistance = 0.0
    radar_detected = radarstate.leadOne.status & radarstate.leadOne.radar

    self.xStop = self.update_stop_dist(x[31])
    stop_model_x = self.xStop

    #self.check_model_stopping(v, v_ego, self.xStop, y)
    self.check_model_stopping(v, v_ego, x[-1], y, radarstate.leadOne.dRel if radarstate.leadOne.status else 1000)

    if self.myDrivingMode == 4:
      self.trafficState = TrafficState.off
    
    #self.update_user_control()

    if carstate.gasPressed or carstate.brakePressed:
      self.user_stop_distance = -1

    if self.xState == XState.e2eStopped:
      if carstate.gasPressed:
        self.xState = XState.e2ePrepare
      elif radar_detected and (radarstate.leadOne.dRel - stop_model_x) < 2.0:
        self.xState = XState.lead
      elif self.stopping_count == 0:
        if self.trafficState == TrafficState.green:
          self.xState = XState.e2ePrepare
      self.stopping_count = max(0, self.stopping_count - 1)
      v_cruise = 0
    elif self.xState == XState.e2eStop:
      self.stopping_count = 0
      if carstate.gasPressed:  # Stop detecting traffic signal for 10 seconds
        #self.xState = XState.e2ePrepare
        self.xState = XState.e2eCruise
        self.traffic_starting_count = 10.0 / DT_MDL
      elif radar_detected and (radarstate.leadOne.dRel - stop_model_x) < 2.0:
        self.xState = XState.lead
      else:
        if self.trafficState == TrafficState.green:
          self.events.add(EventName.trafficSignGreen)
          self.xState = XState.e2ePrepare
        else:
          self.comfort_brake = self.comfortBrake * 0.9
          #self.comfort_brake = COMFORT_BRAKE
          self.trafficStopAdjustRatio = interp(v_ego_kph, [0, 100], [1.0, 0.7])
          stop_dist = self.xStop * interp(self.xStop, [0, 100], [1.0, self.trafficStopAdjustRatio])  ##�����Ÿ��� ���� �����Ÿ� ��������
          if stop_dist > 10.0: ### 10M�̻��϶���, self.actual_stop_distance�� ������Ʈ��.
            self.actual_stop_distance = stop_dist
          stop_model_x = 0
          self.fakeCruiseDistance = 0 if self.actual_stop_distance > 10.0 else 10.0
          if v_ego < 0.3:
            self.stopping_count = 0.5 / DT_MDL
            self.xState = XState.e2eStopped
    elif self.xState == XState.e2ePrepare:
      if radar_detected:
        self.xState = XState.lead
      elif v_ego_kph < 5.0 and self.trafficState != TrafficState.green:
        self.xState = XState.e2eStop
        self.actual_stop_distance = 2.0
      elif v_ego_kph > 5.0: # and stop_model_x > 30.0:
        self.xState = XState.e2eCruise
    else: #XState.lead, XState.cruise, XState.e2eCruise
      self.traffic_starting_count = max(0, self.traffic_starting_count - 1)
      if radar_detected:
        self.xState = XState.lead
      elif self.trafficState == TrafficState.red and abs(carstate.steeringAngleDeg) < 30 and self.traffic_starting_count == 0:
        self.events.add(EventName.trafficStopping)
        self.xState = XState.e2eStop
        self.actual_stop_distance = self.xStop
      else:
        self.xState = XState.e2eCruise

    if self.trafficState in [TrafficState.off, TrafficState.green] or self.xState not in [XState.e2eStop, XState.e2eStopped]:
      stop_model_x = 1000.0

    if self.user_stop_distance >= 0:
      self.user_stop_distance = max(0, self.user_stop_distance - v_ego * DT_MDL)
      self.actual_stop_distance = self.user_stop_distance
      self.xState = XState.e2eStop if self.user_stop_distance > 0 else XState.e2eStopped
      
    mode = 'blended' if self.xState in [XState.e2ePrepare] else 'acc'

    self.comfort_brake *= self.mySafeFactor
    self.actual_stop_distance = max(0, self.actual_stop_distance - (v_ego * DT_MDL))
    
    if stop_model_x == 1000.0: ##  e2eCruise, lead�ΰ��
      self.actual_stop_distance = 0.0
    elif self.actual_stop_distance > 0: ## e2eStop, e2eStopped�ΰ��..
      stop_model_x = 0.0
      
    #self.debugLongText = "XState({}),stop_x={:.1f},stopDist={:.1f},Traffic={}".format(str(self.xState), stop_x, self.actual_stop_distance, str(self.trafficState))
    #��ȣ�� �������� self.xState.value
      
    stop_dist =  stop_model_x + self.actual_stop_distance
    stop_dist = max(stop_dist, v_ego ** 2 / (self.comfort_brake * 2))

    self.v_cruise = v_cruise
    self.stop_dist = stop_dist
    self.mode = mode
    #return v_cruise, stop_dist, mode

    return v_cruise_kph
