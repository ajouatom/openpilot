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
from openpilot.selfdrive.navd.helpers import Coordinate
from openpilot.selfdrive.controls.neokii.navi_controller import SpeedLimiter
EventName = car.CarEvent.EventName


class CarrotPlannerHelper:
  def __init__(self):
    self.params = Params()
    self.params_memory = Params("/dev/shm/params")
    self.params_count = 0
    self.frame = 0

    self.v_cruise_kph = 255

    self.is_metric = self.params.get_bool("IsMetric")
    
    #self.vision_turn = CarrotVisionTurn(self.params)
    self.turnSpeed = 300
    self.curveSpeed = 300

    #self.map_turn = CarrotMapTurnSpeed(self.params, self.params_memory)
    #self.navi_helper = CarrotNaviHelper(self.params, self.params_memory)
    #self.navi_speed_manager = CarrotNaviSpeedManager(self.params, self.params_memory)
    self.activeAPM = 0

    self.gas_override_speed = 0

    self.log = ""

    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.nooHelperActivateCount = 0

  def _params_update(self):
    self.frame += 1
    self.params_count += 1
    if self.params_count == 10:
      #self.vision_turn.update_params()
      pass
    elif self.params_count == 20:
      #self.map_turn.update_params()
      pass
    elif self.params_count == 30:
      #self.navi_helper.update_params()
      pass
    elif self.params_count == 40:
      #self.navi_speed_manager.update_params()
      pass
    elif self.params_count >= 100:
      self.params_count = 0

  def update(self, sm, v_cruise_kph):
    self._params_update()
    enabled = sm['controlsState'].enabled
    v_ego_kph = sm['carState'].vEgo * 3.6
    if enabled:
      self.v_cruise_kph = min(v_cruise_kph, self.v_cruise_kph)
    else:
      self.v_cruise_kph = 255
    
    #vision_turn_kph = self.vision_turn.update(sm, v_cruise_kph, self.v_cruise_kph)
    #self.turnSpeed = self.vision_turn.turnSpeed
    #self.curveSpeed = self.vision_turn.curveSpeed
    #self.event = self.vision_turn.event

    #map_turn_kph = self.map_turn.update(sm, v_cruise_kph, self.v_cruise_kph)
    self.limitSpeed = 0#self.map_turn.limitSpeed
    #if self.map_turn.event >= 0:
    #  self.event = self.map_turn.event

    #navi_helper_kph = self.navi_helper.update(sm, v_cruise_kph, self.v_cruise_kph)
    #self.leftBlinkerExt = self.navi_helper.leftBlinkerExtCount + self.navi_helper.blinkerExtMode
    #self.rightBlinkerExt = self.navi_helper.rightBlinkerExtCount + self.navi_helper.blinkerExtMode

    #if self.navi_helper.event >= 0:
    #  self.event = self.navi_helper.event    

    #navi_speed_manager_kph = self.navi_speed_manager.update(sm, v_cruise_kph, self.v_cruise_kph)
    #self.activeAPM = self.navi_speed_manager.activeAPM
    #if self.navi_speed_manager.event >= 0:
    #  self.event = self.navi_speed_manager.event

    apply_limit_speed, road_limit_speed, left_dist, first_started, cam_type, max_speed_log = \
      SpeedLimiter.instance().get_max_speed(v_cruise_kph, self.is_metric)
    nda_log = "nda_type={} | ".format(cam_type) if apply_limit_speed > 0 else ""
    nda_speed_kph = apply_limit_speed  if apply_limit_speed > 0 else 255

    if sm.alive['carrotMan']:
      carrot_man = sm['carrotMan']
      self.curveSpeed = carrot_man.vTurnSpeed
      desired_speed = carrot_man.desiredSpeed 
      desired_source = carrot_man.desiredSource
      xSpdCountDown = carrot_man.xSpdCountDown if carrot_man.xSpdDist > 0 else 100
      xTurnCountDown = carrot_man.xTurnCountDown if carrot_man.xDistToTurn > 0 else 100
      self.params_memory.put_int_nonblocking("CarrotCountDownSec", min(xSpdCountDown, xTurnCountDown))
      atcType = carrot_man.atcType
      xDistToTurn = carrot_man.xDistToTurn
      nRoadLimitSpeed = carrot_man.nRoadLimitSpeed
      turn_dist = 50
      lane_change_dist = interp(nRoadLimitSpeed, [30, 50, 100], [160, 200, 350])
      self.leftBlinkerExt = self.rightBlinkerExt = 0
      if atcType in ["turn left", "turn right"]:
        if xDistToTurn < turn_dist:
          self.leftBlinkerExt, self.rightBlinkerExt = (20010, 20000) if atcType == "turn left" else (20000, 20010)
        else:
          atcType = "fork left" if atcType == "turn left" else "fork right"

      if atcType in ["fork left", "fork right"] and xDistToTurn < lane_change_dist:
        self.leftBlinkerExt, self.rightBlinkerExt = (10010, 10000) if atcType == "fork left" else (10000, 10010)

      blinkerExt = self.leftBlinkerExt + self.rightBlinkerExt
      self.log = ""
      self.event = -1
      if blinkerExt > 0:
        turning =  blinkerExt >= 40000       
        self.log = "Automatic {} Started. {:.0f}m left".format("Turning" if turning else "Lanechanging", xDistToTurn)
        self.event = EventName.audioTurn if turning else EventName.audioLaneChange

      start_dist = interp(v_ego_kph, [60, 110], [300, 1000])
      if 0 < xDistToTurn < start_dist:
        self.nooHelperActivateCount = max(0, self.nooHelperActivateCount + 1)
      else:
        self.nooHelperActivateCount = min(0, self.nooHelperActivateCount - 1)
        
      if self.autoTurnMapChange > 1:
        if self.nooHelperActivateCount == 10:
          self.params.put_nonblocking("CarrotDisplay", "3")
        elif self.nooHelperActivateCount == - 500:
          self.params.put_nonblocking("CarrotDisplay", "2")

      self.activeAPM = carrot_man.active
      if carrot_man.active >= 2:
        self.activeAPM += 200   # APN active
      elif carrot_man.active >= 1:
        self.activeAPM += 100   # APM active
      if carrot_man.active == 3:
        self.activeAPM += 1000  ## decel alert to hud
    else:
      desired_speed = 250
      desired_source = "none"
      self.curveSpeed = 300
      self.activeAPM = 0


    #self.log = self.vision_turn.log
    #if len(self.log):
    #  self.log += "|"
    #self.log += self.map_turn.log
    #if len(self.log):
    #  self.log += "|"
    #self.log += self.navi_helper.log
    #if len(self.log):
    #  self.log += "|"
    #self.log += self.navi_speed_manager.log

    #self.log = "{} VT{:.1f}, MT{:.1f}, NH{:.1f}, SP{:.1f}".format(self.map_turn.log, vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph)
    #self.v_cruise_kph = min(vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph)

    values_and_names = [
        (v_cruise_kph, "none"),
        #(vision_turn_kph, "vTurn"),
        #(map_turn_kph, "mTurn"),
        #(navi_helper_kph, "noo"),
        #(navi_speed_manager_kph, "navi"),
        (desired_speed, desired_source),
        (nda_speed_kph, "nda"),
    ]

    # min 함수를 사용하여 가장 작은 값을 가진 튜플 찾기
    self.v_cruise_kph, self.source = min(values_and_names, key=lambda x: x[0])
    if self.v_cruise_kph == v_cruise_kph:
      self.source = "none"
      self.gas_override_speed = 0
    else:
      if sm['carState'].gasPressed and self.source not in ["navi", "nda", "section", "cam"]:
        self.gas_override_speed = v_ego_kph
      elif sm['carState'].brakePressed:
        self.gas_override_speed = 0
      #self.log = self.log + "v{:.0f}:m{:.0f},n{:.0f},s{:.0f},g{:.0f}".format(vision_turn_kph, map_turn_kph, navi_helper_kph, navi_speed_manager_kph, self.gas_override_speed)
      #self.log = self.log + "v{:.0f}:,n{:.0f},s{:.0f},g{:.0f}".format(vision_turn_kph, navi_helper_kph, desired_speed, self.gas_override_speed)
      self.log = self.log + "s{:.0f},g{:.0f}".format(desired_speed, self.gas_override_speed)
      self.v_cruise_kph = max(self.v_cruise_kph, self.gas_override_speed)
    return self.v_cruise_kph
