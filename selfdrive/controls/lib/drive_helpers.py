import math
import json

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_MDL, DT_CTRL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params
import numpy as np
import collections

EventName = car.CarEvent.EventName
AudibleAlert = car.CarControl.HUDControl.AudibleAlert

# WARNING: this value was determined based on the model's training distribution,
#          model predictions above this speed can be unpredictable
# V_CRUISE's are in kph
V_CRUISE_MIN = 8
V_CRUISE_MAX = 161 #145
V_CRUISE_UNSET = 255
V_CRUISE_INITIAL = 10 #40
V_CRUISE_INITIAL_EXPERIMENTAL_MODE = 105
IMPERIAL_INCREMENT = 1.6  # should be CV.MPH_TO_KPH, but this causes rounding errors

MIN_DIST = 0.001
MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0

# EU guidelines
MAX_LATERAL_JERK = 5.0

MAX_VEL_ERR = 5.0

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
CRUISE_LONG_PRESS = 50
CRUISE_NEAREST_FUNC = {
  ButtonType.accelCruise: math.ceil,
  ButtonType.decelCruise: math.floor,
}
CRUISE_INTERVAL_SIGN = {
  ButtonType.accelCruise: +1,
  ButtonType.decelCruise: -1,
}


class VCruiseHelper:
  def __init__(self, CP):
    self.CP = CP
    self.v_cruise_kph = V_CRUISE_INITIAL #V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_INITIAL #V_CRUISE_UNSET
    self.v_cruise_kph_last = 0
    self.button_timers = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0}
    self.button_change_states = {btn: {"standstill": False, "enabled": False} for btn in self.button_timers}

    # ajouatom
    self.brake_pressed_count = 0
    self.brake_pressed_frame = 0
    self.gas_pressed_count = 0
    self.gas_pressed_count_prev = 0
    self.gas_pressed_max_aego = 0.0
    self.gas_pressed_value = 0
    self.softHoldActive = 0
    self.button_cnt = 0
    self.long_pressed = False
    self.button_prev = ButtonType.unknown
    self.cruiseActivate = 0
    self.params = Params()
    self.v_cruise_kph_set = V_CRUISE_INITIAL #V_CRUISE_UNSET
    self.cruiseSpeedTarget = 0
    self.roadSpeed = 0
    self.xState = 0
    self.trafficState = 0
    self.sendEvent_frame = 0
    self.softHold_count = 0
    self.cruiseActiveReady = 0
    self.autoCruiseCancelState = 0  # 0: normal, 1:cancel, 2: timer cancel
    self.xIndex = 0
    self.frame = 0
    self._log_timer = 0
    self.debugText = ""
    
    self.debugText2 = ""
    self.blinkerExtMode = 0 # 0: Normal, 10000: voice
    self.rightBlinkerExtCount = 0
    self.leftBlinkerExtCount = 0
    self.cruiseSpeedMax = V_CRUISE_MAX
    self.autoCruiseCancelTimer = 0
    self.sendEvent = None
    self.activeAVM = 0
    self.v_ego_kph_prev = 0.0
    self.gas_tok_frame = 0
    self.xPosValidCount = 0
    self.button_long_time = 40
    self.accel_output = 0.0
    self.traffic_light_q = collections.deque(maxlen=int(2.0/DT_CTRL))
    self.traffic_light_count = -1
    self.traffic_state = 0

    self.v_ego_kph_set = 0
    self.left_sec = 11
    
    #ajouatom: params
    self.params_count = 0
    self.autoResumeFromGasSpeed = self.params.get_int("AutoResumeFromGasSpeed")
    self.autoCancelFromGasMode = self.params.get_int("AutoCancelFromGasMode")
    self.autoResumeFromBrakeReleaseTrafficSign = self.params.get_int("AutoResumeFromBrakeReleaseTrafficSign")
    self.autoCruiseControl = self.params.get_int("AutoCruiseControl")
    self.cruiseButtonMode = self.params.get_int("CruiseButtonMode")
    self.autoSpeedUptoRoadSpeedLimit = float(self.params.get_int("AutoSpeedUptoRoadSpeedLimit")) * 0.01
    self.cruiseOnDist = float(int(self.params.get("CruiseOnDist", encoding="utf8"))) / 100.
    self.softHoldMode = self.params.get_int("SoftHoldMode")
    self.cruiseSpeedMin = self.params.get_int("CruiseSpeedMin")
    self.showDebugUI= self.params.get_int("ShowDebugUI")
    
    self.speedFromPCM = self.params.get_int("SpeedFromPCM")
    self.cruiseEcoControl = self.params.get_int("CruiseEcoControl")
    self.cruiseSpeedUnit = self.params.get_int("CruiseSpeedUnit")

    self.useLaneLineSpeed = self.params.get_int("UseLaneLineSpeed")
    self.params.put_int("UseLaneLineSpeedApply", self.useLaneLineSpeed)

  def _params_update(self, controls):
    self.frame += 1
    self.params_count += 1
    if self.params_count == 10:
      useLaneLineSpeed = self.params.get_int("UseLaneLineSpeed")
      if self.useLaneLineSpeed != useLaneLineSpeed:
        self.params.put_int("UseLaneLineSpeedApply", useLaneLineSpeed)
      self.useLaneLineSpeed = useLaneLineSpeed
    elif self.params_count == 20:
      self.autoResumeFromGasSpeed = self.params.get_int("AutoResumeFromGasSpeed")
      self.autoCancelFromGasMode = self.params.get_int("AutoCancelFromGasMode")
      self.autoResumeFromBrakeReleaseTrafficSign = self.params.get_int("AutoResumeFromBrakeReleaseTrafficSign")
      self.autoCruiseControl = self.params.get_int("AutoCruiseControl")
      self.cruiseButtonMode = self.params.get_int("CruiseButtonMode")
      self.cruiseOnDist = float(self.params.get_int("CruiseOnDist")) / 100.
      self.softHoldMode = self.params.get_int("SoftHoldMode")
      self.cruiseSpeedMin = self.params.get_int("CruiseSpeedMin")
    elif self.params_count == 30:
      self.autoSpeedUptoRoadSpeedLimit = float(self.params.get_int("AutoSpeedUptoRoadSpeedLimit")) * 0.01
    elif self.params_count == 40:
      self.cruiseSpeedUnit = self.params.get_int("CruiseSpeedUnit")
    elif self.params_count == 50:
      self.cruiseEcoControl = self.params.get_int("CruiseEcoControl")
    elif self.params_count >= 100:
      self.showDebugUI = self.params.get_int("ShowDebugUI")      
      self.speedFromPCM = self.params.get_int("SpeedFromPCM")
      self.params_count = 0
    
  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, controls):
    self.v_cruise_kph_last = self.v_cruise_kph

    self._params_update(controls)
    self._add_log("")

    if CS.cruiseState.available:
      if not self.CP.pcmCruise:
        # if stock cruise is completely disabled, then we can use our own set speed logic
        #self._update_v_cruise_non_pcm(CS, enabled, is_metric)
        self._update_v_cruise_apilot(CS, controls)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self._update_event_apilot(CS, controls)
        #self.update_button_timers(CS, enabled)
      else:
        #self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        #self.v_cruise_cluster_kph = self.v_cruise_kph_set = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if self.params.get_int("SpeedFromPCM") == 1:
          self.v_cruise_kph_set = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        self._update_v_cruise_apilot(CS, controls)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        if self.params.get_int("SpeedFromPCM") == 0:
          self._update_event_apilot(CS, controls)
    else:
      self.v_cruise_kph = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.v_cruise_kph_set = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.cruiseActivate = 0
      v_cruise_kph = self.update_apilot_cmd(controls, 30)

    count_down_kph = self.params.get_int("CarrotCountDownSpeed")
    left_sec = self.params.get_int("CarrotCountDownSec")
    if left_sec != self.left_sec and count_down_kph != 0:
      max_left_sec = min(10, max(5, int(self.v_ego_kph_set/10.)))
      if 1 <= left_sec <= max_left_sec and self.v_ego_kph_set > count_down_kph:
        controls.carrot_alert_sound = getattr(AudibleAlert, f'audio{left_sec}')
        #event_name  = getattr(EventName, f'audio{left_sec}')
        #controls.events.add(event_name)
      elif left_sec == 0 and self.left_sec == 1:
        controls.carrot_alert_sound = AudibleAlert.longDisengaged
      self.left_sec = left_sec

  def _update_v_cruise_non_pcm(self, CS, enabled, is_metric):
    # handle button presses. TODO: this should be in state_control, but a decelCruise press
    # would have the effect of both enabling and changing speed is checked after the state transition
    if not enabled:
      return

    long_press = False
    button_type = None

    v_cruise_delta = 1. if is_metric else IMPERIAL_INCREMENT

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers and not b.pressed:
        if self.button_timers[b.type.raw] > CRUISE_LONG_PRESS:
          return  # end long press
        button_type = b.type.raw
        break
    else:
      for k in self.button_timers.keys():
        if self.button_timers[k] and self.button_timers[k] % CRUISE_LONG_PRESS == 0:
          button_type = k
          long_press = True
          break

    if button_type is None:
      return

    # Don't adjust speed when pressing resume to exit standstill
    cruise_standstill = self.button_change_states[button_type]["standstill"] or CS.cruiseState.standstill
    if button_type == ButtonType.accelCruise and cruise_standstill:
      return

    # Don't adjust speed if we've enabled since the button was depressed (some ports enable on rising edge)
    if not self.button_change_states[button_type]["enabled"]:
      return

    v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
    if long_press and self.v_cruise_kph % v_cruise_delta != 0:  # partial interval
      self.v_cruise_kph = CRUISE_NEAREST_FUNC[button_type](self.v_cruise_kph / v_cruise_delta) * v_cruise_delta
    else:
      self.v_cruise_kph += v_cruise_delta * CRUISE_INTERVAL_SIGN[button_type]

    # If set is pressed while overriding, clip cruise speed to minimum of vEgo
    if CS.gasPressed and button_type in (ButtonType.decelCruise, ButtonType.setCruise):
      self.v_cruise_kph = max(self.v_cruise_kph, CS.vEgo * CV.MS_TO_KPH)

    self.v_cruise_kph = clip(round(self.v_cruise_kph, 1), self.cruiseSpeedMin, self.cruiseSpeedMax)

  def update_button_timers(self, CS, enabled):
    # increment timer for buttons still pressed
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1

    for b in CS.buttonEvents:
      if b.type.raw in self.button_timers:
        # Start/end timer and store current state on change of button pressed
        self.button_timers[b.type.raw] = 1 if b.pressed else 0
        self.button_change_states[b.type.raw] = {"standstill": CS.cruiseState.standstill, "enabled": enabled}

  def initialize_v_cruise(self, CS, experimental_mode: bool) -> None:
    # initializing is handled by the PCM
    #if self.CP.pcmCruise:
    #  return

    #carrot
    if len(CS.buttonEvents) == 0:
      return

    initial = V_CRUISE_INITIAL_EXPERIMENTAL_MODE if experimental_mode else V_CRUISE_INITIAL

    # 250kph or above probably means we never had a set speed
    if any(b.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for b in CS.buttonEvents) and self.v_cruise_kph_last < 250:
      self.v_cruise_kph = self.v_cruise_kph_set = self.v_cruise_kph_last
      self._add_log("Button init speed...{:.0f}".format(self.v_cruise_kph))
    else:
      self.v_cruise_kph = self.v_cruise_kph_set = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, self.cruiseSpeedMax)))
      self._add_log("Button init current speed...{:.0f}".format(self.v_cruise_kph))

    self.v_cruise_cluster_kph = self.v_cruise_kph

  def _make_event(self, controls, event_name):
    if (self.frame - self.sendEvent_frame) > (5.0 / DT_CTRL) or event_name != self.sendEvent:
      self.sendEvent = event_name
      controls.events.add(event_name)
      self.sendEvent_frame = self.frame

  def _update_event_apilot(self, CS, controls):
    lp = controls.sm['longitudinalPlan']
    xState = lp.xState
    trafficState = lp.trafficState

    if xState != self.xState and controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0: #0:lead, 1:cruise, 2:e2eCruise, 3:e2eStop, 4:e2ePrepare, 5:e2eStopped
      if xState == 3 and CS.vEgo > 5.0:
        self._make_event(controls, EventName.trafficStopping)  # stopping
      elif xState == 4 and self.softHoldActive == 0:
        self._make_event(controls, EventName.trafficSignGreen) # starting
    self.xState = xState

    if trafficState != self.trafficState: #0: off, 1:red, 2:green
      if self.softHoldActive == 2 and trafficState == 2:
        self._make_event(controls, EventName.trafficSignChanged)
    self.trafficState = trafficState

  def _update_lead(self, controls):
    leadOne = controls.sm['radarState'].leadOne
    if leadOne.status: # and leadOne.radar:
      self.lead_dRel = leadOne.dRel
      self.lead_vRel = leadOne.vRel
      self.lead_vLead = leadOne.vLeadK
    else:
      self.lead_dRel = 0
      self.lead_vRel = 0
      self.lead_vLead = 0

  def _update_v_cruise_apilot(self, CS, controls):

    self.rightBlinkerExtCount = max(self.rightBlinkerExtCount - 1, 0)
    self.leftBlinkerExtCount = max(self.leftBlinkerExtCount - 1, 0)
    if self.rightBlinkerExtCount + self.leftBlinkerExtCount <= 0:
      self.blinkerExtMode = 0

    ## autoCruise가 핸들을 60도이상 돌리면.. 10초간 일시정지된다.
    if abs(CS.steeringAngleDeg) > 60 and self.autoCruiseControl != 3:
      if self.autoCruiseCancelTimer == 0:
        self._add_log_auto_cruise("autoCruise paused for 40 seconds.")
        controls.events.add(EventName.audioPrompt)
      self.autoCruiseCancelTimer = int(10. / DT_CTRL)

    if self.autoCruiseCancelTimer > 0:
      if self.autoCruiseCancelTimer % int(1 / DT_CTRL) == 0:
        self._add_log_auto_cruise("autoCruise paused for {:.0f}seconds.".format(self.autoCruiseCancelTimer*DT_CTRL))
    if self.autoCruiseCancelTimer == 1:
      controls.events.add(EventName.audioPrompt)
      self._add_log_auto_cruise("autoCruise activated.")

    if False:#self.v_ego_kph_set > 20:
      self.autoCruiseCancelTimer = max(self.autoCruiseCancelTimer - 5, 0)
    else:
      self.autoCruiseCancelTimer = max(self.autoCruiseCancelTimer - 1, 0)

    self._update_lead(controls)
    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    if self.v_cruise_kph_set > 200:
      self.v_cruise_kph_set = self.cruiseSpeedMin
    v_cruise_kph = self.v_cruise_kph_set    
    v_cruise_kph = self._update_cruise_carrot(CS, v_cruise_kph, controls)
    v_cruise_kph_apply = self.cruise_control_speed(v_cruise_kph)

    self.v_cruise_kph_set = v_cruise_kph
    self.v_cruise_kph = v_cruise_kph_apply

  def update_apilot_cmd(self, controls, v_cruise_kph):
    msg = controls.sm['roadLimitSpeed']
    nda = controls.sm['naviData']
    if nda.active:
      self.roadSpeed = nda.roadLimitSpeed
    else:
      self.roadSpeed = clip(0, msg.roadLimitSpeed, 150.0)
    self.xPosValidCount = msg.xPosValidCount

    if msg.xIndex > 0 and msg.xIndex != self.xIndex:      
      self.xIndex = msg.xIndex
      if msg.xCmd == "SPEED":
        if msg.xArg == "UP":
          v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
        elif msg.xArg == "DOWN":
          if self.v_ego_kph_set < v_cruise_kph:
            v_cruise_kph = self.v_ego_kph_set
          elif v_cruise_kph > 30:
            v_cruise_kph -= 10
        else:
          v_cruise_kph = clip(int(msg.xArg), self.cruiseSpeedMin, self.cruiseSpeedMax)
      elif msg.xCmd == "CRUISE":
        if msg.xArg == "ON":
          if not controls.enabled:
            self.cruiseActivate = 1
        elif msg.xArg == "OFF":
          if controls.enabled:
            self.cruiseActiveReady = 1
            self.cruiseActivate = -1
            controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "GO":
          if not controls.enabled:
            self.cruiseActivate = 1
          elif self.softHoldActive > 0:
            self.softHoldActive = 0
          #elif self.xState in [XState.softHold, XState.e2eStop]:
          #  controls.cruiseButtonCounter += 1
          else:
            v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
        elif msg.xArg == "STOP":
          #if self.xState in [XState.e2eStop, XState.e2eCruisePrepare]:
          #  controls.cruiseButtonCounter -= 1
          #else:
          v_cruise_kph = 20
      elif msg.xCmd == "LANECHANGE":
        blinkerExtState = self.leftBlinkerExtCount + self.rightBlinkerExtCount
        if msg.xArg == "RIGHT":
          self.rightBlinkerExtCount = 50
        elif msg.xArg == "LEFT":
          self.leftBlinkerExtCount = 50
        if blinkerExtState <= 0 and self.leftBlinkerExtCount + self.rightBlinkerExtCount > 0:
          self._make_event(controls, EventName.audioLaneChange)
      elif msg.xCmd == "RECORD":
        if msg.xArg == "START":
          self.params.put_nonblocking("CarrotRecord", "1")
          controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "STOP":
          self.params.put_nonblocking("CarrotRecord", "2")
          controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "TOGGLE":
          self.params.put_nonblocking("CarrotRecord", "3")
          controls.events.add(EventName.audioPrompt)
      elif msg.xCmd == "DISPLAY":
        if msg.xArg == "MAP":
          self.params.put_nonblocking("CarrotDisplay", "3")
          controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "FULLMAP":
          self.params.put_nonblocking("CarrotDisplay", "4")
          controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "DEFAULT":
          self.params.put_nonblocking("CarrotDisplay", "1")
          controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "ROAD":
          self.params.put_nonblocking("CarrotDisplay", "2")
          controls.events.add(EventName.audioPrompt)
        elif msg.xArg == "TOGGLE":
          self.params.put_nonblocking("CarrotDisplay", "5")
          controls.events.add(EventName.audioPrompt)

      elif msg.xCmd == "DETECT":
        self.debugText2 = "DETECT[{}]={}".format(msg.xIndex, msg.xArg)
        elements = [element.strip() for element in msg.xArg.split(',')]
        self.traffic_light(float(elements[1]), float(elements[2]), elements[0])
        self.traffic_light_count = 0.5 / DT_CTRL
    self.traffic_light_q.append((-1, -1, "none"))
    self.traffic_light_count -= 1
    if self.traffic_light_count < 0:
      self.traffic_light_count = -1
      self.traffic_state = 0
    return v_cruise_kph

  def traffic_light(self, x, y, color):    
    traffic_state1 = 0
    traffic_state2 = 0
    traffic_state11 = 0
    traffic_state22 = 0
    for pdata in self.traffic_light_q:
      px, py, pcolor = pdata
      if abs(x - px) < 0.2 and abs(y - py) < 0.2:
        if pcolor in ["Green Light", "Left turn"]:
          if color in ["Red Light", "Yello Light"]:
            traffic_state11 += 1
          elif color in ["Green Light", "Left turn"]:
            traffic_state2 += 1
        elif pcolor in ["Red Light", "Yello Light"]:
          if color in ["Green Light", "Left turn"]:
            traffic_state22 += 1
          elif color in ["Red Light", "Yello Light"]:
            traffic_state1 += 1

    #print(self.traffic_light_q)
    if traffic_state11 > 0:
      self.traffic_state = 11
      self._add_log("Red light triggered")
      #print("Red light triggered")
    elif traffic_state22 > 0:
      self.traffic_state = 22
      self._add_log("Green light triggered")
      #print("Green light triggered")
    elif traffic_state1 > 0:
      self.traffic_state = 1
      self._add_log("Red light continued")
      #print("Red light continued")
    elif traffic_state2 > 0:
      self.traffic_state = 2
      self._add_log("Green light continued")
      #print("Green light continued")
    else:
      pass
      #self.traffic_state = 0
      #print("TrafficLight none")

    self.traffic_light_q.append((x,y,color))

  def _add_log_auto_cruise(self, log):
    if self.autoCruiseControl > 0:
      self._add_log(log)

  def _add_log(self, log):
    if len(log) == 0:
      self._log_timer = max(0, self._log_timer - 1)
      if self._log_timer <= 0:
        self.debugText = ""
    else:
      self.debugText = log
      self._log_timer = 300

  def _gas_released_cond(self, CS, v_cruise_kph, controls):
    if 0 < self.lead_dRel < CS.vEgo * 0.8 and self.autoCancelFromGasMode > 0:
      self.cruiseActivate = -1
      self._add_log_auto_cruise("Cruise Deactivate from gas.. too close leadCar!")
    elif self.autoCancelFromGasMode > 0 and self.v_ego_kph_set < self.autoResumeFromGasSpeed:
      self._add_log_auto_cruise("Cruise Deactivate from gas speed:{:.0f}".format(self.autoResumeFromGasSpeed));
      self.cruiseActivate = -1
    elif self.xState == 3 and self.autoCancelFromGasMode == 2:
      v_cruise_kph = self.v_ego_kph_set
      self._add_log_auto_cruise("Cruise Deactivate from gas pressed: traffic stopping");
      self.cruiseActivate = -1
      self.autoCruiseCancelTimer = 3.0 / DT_CTRL
    elif self.v_ego_kph_set > self.autoResumeFromGasSpeed > 0:
      if self.cruiseActivate <= 0:
        if self.gas_pressed_value > 0.6 or self.gas_pressed_count_prev > 3.0 / DT_CTRL:
          if self.gas_pressed_max_aego < 1.5 or self.gas_pressed_value < 0.3:
            v_cruise_kph = self.v_ego_kph_set
          self.autoCruiseCancelTimer = 0
          self._add_log_auto_cruise("Cruise Activate from gas(deep/long pressed)")          
        else:
          v_cruise_kph = self.v_ego_kph_set
          self._add_log_auto_cruise("Cruise Activate from gas(speed)")
      self.cruiseActivate = 1
    return v_cruise_kph

  def _brake_released_cond(self, CS, v_cruise_kph, controls):
    if self.autoResumeFromBrakeReleaseTrafficSign:
      if self.autoResumeFromGasSpeed < self.v_ego_kph_set:
        v_cruise_kph = self.v_ego_kph_set
        self._add_log_auto_cruise("Cruise Activate Brake Release")
        self.cruiseActivate = 1
      elif self.xState in [3, 5]:
        #v_cruise_kph = self.v_ego_kph_set
        self._add_log_auto_cruise("Cruise Activate from Traffic sign stop")
        self.cruiseActivate = 1
      elif 0 < self.lead_dRel < 20:
        v_cruise_kph = self.v_ego_kph_set  ## 천천히 주행하다가..지나가는 차를 잘못읽고 자동으로 크루즈가 켜지는 경우 툭튀언
        self._add_log_auto_cruise("Cruise Activate from Lead Car")
        self.cruiseActivate = 1
    return v_cruise_kph

  def _update_cruise_button(self, CS, v_cruise_kph, controls):
    ## ButtonEvent process
    button_kph = v_cruise_kph
    buttonEvents = CS.buttonEvents
    button_speed_up_diff = 1
    button_speed_dn_diff = self.cruiseSpeedUnit if self.cruiseButtonMode in [1, 2, 3] else 1

    button_type = 0
    if self.button_cnt > 0:
      self.button_cnt += 1
    for b in buttonEvents:
      if b.pressed and self.button_cnt==0 and b.type in [ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.gapAdjustCruise, ButtonType.cancel, ButtonType.lfaButton]:
        self.button_cnt = 1
        self.button_prev = b.type
        if b.type in [ButtonType.accelCruise, ButtonType.decelCruise]:
          self.button_long_time = 40
        else:
          self.button_long_time = 70
      elif not b.pressed and self.button_cnt > 0:
        if b.type == ButtonType.cancel:
          button_type = ButtonType.cancel
        elif not self.long_pressed and b.type == ButtonType.accelCruise:
          button_kph += button_speed_up_diff if controls.is_metric else button_speed_up_diff * CV.MPH_TO_KPH
          button_type = ButtonType.accelCruise
        elif not self.long_pressed and b.type == ButtonType.decelCruise:
          button_kph -= button_speed_dn_diff if controls.is_metric else button_speed_dn_diff * CV.MPH_TO_KPH
          button_type = ButtonType.decelCruise
        elif not self.long_pressed and b.type == ButtonType.gapAdjustCruise:
          button_type = ButtonType.gapAdjustCruise
        elif not self.long_pressed and b.type == ButtonType.lfaButton:
          button_type = ButtonType.lfaButton

        self.long_pressed = False
        self.button_cnt = 0
    if self.button_cnt > self.button_long_time:
      self.long_pressed = True
      V_CRUISE_DELTA = 10
      if self.button_prev == ButtonType.cancel:
        button_type = ButtonType.cancel
        self.button_cnt = 0          
      elif self.button_prev == ButtonType.accelCruise:
        button_kph += V_CRUISE_DELTA - button_kph % V_CRUISE_DELTA
        button_type = ButtonType.accelCruise
        self.button_cnt %= self.button_long_time
      elif self.button_prev == ButtonType.decelCruise:
        button_kph -= V_CRUISE_DELTA - -button_kph % V_CRUISE_DELTA
        button_type = ButtonType.decelCruise
        self.button_cnt %= self.button_long_time
      elif self.button_prev == ButtonType.gapAdjustCruise:
        button_type = ButtonType.gapAdjustCruise
        self.button_cnt %= self.button_long_time
      elif self.button_prev == ButtonType.lfaButton:
        button_type = ButtonType.lfaButton
        self.button_cnt %= self.button_long_time

    button_kph = clip(button_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)

    if button_type != 0 and controls.enabled:
      if self.long_pressed:
        if button_type in [ButtonType.accelCruise]:
          v_cruise_kph = button_kph
          self._add_log("Button long pressed..{:.0f}".format(v_cruise_kph))
        elif button_type in [ButtonType.decelCruise]:
          if self.cruiseButtonMode in [2,3] and self.params.get_int("EnableAVM") > 0:
            self.activeAVM = 2 if self.activeAVM == 0 else 0
            self._add_log("Button long pressed..Enable AVM{}".format(self.activeAVM))
          elif self.cruiseButtonMode in [3]:
            self.traffic_light_count = 0.5 / DT_CTRL
            self.traffic_state = 33
            controls.events.add(EventName.audioPrompt)
            self._add_log("Button force decel")
          else:
            v_cruise_kph = button_kph
            self._add_log("Button long pressed..{:.0f}".format(v_cruise_kph))
        elif button_type == ButtonType.gapAdjustCruise:          
          if False: #self.CP.pcmCruise:
            self._add_log("Button long gap pressed ..pcmCruise can't adjust")
          else:
            self._add_log("Button long gap pressed ..")
            self.params.put_int_nonblocking("MyDrivingMode", self.params.get_int("MyDrivingMode") % 4 + 1) # 1,2,3,4 (1:eco, 2:safe, 3:normal, 4:high speed)
        elif button_type == ButtonType.lfaButton:
          self._add_log("Button long lkas pressed ..")
          useLaneLineSpeed = max(1, self.useLaneLineSpeed)
          self.params.put_int_nonblocking("UseLaneLineSpeedApply", useLaneLineSpeed if self.params.get_int("UseLaneLineSpeedApply") == 0 else 0)
      else:
        if button_type == ButtonType.accelCruise:
          if self.softHoldActive > 0 and self.autoCruiseControl > 0:
            self.softHoldActive = 0
            self._add_log("Button softhold released ..")
          elif self.xState in [3, 5] and self.cruiseButtonMode in [3]: ## 5:e2eStopped
            self.traffic_light_count = 0.5 / DT_CTRL
            self.traffic_state = 22
            self._add_log("Button start (traffic ignore)")
          else:
            if self.cruiseButtonMode == 0:
              v_cruise_kph = button_kph
            elif self.cruiseButtonMode in [1,2,3]:
              v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
            self._add_log("Button speed up...{:.0f}".format(v_cruise_kph))
        elif button_type == ButtonType.decelCruise:
          if self.autoCruiseControl == 0 or self.cruiseButtonMode in [0,1]:
            v_cruise_kph = button_kph
            self._add_log("Button speed down...{:.0f}".format(v_cruise_kph))
          elif v_cruise_kph > self.v_ego_kph_set + 0:
            v_cruise_kph = self.v_ego_kph_set
            self._add_log("Button speed set...{:.0f}".format(v_cruise_kph))
          else:
            #v_cruise_kph = button_kph
            self.cruiseActiveReady = 1
            self.cruiseActivate = -1
            controls.events.add(EventName.audioPrompt)
        elif button_type == ButtonType.cancel:
          print("************* cancel button pressed..")
        elif button_type == ButtonType.gapAdjustCruise:
          if False: #self.CP.pcmCruise:
            self._add_log("Button long gap pressed ..pcmCruise can't adjust")
          else:
            self._add_log("Button gap pressed ..")
            longitudinalPersonalityMax = self.params.get_int("LongitudinalPersonalityMax")
            controls.personality = (controls.personality - 1) % longitudinalPersonalityMax
            self.params.put_nonblocking('LongitudinalPersonality', str(controls.personality))
            personality_events = [EventName.personalityAggressive, EventName.personalityStandard, EventName.personalityRelaxed, EventName.personalityMoreRelaxed]
            controls.events.add(personality_events[controls.personality])
         
        elif button_type == ButtonType.lfaButton:
          self._add_log("Button lkas pressed ..")
          self.params.put_int_nonblocking("MyDrivingMode", self.params.get_int("MyDrivingMode") % 4 + 1) # 1,2,3,4 (1:eco, 2:safe, 3:normal, 4:high speed)
          
    elif button_type != 0 and not controls.enabled:
      self.cruiseActivate = 0

    if CS.vEgo > 1.0:
      self.softHoldActive = 0

    if button_type in [ButtonType.cancel, ButtonType.accelCruise, ButtonType.decelCruise]:
      self.autoCruiseCancelTimer = 0
      if button_type == ButtonType.cancel:
        if self.autoCruiseCancelState > 0:
          controls.lateral_allowed_carrot = False if controls.lateral_allowed_carrot else True
          if controls.lateral_allowed_carrot:
            self._add_log("Button cancel : lateral ON")
          else:
            self._add_log("Button cancel : lateral OFF")
        else:
          self._add_log("Button cancel : Cruise OFF")
        self.autoCruiseCancelState = 1
        controls.events.add(EventName.audioPrompt)
        print("autoCruiseCancelSate = {}".format(self.autoCruiseCancelState))
      else:
        self.autoCruiseCancelState = 0
        print("autoCruiseCancelSate = {}".format(self.autoCruiseCancelState))

    if self.brake_pressed_count > 0 or self.gas_pressed_count > 0:
      if self.cruiseActivate > 0:
        self.cruiseActivate = 0

    #if self.brake_pressed_count > 0 or self.gas_pressed_count > 0 or button_type in [ButtonType.cancel, ButtonType.accelCruise, ButtonType.decelCruise]:
    #  if button_type == ButtonType.cancel:
    #    if self.autoCruiseCancelState > 0:
    #      controls.lateral_allowed_carrot = False if controls.lateral_allowed_carrot else True
    #    #self.autoCruiseCancelState = 0 if self.autoCruiseCancelState > 0 else 1
    #    self.autoCruiseCancelState = 1
    #    controls.events.add(EventName.audioPrompt)
    #    print("autoCruiseCancelSate = {}".format(self.autoCruiseCancelState))
    #    self.autoCruiseCancelTimer = 0
    #  elif button_type != 0:
    #    self.autoCruiseCancelState = 0
    #    self.autoCruiseCancelTimer = 0
    #  if self.cruiseActivate > 0:
    #    self.cruiseActivate = 0

    return v_cruise_kph

  def _update_avm(self, CS):
    v_ego_kph = CS.vEgo * 3.6
    if v_ego_kph < 3.0 and self.v_ego_kph_prev >= 3.0:
      self.activeAVM = 1
    elif v_ego_kph > 5.0 and self.v_ego_kph_prev <= 5.0:
      self.activeAVM = 0
    self.v_ego_kph_prev = v_ego_kph

  def _update_cruise_carrot(self, CS, v_cruise_kph, controls):

    #self.cruiseButtonMode = 2
    self._update_avm(CS)
    if v_cruise_kph > 200:
      self._add_log("VCruise: speed initialize....")
      v_cruise_kph = self.cruiseSpeedMin

    if CS.brakePressed:
      self.brake_pressed_count = max(1, self.brake_pressed_count + 1)
      self.softHold_count = self.softHold_count + 1 if self.softHoldMode > 0 and CS.vEgo < 0.1 else 0
      self.softHoldActive = 1 if self.softHold_count > 60 and controls.CP.openpilotLongitudinalControl else 0      
    else:
      self.softHold_count = 0
      self.brake_pressed_count = min(-1, self.brake_pressed_count - 1)

    if self.softHoldActive > 0 or CS.brakePressed:
      self.brake_pressed_frame = self.frame

    gas_tok = False
    if CS.gasPressed:
      self.gas_pressed_count = max(1, self.gas_pressed_count + 1)
      self.softHoldActive = 0
      self.gas_pressed_value = max(CS.gas, self.gas_pressed_value)
      self.gas_pressed_count_prev = self.gas_pressed_count
      self.gas_pressed_max_aego = max(self.gas_pressed_max_aego, CS.aEgo) if self.gas_pressed_count > 1 else 0
    else:
      gas_tok = True if 0 < self.gas_pressed_count < 0.4 / DT_CTRL else False  ## gas_tok: 0.4 seconds
      self.gas_pressed_count = min(-1, self.gas_pressed_count - 1)
      if self.gas_pressed_count < -1:
        self.gas_pressed_max = 0
        self.gas_pressed_count_prev = 0

    if controls.enabled or CS.brakePressed or CS.gasPressed:
      self.cruiseActiveReady = 0
      if CS.gasPressed and self.accel_output < -0.5:
        self.autoCruiseCancelTimer = 5.0 / DT_CTRL #잠시 오토크루멈춤
        self.cruiseActivate = -1
        self._add_log("Cruise off (GasPressed while braking)")

    v_cruise_kph = self._update_cruise_button(CS, v_cruise_kph, controls)

    ## Auto Engage/Disengage via Gas/Brake
    if gas_tok:
      if (self.autoCruiseCancelTimer == 0 or (self.frame - self.gas_tok_frame) < 1.0 / DT_CTRL):  ## 1초이내 더블 엑셀톡인경우..
        self.autoCruiseCancelTimer = 0
        if controls.enabled:
          if (self.frame - self.brake_pressed_frame) < 3.0 / DT_CTRL:
            v_cruise_kph = self.v_ego_kph_set
            self._add_log("Gas tok speed set to current (prev. brake pressed)")
          else:
            v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
            self._add_log("Gas tok speed up...{:.0f}".format(v_cruise_kph))
        elif self.autoResumeFromGasSpeed > 0:
          self._add_log_auto_cruise("Cruise Activate from GasTok")
          #v_cruise_kph = self.v_ego_kph_set
          self.cruiseActivate = 1
      self.gas_tok_frame = self.frame
    elif self.gas_pressed_count == -1:
      v_cruise_kph = self._gas_released_cond(CS, v_cruise_kph, controls)
      if self.autoCruiseCancelTimer > 0 and self.cruiseActivate > 0:
        self.cruiseActivate = 0
        self.cruiseActiveReady = 1

    elif self.brake_pressed_count == -1:
      if self.softHoldActive == 1 and self.softHoldMode > 0:
        self._add_log_auto_cruise("Cruise Activete from SoftHold")
        self.softHoldActive = 2
        self.cruiseActivate = 1
        self.autoCruiseCancelTimer = 0
      else:
        v_cruise_kph =  self._brake_released_cond(CS, v_cruise_kph, controls)
        if self.autoCruiseCancelTimer > 0 and self.cruiseActivate > 0:
          self.cruiseActivate = 0

    elif self.gas_pressed_count > 0 and self.v_ego_kph_set > v_cruise_kph:
      v_cruise_kph = self.v_ego_kph_set
      if V_CRUISE_MAX > v_cruise_kph > self.cruiseSpeedMax:
        self.cruiseSpeedMax = v_cruise_kph
    elif self.cruiseActiveReady > 0 and self.autoCruiseCancelTimer == 0:
      if 0 < self.lead_dRel or self.xState == 3:
        self._add_log_auto_cruise("Cruise Activate from Lead or Traffic sign stop")
        self.cruiseActivate = 1
    elif not controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0 and self.autoCruiseCancelTimer == 0:
      cruiseOnDist = abs(self.cruiseOnDist)
      if self.autoCruiseControl >= 2 and self.lead_vRel < 0 and 0 < self.lead_dRel < CS.vEgo ** 2 / (2.0 * 2):
        self._add_log_auto_cruise("Auto Cruise Activate")
        self.cruiseActivate = 1
      elif cruiseOnDist > 0 and CS.vEgo > 0.02 and  0 < self.lead_dRel < cruiseOnDist:
        self._make_event(controls, EventName.stopStop)
        self._add_log_auto_cruise("CruiseOnDist Activate")
        self.cruiseActivate = 1

    if controls.enabled and self.autoSpeedUptoRoadSpeedLimit > 0.:
      lead_v_kph = self.lead_vLead * CV.MS_TO_KPH + 10.0
      lead_v_kph = min(lead_v_kph, self.v_ego_kph_set + 10)
      road_speed = (30 if self.roadSpeed < 30 else self.roadSpeed) * self.autoSpeedUptoRoadSpeedLimit
      lead_v_kph = max(v_cruise_kph, min(lead_v_kph, road_speed))
      if lead_v_kph > v_cruise_kph and self.lead_dRel < 80:
        #self._add_log_auto_cruise("AutoSpeed up to leadCar={:.0f}kph, road_speed={:.0f}kph".format(lead_v_kph, road_speed))
        v_cruise_kph = lead_v_kph

    v_cruise_kph = self.update_apilot_cmd(controls, v_cruise_kph)

    if self.CP.pcmCruise:
      if self.v_ego_kph_set >= 10 or 0 < self.lead_dRel < 140:
        pass
      else:
        self.cruiseActivate = 0

    if self.autoCruiseControl < 1 or self.autoCruiseCancelState > 0 or not controls.enable_avail or CS.brakeHoldActive:
      if self.cruiseActivate != 0:
        self._add_log_auto_cruise(f"Cancel auto Cruise = {self.cruiseActivate}")
      self.cruiseActivate = 0
      self.softHoldActive = 0
    v_cruise_kph = clip(v_cruise_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)
    return v_cruise_kph

  def v_cruise_speed_up(self, v_cruise_kph):
    roadSpeed = 30 if self.roadSpeed < 30 else self.roadSpeed
    if v_cruise_kph < roadSpeed:
      v_cruise_kph = roadSpeed
    else:
      for speed in range (40, int(self.cruiseSpeedMax), self.cruiseSpeedUnit):
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
    return clip(v_cruise_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)

  def cruise_control_speed(self, v_cruise_kph):
    v_cruise_kph_apply = v_cruise_kph        
    if self.cruiseEcoControl > 0:
      if self.cruiseSpeedTarget > 0:
        if self.cruiseSpeedTarget < v_cruise_kph:
          self.cruiseSpeedTarget = v_cruise_kph
        elif self.cruiseSpeedTarget > v_cruise_kph:
          self.cruiseSpeedTarget = 0
      elif self.cruiseSpeedTarget == 0 and self.v_ego_kph_set + 3 < v_cruise_kph and v_cruise_kph > 20.0:  # 주행중 속도가 떨어지면 다시 크루즈연비제어 시작.
        self.cruiseSpeedTarget = v_cruise_kph

      if self.cruiseSpeedTarget != 0:  ## 크루즈 연비 제어모드 작동중일때: 연비제어 종료지점
        if self.v_ego_kph_set > self.cruiseSpeedTarget: # 설정속도를 초과하면..
          self.cruiseSpeedTarget = 0
        else:
          v_cruise_kph_apply = self.cruiseSpeedTarget + self.cruiseEcoControl  # + 설정 속도로 설정함.
    else:
      self.cruiseSpeedTarget = 0

    return v_cruise_kph_apply

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


def apply_center_deadzone(error, deadzone):
  if (error > - deadzone) and (error < deadzone):
    error = 0.
  return error


def rate_limit(new_value, last_value, dw_step, up_step):
  return clip(new_value, last_value + dw_step, last_value + up_step)


def get_lag_adjusted_curvature(CP, v_ego, psis, curvatures):
  if len(psis) != CONTROL_N:
    psis = [0.0]*CONTROL_N
    curvatures = [0.0]*CONTROL_N
  v_ego = max(MIN_SPEED, v_ego)

  # TODO this needs more thought, use .2s extra for now to estimate other delays
  #delay = CP.steerActuatorDelay + .2
  delay = max(0.01, float(Params().get_int("SteerActuatorDelay")) * 0.01)

  # MPC can plan to turn the wheel and turn back before t_delay. This means
  # in high delay cases some corrections never even get commanded. So just use
  # psi to calculate a simple linearization of desired curvature
  current_curvature_desired = curvatures[0]
  psi = interp(delay, ModelConstants.T_IDXS[:CONTROL_N], psis)
  average_curvature_desired = psi / (v_ego * delay)
  desired_curvature = 2 * average_curvature_desired - current_curvature_desired

  # This is the "desired rate of the setpoint" not an actual desired rate
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = clip(desired_curvature,
                                current_curvature_desired - max_curvature_rate * DT_MDL,
                                current_curvature_desired + max_curvature_rate * DT_MDL)
  return safe_desired_curvature

def clip_curvature(v_ego, prev_curvature, new_curvature):
  v_ego = max(MIN_SPEED, v_ego)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego**2) # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  safe_desired_curvature = clip(new_curvature,
                                prev_curvature - max_curvature_rate * DT_CTRL,
                                prev_curvature + max_curvature_rate * DT_CTRL)

  return safe_desired_curvature


def get_friction(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float,
                 torque_params: car.CarParams.LateralTorqueTuning, friction_compensation: bool) -> float:
  friction_interp = interp(
    apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
    [-friction_threshold, friction_threshold],
    [-torque_params.friction, torque_params.friction]
  )
  friction = float(friction_interp) if friction_compensation else 0.0
  return friction


def get_speed_error(modelV2: log.ModelDataV2, v_ego: float) -> float:
  # ToDo: Try relative error, and absolute speed
  if len(modelV2.temporalPose.trans):
    vel_err = clip(modelV2.temporalPose.trans[0] - v_ego, -MAX_VEL_ERR, MAX_VEL_ERR)
    return float(vel_err)
  return 0.0
