import math

from cereal import car, log
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.realtime import DT_MDL, DT_CTRL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.params import Params
import numpy as np
from common.filter_simple import StreamingMovingAverage
from openpilot.selfdrive.carrot.carrot_functions import CarrotSpeedController

EventName = car.CarEvent.EventName

## 국가법령정보센터: 도로설계기준
V_CURVE_LOOKUP_BP = [0., 1./800., 1./670., 1./560., 1./440., 1./360., 1./265., 1./190., 1./135., 1./85., 1./55., 1./30., 1./15.]
#V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20]
V_CRUVE_LOOKUP_VALS = [300, 150, 120, 110, 100, 90, 80, 70, 60, 50, 45, 35, 30]
MIN_CURVE_SPEED = 20. * CV.KPH_TO_MS


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
    self.gas_pressed_count = 0
    self.gas_pressed_count_prev = 0
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
    self.turnSpeed_prev = 300
    self.curvatureFilter = StreamingMovingAverage(20)
    self.softHold_count = 0
    self.cruiseActiveReady = 0
    self.autoCruiseCancelState = 0  # 0: normal, 1:cancel, 2: timer cancel
    self.xIndex = 0
    self.frame = 0
    self._log_timer = 0
    self.debugText = ""
    self.debugTextNoo = ""
    self.debugText2 = ""
    self.activeAPM = 0
    self.blinkerExtMode = 0 # 0: Normal, 10000: voice
    self.rightBlinkerExtCount = 0
    self.leftBlinkerExtCount = 0
    self.naviDistance = 0
    self.naviSpeed = 0
    self.nav_distance = 0  # for navInstruction
    self.distance_traveled = 0.0
    self.nooHelperActivated = 0
    self.nooHelperActivateCount = 0
    self.curveSpeed = 0.0 # turnSpeed with sign
    self.limitSpeed = 0.0
    self.cruiseSpeedMax = V_CRUISE_MAX
    self.autoCruiseCancelTimer = 0
    self.sendEvent = None

    self.carrot = CarrotSpeedController()

    #ajouatom: params
    self.params_count = 0
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = self.params.get_int("AutoNaviSpeedCtrl")
    self.autoResumeFromGasSpeed = self.params.get_int("AutoResumeFromGasSpeed")
    self.autoCancelFromGasMode = self.params.get_int("AutoCancelFromGasMode")
    self.autoResumeFromBrakeReleaseTrafficSign = self.params.get_int("AutoResumeFromBrakeReleaseTrafficSign")
    self.autoCruiseControl = self.params.get_int("AutoCruiseControl")
    self.cruiseButtonMode = self.params.get_int("CruiseButtonMode")
    self.autoSpeedUptoRoadSpeedLimit = float(self.params.get_int("AutoSpeedUptoRoadSpeedLimit")) * 0.01
    self.autoCurveSpeedCtrlUse = int(self.params.get("AutoCurveSpeedCtrlUse"))
    self.autoCurveSpeedFactor = float(int(self.params.get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
    self.autoCurveSpeedFactorIn = float(int(self.params.get("AutoCurveSpeedFactorIn", encoding="utf8")))*0.01
    self.cruiseOnDist = float(int(self.params.get("CruiseOnDist", encoding="utf8"))) / 100.
    self.softHoldMode = self.params.get_int("SoftHoldMode")
    self.cruiseSpeedMin = self.params.get_int("CruiseSpeedMin")
    self.autoTurnControl = self.params.get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
    self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
    self.autoTurnControlSpeedLaneChange = self.params.get_int("AutoTurnControlSpeedLaneChange")
    self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    self.showDebugUI= self.params.get_int("ShowDebugUI")
    self.enableOSM = self.params.get_int("EnableOSM")
    self.speedFromPCM = self.params.get_int("SpeedFromPCM")

  def _params_update(self):
    self.frame += 1
    self.params_count += 1
    if self.params_count == 10:
      self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
      self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
      self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
      self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
      self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
      self.autoNaviSpeedCtrl = self.params.get_int("AutoNaviSpeedCtrl")
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
      self.autoTurnControl = self.params.get_int("AutoTurnControl")
      self.autoTurnControlTurnEnd = self.params.get_int("AutoTurnControlTurnEnd")
      self.autoTurnMapChange = self.params.get_int("AutoTurnMapChange")
      self.autoTurnControlSpeedLaneChange = self.params.get_int("AutoTurnControlSpeedLaneChange")
      self.autoTurnControlSpeedTurn = self.params.get_int("AutoTurnControlSpeedTurn")
    elif self.params_count >= 100:
      self.autoCurveSpeedCtrlUse = self.params.get_int("AutoCurveSpeedCtrlUse")
      self.autoCurveSpeedFactor = float(self.params.get_int("AutoCurveSpeedFactor"))*0.01
      self.autoCurveSpeedFactorIn = float(self.params.get_int("AutoCurveSpeedFactorIn"))*0.01
      self.showDebugUI = self.params.get_int("ShowDebugUI")
      self.enableOSM = self.params.get_int("EnableOSM")
      self.speedFromPCM = self.params.get_int("SpeedFromPCM")
      self.params_count = 0
    
  @property
  def v_cruise_initialized(self):
    return self.v_cruise_kph != V_CRUISE_UNSET

  def update_v_cruise(self, CS, enabled, is_metric, controls):
    self.v_cruise_kph_last = self.v_cruise_kph

    self._params_update()
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
        #
        #self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        #self.v_cruise_cluster_kph = self.v_cruise_kph_set = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if self.params.get_int("SpeedFromPCM") > 0:
          self.v_cruise_kph_set = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        self._update_v_cruise_apilot(CS, controls)
        self.v_cruise_cluster_kph = self.v_cruise_kph
        self._update_event_apilot(CS, controls)
    else:
      self.v_cruise_kph = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.v_cruise_cluster_kph = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.v_cruise_kph_set = V_CRUISE_INITIAL#V_CRUISE_UNSET
      self.cruiseActivate = 0
      v_cruise_kph = self.update_apilot_cmd(controls, 30)

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
    else:
      self.v_cruise_kph = self.v_cruise_kph_set = int(round(clip(CS.vEgo * CV.MS_TO_KPH, initial, self.cruiseSpeedMax)))

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
    if leadOne.status and leadOne.radar:
      self.lead_dRel = leadOne.dRel
      self.lead_vRel = leadOne.vRel
    else:
      self.lead_dRel = 0
      self.lead_vRel = 0

  def _update_v_cruise_apilot(self, CS, controls):

    self.rightBlinkerExtCount = max(self.rightBlinkerExtCount - 1, 0)
    self.leftBlinkerExtCount = max(self.leftBlinkerExtCount - 1, 0)
    if self.rightBlinkerExtCount + self.leftBlinkerExtCount <= 0:
      self.blinkerExtMode = 0

    ## autoCruise가 핸들을 60도이상 돌리면.. 40초간 일시정지된다.
    if abs(CS.steeringAngleDeg) > 60:
      if self.autoCruiseCancelTimer == 0:
        self._add_log_auto_cruise("autoCruise paused for 40 seconds.")
        controls.events.add(EventName.audioPrompt)
      self.autoCruiseCancelTimer = int(40. / DT_CTRL)

    if self.autoCruiseCancelTimer > 0:
      if self.autoCruiseCancelTimer % int(1 / DT_CTRL) == 0:
        self._add_log_auto_cruise("autoCruise paused for {:.0f}seconds.".format(self.autoCruiseCancelTimer*DT_CTRL))
    if self.autoCruiseCancelTimer == 1:
      controls.events.add(EventName.audioPrompt)
      self._add_log_auto_cruise("autoCruise activated.")

    self.autoCruiseCancelTimer = max(self.autoCruiseCancelTimer - 1, 0)

    self._update_lead(controls)
    self.v_ego_kph_set = int(CS.vEgoCluster * CV.MS_TO_KPH + 0.5)
    if self.v_cruise_kph_set > 200:
      self.v_cruise_kph_set = self.cruiseSpeedMin
    v_cruise_kph = self.v_cruise_kph_set    
    v_cruise_kph = self._update_cruise_carrot(CS, v_cruise_kph, controls)
    v_cruise_kph_apply = self.cruise_control_speed(v_cruise_kph)

    if False:
      carrot_cruise_kph = self.carrot.update(CS, controls, self.v_cruise_kph)
      v_cruise_kph_apply = min(v_cruise_kph_apply, carrot_cruise_kph)
    else:
      self.auto_navi_control(CS, controls)
      apn_limit_kph = self.update_speed_apilot(CS, controls, self.v_cruise_kph)
      osm_limit_kph = self.update_osm_apilot(CS, controls, self.v_cruise_kph)
      v_cruise_kph_apply = min(v_cruise_kph_apply, apn_limit_kph)

    self.turnSpeed = self.apilot_curve(CS, controls)
    if self.autoCurveSpeedCtrlUse > 0:
      v_cruise_kph_apply = min(v_cruise_kph_apply, self.turnSpeed)
    self.v_cruise_kph_set = v_cruise_kph
    self.v_cruise_kph = v_cruise_kph_apply

  def apilot_curve(self, CS, controls):
    if len(controls.sm['modelV2'].orientationRate.z) != 33:
      return 300
    # 회전속도를 선속도 나누면 : 곡률이 됨. [20]은 약 4초앞의 곡률을 보고 커브를 계산함.
    #curvature = abs(controls.sm['modelV2'].orientationRate.z[20] / clip(CS.vEgo, 0.1, 100.0))
    orientationRates = np.array(controls.sm['modelV2'].orientationRate.z, dtype=np.float32)
    # 계산된 결과로, oritetationRates를 나누어 조금더 curvature값이 커지도록 함.
    speed = min(self.turnSpeed_prev / 3.6, clip(CS.vEgo, 0.5, 100.0))    
    #curvature = np.max(np.abs(orientationRates[12:])) / speed  # 12: 약1.4초 미래의 curvature를 계산함.
    #curvature = np.max(np.abs(orientationRates[12:20])) / speed  # 12: 약1.4~3.5초 미래의 curvature를 계산함.
    curvature = np.max(np.abs(orientationRates[12:28])) / speed  
    curvature = self.curvatureFilter.process(curvature) * self.autoCurveSpeedFactor
    turnSpeed = 300
    if abs(curvature) > 0.0001:
      turnSpeed = interp(abs(curvature), V_CURVE_LOOKUP_BP, V_CRUVE_LOOKUP_VALS)
      turnSpeed = clip(turnSpeed, MIN_CURVE_SPEED, 255)
    else:
      turnSpeed = 300

    self.curveSpeed = turnSpeed * np.sign(curvature)

    #print("curvature={:.5f}, speed = {:.1f}".format(curvature, turnSpeed))
    self.turnSpeed_prev = turnSpeed
    speed_diff = max(0, CS.vEgo*3.6 - turnSpeed)
    turnSpeed = turnSpeed - speed_diff * self.autoCurveSpeedFactorIn
    #controls.debugText2 = 'CURVE={:5.1f},curvature={:5.4f},mode={:3.1f}'.format(self.turnSpeed_prev, curvature, self.drivingModeIndex)
    return turnSpeed

  def update_apilot_cmd(self, controls, v_cruise_kph):
    msg = controls.sm['roadLimitSpeed']

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
        blinkerExtState = self.rightBlinkerExtCount + self.rightBlinkerExtCount
        if msg.xArg == "RIGHT":
          self.rightBlinkerExtCount = 50
        elif msg.xArg == "LEFT":
          self.leftBlinkerExtCount = 50
        if blinkerExtState <= 0 and self.rightBlinkerExtCount + self.rightBlinkerExtCount > 0:
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
    return v_cruise_kph

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
      self._add_log_auto_cruise("Cruise Deactivate from gas pressed: traffic stopping");
      self.cruiseActivate = -1
    elif self.v_ego_kph_set > self.autoResumeFromGasSpeed > 0:
      if self.cruiseActivate <= 0:
        if self.gas_pressed_value > 0.6 or self.gas_pressed_count_prev > 3.0 / DT_CTRL:
          if self.autoCruiseCancelTimer > 0:
            v_cruise_kph = self.v_ego_kph_set
            self.autoCruiseCancelTimer = 0
          self._add_log_auto_cruise("Cruise Activate from gas(deep/long pressed)")          
        else:
          v_cruise_kph = self.v_ego_kph_set
          self._add_log_auto_cruise("Cruise Activate from gas(speed)")
      self.cruiseActivate = 1
    return v_cruise_kph

  def _brake_released_cond(self, CS, v_cruise_kph, controls):
    if self.autoResumeFromGasSpeed < self.v_ego_kph_set and self.autoResumeFromBrakeReleaseTrafficSign:
      v_cruise_kph = self.v_ego_kph_set
      self._add_log_auto_cruise("Cruise Activate Brake Release")
      self.cruiseActivate = 1
    elif self.xState == 3 and self.autoResumeFromBrakeReleaseTrafficSign:
      #v_cruise_kph = self.v_ego_kph_set
      self._add_log_auto_cruise("Cruise Activate from Traffic sign stop")
      self.cruiseActivate = 1
    elif 0 < self.lead_dRel < 20:
      self._add_log_auto_cruise("Cruise Activate from Lead Car")
      self.cruiseActivate = 1
    return v_cruise_kph

  def _update_cruise_button(self, CS, v_cruise_kph, controls):
    ## ButtonEvent process
    button_kph = v_cruise_kph
    buttonEvents = CS.buttonEvents
    button_speed_up_diff = 1
    button_speed_dn_diff = 10 if self.cruiseButtonMode in [3, 4] else 1

    button_type = 0
    if self.button_cnt > 0:
      self.button_cnt += 1
    for b in buttonEvents:
      if b.pressed and self.button_cnt==0 and b.type in [ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.gapAdjustCruise, ButtonType.cancel]:
        self.button_cnt = 1
        self.button_prev = b.type
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

        self.long_pressed = False
        self.button_cnt = 0
    if self.button_cnt > 40:
      self.long_pressed = True
      V_CRUISE_DELTA = 10
      if self.button_prev == ButtonType.cancel:
        button_type = ButtonType.cancel
        self.button_cnt = 0          
      elif self.button_prev == ButtonType.accelCruise:
        button_kph += V_CRUISE_DELTA - button_kph % V_CRUISE_DELTA
        button_type = ButtonType.accelCruise
        self.button_cnt %= 40
      elif self.button_prev == ButtonType.decelCruise:
        button_kph -= V_CRUISE_DELTA - -button_kph % V_CRUISE_DELTA
        button_type = ButtonType.decelCruise
        self.button_cnt %= 40
      elif self.button_prev == ButtonType.gapAdjustCruise:
        button_type = ButtonType.gapAdjustCruise
        self.button_cnt = 0

    button_kph = clip(button_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)

    if button_type != 0 and controls.enabled:
      if self.long_pressed:
        if button_type in [ButtonType.accelCruise, ButtonType.decelCruise]:
          v_cruise_kph = button_kph
          self._add_log("Button long pressed..{:.0f}".format(v_cruise_kph))
        elif button_type == ButtonType.gapAdjustCruise:
          self._add_log("Button gap pressed ..")
      else:
        if button_type == ButtonType.accelCruise:
          if self.softHoldActive > 0 and self.autoCruiseControl > 0:
            self.softHoldActive = 0
            self._add_log("Button softhold released ..")
          else:
            if self.cruiseButtonMode == 0:
              v_cruise_kph = button_kph
            elif self.cruiseButtonMode in [1,2]:
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
    elif button_type != 0 and not controls.enabled:
      self.cruiseActivate = 0

    if CS.vEgo > 1.0:
      self.softHoldActive = 0
    if self.brake_pressed_count > 0 or self.gas_pressed_count > 0 or button_type in [ButtonType.cancel, ButtonType.accelCruise, ButtonType.decelCruise]:
      if button_type == ButtonType.cancel:
        self.autoCruiseCancelState = 0 if self.autoCruiseCancelState > 0 else 1
        controls.events.add(EventName.audioPrompt)
        print("autoCruiseCancelSate = {}".format(self.autoCruiseCancelState))
        self.autoCruiseCancelTimer = 0
      elif button_type != 0:
        self.autoCruiseCancelState = 0
        self.autoCruiseCancelTimer = 0
      if self.cruiseActivate > 0:
        self.cruiseActivate = 0

    return v_cruise_kph

  def _update_cruise_carrot(self, CS, v_cruise_kph, controls):

    #self.cruiseButtonMode = 2    

    if v_cruise_kph > 200:
      self._add_log("VCruise: speed initialize....")
      v_cruise_kph = self.cruiseSpeedMin

    if CS.brakePressed:
      self.brake_pressed_count = 1 if self.brake_pressed_count < 0 else self.brake_pressed_count + 1
      self.softHold_count = self.softHold_count + 1 if self.softHoldMode > 0 and CS.vEgo < 0.1 else 0
      self.softHoldActive = 1 if self.softHold_count > 60 else 0
    else:
      self.softHold_count = 0
      self.brake_pressed_count = -1 if self.brake_pressed_count > 0 else self.brake_pressed_count - 1

    gas_tok = False
    if CS.gasPressed:
      self.gas_pressed_count = 1 if self.gas_pressed_count < 0 else self.gas_pressed_count + 1
      self.softHoldActive = 0
      if CS.gas > self.gas_pressed_value:
        self.gas_pressed_value = CS.gas
      self.gas_pressed_count_prev = self.gas_pressed_count
    else:
      gas_tok = True if 0 < self.gas_pressed_count < 60 else False
      self.gas_pressed_count = -1 if self.gas_pressed_count > 0 else self.gas_pressed_count - 1
      if self.gas_pressed_count < -1:
        self.gas_pressed_max = 0
        self.gas_pressed_count_prev = 0

    if controls.enabled or CS.brakePressed or CS.gasPressed:
      self.cruiseActiveReady = 0

    v_cruise_kph = self._update_cruise_button(CS, v_cruise_kph, controls)


    ## Auto Engage/Disengage via Gas/Brake
    if gas_tok and self.autoCruiseCancelTimer == 0:      
      if controls.enabled:
        v_cruise_kph = self.v_cruise_speed_up(v_cruise_kph)
      elif self.autoResumeFromGasSpeed > 0:
        self._add_log_auto_cruise("Cruise Activate from GasTok")
        #v_cruise_kph = self.v_ego_kph_set
        self.cruiseActivate = 1
    elif self.gas_pressed_count == -1:
      v_cruise_kph = self._gas_released_cond(CS, v_cruise_kph, controls)
      if self.autoCruiseCancelTimer > 0 and self.cruiseActivate > 0:
        self.cruiseActivate = 0
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
    elif self.cruiseActiveReady > 0:
      if 0 < self.lead_dRel or self.xState == 3:
        self._add_log_auto_cruise("Cruise Activate from Lead or Traffic sign stop")
        self.cruiseActivate = 1
    elif not controls.enabled and self.brake_pressed_count < 0 and self.gas_pressed_count < 0 and self.autoCruiseCancelTimer == 0:
      cruiseOnDist = abs(self.cruiseOnDist)
      if self.autoCruiseControl >= 2 and self.lead_vRel < 0 and 0 < self.lead_dRel < CS.vEgo ** 2 / (2.5 * 2):
        self._add_log_auto_cruise("Cruise Activated")
        self.cruiseActivate = 1
      if cruiseOnDist > 0 and CS.vEgo > 0.2 and  0 < self.lead_dRel < cruiseOnDist:
        self._make_event(controls, EventName.stopStop)
        if cruiseOnDist > 0:
          self._add_log_auto_cruise("CruiseOnDist Activate")
          self.cruiseActivate = 1
    elif controls.enabled and self.autoSpeedUptoRoadSpeedLimit > 0.:
      if self.lead_vRel > 0.5:
        lead_v_kph = (self.lead_vRel + CS.vEgoCluster) * CV.MS_TO_KPH
        v_cruise_kph = max(v_cruise_kph, min(lead_v_kph, (30 if self.roadSpeed < 30 else self.roadSpeed) * self.autoSpeedUptoRoadSpeedLimit))

    v_cruise_kph = self.update_apilot_cmd(controls, v_cruise_kph)

    if self.CP.pcmCruise:
      if self.v_ego_kph_set >= 10 or 0 < self.lead_dRel < 140:
        pass
      else:
        self.cruiseActivate = 0

    if self.autoCruiseControl < 1 or self.autoCruiseCancelState > 0 or not controls.can_enable or CS.brakeHoldActive:
      if self.cruiseActivate != 0:
        self._add_log_auto_cruise("Cancel auto Cruise = {self.cruiseActivate}")
      self.cruiseActivate = 0
      self.softHoldActive = 0
    v_cruise_kph = clip(v_cruise_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)
    return v_cruise_kph

  def v_cruise_speed_up(self, v_cruise_kph):
    roadSpeed = 30 if self.roadSpeed < 30 else self.roadSpeed
    if v_cruise_kph < roadSpeed:
      v_cruise_kph = roadSpeed
    else:
      for speed in range (40, int(self.cruiseSpeedMax), Params().get_int("CruiseSpeedUnit")):
        if v_cruise_kph < speed:
          v_cruise_kph = speed
          break
    return clip(v_cruise_kph, self.cruiseSpeedMin, self.cruiseSpeedMax)

  def decelerate_for_speed_camera(self, safe_speed, safe_dist, prev_apply_speed, decel_rate, left_dist):
    if left_dist <= safe_dist:
      return safe_speed
    temp = safe_speed*safe_speed + 2*(left_dist - safe_dist)/decel_rate
    dV = (-safe_speed + math.sqrt(temp)) * decel_rate
    apply_speed = min(250 , safe_speed + dV)
    min_speed = prev_apply_speed - (decel_rate * 1.2) * 2 * DT_CTRL
    apply_speed = max(apply_speed, min_speed)
    return apply_speed

  def update_speed_apilot(self, CS, controls, v_cruise_kph_prev):
    v_ego = CS.vEgoCluster
    msg = self.roadLimitSpeed = controls.sm['roadLimitSpeed']
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
      self.activeAPM += 1000

    if isSpeedBump:
      speedLimitType = 1 
      safeDist = self.autoNaviSpeedBumpTime * v_ego
    elif safeSpeed>0 and leftDist>0:
      safeDist = self.autoNaviSpeedCtrlEnd * v_ego

    safeSpeed *= self.autoNaviSpeedSafetyFactor

    log = ""
    if leftDist > 0 and safeSpeed > 0 and safeDist > 0:
      applySpeed = self.decelerate_for_speed_camera(safeSpeed/3.6, safeDist, v_cruise_kph_prev * CV.KPH_TO_MS, self.autoNaviSpeedDecelRate, leftDist) * CV.MS_TO_KPH
      if isSectionLimit and applySpeed > safeSpeed:
        applySpeed = safeSpeed
    else:
      applySpeed = 255

    apTbtDistance = self.naviDistance
    apTbtSpeed = self.naviSpeed
    if apTbtSpeed > 0 and apTbtDistance > 0:
      safeTbtDist = self.autoTurnControlTurnEnd * v_ego
      applyTbtSpeed = self.decelerate_for_speed_camera(apTbtSpeed/3.6, safeTbtDist, v_cruise_kph_prev/3.6, self.autoNaviSpeedDecelRate, apTbtDistance) * 3.6
      if applyTbtSpeed < applySpeed:
        applySpeed = applyTbtSpeed
        safeSpeed = apTbtSpeed
        leftDist = apTbtDistance
        safeDist = safeTbtDist
        speedLimitType = 4

    log = "{},{:.1f}<{:.1f}/{:.1f},{:.1f} B{} A{:.1f}/{:.1f} N{:.1f}/{:.1f} C{:.1f}/{:.1f} V{:.1f}/{:.1f} ".format(
                  msg.roadcate, applySpeed, safeSpeed, leftDist, safeDist,
                  1 if isSpeedBump else 0, 
                  msg.xSpdLimit, msg.xSpdDist,
                  msg.camLimitSpeed, msg.camLimitSpeedLeftDist,
                  CS.speedLimit, CS.speedLimitDistance,
                  apTbtSpeed, apTbtDistance)
    #if applySpeed < 200:
    #  print(log)
    #controls.debugText1 = log
    self.debugText2 = log
    return applySpeed #, roadSpeed, leftDist, speedLimitType

  def cruise_control_speed(self, v_cruise_kph):
    v_cruise_kph_apply = v_cruise_kph    
    cruise_eco_control = self.params.get_int("CruiseEcoControl")
    if cruise_eco_control > 0:
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
          v_cruise_kph_apply = self.cruiseSpeedTarget + cruise_eco_control  # + 설정 속도로 설정함.
    else:
      self.cruiseSpeedTarget = 0

    return v_cruise_kph_apply

  def update_osm_apilot(self, CS, controls, v_cruise_kph):
    if controls.sm.updated['liveMapData']:
      osm = controls.sm['liveMapData']
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

  def auto_navi_control(self, CS, controls):
    v_ego = CS.vEgoCluster
    delta_dist = controls.distance_traveled - self.distance_traveled
    self.distance_traveled = controls.distance_traveled
    self.nav_distance = max(0, self.nav_distance - delta_dist)
    #navInstruction = controls.sm['navInstruction']
    #if navInstruction.speedLimit > 0:
    #  self.limitSpeed = navInstruction.speedLimit * CV.MS_TO_KPH
    #if self.autoNaviSpeedCtrl >= 2:
    #  road_speed = max(self.limitSpeed, self.roadSpeed)
    #  self.cruiseSpeedMax = road_speed * self.autoNaviSpeedSafetyFactor

    if self.autoTurnControl > 0:
      navInstruction = controls.sm['navInstruction']      
      roadLimitSpeed = controls.sm['roadLimitSpeed']
      md = controls.sm['modelV2']
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
        if controls.sm.updated['roadLimitSpeed']:
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

        if blinkerExtState <= 0 and self.leftBlinkerExtCount + self.rightBlinkerExtCount > 0 and v_ego > 0.5:
          self._make_event(controls, EventName.audioTurn if nav_turn else EventName.audioLaneChange)


    else:
      self.naviDistance = 0
      self.naviSpeed = 0

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
