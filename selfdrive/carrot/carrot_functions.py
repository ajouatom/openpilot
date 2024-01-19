from openpilot.common.params import Params

class CarrotDrive:
  def __init__(self):
    self.params = Params()

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
    self.roadSpeed = 30
    self.xState = 0
    self.trafficState = 0
    self.sendEvent_frame = 0
    self.turnSpeed_prev = 300
    self.curvatureFilter = StreamingMovingAverage(20)
    self.softHold_count = 0
    self.cruiseActiveReady = 0
    self.autoCruiseCancelState = False
    self.xIndex = 0
    self.frame = 0
    self._log_timer = 0
    self.debugText = ""
    self.debugTextNoo = ""
    self.debugText2 = ""
    self._first = True
    self.activeAPM = 0
    self.blinkerExtMode = 0 # 0: Normal, 10000: voice
    self.rightBlinkerExtCount = 0
    self.leftBlinkerExtCount = 0
    self.naviDistance = 0
    self.naviSpeed = 0
    self.nav_distance = 0  # for navInstruction
    self.distance_traveled = 0.0
    
    #ajouatom: params
    self.params_count = 0
    self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
    self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
    self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
    self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
    self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
    self.autoNaviSpeedCtrl = 2
    self.autoResumeFromGasSpeed = Params().get_int("AutoResumeFromGasSpeed")
    self.autoCancelFromGasMode = Params().get_int("AutoCancelFromGasMode")
    self.autoResumeFromBrakeReleaseTrafficSign = Params().get_int("AutoResumeFromBrakeReleaseTrafficSign")
    self.autoCruiseControl = Params().get_int("AutoCruiseControl")
    self.cruiseButtonMode = Params().get_int("CruiseButtonMode")
    self.autoSpeedUptoRoadSpeedLimit = float(self.params.get_int("AutoSpeedUptoRoadSpeedLimit")) * 0.01
    self.autoCurveSpeedCtrlUse = int(Params().get("AutoCurveSpeedCtrlUse"))
    self.autoCurveSpeedFactor = float(int(Params().get("AutoCurveSpeedFactor", encoding="utf8")))*0.01
    self.autoCurveSpeedFactorIn = float(int(Params().get("AutoCurveSpeedFactorIn", encoding="utf8")))*0.01
    self.cruiseOnDist = float(int(Params().get("CruiseOnDist", encoding="utf8"))) / 100.
    self.softHoldMode = Params().get_int("SoftHoldMode")
    self.cruiseSpeedMin = Params().get_int("CruiseSpeedMin")
    self.autoTurnControl = Params().get_int("AutoTurnControl")
    self.autoTurnControlTurnEnd = Params().get_int("AutoTurnControlTurnEnd")
    self.autoTurnControlSpeedLaneChange = Params().get_int("AutoTurnControlSpeedLaneChange")
    self.autoTurnControlSpeedTurn = Params().get_int("AutoTurnControlSpeedTurn")

  def _params_update(self):
    self.frame += 1
    self.params_count += 1
    if self.params_count == 10:
      self.autoNaviSpeedBumpSpeed = float(self.params.get_int("AutoNaviSpeedBumpSpeed"))
      self.autoNaviSpeedBumpTime = float(self.params.get_int("AutoNaviSpeedBumpTime"))
      self.autoNaviSpeedCtrlEnd = float(self.params.get_int("AutoNaviSpeedCtrlEnd"))
      self.autoNaviSpeedSafetyFactor = float(self.params.get_int("AutoNaviSpeedSafetyFactor")) * 0.01
      self.autoNaviSpeedDecelRate = float(self.params.get_int("AutoNaviSpeedDecelRate")) * 0.01
      self.autoNaviSpeedCtrl = 2
    elif self.params_count == 20:
      self.autoResumeFromGasSpeed = Params().get_int("AutoResumeFromGasSpeed")
      self.autoCancelFromGasMode = Params().get_int("AutoCancelFromGasMode")
      self.autoResumeFromBrakeReleaseTrafficSign = Params().get_int("AutoResumeFromBrakeReleaseTrafficSign")
      self.autoCruiseControl = Params().get_int("AutoCruiseControl")
      self.cruiseButtonMode = Params().get_int("CruiseButtonMode")
      self.cruiseOnDist = float(Params().get_int("CruiseOnDist")) / 100.
      self.softHoldMode = Params().get_int("SoftHoldMode")
      self.cruiseSpeedMin = Params().get_int("CruiseSpeedMin")
    elif self.params_count == 30:
      self.autoSpeedUptoRoadSpeedLimit = float(self.params.get_int("AutoSpeedUptoRoadSpeedLimit")) * 0.01
    elif self.params_count == 40:
      self.autoTurnControl = Params().get_int("AutoTurnControl")
      self.autoTurnControlTurnEnd = Params().get_int("AutoTurnControlTurnEnd")
      self.autoTurnControlSpeedLaneChange = Params().get_int("AutoTurnControlSpeedLaneChange")
      self.autoTurnControlSpeedTurn = Params().get_int("AutoTurnControlSpeedTurn")
    elif self.params_count >= 100:
      self.autoCurveSpeedCtrlUse = Params().get_int("AutoCurveSpeedCtrlUse")
      self.autoCurveSpeedFactor = float(Params().get_int("AutoCurveSpeedFactor"))*0.01
      self.autoCurveSpeedFactorIn = float(Params().get_int("AutoCurveSpeedFactorIn"))*0.01
      self.params_count = 0
    
