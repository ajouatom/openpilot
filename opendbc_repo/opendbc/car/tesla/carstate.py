import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, TeslaFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.summon = False
    self.summon_prev = False
    self.cruise_enabled_prev = False
    self.fsd14_error_logged = False
    self.suspected_fsd14 = False
    self.suspected_fsd14_clear_frames = 0

    self.hands_on_level = 0
    self.acc_cancel_last = 0
    self.das_control = None
    self.das_body_controls_dat = b""
    self.das_accCancel = False
    self.cruise_override = False
    self.coop_steering = True
    self.infotainment_3_finger_press = 0

  def update_summon_state(self, summon_state: str, cruise_enabled: bool):
    summon_now = summon_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if summon_now and not self.summon_prev and not self.cruise_enabled_prev:
      self.summon = True
    if not summon_now:
      self.summon = False
    self.summon_prev = summon_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()
    length = 0.11

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Wheel speeds (km/h -> m/s)
    ws = cp_party.vl["ESP_wheelSpeeds"]
    ret.wheelSpeeds.fl = ws["ESP_wheelSpeedFrL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.fr = ws["ESP_wheelSpeedFrR"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rl = ws["ESP_wheelSpeedReL"] * CV.KPH_TO_MS
    ret.wheelSpeeds.rr = ws["ESP_wheelSpeedReR"] * CV.KPH_TO_MS

    # Displayed speed
    ui_speed_units_raw = int(cp_party.vl["DI_speed"]["DI_uiSpeedUnits"])
    ui_speed_units = self.can_define.dv.get("DI_speed", {}).get("DI_uiSpeedUnits", {}).get(ui_speed_units_raw, ui_speed_units_raw)
    ui_speed = cp_party.vl["DI_speed"]["DI_uiSpeed"]

    # Infer display unit from consistency with wheel speed first, then fall back to CAN enum/raw bit.
    ui_is_kph = False
    if ret.vEgoRaw > 2.0 and ui_speed > 2.0:
      ui_speed_kph_ms = ui_speed * CV.KPH_TO_MS
      ui_speed_mph_ms = ui_speed * CV.MPH_TO_MS
      ui_is_kph = abs(ui_speed_kph_ms - ret.vEgoRaw) <= abs(ui_speed_mph_ms - ret.vEgoRaw)
    elif ui_speed_units in ("DI_SPEED_KPH", "KPH"):
      ui_is_kph = True
    elif ui_speed_units in ("DI_SPEED_MPH", "MPH"):
      ui_is_kph = False
    else:
      ui_is_kph = ui_speed_units_raw == 1

    ret.vEgoCluster = ui_speed * (CV.KPH_TO_MS if ui_is_kph else CV.MPH_TO_MS)

    # Gas pedal
    pedal_status = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"]
    ret.gas = pedal_status / 100.0
    ret.gasPressed = pedal_status > 0

    # Motor speed (EV: motor RPM from inverter)
    ret.engineRpm = cp_party.vl["DI_torque"]["DI_axleSpeed"]

    # Brake pedal
    # Brake pedal position (0.0-1.0) from iBooster push-rod displacement [0,47] mm
    brake_rod = cp_party.vl["IBST_status"]["IBST_sInputRodDriver"]
    ret.brake = max(0.0, brake_rod / 47.0) if brake_rod > 0 else 0.0
    ret.brakePressed = cp_party.vl["IBST_status"]["IBST_driverBrakeApply"] == 2
    ret.brakeLights = cp_party.vl["ESP_status"]["ESP_brakeLamp"] == 1
    ret.regenBraking = cp_party.vl["DI_systemStatus"]["DI_regenLight"] != 0
    ret.espDisabled = cp_party.vl["ESP_status"]["ESP_espFaultLamp"] != 0
    ret.espActive = cp_party.vl["ESP_status"]["ESP_espModeActive"] != 0

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]
    ret.steeringTorqueEps = -epas_status["EPAS3S_steeringRackForce"] * length / self.CP.steerRatio

    # Stock handsOnLevel uses >0.5 for 0.25s, but this threshold reacts faster.
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED"
    ret.vehicleSensorsInvalid = cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleValidity"] != 1

    # FSD disengages on strong user override (handsOnLevel >= 3) or high angle rate faults (fast override, high speed)
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    self.steering_disengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                           eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units_raw = int(cp_party.vl["DI_state"]["DI_speedUnits"])
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(speed_units_raw, speed_units_raw)
    acc_state = cp_ap_party.vl["DAS_control"]["DAS_accState"]
    # Respect all stock DAS cancel states, not just ACC_CANCEL_GENERIC_SILENT(13).
    # ELDA/ELK triggers ACC_CANCEL_GENERIC(0) which must also be forwarded.
    self.das_accCancel = acc_state in (0, 1, 2, 12, 13, 14, 15)

    summon_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.cruise_override = cruise_state == "OVERRIDE"
    self.update_summon_state(summon_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.summon
    if speed_units in ("KPH", "DI_SPEED_KPH"):
      cruise_is_kph = True
    elif speed_units in ("MPH", "DI_SPEED_MPH"):
      cruise_is_kph = False
    else:
      # Keep cruise unit consistent with displayed speed when enum/raw bit are unreliable.
      cruise_is_kph = ui_is_kph

    ret.cruiseState.speedCluster = cp_party.vl["DI_state"]["DI_digitalSpeed"] * (CV.KPH_TO_MS if cruise_is_kph else CV.MPH_TO_MS)
    ret.cruiseState.speed = max(ret.cruiseState.speedCluster, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Emit a single cancel button event on the rising edge of any stock DAS cancel state.
    # Feeding the raw DAS_accState enum would emit spurious "unknown" events for normal
    # states and miss cancel codes other than 0/13 that das_accCancel already covers.
    acc_cancel = 1 if self.das_accCancel else 0
    ret.buttonEvents = [*create_button_events(acc_cancel, self.acc_cancel_last, {1: ButtonType.cancel})]
    self.acc_cancel_last = acc_cancel

    # DAS_fusedSpeedLimit from DBC is always in kph (scale=5). Do NOT apply ui_is_kph conversion.
    speed_limit = cp_ap_party.vl["DAS_status"]["DAS_fusedSpeedLimit"]
    if 0 < speed_limit <= 150:
      ret.speedLimit = speed_limit

    park_brake_state = self.can_define.dv["DI_state"]["DI_parkBrakeState"].get(int(cp_party.vl["DI_state"]["DI_parkBrakeState"]), None)
    vehicle_hold_state = self.can_define.dv["DI_state"]["DI_vehicleHoldState"].get(int(cp_party.vl["DI_state"]["DI_vehicleHoldState"]), None)
    ret.parkingBrake = park_brake_state == "APPLIED"
    ret.brakeHoldActive = vehicle_hold_state == "STANDSTILL"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
    ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)

    # High beam stalk used as generic toggle (openpilot convention)
    ret.genericToggle = cp_party.vl["UI_warning"]["highBeam"] == 1

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # Stock AEB from DAS — the only reliable collision avoidance signal.
    # DAS_steeringControlType (EMERGENCY_LANE_KEEP) also triggers on ELDA
    # (normal lane correction), so it's NOT used for disengagement.
    # The Panda safety layer handles emergency steering forwarding at
    # the physical level (safety_tesla.h).
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1
    ret.stockFcw = cp_ap_party.vl["DAS_status"]["DAS_forwardCollisionWarning"] != 0

    # LKAS
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # Stock Autosteer should be off (includes FSD)
    # TODO: find for TESLA_MODEL_X and HW2.5 vehicles
    if not (self.CP.flags & TeslaFlags.MISSING_DAS_SETTINGS):
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_status"]["DAS_autopilotState"] not in (0, 1, 2)  # DISABLED, UNAVAILABLE, AVAILABLE

      # Because we don't have FSD 14 detection outside of a set of FW, we should check if this FW is accidentally missing from FSD_14_FW
      # 1. If in Autosteer or FSD, already caught by invalidLkasSetting
      # 2. If in TACC and DAS ever sends ANGLE_CONTROL (1), we can infer it's trying to do LKAS on FSD 14+
      # NOTE: Tesla's latest firmware changed ELDA (Emergency Lane Departure Assist) to use ANGLE_CONTROL (1)
      # instead of EMERGENCY_LANE_KEEP (3). Exclude ELDA by checking eac_status so it doesn't latch suspected_fsd14.
      eac_is_emergency = eac_status == "EMERGENCY_LANE_KEEP"
      angle_control = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 1 and not eac_is_emergency  # ANGLE_CONTROL, excluding ELDA
      if not ret.invalidLkasSetting and angle_control and not self.CP.flags & TeslaFlags.FSD_14:
        self.suspected_fsd14 = True
        self.suspected_fsd14_clear_frames = 0

      if self.suspected_fsd14:
        ret.invalidLkasSetting = True
        if not self.fsd14_error_logged:
          carlog.error("FSD 14 detected, but FW not in FSD_14_FW set")
          self.fsd14_error_logged = True
        # Un-latch if ANGLE_CONTROL has been absent for ~3 s (100 frames @ ~33 Hz).
        # This allows re-engagement after transient triggers (e.g. if ELDA slips through on new FW variants).
        if not angle_control:
          self.suspected_fsd14_clear_frames += 1
          if self.suspected_fsd14_clear_frames >= 100:
            self.suspected_fsd14 = False
            self.suspected_fsd14_clear_frames = 0
        else:
          self.suspected_fsd14_clear_frames = 0

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    # Raw stock DAS_bodyControls bytes (bus 2), used to ride the blinker on the vehicle bus.
    if Bus.cam in can_parsers:
      self.das_body_controls_dat = bytes(can_parsers[Bus.cam].dat.get(0x3E9, b""))

    # 3-finger infotainment press detection (vehicle bus)
    if Bus.adas in can_parsers:
      cp_adas = can_parsers[Bus.adas]
      prev_infotainment = self.infotainment_3_finger_press
      self.infotainment_3_finger_press = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])
      ret.buttonEvents = [*ret.buttonEvents, *create_button_events(
        self.infotainment_3_finger_press, prev_infotainment,
        {3: ButtonType.lkas})]

    return ret

  @staticmethod
  def get_can_parsers(CP):
    parsers = {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
    }
    if CP.flags & TeslaFlags.HAS_VEHICLE_BUS:
      parsers[Bus.adas] = CANParser("tesla_model3_vehicle", [("UI_status2", 2)], CANBUS.vehicle)
    if CP.flags & TeslaFlags.HAS_DAS_BODY_CONTROLS:
      parsers[Bus.cam] = CANParser("tesla_model3_vehicle", [("DAS_bodyControls", 2)], CANBUS.autopilot_party)
    return parsers