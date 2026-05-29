import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])
    self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.hands_on_level = 0
    self.das_control = None
    self.coop_steering = True

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


    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units_raw = int(cp_party.vl["DI_state"]["DI_speedUnits"])
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(speed_units_raw, speed_units_raw)

    ret.cruiseState.enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
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

    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1
    ret.stockFcw = cp_ap_party.vl["DAS_status"]["DAS_forwardCollisionWarning"] != 0

    # Model X and HW 2.5 vehicles are missing DAS_settings

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }
