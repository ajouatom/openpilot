import copy
import math
import time
from dataclasses import dataclass, field
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC8H2F, mk_crc8_fun
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, TeslaFlags

ButtonType = structs.CarState.ButtonEvent.Type

TESLA_GAS_PRESS_ON = 0.8
TESLA_GAS_PRESS_OFF = 0.4
TESLA_TPMS_PRESSURE_SNA = 255 * 0.025
TESLA_TPMS_BAR_TO_PSI = 14.5037738
TESLA_TPMS_MAX_AGE_NS = 10_000_000_000
SPEED_AUTO_RESUME_GESTURE_NS = 1_000_000_000
STOCK_ACC_CANCEL_STATES = (0, 1, 2, 12, 13, 14, 15)
STOCK_ACC_CANCEL_PULSE_FRAMES = 4
TESLA_EAC_NON_FAULT_INHIBITS = ("EAC_ERROR_IDLE", "EAC_ERROR_MIN_SPEED")
TESLA_STALK_CRC = mk_crc8_fun(CRC8H2F)
TESLA_STALK_MAGIC = (0x9B, 0xE8, 0x2A, 0xD3, 0xD3, 0x83, 0x4C, 0x5E,
                     0x3F, 0x5E, 0xE2, 0x28, 0x3A, 0x13, 0xAF, 0xCE)


def update_tesla_gas_pressed(previous: bool, pedal_position: float) -> bool:
  threshold = TESLA_GAS_PRESS_OFF if previous else TESLA_GAS_PRESS_ON
  return float(pedal_position) > threshold


def get_tesla_tpms_pressure(pressure_bar: float) -> float:
  return round(pressure_bar * TESLA_TPMS_BAR_TO_PSI, 1) if pressure_bar < TESLA_TPMS_PRESSURE_SNA else 0.0


def decode_tesla_stalk(data: bytes) -> int | None:
  if len(data) == 4:
    payload = bytes((data[1] & 0xF0, data[2], data[3], 0))
    checksum = TESLA_STALK_CRC(payload) ^ TESLA_STALK_MAGIC[data[1] & 15]
    position = data[2] & 15
    if data[0] == checksum and position < 9:
      return 1 if position >= 5 else 2 if position else 0
  return None


@dataclass
class TeslaStalkInput:
  timestamp: int = 0
  direction: int | None = None
  samples: list[tuple[int, int]] = field(default_factory=list)


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
    self.gas_pressed = False
    # Conservative default: no steering until the first update() proves otherwise
    self.steering_disengage = True
    self.acc_cancel_last = 0
    self.das_control = None
    self.das_accCancel = False
    self.das_acc_state_last = None
    self.das_acc_cancel_frames = 0
    self.cruise_override = False
    self.coop_steering = True
    self.infotainment_3_finger_press = 0
    self._tpms_available = False
    self.tesla_speed_button_template = None
    self.tesla_speed_button_template_nanos = 0
    self.tesla_speed_limit_target = 0.0
    self.tesla_speed_limit_target_nanos = 0
    self.tesla_speed_limit_target_valid = False
    self.tesla_speed_units = "KPH"
    self.tesla_manual_speed_adjustment_counter = 0
    self.tesla_speed_auto_resume_gesture_counter = 0
    self._tesla_speed_resume_up_nanos = 0
    self._tesla_speed_resume_down_nanos = 0
    self._tesla_speed_resume_wait_idle = False
    self._turn_stalk_direction = 0
    self._turn_stalk_counts = [0, 0]
    self._turn_stalk_inputs: dict[int, TeslaStalkInput] = {}
    self._invalid_measurements: tuple[str, ...] = ()

  def _measurement_valid(self, parser: CANParser, message: str, signal: str) -> bool:
    value = parser.vl[message][signal]
    msg = parser.dbc.name_to_msg[message]
    data = parser.dat.get(msg.address)
    if not math.isfinite(value) or (data is not None and len(data) != msg.size):
      return False
    definition = msg.sigs[signal]
    raw = round((value - definition.offset) / definition.factor)
    minimum = -(1 << (definition.size - 1)) if definition.is_signed else 0
    maximum = (1 << (definition.size - int(definition.is_signed))) - 1
    label = self.can_define.dv.get(message, {}).get(signal, {}).get(raw, "")
    return minimum <= raw <= maximum and not label.endswith("SNA")

  def observe_turn_stalk_frame(self, data: bytes, source: int, monotonic_nanos: int) -> None:
    if not self.CP.flags & TeslaFlags.AUTO_TURN_SIGNALS or source not in (CANBUS.party, CANBUS.vehicle):
      return
    stalk = self._turn_stalk_inputs.setdefault(source, TeslaStalkInput())
    stalk.timestamp = monotonic_nanos
    stalk.direction = decode_tesla_stalk(data)
    if stalk.direction is None:
      stalk.samples.clear()
    else:
      stalk.samples.append((monotonic_nanos, stalk.direction))

  def observe_speed_wheel_frame(self, data: bytes, monotonic_nanos: int) -> None:
    if len(data) != 8 or (data[0] & 0x03) != 1:
      return

    raw_tick = data[3] & 0x3F
    if raw_tick == 0:
      self.tesla_speed_button_template = bytes(data)
      self.tesla_speed_button_template_nanos = monotonic_nanos
      self._tesla_speed_resume_wait_idle = False
      return

    if self._tesla_speed_resume_wait_idle:
      return

    signed_tick = raw_tick - 0x40 if raw_tick & 0x20 else raw_tick
    direction = 1 if signed_tick > 0 else -1
    self.tesla_manual_speed_adjustment_counter += 1
    opposite_nanos = self._tesla_speed_resume_down_nanos if direction > 0 else self._tesla_speed_resume_up_nanos
    if opposite_nanos and monotonic_nanos - opposite_nanos <= SPEED_AUTO_RESUME_GESTURE_NS:
      self.tesla_speed_auto_resume_gesture_counter += 1
      self._tesla_speed_resume_up_nanos = 0
      self._tesla_speed_resume_down_nanos = 0
      self._tesla_speed_resume_wait_idle = True
    elif direction > 0:
      self._tesla_speed_resume_up_nanos = monotonic_nanos
      self._tesla_speed_resume_down_nanos = 0
    else:
      self._tesla_speed_resume_down_nanos = monotonic_nanos
      self._tesla_speed_resume_up_nanos = 0

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
    now = time.clock_gettime_ns(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))
    invalid_measurements = []

    # Vehicle speed
    if self._measurement_valid(cp_party, "DI_speed", "DI_vehicleSpeed"):
      ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
      ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    else:
      invalid_measurements.append("DI_speed.DI_vehicleSpeed")
      ret.vEgoRaw, ret.vEgo = self.out.vEgoRaw, self.out.vEgo

    # Wheel speeds (km/h -> m/s)
    ws = cp_party.vl["ESP_wheelSpeeds"]
    for wheel, signal in (("fl", "ESP_wheelSpeedFrL"), ("fr", "ESP_wheelSpeedFrR"),
                          ("rl", "ESP_wheelSpeedReL"), ("rr", "ESP_wheelSpeedReR")):
      if self._measurement_valid(cp_party, "ESP_wheelSpeeds", signal):
        setattr(ret.wheelSpeeds, wheel, ws[signal] * CV.KPH_TO_MS)
      else:
        invalid_measurements.append(f"ESP_wheelSpeeds.{signal}")
        setattr(ret.wheelSpeeds, wheel, getattr(self.out.wheelSpeeds, wheel))

    # Displayed speed
    ui_speed_units_raw = cp_party.vl["DI_speed"]["DI_uiSpeedUnits"]
    if ui_speed_units_raw not in (0, 1):
      invalid_measurements.append("DI_speed.DI_uiSpeedUnits")
    if self._measurement_valid(cp_party, "DI_speed", "DI_uiSpeed") and ui_speed_units_raw in (0, 1):
      ret.vEgoCluster = cp_party.vl["DI_speed"]["DI_uiSpeed"] * (CV.KPH_TO_MS if ui_speed_units_raw == 1 else CV.MPH_TO_MS)
    else:
      invalid_measurements.append("DI_speed.DI_uiSpeed")
      ret.vEgoCluster = self.out.vEgoCluster

    # Gas pedal. Hysteresis (0.8% on / 0.4% off) so DI_accelPedalPos noise
    # above 0 does not spuriously trigger gasPressed.
    pedal_status = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"]
    if self._measurement_valid(cp_party, "DI_systemStatus", "DI_accelPedalPos"):
      ret.gas = pedal_status / 100.0
      self.gas_pressed = update_tesla_gas_pressed(self.gas_pressed, pedal_status)
    else:
      invalid_measurements.append("DI_systemStatus.DI_accelPedalPos")
      ret.gas = self.out.gas
      self.gas_pressed = True
    pedal_time = cp_party.ts_nanos["DI_speed"]["DI_accelPedalPressed"]
    digital_pedal_pressed = (0 < pedal_time <= now and now - pedal_time <= 500_000_000 and
                             cp_party.vl["DI_speed"]["DI_accelPedalPressed"] == 1)
    ret.gasPressed = self.gas_pressed or digital_pedal_pressed

    # Motor speed (EV: motor RPM from inverter)
    ret.engineRpm = cp_party.vl["DI_torque"]["DI_axleSpeed"]

    # Brake pedal. Newer Model Y vehicles do not expose IBST_status on the party bus,
    # while ESP_status is available across the supported Model 3/Y platforms.
    ret.brake = 0.0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2
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
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    # Tesla reports INHIBITED + IDLE/MIN_SPEED during normal EPS transitions,
    # including startup, standstill, and low-speed operation.
    ret.steerFaultTemporary = eac_status == "EAC_INHIBITED" and eac_error_code not in TESLA_EAC_NON_FAULT_INHIBITS

    # Do not set vehicleSensorsInvalid from SCCM_steeringAngleValidity: refreshed
    # Model Y vehicles report 0 while angle/rate remain valid. EPS faults are covered above.

    # FSD disengages on strong user override (handsOnLevel >= 3) or high angle rate faults (fast override, high speed)
    self.steering_disengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                           eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units_raw = cp_party.vl["DI_state"]["DI_speedUnits"]
    acc_state = cp_ap_party.vl["DAS_control"]["DAS_accState"]
    # DAS_accState=0 is the steady idle value when stock ACC is unavailable.
    # Only forward a cancellation after the stock controller was actively on;
    # otherwise CP would continually cancel a new stalk engagement.
    if self.das_acc_state_last in (3, 4) and acc_state in STOCK_ACC_CANCEL_STATES:
      self.das_acc_cancel_frames = STOCK_ACC_CANCEL_PULSE_FRAMES
    self.das_acc_state_last = acc_state
    self.das_accCancel = self.das_acc_cancel_frames > 0
    if self.das_acc_cancel_frames > 0:
      self.das_acc_cancel_frames -= 1

    summon_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.cruise_override = cruise_state == "OVERRIDE"
    self.update_summon_state(summon_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.summon
    cruise_is_kph = speed_units_raw == 1
    if speed_units_raw not in (0, 1):
      invalid_measurements.append("DI_state.DI_speedUnits")
    self.tesla_speed_units = "KPH" if cruise_is_kph else "MPH"
    if self._measurement_valid(cp_party, "DI_state", "DI_digitalSpeed") and speed_units_raw in (0, 1):
      ret.cruiseState.speedCluster = cp_party.vl["DI_state"]["DI_digitalSpeed"] * (CV.KPH_TO_MS if cruise_is_kph else CV.MPH_TO_MS)
    else:
      invalid_measurements.append("DI_state.DI_digitalSpeed")
      ret.cruiseState.speedCluster = self.out.cruiseState.speedCluster
    ret.cruiseState.speed = max(ret.cruiseState.speedCluster, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cp_party.vl["ESP_B"]["ESP_vehicleStandstillSts"] == 1
    ret.accFaulted = cruise_state == "FAULT"

    # Emit a single cancel button event on the rising edge of any stock DAS cancel state.
    # Feeding the raw DAS_accState enum would emit spurious "unknown" events for normal
    # states and miss cancel codes other than 0/13 that das_accCancel already covers.
    acc_cancel = 1 if self.das_accCancel else 0
    ret.buttonEvents = [*create_button_events(acc_cancel, self.acc_cancel_last, {1: ButtonType.cancel})]
    self.acc_cancel_last = acc_cancel

    # The HW4 fused limit is km/h, independent of the selected cruise display unit.
    speed_limit = cp_ap_party.vl["DAS_status"]["DAS_fusedSpeedLimit"]
    speed_limit_time = cp_ap_party.ts_nanos["DAS_status"]["DAS_fusedSpeedLimit"]
    if 0 < speed_limit <= 150 and speed_limit_time > 0:
      self.tesla_speed_limit_target = speed_limit * CV.KPH_TO_MS
      # The shared speed-limit display expects km/h even on MPH vehicles.
      ret.speedLimit = self.tesla_speed_limit_target * CV.MS_TO_KPH
      self.tesla_speed_limit_target_nanos = speed_limit_time
      self.tesla_speed_limit_target_valid = True
    else:
      self.tesla_speed_limit_target = 0.0
      self.tesla_speed_limit_target_nanos = 0
      self.tesla_speed_limit_target_valid = False

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
    if self.CP.flags & TeslaFlags.AUTO_TURN_SIGNALS:
      # Retain raw frame validation and batch ordering for the four-byte stalk.
      vehicle = self._turn_stalk_inputs.get(CANBUS.vehicle)
      party = self._turn_stalk_inputs.get(CANBUS.party)
      source = vehicle if vehicle is not None else party
      samples = {bus: [direction for timestamp, direction in stalk.samples if 0 <= now - timestamp <= 1_000_000_000]
                 for bus, stalk in self._turn_stalk_inputs.items()}
      source_samples = samples.get(CANBUS.vehicle if vehicle is not None else CANBUS.party, [])
      ret.turnSignalDriverValid = (source is not None and source.timestamp > 0 and
                                   0 <= now - source.timestamp <= 1_000_000_000 and source.direction is not None)
      if source is vehicle and party is not None and party.timestamp > 0:
        if 0 <= now - party.timestamp <= 1_000_000_000:
          ret.turnSignalDriverValid &= party.direction is not None and party.direction == source.direction
          party_presses = {d for d in samples.get(CANBUS.party, []) if d}
          source_presses = {d for d in source_samples if d}
          if party_presses and party_presses != source_presses:
            ret.turnSignalDriverValid = False
      if ret.turnSignalDriverValid:
        for direction in source_samples or [source.direction]:
          if direction and direction != self._turn_stalk_direction:
            self._turn_stalk_counts[direction - 1] = (self._turn_stalk_counts[direction - 1] + 1) % 256
          self._turn_stalk_direction = direction
        ret.turnSignalDriverDirection = self._turn_stalk_direction
      ret.leftBlinkerStalkCount, ret.rightBlinkerStalkCount = self._turn_stalk_counts
      for stalk in self._turn_stalk_inputs.values():
        stalk.samples.clear()

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
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

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

    # 3-finger infotainment press detection (vehicle bus)
    if Bus.adas in can_parsers:
      cp_adas = can_parsers[Bus.adas]
      cp_body = can_parsers[Bus.body]
      tpms = cp_body.vl["VCSEC_TPMSDisplay"]
      tpms_time = cp_body.ts_nanos["VCSEC_TPMSDisplay"]["VCSEC_TPMSDisplayPressureFL"]
      tpms_available = (len(cp_body.dat.get(0x25A, b"")) == 8 and 0 < tpms_time <= now and
                        now - tpms_time <= TESLA_TPMS_MAX_AGE_NS)
      if tpms_available:
        ret.tpms.fl = get_tesla_tpms_pressure(tpms["VCSEC_TPMSDisplayPressureFL"])
        ret.tpms.fr = get_tesla_tpms_pressure(tpms["VCSEC_TPMSDisplayPressureFR"])
        ret.tpms.rl = get_tesla_tpms_pressure(tpms["VCSEC_TPMSDisplayPressureRL"])
        ret.tpms.rr = get_tesla_tpms_pressure(tpms["VCSEC_TPMSDisplayPressureRR"])
      if tpms_available != self._tpms_available:
        if tpms_available:
          carlog.info("Tesla TPMS data available")
        else:
          carlog.warning("Tesla TPMS data unavailable (stale or invalid frame)")
        self._tpms_available = tpms_available

      prev_infotainment = self.infotainment_3_finger_press
      self.infotainment_3_finger_press = int(cp_adas.vl["UI_status2"]["UI_activeTouchPoints"])
      ret.buttonEvents = [*ret.buttonEvents, *create_button_events(
        self.infotainment_3_finger_press, prev_infotainment,
        {3: ButtonType.lkas})]

    ret.vehicleSensorsInvalid = bool(invalid_measurements)
    invalid = tuple(invalid_measurements)
    if invalid != self._invalid_measurements:
      if invalid:
        carlog.error(f"Tesla invalid measurements (holding last valid values): {', '.join(invalid)}")
      else:
        carlog.info("Tesla measurements recovered")
      self._invalid_measurements = invalid
    return ret

  @staticmethod
  def get_can_parsers(CP):
    parsers = {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
    }
    if CP.flags & TeslaFlags.HAS_VEHICLE_BUS:
      parsers[Bus.adas] = CANParser("tesla_model3_vehicle", [("UI_status2", 2)], CANBUS.vehicle)
      # Low-rate display telemetry must not create a second bus-off watchdog.
      parsers[Bus.body] = CANParser("tesla_modely_hw4_perception", [("VCSEC_TPMSDisplay", float("nan"))], CANBUS.vehicle)
    return parsers
