from collections import deque
import copy
import math

from cereal import car
from openpilot.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.selfdrive.car.hyundai.hyundaicanfd import CanBus
from openpilot.selfdrive.car.hyundai.values import HyundaiFlags, CAR, DBC, CAN_GEARS, CAMERA_SCC_CAR, \
                                                   CANFD_CAR, Buttons, CarControllerParams, HyundaiExtFlags
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.common.realtime import DT_CTRL
from openpilot.common.params import Params

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
STANDSTILL_THRESHOLD = 12 * 0.03125 * CV.KPH_TO_MS

GearShifter = car.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.cruise_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    # carrot for eGV70
    self.gear_msg_canfd = "ACCELERATOR" if CP.extFlags & HyundaiExtFlags.CANFD_GEARS_NONE else \
                          "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else \
                          "GEAR_ALT_2" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS_2 else \
                          "GEAR_SHIFTER"
    if CP.carFingerprint in CANFD_CAR:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif self.CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    else:  # preferred and elect gear methods use same definition
      self.shifter_values = can_define.dv["LVR12"]["CF_Lvr_Gear"]

    self.accelerator_msg_canfd = "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                                 "ACCELERATOR_ALT" if CP.flags & HyundaiFlags.HYBRID else \
                                 "ACCELERATOR_BRAKE_ALT"
    self.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else \
                                 "CRUISE_BUTTONS"
    self.is_metric = False
    self.buttons_counter = 0

    self.cruise_info = {}
    self.lfa_info = {}

    self.cruise_buttons_msg = None
    self.hda2_lfa_block_msg = None

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.params = CarControllerParams(CP)

    # PFEIFER - AOL {{
    self.main_enabled = False
    # }} PFEIFER - AOL

    self.gear_shifter = GearShifter.drive # Gear_init for Nexo ?? unknown 21.02.23.LSW

    self.totalDistance = 0.0
    self.speedLimitDistance = 0
    self.pcmCruiseGap = 0

  def update(self, cp, cp_cam):
    if self.CP.carFingerprint in CANFD_CAR:
      return self.update_canfd(cp, cp_cam)

    ret = car.CarState.new_message()
    cp_cruise = cp_cam if self.CP.extFlags & HyundaiExtFlags.SCC_BUS2.value or self.CP.carFingerprint in CAMERA_SCC_CAR else cp
    self.is_metric = cp.vl["CLU11"]["CF_Clu_SPEED_UNIT"] == 0
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    ret.doorOpen = any([cp.vl["CGW1"]["CF_Gway_DrvDrSw"], cp.vl["CGW1"]["CF_Gway_AstDrSw"],
                        cp.vl["CGW2"]["CF_Gway_RLDrSw"], cp.vl["CGW2"]["CF_Gway_RRDrSw"]])

    ret.seatbeltUnlatched = cp.vl["CGW1"]["CF_Gway_DrvSeatBeltSw"] == 0

    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHL_SPD11"]["WHL_SPD_FL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_FR"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RL"],
      cp.vl["WHL_SPD11"]["WHL_SPD_RR"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    self.cluster_speed_counter += 1
    if self.cluster_speed_counter > CLUSTER_SAMPLE_RATE:
      self.cluster_speed = cp.vl["CLU15"]["CF_Clu_VehicleSpeed"]
      self.cluster_speed_counter = 0

      # Mimic how dash converts to imperial.
      # Sorento is the only platform where CF_Clu_VehicleSpeed is already imperial when not is_metric
      # TODO: CGW_USM1->CF_Gway_DrLockSoundRValue may describe this
      if not self.is_metric and self.CP.carFingerprint not in (CAR.KIA_SORENTO,):
        self.cluster_speed = math.floor(self.cluster_speed * CV.KPH_TO_MPH + CV.KPH_TO_MPH)

    ret.vEgoCluster = self.cluster_speed * speed_conv

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0
    #ret.steerFaultTemporary = False

    # cruise state
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = self.main_enabled
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False
    else:
      ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      if cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1:  ## carrot
        ret.cruiseState.available = self.main_enabled = True
      ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.nonAdaptive = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 2.  # Shows 'Cruise Control' on dash
      ret.cruiseState.speed = cp_cruise.vl["SCC11"]["VSetDis"] * speed_conv

      cruiseGap = cp_cruise.vl["SCC11"]["TauGapSet"]
      if cruiseGap != self.pcmCruiseGap:
        self.pcmCruiseGap = cruiseGap
        Params().put_int_nonblocking("LongitudinalPersonality", min(3, max(self.pcmCruiseGap - 1, 0)))

    # TODO: Find brake pressure
    ret.brake = 0
    ret.brakePressed = cp.vl["TCS13"]["DriverOverride"] == 2  # 2 includes regen braking by user on HEV/EV
    ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
    ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
    ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
    ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)
    ret.driverOverride = cp.vl["TCS13"]["DriverOverride"]

    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      if self.CP.flags & HyundaiFlags.HYBRID:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
        ret.engineRpm = cp.vl["E_EMS11"]["N"] 
        ret.motorRpm = cp.vl["ELECT_GEAR"]["Elect_Motor_Speed"] * 30
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
        ret.engineRpm = 0.0
        ret.motorRpm = cp.vl["ELECT_GEAR"]["Elect_Motor_Speed"] * 30 # opkr, may multiply deceleration ratio in line with engine rpm
      ret.chargeMeter = cp.vl["EV_Info"]["OPKR_EV_Charge_Level"] # opkr
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])
      ret.engineRpm = 0 #cp.vl["EMS_366"]["N"]
      ret.motorRpm = 0
      ret.chargeMeter = 0

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
    elif self.CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      gear = cp.vl["TCU12"]["CUR_GR"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]

    if not self.CP.carFingerprint in (CAR.HYUNDAI_NEXO):
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))
    else:
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      gear_disp = cp.vl["ELECT_GEAR"]

      gear_shifter = GearShifter.unknown

      if gear == 1546:  # Thank you for Neokii  # fix PolorBear 22.06.05
        gear_shifter = GearShifter.drive
      elif gear == 2314:
        gear_shifter = GearShifter.neutral
      elif gear == 2569:
        gear_shifter = GearShifter.park
      elif gear == 2566:
        gear_shifter = GearShifter.reverse

      if gear_shifter != GearShifter.unknown and self.gear_shifter != gear_shifter:
        self.gear_shifter = gear_shifter

      ret.gearShifter = self.gear_shifter

    if not self.CP.openpilotLongitudinalControl or self.CP.extFlags & HyundaiExtFlags.SCC_BUS2.value:
      aeb_src = "FCA11" if self.CP.flags & HyundaiFlags.USE_FCA.value else "SCC12"
      aeb_sig = "FCA_CmdAct" if self.CP.flags & HyundaiFlags.USE_FCA.value else "AEB_CmdAct"
      aeb_warning = cp_cruise.vl[aeb_src]["CF_VSM_Warn"] != 0
      scc_warning = cp_cruise.vl["SCC12"]["TakeOverReq"] == 1  # sometimes only SCC system shows an FCW
      aeb_braking = cp_cruise.vl[aeb_src]["CF_VSM_DecCmdAct"] != 0 or cp_cruise.vl[aeb_src][aeb_sig] != 0
      ret.stockFcw = (aeb_warning or scc_warning) and not aeb_braking
      ret.stockAeb = aeb_warning and aeb_braking

    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["LCA11"]["CF_Lca_IndLeft"] != 0
      ret.rightBlindspot = cp.vl["LCA11"]["CF_Lca_IndRight"] != 0

    # save the entire LKAS11 and CLU11
    self.lkas11 = copy.copy(cp_cam.vl["LKAS11"])
    self.clu11 = copy.copy(cp.vl["CLU11"])
    self.steer_state = cp.vl["MDPS12"]["CF_Mdps_ToiActive"]  # 0 NOT ACTIVE, 1 ACTIVE
    self.prev_cruise_buttons = self.cruise_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    #carrot {{
    cruise_button = [Buttons.NONE]
    if self.CP.extFlags & HyundaiExtFlags.HAS_LFA_BUTTON.value:
      if cp.vl["BCM_PO_11"]["LFA_Pressed"]:
        cruise_button = [Buttons.LFA_BUTTON]
      else:
        cruise_button = cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"]
    else:
      cruise_button = cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"]
    self.cruise_buttons.extend(cruise_button)
    # }} carrot

    # PFEIFER - AOL {{
    self.prev_main_buttons = self.main_buttons[-1]
    # }} PFEIFER - AOL
    self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])
    self.mdps12 = copy.copy(cp.vl["MDPS12"])
    
    
    ## ajouatom
    tpms_unit = cp.vl["TPMS11"]["UNIT"] * 0.725 if int(cp.vl["TPMS11"]["UNIT"]) > 0 else 1.
    ret.tpms.fl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FL"]
    ret.tpms.fr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_FR"]
    ret.tpms.rl = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RL"]
    ret.tpms.rr = tpms_unit * cp.vl["TPMS11"]["PRESSURE_RR"]

    self.scc11 = cp_cruise.vl["SCC11"] if "SCC11" in cp_cruise.vl else None
    self.scc12 = cp_cruise.vl["SCC12"] if "SCC12" in cp_cruise.vl else None
    self.scc13 = cp_cruise.vl["SCC13"] if "SCC13" in cp_cruise.vl else None
    self.scc14 = cp_cruise.vl["SCC14"] if "SCC14" in cp_cruise.vl else None
    cluSpeed = cp.vl["CLU11"]["CF_Clu_Vanz"]
    decimal = cp.vl["CLU11"]["CF_Clu_VanzDecimal"]
    if 0. < decimal < 0.5:
      cluSpeed += decimal

    ret.vEgoCluster = cluSpeed * speed_conv
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0

    self.totalDistance += ret.vEgo * DT_CTRL 
    ret.totalDistance = self.totalDistance

    if self.CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value:
      speedLimit = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      speedLimitCam = cp.vl["Navi_HU"]["SpeedLim_Nav_Cam"]
      ret.speedLimit = speedLimit if speedLimit < 255 and speedLimitCam == 1 else 0
      if ret.speedLimit>0 and not ret.gasPressed:
        if self.speedLimitDistance <= self.totalDistance:
          self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6  
        self.speedLimitDistance = max(self.totalDistance+1, self.speedLimitDistance) 
      else:
        self.speedLimitDistance = self.totalDistance
      ret.speedLimitDistance = self.speedLimitDistance - self.totalDistance
    else:
      ret.speedLimit = 0
      ret.speedLimitDistance = 0


    # PFEIFER - AOL {{
    if self.prev_main_buttons == 0 and self.main_buttons[-1] != 0:
      self.main_enabled = not self.main_enabled
    # }} PFEIFER - AOL

    return ret

  def update_canfd(self, cp, cp_cam):
    ret = car.CarState.new_message()

    self.is_metric = cp.vl["CRUISE_BUTTONS_ALT"]["DISTANCE_UNIT"] != 1
    speed_factor = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    if self.CP.flags & (HyundaiFlags.EV | HyundaiFlags.HYBRID):
      offset = 255. if self.CP.flags & HyundaiFlags.EV else 1023.
      ret.gas = cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL"] / offset
      ret.gasPressed = ret.gas > 1e-5
    else:
      ret.gasPressed = bool(cp.vl[self.accelerator_msg_canfd]["ACCELERATOR_PEDAL_PRESSED"])

    ret.brakePressed = cp.vl["TCS"]["DriverBraking"] == 1

    ret.doorOpen = cp.vl["DOORS_SEATBELTS"]["DRIVER_DOOR"] == 1
    ret.seatbeltUnlatched = cp.vl["DOORS_SEATBELTS"]["DRIVER_SEATBELT"] == 0

    gear = cp.vl[self.gear_msg_canfd]["GEAR"]
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear))

    tpms_unit = cp.vl["TPMS"]["UNIT"] * 0.725 if int(cp.vl["TPMS"]["UNIT"]) > 0 else 1.
    ret.tpms.fl = tpms_unit * cp.vl["TPMS"]["PRESSURE_FL"]
    ret.tpms.fr = tpms_unit * cp.vl["TPMS"]["PRESSURE_FR"]
    ret.tpms.rl = tpms_unit * cp.vl["TPMS"]["PRESSURE_RL"]
    ret.tpms.rr = tpms_unit * cp.vl["TPMS"]["PRESSURE_RR"]

    # TODO: figure out positions
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_1"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_2"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_3"],
      cp.vl["WHEEL_SPEEDS"]["WHEEL_SPEED_4"],
    )
    ret.vEgoRaw = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.wheelSpeeds.fl <= STANDSTILL_THRESHOLD and ret.wheelSpeeds.rr <= STANDSTILL_THRESHOLD

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0
    #ret.steerFaultTemporary = False

    # carrot test
    left_blinker_lamp = cp.vl["BLINKERS"]["LEFT_LAMP"] or cp.vl["BLINKERS"]["LEFT_LAMP_ALT"] 
    right_blinker_lamp = cp.vl["BLINKERS"]["RIGHT_LAMP"] or cp.vl["BLINKERS"]["RIGHT_LAMP_ALT"] 
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, left_blinker_lamp, right_blinker_lamp)

    # TODO: alt signal usage may be described by cp.vl['BLINKERS']['USE_ALT_LAMP']
    #left_blinker_sig, right_blinker_sig = "LEFT_LAMP", "RIGHT_LAMP"
    #if self.CP.carFingerprint in [CAR.KONA_EV_2ND_GEN, CAR.KIA_CARNIVAL_4TH_GEN]:
    #  left_blinker_sig, right_blinker_sig = "LEFT_LAMP_ALT", "RIGHT_LAMP_ALT"
    #ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["BLINKERS"][left_blinker_sig],
    #                                                                  cp.vl["BLINKERS"][right_blinker_sig])
    if self.CP.enableBsm:
      cp_ = cp_cam if (self.CP.extFlags & HyundaiExtFlags.SCC_BUS2 and self.CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value) else cp
      #ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0
      #ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0
      if self.CP.carFingerprint in [CAR.KIA_CARNIVAL_4TH_GEN]:
        ret.leftBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_LEFT_FOUR"] != 0
        ret.rightBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_RIGHT_FOUR"] != 0
      else:
        ret.leftBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0
        ret.rightBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0

    # cruise state
    # CAN FD cars enable on main button press, set available if no TCS faults preventing engagement
    #ret.cruiseState.available = cp.vl["TCS"]["ACCEnable"] == 0
    # PFEIFER - AOL {{
    ret.cruiseState.available = self.main_enabled
    # }} PFEIFER - AOL
    cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC or self.CP.extFlags & HyundaiExtFlags.SCC_BUS2.value else cp
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.enabled = cp.vl["TCS"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      #if ret.cruiseState.available:
      #  print("cruiseState.available = {},{}".format(ret.cruiseState.available, ret.cruiseState.enabled))
    else:
      #cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC or self.CP.extFlags & HyundaiExtFlags.SCC_BUS2.value else cp
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      if cp_cruise_info.vl["SCC_CONTROL"]["MainMode_ACC"] == 1: # carrot
        ret.cruiseState.available = self.main_enabled = True
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["CRUISE_STANDSTILL"] == 1
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      self.cruise_info = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])

    if self.CP.extFlags & HyundaiExtFlags.SCC_BUS2.value:
      self.cruise_info = copy.copy(cp_cam.vl["SCC_CONTROL"])
      self.lfa_info = copy.copy(cp_cam.vl["LFA"])

    ret.brakeHoldActive = cp.vl["ESP_STATUS"]["AUTO_HOLD"] == 1 and cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] not in (1, 2)
    # Manual Speed Limit Assist is a feature that replaces non-adaptive cruise control on EV CAN FD platforms.
    # It limits the vehicle speed, overridable by pressing the accelerator past a certain point.
    # The car will brake, but does not respect positive acceleration commands in this mode
    # TODO: find this message on ICE & HYBRID cars + cruise control signals (if exists)
    if self.CP.flags & HyundaiFlags.EV:
      ret.cruiseState.nonAdaptive = cp.vl["MANUAL_SPEED_LIMIT_ASSIST"]["MSLA_ENABLED"] == 1

    self.prev_cruise_buttons = self.cruise_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    #carrot {{
    if cp.vl[self.cruise_btns_msg_canfd]["LFA_BTN"]:
      cruise_button = [Buttons.LFA_BUTTON]
    else:
      cruise_button = cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]
    self.cruise_buttons.extend(cruise_button)
    # }} carrot
    

    if self.cruise_btns_msg_canfd in cp.vl_all: #carrot
      if not cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]:
        pass
        #print("empty cruise btns...")
      else:
        self.cruise_buttons_msg = copy.copy(cp.vl_all[self.cruise_btns_msg_canfd])

    # PFEIFER - AOL {{
    self.prev_main_buttons = self.main_buttons[-1]
    # }} PFEIFER - AOL
    self.main_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["ADAPTIVE_CRUISE_MAIN_BTN"])
    # PFEIFER - AOL {{
    if self.main_buttons[-1] != self.prev_main_buttons and not self.main_buttons[-1]: # and self.CP.openpilotLongitudinalControl: #carrot
      self.main_enabled = not self.main_enabled
      print("main_enabled = {}".format(self.main_enabled))
    # }} PFEIFER - AOL
    self.buttons_counter = cp.vl[self.cruise_btns_msg_canfd]["COUNTER"]
    ret.accFaulted = cp.vl["TCS"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    if self.CP.flags & HyundaiFlags.CANFD_HDA2 and not (self.CP.extFlags & HyundaiExtFlags.SCC_BUS2.value):
      self.hda2_lfa_block_msg = copy.copy(cp_cam.vl["CAM_0x362"] if self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING
                                          else cp_cam.vl["CAM_0x2a4"])

    # 측정값을 그냥 넣음... test
    #ret.vCluRatio = 0.945
    speed_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    cluSpeed = cp.vl["CRUISE_BUTTONS_ALT"]["CLU_SPEED"]
    ret.vEgoCluster = cluSpeed * speed_conv
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0
    
    
    self.totalDistance += ret.vEgo * DT_CTRL 
    ret.totalDistance = self.totalDistance
    if self.CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value and False:  ## 차량 네비 정보 삭제...
      speedLimit = 0
      speed_limit_clu_bus_canfd = cp if self.CP.flags & HyundaiFlags.CANFD_HDA2 else cp_cam
      if "CLUSTER_SPEED_LIMIT" in speed_limit_clu_bus_canfd.vl:
        speedLimit = speed_limit_clu_bus_canfd.vl["CLUSTER_SPEED_LIMIT"]["SPEED_LIMIT_1"]
      else:
        if "CLUSTER_SPEED_LIMIT" in cp.vl:
          print("CLUSTER_SPEED_LIMIT in cp")
        elif "CLUSTER_SPEED_LIMIT" in cp_cam.vl:
          print("CLUSTER_SPEED_LIMIT in cp_cam")
        else:
          print("CLUSTER_SPEED_LIMIT none")

      speedLimitCam = 1
      ret.speedLimit = speedLimit if speedLimit < 255 and speedLimitCam == 1 else 0
      if ret.speedLimit>0 and not ret.gasPressed:
        if self.speedLimitDistance <= self.totalDistance:
          self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6  
        self.speedLimitDistance = max(self.totalDistance+1, self.speedLimitDistance) 
      else:
        self.speedLimitDistance = self.totalDistance
      ret.speedLimitDistance = self.speedLimitDistance - self.totalDistance
    else:
      ret.speedLimit = 0
      ret.speedLimitDistance = 0

    return ret

  def get_can_parser(self, CP):
    if CP.carFingerprint in CANFD_CAR:
      return self.get_can_parser_canfd(CP)

    messages = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS13", 50),
      ("TCS15", 10),
      ("CLU11", 50),
      ("CLU15", 5),
      ("ESP12", 100),
      ("CGW1", 10),
      ("CGW2", 5),
      ("CGW4", 5),
      ("WHL_SPD11", 50),
      ("SAS11", 100),
      ("TPMS11", 5), 
    ]

    if not CP.openpilotLongitudinalControl and CP.carFingerprint not in CAMERA_SCC_CAR:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages.append(("FCA11", 50))

    if CP.enableBsm:
      messages.append(("LCA11", 50))

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      messages += [
        ("E_EMS11", 50),
        ("EV_Info", 50),     
      ]
    else:
      messages += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      messages.append(("ELECT_GEAR", 20))
    elif CP.carFingerprint in CAN_GEARS["use_cluster_gears"]:
      pass
    elif CP.carFingerprint in CAN_GEARS["use_tcu_gears"]:
      messages.append(("TCU12", 100))
    else:
      messages.append(("LVR12", 100))
      
    if CP.extFlags & HyundaiExtFlags.HAS_LFA_BUTTON.value:
      messages.append(("BCM_PO_11", 50))

    if CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value:
      messages.append(("Navi_HU", 5))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    if CP.carFingerprint in CANFD_CAR:
      return CarState.get_cam_can_parser_canfd(CP)

    messages = [
      ("LKAS11", 100)
    ]
    if CP.extFlags & HyundaiExtFlags.SCC_BUS2.value:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.extFlags & HyundaiExtFlags.HAS_SCC13.value:
        messages += [
          ("SCC13", 0),
        ]
      if CP.extFlags & HyundaiExtFlags.HAS_SCC14.value:
        messages += [
          ("SCC14", 50),
        ]      
      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages += [
          ("FCA11", 50),
        ]

      if CP.extFlags & HyundaiExtFlags.HAS_LFAHDA.value:
        messages += [("LFAHDA_MFC", 20)]

      return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

    if not CP.openpilotLongitudinalControl and CP.carFingerprint in CAMERA_SCC_CAR:
      messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.flags & HyundaiFlags.USE_FCA.value:
        messages.append(("FCA11", 50))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)

  def get_can_parser_canfd(self, CP):
    messages = [
      (self.accelerator_msg_canfd, 100),
      ("WHEEL_SPEEDS", 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      #("BRAKE", 100),
      ("ESP_STATUS", 100),
      ("TCS", 50),
      ("CRUISE_BUTTONS_ALT", 50),
      ("TPMS", 5),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
    ]

    ## carrot: for EGV70
    if not (CP.extFlags & HyundaiExtFlags.CANFD_GEARS_NONE):
      messages += [
        (self.gear_msg_canfd, 100),
      ]

    if CP.flags & HyundaiFlags.EV:
      messages += [
        ("MANUAL_SPEED_LIMIT_ASSIST", 10),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      messages += [
        ("CRUISE_BUTTONS", 50)
      ]

    ## BSM신호가 ADAS인경우 BUS2로 개조되고, 독립인경우 ECAN에서 들어옴.
    # 개조, 독립 EV6: 1, 1 => True, inADAS: 1, 0 => False
    # 비개조, 0, 0 => True
    if CP.enableBsm:
      if CP.extFlags & HyundaiExtFlags.SCC_BUS2.value and CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value:
        pass
      else:
        messages += [
          ("BLINDSPOTS_REAR_CORNERS", 20),
        ]

    if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not CP.openpilotLongitudinalControl and not (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value):
      messages += [
        ("SCC_CONTROL", 50),
      ]

    #if CP.flags & HyundaiFlags.CANFD_HDA2 and CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value and not (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value):
    #  messages.append(("CLUSTER_SPEED_LIMIT", 10))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).ECAN)

  @staticmethod
  def get_cam_can_parser_canfd(CP):
    messages = []
    if CP.flags & HyundaiFlags.CANFD_HDA2 and not (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value):
      block_lfa_msg = "CAM_0x362" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "CAM_0x2a4"
      messages += [(block_lfa_msg, 20)]
    if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC or CP.extFlags & HyundaiExtFlags.SCC_BUS2.value:
      messages += [
        ("SCC_CONTROL", 50),
      ]
    if CP.extFlags & HyundaiExtFlags.SCC_BUS2.value:
      messages += [
        ("LFA", 20),
      ]

    #if not (CP.flags & HyundaiFlags.CANFD_HDA2) and CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value and (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value) :
    #  messages.append(("CLUSTER_SPEED_LIMIT", 10))

    ## BSM신호가 ADAS인경우 BUS2로 개조되고, 독립인경우 ECAN에서 들어옴.
    # 개조, 독립 EV6: 1, 1 => False, inADAS: 1, 0 => True
    # 비개조, 0, 0 => False
    if CP.enableBsm:
      if CP.extFlags & HyundaiExtFlags.SCC_BUS2.value and CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value:
        messages += [
          ("BLINDSPOTS_REAR_CORNERS", 20),
        ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, CanBus(CP).CAM)
