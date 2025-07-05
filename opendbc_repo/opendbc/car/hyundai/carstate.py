from collections import deque
import copy
import math
import numpy as np

from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from opendbc.car import Bus, create_button_events, structs, DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, CAR, DBC, Buttons, CarControllerParams, CAMERA_SCC_CAR, HyundaiExtFlags
from opendbc.car.interfaces import CarStateBase

from openpilot.common.params import Params


ButtonType = structs.CarState.ButtonEvent.Type

PREV_BUTTON_SAMPLES = 8
CLUSTER_SAMPLE_RATE = 20  # frames
STANDSTILL_THRESHOLD = 12 * 0.03125 * CV.KPH_TO_MS

BUTTONS_DICT = {Buttons.RES_ACCEL: ButtonType.accelCruise, Buttons.SET_DECEL: ButtonType.decelCruise,
                Buttons.GAP_DIST: ButtonType.gapAdjustCruise, Buttons.CANCEL: ButtonType.cancel, Buttons.LFA_BUTTON: ButtonType.lfaButton}

GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])

    self.cruise_buttons: deque = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)
    self.main_buttons: deque = deque([Buttons.NONE] * PREV_BUTTON_SAMPLES, maxlen=PREV_BUTTON_SAMPLES)

    self.gear_msg_canfd = "GEAR" if CP.extFlags & HyundaiExtFlags.CANFD_GEARS_69 else \
                          "ACCELERATOR" if CP.flags & HyundaiFlags.EV else \
                          "GEAR_ALT" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS else \
                          "GEAR_ALT_2" if CP.flags & HyundaiFlags.CANFD_ALT_GEARS_2 else \
                          "GEAR_SHIFTER"
    if CP.flags & HyundaiFlags.CANFD:
      self.shifter_values = can_define.dv[self.gear_msg_canfd]["GEAR"]
    elif CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      self.shifter_values = can_define.dv["ELECT_GEAR"]["Elect_Gear_Shifter"]
    elif self.CP.flags & HyundaiFlags.CLUSTER_GEARS:
      self.shifter_values = can_define.dv["CLU15"]["CF_Clu_Gear"]
    elif self.CP.flags & HyundaiFlags.TCU_GEARS:
      self.shifter_values = can_define.dv["TCU12"]["CUR_GR"]
    elif CP.flags & HyundaiFlags.FCEV:
      self.shifter_values = can_define.dv["EMS20"]["HYDROGEN_GEAR_SHIFTER"]
    else:
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
    self.lfa_alt_info = {}
    self.lfahda_cluster_info = {}
    self.adrv_info_161 = None
    self.adrv_info_200 = None
    self.adrv_info_1ea = None
    self.adrv_info_160 = None
    self.adrv_info_162 = None
    self.hda_info_4a3 = None
    self.new_msg_4b4 = None
    self.tcs_info_373 = None
    self.mdps_info = {}
    self.steer_touch_info = {}

    self.cruise_buttons_msg = None
    self.hda2_lfa_block_msg = None
    self.cluster_speed_limit_msg = None

    # On some cars, CLU15->CF_Clu_VehicleSpeed can oscillate faster than the dash updates. Sample at 5 Hz
    self.cluster_speed = 0
    self.cluster_speed_counter = CLUSTER_SAMPLE_RATE

    self.params = CarControllerParams(CP)

    self.main_enabled = True if Params().get_int("AutoEngage") == 2 else False
    self.gear_shifter = GearShifter.drive # Gear_init for Nexo ?? unknown 21.02.23.LSW

    self.totalDistance = 0.0
    self.speedLimitDistance = 0
    self.pcmCruiseGap = 0

    self.cruise_buttons_alt = False # for CASPER_EV
    self.MainMode_ACC = False
    self.ACCMode = 0
    self.LFA_ICON = 0
    self.paddle_button_prev = 0

    self.lf_distance = 0
    self.rf_distance = 0
    self.lr_distance = 0
    self.rr_distance = 0
    #self.lf_lateral = 0
    #self.rf_lateral = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    if self.CP.flags & HyundaiFlags.CANFD:
      return self.update_canfd(can_parsers)

    ret = structs.CarState()
    cp_cruise = cp_cam if self.CP.flags & HyundaiFlags.CAMERA_SCC else cp
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

    #ret.vEgoCluster = self.cluster_speed * speed_conv

    ret.steeringAngleDeg = cp.vl["SAS11"]["SAS_Angle"]
    ret.steeringRateDeg = cp.vl["SAS11"]["SAS_Speed"]
    ret.yawRate = cp.vl["ESP12"]["YAW_RATE"]
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(
      50, cp.vl["CGW1"]["CF_Gway_TurnSigLh"], cp.vl["CGW1"]["CF_Gway_TurnSigRh"])
    ret.steeringTorque = cp.vl["MDPS12"]["CR_Mdps_StrColTq"]
    ret.steeringTorqueEps = cp.vl["MDPS12"]["CR_Mdps_OutTq"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS12"]["CF_Mdps_ToiUnavail"] != 0 or cp.vl["MDPS12"]["CF_Mdps_ToiFlt"] != 0

    # cruise state
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.available = self.main_enabled #cp.vl["TCS13"]["ACCEnable"] == 0
      ret.cruiseState.enabled = cp.vl["TCS13"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      ret.cruiseState.nonAdaptive = False
    elif not self.CP.flags & HyundaiFlags.CC_ONLY_CAR:
      self.main_enabled = ret.cruiseState.available = cp_cruise.vl["SCC11"]["MainMode_ACC"] == 1
      ret.cruiseState.enabled = cp_cruise.vl["SCC12"]["ACCMode"] != 0
      ret.cruiseState.standstill = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 4.
      ret.cruiseState.nonAdaptive = cp_cruise.vl["SCC11"]["SCCInfoDisplay"] == 2.  # Shows 'Cruise Control' on dash
      ret.cruiseState.speed = cp_cruise.vl["SCC11"]["VSetDis"] * speed_conv

      ret.pcmCruiseGap = cp_cruise.vl["SCC11"]["TauGapSet"]

    # TODO: Find brake pressure
    ret.brake = 0
    if not self.CP.flags & HyundaiFlags.CC_ONLY_CAR:
      ret.brakePressed = cp.vl["TCS13"]["DriverOverride"] == 2  # 2 includes regen braking by user on HEV/EV
      ret.brakeHoldActive = cp.vl["TCS15"]["AVH_LAMP"] == 2  # 0 OFF, 1 ERROR, 2 ACTIVE, 3 READY
      ret.parkingBrake = cp.vl["TCS13"]["PBRAKE_ACT"] == 1
      ret.espDisabled = cp.vl["TCS11"]["TCS_PAS"] == 1
      ret.espActive = cp.vl["TCS11"]["ABS_ACT"] == 1
      ret.accFaulted = cp.vl["TCS13"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED
      ret.brakeLights = bool(cp.vl["TCS13"]["BrakeLight"] or ret.brakePressed)

    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV | HyundaiFlags.FCEV):
      if self.CP.flags & HyundaiFlags.FCEV:
        ret.gas = cp.vl["FCEV_ACCELERATOR"]["ACCELERATOR_PEDAL"] / 254.
      elif self.CP.flags & HyundaiFlags.HYBRID:
        ret.gas = cp.vl["E_EMS11"]["CR_Vcu_AccPedDep_Pos"] / 254.
      else:
        ret.gas = cp.vl["E_EMS11"]["Accel_Pedal_Pos"] / 254.
      ret.gasPressed = ret.gas > 0
    else:
      ret.gas = cp.vl["EMS12"]["PV_AV_CAN"] / 100.
      ret.gasPressed = bool(cp.vl["EMS16"]["CF_Ems_AclAct"])

    # Gear Selection via Cluster - For those Kia/Hyundai which are not fully discovered, we can use the Cluster Indicator for Gear Selection,
    # as this seems to be standard over all cars, but is not the preferred method.
    if self.CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      gear = cp.vl["ELECT_GEAR"]["Elect_Gear_Shifter"]
      ret.gearStep = cp.vl["ELECT_GEAR"]["Elect_Gear_Step"]
    elif self.CP.flags & HyundaiFlags.FCEV:
      gear = cp.vl["EMS20"]["HYDROGEN_GEAR_SHIFTER"]
    elif self.CP.flags & HyundaiFlags.CLUSTER_GEARS:
      gear = cp.vl["CLU15"]["CF_Clu_Gear"]
    elif self.CP.flags & HyundaiFlags.TCU_GEARS:
      gear = cp.vl["TCU12"]["CUR_GR"]
    else:
      gear = cp.vl["LVR12"]["CF_Lvr_Gear"]
      ret.gearStep = cp.vl["LVR11"]["CF_Lvr_GearInf"]

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

    if not self.CP.flags & HyundaiFlags.CC_ONLY_CAR and (not self.CP.openpilotLongitudinalControl or self.CP.flags & HyundaiFlags.CAMERA_SCC):
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
    prev_cruise_buttons = self.cruise_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    #carrot {{
    if self.CP.extFlags & HyundaiExtFlags.CRUISE_BUTTON_ALT.value and cp.vl_all["CRUISE_BUTTON_ALT"]["SET_ME_1"] == 1:
      self.cruise_buttons_alt = True

    cruise_button = [Buttons.NONE]
    if self.cruise_buttons_alt:
      lfa_button = cp.vl.get("CRUISE_BUTTON_LFA", {}).get("CruiseSwLfa", 0)
      cruise_button = [Buttons.LFA_BUTTON] if lfa_button > 0 else cp.vl_all["CRUISE_BUTTON_ALT"]["CruiseSwState"]
    elif self.CP.extFlags & HyundaiExtFlags.HAS_LFA_BUTTON.value and cp.vl.get("BCM_PO_11", {}).get("LFA_Pressed", 0):
      cruise_button = [Buttons.LFA_BUTTON]
    else:
      cruise_button = cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"]
    self.cruise_buttons.extend(cruise_button)
    # }} carrot
    prev_main_buttons = self.main_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwState"])
    if self.cruise_buttons_alt:
      self.main_buttons.extend(cp.vl_all["CRUISE_BUTTON_ALT"]["CruiseSwMain"])
    else:
      self.main_buttons.extend(cp.vl_all["CLU11"]["CF_Clu_CruiseSwMain"])
    self.mdps12 = copy.copy(cp.vl["MDPS12"])

    ret.buttonEvents = [*create_button_events(self.cruise_buttons[-1], prev_cruise_buttons, BUTTONS_DICT),
                        *create_button_events(self.main_buttons[-1], prev_main_buttons, {1: ButtonType.mainCruise})]


    if not self.CP.flags & HyundaiFlags.CC_ONLY_CAR:
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

    if self.CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value:
      speedLimit = cp.vl["Navi_HU"]["SpeedLim_Nav_Clu"]
      speedLimitCam = cp.vl["Navi_HU"]["SpeedLim_Nav_Cam"]
      ret.speedLimit = speedLimit if speedLimit < 255 and speedLimitCam == 1 else 0
      speed_limit_cam = speedLimitCam == 1
    else:
      ret.speedLimit = 0
      ret.speedLimitDistance = 0
      speed_limit_cam = False

    self.update_speed_limit(ret, speed_limit_cam)

    if prev_main_buttons == 0 and self.main_buttons[-1] != 0:
      self.main_enabled = not self.main_enabled

    return ret

  def update_speed_limit(self, ret, speed_limit_cam):
    self.totalDistance += ret.vEgo * DT_CTRL
    if ret.speedLimit > 0 and not ret.gasPressed and speed_limit_cam:
      if self.speedLimitDistance <= self.totalDistance:
        self.speedLimitDistance = self.totalDistance + ret.speedLimit * 6
      self.speedLimitDistance = max(self.totalDistance + 1, self.speedLimitDistance)
    else:
      self.speedLimitDistance = self.totalDistance
    ret.speedLimitDistance = self.speedLimitDistance - self.totalDistance

  def update_canfd(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

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

    if self.CP.extFlags & HyundaiExtFlags.CANFD_TPMS.value:
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

    ret.brakeLights = ret.brakePressed or (ret.aEgo < -0.5) ### TODO: �ӽ÷� brakeLight�� ������.

    ret.steeringRateDeg = cp.vl["STEERING_SENSORS"]["STEERING_RATE"]
    ret.steeringAngleDeg = cp.vl["STEERING_SENSORS"]["STEERING_ANGLE"] * -1
    ret.steeringTorque = cp.vl["MDPS"]["STEERING_COL_TORQUE"]
    ret.steeringTorqueEps = cp.vl["MDPS"]["STEERING_OUT_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > self.params.STEER_THRESHOLD, 5)
    ret.steerFaultTemporary = cp.vl["MDPS"]["LKA_FAULT"] != 0 or cp.vl["MDPS"]["LFA2_FAULT"] != 0
    #ret.steerFaultTemporary = False

    self.mdps_info = copy.copy(cp.vl["MDPS"])
    if self.CP.extFlags & HyundaiExtFlags.STEER_TOUCH:
      self.steer_touch_info = copy.copy(cp.vl["STEER_TOUCH_2AF"])

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
      cp_ = cp_cam if (self.CP.flags & HyundaiFlags.CAMERA_SCC and self.CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value) else cp
      #ret.leftBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0
      #ret.rightBlindspot = cp.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0
      if self.CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value:
        ret.leftBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_LEFT_FOUR"] != 0 or cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_LEFT_THREE"] != 0
        ret.rightBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_RIGHT_FOUR"] != 0 or cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_RIGHT_THREE"] != 0
      else:
        ret.leftBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["FL_INDICATOR"] != 0 or cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_LEFT_TWO"] != 0  or cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_LEFT_FOUR"] != 0
        ret.rightBlindspot = cp_.vl["BLINDSPOTS_REAR_CORNERS"]["FR_INDICATOR"] != 0 or cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_RIGHT_TWO"] != 0 or cp_.vl["BLINDSPOTS_REAR_CORNERS"]["INDICATOR_RIGHT_FOUR"] != 0

    # cruise state
    if cp.vl[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"] in [Buttons.RES_ACCEL, Buttons.SET_DECEL] and self.CP.openpilotLongitudinalControl:
      self.main_enabled = True
    # CAN FD cars enable on main button press, set available if no TCS faults preventing engagement
    ret.cruiseState.available = self.main_enabled #cp.vl["TCS"]["ACCEnable"] == 0
    if self.CP.flags & HyundaiFlags.CAMERA_SCC.value:
      self.MainMode_ACC = cp_cam.vl["SCC_CONTROL"]["MainMode_ACC"] == 1
      self.ACCMode = cp_cam.vl["SCC_CONTROL"]["ACCMode"]
      self.LFA_ICON = cp_cam.vl["LFAHDA_CLUSTER"]["LFA_ICON"]
      
    if self.CP.openpilotLongitudinalControl:
      # These are not used for engage/disengage since openpilot keeps track of state using the buttons
      ret.cruiseState.enabled = cp.vl["TCS"]["ACC_REQ"] == 1
      ret.cruiseState.standstill = False
      if self.MainMode_ACC:
        self.main_enabled = True
    else:
      cp_cruise_info = cp_cam if self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC else cp
      ret.cruiseState.enabled = cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] in (1, 2)
      if cp_cruise_info.vl["SCC_CONTROL"]["MainMode_ACC"] == 1: # carrot
        ret.cruiseState.available = self.main_enabled = True
        ret.pcmCruiseGap = int(np.clip(cp_cruise_info.vl["SCC_CONTROL"]["DISTANCE_SETTING"], 1, 4))
      ret.cruiseState.standstill = cp_cruise_info.vl["SCC_CONTROL"]["CRUISE_STANDSTILL"] == 1
      ret.cruiseState.speed = cp_cruise_info.vl["SCC_CONTROL"]["VSetDis"] * speed_factor
      self.cruise_info = copy.copy(cp_cruise_info.vl["SCC_CONTROL"])
      ret.brakeHoldActive = cp.vl["ESP_STATUS"]["AUTO_HOLD"] == 1 and cp_cruise_info.vl["SCC_CONTROL"]["ACCMode"] not in (1, 2)

    speed_limit_cam = False
    if self.CP.flags & HyundaiFlags.CAMERA_SCC.value:
      self.cruise_info = copy.copy(cp_cam.vl["SCC_CONTROL"])
      self.lfa_info = copy.copy(cp_cam.vl["LFA"])
      if self.CP.flags & HyundaiFlags.ANGLE_CONTROL.value:
        self.lfa_alt_info = copy.copy(cp_cam.vl["LFA_ALT"])
      self.lfahda_cluster_info = copy.copy(cp_cam.vl["LFAHDA_CLUSTER"])
      corner = False
      if self.CP.extFlags & HyundaiExtFlags.CANFD_161.value:
        if "ADRV_0x161" in cp_cam.vl:
          self.adrv_info_161 = copy.copy(cp_cam.vl.get("ADRV_0x161", {}))
        if "CCNC_0x162" in cp_cam.vl:
          self.adrv_info_162 = copy.copy(cp_cam.vl.get("CCNC_0x162", {}))
          ret.leftLongDist = self.lf_distance = cp_cam.vl["CCNC_0x162"]["LF_DETECT_DISTANCE"]
          ret.rightLongDist = self.rf_distance = cp_cam.vl["CCNC_0x162"]["RF_DETECT_DISTANCE"]
          self.lr_distance = cp_cam.vl["CCNC_0x162"]["LR_DETECT_DISTANCE"]
          self.rr_distance = cp_cam.vl["CCNC_0x162"]["RR_DETECT_DISTANCE"]
          ret.leftLatDist = cp_cam.vl["CCNC_0x162"]["LF_DETECT_LATERAL"]
          ret.rightLatDist = cp_cam.vl["CCNC_0x162"]["RF_DETECT_LATERAL"]
          corner = True
      if "ADRV_0x200" in cp_cam.vl:
        self.adrv_info_200 = copy.copy(cp_cam.vl.get("ADRV_0x200", {}))
      if "ADRV_0x1ea" in cp_cam.vl:
        self.adrv_info_1ea = copy.copy(cp_cam.vl.get("ADRV_0x1ea", {}))
        if not corner:
          ret.leftLongDist = cp_cam.vl["ADRV_0x1ea"]["LF_DETECT_DISTANCE"]
          ret.rightLongDist = cp_cam.vl["ADRV_0x1ea"]["RF_DETECT_DISTANCE"]
          ret.leftLatDist = cp_cam.vl["ADRV_0x1ea"]["LF_DETECT_LATERAL"]
          ret.rightLatDist = cp_cam.vl["ADRV_0x1ea"]["RF_DETECT_LATERAL"]
      if "ADRV_0x160" in cp_cam.vl:
        self.adrv_info_160 = copy.copy(cp_cam.vl.get("ADRV_0x160", {}))

      if "HDA_INFO_4A3" in cp.vl:
        self.hda_info_4a3 = copy.copy(cp.vl.get("HDA_INFO_4A3", {}))
        speedLimit = self.hda_info_4a3["SPEED_LIMIT"]
        if not self.is_metric:
          speedLimit *= CV.MPH_TO_KPH
        ret.speedLimit = speedLimit if speedLimit < 255 else 0
        if int(self.hda_info_4a3["NEW_SIGNAL_4"]) == 17:
          speed_limit_cam = True

      if "NEW_MSG_4B4" in cp.vl:
        self.new_msg_4b4 = copy.copy(cp.vl.get("NEW_MSG_4B4", {}))

      if "TCS" in cp.vl:
        self.tcs_info_373 = copy.copy(cp.vl.get("TCS", {}))

      if "CLUSTER_SPEED_LIMIT" in cp.vl:
        self.cluster_speed_limit_msg = copy.copy(cp.vl.get("CLUSTER_SPEED_LIMIT", {}))

    if "GEAR" in cp.vl:
      ret.gearStep = cp.vl["GEAR"]["GEAR_STEP"]
    elif "GEAR_ALT" in cp.vl:
      ret.gearStep = cp.vl["GEAR_ALT"]["GEAR_STEP"]

    # Manual Speed Limit Assist is a feature that replaces non-adaptive cruise control on EV CAN FD platforms.
    # It limits the vehicle speed, overridable by pressing the accelerator past a certain point.
    # The car will brake, but does not respect positive acceleration commands in this mode
    # TODO: find this message on ICE & HYBRID cars + cruise control signals (if exists)
    if self.CP.flags & HyundaiFlags.EV:
      ret.cruiseState.nonAdaptive = cp.vl["MANUAL_SPEED_LIMIT_ASSIST"]["MSLA_ENABLED"] == 1

    prev_cruise_buttons = self.cruise_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    #carrot {{

    if cp.vl[self.cruise_btns_msg_canfd]["LFA_BTN"]:
      cruise_button = [Buttons.LFA_BUTTON]
    else:
      cruise_button = cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]

    self.cruise_buttons.extend(cruise_button)
    # }} carrot


    if self.cruise_btns_msg_canfd in cp.vl:
      self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])
    """
    if self.cruise_btns_msg_canfd in cp.vl: #carrot
      if not cp.vl[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"]:
        pass
        #print("empty cruise btns...")
      else:
        self.cruise_buttons_msg = copy.copy(cp.vl[self.cruise_btns_msg_canfd])
     """
    prev_main_buttons = self.main_buttons[-1]
    #self.cruise_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["CRUISE_BUTTONS"])
    self.main_buttons.extend(cp.vl_all[self.cruise_btns_msg_canfd]["ADAPTIVE_CRUISE_MAIN_BTN"])
    if self.main_buttons[-1] != prev_main_buttons and not self.main_buttons[-1]: # and self.CP.openpilotLongitudinalControl: #carrot
      self.main_enabled = not self.main_enabled
      print("main_enabled = {}".format(self.main_enabled))
    self.buttons_counter = cp.vl[self.cruise_btns_msg_canfd]["COUNTER"]
    ret.accFaulted = cp.vl["TCS"]["ACCEnable"] != 0  # 0 ACC CONTROL ENABLED, 1-3 ACC CONTROL DISABLED

    if self.CP.flags & HyundaiFlags.CANFD_HDA2 and not (self.CP.flags & HyundaiFlags.CAMERA_SCC):
      self.hda2_lfa_block_msg = copy.copy(cp_cam.vl["CAM_0x362"] if self.CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING
                                          else cp_cam.vl["CAM_0x2a4"])

    speed_conv = CV.KPH_TO_MS # if self.is_metric else CV.MPH_TO_MS
    cluSpeed = cp.vl["CRUISE_BUTTONS_ALT"]["CLU_SPEED"]
    ret.vEgoCluster = cluSpeed  * speed_conv # MPH단위에서도 KPH로 나오는듯..
    vEgoClu, aEgoClu = self.update_clu_speed_kf(ret.vEgoCluster)
    ret.vCluRatio = (ret.vEgo / vEgoClu) if (vEgoClu > 3. and ret.vEgo > 3.) else 1.0

    self.update_speed_limit(ret, speed_limit_cam)

    paddle_button = self.paddle_button_prev
    if self.cruise_btns_msg_canfd == "CRUISE_BUTTONS":
      paddle_button = 1 if cp.vl["CRUISE_BUTTONS"]["LEFT_PADDLE"] == 1 else 2 if cp.vl["CRUISE_BUTTONS"]["RIGHT_PADDLE"] == 1 else 0
    elif self.gear_msg_canfd == "GEAR":
      paddle_button = 1 if cp.vl["GEAR"]["LEFT_PADDLE"] == 1 else 2 if cp.vl["GEAR"]["RIGHT_PADDLE"] == 1 else 0

    ret.buttonEvents = [*create_button_events(self.cruise_buttons[-1], prev_cruise_buttons, BUTTONS_DICT),
                        *create_button_events(paddle_button, self.paddle_button_prev, {1: ButtonType.paddleLeft, 2: ButtonType.paddleRight}),
                        *create_button_events(self.main_buttons[-1], prev_main_buttons, {1: ButtonType.mainCruise})]

    self.paddle_button_prev = paddle_button
    return ret

  def get_can_parsers_canfd(self, CP):
    pt_messages = [
      ("WHEEL_SPEEDS", 100),
      ("STEERING_SENSORS", 100),
      ("MDPS", 100),
      #("BRAKE", 100),
      ("ESP_STATUS", 100),
      ("TCS", 50),
      ("CRUISE_BUTTONS_ALT", 50),
      #("TPMS", 5),
      ("BLINKERS", 4),
      ("DOORS_SEATBELTS", 4),
    ]


    if CP.extFlags & HyundaiExtFlags.CANFD_TPMS.value:
      pt_messages += [
        ("TPMS", 5),
      ]

    if CP.flags & HyundaiFlags.EV:
      pt_messages += [
        ("ACCELERATOR", 100),
        ("MANUAL_SPEED_LIMIT_ASSIST", 10),
      ]
    else:
      pt_messages += [
        (self.gear_msg_canfd, 100),
        (self.accelerator_msg_canfd, 100),
      ]

    if not (CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
      pt_messages += [
        ("CRUISE_BUTTONS", 50)
      ]

    if CP.extFlags & HyundaiExtFlags.STEER_TOUCH:
      pt_messages += [
        ("STEER_TOUCH_2AF", 10),
      ]


    ## BSM신호가 ADAS인경우 BUS2로 개조되고, 독립인경우 ECAN에서 들어옴.
    # 개조, 독립 EV6: 1, 1 => True, inADAS: 1, 0 => False
    # 비개조, 0, 0 => True
    if CP.enableBsm:
      if (CP.flags & HyundaiFlags.CAMERA_SCC.value and CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value):
        pass
      else:
        pt_messages += [
          ("BLINDSPOTS_REAR_CORNERS", 20),
        ]

    if not (CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value) and not CP.openpilotLongitudinalControl:
      pt_messages += [
        ("SCC_CONTROL", 50),
      ]

    if CP.extFlags & HyundaiExtFlags.CANFD_4A3:
      pt_messages += [
        ("HDA_INFO_4A3", 5),
      ]
    #if CP.flags & HyundaiFlags.CANFD_HDA2 and CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value and not (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value):
    # 어떤차는 bus2에 있음, 내차는 bus0에 있는데.... 이건 옆두부와 관련이 없나?
    #if CP.flags & HyundaiFlags.CANFD_HDA2:
    #  pt_messages.append(("CLUSTER_SPEED_LIMIT", 10))
    if Params().get_int("CanfdDebug") > 0:
      pt_messages.append(("CLUSTER_SPEED_LIMIT", 10))

    cam_messages = []
    if CP.flags & HyundaiFlags.CANFD_HDA2 and not (CP.flags & HyundaiFlags.CAMERA_SCC.value):
      block_lfa_msg = "CAM_0x362" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "CAM_0x2a4"
      cam_messages += [(block_lfa_msg, 20)]
    if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC:
      cam_messages += [
        ("SCC_CONTROL", 50),
        ("LFA", 20),
        ("LFAHDA_CLUSTER", 20),
      ]
      if CP.flags & HyundaiFlags.CANFD_HDA2:
        cam_messages += [
          ("ADRV_0x200", 20),
          ("ADRV_0x1ea", 20),
          ("ADRV_0x160", 50),
        ]
      if CP.extFlags & HyundaiExtFlags.CANFD_161:
        cam_messages += [
          ("ADRV_0x161", 20),
          ("CCNC_0x162", 20),
        ]
      if CP.flags & HyundaiFlags.ANGLE_CONTROL:
        cam_messages += [
          ("LFA_ALT", 100),
        ]

    #if not (CP.flags & HyundaiFlags.CANFD_HDA2) and CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value and (CP.extFlags & HyundaiExtFlags.SCC_BUS2.value) :
    #  cam_messages.append(("CLUSTER_SPEED_LIMIT", 10))

    ## BSM신호가 ADAS인경우 BUS2로 개조되고, 독립인경우 ECAN에서 들어옴.
    # 개조, 독립 EV6: 1, 1 => False, inADAS: 1, 0 => True
    # 비개조, 0, 0 => False
    if CP.enableBsm:
      if (CP.flags & HyundaiFlags.CAMERA_SCC.value and CP.extFlags & HyundaiExtFlags.BSM_IN_ADAS.value):
        cam_messages += [
          ("BLINDSPOTS_REAR_CORNERS", 20),
        ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).ECAN),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).CAM),
    }

  def get_can_parsers(self, CP):
    if CP.flags & HyundaiFlags.CANFD:
      return self.get_can_parsers_canfd(CP)

    pt_messages = [
      # address, frequency
      ("MDPS12", 50),
      ("TCS11", 100),
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
    if CP.flags & HyundaiFlags.CC_ONLY_CAR:
      pt_messages.remove(("TCS11", 100))
      pt_messages.remove(("TCS13", 50))
      pt_messages.remove(("TCS15", 10))
      pt_messages.remove(("TPMS11", 5))

    if not CP.openpilotLongitudinalControl and not (CP.flags & HyundaiFlags.CAMERA_SCC) and not (CP.flags & HyundaiFlags.CC_ONLY_CAR):
      pt_messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.flags & HyundaiFlags.USE_FCA.value:
        pt_messages.append(("FCA11", 50))

    if CP.enableBsm:
      pt_messages.append(("LCA11", 50))

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      pt_messages.append(("E_EMS11", 50))
      pt_messages.append(("EV_Info", 50))
    elif CP.flags & HyundaiFlags.FCEV:
      pt_messages.append(("FCEV_ACCELERATOR", 100))
    else:
      pt_messages += [
        ("EMS12", 100),
        ("EMS16", 100),
      ]

    if CP.flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
      pt_messages.append(("ELECT_GEAR", 20))
    elif CP.flags & HyundaiFlags.FCEV:
      pt_messages.append(("EMS20", 100))
    elif CP.flags & HyundaiFlags.CLUSTER_GEARS:
      pass
    elif CP.flags & HyundaiFlags.TCU_GEARS:
      pt_messages.append(("TCU12", 100))
    else:
      pt_messages.append(("LVR12", 100))
      pt_messages.append(("LVR11", 100))

    if CP.extFlags & HyundaiExtFlags.HAS_LFA_BUTTON.value:
      pt_messages.append(("BCM_PO_11", 50))

    if CP.extFlags & HyundaiExtFlags.NAVI_CLUSTER.value:
      pt_messages.append(("Navi_HU", 5))

    if CP.extFlags & HyundaiExtFlags.CRUISE_BUTTON_ALT.value:
      pt_messages.append(("CRUISE_BUTTON_ALT", 6))
      pt_messages.append(("CRUISE_BUTTON_LFA", 6))


    cam_messages = [
      ("LKAS11", 100)
    ]
    if CP.flags & HyundaiFlags.CAMERA_SCC.value:
      cam_messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]
      if CP.extFlags & HyundaiExtFlags.HAS_SCC13.value:
        cam_messages += [
          ("SCC13", 0),
        ]
      if CP.extFlags & HyundaiExtFlags.HAS_SCC14.value:
        cam_messages += [
          ("SCC14", 50),
        ]
      if CP.flags & HyundaiFlags.USE_FCA.value:
        cam_messages += [
          ("FCA11", 50),
        ]

      if CP.extFlags & HyundaiExtFlags.HAS_LFAHDA.value:
        cam_messages += [("LFAHDA_MFC", 20)]


    if not CP.openpilotLongitudinalControl and CP.flags & HyundaiFlags.CAMERA_SCC:
      cam_messages += [
        ("SCC11", 50),
        ("SCC12", 50),
      ]

      if CP.flags & HyundaiFlags.USE_FCA.value:
        cam_messages.append(("FCA11", 50))


    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
    }
