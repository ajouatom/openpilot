import copy
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV

def hyundai_crc8(data: bytes) -> int:
  poly = 0x2F
  crc = 0xFF

  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x80:
        crc = ((crc << 1) ^ poly) & 0xFF
      else:
        crc = (crc << 1) & 0xFF

  return crc ^ 0xFF

class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, lka_steering=None) -> None:
    super().__init__(CP, fingerprint)

    if lka_steering is None:
      lka_steering = CP.flags & HyundaiFlags.CANFD_HDA2.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. LKA steering cars
    # have a different harness than the LFA steering variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if lka_steering and Params().get_int("HyundaiCameraSCC") == 0:  #배선개조는 무조건 Bus0가 ECAN임.
      self._a, self._e = 0, 1

    self._a += self.offset
    self._e += self.offset
    self._cam = 2 + self.offset

  @property
  def ECAN(self):
    return self._e

  @property
  def ACAN(self):
    return self._a

  @property
  def CAM(self):
    return self._cam

# CAN LIST (CAM)  - 롱컨개조시... ADAS + CAM
# 160: ADRV_0x160
# 1da: ADRV_0x1da
# 1ea: ADRV_0x1ea
# 200: ADRV_0x200
# 345: ADRV_0x345
# 1fa: CLUSTER_SPEED_LIMIT
# 12a: LFA
# 1e0: LFAHDA_CLUSTER
# 11a:
# 1b5:
# 1a0: SCC_CONTROL

# CAN LIST (ACAN)
# 160: ADRV_0x160
# 51: ADRV_0x51
# 180: CAM_0x180
# ...
# 185: CAM_0x185
# 1b6: CAM_0x1b6
# ...
# 1b9: CAM_0x1b9
# 1fb: CAM_0x1fb
# 2a2 - 2a4
# 2bb - 2be
# LKAS
# 201 - 2a0



def create_steering_messages_camera_scc(frame, packer, CP, CAN, CC, lat_active, apply_steer, CS, apply_angle, max_torque, angle_control):

  emergency_steering = False
  if CS.adrv_info_161 is not None:
    values = CS.adrv_info_161
    emergency_steering = values["ALERTS_1"] in [11, 12, 13, 14, 15, 21, 22, 23, 24, 25, 26]


  ret = []
  values = copy.copy(CS.mdps_info)
  if angle_control:
    if CS.lfa_alt_info is not None:
      values["LFA2_ACTIVE"] = CS.lfa_alt_info["LKAS_ANGLE_ACTIVE"]
  else:
    if CS.lfa_info is not None:
      values["LKA_ACTIVE"] = 1 if CS.lfa_info["STEER_REQ"] == 1 else 0

  if frame % 1000 < 40:
    values["STEERING_COL_TORQUE"] += 220
  ret.append(packer.make_can_msg("MDPS", CAN.CAM, values))

  if frame % 10 == 0:
    if CS.steer_touch_info is not None:
      values = copy.copy(CS.steer_touch_info)
      if frame % 1000 < 40:
        values["TOUCH_DETECT"] = 3
        values["TOUCH1"] = 50
        values["TOUCH2"] = 50
        values["CHECKSUM_"] = 0
        dat = packer.make_can_msg("STEER_TOUCH_2AF", 0, values)[1]
        values["CHECKSUM_"] = hyundai_crc8(dat[1:8])

      ret.append(packer.make_can_msg("STEER_TOUCH_2AF", CAN.CAM, values))

  if angle_control:
    if emergency_steering:
      values = copy.copy(CS.lfa_alt_info)
    else:
      values = {} #CS.lfa_alt_info
      values["LKAS_ANGLE_ACTIVE"] = 2 if CC.latActive else 1
      values["LKAS_ANGLE_CMD"] = -apply_angle
      values["LKAS_ANGLE_MAX_TORQUE"] = max_torque if CC.latActive else 0
    ret.append(packer.make_can_msg("LFA_ALT", CAN.ECAN, values))

    values = copy.copy(CS.lfa_info)
    if not emergency_steering:
      values["LKA_MODE"] = 0
      values["LKA_ICON"] = 2 if CC.latActive else 1
      values["TORQUE_REQUEST"] = -1024  # apply_steer,
      values["VALUE63"] = 0 # LKA_ASSIST
      values["STEER_REQ"] = 0  # 1 if lat_active else 0,
      values["HAS_LANE_SAFETY"] = 0  # hide LKAS settings
      values["LKA_ACTIVE"] = 3 if CC.latActive else 0  # this changes sometimes, 3 seems to indicate engaged
      values["VALUE64"] = 0  #STEER_MODE, NEW_SIGNAL_2
      values["LKAS_ANGLE_CMD"] = -25.6 #-apply_angle,
      values["LKAS_ANGLE_ACTIVE"] = 0 #2 if lat_active else 1,
      values["LKAS_ANGLE_MAX_TORQUE"] = 0 #max_torque if lat_active else 0,
      values["NEW_SIGNAL_1"] = 10

  else:
    values = {}
    values["LKA_MODE"] = 2
    values["LKA_ICON"] = 2 if lat_active else 1
    values["TORQUE_REQUEST"] = apply_steer
    values["STEER_REQ"] = 1 if lat_active else 0
    values["VALUE64"] = 0  # STEER_MODE, NEW_SIGNAL_2
    values["HAS_LANE_SAFETY"] = 0
    values["LKA_ACTIVE"] = 0 # NEW_SIGNAL_1

    values["DampingGain"] = 0 if lat_active else 100  
    #values["VALUE63"] = 0

    #values["VALUE82_SET256"] = 0

  ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret

def create_steering_messages(packer, CP, CAN, enabled, lat_active, apply_steer, apply_angle, max_torque, angle_control):

  ret = []
  if angle_control:
    values = {
      "LKA_MODE": 0,
      "LKA_ICON": 2 if enabled else 1,
      "TORQUE_REQUEST": 0,  # apply_steer,
      "VALUE63": 0, # LKA_ASSIST
      "STEER_REQ": 0,  # 1 if lat_active else 0,
      "HAS_LANE_SAFETY": 0,  # hide LKAS settings
      "LKA_ACTIVE": 3 if lat_active else 0,  # this changes sometimes, 3 seems to indicate engaged
      "VALUE64": 0,  #STEER_MODE, NEW_SIGNAL_2
      "LKAS_ANGLE_CMD": -apply_angle,
      "LKAS_ANGLE_ACTIVE": 2 if lat_active else 1,
      "LKAS_ANGLE_MAX_TORQUE": max_torque if lat_active else 0,

      # test for EV6PE
      "NEW_SIGNAL_1": 10, #2,
      "DampingGain": 9,
      "VALUE231": 146,
      "VALUE239": 1,
      "VALUE247": 255,
      "VALUE255": 255,
    }
  else:
    values = {
      "LKA_MODE": 2,
      "LKA_ICON": 2 if enabled else 1,
      "TORQUE_REQUEST": apply_steer,
      "DampingGain": 3 if enabled else 100,
      "STEER_REQ": 1 if lat_active else 0,
      #"STEER_MODE": 0,
      "HAS_LANE_SAFETY": 0,  # hide LKAS settings
      "VALUE63": 0,
      "VALUE64": 0,
    }

  if CP.flags & HyundaiFlags.CANFD_HDA2:
    lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    if not (CP.flags & HyundaiFlags.CAMERA_SCC.value):
      ret.append(packer.make_can_msg(lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret

def create_suppress_lfa(packer, CAN, CS):
  if CS.msg_0x362 is not None:
    suppress_msg = "CAM_0x362"
    lfa_block_msg = CS.msg_0x362
  elif CS.msg_0x2a4 is not None:
    suppress_msg = "CAM_0x2a4"
    lfa_block_msg = CS.msg_0x2a4
  else:
    return []

  #values = {f"BYTE{i}": lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values = copy.copy(lfa_block_msg)
  values["COUNTER"] = lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return [packer.make_can_msg(suppress_msg, CAN.ACAN, values)]

def create_buttons(packer, CP, CAN, cnt, btn):
  values = {
    "COUNTER": cnt,
    "SET_ME_1": 1,
    "CRUISE_BUTTONS": btn,
  }

  #bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_HDA2 else CAN.CAM
  bus = CAN.ECAN
  return packer.make_can_msg("CRUISE_BUTTONS", bus, values)

def create_acc_cancel(packer, CP, CAN, cruise_info_copy):
  # TODO: why do we copy different values here?
  if CP.flags & HyundaiFlags.CANFD_CAMERA_SCC.value:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "NEW_SIGNAL_1",
      "MainMode_ACC",
      "ACCMode",
      "ZEROS_9",
      "CRUISE_STANDSTILL",
      "ZEROS_5",
      "DISTANCE_SETTING",
      "VSetDis",
    ]}
  else:
    values = {s: cruise_info_copy[s] for s in [
      "COUNTER",
      "CHECKSUM",
      "ACCMode",
      "VSetDis",
      "CRUISE_STANDSTILL",
    ]}
  values.update({
    "ACCMode": 4,
    "aReqRaw": 0.0,
    "aReqValue": 0.0,
  })
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)

def create_lfahda_cluster(packer, CS, CAN, long_active, lat_active):

  if CS.lfahda_cluster_info is not None:
    values = {} #
    values["HDA_CntrlModSta"] = 2 if long_active else 0
    values["HDA_LFA_SymSta"] = 2 if lat_active else 0

    # 
  else:
    return []
  return [packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)]


def create_acc_control_scc2(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, hyundai_jerk, CS):
  enabled = (enabled or CS.softHoldActive > 0) and CS.paddle_button_prev == 0

  acc_mode = 0 if not enabled else (2 if gas_override else 1)

  if hyundai_jerk.carrot_cruise == 1:
    acc_mode = 4 if enabled else 0
    enabled = False
    accel = accel_last = 0.5
   
  elif hyundai_jerk.carrot_cruise == 2:
    accel = accel_last = hyundai_jerk.carrot_cruise_accel

  jerk_u = hyundai_jerk.jerk_u
  jerk_l = hyundai_jerk.jerk_l
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)

  values = copy.copy(CS.cruise_info)
  values["ACCMode"] = acc_mode
  values["MainMode_ACC"] = 1
  values["StopReq"] = 1 if stopping or CS.softHoldActive > 0 else 0  # 1: Stop control is required, 2: Not used, 3: Error Indicator
  values["aReqValue"] = a_val
  values["aReqRaw"] = a_raw
  values["VSetDis"] = set_speed
  #values["JerkLowerLimit"] = jerk if enabled else 1
  #values["JerkUpperLimit"] = 3.0
  values["JerkLowerLimit"] = jerk_l if enabled else 1
  values["JerkUpperLimit"] = 2.0 if stopping or CS.softHoldActive else jerk_u
  values["DISTANCE_SETTING"] = hud_control.leadDistanceBars # + 5
  #values["DISTANCE_SETTING"] = hud_control.leadDistanceBars  + 5

  #values["ACC_ObjDist"] = 1
  #values["ObjValid"] = 0
  #values["OBJ_STATUS"] =  2
  values["NSCCOper"] = 1 if enabled else 0 # 0: off, 1: Ready, 2: Act, 3: Error Indicator
  values["NSCCOnOff"] = 2  # 0: Default, 1: Off, 2: On, 3: Invalid
  #values["SET_ME_3"] = 0x3  # objRelsped와 충돌
  #values["ACC_ObjLatPos"] = - hud_control.leadDPath
  values["DriveMode"] = 0 # 0: Default, 1: Comfort Mode, 2:Normal mode, 3:Dynamic mode, reserved

  hud_lead_info = 0
  if hud_control.leadVisible:
    hud_lead_info = 1 if values["ACC_ObjRelSpd"] > 0 else 2
  values["HUD_LEAD_INFO"] = hud_lead_info  #1: in-path object detected(uncontrollable), 2: controllable long, 3: controllable long & lat, ... reserved

  values["DriverAlert"] = 0   # 1: SCC Disengaged, 2: No SCC Engage condition, 3: SCC Disenganed when the vehicle stops

  values["TARGET_DISTANCE"] = CS.out.vEgo * 1.0 + 4.0

  soft_hold_info = 1 if CS.softHoldActive > 1 and enabled else 0

  # 이거안하면 정지중 뒤로 밀리는 현상 발생하는듯.. (신호정지중에 뒤로 밀리는 경험함.. 시험해봐야)
  if values["InfoDisplay"] != 5: #5: Front Car Departure Notice
    values["InfoDisplay"] = 4 if stopping and CS.out.aEgo > -0.3 else 0  # 1: SCC Mode, 2: Convention Cruise Mode, 3: Object disappered at low speed, 4: Available to resume acceleration control, 5: Front vehicle departure notice, 6: Reserved, 7: Invalid

  values["TakeOverReq"] = 0    # 1: Takeover request, 2: Not used, 3: Error indicator , 이것이 켜지면 가속을 안하는듯함.
  #values["NEW_SIGNAL_4"] = 9 if hud_control.leadVisible else 0
  # AccelLimitBandUpper, Lower
  values["SysFailState"] = 0    # 1: Performance degredation, 2: system temporairy unavailble, 3: SCC Service required , 눈이 묻어 레이더오류시... 2가 됨. 이때 가속을 안함...

  values["AccelLimitBandUpper"] = 0.0   # 이값이 1.26일때 가속을 안하는 증상이 보임.. 
  values["AccelLimitBandLower"] = 0.0

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)

def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, jerk_u, jerk_l, CS):

  enabled = enabled or CS.softHoldActive > 0
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = np.clip(accel, accel_last - jn, accel_last + jn)

  values = {
    "ACCMode": 0 if not enabled else (2 if gas_override else 1),
    "MainMode_ACC": 1,
    "StopReq": 1 if stopping or CS.softHoldActive > 0 else 0,
    "aReqValue": a_val,
    "aReqRaw": a_raw,
    "VSetDis": set_speed,
    #"JerkLowerLimit": jerk if enabled else 1,
    #"JerkUpperLimit": 3.0,
    "JerkLowerLimit": jerk_l if enabled else 1,
    "JerkUpperLimit": jerk_u,

    "ACC_ObjDist": 1,
    #"ObjValid": 0,
    #"OBJ_STATUS": 2,
    "NSCCOper": 0,
    "NSCCOnOff": 2,
    "DriveMode": 0,
    #"SET_ME_3": 0x3,
    "ACC_ObjLatPos": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars, # + 5,
    "InfoDisplay": 4 if stopping and CS.out.cruiseState.standstill else 0,
  }

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)


def create_spas_messages(packer, CAN, frame, left_blink, right_blink):
  ret = []

  values = {
  }
  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, values))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  values = {
    "BLINKER_CONTROL": blink,
  }
  ret.append(packer.make_can_msg("SPAS2", CAN.ECAN, values))

  return ret


def create_fca_warning_light(CP, packer, CAN, frame):
  ret = []
  if CP.flags & HyundaiFlags.CAMERA_SCC.value:
    return ret

  if frame % 2 == 0:
    values = {
      'AEB_SETTING': 0x1,  # show AEB disabled icon
      'SET_ME_2': 0x2,
      'SET_ME_FF': 0xff,
      'SET_ME_FC': 0xfc,
      'SET_ME_9': 0x9,
      #'DATA102': 1,
    }
    ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))
  return ret

def create_tcs_messages(packer, CAN, CS):
  ret = []
  if CS.tcs_info_373 is not None:
    values = copy.copy(CS.tcs_info_373)
    values["DriverBraking"] = 0
    values["DriverBrakingLowSens"] = 0
    ret.append(packer.make_can_msg("TCS", CAN.CAM, values))
  return ret

def forward_button_message(packer, CAN, frame, CS, cruise_button, MainMode_ACC_trigger, LFA_trigger):
  ret = []
  if frame % 2 == 0:
    if CS.cruise_buttons_msg is not None:
      values = copy.copy(CS.cruise_buttons_msg)
      cruise_button_driver = values["CRUISE_BUTTONS"]
      if cruise_button_driver == 0:
        values["CRUISE_BUTTONS"] = cruise_button
      if MainMode_ACC_trigger > 0:
        #values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1
        pass
      elif LFA_trigger > 0:
        values["LFA_BTN"] = 1

      ret.append(packer.make_can_msg(CS.cruise_btns_msg_canfd, CAN.CAM, values))
  return ret

def create_ccnc_messages(CP, packer, CAN, frame, CC, CS, hud_control, disp_angle, left_lane_warning, right_lane_warning, canfd_debug, MainMode_ACC_trigger, LFA_trigger):
  ret = []

  if CP.flags & HyundaiFlags.CAMERA_SCC.value:
    if frame % 2 == 0:
      if CS.adrv_info_160 is not None:
        values = copy.copy(CS.adrv_info_160)
        #values["NEW_SIGNAL_1"] = 0 # steer_temp관련없음, 계기판에러
        #values["SET_ME_9"] = 17 # steer_temp관련없음, 계기판에러
        #values["SET_ME_2"] = 0   #커멘트해도 steer_temp에러남, 2값은 콤마에서 찾은거니...
        #values["DATA102"] = 0  # steer_temp관련없음
        ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

      if CS.cruise_buttons_msg is not None:
        values = copy.copy(CS.cruise_buttons_msg)
        if MainMode_ACC_trigger > 0:
          values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1
        elif LFA_trigger > 0:
          values["LFA_BTN"] = 1
        ret.append(packer.make_can_msg(CS.cruise_btns_msg_canfd, CAN.CAM, values))


    if frame % 5 == 0:
      if CS.adrv_info_161 is not None:
        main_enabled = CS.out.cruiseState.available
        cruise_enabled = CC.enabled
        lat_enabled = CS.out.latEnabled
        lat_active = CC.latActive
        nav_active = hud_control.activeCarrot > 1

        # hdpuse carrot
        hdp_use = int(Params().get("HDPuse"))
        hdp_active = False
        if hdp_use == 1:
            hdp_active = cruise_enabled and nav_active
        elif hdp_use == 2:
            hdp_active = cruise_enabled
        # hdpuse carrot

        values = copy.copy(CS.adrv_info_161)
        #print("adrv_info_161 = ", CS.adrv_info_161)

        values["SETSPEED"] = (6 if hdp_active else 3 if cruise_enabled else 1) if main_enabled else 0
        values["SETSPEED_HUD"] = (5 if hdp_active else 3 if cruise_enabled else 1) if main_enabled else 0
        set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
        values["vSetDis"] = int(set_speed_in_units + 0.5)

        values["DISTANCE"] = 4 if hdp_active else hud_control.leadDistanceBars
        values["DISTANCE_LEAD"] = 2 if cruise_enabled and hud_control.leadVisible else 1 if main_enabled and hud_control.leadVisible else 0
        values["DISTANCE_CAR"] = 3 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["DISTANCE_SPACING"] = 5 if hdp_active else 1 if cruise_enabled else 0

        values["TARGET"] = 1 if main_enabled else 0
        values["TARGET_DISTANCE"] = int(hud_control.leadDistance)

        values["BACKGROUND"] = 6 if CS.paddle_button_prev > 0 else 1 if cruise_enabled else 3 if main_enabled else 7
        values["CENTERLINE"] = 1 if lat_enabled else 0
        values["CAR_CIRCLE"] = 2 if hdp_active else 1 if cruise_enabled else 0

        values["NAV_ICON"] = 2 if nav_active else 0
        values["HDA_ICON"] = 5 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["LFA_ICON"] = 5 if hdp_active else 2 if lat_active else 1 if lat_enabled else 0
        values["LKA_ICON"] = 4 if lat_active else 3 if lat_enabled else 0
        values["FCA_ALT_ICON"] = 0

        if values["ALERTS_2"] in [1, 2, 5]:
          values["ALERTS_2"] = 0
          values["DAW_ICON"] = 0

        values["SOUNDS_2"] = 0  # 2: STEER중지 경고후에도 사운드가 나옴.
        values["SOUNDS_4"] = 0  # 차선변경알림? 에이 그냥0으로..

        if values["ALERTS_3"] in [3, 4, 13, 17, 19, 26, 7, 8, 9, 10]:
          values["ALERTS_3"] = 0
          values["SOUNDS_3"] = 0

        if values["ALERTS_5"] in [1, 2, 4, 5]:
          values["ALERTS_5"] = 0

        if values["ALERTS_5"] in [11] and CS.softHoldActive == 0:
          values["ALERTS_5"] = 0

        curvature = round(CS.out.steeringAngleDeg / 3)

        values["LANELINE_CURVATURE"] = (min(abs(curvature), 15) + (-1 if curvature < 0 else 0)) if lat_active else 0
        values["LANELINE_CURVATURE_DIRECTION"] = 1 if curvature < 0 and lat_active else 0

        if hud_control.leftLaneDepart:
          values["LANELINE_LEFT"] = 4 if (frame // 50) % 2 == 0 else 1
        else:
          values["LANELINE_LEFT"] = 2 if hud_control.leftLaneVisible else 0
        if hud_control.rightLaneDepart:
          values["LANELINE_RIGHT"] = 4 if (frame // 50) % 2 == 0 else 1
        else:
          values["LANELINE_RIGHT"] = 2 if hud_control.rightLaneVisible else 0
        #values["LANELINE_LEFT_POSITION"] = 15
        #values["LANELINE_RIGHT_POSITION"] = 15

        values["LCA_LEFT_ARROW"] = 2 if CS.out.leftBlinker else 0
        values["LCA_RIGHT_ARROW"] = 2 if CS.out.rightBlinker else 0

        values["LCA_LEFT_ICON"] = 1 if CS.out.leftBlindspot else 2
        values["LCA_RIGHT_ICON"] = 1 if CS.out.rightBlindspot else 2

        ret.append(packer.make_can_msg("ADRV_0x161", CAN.ECAN, values))

      if CS.adrv_info_200 is not None:
        values = copy.copy(CS.adrv_info_200)
        values["TauGapSet"] = hud_control.leadDistanceBars
        ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

      if CS.adrv_info_1ea is not None:
        values = copy.copy(CS.adrv_info_1ea)
        #values["HDA_MODE1"] = 8
        #values["HDA_MODE2"] = 1
        if values['LF_DETECT'] == 0 and hud_control.leadLeftDist > 0:
          values['LF_DETECT'] = 3 if hud_control.leadLeftDist > 30 else 4
          values['LF_DETECT_DISTANCE'] = hud_control.leadLeftDist
          values['LF_DETECT_LATERAL'] = hud_control.leadLeftLat
        if values['RF_DETECT'] == 0 and hud_control.leadRightDist > 0:
          values['RF_DETECT'] = 3 if hud_control.leadRightDist > 30 else 4
          values['RF_DETECT_DISTANCE'] = hud_control.leadRightDist
          values['RF_DETECT_LATERAL'] = hud_control.leadRightLat
        ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      if CS.adrv_info_162 is not None:
        values = copy.copy(CS.adrv_info_162)
        if hud_control.leadDistance > 0:
          values["FF_DISTANCE"] = hud_control.leadDistance
          #values["FF_DETECT"] = 11 if hud_control.leadRelSpeed > -0.1 else 12  # bicycle
          #values["FF_DETECT"] = 5 if hud_control.leadRelSpeed > -0.1 else 6 # truck
          ff_type = 3 if hud_control.leadRadar == 1 else 13
          values["FF_DETECT"] = ff_type if hud_control.leadRelSpeed > -0.1 else ff_type + 1
          #values["FF_DETECT_LAT"] = - hud_control.leadDPath

        if True:
          if values['LF_DETECT'] == 0 and hud_control.leadLeftDist > 0:
            values['LF_DETECT'] = 3 if hud_control.leadLeftDist > 30 else 4
            values['LF_DETECT_DISTANCE'] = hud_control.leadLeftDist
            values['LF_DETECT_LATERAL'] = hud_control.leadLeftLat
          if values['RF_DETECT'] == 0 and hud_control.leadRightDist > 0:
            values['RF_DETECT'] = 3 if hud_control.leadRightDist > 30 else 4
            values['RF_DETECT_DISTANCE'] = hud_control.leadRightDist
            values['RF_DETECT_LATERAL'] = hud_control.leadRightLat
        else:
          sensors = [
            ('lf', 'LF_DETECT'),
            ('rf', 'RF_DETECT'),
            ('lr', 'LR_DETECT'),
            ('rr', 'RR_DETECT')
          ]

          for sensor_key, detect_key in sensors:
            distance = getattr(CS, f"{sensor_key}_distance")
            if distance > 0:
              values[detect_key] = 3 if distance > 30 else 4

        """
        values["FAULT_FCA"] = 0
        values["FAULT_LSS"] = 0
        values["FAULT_LFA"] = 0
        values["FAULT_LCA"] = 0
        values["FAULT_DAS"] = 0
        values["FAULT_HDA"] = 0
        """

        if (left_lane_warning and not CS.out.leftBlinker) or (right_lane_warning and not CS.out.rightBlinker):
          values["VIBRATE"] = 1
        ret.append(packer.make_can_msg("CCNC_0x162", CAN.ECAN, values))

    if canfd_debug > 0:
      if frame % 20 == 0: # 아직 시험중..
        if CS.hda_info_4a3 is not None:
          values = copy.copy(CS.hda_info_4a3)
          #if canfd_debug == 1:
          values["SIGNAL_0"] = 5
          values["NEW_SIGNAL_1"] = 4
          values["SPEED_LIMIT"] = 80
          values["NEW_SIGNAL_3"] = 154
          values["NEW_SIGNAL_4"] = 9
          values["NEW_SIGNAL_5"] = 0
          values["NEW_SIGNAL_6"] = 256
          ret.append(packer.make_can_msg("HDA_INFO_4A3", CAN.CAM, values))

  return ret

def create_adrv_messages(CP, packer, CAN, frame):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  if not CP.flags & HyundaiFlags.CAMERA_SCC.value:
    values = {}

    ret.extend(create_fca_warning_light(CP, packer, CAN, frame))
    if frame % 5 == 0:
      values = {
        'HDA_MODE1': 0x8,
        'HDA_MODE2': 0x1,
        #'SET_ME_1C': 0x1c,
        'SET_ME_FF': 0xff,
        #'SET_ME_TMP_F': 0xf,
        #'SET_ME_TMP_F_2': 0xf,
        #'DATA26': 1,  #1
        #'DATA32': 5,  #5
      }
      ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      values = {
        'SET_ME_E1': 0xe1,
        #'SET_ME_3A': 0x3a,
        'TauGapSet' : 1,
        'NEW_SIGNAL_2': 3,
      }
      ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

    if frame % 20 == 0:
      values = {
        'SET_ME_15': 0x15,
      }
      ret.append(packer.make_can_msg("ADRV_0x345", CAN.ECAN, values))

    if frame % 100 == 0:
      values = {
        'SET_ME_22': 0x22,
        'SET_ME_41': 0x41,
      }
      ret.append(packer.make_can_msg("ADRV_0x1da", CAN.ECAN, values))

  return ret

## carrot
def alt_cruise_buttons(packer, CP, CAN, buttons, cruise_btns_msg, cnt):
  cruise_btns_msg["CRUISE_BUTTONS"] = buttons
  cruise_btns_msg["COUNTER"] = (cruise_btns_msg["COUNTER"] + 1 + cnt) % 256
  bus = CAN.ECAN if CP.flags & HyundaiFlags.CANFD_HDA2 else CAN.CAM
  return packer.make_can_msg("CRUISE_BUTTONS_ALT", bus, cruise_btns_msg)
  
def hkg_can_fd_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(2, len(d)):
    crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ d[i]]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 0) & 0xFF)]) & 0xFFFF
  crc = ((crc << 8) ^ CRC16_XMODEM[(crc >> 8) ^ ((address >> 8) & 0xFF)]) & 0xFFFF
  if len(d) == 8:
    crc ^= 0x5F29
  elif len(d) == 16:
    crc ^= 0x041D
  elif len(d) == 24:
    crc ^= 0x819D
  elif len(d) == 32:
    crc ^= 0x9F5B
  return crc
