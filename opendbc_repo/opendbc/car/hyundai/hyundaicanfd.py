from opendbc.car import CanBusBase
from opendbc.car.common.numpy_fast import clip
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params


class CanBus(CanBusBase):
  def __init__(self, CP, fingerprint=None, hda2=None) -> None:
    super().__init__(CP, fingerprint)

    if hda2 is None:
      hda2 = CP.flags & HyundaiFlags.CANFD_HDA2.value if CP is not None else False

    # On the CAN-FD platforms, the LKAS camera is on both A-CAN and E-CAN. HDA2 cars
    # have a different harness than the HDA1 and non-HDA variants in order to split
    # a different bus, since the steering is done by different ECUs.
    self._a, self._e = 1, 0
    if hda2 and Params().get_int("HyundaiCameraSCC") == 0:  #배선개조는 무조건 Bus0가 ECAN임.
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


#                     EV6      K8   IONIQ5      CANIVAL
#  OFF:GEN:HIGHWAY
# LFA
#  LKA_MODE          2:2:2    6:6:6      K8     0:0:0
#                             > 7이 되는경우?
#  VALUE27           0:0:0    0:3:3      K8     0:0:0
#                    K8의 경우 차선이 없으면 0이됨... 카니발은?
#  STEER_REQ         0:1:1    ==         ==     ==
#                    속도가 0이더라도 STR_REQ를 0으로 하지 않음..
#                    하지만, STR_REQ를 0으로 하는경우가 있음.. 이때  TORQUE_REQUEST를 0으로 하지 않음(VALUE104는 100으로 출력함)
#  VALUE104          100:3:3  100:xx:xx  K8     100:xx:xx
#                              xx: cluspeed + 60 (정확하지는 않지만 속도를 따라감)
#  VALUE63           0:0:0    0:0:0      K8     0:0:0
#  VALUE64           0:0:0    0:0:0      K8     0:0:0
#  HAS_LANE_SAFETY   0:0:0    1:1:1      K8     0:0:0
#          LaneSafety를 의미하는것은 아닌것 같음.

# LKAS                                          LKAS_ALT
#  LKA_MODE          2:2:2    6:6:6     K8      2:2:2
#  VALUE27           0:0:3    0:3:3     0:0:0(?) 0:0:0
#  LKA_ASSIST        0:0:0    0:0:0     K8      0:0:0
#  VALUE64           0:0:0    100:xx:xx K8      0:0:0
#  HAS_LANE_SAFETY   1:1:1    1:1:1     K8      0:0:0
#  VALUE104          0:0:0    0:0:0     K8      0:0:0

# 0x1ea
#  HDA_MODE1         8:8:8    8:8:8     K8      8:8:8
#  HDA_MODE2         0:0:1    0:0:1(??) 0:0:1   0:0:1

def create_steering_messages_camera_scc(packer, CP, CAN, enabled, lat_active, apply_steer, CS, apply_angle, max_torque, angle_control):

  ret = []
  if angle_control:
    # EV9(ADRV)
    #203(0xcb), 298(0x12a), 352(0x160), 416(0x1a0), 282(0x11a), 437(0x1b5), 506(0x1fa),
    #698(0x2ba), 353(0x161), 354(0x162), 442(0x1ba), 480(0x1e0), 485(0x1e5), 490(0x1ea),
    #512(0x200), 837(0x345), 908(0x38c), 1402(0x57a), 474(0x1da)
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
      # a torque scale value? ramps up when steering, highest seen is 234
      # "UNKNOWN": 50 if lat_active and not steering_pressed else 0,
      "UNKNOWN": max_torque if lat_active else 0,
    }

  else:

    values = CS.lfa_info
    value_104 = 100 if not lat_active else 60 + CS.out.vEgo * 3.6

    canival_mode = True
    k8_mode = False
    if True:
      values = {}
      values["LKA_MODE"] = 2
      values["LKA_ICON"] = 2 if lat_active else 1
      values["TORQUE_REQUEST"] = apply_steer
      values["STEER_REQ"] = 1 if lat_active else 0
      values["VALUE64"] = 0  # STEER_MODE, NEW_SIGNAL_2
      values["HAS_LANE_SAFETY"] = 0
      values["LKA_ACTIVE"] = 0 # NEW_SIGNAL_1

      #values["VALUE63"] = 0

      #values["VALUE104"] = 3 if lat_active else 100
      #values["VALUE82_SET256"] = 0
    elif canival_mode:
      values["LKA_ICON"] = 2 if enabled else 1
      values["TORQUE_REQUEST"] = apply_steer
      values["STEER_REQ"] = 1 if lat_active else 0
      values["VALUE63"] = 0
      values["VALUE64"] = 0

      values["LKA_MODE"] = 0
      values["LKA_ACTIVE"] = 0
      values["HAS_LANE_SAFETY"] = 0
      values["VALUE104"] = 3 if lat_active else 100
      values["VALUE82_SET256"] = 0
      values["NEW_SIGNAL_1"] = 0
    elif k8_mode: # ioniq5
      values["LKA_ICON"] = 2 if enabled else 1
      values["TORQUE_REQUEST"] = apply_steer
      values["STEER_REQ"] = 1 if lat_active else 0
      values["VALUE63"] = 0
      values["VALUE64"] = 0

      values["LKA_MODE"] = 6
      values["LKA_ACTIVE"] = 3
      values["HAS_LANE_SAFETY"] = 1
      values["VALUE104"] = value_104
      values["VALUE82_SET256"] = 0
      values["NEW_SIGNAL_1"] = 0
    else:
      values["LKA_ICON"] = 2 if enabled else 1
      values["TORQUE_REQUEST"] = apply_steer
      values["STEER_REQ"] = 1 if lat_active else 0
      values["VALUE63"] = 0
      values["VALUE64"] = 0

      values["LKA_MODE"] = 2
      values["LKA_ACTIVE"] = 0
      values["HAS_LANE_SAFETY"] = 0
      values["VALUE104"] = 3 if enabled else 100
      values["VALUE82_SET256"] = 256
      values["NEW_SIGNAL_1"] = 0

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
      # a torque scale value? ramps up when steering, highest seen is 234
      # "UNKNOWN": 50 if lat_active and not steering_pressed else 0,
      "UNKNOWN": max_torque if lat_active else 0,
    }
  else:
    values = {
      "LKA_MODE": 2,
      "LKA_ICON": 2 if enabled else 1,
      "TORQUE_REQUEST": apply_steer,
      "VALUE104": 3 if enabled else 100,
      "STEER_REQ": 1 if lat_active else 0,
      #"STEER_MODE": 0,
      "HAS_LANE_SAFETY": 0,  # hide LKAS settings
      "VALUE63": 0,
      "VALUE64": 0,
    }

  if CP.flags & HyundaiFlags.CANFD_HDA2:
    hda2_lkas_msg = "LKAS_ALT" if CP.flags & HyundaiFlags.CANFD_HDA2_ALT_STEERING else "LKAS"
    if CP.openpilotLongitudinalControl:
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))
    if not (CP.flags & HyundaiFlags.CAMERA_SCC.value):
      ret.append(packer.make_can_msg(hda2_lkas_msg, CAN.ACAN, values))
  else:
    ret.append(packer.make_can_msg("LFA", CAN.ECAN, values))

  return ret

def create_suppress_lfa(packer, CAN, hda2_lfa_block_msg, hda2_alt_steering):
  suppress_msg = "CAM_0x362" if hda2_alt_steering else "CAM_0x2a4"
  msg_bytes = 32 if hda2_alt_steering else 24

  values = {f"BYTE{i}": hda2_lfa_block_msg[f"BYTE{i}"] for i in range(3, msg_bytes) if i != 7}
  values["COUNTER"] = hda2_lfa_block_msg["COUNTER"]
  values["SET_ME_0"] = 0
  values["SET_ME_0_2"] = 0
  values["LEFT_LANE_LINE"] = 0
  values["RIGHT_LANE_LINE"] = 0
  return packer.make_can_msg(suppress_msg, CAN.ACAN, values)

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

def create_lfahda_cluster(packer, CAN, enabled):
  values = {
    "HDA_ICON": 1 if enabled else 0,
    "LFA_ICON": 2 if enabled else 0,
  }
  return packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values)


def create_acc_control_scc2(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, jerk_u, jerk_l, CS):
  enabled = enabled or CS.softHoldActive > 0
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = clip(accel, accel_last - jn, accel_last + jn)

  values = CS.cruise_info
  values["ACCMode"] = 0 if not enabled else (2 if gas_override else 1)
  values["MainMode_ACC"] = 1
  values["StopReq"] = 1 if stopping or CS.softHoldActive > 0 else 0
  values["aReqValue"] = a_val
  values["aReqRaw"] = a_raw
  values["VSetDis"] = set_speed
  #values["JerkLowerLimit"] = jerk if enabled else 1
  #values["JerkUpperLimit"] = 3.0
  values["JerkLowerLimit"] = jerk_l if enabled else 1
  values["JerkUpperLimit"] = jerk_u
  values["DISTANCE_SETTING"] = hud_control.leadDistanceBars # + 5

  #values["ACC_ObjDist"] = 1
  #values["ObjValid"] = 0
  #values["OBJ_STATUS"] =  2
  values["SET_ME_2"] = 0x4
  #values["SET_ME_3"] = 0x3  # objRelsped와 충돌
  values["SET_ME_TMP_64"] = 0x64

  values["NEW_SIGNAL_3"] = 1 if hud_control.leadVisible else 0 #0  # 1이되면 차선이탈방지 알람이 뜬다고...  => 앞에 차가 있으면, 1또는 2가 됨. 전방두부?

  #values["NEW_SIGNAL_4"] = 2

  values["ZEROS_5"] = 0

  values["NEW_SIGNAL_15_DESIRE_DIST"] = CS.out.vEgo * 1.0 + 4.0

  values["CRUISE_STANDSTILL"] = 1 if stopping and CS.out.aEgo > -0.1 else 0

  values["NEW_SIGNAL_2"] = 0    # 이것이 켜지면 가속을 안하는듯함.
  #values["NEW_SIGNAL_4"] = 0    # signal2와 조합하여.. 앞차와 깜박이등이 인식되는것 같음..

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values)

def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, jerk_u, jerk_l, CS):

  enabled = enabled or CS.softHoldActive > 0
  jerk = 5
  jn = jerk / 50
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = clip(accel, accel_last - jn, accel_last + jn)

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
    "SET_ME_2": 0x4,
    #"SET_ME_3": 0x3,
    "SET_ME_TMP_64": 0x64,
    "DISTANCE_SETTING": hud_control.leadDistanceBars, # + 5,
    "CRUISE_STANDSTILL": 1 if stopping and CS.out.cruiseState.standstill else 0,
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


def create_adrv_messages(CP, packer, CAN, frame, CC, CS, hud_control):
  # messages needed to car happy after disabling
  # the ADAS Driving ECU to do longitudinal control

  ret = []

  values = {
  }
  if CP.flags & HyundaiFlags.CAMERA_SCC.value:
    if frame % 5 == 0:
      if CP.extFlags & HyundaiExtFlags.CANFD_161.value:
        if CS.adrv_info_161 is not None:
          main_enabled = CS.out.cruiseState.available
          cruise_enabled = CC.enabled
          values = CS.adrv_info_161
          #print("adrv_info_161 = ", CS.adrv_info_161)
          values["vSetDis"] = int(hud_control.setSpeed * 3.6 + 0.5)
          values["GAP_DIST_SET"] = hud_control.leadDistanceBars

          values["WHEEL_ICON"] = 2 if CC.latActive else 1

          values["CRUISE_INFO6_SET3"] = 3 if cruise_enabled else 0
          values["CRUISE_INFO1_SET2"] = 2 if cruise_enabled else 1 if main_enabled else 0
          values["CRUISE_INFO2_SET2"] = 2 if cruise_enabled else 1 if main_enabled else 0
          values["CRUISE_INFO4_SET3"] = 3 if cruise_enabled else 0
          values["CRUISE_INFO8_SET1"] = 1 if main_enabled else 0
          values["CRUISE_INFO5_SET1"] = 1 if cruise_enabled else 0
          values["SET4_HWAY_ELSE_3"] = 3

          values["START_READY_INFO_MAYBE"] = 0

          values["NEW_SIGNAL_7"] = 0
          values["NEW_SIGNAL_5"] = 0
          values["LANE_ASSIST_CONCERNED"] = 0
          values["LANE_ASSIST_GREEN"] = 0

          #values["CRUISE_INFO10_SET1"] = 1
          #values["CRUISE_INFO11_SET1"] = 1

          values["AUTO_LANE_CHANGE_MESSAGE_SET6"] = 0 # 1: 핸들잡아, 2: 빨리잡아, 6: 자동차선변경준비.

          values["CRUISE_INFO7_HWAY_SET2_ELSE_0"] = 0
          values["CRUISE_INFO9_HWAY_SET2_ELSE_0"] = 0
          #values["NEW_SIGNAL_HWAY_SET1_ELSE_0"] = 1

          values["CRUISE_INFO10_0_TO_4"] = 0 #4 if main_enabled else 0      # message
          values["CRUISE_INFO11_0_TO_1"] = 0 #1 if cruise_enabled else 0    # message
          values["143_SET_0"] = 0

          #LANE_ASSIST_L,R: 0:OFF, 1: GREY, 2: GREEN, 4: WHITE
          values["LANE_ASSIST_L"] = 2
          values["LANE_ASSIST_R"] = 2

          values["NEW_SIGNAL_12"] = 0   ## 띠링 경고

          ret.append(packer.make_can_msg("ADRV_0x161", CAN.ECAN, values))
        else:
          print("no adrv_info_161")

      if CS.adrv_info_200 is not None:
        values = CS.adrv_info_200
        values["TauGapSet"] = hud_control.leadDistanceBars
        ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values))

      if CS.adrv_info_1ea is not None:
        values = CS.adrv_info_1ea
        values["HDA_MODE1"] = 8
        values["HDA_MODE2"] = 1
        ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values))

      if CS.adrv_info_160 is not None:
        values = CS.adrv_info_160
        values["NEW_SIGNAL_1"] = 0 # steer_temp관련없음, 계기판에러
        values["SET_ME_9"] = 17 # steer_temp관련없음, 계기판에러
        values["SET_ME_2"] = 0   #커멘트해도 steer_temp에러남, 2값은 콤마에서 찾은거니...
        values["DATA102"] = 0  # steer_temp관련없음
        ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

      if CS.adrv_info_162 is not None:
        values = CS.adrv_info_162
        values["SIGNAL216"] = 0
        values["SIGNAL219"] = 0   # steer temp.. 발생?
        values["SIGNAL234"] = 0
        values["SIGNAL240"] = 0
        values["SIGNAL246"] = 0
        ret.append(packer.make_can_msg("CORNER_RADAR_HIGHWAY", CAN.ECAN, values))

    if frame % 20 == 0 and False: # 아직 시험중..
      if CS.hda_info_4a3 is not None:
        values = CS.hda_info_4a3
        # SIGNAL_4: 7, SIGNAL_0: 0 으로 해도 .. 옆두부는 나오기도 함.. 아오5
        values["SIGNAL_4"] = 10 if CC.enabled else 0   # 0, 5(고속도로진입), 10(고속도로), 7,5(국도에서 간혹), 0,10(카니발)      , 5(고속도로진입,EV6), 11(고속도로,EV6)
        values["SIGNAL_0"] = 5 if CC.enabled else 0  # 0, 2(고속도로진입), 1(고속도로),                      5(카니발은 항상)  , 2(고속도로진입,EV6), 1(고속도로,EV6)
        values["NEW_SIGNAL_1"] = 4
        values["NEW_SIGNAL_2"] = 0
        values["NEW_SIGNAL_3"] = 154
        values["NEW_SIGNAL_4"] = 9
        values["NEW_SIGNAL_5"] = 0
        values["NEW_SIGNAL_6"] = 256
        values["NEW_SIGNAL_7"] = 0
        ret.append(packer.make_can_msg("HDA_INFO_4A3", CAN.CAM, values))
    if frame % 10 == 0:
      if CS.new_msg_4b4 is not None: #G80 HDA2개조차량은 안나옴...
        values = CS.new_msg_4b4
        values["NEW_SIGNAL_4"] = 146
        values["NEW_SIGNAL_5"] = 72
        values["NEW_SIGNAL_6"] = 44
        ret.append(packer.make_can_msg("NEW_MSG_4B4", CAN.CAM, values))
    return ret
  else:
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
