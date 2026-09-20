import copy
import math
import numpy as np
from opendbc.car import CanBusBase
from opendbc.car.carlog import carlog
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiExtFlags
from openpilot.common.params import Params
from opendbc.car.common.conversions import Conversions as CV
from openpilot.cereal import log

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

ACC_CONTROL_DT = 1.0 / 50.0


def longitudinal_interlock_active(CS) -> bool:
  return CS.out.brakeHoldActive or CS.out.parkingBrake


def apply_accel_jerk_limit(a_raw: float, a_value_last: float, jerk_u: float, jerk_l: float,
                           dt: float = ACC_CONTROL_DT) -> float:
  """Ramp aReqValue toward aReqRaw using the asymmetric stock SCC jerk limits."""
  upper_step = max(0.0, float(jerk_u)) * dt
  lower_step = max(0.0, float(jerk_l)) * dt
  return float(np.clip(a_raw, a_value_last - lower_step, a_value_last + upper_step))


def apply_stopping_experiment(values, CS, controller, accel, previous_value, jerk_u, jerk_l):
  """Apply the stopping/re-entry sequence after the normal SCC interlocks."""
  if controller is None:
    return

  wheels = CS.out.wheelSpeeds
  speeds = [CS.out.vEgo, CS.out.vEgoRaw, wheels.fl, wheels.fr, wheels.rl, wheels.rr]
  finite = all(math.isfinite(v) for v in (*speeds, accel, previous_value, jerk_u, jerk_l))
  speed = max(abs(v) for v in speeds) if finite else 0.0
  blocked = (not finite or not CS.out.canValid or CS.out.brakePressed or CS.out.gasPressed
             or str(CS.out.gearShifter) != "drive" or longitudinal_interlock_active(CS))
  previous_phase = controller.phase
  command = controller.update(
    active=values["ACCMode"] == 1 and not blocked, requested=bool(values["StopReq"]), speed=speed,
    held=CS.canfdSccHoldActive, accel=accel, previous_value=previous_value,
    jerk_u=max(0.0, min(jerk_u, 5.0)), jerk_l=max(1.0, min(jerk_l, 5.0)),
  )
  if blocked or values["ACCMode"] != 1:
    values.update(StopReq=0, aReqRaw=0.0, aReqValue=0.0)
    if not finite:
      values.update(ACCMode=0, JerkUpperLimit=1.0, JerkLowerLimit=1.0)
  elif command is not None:
    values.update(StopReq=command.stop_req, aReqRaw=command.raw, aReqValue=command.value,
                  AccelLimitBandUpper=0.0, AccelLimitBandLower=command.lower)

  if controller.phase != previous_phase:
    carlog.warning({"event": "carrot_stopping", "from": str(previous_phase), "phase": str(controller.phase),
                    "reason": controller.reason, "speed": speed, "aEgo": CS.out.aEgo,
                    "held": CS.canfdSccHoldActive, "retry_used": controller.retried,
                    "StopReq": values["StopReq"], "aReqRaw": values["aReqRaw"], "aReqValue": values["aReqValue"]})


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
  if CS.adrv_0x161 is not None:
    values = CS.adrv_0x161
    emergency_steering = values["ALERTS_1"] in [11, 12, 13, 14, 15, 21, 22, 23, 24, 25, 26]


  ret = []
  if CS.mdps is not None:
    values = copy.copy(CS.mdps)
    #rx_counter = values.pop("COUNTER", None)
    if angle_control:
      if CS.lfa_alt is not None:
        values["LFA2_ACTIVE"] = CS.lfa_alt["LKAS_ANGLE_ACTIVE"]
    else:
      if CS.lfa is not None:
        values["LKA_ACTIVE"] = 1 if CS.lfa["STEER_REQ"] == 1 else 0

    if frame % 1000 < 40:
      values["STEERING_COL_TORQUE"] += 220
    #ret.append(packer.make_can_msg("MDPS", CAN.CAM, values, rx_counter = rx_counter))
    ret.append(packer.make_can_msg("MDPS", CAN.CAM, values))

  if frame % 10 == 0:
    if CS.steer_touch_2af is not None:
      values = copy.copy(CS.steer_touch_2af)
      if frame % 1000 < 40:
        values["TOUCH_DETECT"] = 3
        values["TOUCH1"] = 50
        values["TOUCH2"] = 50
        values["CHECKSUM_"] = 0
        dat = packer.make_can_msg("STEER_TOUCH_2AF", 0, values)[1]
        values["CHECKSUM_"] = hyundai_crc8(dat[1:8])

      ret.append(packer.make_can_msg("STEER_TOUCH_2AF", CAN.CAM, values))

  if angle_control:
    if CS.lfa_alt is not None:
      values = copy.copy(CS.lfa_alt)
      rx_counter = values.pop("COUNTER", None)
      if emergency_steering:
        pass
      else:
        #values = {} #CS.lfa_alt
        values["LKAS_ANGLE_ACTIVE"] = 2 if CC.latActive else 1
        values["LKAS_ANGLE_CMD"] = -apply_angle
        values["LKAS_ANGLE_MAX_TORQUE"] = max_torque if CC.latActive else 0
      ret.append(packer.make_can_msg("LFA_ALT", CAN.ECAN, values, rx_counter = rx_counter))

    if CS.lfa is not None:
      values = copy.copy(CS.lfa)
      rx_counter = values.pop("COUNTER", None)
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
      ret.append(packer.make_can_msg("LFA", CAN.ECAN, values, rx_counter = rx_counter))

  elif CS.lfa is not None:
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
      "DampingGain": 100, #3 if enabled else 100,
      "STEER_REQ": 1 if lat_active else 0,
      #"STEER_MODE": 0,
      "HAS_LANE_SAFETY": 0,  # hide LKAS settings
      "VALUE63": 0,
      "VALUE64": 100,
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
  if CS.cam_0x362 is not None:
    suppress_msg = "CAM_0x362"
    lfa_block_msg = CS.cam_0x362
  elif CS.cam_0x2a4 is not None:
    suppress_msg = "CAM_0x2a4"
    lfa_block_msg = CS.cam_0x2a4
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


  if CS.lfahda_cluster is not None:
    values = copy.copy(CS.lfahda_cluster)
    rx_counter = values.pop("COUNTER", None)
  else:
    return []
    values = {}
    rx_counter = None
    values["LFA_OptUsmSta"] = 2
    values["HDA_OptUsmSta"] = 2
  values["HDA_CntrlModSta"] = 2 if long_active else 0
  values["HDA_LFA_SymSta"] = 2 if lat_active else 0
  return [packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values, rx_counter=rx_counter)]

def create_lfa_icon_non_camera_scc(packer, CS, CAN, CC):
  ret = []
  if CS.adrv_0x161 is not None:
    values = copy.copy(CS.adrv_0x161)
    rx_counter = values.pop("COUNTER", None)

    lat_active = CC.latActive
    lat_enabled = CS.out.latEnabled

    values["LFA_ICON"] = 2 if lat_active else 1 if lat_enabled else 0
    values["LKA_ICON"] = 4 if lat_active else 3 if lat_enabled else 0

    if values["ALERTS_2"] in [1, 2, 5, 6, 10, 21, 22]:
      values["ALERTS_2"] = 0
      values["DAW_ICON"] = 0

    if values["ALERTS_1"] == 0:
      values["SOUNDS_1"] = 0
      values["SOUNDS_2"] = 0
      values["SOUNDS_4"] = 0

    if values["ALERTS_3"] in [3, 4, 11, 12, 13, 14, 17, 19, 26, 7, 8, 9, 10]:
      values["ALERTS_3"] = 0
      values["SOUNDS_3"] = 0

    if values["ALERTS_5"] in [1, 2, 3, 4, 5]:
      values["ALERTS_5"] = 0

    ret.append(packer.make_can_msg("ADRV_0x161", CAN.ECAN, values, rx_counter=rx_counter))
  return ret

def _display_lead(radar_state):
  # Vehicle displays show the nearest valid control lead, including leadTwo.
  # Equal distances retain leadOne; this does not change radar/control roles.
  leads = (getattr(radar_state, name, None) for name in ("leadOne", "leadTwo"))
  return min((lead for lead in leads
              if lead is not None and lead.status and lead.dRel > 0
              and all(math.isfinite(v) for v in (lead.dRel, lead.yRel, lead.vRel))),
             key=lambda lead: lead.dRel, default=None)


def _display_lead_lateral(lead, model_v2):
  # Model position and the cluster use right-positive y; radar yRel is left-positive.
  lateral = -lead.yRel
  position = getattr(model_v2, "position", None)
  if position is not None:
    x, y = position.x, position.y
    if (len(x) >= 2 and len(x) == len(y)
        and all(math.isfinite(v) for v in x) and all(math.isfinite(v) for v in y)
        and all(x[i] > x[i - 1] for i in range(1, len(x)))):
      lateral -= float(np.interp(lead.dRel, x, y))
  return lateral


def _apply_scc_lead(values, radar_state, model_v2=None):
  lead = _display_lead(radar_state)
  # Match the stock no-object encoding; never retain an old camera target.
  values.update(ACC_ObjDist=204.6, ACC_ObjLatPos=0.0, ACC_ObjRelSpd=239.4, HUD_LEAD_INFO=0)
  if lead is not None:
    values["ACC_ObjDist"] = float(np.clip(lead.dRel, 0.1, 204.5))
    # Show the offset from the model path at the lead distance. Bound to
    # the signed 9-bit signal's representable range (0.1 scale, -20 offset).
    values["ACC_ObjLatPos"] = float(np.clip(_display_lead_lateral(lead, model_v2), -45.6, 5.5))
    values["ACC_ObjRelSpd"] = float(np.clip(lead.vRel, -170.0, 239.3))
    values["HUD_LEAD_INFO"] = 1 if lead.vRel > 0 else 2


def create_acc_control_scc2(packer, CAN, enabled, accel_value_last, accel, stopping, gas_override, set_speed, hud_control, hyundai_jerk, CS,
                            stop_controller=None):

  if CS.scc_control is None:
    if stop_controller is not None:
      stop_controller.reset()
    return None, accel_value_last
  interlock_active = longitudinal_interlock_active(CS)
  soft_hold_active = CS.softHoldActive > 0 and CS.out.cruiseState.available
  acc_control_enabled = (enabled or soft_hold_active) and CS.out.cruiseState.available and CS.paddle_button_prev == 0 and not interlock_active
  enabled = acc_control_enabled

  acc_mode = 0 if not enabled else (2 if gas_override else 1)

  if hyundai_jerk.carrot_cruise == 1:
    acc_mode = 4 if enabled else 0
    enabled = False
    accel = accel_value_last = 0.5

  elif hyundai_jerk.carrot_cruise == 2:
    accel = accel_value_last = hyundai_jerk.carrot_cruise_accel

  jerk_u = 2.0 if stopping or soft_hold_active else hyundai_jerk.jerk_u
  jerk_l = hyundai_jerk.jerk_l
  if not enabled or gas_override:
    a_val, a_raw = 0, 0
  else:
    a_raw = accel
    a_val = apply_accel_jerk_limit(a_raw, accel_value_last, jerk_u, jerk_l)

  values = copy.copy(CS.scc_control)
  rx_counter = values.pop("COUNTER", None)
  values["ACCMode"] = acc_mode
  values["MainMode_ACC"] = 1
  values["StopReq"] = 1 if acc_control_enabled and (stopping or soft_hold_active) else 0  # 1: Stop control is required, 2: Not used, 3: Error Indicator
  values["aReqValue"] = a_val
  values["aReqRaw"] = a_raw
  values["VSetDis"] = set_speed
  #values["JerkLowerLimit"] = jerk if enabled else 1
  #values["JerkUpperLimit"] = 3.0
  values["JerkLowerLimit"] = jerk_l if enabled else 1
  values["JerkUpperLimit"] = jerk_u
  values["DISTANCE_SETTING"] = hud_control.leadDistanceBars # + 5
  #values["DISTANCE_SETTING"] = hud_control.leadDistanceBars  + 5

  #values["ObjValid"] = 0
  #values["OBJ_STATUS"] =  2
  #values["NSCCOper"] = 1 if enabled else 0 # 0: off, 1: Ready, 2: Act, 3: Error Indicator
  #values["NSCCOnOff"] = 2  # 0: Default, 1: Off, 2: On, 3: Invalid
  #values["SET_ME_3"] = 0x3  # objRelsped와 충돌
  values["DriveMode"] = 0 # 0: Default, 1: Comfort Mode, 2:Normal mode, 3:Dynamic mode, reserved

  _apply_scc_lead(values, getattr(CS, "radarState", None), getattr(CS, "modelV2", None))

  values["DriverAlert"] = 0   # 1: SCC Disengaged, 2: No SCC Engage condition, 3: SCC Disenganed when the vehicle stops

  values["TARGET_DISTANCE"] = CS.out.vEgo + 4.0 if math.isfinite(CS.out.vEgo) else 4.0

  if stop_controller is not None:
    values["InfoDisplay"] = 0
  elif values["InfoDisplay"] != 5:
    values["InfoDisplay"] = 4 if not interlock_active and stopping and CS.out.aEgo > -0.3 else 0

  values["TakeOverReq"] = 0    # 1: Takeover request, 2: Not used, 3: Error indicator , 이것이 켜지면 가속을 안하는듯함.
  #values["NEW_SIGNAL_4"] = 9 if hud_control.leadVisible else 0
  # AccelLimitBandUpper, Lower
  values["SysFailState"] = 0    # 1: Performance degredation, 2: system temporairy unavailble, 3: SCC Service required , 눈이 묻어 레이더오류시... 2가 됨. 이때 가속을 안함...

  values["AccelLimitBandUpper"] = 0.0   # 이값이 1.26일때 가속을 안하는 증상이 보임..
  values["AccelLimitBandLower"] = 0.0

  values["ZEROS_7"] = 0 if stop_controller is not None else 1
  apply_stopping_experiment(values, CS, stop_controller, accel, accel_value_last, jerk_u, jerk_l)

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values), values["aReqValue"]

def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, jerk_u, jerk_l, CS,
                       stop_controller=None):

  interlock_active = longitudinal_interlock_active(CS)
  soft_hold_active = CS.softHoldActive > 0 and CS.out.cruiseState.available
  acc_control_enabled = (enabled or soft_hold_active) and CS.out.cruiseState.available and not interlock_active
  enabled = acc_control_enabled
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
    "StopReq": 1 if acc_control_enabled and (stopping or soft_hold_active) else 0,
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
    "InfoDisplay": 0 if stop_controller is not None else (
      4 if not interlock_active and stopping and CS.out.cruiseState.standstill else 0),
    "ZEROS_7": 0,
  }

  apply_stopping_experiment(values, CS, stop_controller, accel, accel_last, jerk_u, jerk_l)
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
  if CS.tcs is not None:
    values = copy.copy(CS.tcs)
    #rx_counter = values.pop("COUNTER", None)
    values["DriverBraking"] = 0
    values["NEW_SIGNAL_20"] = 0
    values["NEW_SIGNAL_11"] = 0
    values["DriverBrakingLowSens"] = 0
    #values["NEW_SIGNAL_1"] = 0 # accel과 관련..  옆두부 꺼지는것과 관련? 확인필요
    #values["ACC_REQ"] = 1 # 옆두부 꺼지는것과 관련? 확인필요.. 항상 켜지게함..
    values["NEW_SIGNAL_1"] = 0 if values["ACC_REQ"] == 1 else 1 # 옆두부..
    #ret.append(packer.make_can_msg("TCS", CAN.CAM, values, rx_counter = rx_counter))
    ret.append(packer.make_can_msg("TCS", CAN.CAM, values))
  return ret

def forward_button_message(packer, CAN, frame, CS, cruise_button, MainMode_ACC_trigger, LFA_trigger):
  ret = []
  if frame % 2 == 0:
    if CS.cruise_buttons_msg is not None:
      values = copy.copy(CS.cruise_buttons_msg)
      # A held MAIN is reported on this bit and switches some clusters to LIMIT mode.
      values["NORMAL_CRUISE_MAIN_BTN"] = 0
      #rx_counter = values.pop("COUNTER", None)
      cruise_button_driver = values["CRUISE_BUTTONS"]
      if cruise_button_driver == 0:
        values["CRUISE_BUTTONS"] = cruise_button
      if MainMode_ACC_trigger > 0:
        #values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1
        pass
      elif LFA_trigger > 0:
        values["LFA_BTN"] = 1

      #ret.append(packer.make_can_msg(CS.cruise_btns_msg_canfd, CAN.CAM, values, rx_counter = rx_counter))
      ret.append(packer.make_can_msg(CS.cruise_btns_msg_canfd, CAN.CAM, values))
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
        #'HDA_MODE1': 0x8,
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


def _get_desire_and_lane_changing(md):
  desire = 0
  lane_changing = 0
  if md is not None:
    desire = md.meta.desire.raw
    ds = md.meta.desireState
    if len(ds) > 4:
      if ds[1] > 0.9: lane_changing = 1
      if ds[2] > 0.9: lane_changing = 2
      if ds[3] > 0.9: lane_changing = 3
      if ds[4] > 0.9: lane_changing = 4
  return desire, lane_changing

def _apply_lane_desire(values, desire):
  #values['LANE_CHANGING'] = 0

  if desire == 1:  # 좌회전
    values['LANE_CHANGING'] = 1
    values["LANELINE_CURVATURE"] = 15
    values["LANELINE_CURVATURE_DIRECTION"] = 0

  elif desire == 2:  # 우회전
    values['LANE_CHANGING'] = 2
    values["LANELINE_CURVATURE"] = 15
    values["LANELINE_CURVATURE_DIRECTION"] = 1

  elif desire == 3:  # 좌차선변경
    values['LANE_CHANGING'] = 3

  elif desire == 4:  # 우차선변경
    values['LANE_CHANGING'] = 4

def _suppress_trailer_mode_warning(values, CS):
  # Logs from IONIQ 9 show ALERTS_5=6 is the periodic
  # "driver assistance limited in trailer mode" popup.
  if CS.trailer_connected and values.get("ALERTS_5") == 6:
    values["ALERTS_5"] = 0


def _hide_replaced_adas_service_warning(values):
  # Openpilot replaces the stock lane-change/highway-driving control path, so
  # the camera can latch their service-required flags during low-speed turns.
  # Preserve blocked-sensor warnings and unrelated DAS faults. The original
  # camera-side message remains available in logcan for diagnosis.
  service_warning_hidden = False
  for fault in ("FAULT_LCA", "FAULT_HDA"):
    if values.get(fault) == 1:
      values[fault] = 0
      service_warning_hidden = True

  if service_warning_hidden and values.get("FAULT_DAS") == 1:
    values["FAULT_DAS"] = 0


def _select_cluster_background(cruise_enabled, lat_active, paddle_pressed, paddle_mode):
  if paddle_mode > 0 and paddle_pressed:
    return 6
  return 1 if cruise_enabled else 3 if lat_active else 7


def _apply_cluster_lane_lines(values, CS, lat_active, desire):
  curvature = round(CS.out.steeringAngleDeg / 3)
  mag = min(abs(curvature), 15)
  curv = mag + (-1 if curvature < 0 else 0)
  direction = 1 if curvature < 0 else 0
  values["LANELINE_CURVATURE"] = curv if lat_active else 0
  values["LANELINE_CURVATURE_DIRECTION"] = direction if lat_active else 0
  if desire:
    _apply_lane_desire(values, desire)


def _convert_ccnc_boxes_to_cars(values):
  # Only 0x162 uses 1/2 for gray/white boxes and 3/4 for gray/white cars.
  # 0x1ea has different display enums; FF_DETECT_ALT has no car enum.
  for key in ("FF_DETECT", "LF_DETECT", "RF_DETECT", "LR_DETECT", "RR_DETECT"):
    if values[key] in (1, 2):
      values[key] += 2


def _apply_ccnc_lead(values, radar_state, enabled, model_v2=None):
  lead = _display_lead(radar_state)
  if lead is None:
    values.update(FF_DETECT=0, FF_DISTANCE=204.6, FF_LATERAL=0.0)
    return

  # The DBC exposes FF_LATERAL as unsigned, but OEM negative positions are
  # encoded in 7-bit two's complement (e.g. 11.6 represents -1.2 m).
  stock_lateral = values["FF_LATERAL"]
  if stock_lateral >= 6.4:
    stock_lateral -= 12.8
  lateral = _display_lead_lateral(lead, model_v2)
  same_object = (values["FF_DETECT"] != 0
                 and abs(values["FF_DISTANCE"] - lead.dRel) <= 3.0
                 and abs(stock_lateral - lateral) <= 1.0)
  # Radar leads have no object class. Retain the OEM class only for a matching
  # target; otherwise use the existing generic gray/white car presentation.
  if not same_object:
    values["FF_DETECT"] = 4 if enabled else 3
  values["FF_DISTANCE"] = float(np.clip(lead.dRel, 0.1, 204.5))
  values["FF_LATERAL"] = float(np.clip(lateral, -6.4, 6.3))


def create_ccnc_messages(CP, packer, CAN, frame, CC, CS, hud_control,
                         disp_angle, left_lane_warning, right_lane_warning,
                         enable_corner_radar, stopping, canfd_debug, paddle_mode):
  ret = []
  interlock_active = longitudinal_interlock_active(CS)
  display_lead = _display_lead(getattr(CS, "radarState", None))
  lead_visible = display_lead is not None
  lead_distance = float(np.clip(display_lead.dRel, 0.1, 204.5)) if lead_visible else 0.0

  md = CS.modelV2
  if not hasattr(create_ccnc_messages, '_lane_line_check') or frame % 100 == 0:
    create_ccnc_messages._lane_line_check = Params().get_int("LaneLineCheck")
  lane_line_check = create_ccnc_messages._lane_line_check
  desire, lane_changing = _get_desire_and_lane_changing(md)

  if CP.flags & HyundaiFlags.CAMERA_SCC.value:
    HDA_CntrlModSta = 0
    HDA_LFA_SymSta = 0
    if CS.lfahda_cluster is not None:
      HDA_CntrlModSta = CS.lfahda_cluster["HDA_CntrlModSta"]
      HDA_LFA_SymSta = CS.lfahda_cluster["HDA_LFA_SymSta"]

    if frame % 2 == 0:
      #if CS.adrv_0x160 is not None:
      #  values = copy.copy(CS.adrv_0x160)
      #  ret.append(packer.make_can_msg("ADRV_0x160", CAN.ECAN, values))

      if CS.cruise_buttons_msg is not None:
        values = copy.copy(CS.cruise_buttons_msg)
        # Keep the physical long press on ECAN for CarState, but don't forward it to CAM.
        values["NORMAL_CRUISE_MAIN_BTN"] = 0

        if  HDA_LFA_SymSta == 0 and 0 < frame % 200 < 12:
          values["LFA_BTN"] = 1

        if CC.enabled and not interlock_active:
          if not CS.MainMode_ACC:
            if 10 < frame % 200 <= 16 and CS.out.vEgo > 3.:
              values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1
          elif CS.ACCMode in [0, 4]:
            if 10 < frame % 200 <= 16 and CS.out.vEgo > 3.:
              values["CRUISE_BUTTONS"] = 2
          elif CS.scc_control is not None and CS.scc_control["InfoDisplay"] == 4:
            if 10 < frame % 30 <= 16 and not stopping:
              values["CRUISE_BUTTONS"] = 2
          else:
            if CS.adrv_0x1ea is not None and CS.adrv_0x1ea["HDA_MODE2"] == 0: # if corner radar is disabled, send main btn
              if 10 < frame % 1000 <= 16 and CS.out.vEgo > 3:
                values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1

        ret.append(packer.make_can_msg(CS.cruise_btns_msg_canfd, CAN.CAM, values))

    # --- 0x161/0x200/0x1ea/0x162 (frame%5) ---
    if frame % 5 == 0:
      lat_active = CC.latActive

      if CS.adrv_0x161 is not None:
        main_enabled = CS.out.cruiseState.available
        cruise_enabled = CC.enabled
        lat_enabled = CS.out.latEnabled
        nav_active = hud_control.activeCarrot > 1
        vehicle_navi_available = CS.out.vehicleNaviAvailable
        nav_icon_available = nav_active or vehicle_navi_available

        # hdpuse carrot
        hdp_use = int(Params().get("HDPuse"))
        hdp_active = False
        if hdp_use == 1:
          hdp_active = cruise_enabled and nav_active
        elif hdp_use == 2:
          hdp_active = cruise_enabled
        # hdpuse carrot

        values = copy.copy(CS.adrv_0x161)
        rx_counter = values.pop("COUNTER", None)
        values["SETSPEED"] = (6 if hdp_active else 3 if cruise_enabled else 1) if main_enabled else 0
        values["SETSPEED_HUD"] = (5 if hdp_active else 3 if cruise_enabled else 1) if main_enabled else 0

        set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
        values["vSetDis"] = int(set_speed_in_units + 0.5)

        values["DISTANCE"] = 4 if hdp_active else hud_control.leadDistanceBars
        values["DISTANCE_LEAD"] = 2 if cruise_enabled and lead_visible else 1 if main_enabled and lead_visible else 0
        values["DISTANCE_CAR"] = 3 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["DISTANCE_SPACING"] = 5 if hdp_active else 1 if cruise_enabled else 0

        values["TARGET"] = 1 if lead_visible and cruise_enabled else 0
        values["TARGET_DISTANCE"] = lead_distance

        values["BACKGROUND"] = _select_cluster_background(
          cruise_enabled, lat_active, CS.paddle_button_prev > 0, paddle_mode,
        )
        values["CENTERLINE"] = 1 if HDA_CntrlModSta > 0 else 0
        values["CAR_CIRCLE"] = 2 if hdp_active else 1 if cruise_enabled else 0

        values["NAV_ICON"] = 2 if nav_icon_available and cruise_enabled else 1 if main_enabled and nav_icon_available else 0
        values["HDA_ICON"] = 5 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["LFA_ICON"] = 5 if hdp_active else 2 if lat_active else 1 if lat_enabled else 0
        values["LKA_ICON"] = 4 if lat_active else 3 if lat_enabled else 0
        values["FCA_ALT_ICON"] = 0

        if values["ALERTS_2"] in [1, 2, 5, 6, 10, 21, 22]:
          values["ALERTS_2"] = 0
          values["DAW_ICON"] = 0

        if values["ALERTS_1"] == 0: # alerts가 있으면 사운드도 같이 나옴
          values["SOUNDS_1"] = 0
          values["SOUNDS_2"] = 0
          values["SOUNDS_4"] = 0

        if values["ALERTS_3"] in [3, 4, 11, 12, 13, 14, 17, 19, 20, 26, 27, 28, 7, 8, 9, 10]: # hide gap distance msg.(11,12,13,14), lanechange(19,20,27, 28)
          values["ALERTS_3"] = 0
          values["SOUNDS_3"] = 0

        if values["ALERTS_5"] in [1, 2, 3, 4, 5]:
          values["ALERTS_5"] = 0

        if values["ALERTS_5"] in [11] and CS.softHoldActive == 0:
          values["ALERTS_5"] = 0

        # curvature 표시(0x161쪽 기존 로직 유지)
        _suppress_trailer_mode_warning(values, CS)

        curvature = round(CS.out.steeringAngleDeg / 3)
        values["LANELINE_CURVATURE"] = (min(abs(curvature), 15) + (-1 if curvature < 0 else 0)) if lat_active else 0
        values["LANELINE_CURVATURE_DIRECTION"] = 1 if curvature < 0 and lat_active else 0

        trailer_lane_change_blocked = CS.trailer_connected
        if trailer_lane_change_blocked:
          values["LANELINE_LEFT"] = 2 if hud_control.leftLaneVisible else 0
          values["LANELINE_RIGHT"] = 2 if hud_control.rightLaneVisible else 0
        else:
          lane_color = 6 if md is not None and md.meta.laneChangeAvailableLeft else 2
          if lane_line_check >= 1:
            lane_line_warn_left = CS.out.leftLaneLine % 10 not in (0, 5)
          else:
            lane_line_warn_left = CS.out.leftLaneLine // 10 == 2
          lane_color = 4 if lane_line_warn_left or CS.out.leftBlindspot else lane_color
          if hud_control.leftLaneDepart:
            values["LANELINE_LEFT"] = 4 if (frame // 50) % 2 == 0 else 1
          else:
            values["LANELINE_LEFT"] = lane_color if hud_control.leftLaneVisible else 0

          lane_color = 6 if md is not None and md.meta.laneChangeAvailableRight else 2
          if lane_line_check >= 1:
            lane_line_warn_right = CS.out.rightLaneLine % 10 not in (0, 5)
          else:
            lane_line_warn_right = CS.out.rightLaneLine // 10 == 2
          lane_color = 4 if lane_line_warn_right or CS.out.rightBlindspot else lane_color
          if hud_control.rightLaneDepart:
            values["LANELINE_RIGHT"] = 4 if (frame // 50) % 2 == 0 else 1
          else:
            values["LANELINE_RIGHT"] = lane_color if hud_control.rightLaneVisible else 0

        values["LCA_LEFT_ARROW"] = 2 if CS.out.leftBlinker else 0
        values["LCA_RIGHT_ARROW"] = 2 if CS.out.rightBlinker else 0

        if trailer_lane_change_blocked:
          values["LCA_LEFT_ICON"] = 1 if lat_active else 0
          values["LCA_RIGHT_ICON"] = 1 if lat_active else 0
        else:
          values["LCA_LEFT_ICON"] = (1 if CS.out.leftBlindspot else 2) if lat_active else 0
          values["LCA_RIGHT_ICON"] = (1 if CS.out.rightBlindspot else 2) if lat_active else 0

        values["LANE_LEFT"] = 0 if trailer_lane_change_blocked else 1 if desire in (1, 3) else 0
        values["LANE_RIGHT"] = 0 if trailer_lane_change_blocked else 1 if desire in (2, 4) else 0

        ret.append(packer.make_can_msg("ADRV_0x161", CAN.ECAN, values, rx_counter = rx_counter))

      if CS.adrv_0x200 is not None:
        values = copy.copy(CS.adrv_0x200)
        rx_counter = values.pop("COUNTER", None)
        values["TauGapSet"] = hud_control.leadDistanceBars
        ret.append(packer.make_can_msg("ADRV_0x200", CAN.ECAN, values, rx_counter = rx_counter))

      if CS.adrv_0x1ea is not None:
        values = copy.copy(CS.adrv_0x1ea)
        rx_counter = values.pop("COUNTER", None)
        # blinker hold
        values['LEFT_BLINK_HOLD'] = 1 if lane_changing == 3 else 0
        values['RIGHT_BLINK_HOLD'] = 1 if lane_changing == 4 else 0

        _apply_cluster_lane_lines(values, CS, lat_active, desire)

        ret.append(packer.make_can_msg("ADRV_0x1ea", CAN.ECAN, values, rx_counter = rx_counter))

      if CS.ccnc_0x162 is not None:
        values = copy.copy(CS.ccnc_0x162)

        _convert_ccnc_boxes_to_cars(values)
        _apply_ccnc_lead(values, getattr(CS, "radarState", None), CC.enabled, getattr(CS, "modelV2", None))

        if (left_lane_warning and not CS.out.leftBlinker) or (right_lane_warning and not CS.out.rightBlinker):
          values["VIBRATE"] = 1

        _hide_replaced_adas_service_warning(values)

        if canfd_debug > 0:
          values["FAULT_LSS"] = 0
          values["FAULT_DAS"] = 0

        ret.append(packer.make_can_msg("CCNC_0x162", CAN.ECAN, values))

    # --- NEW_MSG_4B9 (corner radar keep-alive?) ---
    if enable_corner_radar > 0:
      if HDA_CntrlModSta == 0:
        if frame % 500 in [10, 20, 30]:
          values = {
            'BYTE_1': 0,
            'BYTE_2': 0,
            'BYTE_3': 0x80,
            'BYTE_4': 0x8A,
            'BYTE_5': 0x32,
            'BYTE_6': 0x30,
            'BYTE_7': 0x01,
            'BYTE_8': 0x00,
          }
          ret.append(packer.make_can_msg("NEW_MSG_4B9", CAN.CAM, values))
        elif frame % 500 in [40, 50, 60]:
          values = {
            'BYTE_1': 0xff,
            'BYTE_2': 0xff,
            'BYTE_3': 0xff,
            'BYTE_4': 0xff,
            'BYTE_5': 0xff,
            'BYTE_6': 0xff,
            'BYTE_7': 0xff,
            'BYTE_8': 0xff,
          }
          ret.append(packer.make_can_msg("NEW_MSG_4B9", CAN.CAM, values))

      if False:  # canfd_debug > 1 and frame % 20 == 0:
        if CS.hda_info_4a3 is not None:
          values = copy.copy(CS.hda_info_4a3)
          values["LinkClass"] = 1
          values["SPEED_LIMIT"] = 100
          ret.append(packer.make_can_msg("HDA_INFO_4A3", CAN.CAM, values))

  return ret
