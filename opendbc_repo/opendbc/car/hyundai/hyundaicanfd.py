import copy
import math
import numpy as np
from opendbc.car.hyundai.scalar import clip_float
from opendbc.car import CanBusBase, DT_CTRL, structs
from opendbc.car.carlog import carlog
from opendbc.car.crc import CRC16_XMODEM
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiExtFlags
from opendbc.car.hyundai.stopping import MOVING_SPEED, STOP_LOWER_BAND
from openpilot.common.params import Params
from openpilot.common.filter_simple import FirstOrderFilter
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
  return clip_float(a_raw, a_value_last - lower_step, a_value_last + upper_step)


def apply_canfd_stopping(values, CS, controller, accel, previous_value, jerk_u, jerk_l):
  """Apply the zero-aReq stopping sequence after the normal SCC interlocks."""
  if controller is None:
    return

  wheels = CS.out.wheelSpeeds
  speeds = [CS.out.vEgo, CS.out.vEgoRaw, wheels.fl, wheels.fr, wheels.rl, wheels.rr]
  finite = all(math.isfinite(v) for v in (*speeds, CS.out.aEgo, accel, values["aReqValue"], previous_value, jerk_u, jerk_l))
  speed = max(abs(v) for v in speeds) if finite else 0.0
  soft_hold = CS.softHoldActive > 0 and CS.out.cruiseState.available
  # Only an armed, stationary soft hold may remain active while the driver brakes.
  # Ordinary braking and pedal input while moving keep their existing interlock.
  brake_blocked = CS.out.brakePressed and not (soft_hold and speed <= MOVING_SPEED)
  blocked = (not finite or not CS.out.canValid or brake_blocked or CS.out.gasPressed
             or str(CS.out.gearShifter) != "drive" or longitudinal_interlock_active(CS))
  previous_phase = controller.phase
  command = controller.update(
    active=values["ACCMode"] == 1 and not blocked, requested=bool(values["StopReq"]), speed=speed,
    held=CS.canfdSccHoldActive, accel=accel, previous_value=previous_value,
    jerk_u=max(0.0, min(jerk_u, 5.0)), jerk_l=max(1.0, min(jerk_l, 5.0)),
  )
  if blocked or values["ACCMode"] != 1:
    values.update(StopReq=0, aReqRaw=0.0, aReqValue=0.0, AccelLimitBandLower=0.0)
    if not finite:
      values.update(ACCMode=0, JerkUpperLimit=1.0, JerkLowerLimit=1.0)
  elif command is not None:
    values.update(StopReq=command.stop_req, aReqRaw=command.raw, aReqValue=command.value,
                  AccelLimitBandUpper=0.0, AccelLimitBandLower=command.lower)
  else:
    # Establish the fixed band during ordinary active ACC, before any stop
    # request. Approach/held packets retain it instead of adding a band edge
    # alongside StopReq. This is a timing trial, not an OEM hold guarantee.
    values["AccelLimitBandLower"] = STOP_LOWER_BAND

  # Apply only to the final packet, after approach and interlocks select StopReq.
  # The first StopReq frame already carries zero Raw/Value; Upper may rise next.
  values["JerkUpperLimit"] = controller.limit_scc_jerk_upper(values["StopReq"], values["JerkUpperLimit"])

  if controller.phase != previous_phase:
    carlog.warning({"event": "carrot_stopping", "from": str(previous_phase), "phase": str(controller.phase),
                    "reason": controller.reason, "speed": speed, "aEgo": CS.out.aEgo,
                    "held": CS.canfdSccHoldActive,
                    "soft_hold": soft_hold,
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



def create_steering_messages_camera_scc(frame, packer, CP, CAN, CC, lat_active, apply_steer, CS, apply_angle, max_torque, angle_control,
                                        lat_override=False):

  emergency_steering = False
  if CS.adrv_0x161 is not None:
    values = CS.adrv_0x161
    emergency_steering = values["ALERTS_1"] in [11, 12, 13, 14, 15, 21, 22, 23, 24, 25, 26]


  ret = []
  if CS.mdps is not None:
    values = copy.copy(CS.mdps)
    # Seed once from RX; subsequent counters follow actual transmissions, not RX timing.
    rx_counter = values.pop("COUNTER", None)
    if angle_control:
      if CS.lfa_alt is not None:
        values["LFA2_ACTIVE"] = CS.lfa_alt["LKAS_ANGLE_ACTIVE"]
    else:
      if CS.lfa is not None:
        values["LKA_ACTIVE"] = 1 if CS.lfa["STEER_REQ"] == 1 else 0

    if frame % 1000 < 40:
      values["STEERING_COL_TORQUE"] += 220
    ret.append(packer.make_can_msg("MDPS", CAN.CAM, values, rx_counter=rx_counter))

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
        steering = CC.latActive or lat_override  # lat_override: SteerAngleTest
        values["LKAS_ANGLE_ACTIVE"] = 2 if steering else 1
        values["LKAS_ANGLE_CMD"] = -apply_angle
        values["LKAS_ANGLE_MAX_TORQUE"] = max_torque if steering else 0
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

def create_lfahda_cluster(packer, CS, CAN, long_active, lat_active, *, suppress_camera_auto_disengage=False, dm_alert=0):


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
  # The scoped GV70/Sorento HEV stock cameras can request this popup with lateral control,
  # including while longitudinal control is active. Suppress only the observed
  # signature in the outgoing cluster copy;
  # retain raw camera evidence and all fault / hands-off popup identities.
  lfa = CS.lfa if suppress_camera_auto_disengage else None
  mdps = CS.mdps if suppress_camera_auto_disengage else None
  scc = CS.scc_control if suppress_camera_auto_disengage else None
  if (suppress_camera_auto_disengage and lat_active
      and values.get("HDA_InfoPUDis") == 3 and values.get("HDA_InfoPUDis1") == 0
      and values.get("HDA_LFA_WrnSnd") == 0
      and lfa is not None and lfa.get("FCA_SYSWARN") == 1 and lfa.get("VALUE63") == 15
      and mdps is not None and mdps.get("LKA_FAULT") == 0 and mdps.get("LFA2_FAULT") == 0
      and scc is not None and scc.get("SysFailState") == 0):
    values["HDA_InfoPUDis"] = 0
  if dm_alert in (1, 2, 3) and values.get("HDA_InfoPUDis") == 0 and values.get("HDA_InfoPUDis1") == 0:
    values["HDA_InfoPUDis"] = 5  # DBC: hands-off popup, without the sound of value 6
  return [packer.make_can_msg("LFAHDA_CLUSTER", CAN.ECAN, values, rx_counter=rx_counter)]

def _apply_driver_monitoring_alert(values, hud_control, stock):
  level = getattr(hud_control, "driverMonitoringAlert", 0)
  if level not in (1, 2, 3):
    return
  # Use the documented hands-on warning, red at terminal DM. Apply after the
  # legacy stock-warning filter, on a fresh copy each tick.
  # Keep OEM popup/sound precedence. The reported stock terminal-warning pair
  # is ALERTS_2=2 with SOUNDS_2=3 (DBC: constant chime).
  if stock["ALERTS_2"] in (7, 8, 9, 10, 14, 21):
    values["ALERTS_2"] = stock["ALERTS_2"]
    for i in range(1, 5):
      values[f"SOUNDS_{i}"] = stock[f"SOUNDS_{i}"]
  if values["ALERTS_2"] == 0:
    values["ALERTS_2"] = 2 if level == 3 else 1
    if level == 3 and all(values[f"SOUNDS_{i}"] == 0 for i in range(1, 5)):
      values["SOUNDS_2"] = 3


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

    _apply_driver_monitoring_alert(values, getattr(CC, "hudControl", None), CS.adrv_0x161)
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


class DisplayLeadLateralFilter:
  """HUD-only path-relative lateral smoothing, updated once per control tick."""
  def __init__(self):
    self.filter = FirstOrderFilter(0.0, 0.4, DT_CTRL, initialized=False)
    self.target = None
    self.distance = None

  def update(self, radar_state, model_v2=None):
    lead = _display_lead(radar_state)
    if lead is None:
      self.filter.initialized = False
      self.target = self.distance = None
      return 0.0

    # A leadOne/leadTwo role change must not reset the same radar target.
    target = (getattr(lead, "radar", False), getattr(lead, "radarTrackId", -1))
    # Also reject a discontinuous range jump (reused track ID / vision-only lead).
    if target != self.target or (self.distance is not None and abs(lead.dRel - self.distance) > 5.0):
      self.filter.initialized = False
    self.target, self.distance = target, lead.dRel
    return self.filter.update(_display_lead_lateral(lead, model_v2))


def _apply_scc_lead(values, radar_state, model_v2=None, hud_lateral=None):
  lead = _display_lead(radar_state)
  # Match the stock no-object encoding; never retain an old camera target.
  values.update(ACC_ObjDist=204.6, ACC_ObjLatPos=0.0, ACC_ObjRelSpd=239.4, HUD_LEAD_INFO=0)
  if lead is not None:
    values["ACC_ObjDist"] = clip_float(lead.dRel, 0.1, 204.5)
    # Show the offset from the model path at the lead distance. Bound to
    # the signed 9-bit signal's representable range (0.1 scale, -20 offset).
    lateral = _display_lead_lateral(lead, model_v2) if hud_lateral is None else hud_lateral
    values["ACC_ObjLatPos"] = clip_float(lateral, -45.6, 5.5)
    values["ACC_ObjRelSpd"] = clip_float(lead.vRel, -170.0, 239.3)
    # Keep the OEM HUD/cluster lead white around standstill; relative-speed
    # noise near zero must not alternate white (2) and receding gray (1).
    values["HUD_LEAD_INFO"] = 1 if lead.vRel > 0.3 else 2


def create_acc_control_scc2(packer, CAN, enabled, accel_value_last, accel, stopping, gas_override, set_speed, hud_control, hyundai_jerk, CS,
                            stop_controller=None, hud_lateral=None):

  if CS.scc_control is None:
    if stop_controller is not None:
      stop_controller.reset()
    return None, accel_value_last
  interlock_active = longitudinal_interlock_active(CS)
  soft_hold_active = CS.softHoldActive > 0 and CS.out.cruiseState.available
  acc_control_enabled = (enabled or soft_hold_active) and CS.out.cruiseState.available and CS.paddle_button_prev == 0 and not interlock_active
  enabled = acc_control_enabled

  acc_mode = 0 if not enabled else (2 if gas_override else 1)
  previous_value = accel_value_last

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

  _apply_scc_lead(values, getattr(CS, "radarState", None), getattr(CS, "modelV2", None), hud_lateral)

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
  apply_canfd_stopping(values, CS, stop_controller, accel, previous_value, jerk_u, jerk_l)

  # Trial for no-lead stops: OEM SCC_ObjSta is named HUD_LEAD_INFO in this DBC.
  # Change only the state after the final StopReq/interlocks; do not fabricate
  # object distance/speed or latch it into RX. This is not a confirmed ESC gate.
  if stop_controller is not None and values["ACCMode"] == 1 and values["StopReq"] == 1 and values["HUD_LEAD_INFO"] == 0:
    values["HUD_LEAD_INFO"] = 2

  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values), values["aReqValue"]

def create_acc_control(packer, CAN, enabled, accel_last, accel, stopping, gas_override, set_speed, hud_control, jerk_u, jerk_l, CS,
                       stop_controller=None, accel_value_last=None):

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

  # accel_last is the legacy raw target, not necessarily the previous SCC
  # output. Keep ordinary packet limiting; anchor recovery to the returned value.
  previous_value = accel_last if accel_value_last is None else accel_value_last
  apply_canfd_stopping(values, CS, stop_controller, accel, previous_value, jerk_u, jerk_l)
  return packer.make_can_msg("SCC_CONTROL", CAN.ECAN, values), values["aReqValue"]


# SpasSteerTest: the stock parking ECU's steering request (05_can/rspa_steer_20261030.md). SPAS1 byte 3 is 0x4b on
# every stock frame, NEW_SIGNAL_2 the mode (3 idle, 4 ask, 5 active; the EPS echoes it in 0xEA byte 5), NEW_SIGNAL_1 the
# angle (+left, 0.1 deg). Only the parked form is sent: the stock frame gains bytes 5..19 only after D is selected.
def create_spas_steer(packer, CAN, mode, angle):
  return [packer.make_can_msg("SPAS1", CAN.ECAN, {"BYTE3": 0x4b, "NEW_SIGNAL_2": mode, "NEW_SIGNAL_1": angle})]


def create_spas_messages(packer, CAN, frame, left_blink, right_blink, spas2_template=None):
  ret = []

  ret.append(packer.make_can_msg("SPAS1", CAN.ECAN, {}))

  blink = 0
  if left_blink:
    blink = 3
  elif right_blink:
    blink = 4
  # GV80: the car's own SPAS2 (bytes 3/4=01, 10..13=80, 15=03, 20=10 via the BYTEn signals, 10-26 rlog 00000331);
  # other cars keep the zero-filled frame that lit the lamp on the Ioniq 5 and EV4
  values = {k: v for k, v in (spas2_template or {}).items() if k not in ("CHECKSUM", "COUNTER")}
  values["BLINKER_CONTROL"] = blink
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
    # Keep an independent TX sequence even when the latest RX snapshot repeats or skips.
    rx_counter = values.pop("COUNTER", None)
    values["DriverBraking"] = 0
    values["NEW_SIGNAL_20"] = 0
    values["NEW_SIGNAL_11"] = 0
    values["DriverBrakingLowSens"] = 0
    #values["NEW_SIGNAL_1"] = 0 # accel과 관련..  옆두부 꺼지는것과 관련? 확인필요
    #values["ACC_REQ"] = 1 # 옆두부 꺼지는것과 관련? 확인필요.. 항상 켜지게함..
    values["NEW_SIGNAL_1"] = 0 if values["ACC_REQ"] == 1 else 1 # 옆두부..
    ret.append(packer.make_can_msg("TCS", CAN.CAM, values, rx_counter=rx_counter))
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

BLINK_HOLD_CROSS_NEAR_M = 1.0     # the target-side line came this close to the car centre ...
BLINK_HOLD_CROSS_JUMP_M = 1.5     # ... then moved this far away again: the model relabelled it, the centre is across
BLINK_HOLD_RELEASE_PROGRESS = 0.6  # release once the car centre has moved this fraction of a lane width into the new lane


def _blink_hold_side(md, desire, state):
  """Turn-signal hold for ADRV_0x1ea: 3 (left), 4 (right) or 0.

  DesireHelper's request (modelV2.meta.laneChangeBlinkerHold) wins: it holds the lamp while a lever-requested
  change waits (LaneChangeLeverWait) and while a next change asked for during this one is queued. Otherwise the
  current change decides (_blink_hold_change).
  """
  change_hold = _blink_hold_change(md, desire, state)
  meta = getattr(md, "meta", None) if md is not None else None
  requested = str(getattr(meta, "laneChangeBlinkerHold", "none"))
  return 3 if requested == "left" else 4 if requested == "right" else change_hold


def _blink_hold_change(md, desire, state):
  """Hold for the current lane change: 3 (left), 4 (right) or 0.

  Held for the whole confirmed lane change (desire 3/4 = starting + finishing, same as the green
  cluster lane) instead of the model's desireState > 0.9, which falls ~2 s before the change ends and
  let the lamp stop mid-change (2026-09-19 drive: hold covered 49-65 % of the change). Released once the
  model's lane lines show the car centre 60 % of a lane width over (the crossed line, now on the other
  side, 0.1 lane width away), so the lamp does not keep flashing ~2 s after the car is already in the new
  lane. Without a detectable crossing the hold lasts to the end of the desire.
  """
  if desire not in (3, 4):
    state.update(crossed=False, released=False, y_min=None)
    return 0
  if state.get("released"):
    return 0
  lines = getattr(md, "laneLines", None) if md is not None else None
  if lines is not None and len(lines) >= 3 and len(lines[1].y) and len(lines[2].y):
    target, other = (1, 2) if desire == 3 else (2, 1)
    y_target, y_other = float(lines[target].y[0]), float(lines[other].y[0])
    # The relabel is spread over 2-3 model frames (e.g. -0.28 -> -1.36 -> -3.46 m), so compare with the
    # closest approach instead of the previous frame.
    y_min = abs(y_target) if state.get("y_min") is None else min(state["y_min"], abs(y_target))
    state["y_min"] = y_min
    if not state.get("crossed") and y_min < BLINK_HOLD_CROSS_NEAR_M and abs(y_target) - y_min > BLINK_HOLD_CROSS_JUMP_M:
      state["crossed"] = True
    lane_width = abs(y_target) + abs(y_other)
    if state.get("crossed") and abs(y_other) >= (BLINK_HOLD_RELEASE_PROGRESS - 0.5) * lane_width:
      state["released"] = True
      return 0
  return desire


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


LANE_LINE_UNSEEN = 14  # draws nothing on the Ioniq 5 PE cluster; undocumented on purpose, so a cluster update that changes it is noticed
# model laneLineProbs at low speed (10-18 rlogs, night): parking lots 0.01~0.04, city stops 0.13~0.22, highway ~1.0
LANE_HIDE_PROB, LANE_SHOW_PROB, LANE_HIDE_FRAMES = 0.08, 0.12, 100  # hide after 1 s below 0.08, show again from 0.12


def _lane_unseen(state, side, prob, frame):
  hidden, low_since = state.get(side, (False, None))
  if prob >= LANE_SHOW_PROB:
    hidden, low_since = False, None
  elif prob < LANE_HIDE_PROB:
    low_since = frame if low_since is None else low_since
    hidden = hidden or frame - low_since >= LANE_HIDE_FRAMES
  else:
    low_since = None  # in between: keep the current state
  state[side] = (hidden, low_since)
  return hidden


# ClusterLaneTest = field, ClusterLaneTestValue = raw value (test only): DBC fields, or 0x162 bits the DBC does not name
CLUSTER_TEST_FIELDS = {1: "SPEEDLIMIT", 2: "SPEEDLIMIT_FLASH", 3: "SIGNS", 4: "SPEEDLIMIT_WEATHER", 5: "COUNTRY",
                       64: "VIBRATE", 65: "FF_DETECT", 66: "FF_DISTANCE", 67: "FF_LATERAL", 68: "FF_DETECT_ALT",
                       69: "FF_DISTANCE_ALT", 70: "FF_LATERAL_ALT", 71: "LF_DETECT", 72: "LF_DETECT_DISTANCE",
                       73: "LF_DETECT_LATERAL", 74: "RF_DETECT", 75: "RF_DETECT_DISTANCE", 76: "RF_DETECT_LATERAL",
                       77: "LR_DETECT", 78: "LR_DETECT_DISTANCE", 79: "LR_DETECT_LATERAL", 80: "RR_DETECT",
                       81: "RR_DETECT_DISTANCE", 82: "RR_DETECT_LATERAL", 83: "FAULT_FSS", 84: "FAULT_FCA", 85: "FAULT_LSS",
                       86: "FAULT_SLA", 87: "FAULT_DAW", 88: "FAULT_HBA", 89: "FAULT_SCC", 90: "FAULT_LFA", 91: "FAULT_HDA",
                       92: "FAULT_LCA", 93: "FAULT_HDP", 94: "FAULT_DAS", 95: "FAULT_ESS"}  # 0x162
CLUSTER_TEST_FIELDS_161 = {6: "SLA_ICON", 12: "HDA_ICON", 13: "LFA_ICON", 14: "SETSPEED", 15: "SETSPEED_HUD",
                           16: "CAR_CIRCLE", 17: "DISTANCE", 18: "DISTANCE_CAR", 19: "DISTANCE_SPACING", 20: "DISTANCE_LEAD",
                           21: "ALERTS_4", 22: "ALERTS_5", 23: "BACKGROUND", 24: "LANE_HIGHLIGHT", 25: "LANE_ZOOM",
                           26: "CENTERLINE", 27: "ALERTS_1", 28: "ALERTS_2", 29: "ALERTS_3", 34: "FCA_ICON",
                           35: "FCA_ALT_ICON", 36: "LKA_ICON", 37: "HBA_ICON", 38: "FCA_IMAGE", 39: "BCA_LEFT",
                           40: "BCA_RIGHT", 41: "LCA_LEFT_ARROW", 42: "LCA_RIGHT_ARROW", 43: "TARGET", 44: "TARGET_DISTANCE",
                           45: "LANELINE_LEFT", 46: "LANELINE_LEFT_POSITION", 47: "LANELINE_RIGHT",
                           48: "LANELINE_RIGHT_POSITION", 49: "LANELINE_CURVATURE", 50: "LANELINE_CURVATURE_DIRECTION",
                           51: "LANE_HIGHLIGHT_DISTANCE", 52: "LANE_LEFT", 53: "LANE_RIGHT", 54: "MUTE", 55: "SOUNDS_1",
                           56: "SOUNDS_2", 57: "SOUNDS_3", 58: "SOUNDS_4", 59: "NAV_ICON", 60: "LCA_LEFT_ICON",
                           61: "LCA_RIGHT_ICON", 62: "DAW_ICON"}  # 0x161
CLUSTER_TEST_FIELDS_1EA = {96: "NEW_SIGNAL_4", 97: "NEW_SIGNAL_3", 98: "NEW_SIGNAL_2", 99: "NEW_SIGNAL_1",
                           100: "LEFT_BLINK_HOLD", 101: "RIGHT_BLINK_HOLD", 102: "NEW_SIGNAL_5", 103: "HDA_MODE2",
                           104: "LANE_LEFT", 105: "NEW_SIGNAL_6", 106: "LANE_RIGHT", 107: "LANE_CHANGING",
                           108: "LF_DETECT_DISTANCE", 109: "LF_DETECT_LATERAL", 110: "LF_DETECT", 111: "RF_DETECT_DISTANCE",
                           112: "RF_DETECT_LATERAL", 113: "RF_DETECT", 114: "SET_ME_FF", 115: "LR_DETECT_DISTANCE",
                           116: "LR_DETECT_LATERAL", 117: "LR_DETECT", 118: "RR_DETECT_DISTANCE", 119: "RR_DETECT_LATERAL",
                           120: "RR_DETECT", 121: "AUTOLANECHANGE_MSG", 122: "LANELINE_CURVATURE",
                           123: "LANELINE_CURVATURE_DIRECTION", 124: "LANELINE_LEFT_POSITION", 125: "LANELINE_RIGHT_POSITION",
                           126: "NEW_SIGNAL_7"}  # 0x1ea
CLUSTER_TEST_FIELDS_200 = {138: "SET_ME_E1", 139: "TauGapSet", 140: "NEW_SIGNAL_2", 141: "NEW_SIGNAL_1", 142: "TauGapSet_",
                           143: "NEW_SIGNAL_3"}  # 0x200
CLUSTER_TEST_BITS = {7: (53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63), 8: (159, 160, 161, 162),
                     9: (183, 184, 185, 186, 187, 188, 189, 190, 191), 10: (252, 253, 254, 255), 11: (87, 111, 135, 212)}  # 0x162 bits the DBC does not name
CLUSTER_TEST_BITS_161 = {30: (36, 37, 38, 39, 40, 41), 31: (45, 46, 47, 48, 49, 50), 32: (175, 176, 177, 178, 179, 180),
                         33: (246, 247, 248, 249, 250, 251, 252, 253, 254, 255), 63: (63, 215, 223, 227, 231, 235)}  # 0x161
CLUSTER_TEST_BITS_1EA = {127: (42, 71, 86, 87, 95), 128: (57, 58, 59, 60, 61, 62, 63),
                         129: (99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109),
                         130: (110, 111, 112, 113, 114, 115, 116, 117, 118, 119),
                         131: (128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138),
                         132: (147, 148, 149, 150, 151, 158, 159, 171),
                         133: (178, 179, 180, 181, 182, 183, 187, 188, 189, 190, 191),
                         134: (192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203),
                         135: (213, 214, 215, 216, 217, 218, 219, 220, 221, 222),
                         136: (223, 224, 225, 226, 227, 228, 229, 230, 231), 137: (249, 250, 251, 252, 253, 254, 255)}  # 0x1ea
CLUSTER_TEST_BITS_200 = {144: (48, 49, 50, 51, 52, 53, 54, 55), 145: (56, 57, 58, 59, 60, 61, 62, 63)}  # 0x200
# SCC_CONTROL / LFA carry the accel / steering commands: only their display-type fields, and only while parked
CLUSTER_TEST_FIELDS_SCC = {146: "InfoDisplay", 147: "DriverAlert", 148: "ObjDistLevel", 149: "DISTANCE_SETTING",
                           150: "HUD_LEAD_INFO", 151: "NSCCOper", 152: "NSCCOnOff", 153: "DriveMode", 154: "TakeOverReq",
                           155: "CRUSE_INFO_SET_2", 156: "TARGET_DISTANCE", 157: "ACC_ObjDist", 158: "ACC_ObjRelSpd",
                           159: "ACC_ObjLatPos", 160: "VSetDis"}  # 0x1a0
CLUSTER_TEST_FIELDS_LFA = {161: "LKA_MODE", 162: "LKA_WARNING", 163: "LKA_ICON", 164: "FCA_SYSWARN", 165: "LFA_BUTTON",
                           166: "HAS_LANE_SAFETY", 167: "NEW_SIGNAL_1", 168: "VALUE63", 169: "VALUE64", 170: "VALUE231",
                           171: "VALUE239", 172: "VALUE247", 173: "VALUE255"}  # 0x12a
CLUSTER_TEST_PARKED_ONLY = ("SCC_CONTROL", "LFA")
CLUSTER_TEST_MSGS = {'CCNC_0x162': (CLUSTER_TEST_FIELDS, CLUSTER_TEST_BITS), 'ADRV_0x161': (CLUSTER_TEST_FIELDS_161, CLUSTER_TEST_BITS_161),
                     'ADRV_0x1ea': (CLUSTER_TEST_FIELDS_1EA, CLUSTER_TEST_BITS_1EA), 'ADRV_0x200': (CLUSTER_TEST_FIELDS_200, CLUSTER_TEST_BITS_200),
                     'SCC_CONTROL': (CLUSTER_TEST_FIELDS_SCC, {}), 'LFA': (CLUSTER_TEST_FIELDS_LFA, {})}
CLUSTER_TEST_SLOTS = sorted({slot for fields, bits in CLUSTER_TEST_MSGS.values() for slot in (*fields, *bits)})
CLUSTER_TEST_AUTO_SLOT = -1  # ClusterLaneTest -1: step through ClusterLaneTestFrom..To, one value sweep per field


# 0x162 bit 188 (no DBC name; cluster test 9 at 188, 10-29): the HDP icon. Stock never sets it.
HDP_ICON_BIT = 188
# 0x1ea bit 104 (no DBC name; cluster test 129, 10-29): exterior hazard lights, ~0.1 s later. Stock never sets it.
HAZARD_REQUEST_BIT = 104
# 0x161 ALERTS_5 popups on engage / disengage (cluster test 22, 10-29): 19 "always be ready to drive", 21 "HDP released"
ENGAGE_POPUP, DISENGAGE_POPUP = 19, 21
ENGAGE_POPUP_FRAMES = int(2.0 / DT_CTRL)
# A lane change asked for in a tunnel waits for driver torque (desire_helper): say why on the cluster, at least 2 s
TUNNEL_REFUSAL_TEXT = ("ALERTS_4", 215)  # "this tunnel is not operable" (cluster test 21, 10-29)
# openpilot alert -> cluster text (0x161 ALERTS_4 = cluster test 21, ALERTS_5 = 22; 10-29 videos). Keyed by event name, or by
# "name/type" when only one alert type fits (reverse: only when it disengages, not while parked in R).
CLUSTER_ALERT_TEXTS = {
  "speedTooHigh": ("ALERTS_4", 83),                # driving speed is high
  "steerUnavailable": ("ALERTS_5", 9),             # lane keeping assist: not in operating conditions
  "accFaulted": ("ALERTS_5", 4),                   # smart cruise control: not in operating conditions
  "cruiseDisabled": ("ALERTS_5", 3),               # smart cruise control released
  "reverseGear/userDisable": ("ALERTS_4", 129),    # gear shift detected
  **dict.fromkeys(("controlsMismatch", "canError", "canBusMissing", "vehicleSensorsInvalid", "relayMalfunction",
                   "locationdPermanentError", "paramsdPermanentError"), ("ALERTS_4", 147)),  # abnormal operation detected
}


def _cluster_alert_text(alert_type):
  name = alert_type.split("/")[0]
  return CLUSTER_ALERT_TEXTS.get(alert_type if name == "reverseGear" else name)

# ClusterLaneTestValue -1..-4: step the field every 1 / 0.5 / 0.25 / 0.1 s (frames) and show the step in the set speed number
CLUSTER_TEST_SWEEP_FRAMES = {-1: 100, -2: 50, -3: 25, -4: 10}


def _cluster_test():
  params = Params()
  field = params.get_int("ClusterLaneTest")
  return (field, params.get_int("ClusterLaneTestValue")) if field != 0 else (0, 0)


def _cluster_steps(packer, slot):
  # one sweep of the slot: bit groups one bit at a time, fields 0..max (the set speed number only shows up to 255)
  for name, (fields, bit_groups) in CLUSTER_TEST_MSGS.items():
    if slot in bit_groups:
      return len(bit_groups[slot]) + 1
    if slot in fields:
      return min(1 << packer.dbc.name_to_msg[name].sigs[fields[slot]].size, 256)
  return 1


def _cluster_auto_slot(packer, frame, value):
  # ClusterLaneTest -1: the slot under test, moving on after one value sweep (a fixed value: 1 s per slot)
  params = Params()
  lo = params.get_int("ClusterLaneTestFrom") or CLUSTER_TEST_SLOTS[0]
  hi = params.get_int("ClusterLaneTestTo") or CLUSTER_TEST_SLOTS[-1]
  slots = [s for s in CLUSTER_TEST_SLOTS if lo <= s <= hi]
  if not slots:
    return 0
  period = CLUSTER_TEST_SWEEP_FRAMES.get(value, 100)
  state = create_ccnc_messages.__dict__.setdefault('_auto_slot', {})
  if state.get('key') != (lo, hi, period):
    state.update(key=(lo, hi, period), i=0, start=frame)
  steps = _cluster_steps(packer, slots[state['i'] % len(slots)]) if value in CLUSTER_TEST_SWEEP_FRAMES else 1
  if frame - state['start'] >= steps * period:
    state.update(i=state['i'] + 1, start=frame)
  return slots[state['i'] % len(slots)]


def apply_cluster_test_control_msgs(packer, can_sends, CS):
  """Cluster test slots 146+ on SCC_CONTROL / LFA: the raw value into that display field of the frames going out, parked only."""
  field, value = getattr(create_ccnc_messages, '_active_test', (0, 0))
  for name in CLUSTER_TEST_PARKED_ONLY:
    fields = CLUSTER_TEST_MSGS[name][0]
    if field not in fields:
      continue
    if not (CS.out.standstill and CS.out.gearShifter == structs.CarState.GearShifter.park):
      return can_sends
    from opendbc.can.packer import set_value  # opendbc.can.dbc imports this module for the checksum
    definition = packer.dbc.name_to_msg[name]
    out = []
    for addr, dat, bus in can_sends:
      if addr == definition.address:
        d = bytearray(dat)
        set_value(d, definition.sigs[fields[field]], value)
        d[0:2] = hkg_can_fd_checksum(addr, None, d).to_bytes(2, 'little')
        dat = bytes(d)
      out.append((addr, dat, bus))
    return out
  return can_sends


def _hazard_test():
  return Params().get_int("CanfdHazardTest") == 1


def _cluster_sweep(packer, field, frame, period=100):
  # returns (value to send, number to show); bit groups step one bit at a time and show the bit number
  state = create_ccnc_messages.__dict__.setdefault('_sweep', {})
  if state.get('field') != (field, period):
    state.update(field=(field, period), start=frame)
  step = (frame - state['start']) // period
  for name, (fields, bit_groups) in CLUSTER_TEST_MSGS.items():
    if field in bit_groups:
      bits = bit_groups[field]
      i = step % (len(bits) + 1)
      return (0, 0) if i == 0 else (1 << (i - 1), bits[i - 1])
    if field in fields:
      value = step % (1 << packer.dbc.name_to_msg[name].sigs[fields[field]].size)
      return value, value
  return 0, None


def _cluster_test_msg(packer, name, bus, values, rx_counter, field, value):
  # pack a cluster message with the test field (raw value) or raw bits applied last
  fields, bit_groups = CLUSTER_TEST_MSGS[name]
  if field in fields:
    sig = packer.dbc.name_to_msg[name].sigs[fields[field]]
    values[fields[field]] = value * sig.factor + sig.offset
  msg = packer.make_can_msg(name, bus, values, rx_counter=rx_counter)
  return _set_raw_bits(msg, bit_groups[field], value) if field in bit_groups else msg


def _set_raw_bits(msg, bits, value):
  addr, dat, bus = msg
  d = bytearray(dat)
  for i, b in enumerate(bits):
    d[b // 8] = (d[b // 8] & ~(1 << (b % 8))) | (((value >> i) & 1) << (b % 8))
  d[0:2] = hkg_can_fd_checksum(addr, None, d).to_bytes(2, 'little')
  return addr, bytes(d), bus


# Clusters that draw no line for the dashed codes (owner reports 10-28~30) keep the solid colour
NO_DASHED_LANE_CARS = (CAR.HYUNDAI_IONIQ_9, CAR.KIA_EV9)


def _dashed_lane_line(color, lane_line, car=None):
  # 0x161 dashed codes seen on the Ioniq 5 PE cluster: 8 white, 10 green (no orange dashed)
  if car not in NO_DASHED_LANE_CARS and lane_line // 10 > 0 and lane_line % 10 in (0, 5):  # camera: dashed / DLM inner dashed (color 0 = no lane info)
    return {2: 8, 6: 10}.get(color, color)
  return color


TRAFFIC_BACKGROUND = {1: 5, 2: 8}  # red light -> steady red, green light -> steady green (Ioniq 5 PE, 10-20)


def _traffic_display_state(traffic_state, frame):
  # Like the C4 traffic light ball: red while red, green only for 2 s after it turns green
  # (model-based green is on for most normal driving).
  if traffic_state == 2 and _traffic_display_state.last != 2:
    _traffic_display_state.green_until = frame + 200
  _traffic_display_state.last = traffic_state
  return 0 if traffic_state == 2 and frame >= _traffic_display_state.green_until else traffic_state
_traffic_display_state.last, _traffic_display_state.green_until = 0, 0


LANE_HIGHLIGHT_BY_LAT = {1: 1, 2: 3, 3: 4, 4: 5}  # laneless green, lane mode sky blue, steer warn yellow, saturated red (cluster test 24, 10-24)


def _select_cluster_background(cruise_enabled, lat_active, paddle_pressed, paddle_mode, turning=False, traffic_state=0,
                               roundabout=False):
  # A detected traffic light takes over the background until detection ends.
  if traffic_state in TRAFFIC_BACKGROUND:
    return TRAFFIC_BACKGROUND[traffic_state]
  # Reuse the existing flashing-red background to identify an active model turn.
  # Kick and automatic lane changes are held off after a roundabout sign (CarState.naviRoundabout): same flashing red
  if (lat_active and turning) or (paddle_mode > 0 and paddle_pressed) or roundabout:
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


def _normalize_cluster_corner_objects(values, *, ccnc=False):
  # EV5 can report 0x162 corner geometry with DETECT=0 while 0x1ea marks it visible.
  # Use nonzero corner distance to show a gray car, including those hidden types.
  # 0x1ea uses different enums; retain its legacy hidden-to-visible normalization.
  # Neither message blinks or clamps the received distance.
  for side in ("LF", "RF", "LR", "RR"):
    key = f"{side}_DETECT"
    if values[f"{side}_DETECT_DISTANCE"] != 0 and (ccnc or values[key] >= 4):
      values[key] = 3 if ccnc else 1
    elif ccnc and values[key] in (1, 2):  # 10-24 rlogs: stock flicks a box with distance 0 between car frames
      values[key] = 3


def _convert_ccnc_front_box_to_car(values):
  # Preserve the current front-lead presentation independently of corner objects.
  if values["FF_DETECT"] in (1, 2):
    values["FF_DETECT"] += 2


def _apply_ccnc_lead(values, radar_state, enabled, model_v2=None, hud_lateral=None):
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
  values["FF_DISTANCE"] = clip_float(lead.dRel, 0.1, 204.5)
  # Match the OEM object using current geometry, independently of display lag.
  values["FF_LATERAL"] = clip_float(lateral if hud_lateral is None else hud_lateral, -6.4, 6.3)


def create_alt2_adas_button_request(packer, CAN, frame, CC, CS, stopping):
  # This is a button request for Panda, not a replacement vehicle frame.
  # Panda overlays it on fresh 0x10B input, retaining all unknown fields,
  # the received counter, and the vehicle's original transmission cadence.
  source = CS.cruise_buttons_alt2
  lfa_button = 0
  cruise_button = 0
  driver_pressed = source["LFA_BTN"] != 0 or source["CRUISE_BUTTONS"] != 0
  if not driver_pressed:
    lfa_off = CS.lfahda_cluster is not None and CS.lfahda_cluster["HDA_LFA_SymSta"] == 0
    if lfa_off and 0 < frame % 200 < 12:
      lfa_button = 1

    if CC.enabled and not longitudinal_interlock_active(CS) and not lfa_button:
      # Leave a release interval after the LFA pulse before requesting SCC.
      scc_pulse = 20 < frame % 200 <= 26 and CS.out.vEgo > 3.
      if not CS.MainMode_ACC:
        cruise_button = 8 if scc_pulse else 0
      elif CS.ACCMode in (0, 4):
        cruise_button = 2 if scc_pulse else 0
      elif CS.scc_control is not None and CS.scc_control["InfoDisplay"] == 4 and not stopping:
        cruise_button = 2 if 10 < frame % 30 <= 16 else 0
      # Do not toggle MAIN merely because HDA is off: SCC is already active.

  address, data, bus = packer.make_can_msg("CRUISE_BUTTONS_ALT2", CAN.CAM,
                                          {"LFA_BTN": lfa_button, "CRUISE_BUTTONS": cruise_button})
  # The sparse receive DBC intentionally leaves the integrity fields unnamed.
  data = bytearray(data)
  data[:2] = hkg_can_fd_checksum(address, None, data).to_bytes(2, "little")
  return address, bytes(data), bus


def create_ccnc_messages(CP, packer, CAN, frame, CC, CS, hud_control,
                         disp_angle, left_lane_warning, right_lane_warning,
                         enable_corner_radar, stopping, canfd_debug, paddle_mode, hud_lateral=None):
  ret = []
  interlock_active = longitudinal_interlock_active(CS)
  display_lead = _display_lead(getattr(CS, "radarState", None))
  lead_visible = display_lead is not None
  lead_distance = clip_float(display_lead.dRel, 0.1, 204.5) if lead_visible else 0.0

  md = CS.modelV2
  if not hasattr(create_ccnc_messages, '_lane_line_check') or frame % 100 == 0:
    create_ccnc_messages._lane_line_check = Params().get_int("LaneLineCheck")
  lane_line_check = create_ccnc_messages._lane_line_check
  desire, _ = _get_desire_and_lane_changing(md)

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

      if getattr(CS, "cruise_buttons_alt2", None) is not None:
        ret.append(create_alt2_adas_button_request(packer, CAN, frame, CC, CS, stopping))
      elif CS.cruise_buttons_msg is not None:
        values = copy.copy(CS.cruise_buttons_msg)
        # Keep the physical long press on ECAN for CarState, but don't forward it to CAM.
        values["NORMAL_CRUISE_MAIN_BTN"] = 0

        if  HDA_LFA_SymSta == 0 and 0 < frame % 200 < 12:
          values["LFA_BTN"] = 1

        # The cluster's side objects, lead car and target come from stock SCC MAIN; keep it on from ignition.
        # openpilot's own MAIN still follows the driver's button (manual_main_off_latched).
        if not CS.MainMode_ACC and not interlock_active and 10 < frame % 200 <= 16:
          values["ADAPTIVE_CRUISE_MAIN_BTN"] = 1

        if CC.enabled and not interlock_active:
          if not CS.MainMode_ACC:
            pass  # pressed above
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
      test_field, test_value = _cluster_test()
      slot_display = None
      if test_field == CLUSTER_TEST_AUTO_SLOT:
        test_field = slot_display = _cluster_auto_slot(packer, frame, test_value)
      else:
        create_ccnc_messages.__dict__.pop('_auto_slot', None)
      sweep_display = None
      if test_value in CLUSTER_TEST_SWEEP_FRAMES:
        test_value, sweep_display = _cluster_sweep(packer, test_field, frame, CLUSTER_TEST_SWEEP_FRAMES[test_value])
      else:
        create_ccnc_messages.__dict__.pop('_sweep', None)
      create_ccnc_messages._active_test = (test_field, test_value)

      if CS.adrv_0x161 is not None:
        main_enabled = CS.out.cruiseState.available
        cruise_enabled = CC.enabled
        lat_enabled = CS.out.latEnabled
        # steering pauses at standstill; keep the cluster's steering/lane-change icons as if it were active
        lat_icon = lat_active or (cruise_enabled and lat_enabled and CS.out.standstill)
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

        values["DISTANCE"] = hud_control.leadDistanceBars  # real gap bars under the HDP colors too
        values["DISTANCE_LEAD"] = (2 if cruise_enabled else 1) if lead_visible else 0
        values["DISTANCE_CAR"] = 3 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["DISTANCE_SPACING"] = 5 if hdp_active else 1 if cruise_enabled else 0
        if hdp_active and hud_control.leadDistanceBars != 4:
          values["DISTANCE_SPACING"] = 3  # HDP spacing style for gaps 1-3 (cluster test 19-3, 10-20)

        values["TARGET"] = (1 if cruise_enabled else 3) if lead_visible else 0  # blue when engaged, white otherwise
        # The ring marks the target gap like stock (10-24 rlog: stock sends ~1.0/1.3/1.4/2.1 s x v by gap step, not the
        # lead distance). Engaged: the gap openpilot keeps; otherwise the stock value stays.
        desired_distance = getattr(hud_control, "desiredDistance", 0.)
        if cruise_enabled and desired_distance > 0:
          values["TARGET_DISTANCE"] = clip_float(desired_distance, 0.1, 204.5)

        values["BACKGROUND"] = _select_cluster_background(
          cruise_enabled, lat_active, CS.paddle_button_prev > 0, paddle_mode, turning=desire in (1, 2),
          traffic_state=_traffic_display_state(getattr(hud_control, "trafficState", 0), frame),
          roundabout=getattr(CS.out, "naviRoundabout", False),
        )
        values["CENTERLINE"] = 1 if HDA_CntrlModSta > 0 else 0
        values["CAR_CIRCLE"] = 2 if hdp_active else 1 if cruise_enabled else 0

        values["NAV_ICON"] = 2 if nav_icon_available and cruise_enabled else 1 if main_enabled and nav_icon_available else 0
        values["HDA_ICON"] = 5 if hdp_active else 2 if cruise_enabled else 1 if main_enabled else 0
        values["LFA_ICON"] = 5 if hdp_active else 2 if lat_icon else 1 if lat_enabled else 0
        values["LKA_ICON"] = 4 if lat_icon else 3 if lat_enabled else 0
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

        # carrot: with HDP on, a gap change shows the stock "distance N" popup (ALERTS_3 7~10) for 2 s
        gap_bars = hud_control.leadDistanceBars
        if hdp_active and gap_bars != getattr(create_ccnc_messages, '_gap_bars', gap_bars):
          create_ccnc_messages._gap_popup_until = frame + 200
        create_ccnc_messages._gap_bars = gap_bars
        if frame < getattr(create_ccnc_messages, '_gap_popup_until', 0) and values["ALERTS_3"] == 0 and 1 <= gap_bars <= 4:
          values["ALERTS_3"] = 6 + gap_bars

        if values["ALERTS_5"] in [1, 2, 3, 4, 5]:
          values["ALERTS_5"] = 0

        if values["ALERTS_5"] in [11] and CS.softHoldActive == 0:
          values["ALERTS_5"] = 0

        # Ioniq 5 PE only: other clusters' ALERTS_5 table is unchecked (IONIQ 9 uses 6 for trailer mode)
        if cruise_enabled != getattr(create_ccnc_messages, '_was_enabled', cruise_enabled):
          create_ccnc_messages._engage_popup = (ENGAGE_POPUP if cruise_enabled else DISENGAGE_POPUP, frame + ENGAGE_POPUP_FRAMES)
        create_ccnc_messages._was_enabled = cruise_enabled
        popup, popup_until = getattr(create_ccnc_messages, '_engage_popup', (0, 0))
        is_ioniq5pe = getattr(CP, "carFingerprint", None) == CAR.HYUNDAI_IONIQ_5_PE
        if is_ioniq5pe and frame < popup_until and values["ALERTS_5"] == 0:
          values["ALERTS_5"] = popup
        # the alert's own text beats the generic disengage popup; a stock text still wins
        text = _cluster_alert_text(getattr(hud_control, "alertType", "") or "") if is_ioniq5pe else None
        if text and (values[text[0]] == 0 or (text[0] == "ALERTS_5" and values["ALERTS_5"] == popup and frame < popup_until)):
          values[text[0]] = text[1]
        tunnel = getattr(CS.out, "naviTunnel", False)
        if desire in (3, 4):  # the change started (driver torque): nothing refused any more
          create_ccnc_messages._tunnel_refusal_until = 0
        elif tunnel and lat_active and CS.out.leftBlinker != CS.out.rightBlinker:  # one side (not hazards)
          create_ccnc_messages._tunnel_refusal_until = frame + ENGAGE_POPUP_FRAMES
        if is_ioniq5pe and frame < getattr(create_ccnc_messages, '_tunnel_refusal_until', 0) and values[TUNNEL_REFUSAL_TEXT[0]] == 0:
          values[TUNNEL_REFUSAL_TEXT[0]] = TUNNEL_REFUSAL_TEXT[1]

        # curvature 표시(0x161쪽 기존 로직 유지)
        _suppress_trailer_mode_warning(values, CS)

        curvature = round(CS.out.steeringAngleDeg / 3)
        values["LANELINE_CURVATURE"] = (min(abs(curvature), 15) + (-1 if curvature < 0 else 0)) if lat_active else 0
        values["LANELINE_CURVATURE_DIRECTION"] = 1 if curvature < 0 and lat_active else 0

        trailer_lane_change_blocked = CS.trailer_connected
        lane_probs = md.laneLineProbs if md is not None and len(md.laneLineProbs) >= 3 else None
        unseen = create_ccnc_messages.__dict__.setdefault('_lane_unseen', {})
        # hide only when both miss (10-24 rlogs): the model drops all lines for 2~3 s mid lane change, the camera drops them in 26% of stops
        left_seen = hud_control.leftLaneVisible and (
          not (lane_probs is not None and _lane_unseen(unseen, 'L', lane_probs[1], frame)) or CS.out.leftLaneLine // 10 > 0)
        right_seen = hud_control.rightLaneVisible and (
          not (lane_probs is not None and _lane_unseen(unseen, 'R', lane_probs[2], frame)) or CS.out.rightLaneLine // 10 > 0)
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
            values["LANELINE_LEFT"] = _dashed_lane_line(lane_color, CS.out.leftLaneLine, getattr(CP, "carFingerprint", None)) if left_seen or lane_color == 4 else LANE_LINE_UNSEEN  # orange stays

          lane_color = 6 if md is not None and md.meta.laneChangeAvailableRight else 2
          if lane_line_check >= 1:
            lane_line_warn_right = CS.out.rightLaneLine % 10 not in (0, 5)
          else:
            lane_line_warn_right = CS.out.rightLaneLine // 10 == 2
          lane_color = 4 if lane_line_warn_right or CS.out.rightBlindspot else lane_color
          if hud_control.rightLaneDepart:
            values["LANELINE_RIGHT"] = 4 if (frame // 50) % 2 == 0 else 1
          else:
            values["LANELINE_RIGHT"] = _dashed_lane_line(lane_color, CS.out.rightLaneLine, getattr(CP, "carFingerprint", None)) if right_seen or lane_color == 4 else LANE_LINE_UNSEEN

        values["LCA_LEFT_ARROW"] = 2 if CS.out.leftBlinker else 0
        values["LCA_RIGHT_ARROW"] = 2 if CS.out.rightBlinker else 0

        if trailer_lane_change_blocked:
          values["LCA_LEFT_ICON"] = 1 if lat_icon else 0
          values["LCA_RIGHT_ICON"] = 1 if lat_icon else 0
        else:
          values["LCA_LEFT_ICON"] = (1 if CS.out.leftBlindspot else 2) if lat_icon else 0
          values["LCA_RIGHT_ICON"] = (1 if CS.out.rightBlindspot else 2) if lat_icon else 0

        values["LANE_LEFT"] = 0 if trailer_lane_change_blocked else 1 if desire in (1, 3) else 0
        values["LANE_RIGHT"] = 0 if trailer_lane_change_blocked else 1 if desire in (2, 4) else 0

        _apply_driver_monitoring_alert(values, hud_control, CS.adrv_0x161)
        lat_hud = getattr(hud_control, "latHudState", 0)
        if lat_hud and desire in (3, 4):
          lat_hud = 1  # lane change first: stock lights the lane green only while changing lanes (10-24 logs)
        if LANE_LINE_UNSEEN in (values["LANELINE_LEFT"], values["LANELINE_RIGHT"]):
          lat_hud = 0  # a hidden lane line: no highlight, the stock value stays
        if lat_hud in LANE_HIGHLIGHT_BY_LAT:
          values["LANE_HIGHLIGHT"] = LANE_HIGHLIGHT_BY_LAT[lat_hud]
          values["LANE_HIGHLIGHT_DISTANCE"] = 0.  # 0: only the patch around the car (user choice 10-24; stock lane change uses 150 m)
        if sweep_display is not None:
          values["vSetDis"] = min(sweep_display, 255)
        ret.append(_cluster_test_msg(packer, "ADRV_0x161", CAN.ECAN, values, rx_counter, test_field, test_value))

      if CS.adrv_0x200 is not None:
        values = copy.copy(CS.adrv_0x200)
        rx_counter = values.pop("COUNTER", None)
        values["TauGapSet"] = hud_control.leadDistanceBars
        ret.append(_cluster_test_msg(packer, "ADRV_0x200", CAN.ECAN, values, rx_counter, test_field, test_value))

      if CS.adrv_0x1ea is not None:
        values = copy.copy(CS.adrv_0x1ea)
        rx_counter = values.pop("COUNTER", None)
        # blinker hold: the confirmed lane change until the body is across (see _blink_hold_side)
        hold = _blink_hold_side(md, desire, create_ccnc_messages.__dict__.setdefault("_blink_hold", {}))
        external_hold = getattr(hud_control, "physicalBlinkerHold", 0)
        if external_hold in (1, 2, 3):
          hold = {1:0, 2:3, 3:4}[external_hold]
        values['LEFT_BLINK_HOLD'] = 1 if hold == 3 else 0
        values['RIGHT_BLINK_HOLD'] = 1 if hold == 4 else 0

        _apply_cluster_lane_lines(values, CS, lat_active, desire)
        _normalize_cluster_corner_objects(values)

        msg = _cluster_test_msg(packer, "ADRV_0x1ea", CAN.ECAN, values, rx_counter, test_field, test_value)
        if HAZARD_REQUEST_BIT not in CLUSTER_TEST_BITS_1EA.get(test_field, ()) and _hazard_test():
          msg = _set_raw_bits(msg, (HAZARD_REQUEST_BIT,), 1)
        ret.append(msg)

      if CS.ccnc_0x162 is not None:
        values = copy.copy(CS.ccnc_0x162)
        # Seed once from RX; the packer owns the per-message TX counter after that.
        # RX-paced Panda forwarding replaces it with the original RX counter.
        rx_counter = values.pop("COUNTER", None)

        _normalize_cluster_corner_objects(values, ccnc=True)
        _convert_ccnc_front_box_to_car(values)
        _apply_ccnc_lead(values, getattr(CS, "radarState", None), CC.enabled, getattr(CS, "modelV2", None), hud_lateral)
        # auto slot sweep: the slot number on the speed-limit sign (the set speed number shows the value)
        if slot_display and not CLUSTER_TEST_FIELDS.get(test_field, "").startswith("SPEEDLIMIT"):
          values["SPEEDLIMIT"] = slot_display

        if (left_lane_warning and not CS.out.leftBlinker) or (right_lane_warning and not CS.out.rightBlinker):
          values["VIBRATE"] = 1

        _hide_replaced_adas_service_warning(values)

        if canfd_debug > 0:
          values["FAULT_LSS"] = 0
          values["FAULT_DAS"] = 0

        msg = _cluster_test_msg(packer, "CCNC_0x162", CAN.ECAN, values, rx_counter, test_field, test_value)
        # HDP icon while cruise or steering is engaged; Ioniq 5 PE only - other clusters' bit 188 is unchecked
        if (getattr(CP, "carFingerprint", None) == CAR.HYUNDAI_IONIQ_5_PE and (CC.enabled or lat_active) and
            HDP_ICON_BIT not in CLUSTER_TEST_BITS.get(test_field, ())):
          msg = _set_raw_bits(msg, (HDP_ICON_BIT,), 1)
        ret.append(msg)

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
