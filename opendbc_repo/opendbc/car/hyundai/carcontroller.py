from collections import deque

import numpy as np
from opendbc.car.hyundai.scalar import clip_float
from opendbc.can import CANPacker
from opendbc.car import Bus, CanData, DT_CTRL, apply_driver_steer_torque_limits, common_fault_avoidance, make_tester_present_msg, structs, apply_std_steer_angle_limits
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.lane_crossing import LaneCrossing
from opendbc.car.hyundai.stopping import CanfdStopping
from opendbc.car.hyundai.palisade_cgw_test import PalisadeCgwTest, cgw_frame
from opendbc.car.hyundai.steering_handover import SteeringHandover
from opendbc.car.carlog import carlog
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiSafetyFlags, Buttons, CarControllerParams, CAR, CAN_GEARS, HyundaiExtFlags
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.hyundai.values import get_pedal_interceptor_params
from opendbc.car.pedal_interceptor import PedalInterceptor
from openpilot.common.filter_simple import MyMovingAverage

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState


from openpilot.common.params import Params

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2
DRIVER_TORQUE_FILTER_TAU = 0.12
PRE_OVERRIDE_PREDICTION_TIME = 0.15
PRE_OVERRIDE_START_RATIO = 0.90
PRE_OVERRIDE_FULL_RATIO = 1.05
PRE_OVERRIDE_RAW_MIN_RATIO = 0.70
PRE_OVERRIDE_FILTERED_MIN_RATIO = 0.65
PRE_OVERRIDE_MIN_RATE_RATIO = 0.50
PRE_OVERRIDE_CONFIRM_FRAMES = 2
PRE_OVERRIDE_MAX_TORQUE_DELTA = -10.0
# Hand-resting torque blips shorter than this must not stretch recovery_time.
OVERRIDE_COUNT_MIN_FRAMES = int(0.3 / DT_CTRL)
LOW_SPEED_ANGLE_RATE_RAMP_SPEED = 15.0 * CV.KPH_TO_MS
MID_SPEED_ANGLE_RATE_LIMIT_SPEED = 40.0 * CV.KPH_TO_MS
# SteerAngleTest range. The LKAS angle itself stays at STEER_ANGLE_MAX: 10-29 test (20261029_2056) the EPS stopped at
# ~180 deg and raised steerFaultTemporary whenever asked for more
STEER_ANGLE_TEST_MAX = 450.  # deg
# Stock parking assist steering: the camera's LFA turns VALUE63 to 15 when it starts (10-29, two auto parks). openpilot's
# steering-only mode came on during both and pulled against it; at parking speeds, stand aside (no panda blocking)
PARK_ASSIST_VALUE63 = 15
PARK_ASSIST_MAX_KPH = 10.
# latActive stands aside only where VALUE63 15 was seen to mean the stock auto park (other cars: unverified, 11-12 review)
PARK_ASSIST_YIELD_CARS = (CAR.HYUNDAI_IONIQ_5_PE,)
STEER_ANGLE_TEST_RATE = 1.0  # deg/tick toward SteerAngleTest
LARGE_ANGLE_UNWIND_RATE = 1.5  # deg/tick: allow a quicker return from the EPS fault-angle region
CANFD_JERK_UPPER_MIN = 1.0
CANFD_JERK_LIMIT_MAX = 5.0
CANFD_DEPART_JERK_UPPER = 3.0  # StopGoLongAssist departure: no-lead departures already send 2.8-3.3 (02_logs §22)
# CanfdBlinkerKick: in D at any speed (user 10-27: city lane changes are slow) the parking ECU (APRK, 0x7B1) is silenced
# around a kick and SPAS2 BLINKER_CONTROL is sent in its place; out of D its default session returns
APRK_ADDR = 0x7B1
APRK_EXT_SESSION = bytes([0x02, 0x10, 0x03, 0, 0, 0, 0, 0])
APRK_COM_DISABLE = bytes([0x03, 0x28, 0x83, 0x01, 0, 0, 0, 0])
APRK_DEFAULT_SESSION = bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])
# Handoff like the owner's spas.Session: SPAS frames go out only after the ECU answered 10 03 (50 03) and its own SPAS1/
# SPAS2 stayed off ECAN for 0.1 s; they stop the moment a stock frame shows up again. 10-26..30 it never answered on
# bus 0 (0 of ~140) and every kick overwrote a live stock stream - 21 bus-offs and one CAN error at 112 km/h on 10-30
# (02_logs/measurements.md §36). The request now also goes to ACAN, where a second SPAS1 stream runs.
# CanfdBlinkerSpas2Only (Sorento HEV 11-19, 02_logs/measurements.md s70): its parking ECU answers 0x7B1, and 0.51 s into
# our SPAS1+SPAS2 the ESC turned ACC/FCA/AVH off (back 50 ms after our frames stopped, the ECU still silent). Whether
# our SPAS1 (stock byte 14 = 0x02, ours 0) or SPAS2 itself (this car has none) trips it is not known yet:
# 1 leaves the ECU on (no 0x7B1, its own SPAS1 and 0x170 stay) and sends SPAS2 only; 2 silences it as before, SPAS2 only.
# 11-19 evening (s71): 1 and 2 both tripped it 0.5-0.6 s in, so it is our SPAS2 itself (this car sends none). The ESC
# needs ~0.5 s of it without a break: a 51 ms host gap (s70, 507.59) cleared it and it came back only 0.52 s after we
# resumed. 3/4 = 1 sent in bursts, SPAS2_BURST_ON[value] frames on and SPAS2_BURST_OFF off, so it never runs 0.5 s (3: 0.35 s,
# 4: 0.45 s - more on-time, 50 ms under the shortest trip seen; the gap stays above the 30-55 ms the ESC needs); the BCM keeps
# the side it was asked for over a short gap (s70: one more flash 0.48 s after our last frame).
# 11-20 car test (s73, P): 3 and 4 each ran ~20 s with no fault and the lamp kept 0.36/0.36 s. The one fault came at the
# 3 -> 4 switch: the restart left a 2-frame gap, so the ESC saw 0.66 s in a row. A new burst session therefore starts
# SPAS2_BURST_OFF after the last SPAS2 frame of any session (spas2_sent).
SPAS2_BURST_ON = {3: int(0.35 / DT_CTRL), 4: int(0.45 / DT_CTRL)}
SPAS2_BURST_OFF = int(0.1 / DT_CTRL)
APRK_IDLE, APRK_ASK, APRK_WAIT_QUIET, APRK_ACTIVE, APRK_FAILED = range(5)
APRK_REPLY_FRAMES = int(0.4 / DT_CTRL)
APRK_QUIET_NANOS = 100_000_000
# CanfdHazardTest 3/4 (Tucson HDA1, user 11-19), SPAS2 path only (a car whose camera sends 0x1ea ignores them): is
# BLINKER_CONTROL 1 a push-to-toggle switch or a held request? 11-18 logs (02_logs/measurements.md s66, 00000419 seg1)
# point to held: the lamp went off 20-30 ms after our last frame in 4/4 windows, a 0.1 s pause in the fast flash turned it
# off, and the stock hazard-switch bit (0x413 byte3 bit2) never rose. 3 sends one value-1 frame once (toggle: hazards stay
# on; held: one short flash); 0 then 3 sends it again. 4 asks left (3) and right (4) in turn, 0.4 s each: the BCM runs its
# own clean 0.4 s timer for 3/4 even with the stock 0 frames in between (s66 seg1 113 s).
HAZARD_ALT_FRAMES = int(0.4 / DT_CTRL)
# CanfdHazardTest 5 (11-20, s71): the stock parking ECU's own hazards during Tucson remote parking are not value 1 held but
# BLINKER_CONTROL 2 (0.34 s) and 1 (0.32 s) in turn at 50 Hz, starting with 2, while 0x413 shows both lamps on throughout.
# 5 copies that rhythm on the same SPAS2 path (raw values, panda allows all 8).
HAZARD_STOCK_FRAMES = int(0.33 / DT_CTRL)
# CanfdHazardTest 10-17 (user 11-19): BLINKER_CONTROL 0-7 held as is, same path. 2 is "hazards button backlight" in the
# DBC, 5-7 have no name; none of them was tried on a car before (panda allows all 8 values since then).
HAZARD_RAW_FIRST = 10
# 3/4/10-17 wait until carstate had its chance to adopt the camera's ADRV_0x1ea (CANFD_CAMERA_TX_MESSAGES ready count
# 122): before that a 0x1ea car looks like a SPAS2 car and would put these trial values on the bus (review 11-19)
HAZARD_SPAS2_READY = 122
# 3 on a piggybacked stream (no ack, KickMode 1): panda keeps only the newest held 0x16A, so if MenuTest/Kick/remote
# also run, the next host frame could replace the single value-1 frame before a stock frame carries it. Send no SPAS
# frames for 3 host frames after the shot: 30 ms > one 20 ms stock period, < the 50 ms hold limit (review 11-19).
HAZARD_SHOT_HOLD_FRAMES = 3
# SpasSteerTest: stock RSPA handshake, mode 4 then 5 after 0.1 s (2019 log 226.7 -> 226.8 s), angle at 100 deg/s
SPAS_STEER_ASK_FRAMES = 10
SPAS_STEER_RATE = 1.0
# kick: ~2 flashes when the driver starts crossing a lane line without the blinker (lane_crossing.py).
# shortcut: flash count at 1.0 s is not measured (BCM ran on ~1.2 s after the command stopped, EV4 10-26), tune after a drive
BLINKER_PULSE_FRAMES = int(1.0 / DT_CTRL)
# CanfdBlinkerKickModelMode 0 holds its pulse through the change like the CCNC blink hold (hyundaicanfd._blink_hold_change):
# renewed by this gap every frame it still holds, never past the max (DesireHelper owns the lamp for 12 s)
MODEL_HOLD_GAP_FRAMES = int(0.2 / DT_CTRL)
MODEL_HOLD_MAX_FRAMES = int(10.0 / DT_CTRL)
# No kick across a yellow (centre) line. The camera color drops to 0 while the line is crossed (10-30 seg 22: 10 -> 2
# at the kick), so a yellow seen on that side in the last 2 s counts (10-30 seg 1: kick over yellow dashed 20)
YELLOW_HOLD_FRAMES = int(2.0 / DT_CTRL)
# Palisade kick on CGW1 (cgw_blinkers): lamp bit 0.35 s on / 0.35 s off like stock. 60 ms, same as PalisadeBlinkerTest:
# panda's 50 ms rate limit would bounce every frame that arrives a little early (4 of 49 even at 50 ms, 00000e46)
CGW_TX_FRAMES = 6
CGW_FLASH_HALF_FRAMES = int(0.35 / DT_CTRL)
# panda lets one driving CGW1 trial run 5 s (7 flashes) and rearms only after 1 s with no request at all, rejected ones
# included (safety_hyundai.h hyundai_cgw_test_tx): the host stays inside both so a later kick is never refused whole
CGW_TRIAL_FRAMES = int(4.9 / DT_CTRL)
CGW_REARM_FRAMES = int(1.05 / DT_CTRL)
# CanfdBlinkerKickModelMode 2/3/4: 3/5/7 flashes like the stock one-touch setting, one stock flash cycle (0.7 s) each
MODEL_FLASH_FRAMES = 2 * CGW_FLASH_HALF_FRAMES
MODEL_FLASH_COUNTS = {2: 3, 3: 5, 4: 7}
# Bluetooth remote blinkerLeft/Right (card reads the 'blinker' channel): one stock one-touch, 3 flashes, in any gear
REMOTE_FLASHES = 3
# Every stock lever press starts with one BLINKER + BLINKER_TAP frame before the steady frame (Ioniq 5 3372.9 s:
# 4280304010000000 tap -> de90304000000000 hold; GV80 82.96 s: 5a10304010000000 tap). The BCM needs that edge -
# replaying only the steady form (TAP 0) did nothing on either car. GV80's lever is momentary and releases itself.
STALK_TAP_FRAMES = int(0.20 / DT_CTRL)
STALK_TAP_MOMENTARY_FRAMES = int(0.35 / DT_CTRL)
# The stock column module sends 0x3C1 at 5 Hz (200 ms). Sending the same ID faster raises the chance of a
# same-ID collision with it, which is what took the Azera bus down, so the momentary mode sticks to 5 Hz.
STALK_MOMENTARY_HZ_DIV = 20
# BlinkerStalkTest 5/6 (high beam / flash-to-pass): a stock high-beam push is a ~0.15 s HIGHBEAM_FORWARD pulse on 0x3C1
# (05_can/hba_ifs.md, 10-04)
HIGHBEAM_PRESS_FRAMES = int(0.15 / DT_CTRL)

# BlinkerStalkTest: when APRK can't be silenced, imitate the steering column lever (BLINKER_STALKS, 0x3C1); the BCM
# blinks from it (09-18 log: 0x413 ALT lamp at 0.4 s while the lever is on). 10-26 logs: COUNTER_ALT goes +1 on every
# lever change and stays put otherwise; CHECKSUM_MAYBE = CRC8 (poly 0x1D, init 0, xorout 0x65) of bytes 1..7.


def stalk_checksum(body):
  crc = 0
  for b in body:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0x65
CANFD_DEPART_TIME = 3.0
CANFD_DEPART_SPEED = 3.0
CANFD_JERK_ERROR_DELAY = 0.5
CANFD_JERK_ERROR_FILTER_TIME = 0.4
CANFD_JERK_ERROR_DEADBAND = 0.25
CANFD_JERK_ERROR_FULL_SCALE = 0.5
CANFD_JERK_RELEASE_THRESHOLD = 0.1
# Some CAN-FD SCC implementations need a higher lower-jerk limit to follow sustained
# deceleration requests. Keep the historical MPC-jerk limit as the default and blend
# toward this stock-like feedforward only after measured under-deceleration.
CANFD_JERK_LOWER_ACCEL_BP = [0.0, 0.8, 1.2, 1.5, 2.0, 2.5, 3.2]
CANFD_JERK_LOWER_LIMIT_V = [1.2, 1.2, 1.2, 1.7, 3.0, 3.3, 3.7]

vibrate_intervals = [
  (0.0, 0.5),
  (1.0, 1.5),
  #(2.5, 3.0),
  #(3.5, 4.0),
  (5.0, 5.5),
  (6.0, 6.5),
  (7.5, 8.0),
]

def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning

def rate_limit(x, x_last, lo, hi):
  return clip_float(x, x_last + lo, x_last + hi)


def calculate_canfd_jerk_limits(accel: float, jerk: float, tracking_error: float = 0.0) -> tuple[float, float]:
  jerk_u = np.clip(jerk * 2.0, CANFD_JERK_UPPER_MIN, CANFD_JERK_LIMIT_MAX)
  jerk_l_mpc = np.clip(-jerk * 4.0, 1.0, CANFD_JERK_LIMIT_MAX)

  # A clearly positive jerk means the plan is releasing deceleration. In that case, or when
  # the vehicle is already tracking the request, return to the historical jerk-based
  # limit immediately instead of allowing acceleration demand alone to hold braking.
  assist_ratio = 0.0
  if jerk <= CANFD_JERK_RELEASE_THRESHOLD:
    assist_ratio = np.clip((tracking_error - CANFD_JERK_ERROR_DEADBAND) / CANFD_JERK_ERROR_FULL_SCALE, 0.0, 1.0)

  decel_request = max(0.0, -accel)
  jerk_l_feedforward = np.interp(decel_request, CANFD_JERK_LOWER_ACCEL_BP, CANFD_JERK_LOWER_LIMIT_V)
  jerk_l_assist = 1.0 + assist_ratio * (jerk_l_feedforward - 1.0)
  jerk_l = np.clip(max(jerk_l_mpc, jerk_l_assist), 1.0, CANFD_JERK_LIMIT_MAX)
  return float(jerk_u), float(jerk_l)

def apply_steer_angle_limits_physics(desired_sw_deg: float,
                                     last_sw_deg: float,
                                     v_ego: float,
                                     steering_sw_deg: float,
                                     lat_active: bool,
                                     wheelbase_m: float,
                                     steer_ratio: float,
                                     steer_sw_max_deg: float,
                                     model_v2=None) -> float:
  max_lat_accel = 8.5   # m/s^2
  max_lat_jerk  = 4.0   # m/s^3
  y_std_1s = 0.1
  if model_v2 is not None and len(model_v2.position.yStd) > 10:
    model_y_std_1s = float(model_v2.position.yStd[10])
    if np.isfinite(model_y_std_1s) and model_y_std_1s >= 0.0:
      y_std_1s = model_y_std_1s
  max_sw_rate_deg_per_tick = float(np.interp(y_std_1s, [0.1, 0.2, 0.4], [2.0, 1.5, 0.8]))
  v = max(float(v_ego), 1.0)

  target_sw = clip_float(desired_sw_deg, -steer_sw_max_deg, steer_sw_max_deg)
  if v_ego < MID_SPEED_ANGLE_RATE_LIMIT_SPEED:
    # Keep low/mid-speed angle commands quieter without reducing LKAS_ANGLE_MAX_TORQUE.
    # Allow the angle, but slow the arrival: 0~15 kph ramps 0.8->1.1 deg/tick,
    # then 15~40 kph tapers 1.1->0.8 deg/tick. Above 40 kph, physics limits take over.
    low_mid_speed_cap = float(np.interp(v_ego,
                                        [0.0, LOW_SPEED_ANGLE_RATE_RAMP_SPEED, MID_SPEED_ANGLE_RATE_LIMIT_SPEED],
                                        [0.8, 1.1, 0.8]))
    max_sw_rate_deg_per_tick = min(max_sw_rate_deg_per_tick, low_mid_speed_cap)

  target_rw = target_sw / steer_ratio
  last_rw   = float(last_sw_deg) / steer_ratio

  # --- accel limit ---
  rw_max_rad = np.arctan((max_lat_accel * wheelbase_m) / (v * v))
  rw_max = float(np.degrees(rw_max_rad))

  # --- jerk -> rate limit ---
  sec2 = 1.2
  max_drw_dt = (max_lat_jerk * wheelbase_m) / (v * v * sec2)     # rad/s
  max_drw_per_tick = max_drw_dt * DT_CTRL                        # rad/tick
  max_drw_per_tick_deg = float(np.degrees(max_drw_per_tick))

  err = abs(target_sw - last_sw_deg)
  if err > 20.0:
    max_sw_rate_deg_per_tick = min(max_sw_rate_deg_per_tick, 1.0)

  # Once steering is in the large-angle region, do not make the return toward center wait on
  # the normal low-speed/large-error cap. This only relaxes unwinding; winding farther into the
  # turn keeps all existing limits.
  unwinding_large_angle = abs(last_sw_deg) >= MAX_ANGLE and last_sw_deg * (target_sw - last_sw_deg) < 0.0
  if unwinding_large_angle:
    max_sw_rate_deg_per_tick = max(max_sw_rate_deg_per_tick, LARGE_ANGLE_UNWIND_RATE)

  max_drw_per_tick_deg = min(
    max_drw_per_tick_deg,
    max_sw_rate_deg_per_tick / steer_ratio
  )

  # --- rate limit ---
  cmd_rw = rate_limit(target_rw, last_rw, -max_drw_per_tick_deg, max_drw_per_tick_deg)

  # --- accel clip ---
  cmd_rw = clip_float(cmd_rw, -rw_max, rw_max)

  if not lat_active:
    cmd_rw = float(steering_sw_deg) / steer_ratio

  cmd_sw = cmd_rw * steer_ratio
  return clip_float(cmd_sw, -steer_sw_max_deg, steer_sw_max_deg)

def park_assist_active(CS):
  lfa = getattr(CS, "lfa", None)
  return lfa is not None and lfa.get("VALUE63", 0) == PARK_ASSIST_VALUE63 and CS.out.vEgoRaw * CV.MS_TO_KPH < PARK_ASSIST_MAX_KPH


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.accel_value_last = 0.0
    self.display_lead_lateral = hyundaicanfd.DisplayLeadLateralFilter()
    self.canfd_stopping = (CanfdStopping()
                           if CP.flags & HyundaiFlags.CANFD and CP.openpilotLongitudinalControl else None)
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    pedal_interceptor_params = get_pedal_interceptor_params(CP)
    self.pedal_interceptor = (PedalInterceptor(CANPacker(dbc_names[Bus.party]), pedal_interceptor_params)
                              if pedal_interceptor_params is not None else None)
    self.last_button_frame = 0

    self.hyundai_jerk = HyundaiJerk()
    self.speedCameraHapticEndFrame = 0
    self.hapticFeedbackWhenSpeedCamera = 0
    self.max_angle_frames = MAX_ANGLE_FRAMES
    self.steer_angle_test = 0
    self.aprk_diag_active = False
    self.spas_steer_test = 0  # SpasSteerTest: target angle through the parking ECU's SPAS1 request, 0 off
    self.spas_angle, self.aprk_buses = 0., ()
    self.blinking_signal = False # 1Hz
    self.blinking_frame = int(1.0 / DT_CTRL)
    self.soft_hold_mode = 2
    self.depart_jerk = False
    self.blinker_control = False
    self.blinker_kick_mode = 1  # CanfdBlinkerKickMode: 0 only after an ack, 1 panda piggyback, 2 direct 25 Hz, 3 direct 100 Hz
    self.spas2_only = 0  # CanfdBlinkerSpas2Only: 0 SPAS1+SPAS2, 1 ECU left on + SPAS2 only, 2 ECU silenced + SPAS2 only, 3/4 = 1 in bursts
    self.aprk_spas2_only = 0  # the value the running APRK session started with
    self.spas2_sent = -SPAS2_BURST_OFF - 1  # last frame any SPAS2 went out; a 3/4 session starts SPAS2_BURST_OFF after it
    # set by card's KickOutput only on the CANFD_BLINKER_KICK harness (carrot-wip's kick_compatible); our Kick does not gate on it
    self.kick_output_ready = False
    self.remote_blinker = 0  # card: 3 left / 4 right from a Bluetooth remote blinker press, taken by kick_pulse
    self.remote_pulse = False  # the running pulse is a remote press: any gear, no CanfdBlinkerKick needed
    self.blinker_kick_model = False  # CanfdBlinkerKickModelMode != 1: also kick on a model lane change start (kick_pulse)
    self.model_flashes = 0  # CanfdBlinkerKickModelMode 2/3/4 -> 3/5/7 flashes; 0 = hold through the change
    self.blinker_test = 0  # CanfdBlinkerMenuTest: 1 left, 2 right (menu test, any speed/gear)
    self.hazard_test = 0  # CanfdHazardTest (remote hazard, 2 = fast flash): SPAS2 hazards when no camera 0x1ea carries bit 104
    self.hazard_shot_sent = False  # CanfdHazardTest 3: the single value-1 frame went out (resets when the value leaves 3)
    self.hazard_shot_frame = 0
    self.blinker_stalk = 0  # BlinkerStalkTest: 1 left, 2 right (0x3C1 replay)
    self.highbeam_frame = None       # frame the latest push started
    self.highbeam_kind = 1           # 1 high beam, 2 flash-to-pass
    self.highbeam_test = 0           # BlinkerStalkTest 5/6 -> 1/2: one high beam / flash-to-pass now (all cars)
    self.highbeam_test_sent = 0      # the kind last sent; a new value (or 0 in between) sends again
    # CanfdBvmView: 0 off, 1 left, 2 right, 3 both. Spoofs the cluster camera view status (0x400 byte4) so the
    # blind-spot camera is shown without the turn signal. Display-only message, nothing else uses it.
    self.bvm_view = 0
    self.bvm_alt_time = 10  # CanfdBvmAltTime: 0.1 s per side in CanfdBvmView 4
    self.bvm_reverse = 0    # CanfdBvmReverse: 1 left, 2 right while the gear is R (owns the view there)
    self.palisade_cgw_test = PalisadeCgwTest()
    self.palisade_blinker_test = 0
    self.stalk_counter = None
    self.stalk_source_counter = None
    self.stalk_values = None
    self.stalk_press_side = 0   # 1/2 while the lever is being held/tapped
    self.stalk_press_frame = 0
    self.aprk_state, self.aprk_frame, self.aprk_nanos, self.aprk_acked = APRK_IDLE, 0, 0, False
    self.lane_crossing, self.blinker_side, self.blinker_pulse_end = LaneCrossing(), 0, 0
    self.lane_change_prev, self.model_pulse = "off", False  # last modelV2 laneChangeState; pulse from the model trigger
    self.blinker_pulse_start, self.model_hold = 0, {}  # pulse start frame (Palisade flash phase); _blink_hold_change state
    # classic CAN with a stock CGW1 (0x541), any model since 11-14: panda only lets CGW1 out with this flag (interface)
    self.cgw_kick = bool(CP.safetyConfigs) and bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.PALISADE_CGW_TEST)
    self.cgw_trial, self.cgw_last_tx = 0, -CGW_REARM_FRAMES  # frames: our panda CGW1 trial start, last CGW1 request
    self.yellow_until = [0, 0]  # left, right
    self.standstill_frame = -1e6

    self.activateCruise = 0
    self.button_wait = 12
    self.cruise_buttons_msg_values = None
    self.cruise_buttons_msg_cnt = 0
    self.button_spamming_count = 0
    self.prev_clu_speed = 0
    self.button_spam1 = 8
    self.button_spam2 = 30
    self.button_spam3 = 1

    self.apply_angle_last = 0
    self.steering_template_ready = False
    self.lkas_max_torque = 0
    self.angle_max_torque = 250
    self.steering_pressed_prev = False
    self.recovering_from_override = False
    self.full_recovery_frames = 0
    self.repeated_override_count = 0
    self.steering_pressed_frames = 0
    self.override_latched = False
    self.override_release_frames = 0
    self.driver_torque_filtered = 0.0
    self.driver_torque_filtered_prev = 0.0
    self.pre_override_frames = 0

    self.steer_handover = SteeringHandover()
    self.handover_model_frame = None
    self.handover_model_time = 0

    self.lkas11_active = False

    self.canfd_debug = 0
    self.MainMode_ACC_trigger = 0
    self.LFA_trigger = 0

    self.activeCarrot = 0
    self.camera_scc_params = Params().get_int("HyundaiCameraSCC")
    self.is_ldws_car = Params().get_bool("IsLdwsCar")
    self.enable_corner_radar = 0
    self.paddle_mode = Params().get_int("PaddleMode")

    self.steerDeltaUpOrg = self.steerDeltaUp = self.steerDeltaUpLC = self.params.STEER_DELTA_UP
    self.steerDeltaDownOrg = self.steerDeltaDown = self.steerDeltaDownLC = self.params.STEER_DELTA_DOWN

  def stalk_blinkers(self, CS):
    # BlinkerStalkTest 1/2 hold the lever, 3/4 replay GV80's momentary tap. Both start with the BLINKER_TAP edge.
    if self.blinker_stalk not in (1, 2, 3, 4):
      self.stalk_counter = self.stalk_source_counter = self.stalk_values = None
      self.stalk_press_side = 0
      return []
    momentary = self.blinker_stalk in (3, 4)
    side = 1 if self.blinker_stalk in (1, 3) else 2
    if side != self.stalk_press_side:            # rising edge: one press starts here
      self.stalk_press_side, self.stalk_press_frame = side, self.frame
    pressed_for = self.frame - self.stalk_press_frame
    if momentary and pressed_for >= STALK_TAP_MOMENTARY_FRAMES:
      return []                                  # GV80's lever springs back, so stop after the tap
    # Match the stock rate: the column module sends 0x3C1 at 5 Hz. Modes 1/2 keep the old 20 Hz.
    if pressed_for % (STALK_MOMENTARY_HZ_DIV if momentary else 5) != 0 or CS.blinker_stalks is None:
      return []
    # GV80 holds the tap form for the whole press and then releases; the hold cars switch to the steady form after 0.2 s
    tap = pressed_for < (STALK_TAP_MOMENTARY_FRAMES if momentary else STALK_TAP_FRAMES)
    values = dict(CS.blinker_stalks)
    source_counter = int(values.pop("COUNTER_ALT"))
    values.pop("CHECKSUM_MAYBE", None)
    values.update(LEFT_BLINKER=int(side == 1), RIGHT_BLINKER=int(side == 2),
                  LEFT_BLINKER_TAP=int(tap and side == 1), RIGHT_BLINKER_TAP=int(tap and side == 2))
    # Stock captures repeat a counter until the lever state changes; BCM acceptance remains unverified.
    if getattr(self, "stalk_counter", None) is None or source_counter != getattr(self, "stalk_source_counter", None):
      self.stalk_counter = (source_counter + 1) % 16
    elif values != self.stalk_values:
      self.stalk_counter = (self.stalk_counter + 1) % 16
    self.stalk_source_counter, self.stalk_values = source_counter, values.copy()
    values.update(COUNTER_ALT=self.stalk_counter, CHECKSUM_MAYBE=0)
    addr, dat, bus = self.packer.make_can_msg("BLINKER_STALKS", self.CAN.ECAN, values)
    return [CanData(addr, bytes([stalk_checksum(dat[1:])]) + bytes(dat[1:]), bus)]

  def highbeam_elapsed(self, ready):
    """Frames since the current high-beam signal started, None before the first one. BlinkerStalkTest 5 (high beam)
    or 6 (flash-to-pass) sends once when the menu value becomes it, at any gear/speed/engagement, so users can check it
    without engaging or a remote (user 11-18; HighBeamTest merged into BlinkerStalkTest 11-19). carcontroller writes
    no params: the same value never repeats, a different value (5 <-> 6) or 0 in between sends again. Not ready
    (message not seen yet): it stays pending. The engage push (CanfdHighBeamOnEngage) was removed 11-18."""
    if not self.highbeam_test:
      self.highbeam_test_sent = 0
    if self.highbeam_test in (1, 2) and self.highbeam_test != self.highbeam_test_sent and ready:
      self.highbeam_frame, self.highbeam_kind, self.highbeam_test_sent = self.frame, self.highbeam_test, self.highbeam_test
    return None if self.highbeam_frame is None else self.frame - self.highbeam_frame

  def cgw_highbeam(self, CS):
    """BlinkerStalkTest 5/6 on classic CAN (user 11-18): the lever is not on CAN there, so this is the BCM's own CGW1 (0x541)
    with CF_Gway_HeadLampHigh (1: byte4 0x01) or CF_Gway_PassingSW (2: byte6 0x08) set for 0.15 s. CGW1 is the
    gateway's status broadcast: forged CGW1 lit the K5 DL3 cluster, never the exterior lamps (11-14). panda sends it
    behind the stock frame inside the kick's CGW1 trial (safety_hyundai.h hyundai_cgw_highbeam_fits)."""
    raw = getattr(CS, "cgw_test_raw", b"")
    elapsed = self.highbeam_elapsed(len(raw) == 8)
    if elapsed is None or elapsed >= HIGHBEAM_PRESS_FRAMES or elapsed % CGW_TX_FRAMES or len(raw) != 8:
      return []
    if self.frame - self.cgw_last_tx >= CGW_REARM_FRAMES:
      self.cgw_trial = self.frame  # same panda trial bookkeeping as cgw_blinkers
    self.cgw_last_tx = self.frame
    data = bytearray(raw)
    data[4 if self.highbeam_kind == 1 else 6] |= 0x01 if self.highbeam_kind == 1 else 0x08
    return [CanData(0x541, bytes(data), 0)]

  def highbeam_push(self, CS):
    """BlinkerStalkTest 5/6 on CAN-FD (user 11-18): the high-beam lever once like the driver would on 0x3C1, 1 pushed
    (HIGHBEAM_FORWARD, byte2 bit 0x04) or 2 pulled (HIGHBEAM_BACKWARD flash-to-pass, byte3 bit 0x04), released 0.15 s
    later; panda's 0x3C1 shape check allows both. Each frame is the stock frame with that bit and COUNTER_ALT +1 per
    lever change (stalk_blinkers). Possibly ignored: our turn-signal replay on this id reached the bus but not the BCM
    (05_can/blinker_route19_20261026.md)."""
    stalks = getattr(CS, "blinker_stalks", None)  # CAN-FD CarState sets it (None until 0x3C1 is seen)
    elapsed = self.highbeam_elapsed(stalks is not None)
    if elapsed not in (0, HIGHBEAM_PRESS_FRAMES) or stalks is None:
      return []
    values = dict(stalks)
    source_counter = int(values.pop("COUNTER_ALT"))
    values.pop("CHECKSUM_MAYBE", None)
    signal = "HIGHBEAM_BACKWARD" if self.highbeam_kind == 2 else "HIGHBEAM_FORWARD"
    values.update({signal: int(elapsed == 0)}, COUNTER_ALT=(source_counter + (1 if elapsed == 0 else 2)) % 16,
                  CHECKSUM_MAYBE=0)
    addr, dat, bus = self.packer.make_can_msg("BLINKER_STALKS", self.CAN.ECAN, values)
    return [CanData(addr, bytes([stalk_checksum(dat[1:])]) + bytes(dat[1:]), bus)]

  def bvm_view_msg(self, reverse=False):
    # 0x400 BVM_VIEW (byte 4) is what the car broadcasts at 5 Hz while a blind-spot camera is on:
    # 1..3 left, 4..6 right, 0 off (09-12 and 10-26 logs). Stock rate (5 Hz) only: a faster same-ID copy is the
    # collision that took the Azera bus down (STALK_MOMENTARY_HZ_DIV).
    if reverse:
      val = 3 if self.bvm_reverse == 1 else 6  # CanfdBvmReverse (user 11-19) owns the view while in R
    elif self.bvm_view == 4:
      # The cluster only has one camera window (byte 4 is a 3-bit single-window state), so both sides cannot be
      # shown at once. Alternate them so both are visible in turn, CanfdBvmAltTime per side.
      # Floor 0.2 s = one 5 Hz frame per side: switching faster needs a faster same-ID copy than stock (collision risk)
      period = max(20, int(self.bvm_alt_time * 0.1 / DT_CTRL))
      val = 3 if (self.frame // period) % 2 == 0 else 6
    elif self.bvm_view >= 11:
      val = self.bvm_view - 10  # 11..16: byte 4 = 1..6 as-is (car: 1..3 left, 4..6 right); was CanfdBvmRaw until 11-19
    else:
      val = {1: 3, 2: 6, 3: 7}.get(self.bvm_view, 0)
    if self.frame % 20 != 0:
      return []
    return [CanData(0x400, bytes([0, 0, 0, 0, val, 0, 0, 0]), self.CAN.ECAN)]

  def kick_pulse(self, CS):
    """CanfdBlinkerKick trigger, any CAN: starts a pulse (BLINKER_PULSE_FRAMES at least) toward blinker_side (3 left, 4 right) with
    the lamp off, in D, outside a tunnel / roundabout. True while a pulse runs.
    - the driver crossing a lane line (lane_crossing), not across a yellow line and not during a model lane change:
      nothing owns that lamp, so it must not light up a change DesireHelper is running.
    - CanfdBlinkerKickModelMode (not 1): modelV2 laneChangeState rising to laneChangeStarting (remote, app, voice, navigation), only
      when that side's indicatorIsolationMask bit rides in the same modelV2: DesireHelper owned the lamp on that frame, so
      the kicked lamp cannot read back as the driver's blinker (EV4 10-26: a lamp held through the change never went off,
      measurements 31: a kick lamp started a change). Not in a cruise CANCEL latch; cut when the change is cancelled.
      Mode 0: held through Starting / Finishing like the I5 PE cluster blink hold: until the car centre is 60 % of a lane
      width into the new lane (no crossing seen: the end of the change), at most MODEL_HOLD_MAX_FRAMES (classic CAN: the
      5 s panda trial, cgw_blinkers).
      Modes 2/3/4: a fixed 3/5/7 flashes (MODEL_FLASH_FRAMES each) that also run past a completed change, like the stock
      one-touch; a cancelled change still cuts them.
    - Modes 2/3/4 also count the driver-crossing pulse on classic CAN (cgw_kick) only: there carState reads the stock CGW1,
      which never shows our forged lamp (Palisade, K5 P tests). CAN-FD reads the real lamp back as the driver's blinker, so a
      longer unowned pulse there could start a second change: it stays BLINKER_PULSE_FRAMES."""
    # carrot-wip 00d9cae3 gates, kept without its KickOutput lease: no Kick during a cruise CANCEL latch, while the driver
    # works the lever, or with a trailer
    drive = (CS.out.gearShifter == structs.CarState.GearShifter.drive and
             not getattr(CS.out, 'cruiseCancelState', False) and not getattr(CS.out, 'blinkerLever', False) and
             not getattr(CS.out, 'trailerConnected', False))
    lamp = CS.out.leftBlinker or CS.out.rightBlinker
    # lane changes are banned in a tunnel announced as no-lane-change, and for 20 s after a roundabout sign (carstate)
    tunnel = getattr(CS.out, "naviTunnel", False) or getattr(CS.out, "naviRoundabout", False)
    md = getattr(CS, "modelV2", None)
    meta = md.meta if md is not None else None
    state = str(meta.laneChangeState) if meta is not None else None  # None: no fresh modelV2, not a transition
    changing = state in ("laneChangeStarting", "laneChangeFinishing")
    side = self.lane_crossing.update(md, self.frame * DT_CTRL,
                                     self.blinker_control and drive and not lamp and not tunnel and not changing,
                                     CS.out.steeringTorque, torque_min=self.params.STEER_THRESHOLD)  # 250 CAN-FD, 150 classic
    for i, line in enumerate((CS.out.leftLaneLine, CS.out.rightLaneLine)):
      if line // 10 == 2:  # camera color: 0 none, 1 white, 2 yellow, 3 blue
        self.yellow_until[i] = self.frame + YELLOW_HOLD_FRAMES
    if side and self.frame < self.yellow_until[side - 3]:
      side = 0
    model = False
    if state is not None:
      prev = self.lane_change_prev
      bit = {"left": 1, "right": 2}.get(str(meta.laneChangeDirection), 0)  # boundary_indicator.own() bits
      # no yellow-line guard: the model FSM already decided to change lanes there
      model = (state == "laneChangeStarting" and self.lane_change_prev != state and bit and self.blinker_control and
               self.blinker_kick_model and drive and not lamp and not tunnel and
               not getattr(CS.out, "cruiseCancelState", False) and bool(meta.indicatorIsolationMask & bit))
      self.lane_change_prev = state
      if model:
        side = bit + 2
      elif self.model_flashes and prev == "laneChangeFinishing" and state != prev:
        # completed (to off, or to a queued next change's preLaneChange): the counted flashes run out like a stock one-touch
        self.model_pulse = False
      elif state == "off" and self.model_pulse and self.frame < self.blinker_pulse_end:
        self.blinker_pulse_end = self.frame  # the change was cancelled: so is its lamp
      elif (self.model_pulse and not self.model_flashes and changing and self.frame < self.blinker_pulse_end and
            self.frame - self.blinker_pulse_start < MODEL_HOLD_MAX_FRAMES and
            hyundaicanfd._blink_hold_change(md, self.blinker_side, self.model_hold)):
        self.blinker_pulse_end = max(self.blinker_pulse_end, self.frame + MODEL_HOLD_GAP_FRAMES)
    # a Bluetooth remote blinker press is the driver's own request, like a lever tap: any gear (P included), no Kick key
    remote, self.remote_blinker = self.remote_blinker, 0
    if remote:
      side = remote
    if side:
      length = self.model_flashes * MODEL_FLASH_FRAMES if self.model_flashes and (model or self.cgw_kick) else BLINKER_PULSE_FRAMES
      if remote:
        length = REMOTE_FLASHES * MODEL_FLASH_FRAMES
      self.blinker_side, self.blinker_pulse_end, self.model_pulse = side, self.frame + length, model
      self.blinker_pulse_start, self.model_hold, self.remote_pulse = self.frame, {}, bool(remote)
    if not drive and not self.remote_pulse:
      self.blinker_pulse_end, self.model_pulse = min(self.blinker_pulse_end, self.frame), False
    if self.frame >= self.blinker_pulse_end:
      self.remote_pulse = False
    return self.frame < self.blinker_pulse_end

  def restart_pulse(self):
    """The APRK handshake ate the start of the pulse: run its full length again from now (1 s, or a counted 3/5/7)."""
    length = self.blinker_pulse_end - self.blinker_pulse_start
    self.blinker_pulse_start, self.blinker_pulse_end = self.frame, self.frame + length

  def cgw_blinkers(self, CS):
    """Classic CAN kick (Palisade, any model with a stock CGW1 since 11-14): the BCM's own CGW1 (0x541) with the lever side and the lamp bit flashing 0.35 s like
    stock - the PalisadeBlinkerTest frame, while driving in D. panda holds the newest frame and sends it right behind
    the next stock CGW1 (safety_hyundai.h hyundai_rx_tx_hook): one per stock frame (10 Hz), never starting together.
    No yellow-line / tunnel guard here: classic carstate has no lane colour or navi events."""
    pulse = self.kick_pulse(CS)
    raw = getattr(CS, "cgw_test_raw", b"")
    if not pulse or (CS.out.gearShifter != structs.CarState.GearShifter.drive and not self.remote_pulse) or len(raw) != 8:
      return []
    if raw[4] & 0x06 or raw[5] & 0x06:  # hazard or the driver's own lever: theirs, not ours
      self.blinker_pulse_end = 0
      return []
    hold = self.model_pulse and not self.model_flashes  # mode 0: the end follows the change, unknown up front
    if self.frame - self.cgw_last_tx >= CGW_REARM_FRAMES:
      self.cgw_trial = self.frame  # panda rearmed: this pulse opens a new 5 s trial
    elif self.cgw_trial < self.blinker_pulse_start and (hold or self.blinker_pulse_end > self.cgw_trial + CGW_TRIAL_FRAMES):
      # started inside an earlier pulse's trial and would not end there: dark from its start until panda rearms, then
      # in full - never split mid-way. A hold keeps its end, so a change that ends meanwhile leaves no late flash.
      self.blinker_pulse_start += 1
      self.blinker_pulse_end += 0 if hold else 1
      return []
    elif self.frame - self.cgw_trial >= CGW_TRIAL_FRAMES:
      self.blinker_pulse_end = self.frame  # this pulse used its whole trial (the mode 0 hold): it ends here
      return []
    if self.frame % CGW_TX_FRAMES:
      return []
    self.cgw_last_tx = self.frame
    lamp = (self.frame - self.blinker_pulse_start) // CGW_FLASH_HALF_FRAMES % 2 == 0
    return [CanData(*cgw_frame(raw, self.blinker_side - 2, lamp))]

  def aprk_blinkers(self, CC, CS):
    ret = []
    drive = CS.out.gearShifter == structs.CarState.GearShifter.drive
    pulse = self.kick_pulse(CS)
    # the parking ECU is paused only around a kick and restored right after it (menu test: any speed or gear)
    # SpasSteerTest: parked, stopped, hands off. APRK is asked on both buses - bus 0 ignored it 10-26, and a second SPAS1
    # stream with its own counter runs on bus 1 (10-30)
    spas = (self.spas_steer_test != 0 and CS.out.gearShifter == structs.CarState.GearShifter.park and
            CS.out.standstill and not CS.out.steeringPressed)
    # carrot-wip 00d9cae3: never against the other side's lamp; a Kick that may not run cuts its pulse
    opposite = CS.out.rightBlinker if self.blinker_side == 3 else CS.out.leftBlinker
    kick = self.blinker_control and drive and pulse and not opposite
    remote = self.remote_pulse and pulse
    if not (kick or remote):
      self.blinker_pulse_end, self.model_pulse = min(self.blinker_pulse_end, self.frame), False
    # CanfdHazardTest on a car whose camera sends no 0x1ea (HDA1 Tucson): bit 104 has no carrier, so SPAS2 BLINKER_CONTROL 1
    # (DBC "hazards") asks for them instead, any gear, while the test is on. A car with 0x1ea keeps bit 104 (hyundaicanfd).
    haz_mode = self.hazard_test if getattr(CS, "adrv_0x1ea", None) is None else 0
    if haz_mode not in (0, 1, 2) and (getattr(CS, "controls_ready_count", 0) <= HAZARD_SPAS2_READY or
                                      0x1EA in getattr(getattr(CS, "cp_cam", None), "seen_addresses", ())):
      haz_mode = 0
    if haz_mode != 3:
      self.hazard_shot_sent = False
    hazard = haz_mode in (1, 2) or (haz_mode == 3 and not self.hazard_shot_sent)
    alternate = haz_mode == 4  # left/right in turn, HAZARD_ALT_FRAMES each
    raw = haz_mode - HAZARD_RAW_FIRST if HAZARD_RAW_FIRST <= haz_mode <= HAZARD_RAW_FIRST + 7 else None
    if haz_mode == 5:  # stock remote-parking rhythm: 2 then 1, HAZARD_STOCK_FRAMES each
      raw = 2 if (self.frame // HAZARD_STOCK_FRAMES) % 2 == 0 else 1
    want = self.blinker_test in (1, 2) or kick or remote or spas or hazard or alternate or raw is not None
    spas2_only = 0 if spas else self.spas2_only  # SpasSteerTest: its request is SPAS1 itself
    now, stock, ack = getattr(CS, "can_nanos", 0), getattr(CS, "aprk_stock_nanos", 0), getattr(CS, "aprk_ack_nanos", 0)

    def give_back():
      restore = [CanData(APRK_ADDR, APRK_DEFAULT_SESSION, bus) for bus in self.aprk_buses]
      self.aprk_state, self.aprk_buses = (APRK_FAILED if want else APRK_IDLE), ()
      return ret + restore

    if not want:
      ret = give_back()
    elif self.aprk_state != APRK_IDLE and spas2_only != self.aprk_spas2_only:
      # the value changed mid-session, or SpasSteerTest joined/left a SPAS2-only one: end it (10 01 if the ECU was
      # silenced) and start over next frame, so one run is never half one value and half another (review 11-19)
      ret += [CanData(APRK_ADDR, APRK_DEFAULT_SESSION, bus) for bus in self.aprk_buses]
      self.aprk_state, self.aprk_buses = APRK_IDLE, ()
    elif self.aprk_state == APRK_IDLE and spas2_only in (1, 3, 4):
      # no handshake: aprk_buses stays empty, so no tester present and no 10 01 restore; not acked, so a stock frame
      # never ends it (KickMode 2 still thins it to 25 Hz)
      # 3/4: a restart (value change, test or kick ending and coming back) still owes the last burst its gap
      start = max(self.frame, self.spas2_sent + SPAS2_BURST_OFF + 1) if spas2_only in SPAS2_BURST_ON else self.frame
      self.aprk_state, self.aprk_frame, self.aprk_nanos, self.aprk_acked = APRK_ACTIVE, start, now, False
      self.aprk_spas2_only = spas2_only
    elif self.aprk_state == APRK_IDLE:
      self.aprk_buses, self.aprk_spas2_only = (self.CAN.ECAN, self.CAN.ACAN), spas2_only
      ret += [CanData(APRK_ADDR, APRK_EXT_SESSION, bus) for bus in self.aprk_buses]
      self.aprk_state, self.aprk_frame, self.aprk_nanos = APRK_ASK, self.frame, now
    elif self.aprk_state == APRK_ASK:
      if ack >= self.aprk_nanos and ack > 0:
        ret += [CanData(APRK_ADDR, APRK_COM_DISABLE, bus) for bus in self.aprk_buses]
        self.aprk_state, self.aprk_frame = APRK_WAIT_QUIET, self.frame
      elif self.frame - self.aprk_frame >= APRK_REPLY_FRAMES:
        if spas or self.blinker_kick_mode == 0:
          return give_back()  # steering next to the ECU's own SPAS1 is not tried; mode 0 blinks only after an ack
        # No answer (this car: 0 of ~180 requests, 10-24..11-05): the stock stream stays on the bus. Mode 1: panda holds
        # each of our SPAS frames and sends it right behind the stock one (hyundai_canfd_rx_tx_hook), so the same id
        # never starts twice at once - 10-30..11-02 that collision made error frames in 78/111 kicks and bus-off in 33.
        # Modes 2/3 (CANFD_SPAS_DIRECT): frames are queued as they come, at 25 Hz / 100 Hz - fewer / the old collisions.
        self.aprk_state, self.aprk_frame, self.aprk_nanos, self.aprk_acked = APRK_ACTIVE, self.frame, now, False
        if pulse and self.blinker_test not in (1, 2):
          self.restart_pulse()
    elif self.aprk_state == APRK_WAIT_QUIET:
      if self.frame - self.aprk_frame >= APRK_QUIET_NANOS // 10_000_000 and now - stock >= APRK_QUIET_NANOS:
        self.aprk_state, self.aprk_frame, self.aprk_nanos, self.aprk_acked = APRK_ACTIVE, self.frame, now, True
        if spas:
          self.spas_angle = CS.out.steeringAngleDeg
        elif pulse and self.blinker_test not in (1, 2):
          self.restart_pulse()  # the handshake ate part of the pulse
      elif self.frame - self.aprk_frame >= APRK_REPLY_FRAMES:
        return give_back()
    elif self.aprk_state == APRK_ACTIVE:
      if self.aprk_acked and stock > self.aprk_nanos:
        return give_back()  # the ECU came back: stop before our frames collide with its own
      if self.frame % 100 == 0:
        ret += [make_tester_present_msg(APRK_ADDR, bus, suppress_response=True) for bus in self.aprk_buses]
      if spas:
        target = clip_float(self.spas_steer_test, -STEER_ANGLE_TEST_MAX, STEER_ANGLE_TEST_MAX)
        self.spas_angle = rate_limit(target, self.spas_angle, -SPAS_STEER_RATE, SPAS_STEER_RATE)
        mode = 4 if self.frame - self.aprk_frame <= SPAS_STEER_ASK_FRAMES else 5
        return ret + hyundaicanfd.create_spas_steer(self.packer, self.CAN, mode, self.spas_angle)
      if (haz_mode == 3 and self.hazard_shot_sent and not self.aprk_acked and self.blinker_kick_mode == 1 and
          self.frame - self.hazard_shot_frame < HAZARD_SHOT_HOLD_FRAMES):
        return ret  # let a stock frame carry the held value-1 frame first
      # every frame (100 Hz), twice APRK's 50 Hz: 10-26 APRK stayed on, so its BLINKER_CONTROL 0 frames interleave with ours
      if not self.aprk_acked and self.blinker_kick_mode == 2 and self.frame % 4:
        return ret  # direct 25 Hz: a quarter of the frames, a quarter of the collision chances
      burst = SPAS2_BURST_ON.get(spas2_only)
      k = self.frame - self.aprk_frame  # < 0 while a restarted session waits out the gap
      if burst and (k < 0 or k % (burst + SPAS2_BURST_OFF) >= burst):
        return ret  # burst gap: the Sorento ESC never sees 0.5 s of our SPAS2 in a row
      side = (self.frame // HAZARD_ALT_FRAMES) % 2
      left = (pulse and self.blinker_side == 3) or self.blinker_test == 1 or (alternate and side == 0)
      right = (pulse and self.blinker_side == 4) or self.blinker_test == 2 or (alternate and side == 1)
      haz = hazard and (haz_mode == 3 or hyundaicanfd.hazard_request(haz_mode, self.frame))
      ret.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, left, right, getattr(CS, "spas2", None),
                                                         haz, spas1=not spas2_only, raw=raw))
      self.spas2_sent = self.frame
      if haz_mode == 3 and haz:
        self.hazard_shot_sent, self.hazard_shot_frame = True, self.frame  # given back next frame unless another test runs
    return ret

  def update(self, CC, CS, now_nanos):

    if not self.steering_template_ready and self.CP.flags & HyundaiFlags.CANFD and self.CP.flags & HyundaiFlags.CAMERA_SCC:
      template = CS.lfa_alt if self.CP.flags & HyundaiFlags.ANGLE_CONTROL else CS.lfa
      self.steering_template_ready = template is not None
      if not self.steering_template_ready and CC.latActive:
        # Do not run an invisible steering ramp while camera TX discovery is
        # pending. Inactive handling below tracks the wheel and clears torque
        # and handover history, so the first emitted command starts bounded.
        CC = CC.as_builder()
        CC.latActive = False
        CC = CC.as_reader()

    if CC.latActive and self.CP.carFingerprint in PARK_ASSIST_YIELD_CARS and park_assist_active(CS):
      CC = CC.as_builder()
      CC.latActive = False
      CC = CC.as_reader()

    if self.frame % 50 == 0:
      params = Params()
      self.max_angle_frames = params.get_int("MaxAngleFrames")
      self.steer_angle_test = params.get_int("SteerAngleTest")
      self.spas_steer_test = params.get_int("SpasSteerTest")
      self.hazard_test = params.get_int("CanfdHazardTest")
      if self.hazard_test != 3:
        self.hazard_shot_sent = False  # at 0 the update() gate skips aprk_blinkers, so 0 then 3 must reset here
      steerMax = params.get_int("CustomSteerMax")
      steerDeltaUp = params.get_int("CustomSteerDeltaUp")
      steerDeltaDown = params.get_int("CustomSteerDeltaDown")
      steerDeltaUpLC = params.get_int("CustomSteerDeltaUpLC")
      steerDeltaDownLC = params.get_int("CustomSteerDeltaDownLC")
      if steerMax > 0:
        self.params.STEER_MAX = steerMax
      if steerDeltaUp > 0:
        self.steerDeltaUp = steerDeltaUp
        #self.params.ANGLE_TORQUE_UP_RATE = steerDeltaUp
      else:
        self.steerDeltaUp = self.steerDeltaUpOrg
      if steerDeltaDown > 0:
        self.steerDeltaDown = steerDeltaDown
        #self.params.ANGLE_TORQUE_DOWN_RATE = steerDeltaDown
      else:
        self.steerDeltaDown = self.steerDeltaDownOrg

      if steerDeltaUpLC > 0:
        self.steerDeltaUpLC = steerDeltaUpLC
      else:
        self.steerDeltaUpLC = self.steerDeltaUp
      if steerDeltaDownLC > 0:
        self.steerDeltaDownLC = steerDeltaDownLC
      else:
        self.steerDeltaDownLC = self.steerDeltaDown

      self.soft_hold_mode = 1 if params.get_int("AutoCruiseControl") > 1 else 2
      self.depart_jerk = params.get_int("StopGoLongAssist") >= 2
      if self.canfd_stopping is not None:
        self.canfd_stopping.unheld_brake = params.get_int("StopGoLongAssist") >= 1  # stop guard level (stopping.py)
      self.blinker_control = params.get_bool("CanfdBlinkerKick")
      self.blinker_kick_mode = params.get_int("CanfdBlinkerKickMode")
      self.spas2_only = params.get_int("CanfdBlinkerSpas2Only")
      model_mode = params.get_int("CanfdBlinkerKickModelMode")  # 0 hold, 1 off, 2/3/4: 3/5/7 flashes
      self.blinker_kick_model, self.model_flashes = model_mode != 1, MODEL_FLASH_COUNTS.get(model_mode, 0)
      self.blinker_test = params.get_int("CanfdBlinkerMenuTest")
      stalk = params.get_int("BlinkerStalkTest")  # 1-4 lever replay (CAN-FD), 5/6 one high beam / flash-to-pass
      self.blinker_stalk = stalk if stalk in (1, 2, 3, 4) else 0
      self.highbeam_test = stalk - 4 if stalk in (5, 6) else 0
      view = params.get_int("CanfdBvmView")  # 1-4 views, 11-16 byte 4 = 1..6; anything else sends nothing
      self.bvm_view = view if view in (1, 2, 3, 4) or 11 <= view <= 16 else 0
      self.bvm_alt_time = params.get_int("CanfdBvmAltTime")
      self.bvm_reverse = params.get_int("CanfdBvmReverse")
      self.palisade_blinker_test = params.get_int("PalisadeBlinkerTest")
      self.hapticFeedbackWhenSpeedCamera = int(params.get_int("HapticFeedbackWhenSpeedCamera"))

      self.button_spam1 = params.get_int("CruiseButtonTest1")
      self.button_spam2 = params.get_int("CruiseButtonTest2")
      self.button_spam3 = params.get_int("CruiseButtonTest3")
      self.speed_from_pcm = params.get_int("SpeedFromPCM")

      self.canfd_debug = params.get_int("CanfdDebug")
      self.camera_scc_params = params.get_int("HyundaiCameraSCC")
      self.enable_corner_radar = params.get_int("EnableCornerRadar")
      self.paddle_mode = params.get_int("PaddleMode")

    actuators = CC.actuators
    hud_control = CC.hudControl

    if hud_control.modelDesire in [3,4]:
      self.params.STEER_DELTA_UP = self.steerDeltaUpLC
      self.params.STEER_DELTA_DOWN = self.steerDeltaDownLC
    else:
      self.params.STEER_DELTA_UP = self.steerDeltaUp
      self.params.STEER_DELTA_DOWN = self.steerDeltaDown

    angle_control = self.CP.flags & HyundaiFlags.ANGLE_CONTROL

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, self.max_angle_frames,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    #apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
    #                                           CS.out.steeringAngleDeg, CC.latActive, self.params.ANGLE_LIMITS)

    apply_angle = apply_steer_angle_limits_physics(
      actuators.steeringAngleDeg,
      self.apply_angle_last,
      CS.out.vEgoRaw,
      CS.out.steeringAngleDeg,
      CC.latActive,
      self.CP.wheelbase,
      self.CP.steerRatio,
      self.params.ANGLE_LIMITS.STEER_ANGLE_MAX,
      CS.modelV2,
    )


    if angle_control:
      apply_steer_req = CC.latActive

    angle_torque_cap = self.angle_max_torque

    steering_pressed_rising = CS.out.steeringPressed and not self.steering_pressed_prev
    if steering_pressed_rising:
      self.recovering_from_override = True
    self.steering_pressed_frames = self.steering_pressed_frames + 1 if CS.out.steeringPressed and CC.latActive else 0
    if self.steering_pressed_frames == OVERRIDE_COUNT_MIN_FRAMES:
      if 0 < self.full_recovery_frames < int(5.0 / DT_CTRL):
        self.repeated_override_count = min(self.repeated_override_count + 1, 3)
      self.full_recovery_frames = 0

    torque_threshold = max(self.params.STEER_THRESHOLD, 1.0)
    # Filter signed torque so alternating sensor noise cancels out before its
    # magnitude is used for pre-override prediction.
    driver_torque = float(CS.out.steeringTorque)
    driver_torque_abs = abs(driver_torque)
    if not CC.latActive:
      self.driver_torque_filtered = driver_torque
      self.driver_torque_filtered_prev = driver_torque
      self.pre_override_frames = 0
    else:
      torque_filter_alpha = DT_CTRL / (DRIVER_TORQUE_FILTER_TAU + DT_CTRL)
      self.driver_torque_filtered_prev = self.driver_torque_filtered
      self.driver_torque_filtered += torque_filter_alpha * (driver_torque - self.driver_torque_filtered)

    driver_torque_filtered_abs = abs(self.driver_torque_filtered)
    driver_torque_filtered_prev_abs = abs(self.driver_torque_filtered_prev)
    driver_torque_rate = max(0.0, (driver_torque_filtered_abs - driver_torque_filtered_prev_abs) / DT_CTRL)
    torque_ratio = driver_torque_filtered_abs / torque_threshold
    raw_torque_ratio = driver_torque_abs / torque_threshold
    predicted_torque_ratio = (
      driver_torque_filtered_abs + driver_torque_rate * PRE_OVERRIDE_PREDICTION_TIME
    ) / torque_threshold
    pre_override_candidate = (
      CC.latActive and
      not CS.out.steeringPressed and
      raw_torque_ratio > PRE_OVERRIDE_RAW_MIN_RATIO and
      torque_ratio > PRE_OVERRIDE_FILTERED_MIN_RATIO and
      predicted_torque_ratio > PRE_OVERRIDE_START_RATIO and
      driver_torque_rate > torque_threshold * PRE_OVERRIDE_MIN_RATE_RATIO
    )
    self.pre_override_frames = self.pre_override_frames + 1 if pre_override_candidate else 0
    pre_override_yield = 0.0
    if self.pre_override_frames >= PRE_OVERRIDE_CONFIRM_FRAMES:
      pre_override_yield = float(np.interp(
        predicted_torque_ratio,
        [PRE_OVERRIDE_START_RATIO, PRE_OVERRIDE_FULL_RATIO],
        [0.0, 1.0],
      ))
    recovery_allowed = False

    if CS.out.steeringPressed:
      # Start yielding immediately when driver override is confirmed.
      self.override_latched = True
      self.override_release_frames = 0
      torque_delta = -20.0
    elif pre_override_yield > 0.0:
      # Start handing off gently before steeringPressed flips to avoid a sharp torque drop.
      torque_delta = PRE_OVERRIDE_MAX_TORQUE_DELTA * pre_override_yield
    elif self.lkas_max_torque >= self.angle_max_torque:
      # Once fully recovered, hold full authority until the next driver override.
      torque_delta = 0.0
    elif self.override_latched:
      # Hold reduced authority until driver torque stays below 60% for 0.2 seconds.
      self.override_release_frames = self.override_release_frames + 1 if torque_ratio < 0.6 else 0
      if self.override_release_frames >= int(0.2 / DT_CTRL):
        self.override_latched = False
        self.override_release_frames = 0
        recovery_allowed = True
      else:
        torque_delta = 0.0
    else:
      recovery_allowed = True

    if recovery_allowed:
      # Use one-second model uncertainty to set the base torque recovery time.
      # Missing or invalid model data falls back to a moderate 1.5-second recovery.
      y_std_1s = 0.2
      if CS.modelV2 is not None and len(CS.modelV2.position.yStd) > 10:
        model_y_std_1s = float(CS.modelV2.position.yStd[10])
        if np.isfinite(model_y_std_1s) and model_y_std_1s >= 0.0:
          y_std_1s = model_y_std_1s

      recovery_time = float(np.interp(y_std_1s, [0.1, 0.2, 0.3, 0.4], [0.5, 0.8, 1.5, 3.0]))
      recovery_time = max(recovery_time, float(np.interp(
        self.repeated_override_count,
        [0, 1, 2, 3],
        [0.1, 1.0, 2.0, 3.0],
      )))
      base_rate_up = (self.angle_max_torque - self.params.ANGLE_MIN_TORQUE) * DT_CTRL / recovery_time

      # During recovery, taper the rate to zero. Only steeringPressed can reduce authority.
      torque_delta = base_rate_up * float(np.interp(torque_ratio, [0.6, 0.8], [1.0, 0.0]))
    self.lkas_max_torque = clip_float(self.lkas_max_torque + torque_delta, self.params.ANGLE_MIN_TORQUE, angle_torque_cap)

    if not CS.out.steeringPressed and self.recovering_from_override and self.lkas_max_torque >= self.angle_max_torque:
      self.recovering_from_override = False
      self.full_recovery_frames = 1
    elif not CS.out.steeringPressed and self.full_recovery_frames > 0:
      self.full_recovery_frames += 1
      if self.full_recovery_frames >= int(5.0 / DT_CTRL):
        self.full_recovery_frames = 0
        self.repeated_override_count = 0

    if not CC.latActive:
      apply_torque = 0
      self.lkas_max_torque = 0
      self.recovering_from_override = False
      self.full_recovery_frames = 0
      self.repeated_override_count = 0
      self.override_latched = False
      self.override_release_frames = 0
      self.driver_torque_filtered = 0.0
      self.driver_torque_filtered_prev = 0.0
      self.pre_override_frames = 0

    self.steering_pressed_prev = CS.out.steeringPressed if CC.latActive else False

    # Keep legacy history independent, but let an active handover select total
    # authority. A legacy max() would bypass its error-dependent recovery rate.
    steering_authority = self.lkas_max_torque
    if angle_control:
      model = CS.modelV2
      if model is not None and model.frameId != self.handover_model_frame:
        self.handover_model_frame = model.frameId
        self.handover_model_time = now_nanos
      model_valid = (model is not None and 0 <= now_nanos - self.handover_model_time <= 150_000_000 and
                     len(model.position.yStd) > 10 and np.isfinite(model.position.yStd[10]) and
                     0 <= model.position.yStd[10] <= 0.3)
      previous_handover_state = self.steer_handover.state
      steering_authority = self.steer_handover.update(
        # Combined convergence/release recovery is standard for angle control.
        mode=3, now=now_nanos * 1e-9, baseline=self.lkas_max_torque,
        minimum=self.params.ANGLE_MIN_TORQUE, maximum=self.angle_max_torque,
        driver=driver_torque, threshold=torque_threshold, pressed=CS.out.steeringPressed,
        target_error=actuators.steeringAngleDeg - CS.out.steeringAngleDeg,
        command_error=apply_angle - CS.out.steeringAngleDeg, speed=CS.out.vEgoRaw,
        wheelbase=self.CP.wheelbase, steer_ratio=self.CP.steerRatio, active=CC.latActive,
        valid=bool(CS.out.canValid and not CS.out.steerFaultTemporary and not CS.out.steerFaultPermanent and model_valid),
      )
      if self.frame % 100 == 0 or previous_handover_state != self.steer_handover.state:
        carlog.info("SteeringHandover mode=3 state=%s effort=%.3f error=%.3f legacy=%.1f cap=%.1f",
                    self.steer_handover.state, self.steer_handover.effort or 0.0,
                    self.steer_handover.error, self.lkas_max_torque, steering_authority)

    # SteerAngleTest: turn the wheel to that angle while stopped, hands off, without engaging (test only)
    steer_test = (angle_control and not self.aprk_diag_active and self.steer_angle_test != 0 and CS.out.standstill and not CS.out.steeringPressed and
                  not park_assist_active(CS))
    if steer_test:
      target = clip_float(self.steer_angle_test, -STEER_ANGLE_TEST_MAX, STEER_ANGLE_TEST_MAX)
      apply_angle = rate_limit(target, self.apply_angle_last, -STEER_ANGLE_TEST_RATE, STEER_ANGLE_TEST_RATE)
      steering_authority = self.angle_max_torque

    self.apply_angle_last = apply_angle

    # Hold torque with induced temporary fault when cutting the actuation bit
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = clip_float(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    active_speed_decel = hud_control.activeCarrot == 3 and self.activeCarrot != 3 # 3: Speed Decel
    self.activeCarrot = hud_control.activeCarrot
    if active_speed_decel and self.speedCameraHapticEndFrame < 0: # 과속카메라 감속시작
      self.speedCameraHapticEndFrame = self.frame + (8.0 / DT_CTRL)  #8초간 켜줌.
    elif not active_speed_decel:
      self.speedCameraHapticEndFrame = -1

    if 0 <= self.speedCameraHapticEndFrame - self.frame < int(8.0 / DT_CTRL) and self.hapticFeedbackWhenSpeedCamera > 0:
      t = (self.frame - (self.speedCameraHapticEndFrame - int(8.0 / DT_CTRL))) * DT_CTRL

      for start, end in vibrate_intervals:
        if start <= t < end:
          left_lane_warning = right_lane_warning = self.hapticFeedbackWhenSpeedCamera
          break

    if self.frame >= self.speedCameraHapticEndFrame:
      self.speedCameraHapticEndFrame = -1

    if self.frame % self.blinking_frame == 0:
      self.blinking_signal = True
    elif self.frame % self.blinking_frame == self.blinking_frame / 2:
      self.blinking_signal = False



    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if (self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and
        self.CP.openpilotLongitudinalControl and self.pedal_interceptor is None):
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_HDA2.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    camera_scc = self.CP.flags & HyundaiFlags.CAMERA_SCC
    # CAN-FD platforms
    if self.CP.flags & HyundaiFlags.CANFD:
      hda2 = self.CP.flags & HyundaiFlags.CANFD_HDA2
      hda2_long = hda2 and self.CP.openpilotLongitudinalControl
      # steering control
      if camera_scc:
        can_sends.extend(hyundaicanfd.create_steering_messages_camera_scc(self.frame, self.packer, self.CP, self.CAN, CC, apply_steer_req, apply_torque, CS, apply_angle, steering_authority, angle_control,
                                                                              lat_override=steer_test))
      else:
        can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque, apply_angle, steering_authority, angle_control))

      # prevent LFA from activating on HDA2 by sending "no lane lines detected" to ADAS ECU
      if self.frame % 5 == 0 and hda2 and not camera_scc:
        can_sends.extend(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS))

      # LFA and HDA icons
      if self.frame % 5 == 0 and (not hda2 or hda2_long or camera_scc):
        can_sends.extend(hyundaicanfd.create_lfahda_cluster(
          self.packer, CS, self.CAN, CC.longActive, CC.latActive,
          suppress_camera_auto_disengage=bool(camera_scc and self.car_fingerprint in (
            CAR.GENESIS_GV70_1ST_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN,
          )),
          dm_alert=hud_control.driverMonitoringAlert if (CS.adrv_0x161 is None or
                   (camera_scc and not self.CP.openpilotLongitudinalControl)) else 0,
        ))
        if not camera_scc:
          can_sends.extend(hyundaicanfd.create_lfa_icon_non_camera_scc(self.packer, CS, self.CAN, CC))

      # blind-spot camera view (no turn signal needed); CanfdBvmReverse owns it while in R. Gate kept here: sending
      # whenever bvm_reverse is set would put byte 4 = 0 on the bus at 5 Hz outside R too.
      reverse = self.bvm_reverse in (1, 2) and CS.out.gearShifter == structs.CarState.GearShifter.reverse
      if reverse or self.bvm_view:
        can_sends.extend(self.bvm_view_msg(reverse))

      # blinkers
      if self.blinker_stalk in (1, 2, 3, 4) or self.stalk_counter is not None:
        can_sends.extend(self.stalk_blinkers(CS))
      can_sends.extend(self.highbeam_push(CS))
      if hda2 and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, self.frame, CC.leftBlinker, CC.rightBlinker,
                                                           getattr(CS, "spas2", None)))
      # carrot-wip's own SPAS path (CanfdBlinkerControl -> CANFD_SPAS_BLINKERS, card.py) owns APRK/SPAS when it is on
      elif not self.CP.flags & HyundaiFlags.CANFD_SPAS_BLINKERS and (self.blinker_control or self.blinker_test or self.aprk_state or
                                                                    self.spas_steer_test or self.remote_blinker or self.remote_pulse or
                                                                    self.hazard_test):
        can_sends.extend(self.aprk_blinkers(CC, CS))

      if self.camera_scc_params in [2, 3]:
        self.canfd_toggle_adas(CC, CS)
      if self.CP.openpilotLongitudinalControl:
        hud_lateral = self.display_lead_lateral.update(getattr(CS, "radarState", None), getattr(CS, "modelV2", None))
        self.hyundai_jerk.make_jerk(self.CP, CS, accel, actuators, hud_control)
        if CS.out.standstill:
          self.standstill_frame = self.frame
        if (self.depart_jerk and accel > 0.0 and CS.out.vEgo < CANFD_DEPART_SPEED
            and (self.frame - self.standstill_frame) * DT_CTRL < CANFD_DEPART_TIME):
          self.hyundai_jerk.jerk_u = max(self.hyundai_jerk.jerk_u, CANFD_DEPART_JERK_UPPER)
        self.hyundai_jerk.check_carrot_cruise(CC, CS, hud_control, stopping, accel, actuators.aTarget)

        if True: #not camera_scc:
          can_sends.extend(hyundaicanfd.create_ccnc_messages(
            self.CP, self.packer, self.CAN, self.frame, CC, CS, hud_control, apply_angle,
            left_lane_warning, right_lane_warning, self.enable_corner_radar, stopping,
            self.canfd_debug, self.paddle_mode, hud_lateral=hud_lateral,
          ))
          if hda2:
            can_sends.extend(hyundaicanfd.create_adrv_messages(self.CP, self.packer, self.CAN, self.frame))
          else:
            can_sends.extend(hyundaicanfd.create_fca_warning_light(self.CP, self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          if self.CP.flags & HyundaiFlags.CAMERA_SCC.value:
            if CC.enabled:
              CS.stock_adopt_pending = False
            msg = None
            # StockCruiseAdopt: send nothing, so panda keeps forwarding the camera's cruising SCC until openpilot engages
            if not getattr(CS, "stock_adopt_pending", False):
              msg, self.accel_value_last = hyundaicanfd.create_acc_control_scc2(
                self.packer, self.CAN, CC.enabled, self.accel_value_last, accel, stopping, CC.cruiseControl.override,
                set_speed_in_units, hud_control, self.hyundai_jerk, CS, self.canfd_stopping, hud_lateral=hud_lateral,
              )
            if msg is not None:
              can_sends.append(msg)
            can_sends.extend(hyundaicanfd.create_tcs_messages(self.packer, self.CAN, CS)) # for sorento SCC radar...
          else:
            msg, self.accel_value_last = hyundaicanfd.create_acc_control(
              self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping,
              CC.cruiseControl.override, set_speed_in_units, hud_control,
              self.hyundai_jerk.jerk_u, self.hyundai_jerk.jerk_l, CS, self.canfd_stopping,
              accel_value_last=self.accel_value_last,
            )
            can_sends.append(msg)
            self.accel_last = accel
      else:
        # button presses
        if self.camera_scc_params == 3: # camera scc but stock long
          send_button = self.make_spam_button(CC, CS)
          can_sends.extend(hyundaicanfd.forward_button_message(self.packer, self.CAN, self.frame, CS, send_button, self.MainMode_ACC_trigger, self.LFA_trigger))
        else:
          can_sends.extend(self.create_button_messages(CC, CS, use_clu11=False))
    else:
      if self.cgw_kick:
        wheels = CS.out.wheelSpeeds
        parked = (CS.out.gearShifter == structs.CarState.GearShifter.park
                  and all(speed == 0 for speed in (wheels.fl, wheels.fr, wheels.rl, wheels.rr)))
        can_sends.extend(self.palisade_cgw_test.update(
          self.palisade_blinker_test, now_nanos, CS.cgw_test_raw, CS.cgw_test_timestamps,
          parked, not (CC.enabled or CC.latActive or CC.longActive),
        ))
        can_sends.extend(self.cgw_highbeam(CS))
        if self.blinker_control or self.remote_blinker or self.remote_pulse:
          can_sends.extend(self.cgw_blinkers(CS))
      if CS.lkas11 is not None:
        if self.lkas11_active:
          can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                                    torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                                    hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                    left_lane_warning, right_lane_warning, self.is_ldws_car,
                                                    dm_alert=hud_control.driverMonitoringAlert))
        self.lkas11_active = True

      if not self.CP.openpilotLongitudinalControl:
        can_sends.extend(self.create_button_messages(CC, CS, use_clu11=True))
      if self.CP.carFingerprint in CAN_GEARS["send_mdps12"] and CS.mdps12 is not None:  # send mdps12 to LKAS to prevent LKAS error
        can_sends.append(hyundaican.create_mdps12(self.packer, self.frame, CS.mdps12))

      casper_ev = self.CP.carFingerprint == CAR.HYUNDAI_CASPER_EV
      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl and self.pedal_interceptor is None:
        self.hyundai_jerk.make_jerk(self.CP, CS, accel, actuators, hud_control)
        self.hyundai_jerk.check_carrot_cruise(CC, CS, hud_control, stopping, accel, actuators.aTarget)
        #jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
        use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
        if camera_scc:

          can_sends.extend(hyundaican.create_acc_commands_scc(self.packer, CC.enabled, accel, self.hyundai_jerk, int(self.frame / 2),
                                                          hud_control, set_speed_in_units, stopping,
                                                          CC.cruiseControl.override, casper_ev, CS, self.soft_hold_mode))
        else:
          can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, self.hyundai_jerk, int(self.frame / 2),
                                                hud_control, set_speed_in_units, stopping,
                                                CC.cruiseControl.override, use_fca, self.CP, CS, self.soft_hold_mode))


      # 20 Hz LFA MFA message
      if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
        can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC, self.blinking_signal))

      # 5 Hz ACC options
      if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl and self.pedal_interceptor is None:
        if camera_scc:
          if CS.scc13 is not None:
            if casper_ev:
              #can_sends.append(hyundaican.create_acc_opt_copy(CS, self.packer))
              pass
          pass
        else:
          can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))

      # 2 Hz front radar options
      if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not camera_scc and self.pedal_interceptor is None:
        can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends = hyundaicanfd.apply_cluster_test_control_msgs(self.packer, can_sends, CS)
    if self.pedal_interceptor is not None:
      pedal_active = (CC.longActive and not CS.pedal_interceptor_cruise_blocked and
                      not CS.pedal_interceptor_state.faulted and
                      not CC.cruiseControl.override and not CS.out.gasPressed and not CS.out.brakePressed)
      pedal_message = self.pedal_interceptor.update(self.frame, pedal_active, accel, CS.out.vEgo, hud_control.setSpeed)
      if pedal_message is not None:
        can_sends.append(pedal_message)

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    # torqueOutputCan reflects the steering authority value actually sent over CAN.
    # Torque-control platforms send the signed torque command, while angle-control
    # platforms send LKAS_ANGLE_MAX_TORQUE alongside the requested angle.
    new_actuators.torqueOutputCan = steering_authority if angle_control else apply_torque
    new_actuators.steeringAngleDeg = float(apply_angle)
    new_actuators.accel = accel
    new_actuators.stopObjectActive = bool(self.canfd_stopping is not None and self.canfd_stopping.stop_object_active)

    self.frame += 1
    return new_actuators, can_sends


  def create_button_messages(self, CC: structs.CarControl, CS: CarState, use_clu11: bool):
    can_sends = []
    if CS.out.brakePressed or CS.out.brakeHoldActive or CS.out.parkingBrake:
      return can_sends
    if use_clu11:
      if CS.clu11 is None:
        return can_sends

      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif False: #CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

      if self.last_button_frame != self.frame:
        send_button = self.make_spam_button(CC, CS)
        if send_button > 0:
          can_sends.append(hyundaican.create_clu11_button(self.packer, self.frame, CS.clu11, send_button, self.CP))

    else:

      # carrot.. 왜 alt_cruise_button는 값이 리스트일까?, 그리고 왜? 빈데이터가 들어오는것일까?
      if CS.cruise_buttons_msg is not None and self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
        try:
          cruise_buttons_msg_values = {key: value[0] for key, value in CS.cruise_buttons_msg.items()}
        except: # IndexError:
          #print("IndexError....")
          cruise_buttons_msg_values = None
          self.cruise_buttons_msg_cnt += 1
        if cruise_buttons_msg_values is not None:
          self.cruise_buttons_msg_values = cruise_buttons_msg_values
          self.cruise_buttons_msg_cnt = 0

      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
            print("cruiseControl.cancel222222")
            if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
              #can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.scc_control))
              if self.cruise_buttons_msg_values is not None:
                can_sends.append(hyundaicanfd.alt_cruise_buttons(self.packer, self.CP, self.CAN, Buttons.CANCEL, self.cruise_buttons_msg_values, self.cruise_buttons_msg_cnt))

            else:
              for _ in range(20):
                can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif False: #CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

      ## button 스패밍을 안했을때...
      if self.last_button_frame != self.frame:
        dat = self.canfd_speed_control_pcm(CC, CS, self.cruise_buttons_msg_values)
        if dat is not None:
          for _ in range(self.button_spam3):
            can_sends.append(dat)
          self.cruise_buttons_msg_cnt += 1

    return can_sends

  def canfd_toggle_adas(self, CC, CS):
    trigger_min = -200
    trigger_start = 6
    self.MainMode_ACC_trigger = max(trigger_min, self.MainMode_ACC_trigger - 1)
    self.LFA_trigger = max(trigger_min, self.LFA_trigger - 1)
    if CS.out.brakeHoldActive or CS.out.parkingBrake:
      self.MainMode_ACC_trigger = trigger_min
      return
    if self.MainMode_ACC_trigger == trigger_min and self.LFA_trigger == trigger_min:
      if CC.enabled and not CS.MainMode_ACC and CS.out.vEgo > 3.:
        self.MainMode_ACC_trigger = trigger_start
      elif CC.latActive and CS.LFA_ICON == 0:
        self.LFA_trigger = trigger_start

  def canfd_speed_control_pcm(self, CC, CS, cruise_buttons_msg_values):

    alt_buttons = True if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS else False

    if alt_buttons and cruise_buttons_msg_values is None:
      return None

    send_button = self.make_spam_button(CC, CS)
    if send_button > 0:
      if alt_buttons:
        return hyundaicanfd.alt_cruise_buttons(self.packer, self.CP, self.CAN, send_button, cruise_buttons_msg_values, self.cruise_buttons_msg_cnt)
      else:
        return hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter+1, send_button)

    return None


  def make_spam_button(self, CC, CS):
    if CS.out.brakePressed or CS.out.brakeHoldActive or CS.out.parkingBrake:
      self.activateCruise = 0
      return 0

    hud_control = CC.hudControl
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)
    target = int(set_speed_in_units+0.5)
    current = int(CS.out.cruiseState.speed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH) + 0.5)
    v_ego_kph = CS.out.vEgo * CV.MS_TO_KPH

    send_button = 0
    activate_cruise = False

    if CC.enabled:
      if not CS.out.cruiseState.enabled:
        if (hud_control.leadVisible or v_ego_kph > 10.0) and self.activateCruise == 0:
          send_button = Buttons.RES_ACCEL
          self.activateCruise = 1
          activate_cruise = True
      elif CC.cruiseControl.resume:
        send_button = Buttons.RES_ACCEL
      elif target < current and current>= 31 and self.speed_from_pcm != 1:
        send_button = Buttons.SET_DECEL
      elif target > current and current < 160 and self.speed_from_pcm != 1:
        send_button = Buttons.RES_ACCEL
    elif CS.out.activateCruise > 0: # Negative values request cancellation, never RES.
      if (hud_control.leadVisible or v_ego_kph > 10.0) and self.activateCruise == 0:
        self.activateCruise = 1
        send_button = Buttons.RES_ACCEL
        activate_cruise = True

    if CS.out.brakePressed or CS.out.gasPressed:
      self.activateCruise = 0

    if send_button == 0:
      self.button_spamming_count = 0
      self.prev_clu_speed = current
      return 0

    speed_diff = self.prev_clu_speed - current
    spamming_max = self.button_spam1
    if CS.cruise_buttons[-1] != Buttons.NONE:
      self.last_button_frame = self.frame
      self.button_wait = self.button_spam2
      self.button_spamming_count = 0
    elif abs(self.button_spamming_count) >= spamming_max or abs(speed_diff) > 0:
      self.last_button_frame = self.frame
      self.button_wait = self.button_spam2 if abs(self.button_spamming_count) >= spamming_max else 7
      self.button_spamming_count = 0

    self.prev_clu_speed = current
    send_button_allowed = (self.frame - self.last_button_frame) > self.button_wait
    #CC.debugTextCC = "{} speed_diff={:.1f},{:.0f}/{:.0f}, button={}, button_wait={}, count={}".format(
    #  send_button_allowed, speed_diff, target, current, send_button, self.button_wait, self.button_spamming_count)

    if send_button_allowed or activate_cruise or (CC.cruiseControl.resume and self.frame % 2 == 0):
      self.button_spamming_count = self.button_spamming_count + 1 if send_button == Buttons.RES_ACCEL else self.button_spamming_count - 1
      return send_button
    else:
      self.button_spamming_count = 0
    return 0

class HyundaiJerk:
  def __init__(self):
    self.params = Params()
    self.jerk = 0.0
    self.jerk_u = self.jerk_l = 0.0
    self.cb_upper = self.cb_lower = 0.0
    self.jerk_u_min = 0.5
    self.carrot_cruise = 1
    self.carrot_cruise_accel = 0.0
    self.accel_request_history = deque(maxlen=max(1, round(CANFD_JERK_ERROR_DELAY / DT_CTRL)))
    self.jerk_error_filter = MyMovingAverage(max(1, round(CANFD_JERK_ERROR_FILTER_TIME / DT_CTRL)), 0.0)

  def check_carrot_cruise(self, CC, CS, hud_control, stopping, accel, a_target):
    carrot_cruise_decel = self.params.get_float("CarrotCruiseDecel")
    carrot_cruise_atc_decel = self.params.get_float("CarrotCruiseAtcDecel")
    if carrot_cruise_atc_decel >= 0 and 0 < hud_control.atcDistance < 500:
      carrot_cruise_decel = max(carrot_cruise_decel, carrot_cruise_atc_decel)
    self.carrot_cruise = 0
    if CS.out.carrotCruise > 0 and not CC.cruiseControl.override:
      if CS.softHoldActive == 0 and not stopping:
        if CS.out.vEgo > 10/3.6:
          if carrot_cruise_decel < 0:
            if (a_target > -0.1 or accel > -0.1):
              self.carrot_cruise = 1
              self.carrot_cruise_accel = 0.0
          else:
            self.carrot_cruise = 2
            carrot_cruise = min(accel, -carrot_cruise_decel * 0.01)
            self.carrot_cruise_accel = max(carrot_cruise, self.carrot_cruise_accel - 1.0 * DT_CTRL) #  점진적으로 줄임.
    if self.carrot_cruise == 0:
      self.carrot_cruise_accel = CS.out.aEgo

  def make_jerk(self, CP, CS, accel, actuators, hud_control):
    canfd = bool(CP.flags & HyundaiFlags.CANFD)
    jerk_u_min = CANFD_JERK_UPPER_MIN if canfd else self.jerk_u_min
    if actuators.longControlState == LongCtrlState.stopping:
      self.jerk = jerk_u_min / 2 - CS.out.aEgo
    else:
      jerk = actuators.jerk if actuators.longControlState == LongCtrlState.pid else 0.0
      #a_error = actuators.aTarget - CS.out.aEgo
      self.jerk = jerk# + a_error

    jerk_max_l = CANFD_JERK_LIMIT_MAX
    jerk_max_u = jerk_max_l
    if actuators.longControlState == LongCtrlState.off:
      self.jerk_u = jerk_max_u
      self.jerk_l = jerk_max_l
      self.cb_upper = self.cb_lower = 0.0
      self.accel_request_history.clear()
      self.jerk_error_filter.set_all(0.0)
    else:
      if canfd:
        tracking_error = 0.0
        tracking_error_active = actuators.longControlState == LongCtrlState.pid and not CS.out.brakePressed and not CS.out.gasPressed
        if tracking_error_active:
          self.accel_request_history.append(float(accel))
          request_is_sustained = False
          if len(self.accel_request_history) == self.accel_request_history.maxlen:
            delayed_accel = self.accel_request_history[0]
            request_is_sustained = delayed_accel < -1.0 and accel < -1.0 and abs(accel - delayed_accel) < 0.5
            if request_is_sustained:
              # Only assist when measured deceleration is weaker than both the
              # current request and the delay-aligned request. The current-request
              # comparison makes the assist release promptly while aReq unwinds.
              tracking_error = min(CS.out.aEgo - delayed_accel, CS.out.aEgo - accel)
          # Require sustained measured under-deceleration before slowly enabling
          # feedforward. Release it immediately once the plan unwinds or the vehicle
          # meets/exceeds either request, avoiding a filtered braking tail.
          if request_is_sustained and self.jerk <= CANFD_JERK_RELEASE_THRESHOLD and tracking_error > 0.0:
            filtered_tracking_error = self.jerk_error_filter.process(tracking_error)
          else:
            filtered_tracking_error = self.jerk_error_filter.set_all(0.0)
        else:
          self.accel_request_history.clear()
          filtered_tracking_error = self.jerk_error_filter.set_all(0.0)

        self.jerk_u, self.jerk_l = calculate_canfd_jerk_limits(accel, self.jerk, filtered_tracking_error)
        self.cb_upper = self.cb_lower = 0.0
      else:
        self.jerk_u = min(max(self.jerk_u_min, self.jerk * 2.0), jerk_max_u)
        self.jerk_l = min(max(1.0, -self.jerk * 4.0), jerk_max_l)
        self.cb_upper = np.clip(0.9 + accel * 0.2, 0, 1.2)
        self.cb_lower = np.clip(0.8 + accel * 0.2, 0, 1.2)

