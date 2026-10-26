import math
import time

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params
from openpilot.selfdrive.carrot.bluetooth.model import CommandReader

from openpilot.selfdrive.controls.lib.desire_lib.constants import (
  LaneChangeState, LaneChangeDirection, TurnDirection,
  LANE_CHANGE_SPEED_MIN, LANE_CHANGE_TIME_MAX,
  BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT,
  DESIRES, TURN_DESIRES
)
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState
from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type
from openpilot.selfdrive.controls.lib.desire_lib.lever import LEVER_REQUEST_WAIT_S, LaneCrossing, LeverForcedTurn, LeverRequestWait, lever_maneuver

# Without a model-detected crossing, a lever press this early in laneChangeFinishing is still the driver
# re-lighting the lamp of the change that just crossed, not a request for the next one.
STALK_QUEUE_GUARD_S = 0.3
# An accepted App action may wait for interlocks, but must not start unexpectedly much later.
AUTO_APP_WAIT_S = 10.0


class DesireHelper:
  def __init__(self, tesla_auto_signals=False):
    self.params = Params()
    self.bluetooth_commands = CommandReader('lane')
    self.frame = 0

    # FSM core
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_delay = 0.0
    self.maneuver_type = "none"  # "none" / "turn" / "lane_change"

    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.desire_disable_count = 0
    self.turn_disable_count = 0

    # per-side states
    self.left = SideState("left")
    self.right = SideState("right")

    # blinker/ATC state (원본 변수들 유지)
    self.blinker_ignore = False
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.atc_type = ""
    self.atc_active = 0  # 0: 없음, 1: ATC 동작, 2: 충돌

    # auto lane change
    self.auto_lane_change_enable = False
    self.next_lane_change = False

    # turn-signal lever presses (carState.*BlinkerStalkCount): an explicit request for the next lane change
    self.stalk_counts = None
    self.queued_lane_change = BLINKER_NONE
    self.finishing_timer = 0.0
    self.queued_wait = 0.0
    self.lever_wait = LeverRequestWait()
    self.lever_forced_turn = LeverForcedTurn()
    self.crossing = LaneCrossing()
    self.auto_turn_signal = LaneChangeDirection.none
    self.auto_turn_signal_lane_change = False
    self.blinker_hold_direction = LaneChangeDirection.none  # modelV2.meta.laneChangeBlinkerHold
    self.lever_turn = False  # modelV2.meta.leverTurn: the turn desire comes from a latched lever
    self.tesla_auto_signals = tesla_auto_signals
    self.auto_signal_direction = LaneChangeDirection.none
    self.auto_signal_request_id = 0
    self._auto_signal_counter = time.monotonic_ns()
    self._auto_lane_completed = False
    self._auto_lane_direction = LaneChangeDirection.none
    self._auto_app_direction = BLINKER_NONE
    self._auto_app_wait = 0.0
    self._auto_app_deadline = 0.0
    self._auto_turn_consumed = False
    self._auto_atc_type = ""
    self._driver_signal_counts = None
    self._manual_signal_direction = BLINKER_NONE
    self._manual_signal_grace = 0.0
    self._auto_turn_direction = LaneChangeDirection.none
    self._auto_turn_started = 0.0
    self._auto_turn_peak_angle = 0.0
    self._auto_turn_settled = 0.0

    # keep pulse
    self.keep_pulse_timer = 0.0

    # params
    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneLineCheck = 0
    self.laneChangeDelay = 0.0
    self.blinkerLatchedTurn = 0.0  # m/s, 0 = off
    self.laneChangeLeverWait = False

    # misc
    self.prev_desire_enabled = False
    self.desireLog = ""

    # externally readable flags
    self.lane_change_available_left = False
    self.lane_change_available_right = False

  # ─────────────────────────────────────────────
  # params/model
  # ─────────────────────────────────────────────
  def _update_params_periodic(self):
    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd = self.params.get_int("LaneChangeBsd")
      self.laneLineCheck = self.params.get_int("LaneLineCheck")
      self.laneChangeDelay = self.params.get_float("LaneChangeDelay") * 0.1
      self.blinkerLatchedTurn = self.params.get_int("BlinkerLatchedTurn") * CV.KPH_TO_MS
      self.laneChangeLeverWait = self.params.get_bool("LaneChangeLeverWait")

  def _check_desire_state(self, modeldata, carstate, maneuver_type):
    desire_state = modeldata.meta.desireState
    orientation_rate = abs(modeldata.orientationRate.z[5])
    orientation_rate_future = abs(modeldata.orientationRate.z[15])

    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1

    if maneuver_type == "turn" and abs(carstate.steeringAngleDeg) > 80 and orientation_rate_future < orientation_rate:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  # ─────────────────────────────────────────────
  # blinkers/ATC (원본 로직 유지, side 계산은 별개)
  # ─────────────────────────────────────────────
  def _update_driver_blinker(self, carstate):
    st = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    if self.tesla_auto_signals:
      driver = int(getattr(carstate, "turnSignalDriverDirection", 0))
      driver_valid = bool(getattr(carstate, "turnSignalDriverValid", False))
      counts = (getattr(carstate, "leftBlinkerStalkCount", 0), getattr(carstate, "rightBlinkerStalkCount", 0))
      last, self._driver_signal_counts = self._driver_signal_counts, counts
      left_press, right_press = (counts[0] != last[0], counts[1] != last[1]) if last else (False, False)
      press = (BLINKER_LEFT if left_press else BLINKER_RIGHT) if left_press != right_press else BLINKER_NONE
      if st == 3:
        self._manual_signal_direction = BLINKER_NONE
        self._manual_signal_grace = 0.0
      elif not driver_valid:
        self._manual_signal_direction = BLINKER_NONE
        self._manual_signal_grace = 0.0
        st = BLINKER_NONE
      elif driver in (1, 2) or press:
        self._manual_signal_direction = driver or press
        self._manual_signal_grace = 1.0
        st = self._manual_signal_direction
      elif st == self._manual_signal_direction and st:
        self._manual_signal_grace = 0.5
      elif st != 3 and self._manual_signal_grace > DT_MDL:
        self._manual_signal_grace -= DT_MDL
        st = BLINKER_NONE
      else:
        self._manual_signal_direction = BLINKER_NONE
        self._manual_signal_grace = 0.0
        st = BLINKER_NONE
    changed = st != self.driver_blinker_state
    self.driver_blinker_state = st

    enabled = st in (BLINKER_LEFT, BLINKER_RIGHT)
    if self.laneChangeNeedTorque < 0:
      enabled = False
    return st, changed, enabled

  def _update_stalk(self, carstate) -> int:
    """Side of a turn-signal lever press since the last update (BLINKER_LEFT/RIGHT), else BLINKER_NONE.

    The lamp-based blinker cannot show a press while the lamp is already flashing (one press flashes
    for ~4 s); the lever counters can, and a counter never misses a press between 20 Hz updates.
    """
    counts = (getattr(carstate, "leftBlinkerStalkCount", None), getattr(carstate, "rightBlinkerStalkCount", None))
    if counts[0] is None or counts[1] is None:
      return BLINKER_NONE
    last, self.stalk_counts = self.stalk_counts, counts
    if last is None:
      return BLINKER_NONE
    if self.tesla_auto_signals:
      left_press, right_press = counts[0] != last[0], counts[1] != last[1]
      if (carstate.leftBlinker and carstate.rightBlinker) or \
         not getattr(carstate, "turnSignalDriverValid", False) or left_press == right_press:
        return BLINKER_NONE
      direction = BLINKER_LEFT if left_press else BLINKER_RIGHT
      driver = getattr(carstate, "turnSignalDriverDirection", 0)
      return direction if driver in (0, direction) else BLINKER_NONE
    if counts[0] != last[0]:
      return BLINKER_LEFT
    if counts[1] != last[1]:
      return BLINKER_RIGHT
    return BLINKER_NONE

  def _update_atc_blinker(self, carrotMan, driver_blinker_state, remote=None):
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    # 유지 카운트는 DesireHelper에서 관리
    if self.tesla_auto_signals and self._auto_app_direction:
      atc_blinker_state = self._auto_app_direction
    elif self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif not self.tesla_auto_signals and remote in ('laneLeft', 'laneRight'):
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if remote == 'laneLeft' else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif not self.tesla_auto_signals and carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif atc_type in ("turn left", "turn right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active = 1
        self.blinker_ignore = False
    elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ("fork left", "atc left") else BLINKER_RIGHT
        self.atc_active = 1
    else:
      self.atc_active = 0

    # 충돌 시 ATC 무효
    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active = 2

    atc_desire_enabled = atc_blinker_state in (BLINKER_LEFT, BLINKER_RIGHT)
    if self.tesla_auto_signals and self._auto_app_direction and self.lane_change_state not in (
      LaneChangeState.off, LaneChangeState.preLaneChange
    ):
      # The FSM now owns execution. Keep the side for torque cancellation, not a second request.
      atc_desire_enabled = False

    # blinker_ignore
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # 타입 변경 1프레임 무시
    if self.atc_type != atc_type and not (self.tesla_auto_signals and self._auto_app_direction):
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

  def _update_auto_app_request(self, carstate, lateral_active, carrot_man, remote, driver_st, stalk_press):
    new_command = carrot_man.carrotCmdIndex != self.carrot_cmd_index_last and carrot_man.carrotCmd == "LANECHANGE"
    if new_command:
      # Consume even rejected commands: recovery must never execute an old App action.
      self.carrot_cmd_index_last = carrot_man.carrotCmdIndex
    requested = (BLINKER_LEFT if remote == "laneLeft" else BLINKER_RIGHT if remote == "laneRight" else
                 BLINKER_LEFT if new_command and carrot_man.carrotArg == "LEFT" else
                 BLINKER_RIGHT if new_command and carrot_man.carrotArg == "RIGHT" else BLINKER_NONE)
    waiting = self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange)
    now = time.clock_gettime(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))
    permitted = (lateral_active and carstate.canValid and not carstate.trailerConnected and
                 not getattr(carstate, "vehicleSensorsInvalid", False) and
                 str(getattr(carstate, "gearShifter", "drive")) == "drive" and
                 carstate.turnSignalDriverValid and not driver_st and not stalk_press and
                 not self._manual_signal_direction and not (carstate.leftBlinker and carstate.rightBlinker) and
                 (not waiting or carstate.vEgo >= LANE_CHANGE_SPEED_MIN) and self.desire_disable_count == 0)
    self._auto_app_wait += DT_MDL if self._auto_app_direction and waiting else 0.0
    expired = waiting and self._auto_app_direction and (
      self._auto_app_wait >= AUTO_APP_WAIT_S or now >= self._auto_app_deadline)
    if not permitted or expired:
      if self._auto_app_direction and not driver_st:
        self.lane_change_state = LaneChangeState.off
        self.lane_change_direction = LaneChangeDirection.none
      self._auto_app_direction = BLINKER_NONE
    if (requested and permitted and not self._auto_app_direction and
        self.lane_change_state == LaneChangeState.off and not self.auto_signal_request_id):
      self._auto_app_direction, self._auto_app_wait = requested, 0.0
      self._auto_app_deadline = now + AUTO_APP_WAIT_S

  # ─────────────────────────────────────────────
  # per-side processing (핵심: 좌/우 모두 매 프레임 계산)
  # ─────────────────────────────────────────────
  def _process_sides(self, carstate, modeldata, radarState):
    # geometry (좌/우)
    # left: outer laneLines[0], current laneLines[1], edge[0], cur_prob laneLineProbs[1]
    self.left.update_lane_geometry(
      modeldata.laneLines[0], modeldata.laneLineProbs[0],
      modeldata.laneLines[1],
      modeldata.roadEdges[0],
      cur_prob=modeldata.laneLineProbs[1],
    )
    # right: outer laneLines[3], current laneLines[2], edge[1], cur_prob laneLineProbs[2]
    self.right.update_lane_geometry(
      modeldata.laneLines[3], modeldata.laneLineProbs[3],
      modeldata.laneLines[2],
      modeldata.roadEdges[1],
      cur_prob=modeldata.laneLineProbs[2],
    )

    # lane line info (HUD용 raw는 기존대로 leftLaneLine/rightLaneLine)
    self.left.update_lane_line_info(carstate.leftLaneLine)
    self.right.update_lane_line_info(carstate.rightLaneLine)

    # BSD 설정
    ignore_bsd = (self.laneChangeBsd < 0)

    # obstacles
    v_ego = carstate.vEgo
    self.left.update_obstacles(v_ego, radarState.leadLeft, carstate.leftBlindspot, ignore_bsd,
                               bsd_hold_sec=2.0, radar_objects=radarState.leadsLeft)
    self.right.update_obstacles(v_ego, radarState.leadRight, carstate.rightBlindspot, ignore_bsd,
                                bsd_hold_sec=2.0, radar_objects=radarState.leadsRight)

    # compute available (include BSD+object)
    if self.laneLineCheck >= 1:
      left_line_ok = self.left.lane_line_info_mod in (0, 5)
      right_line_ok = self.right.lane_line_info_mod in (0, 5)
    else:
      # Hyundai lane color: 0 none, 1 white, 2 yellow, 3 blue. Only yellow blocks a lane change.
      left_line_ok = self.left.lane_line_info_raw // 10 != 2
      right_line_ok = self.right.lane_line_info_raw // 10 != 2
    self.left.compute_lane_change_available(lane_line_info_lt_20=left_line_ok, ignore_bsd=ignore_bsd)
    self.right.compute_lane_change_available(lane_line_info_lt_20=right_line_ok, ignore_bsd=ignore_bsd)

    self.left.update_triggers()
    self.right.update_triggers()

    # externally readable
    self.lane_change_available_left = self.left.lane_change_available
    self.lane_change_available_right = self.right.lane_change_available

  def _get_selected_side(self, blinker_state: int) -> SideState:
    return self.left if blinker_state == BLINKER_LEFT else self.right

  @staticmethod
  def _is_last_lane(side: SideState) -> bool:
    return side.lane_exist_count.counter <= 0 and not side.lane_change_available_geom

  # ─────────────────────────────────────────────
  # main update
  # ─────────────────────────────────────────────
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):
    self.frame += 1
    self._update_params_periodic()

    # counts
    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay = max(0.0, self.lane_change_delay - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN
    trailer_maneuver_blocked = carstate.trailerConnected

    prev_lane_change_state = self.lane_change_state
    lever_decided_turn = False

    # per-side compute (좌/우 모두)
    self._process_sides(carstate, modeldata, radarState)
    if trailer_maneuver_blocked:
      self.left.lane_change_available = False
      self.right.lane_change_available = False
      self.lane_change_available_left = False
      self.lane_change_available_right = False
      self.auto_lane_change_enable = False
      self.next_lane_change = False
      self.desireLog = "TRAILER:MANEUVER_BLOCKED"

    # desire state from model
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    # blinkers
    driver_st, driver_changed, driver_enabled = self._update_driver_blinker(carstate)
    stalk_press = self._update_stalk(carstate)
    lever = getattr(carstate, "blinkerLever", 0)
    forced_turn = self.lever_forced_turn.update(lever, getattr(carstate, "blinkerLamp", 0))
    if self.lever_wait.update(self.laneChangeLeverWait, stalk_press, lever, driver_st, self.lane_change_state,
                              self.laneChangeDelay):
      driver_enabled = driver_changed = False  # one-touch request timed out: ignore the lamp until it goes dark
    remote = self.bluetooth_commands.read(allowed=(lateral_active and carstate.canValid and
      not below_lane_change_speed and not trailer_maneuver_blocked))
    if self.tesla_auto_signals:
      self._update_auto_app_request(carstate, lateral_active, carrotMan, remote, driver_st, stalk_press)
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st, remote)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    # 선택된 side (FSM은 이 side만 참고)
    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None
    atc_lane_change_requested = (
      atc_enabled and not self._auto_app_direction and
      self.atc_type in ("fork left", "fork right", "atc left", "atc right")
    )
    atc_lane_change_manual_only = (
      atc_lane_change_requested and
      not driver_enabled and
      self.atc_type in ("fork left", "atc left")
    )
    atc_lane_change_only = atc_lane_change_requested and not driver_enabled
    # Do not treat a blocked->available retry as permission to cross solid/unknown lines.
    # Geometry-based ATC at the last lane still works when the lane/road edge opens up.
    atc_lane_change_retry_line_blocked = (
      atc_lane_change_only and
      side is not None and
      side.lane_line_info_mod not in (0, 5)
    )

    # auto lane change trigger (기존 로직 유지하되 side 기반)
    auto_lane_change_trigger = False
    if desire_enabled and side is not None and not trailer_maneuver_blocked:
      # carrot_lane_change_count>0이면 강제 허용
      if self.carrot_lane_change_count > 0 or (self.tesla_auto_signals and self._auto_app_direction):
        auto_lane_change_trigger = side.lane_change_available
      else:
        # 기존 조건: edge_available + (trigger or appeared) + not side_object_detected
        auto_lane_change_trigger = (
          self.auto_lane_change_enable and
          (not atc_lane_change_manual_only) and
          side.edge_available and
          (side.lane_available_trigger or side.lane_appeared) and
          (not side.side_object_detected) and
          (side.bsd_hold_counter == 0)
        )
      self.desireLog = (
        f"{side.name}:ALC={self.auto_lane_change_enable}, "
        #f"L={side.lane_available},E={side.edge_available}, "
        #f"T={side.lane_available_trigger},A={side.lane_appeared}, "
        #f"OBJ={side.side_object_detected},BSD={side.bsd_hold_counter>0}"
      )
    else:
      self.auto_lane_change_enable = False
      self.next_lane_change = False

    # ───────────────────────── FSM ─────────────────────────
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or trailer_maneuver_blocked:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    elif self.desire_disable_count > 0:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction = TurnDirection.none
      self.maneuver_type = "none"

    else:
      # classify maneuver type using selected side
      if desire_enabled and side is not None:
        new_type = classify_maneuver_type(
          blinker_state=blinker_state,
          carstate=carstate,
          side=side,
          turn_desire_state=self.turn_desire_state,
          atc_type=self.atc_type,
          old_type=self.maneuver_type,
        )
      else:
        new_type = "none"
      if self.tesla_auto_signals and self._auto_app_direction:
        new_type = "lane_change"

      # BlinkerLatchedTurn: the driver's lever replaces the speed/lane-line guess (see lever_maneuver).
      lever_type, lever_undecided = (None, False)
      if driver_enabled and side is not None:
        lever_type, lever_undecided = lever_maneuver(lever, v_ego, self.blinkerLatchedTurn, forced_turn)
      if lever_type is not None:
        new_type = lever_type
      lever_decided_turn = lever_type == "turn"

      if trailer_maneuver_blocked and new_type in ("lane_change", "turn"):
        new_type = "none"

      # switching rules
      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in (
        LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting
      ):
        self.maneuver_type = "turn"
        self.lane_change_state = LaneChangeState.off
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ TURN mode ─
      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0:
          self.turn_direction = TurnDirection.none
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
          self.lane_change_direction = self.turn_direction

      # ─ Lane change FSM ─
      else:
        self.turn_direction = TurnDirection.none

        if self.lane_change_state == LaneChangeState.off:
          # A lever press while the lamp is still flashing has no lamp edge but is still a fresh request.
          driver_desire_started = driver_enabled and (driver_changed or stalk_press == blinker_state)
          if desire_enabled and (not self.prev_desire_enabled or driver_desire_started) and \
             not below_lane_change_speed and side is not None:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.lane_change_delay = self.laneChangeDelay

            # 맨 끝 차선이 아니면, ATC 자동 차선변경 비활성
            # (원본 유지: 차선 존재하거나 geom 가능하면 auto off, 아니면 on)
            self.auto_lane_change_enable = self._is_last_lane(side)
            self.next_lane_change = False

        elif self.lane_change_state == LaneChangeState.preLaneChange:
          if side is None:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
          else:
            self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right
            # An explicit lever press asks for this change: no steering-torque confirmation needed.
            if self.next_lane_change and driver_enabled and stalk_press == blinker_state:
              self.next_lane_change = False

            # torque direction cond
            torque_cond = (carstate.steeringTorque > 0) if blinker_state == BLINKER_LEFT else (carstate.steeringTorque < 0)
            torque_applied = carstate.steeringPressed and torque_cond

            # BSD config
            ignore_bsd = (self.laneChangeBsd < 0)
            block_lanechange_bsd = (self.laneChangeBsd == 1)
            bsd_active = (side.bsd_hold_counter > 0) and (not ignore_bsd)
            side_clear_without_line = (side.lane_available or side.edge_available) and \
                                      (not side.side_object_detected) and (not bsd_active)
            atc_driver_confirm = atc_lane_change_requested and driver_enabled
            atc_geometry_release = atc_lane_change_only and auto_lane_change_trigger
            atc_line_release = (atc_driver_confirm or atc_geometry_release) and side_clear_without_line

            # Arm automatic ATC only after this side has actually become the last lane.
            # Keep it latched so a newly appearing lane can start the maneuver later.
            if atc_lane_change_only and self._is_last_lane(side):
              self.auto_lane_change_enable = True

            if not desire_enabled or below_lane_change_speed:
              self.lane_change_state = LaneChangeState.off
              self.lane_change_direction = LaneChangeDirection.none
            else:
              # 차선변경 시작 조건:
              # - side.lane_change_available는 BSD+object 포함(요구사항)
              # - 하지만 BSD 중에도 torque override 허용해야 하므로, BSD 분기를 별도로 둠(원본 동작 유지)
              # LaneLineCheck=2: 실선에서도 토크 override 허용
              solid_line_blocked = (self.laneLineCheck >= 2) and (not side.lane_change_available_geom) and \
                                   (side.lane_available or side.edge_available)
              block_released = side.lane_change_available_released
              # A BSD/radar release must not start pure ATC unless ATC was armed at a last lane.
              # Driver blinkers retain the existing retry behavior.
              block_released_auto = block_released and (driver_enabled or self.auto_lane_change_enable) and \
                                    not atc_lane_change_retry_line_blocked
              start_gate = (side.lane_change_available_geom and self.lane_change_delay == 0) or \
                           side.lane_line_info_edge_detect or solid_line_blocked or block_released_auto or atc_line_release
              # Only automatic Tesla requests wait for their own echoed, physically confirmed lamp session.
              signal_ready = (not self.tesla_auto_signals or driver_enabled or
                              (self.auto_signal_request_id != 0 and
                               getattr(carstate, "autoTurnSignalReadyRequestId", 0) == self.auto_signal_request_id))
              if start_gate and not lever_undecided and signal_ready:
                if solid_line_blocked:
                  if atc_line_release or (torque_applied and not (bsd_active and block_lanechange_bsd)):
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                elif bsd_active:
                  if torque_applied and (not block_lanechange_bsd):
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                elif self.laneChangeNeedTorque > 0 or self.next_lane_change:
                  if torque_applied:
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                elif driver_enabled:
                  # driver blinker면 바로 시작(원본 유지)
                  # 단, object/bzd 막힘은 side.lane_change_available에서 걸림
                  if side.lane_change_available or atc_line_release:
                    self.lane_change_state = LaneChangeState.laneChangeStarting
                else:
                  if torque_applied or ((not atc_lane_change_manual_only) and (
                    auto_lane_change_trigger or side.lane_line_info_edge_detect or block_released_auto
                  )):
                    # 여기서는 시작 직전 안전성 체크
                    if side.lane_change_available or atc_line_release:
                      self.lane_change_state = LaneChangeState.laneChangeStarting

        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          # A press before the car centre crossed the line re-lights this change's lamp and is ignored (never a second
          # lane at once); one after it asks for the next change. The model keeps the lane-change probability up
          # ~1.5 s past the crossing, so waiting for laneChangeFinishing missed them (2026-10-05 drive: 4/4 next-lane
          # presses came 0.1-1.1 s after the crossing, all still in laneChangeStarting).
          since_crossing = self.crossing.update(modeldata, self.lane_change_direction == LaneChangeDirection.left)
          if stalk_press != BLINKER_NONE and since_crossing >= 0.0:
            self.queued_lane_change = stalk_press
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing
            self.finishing_timer = 0.0

        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.finishing_timer += DT_MDL
          crossed = self.crossing.since >= 0.0  # the crossing was seen during laneChangeStarting
          if stalk_press != BLINKER_NONE and (crossed or self.finishing_timer >= STALK_QUEUE_GUARD_S):
            self.queued_lane_change = stalk_press
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none
            if desire_enabled:
              self.lane_change_state = LaneChangeState.preLaneChange
              # Holding the blinker through the change still needs torque for the next one; a lever press
              # made during finishing (same side as the lamp) is an explicit request and does not.
              self.next_lane_change = not (driver_enabled and self.queued_lane_change == blinker_state)
              self.queued_wait = 0.0
            else:
              self.lane_change_state = LaneChangeState.off

    # timer
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    # commit last per-side
    self.left.commit_last()
    self.right.commit_last()

    self.prev_desire_enabled = desire_enabled

    # 반대 방향 토크로 cancel (기존 유지)
    steering_pressed_cancel = carstate.steeringPressed and (
      (carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
      (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT)
    )
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state = LaneChangeState.off
      self.blinker_ignore = True

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.crossing = LaneCrossing()
    # A queued request lives until the next change starts; waiting in preLaneChange (blocked) it gets the same
    # 5 flashes as a one-touch press, and it ends if the lamp side changes or everything goes off.
    if self.lane_change_state == LaneChangeState.preLaneChange and self.queued_lane_change != BLINKER_NONE:
      self.queued_wait += DT_MDL
    if self.lane_change_state == LaneChangeState.off or        (self.lane_change_state == LaneChangeState.laneChangeStarting and
        prev_lane_change_state == LaneChangeState.preLaneChange) or        (self.lane_change_state == LaneChangeState.preLaneChange and
        (self.queued_wait >= LEVER_REQUEST_WAIT_S or self.queued_lane_change != blinker_state)):
      self.queued_lane_change = BLINKER_NONE

    hold = self.lever_wait.update_hold(self.lane_change_state, blinker_state)
    if self.queued_lane_change != BLINKER_NONE:
      # The lamp of a press made during a change may stop before that change ends (2026-10-05: ~2.5 s after the
      # press), which would drop the request just as it becomes due: keep it lit until the queued change starts.
      hold = self.queued_lane_change
    self.blinker_hold_direction = {BLINKER_LEFT: LaneChangeDirection.left,
                                   BLINKER_RIGHT: LaneChangeDirection.right}.get(hold, LaneChangeDirection.none)

    # final desire
    if self.turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    self.lever_turn = lever_decided_turn and self.turn_direction != TurnDirection.none
    if self.tesla_auto_signals:
      self._update_auto_signal(carstate, modeldata, carrotMan, lateral_active, atc_enabled, atc_st, driver_st, prev_lane_change_state)
    if self.tesla_auto_signals and (self.lane_change_state == LaneChangeState.off or self._auto_lane_completed):
      self._auto_app_direction = BLINKER_NONE

    # keep pulse
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    # Origin is latched before injected lamps can look like driver input.
    # preLaneChange is deliberately excluded: indicate only actual lane changes.
    turning = self.desire in (log.Desire.turnLeft, log.Desire.turnRight)
    changing = self.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing)
    continuing = changing if self.auto_turn_signal_lane_change else turning
    if not continuing or self.lane_change_direction != self.auto_turn_signal:
      self.auto_turn_signal = LaneChangeDirection.none
    if self.auto_turn_signal == LaneChangeDirection.none and atc_enabled and not driver_enabled and self.atc_active == 1:
      if turning and self.atc_type in ("turn left", "turn right"):
        self.auto_turn_signal = self.lane_change_direction
        self.auto_turn_signal_lane_change = False
      elif self.lane_change_state == LaneChangeState.laneChangeStarting and atc_lane_change_only:
        self.auto_turn_signal = self.lane_change_direction
        self.auto_turn_signal_lane_change = True
    return self.desire

  def _update_auto_turn_geometry(self, carstate, modeldata):
    if self._auto_turn_direction == LaneChangeDirection.none:
      return
    # Cap'n Proto lists support iteration and integer indexing, but not slicing.
    rates = list(getattr(getattr(modeldata, "orientationRate", None), "z", ()))
    headings = list(getattr(getattr(modeldata, "orientation", None), "z", ()))
    signed_angle = getattr(carstate, "steeringAngleDeg", float("nan"))
    angle = abs(signed_angle)
    if (len(rates) < 16 or len(headings) < 16 or
        not all(math.isfinite(v) for v in (*rates[:16], *headings[:16], angle, carstate.vEgo))):
      if self._auto_turn_started < 0.3:
        self._auto_turn_started = 0.0
      self._auto_turn_settled = 0.0
      return
    moving = carstate.vEgo > 1.0
    turn_sign = 1 if self._auto_turn_direction == LaneChangeDirection.left else -1
    if moving and signed_angle * turn_sign > 20 and rates[5] * turn_sign > 0.1:
      self._auto_turn_started = min(0.3, self._auto_turn_started + DT_MDL)
      self._auto_turn_peak_angle = max(self._auto_turn_peak_angle, angle)
    elif self._auto_turn_started < 0.3:
      self._auto_turn_started = 0.0
    # This lamp-only completion must not inherit the steering FSM's early
    # turn-disable heuristic, nor mistake a stopped/slow corner for straight.
    settled = (self._auto_turn_started >= 0.3 and moving and
               angle <= min(20, self._auto_turn_peak_angle * 0.25) and
               max(abs(v) for v in rates[:16]) < 0.05 and abs(headings[15] - headings[5]) < 0.06)
    self._auto_turn_settled = self._auto_turn_settled + DT_MDL if settled else 0.0
    if self._auto_turn_settled >= 1.0:
      self._auto_turn_consumed = True
      self._auto_turn_direction = LaneChangeDirection.none

  def _update_auto_signal(self, carstate, modeldata, carrot_man, lateral_active, atc_enabled, atc_st, driver_st, previous):
    direction = LaneChangeDirection.none
    turn_type = not self._auto_app_direction and self.atc_type in ("turn left", "turn right", "turn left prepare", "turn right prepare")
    navigation_direction = (LaneChangeDirection.left if turn_type and self.atc_type in ("turn left", "turn left prepare")
                            else LaneChangeDirection.right if turn_type else LaneChangeDirection.none)
    turn_handoff = self.atc_type in ("turn left", "turn right") and self._auto_atc_type == self.atc_type + " prepare"
    if self.atc_type != self._auto_atc_type:
      if navigation_direction not in (LaneChangeDirection.none, self._auto_turn_direction):
        self._auto_turn_direction = LaneChangeDirection.none
        self._auto_turn_started = self._auto_turn_peak_angle = self._auto_turn_settled = 0.0
      if self._auto_turn_direction == LaneChangeDirection.none:
        self._auto_turn_consumed = False
      self._auto_lane_completed = False
    self._auto_atc_type = self.atc_type
    if previous == LaneChangeState.laneChangeFinishing and self.lane_change_state != previous:
      self._auto_lane_completed = True
    if self.lane_change_state == LaneChangeState.laneChangeStarting and previous != self.lane_change_state:
      self._auto_lane_completed = False
    if ((self.carrot_lane_change_count > 0 or self._auto_app_direction) and previous == LaneChangeState.off and
        self.lane_change_state == LaneChangeState.preLaneChange):
      self._auto_lane_completed = False
    self._update_auto_turn_geometry(carstate, modeldata)
    permitted = (self.tesla_auto_signals and lateral_active and carstate.canValid and not carstate.trailerConnected and
                 not getattr(carstate, "vehicleSensorsInvalid", False) and
                 str(getattr(carstate, "gearShifter", "drive")) == "drive" and carstate.turnSignalDriverValid and
                 not driver_st and not self._manual_signal_direction and
                 not (carstate.leftBlinker and carstate.rightBlinker) and self.desire_disable_count == 0)
    if not permitted or self.lane_change_state == LaneChangeState.off or self._auto_lane_completed:
      if not permitted and self._auto_lane_direction != LaneChangeDirection.none:
        self._auto_lane_completed = True
      self._auto_lane_direction = LaneChangeDirection.none
    elif (atc_enabled and self._auto_lane_direction == LaneChangeDirection.none and
          (self.lane_change_state == LaneChangeState.preLaneChange or previous == LaneChangeState.preLaneChange)):
      # The existing FSM owns the maneuver, including a bounded App preparation wait.
      self._auto_lane_direction = LaneChangeDirection.left if atc_st == BLINKER_LEFT else LaneChangeDirection.right
    if permitted:
      if (not self._auto_turn_consumed and self._auto_turn_direction != LaneChangeDirection.none and
          (navigation_direction == self._auto_turn_direction or
           (self.atc_type in ("", "none", "none prepare", "straight", "straight prepare") and self._auto_turn_started >= 0.3))):
        direction = self._auto_turn_direction
      elif atc_enabled and self.turn_direction != TurnDirection.none and not self._auto_turn_consumed:
        direction = LaneChangeDirection.left if self.turn_direction == TurnDirection.turnLeft else LaneChangeDirection.right
      elif self._auto_lane_direction != LaneChangeDirection.none:
        direction = self._auto_lane_direction
      elif (turn_handoff and not self._auto_turn_consumed and self.atc_active != 2 and not self.blinker_ignore and
            self.auto_signal_direction == (LaneChangeDirection.left if self.atc_type == "turn left" else LaneChangeDirection.right)):
        # ATC suppresses maneuver changes for one frame, not an already requested same-direction lamp.
        direction = self.auto_signal_direction
      elif (not self._auto_turn_consumed and not self._auto_lane_completed and self.atc_active != 2 and
            not self.blinker_ignore and getattr(carrot_man, "xTurnInfo", 0) in (1, 2) and
            self.atc_type in ("turn left prepare", "turn right prepare", "atc left", "atc right") and
            0 < getattr(carrot_man, "xDistToTurn", 0) <= min(250, max(40, carstate.vEgo * 8 + 20))):
        direction = LaneChangeDirection.left if carrot_man.xTurnInfo == 1 else LaneChangeDirection.right
      if direction != LaneChangeDirection.none and turn_type and not self._auto_turn_consumed:
        if direction != self._auto_turn_direction:
          self._auto_turn_started = self._auto_turn_peak_angle = self._auto_turn_settled = 0.0
        self._auto_turn_direction = direction
    if direction == LaneChangeDirection.none and self._auto_turn_direction != LaneChangeDirection.none:
      self._auto_turn_consumed = True
      self._auto_turn_direction = LaneChangeDirection.none
    if direction != LaneChangeDirection.none and direction != self.auto_signal_direction:
      self._auto_signal_counter += 1
    self.auto_signal_direction = direction
    self.auto_signal_request_id = self._auto_signal_counter if direction != LaneChangeDirection.none else 0
