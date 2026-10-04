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
from openpilot.selfdrive.controls.lib.desire_lib.lever import LEVER_REQUEST_WAIT_S, LaneCrossing, LeverRequestWait, lever_maneuver

# Without a model-detected crossing, a lever press this early in laneChangeFinishing is still the driver
# re-lighting the lamp of the change that just crossed, not a request for the next one.
STALK_QUEUE_GUARD_S = 0.3


class DesireHelper:
  def __init__(self):
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
    self.crossing = LaneCrossing()
    self.blinker_hold_direction = LaneChangeDirection.none  # modelV2.meta.laneChangeBlinkerHold

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
    if counts[0] != last[0]:
      return BLINKER_LEFT
    if counts[1] != last[1]:
      return BLINKER_RIGHT
    return BLINKER_NONE

  def _update_atc_blinker(self, carrotMan, driver_blinker_state, remote=None):
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    # 유지 카운트는 DesireHelper에서 관리
    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif remote in ('laneLeft', 'laneRight'):
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state = BLINKER_LEFT if remote == 'laneLeft' else BLINKER_RIGHT
      atc_blinker_state = self.carrot_blinker_state
    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
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

    # blinker_ignore
    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state = BLINKER_NONE
      atc_desire_enabled = False

    # 타입 변경 1프레임 무시
    if self.atc_type != atc_type:
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

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
    if self.lever_wait.update(self.laneChangeLeverWait, stalk_press, lever, driver_st, self.lane_change_state,
                              self.laneChangeDelay):
      driver_enabled = driver_changed = False  # one-touch request timed out: ignore the lamp until it goes dark
    remote = self.bluetooth_commands.read(allowed=(lateral_active and carstate.canValid and
      not below_lane_change_speed and not trailer_maneuver_blocked))
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st, remote)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state = driver_st if driver_enabled else atc_st

    # 선택된 side (FSM은 이 side만 참고)
    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None
    atc_lane_change_requested = (
      atc_enabled and
      self.atc_type in ("fork left", "fork right", "atc left", "atc right")
    )
    atc_lane_change_manual_only = (
      atc_enabled and
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
      if self.carrot_lane_change_count > 0:
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

      # BlinkerLatchedTurn: the driver's lever replaces the speed/lane-line guess (see lever_maneuver).
      lever_type, lever_undecided = (None, False)
      if driver_enabled and side is not None:
        lever_type, lever_undecided = lever_maneuver(lever, v_ego, self.blinkerLatchedTurn)
      if lever_type is not None:
        new_type = lever_type

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
              if start_gate and not lever_undecided:
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

    # keep pulse
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    return self.desire
