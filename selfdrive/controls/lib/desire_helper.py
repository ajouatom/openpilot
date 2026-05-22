from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
import numpy as np
from openpilot.common.params import Params

from openpilot.selfdrive.controls.lib.desire_lib.constants import (
  LaneChangeState, LaneChangeDirection, TurnDirection,
  LANE_CHANGE_SPEED_MIN, LANE_CHANGE_TIME_MAX,
  BLINKER_NONE, BLINKER_LEFT, BLINKER_RIGHT,
  DESIRES, TURN_DESIRES
)
from openpilot.selfdrive.controls.lib.desire_lib.side_state import SideState
from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type
class DesireHelper:
  def __init__(self):
    self.params = Params()
    self.frame = 0

    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_delay = 0.0
    self.maneuver_type = "none"

    self.desire = log.Desire.none
    self.turn_direction = TurnDirection.none
    self.enable_turn_desires = True
    self.turn_desire_state = False
    self.desire_disable_count = 0
    self.turn_disable_count = 0

    self.left = SideState("left")
    self.right = SideState("right")

    self.blinker_ignore = False
    self.driver_blinker_state = BLINKER_NONE
    self.carrot_blinker_state = BLINKER_NONE
    self.carrot_lane_change_count = 0
    self.carrot_cmd_index_last = 0
    self.atc_type = ""
    self.atc_active = 0

    self.auto_lane_change_enable = False
    self.next_lane_change = False
    self.lane_change_continuous = False
    self.unsafe_cancel_timer= 0.0

    self.keep_pulse_timer = 0.0

    self.laneChangeNeedTorque = 0
    self.laneChangeBsd = 0
    self.laneLineCheck = 0
    self.laneChangeDelay = 0.0
    self.modelTurnSpeedFactor = 0.0
    self.model_turn_speed = 200.0
    self.side_gap_margin  = 3.0
    self.bsd_clear_sec    = 0.0
    self.object_clear_sec = 0.3
    self.bsd_hold_sec     = 1.5
    self.ignore_bsd = False

    self.prev_desire_enabled = False
    self.desireLog = ""

    self.lane_change_available_left = False
    self.lane_change_available_right = False

  # ─────────────────────────────────────────────
  def _update_params_periodic(self):
    if self.frame % 100 == 0:
      self.laneChangeNeedTorque = self.params.get_int("LaneChangeNeedTorque")
      self.laneChangeBsd        = self.params.get_int("LaneChangeBsd")
      self.ignore_bsd           = (self.laneChangeBsd < 0)
      self.laneLineCheck        = self.params.get_int("LaneLineCheck")
      self.laneChangeDelay      = self.params.get_int("LaneChangeDelay") * 0.1
      self.modelTurnSpeedFactor = self.params.get_int("ModelTurnSpeedFactor") * 0.1
      self.side_gap_margin      = self.params.get_int("SideGapMargin") * 0.1
      self.bsd_clear_sec        = self.params.get_int("BsdClearSec") * 0.1
      self.object_clear_sec     = self.params.get_int("ObjectClearSec") * 0.1
      self.bsd_hold_sec         = self.params.get_int("BsdHoldSec") * 0.1
      self.lane_change_continuous = self.params.get_int("LaneChangeContinuous") > 0

  def _make_model_turn_speed(self, modeldata):
    if self.modelTurnSpeedFactor > 0:
      model_turn_speed = np.interp(self.modelTurnSpeedFactor,
                                   modeldata.velocity.t,
                                   modeldata.velocity.x) * CV.MS_TO_KPH * 1.2
      self.model_turn_speed = self.model_turn_speed * 0.9 + model_turn_speed * 0.1
    else:
      self.model_turn_speed = 200.0

  def _check_desire_state(self, modeldata, carstate, maneuver_type):
    desire_state = modeldata.meta.desireState
    orientation_rate        = abs(modeldata.orientationRate.z[5])
    orientation_rate_future = abs(modeldata.orientationRate.z[15])
    self.turn_desire_state = (desire_state[1] + desire_state[2]) > 0.1
    if maneuver_type == "turn" and abs(carstate.steeringAngleDeg) > 80 and orientation_rate_future < orientation_rate:
      self.turn_disable_count = int(10.0 / DT_MDL)
    else:
      self.turn_disable_count = max(0, self.turn_disable_count - 1)

  # ─────────────────────────────────────────────
  def _update_driver_blinker(self, carstate):
    st = carstate.leftBlinker * 1 + carstate.rightBlinker * 2
    changed = st != self.driver_blinker_state
    self.driver_blinker_state = st
    enabled = st in (BLINKER_LEFT, BLINKER_RIGHT)
    if self.laneChangeNeedTorque < 0:
      enabled = False
    return st, changed, enabled

  def _update_atc_blinker(self, carrotMan, driver_blinker_state):
    atc_type = carrotMan.atcType
    atc_blinker_state = BLINKER_NONE

    if self.carrot_lane_change_count > 0:
      atc_blinker_state = self.carrot_blinker_state
    elif carrotMan.carrotCmdIndex != self.carrot_cmd_index_last and carrotMan.carrotCmd == "LANECHANGE":
      self.carrot_cmd_index_last    = carrotMan.carrotCmdIndex
      self.carrot_lane_change_count = int(0.2 / DT_MDL)
      self.carrot_blinker_state     = BLINKER_LEFT if carrotMan.carrotArg == "LEFT" else BLINKER_RIGHT
      atc_blinker_state             = self.carrot_blinker_state
    elif atc_type in ("turn left", "turn right"):
      if self.atc_active != 2:
        atc_blinker_state    = BLINKER_LEFT if atc_type == "turn left" else BLINKER_RIGHT
        self.atc_active      = 1
        self.blinker_ignore  = False
    elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
      if self.atc_active != 2:
        atc_blinker_state = BLINKER_LEFT if atc_type in ("fork left", "atc left") else BLINKER_RIGHT
        self.atc_active   = 1
    else:
      self.atc_active = 0

    if driver_blinker_state != BLINKER_NONE and atc_blinker_state != BLINKER_NONE and driver_blinker_state != atc_blinker_state:
      atc_blinker_state = BLINKER_NONE
      self.atc_active   = 2

    atc_desire_enabled = atc_blinker_state in (BLINKER_LEFT, BLINKER_RIGHT)

    if driver_blinker_state == BLINKER_NONE:
      self.blinker_ignore = False
    if self.blinker_ignore:
      atc_blinker_state  = BLINKER_NONE
      atc_desire_enabled = False

    if self.atc_type != atc_type:
      atc_desire_enabled = False
    self.atc_type = atc_type

    return atc_blinker_state, atc_desire_enabled

  # ─────────────────────────────────────────────
  def _process_sides(self, carstate, modeldata, radarState):
    self.left.update_lane_geometry(
      modeldata.laneLines[0], modeldata.laneLineProbs[0],
      modeldata.laneLines[1], modeldata.roadEdges[0],
      cur_prob=modeldata.laneLineProbs[1],
    )
    self.right.update_lane_geometry(
      modeldata.laneLines[3], modeldata.laneLineProbs[3],
      modeldata.laneLines[2], modeldata.roadEdges[1],
      cur_prob=modeldata.laneLineProbs[2],
    )

    self.left.update_lane_line_info(carstate.leftLaneLine)
    self.right.update_lane_line_info(carstate.rightLaneLine)

    v_ego = carstate.vEgo

    self.left.update_obstacles(
        v_ego, radarState.leadLeft, carstate.leftBlindspot, self.ignore_bsd,
        bsd_hold_sec       = self.bsd_hold_sec,
        side_gap_margin    = self.side_gap_margin,
        corner_long_dist_f = float(carstate.leftLongDist),
        corner_long_dist_r = float(carstate.leftLongDistRear),
        corner_lat_dist    = float(carstate.leftLatDist),
        object_clear_sec   = self.object_clear_sec,
    )
    self.right.update_obstacles(
        v_ego, radarState.leadRight, carstate.rightBlindspot, self.ignore_bsd,
        bsd_hold_sec       = self.bsd_hold_sec,
        side_gap_margin    = self.side_gap_margin,
        corner_long_dist_f = float(carstate.rightLongDist),
        corner_long_dist_r = float(carstate.rightLongDistRear),
        corner_lat_dist    = float(carstate.rightLatDist),
        object_clear_sec   = self.object_clear_sec,
    )

    # compute available (include BSD+object)
    if self.laneLineCheck >= 1:
      left_line_ok = self.left.lane_line_info_mod in (0, 5)
      right_line_ok = self.right.lane_line_info_mod in (0, 5)
    else:
      left_line_ok = self.left.lane_line_info_raw < 20
      right_line_ok = self.right.lane_line_info_raw < 20
    
    self.left.compute_lane_change_available(lane_line_info_lt_20=left_line_ok,  bsd_level=self.laneChangeBsd, bsd_clear_sec=self.bsd_clear_sec)
    self.right.compute_lane_change_available(lane_line_info_lt_20=right_line_ok, bsd_level=self.laneChangeBsd, bsd_clear_sec=self.bsd_clear_sec)

    self.left.update_triggers()
    self.right.update_triggers()

    self.lane_change_available_left  = self.left.lane_change_available
    self.lane_change_available_right = self.right.lane_change_available

  def _get_selected_side(self, blinker_state: int) -> SideState:
    return self.left if blinker_state == BLINKER_LEFT else self.right

  # ─────────────────────────────────────────────
  def update(self, carstate, modeldata, lateral_active, lane_change_prob, carrotMan, radarState):
    self.frame += 1
    self._update_params_periodic()
    self._make_model_turn_speed(modeldata)

    self.carrot_lane_change_count = max(0, self.carrot_lane_change_count - 1)
    self.lane_change_delay        = max(0.0, self.lane_change_delay - DT_MDL)
    self.unsafe_cancel_timer      = max(0.0, self.unsafe_cancel_timer - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    self._process_sides(carstate, modeldata, radarState)
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    driver_st, driver_changed, driver_enabled = self._update_driver_blinker(carstate)
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state  = driver_st if driver_enabled else atc_st

    side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None
    atc_lane_change_manual_only = (
      atc_enabled and
      not driver_enabled and
      self.atc_type in ("fork left", "atc left")
    )

    # ── lane_change_available 의 False→True 전환 감지 ──────────────
    # commit_last() 는 update() 말미에 호출되므로
    # 이 시점의 _last 값은 "이전 프레임의 available" 임
    avail_now  = side.lane_change_available      if side is not None else False
    avail_last = side.lane_change_available_last if side is not None else False

    clear_ready = (
      side is not None and
      side.bsd_clear_count >= max(1, int(self.bsd_clear_sec / DT_MDL)) and
      side.object_clear_count >= max(1, int(self.object_clear_sec / DT_MDL))
    )

    # ── avail_just_cleared: off 상태에서 깜빡이 켜진 채 available이 된 순간 ──
    # (not avail_last) 조건 제거 → available이면 계속 재진입 허용
    avail_just_cleared = (
        desire_enabled and
        side is not None and
        avail_now and
        self.prev_desire_enabled and        # 깜빡이를 새로 켠 게 아니라 유지 중
        self.lane_change_state == LaneChangeState.off and  # off 상태일 때만
        self.unsafe_cancel_timer <= 0.0 and
        (not self.next_lane_change)
    )

    # 차량이 사라진 직후 즉시 재진입하지 않고,
    # BSD/object clear 안정화와 unsafe cancel cooldown 이후에만 재진입 허용

    # ── auto lane change trigger ──────────────────────────────────
    auto_lane_change_trigger = False

    if desire_enabled and side is not None:
      if self.carrot_lane_change_count > 0:
        auto_lane_change_trigger = side.lane_change_available

      elif self.next_lane_change:
        auto_lane_change_trigger = (
          self.auto_lane_change_enable and
          side.lane_change_available
        )

      else:
        auto_lane_change_trigger = (
          self.auto_lane_change_enable and
          (not atc_lane_change_manual_only) and
          side.edge_available and
          (side.lane_available_trigger or side.lane_appeared) and
          side.lane_change_available
        )

      self.desireLog = f"{side.name}:ALC={self.auto_lane_change_enable}, "
    else:
      self.auto_lane_change_enable = False
      if not desire_enabled:
        self.next_lane_change = False

    # ───────────────────────── FSM ─────────────────────────
    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state     = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction        = TurnDirection.none
      self.maneuver_type         = "none"

    elif self.desire_disable_count > 0:
      self.lane_change_state     = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.turn_direction        = TurnDirection.none
      self.maneuver_type         = "none"

    else:
      if desire_enabled and side is not None:
        new_type = classify_maneuver_type(
          blinker_state=blinker_state, carstate=carstate, side=side,
          turn_desire_state=self.turn_desire_state,
          atc_type=self.atc_type, old_type=self.maneuver_type,
        )
      else:
        new_type = "none"

      if self.maneuver_type == "lane_change" and new_type == "turn" and self.lane_change_state not in (
          LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting):
        self.maneuver_type     = "turn"
        self.lane_change_state = LaneChangeState.off
      elif self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
        self.maneuver_type = new_type

      # ─ TURN mode ─
      if desire_enabled and self.maneuver_type == "turn" and self.enable_turn_desires:
        self.lane_change_state = LaneChangeState.off
        if self.turn_disable_count > 0:
          self.turn_direction        = TurnDirection.none
          self.lane_change_direction = LaneChangeDirection.none
        else:
          self.turn_direction        = TurnDirection.turnLeft if blinker_state == BLINKER_LEFT else TurnDirection.turnRight
          self.lane_change_direction = self.turn_direction

      # ─ Lane change FSM ─
      else:
        self.turn_direction = TurnDirection.none

        # ── off 상태 ──────────────────────────────────────────────
        if self.lane_change_state == LaneChangeState.off:
          reentry = (
            not self.prev_desire_enabled or   # A) 깜빡이 새로 켠 순간
            self.next_lane_change or           # B) 연속 차선변경
            avail_just_cleared                 # C) 대기 중 차량 사라짐
          )


          if desire_enabled and reentry and not below_lane_change_speed and side is not None:
            self.lane_change_state   = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0

            if not self.next_lane_change:
              self.lane_change_delay = self.laneChangeDelay

            lane_exist_counter_side    = side.lane_exist_count.counter
            lane_change_available_geom = side.lane_change_available_geom

            if self.next_lane_change:
              self.auto_lane_change_enable = True
            elif avail_just_cleared:
              # 차량이 방금 사라진 케이스:
              # available이 이미 True이므로 ALC 즉시 활성화
              self.auto_lane_change_enable = True
            else:
              self.auto_lane_change_enable = False if (lane_exist_counter_side > 0 or lane_change_available_geom) else True

        # ── preLaneChange 상태 ───────────────────────────────────
        elif self.lane_change_state == LaneChangeState.preLaneChange:
          if side is None:
            self.lane_change_state     = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

          else:
            self.lane_change_direction = LaneChangeDirection.left if blinker_state == BLINKER_LEFT else LaneChangeDirection.right

            torque_cond    = (carstate.steeringTorque > 0) if blinker_state == BLINKER_LEFT else (carstate.steeringTorque < 0)
            torque_applied = carstate.steeringPressed and torque_cond

            # laneChangeBsd >= 1 일 때만 BSD를 hard block 으로 사용
            bsd_active = (self.laneChangeBsd >= 1) and (side.bsd_hold_counter > 0)
            object_active = side.side_object_detected

            solid_line_blocked = (self.laneLineCheck >= 2) and \
                (not side.lane_change_available_geom) and \
                (side.lane_available or side.edge_available)

            geom_blocked = (not side.lane_change_available_geom) and (not solid_line_blocked)
            unsafe_prechange = bsd_active or object_active or geom_blocked

            start_gate = self.lane_change_delay == 0 and \
                (side.lane_change_available_geom or side.lane_line_info_edge_detect or solid_line_blocked)

            if side.lane_change_available and not self.auto_lane_change_enable:
              self.auto_lane_change_enable = True

            if not desire_enabled or below_lane_change_speed:
              self.lane_change_state     = LaneChangeState.off
              self.lane_change_direction = LaneChangeDirection.none
              self.auto_lane_change_enable = False
              self.next_lane_change        = False

            elif unsafe_prechange:
              # 대기 중 위험 요소가 생기면 강하게 취소
              self.lane_change_state       = LaneChangeState.off
              self.lane_change_direction   = LaneChangeDirection.none
              self.auto_lane_change_enable = False
              self.next_lane_change        = False
              self.lane_change_delay       = max(self.laneChangeDelay, 1.5)
              self.unsafe_cancel_timer     = max(self.unsafe_cancel_timer, 1.5)

            elif not start_gate:
              pass  # 딜레이 중 또는 차선 미감지 → 대기

            elif solid_line_blocked:
              if torque_applied:
                self.lane_change_state = LaneChangeState.laneChangeStarting

            elif self.laneChangeNeedTorque > 0:
              if torque_applied and side.lane_change_available:
                self.lane_change_state = LaneChangeState.laneChangeStarting
              elif self.laneChangeBsd == 0 and torque_applied and side.lane_change_available_no_bsd:
                # 경고 모드(0)에서만 토크로 BSD override 허용
                self.lane_change_state = LaneChangeState.laneChangeStarting

            elif driver_enabled or self.next_lane_change:
              if side.lane_change_available:
                self.lane_change_state = LaneChangeState.laneChangeStarting
              elif self.laneChangeBsd == 0 and side.lane_change_available_no_bsd and torque_applied:
                # laneChangeBsd >= 1 에서는 여기로 시작하면 안 됨
                self.lane_change_state = LaneChangeState.laneChangeStarting

            else:
              # 자동 차선변경 (ALC)
              if (torque_applied or auto_lane_change_trigger or side.lane_line_info_edge_detect) \
                      and side.lane_change_available:
                self.lane_change_state = LaneChangeState.laneChangeStarting

        # ── laneChangeStarting 상태 ──────────────────────────────
        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          bsd_active   = (side is not None) and (self.laneChangeBsd >= 1) and (side.bsd_hold_counter > 0)
          object_active = (side is not None) and side.side_object_detected
          geom_lost    = (side is None) or (not side.lane_change_available_geom)

          unsafe_now = bsd_active or object_active or geom_lost

          if unsafe_now:
            # 차선 변경 도중 위험 감지 → 강하게 즉시 취소
            self.lane_change_direction   = LaneChangeDirection.none
            self.lane_change_state       = LaneChangeState.off
            self.auto_lane_change_enable = False
            self.next_lane_change        = False
            self.lane_change_delay       = max(self.laneChangeDelay, 2.0)
            self.unsafe_cancel_timer     = max(
              self.unsafe_cancel_timer,
              max(self.bsd_clear_sec, self.object_clear_sec, 1.5)
            )

          else:
            avail = side.lane_change_available_hold if side is not None else False

            if not avail:
              # geometry flicker 정도만 finishing으로 넘김
              self.lane_change_direction = LaneChangeDirection.none
              self.lane_change_state     = LaneChangeState.laneChangeFinishing
              self.next_lane_change      = False
            else:
              self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
              if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
                self.lane_change_state = LaneChangeState.laneChangeFinishing

        # ── laneChangeFinishing 상태 ─────────────────────────────
        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            self.lane_change_direction = LaneChangeDirection.none

            clear_ready = (
              side is not None and
              side.bsd_clear_count >= max(1, int(self.bsd_clear_sec / DT_MDL)) and
              side.object_clear_count >= max(1, int(self.object_clear_sec / DT_MDL))
            )

            continuous_ok = (
              desire_enabled and
              self.lane_change_continuous and
              clear_ready and
              self.unsafe_cancel_timer <= 0.0
            )

            if continuous_ok:
              self.lane_change_delay = max(self.laneChangeDelay, 2.5)
              self.next_lane_change  = True
            else:
              self.next_lane_change = False

            self.lane_change_state = LaneChangeState.off


    # ── 타이머 ───────────────────────────────────────────────────
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    # ── commit last ──────────────────────────────────────────────
    self.left.commit_last()
    self.right.commit_last()

    self.prev_desire_enabled = desire_enabled

    # ── 반대 방향 토크 cancel ────────────────────────────────────
    steering_pressed_cancel = carstate.steeringPressed and (
      (carstate.steeringTorque < 0 and blinker_state == BLINKER_LEFT) or
      (carstate.steeringTorque > 0 and blinker_state == BLINKER_RIGHT)
    )
    if steering_pressed_cancel and self.lane_change_state != LaneChangeState.off:
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_state     = LaneChangeState.off
      self.blinker_ignore        = True

    # ── final desire ─────────────────────────────────────────────
    if self.turn_direction != TurnDirection.none:
      self.desire                = TURN_DESIRES[self.turn_direction]
      self.lane_change_direction = self.turn_direction
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # ── keep pulse ───────────────────────────────────────────────
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    return self.desire
