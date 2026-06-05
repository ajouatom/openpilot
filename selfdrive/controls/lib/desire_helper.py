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
    self.lane_change_delay_timer = 0.0
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
    self.unsafe_cancel_timer = 0.0

    self.keep_pulse_timer = 0.0

    self.lane_change_need_torque = 0
    self.lane_change_bsd = 0
    self.lane_line_check = 0
    self.lane_change_delay = 0.0
    self.model_turn_speed_factor = 0.0
    self.model_turn_speed = 200.0
    self.side_gap_margin  = 3.0
    self.bsd_clear_sec    = 0.0
    self.object_clear_sec = 0.3
    self.bsd_hold_sec     = 1.5
    self.ignore_bsd = False

    self.prev_desire_enabled = False
    self.desire_log = ""

    self.lane_change_available_left = False
    self.lane_change_available_right = False

    self.lane_change_completed_on_blinker = False
    self.blinker_off_timer = 0.0
    self.completed_direction = LaneChangeDirection.none

    #  연속 변경 버그 및 1프레임 펄스 신호 제어를 위한 타이머 변수
    self.lane_change_cooldown_timer = 0.0
    self.lane_appeared_block_timer = 0.0

  # ─────────────────────────────────────────────
  def _update_params_periodic(self):
    if self.frame % 100 == 0:
      self.lane_change_need_torque = self.params.get_int("LaneChangeNeedTorque")
      self.lane_change_bsd        = self.params.get_int("LaneChangeBsd")
      self.ignore_bsd           = (self.lane_change_bsd < 0)
      self.lane_line_check        = self.params.get_int("LaneLineCheck")
      self.lane_change_delay      = self.params.get_int("LaneChangeDelay") * 0.1
      self.model_turn_speed_factor = self.params.get_int("ModelTurnSpeedFactor") * 0.1
      self.side_gap_margin      = self.params.get_int("SideGapMargin") * 0.1
      self.bsd_clear_sec        = self.params.get_int("BsdClearSec") * 0.1
      self.object_clear_sec     = self.params.get_int("ObjectClearSec") * 0.1
      self.bsd_hold_sec         = self.params.get_int("BsdHoldSec") * 0.1

  def _make_model_turn_speed(self, modeldata):
    if self.model_turn_speed_factor > 0:
      model_turn_speed = np.interp(self.model_turn_speed_factor,
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
    if self.lane_change_need_torque < 0:
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
        corner_long_dist_r = float(carstate.leftRearLongDist),
        corner_lat_dist    = float(carstate.leftLatDist),
        object_clear_sec   = self.object_clear_sec,
    )
    self.right.update_obstacles(
        v_ego, radarState.leadRight, carstate.rightBlindspot, self.ignore_bsd,
        bsd_hold_sec       = self.bsd_hold_sec,
        side_gap_margin    = self.side_gap_margin,
        corner_long_dist_f = float(carstate.rightLongDist),
        corner_long_dist_r = float(carstate.rightRearLongDist),
        corner_lat_dist    = float(carstate.rightLatDist),
        object_clear_sec   = self.object_clear_sec,
    )

    # compute available (include BSD+object)
    if self.lane_line_check >= 1:
      left_line_ok = self.left.lane_line_info_mod in (0, 5)
      right_line_ok = self.right.lane_line_info_mod in (0, 5)
    else:
      left_line_ok = self.left.lane_line_info_raw // 10 != 2
      right_line_ok = self.right.lane_line_info_raw // 10 != 2

    self.left.compute_lane_change_available(lane_line_info_lt_20=left_line_ok,  bsd_level=self.lane_change_bsd, bsd_clear_sec=self.bsd_clear_sec)
    self.right.compute_lane_change_available(lane_line_info_lt_20=right_line_ok, bsd_level=self.lane_change_bsd, bsd_clear_sec=self.bsd_clear_sec)

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
    self.lane_change_delay_timer  = max(0.0, self.lane_change_delay_timer - DT_MDL)
    self.unsafe_cancel_timer      = max(0.0, self.unsafe_cancel_timer - DT_MDL)

    #  매 프레임 시간 감소 연산
    self.lane_change_cooldown_timer = max(0.0, self.lane_change_cooldown_timer - DT_MDL)
    self.lane_appeared_block_timer  = max(0.0, self.lane_appeared_block_timer - DT_MDL)

    v_ego = carstate.vEgo
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    self._process_sides(carstate, modeldata, radarState)
    self._check_desire_state(modeldata, carstate, self.maneuver_type)

    driver_st, driver_changed, driver_enabled = self._update_driver_blinker(carstate)
    atc_st, atc_enabled = self._update_atc_blinker(carrotMan, driver_st)

    desire_enabled = driver_enabled or atc_enabled
    blinker_state  = driver_st if driver_enabled else atc_st

    # ── 깜빡이 노이즈 필터링 및 반대 방향 전환 감지 ──
    if desire_enabled:
      self.blinker_off_timer = 0.0
      if (blinker_state == BLINKER_LEFT and self.completed_direction == LaneChangeDirection.right) or \
         (blinker_state == BLINKER_RIGHT and self.completed_direction == LaneChangeDirection.left):
        self.lane_change_completed_on_blinker = False

    else:
      self.blinker_off_timer += DT_MDL
      if self.blinker_off_timer > 0.5:
        self.lane_change_completed_on_blinker = False
        self.completed_direction = LaneChangeDirection.none

    # 이미 차선 변경 플래그가 살아서 움직이는 중이라면, 깜빡이 OFF와 무관하게 기존 진행 방향의 side를 유지
    if self.lane_change_state in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
      side = self.left if self.lane_change_direction == LaneChangeDirection.left else self.right
    else:
      side = self._get_selected_side(blinker_state) if blinker_state in (BLINKER_LEFT, BLINKER_RIGHT) else None

    # [이동 및 수정] NameError를 해결하기 위해 side 변수 선언 하단으로 이동 및 펄스 잠금 방식으로 보완
    if side is not None and side.lane_appeared:
      self.lane_appeared_block_timer = 2.5

    # 램프 구간 특유의 높은 곡률(급코너) 상태인지 감지
    is_shaping_ramp = abs(carstate.steeringAngleDeg) > 15.0 or abs(modeldata.orientationRate.z[5]) > 0.05

    atc_lane_change_manual_only = (
      atc_enabled and
      not driver_enabled and
      self.atc_type in ("fork left", "atc left")
    )

    # ── lane_change_available 의 False→True 전환 감지 ──────────────
    avail_now  = side.lane_change_available      if side is not None else False
    avail_last = side.lane_change_available_last if side is not None else False

    # ── avail_just_cleared: off 상태에서 깜빡이 켜진 채 available이 된 순간 ──
    avail_just_cleared = (
      desire_enabled and
      side is not None and
      avail_now and
      not avail_last and
      self.prev_desire_enabled and
      self.lane_change_state == LaneChangeState.off and
      self.unsafe_cancel_timer <= 0.0
    )

    avail_retry = (
      desire_enabled and
      side is not None and
      avail_now and
      avail_last and
      self.prev_desire_enabled and
      self.lane_change_state == LaneChangeState.off and self.unsafe_cancel_timer <= 0.0
    )

    # ── auto lane change trigger ──────────────────────────────────
    auto_lane_change_trigger = False

    if desire_enabled and side is not None:
      if self.carrot_lane_change_count > 0:
        auto_lane_change_trigger = side.lane_change_available
      else:
        auto_lane_change_trigger = (
          self.auto_lane_change_enable and
          (not atc_lane_change_manual_only) and
          (not is_shaping_ramp) and
          (self.lane_appeared_block_timer == 0.0) and  #  차선이 새로 발견된 직후 유예 기간 동안 ALC 금지
          side.edge_available and
          side.lane_available_trigger and
          side.lane_change_available
        )
      self.desire_log = f"{side.name}:ALC={self.auto_lane_change_enable}, "
    else:
      self.auto_lane_change_enable = False

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
            not self.prev_desire_enabled or
            avail_just_cleared or
            avail_retry
          )

          #  완료 플래그 차단 처리 - 단, ATC의 명시적인 새로운 명령 펄스가 돌고 있을 때는 예외 허용
          if self.lane_change_completed_on_blinker:
            if not (atc_enabled and self.carrot_lane_change_count > 0):
              reentry = False

          #  쿨다운 제한 처리 - 단, ATC의 명시적인 새로운 명령 펄스가 돌고 있을 때는 예외 허용
          if self.lane_change_cooldown_timer > 0.0:
            if not (atc_enabled and self.carrot_lane_change_count > 0):
              reentry = False

          if desire_enabled and reentry and not below_lane_change_speed and side is not None:
            self.lane_change_state   = LaneChangeState.preLaneChange
            self.lane_change_ll_prob = 1.0
            self.lane_change_delay_timer   = self.lane_change_delay

            lane_exist_counter_side    = side.lane_exist_count.counter
            lane_change_available_geom = side.lane_change_available_geom

            if avail_just_cleared:
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

            bsd_detected = side.bsd_hold_counter > 0
            object_detected = side.side_object_detected

            vehicle_blocked = False
            if bsd_detected or object_detected:
              if self.lane_change_bsd >= 1:
                vehicle_blocked = True
              elif self.lane_change_bsd == 0:
                vehicle_blocked = not torque_applied

            solid_line_blocked = (self.lane_line_check >= 2) and \
                (not side.lane_change_available_geom) and \
                (side.lane_available or side.edge_available)

            geom_blocked = (not side.lane_change_available_geom) and (not solid_line_blocked)

            unsafe_prechange = vehicle_blocked or geom_blocked

            start_gate = self.lane_change_delay_timer == 0 and \
                (side.lane_change_available_geom or side.lane_line_info_edge_detect or solid_line_blocked)

            if side.lane_change_available and not self.auto_lane_change_enable:
              self.auto_lane_change_enable = True

            if not desire_enabled or below_lane_change_speed:
              self.lane_change_state     = LaneChangeState.off
              self.lane_change_direction = LaneChangeDirection.none
              self.auto_lane_change_enable = False

            elif unsafe_prechange:
              self.lane_change_state       = LaneChangeState.off
              self.lane_change_direction   = LaneChangeDirection.none
              self.auto_lane_change_enable = False
              self.lane_change_delay_timer       = max(self.lane_change_delay, 1.5)
              self.unsafe_cancel_timer     = max(self.unsafe_cancel_timer, 1.5)

            elif not start_gate:
              pass

            elif solid_line_blocked:
              if torque_applied:
                self.lane_change_state = LaneChangeState.laneChangeStarting

            elif self.lane_change_need_torque > 0:
              if torque_applied and side.lane_change_available:
                self.lane_change_state = LaneChangeState.laneChangeStarting
              elif self.lane_change_bsd == 0 and torque_applied and side.lane_change_available_no_bsd:
                self.lane_change_state = LaneChangeState.laneChangeStarting

            elif driver_enabled:
              if side.lane_change_available:
                self.lane_change_state = LaneChangeState.laneChangeStarting
              elif self.lane_change_bsd == 0 and side.lane_change_available_no_bsd and torque_applied:
                self.lane_change_state = LaneChangeState.laneChangeStarting

            else:
              if (torque_applied or auto_lane_change_trigger or side.lane_line_info_edge_detect) \
                      and side.lane_change_available:
                self.lane_change_state = LaneChangeState.laneChangeStarting


        # ── laneChangeStarting 상태 ──────────────────────────────
        elif self.lane_change_state == LaneChangeState.laneChangeStarting:
          if self.lane_change_timer == 0.0:
            self.lane_change_completed_on_blinker = True

          bsd_active   = (side is not None) and (side.bsd_hold_counter > 0) and (self.lane_change_bsd >= 0)
          object_active = (side is not None) and side.side_object_detected
          geom_lost    = (side is None) or (not side.lane_change_available_geom)

          unsafe_now = bsd_active or object_active or geom_lost

          if unsafe_now:
            self.lane_change_direction   = LaneChangeDirection.none
            self.lane_change_state       = LaneChangeState.off
            self.auto_lane_change_enable = False
            self.lane_change_delay_timer       = max(self.lane_change_delay, 2.0)
            self.unsafe_cancel_timer     = max(
              self.unsafe_cancel_timer,
              max(self.bsd_clear_sec, self.object_clear_sec, 1.5)
            )
            self.lane_change_completed_on_blinker = False

          else:
            avail = side.lane_change_available_hold if side is not None else False

            if not avail:
              # 방향을 none으로 지우기 전에 완료된 방향을 먼저 안전하게 저장합니다.
              self.completed_direction = self.lane_change_direction
              self.lane_change_direction = LaneChangeDirection.none
              self.lane_change_state     = LaneChangeState.laneChangeFinishing
            else:
              self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
              if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
                self.lane_change_state = LaneChangeState.laneChangeFinishing

        # ── laneChangeFinishing 상태 ─────────────────────────────
        elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
          self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
          if self.lane_change_ll_prob > 0.99:
            # 다른 예외 조건(prob 감소 등)으로 진입했을 때를 위해 안전장치 추가
            if self.lane_change_direction != LaneChangeDirection.none:
              self.completed_direction = self.lane_change_direction

            self.lane_change_direction = LaneChangeDirection.none
            self.lane_change_state = LaneChangeState.off
            self.unsafe_cancel_timer = max(self.unsafe_cancel_timer, 0.5)

            #  차선 변경이 최종 완료된 직후 일반 주행 상태에 4초의 강제 쿨다운 락 가동
            self.lane_change_cooldown_timer = 1.5


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
