from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, STALK_QUEUE_GUARD_S
from openpilot.selfdrive.controls.lib.desire_lib.lever import LEVER_REQUEST_WAIT_S
from openpilot.selfdrive.controls.lib.desire_lib.constants import TurnDirection

LCS = log.LaneChangeState
LCD = log.LaneChangeDirection


class TestDesireHelperStalkRequest:
  """Turn-signal lever presses (carState.*BlinkerStalkCount) as explicit next-lane-change requests."""

  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    self.helper._process_sides = lambda car, model, radar: None
    self.helper._check_desire_state = lambda model, car, maneuver: None
    self.helper.laneChangeNeedTorque = 0
    self.helper.laneChangeBsd = 0
    self.helper.laneLineCheck = 0
    for side in (self.helper.left, self.helper.right):
      side.lane_available = True
      side.edge_available = False
      side.dist_to_edge_far = 2.0
      side.lane_change_available_geom = True
      side.lane_change_available = True
      side.side_object_detected = False
      side.bsd_hold_counter = 0
      side.lane_line_info_mod = 0
      side.lane_line_info_edge_detect = False
      side.lane_change_available_released = False
      side.lane_available_trigger = False
      side.lane_appeared = False
      side.lane_exist_count.counter = 10
    self.carrot_man = SimpleNamespace(atcType="", carrotCmdIndex=0, carrotCmd="", carrotArg="")
    self.left_count = 0

  def update(self, left_blinker=True, lane_change_prob=0.5, press=False, counters=True, lever=0, v_kph=90.0, lines=None):
    if press:
      self.left_count = (self.left_count + 1) % 256
    car = SimpleNamespace(canValid=True, leftBlinker=left_blinker, rightBlinker=False, vEgo=v_kph / 3.6, aEgo=0.0,
                          trailerConnected=False, steeringTorque=0.0, steeringPressed=False, blinkerLever=lever)
    if counters:
      car.leftBlinkerStalkCount, car.rightBlinkerStalkCount = self.left_count, 0
    model = SimpleNamespace(laneLines=[SimpleNamespace(y=[y]) for y in (-5.0, *lines, 5.0)]) if lines else SimpleNamespace()
    self.helper.update(car, model, True, lane_change_prob, self.carrot_man, SimpleNamespace())
    return self.helper.lane_change_state

  def start_and_cross(self, counters=True):
    """off -> pre -> starting with the left blinker, then hold starting until the model says done."""
    self.update(left_blinker=False, counters=counters)
    assert self.update(press=counters, counters=counters) == LCS.preLaneChange
    assert self.update(counters=counters) == LCS.laneChangeStarting
    for _ in range(5):
      self.update(counters=counters)

  def finish(self, press_at_s=None, counters=True):
    """laneChangeStarting -> finishing -> end; optionally press the lever press_at_s into finishing."""
    for _ in range(int(0.6 / DT_MDL)):                      # ll_prob decays at 2/s; model prob already low
      if self.update(lane_change_prob=0.0, counters=counters) == LCS.laneChangeFinishing:
        break
    assert self.helper.lane_change_state == LCS.laneChangeFinishing
    t = DT_MDL                                                # the transition frame counts as the first
    while self.helper.lane_change_state == LCS.laneChangeFinishing:
      press = press_at_s is not None and abs(t - press_at_s) < DT_MDL / 2
      self.update(lane_change_prob=0.0, press=press, counters=counters)
      t += DT_MDL
    return self.helper.lane_change_state

  def test_blinker_held_through_the_change_still_needs_torque(self):
    self.start_and_cross()
    assert self.finish() == LCS.preLaneChange
    assert self.helper.next_lane_change
    assert self.update() == LCS.preLaneChange              # no torque: no second lane change

  def test_press_while_crossing_is_ignored(self):
    self.start_and_cross()
    self.update(press=True)                                 # re-lighting the lamp mid-change
    assert self.helper.lane_change_state == LCS.laneChangeStarting
    assert self.finish() == LCS.preLaneChange
    assert self.helper.next_lane_change
    assert self.update() == LCS.preLaneChange

  def test_press_during_finishing_queues_the_next_change_without_torque(self):
    self.start_and_cross()
    assert self.finish(press_at_s=STALK_QUEUE_GUARD_S + 0.2) == LCS.preLaneChange
    assert not self.helper.next_lane_change
    assert self.update() == LCS.laneChangeStarting          # one lane at a time: starts after the first ended

  # left change, 2026-10-05 drive: the left line closes in, the model relabels it, then the car settles in the new lane
  CROSSING = [(-1.3, 1.9), (-0.9, 2.2), (-0.5, 2.7), (-0.34, 2.84), (-0.41, 2.83), (-2.5, 0.47), (-2.94, 0.16)]

  def cross(self, press_after_s=None, press_before=False):
    """Still in laneChangeStarting: the car centre crosses the left line; optionally press before or after it."""
    for i, lines in enumerate(self.CROSSING):
      self.update(lines=lines, press=press_before and i == 1)
    t = DT_MDL
    for _ in range(int(1.2 / DT_MDL)):                      # the model keeps the change probability up after it
      press = press_after_s is not None and abs(t - press_after_s) < DT_MDL / 2
      self.update(lines=(-2.6, 0.6), press=press)
      t += DT_MDL
    assert self.helper.lane_change_state == LCS.laneChangeStarting

  def test_press_after_the_crossing_queues_the_next_change(self):
    self.start_and_cross()
    self.cross(press_after_s=0.1)                          # 2026-10-05 #1: pressed 0.1 s after the relabel
    assert self.helper.queued_lane_change == 1
    assert self.helper.blinker_hold_direction == LCD.left  # keep the lamp lit until the queued change starts
    assert self.finish() == LCS.preLaneChange
    assert not self.helper.next_lane_change
    assert self.update() == LCS.laneChangeStarting

  def test_queued_request_keeps_the_lamp_until_the_next_change_starts(self):
    self.start_and_cross()
    self.cross(press_after_s=0.5)
    side = self.helper.left
    side.bsd_hold_counter, side.lane_change_available = 40, False   # BSD when the next change becomes due
    assert self.finish() == LCS.preLaneChange
    assert self.helper.blinker_hold_direction == LCD.left
    for _ in range(int(2.0 / DT_MDL)):
      self.update()
    assert self.helper.blinker_hold_direction == LCD.left     # still waiting, lamp held
    side.bsd_hold_counter, side.lane_change_available = 0, True
    assert self.update() == LCS.laneChangeStarting
    assert self.helper.queued_lane_change == 0                # the queued change started
    self.update()
    assert self.helper.blinker_hold_direction == LCD.none

  def test_blocked_queued_request_is_dropped_after_five_flashes(self):
    self.start_and_cross()
    self.cross(press_after_s=0.5)
    side = self.helper.left
    side.bsd_hold_counter, side.lane_change_available = 40, False
    self.finish()
    for _ in range(int((LEVER_REQUEST_WAIT_S + 0.2) / DT_MDL)):
      self.update()
    assert self.helper.queued_lane_change == 0
    assert self.helper.blinker_hold_direction == LCD.none

  def test_press_before_the_crossing_is_ignored(self):
    self.start_and_cross()
    self.cross(press_before=True)
    assert self.helper.queued_lane_change == 0
    assert self.finish() == LCS.preLaneChange
    assert self.helper.next_lane_change

  def test_press_late_in_the_change_after_the_crossing_still_queues(self):
    self.start_and_cross()
    self.cross(press_after_s=1.1)
    assert self.helper.queued_lane_change == 1

  def test_press_right_after_the_crossing_is_still_the_same_change(self):
    self.start_and_cross()
    assert self.finish(press_at_s=STALK_QUEUE_GUARD_S / 2) == LCS.preLaneChange
    assert self.helper.next_lane_change

  def test_press_while_waiting_for_torque_releases_the_wait(self):
    self.start_and_cross()
    self.finish()
    assert self.helper.next_lane_change
    self.update(press=True)
    assert not self.helper.next_lane_change
    assert self.update() == LCS.laneChangeStarting

  def test_press_while_the_lamp_still_flashes_starts_a_request_from_off(self):
    self.update(left_blinker=True)                          # lamp already flashing: no lamp rising edge left
    self.helper.lane_change_state = LCS.off
    self.helper.prev_desire_enabled = True
    assert self.update() == LCS.off
    assert self.update(press=True) == LCS.preLaneChange

  def test_cars_without_lever_counters_keep_the_lamp_behaviour(self):
    self.start_and_cross(counters=False)
    assert self.finish(counters=False) == LCS.preLaneChange
    assert self.helper.next_lane_change
    assert self.update(counters=False) == LCS.preLaneChange


class TestBlinkerLatchedTurn:
  """BlinkerLatchedTurn (here 50 km/h): at or below it a latched lever is a turn, a one-touch press a lane change;
  above it every lever input is a lane change."""
  setup_method = TestDesireHelperStalkRequest.setup_method
  update = TestDesireHelperStalkRequest.update

  def lever_on(self, lever, v_kph=45.0, steps=1):
    self.update(left_blinker=False, v_kph=v_kph)
    for i in range(steps):
      state = self.update(press=(i == 0), lever=lever, v_kph=v_kph)
    return state

  def test_off_keeps_the_classifier(self):
    self.lever_on(2, steps=3)
    assert self.helper.maneuver_type == "lane_change"       # 45 km/h, lanes visible: the classifier says lane change

  def test_latched_lever_is_a_turn(self):
    self.helper.blinkerLatchedTurn = 50 / 3.6
    self.lever_on(2, steps=3)
    assert self.helper.maneuver_type == "turn"
    assert self.helper.lane_change_state == LCS.off
    assert self.helper.turn_direction == TurnDirection.turnLeft

  def test_latch_passing_the_one_touch_detent_does_not_start_a_lane_change(self):
    self.helper.blinkerLatchedTurn = 50 / 3.6
    assert self.lever_on(1, steps=3) == LCS.preLaneChange    # still in the detent: wait
    self.update(lever=2, v_kph=45.0)
    assert self.helper.maneuver_type == "turn"
    assert self.helper.lane_change_state == LCS.off

  def test_one_touch_is_a_lane_change_after_the_lever_returns(self):
    self.helper.blinkerLatchedTurn = 50 / 3.6
    assert self.lever_on(1, steps=2) == LCS.preLaneChange
    assert self.update(lever=0, v_kph=45.0) == LCS.laneChangeStarting
    assert self.helper.maneuver_type == "lane_change"

  def test_latched_lever_above_the_set_speed_is_a_lane_change(self):
    # At 80 km/h the classifier calls a navigation turn with a far road edge a turn; the lever setting overrides it.
    self.carrot_man.atcType = "turn left"
    self.helper.left.dist_to_edge_far = 5.0
    self.lever_on(2, v_kph=80.0, steps=3)
    assert self.helper.maneuver_type == "turn"               # off: the classifier's guess
    self.setup_method()
    self.carrot_man.atcType = "turn left"
    self.helper.left.dist_to_edge_far = 5.0
    self.helper.blinkerLatchedTurn = 50 / 3.6
    self.lever_on(2, v_kph=80.0, steps=3)
    assert self.helper.maneuver_type == "lane_change"
    assert self.helper.lane_change_state == LCS.laneChangeStarting

  def test_latched_lever_at_the_set_speed_is_a_turn(self):
    self.helper.blinkerLatchedTurn = 50 / 3.6
    self.lever_on(2, v_kph=50.0, steps=3)
    assert self.helper.maneuver_type == "turn"
    assert self.helper.lever_turn                           # cluster message: the lever made this turn

  def test_lever_turn_flag_only_for_a_lever_made_turn(self):
    # off: a navigation turn guessed by the classifier is a turn, but not the lever's
    self.carrot_man.atcType = "turn left"
    self.helper.left.dist_to_edge_far = 5.0
    self.lever_on(2, v_kph=80.0, steps=3)
    assert self.helper.maneuver_type == "turn" and not self.helper.lever_turn
    # on, one-touch at 45 km/h: a lane change, no message
    self.setup_method()
    self.helper.blinkerLatchedTurn = 50 / 3.6
    self.lever_on(1, steps=2)
    self.update(lever=0, v_kph=45.0)
    assert self.helper.maneuver_type == "lane_change" and not self.helper.lever_turn
    # the lever released: the message goes with the turn
    self.setup_method()
    self.helper.blinkerLatchedTurn = 50 / 3.6
    self.lever_on(2, steps=3)
    assert self.helper.lever_turn
    self.update(left_blinker=False, lever=0, v_kph=45.0)
    assert not self.helper.lever_turn


class TestLaneChangeLeverWait:
  """LaneChangeLeverWait: a blocked one-touch request holds the lamp for 5 flashes from the press, then gives up."""
  update = TestDesireHelperStalkRequest.update

  def setup_method(self):
    TestDesireHelperStalkRequest.setup_method(self)
    self.helper.laneChangeLeverWait = True
    self.block(True)

  def block(self, blocked):
    side = self.helper.left                                 # BSD on the left: the FSM waits for torque
    side.bsd_hold_counter = 40 if blocked else 0
    side.lane_change_available = not blocked

  def press(self, lever=0):
    self.update(left_blinker=False)
    assert self.update(press=True, lever=lever) == LCS.preLaneChange

  def wait(self, seconds, lever=0):
    for _ in range(round(seconds / DT_MDL)):
      self.update(lever=lever)
    return self.helper.lane_change_state

  def test_blocked_request_holds_the_lamp_then_gives_up(self):
    self.press()
    assert self.helper.blinker_hold_direction == LCD.left
    assert self.wait(LEVER_REQUEST_WAIT_S - 0.2) == LCS.preLaneChange
    assert self.helper.blinker_hold_direction == LCD.left
    assert self.wait(0.3) == LCS.off                        # 5 flashes after the press: dropped
    assert self.helper.blinker_hold_direction == LCD.none
    self.block(False)
    assert self.wait(1.0) == LCS.off                        # lamp still flashing: no restart from it
    self.update(left_blinker=False)                         # lamp dark: the next input is a new request
    assert self.update() == LCS.preLaneChange

  def test_clearing_within_the_wait_starts_the_change(self):
    self.press()
    self.wait(2.0)
    self.block(False)
    assert self.update() == LCS.laneChangeStarting
    self.update()
    assert self.helper.blinker_hold_direction == LCD.none  # the change itself holds the lamp from here

  def test_a_new_press_restarts_the_count(self):
    self.press()
    self.wait(3.0)
    self.update(press=True)
    assert self.wait(LEVER_REQUEST_WAIT_S - 0.5) == LCS.preLaneChange
    assert self.wait(0.6) == LCS.off

  def test_latched_lever_keeps_waiting(self):
    self.press(lever=2)
    assert self.wait(LEVER_REQUEST_WAIT_S + 1.0, lever=2) == LCS.preLaneChange
    assert self.helper.blinker_hold_direction == LCD.none

  def test_lane_change_delay_extends_the_wait(self):
    self.helper.laneChangeDelay = 5.0                       # longer than the 5 flashes: must not give up first
    self.block(False)
    self.press()
    assert self.wait(LEVER_REQUEST_WAIT_S + 0.5) == LCS.preLaneChange
    assert self.wait(1.0) == LCS.laneChangeStarting

  def test_off_keeps_waiting_on_the_lamp(self):
    self.helper.laneChangeLeverWait = False
    self.press()
    assert self.wait(LEVER_REQUEST_WAIT_S + 1.0) == LCS.preLaneChange
    assert self.helper.blinker_hold_direction == LCD.none
