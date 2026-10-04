from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, STALK_QUEUE_GUARD_S
from openpilot.selfdrive.controls.lib.desire_lib.constants import TurnDirection

LCS = log.LaneChangeState


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

  def update(self, left_blinker=True, lane_change_prob=0.5, press=False, counters=True, lever=0, v_kph=90.0):
    if press:
      self.left_count = (self.left_count + 1) % 256
    car = SimpleNamespace(canValid=True, leftBlinker=left_blinker, rightBlinker=False, vEgo=v_kph / 3.6, aEgo=0.0,
                          trailerConnected=False, steeringTorque=0.0, steeringPressed=False, blinkerLever=lever)
    if counters:
      car.leftBlinkerStalkCount, car.rightBlinkerStalkCount = self.left_count, 0
    self.helper.update(car, SimpleNamespace(), True, lane_change_prob, self.carrot_man, SimpleNamespace())
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
  """BlinkerLatchedTurn: at or below 50 km/h a latched lever is a turn, a one-touch press a lane change."""
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
    self.helper.blinkerLatchedTurn = True
    self.lever_on(2, steps=3)
    assert self.helper.maneuver_type == "turn"
    assert self.helper.lane_change_state == LCS.off
    assert self.helper.turn_direction == TurnDirection.turnLeft

  def test_latch_passing_the_one_touch_detent_does_not_start_a_lane_change(self):
    self.helper.blinkerLatchedTurn = True
    assert self.lever_on(1, steps=3) == LCS.preLaneChange    # still in the detent: wait
    self.update(lever=2, v_kph=45.0)
    assert self.helper.maneuver_type == "turn"
    assert self.helper.lane_change_state == LCS.off

  def test_one_touch_is_a_lane_change_after_the_lever_returns(self):
    self.helper.blinkerLatchedTurn = True
    assert self.lever_on(1, steps=2) == LCS.preLaneChange
    assert self.update(lever=0, v_kph=45.0) == LCS.laneChangeStarting
    assert self.helper.maneuver_type == "lane_change"

  def test_latched_lever_above_50_kph_is_the_classifier(self):
    self.helper.blinkerLatchedTurn = True
    self.lever_on(2, v_kph=60.0, steps=3)
    assert self.helper.maneuver_type == "lane_change"
    assert self.helper.lane_change_state == LCS.laneChangeStarting
