from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper


def make_car_state(left_blinker=False):
  return SimpleNamespace(
    leftBlinker=left_blinker,
    rightBlinker=False,
    vEgo=20.0,
    aEgo=0.0,
    trailerConnected=False,
    steeringTorque=0.0,
    steeringPressed=False,
  )


class TestDesireHelperDriverIntent:
  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    self.helper._make_model_turn_speed = lambda model: None
    self.helper._process_sides = lambda car, model, radar: None
    self.helper._check_desire_state = lambda model, car, maneuver: None
    self.helper.laneChangeNeedTorque = 0
    self.helper.laneChangeBsd = 0
    self.helper.laneLineCheck = 0

    side = self.helper.left
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

    self.carrot_man = SimpleNamespace(
      atcType="fork left",
      carrotCmdIndex=0,
      carrotCmd="",
      carrotArg="",
    )

  def update(self, left_blinker=False, lateral_active=True):
    self.helper.update(
      make_car_state(left_blinker),
      SimpleNamespace(),
      lateral_active,
      0.1,
      self.carrot_man,
      SimpleNamespace(),
    )

  def test_driver_blinker_rearms_lane_change_while_atc_desire_is_already_active(self):
    # The first ATC frame is intentionally ignored when its type changes.
    self.update()

    # ATC keeps the combined desire high while lateral control leaves the FSM off.
    self.update(lateral_active=False)
    assert self.helper.prev_desire_enabled
    assert self.helper.lane_change_state == log.LaneChangeState.off

    # A new physical blinker request must be treated as fresh driver intent even
    # though the combined ATC/driver desire has no rising edge.
    self.update(left_blinker=True)
    assert self.helper.lane_change_state == log.LaneChangeState.preLaneChange

    self.update(left_blinker=True)
    assert self.helper.lane_change_state == log.LaneChangeState.laneChangeStarting

  def test_atc_desire_alone_does_not_rearm_lane_change(self):
    self.update()
    self.update(lateral_active=False)

    self.update()

    assert self.helper.lane_change_state == log.LaneChangeState.off
