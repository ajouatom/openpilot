"""Turn-signal lever rules for DesireHelper (Hyundai CAN-FD: carState.blinkerLever, *BlinkerStalkCount).

Both rules are opt-in and live here so the lane-change FSM only asks two questions:
- lever_maneuver(): BlinkerLatchedTurn - the lever decides turn / lane change.
- LeverRequestWait: LaneChangeLeverWait - a one-touch press asks for one lane change for a fixed number of flashes.
- LaneCrossing: when the car centre crossed the line, so a press after it can ask for the next lane change.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_lib.constants import BLINKER_NONE, LaneChangeState

LEVER_RELEASED, LEVER_ONE_TOUCH, LEVER_LATCHED = 0, 1, 2

TURN_SIGNAL_FLASH_S = 0.8   # one lamp on/off cycle (2026-10-04 lever test, BLINKERS LAMP_ALT edges 0.79-0.81 s)
LEVER_REQUEST_FLASHES = 5
LEVER_REQUEST_WAIT_S = LEVER_REQUEST_FLASHES * TURN_SIGNAL_FLASH_S


def lever_maneuver(lever: int, v_ego: float, turn_speed_max: float):
  """(maneuver, undecided) chosen by the driver's lever, or (None, False) when the lever does not decide.

  turn_speed_max (m/s, 0 = off). At or below it a latched lever is a turn and anything else a lane change; a lever
  still in the one-touch detent is undecided, because a latch passes through it for ~0.1 s. Above it every lever
  input is a lane change.
  """
  if turn_speed_max <= 0:
    return None, False
  if v_ego > turn_speed_max:
    return "lane_change", False
  if lever == LEVER_LATCHED:
    return "turn", False
  return "lane_change", lever == LEVER_ONE_TOUCH


class LeverRequestWait:
  """A one-touch lever press asks for one lane change for LEVER_REQUEST_FLASHES flashes from the press.

  While that change waits in preLaneChange (blocked by whatever the FSM checks: BSD, side objects, lines) the lamp
  is held on the requested side, so a car set to fewer one-touch flashes still flashes that long. If the change has
  not started LEVER_REQUEST_WAIT_S (plus LaneChangeDelay, which the FSM waits out first) after the press, the
  request is dropped and the driver blinker is ignored until
  the lamp goes dark, so neither the lamp's last flashes nor a later release can start it. A latched lever is the
  driver holding the request himself: no timeout.
  """

  def __init__(self):
    self.side = BLINKER_NONE
    self.timer = 0.0
    self.gave_up = False
    self.hold_side = BLINKER_NONE

  def update(self, enabled: bool, stalk_press: int, lever: int, lamp_side: int, lane_change_state,
             start_delay: float = 0.0) -> bool:
    """Before the FSM (lane_change_state is last frame's). True while the driver blinker must be ignored."""
    if not enabled:
      self.__init__()
      return False
    if stalk_press != BLINKER_NONE:
      self.side, self.timer, self.gave_up = stalk_press, 0.0, False
    elif self.side != BLINKER_NONE:
      self.timer += DT_MDL

    if lever == LEVER_LATCHED or lane_change_state == LaneChangeState.laneChangeStarting:
      self.side = BLINKER_NONE
    elif self.side != BLINKER_NONE and self.timer >= LEVER_REQUEST_WAIT_S + start_delay:
      self.side, self.gave_up = BLINKER_NONE, True

    if self.gave_up and lamp_side == BLINKER_NONE:
      self.gave_up = False
    return self.gave_up

  def update_hold(self, lane_change_state, blinker_state: int) -> int:
    """After the FSM: the side whose lamp to hold (BLINKER_NONE when not waiting for the requested change)."""
    waiting = lane_change_state == LaneChangeState.preLaneChange and blinker_state == self.side
    self.hold_side = self.side if waiting else BLINKER_NONE
    return self.hold_side


class LaneCrossing:
  """Seconds since the car centre crossed the line toward the change side (-1 before), from the model lane lines.

  The change-side line comes within NEAR_M of the centre, then the model relabels it and it jumps JUMP_M further
  out; the relabel is spread over 2-3 frames, so compare with the closest approach rather than the previous frame.
  Same rule as the turn-signal release in hyundaicanfd._blink_hold_side. Without lane lines it never fires.
  """
  NEAR_M = 1.0
  JUMP_M = 1.5

  def __init__(self):
    self.y_min = None
    self.since = -1.0

  def update(self, model, left: bool) -> float:
    if self.since >= 0.0:
      self.since += DT_MDL
      return self.since
    lines = getattr(model, "laneLines", None)
    if lines is None or len(lines) < 3 or not len(lines[1].y) or not len(lines[2].y):
      return self.since
    y = abs(float(lines[1 if left else 2].y[0]))
    self.y_min = y if self.y_min is None else min(self.y_min, y)
    if self.y_min < self.NEAR_M and y - self.y_min > self.JUMP_M:
      self.since = 0.0
    return self.since
