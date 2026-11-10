"""Completion of an executing navigation turn, independent of model yaw decay."""

import math

from openpilot.common.realtime import DT_MDL
from .constants import BLINKER_LEFT, BLINKER_RIGHT, LANE_CHANGE_TIME_MAX


class NavigationTurnCompletion:
  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.peak = 0.0
    self.elapsed = 0.0
    self.returned = 0.0
    self.finished = False

  def update(self, direction: int, angle: float, moving: bool) -> bool:
    if direction not in (BLINKER_LEFT, BLINKER_RIGHT) or not math.isfinite(angle):
      self.reset()
      self.finished = True
      return True
    if direction != self.direction:
      self.reset()
      self.direction = direction
    if self.finished:
      return True
    if not moving:
      return False
    self.elapsed += DT_MDL
    signed_angle = angle if direction == BLINKER_LEFT else -angle
    self.peak = max(self.peak, signed_angle)
    # A peak in predicted yaw occurs in the middle of a turn. Require measured
    # wheel return after real steering, not simply decreasing predicted yaw.
    if self.peak > 80.0 and abs(angle) <= 15.0:
      self.returned += DT_MDL
    else:
      self.returned = 0.0
    # Keep a stale, unchanged navigation cue from requesting an unlimited turn.
    self.finished = self.returned >= 0.2 or self.elapsed >= LANE_CHANGE_TIME_MAX
    return self.finished
