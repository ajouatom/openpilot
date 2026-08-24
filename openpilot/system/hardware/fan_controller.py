#!/usr/bin/env python3
import numpy as np

from openpilot.common.pid import PIDController

C3_FAN_DEVICE_TYPES = frozenset(("tici", "tizi"))


class FanController:
  def __init__(self, rate: int, device_type: str) -> None:
    self.last_ignition = False
    self.c3_fan_profile = device_type in C3_FAN_DEVICE_TYPES
    self.target_temp = 75 if self.c3_fan_profile else 70
    # C3/C3X use the proven temperature feed-forward from c3-wip. Keep the
    # current C4/mici controller unchanged.
    self.controller = PIDController(k_p=0, k_i=4e-3, k_f=1 if self.c3_fan_profile else 0, rate=rate)

  def update(self, cur_temp: float, ignition: bool) -> int:
    self.controller.pos_limit = 100 if ignition else 30
    self.controller.neg_limit = 30 if ignition else 0

    if ignition != self.last_ignition:
      self.controller.reset()
    self.last_ignition = ignition

    return int(self.controller.update(
                 error=(cur_temp - self.target_temp),
                 feedforward=np.interp(cur_temp, [60.0, 100.0], [0, 100])
              ))
