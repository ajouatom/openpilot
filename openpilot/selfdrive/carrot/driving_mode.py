"""Driving character and persistent traffic-flow evidence, independent of lead selection."""

import math
from enum import Enum

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL


class DrivingMode(Enum):
  Eco = 1
  Safe = 2
  Normal = 3
  High = 4

  def __str__(self):
    return self.name


def get_mode_lead_response(requested: int, mode: DrivingMode) -> int:
  """Resolve the gap override first; modes may soften, never increase, that choice."""
  ceiling = {DrivingMode.Eco: 2, DrivingMode.Safe: 3}.get(mode, 5)
  return max(0, min(int(requested), ceiling))


class DrivingModeDetector:
  """Enter Safe promptly for stopping; leave only after sustained flow recovery.

  This is a comfort-mode selector, not an obstacle detector or brake trigger.
  Short launches, cut-ins and missing samples cannot clear the queue history.
  """

  STOP_ENTRY_TIME = 0.30
  SLOW_ENTRY_TIME = 8.0
  RECOVERY_TIME = 6.0
  CLEAR_ROAD_TIME = 4.0

  def __init__(self):
    self.congested = False
    self.stop_time = 0.0
    self.slow_time = 0.0
    self.recovery_time = 0.0
    self.clear_time = 0.0
    self.lead_key = None

  def _reset_evidence(self):
    self.stop_time = self.slow_time = self.recovery_time = self.clear_time = 0.0

  def update_data(self, carstate, lead, *, valid=True, dt=DT_MDL):
    if not valid or not math.isfinite(dt) or not 0 < dt <= 0.2 or not math.isfinite(carstate.vEgo):
      self._reset_evidence()
      self.lead_key = None
      return

    ego = max(0.0, carstate.vEgo)
    if not lead.status:
      self.stop_time = self.slow_time = self.recovery_time = 0.0
      self.lead_key = None
      # A disappeared stopped lead is not proof of an open road.
      self.clear_time = self.clear_time + dt if ego >= 15 * CV.KPH_TO_MS else 0.0
      if self.clear_time >= self.CLEAR_ROAD_TIME:
        self.congested = False
      return

    values = (lead.dRel, lead.vLead, lead.vRel, lead.aLeadK)
    if not all(map(math.isfinite, values)) or lead.dRel <= 0:
      self._reset_evidence()
      self.lead_key = None
      return

    self.clear_time = 0.0
    speed = max(0.0, lead.vLead)
    key = (lead.radar, lead.radarTrackId)
    if key != self.lead_key:
      self.recovery_time = 0.0
    self.lead_key = key

    # Approximate approach envelope only for choosing a comfort mode. Actual
    # braking continues to use the planner's physical obstacles and limits.
    approach_distance = min(200.0, max(12.0, ego * ego / (2 * 2.4) + 2 * ego))
    stopping = speed <= 5 * CV.KPH_TO_MS and lead.dRel <= approach_distance
    following = lead.dRel <= min(80.0, max(30.0, 12.0 + 3 * ego))
    slow = following and ego <= 35 * CV.KPH_TO_MS and speed <= 30 * CV.KPH_TO_MS
    self.stop_time = min(self.STOP_ENTRY_TIME, self.stop_time + dt) if stopping else 0.0
    self.slow_time = min(self.SLOW_ENTRY_TIME, self.slow_time + dt) if slow else 0.0

    # Acceleration alone is deliberately not an exit: queues contain strong,
    # short launches. Require sustained speed or a genuinely opening gap.
    flowing = ego >= 35 * CV.KPH_TO_MS and speed >= 35 * CV.KPH_TO_MS
    opening = (speed >= 15 * CV.KPH_TO_MS and lead.vRel >= 1.0
               and lead.dRel >= 8.0 + 1.8 * ego)
    recovering = not stopping and lead.aLeadK >= -0.2 and (flowing or opening)
    self.recovery_time = min(self.RECOVERY_TIME, self.recovery_time + dt) if recovering else 0.0

    if self.recovery_time >= self.RECOVERY_TIME:
      self.congested = False
      self.stop_time = self.slow_time = 0.0
    elif self.stop_time >= self.STOP_ENTRY_TIME or self.slow_time >= self.SLOW_ENTRY_TIME:
      self.congested = True

  def get_mode(self, auto_mode: int):
    cruise_mode = DrivingMode.Eco if int(auto_mode) == 2 else DrivingMode.Normal
    return DrivingMode.Safe if self.congested else cruise_mode
