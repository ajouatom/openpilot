"""Require range evidence before predicting departure from a stopped lead.

Only planner input copies are conditioned. Raw radar measurements remain
available for establishing movement, and MPC still owns the stop/start output.
"""

from dataclasses import dataclass
import math
from typing import Any


STOPPED_EGO_SPEED = 0.10
STOPPED_LEAD_SPEED = 0.30
STABLE_RANGE_SPAN = 0.10
STABLE_TIME = 0.20
# Three 5 cm range bins: a single quantized range step is not departure.
DEPARTURE_DISTANCE = 0.15
DEPARTURE_TIME = 0.10
MAX_FRAME_GAP = 0.20


@dataclass
class _LeadEvidence:
  track_id: int
  since: float
  min_distance: float
  max_distance: float
  anchor_distance: float
  held: bool = False
  departure_since: float | None = None
  departed: bool = False


class StoppingLeadFilter:
  def __init__(self):
    self._leads: dict[str, _LeadEvidence] = {}
    self._last_time: float | None = None
    self.held_mask = 0

  def _hold_lead(self, role: str, lead: Any, v_ego: float, now: float, fresh: bool) -> bool:
    valid = (
      lead.status and lead.radar and lead.radarTrackId >= 0
      and all(math.isfinite(v) for v in (lead.dRel, lead.vRel, lead.vLead, lead.aLeadK, lead.jLead))
      and lead.dRel > 0.2
    )
    if not valid:
      self._leads.pop(role, None)
      return False

    evidence = self._leads.get(role)
    if evidence is not None and evidence.track_id != lead.radarTrackId:
      self._leads.pop(role)
      evidence = None

    # Keep explicit approaching/decelerating observations in the original path.
    if lead.vLead < -STOPPED_LEAD_SPEED or lead.aLeadK < -0.5:
      self._leads.pop(role, None)
      return False

    quiet = v_ego <= STOPPED_EGO_SPEED and abs(lead.vLead) <= STOPPED_LEAD_SPEED
    if evidence is None:
      if not quiet or not fresh:
        return False
      evidence = _LeadEvidence(int(lead.radarTrackId), now, lead.dRel, lead.dRel, lead.dRel)
      self._leads[role] = evidence

    if evidence.departed:
      return False
    if not fresh:
      return evidence.held

    if not evidence.held:
      evidence.min_distance = min(evidence.min_distance, lead.dRel)
      evidence.max_distance = max(evidence.max_distance, lead.dRel)
      if not quiet or evidence.max_distance - evidence.min_distance > STABLE_RANGE_SPAN + 1e-6:
        self._leads.pop(role)
        return False
      if now - evidence.since < STABLE_TIME - 1e-6:
        return False
      evidence.held = True
      evidence.anchor_distance = lead.dRel

    # A closing gap is still supplied to MPC; never freeze the obstacle range.
    # Re-establish evidence at the closer position instead of treating a rebound
    # from a range drop as departure from the old lead position.
    if lead.dRel < evidence.anchor_distance - STABLE_RANGE_SPAN - 1e-6:
      self._leads.pop(role)
      return False

    opening = (
      lead.dRel - evidence.anchor_distance >= DEPARTURE_DISTANCE - 1e-6
      and lead.vRel > 0.0 and lead.vLead > 0.0
    )
    if opening:
      if evidence.departure_since is None:
        evidence.departure_since = now
      if now - evidence.departure_since >= DEPARTURE_TIME - 1e-6:
        evidence.departed = True
        return False
    else:
      evidence.departure_since = None
    return True

  def update(self, radar_state: Any, *, stopping: bool, v_ego: float, mono_time_ns: int, valid: bool = True) -> Any:
    self.held_mask = 0
    now = mono_time_ns * 1e-9
    if not stopping or not valid or not math.isfinite(v_ego) or mono_time_ns <= 0:
      self._leads.clear()
      self._last_time = None
      return radar_state

    if self._last_time is not None and abs(now - self._last_time) > MAX_FRAME_GAP + 1e-6:
      self._leads.clear()
      self._last_time = None
    # A model-clock fallback can expose an older full-radard observation after
    # a fast update. It must neither confirm departure nor erase a held lead.
    fresh = self._last_time is None or now > self._last_time
    self._last_time = now if fresh else self._last_time

    for role, mask in (("leadOne", 1), ("leadTwo", 2)):
      if self._hold_lead(role, getattr(radar_state, role), abs(v_ego), now, fresh):
        self.held_mask |= mask
    if not self.held_mask:
      return radar_state

    output = radar_state.as_builder() if hasattr(radar_state, "as_builder") else radar_state.as_reader().as_builder()
    for role, mask in (("leadOne", 1), ("leadTwo", 2)):
      if self.held_mask & mask:
        lead = getattr(output, role)
        lead.vLead = lead.vLeadK = 0.0
        lead.aLead = lead.aLeadK = 0.0
        lead.jLead = 0.0
    return output
