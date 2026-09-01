"""Low-cost longitudinal refresh for an already validated radar lead.

The full dPath radard remains the sole owner of lead selection. This module
only replaces longitudinal kinematics when the same selected radar track is
present in a newer liveTracks frame. It never promotes a new track.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from openpilot.selfdrive.carrot.radar_motion import (
  CORNER_RADAR_MEASUREMENT_DELAY_S,
  LEAD_ACCEL_FILTER_ALPHA,
  LEAD_ACCEL_TAU_S,
)


FAST_RADAR_MAX_SELECTION_AGE_S = 0.15
FAST_RADAR_MIN_VALIDATION_FRAMES = 2
FAST_RADAR_MAX_VREL_DELTA_MPS = 5.0
FAST_RADAR_BASE_DREL_DELTA_M = 1.5
FAST_RADAR_DREL_TIME_MARGIN_S = 0.20

LEAD_ONE_MASK = 1
LEAD_TWO_MASK = 2


@dataclass(frozen=True)
class FastRadarResult:
  radar_state: Any
  lead_mask: int = 0
  lead_one_track_id: int = -1
  selection_age_s: float = math.inf
  lead_one_reason: str = "inactive"


@dataclass
class _Selection:
  signature: tuple[bool, bool, int] = (False, False, -1)
  consecutive_frames: int = 0


class RadarStateOverride:
  """SubMaster view that substitutes only radarState for one planner pass."""

  def __init__(self, sm: Any, radar_state: Any) -> None:
    self._sm = sm
    self._radar_state = radar_state

  def __getitem__(self, service: str) -> Any:
    if service == "radarState":
      return self._radar_state
    return self._sm[service]

  def __getattr__(self, name: str) -> Any:
    return getattr(self._sm, name)


def _copy_builder(value: Any) -> Any:
  if hasattr(value, "as_builder"):
    return value.as_builder()
  return value.as_reader().as_builder()


def _source_name(point: Any) -> str:
  return str(getattr(point, "radarSource", "")).rsplit(".", 1)[-1]


def _finite(value: Any) -> bool:
  try:
    return math.isfinite(float(value))
  except (TypeError, ValueError):
    return False


class FastRadarOverlay:
  """Refresh selected lead kinematics without repeating dPath association."""

  def __init__(
    self,
    front_radar_delay_s: float,
  ) -> None:
    self.front_radar_delay_s = max(0.0, float(front_radar_delay_s))
    self._selections = {
      "leadOne": _Selection(),
      "leadTwo": _Selection(),
    }
    self._last_radar_state_mono_ns = 0
    self._a_lead_tau: dict[int, float] = {}

  @staticmethod
  def _signature(lead: Any) -> tuple[bool, bool, int]:
    return (
      bool(getattr(lead, "status", False)),
      bool(getattr(lead, "radar", False)),
      int(getattr(lead, "radarTrackId", -1)),
    )

  def observe_radar_state(
    self,
    radar_state: Any,
    radar_state_mono_ns: int,
    valid: bool,
  ) -> None:
    """Count distinct full-radard confirmations for each selected identity."""
    mono_ns = int(radar_state_mono_ns)
    if mono_ns <= self._last_radar_state_mono_ns:
      return
    self._last_radar_state_mono_ns = mono_ns

    active_track_ids = set()
    for role, selection in self._selections.items():
      signature = self._signature(getattr(radar_state, role)) if valid else (False, False, -1)
      if signature == selection.signature:
        selection.consecutive_frames += 1
      else:
        selection.signature = signature
        selection.consecutive_frames = 1
      if signature[0] and signature[1] and signature[2] >= 0:
        active_track_ids.add(signature[2])

    self._a_lead_tau = {
      track_id: tau
      for track_id, tau in self._a_lead_tau.items()
      if track_id in active_track_ids
    }

  def _selection_ready(self, role: str, lead: Any) -> tuple[bool, str]:
    status, radar, track_id = self._signature(lead)
    if not status or not radar or track_id < 0:
      return False, "notRadarLead"
    selection = self._selections[role]
    if selection.signature != (status, radar, track_id):
      return False, "selectionPending"
    if selection.consecutive_frames < FAST_RADAR_MIN_VALIDATION_FRAMES:
      return False, "selectionUnstable"
    return True, "ready"

  def lead_one_ready(self, radar_state: Any) -> bool:
    """Whether full radard has stably selected a physical primary lead."""
    ready, _ = self._selection_ready("leadOne", radar_state.leadOne)
    return ready

  def _measurement_delay_s(self, point: Any) -> float:
    return (
      CORNER_RADAR_MEASUREMENT_DELAY_S
      if _source_name(point).startswith("corner")
      else self.front_radar_delay_s
    )

  def _update_a_lead_tau(self, lead: Any, point: Any, track_id: int) -> float:
    initial_tau = float(getattr(lead, "aLeadTau", LEAD_ACCEL_TAU_S))
    if not math.isfinite(initial_tau) or initial_tau < 0.0:
      initial_tau = LEAD_ACCEL_TAU_S
    tau = self._a_lead_tau.get(track_id, initial_tau)
    a_lead = float(point.aLead)
    j_lead = float(point.jLead)
    if (
      abs(a_lead) < 0.5
      and abs(j_lead) < 0.5
    ):
      tau = LEAD_ACCEL_TAU_S
    else:
      tau *= 1.0 - LEAD_ACCEL_FILTER_ALPHA
    self._a_lead_tau[track_id] = max(0.0, float(tau))
    return self._a_lead_tau[track_id]

  def _overlay_lead(
    self,
    role: str,
    lead: Any,
    point: Any | None,
    v_ego: float,
    selection_age_s: float,
  ) -> tuple[bool, str]:
    ready, reason = self._selection_ready(role, lead)
    if not ready:
      return False, reason
    if point is None:
      return False, "trackMissing"
    if not bool(getattr(point, "measured", False)):
      return False, "trackUnmeasured"

    required_values = (
      point.dRel,
      point.vRel,
      point.aLead,
      point.jLead,
      lead.dRel,
      lead.vRel,
    )
    if not all(_finite(value) for value in required_values):
      return False, "nonFinite"

    delay_s = self._measurement_delay_s(point)
    d_rel = float(point.dRel) + float(point.vRel) * delay_s
    if d_rel <= 0.2:
      return False, "invalidDistance"

    predicted_validated_d_rel = float(lead.dRel) + float(lead.vRel) * max(0.0, selection_age_s)
    d_rel_gate = (
      FAST_RADAR_BASE_DREL_DELTA_M
      + max(abs(float(point.vRel)), abs(float(lead.vRel))) * FAST_RADAR_DREL_TIME_MARGIN_S
    )
    if abs(d_rel - predicted_validated_d_rel) > d_rel_gate:
      return False, "distanceDiscontinuity"
    if abs(float(point.vRel) - float(lead.vRel)) > FAST_RADAR_MAX_VREL_DELTA_MPS:
      return False, "velocityDiscontinuity"

    track_id = int(lead.radarTrackId)
    lead.dRel = d_rel
    lead.vRel = float(point.vRel)
    if _finite(point.aRel):
      lead.aRel = float(point.aRel)
    lead.vLead = float(v_ego) + float(point.vRel)
    lead.vLeadK = lead.vLead
    lead.aLead = float(point.aLead)
    lead.aLeadK = lead.aLead
    lead.jLead = float(point.jLead)
    lead.aLeadTau = self._update_a_lead_tau(lead, point, track_id)
    return True, "active"

  def build(
    self,
    radar_state: Any,
    radar_data: Any,
    v_ego: float,
    radar_state_mono_ns: int,
    live_tracks_mono_ns: int,
    *,
    radar_state_valid: bool,
    live_tracks_valid: bool,
  ) -> FastRadarResult:
    """Return a copied radarState with safe same-track kinematic overlays."""
    output = _copy_builder(radar_state)
    age_s = (int(live_tracks_mono_ns) - int(radar_state_mono_ns)) * 1e-9
    if not radar_state_valid:
      return FastRadarResult(output, selection_age_s=age_s, lead_one_reason="radarStateInvalid")
    if not live_tracks_valid:
      return FastRadarResult(output, selection_age_s=age_s, lead_one_reason="liveTracksInvalid")
    if not (0.0 <= age_s <= FAST_RADAR_MAX_SELECTION_AGE_S):
      return FastRadarResult(output, selection_age_s=age_s, lead_one_reason="selectionStale")

    points_by_id: dict[int, Any | None] = {}
    for point in radar_data.points:
      track_id = int(point.trackId)
      # Ambiguous IDs must never be used to refresh a selected identity.
      points_by_id[track_id] = point if track_id not in points_by_id else None

    lead_mask = 0
    lead_one_reason = "notRadarLead"
    for role, mask in (("leadOne", LEAD_ONE_MASK), ("leadTwo", LEAD_TWO_MASK)):
      lead = getattr(output, role)
      point = points_by_id.get(int(getattr(lead, "radarTrackId", -1)))
      active, reason = self._overlay_lead(role, lead, point, float(v_ego), age_s)
      if active:
        lead_mask |= mask
      if role == "leadOne":
        lead_one_reason = reason

    return FastRadarResult(
      radar_state=output,
      lead_mask=lead_mask,
      lead_one_track_id=(int(output.leadOne.radarTrackId) if lead_mask & LEAD_ONE_MASK else -1),
      selection_age_s=age_s,
      lead_one_reason=lead_one_reason,
    )
