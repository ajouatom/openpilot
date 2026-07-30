"""Lead-role selection for the production dPath radar mode."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  DIRECTIONAL_MIN_CONSISTENCY,
  DIRECTIONAL_MIN_INWARD_DISPLACEMENT_M,
  DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO,
  DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS,
  DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS,
  FRONT_CUT_IN_MIN_DREL_M,
  FULL_PREDICTED_PATH_OVERLAP_SUPPORT_S,
  POSITION_ONLY_MAX_ABS_VLEAD_MPS,
)


CUTIN_MAX_DREL_M = 80.0
PRIMARY_DUPLICATE_MAX_DREL_DELTA_M = 3.5
PRIMARY_DUPLICATE_MAX_YREL_DELTA_M = 1.8
PRIMARY_ROW_MAX_DREL_DELTA_M = 8.0
CUTIN_PRIMARY_FUTURE_MARGIN_M = 2.0
FRONT_CUT_IN_MIN_DPATH_RATE_MPS = 0.75
FRONT_NEAR_PATH_MAX_DREL_M = 10.0
FRONT_NEAR_PATH_MIN_SHORT_INWARD_MPS = 0.50
FRONT_NEAR_PATH_MIN_LONG_INWARD_MPS = 0.20
FRONT_NEAR_PATH_MIN_REPORTED_INWARD_MPS = 0.15
LEAD_TWO_POSITION_HOLD_S = 0.75
LEAD_TWO_LONGITUDINAL_JUMP_M = 2.25
LEAD_TWO_LATERAL_JUMP_M = 1.25


@dataclass(frozen=True)
class DPathLeadSelection:
  cutins: tuple[dict[str, Any], ...]
  lead_two: dict[str, Any] | None


@dataclass(frozen=True)
class DPathLeadCandidate:
  lead: dict[str, Any]
  source: str
  track_id: int
  continuity_id: int
  retainable: bool
  confirmed_cutin: bool

  @property
  def identity(self) -> tuple[str, int, int]:
    return self.source, self.track_id, self.continuity_id


def front_cutin_motion_supported(
  source: str,
  d_path_rate_long: float,
  *,
  d_rel: float = math.inf,
  d_path: float = 0.0,
  d_path_rate_short: float = 0.0,
  reported_normal_speed: float = 0.0,
  current_path_occupancy: bool = False,
  predicted_path_overlap_s: float = 0.0,
  directional_inward_displacement_m: float = 0.0,
  directional_consistency: float = 0.0,
  directional_inward_sample_ratio: float = 0.0,
  tracked_close_entry: bool = False,
  minimum_directional_consistency: float = DIRECTIONAL_MIN_CONSISTENCY,
) -> bool:
  """Require strong motion or sustained direction-supported overlap from front."""
  if source != "frontRadar":
    return True
  if tracked_close_entry:
    return True
  if abs(float(d_path_rate_long)) >= FRONT_CUT_IN_MIN_DPATH_RATE_MPS:
    return True

  side = (
    math.copysign(1.0, float(d_path))
    if abs(float(d_path)) > 1e-6
    else 0.0
  )
  directional_future_overlap = (
    float(d_rel) >= FRONT_CUT_IN_MIN_DREL_M
    and float(predicted_path_overlap_s)
    >= FULL_PREDICTED_PATH_OVERLAP_SUPPORT_S
    and float(directional_inward_displacement_m)
    >= DIRECTIONAL_MIN_INWARD_DISPLACEMENT_M
    and float(directional_consistency)
    >= float(minimum_directional_consistency)
    and float(directional_inward_sample_ratio)
    >= DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO
    and -side * float(d_path_rate_short)
    >= DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS
    and -side * float(d_path_rate_long)
    >= DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS
  )
  return (
    directional_future_overlap
    or (
      current_path_occupancy
      and FRONT_CUT_IN_MIN_DREL_M <= float(d_rel)
      <= FRONT_NEAR_PATH_MAX_DREL_M
      and -side * float(d_path_rate_short)
      >= FRONT_NEAR_PATH_MIN_SHORT_INWARD_MPS
      and -side * float(d_path_rate_long)
      >= FRONT_NEAR_PATH_MIN_LONG_INWARD_MPS
      and -side * float(reported_normal_speed)
      >= FRONT_NEAR_PATH_MIN_REPORTED_INWARD_MPS
    )
  )


class DPathLeadTwoTracker:
  """Select moving in-path/CUT-IN leadTwo and retain it through a stop."""

  def __init__(self) -> None:
    self.active_identity: tuple[str, int, int] | None = None
    self._last_lead: dict[str, Any] | None = None
    self._last_time_s: float | None = None

  def reset(self) -> None:
    self.active_identity = None
    self._last_lead = None
    self._last_time_s = None

  def _position_continuous(
    self,
    time_s: float,
    candidate: DPathLeadCandidate,
  ) -> bool:
    if self._last_lead is None or self._last_time_s is None:
      return True
    dt = time_s - self._last_time_s
    if dt < 0.0 or dt > LEAD_TWO_POSITION_HOLD_S:
      return False
    predicted_d_rel = (
      float(self._last_lead.get("dRel", 0.0))
      + float(self._last_lead.get("vRel", 0.0)) * dt
    )
    predicted_y_rel = (
      float(self._last_lead.get("yRel", 0.0))
      + float(self._last_lead.get("vLat", 0.0)) * dt
    )
    return (
      abs(float(candidate.lead.get("dRel", 0.0)) - predicted_d_rel)
      <= LEAD_TWO_LONGITUDINAL_JUMP_M
      and abs(float(candidate.lead.get("yRel", 0.0)) - predicted_y_rel)
      <= LEAD_TWO_LATERAL_JUMP_M
    )

  def update(
    self,
    time_s: float,
    primary: dict[str, Any] | None,
    candidates: Iterable[DPathLeadCandidate],
    v_ego: float,
  ) -> DPathLeadSelection:
    candidate_values = tuple(candidates)
    active_candidates = tuple(
      candidate
      for candidate in candidate_values
      if (
        candidate.identity == self.active_identity
        and candidate.retainable
        and self._position_continuous(time_s, candidate)
      )
    )
    eligible = tuple(
      candidate
      for candidate in candidate_values
      if (
        candidate.confirmed_cutin
        or candidate in active_candidates
      )
    )
    selection = select_dpath_lead_two(
      primary,
      (candidate.lead for candidate in eligible),
      v_ego,
      allow_stopped_track_ids=frozenset(
        candidate.track_id for candidate in active_candidates
      ),
    )
    selected = next(
      (
        candidate
        for candidate in eligible
        if candidate.lead is selection.lead_two
      ),
      None,
    )
    confirmed_selection = select_dpath_lead_two(
      primary,
      (
        candidate.lead
        for candidate in eligible
        if candidate.confirmed_cutin
      ),
      v_ego,
      allow_stopped_track_ids=frozenset(
        candidate.track_id
        for candidate in active_candidates
        if candidate.confirmed_cutin
      ),
    )
    selection = DPathLeadSelection(
      cutins=confirmed_selection.cutins,
      lead_two=selection.lead_two,
    )
    if selected is not None:
      self.active_identity = selected.identity
      self._last_lead = dict(selected.lead)
      self._last_time_s = float(time_s)
    elif self.active_identity is not None:
      active_source, active_track_id, _ = self.active_identity
      active_track_candidates = tuple(
        candidate
        for candidate in candidate_values
        if (
          candidate.source == active_source
          and candidate.track_id == active_track_id
        )
      )
      if active_track_candidates:
        # A present same-ID object that is no longer physically continuous,
        # is moving away, or is no longer control-eligible ends the hold.
        self.reset()
    return selection


def dpath_control_max_d_rel(_v_ego: float) -> float:
  """Use the configured fixed forward limit for every leadTwo candidate."""
  return CUTIN_MAX_DREL_M


def lead_duplicates_primary(
  lead: dict[str, Any],
  primary: dict[str, Any] | None,
) -> bool:
  if primary is None or not primary.get("status"):
    return False
  lead_track_id = int(lead.get("radarTrackId", -1))
  primary_track_id = int(primary.get("radarTrackId", -1))
  if (
    lead_track_id >= 0
    and primary_track_id >= 0
    and lead_track_id == primary_track_id
  ):
    return True
  return (
    abs(float(lead.get("dRel", 0.0)) - float(primary.get("dRel", 0.0)))
    < PRIMARY_DUPLICATE_MAX_DREL_DELTA_M
    and abs(
      float(lead.get("yRel", 0.0)) - float(primary.get("yRel", 0.0))
    )
    < PRIMARY_DUPLICATE_MAX_YREL_DELTA_M
  )


def cutin_can_compete_with_primary(
  lead: dict[str, Any],
  primary: dict[str, Any] | None,
  *,
  projected_path_entry: bool,
  entry_horizon_s: float | None = None,
) -> bool:
  """Require a CUT-IN to enter the path ahead of the predicted leadOne."""
  if primary is None or not primary.get("status"):
    return True
  lead_d_rel = float(lead.get("dRel", math.inf))
  primary_d_rel = float(primary.get("dRel", math.inf))
  if not math.isfinite(lead_d_rel) or not math.isfinite(primary_d_rel):
    return False
  if entry_horizon_s is not None and float(entry_horizon_s) > 0.0:
    horizon_s = float(entry_horizon_s)
    lead_future_d_rel = (
      lead_d_rel + float(lead.get("vRel", 0.0)) * horizon_s
    )
    primary_future_d_rel = (
      primary_d_rel + float(primary.get("vRel", 0.0)) * horizon_s
    )
    if (
      not math.isfinite(lead_future_d_rel)
      or not math.isfinite(primary_future_d_rel)
    ):
      return False
    return (
      lead_future_d_rel + CUTIN_PRIMARY_FUTURE_MARGIN_M
      < primary_future_d_rel
    )
  return (
    abs(lead_d_rel - primary_d_rel) > PRIMARY_ROW_MAX_DREL_DELTA_M
    or projected_path_entry
  )


def select_dpath_lead_two(
  primary: dict[str, Any] | None,
  candidates: Iterable[dict[str, Any]],
  v_ego: float,
  *,
  allow_stopped_track_ids: frozenset[int] = frozenset(),
) -> DPathLeadSelection:
  """Choose the closest eligible independent leadTwo after leadOne is known."""
  maximum_d_rel = dpath_control_max_d_rel(v_ego)
  primary_d_rel = math.inf
  if primary is not None and primary.get("status"):
    value = float(primary.get("dRel", math.inf))
    if math.isfinite(value):
      primary_d_rel = value

  cutins = tuple(sorted(
    (
      lead for lead in candidates
      if (
        lead.get("status")
        and lead.get("radar")
        and 0.8 < float(lead.get("dRel", 0.0)) <= maximum_d_rel
        and float(lead.get("dRel", 0.0)) < primary_d_rel
        and (
          float(lead.get("vLead", 0.0))
          > POSITION_ONLY_MAX_ABS_VLEAD_MPS
          or int(lead.get("radarTrackId", -1)) in allow_stopped_track_ids
        )
        and not lead_duplicates_primary(lead, primary)
      )
    ),
    key=lambda lead: float(lead["dRel"]),
  ))
  return DPathLeadSelection(
    cutins=cutins,
    lead_two=cutins[0] if cutins else None,
  )
