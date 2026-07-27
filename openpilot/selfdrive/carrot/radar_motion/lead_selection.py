"""Lead-role selection for the production dPath radar mode."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  FRONT_CUT_IN_MIN_DREL_M,
  POSITION_ONLY_MAX_ABS_VLEAD_MPS,
)


CUTIN_MAX_DREL_M = 80.0
PRIMARY_DUPLICATE_MAX_DREL_DELTA_M = 3.5
PRIMARY_DUPLICATE_MAX_YREL_DELTA_M = 1.8
PRIMARY_EXIT_MAX_VLEAD_DELTA_MPS = 2.0
PRIMARY_ROW_MAX_DREL_DELTA_M = 8.0
LEAD_ONE_CUT_OUT_THRESHOLD = 0.60
LEAD_ONE_EXIT_RELEASE_CUT_OUT_THRESHOLD = LEAD_ONE_CUT_OUT_THRESHOLD
FRONT_CUT_IN_MIN_DPATH_RATE_MPS = 0.75
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
  current_path_motion: bool = False

  @property
  def identity(self) -> tuple[str, int, int]:
    return self.source, self.track_id, self.continuity_id


@dataclass
class LeadOneExitLatch:
  """Keep an exiting physical identity out until its exit evidence clears."""

  active_identity: tuple[str, int, int] | None = None
  primary_track_id: int | None = None

  def reset(self) -> None:
    self.active_identity = None
    self.primary_track_id = None

  def start(
    self,
    identity: tuple[str, int, int],
    primary: dict[str, Any],
  ) -> None:
    self.active_identity = identity
    track_id = int(primary.get("radarTrackId", -1))
    self.primary_track_id = track_id if track_id >= 0 else None

  def matches_primary(self, primary: dict[str, Any] | None) -> bool:
    return (
      primary is not None
      and bool(primary.get("status"))
      and self.primary_track_id is not None
      and int(primary.get("radarTrackId", -1)) == self.primary_track_id
    )

  def update(
    self,
    predictions: Iterable[Any],
  ) -> tuple[str, int, int] | None:
    if self.active_identity is None:
      return None
    for prediction in predictions:
      identity = (
        str(prediction.source),
        int(prediction.track_id),
        int(prediction.continuity_id),
      )
      if identity != self.active_identity:
        continue
      if (
        float(getattr(prediction, "cut_in_probability", 0.0)) <= 0.0
        and float(getattr(prediction, "cut_out_probability", 0.0))
        >= LEAD_ONE_EXIT_RELEASE_CUT_OUT_THRESHOLD
      ):
        return self.active_identity
      self.reset()
      return None
    self.reset()
    return None


def can_start_current_path_lead_two(
  source: str,
  d_rel: float,
  current_path_occupancy: bool,
  motion_history_ready: bool = True,
) -> bool:
  """Allow close front points only to retain, never to start, leadTwo."""
  return (
    current_path_occupancy
    and motion_history_ready
    and (
      source != "frontRadar"
      or d_rel >= FRONT_CUT_IN_MIN_DREL_M
    )
  )


def front_cutin_motion_supported(
  source: str,
  d_path_rate_long: float,
) -> bool:
  """Require stronger path-relative motion from front-only lateral estimates."""
  return (
    source != "frontRadar"
    or abs(float(d_path_rate_long)) >= FRONT_CUT_IN_MIN_DPATH_RATE_MPS
  )


class DPathLeadTwoTracker:
  """Select moving in-path/CUT-IN leadTwo and retain it through a stop."""

  def __init__(self) -> None:
    self.active_identity: tuple[str, int, int] | None = None
    self._active_allows_farther = False
    self._last_lead: dict[str, Any] | None = None
    self._last_time_s: float | None = None

  def reset(self) -> None:
    self.active_identity = None
    self._active_allows_farther = False
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
        or candidate.current_path_motion
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
      allow_farther_track_ids=frozenset(
        candidate.track_id
        for candidate in eligible
        if (
          candidate.current_path_motion
          or (
            candidate in active_candidates
            and self._active_allows_farther
          )
        )
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
      retained_path_role = (
        selected.identity == self.active_identity
        and self._active_allows_farther
      )
      self.active_identity = selected.identity
      self._active_allows_farther = (
        selected.current_path_motion or retained_path_role
      )
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


def lead_one_matches_motion_track(
  primary: dict[str, Any] | None,
  motion_lead: dict[str, Any],
) -> bool:
  """Check whether primary and motion output describe the same physical car."""
  if (
    primary is None
    or not primary.get("status")
    or not lead_duplicates_primary(motion_lead, primary)
  ):
    return False
  return (
    abs(
      float(motion_lead.get("vLead", 0.0))
      - float(primary.get("vLead", 0.0))
    )
    <= PRIMARY_EXIT_MAX_VLEAD_DELTA_MPS
  )


def lead_one_exits_path(
  primary: dict[str, Any] | None,
  motion_lead: dict[str, Any],
  cut_out_probability: float,
  cut_in_probability: float,
) -> bool:
  """Release leadOne only when the same physical motion track exits the path."""
  return (
    float(cut_in_probability) <= 0.0
    and float(cut_out_probability) >= LEAD_ONE_CUT_OUT_THRESHOLD
    and lead_one_matches_motion_track(primary, motion_lead)
  )


def cutin_can_compete_with_primary(
  lead: dict[str, Any],
  primary: dict[str, Any] | None,
  *,
  projected_path_entry: bool,
) -> bool:
  """Suppress same-row proximity alone until an actual path entry is forecast."""
  if primary is None or not primary.get("status"):
    return True
  lead_d_rel = float(lead.get("dRel", math.inf))
  primary_d_rel = float(primary.get("dRel", math.inf))
  if not math.isfinite(lead_d_rel) or not math.isfinite(primary_d_rel):
    return False
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
  allow_farther_track_ids: frozenset[int] = frozenset(),
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
        and (
          float(lead.get("dRel", 0.0)) < primary_d_rel
          or int(lead.get("radarTrackId", -1)) in allow_farther_track_ids
        )
        and (
          float(lead.get("vLead", 0.0))
          >= POSITION_ONLY_MAX_ABS_VLEAD_MPS
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
