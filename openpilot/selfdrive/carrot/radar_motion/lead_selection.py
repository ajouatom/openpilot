"""Lead-role selection for the production dPath radar mode."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, replace
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
PRIMARY_PROXIMITY_MIN_VLEAD_DELTA_MPS = 2.5
PRIMARY_ROW_MAX_DREL_DELTA_M = 8.0
CUTIN_PRIMARY_FUTURE_MARGIN_M = 2.0
FRONT_CUT_IN_MIN_DPATH_RATE_MPS = 0.75
# Front-radar azimuth is coarse enough that a parallel adjacent target can
# appear to move inward by nearly a metre without actually approaching the
# ego corridor. Do not control on a forecast-only entry until the measured
# target body is within 0.40 m of the path-overlap boundary.
FRONT_PREDICTED_CUTIN_MAX_ABS_DPATH_M = 2.20
CORNER_FAR_CUTIN_MAX_ABS_DPATH_M = 2.20
CORNER_FAR_CUTIN_MIN_LONG_INWARD_MPS = 0.65
CORNER_FAR_CUTIN_MIN_CLOSING_SPEED_MPS = 3.0
CORNER_DISTANT_CURRENT_PATH_MIN_DREL_M = 45.0
CORNER_DISTANT_CURRENT_PATH_MIN_INWARD_DISPLACEMENT_M = 0.35
FRONT_NEAR_PATH_MAX_DREL_M = 10.0
FRONT_NEAR_PATH_MIN_SHORT_INWARD_MPS = 0.50
FRONT_NEAR_PATH_MIN_LONG_INWARD_MPS = 0.20
FRONT_NEAR_PATH_MIN_REPORTED_INWARD_MPS = 0.15
# A close slow target is normally discarded as position-only noise. Permit it
# only when front and corner radar have independently associated the same
# object and its measured front-radar history shows a sustained path entry.
CROSS_SENSOR_CLOSE_CUTIN_MAX_DREL_M = 12.0
CROSS_SENSOR_CLOSE_CUTIN_MIN_OVERLAP_S = 1.0
CROSS_SENSOR_CLOSE_CUTIN_MIN_INWARD_DISPLACEMENT_M = 0.30
CROSS_SENSOR_CLOSE_CUTIN_MIN_DIRECTIONAL_CONSISTENCY = 0.50
CROSS_SENSOR_CLOSE_CUTIN_MIN_INWARD_SAMPLE_RATIO = 0.50
CROSS_SENSOR_CLOSE_CUTIN_MIN_SHORT_INWARD_MPS = 0.25
CROSS_SENSOR_CLOSE_CUTIN_MIN_LONG_INWARD_MPS = 0.20
LEAD_TWO_POSITION_HOLD_S = 0.75
LEAD_TWO_LONGITUDINAL_JUMP_M = 2.25
LEAD_TWO_LATERAL_JUMP_M = 1.25
STATIONARY_SHADOW_CUT_OUT_PROBABILITY = 0.70
STATIONARY_SHADOW_CONFIRMATION_S = 0.25
STATIONARY_SHADOW_SIGNAL_HOLD_S = 0.75
STATIONARY_SHADOW_MIN_PRIMARY_GAP_M = 3.0
STATIONARY_SHADOW_MAX_DREL_M = 80.0
STATIONARY_SHADOW_MAX_DPATH_M = 0.75
STATIONARY_SHADOW_MAX_ABS_VLEAD_MPS = 1.5
STATIONARY_SHADOW_MIN_PRIMARY_VLEAD_MPS = 4.0
STATIONARY_SHADOW_EQUIVALENCE_BRAKE_MPS2 = 2.5
STATIONARY_PRIMARY_HANDOFF_MAX_ABS_VLEAD_MPS = 4.0
STATIONARY_PRIMARY_HANDOFF_MAX_DPATH_M = 0.75
STATIONARY_PRIMARY_HANDOFF_MIN_MODEL_PROBABILITY = 0.40
STATIONARY_PRIMARY_HANDOFF_CONFIRMATION_S = 0.25
STATIONARY_PRIMARY_HANDOFF_SUPPORT_HOLD_S = 1.0
STATIONARY_PRIMARY_HANDOFF_MIN_CLOSER_MARGIN_M = 1.0


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
  confirmed_stationary_shadow: bool = False
  allow_low_speed: bool = False

  @property
  def identity(self) -> tuple[str, int, int]:
    return self.source, self.track_id, self.continuity_id


class DPathStationaryShadowTracker:
  """Confirm a central stopped object revealed behind a cutting-out lead."""

  def __init__(self) -> None:
    self._identity: tuple[str, int, int] | None = None
    self._since_s: float | None = None
    self._last_signal_s: float | None = None
    self._last_candidate: DPathLeadCandidate | None = None
    self._last_time_s: float | None = None

  def reset(self) -> None:
    self._identity = None
    self._since_s = None
    self._last_signal_s = None
    self._last_candidate = None
    self._last_time_s = None

  def _continuous(
    self,
    time_s: float,
    candidate: DPathLeadCandidate,
  ) -> bool:
    if self._last_candidate is None or self._last_time_s is None:
      return True
    dt = time_s - self._last_time_s
    if dt < 0.0 or dt > LEAD_TWO_POSITION_HOLD_S:
      return False
    previous = self._last_candidate.lead
    predicted_d_rel = (
      float(previous.get("dRel", 0.0))
      + float(previous.get("vRel", 0.0)) * dt
    )
    predicted_y_rel = (
      float(previous.get("yRel", 0.0))
      + float(previous.get("vLat", 0.0)) * dt
    )
    return (
      abs(float(candidate.lead.get("dRel", 0.0)) - predicted_d_rel)
      <= LEAD_TWO_LONGITUDINAL_JUMP_M
      and abs(float(candidate.lead.get("yRel", 0.0)) - predicted_y_rel)
      <= LEAD_TWO_LATERAL_JUMP_M
    )

  @staticmethod
  def _stopped_equivalent_distance(lead: dict[str, Any]) -> float:
    speed = max(0.0, float(lead.get("vLead", 0.0)))
    return (
      float(lead.get("dRel", math.inf))
      + speed * speed
      / (2.0 * STATIONARY_SHADOW_EQUIVALENCE_BRAKE_MPS2)
    )

  def update(
    self,
    time_s: float,
    primary: dict[str, Any] | None,
    primary_cut_out_probability: float,
    candidates: Iterable[DPathLeadCandidate],
  ) -> DPathLeadCandidate | None:
    time_s = float(time_s)
    values = tuple(candidates)
    primary_is_moving = (
      primary is not None
      and bool(primary.get("status"))
      and float(primary.get("vLead", 0.0))
      > STATIONARY_SHADOW_MIN_PRIMARY_VLEAD_MPS
    )
    cut_out_signal = (
      primary_is_moving
      and float(primary_cut_out_probability)
      >= STATIONARY_SHADOW_CUT_OUT_PROBABILITY
    )
    if cut_out_signal:
      self._last_signal_s = time_s

    signal_held = (
      self._last_signal_s is not None
      and time_s - self._last_signal_s <= STATIONARY_SHADOW_SIGNAL_HOLD_S
    )
    eligible = tuple(
      candidate for candidate in values
      if (
        abs(float(candidate.lead.get("vLead", 0.0)))
        <= STATIONARY_SHADOW_MAX_ABS_VLEAD_MPS
        and abs(float(candidate.lead.get("dPath", math.inf)))
        <= STATIONARY_SHADOW_MAX_DPATH_M
        and 0.8 < float(candidate.lead.get("dRel", 0.0))
        <= STATIONARY_SHADOW_MAX_DREL_M
      )
    )
    if not eligible or not signal_held:
      self.reset()
      return None

    active = next((
      candidate for candidate in eligible
      if candidate.identity == self._identity
      and self._continuous(time_s, candidate)
    ), None)
    if active is None and cut_out_signal and primary is not None:
      primary_d_rel = float(primary.get("dRel", math.inf))
      primary_obstacle = self._stopped_equivalent_distance(primary)
      active = min((
        candidate for candidate in eligible
        if (
          float(candidate.lead.get("dRel", 0.0))
          >= primary_d_rel + STATIONARY_SHADOW_MIN_PRIMARY_GAP_M
          and self._stopped_equivalent_distance(candidate.lead)
          < primary_obstacle
        )
      ), key=lambda candidate: float(candidate.lead["dRel"]), default=None)

    if active is None:
      self.reset()
      return None
    if active.identity != self._identity:
      self._identity = active.identity
      self._since_s = time_s
    self._last_candidate = active
    self._last_time_s = time_s
    confirmed = (
      self._since_s is not None
      and time_s - self._since_s >= STATIONARY_SHADOW_CONFIRMATION_S
    )
    return replace(active, confirmed_stationary_shadow=confirmed)


class DPathStationaryPrimaryHandoffTracker:
  """Keep a vision-confirmed stopped corner hypothesis in leadTwo."""

  def __init__(self) -> None:
    self._identity: tuple[str, int, int] | None = None
    self._since_s: float | None = None
    self._last_primary_s: float | None = None
    self._last_primary_candidate: DPathLeadCandidate | None = None

  def reset(self) -> None:
    self._identity = None
    self._since_s = None
    self._last_primary_s = None
    self._last_primary_candidate = None

  @staticmethod
  def _eligible(candidate: DPathLeadCandidate) -> bool:
    lead = candidate.lead
    return (
      candidate.source.startswith("corner")
      and bool(lead.get("status"))
      and abs(float(lead.get("vLead", 0.0)))
      <= STATIONARY_PRIMARY_HANDOFF_MAX_ABS_VLEAD_MPS
      and abs(float(lead.get("dPath", math.inf)))
      <= STATIONARY_PRIMARY_HANDOFF_MAX_DPATH_M
      and 0.8 < float(lead.get("dRel", 0.0))
      <= STATIONARY_SHADOW_MAX_DREL_M
    )

  def _continuous(
    self,
    time_s: float,
    candidate: DPathLeadCandidate,
  ) -> bool:
    previous_candidate = self._last_primary_candidate
    previous_time_s = self._last_primary_s
    if previous_candidate is None or previous_time_s is None:
      return True
    dt = float(time_s) - previous_time_s
    if dt < 0.0 or dt > STATIONARY_PRIMARY_HANDOFF_SUPPORT_HOLD_S:
      return False
    previous = previous_candidate.lead
    predicted_d_rel = (
      float(previous.get("dRel", 0.0))
      + float(previous.get("vRel", 0.0)) * dt
    )
    predicted_y_rel = (
      float(previous.get("yRel", 0.0))
      + float(previous.get("vLat", 0.0)) * dt
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
    active_identity: tuple[str, int, int] | None,
  ) -> DPathLeadCandidate | None:
    time_s = float(time_s)
    values = tuple(candidate for candidate in candidates if self._eligible(candidate))
    primary_track_id = (
      int(primary.get("radarTrackId", -1))
      if primary is not None and primary.get("status") and primary.get("radar")
      else -1
    )
    supported = tuple(
      candidate for candidate in values
      if float(candidate.lead.get("modelProb", 0.0))
      >= STATIONARY_PRIMARY_HANDOFF_MIN_MODEL_PROBABILITY
    )
    primary_candidate = next((
      candidate for candidate in supported
      if candidate.track_id == primary_track_id
    ), None)
    if primary_candidate is None:
      primary_candidate = min(
        supported,
        key=lambda candidate: (
          abs(float(candidate.lead.get("dPath", math.inf))),
          -float(candidate.lead.get("modelProb", 0.0)),
          float(candidate.lead.get("dRel", math.inf)),
        ),
        default=None,
      )
    if (
      primary_candidate is not None
    ):
      if (
        primary_candidate.identity != self._identity
        or not self._continuous(time_s, primary_candidate)
      ):
        self._identity = primary_candidate.identity
        self._since_s = time_s
      self._last_primary_candidate = primary_candidate
      self._last_primary_s = time_s

    if self._identity is None:
      return None
    candidate = next((
      value for value in values if value.identity == self._identity
    ), None)
    if candidate is None or candidate.track_id == primary_track_id:
      return None
    if active_identity == self._identity:
      return candidate
    if (
      self._last_primary_s is None
      or time_s - self._last_primary_s
      > STATIONARY_PRIMARY_HANDOFF_SUPPORT_HOLD_S
      or not self._continuous(time_s, candidate)
    ):
      self.reset()
      return None
    if (
      self._since_s is None
      or time_s - self._since_s
      < STATIONARY_PRIMARY_HANDOFF_CONFIRMATION_S
    ):
      return None
    if primary is None or not primary.get("status"):
      return None
    if (
      float(candidate.lead.get("dRel", math.inf))
      + STATIONARY_PRIMARY_HANDOFF_MIN_CLOSER_MARGIN_M
      >= float(primary.get("dRel", math.inf))
    ):
      return None
    return replace(candidate, confirmed_stationary_shadow=True)


def front_cutin_motion_supported(
  source: str,
  d_path_rate_long: float,
  *,
  d_rel: float = math.inf,
  v_rel: float = 0.0,
  d_path: float = 0.0,
  d_path_rate_short: float = 0.0,
  reported_normal_speed: float = 0.0,
  current_path_occupancy: bool = False,
  predicted_path_overlap_s: float = 0.0,
  directional_inward_displacement_m: float = 0.0,
  directional_consistency: float = 0.0,
  directional_inward_sample_ratio: float = 0.0,
  corner_directional_entry: bool = False,
  tracked_close_entry: bool = False,
  cross_sensor_confirmed: bool = False,
  minimum_directional_consistency: float = DIRECTIONAL_MIN_CONSISTENCY,
) -> bool:
  """Require strong motion or sustained direction-supported overlap from front."""
  if source != "frontRadar":
    side = (
      math.copysign(1.0, float(d_path))
      if abs(float(d_path)) > 1e-6
      else 0.0
    )
    if (
      source.startswith("corner")
      and not current_path_occupancy
      and abs(float(d_path)) > CORNER_FAR_CUTIN_MAX_ABS_DPATH_M
    ):
      # Ego closing speed cannot prove a lateral merge: a stationary roadside
      # vehicle closes at nearly ego speed too. Far corner-only targets must
      # show strong measured path-relative inward motion of their own, or have
      # already passed the predictor's strict directional-history entry gate.
      return (
        -side * float(d_path_rate_long)
        >= CORNER_FAR_CUTIN_MIN_LONG_INWARD_MPS
        or (
          bool(corner_directional_entry)
          and float(v_rel) <= -CORNER_FAR_CUTIN_MIN_CLOSING_SPEED_MPS
        )
      )
    if (
      source.startswith("corner")
      and current_path_occupancy
      and float(d_rel) >= CORNER_DISTANT_CURRENT_PATH_MIN_DREL_M
    ):
      return (
        float(directional_inward_displacement_m)
        >= CORNER_DISTANT_CURRENT_PATH_MIN_INWARD_DISPLACEMENT_M
        and float(directional_consistency)
        >= float(minimum_directional_consistency)
        and float(directional_inward_sample_ratio)
        >= DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO
        and -side * float(d_path_rate_long)
        >= DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS
      )
    return True
  side = (
    math.copysign(1.0, float(d_path))
    if abs(float(d_path)) > 1e-6
    else 0.0
  )
  if tracked_close_entry:
    return True
  measured_near_path = (
    abs(float(d_path)) <= FRONT_PREDICTED_CUTIN_MAX_ABS_DPATH_M
  )
  if (
    cross_sensor_confirmed
    and measured_near_path
    and FRONT_CUT_IN_MIN_DREL_M <= float(d_rel)
    <= CROSS_SENSOR_CLOSE_CUTIN_MAX_DREL_M
    and float(predicted_path_overlap_s)
    >= CROSS_SENSOR_CLOSE_CUTIN_MIN_OVERLAP_S
    and float(directional_inward_displacement_m)
    >= CROSS_SENSOR_CLOSE_CUTIN_MIN_INWARD_DISPLACEMENT_M
    and float(directional_consistency)
    >= CROSS_SENSOR_CLOSE_CUTIN_MIN_DIRECTIONAL_CONSISTENCY
    and float(directional_inward_sample_ratio)
    >= CROSS_SENSOR_CLOSE_CUTIN_MIN_INWARD_SAMPLE_RATIO
    and -side * float(d_path_rate_short)
    >= CROSS_SENSOR_CLOSE_CUTIN_MIN_SHORT_INWARD_MPS
    and -side * float(d_path_rate_long)
    >= CROSS_SENSOR_CLOSE_CUTIN_MIN_LONG_INWARD_MPS
  ):
    return True
  # Front-radar azimuth quantization can create a high one-second dPath rate
  # for a parallel vehicle. Do not bypass the measured direction history.
  strong_directional_motion = (
    measured_near_path
    and -side * float(d_path_rate_long)
    >= FRONT_CUT_IN_MIN_DPATH_RATE_MPS
    and float(directional_consistency)
    >= float(minimum_directional_consistency)
    and float(directional_inward_sample_ratio)
    >= DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO
  )
  if strong_directional_motion:
    return True

  directional_future_overlap = (
    measured_near_path
    and float(d_rel) >= FRONT_CUT_IN_MIN_DREL_M
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
    self._active_stationary_shadow = False
    self._last_lead: dict[str, Any] | None = None
    self._last_time_s: float | None = None

  def reset(self) -> None:
    self.active_identity = None
    self._active_stationary_shadow = False
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
        self.active_identity is not None
        and candidate.source == self.active_identity[0]
        and candidate.continuity_id == self.active_identity[2]
        and candidate.retainable
        and self._position_continuous(time_s, candidate)
      )
    )
    eligible = tuple(
      candidate
      for candidate in candidate_values
      if (
        candidate.confirmed_cutin
        or candidate.confirmed_stationary_shadow
        or candidate in active_candidates
      )
    )
    selection = select_dpath_lead_two(
      primary,
      (candidate.lead for candidate in eligible),
      v_ego,
      allow_stopped_track_ids=frozenset(
        int(candidate.lead.get("radarTrackId", candidate.track_id))
        for candidate in active_candidates
      ) | frozenset(
        int(candidate.lead.get("radarTrackId", candidate.track_id))
        for candidate in eligible
        if (
          candidate.confirmed_stationary_shadow
          or candidate.allow_low_speed
        )
      ),
      allow_farther_track_ids=frozenset(
        int(candidate.lead.get("radarTrackId", candidate.track_id))
        for candidate in eligible
        if (
          candidate.confirmed_stationary_shadow
          or (
            candidate in active_candidates
            and self._active_stationary_shadow
          )
        )
      ),
      allow_primary_proximity_track_ids=frozenset(
        int(candidate.lead.get("radarTrackId", candidate.track_id))
        for candidate in eligible
        if (
          candidate.confirmed_stationary_shadow
          or (
            candidate in active_candidates
            and self._active_stationary_shadow
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
        int(candidate.lead.get("radarTrackId", candidate.track_id))
        for candidate in active_candidates
        if candidate.confirmed_cutin
      ) | frozenset(
        int(candidate.lead.get("radarTrackId", candidate.track_id))
        for candidate in eligible
        if candidate.confirmed_cutin and candidate.allow_low_speed
      ),
    )
    selection = DPathLeadSelection(
      cutins=confirmed_selection.cutins,
      lead_two=selection.lead_two,
    )
    if selected is not None:
      self._active_stationary_shadow = (
        selected.confirmed_stationary_shadow
        or (
          selected.identity == self.active_identity
          and self._active_stationary_shadow
        )
      )
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
  allow_farther_track_ids: frozenset[int] = frozenset(),
  allow_primary_proximity_track_ids: frozenset[int] = frozenset(),
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
          > POSITION_ONLY_MAX_ABS_VLEAD_MPS
          or int(lead.get("radarTrackId", -1)) in allow_stopped_track_ids
        )
        and (
          not lead_duplicates_primary(lead, primary)
          or (
            int(lead.get("radarTrackId", -1))
            in allow_primary_proximity_track_ids
            and primary is not None
            and abs(
              float(lead.get("vLead", 0.0))
              - float(primary.get("vLead", 0.0))
            ) > PRIMARY_PROXIMITY_MIN_VLEAD_DELTA_MPS
          )
        )
      )
    ),
    key=lambda lead: float(lead["dRel"]),
  ))
  return DPathLeadSelection(
    cutins=cutins,
    lead_two=cutins[0] if cutins else None,
  )
