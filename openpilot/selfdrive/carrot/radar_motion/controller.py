#!/usr/bin/env python3
"""Lead-role controller for the independent physical dPath RadarD."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.lead_selection import (
  DPathLeadCandidate,
  DPathStationaryPrimaryHandoffTracker,
  DPathStationaryShadowTracker,
  DPathLeadTwoTracker,
  cutin_can_compete_with_primary,
  front_cutin_motion_supported,
  lead_duplicates_primary,
)
from openpilot.selfdrive.carrot.radar_motion.predictor import (
  CornerCutInPredecelTracker,
  RadarMotionCutIn,
  RadarMotionDecisionTracker,
  RadarMotionPredictor,
  _scoped_motion_points,
  _visible_scoped_motion_points,
  corner_cutin_predecel_score,
  project_to_model_path,
  radar_motion_sensitivity,
)
from openpilot.selfdrive.carrot.radar_motion.primary import (
  FrontRadarKinematicAssociator,
  RadarPointSnapshot,
  VisionRadarMatcher,
  apply_vision_bracket_cutin_support,
  lead_from_vision,
  lead_from_radar_point,
  match_dpath_primary_lead,
  prefer_front_radar_kinematics,
  snapshot_radar_points,
  vision_lead_from_model,
  vision_only_lead_allowed,
)


# modelV2 is polled at 20 Hz while liveTracks may arrive just before or after
# the camera exposure represented by timestampEof. A 0.10 s hard edge drops a
# valid radar cycle when normal scheduling jitter puts it at 0.101-0.113 s.
RADAR_MOTION_MAX_TIME_SKEW_S = 0.15
# The 0x235/0x180/0x430 object stream is one radar cycle old when emitted.
# Keep this separate from the vehicle's front-radar delay.
CORNER_RADAR_MEASUREMENT_DELAY_S = 0.05
LEAD_ACCEL_TAU_S = 1.5
LEAD_ACCEL_FILTER_TAU_S = 0.45
LEAD_ACCEL_DT_S = 0.05
LEAD_ACCEL_FILTER_ALPHA = (
  LEAD_ACCEL_DT_S / (LEAD_ACCEL_FILTER_TAU_S + LEAD_ACCEL_DT_S)
)
STATIONARY_SHADOW_CORNER_MIN_DREL_GATE_M = 7.0
STATIONARY_SHADOW_CORNER_MAX_DREL_GATE_M = 12.0
STATIONARY_SHADOW_CORNER_DREL_GATE_FRACTION = 0.15
STATIONARY_SHADOW_CORNER_MAX_DPATH_M = 1.25
STATIONARY_SHADOW_CORNER_MAX_DPATH_DELTA_M = 1.25
STATIONARY_SHADOW_CORNER_MAX_ABS_VLEAD_MPS = 3.0
STATIONARY_SHADOW_CORNER_MAX_VLEAD_DELTA_MPS = 3.0
SCC_LEAD_TWO_CONFIRMATION_S = 0.15
SCC_LEAD_TWO_MAX_DREL_M = 150.0
SCC_LEAD_TWO_MAX_VLEAD_MPS = 5.0
SCC_LEAD_TWO_MAX_POSITION_ERROR_M = 3.0
SCC_LEAD_TWO_MAX_SPEED_JUMP_MPS = 2.0
SCC_PHYSICAL_MATCH_MIN_DREL_M = 4.0
SCC_PHYSICAL_MATCH_DREL_FRACTION = 0.05
SCC_PHYSICAL_MATCH_MAX_DREL_M = 8.0
SCC_PHYSICAL_MATCH_MAX_VLEAD_DELTA_MPS = 3.0
SCC_PHYSICAL_MATCH_MAX_ABS_DPATH_M = 2.2
SCC_PRIMARY_DUPLICATE_MAX_DREL_DELTA_M = 5.0
SCC_PRIMARY_DUPLICATE_MAX_VLEAD_DELTA_MPS = 3.0
SCC_PRIMARY_CLOSER_MARGIN_M = 1.0


def _is_corner(point: RadarPointSnapshot) -> bool:
  return point.source.startswith("corner")


def _model_path(model: Any) -> tuple[tuple[float, float], ...]:
  position = getattr(model, "position", None)
  if position is None:
    return ()
  return tuple(
    (float(x), float(y))
    for x, y in zip(position.x, position.y, strict=False)
    if math.isfinite(float(x)) and math.isfinite(float(y))
  )


def _model_ego_speed(model: Any, fallback: float) -> float:
  velocity = getattr(model, "velocity", None)
  values = getattr(velocity, "x", ()) if velocity is not None else ()
  try:
    value = float(values[0])
  except (IndexError, TypeError, ValueError):
    return float(fallback)
  return value if math.isfinite(value) else float(fallback)


def stationary_shadow_corner_supported(
  front: RadarPointSnapshot,
  points: Iterable[RadarPointSnapshot],
  path: tuple[tuple[float, float], ...],
) -> bool:
  """Require an independent slow corner return for a stopped front shadow."""
  front_d_path = project_to_model_path(
    path, front.d_rel, front.y_rel,
  ).d_path
  d_rel_gate = min(
    STATIONARY_SHADOW_CORNER_MAX_DREL_GATE_M,
    max(
      STATIONARY_SHADOW_CORNER_MIN_DREL_GATE_M,
      front.d_rel * STATIONARY_SHADOW_CORNER_DREL_GATE_FRACTION,
    ),
  )
  for corner in points:
    if (
      not corner.measured
      or not _is_corner(corner)
      or abs(corner.v_lead)
      > STATIONARY_SHADOW_CORNER_MAX_ABS_VLEAD_MPS
      or abs(front.d_rel - corner.d_rel) > d_rel_gate
      or abs(front.v_lead - corner.v_lead)
      > STATIONARY_SHADOW_CORNER_MAX_VLEAD_DELTA_MPS
    ):
      continue
    corner_d_path = project_to_model_path(
      path, corner.d_rel, corner.y_rel,
    ).d_path
    if (
      abs(corner_d_path) <= STATIONARY_SHADOW_CORNER_MAX_DPATH_M
      and abs(front_d_path - corner_d_path)
      <= STATIONARY_SHADOW_CORNER_MAX_DPATH_DELTA_M
    ):
      return True
  return False


def _scc_physical_support(
  scc: RadarPointSnapshot,
  points: Iterable[RadarPointSnapshot],
  path: tuple[tuple[float, float], ...],
) -> RadarPointSnapshot | None:
  """Prefer an in-path physical return that corroborates the OEM SCC lead."""
  d_rel_gate = min(
    SCC_PHYSICAL_MATCH_MAX_DREL_M,
    max(
      SCC_PHYSICAL_MATCH_MIN_DREL_M,
      scc.d_rel * SCC_PHYSICAL_MATCH_DREL_FRACTION,
    ),
  )
  supported = []
  for point in points:
    if (
      not point.measured
      or point.source == "scc"
      or not (
        point.source == "frontRadar" or _is_corner(point)
      )
      or abs(point.d_rel - scc.d_rel) > d_rel_gate
      or abs(point.v_lead - scc.v_lead)
      > SCC_PHYSICAL_MATCH_MAX_VLEAD_DELTA_MPS
    ):
      continue
    projection = project_to_model_path(
      path, point.d_rel, point.y_rel,
    )
    if abs(projection.d_path) > SCC_PHYSICAL_MATCH_MAX_ABS_DPATH_M:
      continue
    supported.append((
      0 if point.source == "frontRadar" else 1,
      abs(point.d_rel - scc.d_rel),
      abs(point.v_lead - scc.v_lead),
      point,
    ))
  return min(
    supported,
    key=lambda candidate: candidate[:3],
    default=(0, 0.0, 0.0, None),
  )[-1]


class DPathSccLeadTwoTracker:
  """Confirm an opt-in low-speed OEM SCC backup independently of dPath."""

  def __init__(self) -> None:
    self._since_s: float | None = None
    self._last_time_s: float | None = None
    self._last_point: RadarPointSnapshot | None = None

  def reset(self) -> None:
    self._since_s = None
    self._last_time_s = None
    self._last_point = None

  def _continuous(
    self,
    time_s: float,
    point: RadarPointSnapshot,
  ) -> bool:
    if self._last_time_s is None or self._last_point is None:
      return True
    dt = float(time_s) - self._last_time_s
    if dt < 0.0 or dt > RADAR_MOTION_MAX_TIME_SKEW_S:
      return False
    predicted_d_rel = self._last_point.d_rel + self._last_point.v_rel * dt
    return (
      abs(point.d_rel - predicted_d_rel)
      <= SCC_LEAD_TWO_MAX_POSITION_ERROR_M
      and abs(point.v_lead - self._last_point.v_lead)
      <= SCC_LEAD_TWO_MAX_SPEED_JUMP_MPS
    )

  def update(
    self,
    time_s: float,
    points: Iterable[RadarPointSnapshot],
    *,
    enabled: bool,
  ) -> RadarPointSnapshot | None:
    if not enabled:
      self.reset()
      return None
    candidates = tuple(
      point for point in points
      if (
        point.measured
        and point.source == "scc"
        and 0.8 < point.d_rel <= SCC_LEAD_TWO_MAX_DREL_M
        and point.v_lead < SCC_LEAD_TWO_MAX_VLEAD_MPS
      )
    )
    point = min(candidates, key=lambda value: value.d_rel, default=None)
    if point is None:
      self.reset()
      return None
    if not self._continuous(time_s, point):
      self._since_s = float(time_s)
    elif self._since_s is None:
      self._since_s = float(time_s)
    self._last_time_s = float(time_s)
    self._last_point = point
    if (
      self._since_s is None
      or float(time_s) - self._since_s < SCC_LEAD_TWO_CONFIRMATION_S
    ):
      return None
    return point


def _scc_lead_two_can_compete(
  lead: dict[str, Any],
  primary: dict[str, Any] | None,
) -> bool:
  if primary is None or not primary.get("status"):
    return True
  if lead_duplicates_primary(lead, primary):
    return False
  distance_delta = abs(
    float(lead.get("dRel", 0.0))
    - float(primary.get("dRel", 0.0))
  )
  speed_delta = abs(
    float(lead.get("vLead", 0.0))
    - float(primary.get("vLead", 0.0))
  )
  if (
    distance_delta <= SCC_PRIMARY_DUPLICATE_MAX_DREL_DELTA_M
    and speed_delta <= SCC_PRIMARY_DUPLICATE_MAX_VLEAD_DELTA_MPS
  ):
    return False
  return (
    float(lead.get("dRel", math.inf)) + SCC_PRIMARY_CLOSER_MARGIN_M
    < float(primary.get("dRel", math.inf))
  )


@dataclass(frozen=True)
class DPathRadarOutput:
  lead_one: dict[str, Any] | None
  lead_two: dict[str, Any] | None
  lead_left: dict[str, Any] | None
  lead_right: dict[str, Any] | None
  leads_left: tuple[dict[str, Any], ...]
  leads_center: tuple[dict[str, Any], ...]
  leads_right: tuple[dict[str, Any], ...]
  leads_cutin: tuple[dict[str, Any], ...]
  leads_left2: tuple[dict[str, Any], ...]
  leads_right2: tuple[dict[str, Any], ...]
  lead_cutin_risk: dict[str, Any] | None


class RadarLeadDynamics:
  """Mirror conventional radard's per-track aLeadTau and raw jLead output."""

  def __init__(self) -> None:
    self._a_lead_tau: dict[tuple[str, int], float] = {}

  @staticmethod
  def _identity(point: RadarPointSnapshot) -> tuple[str, int]:
    if (
      point.kinematics_source is not None
      and point.kinematics_track_id is not None
    ):
      return point.kinematics_source, point.kinematics_track_id
    return point.source, point.track_id

  def reset(self) -> None:
    self._a_lead_tau.clear()

  def update(
    self,
    points: tuple[RadarPointSnapshot, ...],
    radar_reaction_factor: float,
  ) -> None:
    factor = max(0.0, float(radar_reaction_factor))
    active: set[tuple[str, int]] = set()
    for point in points:
      identity = point.source, point.track_id
      active.add(identity)
      a_lead_tau = self._a_lead_tau.get(identity, LEAD_ACCEL_TAU_S)
      if (
        abs(point.a_lead) < 0.5 * factor
        and abs(point.j_lead) < 0.5
      ):
        a_lead_tau = LEAD_ACCEL_TAU_S * factor
      else:
        a_lead_tau *= 1.0 - LEAD_ACCEL_FILTER_ALPHA
      self._a_lead_tau[identity] = a_lead_tau

    for identity in tuple(self._a_lead_tau):
      if identity not in active:
        del self._a_lead_tau[identity]

  def a_lead_tau(self, point: RadarPointSnapshot) -> float:
    return self._a_lead_tau.get(
      self._identity(point),
      LEAD_ACCEL_TAU_S,
    )


class DPathRadarController:
  """Calculate leadOne, then current-path and dPath CUT-IN leadTwo."""

  def __init__(
    self,
    prefer_corner_radar: bool = False,
    enable_radar_tracks: int = 1,
    cut_in_sensitivity: int = 3,
    front_radar_measurement_delay_s: float = 0.0,
    corner_radar_measurement_delay_s: float = CORNER_RADAR_MEASUREMENT_DELAY_S,
  ) -> None:
    self.primary_matcher = VisionRadarMatcher()
    self.enable_radar_tracks = int(enable_radar_tracks)
    self.front_radar_measurement_delay_s = max(
      0.0, float(front_radar_measurement_delay_s),
    )
    self.corner_radar_measurement_delay_s = max(
      0.0, float(corner_radar_measurement_delay_s),
    )
    self.motion_sensor = "corner" if prefer_corner_radar else "front"
    self.cut_in_sensitivity = max(0, min(5, int(cut_in_sensitivity)))
    self._reset_motion_pipeline()
    self.primary_cut_out_predictor = RadarMotionPredictor()
    self.front_kinematic_associator = FrontRadarKinematicAssociator()
    self.lead_two_tracker = DPathLeadTwoTracker()
    self.stationary_shadow_tracker = DPathStationaryShadowTracker()
    self.stationary_primary_handoff_tracker = (
      DPathStationaryPrimaryHandoffTracker()
    )
    self.scc_lead_two_tracker = DPathSccLeadTwoTracker()
    self.lead_dynamics = RadarLeadDynamics()

  def _reset_motion_pipeline(self) -> None:
    sensitivity = radar_motion_sensitivity(
      self.cut_in_sensitivity,
      self.motion_sensor,
    )
    self.motion_sensitivity = sensitivity
    self.motion_predictor = RadarMotionPredictor(
      directional_min_consistency=(
        sensitivity.directional_min_consistency
      ),
    )
    self.motion_decisions = RadarMotionDecisionTracker(
      threshold=sensitivity.cut_in_threshold,
      confirmation_s=sensitivity.confirmation_s,
    )
    self.cutin_predecel_tracker = CornerCutInPredecelTracker()

  def _points_at_model_time(
    self,
    radar_points: Any,
    v_ego: float,
    radar_to_model_time_s: float,
  ) -> tuple[RadarPointSnapshot, ...]:
    aligned: list[RadarPointSnapshot] = []
    batch: list[Any] = []
    batch_time_delta_s: float | None = None
    for point in radar_points:
      source = str(getattr(point, "radarSource", getattr(point, "source", "")))
      measurement_delay_s = (
        self.corner_radar_measurement_delay_s
        if source.rsplit(".", 1)[-1].startswith("corner")
        else self.front_radar_measurement_delay_s
      )
      time_delta_s = radar_to_model_time_s + measurement_delay_s
      if abs(time_delta_s) > RADAR_MOTION_MAX_TIME_SKEW_S:
        continue
      if (
        batch
        and batch_time_delta_s is not None
        and time_delta_s != batch_time_delta_s
      ):
        aligned.extend(snapshot_radar_points(
          batch, v_ego, batch_time_delta_s,
        ))
        batch.clear()
      batch.append(point)
      batch_time_delta_s = time_delta_s
    if batch and batch_time_delta_s is not None:
      aligned.extend(snapshot_radar_points(
        batch, v_ego, batch_time_delta_s,
      ))
    return tuple(aligned)

  def _select_motion_points(
    self,
    points: tuple[RadarPointSnapshot, ...],
  ) -> tuple[RadarPointSnapshot, ...]:
    corner_points = tuple(point for point in points if _is_corner(point))
    if self.motion_sensor == "front" and corner_points:
      # Sensor selection is latched. A temporary corner dropout never causes a
      # frame-by-frame fallback to front radar.
      self.motion_sensor = "corner"
      self._reset_motion_pipeline()
      self.lead_two_tracker.reset()
      self.stationary_shadow_tracker.reset()
      self.stationary_primary_handoff_tracker.reset()
      self.scc_lead_two_tracker.reset()
    if self.motion_sensor == "corner":
      return corner_points
    return tuple(point for point in points if point.source == "frontRadar")

  @staticmethod
  def _pick_side(
    leads: tuple[dict[str, Any], ...],
  ) -> dict[str, Any] | None:
    return min(
      (
        lead for lead in leads
        if lead["dRel"] > 5.0 and abs(lead["dPath"]) < 3.5
      ),
      key=lambda lead: lead["dRel"],
      default=None,
    )

  @staticmethod
  def _pick_two(
    leads: tuple[dict[str, Any], ...],
  ) -> tuple[dict[str, Any], ...]:
    usable = tuple(
      lead for lead in leads
      if (
        lead["vLead"] > 2.0
        and abs(lead["dPath"]) < 4.2
        and lead["dRel"] > 2.0
      )
    )
    if not usable:
      return ()
    second = next(
      (
        lead for lead in usable[1:]
        if lead["dRel"] - usable[0]["dRel"] >= 5.0
      ),
      None,
    )
    return (usable[0],) if second is None else (usable[0], second)

  def _display_leads(
    self,
    scoped_points: tuple[Any, ...],
    extra_identities: Iterable[tuple[str, int]] = (),
  ) -> tuple[
    tuple[dict[str, Any], ...],
    tuple[dict[str, Any], ...],
    tuple[dict[str, Any], ...],
  ]:
    left = []
    center = []
    right = []
    projection_by_identity = {
      (point.source, point.track_id): projection
      for point, _, projection in scoped_points
    }
    visible_identities = {
      (point.source, point.track_id)
      for point in _visible_scoped_motion_points(scoped_points)
    }
    visible_identities.update(extra_identities)
    for point, _, _ in scoped_points:
      if (point.source, point.track_id) not in visible_identities:
        continue
      d_path = projection_by_identity[
        (point.source, point.track_id)
      ].d_path
      lead = self._lead_from_radar_point(point, d_path, 0.03, 0.0)
      if abs(d_path) < 1.8:
        center.append(lead)
      elif d_path > 0.0:
        left.append(lead)
      else:
        right.append(lead)
    for leads in (left, center, right):
      leads.sort(key=lambda lead: lead["dRel"])
    return tuple(left), tuple(center), tuple(right)

  def _lead_from_radar_point(
    self,
    point: RadarPointSnapshot,
    d_path: float,
    model_probability: float,
    score: float,
  ) -> dict[str, Any]:
    lead = lead_from_radar_point(
      point,
      d_path,
      model_probability,
      score,
    )
    lead["aLeadTau"] = self.lead_dynamics.a_lead_tau(point)
    return lead

  def update(
    self,
    time_s: float,
    v_ego: float,
    radar_points: Any,
    model: Any,
    yaw_rate_rad_s: float = 0.0,
    radar_to_model_time_s: float = 0.0,
    radar_reaction_factor: float = 1.0,
  ) -> DPathRadarOutput:
    path = _model_path(model)
    if len(path) < 2:
      self.primary_matcher.reset()
      self.lead_two_tracker.reset()
      self.stationary_shadow_tracker.reset()
      self.stationary_primary_handoff_tracker.reset()
      self.scc_lead_two_tracker.reset()
      self.primary_cut_out_predictor = RadarMotionPredictor()
      self.lead_dynamics.reset()
      self.cutin_predecel_tracker.reset()
      return DPathRadarOutput(
        None, None, None, None, (), (), (), (), (), (), None,
      )

    points = self._points_at_model_time(
      radar_points,
      v_ego,
      radar_to_model_time_s,
    )
    self.lead_dynamics.update(points, radar_reaction_factor)
    front_kinematic_matches = self.front_kinematic_associator.update(points)

    # This is intentionally first: model lead zero identifies leadOne with
    # front-first radar selection before independent CUT-IN prediction.
    primary_match = match_dpath_primary_lead(
      self.primary_matcher,
      model,
      points,
      path,
      time_s=time_s,
      enable_radar_tracks=self.enable_radar_tracks,
    )
    lead_one = None
    if primary_match is not None:
      lead_one = self._lead_from_radar_point(
        primary_match.point,
        primary_match.d_path,
        primary_match.probability,
        primary_match.score,
      )
    else:
      vision = self.primary_matcher.vision_fallback
      if (
        vision is not None
        and vision_only_lead_allowed(
          self.enable_radar_tracks,
          side_cutin_supported=(
            self.primary_matcher
            .vision_only_side_cutin_supported
          ),
        )
      ):
        lead_one = lead_from_vision(
          vision,
          path,
          v_ego,
          model_v_ego=_model_ego_speed(model, v_ego),
        )
    motion_points = self._select_motion_points(points)
    scoped_motion_points = _scoped_motion_points(motion_points, path)
    predictions = self.motion_predictor.update(
      time_s,
      motion_points,
      path,
      v_ego,
      yaw_rate_rad_s,
      (
        float(lead_one["dRel"])
        if lead_one is not None
        else None
      ),
      scoped_points=scoped_motion_points,
    )
    front_motion_points = tuple(
      point for point in points if point.source == "frontRadar"
    )
    front_scoped_motion_points = _scoped_motion_points(
      front_motion_points, path,
    )
    primary_track_id = (
      int(lead_one.get("radarTrackId", -1))
      if lead_one is not None and lead_one.get("radar")
      else -1
    )
    primary_cut_out_identities = frozenset(
      (point.source, point.track_id)
      for point in front_motion_points
      if point.track_id == primary_track_id
    )
    primary_cut_out_predictions = self.primary_cut_out_predictor.update(
      time_s,
      front_motion_points,
      path,
      v_ego,
      yaw_rate_rad_s,
      scoped_points=front_scoped_motion_points,
      prediction_identities=primary_cut_out_identities,
    )
    primary_cut_out_probability = max((
      float(prediction.cut_out_probability)
      for prediction in primary_cut_out_predictions.values()
      if prediction.track_id == primary_track_id
    ), default=0.0)
    leads_left, leads_center, leads_right = self._display_leads(
      scoped_motion_points,
      predictions,
    )
    active_identity = self.lead_two_tracker.active_identity
    protected_identities = (
      ()
      if active_identity is None
      else ((active_identity[0], active_identity[1]),)
    )
    visible_points = _visible_scoped_motion_points(
      scoped_motion_points,
      (
        float(lead_one["dRel"])
        if lead_one is not None
        else None
      ),
      protected_identities,
    )
    point_by_identity = {
      (point.source, point.track_id): point
      for point in visible_points
    }
    point_by_identity.update(
      {
        (point.source, point.track_id): point
        for point, _, _ in scoped_motion_points
        if (point.source, point.track_id) in predictions
      }
    )
    vision = vision_lead_from_model(model)
    predictions = {
      identity: (
        apply_vision_bracket_cutin_support(
          prediction,
          point,
          points,
          vision,
          lead_one,
        )
        if (
          point := point_by_identity.get(
            (prediction.source, prediction.track_id),
          )
        ) is not None
        else prediction
      )
      for identity, prediction in predictions.items()
    }
    predecel = self.cutin_predecel_tracker.update(
      time_s,
      (
        RadarMotionCutIn(
          prediction,
          corner_cutin_predecel_score(
            prediction,
            point.d_rel,
            point.v_rel,
          ),
        )
        for prediction in predictions.values()
        if (
          self.motion_sensitivity.cut_in_enabled
          and (
            point := point_by_identity.get(
              (prediction.source, prediction.track_id),
            )
          ) is not None
        )
      ),
    )
    lead_cutin_risk = None
    if predecel is not None:
      risk_point = point_by_identity.get((
        predecel.prediction.source,
        predecel.prediction.track_id,
      ))
      if risk_point is not None:
        lead_cutin_risk = self._lead_from_radar_point(
          risk_point,
          predecel.prediction.d_path,
          0.0,
          predecel.score,
        )
        if lead_duplicates_primary(lead_cutin_risk, lead_one):
          lead_cutin_risk = None
    decision = self.motion_decisions.update(
      time_s,
      (
        predictions.values()
        if self.motion_sensitivity.cut_in_enabled
        else ()
      ),
    )
    confirmed = {
      (
        cutin.prediction.source,
        cutin.prediction.track_id,
        cutin.prediction.continuity_id,
      ): cutin
      for cutin in decision.confirmed
    }
    candidates = []
    for prediction in predictions.values():
      point = point_by_identity.get((prediction.source, prediction.track_id))
      if point is None:
        continue
      identity = (
        prediction.source,
        prediction.track_id,
        prediction.continuity_id,
      )
      cutin = confirmed.get(identity)
      front_motion_supported = front_cutin_motion_supported(
        prediction.source,
        prediction.d_path_rate_long,
        d_rel=point.d_rel,
        v_rel=point.v_rel,
        d_path=prediction.d_path,
        d_path_rate_short=getattr(
          prediction, "d_path_rate_short", prediction.d_path_rate_long,
        ),
        reported_normal_speed=getattr(
          prediction, "reported_normal_speed", 0.0,
        ),
        current_path_occupancy=prediction.current_path_occupancy,
        predicted_path_overlap_s=getattr(
          prediction, "predicted_path_overlap_s", 0.0,
        ),
        directional_inward_displacement_m=getattr(
          prediction, "directional_inward_displacement_m", 0.0,
        ),
        directional_consistency=getattr(
          prediction, "directional_consistency", 0.0,
        ),
        directional_inward_sample_ratio=getattr(
          prediction, "directional_inward_sample_ratio", 0.0,
        ),
        tracked_close_entry=getattr(
          prediction, "front_tracked_close_entry", False,
        ),
        minimum_directional_consistency=(
          self.motion_sensitivity.directional_min_consistency
        ),
      )
      lead_point = prefer_front_radar_kinematics(
        point, points, front_kinematic_matches,
      )
      lead_d_path = (
        project_to_model_path(
          path, lead_point.d_rel, lead_point.y_rel,
        ).d_path
        if lead_point is not point
        else prediction.d_path
      )
      lead = self._lead_from_radar_point(
        lead_point,
        lead_d_path,
        0.03,
        (
          cutin.score
          if cutin is not None
          else prediction.path_entry_probability
        ),
      )
      if lead_duplicates_primary(lead, lead_one):
        if self.lead_two_tracker.active_identity == identity:
          self.lead_two_tracker.reset()
        continue
      candidates.append(DPathLeadCandidate(
        lead=lead,
        source=prediction.source,
        track_id=prediction.track_id,
        continuity_id=prediction.continuity_id,
        retainable=(
          prediction.current_path_occupancy
          or prediction.d_path * prediction.d_path_rate_long <= 0.0
        ),
        confirmed_cutin=(
          self.motion_sensitivity.cut_in_enabled
          and cutin is not None
          and front_motion_supported
          and cutin_can_compete_with_primary(
            lead,
            lead_one,
            projected_path_entry=(
              getattr(prediction, "time_to_entry_s", None) is not None
            ),
            entry_horizon_s=getattr(
              prediction,
              "predicted_path_overlap_start_s",
              getattr(prediction, "time_to_entry_s", None),
            ),
          )
        ),
      ))
    stationary_primary_candidates = []
    for point, _, projection in scoped_motion_points:
      if not _is_corner(point):
        continue
      lead = self._lead_from_radar_point(
        point, projection.d_path, 0.03, 0.0,
      )
      stationary_primary_candidates.append(DPathLeadCandidate(
        lead=lead,
        source=point.source,
        track_id=point.track_id,
        continuity_id=0,
        retainable=True,
        confirmed_cutin=False,
      ))
    stationary_primary_handoff = (
      self.stationary_primary_handoff_tracker.update(
        time_s,
        lead_one,
        stationary_primary_candidates,
        active_identity,
      )
    )
    if (
      stationary_primary_handoff is not None
      and not any(
        candidate.identity == stationary_primary_handoff.identity
        for candidate in candidates
      )
    ):
      candidates.append(stationary_primary_handoff)
    stationary_shadow_inputs = []
    for point, _, projection in front_scoped_motion_points:
      identity = (point.source, point.track_id, 0)
      retained_stationary_shadow = active_identity == identity
      corner_supported = stationary_shadow_corner_supported(
        point, points, path,
      )
      if (
        point.radar_track_state < 2
        or not (corner_supported or retained_stationary_shadow)
      ):
        continue
      lead = self._lead_from_radar_point(
        point, projection.d_path, 0.03, primary_cut_out_probability,
      )
      candidate = DPathLeadCandidate(
        lead=lead,
        source=point.source,
        track_id=point.track_id,
        continuity_id=0,
        retainable=True,
        confirmed_cutin=False,
      )
      if corner_supported:
        stationary_shadow_inputs.append(candidate)
      if (
        retained_stationary_shadow
        and point.track_id != primary_track_id
      ):
        candidates.append(candidate)
    stationary_shadow = self.stationary_shadow_tracker.update(
      time_s,
      lead_one,
      primary_cut_out_probability,
      stationary_shadow_inputs,
    )
    if (
      stationary_shadow is not None
      and stationary_shadow.confirmed_stationary_shadow
      and stationary_shadow.track_id != primary_track_id
      and not any(
        candidate.identity == stationary_shadow.identity
        for candidate in candidates
      )
    ):
      candidates.append(stationary_shadow)
    if (
      active_identity is not None
      and not any(candidate.identity == active_identity for candidate in candidates)
    ):
      source, track_id, continuity_id = active_identity
      point = point_by_identity.get((source, track_id))
      if point is not None:
        lead_point = prefer_front_radar_kinematics(
          point, points, front_kinematic_matches,
        )
        d_path = project_to_model_path(
          path, lead_point.d_rel, lead_point.y_rel,
        ).d_path
        lead = self._lead_from_radar_point(
          lead_point, d_path, 0.03, 0.0,
        )
        if lead_duplicates_primary(lead, lead_one):
          self.lead_two_tracker.reset()
        else:
          candidates.append(DPathLeadCandidate(
            lead=lead,
            source=source,
            track_id=track_id,
            continuity_id=continuity_id,
            retainable=True,
            confirmed_cutin=False,
          ))
    selection = self.lead_two_tracker.update(
      time_s,
      lead_one,
      candidates,
      v_ego,
    )
    scc_point = self.scc_lead_two_tracker.update(
      time_s,
      points,
      enabled=self.enable_radar_tracks >= 2,
    )
    scc_lead_two = None
    if scc_point is not None:
      physical_support = _scc_physical_support(
        scc_point, points, path,
      )
      lead_point = physical_support or scc_point
      scc_lead_two = self._lead_from_radar_point(
        lead_point,
        project_to_model_path(
          path, lead_point.d_rel, lead_point.y_rel,
        ).d_path,
        0.03,
        1.0 if physical_support is not None else 0.5,
      )
      if not _scc_lead_two_can_compete(scc_lead_two, lead_one):
        scc_lead_two = None
    lead_two = selection.lead_two
    if (
      scc_lead_two is not None
      and (
        lead_two is None
        or float(scc_lead_two["dRel"]) < float(lead_two["dRel"])
      )
    ):
      lead_two = scc_lead_two
    return DPathRadarOutput(
      lead_one=lead_one,
      lead_two=lead_two,
      lead_left=self._pick_side(leads_left),
      lead_right=self._pick_side(leads_right),
      leads_left=leads_left,
      leads_center=leads_center,
      leads_right=leads_right,
      leads_cutin=selection.cutins,
      leads_left2=self._pick_two(leads_left),
      leads_right2=self._pick_two(leads_right),
      lead_cutin_risk=lead_cutin_risk,
    )
