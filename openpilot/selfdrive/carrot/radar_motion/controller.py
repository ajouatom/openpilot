#!/usr/bin/env python3
"""Lead-role controller for the independent physical dPath RadarD."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.lead_selection import (
  DPathLeadCandidate,
  DPathLeadTwoTracker,
  cutin_can_compete_with_primary,
  front_cutin_motion_supported,
  lead_duplicates_primary,
)
from openpilot.selfdrive.carrot.radar_motion.predictor import (
  RadarMotionDecisionTracker,
  RadarMotionPredictor,
  _scoped_motion_points,
  _visible_scoped_motion_points,
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
    self.front_kinematic_associator = FrontRadarKinematicAssociator()
    self.lead_two_tracker = DPathLeadTwoTracker()
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
      self.lead_dynamics.reset()
      return DPathRadarOutput(
        None, None, None, None, (), (), (), (), (), (),
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
    return DPathRadarOutput(
      lead_one=lead_one,
      lead_two=selection.lead_two,
      lead_left=self._pick_side(leads_left),
      lead_right=self._pick_side(leads_right),
      leads_left=leads_left,
      leads_center=leads_center,
      leads_right=leads_right,
      leads_cutin=selection.cutins,
      leads_left2=self._pick_two(leads_left),
      leads_right2=self._pick_two(leads_right),
    )
