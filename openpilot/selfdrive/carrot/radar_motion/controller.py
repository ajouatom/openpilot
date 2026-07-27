#!/usr/bin/env python3
"""Lead-role controller for the independent physical dPath RadarD."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.lead_selection import (
  select_dpath_lead_two,
)
from openpilot.selfdrive.carrot.radar_motion.predictor import (
  RadarMotionCutIn,
  RadarMotionDecisionTracker,
  RadarMotionPredictor,
  project_to_model_path,
  visible_motion_points,
)
from openpilot.selfdrive.carrot.radar_motion.primary import (
  RadarPointSnapshot,
  VisionRadarMatcher,
  lead_from_radar_point,
  lead_from_vision_match,
  select_primary_radar_points,
  snapshot_radar_points,
)


RADAR_MOTION_MAX_TIME_SKEW_S = 0.10
PRIMARY_IDENTITY_HOLD_S = 0.75
PRIMARY_DUPLICATE_MAX_DREL_DELTA_M = 3.5
PRIMARY_DUPLICATE_MAX_YREL_DELTA_M = 1.4


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


@dataclass(frozen=True)
class _RecentPrimary:
  time_s: float
  d_rel: float
  y_rel: float
  v_rel: float
  v_lat: float


class DPathRadarController:
  """Calculate leadOne first, then independently evaluate dPath CUT-IN leadTwo."""

  def __init__(
    self,
    prefer_corner_radar: bool = False,
    enable_radar_tracks: int = 1,
  ) -> None:
    self.primary_matcher = VisionRadarMatcher()
    self.enable_radar_tracks = int(enable_radar_tracks)
    self.motion_sensor = "corner" if prefer_corner_radar else "front"
    self.motion_predictor = RadarMotionPredictor()
    self.motion_decisions = RadarMotionDecisionTracker()
    self.recent_primaries: dict[int, _RecentPrimary] = {}

  def _select_motion_points(
    self,
    points: tuple[RadarPointSnapshot, ...],
  ) -> tuple[RadarPointSnapshot, ...]:
    corner_points = tuple(point for point in points if _is_corner(point))
    if self.motion_sensor == "front" and corner_points:
      # Sensor selection is latched. A temporary corner dropout never causes a
      # frame-by-frame fallback to front radar.
      self.motion_sensor = "corner"
      self.motion_predictor = RadarMotionPredictor()
      self.motion_decisions = RadarMotionDecisionTracker()
    if self.motion_sensor == "corner":
      return corner_points
    return tuple(point for point in points if point.source == "frontRadar")

  @staticmethod
  def _cutin_lead(
    cutin: RadarMotionCutIn,
    points: dict[tuple[str, int], RadarPointSnapshot],
  ) -> dict[str, Any] | None:
    prediction = cutin.prediction
    point = points.get((prediction.source, prediction.track_id))
    if point is None:
      return None
    return lead_from_radar_point(
      point,
      prediction.d_path,
      0.03,
      cutin.score,
    )

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

  @staticmethod
  def _display_leads(
    points: tuple[RadarPointSnapshot, ...],
    path: tuple[tuple[float, float], ...],
  ) -> tuple[
    tuple[dict[str, Any], ...],
    tuple[dict[str, Any], ...],
    tuple[dict[str, Any], ...],
  ]:
    left = []
    center = []
    right = []
    for point in visible_motion_points(points, path):
      d_path = project_to_model_path(path, point.d_rel, point.y_rel).d_path
      lead = lead_from_radar_point(point, d_path, 0.03, 0.0)
      if abs(d_path) < 1.8:
        center.append(lead)
      elif d_path > 0.0:
        left.append(lead)
      else:
        right.append(lead)
    for leads in (left, center, right):
      leads.sort(key=lambda lead: lead["dRel"])
    return tuple(left), tuple(center), tuple(right)

  def _remember_primary(
    self,
    time_s: float,
    lead: dict[str, Any] | None,
  ) -> None:
    self.recent_primaries = {
      track_id: primary
      for track_id, primary in self.recent_primaries.items()
      if time_s - primary.time_s <= PRIMARY_IDENTITY_HOLD_S
    }
    if lead is None or not lead.get("status") or not lead.get("radar"):
      return
    self.recent_primaries[int(lead["radarTrackId"])] = _RecentPrimary(
      time_s=time_s,
      d_rel=float(lead["dRel"]),
      y_rel=float(lead["yRel"]),
      v_rel=float(lead["vRel"]),
      v_lat=float(lead["vLat"]),
    )

  def _duplicates_recent_primary(
    self,
    time_s: float,
    lead: dict[str, Any],
  ) -> bool:
    track_id = int(lead["radarTrackId"])
    for primary_track_id, primary in self.recent_primaries.items():
      age_s = max(0.0, time_s - primary.time_s)
      if track_id == primary_track_id:
        return True
      if (
        abs(
          float(lead["dRel"])
          - (primary.d_rel + primary.v_rel * age_s)
        )
        < PRIMARY_DUPLICATE_MAX_DREL_DELTA_M
        and abs(
          float(lead["yRel"])
          - (primary.y_rel + primary.v_lat * age_s)
        )
        < PRIMARY_DUPLICATE_MAX_YREL_DELTA_M
      ):
        return True
    return False

  def update(
    self,
    time_s: float,
    v_ego: float,
    radar_points: Any,
    model: Any,
    yaw_rate_rad_s: float = 0.0,
    radar_to_model_time_s: float = 0.0,
  ) -> DPathRadarOutput:
    path = _model_path(model)
    if len(path) < 2:
      self.primary_matcher.reset()
      return DPathRadarOutput(
        None, None, None, None, (), (), (), (), (), (),
      )

    primary_points = snapshot_radar_points(radar_points, v_ego)
    points = (
      snapshot_radar_points(
        radar_points,
        v_ego,
        radar_to_model_time_s,
      )
      if abs(radar_to_model_time_s) <= RADAR_MOTION_MAX_TIME_SKEW_S
      else ()
    )

    # This is intentionally first: model lead zero identifies leadOne only
    # among the independently measured front/SCC stream.
    primary_match = self.primary_matcher.match(
      model,
      select_primary_radar_points(
        primary_points,
        self.enable_radar_tracks,
      ),
      path,
    )
    lead_one = (
      lead_from_vision_match(primary_match)
      if primary_match is not None
      else None
    )
    self._remember_primary(time_s, lead_one)

    motion_points = self._select_motion_points(points)
    leads_left, leads_center, leads_right = self._display_leads(
      motion_points,
      path,
    )
    predictions = self.motion_predictor.update(
      time_s,
      motion_points,
      path,
      v_ego,
      yaw_rate_rad_s,
    )
    decision = self.motion_decisions.update(time_s, predictions.values())
    point_by_identity = {
      (point.source, point.track_id): point
      for point in motion_points
    }
    candidates = tuple(
      lead
      for cutin in decision.confirmed
      if (lead := self._cutin_lead(cutin, point_by_identity)) is not None
      and not self._duplicates_recent_primary(time_s, lead)
    )
    selection = select_dpath_lead_two(lead_one, candidates, v_ego)
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
