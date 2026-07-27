#!/usr/bin/env python3
"""Vision-to-radar primary lead matching for the independent dPath RadarD."""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  project_to_model_path,
)


RADAR_TO_CAMERA_M = 1.52
VISION_LEAD_MIN_PROB = 0.50
VISION_LEAD_HOLD_MIN_PROB = 0.35
VISION_LEAD_HOLD_MAX_FRAMES = 10
VISION_MATCH_DISTANCE_HYSTERESIS_M = 2.0
VISION_MATCH_FRESH_MAX_DPATH_M = 2.0
VISION_MATCH_HELD_MAX_DPATH_M = 4.0
PRIMARY_RADAR_SOURCES = frozenset(("frontRadar", "scc"))
LOW_SPEED_SCC_MAX_VLEAD_MPS = 5.0


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError, IndexError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def _first(values: Any, fallback: float = 0.0) -> float:
  try:
    return _finite(values[0], fallback)
  except (TypeError, IndexError):
    return fallback


def _source(point: Any) -> str:
  source = str(getattr(point, "source", getattr(point, "radarSource", "frontRadar")))
  source = source.rsplit(".", 1)[-1]
  track_id = int(getattr(point, "track_id", getattr(point, "trackId", -1)))
  if source == "frontRadar":
    if 200 <= track_id < 220:
      return "corner235"
    if 240 <= track_id < 250:
      return "corner180"
    if 300 <= track_id < 412:
      return "corner430"
  return source


def _value(point: Any, snake: str, camel: str, fallback: float = 0.0) -> float:
  return _finite(getattr(point, snake, getattr(point, camel, fallback)), fallback)


def _laplacian(value: float, mean: float, scale: float) -> float:
  scale = max(abs(scale), 0.1)
  return math.exp(-abs(value - mean) / scale) / (2.0 * scale)


@dataclass(frozen=True)
class RadarPointSnapshot:
  track_id: int
  source: str
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  v_lead: float
  a_lead: float
  j_lead: float
  measured: bool


@dataclass(frozen=True)
class VisionLead:
  probability: float
  d_rel: float
  y_rel: float
  velocity: float
  x_std: float
  y_std: float
  v_std: float


@dataclass(frozen=True)
class VisionRadarMatch:
  point: RadarPointSnapshot
  probability: float
  score: float
  d_path: float


def snapshot_radar_points(
  points: Iterable[Any],
  v_ego: float,
  time_delta_s: float = 0.0,
) -> tuple[RadarPointSnapshot, ...]:
  """Copy measured radar points into the model timestamp's ego frame."""
  snapshots = []
  for point in points:
    if not bool(getattr(point, "measured", False)):
      continue
    v_rel = _value(point, "v_rel", "vRel")
    yv_rel = _value(point, "yv_rel", "yvRel")
    snapshots.append(RadarPointSnapshot(
      track_id=int(getattr(point, "track_id", getattr(point, "trackId", -1))),
      source=_source(point),
      d_rel=_value(point, "d_rel", "dRel") + v_rel * time_delta_s,
      y_rel=_value(point, "y_rel", "yRel") + yv_rel * time_delta_s,
      v_rel=v_rel,
      a_rel=_value(point, "a_rel", "aRel"),
      yv_rel=yv_rel,
      v_lead=float(v_ego) + v_rel,
      a_lead=_value(point, "a_lead", "aLead"),
      j_lead=_value(point, "j_lead", "jLead"),
      measured=True,
    ))
  return tuple(snapshots)


def select_primary_radar_points(
  points: Iterable[RadarPointSnapshot],
  enable_radar_tracks: int,
) -> tuple[RadarPointSnapshot, ...]:
  """Preserve the removed model RadarD's front/SCC input policy."""
  point_values = tuple(points)
  front = tuple(
    point for point in point_values
    if point.source == "frontRadar" and point.d_rel > 0.2
  )
  scc = tuple(
    point for point in point_values
    if point.source == "scc" and point.d_rel > 0.2
  )
  if enable_radar_tracks <= -2:
    return ()
  if enable_radar_tracks <= 0:
    return scc
  if enable_radar_tracks >= 2:
    return front + tuple(
      point for point in scc
      if point.v_lead < LOW_SPEED_SCC_MAX_VLEAD_MPS
    )
  return front


def vision_lead_from_model(model: Any) -> VisionLead | None:
  """Read the first vision lead exactly as the removed model RadarD did."""
  leads = getattr(model, "leadsV3", ())
  if not leads:
    return None
  lead = leads[0]
  if not getattr(lead, "x", ()) or not getattr(lead, "y", ()) or not getattr(lead, "v", ()):
    return None
  d_rel = _first(lead.x) - RADAR_TO_CAMERA_M
  if d_rel <= 0.5:
    return None
  return VisionLead(
    probability=_finite(getattr(lead, "prob", 0.0)),
    d_rel=d_rel,
    y_rel=-_first(lead.y),
    velocity=_first(lead.v),
    x_std=_first(getattr(lead, "xStd", ()), 1.0),
    y_std=_first(getattr(lead, "yStd", ()), 1.0),
    v_std=_first(getattr(lead, "vStd", ()), 1.0),
  )


class VisionRadarMatcher:
  """Match model lead zero to front/SCC radar before any leadTwo work."""

  def __init__(self) -> None:
    self.last_identity: tuple[str, int] | None = None
    self.low_probability_hold_frames = 0

  def reset(self) -> None:
    self.last_identity = None
    self.low_probability_hold_frames = 0

  @staticmethod
  def _identity(point: RadarPointSnapshot) -> tuple[str, int]:
    return point.source, point.track_id

  def match(
    self,
    model: Any,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
  ) -> VisionRadarMatch | None:
    vision = vision_lead_from_model(model)
    if vision is None:
      self.reset()
      return None

    high_probability = vision.probability > VISION_LEAD_MIN_PROB
    holding_previous = (
      not high_probability
      and vision.probability > VISION_LEAD_HOLD_MIN_PROB
      and self.last_identity is not None
      and self.low_probability_hold_frames < VISION_LEAD_HOLD_MAX_FRAMES
    )
    if not high_probability and not holding_previous:
      self.reset()
      return None

    candidates: list[tuple[RadarPointSnapshot, float, float, bool]] = []
    for point in points:
      if point.source not in PRIMARY_RADAR_SOURCES:
        continue
      identity = self._identity(point)
      held_identity = identity == self.last_identity
      if holding_previous and not held_identity:
        continue
      if not 0.5 < point.d_rel < 180.0:
        continue
      projection = project_to_model_path(path, point.d_rel, point.y_rel)
      if abs(projection.d_path) > (
        VISION_MATCH_HELD_MAX_DPATH_M
        if held_identity
        else VISION_MATCH_FRESH_MAX_DPATH_M
      ):
        continue
      score = (
        _laplacian(point.d_rel, vision.d_rel, vision.x_std)
        * _laplacian(point.y_rel, vision.y_rel, vision.y_std)
        * _laplacian(point.v_lead, vision.velocity, vision.v_std)
      )
      candidates.append((point, score, projection.d_path, held_identity))

    velocity_tolerance = max(
      5.0,
      abs(vision.velocity) * (
        0.3
        + 0.2 * min(max((vision.probability - 0.8) / 0.18, 0.0), 1.0)
      ),
    )
    usable = [
      candidate for candidate in candidates
      if (
        abs(candidate[0].d_rel - vision.d_rel)
        < (
          max(5.0, vision.d_rel * 0.25)
          + (VISION_MATCH_DISTANCE_HYSTERESIS_M if candidate[3] else 0.0)
        )
        and abs(candidate[0].y_rel - vision.y_rel) < 2.0
        and (
          abs(candidate[0].v_lead - vision.velocity) < velocity_tolerance
          or (
            candidate[0].v_lead > 3.0
            and abs(candidate[0].v_lead - vision.velocity)
            < max(velocity_tolerance * 3.0, 20.0)
          )
        )
      )
    ]
    if not usable:
      self.reset()
      return None

    selected = max(
      usable,
      key=lambda candidate: (
        candidate[1],
        -abs(candidate[0].d_rel - vision.d_rel),
      ),
    )
    self.last_identity = self._identity(selected[0])
    self.low_probability_hold_frames = (
      self.low_probability_hold_frames + 1 if holding_previous else 0
    )
    return VisionRadarMatch(
      point=selected[0],
      probability=vision.probability,
      score=selected[1],
      d_path=selected[2],
    )


def lead_from_radar_point(
  point: RadarPointSnapshot,
  d_path: float,
  model_probability: float,
  score: float,
) -> dict[str, Any]:
  return {
    "dRel": float(point.d_rel),
    "yRel": float(point.y_rel),
    "dPath": float(d_path),
    "vRel": float(point.v_rel),
    "aRel": float(point.a_rel),
    "vLead": float(point.v_lead),
    "vLeadK": float(point.v_lead),
    "aLead": float(point.a_lead),
    "aLeadK": float(point.a_lead),
    "aLeadTau": 1.5,
    "jLead": float(point.j_lead),
    "vLat": float(point.yv_rel),
    "status": True,
    "fcw": False,
    "modelProb": float(model_probability),
    "radar": True,
    "radarTrackId": int(point.track_id),
    "score": float(score),
  }


def lead_from_vision_match(match: VisionRadarMatch) -> dict[str, Any]:
  return lead_from_radar_point(
    match.point,
    match.d_path,
    match.probability,
    match.score,
  )
