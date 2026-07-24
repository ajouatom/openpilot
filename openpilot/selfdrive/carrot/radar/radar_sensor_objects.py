#!/usr/bin/env python3
"""Independent radar sensor objects and post-decision control matching."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar.radar_object_fusion import FusedRadarObject


NEAR_SIDE_NO_FRONT_MATCH_DREL_M = 5.0
NEAR_SIDE_MIN_ABS_YREL_M = 0.8
NEAR_SIDE_IDENTITY_MAX_DREL_DELTA_M = 1.5
NEAR_SIDE_IDENTITY_MAX_VREL_DELTA_MPS = 1.0
NEAR_SIDE_IDENTITY_MAX_YREL_DELTA_M = 1.0
LOW_SPEED_SCC_MAX_VLEAD_MPS = 5.0
POST_MATCH_MAX_DREL_DELTA_M = 2.5
POST_MATCH_MAX_VREL_DELTA_MPS = 1.2
POST_MATCH_MAX_YREL_DELTA_M = 0.9
POST_MATCH_EXPECTED_DREL_OFFSET_M = 1.25


@dataclass(frozen=True)
class IndependentRadarObjects:
  front: tuple[FusedRadarObject, ...]
  corner: tuple[FusedRadarObject, ...]

  @property
  def all(self) -> tuple[FusedRadarObject, ...]:
    return self.front + self.corner


def _finite(value: float, fallback: float = 0.0) -> float:
  parsed = float(value)
  return parsed if math.isfinite(parsed) else fallback


def sensor_object(point: Any) -> FusedRadarObject:
  source = str(point.source)
  is_corner = source.startswith("corner")
  is_scc = source == "scc"
  return FusedRadarObject(
    object_id=f"{source}:{point.track_id}",
    d_rel=float(point.d_rel),
    y_rel=float(point.y_rel),
    v_rel=float(point.v_rel),
    a_rel=float(point.a_rel),
    yv_rel=float(point.yv_rel),
    v_lead=float(point.v_lead),
    front_track_id=None if is_corner or is_scc else int(point.track_id),
    corner_track_id=int(point.track_id) if is_corner else None,
    scc_track_id=int(point.track_id) if is_scc else None,
    front_d_rel=None if is_corner or is_scc else float(point.d_rel),
    corner_d_rel=float(point.d_rel) if is_corner else None,
    front_y_rel=None if is_corner or is_scc else float(point.y_rel),
    corner_y_rel=float(point.y_rel) if is_corner else None,
    front_v_rel=None if is_corner or is_scc else float(point.v_rel),
    corner_v_rel=float(point.v_rel) if is_corner else None,
    distance_source=source,
    lateral_source=source,
    match_confidence=0.0,
    pair_age=1,
    a_lead=_finite(getattr(point, "a_lead", 0.0)),
    j_lead=_finite(getattr(point, "j_lead", 0.0)),
  )


def independent_radar_objects(
  points: Iterable[Any],
  include_scc: bool = True,
  enable_radar_tracks: int | None = None,
) -> IndependentRadarObjects:
  measured = [point for point in points if point.measured and point.d_rel > 0.2]
  front_points = [point for point in measured if point.source == "frontRadar"]
  scc_points = [point for point in measured if point.source == "scc"]
  corner_points = [point for point in measured if str(point.source).startswith("corner")]
  if enable_radar_tracks is None:
    # Legacy/offline callers keep their explicit include_scc behavior. Device
    # and production replay always pass EnableRadarTracks below.
    if include_scc and not front_points:
      front_points = scc_points
  elif enable_radar_tracks <= -2:
    front_points = []
  elif enable_radar_tracks <= 0:
    front_points = scc_points if include_scc else []
  elif enable_radar_tracks >= 2 and include_scc:
    front_points.extend(point for point in scc_points if point.v_lead < LOW_SPEED_SCC_MAX_VLEAD_MPS)
  return IndependentRadarObjects(
    front=tuple(sensor_object(point) for point in front_points),
    corner=tuple(sensor_object(point) for point in corner_points),
  )


def match_corner_to_front_identity(corner_prediction: Any, front_predictions: Iterable[Any]) -> Any | None:
  """Associate independently evaluated corner/front predictions by identity."""
  corner = corner_prediction.features.radar_object
  near_side = (
    corner.d_rel < NEAR_SIDE_NO_FRONT_MATCH_DREL_M
    and abs(corner.y_rel) >= NEAR_SIDE_MIN_ABS_YREL_M
  )
  max_d_delta = NEAR_SIDE_IDENTITY_MAX_DREL_DELTA_M if near_side else POST_MATCH_MAX_DREL_DELTA_M
  max_v_delta = NEAR_SIDE_IDENTITY_MAX_VREL_DELTA_MPS if near_side else POST_MATCH_MAX_VREL_DELTA_MPS
  max_y_delta = NEAR_SIDE_IDENTITY_MAX_YREL_DELTA_M if near_side else POST_MATCH_MAX_YREL_DELTA_M

  candidates: list[tuple[float, Any]] = []
  for prediction in front_predictions:
    front = prediction.features.radar_object
    if front.front_track_id is None and front.scc_track_id is None:
      continue
    d_delta = front.d_rel - corner.d_rel
    v_delta = front.v_rel - corner.v_rel
    y_delta = front.y_rel - corner.y_rel
    if (
      abs(d_delta) > max_d_delta
      or abs(v_delta) > max_v_delta
      or abs(y_delta) > max_y_delta
    ):
      continue
    if abs(front.y_rel) > 0.8 and abs(corner.y_rel) > 0.8 and front.y_rel * corner.y_rel < 0.0:
      continue
    expected_d_delta = 0.0 if near_side else POST_MATCH_EXPECTED_DREL_OFFSET_M
    cost = (
      abs(d_delta - expected_d_delta) / max_d_delta
      + abs(v_delta) / max_v_delta
      + abs(y_delta) / max_y_delta
    )
    candidates.append((cost, prediction))
  return min(candidates, key=lambda item: item[0])[1] if candidates else None


def post_match_corner_to_front(corner_prediction: Any, front_predictions: Iterable[Any]) -> Any | None:
  """Find front control values for an already selected corner candidate.

  This is deliberately post-decision association: neither model sees values
  from the other radar. Very close side objects keep corner measurements.
  """
  corner = corner_prediction.features.radar_object
  if (
    corner.d_rel < NEAR_SIDE_NO_FRONT_MATCH_DREL_M
    and abs(corner.y_rel) >= NEAR_SIDE_MIN_ABS_YREL_M
  ):
    return None
  return match_corner_to_front_identity(corner_prediction, front_predictions)
