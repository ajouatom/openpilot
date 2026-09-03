#!/usr/bin/env python3
"""Vision-to-radar primary lead matching for the independent dPath RadarD."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, replace
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  MIN_PREDICTED_PATH_OVERLAP_S,
  RadarMotionPrediction,
  project_to_model_path,
  turning_corner_path_entry_allowed,
)


RADAR_TO_CAMERA_M = 1.52
VISION_LEAD_MIN_PROB = 0.40
VISION_LEAD_HOLD_MIN_PROB = 0.35
VISION_LEAD_HOLD_MAX_FRAMES = 10
VISION_MATCH_DISTANCE_HYSTERESIS_M = 2.0
VISION_MATCH_FRESH_MIN_SCORE = 1.0e-4
VISION_MATCH_FRESH_MAX_DPATH_M = 2.0
VISION_MATCH_HELD_MAX_DPATH_M = 4.0
VISION_MATCH_XSTD_SIGMA = 2.5
VISION_MATCH_XSTD_MAX_M = 15.0
VISION_RADAR_MAX_DISTANCE_ERROR_M = 15.0
VISION_RADAR_FAR_MIN_SEED_PROB = 0.90
VISION_RADAR_FAR_MIN_HOLD_PROB = VISION_LEAD_MIN_PROB
VISION_RADAR_FAR_DISTANCE_ERROR_FRACTION = 0.30
VISION_RADAR_FAR_MAX_DISTANCE_ERROR_M = 30.0
VISION_RADAR_FAR_MAX_DPATH_M = 1.5
VISION_RADAR_FAR_MAX_VLEAD_DELTA_MPS = 10.0
VISION_RADAR_FAR_CONFIRMATION_S = 0.25
VISION_ONLY_CORROBORATION_MAX_DPATH_DELTA_M = 2.0
VISION_ONLY_CORROBORATION_MAX_VLEAD_DELTA_MPS = 20.0
VISION_CORROBORATED_MIN_OBSERVED_S = 0.25
VISION_CORROBORATED_MAX_OBSERVATION_GAP_S = 0.15
VISION_ONLY_RADAR_TRACK_MODE = -2
PRIMARY_RADAR_SOURCES = frozenset(("frontRadar", "scc"))
LOW_SPEED_SCC_MAX_VLEAD_MPS = 5.0
STATIONARY_VISION_MIN_PROB = VISION_LEAD_MIN_PROB
STATIONARY_WEAK_VISION_MIN_PROB = 0.20
STATIONARY_WEAK_VISION_PAIR_CONFIRMATION_S = 0.35
STATIONARY_WEAK_VISION_PAIR_HOLD_S = 0.75
STATIONARY_WEAK_VISION_MAX_DPATH_M = 1.0
STATIONARY_WEAK_VISION_MAX_DISTANCE_ERROR_M = 15.0
STATIONARY_WEAK_VISION_MAX_SPEED_DELTA_MPS = 12.0
STATIONARY_WEAK_PAIR_MAX_DREL_M = 7.0
STATIONARY_WEAK_PAIR_MAX_DPATH_DELTA_M = 1.5
STATIONARY_WEAK_PAIR_MAX_VLEAD_DELTA_MPS = 2.0
STATIONARY_WEAK_PAIR_FRONT_MAX_DPATH_M = 1.0
STATIONARY_WEAK_PAIR_CORNER_MAX_DPATH_M = 0.75
STATIONARY_FRONT_MIN_VISION_SUPPORT_FRAMES = 3
STATIONARY_CONFIRMATION_S = 0.25
STATIONARY_RADAR_ONLY_CONFIRMATION_S = 0.50
STATIONARY_MAX_ABS_VLEAD_MPS = 4.0
STATIONARY_TURN_MIN_ABS_YAW_RATE_RAD_S = 0.10
STATIONARY_TURN_CORNER_MIN_VISION_PROB = 0.80
STATIONARY_HELD_CORNER_MAX_ABS_VLEAD_MPS = 8.0
STATIONARY_MEASUREMENT_DROPOUT_HOLD_S = 0.10
STATIONARY_MAX_VISION_SPEED_DELTA_MPS = 12.0
STATIONARY_TRUSTED_MAX_VISION_SPEED_DELTA_MPS = 20.0
STATIONARY_TURN_FRONT_MIN_ABS_YAW_RATE_RAD_S = 0.02
STATIONARY_TURN_FRONT_FAST_VISION_SPEED_DELTA_MPS = 6.0
STATIONARY_TURN_FRONT_FAST_VISION_MAX_YREL_ERROR_M = 1.25
STATIONARY_VISION_DISTANCE_FRACTION = 0.30
STATIONARY_VISION_DISTANCE_MAX_M = (
  VISION_RADAR_MAX_DISTANCE_ERROR_M
)
STATIONARY_FRESH_MAX_DPATH_M = 2.0
STATIONARY_HELD_MAX_DPATH_M = 4.0
STATIONARY_HELD_FRONT_NO_VISION_MAX_DPATH_M = 1.1
STATIONARY_HELD_FRONT_DEPARTURE_DPATH_M = STATIONARY_FRESH_MAX_DPATH_M
STATIONARY_HELD_FRONT_DEPARTURE_CONFIRMATION_S = 0.25
STATIONARY_RADAR_ONLY_HELD_MAX_DPATH_M = 1.2
STATIONARY_RADAR_ONLY_CORNER_MAX_DPATH_M = 0.50
# A stopped vehicle first seen only by corner radar at close range is
# indistinguishable from a curb, pole, or road-edge reflection that sweeps
# through the curved model path. Real stopped leads normally have already
# been observed farther away or are corroborated by vision/front radar/SCC.
# Keep the useful early corner-only acquisition, but do not turn a newly
# appearing close reflection directly into a braking lead.
STATIONARY_RADAR_ONLY_CORNER_MIN_ACQUISITION_DREL_M = 70.0
STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DREL_M = 7.0
STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DPATH_M = 1.5
STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_VLEAD_MPS = 2.0
STATIONARY_VISION_CROSS_SOURCE_MAX_DISTANCE_ERROR_M = 30.0
STATIONARY_VISION_CROSS_SOURCE_MAX_DREL_M = 5.0
STATIONARY_VISION_CROSS_SOURCE_MAX_YREL_M = 0.75
STATIONARY_VISION_CROSS_SOURCE_MAX_VLEAD_MPS = 2.5
STATIONARY_VISION_CROSS_SOURCE_CORNER_MAX_ABS_VLEAD_MPS = 6.0
STATIONARY_VISION_PATH_OUTLIER_MAX_DPATH_M = 8.0
STATIONARY_VISION_PATH_OUTLIER_MIN_PROB = 0.85
STATIONARY_VISION_PATH_OUTLIER_HOLD_S = 0.20
STATIONARY_LONGITUDINAL_CONTINUITY_M = 2.5
STATIONARY_LATERAL_CONTINUITY_M = 1.5
STATIONARY_CLOSER_HANDOFF_MIN_VISION_PROB = 0.90
STATIONARY_CLOSER_HANDOFF_CONFIRMATION_S = 0.25
STATIONARY_CLOSER_HANDOFF_MIN_DREL_GAIN_M = 1.0
STATIONARY_CLOSER_HANDOFF_MAX_DREL_DELTA_M = 5.0
STATIONARY_CLOSER_HANDOFF_MAX_YREL_DELTA_M = 0.75
STATIONARY_CLOSER_HANDOFF_MAX_VLEAD_DELTA_MPS = 2.0
STATIONARY_CLOSER_HANDOFF_MIN_COST_GAIN = 0.10
STATIONARY_CLOSER_HANDOFF_RANGE_MAX_YREL_DELTA_M = 1.25
STATIONARY_CLOSER_HANDOFF_MAX_DPATH_M = 1.0
STATIONARY_CLOSER_HANDOFF_MAX_VISION_YREL_ERROR_M = 1.0
STATIONARY_CLOSER_HANDOFF_MIN_VISION_RANGE_GAIN_M = 0.75
# Keep radar-only moving promotion disjoint from the stationary fallback.
# A front-only point in this band needs vision, corner, or permitted SCC
# corroboration instead of bypassing stationary-reflection safeguards.
RADAR_ONLY_MOVING_MIN_VLEAD_MPS = STATIONARY_MAX_ABS_VLEAD_MPS
RADAR_ONLY_MOVING_CONFIRMATION_S = 0.25
RADAR_ONLY_MOVING_TENTATIVE_CONFIRMATION_S = 0.75
RADAR_ONLY_MOVING_FAR_CORNER_CONFIRMATION_S = 1.0
RADAR_ONLY_MOVING_CORNER_MAX_LONGITUDINAL_ERROR_RATE_MPS = 2.0
RADAR_ONLY_MOVING_CLOSER_SWITCH_MIN_GAP_M = 3.0
RADAR_ONLY_MOVING_CLOSER_SWITCH_MAX_DPATH_M = 0.5
RADAR_ONLY_MOVING_MAX_DREL_M = 100.0
RADAR_ONLY_MOVING_MID_DREL_M = 60.0
RADAR_ONLY_MOVING_FAR_DREL_M = 80.0
# A close-born corner-only return has neither cross-sensor corroboration nor
# lateral entry history. Do not turn that ambiguous reflection directly into
# a primary control lead; real close cut-ins remain handled by the occupancy
# path, while a far-acquired identity may be held as it approaches.
RADAR_ONLY_MOVING_CORNER_MIN_ACQUISITION_DREL_M = 70.0
RADAR_ONLY_MOVING_NEAR_DPATH_M = 1.1
RADAR_ONLY_MOVING_MID_DPATH_M = 0.9
RADAR_ONLY_MOVING_FAR_DPATH_M = 0.75
RADAR_ONLY_MOVING_MAX_PATH_Y_OFFSET_M = 1.5
RADAR_ONLY_MOVING_RECEDING_MAX_DREL_M = 45.0
RADAR_ONLY_MOVING_RECEDING_VREL_MPS = 0.5
FRONT_KINEMATIC_MATCH_MAX_DREL_DELTA_M = 5.0
FRONT_KINEMATIC_MATCH_MAX_YREL_DELTA_M = 0.75
FRONT_KINEMATIC_MATCH_MAX_VLEAD_DELTA_MPS = 2.0
FRONT_KINEMATIC_HOLD_MAX_DREL_DELTA_M = 12.0
FRONT_KINEMATIC_HOLD_MAX_YREL_DELTA_M = 2.0
FRONT_KINEMATIC_HOLD_MAX_VLEAD_DELTA_MPS = 2.0
VISION_BRACKET_MIN_PROB = 0.90
VISION_BRACKET_MAX_RANGE_GAP_M = 20.0
VISION_BRACKET_MAX_SPEED_DELTA_MPS = 2.0
VISION_BRACKET_MAX_ABS_DPATH_M = 3.0
VISION_BRACKET_MIN_INWARD_RATE_MPS = 0.10
VISION_BRACKET_MIN_HISTORY_S = 1.0
VISION_BRACKET_MIN_INWARD_TRAVEL_M = 0.15
VISION_BRACKET_MIN_MOTION_SUPPORT = 0.70


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


def _normalized_source(source: Any, track_id: int) -> str:
  # radarSource is authoritative in live car.RadarData. Track IDs are local to
  # each vehicle interface, so a numeric range used by Hyundai corner radar
  # can also be reached by another brand's monotonically allocated front
  # tracks (for example Volkswagen). Legacy-log ID recovery belongs in the
  # brand-aware replay adapter, not the production radar path.
  del track_id
  source = str(source)
  return source.rsplit(".", 1)[-1]


def _source(point: Any) -> str:
  source = getattr(point, "source", getattr(point, "radarSource", "frontRadar"))
  track_id = int(getattr(point, "track_id", getattr(point, "trackId", -1)))
  return _normalized_source(source, track_id)


def _value(point: Any, snake: str, camel: str, fallback: float = 0.0) -> float:
  return _finite(getattr(point, snake, getattr(point, camel, fallback)), fallback)


def _laplacian(value: float, mean: float, scale: float) -> float:
  scale = max(abs(scale), 0.1)
  return math.exp(-abs(value - mean) / scale) / (2.0 * scale)


def _stationary_vision_speed_delta_limit(
  point: RadarPointSnapshot,
) -> float:
  if point.source.startswith("corner") or point.source == "scc":
    return STATIONARY_TRUSTED_MAX_VISION_SPEED_DELTA_MPS
  return STATIONARY_MAX_VISION_SPEED_DELTA_MPS


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
  radar_track_state: int = 0
  kinematics_source: str | None = None
  kinematics_track_id: int | None = None


@dataclass(frozen=True)
class VisionLead:
  probability: float
  d_rel: float
  y_rel: float
  velocity: float
  x_std: float
  y_std: float
  v_std: float
  acceleration: float = 0.0


@dataclass(frozen=True)
class VisionRadarMatch:
  point: RadarPointSnapshot
  probability: float
  score: float
  d_path: float


def _front_kinematic_match_cost(
  corner: RadarPointSnapshot,
  front: RadarPointSnapshot,
) -> float | None:
  d_delta = abs(corner.d_rel - front.d_rel)
  y_delta = abs(corner.y_rel - front.y_rel)
  v_delta = abs(corner.v_lead - front.v_lead)
  if (
    d_delta > FRONT_KINEMATIC_MATCH_MAX_DREL_DELTA_M
    or y_delta > FRONT_KINEMATIC_MATCH_MAX_YREL_DELTA_M
    or v_delta > FRONT_KINEMATIC_MATCH_MAX_VLEAD_DELTA_MPS
  ):
    return None
  return (
    d_delta / FRONT_KINEMATIC_MATCH_MAX_DREL_DELTA_M
    + y_delta / FRONT_KINEMATIC_MATCH_MAX_YREL_DELTA_M
    + v_delta / FRONT_KINEMATIC_MATCH_MAX_VLEAD_DELTA_MPS
  )


def _front_kinematic_hold_compatible(
  corner: RadarPointSnapshot,
  front: RadarPointSnapshot,
) -> bool:
  return (
    abs(corner.d_rel - front.d_rel)
    <= FRONT_KINEMATIC_HOLD_MAX_DREL_DELTA_M
    and abs(corner.y_rel - front.y_rel)
    <= FRONT_KINEMATIC_HOLD_MAX_YREL_DELTA_M
    and abs(corner.v_lead - front.v_lead)
    <= FRONT_KINEMATIC_HOLD_MAX_VLEAD_DELTA_MPS
  )


def _with_front_radar_kinematics(
  point: RadarPointSnapshot,
  front: RadarPointSnapshot,
) -> RadarPointSnapshot:
  return RadarPointSnapshot(
    track_id=point.track_id,
    source=point.source,
    d_rel=front.d_rel,
    y_rel=point.y_rel,
    v_rel=front.v_rel,
    a_rel=front.a_rel,
    yv_rel=point.yv_rel,
    v_lead=front.v_lead,
    a_lead=front.a_lead,
    j_lead=front.j_lead,
    measured=point.measured and front.measured,
    kinematics_source=front.source,
    kinematics_track_id=front.track_id,
  )


def prefer_front_radar_kinematics(
  point: RadarPointSnapshot,
  points: Iterable[RadarPointSnapshot],
  front_matches: Mapping[
    tuple[str, int], RadarPointSnapshot
  ] | None = None,
) -> RadarPointSnapshot:
  """Use mutually matched front longitudinal values for a corner track."""
  if not point.source.startswith("corner"):
    return point
  if front_matches is not None:
    front = front_matches.get((point.source, point.track_id))
    return (
      _with_front_radar_kinematics(point, front)
      if front is not None
      else point
    )
  point_values = tuple(points)
  matches = tuple(
    (cost, front)
    for front in point_values
    if (
      front.source == "frontRadar"
      and (
        cost := _front_kinematic_match_cost(point, front)
      ) is not None
    )
  )
  if not matches:
    return point
  _, front = min(
    matches,
    key=lambda match: (match[0], match[1].track_id),
  )
  reverse_matches = tuple(
    (cost, corner)
    for corner in point_values
    if (
      corner.source.startswith("corner")
      and (
        cost := _front_kinematic_match_cost(corner, front)
      ) is not None
    )
  )
  if not reverse_matches:
    return point
  _, matched_corner = min(
    reverse_matches,
    key=lambda match: (match[0], match[1].track_id),
  )
  if (
    matched_corner.source != point.source
    or matched_corner.track_id != point.track_id
  ):
    return point
  return _with_front_radar_kinematics(point, front)


class FrontRadarKinematicAssociator:
  """Keep an established corner/front identity through reflection migration."""

  def __init__(self) -> None:
    self._pairs: dict[tuple[str, int], tuple[str, int]] = {}

  def reset(self) -> None:
    self._pairs.clear()

  def update(
    self,
    points: Iterable[RadarPointSnapshot],
  ) -> dict[tuple[str, int], RadarPointSnapshot]:
    point_values = tuple(points)
    corners = tuple(
      point for point in point_values
      if point.source.startswith("corner")
    )
    fronts = tuple(
      point for point in point_values
      if point.source == "frontRadar"
    )
    point_by_identity = {
      (point.source, point.track_id): point
      for point in point_values
    }
    best_front_by_corner: dict[
      tuple[str, int],
      tuple[float, tuple[str, int]],
    ] = {}
    best_corner_by_front: dict[
      tuple[str, int],
      tuple[float, tuple[str, int]],
    ] = {}
    for corner in corners:
      corner_identity = corner.source, corner.track_id
      for front in fronts:
        cost = _front_kinematic_match_cost(corner, front)
        if cost is None:
          continue
        front_identity = front.source, front.track_id
        corner_match = (cost, front_identity)
        current_front = best_front_by_corner.get(corner_identity)
        if (
          current_front is None
          or (corner_match[0], corner_match[1][1])
          < (current_front[0], current_front[1][1])
        ):
          best_front_by_corner[corner_identity] = corner_match
        front_match = (cost, corner_identity)
        current_corner = best_corner_by_front.get(front_identity)
        if (
          current_corner is None
          or (front_match[0], front_match[1][1])
          < (current_corner[0], current_corner[1][1])
        ):
          best_corner_by_front[front_identity] = front_match
    normal_matches: dict[
      tuple[str, int], RadarPointSnapshot
    ] = {}
    for corner_identity, best_front in best_front_by_corner.items():
      if best_front is None:
        continue
      _, front_identity = best_front
      reverse = best_corner_by_front.get(front_identity)
      if reverse is None or reverse[1] != corner_identity:
        continue
      front = point_by_identity.get(front_identity)
      if front is not None:
        normal_matches[corner_identity] = front

    remembered_pairs: dict[
      tuple[str, int], tuple[str, int]
    ] = {}
    for corner_identity, front_identity in self._pairs.items():
      corner = point_by_identity.get(corner_identity)
      front = point_by_identity.get(front_identity)
      if (
        corner is not None
        and front is not None
        and _front_kinematic_hold_compatible(corner, front)
      ):
        remembered_pairs[corner_identity] = front_identity
    remembered_pairs.update({
      corner_identity: (front.source, front.track_id)
      for corner_identity, front in normal_matches.items()
    })

    matches = dict(normal_matches)
    used_fronts = {
      (front.source, front.track_id)
      for front in matches.values()
    }
    for corner_identity, front_identity in sorted(remembered_pairs.items()):
      if (
        corner_identity in matches
        or front_identity in used_fronts
      ):
        continue
      corner = point_by_identity.get(corner_identity)
      front = point_by_identity.get(front_identity)
      if (
        corner is None
        or front is None
        or not _front_kinematic_hold_compatible(corner, front)
      ):
        continue
      matches[corner_identity] = front
      used_fronts.add(front_identity)

    self._pairs = remembered_pairs
    return matches


def apply_vision_bracket_cutin_support(
  prediction: RadarMotionPrediction,
  point: RadarPointSnapshot,
  points: Iterable[RadarPointSnapshot],
  vision: VisionLead | None,
  lead_one: dict[str, Any] | None,
) -> RadarMotionPrediction:
  """Fuse vision existence with a corner track that brackets the primary lead.

  Vision range is not used for control. A mutually matched front point supplies
  longitudinal radar kinematics while the corner track retains its measured
  dPath history.
  """
  if (
    not isinstance(prediction, RadarMotionPrediction)
    or vision is None
    or vision.probability < VISION_BRACKET_MIN_PROB
    or lead_one is None
    or not bool(lead_one.get("status", True))
    or not point.source.startswith("corner")
    or prediction.current_path_occupancy
    or abs(prediction.d_path) > VISION_BRACKET_MAX_ABS_DPATH_M
    or prediction.predicted_path_overlap_s
    < MIN_PREDICTED_PATH_OVERLAP_S
    or prediction.motion_consistency < VISION_BRACKET_MIN_MOTION_SUPPORT
    or prediction.recent_motion_support < VISION_BRACKET_MIN_MOTION_SUPPORT
  ):
    return prediction

  lead_point = prefer_front_radar_kinematics(point, points)
  if (
    getattr(lead_point, "kinematics_source", None) != "frontRadar"
    or getattr(lead_point, "kinematics_track_id", None) is None
  ):
    return prediction

  lead_one_d_rel = _finite(lead_one.get("dRel"), math.inf)
  if not (
    0.5 < lead_point.d_rel < vision.d_rel < lead_one_d_rel
    and vision.d_rel - lead_point.d_rel
    <= VISION_BRACKET_MAX_RANGE_GAP_M
    and abs(lead_point.v_lead - vision.velocity)
    <= VISION_BRACKET_MAX_SPEED_DELTA_MPS
  ):
    return prediction

  inward_sign = -math.copysign(1.0, prediction.d_path)
  if (
    inward_sign * prediction.d_path_rate_short
    < VISION_BRACKET_MIN_INWARD_RATE_MPS
    or inward_sign * prediction.d_path_rate_long
    < VISION_BRACKET_MIN_INWARD_RATE_MPS
  ):
    return prediction

  past = min(
    (
      sample
      for sample in prediction.history
      if sample.age_s >= VISION_BRACKET_MIN_HISTORY_S
    ),
    key=lambda sample: sample.age_s,
    default=None,
  )
  if (
    past is None
    or abs(past.d_path) - abs(prediction.d_path)
    < VISION_BRACKET_MIN_INWARD_TRAVEL_M
  ):
    return prediction

  fused_probability = min(
    1.0,
    vision.probability
    * prediction.motion_consistency
    * prediction.recent_motion_support,
  )
  return replace(
    prediction,
    cut_in_probability=max(
      prediction.cut_in_probability,
      fused_probability,
    ),
    path_entry_probability=max(
      prediction.path_entry_probability,
      fused_probability,
    ),
    reason="vision-bracketed physical CUT-IN",
  )


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
      radar_track_state=int(_value(
        point, "radar_track_state", "trackState",
      )),
    ))
  return tuple(snapshots)


def snapshot_live_radar_points(
  points: Iterable[Any],
  v_ego: float,
  time_delta_s: float = 0.0,
) -> tuple[RadarPointSnapshot, ...]:
  """Copy production car.RadarData points without probing absent aliases.

  Cap'n Proto field lookup is unusually expensive when the requested field is
  absent. The generic adapter above intentionally supports snake_case replay
  objects, but its fallback lookups dominate the production RadarD CPU budget.
  liveTracks always uses the camelCase car.RadarData schema, so read it directly.
  """
  snapshots = []
  for point in points:
    if not bool(point.measured):
      continue
    track_id = int(point.trackId)
    v_rel = _finite(point.vRel)
    yv_rel = _finite(point.yvRel)
    snapshots.append(RadarPointSnapshot(
      track_id=track_id,
      source=_normalized_source(point.radarSource, track_id),
      d_rel=_finite(point.dRel) + v_rel * time_delta_s,
      y_rel=_finite(point.yRel) + yv_rel * time_delta_s,
      v_rel=v_rel,
      a_rel=_finite(point.aRel),
      yv_rel=yv_rel,
      v_lead=float(v_ego) + v_rel,
      a_lead=_finite(point.aLead),
      j_lead=_finite(point.jLead),
      measured=True,
      radar_track_state=int(_finite(point.trackState)),
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
  if enable_radar_tracks >= 3:
    return front + scc
  if enable_radar_tracks == 2:
    return front + tuple(
      point for point in scc
      if point.v_lead < LOW_SPEED_SCC_MAX_VLEAD_MPS
    )
  return front


def select_dpath_primary_radar_points(
  points: Iterable[RadarPointSnapshot],
  enable_radar_tracks: int,
) -> tuple[RadarPointSnapshot, ...]:
  """Apply the configured front/SCC primary-source policy."""
  if enable_radar_tracks == 3:
    # Mode 3 first tries only the front-radar/vision association. The
    # controller uses SCC unconditionally if that association fails.
    return tuple(
      point for point in points
      if point.source == "frontRadar" and point.d_rel > 0.2
    )
  return select_primary_radar_points(
    points, enable_radar_tracks,
  )


def select_dpath_fallback_radar_points(
  points: Iterable[RadarPointSnapshot],
  enable_radar_tracks: int,
) -> tuple[RadarPointSnapshot, ...]:
  """Keep corner/front fallback while honoring SCC and track settings."""
  point_values = tuple(points)
  configured_primary = select_primary_radar_points(
    point_values, enable_radar_tracks,
  )
  if enable_radar_tracks <= 0:
    return configured_primary
  configured_scc = {
    (point.source, point.track_id)
    for point in configured_primary
    if point.source == "scc"
  }
  return tuple(
    point for point in point_values
    if (
      point.source != "scc"
      or (point.source, point.track_id) in configured_scc
    )
  )


def vision_only_lead_allowed(
  enable_radar_tracks: int,
) -> bool:
  """Allow blue leadOne only when radar tracks are disabled."""
  return enable_radar_tracks <= VISION_ONLY_RADAR_TRACK_MODE


def unconditional_scc_match(
  points: Iterable[RadarPointSnapshot],
) -> VisionRadarMatch | None:
  """Return the nearest valid SCC object without vision or lateral matching."""
  point = min(
    (
      point for point in points
      if point.source == "scc" and point.measured and point.d_rel > 0.2
    ),
    key=lambda candidate: candidate.d_rel,
    default=None,
  )
  if point is None:
    return None
  return VisionRadarMatch(
    point=replace(point, y_rel=0.0, yv_rel=0.0),
    probability=0.0,
    score=1.0,
    d_path=0.0,
  )


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
    acceleration=_first(getattr(lead, "a", ()), 0.0),
  )


class VisionRadarMatcher:
  """Match vision/radar leads and confirm no-vision moving radar leads."""

  def __init__(self) -> None:
    self.last_identity: tuple[str, int] | None = None
    self.low_probability_hold_frames = 0
    self.stationary_identity: tuple[str, int] | None = None
    self._stationary_pending_identity: tuple[str, int] | None = None
    self._stationary_pending_since_s: float | None = None
    self._stationary_pending_vision_support_frames = 0
    self._stationary_last_point: RadarPointSnapshot | None = None
    self._stationary_last_time_s: float | None = None
    self._stationary_seed_probability = 0.0
    self._stationary_seed_score = 0.0
    self._stationary_path_outlier_since_s: float | None = None
    self._stationary_front_departure_since_s: float | None = None
    self._observed_since_s: dict[tuple[str, int], float] = {}
    self._observed_last_s: dict[tuple[str, int], float] = {}
    self._stationary_corner_supported = False
    self._stationary_weak_pair_identity: (
      tuple[int, str, int] | None
    ) = None
    self._stationary_weak_pair_last_vision_time_s: float | None = None
    self._stationary_pending_weak_pair_supported = False
    self._stationary_closer_challenger_identity: (
      tuple[str, int] | None
    ) = None
    self._stationary_closer_challenger_since_s: float | None = None
    self._stationary_closer_challenger_last_point: (
      RadarPointSnapshot | None
    ) = None
    self._stationary_closer_challenger_last_time_s: float | None = None
    self._vision_fallback: VisionLead | None = None
    self._vision_fallback_hold_frames = 0
    self.radar_only_moving_identity: tuple[str, int] | None = None
    self._radar_only_moving_pending_identity: (
      tuple[str, int] | None
    ) = None
    self._radar_only_moving_pending_since_s: float | None = None
    self._radar_only_moving_pending_start_d_rel: float | None = None
    self._radar_only_moving_integrated_v_rel_m = 0.0
    self._radar_only_moving_last_point: RadarPointSnapshot | None = None
    self._radar_only_moving_last_time_s: float | None = None
    self._radar_only_moving_challenger_identity: (
      tuple[str, int] | None
    ) = None
    self._radar_only_moving_challenger_since_s: float | None = None
    self._radar_only_moving_challenger_last_point: (
      RadarPointSnapshot | None
    ) = None
    self._radar_only_moving_challenger_last_time_s: float | None = None
    self._far_vision_radar_identity: tuple[str, int] | None = None
    self._far_vision_radar_since_s: float | None = None
    self._far_vision_radar_last_point: RadarPointSnapshot | None = None
    self._far_vision_radar_last_time_s: float | None = None
    self._rejected_radar_only_moving_identity: tuple[str, int] | None = None
    self._rejected_radar_only_moving_last_point: (
      RadarPointSnapshot | None
    ) = None
    self._rejected_radar_only_moving_last_time_s: float | None = None

  def reset(self) -> None:
    self._reset_moving()
    self._reset_stationary()
    self._reset_radar_only_moving()
    self._reset_far_vision_radar()
    self._reset_rejected_radar_only_moving()
    self._vision_fallback = None
    self._vision_fallback_hold_frames = 0

  def _reset_moving(self) -> None:
    self.last_identity = None
    self.low_probability_hold_frames = 0

  def _reset_stationary(self) -> None:
    self.stationary_identity = None
    self._stationary_pending_identity = None
    self._stationary_pending_since_s = None
    self._stationary_pending_vision_support_frames = 0
    self._stationary_last_point = None
    self._stationary_last_time_s = None
    self._stationary_seed_probability = 0.0
    self._stationary_seed_score = 0.0
    self._stationary_path_outlier_since_s = None
    self._stationary_front_departure_since_s = None
    self._stationary_corner_supported = False
    self._stationary_weak_pair_identity = None
    self._stationary_weak_pair_last_vision_time_s = None
    self._stationary_pending_weak_pair_supported = False
    self._reset_stationary_closer_challenger()

  def _reset_stationary_closer_challenger(self) -> None:
    self._stationary_closer_challenger_identity = None
    self._stationary_closer_challenger_since_s = None
    self._stationary_closer_challenger_last_point = None
    self._stationary_closer_challenger_last_time_s = None

  def _reset_radar_only_moving(self) -> None:
    self.radar_only_moving_identity = None
    self._radar_only_moving_pending_identity = None
    self._radar_only_moving_pending_since_s = None
    self._radar_only_moving_pending_start_d_rel = None
    self._radar_only_moving_integrated_v_rel_m = 0.0
    self._radar_only_moving_last_point = None
    self._radar_only_moving_last_time_s = None
    self._reset_radar_only_moving_challenger()

  def _reset_radar_only_moving_challenger(self) -> None:
    self._radar_only_moving_challenger_identity = None
    self._radar_only_moving_challenger_since_s = None
    self._radar_only_moving_challenger_last_point = None
    self._radar_only_moving_challenger_last_time_s = None

  def _update_radar_only_moving_longitudinal_history(
    self,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> None:
    identity = self._identity(point)
    continuing = (
      identity == self._radar_only_moving_pending_identity
      and self._radar_only_moving_last_point is not None
      and self._radar_only_moving_last_time_s is not None
    )
    if not continuing:
      self._radar_only_moving_pending_identity = identity
      self._radar_only_moving_pending_since_s = time_s
      self._radar_only_moving_pending_start_d_rel = point.d_rel
      self._radar_only_moving_integrated_v_rel_m = 0.0
    else:
      dt = time_s - self._radar_only_moving_last_time_s
      if dt > 0.0:
        self._radar_only_moving_integrated_v_rel_m += 0.5 * (
          self._radar_only_moving_last_point.v_rel + point.v_rel
        ) * dt
    self._radar_only_moving_last_point = point
    self._radar_only_moving_last_time_s = time_s

  def _radar_only_moving_longitudinally_consistent(
    self,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> bool:
    if not point.source.startswith("corner"):
      return True
    if (
      self._radar_only_moving_pending_since_s is None
      or self._radar_only_moving_pending_start_d_rel is None
    ):
      return False
    duration_s = time_s - self._radar_only_moving_pending_since_s
    if duration_s < RADAR_ONLY_MOVING_CONFIRMATION_S:
      return True
    observed_delta_m = (
      point.d_rel - self._radar_only_moving_pending_start_d_rel
    )
    error_rate_mps = abs(
      observed_delta_m - self._radar_only_moving_integrated_v_rel_m
    ) / max(duration_s, 1.0e-3)
    return (
      error_rate_mps
      <= RADAR_ONLY_MOVING_CORNER_MAX_LONGITUDINAL_ERROR_RATE_MPS
    )

  def _reset_far_vision_radar(self) -> None:
    self._far_vision_radar_identity = None
    self._far_vision_radar_since_s = None
    self._far_vision_radar_last_point = None
    self._far_vision_radar_last_time_s = None

  def _reset_rejected_radar_only_moving(self) -> None:
    self._rejected_radar_only_moving_identity = None
    self._rejected_radar_only_moving_last_point = None
    self._rejected_radar_only_moving_last_time_s = None

  def _radar_only_moving_identity_rejected(
    self,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> bool:
    if self._identity(point) != self._rejected_radar_only_moving_identity:
      return False
    if (
      self._rejected_radar_only_moving_last_point is None
      or self._rejected_radar_only_moving_last_time_s is None
      or not self._stationary_position_continuous(
        self._rejected_radar_only_moving_last_point,
        self._rejected_radar_only_moving_last_time_s,
        point,
        time_s,
      )
    ):
      self._reset_rejected_radar_only_moving()
      return False
    self._rejected_radar_only_moving_last_point = point
    self._rejected_radar_only_moving_last_time_s = time_s
    return True

  def _reject_radar_only_moving(
    self,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> None:
    self._rejected_radar_only_moving_identity = self._identity(point)
    self._rejected_radar_only_moving_last_point = point
    self._rejected_radar_only_moving_last_time_s = time_s

  def _update_vision_fallback(
    self,
    vision: VisionLead | None,
  ) -> None:
    if (
      vision is not None
      and vision.probability >= VISION_LEAD_MIN_PROB
    ):
      self._vision_fallback = vision
      self._vision_fallback_hold_frames = 0
      return
    if (
      vision is not None
      and vision.probability > VISION_LEAD_HOLD_MIN_PROB
      and self._vision_fallback is not None
      and self._vision_fallback_hold_frames
      < VISION_LEAD_HOLD_MAX_FRAMES
    ):
      self._vision_fallback = vision
      self._vision_fallback_hold_frames += 1
      return
    self._vision_fallback = None
    self._vision_fallback_hold_frames = 0

  @property
  def vision_fallback(self) -> VisionLead | None:
    return self._vision_fallback

  @staticmethod
  def _identity(point: RadarPointSnapshot) -> tuple[str, int]:
    return point.source, point.track_id

  @staticmethod
  def _stationary_position_continuous(
    previous: RadarPointSnapshot,
    previous_time_s: float,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> bool:
    dt = time_s - previous_time_s
    if not 0.0 <= dt <= 0.25:
      return False
    predicted_d_rel = previous.d_rel + previous.v_rel * dt
    predicted_y_rel = previous.y_rel + previous.yv_rel * dt
    return (
      abs(point.d_rel - predicted_d_rel)
      <= STATIONARY_LONGITUDINAL_CONTINUITY_M
      and abs(point.y_rel - predicted_y_rel)
      <= STATIONARY_LATERAL_CONTINUITY_M
    )

  @staticmethod
  def _stationary_source_rank(point: RadarPointSnapshot) -> int:
    if point.source == "frontRadar":
      return 0
    if point.source.startswith("corner"):
      return 1
    if point.source == "scc":
      return 2
    return 3

  def _stationary_radar_only_initial_projection_points(
    self,
    points: Sequence[RadarPointSnapshot],
    time_s: float,
  ) -> tuple[RadarPointSnapshot, ...]:
    """Project trusted sources and identities before front corroboration."""
    retained_identities = {
      identity
      for identity in (
        self.stationary_identity,
        self._stationary_pending_identity,
        (
          self._identity(self._stationary_last_point)
          if self._stationary_last_point is not None
          else None
        ),
      )
      if identity is not None
    }

    projection_points: list[RadarPointSnapshot] = []
    for point in points:
      identity = self._identity(point)
      if (
        point.source.startswith("corner")
        or point.source == "scc"
        or identity in retained_identities
      ):
        projection_points.append(point)
        continue
      if point.source != "frontRadar":
        continue
      if self._stationary_cross_source_continuous(point, time_s):
        projection_points.append(point)
    return tuple(projection_points)

  @staticmethod
  def _stationary_radar_only_corroborating_front_points(
    points: Sequence[RadarPointSnapshot],
    projected: Sequence[tuple[RadarPointSnapshot, float]],
  ) -> tuple[RadarPointSnapshot, ...]:
    central_corners = tuple(
      point for point, d_path in projected
      if (
        point.source.startswith("corner")
        and abs(d_path) <= STATIONARY_RADAR_ONLY_CORNER_MAX_DPATH_M
      )
    )
    if not central_corners:
      return ()
    projected_identities = {
      (point.source, point.track_id) for point, _ in projected
    }
    return tuple(
      point for point in points
      if (
        point.source == "frontRadar"
        and (point.source, point.track_id) not in projected_identities
        and any(
          abs(point.d_rel - corner.d_rel)
          <= STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DREL_M
          and abs(point.v_lead - corner.v_lead)
          <= STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_VLEAD_MPS
          for corner in central_corners
        )
      )
    )

  @staticmethod
  def _stationary_front_corner_pairs(
    points: Sequence[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
  ) -> tuple[
    tuple[
      RadarPointSnapshot,
      float,
      RadarPointSnapshot,
      float,
    ], ...
  ]:
    """Find central front/corner returns that represent one slow object."""
    fronts = tuple(
      point for point in points
      if (
        point.measured
        and point.source == "frontRadar"
        and 0.5 < point.d_rel < 180.0
        and abs(point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      )
    )
    corners = tuple(
      point for point in points
      if (
        point.measured
        and point.source.startswith("corner")
        and 0.5 < point.d_rel < 180.0
        and abs(point.v_lead)
        <= STATIONARY_VISION_CROSS_SOURCE_CORNER_MAX_ABS_VLEAD_MPS
      )
    )
    pairs: list[
      tuple[
        RadarPointSnapshot,
        float,
        RadarPointSnapshot,
        float,
      ]
    ] = []
    projected_corner_by_identity: dict[tuple[str, int], float] = {}
    for front in fronts:
      matches = tuple(
        corner for corner in corners
        if (
          abs(front.d_rel - corner.d_rel)
          <= STATIONARY_VISION_CROSS_SOURCE_MAX_DREL_M
          and abs(front.y_rel - corner.y_rel)
          <= STATIONARY_VISION_CROSS_SOURCE_MAX_YREL_M
          and abs(front.v_lead - corner.v_lead)
          <= STATIONARY_VISION_CROSS_SOURCE_MAX_VLEAD_MPS
        )
      )
      if not matches:
        continue
      front_d_path = project_to_model_path(
        path, front.d_rel, front.y_rel,
      ).d_path
      if abs(front_d_path) > STATIONARY_FRESH_MAX_DPATH_M:
        continue
      ordered_corners = sorted(
        matches,
        key=lambda corner: (
          abs(front.d_rel - corner.d_rel),
          abs(front.y_rel - corner.y_rel),
          corner.track_id,
        ),
      )
      for corner in ordered_corners:
        corner_identity = corner.source, corner.track_id
        corner_d_path = projected_corner_by_identity.get(corner_identity)
        if corner_d_path is None:
          corner_d_path = project_to_model_path(
            path, corner.d_rel, corner.y_rel,
          ).d_path
          projected_corner_by_identity[corner_identity] = corner_d_path
        if abs(corner_d_path) <= STATIONARY_FRESH_MAX_DPATH_M:
          pairs.append((front, front_d_path, corner, corner_d_path))
          break
    return tuple(pairs)

  @staticmethod
  def _stationary_vision_cross_source_position_cost(
    vision: VisionLead | None,
    front: RadarPointSnapshot,
  ) -> float | None:
    if (
      vision is None
      or vision.probability < STATIONARY_VISION_MIN_PROB
    ):
      return None
    distance_gate = min(
      STATIONARY_VISION_CROSS_SOURCE_MAX_DISTANCE_ERROR_M,
      max(
        VISION_RADAR_MAX_DISTANCE_ERROR_M,
        vision.d_rel * STATIONARY_VISION_DISTANCE_FRACTION,
        min(
          STATIONARY_VISION_CROSS_SOURCE_MAX_DISTANCE_ERROR_M,
          abs(vision.x_std) * 3.0,
        ),
      ),
    )
    lateral_gate = max(2.0, min(4.0, abs(vision.y_std) * 3.0))
    distance_error = abs(front.d_rel - vision.d_rel)
    lateral_error = abs(front.y_rel - vision.y_rel)
    if distance_error > distance_gate or lateral_error > lateral_gate:
      return None
    return distance_error / distance_gate + lateral_error / lateral_gate

  @staticmethod
  def _stationary_vision_cross_source_front_support(
    vision: VisionLead | None,
    pairs: Sequence[
      tuple[
        RadarPointSnapshot,
        float,
        RadarPointSnapshot,
        float,
      ]
    ],
  ) -> tuple[
    tuple[RadarPointSnapshot, float, float, RadarPointSnapshot], ...
  ]:
    """Use vision once to seed a physically paired front/corner object."""
    if (
      vision is None
      or vision.probability < STATIONARY_VISION_MIN_PROB
    ):
      return ()
    supported: list[
      tuple[RadarPointSnapshot, float, float, RadarPointSnapshot]
    ] = []
    for front, front_d_path, corner, _ in pairs:
      position_cost = (
        VisionRadarMatcher._stationary_vision_cross_source_position_cost(
          vision, front,
        )
      )
      if (
        position_cost is None
        or abs(front.v_lead - vision.velocity)
        > STATIONARY_MAX_VISION_SPEED_DELTA_MPS
      ):
        continue
      cost = (
        position_cost
        + abs(front.d_rel - corner.d_rel)
        / STATIONARY_VISION_CROSS_SOURCE_MAX_DREL_M
        + abs(front.y_rel - corner.y_rel)
        / STATIONARY_VISION_CROSS_SOURCE_MAX_YREL_M
        + abs(front.v_lead - corner.v_lead)
        / STATIONARY_VISION_CROSS_SOURCE_MAX_VLEAD_MPS
      )
      supported.append((front, front_d_path, cost, corner))
    return tuple(supported)

  @staticmethod
  def _stationary_cross_source_equivalent(
    first: RadarPointSnapshot,
    second: RadarPointSnapshot,
  ) -> bool:
    if not (
      first.source == "frontRadar"
      and second.source.startswith("corner")
    ):
      return False
    return (
      abs(first.d_rel - second.d_rel)
      <= STATIONARY_VISION_CROSS_SOURCE_MAX_DREL_M
      and abs(first.y_rel - second.y_rel)
      <= STATIONARY_VISION_CROSS_SOURCE_MAX_YREL_M
      and abs(first.v_lead - second.v_lead)
      <= STATIONARY_VISION_CROSS_SOURCE_MAX_VLEAD_MPS
    )

  def _stationary_cross_source_continuous(
    self,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> bool:
    previous = self._stationary_last_point
    previous_time_s = self._stationary_last_time_s
    if (
      not self._stationary_corner_supported
      or previous is None
      or previous_time_s is None
      or point.source == previous.source
      or not (
        point.source.startswith("corner")
        or previous.source.startswith("corner")
      )
      or not (
        point.source == "frontRadar"
        or point.source.startswith("corner")
      )
      or not (
        previous.source == "frontRadar"
        or previous.source.startswith("corner")
      )
    ):
      return False
    return self._stationary_position_continuous(
      previous,
      previous_time_s,
      point,
      time_s,
    )

  def _stationary_corner_slot_continuous(
    self,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> bool:
    previous = self._stationary_last_point
    previous_time_s = self._stationary_last_time_s
    return (
      previous is not None
      and previous_time_s is not None
      and point.source.startswith("corner")
      and point.source == previous.source
      and self._identity(point) != self._identity(previous)
      and abs(point.v_lead) <= STATIONARY_HELD_CORNER_MAX_ABS_VLEAD_MPS
      and self._stationary_position_continuous(
        previous,
        previous_time_s,
        point,
        time_s,
      )
    )

  def _stationary_measurement_dropout_hold(
    self,
    vision: VisionLead | None,
    support_points: Sequence[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float,
  ) -> VisionRadarMatch | None:
    previous = self._stationary_last_point
    previous_time_s = self._stationary_last_time_s
    if (
      self.stationary_identity is None
      or previous is None
      or previous_time_s is None
    ):
      return None
    dt = time_s - previous_time_s
    if not 0.0 < dt <= STATIONARY_MEASUREMENT_DROPOUT_HOLD_S:
      return None
    predicted = replace(
      previous,
      d_rel=previous.d_rel + previous.v_rel * dt,
      y_rel=previous.y_rel + previous.yv_rel * dt,
      measured=False,
    )
    strong_vision = (
      vision is not None
      and vision.probability >= STATIONARY_VISION_MIN_PROB
    )
    vision_supported = (
      strong_vision
      and vision is not None
      and self._stationary_vision_cross_source_position_cost(
        vision, predicted,
      ) is not None
    )
    corner_supported = (
      not strong_vision
      and previous.source in PRIMARY_RADAR_SOURCES
      and any(
        point.measured
        and point.source.startswith("corner")
        and _front_kinematic_hold_compatible(point, predicted)
        for point in support_points
      )
    )
    if not vision_supported and not corner_supported:
      return None
    d_path = project_to_model_path(
      path, predicted.d_rel, predicted.y_rel,
    ).d_path
    if abs(d_path) > STATIONARY_HELD_MAX_DPATH_M:
      return None
    return VisionRadarMatch(
      point=predicted,
      probability=(
        vision.probability
        if vision is not None
        else self._stationary_seed_probability
      ),
      score=max(0.0, 1.0 - self._stationary_seed_score),
      d_path=d_path,
    )

  @staticmethod
  def _stationary_vision_cost(
    vision: VisionLead,
    point: RadarPointSnapshot,
    d_path: float,
    prefer_corner: bool,
  ) -> float | None:
    base_cost = VisionRadarMatcher._stationary_vision_base_cost(
      vision, point,
    )
    if base_cost is None:
      return None
    return (
      base_cost
      + 0.15 * abs(d_path) / STATIONARY_FRESH_MAX_DPATH_M
      - (
        0.35
        if prefer_corner and point.source.startswith("corner")
        else 0.0
      )
    )

  @staticmethod
  def _stationary_vision_base_cost(
    vision: VisionLead | None,
    point: RadarPointSnapshot,
  ) -> float | None:
    if (
      vision is None
      or vision.probability < STATIONARY_VISION_MIN_PROB
      or abs(point.v_lead - vision.velocity)
      > _stationary_vision_speed_delta_limit(point)
    ):
      return None
    distance_gate = max(
      6.0,
      min(
        STATIONARY_VISION_DISTANCE_MAX_M,
        vision.d_rel * STATIONARY_VISION_DISTANCE_FRACTION,
      ),
      min(
        STATIONARY_VISION_DISTANCE_MAX_M,
        abs(vision.x_std) * 3.0,
      ),
    )
    lateral_gate = max(2.0, min(4.0, abs(vision.y_std) * 3.0))
    distance_error = abs(point.d_rel - vision.d_rel)
    lateral_error = abs(point.y_rel - vision.y_rel)
    if distance_error > distance_gate or lateral_error > lateral_gate:
      return None
    return (
      distance_error / distance_gate
      + lateral_error / lateral_gate
    )

  @staticmethod
  def _stationary_radar_only_support(
    candidates: Sequence[
      tuple[RadarPointSnapshot, float]
    ],
  ) -> list[tuple[RadarPointSnapshot, float, float]]:
    """Trust only a central corner/SCC; cross-source support only ranks it."""
    front = tuple(
      candidate
      for candidate in candidates
      if candidate[0].source == "frontRadar"
    )
    supported: list[
      tuple[RadarPointSnapshot, float, float]
    ] = []
    for point, d_path in candidates:
      if point.source == "scc":
        if abs(d_path) <= STATIONARY_RADAR_ONLY_CORNER_MAX_DPATH_M:
          supported.append((
            point,
            d_path,
            abs(d_path) / STATIONARY_RADAR_ONLY_CORNER_MAX_DPATH_M,
          ))
        continue
      if not point.source.startswith("corner"):
        continue
      cross_source = min(
        (
          (
            abs(point.d_rel - front_point.d_rel),
            abs(d_path - front_d_path),
            abs(point.v_lead - front_point.v_lead),
          )
          for front_point, front_d_path in front
          if (
            abs(point.d_rel - front_point.d_rel)
            <= STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DREL_M
            and abs(d_path - front_d_path)
            <= STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DPATH_M
            and abs(point.v_lead - front_point.v_lead)
            <= STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_VLEAD_MPS
          )
        ),
        default=None,
      )
      # Cross-sensor corroboration confirms that the reflection is physical;
      # it must not move a roadside stationary object onto the ego path.
      if abs(d_path) > STATIONARY_RADAR_ONLY_CORNER_MAX_DPATH_M:
        continue
      if (
        cross_source is None
        and point.d_rel
        < STATIONARY_RADAR_ONLY_CORNER_MIN_ACQUISITION_DREL_M
      ):
        continue
      support_cost = (
        abs(d_path) / STATIONARY_RADAR_ONLY_CORNER_MAX_DPATH_M
        if cross_source is None
        else (
          cross_source[0]
          / STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DREL_M
          + cross_source[1]
          / STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_DPATH_M
          + cross_source[2]
          / STATIONARY_RADAR_ONLY_CROSS_SOURCE_MAX_VLEAD_MPS
        )
      )
      supported.append((point, d_path, support_cost))
    return supported

  def _stationary_weak_vision_pair_support(
    self,
    vision: VisionLead | None,
    points: Sequence[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float,
  ) -> list[tuple[RadarPointSnapshot, float, float]]:
    """Use weak vision only to confirm one tight front/corner physical pair."""
    has_current_weak_vision = bool(
      vision is not None
      and vision.probability >= STATIONARY_WEAK_VISION_MIN_PROB
    )
    has_held_pair = bool(
      self._stationary_weak_pair_identity is not None
      and self._stationary_weak_pair_last_vision_time_s is not None
      and time_s - self._stationary_weak_pair_last_vision_time_s
      <= STATIONARY_WEAK_VISION_PAIR_HOLD_S
    )
    if not has_current_weak_vision and not has_held_pair:
      self._stationary_weak_pair_identity = None
      self._stationary_weak_pair_last_vision_time_s = None
      return []

    fronts = tuple(
      (point, project_to_model_path(path, point.d_rel, point.y_rel).d_path)
      for point in points
      if (
        point.measured
        and point.source == "frontRadar"
        and 0.5 < point.d_rel < 180.0
        and abs(point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      )
    )

    corners = tuple(
      (point, project_to_model_path(path, point.d_rel, point.y_rel).d_path)
      for point in points
      if (
        point.measured
        and point.source.startswith("corner")
        and 0.5 < point.d_rel < 180.0
        and abs(point.v_lead)
        <= STATIONARY_VISION_CROSS_SOURCE_CORNER_MAX_ABS_VLEAD_MPS
      )
    )
    pairs = tuple(
      (front, front_d_path, corner, corner_d_path)
      for front, front_d_path in fronts
      for corner, corner_d_path in corners
      if (
        abs(front.d_rel - corner.d_rel)
        <= STATIONARY_WEAK_PAIR_MAX_DREL_M
        and abs(front_d_path - corner_d_path)
        <= STATIONARY_WEAK_PAIR_MAX_DPATH_DELTA_M
        and abs(front.v_lead - corner.v_lead)
        <= STATIONARY_WEAK_PAIR_MAX_VLEAD_DELTA_MPS
        and abs(front_d_path)
        <= STATIONARY_WEAK_PAIR_FRONT_MAX_DPATH_M
        and abs(corner_d_path)
        <= STATIONARY_WEAK_PAIR_CORNER_MAX_DPATH_M
      )
    )
    if not pairs:
      self._stationary_weak_pair_identity = None
      self._stationary_weak_pair_last_vision_time_s = None
      return []

    vision_d_path = (
      project_to_model_path(path, vision.d_rel, vision.y_rel).d_path
      if vision is not None
      else math.inf
    )

    def vision_supports_pair(
      pair: tuple[
        RadarPointSnapshot,
        float,
        RadarPointSnapshot,
        float,
      ],
    ) -> bool:
      front, _, corner, _ = pair
      return bool(
        vision is not None
        and vision.probability >= STATIONARY_WEAK_VISION_MIN_PROB
        and abs(vision_d_path) <= STATIONARY_WEAK_VISION_MAX_DPATH_M
        and min(
          abs(vision.d_rel - front.d_rel),
          abs(vision.d_rel - corner.d_rel),
        ) <= STATIONARY_WEAK_VISION_MAX_DISTANCE_ERROR_M
        and min(
          abs(vision.velocity - front.v_lead),
          abs(vision.velocity - corner.v_lead),
        ) <= STATIONARY_WEAK_VISION_MAX_SPEED_DELTA_MPS
      )

    supported_pairs = tuple(
      pair for pair in pairs if vision_supports_pair(pair)
    )
    if supported_pairs:
      selected_pair = min(
        supported_pairs,
        key=lambda pair: (
          abs(pair[1]) + abs(pair[3]),
          abs(pair[0].d_rel - pair[2].d_rel),
          pair[0].track_id,
          pair[2].track_id,
        ),
      )
      pair_identity = (
        selected_pair[0].track_id,
        selected_pair[2].source,
        selected_pair[2].track_id,
      )
      self._stationary_weak_pair_identity = pair_identity
      self._stationary_weak_pair_last_vision_time_s = time_s
    else:
      if (
        self._stationary_weak_pair_identity is None
        or self._stationary_weak_pair_last_vision_time_s is None
        or time_s - self._stationary_weak_pair_last_vision_time_s
        > STATIONARY_WEAK_VISION_PAIR_HOLD_S
      ):
        self._stationary_weak_pair_identity = None
        self._stationary_weak_pair_last_vision_time_s = None
        return []
      selected_pair = next(
        (
          pair for pair in pairs
          if (
            pair[0].track_id,
            pair[2].source,
            pair[2].track_id,
          ) == self._stationary_weak_pair_identity
        ),
        None,
      )
      if selected_pair is None:
        self._stationary_weak_pair_identity = None
        self._stationary_weak_pair_last_vision_time_s = None
        return []

    front, front_d_path, corner, corner_d_path = selected_pair
    range_cost = (
      abs(front.d_rel - corner.d_rel)
      / STATIONARY_WEAK_PAIR_MAX_DREL_M
    )
    return [
      (
        front,
        front_d_path,
        range_cost
        + abs(front_d_path)
        / STATIONARY_WEAK_PAIR_FRONT_MAX_DPATH_M,
      ),
      (
        corner,
        corner_d_path,
        range_cost
        + abs(corner_d_path)
        / STATIONARY_WEAK_PAIR_CORNER_MAX_DPATH_M,
      ),
    ]

  @staticmethod
  def _stationary_vision_path_compatible(
    vision_d_path: float,
    radar_d_path: float,
    held: bool,
  ) -> bool:
    limit = (
      STATIONARY_HELD_MAX_DPATH_M
      if held else STATIONARY_FRESH_MAX_DPATH_M
    )
    return (
      abs(vision_d_path) <= limit
      and abs(vision_d_path - radar_d_path) <= limit
    )

  def _match_stationary(
    self,
    vision: VisionLead | None,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float | None,
    prefer_corner: bool,
    prefer_primary: bool,
    yaw_rate_rad_s: float,
    allowed_output_sources: frozenset[str] | None = None,
  ) -> VisionRadarMatch | None:
    if time_s is None or not math.isfinite(time_s):
      self._reset_stationary()
      return None
    strong_vision = (
      vision is not None
      and vision.probability >= STATIONARY_VISION_MIN_PROB
    )
    vision_d_path = (
      project_to_model_path(path, vision.d_rel, vision.y_rel).d_path
      if strong_vision and vision is not None else None
    )
    point_values = tuple(points)
    needs_cross_source_pair = (
      strong_vision
      or (
        self._stationary_corner_supported
        and (
          self.stationary_identity is not None
          or self._stationary_pending_identity is not None
        )
      )
    )
    front_corner_pairs = (
      self._stationary_front_corner_pairs(point_values, path)
      if needs_cross_source_pair
      else ()
    )
    front_corner_pair_identities = {
      self._identity(front)
      for front, _, _, _ in front_corner_pairs
    }
    cross_source_front_support = (
      self._stationary_vision_cross_source_front_support(
        vision, front_corner_pairs,
      )
    )
    cross_source_front_support_by_identity = {
      self._identity(point): (point, d_path, cost, corner)
      for point, d_path, cost, corner in cross_source_front_support
    }
    held_corner_vision_position_cost: dict[
      tuple[str, int], float
    ] = {}
    eligible_points: list[RadarPointSnapshot] = []
    held_identity_present = any(
      self._identity(point) == self.stationary_identity
      for point in point_values
    )
    for point in point_values:
      identity = self._identity(point)
      continuous_corner_slot = (
        strong_vision
        and self.stationary_identity is not None
        and not held_identity_present
        and self._stationary_corner_slot_continuous(point, time_s)
      )
      held_corner_paired_front = (
        strong_vision
        and self._stationary_corner_supported
        and identity == self.stationary_identity
        and point.source == "frontRadar"
        and abs(point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      )
      held_corner_position_cost = (
        self._stationary_vision_cross_source_position_cost(
          vision, point,
        )
        if (
          strong_vision
          and identity in (
            self.stationary_identity,
            self._stationary_pending_identity,
          )
          and point.source.startswith("corner")
          and abs(point.v_lead)
          <= STATIONARY_HELD_CORNER_MAX_ABS_VLEAD_MPS
        )
        or continuous_corner_slot
        or held_corner_paired_front
        else None
      )
      if held_corner_position_cost is not None:
        held_corner_vision_position_cost[identity] = (
          held_corner_position_cost
        )
      retained_cross_source_pair = (
        identity in front_corner_pair_identities
        and self._stationary_corner_supported
        and identity in (
          self.stationary_identity,
          self._stationary_pending_identity,
        )
        and (
          not strong_vision
          or self._stationary_vision_cross_source_position_cost(
            vision, point,
          ) is not None
        )
      )
      if (
        not 0.5 < point.d_rel < 180.0
        or (
          abs(point.v_lead) > STATIONARY_MAX_ABS_VLEAD_MPS
          and identity not in held_corner_vision_position_cost
        )
      ):
        continue
      if (
        strong_vision
        and vision is not None
        and abs(point.d_rel - vision.d_rel)
        > VISION_RADAR_MAX_DISTANCE_ERROR_M
        and identity not in held_corner_vision_position_cost
        and identity not in cross_source_front_support_by_identity
        and not retained_cross_source_pair
      ):
        # A confident visual range that has separated from the held radar
        # object terminates that radar identity. Radar modes suppress the
        # unmatched central model lead instead of publishing it as blue L1.
        continue
      eligible_points.append(point)

    projection_points = (
      tuple(eligible_points)
      if strong_vision
      else self._stationary_radar_only_initial_projection_points(
        eligible_points, time_s,
      )
    )
    candidate_values: list[tuple[RadarPointSnapshot, float]] = []

    def append_projected_candidate(point: RadarPointSnapshot) -> None:
      identity = self._identity(point)
      strong_vision_support = (
        strong_vision
        and vision is not None
        and self._stationary_vision_base_cost(vision, point) is not None
      )
      d_path = project_to_model_path(path, point.d_rel, point.y_rel).d_path
      maximum_d_path = STATIONARY_FRESH_MAX_DPATH_M
      if identity == self.stationary_identity:
        if (
          point.source == "frontRadar"
          and not strong_vision
          and not self._stationary_corner_supported
        ):
          # A weak visual seed can otherwise keep a roadside front-radar
          # reflector alive for tens of seconds after vision disappears.
          # Preserve centered stopped leads, but require renewed vision (or
          # independent corner support) for an offset front-only identity.
          maximum_d_path = (
            STATIONARY_HELD_FRONT_NO_VISION_MAX_DPATH_M
          )
        else:
          maximum_d_path = (
            STATIONARY_HELD_MAX_DPATH_M
            if (
              self._stationary_seed_probability
              >= STATIONARY_VISION_MIN_PROB
              or strong_vision_support
            )
            else STATIONARY_RADAR_ONLY_HELD_MAX_DPATH_M
          )
      held_vision_path_outlier = (
        identity == self.stationary_identity
        and (
          self._stationary_seed_probability
          >= STATIONARY_VISION_MIN_PROB
          or strong_vision_support
        )
        and vision is not None
        and vision.probability
        >= STATIONARY_VISION_PATH_OUTLIER_MIN_PROB
        and abs(d_path)
        <= STATIONARY_VISION_PATH_OUTLIER_MAX_DPATH_M
        and (
          self._stationary_path_outlier_since_s is None
          or time_s - self._stationary_path_outlier_since_s
          <= STATIONARY_VISION_PATH_OUTLIER_HOLD_S
        )
      )
      if abs(d_path) <= maximum_d_path or held_vision_path_outlier:
        candidate_values.append((point, d_path))

    for point in projection_points:
      append_projected_candidate(point)
    if not strong_vision:
      for point in self._stationary_radar_only_corroborating_front_points(
        eligible_points, candidate_values,
      ):
        # A fresh no-vision front point cannot become stationary leadOne. Its
        # exact projection is needed only to rank a central corner return.
        append_projected_candidate(point)
    if (
      prefer_corner
      and any(
        point.source.startswith("corner")
        for point, _ in candidate_values
      )
    ):
      candidate_values = [
        candidate
        for candidate in candidate_values
        if candidate[0].source.startswith("corner")
      ]

    supported: list[
      tuple[RadarPointSnapshot, float, float]
    ] = []
    weak_pair_supported: list[
      tuple[RadarPointSnapshot, float, float]
    ] = []
    if strong_vision:
      for point, d_path in candidate_values:
        identity = self._identity(point)
        if (
          vision_d_path is None
          or not self._stationary_vision_path_compatible(
            vision_d_path,
            d_path,
            identity == self.stationary_identity,
          )
        ):
          continue
        # On a bend, ego rotation can sweep a stationary divider or roadside
        # return through the model path. Do not let that lone front return
        # borrow a clearly moving visual lead unless their raw lateral
        # positions also agree tightly. Straight-road stationary acquisition
        # and independently paired front/corner support keep the broad speed
        # tolerance needed while a real lead slows.
        turn_fast_vision_lateral_mismatch = (
          vision is not None
          and point.source == "frontRadar"
          and identity not in cross_source_front_support_by_identity
          and abs(yaw_rate_rad_s)
          >= STATIONARY_TURN_FRONT_MIN_ABS_YAW_RATE_RAD_S
          and abs(point.v_lead - vision.velocity)
          > STATIONARY_TURN_FRONT_FAST_VISION_SPEED_DELTA_MPS
          and abs(point.y_rel - vision.y_rel)
          > STATIONARY_TURN_FRONT_FAST_VISION_MAX_YREL_ERROR_M
        )
        if turn_fast_vision_lateral_mismatch:
          continue
        cost = self._stationary_vision_cost(
          vision, point, d_path, prefer_corner,
        )
        if cost is None:
          held_position_cost = held_corner_vision_position_cost.get(
            identity,
          )
          if held_position_cost is not None:
            cost = (
              held_position_cost
              + 0.15 * abs(d_path) / STATIONARY_HELD_MAX_DPATH_M
            )
          else:
            cross_source = cross_source_front_support_by_identity.get(
              identity,
            )
            if cross_source is not None:
              cost = cross_source[2]
        if cost is not None:
          supported.append((point, d_path, cost))
    else:
      supported = self._stationary_radar_only_support(
        candidate_values,
      )
      weak_pair_supported = self._stationary_weak_vision_pair_support(
        vision,
        point_values,
        path,
        time_s,
      )
      if weak_pair_supported:
        supported = weak_pair_supported
      if not weak_pair_supported and any(
        point.source.startswith("corner")
        for point, _, _ in supported
      ):
        supported = [
          candidate
          for candidate in supported
          if candidate[0].source.startswith("corner")
        ]
    corner_supported = any(
      point.source.startswith("corner")
      for point, _, _ in supported
    ) or any(
      self._identity(point) in cross_source_front_support_by_identity
      for point, _, _ in supported
    )
    if allowed_output_sources is not None:
      candidate_values = [
        candidate for candidate in candidate_values
        if candidate[0].source in allowed_output_sources
      ]
      supported = [
        candidate for candidate in supported
        if candidate[0].source in allowed_output_sources
      ]
    if (
      prefer_primary
      and supported
    ):
      preferred_rank = min(
        self._stationary_source_rank(point)
        for point, _, _ in supported
      )
      supported = [
        candidate
        for candidate in supported
        if self._stationary_source_rank(candidate[0])
        == preferred_rank
      ]

    selected: tuple[RadarPointSnapshot, float, float] | None = None
    if self.stationary_identity is not None:
      if corner_supported:
        self._stationary_corner_supported = True
      supported_with_hold = tuple(
        (
          point,
          d_path,
          cost - (
            0.75
            if self._identity(point) == self.stationary_identity
            else 0.0
          ),
        )
        for point, d_path, cost in supported
      )
      if supported_with_hold:
        selected = min(
          supported_with_hold,
          key=lambda candidate: (
            candidate[2],
            candidate[0].track_id,
          ),
        )
      elif (
        self._stationary_last_point is not None
        and self._stationary_last_time_s is not None
      ):
        continuous_candidates = tuple(
          (point, d_path, self._stationary_seed_score)
          for point, d_path in candidate_values
          if (
            self._identity(point) == self.stationary_identity
            or self._stationary_cross_source_continuous(
              point,
              time_s,
            )
          )
          and self._stationary_position_continuous(
            self._stationary_last_point,
            self._stationary_last_time_s,
            point,
            time_s,
          )
        )
        selected = min(
          continuous_candidates,
          key=lambda candidate: (
            self._stationary_source_rank(candidate[0]),
            abs(candidate[0].d_rel - self._stationary_last_point.d_rel),
          ),
          default=None,
        )
      if selected is None:
        dropout_hold = self._stationary_measurement_dropout_hold(
          vision, point_values, path, time_s,
        )
        if dropout_hold is not None:
          return dropout_hold
        self._reset_stationary()
        return None
    else:
      if supported:
        selected = min(
          supported,
          key=lambda candidate: (
            candidate[2]
            - (
              0.75
              if self._identity(candidate[0])
              == self._stationary_pending_identity
              else 0.0
            ),
            candidate[0].track_id,
          ),
        )
      elif (
        self._stationary_pending_identity is not None
        and self._stationary_last_point is not None
        and self._stationary_last_time_s is not None
      ):
        selected = next(
          (
            (
              point,
              d_path,
              self._stationary_seed_score,
            )
            for point, d_path in candidate_values
            if (
              self._identity(point)
              == self._stationary_pending_identity
              and self._stationary_position_continuous(
                self._stationary_last_point,
                self._stationary_last_time_s,
                point,
                time_s,
              )
            )
          ),
          None,
        )
      if selected is None:
        self._reset_stationary()
        return None

      selected_identity = self._identity(selected[0])
      selected_has_current_support = any(
        self._identity(point) == selected_identity
        for point, _, _ in supported
      )
      radar_only_pending_discontinuous = (
        selected_identity == self._stationary_pending_identity
        and selected_has_current_support
        and self._stationary_seed_probability
        < STATIONARY_VISION_MIN_PROB
        and not self._stationary_pending_weak_pair_supported
        and self._stationary_last_point is not None
        and self._stationary_last_time_s is not None
        and not self._stationary_position_continuous(
          self._stationary_last_point,
          self._stationary_last_time_s,
          selected[0],
          time_s,
        )
      )
      if radar_only_pending_discontinuous:
        # A persistent raw corner ID is not sufficient evidence by itself.
        # Roadside reflections can keep the ID while their range jumps by
        # several metres. Radar-only stationary acquisition must therefore
        # maintain the same kinematic continuity as a held lead throughout
        # its confirmation dwell.
        self._reset_stationary()
        return None
      if selected_identity != self._stationary_pending_identity:
        carry_vision_supported_handoff = (
          self._stationary_seed_probability
          >= STATIONARY_VISION_MIN_PROB
          and self._stationary_cross_source_continuous(
            selected[0], time_s,
          )
        )
        self._stationary_pending_identity = selected_identity
        if not carry_vision_supported_handoff:
          self._stationary_pending_since_s = time_s
          self._stationary_pending_vision_support_frames = 0
        if selected_has_current_support:
          self._stationary_pending_vision_support_frames += 1
        self._stationary_seed_probability = (
          max(
            self._stationary_seed_probability,
            vision.probability if vision is not None else 0.0,
          )
          if carry_vision_supported_handoff
          else (vision.probability if vision is not None else 0.0)
        )
        self._stationary_seed_score = selected[2]
        self._stationary_corner_supported = (
          self._stationary_corner_supported or corner_supported
          if carry_vision_supported_handoff
          else corner_supported
        )
        self._stationary_pending_weak_pair_supported = bool(
          weak_pair_supported
          and any(
            self._identity(point) == selected_identity
            for point, _, _ in weak_pair_supported
          )
        )
      elif selected_has_current_support:
        self._stationary_pending_vision_support_frames += 1
        if strong_vision and vision is not None:
          self._stationary_seed_probability = max(
            self._stationary_seed_probability,
            vision.probability,
          )
        self._stationary_corner_supported = (
          self._stationary_corner_supported or corner_supported
        )
        if self._stationary_seed_probability < STATIONARY_VISION_MIN_PROB:
          # The shorter weak-vision dwell is valid only while the corroborating
          # front/corner pair remains present. If it drops out, fall back to
          # the normal radar-only confirmation interval.
          self._stationary_pending_weak_pair_supported = bool(
            weak_pair_supported
          )
      elif (
        self._stationary_seed_probability
        < STATIONARY_VISION_MIN_PROB
      ):
        # Radar-only acquisition requires current central support throughout
        # its longer confirmation dwell. Position
        # continuity alone can otherwise turn a brief roadside reflection
        # into a delayed stationary control lead.
        self._reset_stationary()
        return None
      elif (
        not selected[0].source.startswith("corner")
        and not self._stationary_corner_supported
      ):
        self._stationary_pending_vision_support_frames = 0
      self._stationary_last_point = selected[0]
      self._stationary_last_time_s = time_s
      required_support_frames = (
        1
        if (
          selected[0].source.startswith("corner")
          or self._stationary_corner_supported
        )
        else STATIONARY_FRONT_MIN_VISION_SUPPORT_FRAMES
      )
      if (
        self._stationary_pending_since_s is None
        or time_s - self._stationary_pending_since_s
        < (
          STATIONARY_CONFIRMATION_S
          if self._stationary_seed_probability
          >= STATIONARY_VISION_MIN_PROB
          else (
            STATIONARY_WEAK_VISION_PAIR_CONFIRMATION_S
            if self._stationary_pending_weak_pair_supported
            else STATIONARY_RADAR_ONLY_CONFIRMATION_S
          )
        )
        or self._stationary_pending_vision_support_frames
        < required_support_frames
      ):
        return None
      self.stationary_identity = selected_identity

    point, d_path, score = selected
    selected_identity = self._identity(point)
    selected_has_current_vision_support = (
      strong_vision
      and any(
        self._identity(candidate) == selected_identity
        for candidate, _, _ in supported
      )
    )
    held_front_departing_path = (
      selected_identity == self.stationary_identity
      and point.source == "frontRadar"
      and not selected_has_current_vision_support
      and abs(d_path) > STATIONARY_HELD_FRONT_DEPARTURE_DPATH_M
    )
    if held_front_departing_path:
      if self._stationary_front_departure_since_s is None:
        self._stationary_front_departure_since_s = time_s
      elif (
        time_s - self._stationary_front_departure_since_s
        >= STATIONARY_HELD_FRONT_DEPARTURE_CONFIRMATION_S
      ):
        # A corroborating corner return proves that the object is real, but
        # not that it still occupies our path. Once vision no longer supports
        # the held stopped lead, release a sustained path departure instead
        # of following the physical object until ego is alongside it.
        self._reset_stationary()
        return None
    else:
      self._stationary_front_departure_since_s = None
    if (
      strong_vision
      and vision is not None
      and any(
        self._identity(candidate) == self._identity(point)
        for candidate, _, _ in supported
      )
    ):
      self._stationary_seed_probability = max(
        self._stationary_seed_probability,
        vision.probability,
      )
    if abs(d_path) > STATIONARY_HELD_MAX_DPATH_M:
      if self._stationary_path_outlier_since_s is None:
        self._stationary_path_outlier_since_s = time_s
    else:
      self._stationary_path_outlier_since_s = None
    self.stationary_identity = self._identity(point)
    self._stationary_last_point = point
    self._stationary_last_time_s = time_s
    return VisionRadarMatch(
      point=point,
      probability=(
        vision.probability
        if vision is not None
        else self._stationary_seed_probability
      ),
      score=min(1.0, max(0.0, 1.0 - score)),
      d_path=d_path,
    )

  @staticmethod
  def _radar_only_moving_source_rank(
    point: RadarPointSnapshot,
  ) -> int:
    if point.source == "frontRadar":
      return 0
    if point.source.startswith("corner"):
      return 1
    if point.source == "scc":
      return 2
    return 3

  @staticmethod
  def _radar_only_moving_dpath_limit(d_rel: float) -> float:
    if d_rel > RADAR_ONLY_MOVING_FAR_DREL_M:
      return RADAR_ONLY_MOVING_FAR_DPATH_M
    if d_rel > RADAR_ONLY_MOVING_MID_DREL_M:
      return RADAR_ONLY_MOVING_MID_DPATH_M
    return RADAR_ONLY_MOVING_NEAR_DPATH_M

  def _confirmed_closer_radar_only_moving(
    self,
    candidates: Sequence[
      tuple[RadarPointSnapshot, float, float]
    ],
    held: tuple[RadarPointSnapshot, float, float],
    time_s: float,
  ) -> tuple[RadarPointSnapshot, float, float] | None:
    eligible_challengers = tuple(
      candidate for candidate in candidates
      if (
        candidate[0].source == "frontRadar"
        and candidate[0].track_id != 0
        and abs(candidate[1])
        <= RADAR_ONLY_MOVING_CLOSER_SWITCH_MAX_DPATH_M
      )
    )
    if not eligible_challengers:
      self._reset_radar_only_moving_challenger()
      return None
    challenger = min(
      eligible_challengers,
      key=lambda candidate: (
        self._radar_only_moving_source_rank(candidate[0]),
        candidate[0].d_rel,
        abs(candidate[1]),
      ),
    )
    challenger_identity = self._identity(challenger[0])
    if (
      challenger_identity == self._identity(held[0])
      or held[0].track_id == 0
      or challenger[0].d_rel
      > held[0].d_rel - RADAR_ONLY_MOVING_CLOSER_SWITCH_MIN_GAP_M
    ):
      self._reset_radar_only_moving_challenger()
      return None

    continuous = (
      challenger_identity == self._radar_only_moving_challenger_identity
      and self._radar_only_moving_challenger_last_point is not None
      and self._radar_only_moving_challenger_last_time_s is not None
      and self._stationary_position_continuous(
        self._radar_only_moving_challenger_last_point,
        self._radar_only_moving_challenger_last_time_s,
        challenger[0],
        time_s,
      )
    )
    if not continuous:
      self._radar_only_moving_challenger_identity = challenger_identity
      self._radar_only_moving_challenger_since_s = time_s
    self._radar_only_moving_challenger_last_point = challenger[0]
    self._radar_only_moving_challenger_last_time_s = time_s
    confirmation_s = (
      RADAR_ONLY_MOVING_TENTATIVE_CONFIRMATION_S
      if challenger[0].radar_track_state == 1
      else RADAR_ONLY_MOVING_CONFIRMATION_S
    )
    if (
      self._radar_only_moving_challenger_since_s is None
      or time_s - self._radar_only_moving_challenger_since_s
      < confirmation_s
    ):
      return None
    return challenger

  def _match_radar_only_moving(
    self,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float | None,
  ) -> VisionRadarMatch | None:
    """Track a central moving radar lead independently of visual range."""
    if time_s is None or not math.isfinite(time_s):
      self._reset_radar_only_moving()
      return None

    point_values = tuple(points)
    candidates: list[
      tuple[RadarPointSnapshot, float, float]
    ] = []
    for point in point_values:
      if (
        self._radar_only_moving_source_rank(point) > 2
        or not 0.5 < point.d_rel <= RADAR_ONLY_MOVING_MAX_DREL_M
        or point.v_lead <= RADAR_ONLY_MOVING_MIN_VLEAD_MPS
        or (
          point.d_rel > RADAR_ONLY_MOVING_RECEDING_MAX_DREL_M
          and point.v_rel > RADAR_ONLY_MOVING_RECEDING_VREL_MPS
        )
      ):
        continue
      d_path = project_to_model_path(
        path, point.d_rel, point.y_rel,
      ).d_path
      d_path_limit = self._radar_only_moving_dpath_limit(
        point.d_rel,
      )
      identity = self._identity(point)
      front_supported = (
        point.source.startswith("corner")
        and getattr(
          prefer_front_radar_kinematics(point, point_values),
          "kinematics_source",
          None,
        ) == "frontRadar"
      )
      corner_previously_acquired = identity in (
        self.radar_only_moving_identity,
        self._radar_only_moving_pending_identity,
      )
      if (
        abs(d_path) > d_path_limit
        or abs(d_path - point.y_rel)
        >= RADAR_ONLY_MOVING_MAX_PATH_Y_OFFSET_M
        or self._radar_only_moving_identity_rejected(point, time_s)
        or (
          point.source.startswith("corner")
          and point.d_rel
          < RADAR_ONLY_MOVING_CORNER_MIN_ACQUISITION_DREL_M
          and not front_supported
          and not corner_previously_acquired
        )
      ):
        continue
      candidates.append((point, d_path, d_path_limit))

    if not candidates:
      self._reset_radar_only_moving()
      return None

    if (
      self.radar_only_moving_identity is not None
      and self._radar_only_moving_last_point is not None
      and self._radar_only_moving_last_time_s is not None
    ):
      continuous = tuple(
        candidate
        for candidate in candidates
        if (
          self._identity(candidate[0])
          == self.radar_only_moving_identity
          or candidate[0].source
          != self._radar_only_moving_last_point.source
        )
        and self._stationary_position_continuous(
          self._radar_only_moving_last_point,
          self._radar_only_moving_last_time_s,
          candidate[0],
          time_s,
        )
      )
      if continuous:
        point, d_path, d_path_limit = min(
          continuous,
          key=lambda candidate: (
            self._radar_only_moving_source_rank(candidate[0]),
            candidate[0].d_rel,
            abs(candidate[1]),
          ),
        )
        closer = self._confirmed_closer_radar_only_moving(
          candidates,
          (point, d_path, d_path_limit),
          time_s,
        )
        if closer is not None:
          point, d_path, d_path_limit = closer
          self._reset_radar_only_moving_challenger()
        self._update_radar_only_moving_longitudinal_history(
          point, time_s,
        )
        if not self._radar_only_moving_longitudinally_consistent(
          point, time_s,
        ):
          self._reject_radar_only_moving(point, time_s)
          self._reset_radar_only_moving()
          return None
        self.radar_only_moving_identity = self._identity(point)
        return VisionRadarMatch(
          point=prefer_front_radar_kinematics(
            point, point_values,
          ),
          probability=0.0,
          score=max(
            0.0, 1.0 - abs(d_path) / d_path_limit,
          ),
          d_path=d_path,
        )
      self._reset_radar_only_moving()

    point, d_path, d_path_limit = min(
      candidates,
      key=lambda candidate: (
        self._radar_only_moving_source_rank(candidate[0]),
        candidate[0].d_rel,
        abs(candidate[1]),
      ),
    )
    identity = self._identity(point)
    pending_continuous = (
      identity == self._radar_only_moving_pending_identity
      and self._radar_only_moving_last_point is not None
      and self._radar_only_moving_last_time_s is not None
      and self._stationary_position_continuous(
        self._radar_only_moving_last_point,
        self._radar_only_moving_last_time_s,
        point,
        time_s,
      )
    )
    if not pending_continuous:
      self._radar_only_moving_pending_identity = None
    self._update_radar_only_moving_longitudinal_history(point, time_s)
    if point.radar_track_state == 1:
      confirmation_s = RADAR_ONLY_MOVING_TENTATIVE_CONFIRMATION_S
    elif (
      point.source.startswith("corner")
      and point.d_rel > RADAR_ONLY_MOVING_FAR_DREL_M
    ):
      # A distant corner-only tunnel/overpass return can look like a moving
      # vehicle for the first few cycles. Observe one full consistency window
      # before publishing L1; a mutually visible front point wins source
      # ranking above, and a vision-supported point uses the regular matcher.
      confirmation_s = RADAR_ONLY_MOVING_FAR_CORNER_CONFIRMATION_S
    else:
      confirmation_s = RADAR_ONLY_MOVING_CONFIRMATION_S
    confirmation_complete = (
      self._radar_only_moving_pending_since_s is not None
      and time_s - self._radar_only_moving_pending_since_s
      >= confirmation_s
    )
    longitudinally_consistent = (
      self._radar_only_moving_longitudinally_consistent(
        point, time_s,
      )
    )
    if (
      not confirmation_complete
      or not longitudinally_consistent
    ):
      if confirmation_complete and not longitudinally_consistent:
        self._reject_radar_only_moving(point, time_s)
        self._reset_radar_only_moving()
      return None

    self.radar_only_moving_identity = identity
    return VisionRadarMatch(
      point=prefer_front_radar_kinematics(
        point, point_values,
      ),
      probability=0.0,
      score=max(0.0, 1.0 - abs(d_path) / d_path_limit),
      d_path=d_path,
    )

  def _match_vision_corroborated_radar(
    self,
    vision: VisionLead | None,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float | None,
  ) -> VisionRadarMatch | None:
    """Recover a physical point rejected only by the probabilistic matcher."""
    if (
      vision is None
      or vision.probability < VISION_LEAD_MIN_PROB
    ):
      return None
    vision_d_path = project_to_model_path(
      path, vision.d_rel, vision.y_rel,
    ).d_path
    candidates: list[
      tuple[RadarPointSnapshot, float, float]
    ] = []
    for point in points:
      if (
        not point.measured
        or self._stationary_source_rank(point) > 2
        or not 0.5 < point.d_rel < 180.0
      ):
        continue
      d_path = project_to_model_path(
        path, point.d_rel, point.y_rel,
      ).d_path
      stationary = abs(point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      retained_corner_velocity_outlier = (
        point.source.startswith("corner")
        and self._identity(point) in (
          self.stationary_identity,
          self._stationary_pending_identity,
        )
        and abs(point.v_lead)
        <= STATIONARY_HELD_CORNER_MAX_ABS_VLEAD_MPS
      )
      if (
        abs(point.d_rel - vision.d_rel)
        > VISION_RADAR_MAX_DISTANCE_ERROR_M
        or abs(d_path) > VISION_MATCH_FRESH_MAX_DPATH_M
        or abs(d_path - vision_d_path)
        > VISION_ONLY_CORROBORATION_MAX_DPATH_DELTA_M
        or (
          abs(point.v_lead - vision.velocity)
          > (
            _stationary_vision_speed_delta_limit(point)
            if stationary
            else VISION_ONLY_CORROBORATION_MAX_VLEAD_DELTA_MPS
          )
          and not retained_corner_velocity_outlier
        )
        or (
          stationary
          and not point.source.startswith("corner")
        )
      ):
        continue
      candidates.append((
        point,
        d_path,
        abs(point.d_rel - vision.d_rel),
      ))
    if not candidates:
      return None

    source_rank = min(
      self._stationary_source_rank(point)
      for point, _, _ in candidates
    )
    candidates = [
      candidate
      for candidate in candidates
      if self._stationary_source_rank(candidate[0]) == source_rank
    ]
    point, d_path, distance_error = min(
      candidates,
      key=lambda candidate: (
        candidate[2],
        abs(candidate[1] - vision_d_path),
        candidate[0].track_id,
      ),
    )
    if (
      time_s is not None
      and math.isfinite(time_s)
      and point.source.startswith("corner")
      and abs(point.v_lead - vision.velocity)
      > max(5.0, abs(vision.velocity) * 0.30)
      and time_s
      - self._observed_since_s.get(self._identity(point), time_s)
      < VISION_CORROBORATED_MIN_OBSERVED_S
    ):
      return None
    if abs(point.v_lead) > STATIONARY_MAX_ABS_VLEAD_MPS:
      self.last_identity = self._identity(point)
      self.low_probability_hold_frames = 0
    return VisionRadarMatch(
      point=point,
      probability=vision.probability,
      score=max(
        0.0,
        1.0
        - distance_error / VISION_RADAR_MAX_DISTANCE_ERROR_M,
      ),
      d_path=d_path,
    )

  @staticmethod
  def _far_vision_radar_distance_error_limit(
    vision_d_rel: float,
  ) -> float:
    """Grow the 15 m association gate smoothly beyond 50 m."""
    return min(
      VISION_RADAR_FAR_MAX_DISTANCE_ERROR_M,
      max(
        VISION_RADAR_MAX_DISTANCE_ERROR_M,
        vision_d_rel * VISION_RADAR_FAR_DISTANCE_ERROR_FRACTION,
      ),
    )

  def _match_far_vision_corroborated_radar(
    self,
    vision: VisionLead | None,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float | None,
  ) -> VisionRadarMatch | None:
    """Confirm a distant moving radar return before relaxing visual range."""
    if (
      vision is None
      or time_s is None
      or not math.isfinite(time_s)
      or vision.probability < VISION_RADAR_FAR_MIN_HOLD_PROB
    ):
      self._reset_far_vision_radar()
      return None
    distance_limit = self._far_vision_radar_distance_error_limit(
      vision.d_rel,
    )
    if distance_limit <= VISION_RADAR_MAX_DISTANCE_ERROR_M:
      self._reset_far_vision_radar()
      return None

    vision_d_path = project_to_model_path(
      path, vision.d_rel, vision.y_rel,
    ).d_path
    candidates: list[
      tuple[RadarPointSnapshot, float, float]
    ] = []
    for point in points:
      if (
        not point.measured
        or self._radar_only_moving_source_rank(point) > 2
        or not 0.5 < point.d_rel < 180.0
        or point.v_lead <= STATIONARY_MAX_ABS_VLEAD_MPS
      ):
        continue
      distance_error = abs(point.d_rel - vision.d_rel)
      if (
        distance_error > distance_limit
        or abs(point.v_lead - vision.velocity)
        > VISION_RADAR_FAR_MAX_VLEAD_DELTA_MPS
      ):
        continue
      d_path = project_to_model_path(
        path, point.d_rel, point.y_rel,
      ).d_path
      if (
        abs(d_path) > VISION_RADAR_FAR_MAX_DPATH_M
        or abs(d_path - vision_d_path)
        > VISION_ONLY_CORROBORATION_MAX_DPATH_DELTA_M
      ):
        continue
      candidates.append((point, d_path, distance_error))

    if not candidates:
      self._reset_far_vision_radar()
      return None
    source_rank = min(
      self._radar_only_moving_source_rank(point)
      for point, _, _ in candidates
    )
    point, d_path, distance_error = min(
      (
        candidate for candidate in candidates
        if self._radar_only_moving_source_rank(candidate[0])
        == source_rank
      ),
      key=lambda candidate: (
        candidate[2],
        abs(candidate[1] - vision_d_path),
        candidate[0].track_id,
      ),
    )
    identity = self._identity(point)
    continuous = (
      identity == self._far_vision_radar_identity
      and self._far_vision_radar_last_point is not None
      and self._far_vision_radar_last_time_s is not None
      and self._stationary_position_continuous(
        self._far_vision_radar_last_point,
        self._far_vision_radar_last_time_s,
        point,
        time_s,
      )
    )
    if not continuous:
      if vision.probability < VISION_RADAR_FAR_MIN_SEED_PROB:
        self._reset_far_vision_radar()
        return None
      self._far_vision_radar_identity = identity
      self._far_vision_radar_since_s = time_s
    self._far_vision_radar_last_point = point
    self._far_vision_radar_last_time_s = time_s
    if (
      self._far_vision_radar_since_s is None
      or time_s - self._far_vision_radar_since_s
      < VISION_RADAR_FAR_CONFIRMATION_S
    ):
      return None
    return VisionRadarMatch(
      point=point,
      probability=vision.probability,
      score=max(0.0, 1.0 - distance_error / distance_limit),
      d_path=d_path,
    )

  def _match_moving(
    self,
    vision: VisionLead | None,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
  ) -> VisionRadarMatch | None:
    if vision is None:
      self._reset_moving()
      return None

    high_probability = vision.probability >= VISION_LEAD_MIN_PROB
    holding_previous = (
      not high_probability
      and vision.probability > VISION_LEAD_HOLD_MIN_PROB
      and self.last_identity is not None
      and self.low_probability_hold_frames < VISION_LEAD_HOLD_MAX_FRAMES
    )
    if not high_probability and not holding_previous:
      self._reset_moving()
      return None

    velocity_tolerance = max(
      5.0,
      abs(vision.velocity) * (
        0.3
        + 0.2 * min(max((vision.probability - 0.8) / 0.18, 0.0), 1.0)
      ),
    )
    distance_tolerance = max(
      5.0,
      vision.d_rel * 0.25,
      min(
        VISION_MATCH_XSTD_MAX_M,
        abs(vision.x_std) * VISION_MATCH_XSTD_SIGMA,
      ),
    )
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
      score = (
        _laplacian(point.d_rel, vision.d_rel, vision.x_std)
        * _laplacian(point.y_rel, vision.y_rel, vision.y_std)
        * _laplacian(point.v_lead, vision.velocity, vision.v_std)
      )
      velocity_error = abs(point.v_lead - vision.velocity)
      if (
        (not held_identity and score < VISION_MATCH_FRESH_MIN_SCORE)
        or abs(point.d_rel - vision.d_rel)
        > VISION_RADAR_MAX_DISTANCE_ERROR_M
        or abs(point.d_rel - vision.d_rel)
        >= (
          distance_tolerance
          + (
            VISION_MATCH_DISTANCE_HYSTERESIS_M
            if held_identity
            else 0.0
          )
        )
        or abs(point.y_rel - vision.y_rel) >= 2.0
        or not (
          velocity_error < velocity_tolerance
          or (
            point.v_lead > 3.0
            and velocity_error
            < max(velocity_tolerance * 3.0, 20.0)
          )
        )
      ):
        continue
      projection = project_to_model_path(path, point.d_rel, point.y_rel)
      if abs(projection.d_path) > (
        VISION_MATCH_HELD_MAX_DPATH_M
        if held_identity
        else VISION_MATCH_FRESH_MAX_DPATH_M
      ):
        continue
      candidates.append((point, score, projection.d_path, held_identity))
    if not candidates:
      self._reset_moving()
      return None

    if any(
      point.source == "frontRadar"
      for point, _, _, _ in candidates
    ):
      candidates = [
        candidate for candidate in candidates
        if candidate[0].source == "frontRadar"
      ]

    selected = max(
      candidates,
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

  def _stationary_closer_handoff_ready(
    self,
    stationary: VisionRadarMatch | None,
    moving: VisionRadarMatch | None,
    vision: VisionLead | None,
    time_s: float | None,
  ) -> bool:
    """Confirm a nearer vision-range match before replacing a held radar ID."""
    held_cost = (
      self._stationary_vision_base_cost(vision, stationary.point)
      if stationary is not None
      else None
    )
    challenger_cost = (
      self._stationary_vision_base_cost(vision, moving.point)
      if moving is not None
      else None
    )
    cost_supported = (
      stationary is not None
      and moving is not None
      and abs(stationary.point.y_rel - moving.point.y_rel)
      <= STATIONARY_CLOSER_HANDOFF_MAX_YREL_DELTA_M
      and held_cost is not None
      and challenger_cost is not None
      and challenger_cost + STATIONARY_CLOSER_HANDOFF_MIN_COST_GAIN
      <= held_cost
    )
    vision_range_supported = (
      stationary is not None
      and moving is not None
      and vision is not None
      and abs(stationary.point.y_rel - moving.point.y_rel)
      <= STATIONARY_CLOSER_HANDOFF_RANGE_MAX_YREL_DELTA_M
      and abs(moving.d_path) <= STATIONARY_CLOSER_HANDOFF_MAX_DPATH_M
      and abs(moving.point.y_rel - vision.y_rel)
      <= STATIONARY_CLOSER_HANDOFF_MAX_VISION_YREL_ERROR_M
      and (
        abs(moving.point.d_rel - vision.d_rel)
        + STATIONARY_CLOSER_HANDOFF_MIN_VISION_RANGE_GAIN_M
        <= abs(stationary.point.d_rel - vision.d_rel)
      )
    )
    eligible = (
      stationary is not None
      and moving is not None
      and vision is not None
      and vision.probability
      >= STATIONARY_CLOSER_HANDOFF_MIN_VISION_PROB
      and time_s is not None
      and math.isfinite(time_s)
      and self._identity(stationary.point) != self._identity(moving.point)
      and stationary.point.source == "frontRadar"
      and moving.point.source == stationary.point.source
      and stationary.point.measured
      and moving.point.measured
      and abs(stationary.point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      and abs(moving.point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
      and (
        STATIONARY_CLOSER_HANDOFF_MIN_DREL_GAIN_M
        <= stationary.point.d_rel - moving.point.d_rel
        <= STATIONARY_CLOSER_HANDOFF_MAX_DREL_DELTA_M
      )
      and abs(stationary.point.v_lead - moving.point.v_lead)
      <= STATIONARY_CLOSER_HANDOFF_MAX_VLEAD_DELTA_MPS
      and (cost_supported or vision_range_supported)
    )
    if not eligible or moving is None or time_s is None:
      self._reset_stationary_closer_challenger()
      return False

    identity = self._identity(moving.point)
    continuing = (
      identity == self._stationary_closer_challenger_identity
      and self._stationary_closer_challenger_since_s is not None
      and self._stationary_closer_challenger_last_point is not None
      and self._stationary_closer_challenger_last_time_s is not None
      and self._stationary_position_continuous(
        self._stationary_closer_challenger_last_point,
        self._stationary_closer_challenger_last_time_s,
        moving.point,
        time_s,
      )
    )
    if not continuing:
      self._stationary_closer_challenger_identity = identity
      self._stationary_closer_challenger_since_s = time_s
    self._stationary_closer_challenger_last_point = moving.point
    self._stationary_closer_challenger_last_time_s = time_s
    return (
      self._stationary_closer_challenger_since_s is not None
      and time_s - self._stationary_closer_challenger_since_s
      >= STATIONARY_CLOSER_HANDOFF_CONFIRMATION_S
    )

  def _adopt_stationary_closer_handoff(
    self,
    match: VisionRadarMatch,
    vision: VisionLead,
    time_s: float,
  ) -> None:
    self.stationary_identity = self._identity(match.point)
    self._stationary_pending_identity = None
    self._stationary_pending_since_s = None
    self._stationary_pending_vision_support_frames = 0
    self._stationary_last_point = match.point
    self._stationary_last_time_s = time_s
    self._stationary_seed_probability = vision.probability
    base_cost = self._stationary_vision_base_cost(vision, match.point)
    self._stationary_seed_score = (
      base_cost if base_cost is not None else max(0.0, 1.0 - match.score)
    )
    self._stationary_path_outlier_since_s = None
    self._stationary_front_departure_since_s = None
    self._reset_stationary_closer_challenger()

  def match(
    self,
    model: Any,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    *,
    time_s: float | None = None,
    stationary_points: Iterable[RadarPointSnapshot] | None = None,
    prefer_corner_stationary: bool = False,
    prefer_primary_stationary: bool = False,
    yaw_rate_rad_s: float = 0.0,
    allowed_output_sources: frozenset[str] | None = None,
  ) -> VisionRadarMatch | None:
    vision = vision_lead_from_model(model)
    self._update_vision_fallback(vision)
    point_values = tuple(points)
    stationary_values = (
      point_values
      if stationary_points is None
      else tuple(stationary_points)
    )
    if (
      math.isfinite(yaw_rate_rad_s)
      and abs(yaw_rate_rad_s)
      >= STATIONARY_TURN_MIN_ABS_YAW_RATE_RAD_S
    ):
      # During a tight turn, side objects sweep laterally through the ego frame
      # and can briefly overlap the curved model path. Besides parked objects,
      # reject a far-lateral moving corner target when curve projection alone
      # places it on-path. A strong vision match, or a lead confirmed before
      # the turn, remains eligible.
      stationary_values = tuple(
        point for point in stationary_values
        if not (
          point.source.startswith("corner")
          and self._identity(point) != self.stationary_identity
          and (
            vision is None
            or vision.probability
            < STATIONARY_TURN_CORNER_MIN_VISION_PROB
          )
          and (
            abs(point.v_lead) <= STATIONARY_MAX_ABS_VLEAD_MPS
            or not turning_corner_path_entry_allowed(
              point.source,
              point.y_rel,
              project_to_model_path(
                path, point.d_rel, point.y_rel,
              ).d_path,
              yaw_rate_rad_s,
            )
          )
        )
      )
    if time_s is not None and math.isfinite(time_s):
      current_identities = {
        self._identity(point)
        for point in (*point_values, *stationary_values)
        if point.measured
      }
      for identity in current_identities:
        last_s = self._observed_last_s.get(identity)
        if (
          last_s is None
          or time_s < last_s
          or time_s - last_s
          > VISION_CORROBORATED_MAX_OBSERVATION_GAP_S
        ):
          self._observed_since_s[identity] = time_s
        self._observed_last_s[identity] = time_s
      stale_identities = tuple(
        identity
        for identity, last_s in self._observed_last_s.items()
        if time_s - last_s > VISION_CORROBORATED_MAX_OBSERVATION_GAP_S
      )
      for identity in stale_identities:
        self._observed_since_s.pop(identity, None)
        self._observed_last_s.pop(identity, None)
    stationary = self._match_stationary(
      vision,
      stationary_values,
      path,
      time_s,
      prefer_corner_stationary,
      prefer_primary_stationary,
      yaw_rate_rad_s,
      allowed_output_sources,
    )
    moving = self._match_moving(vision, point_values, path)
    output_values = (
      stationary_values
      if allowed_output_sources is None
      else tuple(
        point for point in stationary_values
        if point.source in allowed_output_sources
      )
    )
    far_corroborated = self._match_far_vision_corroborated_radar(
      vision,
      output_values,
      path,
      time_s,
    )
    if self._stationary_closer_handoff_ready(
      stationary, moving, vision, time_s,
    ):
      assert moving is not None
      assert vision is not None
      assert time_s is not None
      self._adopt_stationary_closer_handoff(moving, vision, time_s)
      stationary = moving
    if (
      stationary is not None
      and moving is not None
      and self._identity(stationary.point) != self._identity(moving.point)
      and not self._stationary_cross_source_equivalent(
        stationary.point, moving.point,
      )
      and abs(moving.point.v_lead) > STATIONARY_MAX_ABS_VLEAD_MPS
    ):
      self._reset_stationary()
      regular = moving
    else:
      regular = stationary if stationary is not None else moving
    radar_moving = self._match_radar_only_moving(
      output_values,
      path,
      time_s,
    )
    if regular is not None:
      # Mirror conventional radard's closer-second-track preference. Once a
      # central moving point has its own physical continuity, do not let a
      # farther point win solely because an erroneous visual range happens to
      # be closer to it. The independently confirmed point is already source
      # ranked and path scoped by _match_radar_only_moving().
      if (
        moving is not None
        and radar_moving is not None
        and radar_moving.point.d_rel < moving.point.d_rel
      ):
        self.last_identity = self._identity(radar_moving.point)
        self.low_probability_hold_frames = 0
        return radar_moving
      return regular
    corroborated = self._match_vision_corroborated_radar(
      vision, output_values, path, time_s,
    )
    if corroborated is None:
      corroborated = far_corroborated
    if (
      radar_moving is not None
      and (
        corroborated is None
        or radar_moving.point.d_rel < corroborated.point.d_rel
      )
    ):
      return radar_moving
    return corroborated


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


def stationary_vision_support_probability(
  vision: VisionLead | None,
  point: RadarPointSnapshot,
) -> float:
  """Return vision confidence for a central slow radar hypothesis."""
  if (
    vision is None
    or abs(point.v_lead - vision.velocity)
    > STATIONARY_MAX_VISION_SPEED_DELTA_MPS
    or VisionRadarMatcher._stationary_vision_cross_source_position_cost(
      vision, point,
    ) is None
  ):
    return 0.0
  return vision.probability


def match_dpath_primary_lead(
  matcher: VisionRadarMatcher,
  model: Any,
  points: Iterable[RadarPointSnapshot],
  path: Sequence[tuple[float, float]],
  *,
  time_s: float,
  enable_radar_tracks: int,
  stationary_points: Iterable[RadarPointSnapshot] | None = None,
  yaw_rate_rad_s: float = 0.0,
) -> VisionRadarMatch | None:
  """Apply dPath vision-present and no-vision primary fallback orders."""
  point_values = tuple(points)
  stationary_values = (
    point_values
    if stationary_points is None
    else tuple(stationary_points)
  )
  stationary_values = select_dpath_fallback_radar_points(
    stationary_values,
    enable_radar_tracks,
  )
  allowed_output_sources = (
    frozenset()
    if enable_radar_tracks <= -2
    else (
      frozenset(("scc",))
      if enable_radar_tracks <= 0
      else (
        PRIMARY_RADAR_SOURCES
        if enable_radar_tracks == 2
        else frozenset(("frontRadar",))
      )
    )
  )
  return matcher.match(
    model,
    select_dpath_primary_radar_points(
      point_values, enable_radar_tracks,
    ),
    path,
    time_s=time_s,
    stationary_points=stationary_values,
    # Corner tracks may corroborate a configured primary, but never own L1.
    prefer_corner_stationary=False,
    prefer_primary_stationary=True,
    yaw_rate_rad_s=yaw_rate_rad_s,
    allowed_output_sources=allowed_output_sources,
  )


def lead_from_vision(
  vision: VisionLead,
  path: Sequence[tuple[float, float]],
  v_ego: float,
  *,
  model_v_ego: float | None = None,
) -> dict[str, Any]:
  """Publish the conventional final vision-only fallback for leadOne."""
  reference_v_ego = (
    float(v_ego)
    if model_v_ego is None or not math.isfinite(float(model_v_ego))
    else float(model_v_ego)
  )
  v_rel = float(vision.velocity) - reference_v_ego
  v_lead = float(v_ego) + v_rel
  d_path = project_to_model_path(
    path, vision.d_rel, vision.y_rel,
  ).d_path
  return {
    "dRel": float(vision.d_rel),
    "yRel": float(vision.y_rel),
    "dPath": float(d_path),
    "vRel": float(v_rel),
    "aRel": 0.0,
    "vLead": float(v_lead),
    "vLeadK": float(v_lead),
    "aLead": float(vision.acceleration),
    "aLeadK": float(vision.acceleration),
    "aLeadTau": 0.3,
    "jLead": 0.0,
    "vLat": 0.0,
    "status": True,
    "fcw": False,
    "modelProb": float(vision.probability),
    "radar": False,
    "radarTrackId": -1,
    "score": 0.0,
  }
