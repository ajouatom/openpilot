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
)


RADAR_TO_CAMERA_M = 1.52
VISION_LEAD_MIN_PROB = 0.50
VISION_LEAD_HOLD_MIN_PROB = 0.35
VISION_LEAD_HOLD_MAX_FRAMES = 10
VISION_MATCH_DISTANCE_HYSTERESIS_M = 2.0
VISION_MATCH_FRESH_MIN_SCORE = 1.0e-4
VISION_MATCH_FRESH_MAX_DPATH_M = 2.0
VISION_MATCH_HELD_MAX_DPATH_M = 4.0
VISION_MATCH_XSTD_SIGMA = 2.5
VISION_MATCH_XSTD_MAX_M = 15.0
PRIMARY_RADAR_SOURCES = frozenset(("frontRadar", "scc"))
LOW_SPEED_SCC_MAX_VLEAD_MPS = 5.0
STATIONARY_VISION_MIN_PROB = 0.40
STATIONARY_FRONT_MIN_VISION_SUPPORT_FRAMES = 3
STATIONARY_CONFIRMATION_S = 0.25
STATIONARY_MAX_ABS_VLEAD_MPS = 2.5
STATIONARY_MAX_VISION_SPEED_DELTA_MPS = 10.0
STATIONARY_FRESH_MAX_DPATH_M = 2.0
STATIONARY_HELD_MAX_DPATH_M = 4.0
STATIONARY_VISION_PATH_OUTLIER_MAX_DPATH_M = 8.0
STATIONARY_VISION_PATH_OUTLIER_MIN_PROB = 0.90
STATIONARY_VISION_PATH_OUTLIER_HOLD_S = 0.20
STATIONARY_LONGITUDINAL_CONTINUITY_M = 2.5
STATIONARY_LATERAL_CONTINUITY_M = 1.5
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
    point_by_identity = {
      (point.source, point.track_id): point
      for point in point_values
    }
    normal_matches: dict[
      tuple[str, int], RadarPointSnapshot
    ] = {}
    for corner in point_values:
      if not corner.source.startswith("corner"):
        continue
      matched = prefer_front_radar_kinematics(corner, point_values)
      if (
        getattr(matched, "kinematics_source", None) == "frontRadar"
        and getattr(matched, "kinematics_track_id", None) is not None
      ):
        front = point_by_identity.get(
          (
            matched.kinematics_source,
            matched.kinematics_track_id,
          ),
        )
        if front is not None:
          normal_matches[(corner.source, corner.track_id)] = front

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
  """Match model lead zero and retain a vision-confirmed stationary radar."""

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

  def reset(self) -> None:
    self._reset_moving()
    self._reset_stationary()

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
  def _stationary_vision_cost(
    vision: VisionLead,
    point: RadarPointSnapshot,
    d_path: float,
    prefer_corner: bool,
  ) -> float | None:
    if (
      vision.probability < STATIONARY_VISION_MIN_PROB
      or abs(point.v_lead - vision.velocity)
      > STATIONARY_MAX_VISION_SPEED_DELTA_MPS
    ):
      return None
    distance_gate = max(
      6.0,
      min(20.0, vision.d_rel * 0.15),
      min(15.0, abs(vision.x_std) * 3.0),
    )
    lateral_gate = max(2.0, min(4.0, abs(vision.y_std) * 3.0))
    distance_error = abs(point.d_rel - vision.d_rel)
    lateral_error = abs(point.y_rel - vision.y_rel)
    if distance_error > distance_gate or lateral_error > lateral_gate:
      return None
    return (
      distance_error / distance_gate
      + lateral_error / lateral_gate
      + 0.15 * abs(d_path) / STATIONARY_FRESH_MAX_DPATH_M
      - (
        0.35
        if prefer_corner and point.source.startswith("corner")
        else 0.0
      )
    )

  def _match_stationary(
    self,
    vision: VisionLead | None,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    time_s: float | None,
    prefer_corner: bool,
    prefer_primary: bool,
  ) -> VisionRadarMatch | None:
    if time_s is None or not math.isfinite(time_s):
      self._reset_stationary()
      return None
    candidate_values: list[tuple[RadarPointSnapshot, float]] = []
    for point in points:
      identity = self._identity(point)
      if (
        not 0.5 < point.d_rel < 180.0
        or abs(point.v_lead) > STATIONARY_MAX_ABS_VLEAD_MPS
      ):
        continue
      d_path = project_to_model_path(path, point.d_rel, point.y_rel).d_path
      maximum_d_path = (
        STATIONARY_HELD_MAX_DPATH_M
        if identity == self.stationary_identity
        else STATIONARY_FRESH_MAX_DPATH_M
      )
      held_vision_path_outlier = (
        identity == self.stationary_identity
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
    if vision is not None:
      for point, d_path in candidate_values:
        cost = self._stationary_vision_cost(
          vision, point, d_path, prefer_corner,
        )
        if cost is not None:
          supported.append((point, d_path, cost))
    if (
      prefer_primary
      and any(
        point.source == "frontRadar"
        for point, _, _ in supported
      )
    ):
      supported = [
        candidate
        for candidate in supported
        if candidate[0].source == "frontRadar"
      ]
    elif (
      prefer_primary
      and any(
        point.source == "scc"
        for point, _, _ in supported
      )
    ):
      supported = [
        candidate
        for candidate in supported
        if candidate[0].source == "scc"
      ]

    selected: tuple[RadarPointSnapshot, float, float] | None = None
    if self.stationary_identity is not None:
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
        selected = next(
          (
            (point, d_path, self._stationary_seed_score)
            for point, d_path in candidate_values
            if (
              self._identity(point) == self.stationary_identity
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
      selected_has_vision_support = any(
        self._identity(point) == selected_identity
        for point, _, _ in supported
      )
      if selected_identity != self._stationary_pending_identity:
        self._stationary_pending_identity = selected_identity
        self._stationary_pending_since_s = time_s
        self._stationary_pending_vision_support_frames = (
          1 if selected_has_vision_support else 0
        )
        self._stationary_seed_probability = (
          vision.probability if vision is not None else 0.0
        )
        self._stationary_seed_score = selected[2]
      elif selected_has_vision_support:
        self._stationary_pending_vision_support_frames += 1
      elif not selected[0].source.startswith("corner"):
        self._stationary_pending_vision_support_frames = 0
      self._stationary_last_point = selected[0]
      self._stationary_last_time_s = time_s
      required_support_frames = (
        1
        if selected[0].source.startswith("corner")
        else STATIONARY_FRONT_MIN_VISION_SUPPORT_FRAMES
      )
      if (
        self._stationary_pending_since_s is None
        or time_s - self._stationary_pending_since_s
        < STATIONARY_CONFIRMATION_S
        or self._stationary_pending_vision_support_frames
        < required_support_frames
      ):
        return None
      self.stationary_identity = selected_identity

    point, d_path, score = selected
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
      score=max(0.0, 1.0 - score),
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

    high_probability = vision.probability > VISION_LEAD_MIN_PROB
    holding_previous = (
      not high_probability
      and vision.probability > VISION_LEAD_HOLD_MIN_PROB
      and self.last_identity is not None
      and self.low_probability_hold_frames < VISION_LEAD_HOLD_MAX_FRAMES
    )
    if not high_probability and not holding_previous:
      self._reset_moving()
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
        (
          candidate[3]
          or candidate[1] >= VISION_MATCH_FRESH_MIN_SCORE
        )
        and
        abs(candidate[0].d_rel - vision.d_rel)
        < (
          max(
            5.0,
            vision.d_rel * 0.25,
            min(
              VISION_MATCH_XSTD_MAX_M,
              abs(vision.x_std) * VISION_MATCH_XSTD_SIGMA,
            ),
          )
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
      self._reset_moving()
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
  ) -> VisionRadarMatch | None:
    vision = vision_lead_from_model(model)
    stationary = self._match_stationary(
      vision,
      stationary_points if stationary_points is not None else points,
      path,
      time_s,
      prefer_corner_stationary,
      prefer_primary_stationary,
    )
    moving = self._match_moving(vision, points, path)
    if (
      stationary is not None
      and moving is not None
      and self._identity(stationary.point) != self._identity(moving.point)
      and abs(moving.point.v_lead) > STATIONARY_MAX_ABS_VLEAD_MPS
    ):
      self._reset_stationary()
      return moving
    return stationary if stationary is not None else moving


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
