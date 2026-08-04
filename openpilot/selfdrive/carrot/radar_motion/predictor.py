#!/usr/bin/env python3
"""Simple physical radar prediction from measured model-path-relative history.

This module is intentionally independent of conventional radard control. The
same physical output is used by shadow replay and the optional independent
dPath RadarD.
"""

from __future__ import annotations

import bisect
import math
from collections import deque
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from functools import lru_cache
from statistics import median
from typing import Any


MOTION_HORIZONS_S = (
  0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0,
)
MOTION_MIN_DREL_M = -5.0
MOTION_MAX_DREL_M = 100.0
NOMINAL_LANE_WIDTH_M = 3.60
IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M = 1.5 * NOMINAL_LANE_WIDTH_M
POSITION_ONLY_MAX_ABS_VLEAD_MPS = 2.5
FRONT_CUT_IN_MIN_DREL_M = 5.0
FRONT_TRACKED_CLOSE_ENTRY_MIN_DREL_M = 2.0
FRONT_TRACKED_CLOSE_ENTRY_MAX_AGE_S = 0.65
FRONT_TRACKED_CLOSE_ENTRY_MIN_DISPLACEMENT_M = 0.40
FRONT_TRACKED_CLOSE_ENTRY_MIN_CONSISTENCY = 0.90
FRONT_TRACKED_CLOSE_ENTRY_MIN_INWARD_SAMPLE_RATIO = 0.70
FRONT_TRACKED_CLOSE_ENTRY_MIN_SHORT_INWARD_RATE_MPS = 0.50
FRONT_TRACKED_CLOSE_ENTRY_MIN_LONG_INWARD_RATE_MPS = 0.20
FRONT_TRACKED_CLOSE_ENTRY_MIN_REPORTED_INWARD_MPS = 0.15
FRONT_TRACKED_CLOSE_ENTRY_MIN_RECENT_MOTION_SUPPORT = 0.90
ADJACENT_OCCLUSION_MIN_DREL_M = 5.0
ADJACENT_OCCLUSION_RANGE_TOLERANCE_M = 1.0
EGO_PATH_HALF_WIDTH_M = 0.90
TARGET_VEHICLE_HALF_WIDTH_M = 0.90
PATH_OVERLAP_HALF_WIDTH_M = EGO_PATH_HALF_WIDTH_M + TARGET_VEHICLE_HALF_WIDTH_M
PATH_PROXIMITY_MARGIN_M = 1.20
CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M = (
  PATH_OVERLAP_HALF_WIDTH_M + PATH_PROXIMITY_MARGIN_M
)
LANE_BOUNDARY_STRADDLE_HALF_WIDTH_M = (
  0.5 * NOMINAL_LANE_WIDTH_M + TARGET_VEHICLE_HALF_WIDTH_M
)
PATH_STATE_HYSTERESIS_M = 0.12
MIN_PREDICTED_PATH_OVERLAP_S = 0.50
FULL_PREDICTED_PATH_OVERLAP_SUPPORT_S = 1.00
CUT_IN_THRESHOLD = 0.50
CUT_OUT_THRESHOLD = 0.50
CORNER_CUT_IN_THRESHOLD = 0.30
FRONT_CUT_IN_THRESHOLD = 0.67
CUT_IN_CONFIRMATION_S = 0.35
CUT_IN_BOUNDARY_HOLD_S = 0.40
URGENT_NEAR_PATH_CONFIRMATION_S = 0.10
URGENT_NEAR_PATH_MAX_DREL_M = 5.0
URGENT_NEAR_PATH_MAX_CLEARANCE_M = 0.45
URGENT_NEAR_PATH_MIN_INWARD_RATE_MPS = 0.10
NEAR_SIDE_DIRECTIONAL_MIN_DREL_M = 0.8
NEAR_SIDE_DIRECTIONAL_MAX_DREL_M = 5.0
NEAR_SIDE_DIRECTIONAL_MAX_CLEARANCE_M = 0.85
NEAR_SIDE_DIRECTIONAL_MIN_DISPLACEMENT_M = 0.20
NEAR_SIDE_DIRECTIONAL_MIN_CONSISTENCY = 0.90
NEAR_SIDE_DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO = 0.75
NEAR_SIDE_DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS = 0.25
NEAR_SIDE_DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS = 0.15
NEAR_SIDE_DIRECTIONAL_MIN_MOTION_SUPPORT = 0.90
LANE_BOUNDARY_ENTRY_MIN_REPORTED_INWARD_MPS = 0.10
LANE_BOUNDARY_ENTRY_MIN_MOTION_CONSISTENCY = 0.50
MAX_HISTORY_S = 2.0
SHORT_HISTORY_S = 0.45
LONG_HISTORY_S = 1.50
REPORTED_VELOCITY_HISTORY_S = 0.10
DIRECTIONAL_HISTORY_S = 0.80
DIRECTIONAL_MIN_SAMPLES = 6
DIRECTIONAL_MIN_INWARD_DISPLACEMENT_M = 0.20
DIRECTIONAL_MIN_CONSISTENCY = 0.75
DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO = 0.65
DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS = 0.15
DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS = 0.10
DIRECTIONAL_SHORT_RATE_BLEND = 0.50
LONG_HORIZON_CORNER_ENTRY_START_S = 3.5
LONG_HORIZON_CORNER_MIN_INWARD_DISPLACEMENT_M = 0.30
LONG_HORIZON_CORNER_MIN_CONSISTENCY = 0.80
LONG_HORIZON_CORNER_MIN_INWARD_SAMPLE_RATIO = 0.75
STRONG_FRONT_CONFIRMATION_CREDIT_S = 0.06
STRONG_FRONT_MIN_INWARD_DISPLACEMENT_M = 0.50
STRONG_FRONT_MIN_CONSISTENCY = 0.80
STRONG_FRONT_MIN_INWARD_SAMPLE_RATIO = 0.80
STRONG_FRONT_MIN_SHORT_INWARD_RATE_MPS = 0.35
STRONG_FRONT_MIN_LONG_INWARD_RATE_MPS = 0.25
STRONG_FRONT_MIN_MOTION_SUPPORT = 0.90
POSITION_HISTORY_OVERRIDE_MIN_INWARD_DISPLACEMENT_M = 0.45
POSITION_HISTORY_OVERRIDE_MIN_CONSISTENCY = 0.95
POSITION_HISTORY_OVERRIDE_MIN_INWARD_SAMPLE_RATIO = 0.90
POSITION_HISTORY_OVERRIDE_MIN_SHORT_INWARD_RATE_MPS = 0.35
POSITION_HISTORY_OVERRIDE_MIN_LONG_INWARD_RATE_MPS = 0.20
POSITION_HISTORY_OVERRIDE_MIN_RAW_LATERAL_SUPPORT_MPS = 0.15


@dataclass(frozen=True)
class RadarMotionSensitivity:
  level: int
  cut_in_enabled: bool
  cut_in_threshold: float
  confirmation_s: float
  directional_min_consistency: float


_CORNER_CUT_IN_THRESHOLDS = (0.0, 0.42, 0.36, 0.30, 0.25, 0.20)
_FRONT_CUT_IN_THRESHOLDS = (0.0, 0.78, 0.72, 0.67, 0.60, 0.53)
_CUT_IN_CONFIRMATION_TIMES_S = (0.0, 0.50, 0.40, 0.35, 0.25, 0.20)
_DIRECTIONAL_MIN_CONSISTENCIES = (1.0, 0.82, 0.79, 0.75, 0.71, 0.68)


def radar_motion_sensitivity(
  level: int,
  sensor: str,
) -> RadarMotionSensitivity:
  """Return the Carrot Radar CUT-IN policy; level 3 is the current behavior."""
  clamped_level = max(0, min(5, int(level)))
  thresholds = (
    _CORNER_CUT_IN_THRESHOLDS
    if str(sensor) == "corner"
    else _FRONT_CUT_IN_THRESHOLDS
  )
  return RadarMotionSensitivity(
    level=clamped_level,
    cut_in_enabled=clamped_level > 0,
    cut_in_threshold=thresholds[clamped_level],
    confirmation_s=_CUT_IN_CONFIRMATION_TIMES_S[clamped_level],
    directional_min_consistency=(
      _DIRECTIONAL_MIN_CONSISTENCIES[clamped_level]
    ),
  )


@dataclass(frozen=True)
class _SourceConfig:
  missing_hold_s: float
  longitudinal_jump_m: float
  lateral_jump_m: float
  velocity_jump_mps: float
  base_lateral_sigma_m: float
  base_longitudinal_sigma_m: float
  minimum_rate_samples: int
  minimum_path_span_m: float
  use_reported_normal_velocity: bool
  normal_velocity_sigma_mps: float


SOURCE_CONFIGS = {
  "front": _SourceConfig(
    0.35, 1.75, 0.85, 5.0, 0.20, 0.35, 4, 1.5, False, 0.50,
  ),
  "corner": _SourceConfig(
    0.55, 2.25, 1.25, 7.0, 0.32, 0.55, 3, 2.0, True, 0.25,
  ),
}


@dataclass(frozen=True)
class RadarMotionHistorySample:
  time_s: float
  age_s: float
  d_rel: float
  path_x: float
  actual_x: float
  actual_y: float
  d_path: float


@dataclass(frozen=True)
class RadarMotionSample:
  horizon_s: float
  d_rel: float
  path_x: float
  y_rel: float
  d_path: float
  longitudinal_sigma: float
  lateral_sigma: float
  occupancy_prob: float
  path_proximity_score: float


@dataclass(frozen=True)
class RadarMotionPrediction:
  track_id: int
  source: str
  sensor: str
  continuity_id: int
  d_path: float
  d_path_rate_short: float
  d_path_rate_long: float
  d_path_curvature: float
  path_speed_short: float
  path_speed_long: float
  path_speed: float
  vector_heading_deg: float
  reported_heading_deg: float
  reported_normal_speed: float
  normal_speed_disagreement: float
  motion_consistency: float
  recent_motion_support: float
  uncertainty: float
  lane_half_width: float
  current_path_occupancy: bool
  cut_in_detection_allowed: bool
  cut_in_probability: float
  cut_out_probability: float
  path_entry_probability: float
  path_entry_age_s: float | None
  samples: tuple[RadarMotionSample, ...]
  history: tuple[RadarMotionHistorySample, ...]
  history_count: int
  time_to_entry_s: float | None
  reason: str
  predicted_path_overlap_s: float = 0.0
  predicted_path_overlap_start_s: float | None = None
  directional_inward_displacement_m: float = 0.0
  directional_consistency: float = 0.0
  directional_inward_sample_ratio: float = 0.0
  near_side_directional_entry: bool = False
  lane_boundary_directional_entry: bool = False
  front_tracked_close_entry: bool = False

  @property
  def probability(self) -> float:
    return self.cut_in_probability

  @property
  def path_exit_probability(self) -> float:
    return self.cut_out_probability


@dataclass(frozen=True)
class RadarMotionCutIn:
  prediction: RadarMotionPrediction
  score: float


@dataclass(frozen=True)
class RadarMotionDecision:
  confirmed: tuple[RadarMotionCutIn, ...]


@dataclass(frozen=True)
class ModelPathProjection:
  path_s: float
  center_x: float
  center_y: float
  tangent_x: float
  tangent_y: float
  d_path: float


@dataclass(frozen=True)
class _Observation:
  time_s: float
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  v_lead: float
  a_lead: float
  yv_rel: float
  v_ego: float
  ego_distance: float
  ego_x: float
  ego_y: float
  ego_heading: float
  ego_path_s: float
  path_s: float
  path_x_world: float
  path_velocity: float
  normal_velocity: float
  actual_world_x: float
  actual_world_y: float
  d_path: float


@dataclass
class _TrackState:
  source: str
  continuity_id: int
  observations: deque[_Observation] = field(default_factory=deque)
  inside_latched: bool = False
  entry_time_s: float | None = None
  last_seen_s: float = 0.0


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def radar_target_velocity_in_ego_frame(
  v_lead: float,
  yv_rel: float,
  d_rel: float,
  y_rel: float,
  yaw_rate_rad_s: float,
) -> tuple[float, float]:
  """Remove ego-frame rotation from radar-reported target velocity.

  dPath already compares a same-frame radar point and model path and must not
  receive another yaw correction. Radar velocity support is different:
  ``vLead`` and ``yvRel`` are derivatives in the rotating ego frame, so remove
  that rotation before projecting target velocity onto the path tangent and
  normal.
  """
  return (
    _finite(v_lead) - _finite(yaw_rate_rad_s) * _finite(y_rel),
    _finite(yv_rel) + _finite(yaw_rate_rad_s) * _finite(d_rel),
  )


def radar_motion_cut_in_threshold(sensor: str, sensitivity: int = 3) -> float:
  return radar_motion_sensitivity(sensitivity, sensor).cut_in_threshold


def _sensor(source: str) -> str:
  return "corner" if source.startswith("corner") else "front"


def _source(point: Any) -> str:
  return str(getattr(point, "source", getattr(point, "radarSource", "frontRadar")))


def _value(point: Any, snake: str, camel: str, fallback: float = 0.0) -> float:
  return _finite(getattr(point, snake, getattr(point, camel, fallback)), fallback)


def model_path_y(path: Sequence[tuple[float, float]], d_rel: float) -> float:
  """Interpolate model path in radar coordinates (positive left)."""
  if not path:
    raise ValueError("model path is required for dPath prediction")
  if len(path) == 1:
    return -_finite(path[0][1])
  xs = tuple(_finite(point[0]) for point in path)
  index = bisect.bisect_left(xs, d_rel)
  if index <= 0:
    return -_finite(path[0][1])
  if index >= len(path):
    return -_finite(path[-1][1])
  x0, y0 = (_finite(value) for value in path[index - 1])
  x1, y1 = (_finite(value) for value in path[index])
  ratio = 0.0 if abs(x1 - x0) < 1e-6 else (d_rel - x0) / (x1 - x0)
  return -(y0 + (y1 - y0) * ratio)


def _path_key(
  path: Sequence[tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
  if isinstance(path, tuple):
    try:
      hash(path)
    except TypeError:
      pass
    else:
      return path
  return tuple((_finite(x), _finite(y)) for x, y in path)


@lru_cache(maxsize=8)
def _path_geometry(
  path: tuple[tuple[float, float], ...],
) -> tuple[
  tuple[tuple[float, float], ...],
  tuple[tuple[float, float, float, float, float, float], ...],
]:
  """Build immutable radar-path segments once per model frame."""
  points = tuple((_finite(x), -_finite(y)) for x, y in path)
  segments: list[tuple[float, float, float, float, float, float]] = []
  accumulated_s = 0.0
  for (x0, y0), (x1, y1) in zip(points, points[1:], strict=False):
    dx = x1 - x0
    dy = y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-6:
      continue
    segments.append((
      x0,
      y0,
      dx / length,
      dy / length,
      length,
      accumulated_s,
    ))
    accumulated_s += length
  return points, tuple(segments)


def _radar_path(
  path: Sequence[tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
  return _path_geometry(_path_key(path))[0]


@lru_cache(maxsize=256)
def _project_to_model_path_cached(
  path: tuple[tuple[float, float], ...],
  x: float,
  y: float,
) -> ModelPathProjection:
  points, segments = _path_geometry(path)
  if not points:
    raise ValueError("model path is required for dPath prediction")
  if not segments:
    center_x, center_y = points[0]
    return ModelPathProjection(
      x - center_x,
      center_x,
      center_y,
      1.0,
      0.0,
      y - center_y,
    )

  best_values: tuple[
    float, float, float, float, float, float,
  ] | None = None
  best_distance_sq = math.inf
  for (
    x0,
    y0,
    tangent_x,
    tangent_y,
    length,
    accumulated_s,
  ) in segments:
    dx = tangent_x * length
    dy = tangent_y * length
    raw_ratio = ((x - x0) * dx + (y - y0) * dy) / (length * length)
    ratio = min(1.0, max(0.0, raw_ratio))
    center_x = x0 + ratio * dx
    center_y = y0 + ratio * dy
    offset_x = x - center_x
    offset_y = y - center_y
    distance_sq = offset_x * offset_x + offset_y * offset_y
    if distance_sq < best_distance_sq:
      best_distance_sq = distance_sq
      best_values = (
        accumulated_s + ratio * length,
        center_x,
        center_y,
        tangent_x,
        tangent_y,
        -tangent_y * offset_x + tangent_x * offset_y,
      )
  assert best_values is not None
  return ModelPathProjection(*best_values)


def project_to_model_path(
  path: Sequence[tuple[float, float]],
  x: float,
  y: float,
) -> ModelPathProjection:
  """Project an ego-frame radar point onto the model-path centerline."""
  # Projection stays clamped to the measured polyline. The immutable model
  # path and repeated point queries are shared within one model frame.
  return _project_to_model_path_cached(
    _path_key(path),
    _finite(x),
    _finite(y),
  )


def _model_path_point_at_s(
  path_key: tuple[tuple[float, float], ...],
  path_s: float,
  d_path: float,
) -> tuple[float, float]:
  points, usable = _path_geometry(path_key)
  if not points:
    raise ValueError("model path is required for dPath prediction")
  if not usable:
    return points[0][0] + path_s, points[0][1] + d_path

  for index, (
    x0,
    y0,
    tangent_x,
    tangent_y,
    length,
    accumulated_s,
  ) in enumerate(usable):
    segment_s = path_s - accumulated_s
    if segment_s <= length or index == len(usable) - 1:
      if index > 0:
        segment_s = max(0.0, segment_s)
      center_x = x0 + segment_s * tangent_x
      center_y = y0 + segment_s * tangent_y
      return (
        center_x - tangent_y * d_path,
        center_y + tangent_x * d_path,
      )
  return points[-1]


def model_path_point_at_s(
  path: Sequence[tuple[float, float]],
  path_s: float,
  d_path: float = 0.0,
) -> tuple[float, float]:
  """Convert model-path arc distance and normal offset to ego-frame x/y."""
  return _model_path_point_at_s(_path_key(path), path_s, d_path)


def _linear_fit(observations: Sequence[_Observation], field_name: str) -> tuple[float, float]:
  count = len(observations)
  if count < 2:
    return 0.0, 0.0
  time_total = 0.0
  value_total = 0.0
  for observation in observations:
    time_total += observation.time_s
    value_total += float(getattr(observation, field_name))
  time_mean = time_total / count
  value_mean = value_total / count
  denominator = 0.0
  for observation in observations:
    denominator += (observation.time_s - time_mean) ** 2
  if denominator < 1e-6:
    return 0.0, 0.0
  numerator = 0.0
  for observation in observations:
    numerator += (
      (observation.time_s - time_mean)
      * (float(getattr(observation, field_name)) - value_mean)
    )
  slope = numerator / denominator
  intercept = value_mean - slope * time_mean
  residual_sum = 0.0
  for observation in observations:
    residual_sum += (
      float(getattr(observation, field_name))
      - (intercept + slope * observation.time_s)
    ) ** 2
  residual = math.sqrt(residual_sum / count)
  return slope, residual


def _robust_center_and_sigma(values: Sequence[float]) -> tuple[float, float]:
  if not values:
    return 0.0, 0.0
  center = median(values)
  absolute_deviations = tuple(abs(value - center) for value in values)
  return center, 1.4826 * median(absolute_deviations)


def _spatial_fit(
  observations: Sequence[_Observation],
) -> tuple[float, float, float]:
  """Fit dPath against target path progress, not elapsed ego time."""
  count = len(observations)
  if count < 2:
    return 0.0, 0.0, 0.0
  x_total = 0.0
  y_total = 0.0
  x_min = math.inf
  x_max = -math.inf
  for observation in observations:
    x = observation.path_x_world
    x_total += x
    y_total += observation.d_path
    x_min = min(x_min, x)
    x_max = max(x_max, x)
  x_mean = x_total / count
  y_mean = y_total / count
  denominator = 0.0
  for observation in observations:
    denominator += (observation.path_x_world - x_mean) ** 2
  span = x_max - x_min
  if denominator < 1e-6:
    return 0.0, 0.0, span
  numerator = 0.0
  for observation in observations:
    numerator += (
      (observation.path_x_world - x_mean)
      * (observation.d_path - y_mean)
    )
  slope = numerator / denominator
  intercept = y_mean - slope * x_mean
  residual_sum = 0.0
  for observation in observations:
    residual_sum += (
      observation.d_path
      - (intercept + slope * observation.path_x_world)
    ) ** 2
  residual = math.sqrt(residual_sum / count)
  return slope, residual, span


def _window(observations: Sequence[_Observation], duration_s: float) -> tuple[_Observation, ...]:
  if not observations:
    return ()
  cutoff = observations[-1].time_s - duration_s
  return tuple(observation for observation in observations if observation.time_s >= cutoff)


def _directional_history_metrics(
  observations: Sequence[_Observation],
  current_d_path: float,
) -> tuple[float, float, float]:
  """Measure one-way inward progress without smoothing measured positions."""
  directional = _window(observations, DIRECTIONAL_HISTORY_S)
  if (
    len(directional) < DIRECTIONAL_MIN_SAMPLES
    or abs(current_d_path) <= 1e-6
  ):
    return 0.0, 0.0, 0.0
  side = math.copysign(1.0, current_d_path)
  inward_steps = tuple(
    -side * (current.d_path - previous.d_path)
    for previous, current in zip(
      directional[:-1],
      directional[1:],
      strict=True,
    )
  )
  total_travel = sum(abs(step) for step in inward_steps)
  net_inward = sum(inward_steps)
  consistency = (
    max(0.0, min(1.0, net_inward / total_travel))
    if total_travel > 1e-6
    else 0.0
  )
  inward_sample_ratio = (
    sum(step > 0.0 for step in inward_steps) / len(inward_steps)
    if inward_steps
    else 0.0
  )
  return max(0.0, net_inward), consistency, inward_sample_ratio


def _strong_hidden_dpath_entry(
  observations: Sequence[_Observation],
  current_d_path: float,
) -> bool:
  """Expose an occluded point only after strong measured inward history."""
  if abs(current_d_path) >= CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M:
    return False
  (
    inward_displacement,
    consistency,
    inward_sample_ratio,
  ) = _directional_history_metrics(observations, current_d_path)
  reported_lateral_speed, _ = _robust_center_and_sigma(tuple(
    observation.yv_rel
    for observation in _window(observations, REPORTED_VELOCITY_HISTORY_S)
  ))
  side = (
    math.copysign(1.0, current_d_path)
    if abs(current_d_path) > 1e-6
    else 0.0
  )
  return (
    inward_displacement
    >= POSITION_HISTORY_OVERRIDE_MIN_INWARD_DISPLACEMENT_M
    and consistency >= POSITION_HISTORY_OVERRIDE_MIN_CONSISTENCY
    and inward_sample_ratio
    >= POSITION_HISTORY_OVERRIDE_MIN_INWARD_SAMPLE_RATIO
    and -side * reported_lateral_speed
    >= POSITION_HISTORY_OVERRIDE_MIN_RAW_LATERAL_SUPPORT_MPS
  )


def _scoped_motion_points(
  points: Iterable[Any],
  path: Sequence[tuple[float, float]],
) -> tuple[tuple[Any, float, ModelPathProjection], ...]:
  if not path:
    return ()
  path_key = _path_key(path)
  scoped: list[tuple[Any, float, ModelPathProjection]] = []
  for point in points:
    if not bool(getattr(point, "measured", False)):
      continue
    d_rel = _value(point, "d_rel", "dRel")
    if not MOTION_MIN_DREL_M <= d_rel <= MOTION_MAX_DREL_M:
      continue
    y_rel = _value(point, "y_rel", "yRel")
    projection = _project_to_model_path_cached(
      path_key, _finite(d_rel), _finite(y_rel),
    )
    distance_to_path = math.hypot(
      d_rel - projection.center_x,
      y_rel - projection.center_y,
    )
    if distance_to_path <= IMMEDIATE_LANE_SCOPE_HALF_WIDTH_M:
      scoped.append((point, d_rel, projection))
  return tuple(scoped)


def _visible_scoped_motion_points(
  scoped: Sequence[tuple[Any, float, ModelPathProjection]],
  lead_one_d_rel: float | None = None,
  protected_identities: Iterable[tuple[str, int]] = (),
) -> tuple[Any, ...]:
  protected = set(protected_identities)
  overlap_visibility_limit = (
    PATH_OVERLAP_HALF_WIDTH_M + PATH_STATE_HYSTERESIS_M
  )
  primary_limit = (
    float(lead_one_d_rel)
    if lead_one_d_rel is not None
    and math.isfinite(float(lead_one_d_rel))
    and float(lead_one_d_rel) > 0.0
    else None
  )

  nearest: dict[int, float] = {}
  for _, d_rel, projection in scoped:
    d_path = projection.d_path
    if (
      d_rel < ADJACENT_OCCLUSION_MIN_DREL_M
      or abs(d_path) <= overlap_visibility_limit
    ):
      continue
    side = 1 if d_path > 0.0 else -1
    nearest[side] = min(nearest.get(side, math.inf), d_rel)

  return tuple(
    point
    for point, d_rel, projection in scoped
    if (
      d_rel < ADJACENT_OCCLUSION_MIN_DREL_M
      or abs(projection.d_path) <= overlap_visibility_limit
      or (
        (
          str(getattr(point, "source", "")),
          int(getattr(point, "track_id", getattr(point, "trackId", -1))),
        )
        in protected
      )
      or (
        primary_limit is not None
        and d_rel < primary_limit
      )
      or d_rel <= nearest.get(
        1 if projection.d_path > 0.0 else -1, math.inf,
      )
      + ADJACENT_OCCLUSION_RANGE_TOLERANCE_M
    )
  )


def visible_motion_points(
  points: Iterable[Any],
  path: Sequence[tuple[float, float]],
  lead_one_d_rel: float | None = None,
  protected_identities: Iterable[tuple[str, int]] = (),
) -> tuple[Any, ...]:
  """Keep relevant adjacent points without hiding an already selected target."""
  return _visible_scoped_motion_points(
    _scoped_motion_points(points, path),
    lead_one_d_rel,
    protected_identities,
  )


def _normal_interval_probability(mean: float, sigma: float, limit: float) -> float:
  sigma = max(sigma, 0.05)
  scale = sigma * math.sqrt(2.0)
  upper = math.erf((limit - mean) / scale)
  lower = math.erf((-limit - mean) / scale)
  return min(1.0, max(0.0, 0.5 * (upper - lower)))


def _path_proximity_score(d_path: float) -> float:
  """Normalize remaining body-to-corridor clearance over a physical margin."""
  clearance = max(0.0, abs(d_path) - PATH_OVERLAP_HALF_WIDTH_M)
  return min(
    1.0,
    max(0.0, 1.0 - clearance / PATH_PROXIMITY_MARGIN_M),
  )


def _inward_motion_support(
  current_d_path: float,
  sample: RadarMotionSample,
  uncertainty: float,
) -> float:
  """Require predicted inward displacement to exceed measured path noise."""
  inward_displacement = max(
    0.0, abs(current_d_path) - abs(sample.d_path),
  )
  return min(
    1.0,
    inward_displacement / max(uncertainty, 0.10),
  )


def _continuous_path_overlap(
  samples: Sequence[RadarMotionSample],
) -> tuple[float | None, float, tuple[RadarMotionSample, ...]]:
  """Return the longest continuously predicted body/corridor overlap."""
  best: tuple[RadarMotionSample, ...] = ()
  current: list[RadarMotionSample] = []
  for sample in samples:
    if (
      sample.d_rel > 0.0
      and abs(sample.d_path)
      <= PATH_OVERLAP_HALF_WIDTH_M + PATH_STATE_HYSTERESIS_M
    ):
      current.append(sample)
      if (
        len(current) >= 2
        and (
          not best
          or current[-1].horizon_s - current[0].horizon_s
          > best[-1].horizon_s - best[0].horizon_s
        )
      ):
        best = tuple(current)
    else:
      current.clear()

  if len(best) < 2:
    return None, 0.0, ()
  return (
    best[0].horizon_s,
    best[-1].horizon_s - best[0].horizon_s,
    best,
  )


def prediction_sample_at(
  prediction: RadarMotionPrediction,
  horizon_s: float,
) -> RadarMotionSample:
  return min(
    prediction.samples,
    key=lambda sample: abs(sample.horizon_s - horizon_s),
  )


def cutin_probability_at(
  prediction: RadarMotionPrediction,
  horizon_s: float,
) -> float:
  if (
    prediction.current_path_occupancy
    or prediction.predicted_path_overlap_s
    < MIN_PREDICTED_PATH_OVERLAP_S
  ):
    return 0.0
  sample = prediction_sample_at(prediction, horizon_s)
  return (
    max(sample.occupancy_prob, sample.path_proximity_score)
    * _inward_motion_support(
      prediction.d_path,
      sample,
      prediction.uncertainty,
    )
    * prediction.motion_consistency
    * prediction.recent_motion_support
    if (
      abs(sample.d_path) < abs(prediction.d_path)
      and sample.d_rel > 0.0
      and abs(sample.d_path)
      <= PATH_OVERLAP_HALF_WIDTH_M + PATH_STATE_HYSTERESIS_M
    )
    else 0.0
  )


def is_review_candidate(
  prediction: RadarMotionPrediction,
  _d_rel: float,
  horizon_s: float,
  probability_threshold: float,
) -> bool:
  return (
    prediction.history_count >= SOURCE_CONFIGS[prediction.sensor].minimum_rate_samples
    and cutin_probability_at(prediction, horizon_s) >= probability_threshold
  )


class RadarMotionPredictor:
  """Maintain independent front/corner dPath histories and predict path overlap."""

  def __init__(
    self,
    directional_min_consistency: float = DIRECTIONAL_MIN_CONSISTENCY,
  ) -> None:
    self.directional_min_consistency = float(
      directional_min_consistency,
    )
    self._states: dict[str, dict[tuple[str, int], _TrackState]] = {
      "front": {},
      "corner": {},
    }
    self._next_continuity_id = 1
    self._ego_distance_m = 0.0
    self._ego_x_m = 0.0
    self._ego_y_m = 0.0
    self._ego_heading_rad = 0.0
    self._last_update_s: float | None = None
    self._last_v_ego = 0.0
    self._last_yaw_rate = 0.0

  def _new_state(self, source: str, time_s: float) -> _TrackState:
    state = _TrackState(source, self._next_continuity_id, last_seen_s=time_s)
    self._next_continuity_id += 1
    return state

  @staticmethod
  def _continuous(
    state: _TrackState,
    observation: _Observation,
    config: _SourceConfig,
  ) -> bool:
    if not state.observations:
      return True
    previous = state.observations[-1]
    dt = observation.time_s - previous.time_s
    if dt <= 0.0 or dt > config.missing_hold_s:
      return False
    recent = _window(tuple(state.observations), SHORT_HISTORY_S)
    path_slope, _, _ = _spatial_fit(recent)
    d_path_rate = path_slope * previous.path_velocity
    predicted_path_x_world = (
      previous.path_x_world + previous.path_velocity * dt
    )
    predicted_d_path = previous.d_path + d_path_rate * dt
    longitudinal_limit = (
      config.longitudinal_jump_m
      + 0.25 * abs(previous.path_velocity) * dt
    )
    lateral_limit = config.lateral_jump_m + 0.25 * abs(d_path_rate) * dt
    return (
      abs(observation.path_x_world - predicted_path_x_world)
      <= longitudinal_limit
      and abs(observation.d_path - predicted_d_path) <= lateral_limit
      and abs(observation.v_rel - previous.v_rel) <= config.velocity_jump_mps
    )

  def _prediction(
    self,
    state: _TrackState,
    track_id: int,
    observation: _Observation,
    path: Sequence[tuple[float, float]],
    config: _SourceConfig,
  ) -> RadarMotionPrediction:
    observations = tuple(state.observations)
    short = _window(observations, SHORT_HISTORY_S)
    long = _window(observations, LONG_HISTORY_S)
    reported_velocity_window = _window(
      observations, REPORTED_VELOCITY_HISTORY_S,
    )
    short_slope, short_residual, short_path_span = _spatial_fit(short)
    long_slope, long_residual, long_path_span = _spatial_fit(long)
    (
      directional_inward_displacement,
      directional_consistency,
      directional_inward_sample_ratio,
    ) = _directional_history_metrics(observations, observation.d_path)
    _, d_rel_residual = _linear_fit(long, "d_rel")
    path_speed_short, path_x_short_residual = _linear_fit(short, "path_x_world")
    path_speed_long, path_x_long_residual = _linear_fit(long, "path_x_world")
    reported_normal_speed, reported_normal_sigma = _robust_center_and_sigma(
      tuple(
        value.normal_velocity
        for value in reported_velocity_window
      ),
    )
    reported_lateral_speed, _ = _robust_center_and_sigma(
      tuple(value.yv_rel for value in reported_velocity_window),
    )
    reported_path_speed, _ = _robust_center_and_sigma(
      tuple(value.path_velocity for value in long),
    )
    enough_history = (
      len(long) >= config.minimum_rate_samples
      and long_path_span >= config.minimum_path_span_m
    )
    if len(long) >= 2:
      history_path_speed = path_speed_long
      path_speed = 0.70 * history_path_speed + 0.30 * reported_path_speed
    else:
      path_speed = observation.path_velocity
      path_speed_short = observation.path_velocity
      path_speed_long = observation.path_velocity
    if not enough_history:
      short_slope = 0.0
      long_slope = 0.0
    short_rate = short_slope * path_speed
    long_rate = long_slope * path_speed
    curvature_span = max(1.0, long_path_span - 0.5 * short_path_span)
    curvature = max(-0.5, min(0.5, (short_slope - long_slope) / curvature_span))
    # The long-window 2-D path vector supplies the position-derived motion.
    # Corner radar must support that direction with its current measured
    # normal velocity. Use the smaller agreed magnitude so an old trend is not
    # extrapolated after measured lateral motion stops. A stronger same-sign
    # radar velocity is support, not a disagreement.
    position_rate = long_slope * path_speed
    rate = position_rate
    side = (
      math.copysign(1.0, observation.d_path)
      if abs(observation.d_path) > 1e-6
      else 0.0
    )
    inward_short = -side * short_rate
    inward_long = -side * long_rate
    position_history_override = (
      _sensor(state.source) == "corner"
      and enough_history
      and directional_inward_displacement
      >= POSITION_HISTORY_OVERRIDE_MIN_INWARD_DISPLACEMENT_M
      and directional_consistency
      >= POSITION_HISTORY_OVERRIDE_MIN_CONSISTENCY
      and directional_inward_sample_ratio
      >= POSITION_HISTORY_OVERRIDE_MIN_INWARD_SAMPLE_RATIO
      and inward_short
      >= POSITION_HISTORY_OVERRIDE_MIN_SHORT_INWARD_RATE_MPS
      and inward_long
      >= POSITION_HISTORY_OVERRIDE_MIN_LONG_INWARD_RATE_MPS
      and (
        math.copysign(1.0, position_rate)
        * reported_lateral_speed
        >= POSITION_HISTORY_OVERRIDE_MIN_RAW_LATERAL_SUPPORT_MPS
      )
    )
    directional_front_motion = (
      _sensor(state.source) == "front"
      and enough_history
      and directional_inward_displacement
      >= DIRECTIONAL_MIN_INWARD_DISPLACEMENT_M
      and directional_consistency >= self.directional_min_consistency
      and directional_inward_sample_ratio
      >= DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO
      and inward_short >= DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS
      and inward_long >= DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS
    )
    if directional_front_motion:
      responsive_rate = (
        (1.0 - DIRECTIONAL_SHORT_RATE_BLEND) * position_rate
        + DIRECTIONAL_SHORT_RATE_BLEND * short_rate
      )
      if -side * responsive_rate > inward_long:
        rate = responsive_rate
    if (
      config.use_reported_normal_velocity
      and enough_history
      and not position_history_override
    ):
      direction = math.copysign(1.0, position_rate) if abs(position_rate) > 1e-6 else 0.0
      supported_speed = max(0.0, direction * reported_normal_speed)
      rate = direction * min(abs(position_rate), supported_speed)
    path_slope = (
      rate / path_speed
      if abs(path_speed) > 0.1
      else 0.0
    )
    slope_disagreement = abs(short_slope - long_slope)
    vector_heading_deg = math.degrees(math.atan(path_slope))
    reported_heading_deg = math.degrees(math.atan2(
      reported_normal_speed,
      max(abs(reported_path_speed), 0.1),
    ))
    normal_speed_disagreement = (
      0.0
      if position_history_override
      else abs(position_rate - rate)
      if config.use_reported_normal_velocity
      else abs(position_rate - reported_normal_speed)
    )
    recent_motion_support = (
      min(
        1.0,
        max(
          0.0,
          math.copysign(1.0, position_rate) * short_rate
          / max(abs(position_rate), 0.05),
        ),
      )
      if enough_history and abs(position_rate) > 0.05
      else 1.0
    )
    long_duration_s = (
      long[-1].time_s - long[0].time_s
      if len(long) >= 2
      else 0.0
    )
    position_rate_sigma = (
      math.hypot(long_residual, 0.5 * short_residual)
    ) / max(long_duration_s, 0.25)
    normal_velocity_sigma = math.hypot(
      config.normal_velocity_sigma_mps,
      reported_normal_sigma,
      position_rate_sigma,
    )
    motion_consistency = (
      math.exp(-0.5 * (
        normal_speed_disagreement / max(normal_velocity_sigma, 0.15)
      ) ** 2)
      if config.use_reported_normal_velocity and enough_history
      else 1.0
    )
    lateral_base_uncertainty = (
      config.base_lateral_sigma_m
      + long_residual
      + 0.5 * short_residual
      + (0.45 if not enough_history else 0.0)
    )

    enter_limit = PATH_OVERLAP_HALF_WIDTH_M
    exit_limit = PATH_OVERLAP_HALF_WIDTH_M + PATH_STATE_HYSTERESIS_M
    was_inside = state.inside_latched
    if state.inside_latched:
      state.inside_latched = abs(observation.d_path) <= exit_limit
    else:
      state.inside_latched = abs(observation.d_path) <= enter_limit
    if not was_inside and state.inside_latched:
      state.entry_time_s = observation.time_s if enough_history else None
    elif not state.inside_latched:
      state.entry_time_s = None
    path_entry_age_s = (
      observation.time_s - state.entry_time_s
      if state.entry_time_s is not None
      else None
    )

    relative_accel = max(-3.0, min(3.0, observation.a_rel))
    path_accel = max(-3.0, min(3.0, observation.a_lead))
    samples: list[RadarMotionSample] = []
    path_key = _path_key(path)
    for horizon_s in MOTION_HORIZONS_S:
      path_displacement = (
        path_speed * horizon_s
        + 0.5 * path_accel * horizon_s ** 2
      )
      future_path_x = observation.path_s + path_displacement
      ego_path_x = observation.ego_path_s + observation.v_ego * horizon_s
      future_d_rel = future_path_x - ego_path_x
      future_d_path = (
        observation.d_path
        + path_slope * path_displacement
      )
      _, future_y = _model_path_point_at_s(
        path_key,
        future_path_x,
        future_d_path,
      )
      lateral_sigma = (
        lateral_base_uncertainty
        + slope_disagreement * abs(path_displacement)
        + 0.20 * abs(curvature) * path_displacement ** 2
      )
      longitudinal_sigma = (
        config.base_longitudinal_sigma_m
        + d_rel_residual
        + 0.5 * path_x_long_residual
        + 0.25 * path_x_short_residual
        + 0.20 * abs(relative_accel) * horizon_s ** 2
      )
      extrapolation_support = min(
        1.0,
        long_path_span / max(
          abs(path_displacement),
          config.minimum_path_span_m,
        ),
      )
      occupancy_prob = (
        _normal_interval_probability(
          future_d_path, lateral_sigma, PATH_OVERLAP_HALF_WIDTH_M,
        )
        * extrapolation_support
        if future_d_rel > 0.0
        else 0.0
      )
      path_proximity_score = (
        _path_proximity_score(future_d_path) * extrapolation_support
        if future_d_rel > 0.0
        else 0.0
      )
      samples.append(RadarMotionSample(
        horizon_s=horizon_s,
        d_rel=future_d_rel,
        path_x=future_path_x,
        y_rel=future_y,
        d_path=future_d_path,
        longitudinal_sigma=longitudinal_sigma,
        lateral_sigma=lateral_sigma,
        occupancy_prob=occupancy_prob,
        path_proximity_score=path_proximity_score,
      ))

    (
      predicted_path_overlap_start_s,
      predicted_path_overlap_s,
      continuous_overlap_samples,
    ) = _continuous_path_overlap(samples)
    outward_samples = tuple(
      sample for sample in samples
      if abs(sample.d_path) > abs(observation.d_path)
    )
    continuous_overlap_evidence = max(
      (
        max(sample.occupancy_prob, sample.path_proximity_score)
        * _inward_motion_support(
          observation.d_path,
          sample,
          lateral_base_uncertainty,
        )
        for sample in continuous_overlap_samples
      ),
      default=0.0,
    )
    continuous_overlap_support = min(
      1.0,
      predicted_path_overlap_s
      / FULL_PREDICTED_PATH_OVERLAP_SUPPORT_S,
    )
    long_horizon_corner_entry_supported = (
      _sensor(state.source) != "corner"
      or predicted_path_overlap_start_s is None
      or predicted_path_overlap_start_s < LONG_HORIZON_CORNER_ENTRY_START_S
      or (
        directional_inward_displacement
        >= LONG_HORIZON_CORNER_MIN_INWARD_DISPLACEMENT_M
        and directional_consistency
        >= LONG_HORIZON_CORNER_MIN_CONSISTENCY
        and directional_inward_sample_ratio
        >= LONG_HORIZON_CORNER_MIN_INWARD_SAMPLE_RATIO
      )
    )
    raw_path_entry_probability = (
      (
        math.sqrt(
          continuous_overlap_evidence * continuous_overlap_support,
        )
        * motion_consistency
        * recent_motion_support
      )
      if (
        enough_history
        and predicted_path_overlap_s >= MIN_PREDICTED_PATH_OVERLAP_S
        and long_horizon_corner_entry_supported
      )
      else 0.0
    )
    path_clearance = max(
      0.0,
      abs(observation.d_path) - PATH_OVERLAP_HALF_WIDTH_M,
    )
    near_side_directional_entry = (
      _sensor(state.source) == "corner"
      and enough_history
      and NEAR_SIDE_DIRECTIONAL_MIN_DREL_M < observation.d_rel
      <= NEAR_SIDE_DIRECTIONAL_MAX_DREL_M
      and path_clearance <= NEAR_SIDE_DIRECTIONAL_MAX_CLEARANCE_M
      and directional_inward_displacement
      >= NEAR_SIDE_DIRECTIONAL_MIN_DISPLACEMENT_M
      and directional_consistency
      >= NEAR_SIDE_DIRECTIONAL_MIN_CONSISTENCY
      and directional_inward_sample_ratio
      >= NEAR_SIDE_DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO
      and inward_short
      >= NEAR_SIDE_DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS
      and inward_long
      >= NEAR_SIDE_DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS
      and motion_consistency >= NEAR_SIDE_DIRECTIONAL_MIN_MOTION_SUPPORT
      and recent_motion_support >= NEAR_SIDE_DIRECTIONAL_MIN_MOTION_SUPPORT
    )
    near_side_entry_probability = (
      _path_proximity_score(observation.d_path)
      * directional_consistency
      * directional_inward_sample_ratio
      * motion_consistency
      * recent_motion_support
      if near_side_directional_entry
      else 0.0
    )
    lane_boundary_directional_entry = (
      _sensor(state.source) == "corner"
      and enough_history
      and not state.inside_latched
      and observation.d_rel > NEAR_SIDE_DIRECTIONAL_MIN_DREL_M
      and abs(observation.d_path)
      <= LANE_BOUNDARY_STRADDLE_HALF_WIDTH_M
      and directional_inward_displacement
      >= NEAR_SIDE_DIRECTIONAL_MIN_DISPLACEMENT_M
      and directional_consistency
      >= NEAR_SIDE_DIRECTIONAL_MIN_CONSISTENCY
      and directional_inward_sample_ratio
      >= NEAR_SIDE_DIRECTIONAL_MIN_INWARD_SAMPLE_RATIO
      and inward_short
      >= NEAR_SIDE_DIRECTIONAL_MIN_SHORT_INWARD_RATE_MPS
      and inward_long
      >= NEAR_SIDE_DIRECTIONAL_MIN_LONG_INWARD_RATE_MPS
      and -side * reported_normal_speed
      >= LANE_BOUNDARY_ENTRY_MIN_REPORTED_INWARD_MPS
      and motion_consistency
      >= LANE_BOUNDARY_ENTRY_MIN_MOTION_CONSISTENCY
      and recent_motion_support
      >= NEAR_SIDE_DIRECTIONAL_MIN_MOTION_SUPPORT
    )
    lane_boundary_entry_probability = (
      directional_consistency
      * directional_inward_sample_ratio
      * motion_consistency
      * recent_motion_support
      if lane_boundary_directional_entry
      else 0.0
    )
    front_tracked_close_entry = (
      _sensor(state.source) == "front"
      and enough_history
      and FRONT_TRACKED_CLOSE_ENTRY_MIN_DREL_M < observation.d_rel
      < FRONT_CUT_IN_MIN_DREL_M
      and state.inside_latched
      and path_entry_age_s is not None
      and path_entry_age_s <= FRONT_TRACKED_CLOSE_ENTRY_MAX_AGE_S
      and directional_inward_displacement
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_DISPLACEMENT_M
      and directional_consistency
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_CONSISTENCY
      and directional_inward_sample_ratio
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_INWARD_SAMPLE_RATIO
      and inward_short
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_SHORT_INWARD_RATE_MPS
      and inward_long
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_LONG_INWARD_RATE_MPS
      and -side * reported_normal_speed
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_REPORTED_INWARD_MPS
      and recent_motion_support
      >= FRONT_TRACKED_CLOSE_ENTRY_MIN_RECENT_MOTION_SUPPORT
    )
    within_current_cutin_scope = (
      abs(observation.d_path) <= CUT_IN_CURRENT_SCOPE_HALF_WIDTH_M
    )
    cut_in_detection_allowed = (
      within_current_cutin_scope
      and (
        _sensor(state.source) != "front"
        or observation.d_rel >= FRONT_CUT_IN_MIN_DREL_M
        or front_tracked_close_entry
      )
    )
    path_entry_probability = (
      max(
        raw_path_entry_probability,
        near_side_entry_probability,
        lane_boundary_entry_probability,
      )
      if cut_in_detection_allowed
      else 0.0
    )
    cut_in_probability = (
      path_entry_probability
      if enough_history and not state.inside_latched
      else 0.0
    )
    cut_out_probability = (
      (
        max(
          (1.0 - sample.occupancy_prob for sample in outward_samples),
          default=0.0,
        )
        * motion_consistency
      )
      if enough_history and state.inside_latched
      else 0.0
    )
    time_to_entry_s = next((
      sample.horizon_s
      for sample in samples
      if (
        sample.d_rel > 0.0
        and abs(sample.d_path)
        <= PATH_OVERLAP_HALF_WIDTH_M - PATH_STATE_HYSTERESIS_M
      )
    ), None)
    if not enough_history:
      reason = "insufficient measured dPath history"
    elif (
      config.use_reported_normal_velocity
      and motion_consistency < 0.50
    ):
      reason = "position/velocity motion mismatch"
    elif state.inside_latched:
      reason = (
        "physical CUT-OUT shadow"
        if cut_out_probability >= CUT_OUT_THRESHOLD
        else "current path overlap"
      )
    elif not within_current_cutin_scope:
      reason = "outside adjacent-lane CUT-IN scope"
    elif not cut_in_detection_allowed:
      reason = "front CUT-IN below 5m limit"
    elif near_side_directional_entry:
      reason = "near-side directional entry"
    elif lane_boundary_directional_entry:
      reason = "lane-boundary directional entry"
    elif not long_horizon_corner_entry_supported:
      reason = "long-horizon corner direction unconfirmed"
    elif cut_in_probability >= CUT_IN_THRESHOLD:
      reason = "physical CUT-IN shadow"
    else:
      reason = "outside path corridor"
    history = tuple(
      RadarMotionHistorySample(
        sample.time_s,
        observation.time_s - sample.time_s,
        sample.d_rel,
        (
          observation.ego_path_s
          + sample.path_x_world
          - observation.ego_distance
        ),
        (
          math.cos(observation.ego_heading)
          * (sample.actual_world_x - observation.ego_x)
          + math.sin(observation.ego_heading)
          * (sample.actual_world_y - observation.ego_y)
        ),
        (
          -math.sin(observation.ego_heading)
          * (sample.actual_world_x - observation.ego_x)
          + math.cos(observation.ego_heading)
          * (sample.actual_world_y - observation.ego_y)
        ),
        sample.d_path,
      )
      for sample in observations
    )
    return RadarMotionPrediction(
      track_id=track_id,
      source=state.source,
      sensor=_sensor(state.source),
      continuity_id=state.continuity_id,
      d_path=observation.d_path,
      d_path_rate_short=short_rate,
      d_path_rate_long=long_rate,
      d_path_curvature=curvature,
      path_speed_short=path_speed_short,
      path_speed_long=path_speed_long,
      path_speed=path_speed,
      vector_heading_deg=vector_heading_deg,
      reported_heading_deg=reported_heading_deg,
      reported_normal_speed=reported_normal_speed,
      normal_speed_disagreement=normal_speed_disagreement,
      motion_consistency=motion_consistency,
      recent_motion_support=recent_motion_support,
      uncertainty=lateral_base_uncertainty,
      lane_half_width=PATH_OVERLAP_HALF_WIDTH_M,
      current_path_occupancy=state.inside_latched,
      cut_in_detection_allowed=cut_in_detection_allowed,
      cut_in_probability=cut_in_probability,
      cut_out_probability=cut_out_probability,
      path_entry_probability=path_entry_probability,
      path_entry_age_s=(
        path_entry_age_s
      ),
      samples=tuple(samples),
      history=history,
      history_count=len(observations),
      time_to_entry_s=time_to_entry_s,
      reason=reason,
      predicted_path_overlap_s=predicted_path_overlap_s,
      predicted_path_overlap_start_s=predicted_path_overlap_start_s,
      directional_inward_displacement_m=directional_inward_displacement,
      directional_consistency=directional_consistency,
      directional_inward_sample_ratio=directional_inward_sample_ratio,
      near_side_directional_entry=near_side_directional_entry,
      lane_boundary_directional_entry=lane_boundary_directional_entry,
      front_tracked_close_entry=front_tracked_close_entry,
    )

  def update(
    self,
    time_s: float,
    points: Iterable[Any],
    path: Sequence[tuple[float, float]],
    v_ego: float = 0.0,
    yaw_rate_rad_s: float = 0.0,
    lead_one_d_rel: float | None = None,
    scoped_points: Sequence[
      tuple[Any, float, ModelPathProjection]
    ] | None = None,
  ) -> dict[tuple[str, int], RadarMotionPrediction]:
    time_s = float(time_s)
    v_ego = _finite(v_ego)
    yaw_rate_rad_s = _finite(yaw_rate_rad_s)
    if self._last_update_s is not None:
      dt = time_s - self._last_update_s
      if dt > 0.0:
        distance = 0.5 * (self._last_v_ego + v_ego) * dt
        yaw_rate = 0.5 * (self._last_yaw_rate + yaw_rate_rad_s)
        mid_heading = self._ego_heading_rad + 0.5 * yaw_rate * dt
        self._ego_distance_m += distance
        self._ego_x_m += distance * math.cos(mid_heading)
        self._ego_y_m += distance * math.sin(mid_heading)
        self._ego_heading_rad += yaw_rate * dt
    self._last_update_s = time_s
    self._last_v_ego = v_ego
    self._last_yaw_rate = yaw_rate_rad_s
    if not path:
      return {}
    for sensor, states in self._states.items():
      hold_s = SOURCE_CONFIGS[sensor].missing_hold_s
      for key, state in tuple(states.items()):
        if time_s - state.last_seen_s > hold_s:
          states.pop(key)

    point_values = tuple(points)
    scoped_points = (
      tuple(scoped_points)
      if scoped_points is not None
      else _scoped_motion_points(point_values, path)
    )
    visible_points = _visible_scoped_motion_points(
      scoped_points, lead_one_d_rel,
    )
    visible_keys = {
      (_source(point), int(getattr(point, "track_id", getattr(point, "trackId", -1))))
      for point in visible_points
    }
    scoped_keys = {
      (_source(point), int(getattr(point, "track_id", getattr(point, "trackId", -1))))
      for point, _, _ in scoped_points
    }
    for point in point_values:
      source = _source(point)
      sensor = _sensor(source)
      track_id = int(getattr(point, "track_id", getattr(point, "trackId", -1)))
      key = (source, track_id)
      if bool(getattr(point, "measured", False)) and key not in scoped_keys:
        self._states[sensor].pop(key, None)

    predictions: dict[tuple[str, int], RadarMotionPrediction] = {}
    ego_projection = project_to_model_path(path, 0.0, 0.0)
    for point, d_rel, projection in scoped_points:
      source = _source(point)
      sensor = _sensor(source)
      config = SOURCE_CONFIGS[sensor]
      track_id = int(getattr(point, "track_id", getattr(point, "trackId", -1)))
      key = (source, track_id)
      y_rel = _value(point, "y_rel", "yRel")
      d_path = projection.d_path
      v_lead = _value(
        point,
        "v_lead",
        "vLead",
        _value(point, "v_rel", "vRel") + v_ego,
      )
      if abs(v_lead) <= POSITION_ONLY_MAX_ABS_VLEAD_MPS:
        self._states[sensor].pop(key, None)
        continue
      cos_heading = math.cos(self._ego_heading_rad)
      sin_heading = math.sin(self._ego_heading_rad)
      yv_rel = _value(point, "yv_rel", "yvRel")
      target_vx, target_vy = radar_target_velocity_in_ego_frame(
        v_lead,
        yv_rel,
        d_rel,
        y_rel,
        yaw_rate_rad_s,
      )
      path_velocity = (
        projection.tangent_x * target_vx
        + projection.tangent_y * target_vy
      )
      normal_velocity = (
        -projection.tangent_y * target_vx
        + projection.tangent_x * target_vy
      )
      observation = _Observation(
        time_s=time_s,
        d_rel=d_rel,
        y_rel=y_rel,
        v_rel=_value(point, "v_rel", "vRel"),
        a_rel=_value(point, "a_rel", "aRel"),
        v_lead=v_lead,
        a_lead=_value(
          point,
          "a_lead",
          "aLeadK",
          _value(point, "a_rel", "aRel"),
        ),
        yv_rel=yv_rel,
        v_ego=v_ego,
        ego_distance=self._ego_distance_m,
        ego_x=self._ego_x_m,
        ego_y=self._ego_y_m,
        ego_heading=self._ego_heading_rad,
        ego_path_s=ego_projection.path_s,
        path_s=projection.path_s,
        path_x_world=(
          self._ego_distance_m
          + projection.path_s
          - ego_projection.path_s
        ),
        path_velocity=path_velocity,
        normal_velocity=normal_velocity,
        actual_world_x=(
          self._ego_x_m
          + cos_heading * d_rel
          - sin_heading * y_rel
        ),
        actual_world_y=(
          self._ego_y_m
          + sin_heading * d_rel
          + cos_heading * y_rel
        ),
        d_path=d_path,
      )
      state = self._states[sensor].get(key)
      if state is None or not self._continuous(state, observation, config):
        state = self._new_state(source, time_s)
        self._states[sensor][key] = state
      state.observations.append(observation)
      while (
        state.observations
        and time_s - state.observations[0].time_s > MAX_HISTORY_S
      ):
        state.observations.popleft()
      state.last_seen_s = time_s
      if (
        key in visible_keys
        or (
          sensor == "corner"
          and _strong_hidden_dpath_entry(
            tuple(state.observations),
            observation.d_path,
          )
        )
      ):
        predictions[key] = self._prediction(
          state, track_id, observation, path, config,
        )
      elif (
        state.inside_latched
        and abs(observation.d_path)
        > PATH_OVERLAP_HALF_WIDTH_M + PATH_STATE_HYSTERESIS_M
      ):
        # A hidden adjacent target cannot enter the path corridor, but an
        # already latched target can finish leaving it while occluded.
        state.inside_latched = False
        state.entry_time_s = None
    return predictions


class RadarMotionDecisionTracker:
  """Apply the shared, small temporal confirmation to physical CUT-IN evidence."""

  def __init__(
    self,
    threshold: float = CUT_IN_THRESHOLD,
    confirmation_s: float = CUT_IN_CONFIRMATION_S,
    boundary_hold_s: float = CUT_IN_BOUNDARY_HOLD_S,
  ) -> None:
    self.threshold = float(threshold)
    self.confirmation_s = float(confirmation_s)
    self.boundary_hold_s = float(boundary_hold_s)
    self._started_at: dict[tuple[str, int, int], float] = {}
    self._peak_score: dict[tuple[str, int, int], float] = {}

  @staticmethod
  def _key(prediction: RadarMotionPrediction) -> tuple[str, int, int]:
    return (
      prediction.source,
      prediction.track_id,
      prediction.continuity_id,
    )

  def _confirmation_time_s(
    self,
    prediction: RadarMotionPrediction,
  ) -> float:
    """Apply the selected measured-evidence dwell to normal CUT-IN entries."""
    if (
      prediction.near_side_directional_entry
      or prediction.lane_boundary_directional_entry
    ):
      # This evidence already contains a strict 0.8-second, one-way measured
      # history. Waiting for another decision-frame dwell makes a close,
      # longitudinally passing corner reflection disappear before it can be
      # selected.
      return 0.0
    if prediction.front_tracked_close_entry:
      # Like the close corner rule, this is already backed by sustained
      # one-way measured history plus a real OUT-to-IN boundary crossing.
      return 0.0
    current_d_rel = (
      prediction.history[-1].d_rel
      if prediction.history
      else math.inf
    )
    side = (
      math.copysign(1.0, prediction.d_path)
      if abs(prediction.d_path) > 1e-6
      else 0.0
    )
    inward_short = -side * prediction.d_path_rate_short
    inward_long = -side * prediction.d_path_rate_long
    path_clearance = max(
      0.0,
      abs(prediction.d_path) - PATH_OVERLAP_HALF_WIDTH_M,
    )
    urgent_near_path_entry = (
      prediction.sensor == "corner"
      and 0.8 < current_d_rel <= URGENT_NEAR_PATH_MAX_DREL_M
      and path_clearance <= URGENT_NEAR_PATH_MAX_CLEARANCE_M
      and inward_short >= URGENT_NEAR_PATH_MIN_INWARD_RATE_MPS
      and inward_long >= URGENT_NEAR_PATH_MIN_INWARD_RATE_MPS
      and prediction.motion_consistency >= 0.70
      and prediction.recent_motion_support >= 0.70
    )
    strong_front_directional_entry = (
      prediction.sensor == "front"
      and prediction.directional_inward_displacement_m
      >= STRONG_FRONT_MIN_INWARD_DISPLACEMENT_M
      and prediction.directional_consistency
      >= STRONG_FRONT_MIN_CONSISTENCY
      and prediction.directional_inward_sample_ratio
      >= STRONG_FRONT_MIN_INWARD_SAMPLE_RATIO
      and inward_short >= STRONG_FRONT_MIN_SHORT_INWARD_RATE_MPS
      and inward_long >= STRONG_FRONT_MIN_LONG_INWARD_RATE_MPS
      and prediction.motion_consistency >= STRONG_FRONT_MIN_MOTION_SUPPORT
      and prediction.recent_motion_support >= STRONG_FRONT_MIN_MOTION_SUPPORT
    )
    if strong_front_directional_entry:
      # The strict 0.8-second measured direction history already spans the
      # interval represented by the first qualifying sample. Credit at most
      # one 20 Hz radar frame so a physically sustained entry is not rejected
      # only because timestamp deltas land just below the selected dwell.
      return max(
        0.0,
        self.confirmation_s - STRONG_FRONT_CONFIRMATION_CREDIT_S,
      )
    return (
      min(self.confirmation_s, URGENT_NEAR_PATH_CONFIRMATION_S)
      if urgent_near_path_entry
      else self.confirmation_s
    )

  def reset(self) -> None:
    self._started_at.clear()
    self._peak_score.clear()

  def update(
    self,
    time_s: float,
    predictions: Iterable[RadarMotionPrediction],
  ) -> RadarMotionDecision:
    prediction_by_key = {
      self._key(prediction): prediction
      for prediction in predictions
    }
    raw_keys = {
      key
      for key, prediction in prediction_by_key.items()
      if (
        getattr(prediction, "cut_in_detection_allowed", True)
        and (
          (
            not prediction.current_path_occupancy
            and prediction.cut_in_probability >= self.threshold
          )
          or (
            prediction.current_path_occupancy
            and prediction.path_entry_age_s is not None
            and prediction.path_entry_age_s
            <= self.confirmation_s + self.boundary_hold_s
            and prediction.path_entry_probability >= self.threshold
          )
        )
      )
    }

    for key in tuple(self._started_at):
      if key in raw_keys:
        continue
      prediction = prediction_by_key.get(key)
      elapsed_s = time_s - self._started_at[key]
      crossing_inward = (
        prediction is not None
        and getattr(prediction, "cut_in_detection_allowed", True)
        and prediction.current_path_occupancy
        and prediction.d_path * prediction.d_path_rate_long < 0.0
        and prediction.path_entry_age_s is not None
        and prediction.path_entry_age_s
        <= self.confirmation_s + self.boundary_hold_s
        and elapsed_s <= self.confirmation_s + self.boundary_hold_s
      )
      if not crossing_inward:
        self._started_at.pop(key)
        self._peak_score.pop(key, None)

    for key in raw_keys:
      prediction = prediction_by_key[key]
      self._started_at.setdefault(key, time_s)
      self._peak_score[key] = max(
        self._peak_score.get(key, 0.0),
        prediction.cut_in_probability,
        prediction.path_entry_probability,
      )

    active_keys = raw_keys | {
      key
      for key in self._started_at
      if (
        key in prediction_by_key
        and getattr(
          prediction_by_key[key], "cut_in_detection_allowed", True,
        )
        and prediction_by_key[key].current_path_occupancy
        and prediction_by_key[key].d_path
        * prediction_by_key[key].d_path_rate_long < 0.0
        and prediction_by_key[key].path_entry_age_s is not None
        and prediction_by_key[key].path_entry_age_s
        <= self.confirmation_s + self.boundary_hold_s
      )
    }
    confirmed = tuple(
      RadarMotionCutIn(
        prediction_by_key[key],
        self._peak_score[key],
      )
      for key, started_at in self._started_at.items()
      if (
        key in active_keys
        and key in prediction_by_key
        and time_s - started_at
        >= self._confirmation_time_s(prediction_by_key[key])
      )
    )
    return RadarMotionDecision(confirmed)
