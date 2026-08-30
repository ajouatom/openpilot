#!/usr/bin/env python3
"""Small trajectory-based CUT-IN detector.

The detector has one job: turn a short, path-relative radar trajectory into a
future position.  Lead-role selection and primary vision/radar matching stay
outside this module.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  ModelPathProjection,
  project_to_model_path,
  radar_target_velocity_in_ego_frame,
  turning_corner_path_entry_allowed,
)
from openpilot.selfdrive.carrot.radar_motion.primary import (
  RADAR_TO_CAMERA_M,
  RadarPointSnapshot,
)


EGO_PATH_HALF_WIDTH_M = 0.90
TARGET_HALF_WIDTH_M = 1.00
PATH_OVERLAP_HALF_WIDTH_M = EGO_PATH_HALF_WIDTH_M + TARGET_HALF_WIDTH_M
PAIRED_CLOSE_BODY_HALF_WIDTH_M = 2.65
OUTSIDE_HISTORY_MARGIN_M = 0.20
MOTION_SCOPE_HALF_WIDTH_M = 5.40
MAX_HISTORY_S = 1.50
LONG_MOTION_WINDOW_S = 0.90
SHORT_MOTION_WINDOW_S = 0.35
MAX_OBSERVATION_GAP_S = 0.20
MAX_DREL_M = 80.0
CUTIN_MAX_DREL_M = 45.0
FRONT_NEW_CUTIN_MIN_DREL_M = 2.0
FRONT_CLOSE_BORN_MIN_DREL_M = 5.0
CROSS_SENSOR_SLOT_HANDOFF_MAX_DREL_M = 12.0
CROSS_SENSOR_ALIAS_HOLD_S = 0.35
MIN_MOVING_VLEAD_MPS = 0.5

FRONT_CURVE_ALIAS_MIN_ABS_YAW_RATE_RAD_S = 0.025
CORNER_CURVE_ALIAS_MIN_ABS_YAW_RATE_RAD_S = 0.040
CORNER_CURVE_ALIAS_MIN_ABS_YREL_M = 4.0
CORNER_CURVE_ALIAS_MIN_OFFSET_DISCREPANCY_M = 2.0
CORNER_CURVE_MAX_REPORTED_INWARD_MPS = 3.0
CORNER_CURVE_MIN_RATE_DISAGREEMENT_MPS = 1.5
PAIRED_OUTER_BODY_MIN_REPORTED_INWARD_MPS = 0.15
PAIRED_OUTER_BODY_MAX_VREL_MPS = 1.0
PAIRED_OUTER_BODY_MAX_ABS_YAW_RATE_RAD_S = 0.040
PAIRED_OUTER_BODY_MIN_INWARD_PROGRESS_M = 0.15
PAIRED_FAST_PASS_MIN_CLOSING_SPEED_MPS = 5.0
PAIRED_FAST_PASS_MAX_TTC_S = 0.80
PAIRED_FAST_PASS_MIN_HISTORY_S = 0.35

CUTIN_CONFIRMATION_S = 0.10
CUTIN_CURRENT_OVERLAP_CONFIRMATION_S = 0.05
CUTIN_HOLD_S = 0.55
RISK_CONFIRMATION_S = 0.15
RISK_HOLD_S = 0.25

MIN_INWARD_PROGRESS_M = 0.20
MIN_PREDICTED_INWARD_PROGRESS_M = 0.28
MIN_INWARD_RATE_MPS = 0.15
MIN_DIRECTION_CONSISTENCY = 0.65
MIN_EXISTENCE_HISTORY_S = 0.20
TENTATIVE_FRONT_HISTORY_S = 0.75
JITTER_MIN_TRAVEL_M = 0.45
JITTER_MIN_NET_FRACTION = 0.50


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError, IndexError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def prediction_horizon_s(v_ego: float) -> float:
  """Use time, not distance, while allowing more lookahead at low speed."""
  return max(1.40, min(3.00, 3.00 - 0.05 * max(0.0, v_ego)))


@dataclass(frozen=True)
class TrajectoryCutInEstimate:
  point: RadarPointSnapshot
  continuity_id: int
  d_path: float
  d_path_rate: float
  future_d_rel: float
  future_d_path: float
  horizon_s: float
  time_to_overlap_s: float | None
  inward_rate: float
  reported_inward_rate: float
  inward_progress: float
  recent_inward_progress: float
  lateral_travel: float
  lateral_net_fraction: float
  direction_consistency: float
  recent_direction_consistency: float
  recent_v_rel_min: float
  recent_v_rel_spread: float
  recent_abs_yaw_max: float
  history_s: float
  confidence: float
  vision_supported: bool
  cross_sensor_supported: bool
  cross_sensor_track_id: int | None
  current_path: bool
  raw_cutin: bool
  confirmed_cutin: bool
  control_eligible: bool
  predecel_risk: bool
  jittering: bool
  unstable_fast_motion: bool
  front_history_supported: bool
  close_front_supported: bool
  curve_alias: bool
  reason: str

  @property
  def identity(self) -> tuple[str, int, int]:
    return self.point.source, self.point.track_id, self.continuity_id


@dataclass(frozen=True)
class _Observation:
  time_s: float
  d_rel: float
  y_rel: float
  v_rel: float
  v_lead: float
  yaw_rate_rad_s: float
  global_path_s: float
  d_path: float


@dataclass
class _TrackState:
  continuity_id: int
  observations: deque[_Observation] = field(default_factory=deque)
  last_seen_s: float = -math.inf
  cutin_since_s: float | None = None
  cutin_until_s: float = -math.inf
  risk_since_s: float | None = None
  risk_until_s: float = -math.inf

  def reset(self, continuity_id: int) -> None:
    self.continuity_id = continuity_id
    self.observations.clear()
    self.cutin_since_s = None
    self.cutin_until_s = -math.inf
    self.risk_since_s = None
    self.risk_until_s = -math.inf


def _values_since(
  observations: Sequence[_Observation],
  window_s: float,
) -> tuple[_Observation, ...]:
  if not observations:
    return ()
  start_s = observations[-1].time_s - window_s
  return tuple(value for value in observations if value.time_s >= start_s)


def _median_slope(
  observations: Sequence[_Observation],
  attribute: str,
  window_s: float,
) -> float:
  values = _values_since(observations, window_s)
  slopes = []
  for index, first in enumerate(values):
    for second in values[index + 1:]:
      dt = second.time_s - first.time_s
      if dt < 0.10:
        continue
      slopes.append(
        (getattr(second, attribute) - getattr(first, attribute)) / dt
      )
  return float(statistics.median(slopes)) if slopes else 0.0


def _motion_metrics(
  observations: Sequence[_Observation],
  window_s: float = LONG_MOTION_WINDOW_S,
) -> tuple[float, float, float, float, float, float, bool]:
  values = _values_since(observations, window_s)
  if len(values) < 2:
    return 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False

  d_path_rate = _median_slope(values, "d_path", window_s)
  current_side = math.copysign(1.0, values[-1].d_path or values[0].d_path or 1.0)
  inward_rate = max(0.0, -current_side * d_path_rate)
  start_count = min(3, max(1, len(values) // 3))
  end_count = min(2, len(values))
  start_abs = statistics.median(abs(value.d_path) for value in values[:start_count])
  end_abs = statistics.median(abs(value.d_path) for value in values[-end_count:])
  inward_progress = max(0.0, start_abs - end_abs)

  inward_travel = 0.0
  outward_travel = 0.0
  for first, second in zip(values, values[1:], strict=False):
    delta = abs(first.d_path) - abs(second.d_path)
    if abs(delta) < 0.01:
      continue
    if delta > 0.0:
      inward_travel += delta
    else:
      outward_travel -= delta
  total_travel = inward_travel + outward_travel
  direction_consistency = (
    inward_travel / total_travel if total_travel > 1e-6 else 0.0
  )
  net_fraction = inward_progress / max(total_travel, 1e-6)
  jittering = (
    total_travel >= JITTER_MIN_TRAVEL_M
    and net_fraction < JITTER_MIN_NET_FRACTION
  )
  return (
    d_path_rate,
    inward_rate,
    inward_progress,
    total_travel,
    net_fraction,
    direction_consistency,
    jittering,
  )


def _vision_supports(
  point: RadarPointSnapshot,
  model: Any,
) -> bool:
  for lead in getattr(model, "leadsV3", ()):
    probability = _finite(getattr(lead, "prob", 0.0))
    if probability < 0.35:
      continue
    d_rel = _finite(getattr(lead, "x", (0.0,))[0]) - RADAR_TO_CAMERA_M
    y_rel = -_finite(getattr(lead, "y", (0.0,))[0])
    v_lead = _finite(getattr(lead, "v", (point.v_lead,))[0], point.v_lead)
    x_std = max(1.0, _finite(getattr(lead, "xStd", (1.0,))[0], 1.0))
    y_std = max(0.5, _finite(getattr(lead, "yStd", (0.5,))[0], 0.5))
    v_std = max(1.0, _finite(getattr(lead, "vStd", (1.0,))[0], 1.0))
    if (
      abs(point.d_rel - d_rel) <= max(4.0, 0.20 * point.d_rel, 2.0 * x_std)
      and abs(point.y_rel - y_rel) <= max(1.5, 2.0 * y_std)
      and abs(point.v_lead - v_lead) <= max(5.0, 2.0 * v_std)
    ):
      return True
  return False


def _reported_inward_speed(
  point: RadarPointSnapshot,
  projection: ModelPathProjection,
  yaw_rate_rad_s: float,
) -> float:
  velocity_x, velocity_y = radar_target_velocity_in_ego_frame(
    point.v_lead,
    point.yv_rel,
    point.d_rel,
    point.y_rel,
    yaw_rate_rad_s,
  )
  normal_velocity = (
    -projection.tangent_y * velocity_x
    + projection.tangent_x * velocity_y
  )
  side = math.copysign(1.0, projection.d_path or point.y_rel or 1.0)
  return max(0.0, -side * normal_velocity)


def _update_latch(
  state: _TrackState,
  time_s: float,
  raw: bool,
  *,
  risk: bool,
  confirmation_s: float,
  hold_s: float,
) -> bool:
  since_name = "risk_since_s" if risk else "cutin_since_s"
  until_name = "risk_until_s" if risk else "cutin_until_s"
  since = getattr(state, since_name)
  if raw:
    if since is None:
      since = time_s
      setattr(state, since_name, since)
    if time_s - since + 1e-6 >= confirmation_s:
      setattr(state, until_name, time_s + hold_s)
  else:
    setattr(state, since_name, None)
  return time_s <= getattr(state, until_name)


class TrajectoryCutInDetector:
  """Predict CUT-IN position from measured path-relative track history."""

  def __init__(self, sensitivity: int = 3) -> None:
    self.sensitivity = max(0, min(5, int(sensitivity)))
    self._tracks: dict[tuple[str, int], _TrackState] = {}
    self._corner_front_aliases: dict[
      tuple[str, int], tuple[tuple[str, int], float]
    ] = {}
    self._next_continuity_id = 1
    self._last_time_s: float | None = None
    self._last_v_ego = 0.0
    self._ego_distance = 0.0
    self.last_estimates: tuple[TrajectoryCutInEstimate, ...] = ()

  def reset(self) -> None:
    self._tracks.clear()
    self._corner_front_aliases.clear()
    self._last_time_s = None
    self._last_v_ego = 0.0
    self._ego_distance = 0.0
    self.last_estimates = ()

  def _new_continuity_id(self) -> int:
    value = self._next_continuity_id
    self._next_continuity_id += 1
    return value

  @staticmethod
  def _continuous(
    state: _TrackState,
    point: RadarPointSnapshot,
    time_s: float,
  ) -> bool:
    if not state.observations:
      return True
    previous = state.observations[-1]
    dt = time_s - previous.time_s
    if dt <= 0.0 or dt > MAX_OBSERVATION_GAP_S:
      return False
    predicted_d_rel = previous.d_rel + previous.v_rel * dt
    return (
      abs(point.d_rel - predicted_d_rel) <= 2.5
      and abs(point.y_rel - previous.y_rel) <= 1.0
      and abs(point.v_lead - previous.v_lead) <= 7.0
    )

  @staticmethod
  def _existence_supported(
    point: RadarPointSnapshot,
    history_s: float,
    vision_supported: bool,
    cross_sensor_supported: bool,
  ) -> bool:
    if not point.measured:
      return vision_supported and history_s >= 0.10
    if vision_supported or cross_sensor_supported:
      return history_s >= 0.10
    if point.source == "frontRadar" and point.radar_track_state == 1:
      return history_s >= TENTATIVE_FRONT_HISTORY_S
    return history_s >= MIN_EXISTENCE_HISTORY_S

  def update(
    self,
    time_s: float,
    v_ego: float,
    points: Iterable[RadarPointSnapshot],
    path: Sequence[tuple[float, float]],
    model: Any,
    *,
    yaw_rate_rad_s: float = 0.0,
    vision_required_front: bool = False,
    cross_sensor_matches: Mapping[
      tuple[str, int], RadarPointSnapshot
    ] | None = None,
  ) -> tuple[TrajectoryCutInEstimate, ...]:
    time_s = float(time_s)
    v_ego = max(0.0, float(v_ego))
    if self._last_time_s is not None:
      dt = time_s - self._last_time_s
      if dt < 0.0 or dt > 0.50:
        self.reset()
      else:
        self._ego_distance += 0.5 * (self._last_v_ego + v_ego) * dt
    self._last_time_s = time_s
    self._last_v_ego = v_ego

    matches = cross_sensor_matches or {}
    seen: set[tuple[str, int]] = set()
    estimates: list[TrajectoryCutInEstimate] = []
    for point in points:
      unconfirmed_front = (
        point.source == "frontRadar"
        and point.d_rel > 0.8
      )
      if (
        not (point.measured or unconfirmed_front)
        or not (-5.0 <= point.d_rel <= MAX_DREL_M)
      ):
        continue
      projection = project_to_model_path(path, point.d_rel, point.y_rel)
      if abs(projection.d_path) > MOTION_SCOPE_HALF_WIDTH_M:
        continue
      raw_key = point.source, point.track_id
      cross_sensor_point = matches.get(raw_key)
      # Corner radar object slots can swap between adjacent physical targets.
      # When front radar independently associates the target, use that stable
      # physical ID for motion history so a slot handoff does not erase the
      # measured inward trajectory. Fall back to the raw slot if an invalid
      # many-to-one match appears in the same frame.
      stable_cross_key = (
        (f"{point.source}:front", cross_sensor_point.track_id)
        if point.source.startswith("corner")
        and cross_sensor_point is not None
        and cross_sensor_point.source == "frontRadar"
        and point.d_rel <= CROSS_SENSOR_SLOT_HANDOFF_MAX_DREL_M
        else None
      )
      if stable_cross_key is not None:
        self._corner_front_aliases[raw_key] = stable_cross_key, time_s
      else:
        prior_alias = self._corner_front_aliases.get(raw_key)
        if (
          prior_alias is not None
          and point.source.startswith("corner")
          and point.d_rel <= CROSS_SENSOR_SLOT_HANDOFF_MAX_DREL_M
          and time_s - prior_alias[1] <= CROSS_SENSOR_ALIAS_HOLD_S
        ):
          stable_cross_key = prior_alias[0]
      key = stable_cross_key or raw_key
      if key in seen:
        key = raw_key
      seen.add(key)
      state = self._tracks.get(key)
      if state is None:
        state = _TrackState(self._new_continuity_id())
        self._tracks[key] = state
      elif not self._continuous(state, point, time_s):
        state.reset(self._new_continuity_id())

      state.observations.append(_Observation(
        time_s=time_s,
        d_rel=point.d_rel,
        y_rel=point.y_rel,
        v_rel=point.v_rel,
        v_lead=point.v_lead,
        yaw_rate_rad_s=yaw_rate_rad_s,
        global_path_s=self._ego_distance + projection.path_s,
        d_path=projection.d_path,
      ))
      while (
        state.observations
        and time_s - state.observations[0].time_s > MAX_HISTORY_S
      ):
        state.observations.popleft()
      state.last_seen_s = time_s

      history_s = (
        state.observations[-1].time_s - state.observations[0].time_s
        if len(state.observations) >= 2 else 0.0
      )
      (
        d_path_rate,
        inward_rate,
        inward_progress,
        lateral_travel,
        lateral_net_fraction,
        direction_consistency,
        jittering,
      ) = _motion_metrics(state.observations)
      (
        short_rate,
        short_inward_rate,
        short_inward_progress,
        _,
        _,
        short_direction_consistency,
        _,
      ) = _motion_metrics(
        state.observations, SHORT_MOTION_WINDOW_S,
      )
      side = math.copysign(1.0, projection.d_path or point.y_rel or 1.0)
      short_inward_rate = max(
        short_inward_rate,
        max(0.0, -side * short_rate),
      )
      recent_v_rel_values = tuple(
        value.v_rel
        for value in _values_since(state.observations, 0.50)
      )
      recent_v_rel_min = min(recent_v_rel_values, default=point.v_rel)
      recent_v_rel_spread = (
        max(recent_v_rel_values, default=point.v_rel) - recent_v_rel_min
      )
      recent_abs_yaw_max = max((
        abs(value.yaw_rate_rad_s)
        for value in _values_since(state.observations, 0.75)
      ), default=abs(yaw_rate_rad_s))
      reported_inward = _reported_inward_speed(
        point, projection, yaw_rate_rad_s,
      )
      supported_inward_rate = max(
        inward_rate,
        min(short_inward_rate, reported_inward + 0.35),
      )
      horizon_s = prediction_horizon_s(v_ego)
      unstable_fast_lateral_motion = (
        abs(point.v_rel) >= 5.0
        and (
          jittering
          or direction_consistency < 0.75
          or abs(inward_rate - reported_inward) > 0.75
        )
      )
      if unstable_fast_lateral_motion:
        horizon_s *= max(
          0.50, 1.0 - 0.07 * (abs(point.v_rel) - 5.0),
        )
      target_path_speed = _median_slope(
        state.observations, "global_path_s", LONG_MOTION_WINDOW_S,
      )
      relative_path_speed = (
        target_path_speed - v_ego if history_s >= 0.20 else point.v_rel
      )
      relative_path_speed = max(
        point.v_rel - 3.0,
        min(point.v_rel + 3.0, relative_path_speed),
      )
      future_d_rel = point.d_rel + relative_path_speed * horizon_s
      future_d_path = projection.d_path + d_path_rate * horizon_s
      clearance = max(
        0.0, abs(projection.d_path) - PATH_OVERLAP_HALF_WIDTH_M,
      )
      time_to_overlap_s = (
        0.0
        if clearance <= 0.0
        else clearance / supported_inward_rate
        if supported_inward_rate > 0.05
        else None
      )

      vision_supported = _vision_supports(point, model)
      cross_sensor_supported = cross_sensor_point is not None
      paired_front_overlap = (
        point.source.startswith("corner")
        and cross_sensor_point is not None
        and cross_sensor_point.source == "frontRadar"
        # At close range, both radars can return the outer body of a pickup or
        # truck rather than its centre. The motion gate in paired_close_entry
        # still prevents a parallel side reflection from becoming a cut-in.
        and (
          abs(cross_sensor_point.y_rel) <= 2.15
          or (
            abs(cross_sensor_point.y_rel)
            <= PAIRED_CLOSE_BODY_HALF_WIDTH_M
            and reported_inward
            >= PAIRED_OUTER_BODY_MIN_REPORTED_INWARD_MPS
            and point.v_rel <= PAIRED_OUTER_BODY_MAX_VREL_MPS
            and recent_abs_yaw_max
            < PAIRED_OUTER_BODY_MAX_ABS_YAW_RATE_RAD_S
            and inward_progress
            >= PAIRED_OUTER_BODY_MIN_INWARD_PROGRESS_M
          )
        )
      )
      existence_supported = self._existence_supported(
        point,
        history_s,
        vision_supported,
        cross_sensor_supported,
      )
      started_outside = any(
        abs(value.y_rel if point.source == "frontRadar" else value.d_path)
        >= PATH_OVERLAP_HALF_WIDTH_M + OUTSIDE_HISTORY_MARGIN_M
        for value in state.observations
      )
      current_overlap = abs(projection.d_path) <= PATH_OVERLAP_HALF_WIDTH_M
      front_overlap_half_width = (
        2.15 if point.d_rel <= 20.0 else PATH_OVERLAP_HALF_WIDTH_M
      )
      raw_body_overlap = abs(point.y_rel) <= front_overlap_half_width
      ahead_at_overlap = (
        time_to_overlap_s is not None
        and point.d_rel + relative_path_speed * time_to_overlap_s > 0.5
      )
      predicted_overlap = (
        time_to_overlap_s is not None
        and time_to_overlap_s <= horizon_s
        and ahead_at_overlap
      )
      recent_time_to_overlap_s = (
        clearance / short_inward_rate
        if short_inward_rate > 0.05 else math.inf
      )
      recent_predicted_overlap = (
        recent_time_to_overlap_s <= horizon_s
        and point.d_rel + relative_path_speed * recent_time_to_overlap_s > 0.5
      )
      consistent_motion = (
        inward_progress >= MIN_INWARD_PROGRESS_M
        and supported_inward_rate >= MIN_INWARD_RATE_MPS
        and direction_consistency >= MIN_DIRECTION_CONSISTENCY
      )
      strong_predicted_motion = (
        inward_progress >= MIN_PREDICTED_INWARD_PROGRESS_M
        and inward_rate >= MIN_INWARD_RATE_MPS
        and direction_consistency >= MIN_DIRECTION_CONSISTENCY
      )
      front_history_supported = (
        vision_required_front
        and point.source == "frontRadar"
        and not vision_supported
        and history_s >= 0.50
        and point.d_rel <= 20.0
        and abs(projection.d_path) <= 3.20
        and recent_abs_yaw_max < 0.020
        and abs(point.v_rel) <= 5.0
        and recent_v_rel_min >= 0.50
        and recent_v_rel_spread <= 1.00
        and (
          (
            short_inward_progress >= 0.18
            and short_inward_rate >= 0.35
            and short_direction_consistency >= 0.85
          )
          or (
            inward_progress >= 0.35
            and supported_inward_rate >= 0.35
            and direction_consistency >= 0.75
          )
        )
      )
      jitter_override = (
        front_history_supported
        or (
          (vision_supported or cross_sensor_supported)
          and inward_progress >= 0.45
          and direction_consistency >= 0.65
        )
      )
      motion_reliable = not jittering or jitter_override
      paired_fast_pass = (
        point.source.startswith("corner")
        and -point.v_rel >= PAIRED_FAST_PASS_MIN_CLOSING_SPEED_MPS
        and point.d_rel / max(-point.v_rel, 0.1)
        <= PAIRED_FAST_PASS_MAX_TTC_S
        and history_s < PAIRED_FAST_PASS_MIN_HISTORY_S
      )
      paired_close_entry = (
        point.source.startswith("corner")
        and cross_sensor_supported
        and paired_front_overlap
        and point.d_rel <= 8.0
        and abs(projection.d_path) <= 2.75
        and inward_progress >= 0.06
        and supported_inward_rate >= 0.15
        and direction_consistency >= 0.75
        and not paired_fast_pass
      )
      close_direct_entry = (
        point.source.startswith("corner")
        and point.d_rel <= 8.0
        and abs(projection.d_path) <= 2.85
        and history_s >= 0.50
        and inward_progress >= 0.20
        and supported_inward_rate >= 0.25
        and reported_inward >= 0.20
        and direction_consistency >= 0.85
        and predicted_overlap
        and not paired_fast_pass
      )
      curve_alias = (
        not turning_corner_path_entry_allowed(
          point.source,
          point.y_rel,
          projection.d_path,
          yaw_rate_rad_s,
        )
        or (
          point.source.startswith("corner")
          and recent_abs_yaw_max
          >= CORNER_CURVE_ALIAS_MIN_ABS_YAW_RATE_RAD_S
          and abs(point.y_rel) >= CORNER_CURVE_ALIAS_MIN_ABS_YREL_M
          and abs(point.y_rel) - abs(projection.d_path)
          >= CORNER_CURVE_ALIAS_MIN_OFFSET_DISCREPANCY_M
          and not vision_supported
          and reported_inward
          < PAIRED_OUTER_BODY_MIN_REPORTED_INWARD_MPS
        )
        or (
          point.source.startswith("corner")
          and recent_abs_yaw_max
          >= CORNER_CURVE_ALIAS_MIN_ABS_YAW_RATE_RAD_S
          and reported_inward
          > CORNER_CURVE_MAX_REPORTED_INWARD_MPS
          and reported_inward - inward_rate
          >= CORNER_CURVE_MIN_RATE_DISAGREEMENT_MPS
        )
      )
      front_curve_motion_supported = (
        point.source != "frontRadar"
        or recent_abs_yaw_max < FRONT_CURVE_ALIAS_MIN_ABS_YAW_RATE_RAD_S
        or vision_supported
        or cross_sensor_supported
        or reported_inward >= PAIRED_OUTER_BODY_MIN_REPORTED_INWARD_MPS
      )
      turning_corner = (
        point.source.startswith("corner")
        and abs(yaw_rate_rad_s) >= 0.10
      )
      volatile_corner_slot = (
        point.source.startswith("corner") and point.track_id < 1000
      )
      curve_motion_supported = (
        not turning_corner
        or (
          (cross_sensor_supported or vision_supported)
          and reported_inward >= 0.15
          and reported_inward <= 3.0
          and inward_progress >= 0.35
        )
      )
      slot_motion_supported = (
        not volatile_corner_slot
        or paired_close_entry
        or 0.25 <= reported_inward <= 3.0
      )
      corner_motion_plausible = (
        not point.source.startswith("corner")
        or vision_supported
        or cross_sensor_supported
        or inward_rate <= max(2.0, reported_inward + 1.0)
      )
      uncorroborated_away_range_supported = (
        not point.source.startswith("corner")
        or vision_supported
        or cross_sensor_supported
        or point.v_rel <= 0.5
        or point.d_rel <= 30.0
      )
      front_range_ok = (
        point.source != "frontRadar"
        or point.d_rel >= FRONT_NEW_CUTIN_MIN_DREL_M
        or any(
          value.d_rel >= FRONT_NEW_CUTIN_MIN_DREL_M
          for value in state.observations
        )
      )
      close_front_supported = (
        point.source != "frontRadar"
        or vision_supported
        or cross_sensor_supported
        or point.d_rel >= FRONT_CLOSE_BORN_MIN_DREL_M
        or any(
          value.d_rel >= FRONT_CLOSE_BORN_MIN_DREL_M
          for value in state.observations
        )
        or (
          recent_abs_yaw_max < 0.020
          and reported_inward >= 0.50
          and inward_progress >= 0.30
          and direction_consistency >= 0.75
        )
      )
      common_ok = (
        existence_supported
        and started_outside
        and front_range_ok
        and close_front_supported
        and (
          point.source != "frontRadar"
          or not vision_required_front
          or (
            recent_abs_yaw_max < 0.020
            and abs(point.v_rel) <= 5.0
          )
        )
        and (
          point.source != "frontRadar"
          or not vision_required_front
          or vision_supported
          or front_history_supported
        )
        and MIN_MOVING_VLEAD_MPS < point.v_lead
        and 0.8 < point.d_rel <= CUTIN_MAX_DREL_M
        and motion_reliable
        and front_curve_motion_supported
        and not curve_alias
        and curve_motion_supported
        and slot_motion_supported
        and corner_motion_plausible
        and uncorroborated_away_range_supported
        and (
          not point.source.startswith("corner")
          or vision_supported
          or cross_sensor_supported
          or reported_inward >= 0.50
        )
      )
      # A side reflection often reports a briefly slow closing velocity just
      # before it passes the bumper. Keep the longitudinal history in the
      # gate so one optimistic vRel sample cannot turn that pass into leadTwo.
      front_closing_speed = max(
        -point.v_rel, -relative_path_speed, 0.0,
      )
      front_time_to_pass_s = (
        point.d_rel / front_closing_speed
        if front_closing_speed > 0.1 else math.inf
      )
      close_low_speed_entry = (
        v_ego <= 12.0
        and point.d_rel <= 6.0
        and front_time_to_pass_s >= 1.20
      )
      front_entry = (
        (
          raw_body_overlap
          and front_time_to_pass_s >= 1.20
          and consistent_motion
        )
        or (
          vision_supported
          and predicted_overlap
          and inward_progress >= 0.18
          and supported_inward_rate >= 0.20
          and direction_consistency >= 0.75
        )
        or (
          front_history_supported
          and recent_predicted_overlap
        )
      )
      corner_approach_ok = (
        point.v_rel <= 0.5
        or current_overlap
        or abs(projection.d_path) <= 2.35
        or reported_inward >= 0.43
      )
      corner_commitment = (
        current_overlap
        or reported_inward >= 0.43
        or abs(projection.d_path) <= 2.30
        or (
          inward_progress >= 0.60
          and direction_consistency >= 0.90
        )
      )
      projected_corner_entry_supported = (
        point.d_rel > 8.0
        or vision_supported
      )
      corner_entry = (
        corner_approach_ok
        and corner_commitment
        and (
          (current_overlap and consistent_motion)
          or (
            predicted_overlap
            and strong_predicted_motion
            and projected_corner_entry_supported
          )
        )
      )
      raw_cutin = common_ok and (
        front_entry
        if point.source == "frontRadar"
        else corner_entry or paired_close_entry or close_direct_entry
      )
      cutin_confirmation_s = (
        0.0
        if paired_close_entry
        or front_history_supported
        or (point.source == "frontRadar" and raw_body_overlap)
        else max(0.0, (
          CUTIN_CURRENT_OVERLAP_CONFIRMATION_S
          if current_overlap
          else CUTIN_CONFIRMATION_S
        ) + 0.05 * (3 - self.sensitivity))
      )
      confirmed_cutin = _update_latch(
        state,
        time_s,
        raw_cutin,
        risk=False,
        confirmation_s=cutin_confirmation_s,
        hold_s=CUTIN_HOLD_S,
      )
      trajectory_reversed = (
        not raw_cutin
        and not current_overlap
        and not predicted_overlap
        and not recent_predicted_overlap
        and short_inward_progress < 0.05
        and short_direction_consistency < 0.35
        and not vision_supported
        and not cross_sensor_supported
      )
      if trajectory_reversed or curve_alias or not front_curve_motion_supported:
        # A hold bridges brief radar jitter, but must not resurrect a candidate
        # whose recent physical motion has clearly stopped or reversed.
        state.cutin_until_s = -math.inf
        confirmed_cutin = False
      if (
        vision_required_front
        and point.source == "frontRadar"
        and (
          recent_abs_yaw_max >= 0.020
          or abs(point.v_rel) > 5.0
        )
      ):
        confirmed_cutin = False
      closing_time_s = (
        point.d_rel / -point.v_rel if point.v_rel < -0.1 else math.inf
      )
      strong_consistent_entry = (
        inward_progress >= 0.60
        and abs(inward_rate - reported_inward) <= 0.30
      )
      lead_role_relevant = (
        point.v_rel >= 0.5
        or point.d_rel <= 15.0
        or closing_time_s <= 5.0
        or strong_consistent_entry
      )
      # Lead-role promotion is independent of braking need, but still requires
      # physical lane-entry geometry. This keeps an early adjacent-lane alert
      # out of leadTwo while allowing a confirmed vehicle that is pulling away
      # to occupy the auxiliary lead role. The planner and pre-deceleration
      # path separately decide whether longitudinal action is necessary.
      control_eligible = (
        confirmed_cutin
        and lead_role_relevant
        and (
          raw_body_overlap
          if point.source == "frontRadar"
          else (
            current_overlap
            or paired_close_entry
            or close_direct_entry
            or (
              time_to_overlap_s is not None
              and time_to_overlap_s <= 1.90
              and corner_commitment
            )
          )
        )
      )

      raw_risk = (
        common_ok
        and 2.0 < point.d_rel <= 45.0
        and point.v_rel <= -0.5
        and time_to_overlap_s is not None
        and time_to_overlap_s <= 3.0
        and (ahead_at_overlap or close_low_speed_entry)
        and inward_progress >= 0.25
        and supported_inward_rate >= 0.12
        and direction_consistency >= 0.60
        and (
          point.source == "frontRadar"
          or reported_inward >= 0.40
          or abs(projection.d_path) <= 2.30
          or inward_progress >= 0.60
          or point.d_rel <= 6.0
        )
      )
      predecel_risk = _update_latch(
        state,
        time_s,
        raw_risk,
        risk=True,
        confirmation_s=max(
          0.0, RISK_CONFIRMATION_S + 0.05 * (3 - self.sensitivity),
        ),
        hold_s=RISK_HOLD_S,
      )
      if predecel_risk and (
        point.v_rel >= -0.1
        or time_to_overlap_s is None
        or curve_alias
        or not front_curve_motion_supported
      ):
        # The planner independently rejects a non-closing risk. Clear it here
        # as well so the validator and published radarState describe the same
        # action that the vehicle can actually take.
        state.risk_until_s = -math.inf
        predecel_risk = False

      current_path = (
        existence_supported
        and history_s >= 0.25
        and abs(projection.d_path) <= 1.0
        and point.v_lead > 2.0
        and point.d_rel > 0.8
      )
      support_score = max(
        float(vision_supported), float(cross_sensor_supported),
      )
      confidence = min(1.0, max(0.0,
        0.25 * min(history_s / 0.50, 1.0)
        + 0.25 * min(inward_progress / 0.50, 1.0)
        + 0.20 * direction_consistency
        + 0.15 * min(supported_inward_rate / 0.75, 1.0)
        + 0.15 * support_score
      ))
      reason = (
        "confirmed trajectory CUT-IN" if confirmed_cutin
        else "trajectory pre-deceleration" if predecel_risk
        else "uncorroborated close front" if not close_front_supported
        else "corner lateral jitter" if jittering
        else "current path" if current_path
        else "tracking"
      )
      estimates.append(TrajectoryCutInEstimate(
        point=point,
        continuity_id=state.continuity_id,
        d_path=projection.d_path,
        d_path_rate=d_path_rate,
        future_d_rel=future_d_rel,
        future_d_path=future_d_path,
        horizon_s=horizon_s,
        time_to_overlap_s=time_to_overlap_s,
        inward_rate=supported_inward_rate,
        reported_inward_rate=reported_inward,
        inward_progress=inward_progress,
        recent_inward_progress=short_inward_progress,
        lateral_travel=lateral_travel,
        lateral_net_fraction=lateral_net_fraction,
        direction_consistency=direction_consistency,
        recent_direction_consistency=short_direction_consistency,
        recent_v_rel_min=recent_v_rel_min,
        recent_v_rel_spread=recent_v_rel_spread,
        recent_abs_yaw_max=recent_abs_yaw_max,
        history_s=history_s,
        confidence=confidence,
        vision_supported=vision_supported,
        cross_sensor_supported=cross_sensor_supported,
        cross_sensor_track_id=(
          cross_sensor_point.track_id
          if cross_sensor_point is not None else None
        ),
        current_path=current_path,
        raw_cutin=raw_cutin,
        confirmed_cutin=confirmed_cutin,
        control_eligible=control_eligible,
        predecel_risk=predecel_risk,
        jittering=jittering,
        unstable_fast_motion=unstable_fast_lateral_motion,
        front_history_supported=front_history_supported,
        close_front_supported=close_front_supported,
        curve_alias=curve_alias,
        reason=reason,
      ))

    for key, state in tuple(self._tracks.items()):
      if key not in seen and time_s - state.last_seen_s > MAX_OBSERVATION_GAP_S:
        del self._tracks[key]
    self._corner_front_aliases = {
      key: value
      for key, value in self._corner_front_aliases.items()
      if time_s - value[1] <= CROSS_SENSOR_ALIAS_HOLD_S
    }
    self.last_estimates = tuple(estimates)
    return self.last_estimates


__all__ = (
  "PATH_OVERLAP_HALF_WIDTH_M",
  "TrajectoryCutInDetector",
  "TrajectoryCutInEstimate",
  "prediction_horizon_s",
)
