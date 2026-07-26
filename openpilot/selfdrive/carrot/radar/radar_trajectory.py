from __future__ import annotations

from collections import defaultdict, deque
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
import math
from typing import Any


DEFAULT_HORIZONS_S = tuple(index * 0.25 for index in range(9))
TARGET_HORIZONS_S = (0.5, 1.0, 1.5, 2.0)
MODEL_HISTORY_AGES_S = (0.0, 0.25, 0.5, 0.75, 1.0)
MODEL_HISTORY_ENDPOINT_TOLERANCE_S = 0.03
TRACK_CONTINUITY_MAX_GAP_S = 0.35
LIVE_POSE_MAX_AGE_S = 0.20

POINT_FEATURE_NAMES = (
  "v_ego",
  "d_rel",
  "y_rel",
  "v_rel",
  "a_rel",
  "yv_rel",
  "v_lead",
  "a_lead",
  "j_lead",
  "closing_speed",
  "ttc_s",
)

TRAJECTORY_BASE_FEATURE_NAMES = (
  "history_count",
  "d_path",
  "d_path_rate",
  "d_path_rate_sigma",
  "steering_angle_deg",
  "steering_rate_deg_s",
  "yaw_rate_rad_s",
  "yaw_rate_estimated",
  "ego_rotation_lateral_speed",
  "turn_ambiguity",
  "lane_half_width",
  "lane_reliable",
  "lane_probability",
  "path_sigma",
)


def _suffix(value: float) -> str:
  return f"{value:.2f}".replace(".", "p")


TRAJECTORY_HISTORY_FEATURE_NAMES = tuple(
  f"history_{name}_{_suffix(age_s)}"
  for age_s in MODEL_HISTORY_AGES_S
  for name in ("valid", "d_rel", "y_rel", "d_path", "v_rel", "yv_rel")
)
TRAJECTORY_MODEL_FEATURE_NAMES = (
  *POINT_FEATURE_NAMES,
  *TRAJECTORY_BASE_FEATURE_NAMES,
  *TRAJECTORY_HISTORY_FEATURE_NAMES,
)


@dataclass(frozen=True)
class RadarTrajectorySample:
  horizon_s: float
  d_rel: float
  y_rel: float
  d_path: float
  lateral_sigma: float
  occupancy_prob: float
  lane_half_width: float = 1.8


@dataclass(frozen=True)
class RadarTrajectoryHistorySample:
  age_s: float
  d_rel: float
  y_rel: float
  d_path: float
  v_rel: float
  yv_rel: float


@dataclass(frozen=True)
class RadarTrajectory:
  track_id: int
  source: str
  continuity_id: int
  history_count: int
  d_path: float
  d_path_rate: float
  rate_sigma: float
  steering_angle_deg: float
  steering_rate_deg_s: float
  yaw_rate_rad_s: float
  yaw_rate_estimated: bool
  ego_rotation_lateral_speed: float
  turn_ambiguity: float
  lane_half_width: float
  lane_reliable: bool
  lane_probability: float
  path_sigma: float
  entry_probability: float
  time_to_entry_s: float | None
  reason: str
  history: tuple[RadarTrajectoryHistorySample, ...]
  samples: tuple[RadarTrajectorySample, ...]


@dataclass(frozen=True)
class _HistoryPoint:
  time_s: float
  d_rel: float
  y_rel: float
  d_path: float
  v_rel: float
  yv_rel: float


def _finite(value: Any, default: float = 0.0) -> float:
  try:
    result = float(value)
  except (TypeError, ValueError):
    return default
  return result if math.isfinite(result) else default


def _attribute(value: Any, snake_name: str, camel_name: str, default: Any = None) -> Any:
  if hasattr(value, snake_name):
    return getattr(value, snake_name)
  return getattr(value, camel_name, default)


def radar_point_track_id(point: Any) -> int:
  return int(_attribute(point, "track_id", "trackId"))


def radar_point_source(point: Any) -> str:
  source = str(_attribute(point, "source", "radarSource", "frontRadar"))
  track_id = radar_point_track_id(point)
  if source == "frontRadar":
    if 200 <= track_id < 220:
      return "corner235"
    if 240 <= track_id < 250:
      return "corner180"
    if 300 <= track_id < 412:
      return "corner430"
  return source


def radar_point_measured(point: Any) -> bool:
  return bool(_attribute(point, "measured", "measured", True))


def radar_point_value(point: Any, snake_name: str, camel_name: str, default: float = 0.0) -> float:
  return _finite(_attribute(point, snake_name, camel_name, default), default)


def estimated_yaw_rate_rad_s(
  v_ego: float,
  steering_angle_deg: float,
  steer_ratio: float = 14.0,
  wheelbase: float = 2.8,
) -> float:
  """Estimate ego yaw when a platform leaves carState.yawRate at zero."""
  ratio = max(abs(_finite(steer_ratio, 14.0)), 1.0)
  base = max(abs(_finite(wheelbase, 2.8)), 1.5)
  road_wheel_angle = math.radians(_finite(steering_angle_deg) / ratio)
  # livePose's device-frame z is negative for a positive/left steering turn.
  return -_finite(v_ego) * math.tan(road_wheel_angle) / base


def live_pose_yaw_rate_rad_s(live_pose: Any | None) -> float | None:
  """Return validated device-frame yaw rate from livePose."""
  if live_pose is None:
    return None
  angular_velocity = getattr(live_pose, "angularVelocityDevice", None)
  if (
    angular_velocity is None
    or not bool(getattr(angular_velocity, "valid", False))
    or not bool(getattr(live_pose, "inputsOK", False))
    or not bool(getattr(live_pose, "sensorsOK", False))
  ):
    return None
  value = _finite(getattr(angular_velocity, "z", math.nan), math.nan)
  return value if math.isfinite(value) else None


def ego_yaw_rate_rad_s(
  v_ego: float,
  steering_angle_deg: float,
  live_pose: Any | None = None,
  live_pose_age_s: float = math.inf,
  steer_ratio: float = 14.0,
  wheelbase: float = 2.8,
) -> tuple[float, bool, str]:
  """Select fresh livePose, then the unit-consistent steering fallback."""
  pose_rate = live_pose_yaw_rate_rad_s(live_pose)
  if pose_rate is not None and 0.0 <= _finite(live_pose_age_s, math.inf) <= LIVE_POSE_MAX_AGE_S:
    return pose_rate, False, "livePose"
  return (
    estimated_yaw_rate_rad_s(
      v_ego,
      steering_angle_deg,
      steer_ratio,
      wheelbase,
    ),
    True,
    "steering",
  )


def yaw_compensated_lateral_rate(
  yv_rel: float,
  yaw_rate_rad_s: float,
  d_rel: float,
) -> float:
  """Remove livePose/device-yaw motion from the radar lateral rate."""
  return _finite(yv_rel) - _finite(yaw_rate_rad_s) * max(_finite(d_rel), 0.0)


def _line_y(points: Sequence[tuple[float, float]], distance: float) -> float:
  if not points:
    return 0.0
  if distance <= points[0][0]:
    return -points[0][1]
  for first, second in zip(points, points[1:], strict=False):
    if distance <= second[0]:
      span = max(second[0] - first[0], 1e-3)
      ratio = (distance - first[0]) / span
      return -(first[1] + ratio * (second[1] - first[1]))
  return -points[-1][1]


def _line_value(points: Sequence[tuple[float, float]], distance: float, default: float) -> float:
  if not points:
    return default
  if distance <= points[0][0]:
    return points[0][1]
  for first, second in zip(points, points[1:], strict=False):
    if distance <= second[0]:
      span = max(second[0] - first[0], 1e-3)
      ratio = (distance - first[0]) / span
      return first[1] + ratio * (second[1] - first[1])
  return points[-1][1]


def _normal_cdf(value: float) -> float:
  return 0.5 * (1.0 + math.erf(value / math.sqrt(2.0)))


def _interval_probability(center: float, sigma: float, limit: float) -> float:
  sigma = max(sigma, 0.05)
  return min(
    1.0,
    max(0.0, _normal_cdf((limit - center) / sigma) - _normal_cdf((-limit - center) / sigma)),
  )


def path_occupancy_probability(
  d_path_mean: float,
  d_path_sigma: float,
  lane_half_width: float,
) -> float:
  """Probability that a predicted radar-return center lies within the path."""
  return _interval_probability(d_path_mean, d_path_sigma, lane_half_width)


def forward_probability(
  d_rel_mean: float,
  d_rel_sigma: float,
  minimum_d_rel: float = 0.5,
) -> float:
  """Probability that a predicted radar return remains ahead of ego."""
  sigma = max(_finite(d_rel_sigma), 0.05)
  return min(
    1.0,
    max(0.0, _normal_cdf((_finite(d_rel_mean) - minimum_d_rel) / sigma)),
  )


def _linear_rate(history: Sequence[_HistoryPoint]) -> tuple[float, float]:
  if len(history) < 2:
    return 0.0, 1.5
  origin = history[-1].time_s
  xs = [point.time_s - origin for point in history]
  ys = [point.d_path for point in history]
  mean_x = sum(xs) / len(xs)
  mean_y = sum(ys) / len(ys)
  denominator = sum((value - mean_x) ** 2 for value in xs)
  if denominator < 1e-5:
    return 0.0, 1.5
  rate = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys, strict=True)) / denominator
  residuals = [y - (mean_y + rate * (x - mean_x)) for x, y in zip(xs, ys, strict=True)]
  residual_sigma = math.sqrt(sum(value * value for value in residuals) / max(len(residuals) - 1, 1))
  duration = max(origin - history[0].time_s, 0.05)
  rate_sigma = max(0.12, residual_sigma / duration)
  return min(max(rate, -5.0), 5.0), min(rate_sigma, 3.0)


def _sensor_sigma(source: str, distance: float) -> float:
  if source.startswith("corner"):
    return 0.25 + min(max(distance, 0.0), 80.0) * 0.003
  if source == "scc":
    return 0.65
  near_penalty = max(0.0, 8.0 - distance) * 0.10
  return 0.48 + near_penalty + min(max(distance, 0.0), 100.0) * 0.002


def _lane_state(
  distance: float,
  y_rel: float,
  path: Sequence[tuple[float, float]],
  lane_lines: Sequence[Sequence[tuple[float, float]]],
  lane_probs: Sequence[float],
) -> tuple[float, float, float, bool, float]:
  path_y = _line_y(path, distance)
  lane_probability = min(lane_probs[1:3], default=0.0)
  lane_reliable = (
    len(lane_lines) >= 3
    and bool(lane_lines[1])
    and bool(lane_lines[2])
    and len(lane_probs) >= 3
    and lane_probs[1] >= 0.30
    and lane_probs[2] >= 0.30
  )
  if not lane_reliable:
    return path_y, y_rel - path_y, 1.8, False, lane_probability
  left = _line_y(lane_lines[1], distance)
  right = _line_y(lane_lines[2], distance)
  center = 0.5 * (left + right)
  return center, y_rel - center, min(2.2, max(1.5, 0.5 * abs(left - right))), True, lane_probability


def path_relative_state(
  distance: float,
  y_rel: float,
  path: Sequence[tuple[float, float]],
  lane_lines: Sequence[Sequence[tuple[float, float]]],
  lane_probs: Sequence[float],
) -> tuple[float, float, float, bool, float]:
  """Return path center, path-relative lateral position, and current lane metadata."""
  return _lane_state(distance, y_rel, path, lane_lines, lane_probs)


def path_center_occupied(d_path: float, lane_half_width: float) -> bool:
  """Classify the radar return center, not an assumed vehicle body edge."""
  return abs(d_path) <= lane_half_width


def radar_track_continuous(
  previous: Any,
  current: Any,
  dt: float,
  max_gap_s: float = TRACK_CONTINUITY_MAX_GAP_S,
) -> bool:
  """Reject reused IDs while allowing a short, physically plausible sensor dropout."""
  if not 0.0 < dt <= max_gap_s:
    return False
  previous_d = radar_point_value(previous, "d_rel", "dRel")
  current_d = radar_point_value(current, "d_rel", "dRel")
  previous_y = radar_point_value(previous, "y_rel", "yRel")
  current_y = radar_point_value(current, "y_rel", "yRel")
  previous_v = radar_point_value(previous, "v_rel", "vRel")
  current_v = radar_point_value(current, "v_rel", "vRel")
  previous_yv = radar_point_value(previous, "yv_rel", "yvRel")
  current_yv = radar_point_value(current, "yv_rel", "yvRel")
  previous_v_lead = radar_point_value(previous, "v_lead", "vLead")
  current_v_lead = radar_point_value(current, "v_lead", "vLead")
  predicted_d = previous_d + 0.5 * (previous_v + current_v) * dt
  predicted_y = previous_y + 0.5 * (previous_yv + current_yv) * dt
  d_tolerance = 1.25 + 5.0 * dt
  y_tolerance = 0.65 + 2.0 * dt
  velocity_tolerance = 4.0 + 12.0 * dt
  return (
    abs(current_d - predicted_d) <= d_tolerance
    and abs(current_y - predicted_y) <= y_tolerance
    and abs(current_v - previous_v) <= velocity_tolerance
    and abs(current_v_lead - previous_v_lead) <= velocity_tolerance + 2.0
  )


class RadarTrajectoryAnalyzer:
  """Build source-specific, past-only path-relative trajectories for inference."""

  def __init__(self, horizons_s: Iterable[float] = DEFAULT_HORIZONS_S) -> None:
    horizons = sorted({_finite(value) for value in horizons_s if _finite(value) >= 0.0})
    self.horizons_s = tuple(horizons) or DEFAULT_HORIZONS_S
    self._history: dict[tuple[str, int], deque[_HistoryPoint]] = defaultdict(lambda: deque(maxlen=24))
    self._last_points: dict[tuple[str, int], Any] = {}
    self._continuity_ids: dict[tuple[str, int], int] = defaultdict(int)

  def update(
    self,
    time_s: float,
    points: Iterable[Any],
    path: Sequence[tuple[float, float]],
    lane_lines: Sequence[Sequence[tuple[float, float]]],
    lane_probs: Sequence[float],
    path_y_stds: Sequence[tuple[float, float]] = (),
    lane_stds: Sequence[float] = (),
    steering_angle_deg: float = 0.0,
    steering_rate_deg_s: float = 0.0,
    yaw_rate_rad_s: float = 0.0,
    yaw_rate_estimated: bool = False,
  ) -> dict[tuple[str, int], RadarTrajectory]:
    steering_angle_deg = _finite(steering_angle_deg)
    steering_rate_deg_s = _finite(steering_rate_deg_s)
    yaw_rate_rad_s = _finite(yaw_rate_rad_s)
    output: dict[tuple[str, int], RadarTrajectory] = {}
    seen: set[tuple[str, int]] = set()
    for point in points:
      if not radar_point_measured(point):
        continue
      source = radar_point_source(point)
      track_id = radar_point_track_id(point)
      key = (source, track_id)
      seen.add(key)
      d_rel = radar_point_value(point, "d_rel", "dRel")
      y_rel = radar_point_value(point, "y_rel", "yRel")
      v_rel = radar_point_value(point, "v_rel", "vRel")
      raw_y_rate = radar_point_value(point, "yv_rel", "yvRel")
      compensated_y_rate = yaw_compensated_lateral_rate(
        raw_y_rate, yaw_rate_rad_s, d_rel,
      )
      _, d_path, lane_half_width, lane_reliable, lane_probability = _lane_state(
        d_rel, y_rel, path, lane_lines, lane_probs,
      )

      history = self._history[key]
      last_point = self._last_points.get(key)
      if history and (
        last_point is None
        or not radar_track_continuous(last_point, point, time_s - history[-1].time_s)
      ):
        history.clear()
        self._continuity_ids[key] += 1
      history.append(_HistoryPoint(
        time_s, d_rel, y_rel, d_path, v_rel, compensated_y_rate,
      ))
      self._last_points[key] = point
      while history and time_s - history[0].time_s > 1.2:
        history.popleft()

      fitted_rate, rate_sigma = _linear_rate(tuple(history))
      if len(history) >= 3:
        d_path_rate = 0.80 * fitted_rate + 0.20 * compensated_y_rate
      else:
        d_path_rate = compensated_y_rate
        rate_sigma = max(rate_sigma, 0.9)
      d_path_rate = min(max(d_path_rate, -5.0), 5.0)
      ego_rotation_lateral_speed = abs(yaw_rate_rad_s) * max(d_rel, 0.0)
      turn_ambiguity = min(
        1.0,
        ego_rotation_lateral_speed
        / max(abs(d_path_rate) + ego_rotation_lateral_speed, 0.2),
      )

      samples: list[RadarTrajectorySample] = []
      path_limit = lane_half_width
      lane_std = 0.0
      if lane_reliable and len(lane_stds) >= 3:
        lane_std = 0.5 * math.hypot(_finite(lane_stds[1]), _finite(lane_stds[2]))
      current_path_std = max(0.05, _line_value(path_y_stds, d_rel, 0.35))
      for horizon_s in self.horizons_s:
        future_d = max(0.0, d_rel + v_rel * horizon_s)
        center_y, _, future_half_width, future_lane_reliable, _ = _lane_state(
          future_d, 0.0, path, lane_lines, lane_probs,
        )
        future_d_path = d_path + d_path_rate * horizon_s
        future_y = center_y + future_d_path
        path_std = max(0.05, _line_value(path_y_stds, future_d, 0.35))
        sigma = math.sqrt(
          _sensor_sigma(source, future_d) ** 2
          + path_std ** 2
          + (lane_std if future_lane_reliable else 0.45) ** 2
          + (rate_sigma * horizon_s) ** 2
        )
        occupancy = path_occupancy_probability(
          future_d_path, sigma, future_half_width,
        )
        samples.append(RadarTrajectorySample(
          horizon_s, future_d, future_y, future_d_path, sigma, occupancy,
          future_half_width,
        ))

      inward_speed = (
        -d_path_rate * math.copysign(1.0, d_path)
        if abs(d_path) > 0.05
        else 0.0
      )
      time_to_entry = None
      if path_center_occupied(d_path, path_limit):
        time_to_entry = 0.0
      elif inward_speed > 0.05:
        time_to_entry = (abs(d_path) - path_limit) / inward_speed
      entry_probability = max((sample.occupancy_prob for sample in samples), default=0.0)
      if not lane_reliable and abs(d_path) > 3.5:
        reason = "lane-low / path fallback"
      elif inward_speed <= 0.05:
        reason = "not inward"
      elif time_to_entry is None or time_to_entry > self.horizons_s[-1]:
        reason = "entry beyond horizon"
      elif len(history) < 4:
        reason = "short history"
      else:
        reason = "trajectory candidate"

      history_samples = tuple(
        RadarTrajectoryHistorySample(
          age_s=max(0.0, time_s - item.time_s),
          d_rel=item.d_rel,
          y_rel=item.y_rel,
          d_path=item.d_path,
          v_rel=item.v_rel,
          yv_rel=item.yv_rel,
        )
        for item in history
      )
      output[key] = RadarTrajectory(
        track_id=track_id,
        source=source,
        continuity_id=self._continuity_ids[key],
        history_count=len(history),
        d_path=d_path,
        d_path_rate=d_path_rate,
        rate_sigma=rate_sigma,
        steering_angle_deg=steering_angle_deg,
        steering_rate_deg_s=steering_rate_deg_s,
        yaw_rate_rad_s=yaw_rate_rad_s,
        yaw_rate_estimated=bool(yaw_rate_estimated),
        ego_rotation_lateral_speed=ego_rotation_lateral_speed,
        turn_ambiguity=turn_ambiguity,
        lane_half_width=lane_half_width,
        lane_reliable=lane_reliable,
        lane_probability=lane_probability,
        path_sigma=current_path_std,
        entry_probability=entry_probability,
        time_to_entry_s=time_to_entry,
        reason=reason,
        history=history_samples,
        samples=tuple(samples),
      )

    stale = [
      key for key, history in self._history.items()
      if key not in seen and history and time_s - history[-1].time_s > 1.5
    ]
    for key in stale:
      del self._history[key]
      self._last_points.pop(key, None)
      self._continuity_ids.pop(key, None)
    return output


def trajectory_is_review_candidate(
  trajectory: RadarTrajectory,
  distance: float,
  horizon_s: float,
  probability_threshold: float = 0.60,
) -> bool:
  entry_probability = trajectory_entry_probability(trajectory, horizon_s)
  return (
    1.0 < distance < 65.0
    and trajectory.history_count >= 4
    and trajectory.reason == "trajectory candidate"
    and entry_probability >= probability_threshold
    and trajectory.time_to_entry_s is not None
    and trajectory.time_to_entry_s <= horizon_s
  )


def trajectory_entry_probability(
  trajectory: RadarTrajectory,
  horizon_s: float,
) -> float:
  return max(
    (
      sample.occupancy_prob
      for sample in trajectory.samples
      if sample.horizon_s <= horizon_s + 1e-6
    ),
    default=0.0,
  )


def trajectory_sample_at(
  trajectory: RadarTrajectory,
  horizon_s: float,
) -> RadarTrajectorySample:
  return min(trajectory.samples, key=lambda sample: abs(sample.horizon_s - horizon_s))


def _history_at_age(
  history: Sequence[RadarTrajectoryHistorySample],
  age_s: float,
) -> RadarTrajectoryHistorySample | None:
  """Interpolate a measured past trajectory at one fixed model age."""
  if not history:
    return None
  ordered = sorted(history, key=lambda sample: sample.age_s)
  if age_s < ordered[0].age_s - MODEL_HISTORY_ENDPOINT_TOLERANCE_S:
    return None
  if age_s > ordered[-1].age_s + MODEL_HISTORY_ENDPOINT_TOLERANCE_S:
    return None
  if age_s <= ordered[0].age_s:
    return ordered[0]
  if age_s >= ordered[-1].age_s:
    return ordered[-1]
  for recent, older in zip(ordered, ordered[1:], strict=False):
    if age_s > older.age_s:
      continue
    span = max(older.age_s - recent.age_s, 1e-6)
    ratio = (age_s - recent.age_s) / span
    return RadarTrajectoryHistorySample(
      age_s=age_s,
      d_rel=recent.d_rel + ratio * (older.d_rel - recent.d_rel),
      y_rel=recent.y_rel + ratio * (older.y_rel - recent.y_rel),
      d_path=recent.d_path + ratio * (older.d_path - recent.d_path),
      v_rel=recent.v_rel + ratio * (older.v_rel - recent.v_rel),
      yv_rel=recent.yv_rel + ratio * (older.yv_rel - recent.yv_rel),
    )
  return None


def trajectory_feature_row(
  trajectory: RadarTrajectory,
) -> dict[str, float]:
  """Return current and past-only trajectory inputs for source-specific models."""
  row = {
    "history_count": float(trajectory.history_count),
    "d_path": trajectory.d_path,
    "d_path_rate": trajectory.d_path_rate,
    "d_path_rate_sigma": trajectory.rate_sigma,
    "steering_angle_deg": trajectory.steering_angle_deg,
    "steering_rate_deg_s": trajectory.steering_rate_deg_s,
    "yaw_rate_rad_s": trajectory.yaw_rate_rad_s,
    "yaw_rate_estimated": float(trajectory.yaw_rate_estimated),
    "ego_rotation_lateral_speed": trajectory.ego_rotation_lateral_speed,
    "turn_ambiguity": trajectory.turn_ambiguity,
    "lane_half_width": trajectory.lane_half_width,
    "lane_reliable": float(trajectory.lane_reliable),
    "lane_probability": trajectory.lane_probability,
    "path_sigma": trajectory.path_sigma,
  }
  for age_s in MODEL_HISTORY_AGES_S:
    history = _history_at_age(trajectory.history, age_s)
    suffix = f"{age_s:.2f}".replace(".", "p")
    row.update({
      f"history_valid_{suffix}": float(history is not None),
      f"history_d_rel_{suffix}": history.d_rel if history is not None else 0.0,
      f"history_y_rel_{suffix}": history.y_rel if history is not None else 0.0,
      f"history_d_path_{suffix}": history.d_path if history is not None else 0.0,
      f"history_v_rel_{suffix}": history.v_rel if history is not None else 0.0,
      f"history_yv_rel_{suffix}": history.yv_rel if history is not None else 0.0,
    })
  return row


def trajectory_model_feature_row(
  trajectory: RadarTrajectory,
  point: Any,
  v_ego: float,
) -> dict[str, float]:
  """Build the source-independent model row used by training and production."""
  d_rel = radar_point_value(point, "d_rel", "dRel")
  v_rel = radar_point_value(point, "v_rel", "vRel")
  closing_speed = max(0.0, -v_rel)
  row = {
    "v_ego": _finite(v_ego),
    "d_rel": d_rel,
    "y_rel": radar_point_value(point, "y_rel", "yRel"),
    "v_rel": v_rel,
    "a_rel": radar_point_value(point, "a_rel", "aRel"),
    "yv_rel": yaw_compensated_lateral_rate(
      radar_point_value(point, "yv_rel", "yvRel"),
      trajectory.yaw_rate_rad_s,
      d_rel,
    ),
    "v_lead": radar_point_value(point, "v_lead", "vLead", v_ego + v_rel),
    "a_lead": radar_point_value(point, "a_lead", "aLead"),
    "j_lead": radar_point_value(point, "j_lead", "jLead"),
    "closing_speed": closing_speed,
    "ttc_s": min(20.0, d_rel / max(closing_speed, 0.1)) if closing_speed > 0.1 else 20.0,
  }
  row.update(trajectory_feature_row(trajectory))
  if tuple(row) != TRAJECTORY_MODEL_FEATURE_NAMES:
    raise RuntimeError("trajectory model feature schema drift")
  return row
