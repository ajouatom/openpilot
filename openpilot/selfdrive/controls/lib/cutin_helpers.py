import math
from collections import deque
from typing import Any

import numpy as np


CUTIN_ENTER_PROB_GAIN = 0.12
CUTIN_DPATH_RATE_WINDOW_S = 0.25
CUTIN_LANE_MOTION_HORIZON_S = 1.5
CUTIN_DEFAULT_ENTER_MIN_INWARD_SPEED = 0.50
CUTIN_ASSOC_MAX_DREL = 1.5
CUTIN_ASSOC_MAX_YREL = 0.75
CUTIN_ASSOC_MAX_VREL = 2.5
CORNER_TRACK_ID_RANGES = ((200, 220), (240, 250))
CORNER_RADAR_SOURCES = ("corner235", "corner180")
CUTIN_FAST_CONFIRM_MAX_DREL = 10.0
CUTIN_FAST_CONFIRM_MIN_INWARD_SPEED = 0.65
CUTIN_URGENT_CONFIRM_MIN_INWARD_SPEED = 0.55
CUTIN_URGENT_CONFIRM_MAX_TIME_GAP_S = 0.6
CUTIN_IMMINENT_MAX_TIME_GAP_S = 0.35
CUTIN_FAST_CONFIRM_MAX_TIME_GAP_S = 1.2
CUTIN_ASSUMED_VEHICLE_HALF_WIDTH = 0.9
CUTIN_FAST_MAX_LANE_BOUNDARY_TTC_S = 0.55
CUTIN_FAST_NEAR_MAX_LANE_BOUNDARY_TTC_S = 0.7
CUTIN_FAST_MIN_RADAR_INWARD_SPEED = 0.2
CUTIN_FAST_MAX_PULL_AWAY_VREL_MPS = 3.0
CUTIN_PROJECTED_BOOST_MIN_TEMPORAL_INWARD_SPEED = 0.3
FRONT_CUTIN_MIN_DREL_M = 5.0
FRONT_CUTIN_MAX_DREL_M = 50.0
FRONT_CUTIN_MAX_ABS_YREL_M = 7.0
CUTIN_MAX_FRAME_Y_JUMP_M = 0.60
FRONT_CUTIN_MIN_CONFIRM_S = 0.30


def is_corner_track_id(track_id: int) -> bool:
  return any(start <= track_id < end for start, end in CORNER_TRACK_ID_RANGES)


def is_corner_radar_source(source: Any) -> bool:
  return str(source) in CORNER_RADAR_SOURCES


def is_front_radar_cutin_enabled(enable_radar_tracks: int, enable_corner_radar: int, car_brand: str) -> bool:
  return car_brand == "hyundai" and enable_radar_tracks == 3 and enable_corner_radar != 2


def is_front_radar_cutin_candidate(track_id: int, radar_source: str, d_rel: float, y_rel: float,
                                   is_corner_radar: bool) -> bool:
  return (
    not is_corner_radar and
    radar_source != "scc" and
    track_id != 0 and
    FRONT_CUTIN_MIN_DREL_M <= d_rel <= FRONT_CUTIN_MAX_DREL_M and
    abs(y_rel) <= FRONT_CUTIN_MAX_ABS_YREL_M
  )


def is_cutin_track_discontinuous(prev_measured: bool, prev_d_rel: float, prev_y_rel: float, prev_v_lead: float,
                                 d_rel: float, y_rel: float, v_lead: float) -> bool:
  return prev_measured and (
    abs(d_rel - prev_d_rel) > 5.0 or
    abs(y_rel - prev_y_rel) > CUTIN_MAX_FRAME_Y_JUMP_M or
    abs(v_lead - prev_v_lead) > 7.0
  )


def cutin_confirmation_frames(base_frames: int, d_rel: float, inward_speed: float, v_ego: float = 0.0) -> int:
  time_gap = d_rel / max(v_ego, 1.0)
  urgent_distance = d_rel < CUTIN_FAST_CONFIRM_MAX_DREL or (v_ego > 5.0 and time_gap < CUTIN_FAST_CONFIRM_MAX_TIME_GAP_S)
  min_inward_speed = (
    CUTIN_URGENT_CONFIRM_MIN_INWARD_SPEED
    if v_ego > 5.0 and time_gap < CUTIN_URGENT_CONFIRM_MAX_TIME_GAP_S
    else CUTIN_FAST_CONFIRM_MIN_INWARD_SPEED
  )
  if urgent_distance and inward_speed >= min_inward_speed:
    return max(2, base_frames - 1)
  return base_frames


def cutin_min_track_age_frames(base_frames: int, d_rel: float, inward_speed: float, v_ego: float = 0.0) -> int:
  time_gap = d_rel / max(v_ego, 1.0)
  urgent_distance = d_rel < CUTIN_FAST_CONFIRM_MAX_DREL or (v_ego > 5.0 and time_gap < CUTIN_FAST_CONFIRM_MAX_TIME_GAP_S)
  if urgent_distance and inward_speed >= CUTIN_URGENT_CONFIRM_MIN_INWARD_SPEED:
    return min(base_frames, 3)
  return base_frames


def is_fast_cutin_entry(
  d_rel: float,
  v_ego: float,
  d_path: float,
  lane_half_width: float,
  inward_speed: float,
  radar_inward_speed: float = 0.0,
  v_rel: float = 0.0,
) -> bool:
  time_gap = d_rel / max(v_ego, 1.0)
  urgent_distance = d_rel < CUTIN_FAST_CONFIRM_MAX_DREL or (v_ego > 5.0 and time_gap < CUTIN_FAST_CONFIRM_MAX_TIME_GAP_S)
  min_inward_speed = (
    CUTIN_URGENT_CONFIRM_MIN_INWARD_SPEED
    if v_ego > 5.0 and time_gap < CUTIN_URGENT_CONFIRM_MAX_TIME_GAP_S
    else CUTIN_FAST_CONFIRM_MIN_INWARD_SPEED
  )
  if (
    not urgent_distance
    or inward_speed < min_inward_speed
    or radar_inward_speed < CUTIN_FAST_MIN_RADAR_INWARD_SPEED
    or v_rel > CUTIN_FAST_MAX_PULL_AWAY_VREL_MPS
  ):
    return False
  edge_distance = max(0.0, abs(d_path) - lane_half_width - CUTIN_ASSUMED_VEHICLE_HALF_WIDTH)
  max_boundary_ttc = (
    CUTIN_FAST_NEAR_MAX_LANE_BOUNDARY_TTC_S
    if d_rel < CUTIN_FAST_CONFIRM_MAX_DREL
    else CUTIN_FAST_MAX_LANE_BOUNDARY_TTC_S
  )
  return edge_distance / max(inward_speed, 0.1) < max_boundary_ttc


def effective_cutin_inward_speed(
  d_rel: float,
  v_ego: float,
  temporal_inward_speed: float,
  d_path: float,
  projected_d_path: float,
  horizon_s: float,
) -> float:
  time_gap = d_rel / max(v_ego, 1.0)
  urgent_distance = d_rel < CUTIN_FAST_CONFIRM_MAX_DREL or (v_ego > 5.0 and time_gap < CUTIN_FAST_CONFIRM_MAX_TIME_GAP_S)
  if (
    not urgent_distance
    or horizon_s <= 0.0
    or temporal_inward_speed < CUTIN_PROJECTED_BOOST_MIN_TEMPORAL_INWARD_SPEED
  ):
    return temporal_inward_speed
  projected_inward_speed = max(0.0, (abs(d_path) - abs(projected_d_path)) / horizon_s)
  return max(temporal_inward_speed, projected_inward_speed)


def cutin_tuning_from_sensitivity(sensitivity: float) -> dict[str, float]:
  s = float(np.clip(sensitivity, 0.0, 100.0))
  xp = [0.0, 50.0, 100.0]
  return {
    "horizon_s": float(np.interp(s, xp, [0.5, 1.5, 2.5])),
    "confirm_s": float(np.interp(s, xp, [0.35, 0.15, 0.12])),
    "min_track_age_s": float(np.interp(s, xp, [0.50, 0.25, 0.10])),
    "enter_min_x": float(np.interp(s, xp, [3.0, 1.0, 0.5])),
    "enter_max_x": float(np.interp(s, xp, [50.0, 55.0, 65.0])),
    "enter_min_abs_dpath": float(np.interp(s, xp, [1.9, 1.5, 1.2])),
    "enter_future_in_lane_prob": float(np.interp(s, xp, [0.30, 0.15, 0.08])),
    "enter_centering_gain": float(np.interp(s, xp, [0.30, 0.18, 0.10])),
    "enter_min_inward_speed": float(np.interp(s, xp, [0.65, 0.50, 0.35])),
    "enter_min_progress": float(np.interp(s, xp, [0.08, 0.04, 0.03])),
  }


def new_cutin_position_history(dt: float) -> deque[tuple[float, float, float]]:
  frame_count = max(2, int(round(CUTIN_DPATH_RATE_WINDOW_S / dt)) + 1)
  return deque(maxlen=frame_count)


def associate_cutin_tracks(
  previous: dict[int, tuple[float, float, float]],
  current: dict[int, tuple[float, float, float]],
) -> dict[int, int]:
  candidates: list[tuple[float, int, int]] = []
  for current_id, (d_rel, y_rel, v_rel) in current.items():
    for previous_id, (prev_d_rel, prev_y_rel, prev_v_rel) in previous.items():
      d_delta = abs(d_rel - prev_d_rel)
      y_delta = abs(y_rel - prev_y_rel)
      v_delta = abs(v_rel - prev_v_rel)
      if d_delta > CUTIN_ASSOC_MAX_DREL or y_delta > CUTIN_ASSOC_MAX_YREL or v_delta > CUTIN_ASSOC_MAX_VREL:
        continue
      score = d_delta / CUTIN_ASSOC_MAX_DREL + y_delta / CUTIN_ASSOC_MAX_YREL + v_delta / CUTIN_ASSOC_MAX_VREL
      if current_id == previous_id:
        score -= 0.05
      candidates.append((score, current_id, previous_id))

  associations: dict[int, int] = {}
  used_previous: set[int] = set()
  for _, current_id, previous_id in sorted(candidates):
    if current_id in associations or previous_id in used_previous:
      continue
    associations[current_id] = previous_id
    used_previous.add(previous_id)
  return associations


def update_lane_relative_motion(
  history: deque[tuple[float, float, float]],
  d_rel: float,
  y_rel: float,
  lane_xs: Any,
  left_ys: Any,
  right_ys: Any,
  measured: bool,
  discontinuous: bool,
  dt: float,
) -> tuple[float, float]:
  if not measured or discontinuous:
    history.clear()
  if not measured:
    return 0.0, 0.0

  current_center_y = (
    np.interp(d_rel, lane_xs, left_ys)
    + np.interp(d_rel, lane_xs, right_ys)
  ) / 2.0
  history.append((float(d_rel), float(y_rel), float(y_rel + current_center_y)))
  if discontinuous or len(history) < history.maxlen:
    return 0.0, 0.0

  projected_d_paths = []
  observed_d_paths = []
  for sample_d_rel, sample_y_rel, sample_d_path in history:
    center_y = (
      np.interp(sample_d_rel, lane_xs, left_ys)
      + np.interp(sample_d_rel, lane_xs, right_ys)
    ) / 2.0
    projected_d_paths.append(float(sample_y_rel + center_y))
    observed_d_paths.append(sample_d_path)

  sample_count = len(history)
  mean_index = (sample_count - 1) / 2.0
  centered_indices = [index - mean_index for index in range(sample_count)]
  denominator = dt * sum(index * index for index in centered_indices)
  projected_rate = float(
    sum(index * value for index, value in zip(centered_indices, projected_d_paths, strict=True)) / denominator
  )
  observed_rate = float(
    sum(index * value for index, value in zip(centered_indices, observed_d_paths, strict=True)) / denominator
  )
  side = math.copysign(1.0, observed_d_paths[-1])
  projected_inward_speed = max(0.0, -side * projected_rate)
  observed_inward_speed = max(0.0, -side * observed_rate)
  inward_speed = min(projected_inward_speed, observed_inward_speed)
  if inward_speed > 0.0:
    return -side * inward_speed, inward_speed
  return observed_rate, 0.0


def combine_cutin_future_projection(
  d_path: float,
  d_path_rate: float,
  horizon_s: float,
  lane_half_width: float,
  projected_d_path: float,
  projected_in_lane_prob: float,
) -> tuple[float, float]:
  motion_horizon = min(horizon_s, CUTIN_LANE_MOTION_HORIZON_S)
  motion_d_path = d_path + d_path_rate * motion_horizon
  motion_in_lane_prob = max(0.0, 1.0 - abs(motion_d_path) / max(lane_half_width, 0.1))
  moving_inward = abs(motion_d_path) < abs(d_path)
  if moving_inward and motion_in_lane_prob > projected_in_lane_prob:
    return motion_d_path, motion_in_lane_prob
  return projected_d_path, projected_in_lane_prob


def cutin_entry_rejection_reason(
  *,
  enabled: bool,
  lane_line_available: bool,
  corner_track: bool,
  closer_or_matching: bool,
  track_count: int,
  min_track_age: int,
  d_rel: float,
  v_lead: float,
  d_path: float,
  d_path_future: float,
  in_lane_prob: float,
  in_lane_prob_future: float,
  inward_speed: float,
  tuning: dict[str, Any],
  fast_lane_entry: bool = False,
) -> str | None:
  if not enabled:
    return "disabled"
  if not lane_line_available:
    return "lane-lines"
  if not corner_track:
    return "not-corner"
  if not closer_or_matching:
    return "behind-lead"
  if track_count < min_track_age:
    return "track-age"
  if not (tuning["enter_min_x"] < d_rel < tuning["enter_max_x"] and v_lead > 4.0):
    return "range-speed"
  if abs(d_path) < tuning["enter_min_abs_dpath"]:
    return "already-center"
  if not fast_lane_entry:
    if in_lane_prob_future < tuning["enter_future_in_lane_prob"]:
      return "future-lane"
    if (in_lane_prob_future - in_lane_prob) < CUTIN_ENTER_PROB_GAIN:
      return "prob-gain"
    if (abs(d_path) - abs(d_path_future)) < tuning["enter_centering_gain"]:
      return "center-gain"
  if inward_speed < tuning["enter_min_inward_speed"]:
    return "lane-motion"
  return None


def update_cutin_confirmation(
  count: int,
  start_abs_dpath: float,
  d_path: float,
  d_rel: float,
  entering: bool,
  keeping: bool,
  confirm_frames: int,
  sticky_frames: int,
  base_required_progress: float,
  v_ego: float = 0.0,
) -> tuple[int, float]:
  if entering:
    if count >= confirm_frames:
      return sticky_frames, start_abs_dpath
    if count <= 0:
      start_abs_dpath = abs(d_path)
    next_count = count + 1
    distance_progress = max(0.0, d_rel - 25.0) * 0.005
    required_progress = base_required_progress + distance_progress
    if v_ego > 5.0 and d_rel / v_ego < CUTIN_IMMINENT_MAX_TIME_GAP_S:
      required_progress = 0.0
    if next_count >= confirm_frames and start_abs_dpath - abs(d_path) >= required_progress:
      return sticky_frames, start_abs_dpath
    return min(next_count, confirm_frames - 1), start_abs_dpath
  if keeping and count > 0:
    return max(count - 1, 0), start_abs_dpath
  return 0, abs(d_path)
