#!/usr/bin/env python3
"""Train front/corner future path-occupancy models from measured radar tracks."""

from __future__ import annotations

import argparse
import bisect
from collections import defaultdict
from collections.abc import Sequence
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.radar_trajectory import (
  RadarTrajectoryAnalyzer,
  TARGET_HORIZONS_S,
  TRAJECTORY_MODEL_FEATURE_NAMES,
  path_center_occupied,
  path_relative_state,
  radar_point_measured,
  radar_point_source,
  radar_point_track_id,
  radar_track_continuous,
  trajectory_model_feature_row,
)
from openpilot.selfdrive.carrot.radar.radar_trajectory_model import (
  MODEL_VERSION,
  OUTPUT_HEAD_NAMES,
  POSITION_TARGET_NAMES,
  RadarTrajectoryRuntime,
  kinematic_position_baseline,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import RadarFrame, RadarPoint, load_frames


FUTURE_TIME_TOLERANCE_S = 0.09
MIN_HISTORY_COUNT = 3
MIN_SAMPLE_PERIOD_S = 0.20
MAX_NEGATIVE_TO_POSITIVE = 6
MAX_NEGATIVE_ROWS_WITHOUT_POSITIVE = 1200
DATASET_CACHE_VERSION = 8
LOG_CACHE_VERSION = 8
ENTRY_LONGITUDINAL_WEIGHT = 1.5
ENTRY_LATERAL_WEIGHT = 4.0
EXIT_LONGITUDINAL_WEIGHT = 1.25
EXIT_LATERAL_WEIGHT = 2.0
ENTRY_TARGET_PRECISION = 0.80
RLOG_PATTERN = re.compile(r"^rlog(?:\.\d+)?\.zst$", re.IGNORECASE)


@dataclass(frozen=True)
class Dataset:
  features: np.ndarray
  labels: np.ndarray
  lane_half_widths: np.ndarray
  valid: np.ndarray
  current_occupancy: np.ndarray
  sample_ids: np.ndarray
  log_groups: np.ndarray


@dataclass(frozen=True)
class TrackObservation:
  frame_index: int
  time_s: float
  point: RadarPoint
  episode_id: int


@dataclass(frozen=True)
class EvaluationLabel:
  label_id: str
  vehicle_folder: str
  log: str
  source: str
  start_s: float
  end_s: float
  track_ids: tuple[int, ...]
  expected: str

  @property
  def log_group(self) -> str:
    return f"{self.vehicle_folder}/{self.log}"


@dataclass(frozen=True)
class EvaluationRow:
  label_id: str
  source: str
  expected: str
  predicted: str
  selected: bool
  max_probability: float
  max_path_exit_probability: float
  horizon_probabilities: tuple[float, ...]
  horizon_out_probabilities: tuple[float, ...]
  scored_frames: int
  actual_entry_frames: int
  actual_exit_frames: int
  actual_valid_frames: int
  manual_matches_actual: bool | None


def _source_mode(source: str) -> str | None:
  if source.startswith("corner"):
    return "corner"
  if source in ("frontRadar", "scc"):
    return "front"
  return None


def _usable_point(point: RadarPoint) -> bool:
  return (
    radar_point_measured(point)
    and _source_mode(radar_point_source(point)) is not None
    and 0.75 < point.d_rel < 160.0
    and abs(point.y_rel) < 12.0
    and all(math.isfinite(value) for value in (
      point.d_rel, point.y_rel, point.v_rel, point.yv_rel, point.v_lead,
    ))
  )


def _track_observations(
  frames: Sequence[RadarFrame],
) -> tuple[
  dict[tuple[int, str, int], TrackObservation],
  dict[tuple[str, int, int], tuple[TrackObservation, ...]],
]:
  by_frame: dict[tuple[int, str, int], TrackObservation] = {}
  episodes: dict[tuple[str, int, int], list[TrackObservation]] = defaultdict(list)
  last: dict[tuple[str, int], TrackObservation] = {}
  episode_numbers: dict[tuple[str, int], int] = defaultdict(int)
  for frame_index, frame in enumerate(frames):
    for point in frame.points:
      if not _usable_point(point):
        continue
      source = radar_point_source(point)
      track_id = radar_point_track_id(point)
      key = (source, track_id)
      previous = last.get(key)
      if previous is not None and not radar_track_continuous(
        previous.point, point, frame.mono_time_s - previous.time_s,
      ):
        episode_numbers[key] += 1
      observation = TrackObservation(
        frame_index,
        frame.mono_time_s,
        point,
        episode_numbers[key],
      )
      by_frame[(frame_index, source, track_id)] = observation
      episodes[(source, track_id, observation.episode_id)].append(observation)
      last[key] = observation
  return by_frame, {key: tuple(values) for key, values in episodes.items()}


def _future_observation(
  observations: Sequence[TrackObservation],
  target_time_s: float,
) -> TrackObservation | None:
  times = [value.time_s for value in observations]
  index = bisect.bisect_left(times, target_time_s)
  candidates = observations[max(0, index - 1):min(len(observations), index + 2)]
  closest = min(candidates, key=lambda value: abs(value.time_s - target_time_s), default=None)
  if closest is None or abs(closest.time_s - target_time_s) > FUTURE_TIME_TOLERANCE_S:
    return None
  return closest


def _actual_path_state(frame: RadarFrame, point: RadarPoint) -> tuple[float, float]:
  _, d_path, lane_half_width, _, _ = path_relative_state(
    point.d_rel,
    point.y_rel,
    frame.path,
    frame.lane_lines,
    frame.lane_probs,
  )
  return d_path, lane_half_width


def _actual_path_occupancy(frame: RadarFrame, point: RadarPoint) -> float:
  d_path, lane_half_width = _actual_path_state(frame, point)
  return float(path_center_occupied(d_path, lane_half_width))


def _future_targets(
  frames: Sequence[RadarFrame],
  observation: TrackObservation,
  observations: Sequence[TrackObservation],
) -> tuple[list[float], list[float], list[bool]]:
  future_x = []
  future_y = []
  lane_half_widths = []
  valid = []
  for horizon_s in TARGET_HORIZONS_S:
    future = _future_observation(observations, observation.time_s + horizon_s)
    is_valid = future is not None
    valid.append(is_valid)
    if future is None:
      future_x.append(0.0)
      future_y.append(0.0)
      lane_half_widths.append(0.0)
      continue
    d_path, lane_half_width = _actual_path_state(
      frames[future.frame_index], future.point,
    )
    future_x.append(float(future.point.d_rel))
    future_y.append(d_path)
    lane_half_widths.append(lane_half_width)
  return future_x + future_y, lane_half_widths, valid


def _downsample_rows(
  rows: list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]],
) -> list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]]:
  positive = []
  negative = []
  for row in rows:
    horizon_count = len(TARGET_HORIZONS_S)
    future_x = row[1][:horizon_count]
    future_y = row[1][horizon_count:]
    target = any(
      x_value > 0.5 and abs(y_value) <= half_width
      for x_value, y_value, half_width, is_valid in zip(
        future_x, future_y, row[2], row[3], strict=True,
      )
      if is_valid
    )
    (positive if target else negative).append(row)
  limit = (
    max(len(positive) * MAX_NEGATIVE_TO_POSITIVE, 200)
    if positive
    else MAX_NEGATIVE_ROWS_WITHOUT_POSITIVE
  )
  if len(negative) > limit:
    indices = np.linspace(0, len(negative) - 1, limit, dtype=np.int64)
    negative = [negative[int(index)] for index in indices]
  return positive + negative


def _frame_rows(
  frames: Sequence[RadarFrame],
  log_group: str,
) -> dict[str, list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]]]:
  observations_by_frame, episodes = _track_observations(frames)
  analyzer = RadarTrajectoryAnalyzer()
  last_sample_time: dict[tuple[str, int, int], float] = {}
  output: dict[
    str,
    list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]],
  ] = defaultdict(list)
  for frame_index, frame in enumerate(frames):
    trajectories = analyzer.update(
      frame.mono_time_s,
      frame.points,
      frame.path,
      frame.lane_lines,
      frame.lane_probs,
      frame.path_y_stds,
      frame.lane_stds,
      frame.steering_angle_deg,
      frame.steering_rate_deg_s,
      frame.yaw_rate_rad_s,
      frame.yaw_rate_estimated,
    )
    for point in frame.points:
      if not _usable_point(point):
        continue
      source = radar_point_source(point)
      track_id = radar_point_track_id(point)
      observation = observations_by_frame.get((frame_index, source, track_id))
      trajectory = trajectories.get((source, track_id))
      if observation is None or trajectory is None or trajectory.history_count < MIN_HISTORY_COUNT:
        continue
      episode_key = (source, track_id, observation.episode_id)
      if frame.mono_time_s - last_sample_time.get(episode_key, -math.inf) < MIN_SAMPLE_PERIOD_S:
        continue
      labels, lane_half_widths, valid = _future_targets(
        frames, observation, episodes[episode_key],
      )
      if not any(valid):
        continue
      current_occupancy = _actual_path_occupancy(frame, point) > 0.5
      row = trajectory_model_feature_row(trajectory, point, frame.v_ego)
      sample_id = f"{log_group}:{source}:{track_id}:{observation.episode_id}:{frame.time_s:.2f}"
      mode = _source_mode(source)
      assert mode is not None
      output[mode].append((
        [row[name] for name in TRAJECTORY_MODEL_FEATURE_NAMES],
        labels,
        lane_half_widths,
        valid,
        current_occupancy,
        sample_id,
        log_group,
      ))
      last_sample_time[episode_key] = frame.mono_time_s
  return {source: _downsample_rows(rows) for source, rows in output.items()}


def _dataset_from_rows(
  rows: dict[
    str,
    list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]],
  ],
) -> dict[str, Dataset]:
  return {
    source: Dataset(
      features=np.asarray([row[0] for row in source_rows], dtype=np.float32),
      labels=np.asarray([row[1] for row in source_rows], dtype=np.float32),
      lane_half_widths=np.asarray([row[2] for row in source_rows], dtype=np.float32),
      valid=np.asarray([row[3] for row in source_rows], dtype=np.bool_),
      current_occupancy=np.asarray([row[4] for row in source_rows], dtype=np.bool_),
      sample_ids=np.asarray([row[5] for row in source_rows]),
      log_groups=np.asarray([row[6] for row in source_rows]),
    )
    for source, source_rows in rows.items()
    if source_rows
  }


def discover_logs(routes_root: Path) -> list[Path]:
  return sorted(
    Path(directory) / name
    for directory, _, files in os.walk(routes_root)
    for name in files
    if RLOG_PATTERN.fullmatch(name)
  )


def _log_cache_path(cache_dir: Path, relative_log: str) -> Path:
  digest = hashlib.sha256(relative_log.encode("utf-8")).hexdigest()[:24]
  return cache_dir / f"{digest}.npz"


def _segment_log_group(relative_log: str) -> str:
  """Group every rlog variant from one route segment into the same CV fold."""
  return Path(relative_log).parent.as_posix()


def _log_fingerprint(path: Path) -> dict[str, int]:
  stat = path.stat()
  return {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}


def _save_log_rows(
  path: Path,
  relative_log: str,
  fingerprint: dict[str, int],
  rows: dict[
    str,
    list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]],
  ],
) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  values: dict[str, np.ndarray] = {}
  for source, source_rows in rows.items():
    if not source_rows:
      continue
    values.update({
      f"{source}_features": np.asarray([row[0] for row in source_rows], dtype=np.float32),
      f"{source}_labels": np.asarray([row[1] for row in source_rows], dtype=np.float32),
      f"{source}_lane_half_widths": np.asarray([row[2] for row in source_rows], dtype=np.float32),
      f"{source}_valid": np.asarray([row[3] for row in source_rows], dtype=np.bool_),
      f"{source}_current_occupancy": np.asarray([row[4] for row in source_rows], dtype=np.bool_),
      f"{source}_sample_ids": np.asarray([row[5] for row in source_rows]),
      f"{source}_log_groups": np.asarray([row[6] for row in source_rows]),
    })
  np.savez_compressed(
    path,
    cache_version=np.asarray([LOG_CACHE_VERSION], dtype=np.int32),
    relative_log=np.asarray([relative_log]),
    fingerprint_json=np.asarray([json.dumps(fingerprint)]),
    **values,
  )


def _load_log_rows(
  path: Path,
  relative_log: str,
  fingerprint: dict[str, int],
) -> dict[
  str,
  list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]],
] | None:
  try:
    payload = np.load(path, allow_pickle=False)
    if int(payload["cache_version"].reshape(-1)[0]) != LOG_CACHE_VERSION:
      return None
    if str(payload["relative_log"].reshape(-1)[0]) != relative_log:
      return None
    if json.loads(str(payload["fingerprint_json"].reshape(-1)[0])) != fingerprint:
      return None
    output = {}
    for source in ("front", "corner"):
      if f"{source}_features" not in payload:
        continue
      output[source] = [
        (
          feature.tolist(),
          labels.tolist(),
          lane_half_widths.tolist(),
          valid.tolist(),
          bool(current),
          str(sample_id),
          str(log_group),
        )
        for feature, labels, lane_half_widths, valid, current, sample_id, log_group in zip(
          payload[f"{source}_features"],
          payload[f"{source}_labels"],
          payload[f"{source}_lane_half_widths"],
          payload[f"{source}_valid"],
          payload[f"{source}_current_occupancy"],
          payload[f"{source}_sample_ids"],
          payload[f"{source}_log_groups"],
          strict=True,
        )
      ]
    return output
  except (KeyError, OSError, ValueError):
    return None


def _load_evaluation_labels(path: Path) -> list[EvaluationLabel]:
  payload = json.loads(path.read_text(encoding="utf-8"))
  labels = []
  for item in payload.get("labels", ()):
    if not item.get("human_verified", False) or item.get("expected") not in ("detect", "clear"):
      continue
    window = item.get("window", (item["time_s"], item["time_s"]))
    track_ids = tuple(int(value) for value in item.get("track_ids", (item["track_id"],)))
    labels.append(EvaluationLabel(
      label_id=str(item["id"]),
      vehicle_folder=str(item["vehicle_folder"]),
      log=str(item["log"]),
      source=str(item["source"]),
      start_s=float(window[0]),
      end_s=float(window[1]),
      track_ids=track_ids,
      expected=str(item["expected"]),
    ))
  return labels


def evaluation_logs(labels_path: Path, routes_root: Path) -> list[Path]:
  return sorted({
    routes_root / label.vehicle_folder / Path(*label.log.replace("\\", "/").split("/"))
    for label in _load_evaluation_labels(labels_path)
  })


def evaluation_segments(
  labels_path: Path,
  routes_root: Path,
  validation_cases_path: Path | None = None,
) -> set[Path]:
  """Reserve every rlog variant from a manually labeled route segment."""
  logs = set(evaluation_logs(labels_path, routes_root))
  if validation_cases_path is not None and validation_cases_path.is_file():
    payload = json.loads(validation_cases_path.read_text(encoding="utf-8"))
    logs.update(
      routes_root / str(item["vehicle_folder"]) / Path(
        *str(item["log"]).replace("\\", "/").split("/"),
      )
      for item in payload.get("cases", ())
      if item.get("human_verified", False)
      and item.get("expected") in ("detect", "clear")
    )
  return {path.parent.resolve() for path in logs}


def build_dataset(
  log_paths: Sequence[Path],
  routes_root: Path,
  cache_path: Path | None,
  log_cache_dir: Path | None = None,
) -> tuple[dict[str, Dataset], dict[str, Any]]:
  rows: dict[
    str,
    list[tuple[list[float], list[float], list[float], list[bool], bool, str, str]],
  ] = defaultdict(list)
  skipped: list[str] = []
  cached_logs = 0
  for index, log_path in enumerate(log_paths, 1):
    try:
      relative = log_path.relative_to(routes_root).as_posix()
    except ValueError:
      relative = str(log_path)
    print(f"[{index:04d}/{len(log_paths):04d}] {relative}", flush=True)
    frame_rows = None
    fingerprint = _log_fingerprint(log_path)
    log_cache_path = (
      _log_cache_path(log_cache_dir, relative)
      if log_cache_dir is not None
      else None
    )
    if log_cache_path is not None and log_cache_path.is_file():
      frame_rows = _load_log_rows(log_cache_path, relative, fingerprint)
      if frame_rows is not None:
        cached_logs += 1
        print("  cached", flush=True)
    try:
      if frame_rows is None:
        frame_rows = _frame_rows(load_frames(log_path), relative)
        if log_cache_path is not None:
          _save_log_rows(log_cache_path, relative, fingerprint, frame_rows)
    except Exception as exc:
      skipped.append(f"{relative}: {type(exc).__name__}: {exc}")
      print(f"  skipped: {skipped[-1]}", flush=True)
      continue
    segment_group = _segment_log_group(relative)
    for source, source_rows in frame_rows.items():
      rows[source].extend(
        (*row[:-1], segment_group)
        for row in source_rows
      )
  datasets = _dataset_from_rows(rows)
  metadata = {
    "provenance": "self-supervised same-vehicle measured future x/y position",
    "target_semantics": "future dRel and lane/path-center-relative dPath at each horizon",
    "feature_semantics": "fresh livePose or steering yaw-compensated radar lateral rates",
    "target_horizons_s": TARGET_HORIZONS_S,
    "manual_training_rows": 0,
    "requested_logs": len(log_paths),
    "loaded_logs": len(log_paths) - len(skipped),
    "incremental_cached_logs": cached_logs,
    "skipped_logs": skipped,
  }
  if cache_path is not None:
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
      cache_path,
      cache_version=np.asarray([DATASET_CACHE_VERSION], dtype=np.int32),
      feature_names=np.asarray(TRAJECTORY_MODEL_FEATURE_NAMES),
      target_horizons_s=np.asarray(TARGET_HORIZONS_S, dtype=np.float32),
      provenance=np.asarray([metadata["provenance"]]),
      manual_training_rows=np.asarray([0], dtype=np.int32),
      metadata_json=np.asarray([json.dumps(metadata)]),
      **{
        f"{source}_{name}": value
        for source, dataset in datasets.items()
        for name, value in (
          ("features", dataset.features),
          ("labels", dataset.labels),
          ("lane_half_widths", dataset.lane_half_widths),
          ("valid", dataset.valid),
          ("current_occupancy", dataset.current_occupancy),
          ("sample_ids", dataset.sample_ids),
          ("log_groups", dataset.log_groups),
        )
      },
    )
  return datasets, metadata


def load_dataset(path: Path) -> tuple[dict[str, Dataset], dict[str, Any]]:
  with np.load(path, allow_pickle=False) as payload:
    cache_version = int(payload["cache_version"].reshape(-1)[0])
    names = tuple(str(value) for value in payload["feature_names"].tolist())
    horizons = tuple(float(value) for value in payload["target_horizons_s"].tolist())
    manual_rows = int(payload["manual_training_rows"].reshape(-1)[0])
    if cache_version != DATASET_CACHE_VERSION:
      raise ValueError("cached trajectory feature semantics are stale")
    if names != TRAJECTORY_MODEL_FEATURE_NAMES:
      raise ValueError("cached trajectory feature schema mismatch")
    if horizons != TARGET_HORIZONS_S:
      raise ValueError("cached trajectory target horizon schema mismatch")
    if manual_rows != 0:
      raise ValueError("manual labels are forbidden in the trajectory training cache")
    datasets = {}
    for source in ("front", "corner"):
      if f"{source}_features" not in payload:
        continue
      # Per-row sample strings are useful in the persistent cache for auditing,
      # but neither fitting nor grouped validation consumes them. Avoid loading
      # several gigabytes of duplicate Unicode data into every training fold.
      _, log_group_codes = np.unique(
        payload[f"{source}_log_groups"],
        return_inverse=True,
      )
      datasets[source] = Dataset(
        payload[f"{source}_features"],
        payload[f"{source}_labels"],
        payload[f"{source}_lane_half_widths"],
        payload[f"{source}_valid"].astype(np.bool_),
        payload[f"{source}_current_occupancy"].astype(np.bool_),
        np.empty(0, dtype=np.str_),
        log_group_codes.astype(np.int32),
      )
    metadata = json.loads(str(payload["metadata_json"].reshape(-1)[0]))
  return datasets, metadata


def _row_targets(dataset: Dataset) -> tuple[np.ndarray, np.ndarray]:
  usable = np.any(dataset.valid, axis=1)
  targets = np.any(_occupancy_targets(dataset) & dataset.valid, axis=1)
  return targets, usable


def _model_targets(dataset: Dataset) -> tuple[np.ndarray, np.ndarray]:
  """Return future x/y positions and the matching per-axis validity mask."""
  return dataset.labels, np.concatenate((dataset.valid, dataset.valid), axis=1)


def _residual_model_targets(dataset: Dataset) -> tuple[np.ndarray, np.ndarray]:
  """Return residuals from the same kinematic baseline used at inference."""
  targets, valid = _model_targets(dataset)
  return targets - kinematic_position_baseline(dataset.features), valid


def _training_head_weights(dataset: Dataset) -> np.ndarray:
  """Emphasize real path transitions without using manual annotations."""
  horizon_count = len(TARGET_HORIZONS_S)
  weights = np.concatenate((dataset.valid, dataset.valid), axis=1).astype(np.float32)
  occupied = _occupancy_targets(dataset) & dataset.valid
  outside = _outside_targets(dataset) & dataset.valid
  entry_rows = ~dataset.current_occupancy & np.any(occupied, axis=1)
  exit_rows = dataset.current_occupancy & np.any(outside, axis=1)
  weights[entry_rows, :horizon_count] *= ENTRY_LONGITUDINAL_WEIGHT
  weights[entry_rows, horizon_count:] *= ENTRY_LATERAL_WEIGHT
  weights[exit_rows, :horizon_count] *= EXIT_LONGITUDINAL_WEIGHT
  weights[exit_rows, horizon_count:] *= EXIT_LATERAL_WEIGHT
  return weights


def _occupancy_targets(dataset: Dataset) -> np.ndarray:
  horizon_count = len(TARGET_HORIZONS_S)
  future_x = dataset.labels[:, :horizon_count]
  future_y = dataset.labels[:, horizon_count:]
  return (
    (future_x > 0.5)
    & (np.abs(future_y) <= dataset.lane_half_widths)
  )


def _outside_targets(dataset: Dataset) -> np.ndarray:
  horizon_count = len(TARGET_HORIZONS_S)
  future_x = dataset.labels[:, :horizon_count]
  future_y = dataset.labels[:, horizon_count:]
  return (
    (future_x > 0.5)
    & (np.abs(future_y) > dataset.lane_half_widths)
  )


def _group_folds(dataset: Dataset, seed: int, fold_count: int) -> list[np.ndarray]:
  groups, group_codes = np.unique(dataset.log_groups, return_inverse=True)
  targets, usable = _row_targets(dataset)
  effective_folds = min(fold_count, len(groups))
  if effective_folds < 2:
    raise RuntimeError("not enough log groups for grouped validation")
  rng = np.random.default_rng(seed)
  shuffled_codes = rng.permutation(len(groups))
  usable_counts = np.bincount(
    group_codes[usable],
    minlength=len(groups),
  )
  positive_counts = np.bincount(
    group_codes[usable],
    weights=targets[usable],
    minlength=len(groups),
  )
  rates = np.divide(
    positive_counts,
    usable_counts,
    out=np.zeros(len(groups), dtype=np.float64),
    where=usable_counts > 0,
  )
  ranked_codes = sorted(
    shuffled_codes,
    key=lambda code: float(rates[code]),
    reverse=True,
  )
  group_folds = np.zeros(len(groups), dtype=np.int8)
  for index, group_code in enumerate(ranked_codes):
    cycle, offset = divmod(index, effective_folds)
    bucket_index = offset if cycle % 2 == 0 else effective_folds - 1 - offset
    group_folds[group_code] = bucket_index
  return [group_folds[group_codes] == fold for fold in range(effective_folds)]


def _require_split(source: str, name: str, dataset: Dataset) -> None:
  targets, usable = _row_targets(dataset)
  if not np.any(usable):
    raise RuntimeError(f"{source} {name} split has no valid targets")
  if len(np.unique(targets[usable])) < 2:
    raise RuntimeError(f"{source} {name} split must contain occupied and clear samples")


class GaussianPositionMLP:
  def __init__(self, inputs: int, targets: int, hidden1: int, hidden2: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    outputs = targets * 2
    output_weights = (
      rng.standard_normal((hidden2, outputs)) * math.sqrt(1.0 / hidden2)
    ).astype(np.float32)
    output_weights[:, targets:] = 0.0
    self.p = {
      "w1": (rng.standard_normal((inputs, hidden1)) * math.sqrt(2.0 / inputs)).astype(np.float32),
      "b1": np.zeros(hidden1, dtype=np.float32),
      "w2": (rng.standard_normal((hidden1, hidden2)) * math.sqrt(2.0 / hidden1)).astype(np.float32),
      "b2": np.zeros(hidden2, dtype=np.float32),
      "w3": output_weights,
      "b3": np.zeros(outputs, dtype=np.float32),
    }
    self.targets = targets
    self.m = {name: np.zeros_like(value) for name, value in self.p.items()}
    self.v = {name: np.zeros_like(value) for name, value in self.p.items()}
    self.step = 0

  def outputs(self, x: np.ndarray) -> np.ndarray:
    h1 = np.maximum(x @ self.p["w1"] + self.p["b1"], 0.0)
    h2 = np.maximum(h1 @ self.p["w2"] + self.p["b2"], 0.0)
    return h2 @ self.p["w3"] + self.p["b3"]

  def update(
    self,
    x: np.ndarray,
    y: np.ndarray,
    head_weights: np.ndarray,
    learning_rate: float,
    l2_weight: float,
  ) -> float:
    z1 = x @ self.p["w1"] + self.p["b1"]
    h1 = np.maximum(z1, 0.0)
    z2 = h1 @ self.p["w2"] + self.p["b2"]
    h2 = np.maximum(z2, 0.0)
    output = h2 @ self.p["w3"] + self.p["b3"]
    means = output[:, :self.targets]
    raw_log_stds = output[:, self.targets:]
    log_stds = np.clip(raw_log_stds, -4.0, 3.0)
    residual = means - y
    inverse_variance = np.exp(-2.0 * log_stds)
    weight = head_weights.astype(np.float32)
    denominator = max(float(weight.sum()), 1.0)
    loss = float(np.sum(
      weight * (0.5 * residual * residual * inverse_variance + log_stds),
    ) / denominator)
    loss += 0.5 * l2_weight * sum(
      float(np.sum(self.p[name] ** 2)) for name in ("w1", "w2", "w3")
    )
    d_means = weight * residual * inverse_variance / denominator
    within_clip = (raw_log_stds >= -4.0) & (raw_log_stds <= 3.0)
    d_log_stds = (
      weight * (1.0 - residual * residual * inverse_variance)
      * within_clip / denominator
    )
    d_output = np.concatenate((d_means, d_log_stds), axis=1)
    gradients = {
      "w3": h2.T @ d_output,
      "b3": d_output.sum(axis=0),
    }
    d_h2 = d_output @ self.p["w3"].T
    d_z2 = d_h2 * (z2 > 0.0)
    gradients["w2"] = h1.T @ d_z2
    gradients["b2"] = d_z2.sum(axis=0)
    d_h1 = d_z2 @ self.p["w2"].T
    d_z1 = d_h1 * (z1 > 0.0)
    gradients["w1"] = x.T @ d_z1
    gradients["b1"] = d_z1.sum(axis=0)
    for name in ("w1", "w2", "w3"):
      gradients[name] += l2_weight * self.p[name]

    self.step += 1
    for name, parameter in self.p.items():
      gradient = gradients[name]
      gradient_norm = float(np.linalg.norm(gradient))
      if gradient_norm > 10.0:
        gradient *= 10.0 / gradient_norm
      self.m[name] = 0.9 * self.m[name] + 0.1 * gradient
      self.v[name] = 0.999 * self.v[name] + 0.001 * gradient * gradient
      corrected_m = self.m[name] / (1.0 - 0.9 ** self.step)
      corrected_v = self.v[name] / (1.0 - 0.999 ** self.step)
      parameter -= learning_rate * corrected_m / (np.sqrt(corrected_v) + 1e-8)
    return loss


def _sample_metrics(
  scores: np.ndarray,
  targets: np.ndarray,
  usable: np.ndarray,
  threshold: float,
) -> dict[str, float | int]:
  predicted = scores >= threshold
  predicted = predicted[usable]
  targets = targets[usable]
  tp = int(np.sum(predicted & targets))
  fp = int(np.sum(predicted & ~targets))
  fn = int(np.sum(~predicted & targets))
  tn = int(np.sum(~predicted & ~targets))
  precision = tp / max(tp + fp, 1)
  recall = tp / max(tp + fn, 1)
  return {
    "precision": precision,
    "recall": recall,
    "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
    "accuracy": (tp + tn) / max(tp + fp + fn + tn, 1),
    "tp": tp,
    "fp": fp,
    "fn": fn,
    "tn": tn,
  }


def _normal_cdf_array(values: np.ndarray) -> np.ndarray:
  # Abramowitz-Stegun approximates erf(x); normal CDF needs erf(z / sqrt(2)).
  scaled = np.abs(values) / math.sqrt(2.0)
  t = 1.0 / (1.0 + 0.3275911 * scaled)
  polynomial = (
    (
      (
        (
          1.061405429 * t
          - 1.453152027
        ) * t
        + 1.421413741
      ) * t
      - 0.284496736
    ) * t
    + 0.254829592
  ) * t
  erf = 1.0 - polynomial * np.exp(-scaled * scaled)
  erf = np.copysign(erf, values)
  return 0.5 * (1.0 + erf)


def _position_probabilities(
  means: np.ndarray,
  stds: np.ndarray,
  lane_half_widths: np.ndarray,
) -> np.ndarray:
  horizon_count = len(TARGET_HORIZONS_S)
  future_x = means[:, :horizon_count]
  future_y = means[:, horizon_count:]
  x_stds = np.maximum(stds[:, :horizon_count], 0.05)
  y_stds = np.maximum(stds[:, horizon_count:], 0.05)
  ahead = _normal_cdf_array((future_x - 0.5) / x_stds)
  upper = (lane_half_widths - future_y) / y_stds
  lower = (-lane_half_widths - future_y) / y_stds
  lane_in = np.clip(_normal_cdf_array(upper) - _normal_cdf_array(lower), 0.0, 1.0)
  path_in = ahead * lane_in
  path_out = ahead * (1.0 - lane_in)
  return np.concatenate((path_in, path_out), axis=1).astype(np.float32)


def _sigma_calibration(
  means: np.ndarray,
  stds: np.ndarray,
  dataset: Dataset,
) -> np.ndarray:
  _, valid = _model_targets(dataset)
  weights = _training_head_weights(dataset)
  values = []
  for index in range(means.shape[1]):
    ratios = (
      np.abs(dataset.labels[valid[:, index], index] - means[valid[:, index], index])
      / np.maximum(stds[valid[:, index], index], 0.05)
    )
    values.append(
      min(4.0, max(0.25, _weighted_quantile(
        ratios, weights[valid[:, index], index], 0.6827,
      )))
      if len(ratios)
      else 1.0
    )
  return np.asarray(values, dtype=np.float32)


def _weighted_quantile(
  values: np.ndarray,
  weights: np.ndarray,
  quantile: float,
) -> float:
  """Return a deterministic weighted quantile for task-weighted calibration."""
  if len(values) == 0:
    raise ValueError("weighted quantile requires at least one value")
  order = np.argsort(values, kind="stable")
  sorted_values = values[order]
  sorted_weights = np.maximum(weights[order], 0.0)
  total = float(np.sum(sorted_weights))
  if total <= 0.0:
    return float(np.quantile(sorted_values, quantile))
  cumulative = np.cumsum(sorted_weights)
  index = int(np.searchsorted(cumulative, min(max(quantile, 0.0), 1.0) * total, side="left"))
  return float(sorted_values[min(index, len(sorted_values) - 1)])


def _event_scores_targets(
  probabilities: np.ndarray,
  dataset: Dataset,
  event: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  valid = np.any(dataset.valid, axis=1)
  horizon_count = len(TARGET_HORIZONS_S)
  occupied = _occupancy_targets(dataset)
  outside = _outside_targets(dataset)
  if event == "entry":
    scores = np.max(np.where(dataset.valid, probabilities[:, :horizon_count], 0.0), axis=1)
    targets = np.any(occupied & dataset.valid, axis=1)
    usable = valid & ~dataset.current_occupancy
  elif event == "exit":
    scores = np.max(np.where(dataset.valid, probabilities[:, horizon_count:], 0.0), axis=1)
    targets = np.any(outside & dataset.valid, axis=1)
    usable = valid & dataset.current_occupancy
  else:
    raise ValueError(f"unsupported event {event}")
  return scores, targets, usable


def _choose_threshold(
  probabilities: np.ndarray,
  dataset: Dataset,
  event: str,
  target_precision: float | None = None,
) -> tuple[float, dict[str, float | int]]:
  scores, targets, usable = _event_scores_targets(probabilities, dataset, event)
  best = (0.5, {"f1": -1.0, "precision": -1.0})
  for threshold in np.linspace(0.30, 0.995, 140):
    metrics = _sample_metrics(scores, targets, usable, float(threshold))
    if target_precision is None:
      score = (float(metrics["f1"]), float(metrics["precision"]))
      best_score = (float(best[1]["f1"]), float(best[1].get("precision", -1.0)))
    else:
      qualifies = float(metrics["precision"]) >= target_precision and int(metrics["tp"]) >= 1
      best_qualifies = (
        float(best[1].get("precision", -1.0)) >= target_precision
        and int(best[1].get("tp", 0)) >= 1
      )
      score = (
        float(qualifies),
        float(metrics["recall"]) if qualifies else float(metrics["precision"]),
        float(metrics["precision"]),
      )
      best_score = (
        float(best_qualifies),
        float(best[1].get("recall", -1.0)) if best_qualifies else float(best[1].get("precision", -1.0)),
        float(best[1].get("precision", -1.0)),
      )
    if score > best_score:
      best = float(threshold), metrics
  return best


def _horizon_metrics(probabilities: np.ndarray, dataset: Dataset) -> dict[str, dict[str, dict[str, float | int]]]:
  result: dict[str, dict[str, dict[str, float | int]]] = {"in": {}, "out": {}}
  horizon_count = len(TARGET_HORIZONS_S)
  occupied_targets = _occupancy_targets(dataset)
  outside_targets = _outside_targets(dataset)
  for state, offset in (("in", 0), ("out", horizon_count)):
    for index, horizon_s in enumerate(TARGET_HORIZONS_S):
      valid = dataset.valid[:, index]
      predicted = probabilities[valid, offset + index] >= 0.5
      targets = (
        occupied_targets[valid, index]
        if state == "in"
        else outside_targets[valid, index]
      )
      tp = int(np.sum(predicted & targets))
      fp = int(np.sum(predicted & ~targets))
      fn = int(np.sum(~predicted & targets))
      tn = int(np.sum(~predicted & ~targets))
      precision = tp / max(tp + fp, 1)
      recall = tp / max(tp + fn, 1)
      result[state][f"{horizon_s:.1f}"] = {
        "precision": precision,
        "recall": recall,
        "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "tn": tn,
      }
  return result


def _position_metrics(
  means: np.ndarray,
  stds: np.ndarray,
  dataset: Dataset,
) -> dict[str, dict[str, float]]:
  _, valid = _model_targets(dataset)
  result = {}
  for index, name in enumerate(POSITION_TARGET_NAMES):
    residual = means[valid[:, index], index] - dataset.labels[valid[:, index], index]
    sigma = stds[valid[:, index], index]
    result[name] = {
      "mae": float(np.mean(np.abs(residual))) if len(residual) else 0.0,
      "rmse": float(np.sqrt(np.mean(residual * residual))) if len(residual) else 0.0,
      "mean_std": float(np.mean(sigma)) if len(sigma) else 0.0,
      "within_1std": float(np.mean(np.abs(residual) <= sigma)) if len(residual) else 0.0,
    }
  return result


def _subset(dataset: Dataset, mask: np.ndarray) -> Dataset:
  return Dataset(
    dataset.features[mask],
    dataset.labels[mask],
    dataset.lane_half_widths[mask],
    dataset.valid[mask],
    dataset.current_occupancy[mask],
    np.empty(0, dtype=dataset.sample_ids.dtype),
    np.empty(0, dtype=dataset.log_groups.dtype),
  )


def _fit_model(
  source: str,
  train: Dataset,
  validation: Dataset | None,
  epochs: int,
  batch_size: int,
  learning_rate: float,
  seed: int,
  l2_weight: float,
  verbose: bool,
) -> tuple[
  GaussianPositionMLP,
  np.ndarray,
  np.ndarray,
  np.ndarray,
  np.ndarray,
  int,
]:
  mean = train.features.mean(axis=0)
  std = train.features.std(axis=0)
  std[std < 1e-4] = 1.0
  normalized = (train.features - mean) / std
  validation_x = None if validation is None else (validation.features - mean) / std
  raw_targets, train_valid = _residual_model_targets(train)
  train_weights = _training_head_weights(train)
  validation_weights = None if validation is None else _training_head_weights(validation)
  target_mean = np.zeros(raw_targets.shape[1], dtype=np.float32)
  target_std = np.ones(raw_targets.shape[1], dtype=np.float32)
  for index in range(raw_targets.shape[1]):
    usable = train_valid[:, index]
    if np.any(usable):
      target_mean[index] = float(np.mean(raw_targets[usable, index]))
      target_std[index] = max(0.10, float(np.std(raw_targets[usable, index])))
  train_targets = (raw_targets - target_mean) / target_std
  model = GaussianPositionMLP(
    normalized.shape[1], len(POSITION_TARGET_NAMES), 48, 24, seed,
  )
  rng = np.random.default_rng(seed)
  best_validation_mae = math.inf
  best_parameters: dict[str, np.ndarray] | None = None
  best_epoch = epochs
  patience = 0
  for epoch in range(epochs):
    order = rng.permutation(len(normalized))
    losses = []
    for start in range(0, len(order), batch_size):
      indices = order[start:start + batch_size]
      losses.append(model.update(
        normalized[indices],
        train_targets[indices],
        train_weights[indices],
        learning_rate,
        l2_weight,
      ))

    epoch_mae = 0.0
    if validation is not None and validation_x is not None:
      validation_output = model.outputs(validation_x)
      validation_means = (
        validation_output[:, :len(POSITION_TARGET_NAMES)] * target_std
        + target_mean
        + kinematic_position_baseline(validation.features)
      )
      assert validation_weights is not None
      absolute_error = np.abs(validation_means - validation.labels)
      epoch_mae = float(np.sum(absolute_error * validation_weights) / max(
        float(np.sum(validation_weights)), 1.0,
      ))
      if epoch_mae < best_validation_mae - 1e-6:
        best_validation_mae = epoch_mae
        best_parameters = {name: value.copy() for name, value in model.p.items()}
        best_epoch = epoch + 1
        patience = 0
      else:
        patience += 1
    show_progress = verbose or validation is not None
    if show_progress and (epoch == 0 or (epoch + 1) % 5 == 0):
      suffix = "" if validation is None else f" val-mae {epoch_mae:.3f}m"
      print(
        f"{source} epoch {epoch + 1:03d}/{epochs} loss {np.mean(losses):.4f}{suffix}",
        flush=True,
      )
    if validation is not None and epoch >= 20 and patience >= 18:
      break
  if best_parameters is not None:
    model.p = best_parameters
  return model, mean, std, target_mean, target_std, best_epoch


def train_source(
  source: str,
  dataset: Dataset,
  output: Path,
  epochs: int,
  batch_size: int,
  learning_rate: float,
  seed: int,
  l2_weight: float = 1e-4,
  fold_count: int = 5,
) -> dict[str, Any]:
  _require_split(source, "complete", dataset)
  folds = _group_folds(dataset, seed, fold_count)
  target_count = len(POSITION_TARGET_NAMES)
  out_of_fold_means = np.zeros((len(dataset.features), target_count), dtype=np.float32)
  out_of_fold_stds = np.zeros((len(dataset.features), target_count), dtype=np.float32)
  best_epochs: list[int] = []
  for fold_index, validation_mask in enumerate(folds):
    train = _subset(dataset, ~validation_mask)
    validation = _subset(dataset, validation_mask)
    _require_split(source, f"fold {fold_index + 1} train", train)
    _require_split(source, f"fold {fold_index + 1} validation", validation)
    print(f"{source} fold {fold_index + 1}/{len(folds)}", flush=True)
    (
      fold_model,
      fold_mean,
      fold_std,
      fold_target_mean,
      fold_target_std,
      best_epoch,
    ) = _fit_model(
      source,
      train,
      validation,
      epochs,
      batch_size,
      learning_rate,
      seed + fold_index,
      l2_weight,
      verbose=False,
    )
    validation_x = (validation.features - fold_mean) / fold_std
    fold_output = fold_model.outputs(validation_x)
    out_of_fold_means[validation_mask] = (
      fold_output[:, :target_count] * fold_target_std + fold_target_mean
      + kinematic_position_baseline(validation.features)
    )
    out_of_fold_stds[validation_mask] = (
      np.exp(np.clip(fold_output[:, target_count:], -4.0, 3.0))
      * fold_target_std
    )
    best_epochs.append(best_epoch)
  sigma_calibration = _sigma_calibration(
    out_of_fold_means, out_of_fold_stds, dataset,
  )
  calibrated_stds = np.maximum(
    out_of_fold_stds * sigma_calibration,
    0.05,
  )
  out_of_fold_probabilities = _position_probabilities(
    out_of_fold_means,
    calibrated_stds,
    dataset.lane_half_widths,
  )
  balanced_entry_threshold, balanced_entry_metrics = _choose_threshold(
    out_of_fold_probabilities, dataset, "entry",
  )
  entry_threshold, entry_metrics = _choose_threshold(
    out_of_fold_probabilities, dataset, "entry", target_precision=ENTRY_TARGET_PRECISION,
  )
  balanced_exit_threshold, balanced_exit_metrics = _choose_threshold(
    out_of_fold_probabilities, dataset, "exit",
  )
  exit_threshold, exit_metrics = _choose_threshold(
    out_of_fold_probabilities, dataset, "exit", target_precision=0.95,
  )
  final_epochs = max(10, int(round(float(np.median(best_epochs)))))
  training_summary = (
    f"{source} CV entry {entry_threshold:.3f} "
    + f"P {float(entry_metrics['precision']):.3f} R {float(entry_metrics['recall']):.3f}; "
    + f"exit {exit_threshold:.3f} "
    + f"P {float(exit_metrics['precision']):.3f} R {float(exit_metrics['recall']):.3f}; "
    + f"final {final_epochs} epochs"
  )
  print(training_summary, flush=True)
  model, mean, std, target_mean, target_std, _ = _fit_model(
    source,
    dataset,
    None,
    final_epochs,
    batch_size,
    learning_rate,
    seed + 100,
    l2_weight,
    verbose=True,
  )
  output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(
    output,
    model_version=np.asarray([MODEL_VERSION], dtype=np.int32),
    sensor_mode=np.asarray([source]),
    feature_names=np.asarray(TRAJECTORY_MODEL_FEATURE_NAMES),
    target_horizons_s=np.asarray(TARGET_HORIZONS_S, dtype=np.float32),
    output_head_names=np.asarray(OUTPUT_HEAD_NAMES),
    training_provenance=np.asarray([
      "self-supervised same-vehicle measured future x/y distribution; transition-weighted residual over past-only kinematics"
    ]),
    entry_target_precision=np.asarray([ENTRY_TARGET_PRECISION], dtype=np.float32),
    entry_longitudinal_weight=np.asarray([ENTRY_LONGITUDINAL_WEIGHT], dtype=np.float32),
    entry_lateral_weight=np.asarray([ENTRY_LATERAL_WEIGHT], dtype=np.float32),
    exit_longitudinal_weight=np.asarray([EXIT_LONGITUDINAL_WEIGHT], dtype=np.float32),
    exit_lateral_weight=np.asarray([EXIT_LATERAL_WEIGHT], dtype=np.float32),
    manual_training_rows=np.asarray([0], dtype=np.int32),
    feature_mean=mean.astype(np.float32),
    feature_std=std.astype(np.float32),
    target_mean=target_mean.astype(np.float32),
    target_std=target_std.astype(np.float32),
    sigma_calibration=sigma_calibration.astype(np.float32),
    threshold=np.asarray([entry_threshold], dtype=np.float32),
    entry_threshold=np.asarray([entry_threshold], dtype=np.float32),
    exit_threshold=np.asarray([exit_threshold], dtype=np.float32),
    **model.p,
  )
  targets, usable = _row_targets(dataset)
  return {
    "source": source,
    "rows": len(dataset.features),
    "logs": len(np.unique(dataset.log_groups)),
    "occupied_rows": int(np.sum(targets & usable)),
    "clear_rows": int(np.sum(~targets & usable)),
    "entry_threshold": entry_threshold,
    "entry_target_precision": ENTRY_TARGET_PRECISION,
    "transition_training_weights": {
      "entry_longitudinal": ENTRY_LONGITUDINAL_WEIGHT,
      "entry_lateral": ENTRY_LATERAL_WEIGHT,
      "exit_longitudinal": EXIT_LONGITUDINAL_WEIGHT,
      "exit_lateral": EXIT_LATERAL_WEIGHT,
    },
    "entry_cross_validation": entry_metrics,
    "balanced_entry_threshold": balanced_entry_threshold,
    "balanced_entry_cross_validation": balanced_entry_metrics,
    "exit_threshold": exit_threshold,
    "exit_cross_validation": exit_metrics,
    "balanced_exit_threshold": balanced_exit_threshold,
    "balanced_exit_cross_validation": balanced_exit_metrics,
    "horizon_cross_validation": _horizon_metrics(
      out_of_fold_probabilities, dataset,
    ),
    "position_cross_validation": _position_metrics(
      out_of_fold_means, calibrated_stds, dataset,
    ),
    "sigma_calibration": sigma_calibration.tolist(),
    "fold_best_epochs": best_epochs,
    "final_epochs": final_epochs,
  }


def evaluate_manual_labels(
  labels_path: Path,
  routes_root: Path,
  front_model_path: Path,
  corner_model_path: Path,
) -> tuple[list[EvaluationRow], dict[str, dict[str, float | int]]]:
  labels = _load_evaluation_labels(labels_path)
  by_log: dict[str, list[EvaluationLabel]] = defaultdict(list)
  for label in labels:
    by_log[label.log_group].append(label)
  rows = []
  for index, log_labels in enumerate(by_log.values(), 1):
    log_path = routes_root / log_labels[0].vehicle_folder / Path(
      *log_labels[0].log.replace("\\", "/").split("/"),
    )
    print(f"evaluation [{index:02d}/{len(by_log):02d}] {log_path}")
    frames = load_frames(log_path)
    observations_by_frame, episodes = _track_observations(frames)
    runtime = RadarTrajectoryRuntime(
      front_model_path=front_model_path,
      corner_model_path=corner_model_path,
      corner_radar_enabled=True,
    )
    scores: dict[
      str,
      list[tuple[float, float, tuple[float, ...], tuple[float, ...], bool]],
    ] = defaultdict(list)
    actual: dict[str, list[tuple[bool, bool]]] = defaultdict(list)
    for frame_index, frame in enumerate(frames):
      result = runtime.update(
        frame.mono_time_s,
        frame.v_ego,
        frame.points,
        frame.path,
        frame.lane_lines,
        frame.lane_probs,
        frame.path_y_stds,
        frame.lane_stds,
        frame.steering_angle_deg,
        frame.steering_rate_deg_s,
        frame.yaw_rate_rad_s,
        frame.yaw_rate_estimated,
      )
      if not result.available:
        raise RuntimeError(result.error)
      for label in log_labels:
        if not label.start_s <= frame.time_s <= label.end_s:
          continue
        source_predictions = result.corner_predictions if label.source == "corner" else result.front_predictions
        source_decision = result.corner_decision if label.source == "corner" else result.front_decision
        prediction = max(
          (
            value for value in source_predictions
            if value.track_id in label.track_ids
            and ((label.source == "corner") == value.source.startswith("corner"))
          ),
          key=lambda value: value.probability,
          default=None,
        )
        if prediction is None:
          selected = False
        else:
          selected = any(
            value.track_id in label.track_ids
            and value.trajectory.continuity_id == prediction.trajectory.continuity_id
            for value in source_decision.confirmed
          )
          scores[label.label_id].append((
            prediction.path_in_probability,
            prediction.path_out_probability,
            prediction.horizon_probabilities,
            prediction.horizon_out_probabilities,
            selected,
          ))

        matching_observations = [
          observation
          for point in frame.points
          if radar_point_track_id(point) in label.track_ids
          if ((label.source == "corner") == radar_point_source(point).startswith("corner"))
          if (observation := observations_by_frame.get((
            frame_index, radar_point_source(point), radar_point_track_id(point),
          ))) is not None
        ]
        for observation in matching_observations:
          future_positions, future_half_widths, valid = _future_targets(
            frames,
            observation,
            episodes[(radar_point_source(observation.point), radar_point_track_id(observation.point), observation.episode_id)],
          )
          if not any(valid):
            continue
          current_occupancy = _actual_path_occupancy(frame, observation.point) > 0.5
          horizon_count = len(TARGET_HORIZONS_S)
          future_x = future_positions[:horizon_count]
          future_y = future_positions[horizon_count:]
          future_ahead = tuple(x_value > 0.5 for x_value in future_x)
          future_occupancy = tuple(
            ahead and path_center_occupied(y_value, half_width)
            for ahead, y_value, half_width in zip(
              future_ahead, future_y, future_half_widths, strict=True,
            )
          )
          future_outside = tuple(
            ahead and not path_center_occupied(y_value, half_width)
            for ahead, y_value, half_width in zip(
              future_ahead, future_y, future_half_widths, strict=True,
            )
          )
          future_occupied = any(
            occupied for occupied, is_valid in zip(
              future_occupancy, valid, strict=True,
            )
            if is_valid
          )
          future_clear = any(
            outside for outside, is_valid in zip(
              future_outside, valid, strict=True,
            )
            if is_valid
          )
          actual[label.label_id].append((
            not current_occupancy and future_occupied,
            current_occupancy and future_clear,
          ))
    for label in log_labels:
      values = scores[label.label_id]
      max_probability = max((value[0] for value in values), default=0.0)
      max_path_exit_probability = max((value[1] for value in values), default=0.0)
      horizon_probabilities = tuple(
        max((value[2][horizon_index] for value in values), default=0.0)
        for horizon_index in range(len(TARGET_HORIZONS_S))
      )
      horizon_out_probabilities = tuple(
        max((value[3][horizon_index] for value in values), default=0.0)
        for horizon_index in range(len(TARGET_HORIZONS_S))
      )
      selected = any(value[4] for value in values)
      actual_values = actual[label.label_id]
      actual_entry_frames = sum(value[0] for value in actual_values)
      actual_exit_frames = sum(value[1] for value in actual_values)
      manual_matches_actual = (
        None
        if not actual_values
        else (
          (label.expected == "detect" and actual_entry_frames > 0)
          or (label.expected == "clear" and actual_entry_frames == 0)
        )
      )
      rows.append(EvaluationRow(
        label_id=label.label_id,
        source=label.source,
        expected=label.expected,
        predicted="detect" if selected else "clear",
        selected=selected,
        max_probability=max_probability,
        max_path_exit_probability=max_path_exit_probability,
        horizon_probabilities=horizon_probabilities,
        horizon_out_probabilities=horizon_out_probabilities,
        scored_frames=len(values),
        actual_entry_frames=actual_entry_frames,
        actual_exit_frames=actual_exit_frames,
        actual_valid_frames=len(actual_values),
        manual_matches_actual=manual_matches_actual,
      ))

  tables = {}
  for source in ("front", "corner"):
    source_rows = [row for row in rows if row.source == source]
    tp = sum(row.expected == "detect" and row.predicted == "detect" for row in source_rows)
    fp = sum(row.expected == "clear" and row.predicted == "detect" for row in source_rows)
    fn = sum(row.expected == "detect" and row.predicted == "clear" for row in source_rows)
    tn = sum(row.expected == "clear" and row.predicted == "clear" for row in source_rows)
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    tables[source] = {
      "labels": len(source_rows),
      "precision": precision,
      "recall": recall,
      "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
      "accuracy": (tp + tn) / max(tp + fp + fn + tn, 1),
      "tp": tp,
      "fp": fp,
      "fn": fn,
      "tn": tn,
      "manual_actual_scorable": sum(row.manual_matches_actual is not None for row in source_rows),
      "manual_actual_unscorable": sum(row.manual_matches_actual is None for row in source_rows),
      "manual_actual_agree": sum(row.manual_matches_actual is True for row in source_rows),
      "manual_actual_disagree": sum(row.manual_matches_actual is False for row in source_rows),
      "manual_detect_without_actual_entry": sum(
        row.actual_valid_frames > 0
        and row.expected == "detect"
        and row.actual_entry_frames == 0
        for row in source_rows
      ),
      "manual_clear_with_actual_entry": sum(
        row.actual_valid_frames > 0
        and row.expected == "clear"
        and row.actual_entry_frames > 0
        for row in source_rows
      ),
    }
  return rows, tables


def evaluation_markdown(tables: dict[str, dict[str, float | int]]) -> str:
  lines = [
    "| model | labels | precision | recall | F1 | accuracy | TP | FP | FN | TN | manual/future |",
    "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
  ]
  for source in ("front", "corner"):
    values = tables[source]
    lines.append(
      f"| {source} | {values['labels']} | {float(values['precision']):.3f} | "
      + f"{float(values['recall']):.3f} | {float(values['f1']):.3f} | "
      + f"{float(values['accuracy']):.3f} | {values['tp']} | {values['fp']} | "
      + f"{values['fn']} | {values['tn']} | "
      + f"{values['manual_actual_agree']}/{values['manual_actual_scorable']} |"
    )
  return "\n".join(lines)


def parse_args() -> argparse.Namespace:
  root = Path(__file__).resolve().parents[2]
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
    "--evaluation-labels",
    type=Path,
    default=root / "cluster" / "radar_trajectory_labels.json",
    help="human CUT-IN/CLEAR labels reserved for evaluation and never used for fitting",
  )
  parser.add_argument(
    "--validation-cases",
    type=Path,
    default=root / "cluster" / "cutin_validation_cases.json",
    help="additional human validation cases reserved from fitting",
  )
  parser.add_argument("--routes-root", type=Path, default=Path("W:/routes"))
  parser.add_argument("--max-logs", type=int)
  parser.add_argument(
    "--cache",
    type=Path,
    default=Path(".tmp_radar_review/trajectory_position_dataset_v8.npz"),
  )
  parser.add_argument(
    "--log-cache-dir",
    type=Path,
    default=Path(".tmp_radar_review/path_position_log_cache_v8"),
    help="incremental per-log cache; safe to reuse after interrupted runs",
  )
  parser.add_argument("--rebuild-cache", action="store_true")
  parser.add_argument("--output-dir", type=Path, default=root / "radar" / "models")
  parser.add_argument("--report", type=Path, default=root / "radar" / "models" / "radar_path_occupancy_report.json")
  parser.add_argument("--epochs", type=int, default=70)
  parser.add_argument("--batch-size", type=int, default=512)
  parser.add_argument("--learning-rate", type=float, default=0.001)
  parser.add_argument("--l2-weight", type=float, default=1e-4)
  parser.add_argument("--fold-count", type=int, default=5)
  parser.add_argument("--seed", type=int, default=20260725)
  parser.add_argument("--skip-evaluation", action="store_true")
  parser.add_argument("--evaluate-only", action="store_true")
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  front_path = args.output_dir / "radar_path_occupancy_front.npz"
  corner_path = args.output_dir / "radar_path_occupancy_corner.npz"
  training_reports: list[dict[str, Any]] = []
  dataset_metadata: dict[str, Any] = {}
  if not args.evaluate_only:
    if args.rebuild_cache or not args.cache.is_file():
      log_paths = discover_logs(args.routes_root)
      held_out_segments = evaluation_segments(
        args.evaluation_labels, args.routes_root, args.validation_cases,
      )
      log_paths = [
        path for path in log_paths
        if path.parent.resolve() not in held_out_segments
      ]
      if args.max_logs is not None:
        log_paths = log_paths[:args.max_logs]
      datasets, dataset_metadata = build_dataset(
        log_paths, args.routes_root, args.cache, args.log_cache_dir,
      )
    else:
      datasets, dataset_metadata = load_dataset(args.cache)
    for source, output in (("front", front_path), ("corner", corner_path)):
      if source not in datasets:
        raise RuntimeError(f"no {source} trajectory samples")
      training_reports.append(train_source(
        source,
        datasets[source],
        output,
        args.epochs,
        args.batch_size,
        args.learning_rate,
        args.seed + (source == "corner"),
        args.l2_weight,
        args.fold_count,
      ))

  evaluation_rows: list[EvaluationRow] = []
  evaluation_tables: dict[str, dict[str, float | int]] = {}
  if not args.skip_evaluation:
    evaluation_rows, evaluation_tables = evaluate_manual_labels(
      args.evaluation_labels,
      args.routes_root,
      front_path,
      corner_path,
    )
    print("\nManual-label validation only (never used for fitting):")
    print(evaluation_markdown(evaluation_tables))
  existing_report = {}
  if args.evaluate_only and args.report.is_file():
    existing_report = json.loads(args.report.read_text(encoding="utf-8"))
  report = {
    **existing_report,
    "model_version": MODEL_VERSION,
    "target_horizons_s": TARGET_HORIZONS_S,
    "feature_names": TRAJECTORY_MODEL_FEATURE_NAMES,
    "training_label_source": "same-vehicle measured future x/y positions only",
    "manual_training_rows": 0,
    "manual_evaluation_scope": "all rlog variants in complete labeled segments are always held out from fitting",
    "dataset": dataset_metadata or existing_report.get("dataset", {}),
    "training": training_reports or existing_report.get("training", []),
    "manual_evaluation": {
      "held_out_segments": len(evaluation_segments(
        args.evaluation_labels, args.routes_root, args.validation_cases,
      )),
      "summary": evaluation_tables,
      "rows": [asdict(row) for row in evaluation_rows],
    },
  }
  args.report.parent.mkdir(parents=True, exist_ok=True)
  args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
  print(f"report: {args.report}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
