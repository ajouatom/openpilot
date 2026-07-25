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
  VEHICLE_HALF_WIDTH_M,
  path_relative_state,
  radar_point_measured,
  radar_point_source,
  radar_point_track_id,
  radar_track_continuous,
  trajectory_model_feature_row,
)
from openpilot.selfdrive.carrot.radar.radar_trajectory_model import (
  MODEL_VERSION,
  RadarTrajectoryRuntime,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_simulator import RadarFrame, RadarPoint, load_frames


FUTURE_TIME_TOLERANCE_S = 0.09
MIN_HISTORY_COUNT = 3
MIN_SAMPLE_PERIOD_S = 0.20
MAX_NEGATIVE_TO_POSITIVE = 6
MAX_NEGATIVE_ROWS_WITHOUT_POSITIVE = 1200
LOG_CACHE_VERSION = 1
RLOG_PATTERN = re.compile(r"^rlog(?:\.\d+)?\.zst$", re.IGNORECASE)


@dataclass(frozen=True)
class Dataset:
  features: np.ndarray
  labels: np.ndarray
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
  return float(abs(d_path) <= lane_half_width + VEHICLE_HALF_WIDTH_M)


def _future_targets(
  frames: Sequence[RadarFrame],
  observation: TrackObservation,
  observations: Sequence[TrackObservation],
) -> tuple[list[float], list[bool]]:
  labels = []
  valid = []
  for horizon_s in TARGET_HORIZONS_S:
    future = _future_observation(observations, observation.time_s + horizon_s)
    is_valid = future is not None
    valid.append(is_valid)
    labels.append(
      _actual_path_occupancy(frames[future.frame_index], future.point)
      if future is not None
      else 0.0
    )
  return labels, valid


def _downsample_rows(
  rows: list[tuple[list[float], list[float], list[bool], bool, str, str]],
) -> list[tuple[list[float], list[float], list[bool], bool, str, str]]:
  positive = []
  negative = []
  for row in rows:
    target = any(
      value > 0.5
      for value, valid in zip(row[1], row[2], strict=True)
      if valid
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
) -> dict[str, list[tuple[list[float], list[float], list[bool], bool, str, str]]]:
  observations_by_frame, episodes = _track_observations(frames)
  analyzer = RadarTrajectoryAnalyzer()
  last_sample_time: dict[tuple[str, int, int], float] = {}
  output: dict[str, list[tuple[list[float], list[float], list[bool], bool, str, str]]] = defaultdict(list)
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
      labels, valid = _future_targets(frames, observation, episodes[episode_key])
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
        valid,
        current_occupancy,
        sample_id,
        log_group,
      ))
      last_sample_time[episode_key] = frame.mono_time_s
  return {source: _downsample_rows(rows) for source, rows in output.items()}


def _dataset_from_rows(
  rows: dict[str, list[tuple[list[float], list[float], list[bool], bool, str, str]]],
) -> dict[str, Dataset]:
  return {
    source: Dataset(
      features=np.asarray([row[0] for row in source_rows], dtype=np.float32),
      labels=np.asarray([row[1] for row in source_rows], dtype=np.float32),
      valid=np.asarray([row[2] for row in source_rows], dtype=np.bool_),
      current_occupancy=np.asarray([row[3] for row in source_rows], dtype=np.bool_),
      sample_ids=np.asarray([row[4] for row in source_rows]),
      log_groups=np.asarray([row[5] for row in source_rows]),
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


def _log_fingerprint(path: Path) -> dict[str, int]:
  stat = path.stat()
  return {"size": stat.st_size, "mtime_ns": stat.st_mtime_ns}


def _save_log_rows(
  path: Path,
  relative_log: str,
  fingerprint: dict[str, int],
  rows: dict[str, list[tuple[list[float], list[float], list[bool], bool, str, str]]],
) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  values: dict[str, np.ndarray] = {}
  for source, source_rows in rows.items():
    if not source_rows:
      continue
    values.update({
      f"{source}_features": np.asarray([row[0] for row in source_rows], dtype=np.float32),
      f"{source}_labels": np.asarray([row[1] for row in source_rows], dtype=np.float32),
      f"{source}_valid": np.asarray([row[2] for row in source_rows], dtype=np.bool_),
      f"{source}_current_occupancy": np.asarray([row[3] for row in source_rows], dtype=np.bool_),
      f"{source}_sample_ids": np.asarray([row[4] for row in source_rows]),
      f"{source}_log_groups": np.asarray([row[5] for row in source_rows]),
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
) -> dict[str, list[tuple[list[float], list[float], list[bool], bool, str, str]]] | None:
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
          valid.tolist(),
          bool(current),
          str(sample_id),
          str(log_group),
        )
        for feature, labels, valid, current, sample_id, log_group in zip(
          payload[f"{source}_features"],
          payload[f"{source}_labels"],
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


def build_dataset(
  log_paths: Sequence[Path],
  routes_root: Path,
  cache_path: Path | None,
  log_cache_dir: Path | None = None,
) -> tuple[dict[str, Dataset], dict[str, Any]]:
  rows: dict[str, list[tuple[list[float], list[float], list[bool], bool, str, str]]] = defaultdict(list)
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
    for source, source_rows in frame_rows.items():
      rows[source].extend(source_rows)
  datasets = _dataset_from_rows(rows)
  metadata = {
    "provenance": "self-supervised same-vehicle measured future path occupancy",
    "target_semantics": "unconditional path occupancy at each future horizon",
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
          ("valid", dataset.valid),
          ("current_occupancy", dataset.current_occupancy),
          ("sample_ids", dataset.sample_ids),
          ("log_groups", dataset.log_groups),
        )
      },
    )
  return datasets, metadata


def load_dataset(path: Path) -> tuple[dict[str, Dataset], dict[str, Any]]:
  payload = np.load(path, allow_pickle=False)
  names = tuple(str(value) for value in payload["feature_names"].tolist())
  horizons = tuple(float(value) for value in payload["target_horizons_s"].tolist())
  manual_rows = int(payload["manual_training_rows"].reshape(-1)[0])
  if names != TRAJECTORY_MODEL_FEATURE_NAMES:
    raise ValueError("cached trajectory feature schema mismatch")
  if horizons != TARGET_HORIZONS_S:
    raise ValueError("cached trajectory target horizon schema mismatch")
  if manual_rows != 0:
    raise ValueError("manual labels are forbidden in the trajectory training cache")
  datasets = {
    source: Dataset(
      payload[f"{source}_features"],
      payload[f"{source}_labels"],
      payload[f"{source}_valid"].astype(np.bool_),
      payload[f"{source}_current_occupancy"].astype(np.bool_),
      payload[f"{source}_sample_ids"],
      payload[f"{source}_log_groups"],
    )
    for source in ("front", "corner")
    if f"{source}_features" in payload
  }
  metadata = json.loads(str(payload["metadata_json"].reshape(-1)[0]))
  return datasets, metadata


def _row_targets(dataset: Dataset) -> tuple[np.ndarray, np.ndarray]:
  usable = np.any(dataset.valid, axis=1)
  targets = np.any((dataset.labels > 0.5) & dataset.valid, axis=1)
  return targets, usable


def _group_folds(dataset: Dataset, seed: int, fold_count: int) -> list[np.ndarray]:
  groups = np.unique(dataset.log_groups)
  targets, usable = _row_targets(dataset)
  effective_folds = min(fold_count, len(groups))
  if effective_folds < 2:
    raise RuntimeError("not enough log groups for grouped validation")
  rng = np.random.default_rng(seed)
  shuffled = groups.copy()
  rng.shuffle(shuffled)
  ranked = sorted(
    shuffled,
    key=lambda group: float(np.mean(targets[(dataset.log_groups == group) & usable])),
    reverse=True,
  )
  buckets: list[list[str]] = [[] for _ in range(effective_folds)]
  for index, group in enumerate(ranked):
    cycle, offset = divmod(index, effective_folds)
    bucket_index = offset if cycle % 2 == 0 else effective_folds - 1 - offset
    buckets[bucket_index].append(group)
  return [np.isin(dataset.log_groups, bucket) for bucket in buckets]


def _require_split(source: str, name: str, dataset: Dataset) -> None:
  targets, usable = _row_targets(dataset)
  if not np.any(usable):
    raise RuntimeError(f"{source} {name} split has no valid targets")
  if len(np.unique(targets[usable])) < 2:
    raise RuntimeError(f"{source} {name} split must contain occupied and clear samples")


class BinaryMLP:
  def __init__(self, inputs: int, outputs: int, hidden1: int, hidden2: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    self.p = {
      "w1": (rng.standard_normal((inputs, hidden1)) * math.sqrt(2.0 / inputs)).astype(np.float32),
      "b1": np.zeros(hidden1, dtype=np.float32),
      "w2": (rng.standard_normal((hidden1, hidden2)) * math.sqrt(2.0 / hidden1)).astype(np.float32),
      "b2": np.zeros(hidden2, dtype=np.float32),
      "w3": (rng.standard_normal((hidden2, outputs)) * math.sqrt(1.0 / hidden2)).astype(np.float32),
      "b3": np.zeros(outputs, dtype=np.float32),
    }
    self.m = {name: np.zeros_like(value) for name, value in self.p.items()}
    self.v = {name: np.zeros_like(value) for name, value in self.p.items()}
    self.step = 0

  def logits(self, x: np.ndarray) -> np.ndarray:
    h1 = np.maximum(x @ self.p["w1"] + self.p["b1"], 0.0)
    h2 = np.maximum(h1 @ self.p["w2"] + self.p["b2"], 0.0)
    return h2 @ self.p["w3"] + self.p["b3"]

  def update(
    self,
    x: np.ndarray,
    y: np.ndarray,
    valid: np.ndarray,
    positive_weight: np.ndarray,
    learning_rate: float,
    l2_weight: float,
  ) -> float:
    z1 = x @ self.p["w1"] + self.p["b1"]
    h1 = np.maximum(z1, 0.0)
    z2 = h1 @ self.p["w2"] + self.p["b2"]
    h2 = np.maximum(z2, 0.0)
    logits = h2 @ self.p["w3"] + self.p["b3"]
    probability = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
    weight = np.where(y > 0.5, positive_weight[None, :], 1.0) * valid
    denominator = max(float(weight.sum()), 1.0)
    loss = -float(np.sum(weight * (
      y * np.log(np.maximum(probability, 1e-7))
      + (1.0 - y) * np.log(np.maximum(1.0 - probability, 1e-7))
    )) / denominator)
    loss += 0.5 * l2_weight * sum(
      float(np.sum(self.p[name] ** 2)) for name in ("w1", "w2", "w3")
    )
    d_logits = weight * (probability - y) / denominator
    gradients = {
      "w3": h2.T @ d_logits,
      "b3": d_logits.sum(axis=0),
    }
    d_h2 = d_logits @ self.p["w3"].T
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


def _event_scores_targets(
  probabilities: np.ndarray,
  dataset: Dataset,
  event: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  valid = np.any(dataset.valid, axis=1)
  if event == "entry":
    scores = np.max(np.where(dataset.valid, probabilities, 0.0), axis=1)
    targets = np.any((dataset.labels > 0.5) & dataset.valid, axis=1)
    usable = valid & ~dataset.current_occupancy
  elif event == "exit":
    scores = np.max(np.where(dataset.valid, 1.0 - probabilities, 0.0), axis=1)
    targets = np.any((dataset.labels <= 0.5) & dataset.valid, axis=1)
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


def _horizon_metrics(probabilities: np.ndarray, dataset: Dataset) -> dict[str, dict[str, float | int]]:
  result = {}
  for index, horizon_s in enumerate(TARGET_HORIZONS_S):
    valid = dataset.valid[:, index]
    predicted = probabilities[valid, index] >= 0.5
    targets = dataset.labels[valid, index] > 0.5
    tp = int(np.sum(predicted & targets))
    fp = int(np.sum(predicted & ~targets))
    fn = int(np.sum(~predicted & targets))
    tn = int(np.sum(~predicted & ~targets))
    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    result[f"{horizon_s:.1f}"] = {
      "precision": precision,
      "recall": recall,
      "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
      "tp": tp,
      "fp": fp,
      "fn": fn,
      "tn": tn,
    }
  return result


def _subset(dataset: Dataset, mask: np.ndarray) -> Dataset:
  return Dataset(
    dataset.features[mask],
    dataset.labels[mask],
    dataset.valid[mask],
    dataset.current_occupancy[mask],
    dataset.sample_ids[mask],
    dataset.log_groups[mask],
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
) -> tuple[BinaryMLP, np.ndarray, np.ndarray, int]:
  mean = train.features.mean(axis=0)
  std = train.features.std(axis=0)
  std[std < 1e-4] = 1.0
  normalized = (train.features - mean) / std
  validation_x = None if validation is None else (validation.features - mean) / std
  model = BinaryMLP(normalized.shape[1], len(TARGET_HORIZONS_S), 48, 24, seed)
  positive_weight = np.ones(len(TARGET_HORIZONS_S), dtype=np.float32)
  for index in range(len(TARGET_HORIZONS_S)):
    valid = train.valid[:, index]
    positives = np.sum((train.labels[:, index] > 0.5) & valid)
    negatives = np.sum((train.labels[:, index] <= 0.5) & valid)
    positive_weight[index] = min(20.0, max(1.0, float(negatives) / max(float(positives), 1.0)))
  rng = np.random.default_rng(seed)
  best_validation_f1 = -1.0
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
        train.labels[indices],
        train.valid[indices],
        positive_weight,
        learning_rate,
        l2_weight,
      ))

    epoch_f1 = 0.0
    if validation is not None and validation_x is not None:
      validation_prob = 1.0 / (
        1.0 + np.exp(-np.clip(model.logits(validation_x), -30.0, 30.0))
      )
      _, epoch_metrics = _choose_threshold(validation_prob, validation, "entry")
      epoch_f1 = float(epoch_metrics["f1"])
      if epoch_f1 > best_validation_f1 + 1e-6:
        best_validation_f1 = epoch_f1
        best_parameters = {name: value.copy() for name, value in model.p.items()}
        best_epoch = epoch + 1
        patience = 0
      else:
        patience += 1
    if verbose and (epoch == 0 or (epoch + 1) % 10 == 0):
      suffix = "" if validation is None else f" val-f1 {epoch_f1:.3f}"
      print(f"{source} epoch {epoch + 1:03d}/{epochs} loss {np.mean(losses):.4f}{suffix}")
    if validation is not None and epoch >= 20 and patience >= 18:
      break
  if best_parameters is not None:
    model.p = best_parameters
  return model, mean, std, best_epoch


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
  out_of_fold = np.zeros((len(dataset.features), len(TARGET_HORIZONS_S)), dtype=np.float32)
  best_epochs: list[int] = []
  for fold_index, validation_mask in enumerate(folds):
    train = _subset(dataset, ~validation_mask)
    validation = _subset(dataset, validation_mask)
    _require_split(source, f"fold {fold_index + 1} train", train)
    _require_split(source, f"fold {fold_index + 1} validation", validation)
    print(f"{source} fold {fold_index + 1}/{len(folds)}")
    fold_model, fold_mean, fold_std, best_epoch = _fit_model(
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
    out_of_fold[validation_mask] = 1.0 / (
      1.0 + np.exp(-np.clip(fold_model.logits(validation_x), -30.0, 30.0))
    )
    best_epochs.append(best_epoch)
  balanced_entry_threshold, balanced_entry_metrics = _choose_threshold(
    out_of_fold, dataset, "entry",
  )
  entry_threshold, entry_metrics = _choose_threshold(
    out_of_fold, dataset, "entry", target_precision=0.90,
  )
  balanced_exit_threshold, balanced_exit_metrics = _choose_threshold(
    out_of_fold, dataset, "exit",
  )
  exit_threshold, exit_metrics = _choose_threshold(
    out_of_fold, dataset, "exit", target_precision=0.95,
  )
  final_epochs = max(10, int(round(float(np.median(best_epochs)))))
  training_summary = (
    f"{source} CV entry {entry_threshold:.3f} "
    + f"P {float(entry_metrics['precision']):.3f} R {float(entry_metrics['recall']):.3f}; "
    + f"exit {exit_threshold:.3f} "
    + f"P {float(exit_metrics['precision']):.3f} R {float(exit_metrics['recall']):.3f}; "
    + f"final {final_epochs} epochs"
  )
  print(training_summary)
  model, mean, std, _ = _fit_model(
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
    training_provenance=np.asarray(["self-supervised same-vehicle measured future path occupancy"]),
    manual_training_rows=np.asarray([0], dtype=np.int32),
    feature_mean=mean.astype(np.float32),
    feature_std=std.astype(np.float32),
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
    "entry_cross_validation": entry_metrics,
    "balanced_entry_threshold": balanced_entry_threshold,
    "balanced_entry_cross_validation": balanced_entry_metrics,
    "exit_threshold": exit_threshold,
    "exit_cross_validation": exit_metrics,
    "balanced_exit_threshold": balanced_exit_threshold,
    "balanced_exit_cross_validation": balanced_exit_metrics,
    "horizon_cross_validation": _horizon_metrics(out_of_fold, dataset),
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
    scores: dict[str, list[tuple[float, float, tuple[float, ...], bool]]] = defaultdict(list)
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
            prediction.probability,
            prediction.path_exit_probability,
            prediction.horizon_probabilities,
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
          future_labels, valid = _future_targets(
            frames,
            observation,
            episodes[(radar_point_source(observation.point), radar_point_track_id(observation.point), observation.episode_id)],
          )
          if not any(valid):
            continue
          current_occupancy = _actual_path_occupancy(frame, observation.point) > 0.5
          future_occupied = any(
            occupied > 0.5 for occupied, is_valid in zip(future_labels, valid, strict=True)
            if is_valid
          )
          future_clear = any(
            occupied <= 0.5 for occupied, is_valid in zip(future_labels, valid, strict=True)
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
      selected = any(value[3] for value in values)
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
  parser.add_argument("--routes-root", type=Path, default=Path("W:/routes"))
  parser.add_argument(
    "--include-evaluation-logs",
    action="store_true",
    help="also fit on logs carrying manual labels (default reserves those complete logs)",
  )
  parser.add_argument("--max-logs", type=int)
  parser.add_argument(
    "--cache",
    type=Path,
    default=Path(".tmp_radar_review/trajectory_self_supervised_dataset.npz"),
  )
  parser.add_argument(
    "--log-cache-dir",
    type=Path,
    default=Path(".tmp_radar_review/path_occupancy_log_cache_v3"),
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
      if not args.include_evaluation_logs:
        evaluation_paths = {
          path.resolve() for path in evaluation_logs(args.evaluation_labels, args.routes_root)
        }
        log_paths = [path for path in log_paths if path.resolve() not in evaluation_paths]
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
    "training_label_source": "same-vehicle measured future path occupancy only",
    "manual_training_rows": 0,
    "manual_evaluation_scope": "complete held-out labeled logs unless include-evaluation-logs was requested",
    "dataset": dataset_metadata or existing_report.get("dataset", {}),
    "training": training_reports or existing_report.get("training", []),
    "manual_evaluation": {
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
