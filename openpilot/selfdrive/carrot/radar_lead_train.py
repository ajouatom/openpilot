#!/usr/bin/env python3
"""Train the compact radar lead candidate MLP used by radar_lead_simulator.py."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar_lead_simulator import MODEL_FEATURE_NAMES


SOURCE_FEATURE_NAMES = ("source_front", "source_scc", "source_corner235", "source_corner180")
MAX_SOURCE_BALANCE_WEIGHT = 8.0


@dataclass(frozen=True)
class TrainingData:
  features: np.ndarray
  labels: np.ndarray
  groups: np.ndarray
  sources: np.ndarray
  sample_weights: np.ndarray
  manual: np.ndarray


def _open_dataset(path: Path) -> Any:
  return gzip.open(path, "rt", newline="", encoding="utf-8") if path.suffix.lower() == ".gz" else path.open(
    "r", newline="", encoding="utf-8"
  )


def load_datasets(paths: list[Path], manual_weight: float) -> TrainingData:
  feature_rows: list[list[float]] = []
  labels: list[float] = []
  groups: list[int] = []
  sample_weights: list[float] = []
  manual: list[bool] = []
  sources: list[int] = []
  group_ids: dict[tuple[int, str], int] = {}

  for dataset_index, path in enumerate(paths):
    if not path.is_file():
      raise RuntimeError(f"dataset does not exist: {path}")
    with _open_dataset(path) as source:
      reader = csv.DictReader(source)
      missing = [name for name in MODEL_FEATURE_NAMES if name not in (reader.fieldnames or [])]
      if missing:
        raise RuntimeError(f"dataset feature schema is outdated ({missing[0]} missing): {path}")
      for row in reader:
        feature_rows.append([float(row[name] or 0.0) for name in MODEL_FEATURE_NAMES])
        labels.append(float(row["is_positive"]))
        group_key = (dataset_index, row["frame"])
        groups.append(group_ids.setdefault(group_key, len(group_ids)))
        sources.append(int(float(row["source_corner235"] or 0.0) > 0.5 or float(row["source_corner180"] or 0.0) > 0.5))
        is_manual = row["label_source"] == "manual"
        sample_weights.append(manual_weight if is_manual else 1.0)
        manual.append(is_manual)

  if not feature_rows:
    raise RuntimeError("no training rows found")
  features = np.nan_to_num(np.asarray(feature_rows, dtype=np.float32), nan=0.0, posinf=1e4, neginf=-1e4)
  labels_array = np.asarray(labels, dtype=np.float32)
  sample_weights_array = np.asarray(sample_weights, dtype=np.float32)
  source_indices = [MODEL_FEATURE_NAMES.index(name) for name in SOURCE_FEATURE_NAMES]
  positive_mask = labels_array > 0.5
  source_counts = np.asarray([
    np.sum(positive_mask & (features[:, feature_index] > 0.5)) for feature_index in source_indices
  ], dtype=np.float32)
  nonzero_counts = source_counts[source_counts > 0.0]
  if len(nonzero_counts):
    dominant_count = float(nonzero_counts.max())
    for feature_index, count in zip(source_indices, source_counts, strict=True):
      if count <= 0.0:
        continue
      source_positive = positive_mask & (features[:, feature_index] > 0.5)
      source_weight = min(MAX_SOURCE_BALANCE_WEIGHT, math.sqrt(dominant_count / float(count)))
      sample_weights_array[source_positive] *= source_weight
  return TrainingData(
    features=features,
    labels=labels_array,
    groups=np.asarray(groups, dtype=np.int32),
    sources=np.asarray(sources, dtype=np.int8),
    sample_weights=sample_weights_array,
    manual=np.asarray(manual, dtype=np.bool_),
  )


def split_by_group(groups: np.ndarray, validation_fraction: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
  unique_groups = np.unique(groups)
  if len(unique_groups) < 2:
    raise RuntimeError("at least two labeled frame/role groups are required")
  rng = np.random.default_rng(seed)
  rng.shuffle(unique_groups)
  validation_count = min(len(unique_groups) - 1, max(1, int(round(len(unique_groups) * validation_fraction))))
  validation_groups = unique_groups[:validation_count]
  validation_mask = np.isin(groups, validation_groups)
  return np.flatnonzero(~validation_mask), np.flatnonzero(validation_mask)


def combine_training_and_validation(training: TrainingData, validation: TrainingData) -> tuple[TrainingData, np.ndarray, np.ndarray]:
  group_offset = int(training.groups.max()) + 1
  combined = TrainingData(
    features=np.concatenate((training.features, validation.features), axis=0),
    labels=np.concatenate((training.labels, validation.labels), axis=0),
    groups=np.concatenate((training.groups, validation.groups + group_offset), axis=0),
    sources=np.concatenate((training.sources, validation.sources), axis=0),
    sample_weights=np.concatenate((training.sample_weights, validation.sample_weights), axis=0),
    manual=np.concatenate((training.manual, validation.manual), axis=0),
  )
  training_indices = np.arange(len(training.labels), dtype=np.int64)
  validation_indices = np.arange(len(training.labels), len(combined.labels), dtype=np.int64)
  return combined, training_indices, validation_indices


def sigmoid(logits: np.ndarray) -> np.ndarray:
  return 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))


class TinyMLP:
  def __init__(self, input_size: int, hidden1: int, hidden2: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    self.parameters = {
      "w1": (rng.standard_normal((input_size, hidden1)) * math.sqrt(2.0 / input_size)).astype(np.float32),
      "b1": np.zeros(hidden1, dtype=np.float32),
      "w2": (rng.standard_normal((hidden1, hidden2)) * math.sqrt(2.0 / hidden1)).astype(np.float32),
      "b2": np.zeros(hidden2, dtype=np.float32),
      "w3": (rng.standard_normal((hidden2, 1)) * math.sqrt(1.0 / hidden2)).astype(np.float32),
      "b3": np.zeros(1, dtype=np.float32),
    }

  def predict_logits(self, features: np.ndarray) -> np.ndarray:
    p = self.parameters
    hidden1 = np.maximum(features @ p["w1"] + p["b1"], 0.0)
    hidden2 = np.maximum(hidden1 @ p["w2"] + p["b2"], 0.0)
    return (hidden2 @ p["w3"] + p["b3"]).reshape(-1)

  def gradients(
    self,
    features: np.ndarray,
    labels: np.ndarray,
    sample_weights: np.ndarray,
    positive_weight: float,
    l2: float,
  ) -> tuple[float, dict[str, np.ndarray]]:
    p = self.parameters
    z1 = features @ p["w1"] + p["b1"]
    hidden1 = np.maximum(z1, 0.0)
    z2 = hidden1 @ p["w2"] + p["b2"]
    hidden2 = np.maximum(z2, 0.0)
    logits = (hidden2 @ p["w3"] + p["b3"]).reshape(-1)
    probabilities = sigmoid(logits)
    weights = sample_weights * np.where(labels > 0.5, positive_weight, 1.0)
    denominator = max(float(weights.sum()), 1.0)
    loss = -float(np.sum(weights * (
      labels * np.log(np.maximum(probabilities, 1e-7)) +
      (1.0 - labels) * np.log(np.maximum(1.0 - probabilities, 1e-7))
    )) / denominator)

    d_logits = weights * (probabilities - labels) / denominator
    gradients: dict[str, np.ndarray] = {}
    gradients["w3"] = hidden2.T @ d_logits[:, None] + l2 * p["w3"]
    gradients["b3"] = np.asarray([d_logits.sum()], dtype=np.float32)
    d_hidden2 = d_logits[:, None] @ p["w3"].T
    d_z2 = d_hidden2 * (z2 > 0.0)
    gradients["w2"] = hidden1.T @ d_z2 + l2 * p["w2"]
    gradients["b2"] = d_z2.sum(axis=0)
    d_hidden1 = d_z2 @ p["w2"].T
    d_z1 = d_hidden1 * (z1 > 0.0)
    gradients["w1"] = features.T @ d_z1 + l2 * p["w1"]
    gradients["b1"] = d_z1.sum(axis=0)
    return loss, gradients


class Adam:
  def __init__(self, parameters: dict[str, np.ndarray], learning_rate: float) -> None:
    self.learning_rate = learning_rate
    self.step = 0
    self.m = {name: np.zeros_like(value) for name, value in parameters.items()}
    self.v = {name: np.zeros_like(value) for name, value in parameters.items()}

  def update(self, parameters: dict[str, np.ndarray], gradients: dict[str, np.ndarray]) -> None:
    self.step += 1
    for name, parameter in parameters.items():
      gradient = gradients[name]
      self.m[name] = 0.9 * self.m[name] + 0.1 * gradient
      self.v[name] = 0.999 * self.v[name] + 0.001 * gradient * gradient
      corrected_m = self.m[name] / (1.0 - 0.9 ** self.step)
      corrected_v = self.v[name] / (1.0 - 0.999 ** self.step)
      parameter -= self.learning_rate * corrected_m / (np.sqrt(corrected_v) + 1e-8)


def weighted_bce(
  probabilities: np.ndarray,
  labels: np.ndarray,
  sample_weights: np.ndarray,
  positive_weight: float,
) -> float:
  weights = sample_weights * np.where(labels > 0.5, positive_weight, 1.0)
  return -float(np.sum(weights * (
    labels * np.log(np.maximum(probabilities, 1e-7)) +
    (1.0 - labels) * np.log(np.maximum(1.0 - probabilities, 1e-7))
  )) / max(float(weights.sum()), 1.0))


def fit_probability_calibration(
  logits: np.ndarray,
  labels: np.ndarray,
  max_iterations: int = 50,
) -> tuple[float, float]:
  """Fit an affine sigmoid calibration without the training class weights."""
  if len(logits) == 0 or np.all(labels == labels[0]):
    return 1.0, 0.0
  design = np.column_stack((logits.astype(np.float64), np.ones(len(logits), dtype=np.float64)))
  parameters = np.asarray([1.0, 0.0], dtype=np.float64)
  regularization = np.diag((1e-3, 1e-4))
  for _ in range(max_iterations):
    calibrated_logits = np.clip(design @ parameters, -30.0, 30.0)
    probabilities = 1.0 / (1.0 + np.exp(-calibrated_logits))
    gradient = design.T @ (probabilities - labels) / len(logits) + regularization @ parameters
    curvature = probabilities * (1.0 - probabilities)
    hessian = design.T @ (design * curvature[:, None]) / len(logits) + regularization
    try:
      step = np.linalg.solve(hessian, gradient)
    except np.linalg.LinAlgError:
      break
    parameters -= step
    if float(np.max(np.abs(step))) < 1e-7:
      break
  scale, bias = (float(value) for value in parameters)
  if scale <= 0.0 or not (math.isfinite(scale) and math.isfinite(bias)):
    return 1.0, 0.0
  return float(np.clip(scale, 0.05, 10.0)), float(np.clip(bias, -15.0, 15.0))


def probability_calibration_metrics(
  probabilities: np.ndarray,
  labels: np.ndarray,
  bins: int = 10,
) -> dict[str, float]:
  if len(probabilities) == 0:
    return {"brier": 0.0, "log_loss": 0.0, "ece": 0.0}
  clipped = np.clip(probabilities.astype(np.float64), 1e-7, 1.0 - 1e-7)
  brier = float(np.mean((clipped - labels) ** 2))
  log_loss = -float(np.mean(labels * np.log(clipped) + (1.0 - labels) * np.log(1.0 - clipped)))
  expected_calibration_error = 0.0
  for lower in np.linspace(0.0, 1.0, bins, endpoint=False):
    upper = lower + 1.0 / bins
    mask = (clipped >= lower) & (clipped < upper if upper < 1.0 else clipped <= upper)
    if np.any(mask):
      expected_calibration_error += float(np.mean(mask)) * abs(float(np.mean(clipped[mask]) - np.mean(labels[mask])))
  return {"brier": brier, "log_loss": log_loss, "ece": expected_calibration_error}


def group_metrics(
  probabilities: np.ndarray,
  labels: np.ndarray,
  groups: np.ndarray,
  threshold: float | tuple[float, float],
  sources: np.ndarray | None = None,
  max_outputs: int = 2,
) -> dict[str, float | int]:
  exact = 0
  groups_total = 0
  target_exact = 0
  target_total = 0
  none_correct = 0
  none_total = 0
  true_positive = 0
  false_positive = 0
  false_negative = 0
  thresholds = (float(threshold), float(threshold)) if isinstance(threshold, (int, float)) else threshold
  for group in np.unique(groups):
    indices = np.flatnonzero(groups == group)
    predicted: set[int] = set()
    source_values = (0,) if sources is None else (0, 1)
    for source_index in source_values:
      source_indices = indices if sources is None else indices[sources[indices] == source_index]
      ranked = source_indices[np.argsort(probabilities[source_indices])[::-1]]
      selected = ranked[probabilities[ranked] >= thresholds[source_index]][:max_outputs]
      predicted.update(selected.tolist())
    targets = set(indices[labels[indices] > 0.5].tolist())
    is_exact = predicted == targets
    exact += int(is_exact)
    groups_total += 1
    true_positive += len(predicted & targets)
    false_positive += len(predicted - targets)
    false_negative += len(targets - predicted)
    if targets:
      target_exact += int(is_exact)
      target_total += 1
    else:
      none_correct += int(not predicted)
      none_total += 1
  precision = true_positive / max(true_positive + false_positive, 1)
  recall = true_positive / max(true_positive + false_negative, 1)
  return {
    "exact": exact,
    "groups": groups_total,
    "target_exact": target_exact,
    "target_groups": target_total,
    "none_correct": none_correct,
    "none_groups": none_total,
    "precision": precision,
    "recall": recall,
    "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
  }


def calibrate_threshold(
  probabilities: np.ndarray,
  labels: np.ndarray,
  groups: np.ndarray,
  sources: np.ndarray | None = None,
  source_index: int = 0,
) -> float:
  if sources is not None:
    mask = sources == source_index
    probabilities = probabilities[mask]
    labels = labels[mask]
    groups = groups[mask]
  if len(probabilities) == 0:
    return 0.5
  best_threshold = 0.5
  best_score = -1.0
  for threshold in np.linspace(0.05, 0.95, 91):
    metrics = group_metrics(probabilities, labels, groups, float(threshold))
    target_accuracy = float(metrics["target_exact"]) / max(int(metrics["target_groups"]), 1)
    none_accuracy = float(metrics["none_correct"]) / max(int(metrics["none_groups"]), 1)
    if metrics["target_groups"] and metrics["none_groups"]:
      score = 0.5 * (target_accuracy + none_accuracy)
    else:
      score = float(metrics["f1"])
    if score > best_score + 1e-9 or (abs(score - best_score) <= 1e-9 and threshold > best_threshold):
      best_score = score
      best_threshold = float(threshold)
  return best_threshold


def parse_hidden(value: str) -> tuple[int, int]:
  try:
    first, second = (int(part) for part in value.split(",", 1))
  except (TypeError, ValueError) as exc:
    raise argparse.ArgumentTypeError("hidden sizes must look like 48,24") from exc
  if first <= 0 or second <= 0:
    raise argparse.ArgumentTypeError("hidden sizes must be positive")
  return first, second


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Train a compact radar lead candidate MLP")
  parser.add_argument("datasets", nargs="+", type=Path, help="training CSV or CSV.GZ files")
  parser.add_argument("--output", required=True, type=Path, help="output .npz model")
  parser.add_argument("--epochs", type=int, default=30)
  parser.add_argument("--batch-size", type=int, default=2048)
  parser.add_argument("--learning-rate", type=float, default=0.002)
  parser.add_argument("--hidden", type=parse_hidden, default=(48, 24), help="two hidden sizes, e.g. 48,24")
  parser.add_argument("--validation", type=float, default=0.2)
  parser.add_argument(
    "--validation-dataset", action="append", type=Path, default=[],
    help="held-out CSV/CSV.GZ; repeat for multiple logs",
  )
  parser.add_argument("--manual-weight", type=float, default=4.0)
  parser.add_argument("--positive-weight", type=float, help="default: auto from training class balance")
  parser.add_argument("--l2", type=float, default=1e-5)
  parser.add_argument("--patience", type=int, default=8, help="stop after this many epochs without validation improvement")
  parser.add_argument("--seed", type=int, default=42)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if not 0.0 < args.validation < 1.0:
    raise SystemExit("--validation must be between 0 and 1")
  if args.epochs <= 0 or args.batch_size <= 0:
    raise SystemExit("--epochs and --batch-size must be positive")

  print(f"Loading {len(args.datasets)} dataset(s) ...", flush=True)
  training_data = load_datasets(args.datasets, args.manual_weight)
  if args.validation_dataset:
    held_out_data = load_datasets(args.validation_dataset, args.manual_weight)
    data, train_indices, validation_indices = combine_training_and_validation(training_data, held_out_data)
    validation_kind = f"held-out logs ({len(args.validation_dataset)})"
  else:
    data = training_data
    train_indices, validation_indices = split_by_group(data.groups, args.validation, args.seed)
    validation_kind = "random frame groups"
  mean = data.features[train_indices].mean(axis=0, dtype=np.float64).astype(np.float32)
  std = data.features[train_indices].std(axis=0, dtype=np.float64).astype(np.float32)
  std[std < 1e-5] = 1.0
  features = ((data.features - mean) / std).astype(np.float32)

  train_positives = max(float(data.labels[train_indices].sum()), 1.0)
  train_negatives = max(float(len(train_indices) - train_positives), 1.0)
  positive_weight = args.positive_weight or min(30.0, train_negatives / train_positives)
  hidden1, hidden2 = args.hidden
  model = TinyMLP(len(MODEL_FEATURE_NAMES), hidden1, hidden2, args.seed)
  optimizer = Adam(model.parameters, args.learning_rate)
  rng = np.random.default_rng(args.seed)
  best_loss = math.inf
  best_epoch = 0
  best_parameters = {name: value.copy() for name, value in model.parameters.items()}

  print(
    f"rows {len(data.labels)}  groups {len(np.unique(data.groups))}  train {len(train_indices)}  "
    f"validation {len(validation_indices)} [{validation_kind}]  manual rows {int(data.manual.sum())}  "
    f"positive weight {positive_weight:.2f}"
  )
  if not np.any(data.manual):
    print("WARNING: no manual ground truth; metrics measure imitation of dataset teacher labels only")
  for epoch in range(1, args.epochs + 1):
    shuffled = rng.permutation(train_indices)
    train_loss_sum = 0.0
    batches = 0
    for start in range(0, len(shuffled), args.batch_size):
      batch = shuffled[start:start + args.batch_size]
      loss, gradients = model.gradients(
        features[batch], data.labels[batch], data.sample_weights[batch], positive_weight, args.l2
      )
      optimizer.update(model.parameters, gradients)
      train_loss_sum += loss
      batches += 1

    validation_probabilities = sigmoid(model.predict_logits(features[validation_indices]))
    validation_loss = weighted_bce(
      validation_probabilities,
      data.labels[validation_indices],
      data.sample_weights[validation_indices],
      positive_weight,
    )
    if validation_loss < best_loss:
      best_loss = validation_loss
      best_epoch = epoch
      best_parameters = {name: value.copy() for name, value in model.parameters.items()}
    if epoch == 1 or epoch % 5 == 0 or epoch == args.epochs:
      print(f"epoch {epoch:3d}/{args.epochs}  train {train_loss_sum / max(batches, 1):.4f}  val {validation_loss:.4f}")
    if args.patience > 0 and epoch - best_epoch >= args.patience:
      print(f"early stop at epoch {epoch}; best validation epoch {best_epoch}")
      break

  model.parameters = best_parameters
  validation_logits = model.predict_logits(features[validation_indices])
  validation_labels = data.labels[validation_indices]
  validation_groups = data.groups[validation_indices]
  validation_sources = data.sources[validation_indices]
  validation_probabilities = np.zeros_like(validation_logits, dtype=np.float32)
  calibrations: list[tuple[float, float]] = []
  calibration_by_source: dict[str, dict[str, float]] = {}
  for source_index, source_name in enumerate(("front", "corner")):
    mask = validation_sources == source_index
    scale, bias = fit_probability_calibration(validation_logits[mask], validation_labels[mask])
    calibrations.append((scale, bias))
    validation_probabilities[mask] = sigmoid(validation_logits[mask] * scale + bias)
    calibration_by_source[source_name] = probability_calibration_metrics(
      validation_probabilities[mask], validation_labels[mask]
    )
  thresholds = tuple(
    calibrate_threshold(validation_probabilities, validation_labels, validation_groups, validation_sources, source_index)
    for source_index in range(2)
  )
  validation_metrics = group_metrics(
    validation_probabilities, validation_labels, validation_groups, thresholds, validation_sources
  )
  calibration_metrics = probability_calibration_metrics(validation_probabilities, validation_labels)
  metrics: dict[str, Any] = {
    "validation_loss": best_loss,
    "best_epoch": best_epoch,
    "positive_weight": positive_weight,
    "thresholds": thresholds,
    "calibrations": calibrations,
    "calibration_by_source": calibration_by_source,
    **calibration_metrics,
    **validation_metrics,
  }
  print(
    f"selected: exact {validation_metrics['exact']}/{validation_metrics['groups']}  "
    f"threshold front {thresholds[0]:.2f} corner {thresholds[1]:.2f}  "
    f"target {validation_metrics['target_exact']}/{validation_metrics['target_groups']}  "
    f"none {validation_metrics['none_correct']}/{validation_metrics['none_groups']}  "
    f"precision {validation_metrics['precision'] * 100:.1f}%  recall {validation_metrics['recall'] * 100:.1f}%  "
    f"Brier {calibration_metrics['brier']:.4f}  ECE {calibration_metrics['ece']:.4f}"
  )

  output = args.output if args.output.suffix.lower() == ".npz" else args.output.with_suffix(".npz")
  output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(
    output,
    version=np.asarray([4], dtype=np.int32),
    feature_names=np.asarray(MODEL_FEATURE_NAMES),
    mean=mean,
    std=std,
    thresholds=np.asarray(thresholds, dtype=np.float32),
    calibration=np.asarray(calibrations, dtype=np.float32),
    metrics=np.asarray(json.dumps(metrics, sort_keys=True)),
    **model.parameters,
  )
  print(f"model written: {output}  size {output.stat().st_size / 1024.0:.1f} KiB")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
