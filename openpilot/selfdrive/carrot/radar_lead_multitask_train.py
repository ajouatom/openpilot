#!/usr/bin/env python3
"""Train the production fused-radar lead/cut-in/external MLP."""

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

from openpilot.selfdrive.carrot.radar_lead_model import MODEL_FEATURE_NAMES, MODEL_HEADS


@dataclass(frozen=True)
class TrainingData:
  features: np.ndarray
  labels: np.ndarray
  weights: np.ndarray
  groups: np.ndarray
  manual: np.ndarray


def _open(path: Path) -> Any:
  return gzip.open(path, "rt", newline="", encoding="utf-8") if path.suffix.lower() == ".gz" else path.open(
    "r", newline="", encoding="utf-8"
  )


def load_datasets(paths: list[Path]) -> TrainingData:
  try:
    import pandas as pd
  except ImportError:
    pd = None

  if pd is not None:
    feature_chunks: list[np.ndarray] = []
    label_chunks: list[np.ndarray] = []
    weight_chunks: list[np.ndarray] = []
    group_chunks: list[np.ndarray] = []
    manual_chunks: list[np.ndarray] = []
    group_offset = 0
    columns = [
      *MODEL_FEATURE_NAMES,
      *(f"{name}_label" for name in MODEL_HEADS),
      *(f"{name}_weight" for name in MODEL_HEADS),
      *(f"{name}_source" for name in MODEL_HEADS),
      "frame",
    ]
    for path in paths:
      frame = pd.read_csv(path, usecols=columns)
      feature_chunks.append(frame[list(MODEL_FEATURE_NAMES)].fillna(0.0).to_numpy(dtype=np.float32))
      label_chunks.append(frame[[f"{name}_label" for name in MODEL_HEADS]].to_numpy(dtype=np.float32))
      weight_chunks.append(frame[[f"{name}_weight" for name in MODEL_HEADS]].to_numpy(dtype=np.float32))
      local_groups, unique_groups = pd.factorize(frame["frame"], sort=False)
      group_chunks.append(local_groups.astype(np.int32) + group_offset)
      group_offset += len(unique_groups)
      manual_chunks.append(np.column_stack([
        frame[f"{name}_source"].eq("manual").to_numpy(dtype=np.bool_) for name in MODEL_HEADS
      ]))
    if not feature_chunks:
      raise RuntimeError("no fused training rows found")
    return TrainingData(
      features=np.nan_to_num(np.concatenate(feature_chunks), nan=0.0, posinf=1e4, neginf=-1e4),
      labels=np.concatenate(label_chunks),
      weights=np.concatenate(weight_chunks),
      groups=np.concatenate(group_chunks),
      manual=np.concatenate(manual_chunks),
    )

  feature_rows: list[list[float]] = []
  labels: list[tuple[float, ...]] = []
  weights: list[tuple[float, ...]] = []
  groups: list[int] = []
  manual: list[tuple[bool, ...]] = []
  group_ids: dict[tuple[int, str], int] = {}
  for dataset_index, path in enumerate(paths):
    with _open(path) as source:
      reader = csv.DictReader(source)
      missing = [name for name in MODEL_FEATURE_NAMES if name not in (reader.fieldnames or [])]
      if missing:
        raise RuntimeError(f"fused dataset schema is outdated ({missing[0]} missing): {path}")
      for row in reader:
        feature_rows.append([float(row[name] or 0.0) for name in MODEL_FEATURE_NAMES])
        labels.append(tuple(float(row[f"{name}_label"]) for name in MODEL_HEADS))
        weights.append(tuple(float(row[f"{name}_weight"]) for name in MODEL_HEADS))
        groups.append(group_ids.setdefault((dataset_index, row["frame"]), len(group_ids)))
        manual.append(tuple(row[f"{name}_source"] == "manual" for name in MODEL_HEADS))
  if not feature_rows:
    raise RuntimeError("no fused training rows found")
  return TrainingData(
    features=np.nan_to_num(np.asarray(feature_rows, dtype=np.float32), nan=0.0, posinf=1e4, neginf=-1e4),
    labels=np.asarray(labels, dtype=np.float32),
    weights=np.asarray(weights, dtype=np.float32),
    groups=np.asarray(groups, dtype=np.int32),
    manual=np.asarray(manual, dtype=np.bool_),
  )


def load_cache(path: Path, sources: list[Path]) -> tuple[TrainingData, np.ndarray, np.ndarray] | None:
  if not path.exists():
    return None
  with np.load(path, allow_pickle=False) as cache:
    expected = tuple(str(source.resolve()) for source in sources)
    actual = tuple(str(value) for value in cache["sources"].tolist())
    if actual != expected:
      return None
    data = TrainingData(
      features=cache["features"], labels=cache["labels"], weights=cache["weights"],
      groups=cache["groups"], manual=cache["manual"],
    )
    return data, cache["train_indices"], cache["validation_indices"]


def save_cache(
  path: Path, sources: list[Path], data: TrainingData, train_indices: np.ndarray, validation_indices: np.ndarray,
) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  np.savez(
    path,
    sources=np.asarray([str(source.resolve()) for source in sources]),
    features=data.features, labels=data.labels, weights=data.weights, groups=data.groups, manual=data.manual,
    train_indices=train_indices, validation_indices=validation_indices,
  )


def combine(training: TrainingData, validation: TrainingData) -> tuple[TrainingData, np.ndarray, np.ndarray]:
  offset = int(training.groups.max()) + 1
  data = TrainingData(
    features=np.concatenate((training.features, validation.features)),
    labels=np.concatenate((training.labels, validation.labels)),
    weights=np.concatenate((training.weights, validation.weights)),
    groups=np.concatenate((training.groups, validation.groups + offset)),
    manual=np.concatenate((training.manual, validation.manual)),
  )
  split = len(training.features)
  return data, np.arange(split), np.arange(split, len(data.features))


def random_group_split(data: TrainingData, fraction: float, seed: int) -> tuple[np.ndarray, np.ndarray]:
  groups = np.unique(data.groups)
  rng = np.random.default_rng(seed)
  rng.shuffle(groups)
  count = min(len(groups) - 1, max(1, round(len(groups) * fraction)))
  validation = np.isin(data.groups, groups[:count])
  return np.flatnonzero(~validation), np.flatnonzero(validation)


def sigmoid(logits: np.ndarray) -> np.ndarray:
  return 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))


class MultiHeadMLP:
  def __init__(self, inputs: int, hidden1: int, hidden2: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    self.parameters = {
      "w1": (rng.standard_normal((inputs, hidden1)) * math.sqrt(2.0 / inputs)).astype(np.float32),
      "b1": np.zeros(hidden1, dtype=np.float32),
      "w2": (rng.standard_normal((hidden1, hidden2)) * math.sqrt(2.0 / hidden1)).astype(np.float32),
      "b2": np.zeros(hidden2, dtype=np.float32),
      "w3": (rng.standard_normal((hidden2, len(MODEL_HEADS))) * math.sqrt(1.0 / hidden2)).astype(np.float32),
      "b3": np.zeros(len(MODEL_HEADS), dtype=np.float32),
    }

  def logits(self, features: np.ndarray) -> np.ndarray:
    p = self.parameters
    hidden1 = np.maximum(features @ p["w1"] + p["b1"], 0.0)
    hidden2 = np.maximum(hidden1 @ p["w2"] + p["b2"], 0.0)
    return hidden2 @ p["w3"] + p["b3"]

  def gradients(
    self, features: np.ndarray, labels: np.ndarray, weights: np.ndarray,
    positive_weights: np.ndarray, l2: float,
  ) -> tuple[float, dict[str, np.ndarray]]:
    p = self.parameters
    z1 = features @ p["w1"] + p["b1"]
    hidden1 = np.maximum(z1, 0.0)
    z2 = hidden1 @ p["w2"] + p["b2"]
    hidden2 = np.maximum(z2, 0.0)
    logits = hidden2 @ p["w3"] + p["b3"]
    probabilities = sigmoid(logits)
    effective = weights * np.where(labels > 0.5, positive_weights[None, :], 1.0)
    denominator = np.maximum(effective.sum(axis=0), 1.0)
    loss = -float(np.mean(np.sum(effective * (
      labels * np.log(np.maximum(probabilities, 1e-7))
      + (1.0 - labels) * np.log(np.maximum(1.0 - probabilities, 1e-7))
    ), axis=0) / denominator))
    d_logits = effective * (probabilities - labels) / denominator[None, :] / len(MODEL_HEADS)
    gradients: dict[str, np.ndarray] = {}
    gradients["w3"] = hidden2.T @ d_logits + l2 * p["w3"]
    gradients["b3"] = d_logits.sum(axis=0)
    d_hidden2 = d_logits @ p["w3"].T
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
      m = self.m[name] / (1.0 - 0.9 ** self.step)
      v = self.v[name] / (1.0 - 0.999 ** self.step)
      parameter -= self.learning_rate * m / (np.sqrt(v) + 1e-8)


def weighted_losses(
  probabilities: np.ndarray, labels: np.ndarray, weights: np.ndarray, positive_weights: np.ndarray,
) -> np.ndarray:
  effective = weights * np.where(labels > 0.5, positive_weights[None, :], 1.0)
  numerator = -np.sum(effective * (
    labels * np.log(np.maximum(probabilities, 1e-7))
    + (1.0 - labels) * np.log(np.maximum(1.0 - probabilities, 1e-7))
  ), axis=0)
  return numerator / np.maximum(effective.sum(axis=0), 1.0)


def fit_calibration(logits: np.ndarray, labels: np.ndarray, weights: np.ndarray) -> tuple[float, float]:
  valid = weights > 0.0
  logits = logits[valid].astype(np.float64)
  labels = labels[valid].astype(np.float64)
  weights = weights[valid].astype(np.float64)
  if not len(logits) or np.all(labels == labels[0]):
    return 1.0, 0.0
  design = np.column_stack((logits, np.ones(len(logits))))
  parameters = np.asarray([1.0, 0.0])
  regularization = np.diag((1e-3, 1e-4))
  for _ in range(50):
    probability = sigmoid(design @ parameters)
    normalizer = max(float(weights.sum()), 1.0)
    gradient = design.T @ (weights * (probability - labels)) / normalizer + regularization @ parameters
    curvature = weights * probability * (1.0 - probability)
    hessian = design.T @ (design * curvature[:, None]) / normalizer + regularization
    try:
      step = np.linalg.solve(hessian, gradient)
    except np.linalg.LinAlgError:
      break
    parameters -= step
    if np.max(np.abs(step)) < 1e-7:
      break
  scale, bias = (float(value) for value in parameters)
  if scale <= 0.0 or not math.isfinite(scale + bias):
    return 1.0, 0.0
  return float(np.clip(scale, 0.05, 10.0)), float(np.clip(bias, -15.0, 15.0))


def group_metrics(
  probabilities: np.ndarray, labels: np.ndarray, weights: np.ndarray, groups: np.ndarray,
  threshold: float, max_outputs: int,
) -> dict[str, float | int]:
  tp = fp = fn = none_correct = none_groups = target_groups = 0
  for group in np.unique(groups):
    indices = np.flatnonzero((groups == group) & (weights > 0.0))
    if not len(indices):
      continue
    ranked = indices[np.argsort(probabilities[indices])[::-1]]
    selected = set(ranked[probabilities[ranked] >= threshold][:max_outputs].tolist())
    targets = set(indices[labels[indices] > 0.5].tolist())
    tp += len(selected & targets)
    fp += len(selected - targets)
    fn += len(targets - selected)
    if targets:
      target_groups += 1
    else:
      none_groups += 1
      none_correct += int(not selected)
  precision = tp / max(tp + fp, 1)
  recall = tp / max(tp + fn, 1)
  return {
    "precision": precision, "recall": recall,
    "f1": 2.0 * precision * recall / max(precision + recall, 1e-9),
    "tp": tp, "fp": fp, "fn": fn,
    "none_accuracy": none_correct / max(none_groups, 1),
    "none_groups": none_groups, "target_groups": target_groups,
  }


def choose_threshold(
  probabilities: np.ndarray, labels: np.ndarray, weights: np.ndarray, groups: np.ndarray,
  head_index: int,
) -> tuple[float, dict[str, float | int]]:
  max_outputs = 2
  beta = {"cutin": 0.5, "external": 0.7}.get(MODEL_HEADS[head_index], 1.0)
  best_threshold = 0.5
  best_score = -1.0
  best_metrics: dict[str, float | int] = {}
  threshold_values = np.unique(np.concatenate((
    np.linspace(0.20, 0.95, 76),
    np.asarray((0.96, 0.97, 0.98, 0.99, 0.995), dtype=np.float64),
  )))
  for threshold in threshold_values:
    metrics = group_metrics(probabilities, labels, weights, groups, float(threshold), max_outputs)
    precision = float(metrics["precision"])
    recall = float(metrics["recall"])
    score = (1.0 + beta * beta) * precision * recall / max(beta * beta * precision + recall, 1e-9)
    # Cut-in mistakes directly affect control, so equal F0.5 scores prefer the stricter threshold.
    if score > best_score + 1e-9 or (abs(score - best_score) <= 1e-9 and threshold > best_threshold):
      best_score = score
      best_threshold = float(threshold)
      best_metrics = metrics
  return best_threshold, best_metrics


def parse_hidden(value: str) -> tuple[int, int]:
  first, second = (int(part) for part in value.split(",", 1))
  if first <= 0 or second <= 0:
    raise argparse.ArgumentTypeError("hidden sizes must be positive")
  return first, second


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Train fused radar lead/cut-in/external MLP")
  parser.add_argument("datasets", nargs="+", type=Path)
  parser.add_argument("--validation-dataset", action="append", type=Path, default=[])
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--epochs", type=int, default=40)
  parser.add_argument("--batch-size", type=int, default=2048)
  parser.add_argument("--learning-rate", type=float, default=0.002)
  parser.add_argument("--hidden", type=parse_hidden, default=(64, 32))
  parser.add_argument("--validation", type=float, default=0.2)
  parser.add_argument("--l2", type=float, default=1e-5)
  parser.add_argument("--max-positive-weight", type=float, default=30.0)
  parser.add_argument("--patience", type=int, default=10)
  parser.add_argument("--primary-head", choices=("all", *MODEL_HEADS), default="all")
  parser.add_argument("--train-head", choices=("all", *MODEL_HEADS), default="all")
  parser.add_argument("--head-only", action="store_true", help="only update the selected output column")
  parser.add_argument(
    "--stationary-external", action="store_true",
    help="train the external head only on candidates with |vLead| < 1.8 m/s",
  )
  parser.add_argument("--init-model", type=Path, help="fine-tune an existing compatible multitask model")
  parser.add_argument("--cache", type=Path, help="reuse the parsed train/validation arrays")
  parser.add_argument("--seed", type=int, default=42)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if args.head_only and (args.train_head == "all" or args.init_model is None):
    raise SystemExit("--head-only requires --train-head and --init-model")
  if args.stationary_external and args.train_head != "external":
    raise SystemExit("--stationary-external requires --train-head external")
  sources = [*args.datasets, *args.validation_dataset]
  cached = load_cache(args.cache, sources) if args.cache is not None else None
  if cached is not None:
    data, train_indices, validation_indices = cached
    validation_kind = "cached held-out logs" if args.validation_dataset else "cached random frame groups"
  else:
    training = load_datasets(args.datasets)
    if args.validation_dataset:
      data, train_indices, validation_indices = combine(training, load_datasets(args.validation_dataset))
      validation_kind = "held-out logs"
    else:
      data = training
      train_indices, validation_indices = random_group_split(data, args.validation, args.seed)
      validation_kind = "random frame groups"
    if args.cache is not None:
      save_cache(args.cache, sources, data, train_indices, validation_indices)
  if args.stationary_external:
    v_lead_index = MODEL_FEATURE_NAMES.index("v_lead")
    external_index = MODEL_HEADS.index("external")
    focused = (np.abs(data.features[:, v_lead_index]) < 1.8) & (data.weights[:, external_index] > 0.0)
    train_mask = np.zeros(len(data.features), dtype=np.bool_)
    validation_mask = np.zeros(len(data.features), dtype=np.bool_)
    train_mask[train_indices] = True
    validation_mask[validation_indices] = True
    data = TrainingData(
      features=data.features[focused], labels=data.labels[focused], weights=data.weights[focused],
      groups=data.groups[focused], manual=data.manual[focused],
    )
    train_indices = np.flatnonzero(train_mask[focused])
    validation_indices = np.flatnonzero(validation_mask[focused])
    validation_kind += ", stationary external"
  initial_parameters: dict[str, np.ndarray] | None = None
  initial_thresholds: np.ndarray | None = None
  initial_calibration: np.ndarray | None = None
  auxiliary_model_arrays: dict[str, np.ndarray] = {}
  if args.init_model is not None:
    with np.load(args.init_model, allow_pickle=False) as initial:
      if tuple(str(value) for value in initial["feature_names"].tolist()) != MODEL_FEATURE_NAMES:
        raise SystemExit("--init-model feature schema mismatch")
      if tuple(str(value) for value in initial["head_names"].tolist()) != MODEL_HEADS:
        raise SystemExit("--init-model head schema mismatch")
      mean = initial["mean"].astype(np.float32)
      std = initial["std"].astype(np.float32)
      initial_parameters = {
        name: initial[name].astype(np.float32) for name in ("w1", "b1", "w2", "b2", "w3", "b3")
      }
      initial_thresholds = initial["thresholds"].astype(np.float32)
      initial_calibration = initial["calibration"].astype(np.float32)
      auxiliary_model_arrays = {
        name: initial[name].copy() for name in initial.files
        if name.startswith(("stationary_", "anticipatory_"))
      }
    hidden = (initial_parameters["w1"].shape[1], initial_parameters["w2"].shape[1])
  else:
    mean = data.features[train_indices].mean(axis=0, dtype=np.float64).astype(np.float32)
    std = data.features[train_indices].std(axis=0, dtype=np.float64).astype(np.float32)
    std[std < 1e-5] = 1.0
    hidden = args.hidden
  features = ((data.features - mean) / std).astype(np.float32)
  positive_weights = np.ones(len(MODEL_HEADS), dtype=np.float32)
  for head in range(len(MODEL_HEADS)):
    valid = data.weights[train_indices, head] > 0.0
    positives = np.sum(data.weights[train_indices, head][valid] * data.labels[train_indices, head][valid])
    negatives = np.sum(data.weights[train_indices, head][valid] * (1.0 - data.labels[train_indices, head][valid]))
    positive_weights[head] = min(args.max_positive_weight, max(1.0, negatives / max(positives, 1.0)))
  training_weights = data.weights.copy()
  if args.train_head != "all":
    selected_head = MODEL_HEADS.index(args.train_head)
    training_weights[:, np.arange(len(MODEL_HEADS)) != selected_head] = 0.0
  model = MultiHeadMLP(len(MODEL_FEATURE_NAMES), *hidden, args.seed)
  if initial_parameters is not None:
    model.parameters = {name: value.copy() for name, value in initial_parameters.items()}
  optimizer = Adam(model.parameters, args.learning_rate)
  rng = np.random.default_rng(args.seed)
  best_loss = math.inf
  best_epoch = 0
  best = {name: value.copy() for name, value in model.parameters.items()}
  print(
    f"rows {len(data.features)} train {len(train_indices)} validation {len(validation_indices)} [{validation_kind}] "
    "positive weights " + " ".join(
      f"{name} {positive_weights[index]:.2f}" for index, name in enumerate(MODEL_HEADS)
    ), flush=True,
  )
  for epoch in range(1, args.epochs + 1):
    shuffled = rng.permutation(train_indices)
    train_loss = 0.0
    batches = 0
    for start in range(0, len(shuffled), args.batch_size):
      batch = shuffled[start:start + args.batch_size]
      loss, gradients = model.gradients(
        features[batch], data.labels[batch], training_weights[batch], positive_weights, args.l2,
      )
      if args.head_only:
        for name in ("w1", "b1", "w2", "b2"):
          gradients[name].fill(0.0)
        frozen_heads = np.arange(len(MODEL_HEADS)) != MODEL_HEADS.index(args.train_head)
        gradients["w3"][:, frozen_heads] = 0.0
        gradients["b3"][frozen_heads] = 0.0
      optimizer.update(model.parameters, gradients)
      train_loss += loss
      batches += 1
    validation_prob = sigmoid(model.logits(features[validation_indices]))
    validation_losses = weighted_losses(
      validation_prob, data.labels[validation_indices], data.weights[validation_indices], positive_weights,
    )
    validation_loss = float(np.mean(validation_losses)) if args.primary_head == "all" else float(
      validation_losses[MODEL_HEADS.index(args.primary_head)]
    )
    if validation_loss < best_loss:
      best_loss = validation_loss
      best_epoch = epoch
      best = {name: value.copy() for name, value in model.parameters.items()}
    if epoch == 1 or epoch % 5 == 0:
      validation_text = " ".join(
        f"{name} {validation_losses[index]:.4f}" for index, name in enumerate(MODEL_HEADS)
      )
      print(f"epoch {epoch:3d} train {train_loss / max(batches, 1):.4f} val {validation_text}")
    if epoch - best_epoch >= args.patience:
      print(f"early stop {epoch}; best {best_epoch}")
      break
  model.parameters = best
  validation_logits = model.logits(features[validation_indices])
  validation_labels = data.labels[validation_indices]
  validation_weights = data.weights[validation_indices]
  calibration = np.zeros((len(MODEL_HEADS), 2), dtype=np.float32)
  calibrated = np.zeros_like(validation_logits)
  thresholds = np.zeros(len(MODEL_HEADS), dtype=np.float32)
  metrics: dict[str, Any] = {"validation_loss": best_loss, "best_epoch": best_epoch}
  for head_index, name in enumerate(MODEL_HEADS):
    preserve_head = args.init_model is not None and args.train_head != "all" and name != args.train_head
    if preserve_head:
      assert initial_calibration is not None and initial_thresholds is not None
      scale, bias = (float(value) for value in initial_calibration[head_index])
    else:
      scale, bias = fit_calibration(
        validation_logits[:, head_index], validation_labels[:, head_index], validation_weights[:, head_index],
      )
    calibration[head_index] = (scale, bias)
    calibrated[:, head_index] = sigmoid(validation_logits[:, head_index] * scale + bias)
    if preserve_head:
      threshold = float(initial_thresholds[head_index])
      head_metrics = group_metrics(
        calibrated[:, head_index], validation_labels[:, head_index], validation_weights[:, head_index],
        data.groups[validation_indices], threshold, 2,
      )
    else:
      threshold, head_metrics = choose_threshold(
        calibrated[:, head_index], validation_labels[:, head_index], validation_weights[:, head_index],
        data.groups[validation_indices], head_index,
      )
    thresholds[head_index] = threshold
    metrics[name] = {"threshold": threshold, **head_metrics}
    print(
      f"{name:5s} threshold {threshold:.2f} precision {float(head_metrics['precision']) * 100:.1f}% "
      f"recall {float(head_metrics['recall']) * 100:.1f}% fp {head_metrics['fp']} fn {head_metrics['fn']}",
    )
  output = args.output if args.output.suffix.lower() == ".npz" else args.output.with_suffix(".npz")
  output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(
    output,
    version=np.asarray([6], dtype=np.int32),
    feature_names=np.asarray(MODEL_FEATURE_NAMES),
    head_names=np.asarray(MODEL_HEADS),
    mean=mean, std=std, thresholds=thresholds, calibration=calibration,
    metrics=np.asarray(json.dumps(metrics, sort_keys=True)),
    **auxiliary_model_arrays,
    **model.parameters,
  )
  print(f"model written: {output} ({output.stat().st_size / 1024.0:.1f} KiB)")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
