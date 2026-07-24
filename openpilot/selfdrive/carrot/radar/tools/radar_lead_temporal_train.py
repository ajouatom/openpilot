#!/usr/bin/env python3
"""Train a dedicated multi-scale cut-in trajectory classifier."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar.radar_lead_model import (
  MODEL_HEADS,
  TEMPORAL_CUTIN_FEATURE_NAMES,
  temporal_cutin_eligibility,
  temporal_cutin_feature_matrix,
)
from openpilot.selfdrive.carrot.radar.tools.radar_lead_multitask_train import (
  Adam,
  TrainingData,
  fit_calibration,
  group_metrics,
  load_datasets,
  sigmoid,
)


def _discover(roots: list[Path]) -> list[Path]:
  return sorted({
    path.resolve() for root in roots
    for path in ([root] if root.is_file() else root.glob("*.fused.csv.gz"))
    if path.is_file()
  })


def _split_sources(
  sources: list[Path], validation_fraction: float, validation_substrings: tuple[str, ...], seed: int,
) -> tuple[list[Path], list[Path]]:
  forced = [source for source in sources if any(value.lower() in source.name.lower() for value in validation_substrings)]
  remaining = [source for source in sources if source not in forced]
  count = max(1, round(len(sources) * validation_fraction) - len(forced))
  rng = np.random.default_rng(seed)
  selected = set(rng.choice(len(remaining), min(count, len(remaining)), replace=False).tolist()) if remaining else set()
  validation = [*forced, *(source for index, source in enumerate(remaining) if index in selected)]
  training = [source for index, source in enumerate(remaining) if index not in selected]
  if not training or not validation:
    raise RuntimeError("source-level train/validation split is empty")
  return training, validation


def _prepare(data: TrainingData) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
  head = MODEL_HEADS.index("cutin")
  eligible = temporal_cutin_eligibility(data.features, np) & (data.weights[:, head] > 0.0)
  return (
    temporal_cutin_feature_matrix(data.features[eligible], np),
    data.labels[eligible, head], data.weights[eligible, head], data.groups[eligible], data.manual[eligible, head],
  )


def _sample_training(
  labels: np.ndarray, manual: np.ndarray, negative_ratio: float, seed: int,
) -> np.ndarray:
  positive = labels > 0.5
  required = positive | manual
  negative = np.flatnonzero(~required)
  limit = min(len(negative), max(1, round(max(int(positive.sum()), 1) * negative_ratio)))
  rng = np.random.default_rng(seed)
  selected_negative = rng.choice(negative, limit, replace=False) if limit < len(negative) else negative
  return np.sort(np.concatenate((np.flatnonzero(required), selected_negative)))


def _random_group_indices(
  groups: np.ndarray, labels: np.ndarray, validation_fraction: float, seed: int,
) -> tuple[np.ndarray, np.ndarray]:
  unique_groups = np.unique(groups)
  validation_count = min(len(unique_groups) - 1, max(1, round(len(unique_groups) * validation_fraction)))
  for attempt in range(100):
    rng = np.random.default_rng(seed + attempt)
    validation_groups = rng.choice(unique_groups, validation_count, replace=False)
    validation_mask = np.isin(groups, validation_groups)
    train_indices = np.flatnonzero(~validation_mask)
    validation_indices = np.flatnonzero(validation_mask)
    if labels[train_indices].sum() > 0.0 and labels[validation_indices].sum() > 0.0:
      return train_indices, validation_indices
  raise RuntimeError("random group split could not place positives in both partitions")


class BinaryMLP:
  def __init__(self, inputs: int, hidden1: int, hidden2: int, seed: int) -> None:
    rng = np.random.default_rng(seed)
    self.parameters = {
      "w1": (rng.standard_normal((inputs, hidden1)) * math.sqrt(2.0 / inputs)).astype(np.float32),
      "b1": np.zeros(hidden1, dtype=np.float32),
      "w2": (rng.standard_normal((hidden1, hidden2)) * math.sqrt(2.0 / hidden1)).astype(np.float32),
      "b2": np.zeros(hidden2, dtype=np.float32),
      "w3": (rng.standard_normal(hidden2) * math.sqrt(1.0 / hidden2)).astype(np.float32),
      "b3": np.zeros(1, dtype=np.float32),
    }

  def logits(self, features: np.ndarray) -> np.ndarray:
    p = self.parameters
    hidden1 = np.maximum(features @ p["w1"] + p["b1"], 0.0)
    hidden2 = np.maximum(hidden1 @ p["w2"] + p["b2"], 0.0)
    return hidden2 @ p["w3"] + p["b3"][0]

  def gradients(
    self, features: np.ndarray, labels: np.ndarray, weights: np.ndarray, positive_weight: float, l2: float,
  ) -> tuple[float, dict[str, np.ndarray]]:
    p = self.parameters
    z1 = features @ p["w1"] + p["b1"]
    hidden1 = np.maximum(z1, 0.0)
    z2 = hidden1 @ p["w2"] + p["b2"]
    hidden2 = np.maximum(z2, 0.0)
    probabilities = sigmoid(hidden2 @ p["w3"] + p["b3"][0])
    effective = weights * np.where(labels > 0.5, positive_weight, 1.0)
    denominator = max(float(effective.sum()), 1.0)
    loss = -float(np.sum(effective * (
      labels * np.log(np.maximum(probabilities, 1e-7))
      + (1.0 - labels) * np.log(np.maximum(1.0 - probabilities, 1e-7))
    )) / denominator)
    d_logits = effective * (probabilities - labels) / denominator
    gradients: dict[str, np.ndarray] = {}
    gradients["w3"] = hidden2.T @ d_logits + l2 * p["w3"]
    gradients["b3"] = np.asarray([d_logits.sum()], dtype=np.float32)
    d_z2 = d_logits[:, None] * p["w3"][None, :] * (z2 > 0.0)
    gradients["w2"] = hidden1.T @ d_z2 + l2 * p["w2"]
    gradients["b2"] = d_z2.sum(axis=0)
    d_z1 = (d_z2 @ p["w2"].T) * (z1 > 0.0)
    gradients["w1"] = features.T @ d_z1 + l2 * p["w1"]
    gradients["b1"] = d_z1.sum(axis=0)
    return loss, gradients


def _choose_threshold(
  probabilities: np.ndarray, labels: np.ndarray, weights: np.ndarray, groups: np.ndarray,
) -> tuple[float, dict[str, float | int]]:
  best_threshold = 0.5
  best_score = -1.0
  best_metrics: dict[str, float | int] = {}
  for threshold in np.unique(np.concatenate((np.linspace(0.50, 0.95, 46), np.asarray((0.96, 0.97, 0.98, 0.99))))):
    metrics = group_metrics(probabilities, labels, weights, groups, float(threshold), 2)
    precision = float(metrics["precision"])
    recall = float(metrics["recall"])
    beta = 0.5
    score = (1.0 + beta * beta) * precision * recall / max(beta * beta * precision + recall, 1e-9)
    if score > best_score + 1e-9 or (abs(score - best_score) <= 1e-9 and threshold > best_threshold):
      best_score, best_threshold, best_metrics = score, float(threshold), metrics
  return best_threshold, best_metrics


def _parse_hidden(value: str) -> tuple[int, int]:
  first, second = (int(part) for part in value.split(",", 1))
  if first <= 0 or second <= 0:
    raise argparse.ArgumentTypeError("hidden sizes must be positive")
  return first, second


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Train a dedicated temporal cut-in MLP")
  parser.add_argument("datasets", nargs="+", type=Path)
  parser.add_argument("--validation-dataset", action="append", type=Path, default=[])
  parser.add_argument(
    "--review-dataset", action="append", type=Path, default=[],
    help="strong reviewed examples added only to training, outside the corpus split",
  )
  parser.add_argument(
    "--review-weight-scale", type=float, default=0.10,
    help="weight applied to the separately supplied reviewed examples",
  )
  parser.add_argument("--base-model", required=True, type=Path)
  parser.add_argument(
    "--init-temporal-model", type=Path,
    help="initialize the temporal head and normalization from an existing compatible artifact",
  )
  parser.add_argument(
    "--distill-init", action="store_true",
    help="preserve the initialized temporal head on non-manual corpus rows",
  )
  parser.add_argument(
    "--distill-weight", type=float, default=0.25,
    help="training weight for initialized-model soft targets on non-manual rows",
  )
  parser.add_argument(
    "--distill-all-corpus", action="store_true",
    help="distill every corpus row; separately supplied review datasets remain hard labels",
  )
  parser.add_argument(
    "--preserve-init-calibration", action="store_true",
    help="retain initialized temporal calibration and threshold instead of fitting weak validation labels",
  )
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--validation-fraction", type=float, default=0.20)
  parser.add_argument("--validation-substring", action="append", default=[])
  parser.add_argument("--split-mode", choices=("source", "random-group"), default="source")
  parser.add_argument("--negative-ratio", type=float, default=20.0)
  parser.add_argument(
    "--manual-weight-scale", type=float, default=1.0,
    help="scale video/manual-label weights after loading the source datasets",
  )
  parser.add_argument("--hidden", type=_parse_hidden, default=(48, 24))
  parser.add_argument("--epochs", type=int, default=60)
  parser.add_argument("--batch-size", type=int, default=4096)
  parser.add_argument("--learning-rate", type=float, default=0.0015)
  parser.add_argument("--l2", type=float, default=2e-5)
  parser.add_argument("--max-positive-weight", type=float, default=5.0)
  parser.add_argument("--patience", type=int, default=12)
  parser.add_argument("--model-version", type=int, default=0, help="artifact version; defaults to the base model")
  parser.add_argument("--sensor-mode", choices=("fused", "front", "corner"), default="fused")
  parser.add_argument(
    "--temporal-only", action="store_true",
    help="make the dedicated temporal head the sole production cut-in classifier",
  )
  parser.add_argument("--seed", type=int, default=42)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if args.manual_weight_scale <= 0.0:
    raise SystemExit("--manual-weight-scale must be positive")
  if args.review_weight_scale <= 0.0:
    raise SystemExit("--review-weight-scale must be positive")
  if args.distill_weight <= 0.0:
    raise SystemExit("--distill-weight must be positive")
  if args.distill_init and args.init_temporal_model is None:
    raise SystemExit("--distill-init requires --init-temporal-model")
  if args.distill_all_corpus and not args.distill_init:
    raise SystemExit("--distill-all-corpus requires --distill-init")
  if args.preserve_init_calibration and args.init_temporal_model is None:
    raise SystemExit("--preserve-init-calibration requires --init-temporal-model")
  sources = _discover(args.datasets)
  if len(sources) < 5:
    raise SystemExit("at least five fused datasets are required")
  validation_sources = _discover(args.validation_dataset)
  if validation_sources:
    training_sources = sources
    print(f"sources train {len(training_sources)} validation {len(validation_sources)} (explicit)", flush=True)
    train_features, train_labels, train_weights, train_groups, train_manual = _prepare(load_datasets(training_sources))
    validation_features, validation_labels, validation_weights, validation_groups, _ = _prepare(
      load_datasets(validation_sources),
    )
  elif args.split_mode == "source":
    training_sources, validation_sources = _split_sources(
      sources, args.validation_fraction, tuple(args.validation_substring), args.seed,
    )
    print(f"sources train {len(training_sources)} validation {len(validation_sources)}", flush=True)
    train_features, train_labels, train_weights, train_groups, train_manual = _prepare(load_datasets(training_sources))
    validation_features, validation_labels, validation_weights, validation_groups, _ = _prepare(
      load_datasets(validation_sources),
    )
  else:
    all_features, all_labels, all_weights, all_groups, all_manual = _prepare(load_datasets(sources))
    train_indices, validation_indices = _random_group_indices(
      all_groups, all_labels, args.validation_fraction, args.seed,
    )
    train_features, validation_features = all_features[train_indices], all_features[validation_indices]
    train_labels, validation_labels = all_labels[train_indices], all_labels[validation_indices]
    train_weights, validation_weights = all_weights[train_indices], all_weights[validation_indices]
    train_groups, validation_groups = all_groups[train_indices], all_groups[validation_indices]
    train_manual = all_manual[train_indices]
    training_sources = validation_sources = sources
    print(
      f"random groups train {len(np.unique(train_groups))} validation {len(np.unique(validation_groups))} "
      f"from {len(sources)} sources", flush=True,
    )
  initialized_parameters: dict[str, np.ndarray] | None = None
  initialized_hidden = args.hidden
  initialized_calibration: np.ndarray | None = None
  initialized_threshold: float | None = None
  if args.init_temporal_model is not None:
    with np.load(args.init_temporal_model, allow_pickle=False) as initialized:
      names = tuple(str(value) for value in initialized["temporal_cutin_feature_names"].tolist())
      if names != TEMPORAL_CUTIN_FEATURE_NAMES:
        raise SystemExit("--init-temporal-model feature schema mismatch")
      mean = initialized["temporal_cutin_mean"].astype(np.float32)
      std = initialized["temporal_cutin_std"].astype(np.float32)
      initialized_parameters = {
        name: initialized[f"temporal_cutin_{name}"].astype(np.float32)
        for name in ("w1", "b1", "w2", "b2", "w3", "b3")
      }
      initialized_hidden = (
        int(initialized_parameters["w1"].shape[1]),
        int(initialized_parameters["w2"].shape[1]),
      )
      initialized_calibration = initialized["temporal_cutin_calibration"].astype(np.float32)
      initialized_threshold = float(initialized["temporal_cutin_threshold"][0])
  else:
    mean = train_features.mean(axis=0, dtype=np.float64).astype(np.float32)
    std = train_features.std(axis=0, dtype=np.float64).astype(np.float32)
    std[std < 1e-5] = 1.0

  distilled_rows = 0
  if args.distill_init:
    assert initialized_parameters is not None
    normalized = ((train_features - mean) / std).astype(np.float32)
    hidden1 = np.maximum(
      normalized @ initialized_parameters["w1"] + initialized_parameters["b1"], 0.0,
    )
    hidden2 = np.maximum(
      hidden1 @ initialized_parameters["w2"] + initialized_parameters["b2"], 0.0,
    )
    teacher_probabilities = sigmoid(
      hidden2 @ initialized_parameters["w3"] + initialized_parameters["b3"][0],
    )
    distill = np.ones(len(train_manual), dtype=np.bool_) if args.distill_all_corpus else ~train_manual
    train_labels = train_labels.copy()
    train_weights = train_weights.copy()
    train_manual = train_manual.copy()
    train_labels[distill] = teacher_probabilities[distill]
    train_weights[distill] = args.distill_weight
    train_manual[distill] = False
    distilled_rows = int(np.sum(distill))

  selected = _sample_training(train_labels, train_manual, args.negative_ratio, args.seed)
  train_features = train_features[selected]
  train_labels = train_labels[selected]
  train_weights = train_weights[selected]
  train_groups = train_groups[selected]
  train_manual = train_manual[selected]
  train_weights[train_manual] *= args.manual_weight_scale

  review_sources = _discover(args.review_dataset)
  review_rows = review_positives = 0
  if review_sources:
    review_features, review_labels, review_weights, review_groups, _ = _prepare(load_datasets(review_sources))
    review_rows = len(review_features)
    review_positives = int(np.sum(review_labels > 0.5))
    group_offset = int(train_groups.max()) + 1 if len(train_groups) else 0
    train_features = np.concatenate((train_features, review_features))
    train_labels = np.concatenate((train_labels, review_labels))
    train_weights = np.concatenate((train_weights, review_weights * args.review_weight_scale))
    train_groups = np.concatenate((train_groups, review_groups + group_offset))
    train_manual = np.concatenate((train_manual, np.ones(len(review_features), dtype=np.bool_)))

  train_features = ((train_features - mean) / std).astype(np.float32)
  validation_features = ((validation_features - mean) / std).astype(np.float32)
  positive_mass = float(np.sum(train_weights * train_labels))
  negative_mass = float(np.sum(train_weights * (1.0 - train_labels)))
  positive_weight = min(args.max_positive_weight, max(1.0, negative_mass / max(positive_mass, 1.0)))
  print(
    f"eligible train {len(train_features)} validation {len(validation_features)} "
    f"positives {int(np.sum(train_labels > 0.5))}/{int(np.sum(validation_labels > 0.5))} "
    f"manual {int(np.sum(train_manual))} manual-scale {args.manual_weight_scale:.2f} "
    f"review {review_rows}/{review_positives} review-scale {args.review_weight_scale:.3f} "
    f"distilled {distilled_rows} distill-weight {args.distill_weight:.3f} "
    f"positive-weight {positive_weight:.2f}", flush=True,
  )

  model = BinaryMLP(len(TEMPORAL_CUTIN_FEATURE_NAMES), *initialized_hidden, args.seed)
  if initialized_parameters is not None:
    if any(model.parameters[name].shape != value.shape for name, value in initialized_parameters.items()):
      raise SystemExit("--init-temporal-model network shape mismatch")
    model.parameters = {name: value.copy() for name, value in initialized_parameters.items()}
    print(
      f"initialized temporal head from {args.init_temporal_model} hidden={initialized_hidden}",
      flush=True,
    )
  optimizer = Adam(model.parameters, args.learning_rate)
  rng = np.random.default_rng(args.seed)
  best_loss = math.inf
  best_epoch = 0
  best = {name: value.copy() for name, value in model.parameters.items()}
  for epoch in range(1, args.epochs + 1):
    total_loss = 0.0
    batches = 0
    shuffled = rng.permutation(len(train_features))
    for start in range(0, len(shuffled), args.batch_size):
      batch = shuffled[start:start + args.batch_size]
      loss, gradients = model.gradients(
        train_features[batch], train_labels[batch], train_weights[batch], positive_weight, args.l2,
      )
      optimizer.update(model.parameters, gradients)
      total_loss += loss
      batches += 1
    validation_probabilities = sigmoid(model.logits(validation_features))
    effective = validation_weights * np.where(validation_labels > 0.5, positive_weight, 1.0)
    validation_loss = -float(np.sum(effective * (
      validation_labels * np.log(np.maximum(validation_probabilities, 1e-7))
      + (1.0 - validation_labels) * np.log(np.maximum(1.0 - validation_probabilities, 1e-7))
    )) / max(float(effective.sum()), 1.0))
    if validation_loss < best_loss:
      best_loss = validation_loss
      best_epoch = epoch
      best = {name: value.copy() for name, value in model.parameters.items()}
    if epoch == 1 or epoch % 5 == 0:
      print(f"epoch {epoch:3d} train {total_loss / max(batches, 1):.4f} validation {validation_loss:.4f}")
    if epoch - best_epoch >= args.patience:
      print(f"early stop {epoch}; best {best_epoch}")
      break
  model.parameters = best
  raw_logits = model.logits(validation_features)
  if args.preserve_init_calibration:
    assert initialized_calibration is not None and initialized_threshold is not None
    calibration = initialized_calibration
    threshold = initialized_threshold
    probabilities = sigmoid(raw_logits * calibration[0] + calibration[1])
    metrics = group_metrics(
      probabilities, validation_labels, validation_weights, validation_groups, threshold, 2,
    )
  else:
    calibration = fit_calibration(raw_logits, validation_labels, validation_weights)
    probabilities = sigmoid(raw_logits * calibration[0] + calibration[1])
    threshold, metrics = _choose_threshold(
      probabilities, validation_labels, validation_weights, validation_groups,
    )
  print(
    f"threshold {threshold:.3f} precision {float(metrics['precision']) * 100:.1f}% "
    f"recall {float(metrics['recall']) * 100:.1f}% f1 {float(metrics['f1']) * 100:.1f}% "
    f"fp {metrics['fp']} fn {metrics['fn']}", flush=True,
  )

  with np.load(args.base_model, allow_pickle=False) as base:
    arrays = {
      name: base[name].copy() for name in base.files
      if not name.startswith("temporal_cutin_")
    }
    base_version = int(base["version"][0])
  arrays["version"] = np.asarray([args.model_version or base_version], dtype=np.int32)
  arrays["sensor_mode"] = np.asarray(args.sensor_mode)
  arrays["cutin_temporal_only"] = np.asarray([args.temporal_only], dtype=np.bool_)
  arrays.update({
    "temporal_cutin_feature_names": np.asarray(TEMPORAL_CUTIN_FEATURE_NAMES),
    "temporal_cutin_mean": mean,
    "temporal_cutin_std": std,
    **{f"temporal_cutin_{name}": value for name, value in model.parameters.items()},
    "temporal_cutin_calibration": np.asarray(calibration, dtype=np.float32),
    "temporal_cutin_threshold": np.asarray([threshold], dtype=np.float32),
    "temporal_cutin_metrics": np.asarray(json.dumps({
      "validation_loss": best_loss, "best_epoch": best_epoch,
      "train_sources": len(training_sources), "validation_sources": len(validation_sources),
      "review_sources": len(review_sources), "review_rows": review_rows,
      "review_weight_scale": args.review_weight_scale,
      "init_temporal_model": str(args.init_temporal_model or ""),
      "distill_init": args.distill_init, "distilled_rows": distilled_rows,
      "distill_weight": args.distill_weight,
      "distill_all_corpus": args.distill_all_corpus,
      "preserve_init_calibration": args.preserve_init_calibration,
      **metrics,
    }, sort_keys=True)),
  })
  output = args.output if args.output.suffix.lower() == ".npz" else args.output.with_suffix(".npz")
  output.parent.mkdir(parents=True, exist_ok=True)
  np.savez_compressed(output, **arrays)
  print(f"model written: {output} ({output.stat().st_size / 1024.0:.1f} KiB)")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
