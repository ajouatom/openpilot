#!/usr/bin/env python3
"""Train the geometry-gated anticipatory cut-in auxiliary classifier."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

from openpilot.selfdrive.carrot.radar_lead_model import (
  ANTICIPATORY_FEATURE_NAMES,
  anticipatory_eligibility,
  anticipatory_feature_matrix,
)
from openpilot.selfdrive.carrot.radar_lead_multitask_train import (
  Adam,
  choose_threshold,
  fit_calibration,
  group_metrics,
  load_datasets,
  sigmoid,
)


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(description="Train an auxiliary early cut-in classifier")
  parser.add_argument("datasets", nargs="+", type=Path)
  parser.add_argument("--validation-dataset", action="append", type=Path, default=[])
  parser.add_argument("--base-model", required=True, type=Path)
  parser.add_argument("--output", required=True, type=Path)
  parser.add_argument("--epochs", type=int, default=60)
  parser.add_argument("--batch-size", type=int, default=4096)
  parser.add_argument("--learning-rate", type=float, default=0.002)
  parser.add_argument("--l2", type=float, default=1e-4)
  parser.add_argument("--max-positive-weight", type=float, default=10.0)
  parser.add_argument("--patience", type=int, default=12)
  parser.add_argument("--seed", type=int, default=42)
  return parser.parse_args()


def main() -> int:
  args = parse_args()
  if not args.validation_dataset:
    raise SystemExit("at least one --validation-dataset is required")
  training = load_datasets(args.datasets)
  validation = load_datasets(args.validation_dataset)
  train_mask = anticipatory_eligibility(training.features, np)
  validation_mask = anticipatory_eligibility(validation.features, np)
  train_x = anticipatory_feature_matrix(training.features[train_mask], np)
  validation_x = anticipatory_feature_matrix(validation.features[validation_mask], np)
  train_y = training.labels[train_mask, 1]
  validation_y = validation.labels[validation_mask, 1]
  train_weights = training.weights[train_mask, 1]
  validation_weights = validation.weights[validation_mask, 1]
  validation_groups = validation.groups[validation_mask]
  if np.sum(train_y > 0.5) < 10 or np.sum(validation_y > 0.5) < 5:
    raise RuntimeError("not enough eligible anticipatory cut-in labels")

  mean = train_x.mean(axis=0, dtype=np.float64).astype(np.float32)
  std = train_x.std(axis=0, dtype=np.float64).astype(np.float32)
  std[std < 1e-5] = 1.0
  train_x = ((train_x - mean) / std).astype(np.float32)
  validation_x = ((validation_x - mean) / std).astype(np.float32)
  positive_mass = float(np.sum(train_weights * train_y))
  negative_mass = float(np.sum(train_weights * (1.0 - train_y)))
  positive_weight = min(args.max_positive_weight, max(1.0, negative_mass / max(positive_mass, 1.0)))

  rng = np.random.default_rng(args.seed)
  parameters = {
    "w": np.zeros(train_x.shape[1], dtype=np.float32),
    "b": np.zeros(1, dtype=np.float32),
  }
  optimizer = Adam(parameters, args.learning_rate)
  best_loss = math.inf
  best_epoch = 0
  best = {name: value.copy() for name, value in parameters.items()}
  print(
    f"eligible rows train {len(train_x)} validation {len(validation_x)} "
    f"positives {int(np.sum(train_y > 0.5))}/{int(np.sum(validation_y > 0.5))} "
    f"positive weight {positive_weight:.2f}", flush=True,
  )
  for epoch in range(1, args.epochs + 1):
    total_loss = 0.0
    batches = 0
    shuffled = rng.permutation(len(train_x))
    for start in range(0, len(train_x), args.batch_size):
      batch = shuffled[start:start + args.batch_size]
      labels = train_y[batch]
      weights = train_weights[batch]
      effective = weights * np.where(labels > 0.5, positive_weight, 1.0)
      denominator = max(float(effective.sum()), 1.0)
      probabilities = sigmoid(train_x[batch] @ parameters["w"] + parameters["b"][0])
      total_loss -= float(np.sum(effective * (
        labels * np.log(np.maximum(probabilities, 1e-7))
        + (1.0 - labels) * np.log(np.maximum(1.0 - probabilities, 1e-7))
      )) / denominator)
      d_logits = effective * (probabilities - labels) / denominator
      gradients = {
        "w": train_x[batch].T @ d_logits + args.l2 * parameters["w"],
        "b": np.asarray([d_logits.sum()], dtype=np.float32),
      }
      optimizer.update(parameters, gradients)
      batches += 1
    validation_prob = sigmoid(validation_x @ parameters["w"] + parameters["b"][0])
    effective = validation_weights * np.where(validation_y > 0.5, positive_weight, 1.0)
    validation_loss = -float(np.sum(effective * (
      validation_y * np.log(np.maximum(validation_prob, 1e-7))
      + (1.0 - validation_y) * np.log(np.maximum(1.0 - validation_prob, 1e-7))
    )) / max(float(effective.sum()), 1.0))
    if validation_loss < best_loss:
      best_loss = validation_loss
      best_epoch = epoch
      best = {name: value.copy() for name, value in parameters.items()}
    if epoch == 1 or epoch % 5 == 0:
      print(f"epoch {epoch:3d} train {total_loss / max(batches, 1):.4f} validation {validation_loss:.4f}")
    if epoch - best_epoch >= args.patience:
      print(f"early stop {epoch}; best {best_epoch}")
      break
  parameters = best
  raw_validation_logits = validation_x @ parameters["w"] + parameters["b"][0]
  calibration = fit_calibration(raw_validation_logits, validation_y, validation_weights)
  probabilities = sigmoid(raw_validation_logits * calibration[0] + calibration[1])
  threshold, metrics = choose_threshold(
    probabilities, validation_y, validation_weights, validation_groups, 1,
  )
  print(
    f"threshold {threshold:.3f} precision {float(metrics['precision']) * 100:.1f}% "
    f"recall {float(metrics['recall']) * 100:.1f}% fp {metrics['fp']} fn {metrics['fn']}",
  )

  with np.load(args.base_model, allow_pickle=False) as base:
    arrays = {name: base[name].copy() for name in base.files if not name.startswith("anticipatory_")}
  arrays["version"] = np.asarray([13], dtype=np.int32)
  output = args.output if args.output.suffix.lower() == ".npz" else args.output.with_suffix(".npz")
  output.parent.mkdir(parents=True, exist_ok=True)
  arrays.update({
    "anticipatory_feature_names": np.asarray(ANTICIPATORY_FEATURE_NAMES),
    "anticipatory_mean": mean,
    "anticipatory_std": std,
    "anticipatory_w": parameters["w"],
    "anticipatory_b": parameters["b"],
    "anticipatory_calibration": np.asarray(calibration, dtype=np.float32),
    "anticipatory_threshold": np.asarray([threshold], dtype=np.float32),
    "anticipatory_metrics": np.asarray(json.dumps({
      "validation_loss": best_loss, "best_epoch": best_epoch, **metrics,
    }, sort_keys=True)),
  })
  np.savez_compressed(output, **arrays)
  print(f"model written: {output} ({output.stat().st_size / 1024.0:.1f} KiB)")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
