"""Fit a small offline signal-color probe; never imported by vehicle control.

Input JSONL rows: group, split (train/evaluation), truth (red/green), features.
Features: current red/green/left/right, differences from causal red baseline,
differences from previous observation, baseline-present, x[-1]/100, v[0]/15,
v[-1]/15, their causal differences, model-present. See the investigation's
frozen data protocol. Manual-ROI scores do not measure autonomous detection.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sigmoid(x):
  return 1. / (1. + np.exp(-np.clip(x, -60., 60.)))


def fit(rows, dimensions, steps=2500):
  train = [r for r in rows if r['split'] == 'train']
  x = np.asarray([r['features'][:dimensions] for r in train], dtype=np.float64)
  y = np.asarray([r['truth'] == 'green' for r in train], dtype=np.float64)
  if not np.isfinite(x).all() or len(set(y)) != 2:
    raise ValueError('finite input and both training classes required')
  # Equal weight per encounter/color cell; neighboring video frames are not
  # independent evidence, and a long red wait must not dominate optimization.
  cells = [(r['group'], r['truth']) for r in train]
  weights = np.array([1. / cells.count(c) for c in cells])
  weights /= weights.sum()
  mean = (x * weights[:, None]).sum(axis=0)
  scale = np.maximum(np.sqrt(((x - mean)**2 * weights[:, None]).sum(axis=0)), .1)
  z = np.column_stack([(x - mean) / scale, np.ones(len(x))])
  coefficient = np.zeros(dimensions + 1)
  for _ in range(steps):
    gradient = z.T @ (weights * (sigmoid(z @ coefficient) - y))
    gradient[:-1] += .01 * coefficient[:-1]
    coefficient -= .08 * gradient
  loss = -(weights * (y * np.log(sigmoid(z @ coefficient)) + (1-y) * np.log(sigmoid(-z @ coefficient)))).sum()
  return dict(schema=1, purpose='offline_manual_roi_color_probe_only', dimensions=dimensions,
              mean=mean.tolist(), scale=scale.tolist(), coefficient=coefficient.tolist(),
              training_groups=sorted({r['group'] for r in train}), training_rows=len(train),
              steps=steps, learning_rate=.08, l2=.01, cross_entropy=float(loss))


def predict(model, features):
  x = np.asarray(features[:model['dimensions']], dtype=np.float64)
  if x.shape != (model['dimensions'],) or not np.isfinite(x).all():
    return None
  z = (x - model['mean']) / model['scale']
  return float(sigmoid(z @ np.asarray(model['coefficient'][:-1]) + model['coefficient'][-1]))


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('dataset', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  rows = [json.loads(line) for line in args.dataset.read_text(encoding='utf-8').splitlines()]
  train_groups = {r['group'] for r in rows if r['split'] == 'train'}
  if train_groups & {r['group'] for r in rows if r['split'] != 'train'}:
    raise ValueError('encounter crosses fit/evaluation boundary')
  args.output.mkdir(parents=True, exist_ok=True)
  summary = {}
  for name, dimensions in [('visual_current', 4), ('visual_temporal', 13), ('visual_model_temporal', 20)]:
    model = fit(rows, dimensions)
    model['dataset_sha256'] = hashlib.sha256(args.dataset.read_bytes()).hexdigest()
    dest = args.output / f'{name}.json'
    dest.write_text(json.dumps(model, indent=2), encoding='utf-8')
    records = []
    for row in rows:
      p = predict(model, row['features'])
      prediction = 'unknown' if p is None else 'green' if p >= .9 else 'red' if p <= .1 else 'unknown'
      records.append({k: row[k] for k in ('group', 'split', 'segment', 'frame', 'timestamp', 'truth')} |
                     dict(p_green=p, prediction=prediction))
    stats = {}
    for group in sorted({r['group'] for r in records}):
      rr = [r for r in records if r['group'] == group]
      stats[group] = dict(split=rr[0]['split'], n=len(rr),
                         confusion={truth: {pred: sum(r['truth'] == truth and r['prediction'] == pred for r in rr)
                                            for pred in ('red', 'green', 'unknown')} for truth in ('red', 'green')})
    summary[name] = dict(model_sha256=hashlib.sha256(dest.read_bytes()).hexdigest(), groups=stats,
                         training_cross_entropy=model['cross_entropy'])
    (args.output / f'{name}_predictions.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records), encoding='utf-8')
  (args.output / 'evaluation.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
  print(json.dumps(summary, indent=2))


if __name__ == '__main__':
  main()
