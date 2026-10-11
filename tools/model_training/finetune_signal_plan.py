#!/usr/bin/env python3
"""Offline supervised fine-tuning of an existing driving ONNX plan branch.

No camera/controller/device writes. Inputs are trusted local replay archives.
The auxiliary color head is training-only; exported ONNX structure is unchanged.
"""
import argparse
import copy
import hashlib
import json
import pickle
from pathlib import Path

import numpy as np
import onnx
from onnx import numpy_helper
import torch
from torch import nn
from torch.nn import functional as F

T = (np.arange(33) / 32) ** 2 * 10
PREFIX = 'policy/policy_model.temporal_hydra.'
BLOCKS = ['in_layer.plan', 'res_layer.plan.0', 'res_layer.plan.2', 'final_layer.plan']
LONG = np.array([15 * i + c for i in range(33) if T[i] >= .5 for c in (0, 3, 6)])
ALL_LONG = np.array([15 * i + c for i in range(33) for c in (0, 3, 6)])


class Plan(nn.Module):
  def __init__(self, arrays):
    super().__init__()
    self.layers = nn.ModuleList()
    for name in BLOCKS:
      w, b = [arrays[PREFIX + name + '.' + suffix].astype(np.float32) for suffix in ('weight', 'bias')]
      layer = nn.Linear(w.shape[1], w.shape[0])
      layer.weight.data.copy_(torch.from_numpy(w))
      layer.bias.data.copy_(torch.from_numpy(b))
      self.layers.append(layer)
    self.register_buffer('scale', torch.from_numpy(arrays[PREFIX + 'scale_layer.plan.scale'].astype(np.float32)))
    self.color = nn.Linear(256, 2)

  def forward(self, x):
    a = F.relu(self.layers[0](x))
    h = F.relu(a + self.layers[2](F.relu(self.layers[1](a))))
    return self.layers[3](h) * self.scale, self.color(h)

  def freeze(self, mode):
    for layer in self.layers[:3]:
      layer.requires_grad_(mode == 'plan')
    self.color.requires_grad_(mode == 'plan')
    mask = torch.zeros(990)
    mask[LONG] = 1
    self.layers[3].weight.register_hook(lambda grad: grad * mask[:, None])
    self.layers[3].bias.register_hook(lambda grad: grad * mask)


def read_json(path):
  return json.loads(Path(path).read_text(encoding='utf-8-sig'))


class Dataset:
  def __init__(self, root, protocol, base):
    self.root, self.protocol = root, protocol
    self.entries = read_json(root / 'manifest.json')
    self.cache, self.drives = {}, {}
    self.groups = []
    self.normals = {s: [] for s in ('train', 'development', 'evaluation')}
    self.base = base
    for entry in self.entries:
      seg = entry['segment']
      z = np.load(root / 'features' / f'{seg}.npz')
      self.cache[seg] = {k: z[k] for k in z.files}
      with (Path(entry['folder']) / 'decoded.pkl').open('rb') as stream:
        drive = pickle.load(stream)
      self.drives[seg] = {
        'carState': [(t, valid, {'vEgo': v['vEgo'], 'aEgo': v['aEgo']}) for t, valid, v in drive['carState']],
        'roadEncodeIdx': drive['roadEncodeIdx'][:1],
      }
      del drive
    for entry in self.entries:
      seg, split = entry['segment'], entry['split']
      z = self.cache[seg]
      rows = z['rows']
      labels = [a for a in protocol['labels'] if a['segment'] == seg]
      occupied = np.zeros(len(rows), bool)
      red_interval = np.zeros(len(rows), bool)
      for a in labels:
        interval = (rows[:, 1] >= a['start']) & (rows[:, 1] < a['end'])
        occupied |= interval
        if a['kind'] == 'red':
          red_interval |= interval
        q = interval & (rows[:, 0] % protocol['sample_stride'] == 0)
        if a['kind'] == 'red':
          q &= abs(rows[:, 2]) < .03
        if not q.any():
          continue
        ids = np.flatnonzero(q)
        h = z['upstream'][q].astype(np.float32)
        baseline = self.predict(h)
        target = baseline.copy()
        if a['kind'] == 'red':
          target[:, LONG] = 0
        elif a['kind'] == 'green_recorded':
          recorded = self.recorded(seg, rows[q, 1])
          target[:, LONG] = recorded.reshape(-1, 495)[:, LONG]
        self.groups.append({'event': a['event'], 'kind': a['kind'], 'split': split,
                            'segment': seg, 'indices': ids, 'h': h, 'base': baseline, 'target': target})
      # Preserve existing decelerating approaches; these are teacher outputs,
      # not surveyed stop-line ground truth or endorsements of every old stop.
      stopping_approach = red_interval & (z['plan'][:, -1, 3] < z['plan'][:, 0, 3] * .7)
      normal = ((~occupied) | stopping_approach) & (rows[:, 0] % protocol['normal_stride'] == 0) & (rows[:, 1] > 6)
      normal &= (rows[:, 2] > .5) & (z['plan'][:, 0, 3] >= rows[:, 2] * .7)
      if normal.any():
        h = z['upstream'][normal].astype(np.float32)
        self.normals[split].append({'segment': seg, 'h': h, 'base': self.predict(h)})

  def predict(self, h):
    with torch.no_grad():
      return self.base(torch.from_numpy(h))[0].numpy()

  def recorded(self, seg, times):
    d = self.drives[seg]
    cs = list(d['carState'])
    following = seg.rsplit('--', 1)[0] + '--' + str(int(seg.rsplit('--', 1)[1]) + 1)
    if following in self.drives:
      cs += self.drives[following]['carState']
    ct = np.array([r[0] for r in cs])
    assert np.all(np.diff(ct) > 0)
    v = np.maximum(0, [r[2]['vEgo'] for r in cs])
    accel = np.array([r[2]['aEgo'] for r in cs])
    distance = np.r_[0, np.cumsum(np.diff(ct) * (v[:-1] + v[1:]) / 2)]
    origin = d['roadEncodeIdx'][0][2]['timestampEof'] / 1e9
    result = []
    for t in times:
      now = origin + t
      future = now + T
      assert future[-1] <= ct[-1], (seg, t)
      p = np.zeros((33, 15))
      p[:, 0] = np.interp(future, ct, distance) - np.interp(now, ct, distance)
      p[:, 3] = np.interp(future, ct, v)
      p[:, 6] = np.interp(future, ct, accel)
      result.append(p)
    return np.array(result)


def metrics(base, new):
  a, b = base[:, :495].reshape(-1, 33, 15), new[:, :495].reshape(-1, 33, 15)
  interp = lambda p, c: np.array([np.interp(2, T, r[:, c]) for r in p])
  return {'n': len(a), 'baseline_v10_mae0': float(abs(a[:, -1, 3]).mean()),
          'candidate_v10_mae0': float(abs(b[:, -1, 3]).mean()),
          'baseline_v10_mean': float(a[:, -1, 3].mean()), 'candidate_v10_mean': float(b[:, -1, 3].mean()),
          'baseline_v10_above5': int((a[:, -1, 3] > 5).sum()),
          'candidate_v10_above5': int((b[:, -1, 3] > 5).sum()),
          'negative_v10_fraction': float((b[:, -1, 3] < -.1).mean()),
          'v10_drift': float(abs(b[:, -1, 3] - a[:, -1, 3]).mean()),
          'x10_drift': float(abs(b[:, -1, 0] - a[:, -1, 0]).mean()),
          'v2_drift': float(abs(interp(b, 3) - interp(a, 3)).mean()),
          'y_drift': float(abs(b[:, :, 1] - a[:, :, 1]).mean()),
          'y_max_drift': float(abs(b[:, :, 1] - a[:, :, 1]).max()),
          'y2_drift': float(abs(interp(b, 1) - interp(a, 1)).mean()),
          'v0_max_drift': float(abs(b[:, 0, 3] - a[:, 0, 3]).max()),
          'a0_max_drift': float(abs(b[:, 0, 6] - a[:, 0, 6]).max()),
          'candidate_min_future_v': float(b[:, T >= .5, 3].min()),
          'candidate_min_future_dx': float(np.diff(b[:, T >= .5, 0], axis=1).min()),
          'x_velocity_residual_mae': float(abs(np.diff(b[:, :, 0], axis=1) - .5 * (b[:, 1:, 3] + b[:, :-1, 3]) * np.diff(T)).mean()),
          'baseline_x_velocity_residual_mae': float(abs(np.diff(a[:, :, 0], axis=1) - .5 * (a[:, 1:, 3] + a[:, :-1, 3]) * np.diff(T)).mean()),
          'uncertainty_drift': float(abs(new[:, 495:] - base[:, 495:]).mean())}


def evaluate(data, model, split):
  groups = {}
  with torch.no_grad():
    for item in data.groups:
      if item['split'] != split:
        continue
      group = groups.setdefault(item['event'], {'kind': item['kind'], 'base': [], 'new': []})
      group['base'].append(item['base'])
      group['new'].append(model(torch.from_numpy(item['h']))[0].numpy())
    result = {'events': {}}
    for name, item in groups.items():
      result['events'][name] = {'kind': item['kind'], **metrics(np.concatenate(item['base']), np.concatenate(item['new']))}
    normal = data.normals[split]
    if normal:
      h = np.concatenate([x['h'] for x in normal])
      result['normal'] = metrics(np.concatenate([x['base'] for x in normal]), model(torch.from_numpy(h))[0].numpy())
  return result


def failures(report, protocol):
  g = protocol['gates']
  out = []
  for name, item in {'normal': report.get('normal'), **report['events']}.items():
    if item is None:
      continue
    for key in ('y_drift', 'y_max_drift', 'y2_drift', 'uncertainty_drift'):
      if item[key] > g[key]:
        out.append(name + ':' + key)
    if name == 'normal':
      for key in ('v10_drift', 'x10_drift', 'v2_drift'):
        if item[key] > g[key]:
          out.append(name + ':' + key)
    elif item['kind'] == 'red':
      if item['candidate_v10_mae0'] > max(.15, item['baseline_v10_mae0'] * .9):
        out.append(name + ':red_error')
      if item['candidate_v10_above5']:
        out.append(name + ':red_progress')
      if item['negative_v10_fraction'] > .05:
        out.append(name + ':negative_speed')
    elif item['baseline_v10_mean'] - item['candidate_v10_mean'] > .25:
      out.append(name + ':green_suppression')
    if item.get('kind') == 'green_delay' and item['candidate_v10_mean'] - item['baseline_v10_mean'] < 1:
      out.append(name + ':green_delay_not_improved')
  return out


def fit(data, base, config, protocol):
  torch.manual_seed(20261010)
  model = copy.deepcopy(base)
  model.freeze(config['mode'])
  train = [a for a in data.groups if a['split'] == 'train']
  normals = data.normals['train']
  hs, bs, targets, weights, colors = [], [], [], [], []
  total_normal = sum(len(a['h']) for a in normals)
  counts = {}
  for a in train:
    counts[a['event']] = counts.get(a['event'], 0) + len(a['h'])
  for a in normals + train:
    n = len(a['h'])
    supervised = 'target' in a
    hs.append(a['h']); bs.append(a['base']); targets.append(a.get('target', a['base']))
    weights.append(np.full(n, config['mass'] / counts[a['event']] if supervised else 1 / total_normal))
    colors.append(np.full(n, int(a['kind'] != 'red') if supervised else -1))
  x, original, target = [torch.from_numpy(np.concatenate(v)).float() for v in (hs, bs, targets)]
  w = torch.from_numpy(np.concatenate(weights)).float()
  w /= w.sum()
  color = torch.from_numpy(np.concatenate(colors)).long()
  norm = torch.ones(990)
  for i in range(33):
    norm[i * 15] = 10
    norm[i * 15 + 3] = 3
  keep = np.setdiff1d(np.arange(990), LONG)
  scale_keep = norm[keep]
  optimizer = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=protocol['learning_rate'])
  generator = torch.Generator().manual_seed(20261010)
  trace = []
  for step in range(protocol['steps']):
    ids = torch.multinomial(w, protocol['batch_size'], replacement=True, generator=generator)
    pred, logits = model(x[ids])
    longitudinal = (((pred[:, LONG] - target[ids][:, LONG]) / norm[LONG]) ** 2).mean()
    preserve = (((pred[:, keep] - original[ids][:, keep]) / scale_keep) ** 2).mean()
    # Shared plan features also feed lateral and uncertainty rows; explicitly distill them.
    regularize = sum((a - b.detach()).square().mean() for a, b in zip(model.layers.parameters(), base.layers.parameters()))
    q = color[ids] >= 0
    aux = F.cross_entropy(logits[q], color[ids][q]) if q.any() and config['mode'] == 'plan' else torch.tensor(0.)
    loss = longitudinal + 5 * preserve + .01 * regularize + config['aux_weight'] * aux
    optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), 10); optimizer.step()
    if step % 100 == 0 or step == protocol['steps'] - 1:
      row = {'step': step, 'loss': float(loss.detach()), 'long': float(longitudinal.detach()), 'preserve': float(preserve.detach()), 'color': float(aux.detach())}
      trace.append(row)
      print(config['name'], row, flush=True)
  return model, trace


def export(model, source, destination):
  graph = onnx.load(source)
  updates = {}
  for name, layer in zip(BLOCKS, model.layers):
    for suffix in ('weight', 'bias'):
      updates[PREFIX + name + '.' + suffix] = getattr(layer, suffix).detach().numpy().astype(np.float16)
  changed = []
  for tensor in graph.graph.initializer:
    if tensor.name in updates:
      new = updates[tensor.name]
      if not np.array_equal(numpy_helper.to_array(tensor), new):
        changed.append(tensor.name)
      tensor.CopyFrom(numpy_helper.from_array(new, tensor.name))
  onnx.checker.check_model(graph)
  onnx.save(graph, destination)
  return {'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(), 'changed_initializers': changed,
          'nodes_unchanged': True, 'input_output_contract_unchanged': True}


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--source', type=Path, required=True)
  parser.add_argument('--data', type=Path, required=True)
  args = parser.parse_args()
  torch.set_num_threads(4)
  torch.manual_seed(20261010)
  protocol = read_json(args.data / 'protocol.json')
  arrays = {x.name: numpy_helper.to_array(x).copy() for x in onnx.load(args.source).graph.initializer}
  base = Plan(arrays)
  data = Dataset(args.data, protocol, base)
  counts = [{'event': a['event'], 'segment': a['segment'], 'split': a['split'], 'kind': a['kind'], 'n': len(a['h'])} for a in data.groups]
  (args.data / 'training_counts.json').write_text(json.dumps({'events': counts, 'normal': {s: sum(len(a['h']) for a in data.normals[s]) for s in data.normals}}, indent=2))
  results, models = [], {}
  (args.data / 'candidates').mkdir(exist_ok=True)
  for config in protocol['candidates']:
    model, trace = fit(data, base, config, protocol)
    # Selection reflects FP16 exported weights, though arithmetic here is FP32.
    with torch.no_grad():
      for p in model.layers.parameters():
        p.copy_(p.half().float())
    report = evaluate(data, model, 'development')
    fail = failures(report, protocol)
    red = [a['candidate_v10_mae0'] for a in report['events'].values() if a['kind'] == 'red']
    artifact = export(model, args.source, args.data / 'candidates' / (config['name'] + '.onnx'))
    torch.save(model.state_dict(), args.data / 'candidates' / (config['name'] + '.pt'))
    results.append({'config': config, 'trace': trace, 'development': report, 'failures': fail, 'red_score': float(np.mean(red)), 'artifact': artifact})
    models[config['name']] = model
    print('CANDIDATE', config['name'], 'FAILURES', fail, 'RED', np.mean(red), flush=True)
  chosen = min(results, key=lambda r: (len(r['failures']), r['red_score']))
  frozen = {'selected': chosen['config']['name'], 'development_passed': not chosen['failures'], 'candidates': results,
            'protocol_sha256': hashlib.sha256((args.data/'protocol.json').read_bytes()).hexdigest()}
  (args.data / 'selection_frozen.json').write_text(json.dumps(frozen, indent=2))
  selected = models[frozen['selected']]
  artifact = export(selected, args.source, args.data / 'internal_signal_night_candidate.onnx')
  torch.save(selected.state_dict(), args.data / 'candidate.pt')
  evaluation = evaluate(data, selected, 'evaluation')
  result = {'selection': frozen['selected'], 'train': evaluate(data, selected, 'train'), 'development': chosen['development'],
            'evaluation': evaluation, 'development_failures': chosen['failures'], 'evaluation_failures': failures(evaluation, protocol),
            'artifact': artifact, 'control_deployment': False}
  result['offline_gates_passed'] = not result['development_failures'] and not result['evaluation_failures']
  (args.data / 'result.json').write_text(json.dumps(result, indent=2))
  print('COMPLETE', json.dumps({k: v for k, v in result.items() if k not in ('train', 'development', 'evaluation')}), flush=True)


if __name__ == '__main__':
  main()
