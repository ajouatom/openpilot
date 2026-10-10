#!/usr/bin/env python3
"""Offline original vision/temporal weight training with regenerated history.

Trusted private replay input only. Never installs a model or connects to a car.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import onnx
import torch
from torch.nn import functional as F

from signal_vision_model import VisionTrial
from finetune_signal_plan import Dataset, Plan, LONG, metrics, failures


def load_json(path):
  return json.loads(Path(path).read_text(encoding='utf-8-sig'))


class ReplayData:
  def __init__(self, root, old_root, source, protocol, device):
    self.root, self.device = root, device
    self.entries = load_json(root / 'manifest.json')
    self.offsets = {}
    maps, desires, original, sequences, valid, rows = [], [], [], [], [], []
    cursor, start, previous = 0, 0, None
    for item in self.entries:
      seg = item['segment']
      contiguous = previous and previous.rsplit('--', 1)[0] == seg.rsplit('--', 1)[0] and int(seg.rsplit('--', 1)[1]) == int(previous.rsplit('--', 1)[1]) + 1
      if not contiguous:
        start = cursor
      z = np.load(root / 'features' / f'{seg}.npz')
      n = len(z['rows'])
      self.offsets[seg] = (cursor, n)
      maps.append(z['upstream']); desires.append(z['desire']); original.append(z['outputs']); rows.append(z['rows'])
      ids = np.arange(cursor, cursor + n)[:, None] + np.arange(-32, 1, 4)[None]
      valid.append(ids >= start); sequences.append(np.maximum(ids, start))
      cursor += n; previous = seg
    self.maps = torch.from_numpy(np.concatenate(maps)).to(device)
    self.desire = torch.from_numpy(np.concatenate(desires).astype(np.float32)).to(device)
    self.sequence = torch.from_numpy(np.concatenate(sequences)).to(device)
    self.valid = torch.from_numpy(np.concatenate(valid)).to(device)
    self.original = np.concatenate(original).astype(np.float32)
    self.rows = np.concatenate(rows)
    self.traffic = torch.tensor([[1., 0.]], device=device)
    arrays = {x.name: onnx.numpy_helper.to_array(x).copy() for x in onnx.load(source).graph.initializer}
    # Reuse the frozen encounter labels/recorded green trajectory target builder.
    meta = Dataset(old_root, protocol, Plan(arrays))
    self.groups = []
    self.normal = {}
    for item in meta.groups:
      offset, _ = self.offsets[item['segment']]
      self.groups.append({k: item[k] for k in ('event', 'kind', 'split', 'segment')} |
                         {'ids': item['indices'] + offset, 'target': item['target'][:, LONG].copy()})
    for split in ('train', 'development', 'evaluation'):
      ids = []
      for entry in self.entries:
        if entry['split'] != split:
          continue
        seg = entry['segment']; z = meta.cache[seg]; rr = z['rows']; occupied = np.zeros(len(rr), bool); red = np.zeros(len(rr), bool)
        for a in protocol['labels']:
          if a['segment'] == seg:
            interval = (rr[:, 1] >= a['start']) & (rr[:, 1] < a['end'])
            occupied |= interval
            if a['kind'] == 'red':
              red |= interval
        stopping = red & (z['plan'][:, -1, 3] < z['plan'][:, 0, 3] * .7)
        q = ((~occupied) | stopping) & (rr[:, 0] % protocol['normal_stride'] == 0) & (rr[:, 1] > 6)
        q &= (rr[:, 2] > .5) & (z['plan'][:, 0, 3] >= rr[:, 2] * .7)
        ids.extend((np.flatnonzero(q) + self.offsets[seg][0]).tolist())
      self.normal[split] = np.array(ids, dtype=np.int64)
    print('DATA', len(self.original), 'maps_MB', self.maps.numel() * 2 / 1e6, flush=True)

  def sequence_forward(self, model, ids):
    seq = self.sequence[ids]
    assert bool(self.valid[ids].all()), 'training sample needs real complete history'
    return model(self.maps[seq].float(), self.desire[ids], self.traffic.expand(len(ids), -1))


@torch.no_grad()
def replay(model, data):
  """Recompute every image embedding before rebuilding every temporal input."""
  n = len(data.original)
  vision = torch.empty((n, 1576), device=data.device)
  for lo in range(0, n, 256):
    vision[lo:lo + 256] = model.vision(data.maps[lo:lo + 256].float())
  policy = torch.empty((n, 1000), device=data.device)
  for lo in range(0, n, 256):
    seq = data.sequence[lo:lo + 256]
    h = vision[seq, 1064:1576] * data.valid[lo:lo + 256, :, None]
    policy[lo:lo + 256] = model.policy(h, data.desire[lo:lo + 256], data.traffic.expand(len(seq), -1))
  return torch.cat([vision, policy], 1)


def reports(data, base, new, norm, split):
  result = {'events': {}}
  by_event = {}
  for item in data.groups:
    if item['split'] == split:
      event = by_event.setdefault(item['event'], {'kind': item['kind'], 'ids': []})
      event['ids'].extend(item['ids'].tolist())
  for name, item in {'normal': {'ids': data.normal[split], 'kind': 'preserve'}, **by_event}.items():
    ids = item['ids']
    if not len(ids):
      continue
    m = metrics(base[ids, 1576:2566], new[ids, 1576:2566])
    m['vision_normalized_drift'] = float(abs((new[ids, :1064] - base[ids, :1064]) / norm[:1064]).mean())
    m['hidden_drift'] = float(abs(new[ids, 1064:1576] - base[ids, 1064:1576]).mean())
    if name == 'normal':
      result['normal'] = m
    else:
      result['events'][name] = {'kind': item['kind'], **m}
  return result


def gates(report, protocol):
  fail = failures(report, protocol)
  for name, item in {'normal': report.get('normal'), **report['events']}.items():
    if item is None:
      continue
    if item['vision_normalized_drift'] > .01:
      fail.append(name + ':vision_drift')
    if item['v0_max_drift'] > .15:
      fail.append(name + ':initial_speed_drift')
  return fail


def train(base, data, teacher, norm, config, protocol):
  torch.manual_seed(20261010)
  model = copy.deepcopy(base)
  model.configure(config['temporal'])
  ids = [data.normal['train']]; targets = [teacher[ids[0]][:, 1576 + LONG].cpu().numpy()]
  weights = [np.full(len(ids[0]), 1 / len(ids[0]))]; colors = [np.full(len(ids[0]), -1)]
  groups = [x for x in data.groups if x['split'] == 'train']
  totals = {}
  for item in groups:
    totals[item['event']] = totals.get(item['event'], 0) + len(item['ids'])
  for item in groups:
    ids.append(item['ids']); targets.append(item['target'])
    weights.append(np.full(len(item['ids']), .3 / totals[item['event']]))
    colors.append(np.full(len(item['ids']), int(item['kind'] != 'red')))
  sample_ids = torch.from_numpy(np.concatenate(ids)).to(data.device)
  targets = torch.from_numpy(np.concatenate(targets)).float().to(data.device)
  colors = torch.from_numpy(np.concatenate(colors)).long().to(data.device)
  weights = torch.from_numpy(np.concatenate(weights)).float().to(data.device)
  weights /= weights.sum()
  parameter_count = sum(p.numel() for p in model.parameters() if p.requires_grad)
  optimizer = torch.optim.Adam([p for p in model.parameters() if p.requires_grad], lr=config['lr'])
  keep = np.setdiff1d(np.arange(1576, 2576), 1576 + LONG)
  trace = []; gradient_audit = None; begin = time.monotonic()
  for step in range(protocol['steps']):
    selection = torch.multinomial(weights, protocol['batch_size'], replacement=True)
    batch = sample_ids[selection]
    output, logits, hidden = data.sequence_forward(model, batch)
    original = teacher[batch]
    target_loss = (((output[:, 1576 + LONG] - targets[selection]) / norm[1576 + LONG]) ** 2).mean()
    policy_keep = (((output[:, keep] - original[:, keep]) / norm[keep]) ** 2).mean()
    vision_keep = (((output[:, :1064] - original[:, :1064]) / norm[:1064]) ** 2).mean()
    hidden_keep = (((hidden - teacher[data.sequence[batch], 1064:1576]) / .02) ** 2).mean()
    q = colors[selection] >= 0
    auxiliary = F.cross_entropy(logits[q], colors[selection][q]) if q.any() else torch.tensor(0., device=data.device)
    loss = target_loss + 5 * policy_keep + 10 * vision_keep + .1 * hidden_keep + .1 * auxiliary
    assert bool(torch.isfinite(loss)), (config['name'], step)
    if step == 0:
      # Prove x/v/a loss itself reaches real original camera weights, without aux loss.
      target_loss.backward(retain_graph=True)
      gradient_audit = {name: float(model.vision.w(name).grad.norm()) for name in model.vision.names if model.vision.w(name).requires_grad}
      assert any(v > 0 for v in gradient_audit.values()) and all(np.isfinite(v) for v in gradient_audit.values())
      optimizer.zero_grad()
    optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), 10); optimizer.step()
    if step % 100 == 0 or step == protocol['steps'] - 1:
      row = {'step': step, 'loss': float(loss.detach()), 'trajectory': float(target_loss.detach()), 'vision_keep': float(vision_keep.detach()),
             'policy_keep': float(policy_keep.detach()), 'hidden_keep': float(hidden_keep.detach()), 'color': float(auxiliary.detach()), 'seconds': time.monotonic() - begin}
      trace.append(row); print(config['name'], row, flush=True)
  return model, trace, gradient_audit, parameter_count


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--source', type=Path, required=True)
  parser.add_argument('--data', type=Path, required=True)
  parser.add_argument('--previous-data', type=Path, required=True)
  args = parser.parse_args()
  torch.set_num_threads(4); torch.manual_seed(20261010)
  torch.backends.cuda.matmul.allow_tf32 = False
  torch.backends.cudnn.allow_tf32 = False
  torch.backends.cudnn.deterministic = True
  assert torch.cuda.is_available()
  device = torch.device('cuda')
  protocol = load_json(args.data / 'protocol.json')
  data = ReplayData(args.data, args.previous_data, args.source, protocol, device)
  base = VisionTrial(args.source).to(device)
  teacher = replay(base, data)
  baseline = teacher.cpu().numpy()
  np.save(args.data / 'baseline_outputs.npy', baseline)
  error = abs(baseline - data.original)
  parity = {'frames': len(error), 'plan_v_max': float(error[:, 1576:2071].reshape(-1, 33, 15)[:, :, 3].max()),
            'plan_x_max': float(error[:, 1576:2071].reshape(-1, 33, 15)[:, :, 0].max()),
            'hidden_max': float(error[:, 1064:1576].max()), 'whole_max': float(error.max())}
  (args.data / 'baseline_parity.json').write_text(json.dumps(parity, indent=2)); print('BASELINE_PARITY', parity, flush=True)
  assert parity['plan_v_max'] < .15 and parity['plan_x_max'] < 1., parity
  norm = np.maximum(.1, np.sqrt(np.mean(baseline[data.normal['train']] ** 2, axis=0)))
  for i in range(33):
    norm[1576 + 15 * i] = 10
    norm[1576 + 15 * i + 3] = 3
    norm[1576 + 15 * i + 6] = 1
  norm_gpu = torch.tensor(norm, device=device)
  np.save(args.data / 'normalization.npy', norm)
  (args.data / 'candidates').mkdir(exist_ok=True)
  results = []
  for config in protocol['candidates']:
    model, trace, audit, count = train(base, data, teacher, norm_gpu, config, protocol)
    # All selection/evaluation weights are rounded exactly to the exported FP16 values.
    with torch.no_grad():
      for module in (model.vision, model.policy):
        for p in module.parameters():
          p.copy_(p.half().float())
    path = args.data / 'candidates' / (config['name'] + '.onnx')
    changed = model.export(args.source, path)
    torch.save(model.state_dict(), args.data / 'candidates' / (config['name'] + '.pt'))
    candidate = replay(model, data).cpu().numpy()
    assert np.isfinite(candidate).all(), config['name']
    np.save(args.data / 'candidates' / (config['name'] + '_outputs.npy'), candidate)
    report = reports(data, baseline, candidate, norm, 'development')
    fail = gates(report, protocol)
    score = float(np.mean([a['candidate_v10_mae0'] for a in report['events'].values() if a['kind'] == 'red']))
    result = {'config': config, 'trace': trace, 'xva_gradient_to_vision': audit, 'trainable_parameters_including_aux': count,
              'changed_initializers': changed, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'development': report,
              'failures': fail, 'red_score': score}
    results.append(result)
    print('CANDIDATE', config['name'], 'RED', score, 'FAILURES', fail, flush=True)
    del model, candidate
  selected = min(results, key=lambda x: (len(x['failures']), x['red_score']))
  frozen = {'selected': selected['config']['name'], 'candidates': results, 'protocol_sha256': hashlib.sha256((args.data/'protocol.json').read_bytes()).hexdigest()}
  (args.data / 'selection_frozen.json').write_text(json.dumps(frozen, indent=2))
  candidate = np.load(args.data / 'candidates' / (frozen['selected'] + '_outputs.npy'))
  evaluation = reports(data, baseline, candidate, norm, 'evaluation')
  result = {'selected': frozen['selected'], 'train': reports(data, baseline, candidate, norm, 'train'),
            'development': selected['development'], 'evaluation': evaluation, 'development_failures': selected['failures'],
            'evaluation_failures': gates(evaluation, protocol), 'vehicle_installed': False}
  result['offline_gates_passed'] = not result['development_failures'] and not result['evaluation_failures']
  (args.data / 'result.json').write_text(json.dumps(result, indent=2))
  print('COMPLETE', result['selected'], result['offline_gates_passed'], result['evaluation_failures'], flush=True)


if __name__ == '__main__':
  main()
