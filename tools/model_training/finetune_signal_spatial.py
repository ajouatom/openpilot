#!/usr/bin/env python3
"""Offline original ONNX fine-tuning with reviewed spatial and stop-line labels.

Uses trusted private replay data. Does not install or connect to a vehicle.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import torch
from torch.nn import functional as F

from finetune_signal_vision import ReplayData, reports, gates, load_json
from finetune_signal_plan import LONG
from signal_spatial_model import SpatialTrial


@torch.no_grad()
def replay(model, data):
  """Rebuild all candidate histories; small batches bound deeper CNN memory."""
  n = len(data.original)
  vision = torch.empty((n, 1576), device=data.device)
  for lo in range(0, n, 64):
    vision[lo:lo + 64] = model.vision(data.maps[lo:lo + 64].float())
  policy = torch.empty((n, 1000), device=data.device)
  for lo in range(0, n, 64):
    seq = data.sequence[lo:lo + 64]
    hidden = vision[seq, 1064:1576] * data.valid[lo:lo + 64, :, None]
    policy[lo:lo + 64] = model.policy(hidden, data.desire[lo:lo + 64], data.traffic.expand(len(seq), -1))
  return torch.cat([vision, policy], dim=1)


class AuxData:
  def __init__(self, root, data):
    self.data = data
    self.rows = [r for r in load_json(root / 'spatial_labels.json') if r['eligible']]
    self.stops = [r for r in load_json(root / 'stop_line_labels.json') if r['pixel_eligible']]
    tensor = lambda x, dtype=torch.float32: torch.tensor(x, device=data.device, dtype=dtype)
    self.ids = tensor([data.offsets[r['segment']][0] + r['frame'] for r in self.rows], torch.long)
    self.stream = tensor([int(r['stream'] == 'wide') for r in self.rows], torch.long)
    self.target = tensor([r['class'] for r in self.rows], torch.long)
    self.point = tensor([r['point_model'] for r in self.rows]) / tensor([512, 256])
    self.stop_ids = tensor([data.offsets[r['segment']][0] + r['frame'] for r in self.stops], torch.long)
    self.stop_target = tensor([[*r['point_model'], r['camera_to_line_m']] for r in self.stops]) / tensor([512, 256, 30])
    self.stop_mask = tensor([[1, 1, float(r['distance_eligible'])] for r in self.stops])
    self.train = tensor([i for i, r in enumerate(self.rows) if r['split'] == 'train'], torch.long)
    self.stop_train = tensor([i for i, r in enumerate(self.stops) if r['split'] == 'train'], torch.long)
    counts = torch.bincount(self.target[self.train], minlength=3).clamp_min(1)
    self.weights = 1 / counts[self.target[self.train]].float()

  def spatial_loss(self, logits, centers, indices):
    n = len(indices)
    grid = (self.point[indices] * 2 - 1).reshape(n, 1, 1, 2)
    # Continuous interpolation on the original 8x16 feature grid, per camera.
    cls = F.grid_sample(logits, grid, align_corners=True).reshape(n, 2, 3)
    xy = F.grid_sample(centers, grid, align_corners=True).reshape(n, 2, 2)
    at = torch.arange(n, device=indices.device)
    cls, xy = cls[at, self.stream[indices]], xy[at, self.stream[indices]]
    positive = self.target[indices] > 0
    location = F.smooth_l1_loss(xy[positive], self.point[indices][positive], beta=.05) if positive.any() else xy.sum() * 0
    return F.cross_entropy(cls, self.target[indices]) + 5 * location, cls, xy

  def losses(self, model, n, detach=False):
    a = self.train[torch.multinomial(self.weights, n, replacement=True)]
    b = self.stop_train[torch.randint(len(self.stop_train), (min(8, n),), device=self.data.device)]
    if detach:
      with torch.no_grad():
        va, taps = model.vision(self.data.maps[self.ids[a]].float(), capture=('vision/add_9',))
        vb = model.vision(self.data.maps[self.stop_ids[b]].float())
      logits = model.spatial_head(taps['vision/add_9']); centers = model.center_head(taps['vision/add_9'])
      stops = model.stop_head(vb[:, 1064:1576])
    else:
      logits, centers, _ = model.auxiliary(self.data.maps[self.ids[a]].float())
      _, _, stops = model.auxiliary(self.data.maps[self.stop_ids[b]].float())
    spatial, _, _ = self.spatial_loss(logits, centers, a)
    diff = F.smooth_l1_loss(stops, self.stop_target[b], beta=.05, reduction='none')
    geometry = (diff * self.stop_mask[b]).sum() / self.stop_mask[b].sum()
    return spatial, geometry

  @torch.no_grad()
  def evaluate(self, model, split):
    idx = torch.tensor([i for i, r in enumerate(self.rows) if r['split'] == split], device=self.data.device)
    logits, centers, _ = model.auxiliary(self.data.maps[self.ids[idx]].float())
    loss, cls, xy = self.spatial_loss(logits, centers, idx)
    predicted = cls.argmax(-1); truth = self.target[idx]
    positive = truth > 0
    report = {'points': len(idx), 'correct_at_provided_location': int((predicted == truth).sum()),
              'red_as_green': int(((truth == 1) & (predicted == 2)).sum()),
              'negative_as_signal': int(((truth == 0) & (predicted > 0)).sum()),
              'class_counts': torch.bincount(truth, minlength=3).tolist(),
              'center_mae_model_px': (abs(xy[positive] - self.point[idx][positive]) * torch.tensor([512, 256], device=idx.device)).mean(0).tolist(),
              'loss': float(loss), 'not_full_frame_detection': True}
    ss = [i for i, r in enumerate(self.stops) if r['split'] == split]
    if ss:
      _, _, pred = model.auxiliary(self.data.maps[self.stop_ids[ss]].float())
      errors = abs(pred - self.stop_target[ss]) * torch.tensor([512, 256, 30], device=idx.device)
      valid = self.stop_mask[ss, 2] > 0
      report['stop'] = {'points': len(ss), 'pixel_mae_xy': errors[:, :2].mean(0).tolist(),
                        'weak_metric_mae_m': float(errors[valid, 2].mean()) if valid.any() else None,
                        'metric_samples': int(valid.sum())}
    return report


def fit(base, data, aux, teacher, norm, config, protocol):
  torch.manual_seed(20261010)
  model = copy.deepcopy(base); model.configure()
  original = [p for p in model.vision.parameters() if p.requires_grad]
  auxiliary = model.auxiliary_parameters()
  optimizer = torch.optim.Adam([{'params': original, 'lr': config['lr']}, {'params': auxiliary, 'lr': protocol['aux_learning_rate']}])
  ids = [data.normal['train']]; targets = [teacher[ids[0]][:, 1576 + LONG].cpu().numpy()]
  weights = [np.full(len(ids[0]), 1 / len(ids[0]))]; colors = [np.full(len(ids[0]), -1)]
  groups = [x for x in data.groups if x['split'] == 'train']; totals = {}
  for item in groups:
    totals[item['event']] = totals.get(item['event'], 0) + len(item['ids'])
  for item in groups:
    ids.append(item['ids']); targets.append(item['target'])
    weights.append(np.full(len(item['ids']), .3 / totals[item['event']]))
    colors.append(np.full(len(item['ids']), int(item['kind'] != 'red')))
  ids = torch.tensor(np.concatenate(ids), device=data.device)
  targets = torch.tensor(np.concatenate(targets), device=data.device)
  colors = torch.tensor(np.concatenate(colors), device=data.device)
  weights = torch.tensor(np.concatenate(weights), device=data.device); weights /= weights.sum()
  keep = np.setdiff1d(np.arange(1576, 2576), 1576 + LONG)
  trace = []; begin = time.monotonic(); audit = {}
  for step in range(protocol['steps']):
    selected = torch.multinomial(weights, protocol['batch_size'], replacement=True)
    batch = ids[selected]; output, logits, hidden = data.sequence_forward(model, batch)
    target = (((output[:, 1576 + LONG] - targets[selected]) / norm[1576 + LONG]) ** 2).mean()
    policy_keep = (((output[:, keep] - teacher[batch][:, keep]) / norm[keep]) ** 2).mean()
    vision_keep = (((output[:, :1064] - teacher[batch][:, :1064]) / norm[:1064]) ** 2).mean()
    hidden_keep = (((hidden - teacher[data.sequence[batch], 1064:1576]) / .02) ** 2).mean()
    q = colors[selected] >= 0
    color = F.cross_entropy(logits[q], colors[selected][q]) if q.any() else output.sum() * 0
    spatial, geometry = aux.losses(model, protocol['aux_batch_size'], detach=config['spatial'] == config['geometry'] == 0)
    # Control heads remain diagnostic: their features have no camera gradients.
    strength = config['spatial'] if config['spatial'] else 1.
    geo_strength = config['geometry'] if config['geometry'] else 1.
    if config['geometry'] == 0 and config['spatial'] > 0:
      # Spatial-only candidates must not receive an unrequested geometry gradient.
      geometry = geometry * 0
    loss = target + 5 * policy_keep + 10 * vision_keep + .1 * hidden_keep + .1 * color + strength * spatial + geo_strength * geometry
    assert torch.isfinite(loss), (config, step)
    if step == 0:
      name = next(n for n in model.vision.names if '.stages.2.blocks.5.token_mixer.reparam_conv.weight' in n)
      weight = model.vision.w(name)
      audit['xva_gradient_to_deeper_vision'] = float(torch.autograd.grad(target, weight, retain_graph=True)[0].norm())
      if config['spatial']:
        audit['spatial_gradient_to_deeper_vision'] = float(torch.autograd.grad(spatial, weight, retain_graph=True)[0].norm())
      if config['geometry']:
        audit['geometry_gradient_to_deeper_vision'] = float(torch.autograd.grad(geometry, weight, retain_graph=True)[0].norm())
      assert all(np.isfinite(v) and v > 0 for v in audit.values()), audit
    optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(original, 10); optimizer.step()
    if step % 200 == 0 or step == protocol['steps'] - 1:
      row = {'step': step, 'loss': float(loss.detach()), 'trajectory': float(target.detach()), 'spatial': float(spatial.detach()),
             'geometry': float(geometry.detach()), 'vision_keep': float(vision_keep.detach()), 'seconds': time.monotonic() - begin}
      trace.append(row); print(config['name'], row, flush=True)
  return model, trace, audit, sum(p.numel() for p in original)


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument('--source', type=Path, required=True); p.add_argument('--data', type=Path, required=True)
  p.add_argument('--previous-data', type=Path, required=True)
  args = p.parse_args()
  torch.set_num_threads(4); torch.manual_seed(20261010)
  torch.backends.cuda.matmul.allow_tf32 = False; torch.backends.cudnn.allow_tf32 = False
  torch.backends.cudnn.deterministic = True
  protocol = load_json(args.data / 'protocol.json')
  for name, expected in protocol['label_hashes'].items():
    assert hashlib.sha256((args.data / name).read_bytes()).hexdigest() == expected
  data = ReplayData(args.data, args.previous_data, args.source, protocol, torch.device('cuda'))
  base = SpatialTrial(args.source).to(data.device); teacher = replay(base, data); baseline = teacher.cpu().numpy()
  np.save(args.data / 'baseline_outputs.npy', baseline)
  error = abs(baseline - data.original); plan_error = error[:, 1576:2071].reshape(-1, 33, 15)
  parity = {'frames': len(error), 'x_max': float(plan_error[:, :, 0].max()), 'v_max': float(plan_error[:, :, 3].max()),
            'hidden_max': float(error[:, 1064:1576].max()), 'whole_max': float(error.max())}
  (args.data / 'baseline_parity.json').write_text(json.dumps(parity, indent=2)); print('PARITY', parity, flush=True)
  assert parity['x_max'] < 1 and parity['v_max'] < .15
  aux = AuxData(args.data, data)
  optimizer = torch.optim.Adam(base.auxiliary_parameters(), lr=protocol['aux_learning_rate'])
  for step in range(protocol['aux_warmup_steps']):
    spatial, geometry = aux.losses(base, protocol['aux_batch_size'], detach=True)
    optimizer.zero_grad(); (spatial + geometry).backward(); optimizer.step()
  print('AUX_WARMUP_COMPLETE', aux.evaluate(base, 'train'), flush=True)
  norm = np.maximum(.1, np.sqrt(np.mean(baseline[data.normal['train']] ** 2, axis=0)))
  for i in range(33):
    norm[1576 + i * 15] = 10; norm[1576 + i * 15 + 3] = 3; norm[1576 + i * 15 + 6] = 1
  np.save(args.data / 'normalization.npy', norm); norm_gpu = torch.tensor(norm, device=data.device)
  (args.data / 'candidates').mkdir(exist_ok=True); results = []
  for config in protocol['candidates']:
    model, trace, audit, count = fit(base, data, aux, teacher, norm_gpu, config, protocol)
    with torch.no_grad():
      for module in (model.vision, model.policy):
        for weight in module.parameters():weight.copy_(weight.half().float())
    path = args.data / 'candidates' / (config['name'] + '.onnx')
    changed = model.export(args.source, path); torch.save(model.state_dict(), path.with_suffix('.pt'))
    predicted = replay(model, data).cpu().numpy(); assert np.isfinite(predicted).all()
    np.save(args.data / 'candidates' / (config['name'] + '_outputs.npy'), predicted)
    report = reports(data, baseline, predicted, norm, 'development'); fail = gates(report, protocol)
    result = {'config': config, 'changed_initializers': changed, 'original_trainable_parameters': count,
              'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'trace': trace, 'gradient_audit': audit,
              'development': report, 'failures': fail,
              'red_score': float(np.mean([e['candidate_v10_mae0'] for e in report['events'].values() if e['kind'] == 'red'])),
              'auxiliary': {split: aux.evaluate(model, split) for split in ('train', 'development')}}
    results.append(result); print('CANDIDATE', config['name'], len(fail), result['red_score'], flush=True)
    del model, predicted
  chosen = min(results, key=lambda r: (len(r['failures']), r['red_score']))
  frozen = {'selected': chosen['config']['name'], 'candidates': results,
            'protocol_sha256': hashlib.sha256((args.data / 'protocol.json').read_bytes()).hexdigest()}
  (args.data / 'selection_frozen.json').write_text(json.dumps(frozen, indent=2))
  prediction = np.load(args.data / 'candidates' / (frozen['selected'] + '_outputs.npy'))
  evaluation = reports(data, baseline, prediction, norm, 'evaluation')
  model = copy.deepcopy(base); model.load_state_dict(torch.load(args.data / 'candidates' / (frozen['selected'] + '.pt'), weights_only=True))
  result = {'selected': frozen['selected'], 'development': chosen['development'], 'evaluation': evaluation,
            'development_failures': chosen['failures'], 'evaluation_failures': gates(evaluation, protocol),
            'auxiliary_evaluation': aux.evaluate(model, 'evaluation'), 'vehicle_installed': False}
  result['offline_gates_passed'] = not result['development_failures'] and not result['evaluation_failures']
  (args.data / 'result.json').write_text(json.dumps(result, indent=2))
  print('COMPLETE', result['selected'], result['offline_gates_passed'], flush=True)


if __name__ == '__main__':
  main()
