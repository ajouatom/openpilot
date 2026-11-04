"""Offline recorded-input comparison, never a vehicle controller or closed-loop simulation.

Existing messages without yaw-rate std cannot reconstruct model uncertainty.
Constant-sigma results are explicitly sensitivity scenarios, not model predictions.
"""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import csv
import io
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import zstandard

from openpilot.cereal import log
from openpilot.selfdrive.carrot.curve_speed import curve_speed, VisionCurveSpeed
from tools.modeld.ssh_curve_speed_reference import CurveSpeed as ReferenceCurve  # noqa: TID251 - offline top-level tool

SIGMAS = (0.0, 0.01, 0.03, 0.05)  # rad/s; hypothetical constants, not measured uncertainty


def metrics(values):
  a = np.asarray(values, dtype=float)
  return {'count': len(a), 'median': float(np.median(a)), 'p05': float(np.quantile(a, .05)),
          'p95': float(np.quantile(a, .95)), 'min': float(a.min()), 'max': float(a.max())} if len(a) else {'count': 0}


def read_segment(path):
  raw = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(path.read_bytes())).read()
  models, cars, plans, backends = [], [], [], []
  settings = {'AutoCurveSpeedFactor': 100, 'AutoCurveSpeedLowerLimit': 30}
  setting_present = []
  identity, hashes = {}, set()
  for event in log.Event.read_multiple_bytes(raw):
    kind, t = event.which(), event.logMonoTime / 1e9
    if kind == 'initData':
      identity = {k: str(getattr(event.initData, k)) for k in ('gitCommit', 'deviceType')}
      for entry in event.initData.params.entries:
        if entry.key in settings:
          try:
            settings[entry.key] = int(entry.value)
            setting_present.append(entry.key)
          except (ValueError, TypeError):
            pass
    elif kind == 'carState':
      c = event.carState
      cars.append((t, event.valid, float(c.vEgo), float(c.aEgo), float(c.vCluRatio)))
    elif kind == 'longitudinalPlan':
      plans.append((t, event.valid, float(event.longitudinalPlan.aTarget)))
    elif kind == 'modelV2':
      m = event.modelV2
      models.append((t, event.valid, m.frameId, list(m.position.t), list(m.position.x), list(m.position.y), list(m.position.z),
                     list(m.velocity.x), list(m.orientationRate.z), list(m.orientationRate.zStd), len(m.rawPredictions)))
    elif kind == 'logMessage':
      try:
        v = json.loads(event.logMessage).get('msg', {})
      except (ValueError, TypeError):
        continue
      if isinstance(v, dict) and v.get('event') == 'runtimeTiming':
        if v.get('component') == 'modeld':
          backends.append((t, v['backend']))
        if v.get('model_sha256'):
          hashes.add(v['model_sha256'])
  return sorted(models), sorted(cars), sorted(plans), sorted(backends), identity, settings, setting_present, sorted(hashes)


def compare(path):
  models, cars, plans, backends, identity, settings, present, hashes = read_segment(path)
  car_times, plan_times, backend_times = ([v[0] for v in a] for a in (cars, plans, backends))
  histories = {sigma: VisionCurveSpeed() for sigma in SIGMAS}
  references = {sigma: ReferenceCurve(.05) for sigma in SIGMAS}
  measured_current, measured_ref = VisionCurveSpeed(), ReferenceCurve(.05)
  rows, skipped = [], Counter()
  last = None
  for t, valid, frame, tt, xx, yy, zz, vv, yaw, std, raw_bytes in models:
    ci = bisect_right(car_times, t) - 1
    if not valid or ci < 0 or not cars[ci][1] or t - cars[ci][0] > .15:
      skipped['invalid_or_stale_input'] += 1
      continue
    _, _, speed, accel, ratio = cars[ci]
    if not (len(tt) == len(xx) == len(yy) == len(zz) == len(vv) == len(yaw) == 33
            and np.isfinite([tt, xx, yy, zz, vv, yaw]).all() and np.isfinite([speed, accel, ratio]).all()):
      skipped['invalid_shape_or_numbers'] += 1
      continue
    if last is None or t - last > .2:
      histories = {sigma: VisionCurveSpeed() for sigma in SIGMAS}
      references = {sigma: ReferenceCurve(.05) for sigma in SIGMAS}
      measured_current, measured_ref = VisionCurveSpeed(), ReferenceCurve(.05)
    last = t
    model = NS(position=NS(t=tt, x=xx, y=yy, z=zz), velocity=NS(x=vv), orientationRate=NS(z=yaw, zStd=[]))
    pi, bi = bisect_right(plan_times, t)-1, bisect_right(backend_times, t)-1
    # Runtime timing is periodic. Do not backfill identities before the first record.
    backend = backends[bi][1] if bi >= 0 else 'unidentified'
    row = {'t': t, 'frame': frame, 'speed_kph': speed*3.6, 'backend': backend,
           'measured_std': len(std) == 33, 'raw_prediction_bytes': raw_bytes,
           'recorded_accel': plans[pi][2] if pi >= 0 and plans[pi][1] and t-plans[pi][0] <= .15 else None}
    curvature_mask = (np.asarray(tt) >= .5) & (np.asarray(tt) <= 6.) & (np.asarray(vv) >= 3.)
    row['near_straight_prediction'] = bool(speed >= 40/3.6 and np.any(curvature_mask) and
      np.max(np.abs(np.asarray(yaw)[curvature_mask])/np.asarray(vv)[curvature_mask]) < .001)
    for sigma in SIGMAS:
      # Same recorded path/speed, with an explicitly hypothetical yaw-rate std.
      signed = np.where(np.asarray(yaw) < 0, -1., 1.) * (np.abs(yaw) + sigma)
      model.orientationRate.z = signed.tolist()
      result = curve_speed(model, speed, settings['AutoCurveSpeedFactor']*.01,
                           settings['AutoCurveSpeedLowerLimit'], speed_ratio=ratio, a_ego=accel)
      ceiling = abs(histories[sigma].update(result, t, model_time=t))
      model.orientationRate.z, model.orientationRate.zStd = yaw, [sigma]*33
      row[f'current_speed_{sigma}'] = ceiling
      effective_ratio = ratio if .5 < ratio <= 1.2 else 1.0
      row[f'current_below_ego_{sigma}'] = ceiling < speed*3.6/effective_ratio - 1.0
      row[f'reference_accel_{sigma}'] = references[sigma].update(model, speed)
    if len(std) == 33 and np.isfinite(std).all() and min(std) >= 0:
      model.orientationRate.zStd = std
      row['measured_reference_accel'] = measured_ref.update(model, speed)
      model.orientationRate.z = (np.where(np.asarray(yaw)<0, -1., 1.)*(np.abs(yaw)+std)).tolist()
      result = curve_speed(model, speed, settings['AutoCurveSpeedFactor']*.01,
                           settings['AutoCurveSpeedLowerLimit'], speed_ratio=ratio, a_ego=accel)
      row['measured_current_speed'] = abs(measured_current.update(result, t, model_time=t))
    rows.append(row)
  summary = {'file': path.name, 'identity': identity, 'model_hashes': hashes, 'settings': settings,
             'settings_present': present, 'models': len(models), 'usable': len(rows), 'skipped': dict(skipped),
             'measured_std_frames': sum(len(m[9]) == 33 for m in models),
             'raw_prediction_frames': sum(m[10] > 0 for m in models), 'backends': dict(Counter(r['backend'] for r in rows))}
  return summary, rows


def summarize(rows):
  moving = [r for r in rows if r['speed_kph'] >= 10]
  result = {'frames': len(rows), 'moving_frames': len(moving), 'scenarios': {}}
  for s in SIGMAS:
    caps = [r[f'reference_accel_{s}'] for r in moving]
    lower = [r['current_speed_0.0']-r[f'current_speed_{s}'] for r in moving]
    result['scenarios'][str(s)] = {
      'current_below_ego_frames': sum(r[f'current_below_ego_{s}'] for r in moving),
      'current_lower_by_over_1kph': sum(v>1 for v in lower), 'current_speed_reduction_kph': metrics(lower),
      'near_straight_extra_current_limit': sum(
        r['near_straight_prediction'] and not r['current_below_ego_0.0'] and r[f'current_below_ego_{s}'] for r in moving),
      'near_straight_extra_reference_braking': sum(
        r['near_straight_prediction'] and r['reference_accel_0.0'] >= -.1 and r[f'reference_accel_{s}'] < -.1 for r in moving),
      'reference_negative_cap_frames': sum(v < -.1 for v in caps), 'reference_accel_cap': metrics(caps),
      'reference_more_restrictive_by_0_1': sum(r[f'reference_accel_{s}'] < r['reference_accel_0.0']-.1 for r in moving),
      'reference_below_recorded_accel': sum(r['recorded_accel'] is not None and r[f'reference_accel_{s}'] < r['recorded_accel']-.1 for r in moving),
    }
  return result


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('logs', nargs='+', type=Path)
  ap.add_argument('--output-dir', required=True, type=Path)
  args = ap.parse_args()
  args.output_dir.mkdir(parents=True, exist_ok=True)
  summaries, combined = [], []
  for path in args.logs:
    summary, rows = compare(path)
    summary['comparison'] = summarize(rows)
    summaries.append(summary)
    combined.extend(rows)
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with (args.output_dir/(path.stem+'.csv')).open('w', newline='') as stream:
      writer = csv.DictWriter(stream, fieldnames=fields)
      writer.writeheader()
      writer.writerows(rows)
    print(path.name, 'frames', summary['usable'], 'std', summary['measured_std_frames'], 'settings', summary['settings'], flush=True)
  groups = defaultdict(list)
  for row in combined:
    groups[row['backend']].append(row)
  result = {'limitations': ['Recorded-input open-loop comparison, not actual acceleration or road acceptance.',
                           'sigma=0.01/0.03/0.05 are hypothetical rad/s, not recovered model uncertainty.',
                           'Existing speed ceilings and new acceleration caps are different control interfaces.',
                           'Each supplied segment cold-starts the policy state; recordings are not an exhaustive route.'],
            'segments': summaries, 'total': summarize(combined), 'by_backend': {k: summarize(v) for k,v in groups.items()}}
  (args.output_dir/'summary.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
