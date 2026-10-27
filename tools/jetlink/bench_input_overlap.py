#!/usr/bin/env python3
"""Parked isolated QCOM warp/context A/B/A. Never sends model/control outputs."""
import argparse
from functools import partial
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace as NS

import numpy as np


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--source', type=Path, required=True, help='Candidate jetlink/model.py, outside the live checkout')
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--frames', type=int, default=100)
  args = parser.parse_args()
  if args.frames < 20 or args.output.exists():
    parser.error('Use at least 20 frames and a new output path')
  # Both this process and QCOM context have lower priority than live inference.
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  os.sched_setaffinity(0, os.sched_getaffinity(0) & {0, 1, 2, 3})
  os.nice(15)
  os.environ.update(DEV='QCOM', WARP_DEV='QCOM', QCOM_PRIORITY='15')
  from openpilot.cereal import messaging
  from openpilot.selfdrive.modeld.jetlink.inputs import fill_inputs
  from openpilot.selfdrive.modeld.jetlink.contracts import JETSON_SPEC
  from openpilot.selfdrive.modeld.jetlink.warp import validation_matrices
  sm = messaging.SubMaster(['carState', 'selfdriveState', 'carControl'])
  def guard():
    sm.update(100)
    now = time.monotonic()
    if not all(sm.valid[k] and sm.alive[k] and 0 <= now - sm.recv_time[k] < .25 for k in sm.services):
      raise RuntimeError('Fresh vehicle state required')
    cs, sd = sm['carState'], sm['selfdriveState']
    if str(cs.gearShifter) != 'park' or not cs.standstill or cs.vEgo != 0 or sd.enabled or sd.active or sm['carControl'].enabled:
      raise RuntimeError('Park/inactive required')
  for _ in range(20):
    sm.update(100)
  guard()
  module_spec = importlib.util.spec_from_file_location('candidate_jetlink_model', args.source)
  candidate = importlib.util.module_from_spec(module_spec)
  module_spec.loader.exec_module(candidate)
  warp = candidate.Warp(1344, 760)
  # Compilation can take seconds. Refresh every subscribed service before
  # evaluating its freshness; a single poll may deliver only one service.
  for _ in range(20):
    sm.update(100)
  guard()
  rng = np.random.default_rng(81014)
  bufs = {key: NS(data=rng.integers(0, 256, warp.size, np.uint8)) for key in ('img', 'big_img')}
  spec = JETSON_SPEC
  packed = np.zeros(spec.packed_nelem, np.float32)
  views = {k: a.reshape(shape) for (k, shape), a in zip(spec.packed_shapes.items(),
           np.split(packed, np.cumsum(spec.packed_sizes[:-1])), strict=True)}
  matrices = validation_matrices()
  hashes, variants = [], {}
  for name, overlap in (('before', False), ('overlap', True), ('after', False)):
    previous = np.zeros(8, np.float32)
    packed.fill(0)
    rows, observed = [], []
    for i in range(args.frames):
      guard()
      matrix = np.asarray(matrices[i % len(matrices)], np.float32)
      transforms = {'img': matrix, 'big_img': matrix.copy()}
      inputs = {'desire_pulse': np.eye(8, dtype=np.float32)[i % 8],
                'traffic_convention': np.array([1, 0], np.float32), 'action_t': np.array([.2, .3], np.float32)}
      if i % 3:
        inputs['nav_map'] = (np.arange(65536, dtype=np.uint8) + i).reshape(256, 256)
        inputs['navigation'] = np.full(47, i / 100, np.float32)
      started = time.perf_counter()
      if overlap:
        image = warp(bufs, transforms, prepare=partial(fill_inputs, views, inputs, previous))
      else:
        fill_inputs(views, inputs, previous)
        image = warp(bufs, transforms)
      elapsed = time.perf_counter() - started
      observed.append(hashlib.sha256(image.tobytes() + packed.tobytes() + previous.tobytes()).hexdigest())
      stages = (warp.stage_timings if overlap else
                dict(zip(('warp_prepare_ms', 'warp_submit_ms', 'input_prepare_ms', 'warp_read_ms'),
                         (warp.timings[0] * 1000, warp.timings[1] * 1000, 0., warp.timings[2] * 1000), strict=True)))
      rows.append([elapsed * 1000, *stages.values()])
      time.sleep(max(0, .05 - (time.perf_counter() - started)))
    if hashes and observed != hashes:
      raise RuntimeError('Image/context/desire parity failed')
    hashes = observed
    values = np.asarray(rows[10:])
    variants[name] = {key: {'mean_ms': float(values[:, j].mean()), 'p99_ms': float(np.percentile(values[:, j], 99)),
                            'max_ms': float(values[:, j].max())}
                      for j, key in enumerate(('total', 'warp_prepare', 'warp_submit', 'input_prepare', 'warp_read'))}
  guard()
  result = {'scope': 'parked isolated synthetic QCOM warp and context, no USB/model/control publication',
            'frames_per_variant': args.frames, 'all_images_context_desire_exact': True,
            'source_sha256': hashlib.sha256(args.source.read_bytes()).hexdigest(), 'variants': variants}
  with args.output.open('x') as out:
    json.dump(result, out, indent=2)
  print(json.dumps(result))


if __name__ == '__main__':
  main()
