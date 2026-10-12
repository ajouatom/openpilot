"""Paired offline replay of local decoded.json.gz/fcamera.hevc segments.

Uses exact roadEncodeIdx frame mapping and recorded observer capture cadence.
No network, Params, messaging or vehicle output. Does not simulate vehicle motion.
Decoded input is the private signal investigation format, not arbitrary rlogs.
"""
import argparse
from collections import Counter
import gzip
import json
from pathlib import Path
import time

import cv2
import numpy as np

from signal_tracker import SignalTracker
from signal_motion_gate import GreenMotionGate, GreenPoseGate


def run(routes, output, motion_dir=None, pose_dir=None):
  output.mkdir(parents=True, exist_ok=True)
  cv2.setNumThreads(1)
  summaries = []
  trackers = None
  previous_segment = None
  segments = sorted(routes.glob('*/fcamera.hevc'), key=lambda p: (p.parent.name.rsplit('--', 1)[0], int(p.parent.name.rsplit('--', 1)[1])))
  # A result may be logged in the next segment after its captured image. Join
  # across available segments of the same route before walking encoded frames.
  decoded = {p.parent.name: json.loads(gzip.decompress((p.parent/'decoded.json.gz').read_bytes())) for p in segments}
  indexes = {}
  samples_by_segment = {name: {} for name in decoded}
  for name, data in decoded.items():
    route = name.rsplit('--', 1)[0]
    for _, record in data['roadEncodeIdx']:
      key = (route, record['frameId'])
      if key in indexes:
        raise ValueError(f'duplicate encoded frame: {key}')
      indexes[key] = (name, record['segmentId'])
  unmapped = []
  for name, data in decoded.items():
    route = name.rsplit('--', 1)[0]
    for _, record in data['daytime']:
      target = indexes.get((route, record['frame_id']))
      if target is None:
        unmapped.append(dict(log_segment=name, frame_id=record['frame_id']))
        continue
      target_name, index = target
      if index in samples_by_segment[target_name]:
        raise ValueError(f'duplicate sample at {target_name}:{index}')
      samples_by_segment[target_name][index] = record
  (output/'unmapped.json').write_text(json.dumps(unmapped, indent=2), encoding='utf-8')
  motion_by_route = {}
  if motion_dir is not None:
    for name in decoded:
      rows = json.loads(gzip.decompress((motion_dir/(name+'.json.gz')).read_bytes()))
      motion_by_route.setdefault(name.rsplit('--', 1)[0], []).extend(rows)
    for rows in motion_by_route.values():
      rows.sort(key=lambda r: r['timestamp'])
  variants = ['baseline', 'candidate'] + (['motion_candidate'] if motion_dir is not None else [])
  pose_by_route = {}
  if pose_dir is not None:
    variants.append('pose_candidate')
    for name in decoded:
      rows = json.loads(gzip.decompress((pose_dir/(name+'.json.gz')).read_bytes()))
      pose_by_route.setdefault(name.rsplit('--', 1)[0], []).extend(rows)
    for rows in pose_by_route.values():
      rows.sort(key=lambda r: r['timestamp'])
  for video in segments:
    segment = video.parent.name
    route, number = segment.rsplit('--', 1)
    number = int(number)
    if previous_segment != (route, number-1):
      trackers = [SignalTracker(daytime_cores=True, robust_tracking=True, conservative_green=enabled) for enabled in (False, True)]
      if motion_dir is not None:
        trackers.append(SignalTracker(daytime_cores=True, robust_tracking=True, conservative_green=True))
        gate = GreenMotionGate()
        motion_index = 0
      if pose_dir is not None:
        trackers.append(SignalTracker(daytime_cores=True, robust_tracking=True, conservative_green=True))
        pose_gate = GreenPoseGate()
        pose_index = 0
    previous_segment = route, number
    samples = samples_by_segment[segment]
    stats = [Counter() for _ in variants]
    timings = [[] for _ in variants]
    capture = cv2.VideoCapture(str(video))
    processed = set()
    with gzip.open(output/(segment+'.jsonl.gz'), 'wt', encoding='utf-8') as stream:
      index = 0
      while capture.grab():
        if index in samples:
          ok, bgr = capture.retrieve()
          if not ok:
            raise RuntimeError(f'cannot decode {segment}:{index}')
          record = samples[index]
          rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
          capture_time = record['timestamp_eof']/1e9
          allow_green = True
          if motion_dir is not None:
            rows = motion_by_route[route]
            while motion_index < len(rows) and rows[motion_index]['timestamp'] <= capture_time:
              sample = rows[motion_index]
              gate.update(sample['timestamp'], sample['angle'], sample['rate'], sample['speed'], sample['valid'])
              motion_index += 1
            allow_green = gate.allows_green(capture_time)
          pose_allows_green = True
          if pose_dir is not None:
            rows = pose_by_route[route]
            while pose_index < len(rows) and rows[pose_index]['timestamp'] <= capture_time:
              sample = rows[pose_index]
              pose_gate.update(sample['timestamp'], sample['sample_time'], sample['yaw'], sample['yaw_std'], sample['valid'])
              pose_index += 1
            pose_allows_green = pose_gate.allows_green(capture_time)
          # Alternate order to avoid always giving one variant the warm cache.
          results = [None for _ in variants]
          order = [(index+j) % len(variants) for j in range(len(variants))]
          for variant in order:
            start = time.perf_counter()
            permitted = {'motion_candidate': allow_green, 'pose_candidate': pose_allows_green}.get(variants[variant], True)
            kwargs = dict(allow_green=permitted) if variants[variant] in ('motion_candidate', 'pose_candidate') else {}
            result = trackers[variant].process(rgb, capture_time, **kwargs)
            timings[variant].append((time.perf_counter()-start)*1000)
            fresh = [t for t in result['tracks'] if t['age'] == 0]
            stats[variant].update(t['evidence']['raw'] for t in fresh)
            stats[variant]['green_frames'] += any(t['evidence']['raw'] == 'green' for t in fresh)
            if variants[variant] in ('motion_candidate', 'pose_candidate'):
              stats[variant]['green_blocked_frames'] += not permitted
            results[variant] = result
          stream.write(json.dumps(dict(segment=segment, index=index, frame_id=record['frame_id'],
                                       timestamp=record['timestamp_eof']/1e9, session=record['session'],
                                       recorded_age_ms=record['result_age_ms'],
                                       **dict(zip(variants, results, strict=True)),
                                       **(dict(motion_allows_green=allow_green) if motion_dir is not None else {}),
                                       **(dict(pose_allows_green=pose_allows_green) if pose_dir is not None else {})))+'\n')
          processed.add(index)
        index += 1
    capture.release()
    if processed != set(samples):
      raise RuntimeError(f'missing encoded frames: {segment}: {set(samples)-processed}')
    summary = dict(segment=segment, samples=len(processed), decoded_frames=index,
                   **{v: dict(stats[i]) for i, v in enumerate(variants)},
                   desktop_ms=[dict(mean=float(np.mean(v)), p95=float(np.percentile(v, 95))) if v else {} for v in timings])
    summaries.append(summary)
    (output/'summary.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
    print(json.dumps(summary), flush=True)
  return summaries


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('routes', type=Path)
  parser.add_argument('output', type=Path)
  parser.add_argument('--motion-dir', type=Path, help='Optional per-segment .json.gz carState extraction for a third steering-gated tracker')
  parser.add_argument('--pose-dir', type=Path, help='Optional per-segment .json.gz calibrated livePose extraction for a yaw-gated tracker')
  args = parser.parse_args()
  run(args.routes, args.output, args.motion_dir, args.pose_dir)
