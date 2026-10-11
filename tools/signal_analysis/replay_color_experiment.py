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


def run(routes, output):
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
  for video in segments:
    segment = video.parent.name
    route, number = segment.rsplit('--', 1)
    number = int(number)
    if previous_segment != (route, number-1):
      trackers = [SignalTracker(daytime_cores=True, robust_tracking=True, conservative_green=enabled) for enabled in (False, True)]
    previous_segment = route, number
    samples = samples_by_segment[segment]
    stats = [Counter(), Counter()]
    timings = [[], []]
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
          # Alternate order to avoid always giving one variant the warm cache.
          results = [None, None]
          for variant in (index % 2, 1-index % 2):
            start = time.perf_counter()
            result = trackers[variant].process(rgb, record['timestamp_eof']/1e9)
            timings[variant].append((time.perf_counter()-start)*1000)
            fresh = [t for t in result['tracks'] if t['age'] == 0]
            stats[variant].update(t['evidence']['raw'] for t in fresh)
            stats[variant]['green_frames'] += any(t['evidence']['raw'] == 'green' for t in fresh)
            results[variant] = result
          stream.write(json.dumps(dict(segment=segment, index=index, frame_id=record['frame_id'],
                                       timestamp=record['timestamp_eof']/1e9, session=record['session'],
                                       recorded_age_ms=record['result_age_ms'], baseline=results[0], candidate=results[1]))+'\n')
          processed.add(index)
        index += 1
    capture.release()
    if processed != set(samples):
      raise RuntimeError(f'missing encoded frames: {segment}: {set(samples)-processed}')
    summary = dict(segment=segment, samples=len(processed), decoded_frames=index,
                   baseline=dict(stats[0]), candidate=dict(stats[1]),
                   desktop_ms=[dict(mean=float(np.mean(v)), p95=float(np.percentile(v, 95))) if v else {} for v in timings])
    summaries.append(summary)
    (output/'summary.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
    print(json.dumps(summary), flush=True)
  return summaries


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('routes', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  run(args.routes, args.output)
