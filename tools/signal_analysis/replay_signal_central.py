"""Compare central-selection variants on private decoded logs, offline only.

Usage: python -m tools.signal_analysis.replay_signal_central manifest.json output.json
Manifest: [{"path": ".../routes/segment/decoded.json.gz"}, ...].
Each decoded file maps topics to [monotonic arrival seconds, payload] rows;
events contains cloudlog objects with signalTrackingShadow in their msg field.
No videos/logs are distributed with this tool. Counts are not accuracy labels.
"""
import argparse
import bisect
from collections import Counter
from dataclasses import asdict
import gzip
import json
from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard
from tools.signal_analysis.signal_central_selection import CentralSignalSelector, RoadSignalRegion


def namespace(value):
  if isinstance(value, dict):
    return SimpleNamespace(**{k: namespace(v) for k, v in value.items()})
  if isinstance(value, list):
    return [namespace(v) for v in value]
  return value


class Timeline:
  def __init__(self, data):
    self.data = {k: sorted(rows, key=lambda r: r[0]) for k, rows in data.items()}
    self.times = {k: [t for t, _ in rows] for k, rows in self.data.items()}

  def latest(self, key, now, max_age):
    i = bisect.bisect_right(self.times.get(key, []), now)-1
    if i < 0 or not 0 <= now-self.times[key][i] <= max_age:
      return None
    return self.data[key][i][1]


def start_observation(record, suppressed=()):
  # Match publish_start_observation's non-neural, first-20 contract BEFORE
  # filtering. Never promote a previously untransported 21st candidate.
  return dict(timestamp=record['timestamp_eof']/1e9, frame_id=record['frame_id'], session=record['session'],
              tracks=[dict(box=t['box'], raw=t['evidence']['raw'] if t['age'] == 0 and t['observations'] >= 3 else 'unknown',
                           id=t['id'], seen_red=t.get('seen_red', False))
                      for t in record['tracks'][:20] if t['id'] not in suppressed])


def compare_segment(data):
  """Cold-start BOTH variants per segment; recorded arrivals, no future context.

  This is a comparison of the stopped guard, not a planner/MPC or moving
  preparation replay. Mode/driver overrides and unrecorded transport latency
  cannot be reconstructed as a vehicle outcome from these decoded rows.
  """
  timeline = Timeline(data)
  selectors = [CentralSignalSelector(image_fallback=True, preserve_green_rows=False),
               CentralSignalSelector(image_fallback=True),
               CentralSignalSelector(image_fallback=True, all_colours_trial=True)]
  count = Counter()
  observations, selections = [], []
  for arrival, event in timeline.data.get('events', []):
    record = event.get('msg')
    if not isinstance(record, dict) or record.get('event') != 'signalTrackingShadow' or record.get('neural'):
      continue
    capture = record['timestamp_eof']/1e9
    model = timeline.latest('modelV2', capture, .25)
    calib = timeline.latest('liveCalibration', capture, 2.)
    camera = timeline.latest('roadCameraState', capture, .25)
    region = None
    # These observer coordinates support only the OS04C10 reference image.
    if (model and model.get('valid') and calib and calib.get('valid') and calib.get('calStatus') == 'calibrated'
        and camera and camera.get('valid') and camera.get('sensor') == 'os04c10'):
      region = RoadSignalRegion(namespace(model), calib['rpyCalib'])
    results = [selector.update(record, region) for selector in selectors]
    count['observations'] += 1
    count['road_geometry'] += bool(region and region.ready)
    count['anchored'] += bool(results[1].anchor_ids)
    for track in record['tracks']:
      raw = track.get('evidence', {}).get('raw', 'unknown')
      if track.get('age') == 0:
        count['raw_'+raw] += 1
      for name, result in zip(('initial', 'revised', 'all_colours'), results, strict=True):
        if track['id'] in result.suppressed_ids:
          count[name+'_excluded_'+raw] += 1
          count[name+'_excluded_red_history'] += bool(track.get('seen_red'))
    observations.append((arrival, [start_observation(record), start_observation(record, results[1].suppressed_ids)]))
    selections.append(dict(frame=record['frame_id'], capture=capture, initial=list(results[0].suppressed_ids),
                           revised=list(results[1].suppressed_ids), all_colours=list(results[2].suppressed_ids),
                           anchors=list(results[1].anchor_ids)))
  guard_results = []
  obs_times = [t for t, _ in observations]
  for delay in (0., .02):
    guards = [SignalStartGuard(), SignalStartGuard()]
    counts = [Counter(), Counter()]
    differences, releases = [], [[], []]
    for now, plan in timeline.data.get('longitudinalPlan', []):
      i = bisect.bisect_right(obs_times, now-delay)-1
      if i < 0:
        continue
      inputs = [timeline.latest(k, now, .25) for k in ('carState', 'carControl', 'selfdriveState', 'modelV2', 'radarState')]
      if any(v is None for v in inputs):
        continue
      cs, cc, sd, _, _ = inputs
      context = dict(enabled=sd['enabled'] and cc['longActive'], valid=all(v.get('valid', False) for v in inputs) and cs['canValid'],
                     drive=cs['gearShifter'] == 'drive', gas=cs['gasPressed'], speed=cs['vEgo'],
                     departure=plan['trafficState'] == 2 or plan['hasLead'])
      decisions = [guard.update(now, obs, **context) for guard, obs in zip(guards, observations[i][1], strict=True)]
      for j, decision in enumerate(decisions):
        counts[j].update(frames=1, hold=int(decision.hold), green=int(decision.green_confirmed), release=int(decision.released))
        if decision.released:
          releases[j].append(now)
      if decisions[0] != decisions[1]:
        differences.append(dict(now=now, original=asdict(decisions[0]), revised=asdict(decisions[1])))
    guard_results.append(dict(delay=delay, counts=list(map(dict, counts)), releases=releases, differences=differences))
  return dict(counts=dict(count), selections=selections, guard=guard_results)


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('manifest', type=Path)
  parser.add_argument('output', type=Path)
  args = parser.parse_args()
  results = []
  seen = set()
  for entry in json.loads(args.manifest.read_text(encoding='utf-8-sig')):
    path = Path(entry['path'])
    if path.parent.name in seen:
      raise ValueError('duplicate segment '+path.parent.name)
    seen.add(path.parent.name)
    with gzip.open(path, 'rt', encoding='utf-8') as stream:
      result = compare_segment(json.load(stream))
    result['segment'] = path.parent.name
    results.append(result)
    print(json.dumps(dict(segment=result['segment'], **result['counts'],
                          guard_differences=[len(g['differences']) for g in result['guard']])), flush=True)
  args.output.write_text(json.dumps(results), encoding='utf-8')
  print(json.dumps(dict(total=dict(sum((Counter(r['counts']) for r in results), Counter())))))


if __name__ == '__main__':
  main()
