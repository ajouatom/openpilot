"""Experimental 3s lane-local paint map, indexed by ego travel, not frame votes."""
import math

from openpilot.selfdrive.carrot.lane_type.native import _runs

BIN_M = .25
WINDOW_NS = 3_000_000_000


class DistanceEvidence:
  def __init__(self):
    self.state = None

  def reset(self):
    self.state = None

  def update(self, profile, boundary, speed, stamp, *, lane_change=False, tracked=True):
    unknown = {'type': 'unknown', 'color': 'unknown', 'typeConfidence': 0., 'colorConfidence': 0.,
      'reason': 'insufficient_distinct_distance_evidence', 'confidenceKind': 'experimental_evidence_gate_not_probability',
      'trackingReset': False, 'observedDistinctM': 0., 'accumulatedTravelM': 0., 'conflictBins': 0,
      'dashedPatternObserved': False}
    valid = (tracked and not lane_change and isinstance(stamp, int) and not isinstance(stamp, bool) and
             len(boundary or []) == 2 and all(math.isfinite(x) for x in boundary) and
             math.isfinite(speed) and speed >= 0)
    if not valid:
      self.reset()
      return dict(unknown, trackingReset=True, reason='lane_change_or_tracking_lost')
    prev = self.state
    reset = prev and (not 0 < stamp-prev['stamp'] <= 300_000_000 or
                     max(abs(a-b) for a, b in zip(boundary, prev['boundary'], strict=True)) > .35)
    if reset:
      self.reset()
      prev = None
    if prev is None:
      prev = {'stamp': stamp, 'boundary': boundary, 'speed': speed, 'travel': 0., 'bins': {}}
    else:
      prev['travel'] += .5*(speed+prev['speed'])*(stamp-prev['stamp'])/1e9
    bins = prev['bins']
    for key in list(bins):
      if stamp-bins[key]['stamp'] > WINDOW_NS:
        del bins[key]
    # Collapse ALL native rows of this frame into one observation per physical
    # bin before any history update. Overlapping frames never increase length.
    frame_bins = {}
    for row in profile:
      if row['ink'] is None or row.get('confidence', 0) < .8 or not 4.5 <= row['distance'] <= 25.01:
        continue
      key = math.floor((prev['travel']+row['distance'])/BIN_M)
      entry = frame_bins.setdefault(key, {'ink': set(), 'color': set(), 'distance': row['distance']})
      entry['ink'].add(row['ink'])
      if row['color']:
        entry['color'].add(row['color'])
    for key, row in frame_bins.items():
      ink = next(iter(row['ink'])) if len(row['ink']) == 1 else None
      color = next(iter(row['color'])) if len(row['color']) == 1 else None
      old = bins.get(key)
      if old and old['ink'] != ink:
        ink = None  # contradictory physical observation is UNKNOWN, not a vote
      if old and old['color'] and old['color'] != color:
        color = None
      near_color = old.get('near_color') if old else None
      near_stamp = old.get('near_stamp') if old else None
      if row['distance'] <= 12 and ink == 1 and color:
        near_color, near_stamp = color, stamp
      if old and old.get('near_color') and color and old['near_color'] != color:
        near_color, near_stamp = None, None
      bins[key] = {'ink': ink, 'color': color, 'stamp': stamp, 'near_color': near_color, 'near_stamp': near_stamp}
    prev.update(stamp=stamp, boundary=boundary, speed=speed)
    self.state = prev
    unknown.update(trackingReset=bool(reset), accumulatedTravelM=prev['travel'],
                   conflictBins=sum(row['ink'] is None for row in bins.values()))
    # Current visibility is required. Old evidence cannot repeatedly authorize
    # a hidden current boundary while the display's 0.5s hold is expiring.
    known_now = [r for r in profile if r['ink'] is not None and r['distance'] <= 20]
    if len(known_now) < 12 or len(known_now)/max(1, sum(r['distance'] <= 20 for r in profile)) < .7:
      return dict(unknown, reason='current_boundary_insufficient')
    if not bins:
      return unknown
    ordered = [{'distance': key*BIN_M, 'ink': bins.get(key, {}).get('ink')}
               for key in range(min(bins), max(bins)+1)]
    runs = _runs(ordered)
    distinct = sum(v['ink'] is not None for v in bins.values())*BIN_M
    unknown.update(observedDistinctM=distinct, physicalPattern=runs)
    kind = 'unknown'
    if runs['repeatingPatternObserved'] and distinct >= 18:
      kind = 'dashed'
    elif distinct >= 22 and runs['knownFraction'] >= .98 and runs['coverage'] >= .98:
      kind = 'solid'
    # Nearest actual paint only. A far type crop cannot silently decide colour.
    near = [v for v in bins.values() if v['ink'] == 1 and v['near_color'] and
            v['near_stamp'] is not None and 0 <= stamp-v['near_stamp'] <= WINDOW_NS]
    colors = [v['near_color'] for v in near]
    color = 'unknown'
    if len(colors) >= 3:
      yellow = colors.count('yellow')/len(colors)
      color = 'yellow' if yellow >= .97 else 'white' if yellow <= .03 else 'unknown'
    result = dict(unknown, type=kind, color=color, typeConfidence=.99 if kind != 'unknown' else 0.,
      colorConfidence=.99 if color != 'unknown' else 0.,
      dashedPatternObserved=kind == 'dashed', reason='distance_aligned_native_evidence')
    if kind != 'unknown':
      result.update(typeMonoTimeNanos=stamp, dashedPatternMonoTimeNanos=stamp)
    if color != 'unknown':
      result['colorMonoTimeNanos'] = max(v['near_stamp'] for v in near)
    return result
