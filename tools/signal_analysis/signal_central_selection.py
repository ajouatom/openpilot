"""Road-centred candidate selection experiment; not a signal-to-lane classifier.

Not imported by the vehicle observer or planner. The conservative variant
rejects only peripheral green-first proposals while a
stable central signal is observed. Red and red-history tracks are never removed.
The all-colours variant is for offline comparison ONLY: geometry cannot prove
that a peripheral red belongs to another lane.
"""
from dataclasses import dataclass
import math

import numpy as np

from openpilot.common.transformations.orientation import rot_from_euler
from openpilot.selfdrive.carrot.signal_stop_prepare import road_corridor


class RoadSignalRegion:
  """A broad assumed overhead volume, not measured lamp range or stop distance."""
  def __init__(self, model, rpy, focal=1141.5):
    self.ready = False
    self.distances = np.arange(15., 161., 2.5)
    try:
      if len(rpy) != 3 or not all(math.isfinite(x) for x in (*rpy, focal)) or focal <= 0:
        return
      args = (self.distances, model.roadEdges, model.roadEdgeStds,
              model.laneLines, model.laneLineProbs, model.laneLineStds)
      self.inner = road_corridor(*args, lane_margin=0.)
      self.outer = road_corridor(*args, lane_margin=3.5)
      if self.inner is None or self.outer is None:
        return
      # A wide road centre is not an ego-lane estimate.
      self.usable = self.inner[3] & self.outer[3] & (self.inner[1]-self.inner[0] <= 12.)
      self.rotation, self.focal = rot_from_euler(rpy).T, focal
      self.ready = bool(self.usable.any())
    except (AttributeError, TypeError, ValueError, IndexError):
      pass

  def classify(self, box):
    """Return central / peripheral / unknown. Invalid geometry never excludes."""
    if not self.ready:
      return 'unknown'
    try:
      x1, y1, x2, y2 = box
      if not (all(math.isfinite(v) for v in box) and 0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760):
        return 'unknown'
      ray = self.rotation @ np.array([1., ((x1+x2)/2-672.)/self.focal, ((y1+y2)/2-380.)/self.focal])
      if ray[0] <= 0:
        return 'unknown'
      y, z = self.distances[None, :]*ray[1:, None]/ray[0]
      # Includes mounting uncertainty and a half-box margin. Do not infer size
      # or range from night-core boxes, which are synthetic housings.
      margin = self.distances*(x2-x1)/(2*self.focal)
      height_margin = self.distances*(y2-y1)/(2*self.focal)
      plausible = self.usable & (self.inner[2]-z >= 4.-height_margin) & (self.inner[2]-z <= 8.+height_margin)
      if not plausible.any():
        # No height/range support is ambiguous, not proof of a roadside object.
        return 'unknown'
      for name, corridor in [('central', self.inner), ('adjacent', self.outer)]:
        if np.any(plausible & (y >= corridor[0]-margin) & (y <= corridor[1]+margin)):
          return name
      return 'peripheral'
    except (TypeError, ValueError, IndexError):
      return 'unknown'


@dataclass(frozen=True)
class CentralSelection:
  selected_ids: tuple
  suppressed_ids: tuple
  anchor_ids: tuple
  reason: str


class CentralSignalSelector:
  CONFIRM = .35
  MAX_GAP = .25
  DARK_HOLD = .5

  def __init__(self, *, all_colours_trial=False, image_fallback=False, preserve_green_rows=True):
    self.all_colours_trial = all_colours_trial
    self.image_fallback = image_fallback
    self.preserve_green_rows = preserve_green_rows
    self.reset()

  def reset(self):
    self.session = None
    self.stamp = -math.inf
    self.anchors = {}
    self.previous = None

  @staticmethod
  def green_row_member(track, tracks):
    """Keep distinct, similarly sized current greens on the same overhead row.

    This only prevents exclusion: it never confirms green or grants departure.
    Overlapping duplicate boxes and adjacent letters cannot form a pair.
    """
    a = track['box']
    aw, ah = a[2]-a[0], a[3]-a[1]
    if track.get('age') != 0 or track.get('evidence', {}).get('raw') != 'green':
      return False
    for other in tracks:
      if other['id'] == track['id'] or other.get('age') != 0 or other.get('evidence', {}).get('raw') != 'green':
        continue
      b = other['box']
      bw, bh = b[2]-b[0], b[3]-b[1]
      dx = abs((a[0]+a[2]-b[0]-b[2])/2)
      if (min(aw, ah, bw, bh) > 0 and .5 <= aw/bw <= 2 and .5 <= ah/bh <= 2
          and abs((a[1]+a[3]-b[1]-b[3])/2) <= max(8., .75*max(ah, bh))
          and 1.5*max(aw, bw) <= dx <= min(400., 12*max(aw, bw))):
        return True
    return False

  def update(self, record, region):
    stamp = record['timestamp_eof']/1e9
    session = record['session']
    tracks = record['tracks']
    ids = tuple(t['id'] for t in tracks)
    if session == self.session and stamp == self.stamp and self.previous is not None:
      return self.previous
    if not math.isfinite(stamp) or session != self.session or not 0 < stamp-self.stamp <= self.MAX_GAP:
      self.anchors.clear()
    self.session, self.stamp = session, stamp
    image_only = region is None or not region.ready
    if not math.isfinite(stamp) or (image_only and not self.image_fallback):
      self.anchors.clear()
      self.previous = CentralSelection(ids, (), (), 'geometry_or_time')
      return self.previous
    def classify(box):
      if not image_only:
        return region.classify(box)
      x1, y1, x2, y2 = box
      if not (all(math.isfinite(v) for v in box) and 0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760):
        return 'unknown'
      return 'central' if .35*1344 <= (x1+x2)/2 <= .65*1344 and .08*760 <= (y1+y2)/2 <= .52*760 else 'adjacent'
    classes = {t['id']: classify(t['box']) for t in tracks if t.get('age') == 0}
    anchors = {}
    for t in tracks:
      ident = t['id']
      previous = self.anchors.get(ident)
      if classes.get(ident) not in ('central', 'adjacent'):
        continue
      raw = t.get('evidence', {}).get('raw', 'unknown')
      confirmed = (classes[ident] == 'central' and t.get('observations', 0) >= 3
                   and t.get('state') == raw and (raw == 'red' or (raw == 'green' and t.get('seen_red'))))
      if confirmed:
        anchors[ident] = (previous[0] if previous else stamp, stamp)
      elif previous and stamp-previous[1] <= self.DARK_HOLD:
        anchors[ident] = previous
    self.anchors = anchors
    stable = tuple(ident for ident, (since, last) in anchors.items() if last-since >= self.CONFIRM)
    boxes = [t['box'] for t in tracks if t['id'] in stable]
    def peripheral(t):
      if not image_only:
        return classes.get(t['id']) == 'peripheral'
      if classes.get(t['id']) == 'unknown' or t.get('age') != 0:
        return False
      x1, _, x2, _ = t['box']
      # Both sides of a stable central group remain eligible. Only clearly
      # separate side objects are excluded; multiple central heads stay intact.
      return bool(boxes) and all(x2 < b[0]-max(80., 2*(b[2]-b[0]), 2*(x2-x1)) or
                                x1 > b[2]+max(80., 2*(b[2]-b[0]), 2*(x2-x1)) for b in boxes)
    suppressed = tuple(t['id'] for t in tracks if stable and peripheral(t)
                       and (self.all_colours_trial or (t.get('evidence', {}).get('raw') == 'green' and not t.get('seen_red')
                            and not (self.preserve_green_rows and self.green_row_member(t, tracks)))))
    self.previous = CentralSelection(tuple(i for i in ids if i not in suppressed), suppressed, stable,
                                     ('image_central' if image_only else 'road_central') if stable else 'no_stable_central')
    return self.previous
