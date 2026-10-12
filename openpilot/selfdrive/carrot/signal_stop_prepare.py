"""Experimental red pre-deceleration. No permission to depart or stop-line estimate.

Geometry rejects implausible housings; it cannot prove signal identity or lane
applicability. Metric ranges assume housing dimensions and mounting heights.
"""
from dataclasses import dataclass
from itertools import combinations
import math

import numpy as np


LANE_SIGNAL_MARGIN = 3.5  # metres outside EACH ego-lane boundary; experimental


def road_corridor(distances, edges, edge_stds, lane_lines=(), lane_probs=(), lane_stds=(), *, lane_margin=LANE_SIGNAL_MARGIN):
  """Interpolate trustworthy road/ego-lane boundaries, never the time-based plan.

  Expand the two ego-lane lines to include offset/adjacent overhead signals.
  This is a candidate search region, not proof of which lane a signal controls.
  Reliable road edges bound it with 0.3m inward clearance, or act as fallback.
  Check source agreement BEFORE expansion. Never extrapolate either curve.
  """
  if not math.isfinite(lane_margin) or not 0 <= lane_margin <= 5.:
    return None
  def pair(curves, confidences, lanes=False):
    try:
      if len(curves) != 2 or len(confidences) != 2 or not all(math.isfinite(s) and 0 <= s <= .5 for s in confidences):
        return None
      values = []
      for curve in curves:
        xs, ys, zs = [np.asarray(getattr(curve, k), dtype=float) for k in ('x', 'y', 'z')]
        if (len(xs) < 2 or xs.ndim != 1 or ys.shape != xs.shape or zs.shape != xs.shape
            or not all(np.isfinite(a).all() for a in (xs, ys, zs)) or np.any(np.diff(xs) <= 0)):
          return None
        values.append((np.interp(distances, xs, ys), np.interp(distances, xs, zs),
                       (distances >= xs[0]) & (distances <= xs[-1])))
      left, right = values
      width = right[0]-left[0]
      valid = left[2] & right[2] & (width >= 2.) & (width <= (5. if lanes else 20.))
      return left[0], right[0], (left[1]+right[1])/2, valid
    except (AttributeError, TypeError, ValueError, IndexError):
      return None

  road = pair(edges, edge_stds)
  if road is not None:
    road = (road[0]+.3, road[1]-.3, road[2], road[3])
  lane = None
  try:
    if (len(lane_lines) == len(lane_probs) == len(lane_stds) == 4
        and all(math.isfinite(lane_probs[i]) and .8 <= lane_probs[i] <= 1. for i in (1, 2))):
      lane = pair(list(lane_lines)[1:3], list(lane_stds)[1:3], lanes=True)
  except (TypeError, ValueError, IndexError):
    pass
  if lane is None:
    return None if road is None else (*road, 'edges')
  left, right, z, valid = lane
  if road is not None:
    # A reliable road corridor contradicting reliable lanes must not be ignored.
    valid &= road[3] & (np.maximum(left, road[0]) < np.minimum(right, road[1]))
  left, right = left-lane_margin, right+lane_margin
  if road is not None:
    left = np.maximum(left, road[0])
    right = np.minimum(right, road[1])
  return left, right, z, valid, 'lanes' if road is None else 'lanes+edges'


def housing_range(box, focal, rpy, edges, edge_stds, lane_lines=(), lane_probs=(), lane_stds=(), *, lane_margin=LANE_SIGNAL_MARGIN):
  """Return a plausible range, not a measured distance. Full housings only."""
  try:
    from openpilot.common.transformations.orientation import rot_from_euler
    x1, y1, x2, y2 = map(float, box)
    if not (all(math.isfinite(v) for v in (*box, focal, *rpy))
            and len(rpy) == 3 and focal > 0
            and 0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760
            and 16 <= x2-x1 <= 160 and 4 <= y2-y1 <= 50
            and 2.6 <= (x2-x1)/(y2-y1) <= 6.8):
      return None
    # Red lamp occupies the left end of the measured horizontal housing.
    ray = rot_from_euler(rpy).T @ np.array([1., (x1+.125*(x2-x1)-672)/focal, ((y1+y2)/2-380)/focal])
    if ray[0] <= 0:
      return None
    ray /= ray[0]
    lo, hi = focal*.7/(x2-x1), focal*2.5/(x2-x1)
    ds = np.linspace(lo, hi, 65)
    corridor = road_corridor(ds, edges, edge_stds, lane_lines, lane_probs, lane_stds, lane_margin=lane_margin)
    if corridor is None:
      return None
    left, right, road_z, valid, _ = corridor
    side = ray[1]*ds
    height = road_z-ray[2]*ds
    possible = (valid & (side >= left) & (side <= right)
                & (height >= 3.) & (height <= 9.))
    valid = ds[possible]
    if len(valid) < 3 or valid[0] < 25 or valid[-1] > 140:
      return None
    return float(valid[0]), float(valid[-1])
  except (AttributeError, TypeError, ValueError, IndexError):
    return None


def housing_alignment(box, distance, focal, rpy, edges, edge_stds, lane_lines=(), lane_probs=(), lane_stds=()):
  """Signed housing extent relative to ego-lane half-width over the FULL range.

  Zero is the projected ego-lane center; +/-1 are the unexpanded boundaries.
  Use both housing ends, not the illuminated lamp, to avoid color-dependent
  position. This is a geometric hypothesis, not signal-to-movement assignment.
  Road edges can constrain lanes, but road center alone cannot identify ego lane.
  """
  try:
    from openpilot.common.transformations.orientation import rot_from_euler
    x1, y1, x2, y2 = map(float, box)
    lo, hi = map(float, distance)
    if (len(rpy) != 3 or not all(math.isfinite(v) for v in (*box, lo, hi, focal, *rpy))
        or focal <= 0 or not 25 <= lo < hi <= 140
        or not (0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760)):
      return None
    # Include every source knot: lane interpolation is piecewise linear, so the
    # normalized lateral extrema occur at knots or range endpoints, not between.
    knots = [v for curve in (*edges, *lane_lines) for v in curve.x if lo < v < hi]
    ds = np.unique([lo, hi, *knots])
    corridor = road_corridor(ds, edges, edge_stds, lane_lines, lane_probs, lane_stds, lane_margin=0.)
    if corridor is None or corridor[-1] == 'edges' or not corridor[3].all():
      return None
    # Road clipping must not move the ego-lane center toward one edge.
    lane = road_corridor(ds, (), (), lane_lines, lane_probs, lane_stds, lane_margin=0.)
    if lane is None or not lane[3].all():
      return None
    left, right = lane[:2]
    half_width = (right-left)/2
    if np.any(half_width < 1.):
      return None
    rays = rot_from_euler(rpy).T @ np.array([
      [1., 1.], [(x1-672)/focal, (x2-672)/focal], [((y1+y2)/2-380)/focal]*2])
    if np.any(rays[0] <= 0):
      return None
    sides = rays[1]/rays[0]
    bounds = (sides[:, None]*ds-(left+right)/2)/half_width
    return float(bounds.min()), float(bounds.max())
  except (AttributeError, TypeError, ValueError, IndexError):
    return None


def central_red_priority(reds, greens, alignment):
  """Return stable support IDs only for clearly central red versus lateral green.

  Bands are experimental half-lane-width units, not learned probabilities.
  One red must fit +/-0.75; two distinct reds may fit +/-1.25. Every green must
  lie wholly outside +/-1.75 and at least0.5 farther out than the red support.
  """
  if alignment is None:
    return None
  red_extents = {}
  for tid, distance, box in reds:
    bounds = alignment(box, distance)
    if bounds is not None and len(bounds) == 2 and all(math.isfinite(v) for v in bounds) and bounds[0] <= bounds[1]:
      red_extents[tid] = max(abs(v) for v in bounds)
  support = tuple(sorted(tid for tid, extent in red_extents.items() if extent <= .75))
  kind = 'central_red'
  if not support:
    support = tuple(sorted(tid for tid, extent in red_extents.items() if extent <= 1.25))
    if len(support) < 2:
      return None
    kind = 'central_red_pair'
  extent = max(red_extents[tid] for tid in support)
  green_ids = []
  for tid, distance, box in greens:
    bounds = alignment(box, distance)
    if (bounds is None or len(bounds) != 2 or not all(math.isfinite(v) for v in bounds)
        or bounds[0] > bounds[1] or bounds[0] <= 0 <= bounds[1]):
      return None
    nearest = min(abs(v) for v in bounds)
    if nearest < 1.75 or nearest-extent < .5:
      return None
    for _, _, red_box in reds:
      if min(box[2], red_box[2]) > max(box[0], red_box[0]) and min(box[3], red_box[3]) > max(box[1], red_box[1]):
        return None
    green_ids.append(tid)
  if len(set(green_ids)) != len(green_ids) or set(green_ids) & {tid for tid, _, _ in reds}:
    return None
  return kind, support, tuple(sorted(green_ids))


@dataclass(frozen=True)
class PrepareDecision:
  active: bool = False
  entered: bool = False
  reason: str = 'disabled'
  track_id: int = -1
  distance_range: tuple | None = None
  track_ids: tuple = ()
  priority: str = 'none'
  priority_ids: tuple = ()
  side_green_ids: tuple = ()


class SignalStopPrepare:
  MAX_AGE = .2
  MAX_GAP = .25
  CONFIRM = .6
  TARGET_ACCEL = -.5
  RAMP = .25  # m/s^3; zero to -0.5 takes two seconds

  def __init__(self):
    self.session = None
    self.last_frame = -1
    self.last_capture = -math.inf
    self.histories = {}
    self.priority_key = None
    self.priority_since = None
    self.priority_count = 0
    self.active = False
    self.last_now = None
    self.cooldown = -math.inf
    self.last_sound = -math.inf

  def reset(self, reason):
    self.histories.clear()
    self.priority_key = None
    self.priority_since = None
    self.priority_count = 0
    self.active = False
    return PrepareDecision(reason=reason)

  def update(self, now, observation, *, enabled, valid, drive, speed, gas, brake,
             lead, turning, model_stopping, entry_allowed, geometry, alignment=None):
    if not math.isfinite(now) or (self.last_now is not None and now <= self.last_now):
      return self.reset('clock')
    self.last_now = now
    if gas or brake:
      self.cooldown = now+10.
    if (not enabled or not valid or not drive or gas or brake or lead or turning
        or model_stopping or not entry_allowed or not math.isfinite(speed)
        or not 5/3.6 < speed < 82/3.6 or now < self.cooldown):
      return self.reset('ineligible')
    try:
      stamp = float(observation['timestamp'])
      frame = int(observation['frame_id'])
      session = observation['session']
      if (observation.get('version') != 1 or observation.get('stream') != 'road'
          or observation.get('size') != [1344, 760] or not isinstance(session, str) or not session
          or not math.isfinite(stamp) or not 0 <= now-stamp <= self.MAX_AGE):
        return self.reset('stale')
      if session != self.session:
        self.reset('new_session')
        self.session = session
        self.last_frame, self.last_capture = -1, -math.inf
      if frame < self.last_frame or stamp < self.last_capture:
        return self.reset('reordered')
      fresh = frame != self.last_frame and stamp > self.last_capture
      if not fresh and (frame != self.last_frame or stamp != self.last_capture):
        return self.reset('inconsistent_frame')
      if fresh:
        if stamp-self.last_capture > self.MAX_GAP:
          self.reset('capture_gap')
        self.last_frame, self.last_capture = frame, stamp
      candidates = []
      greens = []
      for t in observation['tracks']:
        e = t['evidence']
        # Core proposals synthesize a box around one lamp; its width is NOT a
        # measured housing width. Templates alone cannot establish a new red.
        if (t['age'] != 0 or t['state'] not in ('red', 'green') or e['raw'] != t['state']
            or e.get('source') != 'detected' or t['observations'] < 3
            or not .5 <= float(e.get('quality', 0)) <= 1):
          continue
        distance = geometry(t['box'])
        if distance is None:
          continue
        if t['state'] == 'green':
          greens.append((int(t['id']), distance, t['box']))
        elif (.4 <= float(e.get('red_score', 0)) <= 1
              and 0 <= float(e.get('green_score', 1)) < .08):
          candidates.append((int(t['id']), distance, t['box']))
      if not 1 <= len(candidates) <= 4:
        return self.reset('no_red' if not candidates else 'too_many_red')
      ids = tuple(sorted(tid for tid, _, _ in candidates))
      distance = (max(d[0] for _, d, _ in candidates), min(d[1] for _, d, _ in candidates))
      if len(set(ids)) != len(ids) or distance[0] >= distance[1]:
        return self.reset('incompatible_red_group')
      # A common plausible distance plus a similar row/scale supports a group
      # hypothesis, NOT proof of common lane/direction. Reject overlapping boxes
      # (duplicate detections), distinct rows and very different apparent scales.
      for (_, _, a), (_, _, b) in combinations(candidates, 2):
        aw, bw = a[2]-a[0], b[2]-b[0]
        overlaps = min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])
        if (overlaps or max(aw, bw) > 2*min(aw, bw)
            or abs((a[1]+a[3]-b[1]-b[3])/2) > 2*max(a[3]-a[1], b[3]-b[1])):
          return self.reset('incompatible_red_group')
      priority = ('red_only', (), ())
      if greens:
        priority = central_red_priority(candidates, greens, alignment)
        if priority is None:
          return self.reset('mixed_colors')
      # The same central support AND lateral green identities must maintain
      # their separation over time; a one-frame geometry jump is insufficient.
      if priority != self.priority_key:
        self.priority_key = priority
        self.priority_since, self.priority_count = None, 0
      if fresh:
        if self.priority_since is None:
          self.priority_since = stamp
        self.priority_count += 1
      # Confirm each identity independently. Extra lamps do not multiply frame
      # counts, and a newly appearing lamp cannot inherit a vanished lamp's age.
      self.histories = {tid: h for tid, h in self.histories.items() if tid in ids}
      if fresh:
        for tid in ids:
          since, count = self.histories.get(tid, (stamp, 0))
          self.histories[tid] = (since, count+1)
      confirmed = [tid for tid, (since, count) in self.histories.items()
                   if count >= 4 and stamp-since >= self.CONFIRM-1e-9]
      tid = min(confirmed) if confirmed else ids[0]
      was_active = self.active
      self.active = bool(confirmed)
      if greens:
        self.active = (self.active and all(tid in confirmed for tid in priority[1])
                       and self.priority_count >= 4 and self.priority_since is not None
                       and stamp-self.priority_since >= self.CONFIRM-1e-9)
        tid = priority[1][0]
      entered = self.active and not was_active and now-self.last_sound >= 10.
      if entered:
        self.last_sound = now
      return PrepareDecision(self.active, entered, 'red_prepare' if self.active else 'confirming', tid, distance, ids, *priority)
    except (KeyError, TypeError, ValueError, OverflowError):
      return self.reset('malformed')


class PrepareAccel:
  """Final upper acceleration cap; stronger existing braking always wins."""
  def __init__(self):
    self.cap = None

  def apply(self, target, previous, dt, active):
    if not active or not all(math.isfinite(x) for x in (target, previous, dt)) or not 0 < dt <= .2:
      self.cap = None
      return target
    if self.cap is None:
      self.cap = max(SignalStopPrepare.TARGET_ACCEL, previous)
    self.cap = max(SignalStopPrepare.TARGET_ACCEL, self.cap-SignalStopPrepare.RAMP*dt)
    return min(target, self.cap)
