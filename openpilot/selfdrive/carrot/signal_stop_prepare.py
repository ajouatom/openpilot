"""Experimental red pre-deceleration. No permission to depart or stop-line estimate.

Geometry rejects implausible housings; it cannot prove signal identity or lane
applicability. Metric ranges assume housing dimensions and mounting heights.
"""
from dataclasses import dataclass
import math

import numpy as np


def housing_range(box, focal, rpy, edges, edge_stds, path):
  """Return a plausible range, not a measured distance. Full housings only."""
  try:
    from openpilot.common.transformations.orientation import rot_from_euler
    x1, y1, x2, y2 = map(float, box)
    if not (all(math.isfinite(v) for v in (*box, focal, *rpy, *edge_stds))
            and len(rpy) == 3 and len(edge_stds) == 2 and focal > 0
            and 0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760
            and 16 <= x2-x1 <= 160 and 4 <= y2-y1 <= 50
            and 2.6 <= (x2-x1)/(y2-y1) <= 6.8
            and all(0 <= s <= .5 for s in edge_stds)):
      return None
    # Red lamp occupies the left end of the measured horizontal housing.
    ray = rot_from_euler(rpy).T @ np.array([1., (x1+.125*(x2-x1)-672)/focal, ((y1+y2)/2-380)/focal])
    if ray[0] <= 0:
      return None
    ray /= ray[0]
    lo, hi = focal*.7/(x2-x1), focal*2.5/(x2-x1)
    ds = np.linspace(lo, hi, 65)
    values = []
    for curve in (*edges, path):
      xs, ys, zs = [np.asarray(getattr(curve, k), dtype=float) for k in ('x', 'y', 'z')]
      if (len(xs) < 2 or len(xs) != len(ys) or len(xs) != len(zs)
          or not all(np.isfinite(a).all() for a in (xs, ys, zs)) or np.any(np.diff(xs) <= 0)):
        return None
      values.append((np.interp(ds, xs, ys), np.interp(ds, xs, zs), (ds >= xs[0]) & (ds <= xs[-1])))
    left, right, center = values
    side = ray[1]*ds
    height = (left[1]+right[1])/2-ray[2]*ds
    possible = (left[2] & right[2] & center[2] & (left[0] < right[0])
                & (side >= left[0]+.3) & (side <= right[0]-.3)
                & (np.abs(side-center[0]) <= 2.) & (height >= 3.) & (height <= 9.))
    valid = ds[possible]
    if len(valid) < 3 or valid[0] < 25 or valid[-1] > 140:
      return None
    return float(valid[0]), float(valid[-1])
  except (AttributeError, TypeError, ValueError, IndexError):
    return None


@dataclass(frozen=True)
class PrepareDecision:
  active: bool = False
  entered: bool = False
  reason: str = 'disabled'
  track_id: int = -1
  distance_range: tuple | None = None


class SignalStopPrepare:
  MAX_AGE = .2
  MAX_GAP = .25
  CONFIRM = .6
  TARGET_ACCEL = -.5
  RAMP = .25  # m/s^3; zero to -0.5 takes two seconds

  def __init__(self):
    self.key = None
    self.last_frame = -1
    self.last_capture = -math.inf
    self.since = None
    self.count = 0
    self.active = False
    self.last_now = None
    self.cooldown = -math.inf
    self.last_sound = -math.inf

  def reset(self, reason):
    self.key = None
    self.since = None
    self.count = 0
    self.active = False
    return PrepareDecision(reason=reason)

  def update(self, now, observation, *, enabled, valid, drive, speed, gas, brake,
             lead, turning, model_stopping, entry_allowed, geometry):
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
      stamp = float(observation['timestamp']); frame = int(observation['frame_id'])
      session = observation['session']
      if (observation.get('version') != 1 or observation.get('stream') != 'road'
          or observation.get('size') != [1344, 760] or not isinstance(session, str) or not session
          or not math.isfinite(stamp) or not 0 <= now-stamp <= self.MAX_AGE):
        return self.reset('stale')
      candidates = []
      for t in observation['tracks']:
        e = t['evidence']
        # Core proposals synthesize a box around one lamp; its width is NOT a
        # measured housing width. Templates alone cannot establish a new red.
        if (t['age'] != 0 or t['state'] != 'red' or e['raw'] != 'red'
            or e.get('source') != 'detected' or t['observations'] < 3
            or not .5 <= float(e.get('quality', 0)) <= 1
            or not .4 <= float(e.get('red_score', 0)) <= 1
            or not 0 <= float(e.get('green_score', 1)) < .08):
          continue
        distance = geometry(t['box'])
        if distance is not None:
          candidates.append((int(t['id']), distance))
      # Ambiguous multiple housings do not select one arbitrarily.
      if len(candidates) != 1:
        return self.reset('no_unique_red')
      tid, distance = candidates[0]
      key = (session, tid)
      if key != self.key:
        self.reset('new_track')
        self.key = key
        self.last_frame = -1
        self.last_capture = -math.inf
      if frame < self.last_frame or stamp < self.last_capture:
        return self.reset('reordered')
      if frame != self.last_frame and stamp > self.last_capture:
        if stamp-self.last_capture > self.MAX_GAP:
          self.since, self.count, self.active = stamp, 0, False
        self.count += 1
        self.last_frame, self.last_capture = frame, stamp
      elif frame != self.last_frame or stamp != self.last_capture:
        return self.reset('inconsistent_frame')
      was_active = self.active
      self.active = self.count >= 4 and self.since is not None and stamp-self.since >= self.CONFIRM-1e-9
      entered = self.active and not was_active and now-self.last_sound >= 10.
      if entered:
        self.last_sound = now
      return PrepareDecision(self.active, entered, 'red_prepare' if self.active else 'confirming', tid, distance)
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
