"""Opt-in stopped-car red veto, separate from moving signal assistance.

Image position is only a forward-signal hypothesis, not ego-lane association.
Unknown/stale observations cannot release a latched stop. Green only removes
this veto; the original planner still decides whether to go.
"""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import time

from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime, requested, MAX_BYTES

ENABLE_PATH = Path('/data/signal-color-shadow/start_guard_enabled')
OBSERVATION_PATH = Path('/dev/shm/carrot_signal_start_observation.json')


def publish_start_observation(record, directory=ENABLE_PATH.parent, path=OBSERVATION_PATH):
  if not requested(Path(directory) / ENABLE_PATH.name):
    return
  tracks = []
  for t in record['tracks'][:20]:
    raw = t.get('evidence', {}).get('raw', 'unknown')
    if record.get('neural'):
      labels = t.get('label', '').split(',')
      raw = ('red' if 'red' in labels else 'green' if labels == ['green'] else 'unknown')
      score = t.get('score', 0)
      if not math.isfinite(score) or not .8 <= score <= 1. or not 0 <= t.get('neural_age', math.inf) <= .5:
        raw = 'unknown'
    elif t.get('observations', 0) < 3:
      raw = 'unknown'
    if t.get('age') != 0:
      raw = 'unknown'
    tracks.append(dict(box=t['box'], raw=raw))
  obj = dict(version=1, stream='road', size=[1344, 760], session=record['session'],
             frame_id=record['frame_id'], timestamp=record['timestamp_eof']/1e9, tracks=tracks)
  data = json.dumps(obj, allow_nan=False)
  if len(data.encode()) > MAX_BYTES:
    raise ValueError('start observation too large')
  path = Path(path)
  tmp = path.with_suffix('.tmp')
  tmp.write_text(data)
  tmp.replace(path)


@dataclass(frozen=True)
class StartDecision:
  hold: bool = False
  warn: bool = False
  released: bool = False
  reason: str = 'disabled'


class SignalStartGuard:
  # Distinct from the old moving assist's 200 ms deadline: measured QCOM
  # capture-to-result is 335 ms mean / 400 ms max in the parked trial.
  MAX_AGE = .65
  MAX_GAP = .7

  def __init__(self):
    self.reset()
    self.last_now = -math.inf
    self.cooldown = -math.inf

  def reset(self):
    self.hold = False
    self.warned = False
    self.session = None
    self.box = None
    self.frame = -1
    self.timestamp = -math.inf
    self.pending = None
    self.since = 0.
    self.count = 0

  @staticmethod
  def front(box):
    try:
      a, b, c, d = box
      return (all(math.isfinite(v) for v in box) and 0 <= a < c <= 1344 and 0 <= b < d <= 760
              and .3 <= (a+c)/2688 <= .75 and .08 <= (b+d)/1520 <= .55)
    except (TypeError, ValueError):
      return False

  @staticmethod
  def matches(a, b):
    intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    area_a, area_b = (a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1])
    return intersection / (area_a+area_b-intersection) >= .6

  def observe(self, now, observation):
    if not observation:
      return 'missing'
    try:
      ts, fid, session = observation['timestamp'], observation['frame_id'], observation['session']
      if not math.isfinite(ts) or not 0 <= now-ts <= self.MAX_AGE:
        self.pending, self.count = None, 0
        return 'stale_or_future'
      if not isinstance(fid, int) or not isinstance(session, str) or not session:
        raise ValueError('invalid identity')
      if session != self.session:
        # A restart cannot inherit green release rights; reacquire red first.
        self.session, self.box = session, None
        self.timestamp, self.frame = -math.inf, -1
        self.pending, self.count = None, 0
      if ts <= self.timestamp or fid <= self.frame:
        return 'repeated_or_reordered'
      gap = ts-self.timestamp
      self.timestamp, self.frame = ts, fid
      tracks = [t for t in observation['tracks'] if self.front(t['box'])]
      matches = [t for t in tracks if self.box is not None and self.matches(self.box, t['box'])]
      red = [t for t in tracks if t['raw'] == 'red']
      if self.box is None or (not matches and red):
        selected = min(red, key=lambda t: abs((t['box'][0]+t['box'][2])/2688-.5)) if red else None
        if selected is not None:
          self.box = selected['box']
          self.pending, self.count = None, 0
      else:
        # Ambiguous overlapping objects, another red, or a lost housing must
        # not grant green release. Keep the fixed red anchor while stopped.
        red_matches = [t for t in matches if t['raw'] == 'red']
        selected = red_matches[0] if red_matches else matches[0] if len(matches) == 1 else None
      color = selected['raw'] if selected is not None else 'unknown'
      if color == 'red':
        # Settle camera pitch/box jitter using fresh red only. A green box
        # cannot walk the reference onto a neighboring lamp.
        self.box = selected['box']
      if color == 'green' and red:
        color = 'unknown'
      if color not in ('red', 'green') or color != self.pending or gap > self.MAX_GAP:
        self.pending, self.since, self.count = color, ts, 1
      else:
        self.count += 1
      if color == 'red' and self.count >= 2 and ts-self.since >= .15-1e-9:
        return 'red'
      if color == 'green' and self.count >= 3 and ts-self.since >= .7-1e-9:
        return 'green'
      return 'unconfirmed'
    except (KeyError, TypeError, ValueError, OverflowError):
      self.pending, self.count = None, 0
      return 'malformed'

  def update(self, now, observation, *, enabled, valid, drive, gas, speed, departure):
    if not enabled or not drive:
      self.reset()
      self.last_now = now if math.isfinite(now) else -math.inf
      return StartDecision()
    if gas:
      self.reset()
      self.last_now = now if math.isfinite(now) else -math.inf
      self.cooldown = now+10. if math.isfinite(now) else math.inf
      return StartDecision(reason='driver_gas')
    if not math.isfinite(now) or now < self.last_now:
      # Clock discontinuity cannot turn a held red into permission to go.
      self.pending, self.count, self.box, self.session = None, 0, None, None
      return StartDecision(self.hold, reason='clock_discontinuity')
    self.last_now = now
    if now < self.cooldown:
      return StartDecision(reason='driver_cooldown')
    if not valid or not math.isfinite(speed):
      self.pending, self.count = None, 0
      return StartDecision(self.hold, reason='invalid_context')
    if not self.hold and abs(speed) > .3:
      self.reset()
      return StartDecision(reason='moving_no_entry')
    evidence = self.observe(now, observation)
    released = False
    if evidence == 'red':
      self.hold = True
    elif evidence == 'green' and self.hold:
      released = True
      self.reset()
    warn = self.hold and departure and not self.warned
    self.warned |= warn
    return StartDecision(self.hold, warn, released, evidence)


class SignalStartRuntime(SignalAssistRuntime):
  def __init__(self, enable_path=ENABLE_PATH, observation_path=OBSERVATION_PATH):
    super().__init__(enable_path, observation_path)
    self.guard = SignalStartGuard()  # mode 0 may be enabled later without restart

  def record(self, decision, speed, distance):
    if not self.enabled and self.last_decision is None:
      return
    now = time.monotonic()
    key = (decision.hold, decision.warn, decision.released, decision.reason)
    if key != self.last_decision or now-self.last_log >= 1:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event('signalStartGuard', **asdict(decision), speed=float(speed),
                     model_stop_distance=float(distance), enabled=self.enabled)
      self.last_decision, self.last_log = key, now
