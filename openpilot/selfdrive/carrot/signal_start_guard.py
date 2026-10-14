"""Opt-in stopped-car red veto, separate from moving signal assistance.

Image position is only a forward-signal hypothesis, not ego-lane association.
Unknown/stale observations cannot release a latched stop. A confirmed red-to-green
transition may feed the normal departure state machine, with driver gates intact.
"""
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import time

from openpilot.selfdrive.carrot.signal_assist_runtime import SignalAssistRuntime, requested, MAX_BYTES

ENABLE_PATH = Path('/data/signal-color-shadow/start_guard_enabled')
OBSERVATION_PATH = Path('/dev/shm/carrot_signal_start_observation.json')


def observation_color(observation, now, *, enabled, valid, drive):
  """Display-only current evidence: 0 unknown, 1 red, 2 green, 3 mixed.

  Never reuse the stop latch or UI memory as a current color observation.
  """
  if not (enabled and valid and drive) or observation is None:
    return 0
  try:
    stamp = observation['timestamp']
    if not math.isfinite(now) or not math.isfinite(stamp) or not 0 <= now-stamp <= SignalStartGuard.MAX_AGE:
      return 0
    colors = {t['raw'] for t in observation['tracks'] if SignalStartGuard.front(t['box'])}
    return int('red' in colors) + 2*int('green' in colors)
  except (KeyError, TypeError, ValueError):
    return 0


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
    tracks.append(dict(box=t['box'], raw=raw, id=t.get('id'),
                       seen_red=bool(t.get('seen_red', False)) and not record.get('neural', False)))
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
  green_confirmed: bool = False


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
    self.red_anchors = []
    self.track_id = None
    self.release_authorized = False
    self.green_evidence = False

  def reset_votes(self):
    self.green_evidence = False
    self.pending, self.count = None, 0
    for anchor in self.red_anchors:
      anchor['pending'], anchor['count'] = None, 0

  def observe_group(self, tracks, timestamp, gap):
    """A central red-to-green transition may resolve a lost primary.

    Green never moves an anchor. Duplicate/overlapping detections cannot become
    two votes. Conflicting reds veto release except a much smaller/lower group
    separated from TWO independently confirmed overhead transitions.
    Outside the central image band, keep the existing two-housing fallback.
    """
    reds = [t for t in tracks if t['raw'] == 'red']
    for track in reds:
      if len(self.red_anchors) < 8 and not any(self.anchor_matches(a['box'], a.get('id'), track) for a in self.red_anchors):
        self.red_anchors.append({'box': list(track['box']), 'id': track.get('id'), 'red': False, 'pending': None, 'count': 0, 'since': timestamp})
    greens = []
    for anchor in self.red_anchors:
      matches = [t for t in tracks if self.anchor_matches(anchor['box'], anchor.get('id'), t)]
      color = matches[0]['raw'] if len(matches) == 1 else 'unknown'
      if color == 'green' and (not anchor['red'] or any(
          not self.separate_lower_red(t['box'], anchor['box'], anchor['box']) for t in reds)):
        color = 'unknown'
      if color not in ('red', 'green') or color != anchor['pending'] or gap > self.MAX_GAP:
        anchor.update(pending=color, since=timestamp, count=1)
      else:
        anchor['count'] += 1
      if color == 'red':
        anchor['box'] = list(matches[0]['box'])
        anchor['id'] = matches[0].get('id')
        if anchor['count'] >= 2 and timestamp-anchor['since'] >= .15-1e-9:
          anchor['red'] = True
      elif color == 'green' and anchor['count'] >= 3 and timestamp-anchor['since'] >= .7-1e-9:
        greens.append(anchor['box'])
    # Image-centre support is a hypothesis, not signal-to-lane assignment.
    # Each candidate already needs its own confirmed red history and fresh
    # same-box green streak; a newly appearing green cannot release the stop.
    if not reds and any(.45 <= (box[0]+box[2])/2688 <= .55 for box in greens):
      return 'green_transition'
    for i, a in enumerate(greens):
      for b in greens[i+1:]:
        # Disjoint housings, comparable size and row: not two fits to one lamp.
        aw, bw = a[2]-a[0], b[2]-b[0]
        if (min(a[2], b[2]) <= max(a[0], b[0]) and max(aw, bw) <= 2*min(aw, bw)
            and abs((a[1]+a[3]-b[1]-b[3])/2) <= max(a[3]-a[1], b[3]-b[1])
            and all(self.separate_lower_red(t['box'], a, b) for t in reds)):
          return 'green_group'
    return None

  @staticmethod
  def separate_lower_red(red, a, b):
    """Only a paired overhead transition may exclude much smaller/lower reds.

    This is an image grouping hypothesis, not metric distance or lane assignment.
    A same-row, larger, overlapping, or insufficiently separated red still wins.
    """
    width = min(a[2]-a[0], b[2]-b[0])
    height = min(a[3]-a[1], b[3]-b[1])
    return (red[2]-red[0] <= .7*width and red[3]-red[1] <= .7*height
            and red[1] >= max(a[3], b[3]) + 2*max(a[3]-a[1], b[3]-b[1]))

  @classmethod
  def anchor_matches(cls, box, track_id, track):
    other = track['box']
    if cls.matches(box, other):
      return True
    # Same producer/session identity plus red history can bridge bounded box
    # resizing. Green cannot walk the fixed red reference onto a new housing.
    if (not isinstance(track_id, int) or isinstance(track_id, bool) or track.get('id') != track_id
        or not track.get('seen_red', False)):
      return False
    w, h = box[2]-box[0], box[3]-box[1]
    ow, oh = other[2]-other[0], other[3]-other[1]
    inter = max(0, min(box[2], other[2])-max(box[0], other[0])) * max(0, min(box[3], other[3])-max(box[1], other[1]))
    return (.5 <= ow*oh/(w*h) <= 2. and inter/(w*h+ow*oh-inter) >= .4
            and abs((box[0]+box[2]-other[0]-other[2])/2) <= .25*w
            and abs((box[1]+box[3]-other[1]-other[3])/2) <= .25*h)

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
        self.reset_votes()
        return 'stale_or_future'
      if not isinstance(fid, int) or not isinstance(session, str) or not session:
        raise ValueError('invalid identity')
      if session != self.session:
        # A restart cannot inherit green release rights; reacquire red first.
        self.session, self.box, self.track_id = session, None, None
        self.release_authorized = self.green_evidence = False
        self.timestamp, self.frame = -math.inf, -1
        self.pending, self.count = None, 0
        self.red_anchors = []
      if ts <= self.timestamp or fid <= self.frame:
        return 'repeated_or_reordered'
      gap = ts-self.timestamp
      self.timestamp, self.frame = ts, fid
      tracks = [t for t in observation['tracks'] if self.front(t['box'])]
      group_green = self.observe_group(tracks, ts, gap)
      matches = [t for t in tracks if self.box is not None and self.anchor_matches(self.box, self.track_id, t)]
      red = [t for t in tracks if t['raw'] == 'red']
      if self.box is None or (not matches and red):
        selected = min(red, key=lambda t: abs((t['box'][0]+t['box'][2])/2688-.5)) if red else None
        if selected is not None:
          self.box = selected['box']
          self.track_id = selected.get('id')
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
        self.track_id = selected.get('id')
      if color == 'green' and red:
        color = 'unknown'
      if group_green:
        return group_green
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
      self.reset_votes()
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
      self.red_anchors = []
      self.release_authorized = self.green_evidence = False
      return StartDecision(self.hold, reason='clock_discontinuity')
    self.last_now = now
    if now < self.cooldown:
      return StartDecision(reason='driver_cooldown')
    if not valid or not math.isfinite(speed):
      self.reset_votes()
      return StartDecision(self.hold, reason='invalid_context')
    if not self.hold and abs(speed) > .3 and not (self.release_authorized and abs(speed) < 5/3.6):
      self.reset()
      return StartDecision(reason='moving_no_entry')
    evidence = self.observe(now, observation)
    released = False
    if evidence != 'repeated_or_reordered':
      self.green_evidence = evidence in ('green', 'green_group', 'green_transition')
    if evidence == 'red':
      self.release_authorized = False
      self.hold = self.hold or abs(speed) <= .3
    elif evidence in ('green', 'green_group', 'green_transition') and self.hold:
      released = True
      # Keep the fixed red anchors while stopped so the same fresh paired
      # transition continues to exclude a separate lower red group. Missing
      # green removes that exclusion; moving/disable/restart clears identity.
      self.hold = self.warned = False
      self.release_authorized = True
    warn = self.hold and departure and not self.warned
    self.warned |= warn
    green = (self.release_authorized and self.green_evidence and not self.hold
             and 0 <= now-self.timestamp <= self.MAX_AGE)
    return StartDecision(self.hold, warn, released, evidence, green)


class SignalStartRuntime(SignalAssistRuntime):
  def __init__(self, enable_path=ENABLE_PATH, observation_path=OBSERVATION_PATH):
    super().__init__(enable_path, observation_path)
    self.guard = SignalStartGuard()  # mode 0 may be enabled later without restart

  def record(self, decision, speed, distance):
    if not self.enabled and self.last_decision is None:
      return
    now = time.monotonic()
    key = (decision.hold, decision.warn, decision.released, decision.reason, decision.green_confirmed)
    if key != self.last_decision or now-self.last_log >= 1:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event('signalStartGuard', **asdict(decision), speed=float(speed),
                     model_stop_distance=float(distance), enabled=self.enabled)
      self.last_decision, self.last_log = key, now
