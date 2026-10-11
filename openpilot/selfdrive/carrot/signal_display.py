"""Opt-in visualization transport. Never read by the planner or control helpers."""
import json
import math
from pathlib import Path
import time

DISPLAY_FLAG = Path('/data/signal-color-shadow/display_enabled')
DISPLAY_PATH = Path('/dev/shm/carrot_signal_display.json')
STATUS_PATH = Path('/dev/shm/carrot_signal_display_status.json')
MAX_BYTES = 8192
MAX_AGE = .35
MAX_CAMERA_SKEW = .25
DISPLAY_HOLD = 2.0  # UI memory only; never extends the control observation lifetime.
SIZE = (1344, 760)


def display_requested(path=DISPLAY_FLAG):
  try:
    return Path(path).read_text().strip() == '1'
  except OSError:
    return False


def publish_display(record, *, path=DISPLAY_PATH):
  tracks = [dict(id=t['id'], box=t['box'], state=t['state'], age=t['age'], raw=t['evidence']['raw'])
            for t in record['tracks'][:20]]
  payload = dict(version=1, stream='road', size=SIZE, frame_id=record['frame_id'],
                 timestamp=record['timestamp_eof'] / 1e9, session=record['session'],
                 engine=('neural_' + record['backend']) if record.get('neural') else
                        'trial' if record.get('revalidated') or record.get('daytime_comparison') else 'legacy',
                 tracks=tracks)
  text = json.dumps(payload, allow_nan=False)
  if len(text.encode()) > MAX_BYTES:
    raise ValueError('signal display payload too large')
  path = Path(path)
  temp = path.with_suffix('.tmp')
  temp.write_text(text)
  temp.replace(path)


def valid_record(record, now, camera_frame, *, hold=False):
  try:
    fid, timestamp, width, height, wide = camera_frame
    limit = DISPLAY_HOLD if hold else MAX_AGE
    skew = DISPLAY_HOLD if hold else MAX_CAMERA_SKEW
    return (not wide and (width, height) == SIZE and record['version'] == 1
            and record['stream'] == 'road' and record['size'] == list(SIZE)
            and 0 <= now - timestamp <= MAX_AGE and 0 <= now - record['timestamp'] <= limit
            and abs(timestamp - record['timestamp']) <= skew
            and abs(fid - record['frame_id']) <= (40 if hold else 5)
            and isinstance(record['session'], str) and bool(record['session'])
            and isinstance(record['tracks'], list) and len(record['tracks']) <= 20)
  except (KeyError, TypeError, ValueError, OverflowError):
    return False


def visible_tracks(record, now, camera_frame, *, hold=False):
  """Return current-image candidates only, in the road image's pixel coordinates."""
  try:
    if not valid_record(record, now, camera_frame, hold=hold):
      return ()
    _, _, width, height, _ = camera_frame
    tracks = record['tracks']
    if not isinstance(tracks, list) or len(tracks) > 20:
      return ()
    result = []
    for t in tracks:
      x1, y1, x2, y2 = t['box']
      if not (all(isinstance(v, (int, float)) and math.isfinite(v) for v in t['box'])
              and 0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height
              and t['age'] == 0 and isinstance(t['id'], int) and 0 < t['id'] < 1_000_000):
        continue
      # Display current color evidence independently of departure qualification.
      # A green-first track cannot satisfy the observer's red-history gate.
      state = t['raw'] if t['raw'] in ('red', 'green') else 'unknown'
      confirmed = state != 'unknown' and t['state'] == state
      result.append(dict(id=t['id'], box=tuple(t['box']), state=state, confirmed=confirmed))
    return tuple(result)
  except (KeyError, TypeError, ValueError, OverflowError):
    return ()


class DisplayContinuity:
  """Bounded previous-result display. No permission or observer-state output."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.key = None
    self.tracks = {}

  def update(self, record, now, camera_frame):
    if not valid_record(record, now, camera_frame, hold=True):
      self.reset()
      return ()
    key = (record['session'], record['timestamp'], record['frame_id'])
    if self.key and (key[0] != self.key[0] or key[1] < self.key[1] or key[2] < self.key[2]):
      self.reset()
    if key != self.key:
      current = visible_tracks(record, now, camera_frame, hold=True)
      current_ids = {t['id'] for t in current}
      inherited = {}
      # Neural inference can assign a new ID to the same housing. Preserve
      # display color through a unique geometric match, without renewing its
      # timestamp or changing any observer/control state.
      def same_housing(a, b):
        intersection = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
        union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
        return intersection / max(union, 1) >= .5
      for t in current:
        if t['id'] in self.tracks or t['state'] != 'unknown':
          continue
        matches = [(tid, old) for tid, old in self.tracks.items() if tid not in current_ids
                   and now-old['color_time'] <= DISPLAY_HOLD and same_housing(old['track']['box'], t['box'])]
        if len(matches) == 1:
          tid, old = matches[0]
          if sum(same_housing(old['track']['box'], other['box']) for other in current) == 1:
            inherited[t['id']] = old
      # A replacement ID at the same housing must not leave a duplicate old color.
      for tid, old in list(self.tracks.items()):
        a = old['track']['box']
        if tid not in current_ids and any(
            max(a[0], t['box'][0]) < min(a[2], t['box'][2])
            and max(a[1], t['box'][1]) < min(a[3], t['box'][3]) for t in current):
          del self.tracks[tid]
      for t in current:
        previous = self.tracks.get(t['id'], inherited.get(t['id']))
        color_time = record['timestamp']
        if t['state'] == 'unknown' and previous and now - previous['color_time'] <= DISPLAY_HOLD:
          t = dict(t, state=previous['track']['state'], confirmed=False)
          color_time = previous['color_time']
        self.tracks[t['id']] = dict(track=t, observed=record['timestamp'], color_time=color_time, key=key)
      self.key = key
    fresh_ids = {t['id'] for t in visible_tracks(record, now, camera_frame)}
    result = []
    for tid, saved in sorted(self.tracks.items(), key=lambda item: -item[1]['observed']):
      if now - saved['observed'] > DISPLAY_HOLD:
        continue
      track = dict(saved['track'])
      # Unknown frames may preserve geometry, but never refresh an old color.
      if now - saved['color_time'] > DISPLAY_HOLD:
        track.update(state='unknown', confirmed=False)
      held = saved['key'] != key or tid not in fresh_ids or saved['color_time'] != saved['observed']
      track.update(held=held, age=now - saved['observed'])
      if held:
        track['confirmed'] = False
      result.append(track)
    result = result[:20]
    self.tracks = {t['id']: self.tracks[t['id']] for t in result}
    return tuple(result)


def project_box(box, video_rect):
  """Same full-image-to-destination mapping used by LiveRoadCamera.draw()."""
  x, y, w, h = video_rect
  x1, y1, x2, y2 = box
  return (x + x1 * w / SIZE[0], y + y1 * h / SIZE[1], (x2-x1) * w / SIZE[0], (y2-y1) * h / SIZE[1])


class SignalDisplayReader:
  def __init__(self, flag=DISPLAY_FLAG, path=DISPLAY_PATH):
    self.flag, self.path = Path(flag), Path(path)
    self.enabled = False
    self.next_flag_read = self.next_read = 0.
    self.record = None
    self.next_status = 0.
    self.continuity = DisplayContinuity()
    self.held_count = 0

  def record_draw(self, camera_frame, count, *, path=STATUS_PATH, now=None):
    now = time.monotonic() if now is None else now
    if not self.enabled or now < self.next_status:
      return
    self.next_status = now + 1.
    try:
      record = self.record if isinstance(self.record, dict) else {}
      value = dict(display_revision=4, timestamp=now, boxes_drawn=count, held_boxes=self.held_count, camera_frame=camera_frame,
                   observation_frame=record.get('frame_id'), engine=record.get('engine'),
                   observation_timestamp=record.get('timestamp'))
      path = Path(path); temp = path.with_suffix('.tmp')
      temp.write_text(json.dumps(value, allow_nan=False)); temp.replace(path)
    except (OSError, ValueError, TypeError):
      pass

  def read(self, camera_frame, now=None):
    now = time.monotonic() if now is None else now
    if now >= self.next_flag_read:
      self.enabled = display_requested(self.flag)
      self.next_flag_read = now + .5
    if not self.enabled:
      self.record = None
      self.continuity.reset()
      return ()
    if now >= self.next_read:
      self.next_read = now + .05
      try:
        with self.path.open('rb') as f:
          raw = f.read(MAX_BYTES + 1)
        self.record = json.loads(raw) if len(raw) <= MAX_BYTES else None
      except (OSError, ValueError):
        self.record = None
    tracks = self.continuity.update(self.record, now, camera_frame)
    self.held_count = sum(t['held'] for t in tracks)
    return tracks
