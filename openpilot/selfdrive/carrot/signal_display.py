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
                 engine='trial' if record.get('revalidated') or record.get('daytime_comparison') else 'legacy',
                 tracks=tracks)
  text = json.dumps(payload, allow_nan=False)
  if len(text.encode()) > MAX_BYTES:
    raise ValueError('signal display payload too large')
  path = Path(path)
  temp = path.with_suffix('.tmp')
  temp.write_text(text)
  temp.replace(path)


def visible_tracks(record, now, camera_frame):
  """Return current-image candidates only, in the road image's pixel coordinates."""
  try:
    fid, timestamp, width, height, wide = camera_frame
    if wide or (width, height) != SIZE or record['version'] != 1 or record['stream'] != 'road' or record['size'] != list(SIZE):
      return ()
    age = now - record['timestamp']
    if not (0 <= age <= MAX_AGE and 0 <= now - timestamp <= MAX_AGE
            and abs(timestamp - record['timestamp']) <= MAX_CAMERA_SKEW
            and abs(fid - record['frame_id']) <= 5 and isinstance(record['session'], str)):
      return ()
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
      state = t['state'] if t['state'] in ('red', 'green') and t['raw'] == t['state'] else 'unknown'
      result.append(dict(id=t['id'], box=tuple(t['box']), state=state))
    return tuple(result)
  except (KeyError, TypeError, ValueError, OverflowError):
    return ()


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

  def record_draw(self, camera_frame, count, *, path=STATUS_PATH, now=None):
    now = time.monotonic() if now is None else now
    if not self.enabled or now < self.next_status:
      return
    self.next_status = now + 1.
    try:
      record = self.record if isinstance(self.record, dict) else {}
      value = dict(timestamp=now, boxes_drawn=count, camera_frame=camera_frame,
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
      return ()
    if now >= self.next_read:
      self.next_read = now + .05
      try:
        with self.path.open('rb') as f:
          raw = f.read(MAX_BYTES + 1)
        self.record = json.loads(raw) if len(raw) <= MAX_BYTES else None
      except (OSError, ValueError):
        self.record = None
    return visible_tracks(self.record, now, camera_frame)
