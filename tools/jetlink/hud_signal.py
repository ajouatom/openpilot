"""Bounded display-only signal metadata; never a planner or model input."""
import json
import math
import sys
import time

from openpilot.selfdrive.carrot import signal_display as signals

MAX_WIRE_BYTES = 4096


def clean_record(value, now):
  """Allowlist the existing display schema and discard non-current detections."""
  try:
    stamp, fid = value['timestamp'], value['frame_id']
    session = value['session']
    if (type(fid) is not int or not 0 <= fid < 2**32 or
        type(stamp) not in (int, float) or not math.isfinite(stamp) or
        not isinstance(session, str) or not 0 < len(session) <= 128):
      return None
    camera = (fid, now, *signals.SIZE, False)
    if not signals.valid_record(value, now, camera, hold=True):
      return None
    tracks = []
    for t in value['tracks']:
      # visible_tracks validates all box coordinates/IDs and current age.
      if not signals.visible_tracks(dict(value, tracks=[t]), now, camera, hold=True):
        continue
      tracks.append(dict(id=t['id'], box=list(t['box']), age=0,
                         state=t['state'] if t['state'] in ('red', 'green') else 'unknown',
                         raw=t['raw'] if t['raw'] in ('red', 'green') else 'unknown'))
    result = dict(version=1, stream='road', size=list(signals.SIZE), timestamp=stamp,
                  frame_id=fid, session=session, tracks=tracks)
    envelope = dict(version=1, enabled=True, record=result)
    if len(json.dumps(envelope, separators=(',', ':'), allow_nan=False).encode()) > MAX_WIRE_BYTES:
      return None
    return result
  except (KeyError, TypeError, ValueError, OverflowError):
    return None


def signal_packet(now=None, path=signals.DISPLAY_PATH):
  now = time.monotonic() if now is None else now
  enabled = signals.display_requested()
  value = None
  if enabled:
    try:
      with path.open('rb') as source:
        raw = source.read(signals.MAX_BYTES + 1)
      if len(raw) <= signals.MAX_BYTES:
        value = clean_record(json.loads(raw), now)
    except (OSError, ValueError, TypeError):
      pass
  return dict(version=1, enabled=bool(enabled), record=value)


class RemoteSignals:
  """Read-only adapter scoped to the Jetson renderer's process."""
  def __init__(self, read_snapshot, clock=time.monotonic):
    self.read_snapshot, self.clock = read_snapshot, clock
    self.key = self.value = None
    self.enabled = False
    self.color = 0
    self.color_time = self.capture_time = -math.inf
    self.summary_key = None

  def record(self):
    self.enabled = False
    snapshot = self.read_snapshot()
    if snapshot is None:
      return None
    try:
      received, packet = snapshot
      now, sent = self.clock(), packet['sent']
      value = packet.get('signal_display')
      if (not 0 <= now - received < .5 or not math.isfinite(sent) or
          not isinstance(value, dict) or value.get('version') != 1):
        return None
      self.enabled = value.get('enabled') is True
      if not self.enabled:
        return None
      source = clean_record(value.get('record'), sent)
      if source is None:
        return None
      key = (source['session'], source['timestamp'], source['frame_id'])
      stamp = received - (sent - source['timestamp'])
      # A repeated observation cannot acquire a later capture time even if
      # packet arrival jitter changes the estimated host/source clock offset.
      if key == self.key:
        stamp = min(stamp, self.value['timestamp'])
      self.key, self.value = key, dict(source, timestamp=stamp)
      if not 0 <= now - stamp <= signals.DISPLAY_HOLD:
        return None
      return self.value
    except (KeyError, ValueError, TypeError, OverflowError):
      return None

  def requested(self, *args):
    self.record()
    return self.enabled

  def status(self):
    record, now = self.record(), self.clock()
    if record is None:
      self.summary_key, self.color = None, 0
      self.color_time = self.capture_time = -math.inf
      return 0, False, False
    key = self.key
    if self.summary_key and (key[0] != self.summary_key[0] or key[1] < self.summary_key[1] or key[2] < self.summary_key[2]):
      self.color, self.color_time = 0, -math.inf
    if key != self.summary_key:
      camera = (record['frame_id'], now, *signals.SIZE, False)
      colors = {t['state'] for t in signals.visible_tracks(record, now, camera, hold=True)}
      color = int('red' in colors) + 2 * int('green' in colors)
      if color:
        self.color, self.color_time = color, record['timestamp']
      self.summary_key, self.capture_time = key, record['timestamp']
    else:
      # Earlier clock mapping may shorten, never renew the held color.
      if self.color_time == self.capture_time:
        self.color_time = min(self.color_time, record['timestamp'])
      self.capture_time = min(self.capture_time, record['timestamp'])
    color = self.color if 0 <= now - self.color_time <= signals.DISPLAY_HOLD else 0
    held = bool(color and (now - self.color_time > signals.MAX_AGE or self.color_time != self.capture_time))
    return color, held, True

  def reader(self, *args, **kwargs):
    return RemoteSignalReader(self)


class RemoteSignalReader:
  def __init__(self, remote):
    self.remote = remote
    self.enabled = False
    self.record = None
    self.held_count = 0
    self.continuity = signals.DisplayContinuity()

  def read(self, camera_frame, now=None):
    now = self.remote.clock() if now is None else now
    self.record = self.remote.record()
    self.enabled = self.remote.enabled
    tracks = self.continuity.update(self.record, now, camera_frame)
    self.held_count = sum(t['held'] for t in tracks)
    return tracks

  def record_draw(self, *args, **kwargs):
    pass  # Never write back into the vehicle's observation/status files.


def install(read_snapshot):
  remote = RemoteSignals(read_snapshot)
  signals.SignalDisplayReader = remote.reader
  # Handle either import order; the shared renderer imports this class by name.
  for name in ('cluster_renderer', 'openpilot.selfdrive.carrot.cluster.cluster_renderer'):
    module = sys.modules.get(name)
    if module is not None:
      module.SignalDisplayReader = remote.reader
  signals.signal_display_status = remote.status
  signals.display_requested = remote.requested
  return remote
