"""Explicit per-device experimental opt-in and bounded tmpfs observation transport."""
from dataclasses import asdict
import json
from pathlib import Path
import time

from openpilot.selfdrive.carrot.signal_assist import SignalAssist

ENABLE_PATH = Path('/data/signal-color-shadow/assist_enabled')
OBSERVATION_PATH = Path('/dev/shm/carrot_signal_observation.json')
MAX_BYTES = 32768


def requested(path=ENABLE_PATH):
  try:
    return Path(path).read_text().strip() == '1'
  except OSError:
    return False


def publish_observation(result, frame_id, timestamp, session, path=OBSERVATION_PATH):
  tracks = [dict(id=t['id'], box=t['box'], state=t['state'], age=t['age'], observations=t['observations'],
                 evidence={'raw': t['evidence']['raw']}) for t in result['tracks'][:20]]
  record = dict(version=1, stream='road', size=[1344, 760], frame_id=frame_id,
                timestamp=timestamp, session=session, tracks=tracks)
  text = json.dumps(record, allow_nan=False)
  if len(text.encode()) > MAX_BYTES:
    raise ValueError('signal observation too large')
  path = Path(path)
  temp = path.with_suffix('.tmp')
  temp.write_text(text)
  temp.replace(path)


class SignalAssistRuntime:
  def __init__(self, enable_path=ENABLE_PATH, observation_path=OBSERVATION_PATH):
    self.enable_path, self.observation_path = Path(enable_path), Path(observation_path)
    self.assist = SignalAssist() if requested(self.enable_path) else None
    self.last_mode_check = -1.
    self.enabled = self.assist is not None
    self.last_log = -1.
    self.last_decision = None
    self.read_ms = 0.

  def read(self, sm, now=None):
    now = time.monotonic() if now is None else now
    start = time.monotonic()
    if now - self.last_mode_check >= .5:
      self.enabled = requested(self.enable_path)
      self.last_mode_check = now
    observation = None
    if self.enabled:
      try:
        with self.observation_path.open('rb') as f:
          raw = f.read(MAX_BYTES + 1)
        if len(raw) <= MAX_BYTES:
          obj = json.loads(raw)
          if obj.get('version') == 1 and obj.get('stream') == 'road' and obj.get('size') == [1344, 760]:
            observation = obj
      except (OSError, ValueError, TypeError, AttributeError):
        pass
    services = ('carState', 'modelV2', 'radarState', 'selfdriveState', 'carControl')
    valid = all(sm.seen[s] and sm.valid[s] and sm.alive[s] and 0 <= now - sm.recv_time[s] <= .25 for s in services)
    cs, sd, cc = sm['carState'], sm['selfdriveState'], sm['carControl']
    context = dict(now=now, enabled=self.enabled and sd.enabled and cc.longActive,
                   valid=valid and cs.canValid, drive=str(cs.gearShifter) == 'drive')
    self.read_ms = (time.monotonic() - start) * 1000
    return observation, context

  def record(self, decision, speed, distance):
    now = time.monotonic()
    value = asdict(decision)
    key = (decision.hold, decision.red_sign, decision.released, decision.track_id)
    if key != self.last_decision or now - self.last_log >= 1.:
      from openpilot.common.swaglog import cloudlog
      cloudlog.event('signalAssist', **value, speed=float(speed), model_stop_distance=float(distance),
                     transport_read_ms=self.read_ms, enabled=self.enabled)
      self.last_decision, self.last_log = key, now
