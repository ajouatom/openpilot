"""Experimental lead-lamp observation and display only. Never a control input."""
import json
import math
from pathlib import Path
import time

PARAM = 'LeadBrakeLights'
PATH = Path('/dev/shm/carrot_lead_brake_lights.json')
_params = None
_enabled = False
_next_read = 0.


def requested():
  global _params, _enabled, _next_read
  from openpilot.selfdrive.carrot.signal_mode import signal_mode
  try:
    mode = signal_mode()
  except Exception:
    return False
  if mode == 0:
    return False
  now = time.monotonic()
  if now >= _next_read:
    try:
      if _params is None:
        from openpilot.common.params import Params
        _params = Params()
      _enabled = _params.get_bool(PARAM)
    except Exception:
      _enabled = False
    _next_read = now + .5
  return _enabled


def display_result(record, now, camera_frame):
  """No display hold: stale, wrong-camera or unrequested data disappears."""
  try:
    fid, stamp, width, height, wide = camera_frame
    if (wide or (width, height) != (1344, 760) or record['version'] != 1
        or record['control_permission'] is not False or not 0 <= now-stamp <= .35
        or not 0 <= now-record['timestamp'] <= .35 or abs(stamp-record['timestamp']) > .25
        or abs(fid-record['frame_id']) > 5 or record['state'] not in ('candidate', 'unknown')):
      return None
    box = record.get('box')
    if box is not None:
      if (len(box) != 4 or not all(isinstance(x, (float, int)) and math.isfinite(x) for x in box)
          or not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height):
        return None
    return record
  except (KeyError, TypeError, ValueError, OverflowError):
    return None


class BrakeDisplayReader:
  def __init__(self, path=PATH):
    self.path = Path(path)
    self.record = None
    self.next_read = 0.

  def read(self, camera_frame, now=None):
    if not requested():
      self.record = None
      return None
    now = time.monotonic() if now is None else now
    if now >= self.next_read:
      self.next_read = now + .05
      try:
        with self.path.open('rb') as f:
          raw = f.read(4097)
        self.record = json.loads(raw) if len(raw) <= 4096 else None
      except (OSError, ValueError):
        self.record = None
    return display_result(self.record, now, camera_frame)


class BrakeRuntime:
  """Lazy subscriptions; OFF does not create a subscriber or import the detector.

  Signal results are published first. At most 10 Hz of bounded crop work is
  admitted only when the signal iteration has at least 5 ms left in 50 ms.
  Missing evidence is UNKNOWN, never a verified brake-off or permission to go.
  """
  def __init__(self):
    self.sm = self.detector = None
    self.next_sample = 0.
    self.next_log = 0.
    self.session = None

  def update(self, rgb, frame_id, stamp, session, work_start, logger=None):
    if not requested():
      self.sm = self.detector = None
      self.next_sample = 0.
      self.session = None
      return
    if session != self.session:
      if self.detector is not None:
        self.detector.reset()
      self.next_sample = 0.
      self.session = session
    now = time.monotonic()
    if now < self.next_sample:
      return
    self.next_sample = now + .1
    result = dict(state='unknown', reason='signal_budget', box=None)
    if now-work_start < .045:
      if self.sm is None:
        import openpilot.cereal.messaging as messaging
        from tools.signal_analysis.lead_brake_detector import LeadBrakeDetector
        self.sm = messaging.SubMaster(['radarState', 'modelV2', 'carState', 'liveCalibration'])
        self.detector = LeadBrakeDetector()
      self.sm.update(0)
      topics = ('radarState', 'modelV2', 'carState', 'liveCalibration')
      # Calibration is published slower; require a bounded, valid sample too.
      fresh = all(self.sm.seen[k] and self.sm.valid[k]
                  and 0 <= now-self.sm.logMonoTime[k]/1e9 <= (2. if k == 'liveCalibration' else .25)
                  for k in topics)
      if fresh:
        result = self.detector.process(rgb, stamp, self.sm['radarState'].leadOne.to_dict(),
                                       self.sm['modelV2'].leadsV3[0].to_dict() if self.sm['modelV2'].leadsV3 else {},
                                       self.sm['carState'].to_dict(), self.sm['liveCalibration'].to_dict())
      else:
        self.detector.reset()
        result['reason'] = 'stale_context'
    elif self.detector is not None:
      self.detector.reset()
    elapsed = (time.monotonic()-now)*1000
    if elapsed > 5:
      result = dict(state='unknown', reason='work_budget', box=None)
      self.next_sample = time.monotonic() + .5
      if self.detector is not None:
        self.detector.reset()
    record = dict(version=1, frame_id=frame_id, timestamp=stamp, session=session,
                  work_ms=elapsed, control_permission=False, **result)
    temp = PATH.with_suffix('.tmp')
    temp.write_text(json.dumps(record, allow_nan=False))
    temp.replace(PATH)
    if logger is not None and now >= self.next_log:
      logger('leadBrakeLights', **record)
      self.next_log = now + 1.
