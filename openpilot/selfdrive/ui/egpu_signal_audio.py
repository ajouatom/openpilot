"""Opt-in perception tones. No route relevance, GO decision, or actuator output."""
import json
import math
import os
from pathlib import Path

MODEL_ID = "signal-v33-observe-s260911"


def get(value, key, default=None):
  return value.get(key, default) if isinstance(value, dict) else getattr(value, key, default)


class SignalObservation:
  def __init__(self):
    self.candidate = None
    self.since = self.last_seen = 0.
    self.count = 0
    self.frame = -1
    self.announced = None
    self.last_tone = -math.inf

  def update(self, message, *, now, valid, transport_age, enabled, blocked=False):
    color = None
    try:
      frame = int(get(message, 'frameId', -1))
      fresh = (enabled and not blocked and valid and math.isfinite(transport_age) and 0 <= transport_age <= .35
               and frame >= 0 and math.isfinite(now)
               and get(message, 'modelId') == MODEL_ID and str(get(message, 'camera')) == 'road'
               and str(get(message, 'state')) == 'run'
               and 0 <= now-float(get(message, 'timestampEof', 0))/1e9 <= .35)
      if fresh:
        colors = set()
        for d in list(get(message, 'detections', ()))[:100]:
          label = str(get(d, 'label', ''))
          score = float(get(d, 'confidence', 0))
          box = [float(get(d, k, math.nan)) for k in ('x1', 'y1', 'x2', 'y2')]
          if (label in ('red_visible', 'green_visible') and math.isfinite(score) and .25 <= score <= 1
              and all(math.isfinite(v) for v in box) and 0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1):
            colors.add(label)
        if len(colors) == 1:
          color = next(iter(colors))
    except (TypeError, ValueError, OverflowError):
      color = None
    if color is None:
      self.candidate = None
      self.count = 0
      if not enabled or blocked or now-self.last_seen >= 2:
        self.announced = None
      return None
    if frame == self.frame:
      return None
    if frame < self.frame or now-self.last_seen > .35 or color != self.candidate:
      self.candidate, self.since, self.count = color, now, 0
    self.frame = frame
    self.last_seen = now
    self.count += 1
    if self.count >= 3 and now-self.since >= .3 and color != self.announced and now-self.last_tone >= 3:
      self.announced, self.last_tone = color, now
      return color
    return None


class SignalAudio:
  def __init__(self, sample_rate):
    import numpy as np
    self.state = SignalObservation()
    self.path = Path(os.getenv('EGPU_YOLO_DIR', '/data/egpu_yolo'))/'signal_observation.json'
    self.next_check = 0.
    self.enabled = False
    self.samples = None
    self.position = 0
    t = np.arange(round(sample_rate*.12), dtype=np.float32)/sample_rate
    envelope = np.minimum(1, t/.01)*np.minimum(1, (.12-t)/.025)
    self.tones = {name: (.2*np.sin(2*np.pi*hz*t)*envelope).astype(np.float32)
                  for name, hz in [('red_visible', 550), ('green_visible', 850)]}

  def update(self, message, *, now, valid, transport_age, blocked):
    if now >= self.next_check:
      self.next_check = now+1.
      try:
        config = json.loads(self.path.read_text())
        self.enabled = config.get('enabled') is True and config.get('model_id') == MODEL_ID
      except (OSError, ValueError, TypeError, AttributeError):
        self.enabled = False
    color = self.state.update(message, now=now, valid=valid, transport_age=transport_age,
                              enabled=self.enabled, blocked=blocked)
    if blocked or not self.enabled:
      self.samples = None
    elif color is not None:
      self.position = 0
      self.samples = self.tones[color]

  def render(self, frames, *, priority):
    import numpy as np
    out = np.zeros(frames, dtype=np.float32)
    if priority:
      self.samples = None
    samples = self.samples
    if samples is not None:
      count = min(frames, len(samples)-self.position)
      out[:count] = samples[self.position:self.position+count]
      self.position += count
      if self.position >= len(samples):
        self.samples = None
    return out
