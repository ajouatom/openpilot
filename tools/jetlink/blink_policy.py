"""Pure indicator policy; never requests steering or changes ATC admission."""
from collections import deque
import uuid
import math

NEAR_STOP = 5 / 3.6
MAX_HOLD = 300.


class Policy:
  def __init__(self):
    self.session = uuid.uuid4().hex
    self.serial = 0
    self.seen = deque(maxlen=256)
    self.hold = None
    self.taps = []
    self.auto_previous = None
    self.auto_armed = False
    self.suppressed = False
    self.mode = 0
    self.turned = False
    self.centered = None

  def start(self, direction, source, now):
    self.serial += 1
    self.hold = {'id': f'{self.session}:{self.serial}', 'direction': direction, 'source': source, 'time': now}
    self.turned, self.centered = False, None

  def update(self, now, mode, ok, commands, speed, angle, gear, remote, automatic=None, auto_valid=True):
    # Changing the setting or reconnecting never joins an already-running maneuver.
    if mode != self.mode or not ok:
      self.hold = None
      self.auto_armed = False
      self.suppressed = automatic is not None
      self.mode = mode
    self.taps = [e for e in self.taps if 0 <= now-e['time'] < .4]
    if not ok:
      self.taps = []
      return [], None
    eligible = automatic if mode >= (2 if automatic and automatic[0] == 'lane' else 1) and mode else None
    if not auto_valid:
      self.auto_armed = False
      self.suppressed = True
    if eligible is None and auto_valid:
      self.auto_armed = True
      self.suppressed = False
    if self.hold:
      h = self.hold
      if (mode == 0 or now-h['time'] >= MAX_HOLD or gear not in ('park', 'neutral', 'drive') or
          (h['source'] == 'bluetooth' and not remote) or
          (h['source'] == 'bluetooth' and not math.isfinite(angle)) or
          (h['source'] != 'bluetooth' and (not auto_valid or eligible != (h['source'], h['direction'])))):
        self.hold = None
    manual = False
    for event in commands:
      if event['id'] in self.seen:
        continue
      self.seen.append(event['id'])
      manual = True
      self.suppressed = eligible is not None
      previous = self.hold
      self.hold = None
      if previous and previous['direction'] == event['direction']:
        continue  # Same button cancels any held signal, including an ATC signal.
      if mode and math.isfinite(angle) and gear in ('park', 'neutral', 'drive') and (abs(speed) <= NEAR_STOP or previous and previous['source'] == 'bluetooth'):
        self.start(event['direction'], 'bluetooth', event['time'])
        self.hold['device'] = event.get('device')
      else:
        self.taps.append(event)
    if (not manual and not self.hold and eligible and eligible != self.auto_previous and
        self.auto_armed and not self.suppressed):
      self.start(eligible[1], eligible[0], now)
    self.auto_previous = eligible
    if self.hold and self.hold['source'] == 'bluetooth':
      # Physical completion: steer into the selected turn, then return near center
      # while moving. Do not infer completion just from departing a stop.
      signed_angle = angle * (1 if self.hold['direction'] == 'left' else -1)
      self.turned |= signed_angle >= 30 and abs(speed) >= 1 / 3.6
      if self.turned and abs(angle) <= 10 and abs(speed) >= 1 / 3.6:
        self.centered = now if self.centered is None else self.centered
        if now-self.centered >= .4:
          self.hold = None
      else:
        self.centered = None
    return self.taps[-4:], self.hold
