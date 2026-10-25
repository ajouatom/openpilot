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

  def acknowledged(self, now, activated_id, driver_override=False):
    # A rejected/unsuccessful start must not become an invisible five-minute
    # toggle: the next physical button press should be a new request.
    if self.hold and (driver_override or now-self.hold['time'] > 4 and activated_id != self.hold['id']):
      self.hold = None
    return self.hold

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


class AutomaticSignals:
  """Start turns at the audioTurn cue; retain them through actual turn desire."""
  def __init__(self):
    self.cue = None
    self.direction = None
    self.turn_started = False
    self.mode = None

  def update(self, now, mode, valid, cue, direction, atc_type, desire, lane=None):
    previous, self.cue = self.cue, cue
    changed_mode, self.mode = mode != self.mode, mode
    if not valid or not mode or changed_mode:
      self.direction = None
      self.turn_started = False
      if not valid:
        self.cue = None
      return ('lane', lane) if valid and lane in ('left', 'right') else None
    # Consume cues even while invalid/disabled. Never join a cue already present
    # on startup, reconnect or setting changes, or replay a consumed cue.
    if (previous is not None and cue != previous and math.isfinite(cue) and
        0 <= now-cue < .35 and direction in ('left', 'right') and atc_type == 'turn '+direction):
      self.direction = direction
      self.turn_started = False
    if self.direction:
      expected = 'turnLeft' if self.direction == 'left' else 'turnRight'
      if desire == expected:
        self.turn_started = True
      elif self.turn_started or desire in ('turnLeft', 'turnRight') or atc_type != 'turn '+self.direction:
        self.direction = None
      if self.direction:
        return ('turn', self.direction)
    return ('lane', lane) if lane in ('left', 'right') else None
