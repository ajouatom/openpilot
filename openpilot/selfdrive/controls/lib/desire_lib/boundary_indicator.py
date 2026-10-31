"""Driver-initiated boundary approach indication, with lamp-feedback isolation.

This observes a manual maneuver; it never supplies a lane-change desire.
"""

import math
from openpilot.selfdrive.controls.lib.desire_lib.boundary_lane import adjacent_lane


TRACK_SEC = 1.0  # Brief confidence loss during a continuous driver crossing.
QUIET_SEC = 3.0  # Earliest new request; extinction is required independently.
ISOLATE_SEC = 12.0  # Keep delayed lamp feedback out of automatic steering.


class BoundaryIndicator:
  def __init__(self, journal=None):
    self.journal = journal if isinstance(journal, dict) else {}
    self.initialized = False
    self.previous = None
    self.confident = [None, None]
    self.rates = [0.0, 0.0]
    self.intent = 0
    self.intent_since = 0.0
    self.episode = None
    self.lane_since = [None, None]
    self.relabel_until = 0.0
    self.blocked = 0
    self.block_until = 0.0
    self.isolate_until = 0.0
    self.dark_since = None
    self.request_direction = 0
    self.request_time = 0.0

  def _reset_geometry(self):
    self.previous = None
    self.confident = [None, None]
    self.rates = [0.0, 0.0]
    self.intent = 0
    self.episode = None
    self.lane_since = [None, None]

  def update(self, now, cs, md, *, enabled, valid, busy):
    lamps = int(cs.leftBlinker) | (int(cs.rightBlinker) << 1)
    if not self.initialized:
      self.initialized = True
      stamp, direction = self.journal.get('time'), self.journal.get('direction')
      if type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now - stamp < 20 and direction in (1, 2):
        mask = self.journal.get('mask', direction)
        self.blocked = mask if type(mask) is int and mask in (1, 2, 3) and mask & direction else direction
        self.block_until = max(now + 1.0, stamp + QUIET_SEC)
        self.isolate_until = max(now + 1.0, stamp + ISOLATE_SEC)
      elif enabled:
        # Suppress only new detections at startup. Without an owned journal,
        # ordinary driver/ATC lamps must retain their original interpretation.
        self.relabel_until = now + 1.0
    if lamps or not valid:
      self.dark_since = None
    else:
      self.dark_since = now if self.dark_since is None else self.dark_since
    dark = self.dark_since is not None and now - self.dark_since >= 1.0
    if self.blocked and now >= self.isolate_until and dark:
      self.blocked = 0
    if not enabled or not valid or busy:
      self.request_direction = 0
      self._reset_geometry()
      return
    if (
      lamps
      or cs.blinkerLever
      or not cs.canValid
      or str(cs.gearShifter) != 'drive'
      or cs.trailerConnected
      or cs.steerFaultTemporary
      or cs.steerFaultPermanent
      or not math.isfinite(cs.vEgo)
      or not math.isfinite(cs.steeringTorque)
      or cs.vEgo < 30 / 3.6
    ):
      self.request_direction = 0
      self._reset_geometry()
      return
    if now < self.block_until or (self.blocked and not dark):
      self._reset_geometry()
      # Preserve the same fresh edge after torque release, never replay a timer.
      if not 0 <= now - self.request_time < .2 or not adjacent_lane(cs, md, self.request_direction, now):
        self.request_direction = 0
      return
    self.request_direction = 0
    try:
      left, right = md.laneLines[1].y[0], md.laneLines[2].y[0]
      probs = (md.laneLineProbs[1], md.laneLineProbs[2])
      if (
        not all(math.isfinite(x) for x in (left, right, *probs, cs.steeringTorque)) or not left < 0 < right or not 2.5 < right - left < 4.8
      ):
        self._reset_geometry()
        return
      for i, probability in enumerate(probs):
        if probability > 0.5:
          self.confident[i] = now
      y = (-left, right)
      old, self.previous = self.previous, (md.frameId, now, y)
      if old is None:
        return
      frame, stamp, prev_y = old
      dt = now - stamp
      if md.frameId <= frame or not 0.02 <= dt <= 0.15:
        self._reset_geometry()
        return
      if max(abs(y[i] - prev_y[i]) for i in (0, 1)) > 0.4:
        self.relabel_until = now + 2.0
        self._reset_geometry()
        return
      self.rates = [0.5 * self.rates[i] + 0.5 * (prev_y[i] - y[i]) / dt for i in (0, 1)]
      for i in (0, 1):
        self.lane_since[i] = (now if self.lane_since[i] is None else self.lane_since[i]) if adjacent_lane(cs, md, i + 1, now) else None
      direction = (1 if cs.steeringTorque > 0 else 2 if cs.steeringTorque < 0 else 0) if cs.steeringPressed else 0
      if direction != self.intent:
        self.intent, self.intent_since = direction, now
      confirmed = direction if direction and now - self.intent_since >= .15 else 0
      if now < self.relabel_until:
        return
      if self.episode is None and confirmed and self.lane_since[confirmed - 1] is not None and now - self.lane_since[confirmed - 1] >= .2:
        self.episode = {'direction':confirmed, 'start':now, 'intent':now, 'initial':y[confirmed - 1],
                        'minimum':y[confirmed - 1], 'approach':now}
      episode = self.episode
      if episode is None:
        return
      direction = episode['direction']
      i = direction - 1
      if confirmed == direction:
        episode['intent'] = now
      if self.rates[i] > .1:
        episode['approach'] = now
      episode['minimum'] = min(episode['minimum'], y[i])
      # A brief release or countersteer does not erase continued approach. Actual
      # retreat, stalled approach, stale intent or lost adjacent lane does.
      if (self.lane_since[i] is None or now - episode['intent'] > 1.5 or now - episode['start'] > 4 or
          now - episode['approach'] > .3 or y[i] - episode['minimum'] > .15):
        self.episode = None
        self.relabel_until = now + .2
        return
      # The crossed boundary must have been observed recently. The opposite
      # boundary supplies geometric consistency, not a second confidence veto.
      observed = self.confident[i]
      if observed is None or not 0 <= now - observed <= TRACK_SEC:
        return
      # Both boundaries must move consistently; widening forks are not crossings.
      if (episode['initial'] - y[i] >= .15 and y[i] < 0.9 and self.rates[i] > 0.4 and
          self.rates[1 - i] < -0.4 and abs(sum(self.rates)) < 0.35):
        self.request_direction, self.request_time = direction, now
        self.blocked |= direction
        self.block_until, self.isolate_until, self.dark_since = now + QUIET_SEC, now + ISOLATE_SEC, None
    except (AttributeError, IndexError, TypeError, ValueError):
      self._reset_geometry()

  def filter_lamp(self, direction):
    return 0 if direction & self.blocked else direction
