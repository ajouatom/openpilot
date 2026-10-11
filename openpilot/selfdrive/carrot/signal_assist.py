"""Experimental signal supervision; never grants acceleration permission.

Forward-image selection is a hypothesis, not proof of the applicable ego lane.
Normal devices leave the explicit local opt-in disabled. Replay must pass actual
arrival times, fresh vehicle status and unmodified observer records.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SignalDecision:
  hold: bool = False
  red_sign: bool = False
  released: bool = False
  evidence: str = 'unknown'
  reason: str = 'disabled'
  track_id: int | None = None


class SignalAssist:
  MAX_AGE = .2
  RED_CONFIRM = .3
  GREEN_CONFIRM = .4
  MAX_GAP = .25

  def __init__(self):
    self.reset()
    self.last_now = None
    self.suppressed_until = -math.inf

  def reset(self):
    self.hold = False
    self.committed = False
    self.session = None
    self.selected = None
    self.box = None
    self.last_frame = -1
    self.last_timestamp = -math.inf
    self.pending = 'unknown'
    self.pending_since = -math.inf
    self.count = 0
    self.confirmed = 'unknown'

  def _forget_track(self):
    self.selected = None
    self.box = None
    self.pending = self.confirmed = 'unknown'
    self.count = 0

  @staticmethod
  def _front(track):
    try:
      x1, y1, x2, y2 = track['box']
      return (all(math.isfinite(v) for v in (x1, y1, x2, y2, track['age']))
              and 0 <= x1 < x2 <= 1344 and 0 <= y1 < y2 <= 760
              and .3 <= (x1 + x2) / 2688 <= .75 and .08 <= (y1 + y2) / 1520 <= .55
              and 0 <= track['age'] <= .125 and track['observations'] >= 3)
    except (KeyError, TypeError, ValueError):
      return False

  def _observe(self, now, observation):
    if not observation:
      return 'missing'
    try:
      timestamp = float(observation['timestamp'])
      frame = int(observation['frame_id'])
      session = observation['session']
      tracks = observation['tracks']
      if not math.isfinite(timestamp) or not 0 <= now - timestamp <= self.MAX_AGE + 1e-9:
        return 'stale_or_future'
      if session != self.session:
        # A restarted worker can reacquire RED, never inherit green permission.
        self._forget_track()
        self.last_timestamp, self.last_frame = -math.inf, -1
        self.session = session
      if timestamp <= self.last_timestamp or frame <= self.last_frame:
        return 'repeated_or_reordered'
      gap = timestamp - self.last_timestamp
      self.last_timestamp, self.last_frame = timestamp, frame
      eligible = [t for t in tracks if self._front(t)]
      # Night proposals can represent both the housing and its individual red
      # lamp. A nested red-only box cannot observe the green lamp on the right.
      # Prefer the containing, independently tracked housing before selection.
      def nested_lamp(small, large):
        a, b = small['box'], large['box']
        sa, ba = (a[2]-a[0])*(a[3]-a[1]), (b[2]-b[0])*(b[3]-b[1])
        intersection = max(0, min(a[2],b[2])-max(a[0],b[0])) * max(0, min(a[3],b[3])-max(a[1],b[1]))
        return small['id'] != large['id'] and sa < .55 * ba and intersection > .8 * sa
      eligible = [t for t in eligible if not any(nested_lamp(t, other) for other in eligible)]
      selected = next((t for t in eligible if t['id'] == self.selected), None)
      if selected is None:
        # Acquire only an observed red. Green in a different/new track cannot
        # release an already latched stop, even at a plausible image position.
        red = [t for t in eligible if t['state'] == 'red' and t['evidence']['raw'] == 'red']
        if not red:
          self.pending = self.confirmed = 'unknown'
          self.count = 0
          return 'selected_track_missing'
        selected = min(red, key=lambda t: abs((t['box'][0] + t['box'][2]) / 2688 - .5))
        if selected['id'] != self.selected:
          self._forget_track()
          self.selected = selected['id']
      self.box = selected['box']
      raw = selected['evidence']['raw']
      state = selected['state'] if raw == selected['state'] else 'unknown'
      if state not in ('red', 'green'):
        state = 'unknown'
      if state == 'unknown' or state != self.pending or gap > self.MAX_GAP:
        self.pending, self.pending_since, self.count = state, timestamp, 1
        self.confirmed = 'unknown'
      else:
        self.count += 1
        duration = self.RED_CONFIRM if state == 'red' else self.GREEN_CONFIRM
        if self.count >= 3 and timestamp - self.pending_since >= duration - 1e-9:
          self.confirmed = state
      # A duty-limited producer may observe 3-4 consecutive green frames while
      # only two of its results arrive before the consumer's 200 ms deadline.
      # Reuse the producer's same-ID history instead of requiring a second
      # independent 400 ms streak. The final image must still be fresh, seen
      # now, raw green, confirmed green, and attached to our selected red ID.
      # Older producers omit these fields and retain the consumer-only path.
      since = selected.get('support_since')
      count = selected.get('support_count', 0)
      if (state == 'green' and selected.get('seen_red') is True and selected['age'] == 0
          and selected.get('support_state') == 'green'
          and isinstance(since, (int, float)) and math.isfinite(since)
          and isinstance(count, int) and 3 <= count <= selected['observations']
          and self.GREEN_CONFIRM - 1e-9 <= timestamp - since
          and timestamp - since <= (count - 1) * self.MAX_GAP + 1e-6):
        self.confirmed = 'green'
      return 'selected_forward_track'
    except (KeyError, TypeError, ValueError, OverflowError):
      self.pending = self.confirmed = 'unknown'
      self.count = 0
      return 'malformed'

  def update(self, now, observation, *, enabled, valid, drive, gas, speed,
             model_distance, model_y, comfort_brake, lead, entry_allowed, turning=False):
    values = (now, speed, model_distance, model_y, comfort_brake)
    finite = all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
    rollback = self.last_now is not None and now < self.last_now
    self.last_now = now
    if not finite or rollback or not enabled or not valid or not drive:
      self.reset()
      return SignalDecision(reason='inactive_or_invalid')
    if gas:
      self.reset()
      self.suppressed_until = now + 10.
      return SignalDecision(reason='driver_gas_override')
    if now < self.suppressed_until:
      return SignalDecision(reason='driver_override_cooldown')
    reason = self._observe(now, observation)
    fresh = 0 <= now - self.last_timestamp <= self.MAX_AGE + 1e-9
    evidence = self.confirmed if fresh else 'unknown'
    released = False
    if evidence == 'green':
      released = self.hold or self.committed
      self.hold = self.committed = False
      reason = 'confirmed_same_track_green'
    elif evidence == 'red' and abs(speed) <= .3 and not turning:
      self.hold = True
      reason = 'confirmed_red_stop'
    elif (evidence == 'red' and .3 < speed < 82 / 3.6 and not turning
          and not lead and entry_allowed and abs(model_y) < 5 and comfort_brake > 0):
      # Use the existing model's distance, never derive distance from lamp color.
      # Exclude implausibly short distances requiring more than comfort braking.
      minimum = max(5., speed * speed / (2 * comfort_brake))
      maximum = 120. + min(1., max(0., (speed * 3.6 - 60.) / 20.)) * 30.
      if minimum <= model_distance <= maximum:
        self.committed = True
        reason = 'confirmed_red_with_model_distance'
      else:
        reason = 'red_without_usable_stop_distance'
    if self.committed and abs(speed) <= .3:
      self.hold = True
    if self.hold and evidence == 'unknown':
      reason = 'hold_through_unknown'
    # Unknown cannot undo a committed stop. Driver gas, disengagement/invalidity,
    # parking or confirmed same-track green provide explicit exit paths.
    return SignalDecision(self.hold, self.committed or self.hold, released, evidence, reason, self.selected)
