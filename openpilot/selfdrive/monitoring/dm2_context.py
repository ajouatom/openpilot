"""DM2 context and input edges. No vehicle actuation or changes to radar selection."""
from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObjectObservation:
  x: float
  y: float
  speed: float
  relative_speed: float


@dataclass(frozen=True)
class TrafficTrack:
  last_seen: float
  first_seen: float
  observation: ObjectObservation
  confirmed: bool


class TrafficContext:
  STRICT_SECONDS = 20.0
  FORGET_SECONDS = 2.0
  MIN_MOVING_SPEED = 2.0  # ground speed, m/s; equal-speed traffic still counts
  CONFIRM_SECONDS = 0.2

  def __init__(self):
    self.tracks = []
    self.strict_until = 0.0
    self.clear_since = None

  def update(self, now, objects, healthy, straight, coverage):
    """Associate positions, including vision-only IDs and lane changes.

    Twenty seconds starts on confirmed entry, not every occupied frame. Moving
    candidates immediately revoke the empty-road bonus; persistence prevents a
    single radar spike from starting the full override. No radar outputs change.
    """
    if not healthy:
      self.clear_since = None
      return True, False
    old = [track for track in self.tracks if now - track.last_seen <= self.FORGET_SECONDS]
    remaining = set(range(len(old)))
    current = []
    new_moving = False
    for obj in objects:
      if not all(math.isfinite(v) for v in (obj.x, obj.y, obj.speed, obj.relative_speed)):
        self.clear_since = None
        return True, False
      if not (-10 <= obj.x <= 150 and abs(obj.y) <= 6):
        continue
      if abs(obj.speed) < self.MIN_MOVING_SPEED:
        continue
      # Collapse duplicates across leadOne/leadTwo/adjacent lists.
      if any(abs(obj.x - track.observation.x) < 2 and abs(obj.y - track.observation.y) < 1 for track in current):
        continue
      candidates = [(abs(obj.x - (prev.x + prev.relative_speed * (now - track.last_seen))) + 3 * abs(obj.y - prev.y), i)
                    for i in remaining for track in [old[i]] for prev in [track.observation]
                    if abs(obj.x - (prev.x + prev.relative_speed * (now - track.last_seen))) < 8 and abs(obj.y - prev.y) < 2]
      first_seen, confirmed = now, False
      if candidates:
        index = min(candidates)[1]
        remaining.remove(index)
        previous = old[index]
        first_seen = previous.first_seen if now - previous.last_seen <= 0.3 else now
        confirmed = previous.confirmed
      if not confirmed and now - first_seen >= self.CONFIRM_SECONDS:
        confirmed = True
        new_moving = True
      current.append(TrafficTrack(now, first_seen, obj, confirmed))
    self.tracks = current + [old[i] for i in remaining]
    if new_moving:
      self.strict_until = now + self.STRICT_SECONDS
    clear = coverage and straight and not self.tracks
    self.clear_since = (now if self.clear_since is None else self.clear_since) if clear else None
    return now < self.strict_until, self.clear_since is not None and now - self.clear_since >= 10


class CancelPressSequence:
  """Detect three deliberate physical cancel presses within one time window."""
  REQUIRED_PRESSES = 3
  WINDOW_SECONDS = 3.0

  def __init__(self):
    self.press_count = 0
    self.first_press_time = -math.inf
    self.cancel_held = False

  def _reset_sequence(self):
    self.press_count = 0
    self.first_press_time = -math.inf

  def reset_input_stream(self):
    self._reset_sequence()
    self.cancel_held = False

  def update(self, event_time, buttons, driving):
    valid_driving = bool(driving) and math.isfinite(event_time)
    if not valid_driving:
      self._reset_sequence()

    triggered = False
    for kind, pressed in buttons:
      if kind != "cancel":
        self._reset_sequence()
        triggered = False
        continue

      if not pressed:
        self.cancel_held = False
        continue
      if self.cancel_held:
        continue

      self.cancel_held = True
      if not valid_driving:
        continue

      elapsed = event_time - self.first_press_time
      if self.press_count == 0 or elapsed < 0 or elapsed > self.WINDOW_SECONDS:
        self.press_count = 1
        self.first_press_time = event_time
      else:
        self.press_count += 1

      if self.press_count == self.REQUIRED_PRESSES:
        triggered = True
        self._reset_sequence()

    return triggered


class AutomaticCancelFilter:
  """Conservatively reject CANCEL edges correlated with our own request."""
  ECHO_WINDOW_SECONDS = 0.15
  RELEASE_SUPPRESSION_SECONDS = 0.5

  def __init__(self):
    self.requests = deque(maxlen=64)
    self.suppress_release = False
    self.suppress_release_until = -math.inf

  def record(self, event_time, requested):
    if requested and math.isfinite(event_time):
      self.requests.append(event_time)

  def reset_input_stream(self):
    self.suppress_release = False
    self.suppress_release_until = -math.inf

  def is_physical(self, event_time):
    if not math.isfinite(event_time):
      return False
    while self.requests and event_time - self.requests[0] > self.ECHO_WINDOW_SECONDS:
      self.requests.popleft()
    return not any(0 <= event_time - request <= self.ECHO_WINDOW_SECONDS for request in self.requests)

  def filter_buttons(self, event_time, buttons):
    """Remove an automatic CANCEL press and its paired release."""
    if math.isfinite(event_time) and event_time > self.suppress_release_until:
      self.suppress_release = False
    filtered = []
    for kind, pressed in buttons:
      if kind != "cancel":
        filtered.append((kind, pressed))
      elif pressed:
        automatic = not self.is_physical(event_time)
        if automatic:
          self.suppress_release = True
          self.suppress_release_until = event_time + self.RELEASE_SUPPRESSION_SECONDS
        else:
          filtered.append((kind, pressed))
      elif self.suppress_release:
        self.suppress_release = False
        self.suppress_release_until = -math.inf
      else:
        filtered.append((kind, pressed))
    return filtered


class InteractionEdges:
  SPEED_BUTTONS = {"accelCruise", "decelCruise", "resumeCruise", "setCruise"}
  RESPONSE_BUTTONS = SPEED_BUTTONS | {"gapAdjustCruise", "cancel", "mainCruise", "lkas", "lfaButton"}

  def __init__(self):
    self.previous = (False, False, False)
    self.held_buttons = set()
    self.last_response = -math.inf

  def update(self, now, gas, brake, steering, buttons):
    state = (bool(gas), bool(brake), bool(steering))
    edges = tuple(value and not prev for value, prev in zip(state, self.previous, strict=True))
    self.previous = state
    button_edge = False
    for kind, pressed in buttons:
      if kind not in self.RESPONSE_BUTTONS:
        continue
      if pressed:
        button_edge |= kind not in self.held_buttons
        self.held_buttons.add(kind)
      else:
        self.held_buttons.discard(kind)
    # A continuously held pedal/button/steering signal is not repeated evidence.
    response = any(edges) or button_edge
    if response:
      self.last_response = now
    return response


class SteeringTouchEvidence:
  """Continuous wheel contact is distinct from a new driver interaction."""
  def __init__(self):
    self.previous = None
    self.last_timestamp = 0

  def update(self, now, touch, car_valid):
    timestamp = touch.sampleMonoTime
    fresh = (car_valid and touch.available and touch.valid and timestamp > 0 and
             0 <= now - timestamp / 1e9 <= 0.25 and timestamp >= self.last_timestamp)
    if not fresh:
      self.previous = None
      return False, False
    held = touch.touched
    edge = held and self.previous is False and timestamp > self.last_timestamp
    self.previous = held
    self.last_timestamp = timestamp
    # Reconnection while already held is not a new press. A valid release must
    # precede each edge, so dropouts cannot repeatedly renew camera grace.
    return held, edge


class CameraAvailability:
  """Immediate fallback on unusable data, two seconds of health before recovery."""
  def __init__(self):
    self.healthy_since = None

  def update(self, now, usable):
    if not usable:
      self.healthy_since = None
      return False
    if self.healthy_since is None:
      self.healthy_since = now
    return now - self.healthy_since >= 2.0
