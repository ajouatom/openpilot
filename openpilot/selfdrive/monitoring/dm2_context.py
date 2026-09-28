"""DM2 context and input edges. No vehicle actuation or changes to radar selection."""
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
