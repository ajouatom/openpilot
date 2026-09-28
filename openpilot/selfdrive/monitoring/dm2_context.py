"""DM2 context and input edges. No vehicle actuation or changes to radar selection."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ObjectObservation:
  x: float
  y: float
  speed: float
  relative_speed: float


class TrafficContext:
  STRICT_SECONDS = 10.0
  FORGET_SECONDS = 2.0

  def __init__(self):
    self.tracks = []
    self.strict_until = 0.0
    self.clear_since = None

  def update(self, now, objects, healthy, straight, coverage):
    """Associate positions, including vision-only IDs and lane changes.

    Ten seconds starts on entry, not every occupied frame. Brief dropouts retain
    identity; an unobserved interval cannot be evidence of an empty road.
    """
    if not healthy:
      self.clear_since = None
      return True, False
    old = [(t, obj) for t, obj in self.tracks if now - t <= self.FORGET_SECONDS]
    remaining = set(range(len(old)))
    current = []
    new_moving = False
    for obj in objects:
      if not all(math.isfinite(v) for v in (obj.x, obj.y, obj.speed, obj.relative_speed)):
        self.clear_since = None
        return True, False
      if not (-10 <= obj.x <= 150 and abs(obj.y) <= 6):
        continue
      # Collapse duplicates across leadOne/leadTwo/adjacent lists.
      if any(abs(obj.x - other.x) < 2 and abs(obj.y - other.y) < 1 for _, other in current):
        continue
      candidates = [(abs(obj.x - (prev.x + prev.relative_speed * (now - t))) + 3 * abs(obj.y - prev.y), i)
                    for i in remaining for t, prev in [old[i]]
                    if abs(obj.x - (prev.x + prev.relative_speed * (now - t))) < 8 and abs(obj.y - prev.y) < 2]
      if candidates:
        remaining.remove(min(candidates)[1])
      elif abs(obj.speed) > 1.0:
        new_moving = True
      current.append((now, obj))
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
    self.last_pedal_or_speed = -math.inf

  def update(self, now, gas, brake, steering, buttons):
    state = (bool(gas), bool(brake), bool(steering))
    edges = tuple(value and not prev for value, prev in zip(state, self.previous, strict=True))
    self.previous = state
    speed_edge = False
    button_edge = False
    for kind, pressed in buttons:
      if kind not in self.RESPONSE_BUTTONS:
        continue
      if pressed:
        button_edge |= kind not in self.held_buttons
        speed_edge |= kind in self.SPEED_BUTTONS and kind not in self.held_buttons
        self.held_buttons.add(kind)
      else:
        self.held_buttons.discard(kind)
    # A continuously held pedal/button/steering signal is not repeated evidence.
    response = any(edges) or button_edge
    if response:
      self.last_response = now
    if edges[0] or edges[1] or speed_edge:
      self.last_pedal_or_speed = now
    return response

  def timeout_factor(self, now, experimental, strict, clear):
    if not experimental or strict:
      return 1.0
    # Both bonuses are bounded; an automatic vCruise/vEgo change is never an input.
    return 2.0 + (0.2 if now - self.last_pedal_or_speed < 15 else 0) + (0.2 if clear else 0)


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
