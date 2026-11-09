"""Driver turn observations used exclusively by the physical-indicator path."""
import math
from openpilot.selfdrive.controls.lib.desire_lib.boundary_lane import MANUAL_INDICATOR_MIN_SPEED


class ManualTurnMovement:
  source = 'manual_turn'

  def __init__(self, now, direction):
    self.direction = direction
    self.started = self.previous = now
    self.settled = None
    self.reason = 'active'

  def update(self, now, cs, yaw_rate):
    if yaw_rate is None or not math.isfinite(yaw_rate) or not math.isfinite(cs.steeringAngleDeg):
      self.reason = 'inputs'
      return False
    elapsed = now - self.previous
    self.previous = now
    if not 0 < elapsed <= .15 or now - self.started >= 30:
      self.reason = 'timeout'
      return False
    sign = 1 if self.direction == 1 else -1
    if cs.steeringPressed and cs.steeringTorque * sign < 0 and cs.steeringAngleDeg * sign < 0:
      self.reason = 'driver_opposite'
      return False
    returned = abs(cs.steeringAngleDeg) <= 15 and abs(yaw_rate) <= math.radians(3) and cs.vEgo >= 1 / 3.6
    self.settled = (now if self.settled is None else self.settled) if returned else None
    if self.settled is not None and now - self.settled >= .3:
      self.reason = 'completed'
      return False
    return True


class ManualTurnIndicator:
  def __init__(self):
    self.previous = None
    self.intent = 0
    self.intent_since = 0.
    self.last_pressed = 0.
    self.pressed_since = None
    self.intent_confirmed = False
    self.initial_angle = 0.
    self.rotation = 0.
    self.confirmed_since = None
    self.candidate = None
    self.reason = 'idle'

  def reset(self, reason):
    self.previous = None
    self.intent = 0
    self.pressed_since = None
    self.intent_confirmed = False
    self.rotation = 0.
    self.confirmed_since = None
    self.candidate = None
    self.reason = reason

  def update(self, now, cs, frame, yaw_rate, *, allowed, following_curve=False):
    self.candidate = None
    if (not allowed or yaw_rate is None or not all(math.isfinite(v) for v in
        (now, yaw_rate, cs.vEgo, cs.steeringAngleDeg, cs.steeringTorque)) or not MANUAL_INDICATOR_MIN_SPEED <= cs.vEgo <= 60 / 3.6):
      self.reset('admission')
      return None
    previous, self.previous = self.previous, (now, frame, cs.steeringAngleDeg)
    if previous is None:
      return None
    dt = now - previous[0]
    if not .02 <= dt <= .15 or frame <= previous[1] or abs(cs.steeringAngleDeg - previous[2]) > 100:
      self.reset('inputs')
      return None
    direction = (1 if cs.steeringTorque > 0 else 2 if cs.steeringTorque < 0 else 0) if cs.steeringPressed else 0
    if direction and direction != self.intent:
      self.intent, self.intent_since = direction, now
      self.initial_angle = previous[2]
      self.rotation = 0.
      self.confirmed_since = None
      self.pressed_since = now
      self.intent_confirmed = False
    if direction:
      self.last_pressed = now
      self.pressed_since = now if self.pressed_since is None else self.pressed_since
      self.intent_confirmed |= now - self.pressed_since >= .15
    else:
      self.pressed_since = None
    if not self.intent or now - self.last_pressed > .8 or now - self.intent_since > 4:
      self.reset('driver')
      return None
    sign = 1 if self.intent == 1 else -1
    # Pose z is positive for right rotation, while Hyundai steering is positive left.
    signed_yaw = -yaw_rate * sign
    self.rotation += signed_yaw * dt
    if following_curve:
      self.confirmed_since = None
      self.reason = 'road_curve'
      return None
    self.candidate = (self.intent, 'manual_turn')
    self.reason = 'confirming'
    turn = (self.intent_confirmed and cs.steeringAngleDeg * sign >= 42 and
            (cs.steeringAngleDeg - self.initial_angle) * sign >= 22 and
            signed_yaw >= math.radians(3) and self.rotation >= math.radians(1.5))
    self.confirmed_since = (now if self.confirmed_since is None else self.confirmed_since) if turn else None
    if self.confirmed_since is not None and now - self.confirmed_since >= .10:
      movement = ManualTurnMovement(now, self.intent)
      self.reset('manual_turn')
      return movement
    return None
