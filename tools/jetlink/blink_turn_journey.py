"""TBT turn lifetime anchored to wheel-speed distance, with driver abort evidence."""
import math


class Travel:
  def __init__(self):
    self.previous = None
    self.distance = 0.
    self.epoch = 0

  def update(self, now, speed, valid):
    old, self.previous = self.previous, (now, speed) if valid and math.isfinite(speed) and speed >= 0 else None
    if self.previous is None or old is not None and not 0 < now-old[0] <= .35:
      self.epoch += 1
      return None
    if old is not None:
      self.distance += (speed+old[1])*.5*(now-old[0])
    return self.epoch, self.distance


class TurnJourney:
  def __init__(self):
    self.cue = None
    self.mode = None
    self.active = None
    self.pending = None
    self.reason = 'idle'
    self.cancelled_until = 0.

  def start(self, now, direction, distance, travel, heading, target):
    self.active = {'direction':direction, 'start':now, 'end':travel[1]+distance+30.,
                   'turn':travel[1]+distance, 'epoch':travel[0], 'heading':heading,
                   'rotation':0., 'entered':False,
                   'target':target, 'centred':None, 'peak_angle':0.}
    self.reason = 'approach'

  def update(self, now, mode, valid, cue, direction, atc_type, *, travel, distance, speed, angle,
             heading, pressed, torque, target=None, cancelled=False):
    previous, self.cue = self.cue, cue
    changed, self.mode = self.mode != mode, mode
    if (not valid or not mode or changed or travel is None or heading is None or cancelled or
        not all(math.isfinite(v) for v in (now, cue, speed, angle, heading, torque, travel[1]))):
      self.active, self.pending, self.reason = None, None, 'inputs'
      return None
    fresh = (previous is not None and cue != previous and 0 <= now-cue < .35 and
             direction in ('left', 'right') and atc_type == 'turn '+direction and
             distance is not None and math.isfinite(distance) and 0 <= distance <= 250 and
             now >= self.cancelled_until)
    if fresh:
      if self.active is None:
        self.start(now, direction, distance, travel, heading, target)
      else:
        a = self.active
        next_turn = travel[1]+distance
        same_target = (a['target'] is not None and target is not None and a['target'] == target)
        # Repeated audio for the same approach must not keep pushing the end away.
        if not same_target and next_turn > a['turn']+10:
          # Preserve a separate next cue, including consecutive same-side turns.
          # Returning the wheel completes this turn, then hands over without
          # replaying a tap for a same-direction continuation.
          self.pending = (direction, next_turn, target)
    a = self.active
    if not a:
      return None
    if (a['epoch'] != travel[0] or now-a['start'] >= 300 or
        (a['target'] is not None and target is not None and a['target'][2] != target[2])):
      self.active, self.pending, self.reason = None, None, 'route_or_distance'
      return None
    sign = 1 if a['direction'] == 'left' else -1
    delta = math.atan2(math.sin(heading-a['heading']), math.cos(heading-a['heading'])) * -sign
    a['heading'] = heading
    if speed >= 1/3.6:
      a['rotation'] += delta
    a['peak_angle'] = max(a['peak_angle'], angle*sign)
    a['entered'] |= angle*sign >= 30 and a['rotation'] >= math.radians(20)
    # The driver can abandon a started turn by steering through centre into
    # the opposite direction. A receiver status alone is not steering intent.
    opposite = (a['peak_angle'] >= 30 and pressed and torque*sign < 0 and
                angle*sign < 0 and speed >= 1/3.6)
    if opposite:
      self.active, self.pending, self.reason = None, None, 'driver_opposite'
      self.cancelled_until = now+3.
      return None
    centred = abs(angle) <= 10 and speed >= 1/3.6
    a['centred'] = (now if a['centred'] is None else a['centred']) if centred else None
    complete = a['centred'] is not None and now-a['centred'] >= .4
    # Only a turn with measured rotation and selected-side steering can finish
    # on wheel return. Approach/traffic-light waits must not end at a straight
    # wheel. No settling delay after return within 15 degrees while moving.
    returned = a['entered'] and a['peak_angle'] > 30 and abs(angle) <= 15 and speed >= 1/3.6
    expired = travel[1] >= a['end'] and (complete or travel[1] >= a['end']+30)
    if returned or expired:
      self.active, self.reason = None, 'completed'
      if self.pending:
        next_direction, next_turn, next_target = self.pending
        self.pending = None
        if -30 < next_turn-travel[1] <= 250:
          self.start(now, next_direction, next_turn-travel[1], travel, heading, next_target)
    if self.active:
      self.reason = 'turning' if self.active['entered'] else 'approach'
      return ('turn', self.active['direction'])
    return None
