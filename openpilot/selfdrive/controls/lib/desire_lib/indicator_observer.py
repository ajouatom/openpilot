"""Lamp-only observations of model motion. No lane-change desire is produced."""
import math
from collections import deque

from openpilot.selfdrive.controls.lib.desire_lib.boundary_indicator import QUIET_SEC
from openpilot.selfdrive.controls.lib.desire_lib.boundary_lane import adjacent_lane
from openpilot.selfdrive.controls.lib.desire_lib.indicator_feedback import TapEcho
from openpilot.selfdrive.controls.lib.desire_lib.manual_turn_indicator import ManualTurnIndicator

LINE_CONFIDENCE = .10
CONFIRM_SEC = .20
MANUAL_TURN_QUIET_SEC = 2.


def interpolate(x, xs, ys):
  xs, ys = list(xs), list(ys)
  if len(xs) != len(ys) or len(xs) < 2 or not all(math.isfinite(v) for v in (*xs, *ys, x)):
    raise ValueError('geometry')
  if any(b <= a for a, b in zip(xs, xs[1:], strict=False)) or x < xs[0] or x > xs[-1]:
    raise ValueError('range')
  for i in range(1, len(xs)):
    if x <= xs[i]:
      f = (x-xs[i-1])/(xs[i]-xs[i-1])
      return ys[i-1] + f*(ys[i]-ys[i-1])
  raise ValueError('range')


def path_direction(md, curvature):
  """Compare curved boundary and center path at identical forward positions."""
  try:
    if not math.isfinite(curvature):
      return 0
    p = md.position
    for direction, index, sign in ((1, 1, -1), (2, 2, 1)):
      if not math.isfinite(md.laneLineProbs[index]) or md.laneLineProbs[index] < LINE_CONFIDENCE:
        continue
      line = md.laneLines[index]
      near = [interpolate(x, line.x, line.y) for x in (5., 10., 15.)]
      slope = (near[2]-near[0])/10.
      lane_curvature = (near[2]-2*near[1]+near[0])/25. / (1+slope*slope)**1.5
      if (curvature-lane_curvature)*sign <= .0001:
        continue
      margins = []
      for t in (1., 1.5, 2., 2.5, 3.):
        x = interpolate(t, p.t, p.x)
        y = interpolate(t, p.t, p.y)
        margins.append(sign*(y-interpolate(x, line.x, line.y)))
      # The center path starts inside, then crosses and stays across the boundary.
      if margins[0] < 0 and margins[-1] >= .15 and margins[-2] >= .05 and margins[-1] > margins[0]+.3:
        return direction
  except (AttributeError, IndexError, TypeError, ValueError):
    pass
  return 0


def following_road_curve(md, speed, yaw_rate):
  """Recognized curved lane geometry agreeing with the vehicle's current arc."""
  try:
    if yaw_rate is None or speed < 1 or min(md.laneLineProbs[1:3]) < .5:
      return False
    curvature = yaw_rate / speed
    if abs(curvature) < .002:
      return False
    curves = []
    for line in md.laneLines[1:3]:
      near = [interpolate(x, line.x, line.y) for x in (5., 10., 15.)]
      slope = (near[2] - near[0]) / 10.
      curves.append((near[2] - 2 * near[1] + near[0]) / 25. / (1 + slope * slope)**1.5)
    return all(c * curvature > 0 and abs(c - curvature) <= max(.002, abs(curvature) * .5) for c in curves)
  except (AttributeError, IndexError, TypeError, ValueError):
    return False


def anticipated_lane(md, samples):
  """Confirm the two nearest forecast rows against measured motion of both lane boundaries."""
  forecast = getattr(md.meta, 'desirePrediction', ())
  if len(forecast) != 32 or len(samples) < 2:
    return 0
  old_t, old_l, old_r, _, _ = samples[0]
  t, left, right, _, _ = samples[-1]
  if not .30 <= t-old_t <= .55:
    return 0
  for side, index, sign in ((1, 3, -1), (2, 4, 1)):
    if (0 < (-left if side == 1 else right) <= 1.5 and
        min(sign*(old_l-left), sign*(old_r-right)) >= .06 and
        any(sample[2+side] >= LINE_CONFIDENCE for sample in samples)):
      # Two 8-class softmax rows; check the cheap geometric gates first.
      rows = ([forecast[j+i] for i in range(8)] for j in (0, 8))
      if all(all(math.isfinite(x) and 0 <= x <= 1 for x in row) and
             row[index] >= .65 and row[index] == max(row) for row in rows):
        return side
  return 0


class Movement:
  """Bounded observation lifetime, including return-before-crossing cancellation."""
  def __init__(self, now, direction, source, cs, md):
    self.direction, self.source, self.started = direction, source, now
    self.initial = abs(md.laneLines[direction].y[0])
    self.minimum = self.initial
    self.last = (now, md.laneLines[1].y[0], md.laneLines[2].y[0])
    self.crossed = False
    self.settled = None
    self.peak_angle = 0.
    self.reason = 'active'

  def update(self, now, cs, md):
    try:
      left, right = md.laneLines[1].y[0], md.laneLines[2].y[0]
      angle = cs.steeringAngleDeg
      if not all(math.isfinite(v) for v in (now, left, right, angle)) or (self.source != 'model_turn' and not 2.5 < right-left < 4.8):
        self.reason = 'geometry'
        return False
      stamp, old_left, old_right = self.last
      self.last = (now, left, right)
      if not 0 < now-stamp <= .15 or now-self.started >= (30 if self.source == 'model_turn' else 12):
        self.reason = 'timeout'
        return False
      if self.source == 'model_turn':
        signed_angle = angle*(1 if self.direction == 1 else -1)
        self.peak_angle = max(self.peak_angle, signed_angle)
        done = self.peak_angle >= 30 and abs(angle) <= 10 and cs.vEgo > 1/3.6
      else:
        selected = -left if self.direction == 1 else right
        old_selected = -old_left if self.direction == 1 else old_right
        # A pair shifting one lane after reaching its edge is a completed crossing,
        # not a new request. An arbitrary relabel far from the boundary is invalid.
        jump = max(abs(left-old_left), abs(right-old_right)) > .5
        relabel = jump and self.minimum < .9 and selected-old_selected > 1.5 and abs((left-old_left)-(right-old_right)) < .5
        if jump and not relabel:
          self.reason = 'relabel'
          return False
        self.crossed |= selected <= 0 or relabel
        self.minimum = min(self.minimum, selected)
        if not self.crossed and selected-self.minimum > .2:
          self.reason = 'returned'
          return False
        done = self.crossed and left < 0 < right and min(-left, right) > 1.0 and abs(selected-old_selected) < .08
      self.settled = (now if self.settled is None else self.settled) if done else None
      if self.settled is not None and now-self.settled >= .4:
        self.reason = 'completed'
        return False
      return True
    except (AttributeError, IndexError, TypeError, ValueError):
      self.reason = 'geometry'
      return False


class ForecastLaneMovement(Movement):
  """A split can widen before the model assigns the boundaries of the new lane."""
  def __init__(self, now, direction, source, cs, md):
    super().__init__(now, direction, source, cs, md)
    self.far_start = self.far_peak = abs(md.laneLines[3-direction].y[0])
    self.progress_at = now
    self.split = False

  def update(self, now, cs, md):
    left, right = md.laneLines[1].y[0], md.laneLines[2].y[0]
    near, far = (-left, right) if self.direction == 1 else (right, -left)
    stamp, old_left, old_right = self.last
    old_near, old_far = (-old_left, old_right) if self.direction == 1 else (old_right, -old_left)
    if (not all(math.isfinite(v) for v in (now, left, right, cs.vEgo)) or not 0 < now-stamp <= .15 or
        cs.steeringPressed or not left < 0 < right or not 2.5 < right-left < 5.5 or cs.vEgo < 30/3.6):
      self.reason = 'cancelled'
      return False
    widening = (near > self.minimum + .20 and far >= self.far_start + .20 and
                near + far >= self.initial + self.far_start + .30)
    if not self.split and not widening:
      return super().update(now, cs, md)
    self.split = True
    index = self.direction + 2
    pred, current = md.meta.desirePrediction, md.meta.desireState
    if len(pred) != 32 or len(current) != 8:
      self.reason = 'inputs'
      return False
    rows = ([pred[i] for i in range(8)], list(current))
    if (not all(math.isfinite(p) and 0 <= p <= 1 for row in rows for p in row) or
        any(max(row[3-self.direction], row[5-self.direction]) >= .5 for row in rows) or
        not any(row[index] >= .55 and row[index] == max(row) for row in rows)):
      self.reason = 'completed'
      return False
    # Both labels moving together into the new lane is different from the
    # target line alone moving away. Require evidence from the far side first.
    relabel = (self.far_peak >= self.far_start + .55 and self.far_peak-far >= .60 and
               near-self.minimum >= .55 and near-old_near > .15 and old_far-far > .15)
    if max(abs(near-old_near), abs(far-old_far)) > 1.5 or (
        not self.crossed and not relabel and self.far_peak-far > .45):
      self.reason = 'returned'
      return False
    self.crossed |= relabel
    settled = self.crossed and min(near, far) > 1. and abs(near-old_near) < .08
    self.settled = (now if self.settled is None else self.settled) if settled else None
    if self.settled is not None and now-self.settled >= .4:
      self.reason = 'completed'
      return False
    if far >= self.far_peak + .05:
      self.far_peak, self.progress_at = far, now
    if now-self.started >= 12 or (not self.crossed and now-self.progress_at > .75):
      self.reason = 'timeout'
      return False
    self.last = (now, left, right)
    return True


class ModelIndicator:
  def __init__(self):
    self.active = None
    self.request_time = 0.
    self.candidate = None
    self.approach = deque(maxlen=10)
    self.evidence = deque(maxlen=12)
    self.since = 0.
    self.previous = None
    self.reason = 'idle'
    self.manual_seen = 0.
    self.tap_echo = TapEcho()
    self.manual_turn = ManualTurnIndicator()

  def update(self, now, cs, md, *, valid, lateral_active, desire_enabled, path_enabled, busy, isolation, manual_edge=0, command_curvature=None,
             feedback=None, manual_turn_enabled=False, yaw_rate=None):
    self.reason = 'idle'
    if self.active is None and not manual_edge and not (desire_enabled or path_enabled or manual_turn_enabled):
      self.manual_turn.reset('disabled')
      self.evidence.clear()
      self.previous = None
      self.reason = 'disabled'
      return
    try:
      echo = self.active is not None and self.tap_echo.matches(now, cs, self.active.direction, self.request_time, feedback)
      common = (valid and cs.canValid and str(cs.gearShifter) == 'drive' and not cs.trailerConnected and
                not cs.steerFaultTemporary and not cs.steerFaultPermanent and (not cs.blinkerLever or echo) and
                math.isfinite(cs.vEgo) and math.isfinite(cs.steeringTorque))
      if self.active:
        self.manual_turn.reset('busy')
        a = self.active
        allowed = (manual_turn_enabled if a.source == 'manual_turn' else a.source == 'manual' or lateral_active and
                   (path_enabled if a.source == 'model_path' else desire_enabled))
        opposite_lamp = cs.rightBlinker if a.direction == 1 else cs.leftBlinker
        if not common or busy or not allowed or opposite_lamp or not a.update(now, cs, yaw_rate if a.source == 'manual_turn' else md):
          self.reason = a.reason if common and allowed and not busy and not opposite_lamp else 'cancelled'
          self.active = None
        else:
          # Keep ownership throughout the hold, including a stopped/long turn.
          isolation.own(now, a.direction)
          self.reason = a.source
        self.candidate = None
        self.evidence.clear()
        self.previous = None
        return
      if manual_edge and common and isolation.request_time != self.manual_seen:
        self.manual_turn.reset('busy')
        self.manual_seen = isolation.request_time
        self.active = Movement(now, manual_edge, 'manual', cs, md)
        self.request_time = now
        self.reason = 'manual'
        return
      # A new driver turn may be observed before other automatic sources rearm.
      # Lamp masking and the uninterrupted dark interval still apply.
      manual_turn_after = isolation.block_until - QUIET_SEC + MANUAL_TURN_QUIET_SEC
      # At low speeds a lane-boundary relabel is common in parking areas. A
      # manual turn uses measured steering and yaw independently of those lines.
      lane_labels_ready = cs.vEgo < 30 / 3.6 or now >= isolation.relabel_until
      turn = self.manual_turn.update(now, cs, md.frameId, yaw_rate,
        allowed=(manual_turn_enabled and common and not busy and not cs.leftBlinker and not cs.rightBlinker and
                 now >= manual_turn_after and lane_labels_ready and
                 isolation.dark_since is not None and now - isolation.dark_since >= 1.),
        following_curve=manual_turn_enabled and following_road_curve(md, cs.vEgo, yaw_rate))
      if turn is not None:
        self.active, self.request_time, self.reason = turn, now, turn.source
        isolation.own(now, turn.direction)
        self.candidate, self.previous = None, None
        self.evidence.clear()
        return
      if (not common or not lateral_active or busy or cs.leftBlinker or cs.rightBlinker or
          cs.steeringPressed or cs.vEgo < 10/3.6 or now < isolation.block_until or
          now < isolation.relabel_until or isolation.blocked):
        self.candidate, self.previous = None, None
        self.evidence.clear()
        self.reason = 'admission'
        return
      left, right = md.laneLines[1].y[0], md.laneLines[2].y[0]
      old, self.previous = self.previous, (now, md.frameId, left, right)
      if (old is None or not all(math.isfinite(v) for v in (left, right)) or not left < 0 < right or
          not 2.5 < right-left < 4.8 or not .02 <= now-old[0] <= .15 or md.frameId <= old[1] or
          max(abs(left-old[2]), abs(right-old[3])) > .4):
        self.candidate = None
        self.reason = 'geometry'
        self.approach.clear()
        self.evidence.clear()
        return
      candidate = None
      forecast = False
      if desire_enabled and cs.vEgo >= 30/3.6:
        self.approach.append((now, left, right, md.laneLineProbs[1], md.laneLineProbs[2]))
      else:
        self.approach.clear()
      if desire_enabled:
        probs = list(md.meta.desireState)
        if len(probs) >= 5 and all(math.isfinite(p) and 0 <= p <= 1 for p in probs):
          index = max(range(1, 5), key=lambda i: probs[i])
          direction = 1 if index in (1, 3) else 2
          # Dominant categorical prediction, not merely a nonzero softmax value.
          opposite = max(probs[i] for i in ((2, 4) if direction == 1 else (1, 3)))
          if probs[index] >= .5 and probs[index] >= probs[0] and probs[index]-opposite >= .3:
            if index < 3 or cs.vEgo >= 30/3.6:
              candidate = (direction, 'model_turn' if index < 3 else 'model_desire')
          if candidate is None and probs[0] == max(probs) and cs.vEgo >= 30/3.6:
            direction = anticipated_lane(md, self.approach)
            if direction:
              forecast = True
              candidate = (direction, 'model_desire')
      if candidate is None and path_enabled and cs.vEgo >= 30/3.6:
        direction = path_direction(md, command_curvature) if command_curvature is not None else 0
        if direction:
          candidate = (direction, 'model_path')
      if candidate and candidate[1] != 'model_turn' and not adjacent_lane(cs, md, candidate[0], now):
        candidate = None
      self.evidence.append((now, candidate))
      evidence = [(t,c) for t,c in self.evidence if now-t <= .4]
      matches = [t for t,c in evidence if c == candidate and c is not None]
      if matches:
        evidence = [(t,c) for t,c in evidence if t >= matches[0]]
      opposite = any(c is not None and candidate is not None and c[0] != candidate[0] for _,c in evidence)
      self.candidate = candidate
      if candidate and not opposite and len(matches) >= 4 and len(matches)*2 >= len(evidence) and now-matches[0] >= CONFIRM_SEC:
        movement = ForecastLaneMovement if forecast else Movement
        self.active = movement(now, *candidate, cs, md)
        self.request_time = now
        isolation.own(now, candidate[0])
        self.reason = candidate[1]
    except (AttributeError, IndexError, TypeError, ValueError):
      self.active, self.candidate, self.previous = None, None, None
      self.reason = 'invalid'
      self.manual_turn.reset('invalid')
