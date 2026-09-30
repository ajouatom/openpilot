"""Experimental angle-control authority recovery. This does not infer driver consent.

The legacy controller runs independently. Only the additional CAN authority ceiling
is managed here; physical torque is not estimated. steeringPressed and angle limits
are unchanged.
"""
from collections import deque
from math import atan, degrees, isfinite
from statistics import median


class SteeringHandover:
  def __init__(self):
    self.reset()

  def reset(self, mode=0, now=None):
    self.mode = mode
    self.last_time = now
    self.raw_history = deque(maxlen=3)
    self.history = deque()
    self.effort = None
    self.error = 0.0
    self.state = "waiting"
    self.cap = 0.0
    self.qualify = 0.0
    self.release = 0.0
    self.strong = 0.0
    self.armed = False
    self.last_strong = None
    self.low_since = None
    self.offer_since = 0.0
    self.offer_effort = 0.0
    self.offer_sign = 0.0
    self.previous_driver = 0.0
    self.direction_time = 0.0

  def update(self, *, mode, now, baseline, minimum, maximum, driver, threshold,
             pressed, target_error, command_error, speed, wheelbase, steer_ratio,
             active, valid):
    # Invalid modes and mode 0 are exact passthroughs, including disengaged zero.
    mode = mode if mode in (1, 2, 3) else 0
    if mode == 0 or mode != self.mode:
      self.reset(mode, now)
      return baseline

    dt = now - self.last_time if self.last_time is not None else 0.0
    self.last_time = now
    values = (now, dt, baseline, minimum, maximum, driver, threshold, target_error,
              command_error, speed, wheelbase, steer_ratio)
    if (not active or not valid or not all(isfinite(v) for v in values) or
        not 0.0 < dt <= 0.03 or threshold <= 0 or wheelbase <= 0 or steer_ratio <= 0 or
        not 0 <= minimum < maximum or not 0 <= baseline <= maximum):
      interrupted = active and self.state != "waiting"
      self.reset(mode, now)
      if interrupted:
        self.state = "blocked"
      return baseline

    raw = abs(driver) / threshold
    self.direction_time = self.direction_time + dt if driver * self.previous_driver > 0 else 0.0
    self.previous_driver = driver
    self.raw_history.append(raw)
    magnitude = median(self.raw_history)
    # Filter magnitude, not signed force: alternating opposition must not look
    # like release. Keep raw force for prompt vetoes of filtered decisions.
    self.effort = magnitude if self.effort is None else self.effort + dt / (0.12 + dt) * (magnitude - self.effort)
    error = max(abs(target_error), abs(command_error))
    self.error += dt / (0.08 + dt) * (error - self.error)
    self.history.append((now, self.effort, self.error))
    while len(self.history) > 1 and now - self.history[0][0] > 0.22:
      self.history.popleft()
    span = now - self.history[0][0]
    effort_rate = (self.effort - self.history[0][1]) / max(span, dt)
    error_rate = (self.error - self.history[0][2]) / max(span, dt)
    # Bound both the original target and the actually transmitted angle. At high
    # speed the bicycle-model acceleration equivalent tightens the two-degree gate.
    error_limit = min(2.0, degrees(atan(0.5 * wheelbase / max(speed * speed, 25.0))) * steer_ratio)
    aligned = error <= error_limit and self.error <= error_limit
    low = not pressed and raw <= 0.3 and magnitude <= 0.3
    self.release = self.release + dt if low else 0.0

    # A brief zero crossing alone is insufficient. Arm on sustained override,
    # then confirm a rapid fall with 100 ms of continuously low, unpressed force.
    if pressed and magnitude > 1.0 and raw > 1.0:
      self.strong += dt
      self.last_strong = now
      if self.strong >= 0.3:
        self.armed = True
    else:
      self.strong = 0.0
    if self.armed and low:
      if self.low_since is None and self.last_strong is not None and now - self.last_strong <= 0.25:
        self.low_since = now
    else:
      self.low_since = None
    rapid = self.low_since is not None and now - self.low_since >= 0.1
    if rapid or (self.last_strong is not None and now - self.last_strong > 0.4):
      self.armed = False
      self.low_since = None

    # Mode 3 selects this branch before the offer branch. Gains never add.
    if mode in (2, 3) and rapid and aligned:
      self.state = "rapid"
      self.cap = max(self.cap, baseline)

    if self.state in ("rapid", "handoff"):
      if pressed or raw > 0.6 or not aligned or error_rate > 2.0:
        self.state = "blocked"
      elif baseline >= max(self.cap, maximum if self.state == "rapid" else minimum):
        self.state = "waiting"
      elif self.state == "rapid":
        # Same 0.5 s ramp as the fastest legacy recovery, with an earlier start.
        self.cap = min(maximum, self.cap + (maximum - minimum) * dt / 0.5)

    if self.state == "offering":
      reversed_force = driver * self.offer_sign < 0 and raw > 0.7
      rejected = (reversed_force or raw > 3.2 or raw > self.offer_effort + 0.4 or
                  self.effort > self.offer_effort + 0.15 or effort_rate > 0.6 or
                  not aligned or error_rate > 2.0)
      elapsed = now - self.offer_since
      no_response = elapsed >= 0.6 and self.offer_effort - self.effort < 0.1
      if rejected:
        self.state = "blocked"
      elif self.release >= 0.1:
        # Mode 1 retains only the offered ceiling while legacy recovery catches
        # up. Mode 3 may already have selected rapid recovery above.
        self.state = "handoff"
      elif no_response or elapsed >= 1.5:
        self.state = "blocked"

    if self.state == "blocked" and self.release >= 0.2:
      self.state = "waiting"

    eligible = (mode in (1, 3) and self.state == "waiting" and span >= 0.18 and
                pressed and self.direction_time >= 0.2 and 0.6 < self.effort < 3.0 and raw <= min(3.2, self.effort + 0.4) and
                baseline <= 80 and aligned and effort_rate <= 0.15 and
                (self.error <= min(1.0, error_limit) and error_rate <= 0.25 or error_rate < -0.5))
    self.qualify = self.qualify + dt if eligible else 0.0
    if self.qualify >= 0.25:
      self.state = "offering"
      self.offer_since = now
      self.offer_effort = self.effort
      self.offer_sign = driver
      self.cap = max(self.cap, baseline)
      self.qualify = 0.0

    if self.state == "offering":
      # CAN ceiling, NOT calibrated physical torque or a guaranteed tactile cue.
      strength = max(raw, self.effort)
      offer_cap = 80.0 if strength <= 1.3 else (80.0 - (strength - 1.3) * 50.0 if strength <= 2.0
                                                else max(minimum, 45.0 - (strength - 2.0) * 20.0))
      self.cap = min(offer_cap, self.cap + 110.0 * dt)
    elif self.state not in ("rapid", "handoff"):
      self.cap = max(minimum, self.cap - 2000.0 * dt)

    return max(baseline, min(maximum, self.cap))
