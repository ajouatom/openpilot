"""Experimental angle-control handover; force trends do not establish consent.

While a handover owns the output, it selects the total CAN authority ceiling.
Angle commands and their limits stay unchanged. Legacy recovery history continues
independently for bounded handback.
"""
from collections import deque
from math import atan, degrees, isfinite
from statistics import median


def approach(value, target, step):
  return value + max(-step, min(step, target - value))


class SteeringHandover:
  def __init__(self):
    self.reset()

  def reset(self, mode=0, now=None):
    self.mode = mode
    self.last_time = now
    self.raw_history = deque(maxlen=3)
    self.history = deque()
    self.force_history = deque()
    self.effort = None
    self.error = 0.0
    self.state = "waiting"
    self.cap = 0.0
    self.qualify = 0.0
    self.release = 0.0
    self.strong = 0.0
    self.armed_at = None
    self.release_qualify = 0.0
    self.offer_since = 0.0
    self.offer_effort = 0.0
    self.offer_sign = 0.0
    self.capture_effort = 0.0
    self.capture_since = 0.0
    self.previous_driver = 0.0
    self.direction_time = 0.0

  def update(self, *, mode, now, baseline, minimum, maximum, driver, threshold,
             pressed, target_error, command_error, speed, wheelbase, steer_ratio,
             active, valid):
    mode = mode if mode in (1, 2, 3) else 0
    if mode == 0 or mode != self.mode or not active:
      self.reset(mode, now)
      return baseline

    dt = now - self.last_time if self.last_time is not None else 0.0
    self.last_time = now
    values = (now, dt, baseline, minimum, maximum, driver, threshold, target_error,
              command_error, speed, wheelbase, steer_ratio)
    if (not valid or not all(isfinite(v) for v in values) or
        not 0.0 < dt <= 0.03 or threshold <= 0 or wheelbase <= 0 or steer_ratio <= 0 or
        not 0 <= minimum < maximum or not 0 <= baseline <= maximum):
      owned, cap = self.state != "waiting", self.cap
      self.reset(mode, now)
      if owned:
        # Invalidity cannot raise a reduced experimental ceiling to a higher
        # legacy ceiling. Discard evidence; normal angle limits still run.
        self.state = "blocked"
        self.cap = min(baseline, cap)
        return self.cap
      return baseline

    raw = abs(driver) / threshold
    self.direction_time = self.direction_time + dt if driver * self.previous_driver > 0 else 0.0
    self.previous_driver = driver
    self.raw_history.append(raw)
    magnitude = median(self.raw_history)
    # Magnitude filtering avoids cancellation of alternating driver forces.
    self.effort = magnitude if self.effort is None else self.effort + dt / (0.12 + dt) * (magnitude - self.effort)
    error = max(abs(target_error), abs(command_error))
    self.error += dt / (0.08 + dt) * (error - self.error)
    self.history.append((now, self.effort, self.error))
    while len(self.history) > 1 and now - self.history[0][0] > 0.22:
      self.history.popleft()
    span = now - self.history[0][0]
    effort_change = self.effort - self.history[0][1]
    error_change = self.error - self.history[0][2]
    # Full offer tolerance followed by gradual rolloff, not an error-rate veto.
    # Tighten at speed using a bicycle-model acceleration-error equivalent.
    margin = min(3.0, degrees(atan(0.8 * wheelbase / max(speed * speed, 25.0))) * steer_ratio)
    error_quality = max(0.0, min(1.0, (2.0 * margin - max(error, self.error)) / margin))
    converging = max(error, self.error) <= margin or error_change < -0.05
    yielding = effort_change <= 0.05
    low = not pressed and raw <= 0.6 and magnitude <= 0.6
    self.release = self.release + dt if low else 0.0

    # Early, limited capture uses force decline only. Steering error sets the
    # subsequent recovery rate, never permission to start recovering.
    self.strong = self.strong + dt if pressed and raw > 1.0 and magnitude > 1.0 and self.direction_time > 0 else 0.0
    if self.strong >= 0.15:
      self.armed_at = now
    self.force_history.append((now, raw))
    while len(self.force_history) > 1 and now - self.force_history[0][0] > 0.4:
      self.force_history.popleft()
    force_span = now - self.force_history[0][0]
    fall = (self.force_history[0][1] - magnitude) / max(force_span, dt)
    peak = max(value for _, value in self.force_history)
    release_candidate = (self.armed_at is not None and now - self.armed_at <= 0.5 and
                         not pressed and raw <= 1.0 and magnitude <= 1.0 and
                         peak - magnitude >= 0.25 and peak >= 1.05 and fall >= 0.5)
    self.release_qualify = self.release_qualify + dt if release_candidate else 0.0
    if (mode in (2, 3) and self.release_qualify >= 0.03 and baseline <= 80 and
        self.state not in ("capture", "recover", "returning")):
      self.cap = baseline if self.state == "waiting" else self.cap
      self.state = "capture"
      self.capture_since = now
      self.capture_effort = raw
      self.armed_at = None
      self.release_qualify = 0.0

    # Mode 3 selects capture before the offer; never add both increments.
    if self.state in ("capture", "recover"):
      renewed = pressed or raw > (max(0.8, self.capture_effort + 0.15) if self.state == "capture" else 0.8)
      if renewed:
        self.state = "blocked"
      elif self.state == "capture":
        if self.release >= 0.06:
          self.state = "recover"
        elif now - self.capture_since >= 0.3:
          self.state = "withdrawing"
          self.offer_effort = self.effort
          self.offer_sign = driver

    if self.state in ("offering", "withdrawing"):
      reversed_force = driver * self.offer_sign < 0 and raw > 0.7
      renewed = (reversed_force or raw > 3.2 or raw > self.offer_effort + 0.4 or
                 self.effort > self.offer_effort + 0.15 or effort_change > 0.2)
      elapsed = now - self.offer_since
      if renewed:
        self.state = "blocked"
      elif self.state == "offering" and self.release >= 0.1:
        # Mode 1 hands back to legacy recovery with a bounded ceiling transition.
        self.state = "returning"
      elif self.state == "offering" and ((elapsed >= 0.6 and self.offer_effort - self.effort < 0.1) or elapsed >= 1.5):
        self.state = "withdrawing"

    eligible = (mode in (1, 3) and self.state == "waiting" and span >= 0.18 and
                pressed and self.direction_time >= 0.2 and 0.6 < self.effort < 3.0 and
                raw <= min(3.2, self.effort + 0.4) and baseline <= 80 and
                error_quality > 0 and converging and yielding)
    self.qualify = self.qualify + dt if eligible else 0.0
    if self.qualify >= 0.25:
      self.state = "offering"
      self.offer_since = now
      self.offer_effort = self.effort
      self.offer_sign = driver
      self.cap = baseline
      self.qualify = 0.0

    if self.state == "offering":
      strength = max(raw, self.effort)
      force_cap = 80.0 if strength <= 1.3 else (80.0 - (strength - 1.3) * 50.0 if strength <= 2.0
                                                else max(minimum, 45.0 - (strength - 2.0) * 20.0))
      desired = minimum + (min(maximum, force_cap) - minimum) * error_quality
      if desired <= self.cap or (converging and yielding):
        self.cap = approach(self.cap, desired, 110.0 * dt)
    elif self.state == "capture":
      self.cap = approach(self.cap, min(maximum, 45.0), 110.0 * dt)
    elif self.state == "recover":
      # Preserve the model/limited angle command. Small steering error permits
      # the fastest ramp; large error slows it without blocking recovery. Use
      # both raw and filtered error so growth slows promptly and noise cannot
      # immediately restore the fastest ramp.
      quality = max(0.25, min(1.0, margin / max(margin, error, self.error)))
      self.cap = approach(self.cap, maximum, (maximum - minimum) / 0.5 * quality * dt)
      if self.cap == maximum and baseline == maximum:
        self.state = "waiting"
    elif self.state == "withdrawing":
      # No response or uncertain alignment is not an abrupt driver veto.
      self.cap = approach(self.cap, minimum, 110.0 * dt)
      if self.cap == minimum:
        self.state = "blocked"
    elif self.state == "blocked":
      self.cap = approach(self.cap, minimum, 2000.0 * dt)
      if self.release >= 0.2 and self.cap == minimum:
        self.state = "returning"
    elif self.state == "returning":
      if pressed or raw > 0.8:
        self.state = "blocked"
        self.cap = approach(self.cap, minimum, 2000.0 * dt)
      else:
        # Hold an accepted offer until legacy catches up, then join its ramp.
        # Do not shed assistance solely because legacy is still release-latched.
        self.cap = approach(self.cap, max(self.cap, baseline), 110.0 * dt)
        if self.cap == baseline:
          self.state = "waiting"

    if self.state == "waiting":
      self.cap = baseline
      return baseline
    # The legacy ceiling must not bypass an active experimental transition.
    self.cap = max(minimum, min(maximum, self.cap))
    return self.cap
