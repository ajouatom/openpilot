"""Offline rotation-context gates. Thresholds are experimental, not vehicle policy."""
import math


class GreenMotionGate:
  MAX_AGE = .2
  STRAIGHT_TIME = .5
  ENTER_ANGLE = 15.
  EXIT_ANGLE = 8.
  ENTER_RATE = 90.
  EXIT_RATE = 30.
  STOP_SPEED = .3

  def __init__(self):
    self.last_time = None
    self.straight_since = None
    self.allowed = False

  def update(self, timestamp, angle, rate, speed, valid=True):
    values = (timestamp, angle, rate, speed)
    if (not valid or not all(math.isfinite(v) for v in values) or
        (self.last_time is not None and timestamp <= self.last_time)):
      self.last_time = None
      self.straight_since = None
      self.allowed = False
      return
    if self.last_time is None or timestamp-self.last_time > self.MAX_AGE:
      self.allowed = False
      self.straight_since = None
    self.last_time = timestamp
    stopped = abs(speed) <= self.STOP_SPEED
    if not stopped and (abs(angle) >= self.ENTER_ANGLE or abs(rate) >= self.ENTER_RATE):
      self.allowed = False
      self.straight_since = None
    elif not self.allowed:
      # A turned steering wheel at rest is not camera rotation. The recorded
      # green transition includes a stationary car with 39.5-degree steering.
      if stopped or (abs(angle) <= self.EXIT_ANGLE and abs(rate) <= self.EXIT_RATE):
        if self.straight_since is None:
          self.straight_since = timestamp
        self.allowed = timestamp-self.straight_since >= self.STRAIGHT_TIME-1e-9
      else:
        self.straight_since = None

  def allows_green(self, capture_time):
    return (self.allowed and self.last_time is not None and math.isfinite(capture_time) and
            0 <= capture_time-self.last_time <= self.MAX_AGE)


class GreenPoseGate:
  """Use calibrated body yaw rate, with independent arrival/sample freshness.

  Missing/unreliable pose does not fall back to steering-based green permission.
  Callers must include pose/filter/sensor/calibration validity in `valid`.
  """
  MAX_AGE = .2
  STRAIGHT_TIME = .5
  ENTER_YAW = 3.  # degrees per second, provisional offline thresholds
  EXIT_YAW = 1.5
  MAX_STD = 2.

  def __init__(self):
    self.arrival = None
    self.sample = None
    self.straight_since = None
    self.allowed = False

  def update(self, arrival, sample, yaw, yaw_std, valid=True):
    if (not valid or not all(math.isfinite(v) for v in (arrival, sample, yaw, yaw_std)) or
        not 0 <= arrival-sample <= self.MAX_AGE or not 0 <= yaw_std <= self.MAX_STD or
        (self.arrival is not None and arrival <= self.arrival) or
        (self.sample is not None and sample <= self.sample)):
      self.__init__()
      return
    if self.sample is None or sample-self.sample > self.MAX_AGE:
      self.allowed = False
      self.straight_since = None
    self.arrival, self.sample = arrival, sample
    if abs(yaw) >= self.ENTER_YAW:
      self.allowed = False
      self.straight_since = None
    elif not self.allowed:
      if abs(yaw) <= self.EXIT_YAW:
        if self.straight_since is None:
          self.straight_since = sample
        self.allowed = sample-self.straight_since >= self.STRAIGHT_TIME-1e-9
      else:
        self.straight_since = None

  def allows_green(self, capture_time):
    return (self.allowed and self.arrival is not None and self.sample is not None and
            math.isfinite(capture_time) and self.arrival <= capture_time and
            0 <= capture_time-self.sample <= self.MAX_AGE)
