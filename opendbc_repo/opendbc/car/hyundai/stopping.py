"""Hyundai CAN FD: retain normal stop acceleration with bounded re-entry.

Thresholds below are experimental, not OEM acceptance conditions. This controller
cannot guarantee stopping; ECU response must be measured on the vehicle.
"""
from dataclasses import dataclass
from enum import StrEnum


DT = 0.02  # SCC_CONTROL is transmitted at 50 Hz
ENTRY_SPEED = 0.7  # m/s; retain approach braking above the low-speed stop region
STOP_SPEED = 0.05
MOVING_SPEED = 0.10
STOP_CONFIRM_TIME = 0.2
NO_PROGRESS_TIME = 0.6
PROGRESS_SPEED = 0.03
REQUEST_TIME_LIMIT = 3.0
REQUEST_DISTANCE_LIMIT = 0.5  # m; applies only when speed reduction also stalls
DISTANCE_NO_PROGRESS_TIME = 0.3
DECEL_ACTIVE = -0.02  # m/s^2; noise band around zero, not an OEM threshold
DECEL_LOST_TIME = 0.3
REACCEL_THRESHOLD = 0.03  # m/s^2, accompanied by a measured speed rebound
REACCEL_CONFIRM_TIME = 0.2
RELEASE_TIME_LIMIT = 1.0
RECOVERY_ACCEL = -0.5
STOP_LOWER_BAND = 0.20  # fixed experimental value; never copied from the stock SCC


class StopPhase(StrEnum):
  idle = "idle"
  approach = "approach"
  request = "request"
  release = "release"
  retry = "retry"
  fallback = "fallback"
  held = "held"


@dataclass(frozen=True)
class StopCommand:
  stop_req: int
  raw: float
  value: float
  lower: float


class CanfdStopping:
  def __init__(self):
    self.reset()

  def reset(self):
    self.phase = StopPhase.idle
    self.reason = "inactive"
    self.retried = False
    self.elapsed = 0.0
    self.no_progress = 0.0
    self.distance = 0.0
    self.reference_speed = 0.0
    self.minimum_speed = 0.0
    self.decel_lost_time = 0.0
    self.reaccel_time = 0.0
    self.stopped_time = 0.0
    self.rolling_time = 0.0
    self.last_value = 0.0

  def enter(self, phase: StopPhase, speed: float, reason: str):
    self.phase = phase
    self.reason = reason
    self.elapsed = self.no_progress = self.distance = 0.0
    self.reference_speed = speed
    self.minimum_speed = speed
    self.decel_lost_time = self.reaccel_time = 0.0

  def update(self, *, active: bool, requested: bool, speed: float, a_ego: float, held: bool,
             accel: float, value: float, previous_value: float, jerk_u: float, jerk_l: float) -> StopCommand | None:
    # Caller validates sensor values and applies pedal/CAN/hold interlocks.
    if not active or not requested:
      self.reset()
      return None

    self.last_value = previous_value
    if self.phase == StopPhase.idle:
      self.enter(StopPhase.approach if speed > ENTRY_SPEED else StopPhase.request, speed, "stop_requested")

    self.elapsed += DT
    self.distance += speed * DT
    self.stopped_time = self.stopped_time + DT if speed <= STOP_SPEED else 0.0
    self.rolling_time = self.rolling_time + DT if speed > MOVING_SPEED else 0.0
    self.no_progress += DT
    if speed <= self.reference_speed - PROGRESS_SPEED:
      self.reference_speed = speed
      self.no_progress = 0.0
    self.minimum_speed = min(self.minimum_speed, speed)
    # aEgo rising from negative toward zero is also normal stop settling. Require
    # sustained loss of deceleration for the creep watchdogs, or positive aEgo
    # together with an actual rebound in speed before treating it as reacceleration.
    self.decel_lost_time = self.decel_lost_time + DT if a_ego >= DECEL_ACTIVE else 0.0
    reaccelerating = (a_ego >= REACCEL_THRESHOLD and speed > STOP_SPEED
                      and speed >= self.minimum_speed + PROGRESS_SPEED)
    self.reaccel_time = self.reaccel_time + DT if reaccelerating else 0.0
    decel_lost = self.decel_lost_time >= DECEL_LOST_TIME

    # Use measured motion as well as the ESC indication. A held indication alone
    # must not conceal rolling, and a missing indication alone must not release hold.
    stopped = (held and speed <= MOVING_SPEED) or self.stopped_time >= STOP_CONFIRM_TIME
    if stopped:
      if self.phase != StopPhase.held:
        self.enter(StopPhase.held, speed, "stop_observed")
    elif self.phase in (StopPhase.request, StopPhase.retry, StopPhase.held) and self.reaccel_time >= REACCEL_CONFIRM_TIME:
      self._recover(speed, "speed_rebound")
    elif self.phase == StopPhase.held:
      if self.rolling_time >= STOP_CONFIRM_TIME and decel_lost:
        self._recover(speed, "motion_after_hold")
    elif self.phase == StopPhase.approach:
      if speed <= ENTRY_SPEED:
        self.enter(StopPhase.request, speed, "entry_speed")
    elif self.phase in (StopPhase.request, StopPhase.retry):
      if speed > ENTRY_SPEED:
        self._recover(speed, "speed_above_entry")
      elif decel_lost and speed > MOVING_SPEED and self.no_progress >= NO_PROGRESS_TIME:
        self._recover(speed, "speed_not_reducing")
      elif decel_lost and speed > STOP_SPEED and self.elapsed >= REQUEST_TIME_LIMIT and self.no_progress >= DISTANCE_NO_PROGRESS_TIME:
        self._recover(speed, "request_timeout")
      elif decel_lost and speed > MOVING_SPEED and self.distance >= REQUEST_DISTANCE_LIMIT and self.no_progress >= DISTANCE_NO_PROGRESS_TIME:
        self._recover(speed, "creep_distance")
    elif self.phase == StopPhase.release and self.elapsed >= RELEASE_TIME_LIMIT:
      self.enter(StopPhase.retry, speed, "reassert_once")

    if self.phase in (StopPhase.request, StopPhase.retry, StopPhase.held):
      # LongControl owns the stopping target and the normal packet builder owns
      # aReqValue limiting. Retry must not replace either with a fixed target.
      self.last_value = min(value, 0.0)
      return StopCommand(1, min(accel, 0.0), self.last_value, STOP_LOWER_BAND)

    # StopReq is released while requesting ordinary deceleration. Retain a
    # stronger existing braking request; never send a positive recovery request.
    return self._decelerate(accel, jerk_u, jerk_l)

  def _decelerate(self, accel: float, jerk_u: float, jerk_l: float) -> StopCommand:
    raw = min(accel, RECOVERY_ACCEL)
    self.last_value = min(0.0, max(self.last_value - jerk_l * DT, min(raw, self.last_value + jerk_u * DT)))
    return StopCommand(0, raw, self.last_value, 0.0)

  def _recover(self, speed: float, reason: str):
    # One timed re-entry per stopping episode. If that fails, retain ordinary
    # deceleration until stopping is observed instead of periodically releasing hold.
    self.enter(StopPhase.fallback if self.retried else StopPhase.release, speed, reason)
    self.retried = True
