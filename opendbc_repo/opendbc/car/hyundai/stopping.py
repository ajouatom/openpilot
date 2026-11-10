"""Hyundai CAN FD: retain gentle braking until the SCC hold is confirmed.

Thresholds below are experimental, not OEM acceptance conditions. This controller
cannot guarantee stopping; ECU response must be measured on the vehicle.
"""
from dataclasses import dataclass
from enum import StrEnum


DT = 0.02  # SCC_CONTROL is transmitted at 50 Hz
ENTRY_SPEED = 0.55  # m/s; retain approach braking until about 2 km/h
STOP_SPEED = 0.05
MOVING_SPEED = 0.10
STOP_CONFIRM_TIME = 0.2
APPROACH_ACCEL = -0.5
HOLD_PENDING_ACCEL = -0.05
HOLD_SETTLE_ACCEL = -0.25
HOLD_APPLY_JERK = 0.35
HOLD_SETTLE_APPLY_JERK = 1.0
STOP_RELEASE_JERK = 1.5
ROLLBACK_ARM_SPEED = 0.03
ROLLBACK_TRIGGER_SPEED = 0.05
ROLLBACK_CONFIRM_TIME = 0.04
STOP_LOWER_BAND = 0.20  # fixed during active ACC, including before StopReq; never copied from stock


class StopPhase(StrEnum):
  idle = "idle"
  approach = "approach"
  request = "request"
  held = "held"


@dataclass(frozen=True)
class StopCommand:
  stop_req: int
  raw: float
  value: float
  lower: float


class CanfdStopping:
  def __init__(self):
    # Packet history survives episode resets; it follows the final StopReq,
    # including approach and interlock overrides.
    self.last_scc_stop_req = 0
    self.last_scc_jerk_upper = 1.0
    self.reset()

  def limit_scc_jerk_upper(self, stop_req: int, jerk_upper: float) -> float:
    """Separate an Upper increase from the first StopReq frame, not its onset."""
    if stop_req == 1 and self.last_scc_stop_req != 1:
      jerk_upper = min(jerk_upper, self.last_scc_jerk_upper)
    self.last_scc_stop_req = stop_req
    self.last_scc_jerk_upper = jerk_upper
    return jerk_upper

  def reset(self):
    self.stop_object_active = False
    self.phase = StopPhase.idle
    self.reason = "inactive"
    self.stopped_time = 0.0
    self.settle_brake = False
    self.rollback_armed = False
    self.rollback_time = 0.0

  def enter(self, phase: StopPhase, reason: str):
    self.phase = phase
    self.reason = reason

  def update(self, *, active: bool, requested: bool, speed: float, held: bool,
             accel: float, previous_value: float, jerk_u: float, jerk_l: float) -> StopCommand | None:
    # Caller validates sensor values and applies pedal/CAN/hold interlocks.
    if not active or not requested:
      self.reset()
      return None

    if self.phase == StopPhase.idle:
      self.enter(StopPhase.approach if speed > ENTRY_SPEED else StopPhase.request, "stop_requested")
    if self.phase == StopPhase.approach and speed <= ENTRY_SPEED:
      self.enter(StopPhase.request, "entry_speed")

    self.stopped_time = self.stopped_time + DT if speed <= STOP_SPEED else 0.0
    if speed <= ROLLBACK_ARM_SPEED:
      self.rollback_armed = True
    rollback_candidate = self.rollback_armed and not held and speed >= ROLLBACK_TRIGGER_SPEED
    self.rollback_time = self.rollback_time + DT if rollback_candidate else 0.0
    if self.rollback_time >= ROLLBACK_CONFIRM_TIME:
      self.settle_brake = True
    stationary_confirmed = self.stopped_time >= STOP_CONFIRM_TIME
    stopped = (held and speed <= MOVING_SPEED) or stationary_confirmed
    if stopped:
      if self.phase != StopPhase.held:
        self.enter(StopPhase.held, "stop_observed")
    elif self.phase == StopPhase.held:
      # Motion changes diagnostics only. Never release StopReq to retry, even
      # when speed rises above the initial entry threshold after a request.
      self.enter(StopPhase.request, "motion_after_hold")

    if self.phase in (StopPhase.request, StopPhase.held):
      # Enter with a light request, then add holding force only near standstill.
      # Limit both applying and releasing the request so either target change
      # cannot create a one-frame low-speed pitch.
      hold_confirmed = held and speed <= MOVING_SPEED
      raw = (0.0 if hold_confirmed else
             HOLD_SETTLE_ACCEL if self.settle_brake else HOLD_PENDING_ACCEL)
      if raw >= previous_value:
        value = min(raw, previous_value + min(jerk_u, STOP_RELEASE_JERK) * DT)
      else:
        apply_jerk = HOLD_SETTLE_APPLY_JERK if self.settle_brake else HOLD_APPLY_JERK
        value = max(raw, previous_value - min(jerk_l, apply_jerk) * DT)
      return StopCommand(1, raw, value, STOP_LOWER_BAND)

    # Preserve the existing approach deceleration until initial low-speed entry.
    raw = min(accel, APPROACH_ACCEL)
    value = min(0.0, max(previous_value - jerk_l * DT, min(raw, previous_value + jerk_u * DT)))
    return StopCommand(0, raw, value, STOP_LOWER_BAND)
