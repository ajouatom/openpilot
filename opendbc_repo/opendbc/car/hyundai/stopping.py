"""Hyundai CAN FD: zero aReq with StopReq, without automatic release/retry.

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
APPROACH_ACCEL = -0.5
STOP_LOWER_BAND = 0.20  # fixed experimental value; never copied from the stock SCC


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
    self.phase = StopPhase.idle
    self.reason = "inactive"
    self.stopped_time = 0.0

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
    stopped = (held and speed <= MOVING_SPEED) or self.stopped_time >= STOP_CONFIRM_TIME
    if stopped:
      if self.phase != StopPhase.held:
        self.enter(StopPhase.held, "stop_observed")
    elif self.phase == StopPhase.held:
      # Motion changes diagnostics only. Never release StopReq to retry, even
      # when speed rises above the initial entry threshold after a request.
      self.enter(StopPhase.request, "motion_after_hold")

    if self.phase in (StopPhase.request, StopPhase.held):
      # Match the observed Ioniq stock entry: both aReq fields become zero in
      # the FIRST StopReq frame, bypassing the ordinary aReqValue jerk ramp.
      return StopCommand(1, 0.0, 0.0, STOP_LOWER_BAND)

    # Preserve the existing approach deceleration until initial low-speed entry.
    raw = min(accel, APPROACH_ACCEL)
    value = min(0.0, max(previous_value - jerk_l * DT, min(raw, previous_value + jerk_u * DT)))
    return StopCommand(0, raw, value, 0.0)
