"""Tesla-only supplementary stop-line constraint, independent of model/lead stops."""
from dataclasses import dataclass
import math

from opendbc.car.tesla.values import TeslaFlags

MAX_AGE_NS = 500_000_000
RED_CONFIRM_NS = 200_000_000
GREEN_CONFIRM_NS = 500_000_000
STOP_BUFFER = 2.0
ENTRY_DISTANCE = 126.0


@dataclass(frozen=True)
class TrafficStopDecision:
  active: bool = False
  distance: float = 0.0
  hold: bool = False
  warn: bool = False
  fault: bool = False
  reason: str = "waiting"
  required_decel: float = 0.0


def create_tesla_traffic_stop(cp):
  if cp.brand == "tesla" and cp.openpilotLongitudinalControl and not cp.dashcamOnly and cp.flags & TeslaFlags.CAN_TRAFFIC_STOP:
    return TeslaTrafficStop()
  return None


class TeslaTrafficStop:
  def __init__(self):
    self.active = False
    self.distance = 0.0
    self.last_now = 0
    self.last_sample = 0
    self.supported_sample = 0
    self.pending_color = 0
    self.pending_since = 0
    self.pending_count = 0
    self.pending_distance = 0.0
    self.pending_now = 0
    self.pending_sample = 0
    self.fault = ""
    self.overridden = False

  def clear(self, now):
    self.active = False
    self.pending_color = self.pending_since = self.pending_count = 0
    self.supported_sample = 0
    self.fault = ""
    self.last_sample = max(self.last_sample, now)

  def update(self, cs, now, *, engaged, control_active):
    dt = (now - self.last_now) / 1e9 if self.last_now else 0.0
    self.last_now = now
    if not engaged:
      self.overridden = False
    if cs.gasPressed or cs.brakePressed:
      self.overridden = True
    speed = float(cs.vEgo)
    if (not control_active or not engaged or self.overridden or not math.isfinite(speed) or speed < 0
        or cs.gearShifter != "drive" or not cs.canValid or cs.vehicleSensorsInvalid
        or cs.doorOpen or cs.seatbeltUnlatched or cs.parkingBrake or cs.accFaulted or cs.espDisabled):
      self.clear(now)
      return TrafficStopDecision(reason="driver_override" if self.overridden else "inactive")

    if self.active:
      if not 0 < dt <= 0.2:
        self.fault = "clock_discontinuity"
      self.distance -= speed * max(0.0, min(dt, 0.2))
      if self.distance < -5.0:
        self.active = False
        self.fault = "passed_stop_line"

    observation = cs.teslaTrafficControl
    sample = int(observation.sampleMonoTime)
    valid = (observation.valid and 0 < sample <= now and now - sample <= MAX_AGE_NS
             and math.isfinite(observation.stopLineDistance) and 0 <= observation.stopLineDistance < 255
             and observation.light in ("red", "green", "yellow"))
    if sample < self.last_sample:
      valid = False
    if sample > self.last_sample:
      self.last_sample = sample
      if valid and not self.fault:
        distance = float(observation.stopLineDistance) - speed * ((now - sample) / 1e9)
        color = int(observation.light.raw)
        tolerance = max(3.0, speed * 0.15)
        if self.active and abs(distance - self.distance) > tolerance:
          self.fault = "stop_line_jump"
        elif color in (1, 2):
          self.supported_sample = sample
          consistent = (self.pending_color == color and self.pending_since > 0
                        and 0 < sample - self.pending_sample <= MAX_AGE_NS
                        and abs(distance - (self.pending_distance - speed * (now - self.pending_now) / 1e9)) <= tolerance)
          if not consistent:
            self.pending_since, self.pending_count = sample, 0
          self.pending_color = color
          self.pending_count += 1
          self.pending_distance = distance
          self.pending_now, self.pending_sample = now, sample
          elapsed = sample - self.pending_since
          if self.active:
            # Never move an owned stop line away from the car on a range spike.
            self.distance = min(self.distance, distance)
            if color == 2 and self.pending_count >= 3 and elapsed >= GREEN_CONFIRM_NS:
              self.clear(now)
              return TrafficStopDecision(reason="green_released")
          elif color == 1 and 0 < distance < ENTRY_DISTANCE and self.pending_count >= 3 and elapsed >= RED_CONFIRM_NS:
            self.active, self.distance = True, distance
        else:
          self.pending_color = self.pending_since = self.pending_count = 0
      else:
        self.pending_color = self.pending_since = self.pending_count = 0
    elif not valid:
      self.pending_color = self.pending_since = self.pending_count = 0

    if self.active and now - self.supported_sample > MAX_AGE_NS:
      self.fault = self.fault or "traffic_signal_lost"
    if not self.active:
      if not valid:
        self.pending_color = self.pending_since = self.pending_count = 0
      return TrafficStopDecision(warn=bool(self.fault), fault=bool(self.fault),
                                 reason=self.fault or observation.status or "confirming")

    available = self.distance - STOP_BUFFER - speed * 0.3
    required = speed * speed / (2 * max(available, 0.1))
    hold = speed < 0.3 and self.distance <= STOP_BUFFER + 1.0
    # This is a feasibility warning, not permission to increase actuator limits.
    late = required > 3.48 and not hold
    reason = self.fault or ("insufficient_distance" if late else "holding" if hold else "stopping")
    return TrafficStopDecision(True, max(0.0, self.distance), hold, bool(self.fault) or late, bool(self.fault), reason, required)
