"""Optional receive-only APP_trafficControl; never decode legacy DAS_road as APP."""
import time

from opendbc.can import CANParser
from opendbc.car.tesla.values import CANBUS

MAX_AGE_NS = 500_000_000


def boot_time_ns():
  return time.clock_gettime_ns(getattr(time, "CLOCK_BOOTTIME", time.CLOCK_MONOTONIC))


class TeslaTrafficControl:
  def __init__(self):
    self.parser = CANParser("tesla_modely_hw4_perception", [("APP_trafficControl", float("nan"))], CANBUS.autopilot_party)
    self.sample = 0
    self.available = False
    self.valid = False
    self.distance = 0.0
    self.light = 0
    self.status = "waiting_for_app_traffic_control"
    self.payload = None

  def observe(self, data, source, sample):
    if source != CANBUS.autopilot_party:
      return
    now = boot_time_ns()
    if not 0 < sample <= now or now - sample > MAX_AGE_NS or sample < self.sample:
      return
    if sample == self.sample:
      if data != self.payload:
        self.valid = False
        self.status = "conflicting_timestamp"
      return
    self.available, self.sample, self.payload = True, sample, bytes(data)
    self.valid, self.light = False, 0
    if len(data) != 8:
      self.status = "unsupported_dlc"
      return

    self.parser.update([(sample, [(0x25D, data, source)])])
    fields = self.parser.vl["APP_trafficControl"]
    self.distance = float(fields["APP_tcControlDistance"])
    self.light = int(fields["APP_tcControlLightState"])
    self.valid = (
      fields["APP_tcControlType"] == 3
      and fields["APP_tcFeatureState"] in (2, 3)
      and fields["APP_tcControlSource"] in (2, 3)
      and fields["APP_tcVisionLight"] == 1
      and fields["APP_tcUnavailableReason"] == 0
      and fields["APP_tcWarningSuppressionReason"] == 0
      and self.light in (1, 2, 3)
      and 0 <= self.distance < 255
    )
    self.status = "app_traffic_control" if self.valid else "invalid_app_traffic_control"

  def publish(self, target, now):
    target.available = self.available
    target.valid = self.valid and 0 < self.sample <= now and now - self.sample <= MAX_AGE_NS
    target.sampleMonoTime = self.sample
    target.stopLineDistance = self.distance
    target.light = self.light if self.light in (1, 2, 3) else 0
    target.status = self.status if not self.valid or target.valid else "stale"
