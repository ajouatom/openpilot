"""Optional Hyundai CAN-FD observations, with explicit length and freshness."""

import math


def fresh_values(parser, name, address, size, now, max_age_ns=150_000_000):
  if parser is None or parser.bus_timeout or len(parser.dat.get(address, b"")) != size:
    return None
  stamp = max(parser.ts_nanos.get(name, {}).values(), default=0)
  if stamp <= 0 or not 0 <= now - stamp <= max_age_ns:
    return None
  return parser.vl.get(name)


def front_bsd(parser, now):
  name = "BLINDSPOTS_FRONT_CORNER_1"
  if parser is None or name not in parser.vl:
    return None  # This optional channel has not been admitted.
  values = fresh_values(parser, name, 0x1E5, 16, now)
  if values is None:
    return True, True  # Loss of an admitted channel is not evidence of clearance.
  return tuple(values.get(field) != 0 for field in ("INDICATOR_LEFT", "INDICATOR_RIGHT"))


class SteeringRateSign:
  def __init__(self):
    self.angle = None
    self.stamp = 0
    self.sign = 0

  def update(self, parser, now):
    values = fresh_values(parser, "STEERING_SENSORS", 0x125, 16, now)
    if values is None:
      self.angle, self.stamp, self.sign = None, 0, 0
      return 0.0
    stamp = max(parser.ts_nanos["STEERING_SENSORS"].values())
    angle = -values["STEERING_ANGLE"]
    if stamp > self.stamp:
      if self.angle is not None and stamp - self.stamp <= 150_000_000:
        delta = angle - self.angle
        if abs(delta) > 1e-3:
          self.sign = math.copysign(1.0, delta)
      else:
        self.sign = 0
      self.angle, self.stamp = angle, stamp
    return values["STEERING_RATE"] * self.sign
