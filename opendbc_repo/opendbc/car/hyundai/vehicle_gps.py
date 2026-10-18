"""DBC-selected receive-only position profile, not an OEM GNSS fix flag."""
import math

MESSAGE = 'VEHICLE_POSITION_374'
TIMEOUT = 2.5


def supports_vehicle_gps(cp):
  message = cp.dbc.name_to_msg.get(MESSAGE)
  if message is None or message.address != 0x374 or message.size != 8:
    return False
  for name, bit in (('LATITUDE', 0), ('LONGITUDE', 32)):
    signal = message.sigs.get(name)
    if (signal is None or signal.start_bit != bit or signal.size != 32 or
        not signal.is_little_endian or signal.is_signed or signal.factor != 1e-6 or signal.offset != 0):
      return False
  return True


def register_vehicle_gps(cp):
  if supports_vehicle_gps(cp):
    cp._add_message(MESSAGE, math.nan)
  return cp


def displacement(a, b):
  dy = math.radians(b[0] - a[0]) * 6371000
  dx = math.radians(b[1] - a[1]) * 6371000 * math.cos(math.radians((a[0] + b[0]) / 2))
  return math.hypot(dx, dy), math.degrees(math.atan2(dx, dy)) % 360


class VehicleGPS:
  def __init__(self):
    self.stamp = self.tick = self.heading_stamp = 0
    self.point = None
    self.runs = 0
    self.travel = self.frozen_travel = 0.
    self.heading = 0.
    self.accepted = False
    self.frozen = False

  def update(self, cp, speed, reverse=False):
    if not supports_vehicle_gps(cp):
      return {}
    now = cp._last_update_nanos
    dt = (now - self.tick) / 1e9 if self.tick else 0.
    self.tick = now
    speed_ok = math.isfinite(speed) and 0 <= speed <= 90
    if not 0 <= dt <= TIMEOUT or not speed_ok:
      self.runs = 0
      self.accepted = False
    if speed_ok and 0 <= dt <= TIMEOUT:
      self.travel += speed * dt
      self.frozen_travel += speed * dt
    stamp = cp.ts_nanos.get(MESSAGE, {}).get('LATITUDE', 0)
    fresh = stamp > 0 and 0 <= (now - stamp) / 1e9 <= TIMEOUT and not cp.bus_timeout
    if stamp != self.stamp:
      elapsed = (stamp - self.stamp) / 1e9
      self.stamp = stamp
      dat = cp.dat.get(0x374, b'')
      # Positive northern/eastern coordinates are the only encoding validated
      # in recordings. Do not guess signed or offset encodings elsewhere.
      point = (int.from_bytes(dat[:4], 'little') / 1e6, int.from_bytes(dat[4:8], 'little') / 1e6)
      layout = len(dat) == 8 and 0 < point[0] <= 85.05112878 and 0 < point[1] <= 180
      distance, course = displacement(self.point, point) if layout and self.point else (0., 0.)
      consistent = (self.point is not None and .5 <= elapsed <= TIMEOUT and
                    distance <= min(90 * elapsed, 15 + 2 * self.travel))
      if layout and fresh and speed_ok:
        self.runs = min(3, self.runs + 1) if consistent else 1
        if self.point != point:
          self.frozen_travel = 0.
        # Only a newly repeated coordinate proves a freeze. Normal travel
        # between two 1 Hz samples must not invalidate highway-speed fixes.
        self.frozen = self.frozen_travel > 15
        if consistent and self.runs >= 3 and distance >= 5 and speed >= 2 and not reverse:
          self.heading, self.heading_stamp = course, stamp
        elif not consistent:
          self.heading_stamp = 0
        self.point = point
        self.accepted = self.runs >= 3
      else:
        self.point = None
        self.runs = 0
        self.heading_stamp = 0
        self.accepted = False
      self.travel = 0.
    valid = fresh and self.accepted and speed_ok and not self.frozen
    if not fresh:
      self.runs = 0
      self.accepted = False
      self.heading_stamp = 0
    heading_valid = (valid and not reverse and self.heading_stamp > 0 and
                     0 <= (now - self.heading_stamp) / 1e9 <= (30 if speed < .5 else TIMEOUT))
    return dict(available=self.stamp > 0, valid=valid, latitude=self.point[0] if self.point else 0.,
                longitude=self.point[1] if self.point else 0., sampleMonoTime=self.stamp,
                headingDeg=self.heading, headingValid=heading_valid)
