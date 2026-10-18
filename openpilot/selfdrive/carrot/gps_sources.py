"""Independent position fixes; a heartbeat or source switch never renews a fix."""
from dataclasses import dataclass
import math

PRIORITY = ('external_navi', 'phone', 'vehicle', 'device')


@dataclass(frozen=True)
class Fix:
  latitude: float
  longitude: float
  heading: float | None
  speed: float
  stamp: float


class GPSSources:
  def __init__(self):
    self.fixes: dict[str, Fix] = {}

  def remove(self, source):
    self.fixes.pop(source, None)

  def put(self, source, latitude, longitude, heading, speed, stamp):
    if (not all(math.isfinite(v) for v in (latitude, longitude, speed, stamp)) or
        not -85.05112878 <= latitude <= 85.05112878 or not -180 <= longitude <= 180 or
        (latitude == 0 and longitude == 0) or stamp <= 0 or not 0 <= speed <= 90):
      self.remove(source)
      return
    heading = heading if heading is not None and math.isfinite(heading) and 0 <= heading < 360 else None
    self.fixes[source] = Fix(latitude, longitude, heading, speed, stamp)

  def select(self, now):
    for source in PRIORITY:
      fix = self.fixes.get(source)
      if fix is not None and 0 <= now - fix.stamp < (2.5 if source == 'vehicle' else 3.):
        return source, fix
    return 'none', None
