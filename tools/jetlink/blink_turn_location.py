"""Optional turn passage evidence from raw vehicle GPS, never route-snapped pose."""
import math


def coordinates(lat, lon):
  return all(math.isfinite(v) for v in (lat, lon)) and abs(lat) <= 85 and abs(lon) <= 180


def offset(origin, point):
  lat, lon = origin
  return ((point[1] - lon) * 111320 * math.cos(math.radians(lat)), (point[0] - lat) * 110540)


def observations(sm, now):
  """Optional inputs: old senders/missing GPS retain physical-completion fallback."""
  position, target = None, None
  for name in ('gpsLocation', 'gpsLocationExternal'):
    if name not in sm.services or not sm.valid[name] or not sm.alive[name]:
      continue
    g = sm[name]
    stamp = sm.recv_time[name]
    if (0 <= now-stamp < 1.5 and g.hasFix and coordinates(g.latitude, g.longitude) and
        math.isfinite(g.horizontalAccuracy) and 0 < g.horizontalAccuracy <= 8):
      if position is None or stamp > position[2]:
        position = (g.latitude, g.longitude, stamp, g.horizontalAccuracy)
  name = 'carrotNavi'
  if name in sm.services and sm.valid[name] and sm.alive[name] and 0 <= now-sm.recv_time[name] < 1.5:
    n = sm[name]
    g = n.guidanceCurrent
    nav = sm['carrotMan']
    if (n.connected and n.sessionId and n.navigationStatus.guidanceActive and not n.navigationStatus.offRoute and
        g.meta.present and 0 <= now-g.meta.receivedMonoTimeNanos/1e9 < 2 and g.pointValid and
        coordinates(g.latitude, g.longitude) and nav.xTurnInfo in (1, 2) and
        0 < g.distanceM <= 150 and math.isfinite(nav.xDistToTurn) and abs(g.distanceM-nav.xDistToTurn) <= 15):
      target = (g.latitude, g.longitude, n.sessionId)
  return position, target


class TurnLocation:
  def __init__(self, position=None, target=None, heading=None, distance=None):
    self.target = None
    self.closest = math.inf
    self.previous = None
    self.straight_since = None
    self.passed = False
    self.cancelled = False
    if position is None or target is None or heading is None or distance is None:
      return
    if not all(math.isfinite(v) for v in (heading, distance)) or not 0 < distance <= 150:
      return
    east, north = offset(target[:2], position[:2])
    radius = math.hypot(east, north)
    # Do not interpret a curved approach or an unrelated instruction as a straight
    # crossing. The point must be ahead and consistent with the TBT distance.
    bearing = math.atan2(-east, -north)
    error = math.atan2(math.sin(bearing-heading), math.cos(bearing-heading))
    if radius < 8 or abs(radius-distance) > 25 or abs(error) > math.radians(25):
      return
    self.target = target
    self.incoming = heading

  def update(self, now, position, *, heading, speed, angle, turned):
    if self.target is None or self.cancelled:
      return self.passed, self.cancelled
    if position is None or heading is None or not all(math.isfinite(v) for v in (heading, speed, angle)):
      self.previous, self.straight_since, self.closest, self.passed = None, None, math.inf, False
      return False, False
    east, north = offset(self.target[:2], position[:2])
    radius = math.hypot(east, north)
    stamp = position[2]
    if self.previous is not None:
      old_stamp, old_east, old_north = self.previous
      if stamp == old_stamp:
        return self.passed, False  # Repeated receipt is not new motion evidence.
      dt = stamp-old_stamp
      if not 0 < dt <= 2 or math.hypot(east-old_east, north-old_north) > max(10, max(0, speed)*dt + 2*position[3]):
        self.previous, self.straight_since, self.closest, self.passed = None, None, math.inf, False
        return False, False
    self.previous = (stamp, east, north)
    self.closest = min(self.closest, radius)
    along = east*math.sin(self.incoming) + north*math.cos(self.incoming)
    cross = east*math.cos(self.incoming) - north*math.sin(self.incoming)
    self.passed = self.closest <= 15 and radius >= self.closest + 15
    rotation = abs(math.atan2(math.sin(heading-self.incoming), math.cos(heading-self.incoming)))
    straight = (not turned and self.closest <= 15 and along >= 20 + 2*position[3] and
                abs(cross) <= 10 and rotation < math.radians(15) and abs(angle) <= 10 and speed >= 2)
    self.straight_since = (stamp if self.straight_since is None else self.straight_since) if straight else None
    self.cancelled = self.straight_since is not None and stamp-self.straight_since >= 1
    return self.passed, self.cancelled


def navigation_cancelled(sm, now):
  name = 'carrotNavi'
  if name not in sm.services or not sm.valid[name] or not sm.alive[name] or not 0 <= now-sm.recv_time[name] < 1.5:
    return False
  n = sm[name]
  status = n.navigationStatus
  return (n.connected and status.meta.present and 0 <= now-status.meta.receivedMonoTimeNanos/1e9 < 2 and
          (status.offRoute or not status.guidanceActive))
