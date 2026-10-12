"""Bounded route ownership shared by the external navigation bridges.

An external TBT session owns navigation even when it has no geometry. Mixing
its instruction with a saved Mapbox route would give the model contradictory
inputs. Mapbox geometry is retained separately for resumption after disconnect.
"""
import math

MAX_ROUTE_POINTS = 4096


def coordinates(points):
  result = []
  for point in points:
    lat = float(point['latitude'] if isinstance(point, dict) else point.latitude)
    lon = float(point['longitude'] if isinstance(point, dict) else point.longitude)
    if not math.isfinite(lat) or not math.isfinite(lon) or abs(lat) > 85.05112878 or abs(lon) > 180:
      return ()
    result.append((lat, lon))
    if len(result) > MAX_ROUTE_POINTS:
      return ()
  return tuple(result)


class RouteSource:
  def __init__(self):
    self.mapbox = ()
    self.legacy = ()
    self.selected = None
    self.external_was_active = False

  def select(self, external_active, v2=None):
    if v2 is not None:
      route = v2.route.polyline if v2.route.present and not v2.off_route else ()
      choice = ('external', tuple(route))
    elif external_active:
      choice = ('external', self.legacy)
    else:
      # A disconnected sender must provide its route again on reconnect.
      if self.external_was_active:
        self.legacy = ()
      choice = ('mapbox', self.mapbox)
    self.external_was_active = external_active or v2 is not None
    if choice == self.selected:
      return None
    self.selected = choice
    return [{'latitude': lat, 'longitude': lon} for lat, lon in choice[1]]
