import math

import pytest

from openpilot.selfdrive.carrot.gps_sources import GPSSources, PRIORITY


def test_priority_and_handover_never_renew_receipt():
  gps = GPSSources()
  for i, name in enumerate(PRIORITY):
    gps.put(name, 37 + i / 10, 127, 90, 10, 10 + i / 10)
  for expected in PRIORITY:
    source, fix = gps.select(11)
    assert source == expected
    assert fix.stamp == 10 + PRIORITY.index(expected) / 10
    gps.remove(expected)
  assert gps.select(11) == ('none', None)


def test_disconnected_phone_expires_and_does_not_hide_vehicle():
  gps = GPSSources()
  gps.put('phone', 37, 127, 0, 0, 10)
  gps.put('vehicle', 38, 128, None, 0, 12)
  assert gps.select(12.9)[0] == 'phone'
  assert gps.select(13)[0] == 'vehicle'
  assert gps.select(14.5)[0] == 'none'


@pytest.mark.parametrize('lat,lon', [(0, 0), (math.nan, 127), (37, math.inf), (90, 127), (37, 181)])
def test_invalid_coordinates_invalidate_old_fix(lat, lon):
  gps = GPSSources()
  gps.put('external_navi', 37, 127, 90, 10, 10)
  gps.put('external_navi', lat, lon, 90, 10, 11)
  assert gps.select(11)[0] == 'none'


def test_future_fix_and_unknown_heading():
  gps = GPSSources()
  gps.put('vehicle', 37, 127, None, 0, 10)
  assert gps.select(9)[0] == 'none'
  assert gps.select(10)[1].heading is None
  gps.put('phone', 37, 127, -1, 0, 10)
  assert gps.select(10)[1].heading is None
