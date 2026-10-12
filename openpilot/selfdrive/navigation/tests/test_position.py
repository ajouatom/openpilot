from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.navigation import helpers


class SM(dict):
  pass


@pytest.fixture
def sm(monkeypatch):
  monkeypatch.setattr(helpers.time, 'monotonic', lambda: 10.)
  sm = SM(carrotMan=NS(positionFixMonoTime=9_000_000_000, positionMonoTime=9_900_000_000,
                      xPosLat=37., xPosLon=127., xPosAngle=45., xPosSpeed=36.),
          gpsLocation=NS(latitude=38., longitude=128., bearingDeg=60., speed=9., hasFix=True),
          gpsLocationExternal=NS(hasFix=False))
  sm.valid = dict.fromkeys(sm, True)
  sm.recv_time = dict.fromkeys(sm, 9.9)
  sm.logMonoTime = dict.fromkeys(sm, 9_900_000_000)
  return sm


def test_fused_external_position_keeps_original_prediction_time(sm):
  position = helpers.navigation_position(sm)
  assert (position.latitude, position.speed, position.time) == (37., 10., 9.9)


@pytest.mark.parametrize('field,value', [('positionFixMonoTime', 8_000_000_000), ('positionMonoTime', 9_000_000_000),
                                       ('positionFixMonoTime', 11_000_000_000), ('xPosLat', float('nan'))])
def test_rebroadcast_cannot_renew_stale_or_invalid_fused_position(sm, field, value):
  setattr(sm['carrotMan'], field, value)
  assert helpers.navigation_position(sm).latitude == 38.
  sm['gpsLocation'].hasFix = False
  assert helpers.navigation_position(sm) is None


def test_stopped_publisher_expires_even_with_recent_fix(sm):
  sm.recv_time = dict.fromkeys(sm, 7.)
  assert helpers.navigation_position(sm) is None
