from types import SimpleNamespace as NS

import pytest

from opendbc.car import structs
from openpilot.selfdrive.carrot import carrot_serv
from openpilot.selfdrive.carrot.tests.test_carrot_navi_serv import _serv


class SM(dict):
  pass


@pytest.fixture
def context(monkeypatch):
  clock = [10.]
  monkeypatch.setattr(carrot_serv.time, 'monotonic', lambda: clock[0])
  sm = SM(carState=structs.CarState(vehicleGps={'valid': True, 'latitude': 37., 'longitude': 127.,
                                                   'sampleMonoTime': 10_000_000_000, 'headingDeg': 90., 'headingValid': True}),
          gpsLocation=NS(hasFix=True, latitude=38., longitude=128., bearingDeg=180., speed=0.))
  sm.updated = dict.fromkeys(sm, True)
  sm.alive = dict.fromkeys(sm, True)
  sm.valid = dict.fromkeys(sm, True)
  sm.recv_time = dict.fromkeys(sm, 10.)
  return _serv(), sm, clock


def test_live_priority_and_source_handover_preserve_original_fix_time(context):
  serv, sm, clock = context
  serv._gps_sources().put('external_navi', 36., 126., 45., 0., 10.)
  # A phone packet arriving while external navigation is active stays available.
  serv.update({'latitude': 35., 'longitude': 125., 'heading': 20., 'accuracy': 5., 'gps_speed': 0.})
  serv._update_gps(0., sm, 'gpsLocation')
  assert (serv.position_source, serv.vpPosPointLat) == ('external_navi', 36.)
  serv._gps_sources().remove('external_navi')
  clock[0] = 10.5
  serv._update_gps(0., sm, 'gpsLocation')
  assert (serv.position_source, serv.vpPosPointLat, serv.last_calculate_gps_time) == ('phone', 35., 10.)
  serv.update({'latitude': 35., 'longitude': 125., 'heading': 20., 'accuracy': 100.})
  serv._update_gps(0., sm, 'gpsLocation')
  assert (serv.position_source, serv.vpPosPointLat) == ('vehicle', 37.)
  sm['carState'].vehicleGps.valid = False
  serv._update_gps(0., sm, 'gpsLocation')
  assert (serv.position_source, serv.vpPosPointLat) == ('device', 38.)
  sm.updated['gpsLocation'] = False
  clock[0] = 13.
  serv._update_gps(0., sm, 'gpsLocation')
  assert serv.position_source == 'none' and serv.vpPosPointLat == 0.


def test_unknown_course_cannot_extrapolate_vehicle_position(context):
  serv, sm, clock = context
  sm['carState'].vehicleGps.headingValid = False
  clock[0] = 11.
  serv.nPosAngle = 180.
  serv._update_gps(20., sm, 'gpsLocation')
  assert not serv.position_heading_valid
  assert (serv.vpPosPointLat, serv.vpPosPointLon) == (37., 127.)
  assert serv.last_calculate_gps_time == 10.


@pytest.mark.parametrize('failure', ['dead', 'invalid', 'stale'])
def test_unusable_vehicle_falls_back_to_device(context, failure):
  serv, sm, clock = context
  if failure == 'dead':
    sm.alive['carState'] = False
  elif failure == 'invalid':
    sm.valid['carState'] = False
  else:
    sm['carState'].vehicleGps.sampleMonoTime = 7_000_000_000
  serv._update_gps(0., sm, 'gpsLocation')
  assert serv.position_source == 'device'


def test_bad_ego_speed_does_not_produce_nonfinite_coordinates(context):
  serv, sm, clock = context
  clock[0] = 11.
  serv._update_gps(float('nan'), sm, 'gpsLocation')
  assert (serv.vpPosPointLat, serv.vpPosPointLon) == (37., 127.)
