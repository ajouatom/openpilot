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


def test_new_position_without_heading_cannot_orient_model_map(sm):
  sm['carrotMan'].positionSource = 'vehicle'
  sm['carrotMan'].positionHeadingValid = False
  assert helpers.navigation_position(sm).latitude == 38.
  sm['gpsLocation'].hasFix = False
  assert helpers.navigation_position(sm) is None
  sm['carrotMan'].positionHeadingValid = True
  assert helpers.navigation_position(sm).latitude == 37.


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


@pytest.fixture
def parked(sm, monkeypatch):
  clock = [10.]
  monkeypatch.setattr(helpers.time, 'monotonic', lambda: clock[0])
  sm['carState'] = NS(gearShifter='park', standstill=True, vEgo=0.)
  sm.valid['carState'] = True
  sm.alive = dict.fromkeys(sm, True)
  sm.recv_time['carState'] = 10.
  sm['carrotMan'].xPosSpeed = 0.
  position = helpers.ParkedPosition()
  def tick(now, fresh=True):
    clock[0] = now
    sm.recv_time['carState'] = now
    if fresh:
      sm.recv_time['carrotMan'] = now
      sm['carrotMan'].positionMonoTime = sm['carrotMan'].positionFixMonoTime = int(now * 1e9)
    else:
      sm.valid['carrotMan'] = sm.valid['gpsLocation'] = False
    return position.update(sm)
  for i in range(12):
    tick(10. + i * .1)
  return sm, position, tick, clock


def test_parked_map_survives_gps_loss_without_renewing_fix(parked):
  _, position, tick, _ = parked
  original = position.anchor
  for i in range(1, 12001):
    result = tick(11.1 + i * .1, fresh=False)
    assert result is original
  assert position.held and result.speed == 0. and result.time == 11.1


@pytest.mark.parametrize('change', ['drive', 'reverse', 'rolling', 'not_standstill', 'invalid', 'dead', 'nan'])
def test_parked_map_rejects_motion_or_lost_vehicle_evidence(parked, change):
  sm, position, tick, _ = parked
  if change in ('drive', 'reverse'):
    sm['carState'].gearShifter = change
  elif change == 'rolling':
    sm['carState'].vEgo = .02
  elif change == 'not_standstill':
    sm['carState'].standstill = False
  elif change == 'invalid':
    sm.valid['carState'] = False
  elif change == 'dead':
    sm.alive['carState'] = False
  else:
    sm['carState'].vEgo = float('nan')
  assert tick(11.2, fresh=False) is None
  assert position.anchor is None and not position.held
  sm['carState'] = NS(gearShifter='park', standstill=True, vEgo=0.)
  sm.valid['carState'] = sm.alive['carState'] = True
  assert tick(11.3, fresh=False) is None


def test_parked_map_cannot_revive_cached_fix_after_process_gap(parked):
  _, position, tick, _ = parked
  assert tick(12., fresh=False) is None
  assert position.anchor is None


def test_parked_map_rejects_stale_vehicle_heartbeat(parked):
  sm, position, _, clock = parked
  sm.valid['carrotMan'] = sm.valid['gpsLocation'] = False
  sm.recv_time['carState'] = 10.
  clock[0] = 11.2
  assert position.update(sm) is None


def test_parked_map_never_seeds_from_old_fix_at_startup(parked):
  sm, _, _, _ = parked
  sm.valid['carrotMan'] = sm.valid['gpsLocation'] = False
  assert helpers.ParkedPosition().update(sm) is None


def test_fresh_fix_replaces_parked_anchor(parked):
  sm, position, tick, _ = parked
  sm['carrotMan'].xPosLat = 37.001
  assert tick(11.2).latitude == 37.001
  assert tick(11.3, fresh=False).latitude == 37.001
  assert position.held


def test_moving_gps_never_seeds_parked_anchor(parked):
  sm, position, tick, _ = parked
  sm['carrotMan'].xPosSpeed = 3.6
  tick(11.2)
  assert position.anchor is None
  assert tick(11.3, fresh=False) is None


def test_extrapolated_position_cannot_seed_from_fix_before_parking(parked):
  sm, position, _, clock = parked
  position.anchor = None
  sm['carrotMan'].positionFixMonoTime = 9_900_000_000
  sm['carrotMan'].positionMonoTime = 11_200_000_000
  sm.recv_time['carrotMan'] = sm.recv_time['carState'] = clock[0] = 11.2
  assert position.update(sm) is not None
  assert position.anchor is None
