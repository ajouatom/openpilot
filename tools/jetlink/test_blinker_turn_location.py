"""Actual GPS passage must not be inferred from route-snapped pose or a timer."""
import math
from types import SimpleNamespace as N

import pytest

from blink_turn_location import TurnLocation, observations, navigation_cancelled
from blink_policy import AutomaticSignals

TARGET = (37., 127., 'session')


def position(north, stamp, east=0, accuracy=2):
  return (37 + north/110540, 127 + east/(111320*math.cos(math.radians(37))), stamp, accuracy)


def tracker():
  return TurnLocation(position(-40, 10), TARGET, 0., 40.)


def test_raw_straight_passage_cancels_only_after_near_passage_and_confirmation():
  t = tracker()
  for now, north in ((10, -40), (11, -20), (12, 0), (13, 20), (14, 30)):
    assert not t.update(now, position(north, now), heading=0, speed=20, angle=0, turned=False)[1]
  assert t.update(15, position(50, 15), heading=0, speed=20, angle=0, turned=False) == (True, True)


@pytest.mark.parametrize('fault', ['stopped', 'turning', 'parallel', 'jump', 'gap', 'missing', 'duplicate'])
def test_no_straight_cancel_without_continuous_real_crossing(fault):
  t = tracker()
  t.update(10, position(-40, 10), heading=0, speed=20, angle=0, turned=False)
  for i in range(1, 8):
    now = 10+i
    north = -40 + i*20
    p = position(north, now, 30 if fault == 'parallel' else 0)
    if fault == 'jump':
      p = position(200+i*20, now)
    if fault == 'missing':
      p = None
    if fault == 'duplicate':
      p = position(30, 11)
    if fault == 'gap':
      p = position(north, 10+i*3)
    assert not t.update(now, p, heading=.8 if fault == 'turning' else 0,
                        speed=0 if fault == 'stopped' else 20, angle=0, turned=fault == 'turning')[1]


@pytest.mark.parametrize('fault', ['no_point', 'no_fix', 'distance', 'curved', 'behind'])
def test_uncertain_anchor_cannot_enable_location_cancel(fault):
  t = TurnLocation(None if fault == 'no_fix' else position(40 if fault == 'behind' else -40, 10),
                   None if fault == 'no_point' else TARGET,
                   .8 if fault == 'curved' else 0, 120 if fault == 'distance' else 40)
  assert t.target is None


def test_passage_can_complete_real_turn_even_when_tbt_distance_lags():
  a = AutomaticSignals()
  a.update(9, 1, True, 0, 'none', '', 'none')
  def step(now, north, east=0, angle=0, heading=0):
    return a.update(now, 1, True, 10, 'right', 'turn right', 'none', speed=10,
                    angle=angle, heading=heading, distance=40, target=TARGET, position=position(north, now, east))
  assert step(10, -40)
  for now, north in ((11,-30),(12,-20),(13,-10),(14,0)):
    assert step(now, north)
  assert step(15, 0, 10, -60, .8)
  assert step(16, 0, 20, -5, 1.5)
  assert step(16.5, 0, 25, -5, 1.5) is None
  assert a.reason == 'completed'


def test_straight_cancel_does_not_replay_same_audio_cue():
  a = AutomaticSignals()
  a.update(9, 1, True, 0, 'none', '', 'none')
  for i in range(7):
    now = 10+i
    result = a.update(now, 1, True, 10, 'right', 'turn right', 'none', speed=20,
                      heading=0, angle=0, distance=40, target=TARGET, position=position(-40+i*20, now))
  assert result is None and a.reason == 'straight_passed'
  assert a.update(17, 1, True, 10, 'right', 'turn right', 'none') is None
  assert a.update(18, 1, True, 18, 'right', 'turn right', 'none', target=TARGET) is None
  assert a.reason == 'cancelled_target'
  assert a.update(19, 1, True, 19, 'right', 'turn right', 'none', target=(37.001,127,'session'))


def scene():
  gps = N(hasFix=True, latitude=37., longitude=127., horizontalAccuracy=2.)
  g = N(meta=N(present=True, receivedMonoTimeNanos=10_000_000_000), pointValid=True,
        latitude=37.0004, longitude=127., distanceM=40)
  status = N(meta=N(present=True, receivedMonoTimeNanos=10_000_000_000), guidanceActive=True, offRoute=False)
  nav = N(connected=True, sessionId='session', guidanceCurrent=g, navigationStatus=status)
  class SM(dict):
    pass
  sm = SM(gpsLocationExternal=gps, carrotNavi=nav, carrotMan=N(xTurnInfo=2, xDistToTurn=40))
  sm.services=list(sm)
  sm.valid=dict.fromkeys(sm, True)
  sm.alive=dict.fromkeys(sm, True)
  sm.recv_time=dict.fromkeys(sm, 10.)
  return sm


@pytest.mark.parametrize('fault', ['fix', 'accuracy', 'stale', 'nan'])
def test_source_rejects_unusable_raw_vehicle_gps(fault):
  sm = scene()
  gps = sm['gpsLocationExternal']
  if fault == 'fix':
    gps.hasFix = False
  if fault == 'accuracy':
    gps.horizontalAccuracy = 20
  if fault == 'stale':
    sm.recv_time['gpsLocationExternal'] = 1
  if fault == 'nan':
    gps.latitude = math.nan
  assert observations(sm, 10.1)[0] is None


@pytest.mark.parametrize('fault', ['point', 'stale', 'distance', 'off_route', 'inactive'])
def test_source_rejects_unusable_target(fault):
  sm = scene()
  nav = sm['carrotNavi']
  if fault == 'point':
    nav.guidanceCurrent.pointValid = False
  if fault == 'stale':
    nav.guidanceCurrent.meta.receivedMonoTimeNanos = 1
  if fault == 'distance':
    nav.guidanceCurrent.distanceM = 100
  if fault == 'off_route':
    nav.navigationStatus.offRoute = True
  if fault == 'inactive':
    nav.navigationStatus.guidanceActive = False
  assert observations(sm, 10.1)[1] is None
  assert navigation_cancelled(sm, 10.1) == (fault in ('off_route', 'inactive'))


def test_stale_navigation_status_never_invents_cancellation():
  sm = scene()
  sm['carrotNavi'].navigationStatus.offRoute = True
  assert navigation_cancelled(sm, 10.1)
  assert not navigation_cancelled(sm, 15)


def test_current_schema_observations_use_raw_gps_not_phone_vehicle():
  pytest.importorskip('capnp')
  from openpilot.cereal import log, custom
  sm = scene()
  n = custom.CarrotNaviState.new_message(connected=True, sessionId='session')
  n.guidanceCurrent.meta.present = True
  n.guidanceCurrent.meta.receivedMonoTimeNanos = 10_000_000_000
  n.guidanceCurrent.pointValid = True
  n.guidanceCurrent.latitude, n.guidanceCurrent.longitude = 37.0004, 127.
  n.guidanceCurrent.distanceM = 40
  n.navigationStatus.guidanceActive = True
  n.vehicle.latitude, n.vehicle.longitude = 0, 0
  gps = log.GpsLocationData.new_message(hasFix=True, latitude=37, longitude=127, horizontalAccuracy=2)
  with custom.CarrotNaviState.from_bytes(n.to_bytes()) as nav, log.GpsLocationData.from_bytes(gps.to_bytes()) as g:
    sm['carrotNavi'], sm['gpsLocationExternal'] = nav, g
    pos, target = observations(sm, 10.1)
    assert pos[:2] == (37., 127.) and target == (37.0004, 127., 'session')


def test_session_change_cancels_owned_turn_without_replay():
  a = AutomaticSignals()
  a.update(9, 1, True, 0, 'none', '', 'none')
  assert a.update(10, 1, True, 10, 'right', 'turn right', 'none', distance=40,
                  position=position(-40, 10), target=TARGET)
  assert a.update(11, 1, True, 10, 'right', 'turn right', 'none', distance=40,
                  position=position(-30, 11), target=(37,127,'replacement')) is None
  assert a.reason == 'navigation_session'


def test_known_target_still_ahead_blocks_premature_completion():
  a = AutomaticSignals()
  a.update(9, 1, True, 0, 'none', '', 'none')
  assert a.update(10, 1, True, 10, 'right', 'turn right', 'none', distance=40,
                  position=position(-40, 10), target=TARGET)
  for now, north, angle in ((11,-30,-60),(12,-20,-5),(12.5,-15,-5)):
    assert a.update(now, 1, True, 10, 'right', 'straight', 'none', speed=10,
                    angle=angle, heading=.6, distance=0, position=position(north, now))
