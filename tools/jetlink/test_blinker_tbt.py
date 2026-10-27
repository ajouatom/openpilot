from types import SimpleNamespace as NS

import pytest

from blink_policy import Policy
from blink_tbt import TbtBlinker, candidate


def scene():
  cs = NS(vEgo=20., gearShifter='drive', canValid=True, trailerConnected=False,
          steeringPressed=False, steeringTorque=0., steerFaultTemporary=False, steerFaultPermanent=False,
          tbtRearInputsAvailable=True, tbtRightRearBlocked=False, rightBlindspot=False,
          rightLaneLine=10, leftBlinker=False, rightBlinker=False, aEgo=0.)
  model = NS(laneLineProbs=[.9]*4, meta=NS(laneChangeAvailableRight=True, laneWidthRight=3.5,
             desireState=[0.]*8, laneChangeState='off', laneChangeDirection='none'))
  nav = NS(xTurnInfo=2, xDistToTurn=230., atcType='none', carrotCmdIndex=0, carrotCmd='', carrotArg='')
  return {'cs': cs, 'model': model, 'nav': nav, 'enabled': True, 'valid': True}


def arm(tbt, s, start=10.):
  for i in range(22):
    result = tbt.update(start+i*.05, **s)
  return result


@pytest.mark.parametrize('obj,field,value', [
  ('nav','xTurnInfo',1), ('nav','xTurnInfo',4), ('nav','xDistToTurn',323.),
  ('nav','xDistToTurn',0.), ('nav','xDistToTurn',float('nan')),
  ('cs','vEgo',8.), ('cs','gearShifter','park'), ('cs','canValid',False),
  ('cs','trailerConnected',True), ('cs','steeringPressed',True),
  ('cs','tbtRearInputsAvailable',False), ('cs','tbtRightRearBlocked',True),
  ('cs','rightBlindspot',True), ('cs','rightLaneLine',11), ('cs','rightLaneLine',20),
  ('cs','rightLaneLine',-1), ('cs','steerFaultTemporary',True),
])
def test_ineligible_conditions_never_light(obj, field, value):
  s = scene()
  setattr(s[obj], field, value)
  assert arm(TbtBlinker(), s) is None


def test_lane_required_not_just_road_edge():
  s = scene()
  s['model'].laneLineProbs[3] = .1
  assert not candidate(s['cs'], s['model'], s['nav'])
  assert arm(TbtBlinker(), s) is None


def test_enabled_fresh_and_stable_required():
  for field in ('enabled', 'valid'):
    s = scene()
    s[field] = False
    assert arm(TbtBlinker(), s) is None
  t, s = TbtBlinker(), scene()
  assert t.update(10., **s) is None
  assert t.update(11., **s) is None  # scheduler/input gap cannot count as confirmation
  assert arm(t, s, 11.05) == ('lane', 'right')


def test_physical_policy_receives_one_right_indicator_session():
  t, p, s = TbtBlinker(), Policy(), scene()
  ids = set()
  for i in range(30):
    now = 10+i*.05
    automatic = t.update(now, **s)
    taps, hold = p.update(now, 2, True, [], 20., 0., 'drive', False, automatic, True)
    assert not taps
    if hold:
      assert hold['source'] == 'lane' and hold['direction'] == 'right'
      ids.add(hold['id'])
  assert len(ids) == 1


@pytest.mark.parametrize('event', ['steering', 'bsd', 'stale', 'manual', 'route_jump', 'opposite'])
def test_cancel_before_start_does_not_retrigger_same_tbt(event):
  t, s = TbtBlinker(), scene()
  assert arm(t, s) == ('lane', 'right')
  if event == 'steering':
    s['cs'].steeringPressed = True
  if event == 'bsd':
    s['cs'].rightBlindspot = True
  if event == 'stale':
    s['valid'] = False
  if event == 'manual':
    s['manual'] = True
  if event == 'route_jump':
    t.last_distance = 160.
  if event == 'opposite':
    s['cs'].leftBlinker = True
  assert t.update(11.1, **s) is None
  s = scene()
  for i in range(50):
    assert t.update(11.15+i*.05, **s) is None


def test_repeat_only_after_finish_lamps_off_and_fresh_confirmation():
  t, s = TbtBlinker(), scene()
  assert arm(t, s) == ('lane', 'right')
  s['cs'].rightBlinker = True
  s['model'].meta.laneChangeState = 'laneChangeStarting'
  s['model'].meta.laneChangeDirection = 'right'
  s['model'].meta.laneChangeAvailableRight = False  # crossing changes geometry
  assert t.update(11.1, **s) == ('lane', 'right')
  s['model'].meta.laneChangeState = 'laneChangeFinishing'
  assert t.update(11.15, **s) == ('lane', 'right')
  s['model'].meta.laneChangeState = 'preLaneChange'
  assert t.update(11.2, **s) is None
  for i in range(50):
    assert t.update(11.25+i*.05, **s) is None
  s['cs'].rightBlinker = False
  s['model'].meta.laneChangeState = 'off'
  s['model'].meta.laneChangeAvailableRight = True
  assert arm(t, s, 13.75) == ('lane', 'right')


def test_interrupted_maneuver_is_not_completion():
  t, s = TbtBlinker(), scene()
  arm(t, s)
  s['model'].meta.laneChangeState = 'laneChangeStarting'
  s['model'].meta.laneChangeDirection = 'right'
  assert t.update(11.1, **s)
  s['model'].meta.laneChangeState = 'off'
  assert t.update(11.15, **s) is None
  assert t.phase == 'cancelled'


def test_start_timeout_does_not_keep_tapping():
  t, s = TbtBlinker(), scene()
  arm(t, s)
  for i in range(100):
    t.update(11.1+i*.05, **s)
  assert t.phase == 'cancelled'


@pytest.mark.parametrize('kph', [30., 60., 100.])
def test_lane_change_speed_and_fixed_250_metre_boundary(kph):
  s = scene()
  s['cs'].vEgo = kph / 3.6
  s['nav'].xDistToTurn = 250.
  assert candidate(s['cs'], s['model'], s['nav'])
  s['nav'].xDistToTurn = 250.1
  assert not candidate(s['cs'], s['model'], s['nav'])


def test_no_fixed_sixty_metre_cutoff_but_actual_turn_stops_trigger():
  s, t = scene(), TbtBlinker()
  s['nav'].xDistToTurn = 40.
  assert arm(t, s) == ('lane', 'right')
  s['model'].meta.desire = 'turnRight'
  assert t.update(11.1, **s) is None
  assert t.reason == 'turn_started'
  s['model'].meta.desire = 'none'
  assert arm(t, s, 11.15) is None


def test_real_lamp_feedback_uses_unchanged_desire_path(monkeypatch):
  import sys
  # Exercise the real maneuver policy without requiring device Params bindings.
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS()))
  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware', NS(PC=True, TICI=False))
  from openpilot.selfdrive.controls.lib import desire_helper as module
  from openpilot.cereal import log
  monkeypatch.setattr(module, 'Params', lambda: NS())
  h = module.DesireHelper()
  h._update_params_periodic = lambda: None
  h._check_desire_state = lambda *args: None
  h._process_sides = lambda *args: None
  h.bluetooth_commands = NS(read=lambda **kw: None)
  side = h.right
  side.lane_available = side.lane_change_available = side.lane_change_available_geom = True
  side.lane_exist_count.counter = 20
  side.dist_to_edge_far = 3.5
  s = scene()
  # New rear fields have no authority over ordinary manual/remote admission.
  s['cs'].tbtRightRearBlocked = True
  h.update(s['cs'], s['model'], True, .1, s['nav'], NS())
  assert h.lane_change_state == log.LaneChangeState.off
  s['cs'].rightBlinker = True  # actual lamp feedback, not a lane command
  h.update(s['cs'], s['model'], True, .1, s['nav'], NS())
  assert h.lane_change_state == log.LaneChangeState.preLaneChange
  h.update(s['cs'], s['model'], True, .1, s['nav'], NS())
  assert h.desire == log.Desire.laneChangeRight


@pytest.mark.parametrize('atc,torque,blocked,expected', [(4, 0, False, True), (2, 0, False, False),
                                                       (4, 1, False, False), (4, 0, True, False)])
def test_actual_source_loop_only_emits_physical_indicator(monkeypatch, atc, torque, blocked, expected):
  import json
  import sys
  from contextlib import nullcontext
  import blink_source as source
  from test_blinker import Vehicle
  from openpilot.selfdrive.carrot.bluetooth import model as bluetooth
  s, sm, sent, clock = scene(), Vehicle(), [], [10.]
  sm['carState'] = s['cs']
  sm['carState'].steeringAngleDeg = 0.
  sm['carState'].tbtRightRearBlocked = blocked
  sm['modelV2'], sm['carrotMan'] = s['model'], s['nav']
  sm['modelV2'].meta.desire = 'none'
  sm['modelV2'].meta.autoTurnSignal = 'none'
  sm['modelV2'].meta.autoTurnSignalLaneChange = False
  sm['selfdriveState'].atcTurnStartMonoTime = 0
  sm['selfdriveState'].atcTurnDirection = 'none'
  sm.services = list(sm)
  sm.valid = dict.fromkeys(sm, True)
  sm.alive = dict.fromkeys(sm, True)
  def tick(_):
    clock[0] = 10. + len(sent) * .05
    sm.recv_time = dict.fromkeys(sm, clock[0])
  sm.update = tick
  values = {'AutoBlinkerControl': 2, 'AutoTurnControl': atc, 'LaneChangeNeedTorque': torque}
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(
    get=lambda _: b'cp', get_int=lambda key: values.get(key, 0), get_bool=lambda _: False)))
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', NS(messaging=NS(SubMaster=lambda _: sm),
    car=NS(CarParams=NS(from_bytes=lambda _: nullcontext(NS(carFingerprint='HYUNDAI_IONIQ_5_PE'))))))
  sock = NS(setblocking=lambda _: None, recv=lambda _: b'', send=lambda raw: sent.append(json.loads(raw)))
  monkeypatch.setattr(source.socket, 'socket', lambda **kw: sock)
  monkeypatch.setattr(source.select, 'select', lambda *args: ([sock] if len(sent) == 36 else [], [], []))
  monkeypatch.setattr(source.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(source.time, 'sleep', lambda _: None)
  monkeypatch.setattr(bluetooth, 'read_json', lambda *args: {})
  monkeypatch.setattr(bluetooth, 'config', dict)
  monkeypatch.setattr(bluetooth, 'atomic_json', lambda *args: None)
  source.main(0)
  holds = [row['hold'] for row in sent if row['hold']]
  assert bool(holds) == expected
  assert all(not row['events'] for row in sent)
  assert all(h['source'] == 'lane' and h['direction'] == 'right' for h in holds)
  assert s['nav'].carrotCmd == '' and s['nav'].carrotCmdIndex == 0
