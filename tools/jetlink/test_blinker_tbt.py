from types import SimpleNamespace as NS

import pytest

from blink_policy import Policy
from blink_tbt import TbtBlinker, candidate, right_front_clear


def scene():
  cs = NS(vEgo=20., gearShifter='drive', canValid=True, trailerConnected=False,
          steeringPressed=False, steeringTorque=0., steerFaultTemporary=False, steerFaultPermanent=False,
          tbtRearInputsAvailable=True, tbtRightRearBlocked=False, rightBlindspot=False,
          rightLaneLine=10, leftBlinker=False, rightBlinker=False, aEgo=0.)
  model = NS(laneLineProbs=[.9]*4, meta=NS(laneChangeAvailableRight=True, laneWidthRight=3.5,
             desireState=[0.]*8, laneChangeState='off', laneChangeDirection='none'))
  nav = NS(xTurnInfo=2, xDistToTurn=230., atcType='none', carrotCmdIndex=0, carrotCmd='', carrotArg='')
  return {'cs': cs, 'model': model, 'nav': nav, 'enabled': True, 'valid': True, 'front_clear': True}


def arm(tbt, s, start=10.):
  for i in range(22):
    result = tbt.update(start+i*.05, **s)
  return result


@pytest.mark.parametrize('obj,field,value', [
  ('nav','xTurnInfo',1), ('nav','xTurnInfo',3), ('nav','xDistToTurn',323.),
  ('nav','xDistToTurn',0.), ('nav','xDistToTurn',float('nan')),
  ('cs','vEgo',8.), ('cs','gearShifter','park'), ('cs','canValid',False),
  ('cs','trailerConnected',True), ('cs','steeringPressed',True),
  ('cs','tbtRearInputsAvailable',False), ('cs','tbtRightRearBlocked',True),
  ('cs','rightLaneLine',11), ('cs','rightLaneLine',20),
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
    s['cs'].tbtRightRearBlocked = True
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


@pytest.mark.parametrize('front', ['clear', 'close', 'closing', 'stale', 'invalid', 'fault', 'unavailable'])
@pytest.mark.parametrize('ordinary', ['none', 'turn', 'lane'])
@pytest.mark.parametrize('atc,addon,torque,blocked,expected', [
  (1, True, 0, False, True), (2, True, 0, False, True),
  (1, False, 0, False, False), (2, False, 0, False, False),
  (0, True, 0, False, False), (3, True, 0, False, False), (4, True, 0, False, False),
  (1, True, 1, False, False), (2, True, 0, True, False),
])
def test_actual_source_loop_only_emits_physical_indicator(monkeypatch, atc, addon, torque, blocked, expected, ordinary, front):
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
  sm['radarState'] = radar_scene()
  if front in ('close', 'closing'):
    sm['radarState'].leadsRight = [NS(status=True, dRel=4.9 if front == 'close' else 15., vRel=-4.)]
  sm['radarState'].radarErrors.radarFault = front == 'fault'
  expected = expected and front == 'clear'
  sm['livePose'] = NS(inputsOK=True, sensorsOK=True, posenetOK=True, orientationNED=NS(valid=True, z=0.))
  sm.services = list(sm)
  sm.valid = dict.fromkeys(sm, True)
  sm.alive = dict.fromkeys(sm, True)
  def tick(_):
    clock[0] = 10. + len(sent) * .05
    sm.recv_time = dict.fromkeys(sm, clock[0])
    if front == 'stale':
      sm.recv_time['radarState'] -= .36
    sm.valid['radarState'] = front != 'invalid'
    if len(sent) >= 24 and ordinary == 'turn':
      sm['selfdriveState'].atcTurnStartMonoTime = 11200000000
      sm['selfdriveState'].atcTurnDirection = 'left'
      sm['carrotMan'].atcType = 'turn left'
    if len(sent) >= 24 and ordinary == 'lane':
      sm['modelV2'].meta.autoTurnSignal = 'left'
      sm['modelV2'].meta.autoTurnSignalLaneChange = True
  sm.update = tick
  values = {'AutoBlinkerControl': 2, 'AutoTurnControl': atc, 'LaneChangeNeedTorque': torque,
            'AutoTurnControlRightLaneChange': addon}
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(
    get=lambda _: b'cp', get_int=lambda key: values.get(key, 0), get_bool=lambda key: bool(values.get(key, False)))))
  monkeypatch.setitem(sys.modules, 'openpilot.cereal', NS(messaging=NS(SubMaster=lambda _: sm),
    car=NS(CarParams=NS(from_bytes=lambda _: nullcontext(NS(carFingerprint='HYUNDAI_IONIQ_5_PE', radarUnavailable=front == 'unavailable'))))))
  sock = NS(setblocking=lambda _: None, recv=lambda _: b'', send=lambda raw: sent.append(json.loads(raw)))
  monkeypatch.setattr(source.socket, 'socket', lambda **kw: sock)
  monkeypatch.setattr(source.select, 'select', lambda *args: ([sock] if len(sent) == 36 else [], [], []))
  monkeypatch.setattr(source.time, 'monotonic', lambda: clock[0])
  monkeypatch.setattr(source.time, 'sleep', lambda _: None)
  monkeypatch.setattr(bluetooth, 'read_json', lambda *args: {})
  monkeypatch.setattr(bluetooth, 'config', dict)
  monkeypatch.setattr(bluetooth, 'atomic_json', lambda *args: None)
  source.main(0)
  holds = [row['hold'] for row in sent[:24] if row['hold']]
  assert bool(holds) == expected
  assert all(not row['events'] for row in sent)
  assert all(h['source'] == 'lane' and h['direction'] == 'right' for h in holds)
  assert s['nav'].carrotCmd == '' and s['nav'].carrotCmdIndex == 0

  if ordinary != 'none':
    # The add-on may already own a right indicator; existing ATC must replace it.
    later = [row['hold'] for row in sent[24:]]
    assert all(h and h['source'] == ordinary and h['direction'] == 'left' for h in later)


@pytest.mark.parametrize("turn,atc", [(2, "atc right"), (4, "fork right")])
def test_existing_right_atc_preparation_does_not_block_250m_indicator(turn, atc):
  s = scene()
  s['model'].meta.laneChangeState = 'preLaneChange'
  s['model'].meta.laneChangeDirection = 'right'
  s['nav'].atcType = atc
  s['nav'].xTurnInfo = turn
  assert arm(TbtBlinker(), s) == ('lane', 'right')
  assert s['nav'].carrotCmd == ''


@pytest.mark.parametrize(
  'state,direction,atc',
  [
    ('laneChangeStarting', 'right', 'atc right'),
    ('laneChangeFinishing', 'right', 'atc right'),
    ('preLaneChange', 'left', 'atc right'),
    ('preLaneChange', 'right', 'atc left'),
    ('preLaneChange', 'right', 'none'),
    ('preLaneChange', 'right', 'turn right'),
  ],
)
def test_250m_never_joins_other_preparations_or_active_maneuvers(state, direction, atc):
  s = scene()
  s['model'].meta.laneChangeState = state
  s['model'].meta.laneChangeDirection = direction
  s['nav'].atcType = atc
  assert arm(TbtBlinker(), s) is None


@pytest.mark.parametrize('block', ['rear', 'bsd', 'torque', 'busy', 'lane'])
def test_existing_preparation_keeps_all_250m_interlocks(block):
  s = scene()
  s['model'].meta.laneChangeState = 'preLaneChange'
  s['model'].meta.laneChangeDirection = 'right'
  s['nav'].atcType = 'atc right'
  if block == 'rear':
    s['cs'].tbtRightRearBlocked = True
  if block == 'bsd':
    s['cs'].tbtRightRearBlocked = True
  if block == 'torque':
    s['cs'].steeringPressed = True
  if block == 'busy':
    s['busy'] = True
  if block == 'lane':
    s['model'].meta.laneChangeAvailableRight = False
  assert arm(TbtBlinker(), s) is None


@pytest.mark.parametrize('turn', [2, 4])
def test_driver_correction_before_any_request_resets_confirmation_not_whole_route(turn):
  t, s = TbtBlinker(), scene()
  s['nav'].xTurnInfo = turn
  assert t.update(10., **s) is None
  s['cs'].steeringPressed = True
  assert t.update(10.2, **s) is None
  assert t.phase == 'idle'
  s['cs'].steeringPressed = False
  for i in range(20):
    assert t.update(10.25+i*.05, **s) is None
  assert t.update(11.3, **s) == ('lane', 'right')


@pytest.mark.parametrize('turn', [2, 4])
def test_fork_support_does_not_bypass_rear_or_distance_gate(turn):
  s = scene()
  s['nav'].xTurnInfo = turn
  s['nav'].xDistToTurn = 284.
  assert arm(TbtBlinker(), s) is None
  s['nav'].xDistToTurn = 240.
  s['cs'].tbtRightRearBlocked = True
  assert arm(TbtBlinker(), s) is None



def test_only_dedicated_oem_bsd_input_controls_the_addon_rear_gate():
  s = scene()
  # These are display/merged fields, not the fresh OEM BSD source. Ordinary
  # DesireHelper side-object admission is still represented by availability.
  s['cs'].rightRearLongDist = 25.0
  s['cs'].rightBlindspot = True
  assert arm(TbtBlinker(), s) == ('lane', 'right')
  s['cs'].tbtRightRearBlocked = True
  assert arm(TbtBlinker(), s) is None
  s['cs'].tbtRightRearBlocked = False
  s['model'].meta.laneChangeAvailableRight = False
  assert arm(TbtBlinker(), s) is None



def radar_scene():
  return NS(radarErrors=NS(canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False),
            leadsRight=[], leadRight=NS(status=False))


@pytest.mark.parametrize('distance,velocity,expected', [
  (0., 0., False), (4.99, 10., False), (5., 0., True), (50., -10., True),
  (15., -4., False), (17., -4., True), (16.99, -4., False), (5., -0.01, False),
  (-2., -10., True), (float('nan'), 0., False), (float('inf'), 0., False),
  (10., float('nan'), False), (10., float('inf'), False),
])
def test_front_gap_and_three_second_projection_include_tracks_omitted_by_primary(distance, velocity, expected):
  radar = radar_scene()
  radar.leadRight = NS(status=True, dRel=60., vRel=0.)
  radar.leadsRight = [NS(status=True, dRel=distance, vRel=velocity), NS(status=True, dRel=60., vRel=0.)]
  assert right_front_clear(radar) == expected


@pytest.mark.parametrize('error', ['canError', 'radarFault', 'wrongConfig', 'radarUnavailableTemporary'])
def test_no_front_target_only_passes_with_healthy_radar(error):
  radar = radar_scene()
  assert right_front_clear(radar)
  setattr(radar.radarErrors, error, True)
  assert not right_front_clear(radar)
  assert not right_front_clear(None)


def test_front_gap_must_stay_clear_during_confirmation_and_waiting():
  s, t = scene(), TbtBlinker()
  s['front_clear'] = False
  assert arm(t, s) is None
  s['front_clear'] = True
  assert arm(t, s, 11.1) == ('lane', 'right')
  s['front_clear'] = False
  assert t.update(12.2, **s) is None
  assert t.phase == 'cancelled'
  s['front_clear'] = True
  assert arm(t, s, 12.25) is None
