from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.carrot import signal_road_policy as policy
from openpilot.selfdrive.carrot.carrot_navi_cereal import _lane
from openpilot.selfdrive.carrot.signal_assist import SignalAssist
from openpilot.selfdrive.carrot.signal_start_guard import SignalStartGuard
from openpilot.selfdrive.carrot.signal_stop_prepare import SignalStopPrepare
from openpilot.selfdrive.carrot.tests.test_signal_stop_prepare import observation
from openpilot.selfdrive.carrot.tests.test_signal_start_guard import observation as start_observation
from tools.signal_analysis.replay_signal_assist import planner_class, REPO
from tools.signal_analysis.test_signal_assist_planner import inputs


class SM(dict):
  def __init__(self, category=0, speed=22., distance=91., lead=False):
    super().__init__(inputs(speed=speed, distance=distance, lead=lead))
    self.seen = self.valid = self.alive = {'carrotNavi': True}
    self.recv_time = {'carrotNavi': 100.}
    self['carrotNavi'] = NS(schemaVersion=1, connected=True, sessionId='test',
      navigationStatus=NS(meta=NS(present=True), offRoute=False),
      laneCurrent=NS(meta=NS(present=True, receivedMonoTimeNanos=100_000_000_000),
                     roadCategory=category, roadCategoryValid=True))


@pytest.mark.parametrize('category,expected', [(0, True), (1, True), (2, False), (8, False), (-1, False), (12, False)])
def test_explicit_road_class(category, expected):
  assert policy.signal_detection_suspended(SM(category), 100.) is expected


@pytest.mark.parametrize('bad', ['unseen', 'invalid', 'dead', 'disconnected', 'session', 'version', 'offroute',
  'absent', 'missing_category', 'stale_message', 'future_message', 'stale_item', 'future_item', 'zero_time'])
def test_unknown_data_never_means_highway(bad):
  sm = SM(); navi = sm['carrotNavi']; lane = navi.laneCurrent
  if bad == 'unseen': sm.seen = {'carrotNavi': False}
  if bad == 'invalid': sm.valid = {'carrotNavi': False}
  if bad == 'dead': sm.alive = {'carrotNavi': False}
  if bad == 'disconnected': navi.connected = False
  if bad == 'session': navi.sessionId = ''
  if bad == 'version': navi.schemaVersion = 0
  if bad == 'offroute': navi.navigationStatus.offRoute = True
  if bad == 'absent': lane.meta.present = False
  if bad == 'missing_category': lane.roadCategoryValid = False
  if bad == 'stale_message': sm.recv_time['carrotNavi'] = 97.
  if bad == 'future_message': sm.recv_time['carrotNavi'] = 101.
  if bad == 'stale_item': lane.meta.receivedMonoTimeNanos = 89_000_000_000
  if bad == 'future_item': lane.meta.receivedMonoTimeNanos = 101_000_000_000
  if bad == 'zero_time': lane.meta.receivedMonoTimeNanos = 0
  assert not policy.signal_detection_suspended(sm, 100.)


@pytest.mark.parametrize('value,valid', [(0, True), (1, True), (8, True), (12, True),
  (None, False), ('0', False), (True, False), (-1, False), (13, False), (float('nan'), False)])
def test_source_category_presence_and_native_schema(value, valid):
  lane = _lane({'present': True, 'received_mono_ns': 100_000_000_000}, {'road_category': value})
  assert lane['roadCategoryValid'] is valid
  msg = log.Event.new_message(); navi = msg.init('carrotNavi')
  navi.schemaVersion = 1; navi.connected = True; navi.sessionId = 'test'; navi.laneCurrent = lane
  sm = SM(); sm['carrotNavi'] = navi
  assert policy.signal_detection_suspended(sm, 100.) == (valid and value in (0, 1))


def make_planner(monkeypatch):
  monkeypatch.setattr(policy.time, 'monotonic', lambda: 100.)
  cls, ns = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf8'))
  p = cls(signal_assist=SignalAssist(), start_guard=SignalStartGuard(), stop_prepare=SignalStopPrepare())
  return p, ns


def update(p, sm, t=100., red=False):
  ctx = dict(now=t, enabled=True, valid=True, drive=True)
  return p.update(sm, 85., 'acc', signal_context=ctx, start_context=ctx,
    signal_observation=observation(t) if red else None,
    start_observation=start_observation(t) if red else None,
    prepare_observation=observation(t) if red else None,
    prepare_context=dict(ctx, turning=False, geometry=lambda box: (30., 70.), actuate=True))


@pytest.mark.parametrize('category', [0, 1, 8])
def test_incident_model_outputs_stop_only_off_highway(monkeypatch, category):
  p, _ = make_planner(monkeypatch); sm = SM(category)
  sm['modelV2'].velocity.x = [21.02] + [2.039] * 32
  for i in range(20): update(p, sm, 100.+i*.05, red=True)
  assert p.signal_detection_suspended == (category in (0, 1))
  if category in (0, 1):
    assert p.trafficState.value == 0 and p.xState.name == 'e2eCruise'
    assert p.stop_dist == 1000. and p.v_cruise > 20.
    assert not p.signal_decision.hold and not p.start_decision.hold and not p.prepare_control_active
    assert not p.events.events
  else:
    assert p.trafficState.value == 1 and p.xState.name == 'e2eStop'
    assert p.stop_dist < 100.


def test_road_exit_resumes_detection_without_changing_user_mode(monkeypatch):
  p, _ = make_planner(monkeypatch); sm = SM()
  sm['modelV2'].velocity.x = [21.] + [2.] * 32
  update(p, sm); assert p.xState.name == 'e2eCruise'
  sm['carrotNavi'].laneCurrent.roadCategory = 8
  update(p, sm); assert p.xState.name == 'e2eStop'
  assert p.trafficLightDetectMode == 2


@pytest.mark.parametrize('state', ['e2eStop', 'e2eStopped', 'e2ePrepare', 'e2eStopPrepare'])
def test_highway_clears_signal_state_and_keeps_lead_priority(monkeypatch, state):
  p, ns = make_planner(monkeypatch); sm = SM(lead=True)
  p.xState = ns['XState'][state]; p.actual_stop_distance = 5.
  update(p, sm)
  assert p.xState.name == 'lead' and p.stop_dist == 1000.


def test_highway_preserves_user_stop_and_soft_hold(monkeypatch):
  p, _ = make_planner(monkeypatch); sm = SM(speed=10.)
  p.user_stop_distance = 25.
  update(p, sm)
  assert p.xState.name == 'e2eStop' and 23. < p.stop_dist < 25.
  p.user_stop_distance = -1; sm['carState'].softHoldActive = 1
  update(p, sm)
  assert p.xState.name == 'e2eStopped'


def test_highway_clears_acquired_red_veto_without_green_event(monkeypatch):
  p, _ = make_planner(monkeypatch); sm = SM(8, speed=0.)
  for i in range(6): update(p, sm, 100.+i*.1, red=True)
  assert p.start_decision.hold
  sm['carrotNavi'].laneCurrent.roadCategory = 0
  update(p, sm, 100.7, red=True)
  assert not p.start_decision.hold and not p.signal_decision.hold
  assert p.xState.name == 'e2eCruise' and not p.events.events


def test_highway_ignores_external_signal_transition_but_keeps_speed_cap(monkeypatch):
  p, ns = make_planner(monkeypatch); sm = SM(speed=0.)
  sm['carState'].softHoldActive = 1
  p.trafficState_carrot = 1
  ns['get_carrot_man'] = lambda sm: NS(trafficState=2, atcType='', activeCarrot=2,
                                      xDistToTurn=200, desiredSpeed=40.)
  result = update(p, sm)
  assert result == 40. and p.xState.name == 'e2eStopped'
  assert p.trafficState_carrot == 0 and not p.events.events


def test_highway_cancels_already_active_preparation(monkeypatch):
  p, _ = make_planner(monkeypatch); sm = SM(8, speed=15., distance=160.)
  for i in range(25): update(p, sm, 100.+i*.05, red=True)
  assert p.prepare_control_active
  sm['carrotNavi'].laneCurrent.roadCategory = 1
  update(p, sm, 101.3, red=True)
  assert not p.prepare_control_active and not p.prepare_decision.active
  assert p.xState.name == 'e2eCruise' and not p.events.events
