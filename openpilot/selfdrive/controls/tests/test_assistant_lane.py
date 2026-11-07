"""Authenticated voice intent through the actual DesireHelper state machine."""
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.tests.test_desire_helper_stalk import TestDesireHelperStalkRequest as StalkFixture
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import AssistantLane
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json


@pytest.fixture
def voice(tmp_path):
  base = StalkFixture()
  base.setup_method()
  helper = base.helper
  now = [100.]
  helper.assistant_lane = AssistantLane(tmp_path, lambda: now[0])
  cs = car.CarState(canValid=True, gearShifter='drive', vEgo=25, vehicleSensorsInvalid=False)
  def tick(**kwargs):
    now[0] += .05
    helper.update(cs, SimpleNamespace(), True, kwargs.get('prob', .5), base.carrot_man, SimpleNamespace(), assistant_valid=kwargs.get('valid', True))
    return helper.lane_change_state
  def request(direction='left'):
    atomic_json(tmp_path / 'request.json', {'id': 'a' * 43, 'time': now[0], 'direction': direction})
  return helper, cs, tick, request, lambda: read_json(tmp_path / 'result.json')


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_voice_enters_existing_lane_change_fsm_once(voice, direction):
  helper, cs, tick, request, result = voice
  tick()
  request(direction)
  assert tick() == log.LaneChangeState.preLaneChange
  assert tick() == log.LaneChangeState.laneChangeStarting
  assert result()['status'] == 'running'
  assert not cs.steeringPressed and not cs.buttonEvents


@pytest.mark.parametrize('condition', ['bsd', 'lane_missing', 'invalid', 'park', 'torque_required'])
def test_normal_vehicle_conditions_are_not_bypassed(voice, condition):
  helper, cs, tick, request, result = voice
  tick()
  if condition == 'bsd':
    cs.leftBlindspot = True
    helper.laneChangeBsd = -1  # Voice still respects a measured blind spot.
  elif condition == 'lane_missing':
    helper.left.lane_change_available_geom = False
  elif condition == 'park':
    cs.gearShifter = 'park'
  elif condition == 'torque_required':
    helper.laneChangeNeedTorque = 1
  request()
  tick(valid=condition != 'invalid')
  for _ in range(65):
    tick()
    assert helper.lane_change_state != log.LaneChangeState.laneChangeStarting
  assert result()['status'] in ('rejected', 'expired')


def test_tesla_voice_waits_for_real_signal_acknowledgement(voice):
  helper, cs, tick, request, result = voice
  helper.tesla_auto_signals = True
  cs.turnSignalDriverValid = True
  tick()
  request()
  for _ in range(5):
    tick()
    assert helper.lane_change_state != log.LaneChangeState.laneChangeStarting
  assert helper.auto_signal_request_id > 0
  cs.autoTurnSignalDirection = 1
  cs.leftBlinker = True
  cs.autoTurnSignalReadyRequestId = helper.auto_signal_request_id
  for _ in range(3):
    tick()
  assert helper.lane_change_state == log.LaneChangeState.laneChangeStarting
  assert result()['status'] == 'running'


@pytest.fixture
def pe(voice):
  helper = voice[0]
  helper.boundary_indicator_supported = True
  helper.voice_lane_signals_enabled = True
  return voice


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_pe_voice_signal_before_lane_start(pe, direction, monkeypatch):
  from pathlib import Path
  monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[4] / 'tools/jetlink'))
  from blink_voice import VoiceLaneSignal
  from blink_policy import Policy

  helper, cs, tick, request, result = pe
  policy, lamp = Policy(), VoiceLaneSignal()
  dark = SimpleNamespace(leftBlinker=False, rightBlinker=False)
  now = helper.assistant_lane.clock
  def update_lamp(meta, active_id=None):
    intent = lamp.select(meta, cs, allowed=True, peer_ready=True, hold=policy.hold, busy=False)
    _, hold = policy.update(now(), 2, True, [], 25., 0., 'drive', False, intent)
    lamp.bind(hold)
    if lamp.confirmed(hold, active_id, {'accepted_id': active_id}, cs):
      atomic_json(helper.assistant_lane.root/'signal.json', {'id':lamp.request, 'direction':lamp.direction, 'time':now(), 'confirmed':True})
    return hold

  tick()
  update_lamp(SimpleNamespace(assistantLaneSignalId='', laneChangeDirection='none', laneChangeState='off'))
  request(direction)
  assert tick() == log.LaneChangeState.preLaneChange
  assert not helper.assistant_lane.signal_id  # delay, torque and classifier gates run before requesting a lamp
  assert tick() == log.LaneChangeState.preLaneChange
  identity = helper.assistant_lane.signal_id
  assert identity == 'a' * 43
  meta = SimpleNamespace(assistantLaneSignalId=identity, laneChangeDirection=direction, laneChangeState='preLaneChange')
  hold = update_lamp(meta)
  assert hold['direction'] == direction and hold['source'] == 'lane'
  assert not lamp.confirmed(hold, hold['id'], {'accepted_id':hold['id']}, dark)
  assert tick() == log.LaneChangeState.preLaneChange
  setattr(cs, direction + 'Blinker', True)
  update_lamp(meta, 'other-receiver-hold')
  assert tick() == log.LaneChangeState.preLaneChange
  path = helper.assistant_lane.root/'signal.json'
  for fields in [{'id':'other'}, {'direction':'right' if direction=='left' else 'left'}, {'time':now()-1}]:
    atomic_json(path, {'id':identity, 'direction':direction, 'time':now(), 'confirmed':True, **fields})
    assert tick() == log.LaneChangeState.preLaneChange
  update_lamp(meta, hold['id'])
  assert tick() == log.LaneChangeState.laneChangeStarting
  assert result()['status'] == 'running'
  meta.laneChangeState = 'laneChangeStarting'
  assert update_lamp(meta)['id'] == hold['id']
  phases = {log.LaneChangeState.laneChangeStarting:'laneChangeStarting', log.LaneChangeState.laneChangeFinishing:'laneChangeFinishing'}
  seen_finishing = False
  for _ in range(60):
    phase = tick(prob=0.)
    meta.assistantLaneSignalId = helper.assistant_lane.signal_id
    meta.laneChangeState = phases.get(phase, 'off')
    remaining = update_lamp(meta)
    if not meta.assistantLaneSignalId:
      assert remaining is None
      break
    assert remaining['id'] == hold['id']
    seen_finishing |= phase == log.LaneChangeState.laneChangeFinishing
  assert seen_finishing and not helper.assistant_lane.signal_id
  for _ in range(10):
    assert tick() != log.LaneChangeState.laneChangeStarting


@pytest.mark.parametrize('fault', ['bsd', 'lane', 'torque', 'mode', 'opposite', 'hazards', 'cancel', 'no_feedback'])
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_pe_voice_does_not_start_under_blocked_conditions(pe, direction, fault):
  helper, cs, tick, request, result = pe
  tick()
  if fault == 'torque':
    helper.laneChangeNeedTorque = 1
  if fault == 'mode':
    helper.voice_lane_signals_enabled = False
  if fault == 'lane':
    getattr(helper, direction).lane_change_available_geom = False
  request(direction)
  tick()
  tick()
  if fault in ('mode', 'lane', 'torque'):
    assert not helper.assistant_lane.signal_id
  else:
    assert helper.assistant_lane.signal_id
  if fault == 'bsd':
    setattr(cs, direction+'Blindspot', True)
  if fault in ('opposite','hazards'):
    setattr(cs, ('right' if direction == 'left' else 'left')+'Blinker', True)
    if fault == 'hazards':
      setattr(cs, direction+'Blinker', True)
  if fault == 'cancel':
    atomic_json(helper.assistant_lane.root/'cancel.json', {'id':'a'*43})
  # Continuing wrong-side feedback must not turn into an opposite voice maneuver.
  assert tick() != log.LaneChangeState.laneChangeStarting
  for _ in range(66):
    assert tick() != log.LaneChangeState.laneChangeStarting
  assert not helper.assistant_lane.signal_id
  assert result()['status'] in ('rejected', 'expired', 'cancelled')
  if fault == 'no_feedback':
    assert result()['reason'] == 'indicator_not_confirmed'


@pytest.mark.parametrize('direction', ['left', 'right'])
@pytest.mark.parametrize('physical_override', [False, True])
def test_pe_tap_echo_requires_this_request_and_never_confirms_lamp(pe, direction, physical_override):
  helper, cs, tick, request, result = pe
  tick()
  request(direction)
  tick()
  tick()
  now = helper.assistant_lane.clock()
  side = 1 if direction == 'left' else 2
  setattr(cs, direction+'BlinkerStalkCount', 1)
  cs.blinkerLever = 2 if physical_override else 1
  atomic_json(helper.assistant_lane.root/'signal.json', {
    'id':helper.assistant_lane.signal_id, 'time':now, 'direction':direction, 'confirmed':False,
    'tap':{'id':'hold:1', 'direction':side, 'request':0., 'start':now, 'counts':[0, 0]}})
  assert tick() != log.LaneChangeState.laneChangeStarting
  assert result()['status'] == ('rejected' if physical_override else 'waiting')
