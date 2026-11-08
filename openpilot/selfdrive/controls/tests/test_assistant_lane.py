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


@pytest.fixture(params=['HYUNDAI_IONIQ_5_PE', 'TEST_OTHER_PLATFORM'])
def signal_vehicle(voice, request):
  helper = voice[0]
  helper.boundary_indicator_supported = request.param == 'HYUNDAI_IONIQ_5_PE'
  helper.assistant_lane.fingerprint = request.param
  atomic_json(helper.assistant_lane.root/'indicator-device.json',
              {'fingerprint':request.param, 'route':'v'*43, 'time':helper.assistant_lane.clock()})
  return voice


@pytest.mark.parametrize('direction', ['left', 'right'])
def test_adapter_signal_and_lane_change_start_together(signal_vehicle, direction, monkeypatch):
  from pathlib import Path
  monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[4] / 'tools/jetlink'))
  from blink_voice import VoiceLaneSignal
  from blink_policy import Policy

  helper, cs, tick, request, result = signal_vehicle
  policy, lamp = Policy(), VoiceLaneSignal()
  atomic_json(helper.assistant_lane.root/'indicator-device.json',
              {'fingerprint':helper.assistant_lane.fingerprint, 'route':lamp.route, 'time':helper.assistant_lane.clock()})
  dark = SimpleNamespace(leftBlinker=False, rightBlinker=False)
  now = helper.assistant_lane.clock
  def update_lamp(meta, active_id=None):
    intent = lamp.select(meta, cs, allowed=True, peer_ready=True, hold=policy.hold, busy=False)
    _, hold = policy.update(now(), 2, True, [], 25., 0., 'drive', False, intent)
    lamp.bind(hold)
    if lamp.confirmed(hold, active_id, {'accepted_id': active_id}, cs):
      atomic_json(helper.assistant_lane.root/'signal.json', {'id':lamp.request, 'route':lamp.route, 'direction':lamp.direction, 'time':now(), 'confirmed':True})
    return hold

  tick()
  update_lamp(SimpleNamespace(assistantLaneSignalId='', laneChangeDirection='none', laneChangeState='off'))
  request(direction)
  assert tick() == log.LaneChangeState.preLaneChange
  assert not helper.assistant_lane.signal_id  # delay, torque and classifier gates run before requesting a lamp
  assert tick() == log.LaneChangeState.laneChangeStarting
  identity = helper.assistant_lane.signal_id
  assert identity == 'a' * 43
  meta = SimpleNamespace(assistantLaneSignalId=identity, assistantLaneSignalRoute=lamp.route, laneChangeDirection=direction, laneChangeState='laneChangeStarting')
  hold = update_lamp(meta)
  assert hold['direction'] == direction and hold['source'] == 'lane'
  assert not lamp.confirmed(hold, hold['id'], {'accepted_id':hold['id']}, dark)
  assert tick() == log.LaneChangeState.laneChangeStarting
  setattr(cs, direction + 'Blinker', True)
  # Reproduce the recorded S3 behavior: the lamp lights but the tap never
  # receives its completion ACK. Lamp feedback cannot start a second maneuver.
  update_lamp(meta)
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


@pytest.mark.parametrize('fault', ['bsd', 'lane', 'torque', 'opposite', 'hazards', 'cancel'])
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_adapter_voice_does_not_start_under_blocked_conditions(signal_vehicle, direction, fault):
  helper, cs, tick, request, result = signal_vehicle
  tick()
  if fault == 'torque':
    helper.laneChangeNeedTorque = 1
  if fault == 'lane':
    getattr(helper, direction).lane_change_available_geom = False
  request(direction)
  tick()
  assert not helper.assistant_lane.signal_id
  if fault == 'bsd':
    setattr(cs, direction+'Blindspot', True)
  if fault in ('opposite','hazards'):
    setattr(cs, ('right' if direction == 'left' else 'left')+'Blinker', True)
    if fault == 'hazards':
      setattr(cs, direction+'Blinker', True)
  if fault == 'cancel':
    atomic_json(helper.assistant_lane.root/'cancel.json', {'id':'a'*43})
  for _ in range(66):
    tick()
    assert result()['status'] != 'running'
  assert not helper.assistant_lane.signal_id
  assert result()['status'] in ('rejected', 'expired', 'cancelled')


@pytest.mark.parametrize('direction', ['left', 'right'])
@pytest.mark.parametrize('physical_override', [False, True])
def test_adapter_tap_echo_requires_this_request_and_never_confirms_lamp(signal_vehicle, direction, physical_override):
  helper, cs, tick, request, result = signal_vehicle
  tick()
  request(direction)
  tick()
  tick()
  now = helper.assistant_lane.clock()
  side = 1 if direction == 'left' else 2
  setattr(cs, direction+'BlinkerStalkCount', 1)
  cs.blinkerLever = 2 if physical_override else 1
  atomic_json(helper.assistant_lane.root/'signal.json', {
    'id':helper.assistant_lane.signal_id, 'route':'v'*43, 'time':now, 'direction':direction, 'confirmed':False,
    'tap':{'id':'hold:1', 'direction':side, 'request':0., 'start':now, 'counts':[0, 0]}})
  tick()
  assert result()['status'] == ('interrupted' if physical_override else 'running')


@pytest.mark.parametrize('fingerprint', ['HYUNDAI_IONIQ_5_PE', 'TEST_OTHER_PLATFORM'])
@pytest.mark.parametrize('fault', ['absent', 'stale', 'different_vehicle'])
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_optional_indicator_keeps_normal_lane_change(voice, fingerprint, fault, direction):
  helper, cs, tick, request, result = voice
  helper.assistant_lane.fingerprint = fingerprint
  helper.boundary_indicator_supported = fingerprint == 'HYUNDAI_IONIQ_5_PE'
  if fault != 'absent':
    atomic_json(helper.assistant_lane.root/'indicator-device.json', {
      'fingerprint':fingerprint if fault == 'stale' else 'another vehicle', 'route':'v'*43,
      'time':helper.assistant_lane.clock() - (1 if fault == 'stale' else 0)})
  tick()
  request(direction)
  assert tick() == log.LaneChangeState.preLaneChange
  assert helper.assistant_lane.signal_route == 'ordinary'
  assert tick() == log.LaneChangeState.laneChangeStarting
  assert not helper.assistant_lane.signal_id


def test_selected_adapter_is_not_replaced_if_it_disconnects(signal_vehicle):
  helper, cs, tick, request, result = signal_vehicle
  tick()
  request('right')
  tick()
  (helper.assistant_lane.root/'indicator-device.json').unlink()
  cs.rightBlinker = False
  assert tick() == log.LaneChangeState.laneChangeStarting
  assert helper.assistant_lane.signal_route == 'v'*43
  assert result()['status'] == 'running'

@pytest.mark.parametrize('output', ['EA_02', 'SPAS2'])
@pytest.mark.parametrize('direction', ['left', 'right'])
def test_native_signal_needs_transmitted_command_and_physical_lamp(voice, output, direction):
  from opendbc.can.packer import CANPacker
  from openpilot.selfdrive.car.voice_indicator import NativeVoiceIndicator

  helper, cs, tick, request, result = voice
  lane = helper.assistant_lane
  lane.native_signal = True
  driver = NativeVoiceIndicator(output, lane.root)
  now = lane.clock
  tick()
  request(direction)
  tick()
  tick()
  assert lane.signal_route == 'native' and lane.signal_id
  left = direction == 'left'
  cc = SimpleNamespace(assistantLaneSignalId=lane.signal_id, latActive=True, leftBlinker=left, rightBlinker=not left)
  body = SimpleNamespace(left_blinker_active=False, right_blinker_active=False)
  packer = CANPacker('vw_meb' if output == 'EA_02' else 'hyundai_canfd_generated')
  field, value = ('EA_Blinken', 1 if left else 2) if output == 'EA_02' else ('BLINKER_CONTROL', 3 if left else 4)
  frame = packer.make_can_msg(output, 0, {field:value})
  wrong = packer.make_can_msg(output, 0, {field:(3-value if output == 'EA_02' else 7-value)})
  def lamps(on):
    setattr(body, direction + '_blinker_active', on)
    if output == 'SPAS2':
      setattr(cs, direction + 'Blinker', on)
  driver.update(now(), cc, cs, body, [wrong])
  lamps(True)
  driver.update(now(), cc, cs, body, [])
  assert tick() == log.LaneChangeState.preLaneChange
  lamps(False)
  driver.update(now(), cc, cs, body, [frame])
  assert tick() == log.LaneChangeState.preLaneChange
  lamps(True)
  driver.update(now(), cc, cs, body, [])
  assert tick() == log.LaneChangeState.laneChangeStarting
  assert result()['status'] == 'running'


def test_native_indicator_requires_enabled_vehicle_interface():
  from opendbc.car.hyundai.values import HyundaiFlags
  from opendbc.car.volkswagen.values import VolkswagenFlags
  from openpilot.selfdrive.car.voice_indicator import native_indicator

  cp = SimpleNamespace(brand='volkswagen', flags=0, passive=False)
  assert native_indicator(cp) is None
  cp.flags = VolkswagenFlags.STOCK_EA_PRESENT
  assert native_indicator(cp) == 'EA_02'
  cp.brand, cp.flags = 'hyundai', HyundaiFlags.CANFD | HyundaiFlags.CANFD_HDA2
  assert native_indicator(cp) is None
  cp.flags |= HyundaiFlags.ENABLE_BLINKERS
  assert native_indicator(cp) == 'SPAS2'
  cp.passive = True
  assert native_indicator(cp) is None


@pytest.mark.parametrize('route,armed,active,sends', [
  ('', False, True, True), ('ordinary', False, True, True), ('v'*43, True, True, False),
  ('native', False, True, False), ('native', True, False, False), ('native', True, True, True)])
def test_existing_native_blinker_output_is_bound_only_for_native_voice(route, armed, active, sends):
  from openpilot.selfdrive.car.voice_indicator import set_lane_change_blinkers
  meta = log.ModelDataV2.new_message().meta
  meta.laneChangeState = 'preLaneChange'
  meta.laneChangeDirection = 'right'
  meta.assistantLaneSignalRoute = route
  meta.assistantLaneSignalId = 'n'*43 if armed else ''
  cc = car.CarControl.new_message()
  cc.latActive = active
  set_lane_change_blinkers(cc, meta)
  assert cc.rightBlinker == sends and not cc.leftBlinker
  assert cc.assistantLaneSignalId == ('n'*43 if route == 'native' and sends else '')
