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
    helper.update(cs, SimpleNamespace(), True, .5, base.carrot_man, SimpleNamespace(), assistant_valid=kwargs.get('valid', True))
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
