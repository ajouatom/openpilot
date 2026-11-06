"""Voice requests through the full production cruise helper, with no vehicle IO."""
import time

import pytest

from openpilot.selfdrive.car.tests.test_toyota_pcm_mode0 import full_helper, make_cs, SubMaster, step  # noqa: F401
from openpilot.selfdrive.carrot.assistant_speed import AssistantSpeed
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json


@pytest.fixture
def voice(full_helper, tmp_path):  # noqa: F811 - imported pytest fixture
  full_helper.assistant_speed = AssistantSpeed(tmp_path)
  full_helper._cruise_speed_min = 30
  full_helper._cruise_speed_max = 160
  return full_helper, tmp_path


def request(root, action, value=None):
  atomic_json(root / 'request.json', {'id': 'a' * 43, 'time': time.monotonic(), 'action': action, 'value': value})


def state(cs):
  sm = SubMaster(cs)
  sm.alive['carControl'] = sm.valid['carControl'] = True
  sm.recv_time['carControl'] = time.monotonic()
  return sm


@pytest.mark.parametrize(('action', 'value', 'expected'), [('speed_set', 100, 100), ('speed_down', 10, 73)])
def test_absolute_and_decrease_apply_once_without_engagement(voice, action, value, expected):
  helper, root = voice
  helper._lat_enabled = False
  cs = make_cs()
  request(root, action, value)
  sm = state(cs)
  assert step(helper, cs, sm) == expected
  assert not helper._lat_enabled
  assert read_json(root / 'result.json')['status'] == 'applied'
  assert step(helper, cs, sm) == expected  # The same file is never another press.


def test_speed_up_uses_remote_res_behavior_without_lateral_enable(voice):
  helper, root = voice
  cs = make_cs()
  helper._lat_enabled = False
  request(root, 'speed_up')
  actual = step(helper, cs, state(cs))
  assert not helper._lat_enabled
  result = read_json(root / 'result.json')
  helper.v_cruise_kph = helper.v_cruise_cluster_kph = 83
  helper.bluetooth_commands.action = 'accelCruise'
  remote = step(helper, cs, state(cs))
  assert actual == remote and actual > 83
  assert result['current'] == actual and result['status'] == 'applied'


@pytest.mark.parametrize('condition', ['stale', 'disabled', 'long_inactive', 'brake', 'gas', 'park', 'can_invalid', 'stock_long', 'pcm_follow'])
def test_unsafe_or_unavailable_state_rejects_and_never_runs_later(voice, condition):
  helper, root = voice
  cs = make_cs()
  sm = state(cs)
  if condition == 'stale':
    sm.recv_time['carControl'] -= 1
  elif condition == 'disabled':
    sm['carControl'].enabled = False
  elif condition == 'long_inactive':
    sm['carControl'].longActive = False
  elif condition == 'brake':
    cs.brakePressed = True
  elif condition == 'gas':
    cs.gasPressed = True
  elif condition == 'park':
    cs.gearShifter = 'park'
  elif condition == 'can_invalid':
    cs.canValid = False
  elif condition == 'stock_long':
    helper.CP.openpilotLongitudinalControl = False
  elif condition == 'pcm_follow':
    helper.params.values['SpeedFromPCM'] = 1
  request(root, 'speed_set', 100)
  step(helper, cs, sm)
  assert read_json(root / 'result.json')['status'] == 'rejected'
  cs = make_cs()
  helper.CP.openpilotLongitudinalControl = True
  helper.speed_from_pcm = 0
  helper.params.values['SpeedFromPCM'] = 0
  step(helper, cs, state(cs))
  assert helper.v_cruise_kph != 100
