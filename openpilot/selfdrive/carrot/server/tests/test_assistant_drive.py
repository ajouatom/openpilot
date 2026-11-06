from pathlib import Path
import pytest

from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json
from openpilot.selfdrive.carrot.server.services.assistant_drive import AssistantDrive
from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import AssistantLane
from openpilot.selfdrive.carrot.assistant_speed import AssistantSpeed


@pytest.fixture
def system(tmp_path):
  now = [100.]
  service = AssistantDrive(tmp_path / 'grants.json', lambda: True, tmp_path / 'runtime', lambda: now[0])
  lane = AssistantLane(service.root, lambda: now[0])
  speed = AssistantSpeed(service.root / 'speed', lambda: now[0])
  token = service.enable('http://carrot:7000')
  state = {'valid': True, 'lateral_active': True, 'drive': True, 'trailer': False, 'speed': 20., 'phase': 'off',
           'driver_busy': False, 'driver_override': False, 'other_request': False,
           'left': {'available': True, 'blindspot': False}, 'right': {'available': True, 'blindspot': False}}
  return now, service, lane, speed, token, state


def send(system, **fields):
  _, service, _, _, token, _ = system
  return service.execute(token, 'http://carrot:7000', {'id': 'x' * 43, 'operation': 'execute', 'action': 'lane_left', **fields})


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_crossing_and_finished_state_confirm_completion(system, side):
  now, service, lane, _, _, state = system
  assert send(system, action='lane_' + side)['status'] == 'waiting'
  assert lane.before(state) == (side, False)
  lane.after('preLaneChange', side)
  assert send(system)['status'] == 'waiting'
  lane.after('laneChangeStarting', side, crossed=True)
  assert send(system)['status'] == 'running'
  lane.after('laneChangeFinishing', side, crossed=True)
  lane.after('off', 'none')
  assert send(system)['status'] == 'completed'
  assert lane.before(state) == (None, False)
  assert read_json(service.root / 'request.json')['time'] == now[0]


@pytest.mark.parametrize('field,value', [('valid', False), ('lateral_active', False), ('drive', False), ('trailer', True),
                                      ('speed', 0), ('speed', float('nan')), ('driver_busy', True), ('other_request', True),
                                      ('phase', 'laneChangeStarting')])
def test_rejected_intent_never_runs_after_condition_recovers(system, field, value):
  _, _, lane, _, _, state = system
  send(system)
  original = state[field]
  state[field] = value
  assert lane.before(state)[0] is None
  assert send(system)['status'] == 'rejected'
  state[field] = original
  assert lane.before(state) == (None, False)


@pytest.mark.parametrize('reason', ['blindspot', 'available'])
def test_obstacle_while_preparing_cancels_only_pending_intent(system, reason):
  _, _, lane, _, _, state = system
  send(system)
  assert lane.before(state)[0] == 'left'
  state['left'][reason] = reason == 'blindspot'
  assert lane.before(state) == (None, True)
  state['left'][reason] = reason != 'blindspot'
  assert lane.before(state) == (None, False)


def test_reset_without_crossing_is_not_success_and_running_is_not_forced_off(system):
  _, _, lane, _, _, state = system
  send(system)
  lane.before(state)
  lane.after('laneChangeStarting', 'left')
  lane.after('laneChangeFinishing', 'left')
  lane.after('off', 'none')
  assert send(system)['status'] == 'unverified'


@pytest.mark.parametrize('delay', [1.01, 50])
def test_stale_delivery_never_runs(system, delay):
  now, _, lane, _, _, state = system
  send(system)
  now[0] += delay
  assert lane.before(state) == (None, False)
  assert send(system)['status'] == 'expired'


def test_model_stall_and_revocation_cancel_pending_only(system):
  now, service, lane, _, token, state = system
  send(system)
  lane.before(state)
  now[0] += .4
  assert lane.before(state) == (None, True)
  now[0] += .1
  send(system, id='y' * 43)
  lane.before(state)
  service.revoke(token, 'http://carrot:7000')
  assert lane.before(state) == (None, True)
  with pytest.raises(PermissionError):
    send(system)


@pytest.mark.parametrize('action,value,target', [('speed_set', 100, 100), ('speed_down', 10, 70), ('speed_up', None, 81)])
def test_speed_value_readback_and_dedup(system, action, value, target):
  _, _, _, speed, _, _ = system
  send(system, action=action, value=value)
  command = speed.read(allowed=True, current=80, minimum=5, maximum=145)
  assert command['action'] == action
  if action != 'speed_up':
    assert command['target'] == target
  speed.finish(target)
  assert send(system)['current'] == target
  assert speed.read(allowed=True, current=target, minimum=5, maximum=145) is None


@pytest.mark.parametrize('allowed,target', [(False, 100), (True, 199), (True, 1)])
def test_speed_conditions_and_range_reject_without_changes(system, allowed, target):
  _, _, _, speed, _, _ = system
  send(system, action='speed_set', value=target)
  assert speed.read(allowed=allowed, current=80, minimum=5, maximum=145) is None
  assert send(system)['status'] == 'rejected'


def test_separate_permissions_ownership_and_one_request(system):
  _, service, _, _, token, _ = system
  send(system)
  with pytest.raises(ValueError):
    send(system, id='y' * 43, action='speed_up')
  with pytest.raises(PermissionError):
    service.execute(token, 'http://other:7000', {'id': 'x' * 43})
  another = service.enable('http://carrot:7000')
  with pytest.raises(PermissionError):
    service.execute(another, 'http://carrot:7000', {'id': 'x' * 43})
  restarted = AssistantDrive(Path(service.permissions.path), lambda: True, service.root, service.clock)
  with pytest.raises(ValueError):
    restarted.execute(token, 'http://carrot:7000', {'id': 'x' * 43, 'operation': 'status'})


def test_consumer_restart_and_cancel_never_replay(system):
  now, service, _, _, _, state = system
  send(system)
  now[0] += .01
  restarted = AssistantLane(service.root, lambda: now[0])
  assert restarted.before(state)[0] is None
  assert send(system)['status'] == 'expired'
  atomic_json(service.root / 'request.json', {'id': 'z' * 43, 'time': now[0], 'direction': 'right'})
  atomic_json(service.root / 'cancel.json', {'id': 'z' * 43})
  assert restarted.before(state) == (None, True)


def test_feedback_write_failure_cannot_restore_rejected_intent(system, monkeypatch):
  _, _, lane, speed, _, state = system
  def unavailable(*args, **kwargs):
    raise OSError('unavailable')
  send(system)
  monkeypatch.setattr('openpilot.selfdrive.controls.lib.desire_lib.assistant_lane.atomic_json', unavailable)
  state['left']['blindspot'] = True
  assert lane.before(state) == (None, False)
  assert lane.active is None
  state['left']['blindspot'] = False
  assert lane.before(state) == (None, False)
  atomic_json(speed.root / 'request.json', {'id': 'z' * 43, 'time': 100., 'action': 'speed_up'})
  monkeypatch.setattr('openpilot.selfdrive.carrot.assistant_speed.atomic_json', unavailable)
  assert speed.read(allowed=False, current=80, minimum=30, maximum=160) is None
  assert speed.active is None
  assert speed.read(allowed=True, current=80, minimum=30, maximum=160) is None


def test_revocation_after_web_restart_cancels_pending_intent(system):
  _, service, lane, _, token, state = system
  send(system)
  assert lane.before(state)[0] == 'left'
  restarted = AssistantDrive(service.permissions.path, lambda: True, service.root, service.clock)
  restarted.revoke(token, 'http://carrot:7000')
  assert lane.before(state) == (None, True)
  assert lane.active is None
