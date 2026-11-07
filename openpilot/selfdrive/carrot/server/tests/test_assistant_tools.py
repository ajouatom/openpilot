import secrets

import pytest

from openpilot.selfdrive.carrot.server.services.assistant_tools import AssistantTools


@pytest.fixture
def env(tmp_path):
  state = {'parked': True, 'now': 0, 'jobs': {}, 'starts': [], 'records': []}
  def start(body, guard):
    guard()
    state['starts'].append(body)
    identity = f'{len(state["starts"]):012x}'
    state['jobs'][identity] = {'status': 'running', 'result': None, 'guard': guard}
    return identity
  def make():
    return AssistantTools(tmp_path / 'tools.json', lambda: state['parked'], start,
                          state['jobs'].get, state['records'].append, lambda: state['now'])
  service = make()
  origin = 'http://192.168.0.1:7000'
  token = service.enable(origin)
  def execute(op, **fields):
    return service.execute(token, origin, {'id': secrets.token_urlsafe(32), 'operation': op, **fields})
  return service, state, execute, token, origin, make


def test_prepare_confirm_fixed_payload_once(env):
  service, state, execute, token, origin, _ = env
  prepared = execute('prepare', action='git_reset_hard', target='evil', cmd='rm -rf /')
  assert not state['starts'] and prepared['confirmation_required']
  command = {'id': 'c' * 43, 'operation': 'commit', 'proposal_id': prepared['proposal_id']}
  result = service.execute(token, origin, command)
  assert service.execute(token, origin, command) == result
  assert state['starts'] == [{'action': 'git_reset', 'mode': 'hard', 'target': 'HEAD'}]
  with pytest.raises(ValueError):
    execute('commit', proposal_id=prepared['proposal_id'])


@pytest.mark.parametrize('action', ['shell_cmd', 'git_checkout', 'git_remote_set', 'restore_settings', 'enable_ssh'])
def test_arbitrary_commands_and_sensitive_selection_are_not_executable(env, action):
  with pytest.raises(ValueError):
    env[2]('prepare', action=action)
  assert not env[1]['starts']


def test_scopes_origin_and_revocation(env):
  service, state, execute, token, origin, _ = env
  prepared = execute('prepare', action='reboot')
  for bad_token, bad_origin in [('wrong', origin), (token, 'http://evil')]:
    with pytest.raises(PermissionError):
      service.execute(bad_token, bad_origin, {'id': 'x' * 43, 'operation': 'commit', 'proposal_id': prepared['proposal_id']})
  other = service.enable(origin)
  with pytest.raises(ValueError):
    service.execute(other, origin, {'id': 'x' * 43, 'operation': 'commit', 'proposal_id': prepared['proposal_id']})
  service.revoke(token, origin)
  with pytest.raises(PermissionError):
    execute('commit', proposal_id=prepared['proposal_id'])
  assert not state['starts']


def test_expiry_correction_cancel_and_restart_invalidate_proposals(env):
  service, state, execute, token, origin, make = env
  first = execute('prepare', action='reboot')
  second = execute('prepare', action='backup_settings')
  with pytest.raises(ValueError):
    execute('commit', proposal_id=first['proposal_id'])
  state['now'] = 60
  with pytest.raises(ValueError):
    execute('commit', proposal_id=second['proposal_id'])
  third = execute('prepare', action='reboot')
  execute('cancel')
  with pytest.raises(ValueError):
    execute('commit', proposal_id=third['proposal_id'])
  fourth = execute('prepare', action='reboot')
  with pytest.raises(ValueError):
    make().execute(token, origin, {'id': 'c' * 43, 'operation': 'commit', 'proposal_id': fourth['proposal_id']})
  assert not state['starts']


def test_recheck_movement_and_permission_at_dispatch(env):
  service, state, execute, token, origin, _ = env
  proposal = execute('prepare', action='git_pull')
  state['parked'] = False
  with pytest.raises(ValueError):
    execute('commit', proposal_id=proposal['proposal_id'])
  assert not state['starts']
  state['parked'] = True
  job = execute('commit', proposal_id=proposal['proposal_id'])
  guard = state['jobs'][job['job_id']]['guard']
  state['parked'] = False
  with pytest.raises(ValueError):
    guard()
  state['parked'] = True
  service.revoke(token, origin)
  with pytest.raises(PermissionError):
    guard()


@pytest.mark.parametrize('action,expected', [('git_pull', 'completed'), ('reboot', 'requested'), ('server_tmux_log', 'requested')])
def test_truthful_private_job_status(env, action, expected):
  service, state, execute, token, origin, _ = env
  proposal = execute('prepare', action=action)
  started = execute('commit', proposal_id=proposal['proposal_id'])
  job_id = started['job_id']
  assert execute('status', job_id=job_id)['status'] == 'running'
  state['jobs'][job_id].update(status='done', result={'ok': True, 'out': 'PRIVATE DIAGNOSTICS'}, log='PRIVATE')
  result = execute('status', job_id=job_id)
  assert result == {'status': expected, 'action': action, 'job_id': job_id}
  other = service.enable(origin)
  with pytest.raises(ValueError):
    service.execute(other, origin, {'id': 'z' * 43, 'operation': 'status', 'job_id': job_id})
  del state['jobs'][job_id]
  assert execute('status', job_id=job_id)['status'] == 'unknown'


def test_recording_uses_explicit_target_and_reports_only_request(env):
  _, state, execute, *_ = env
  state['parked'] = False
  for action in ['record_start', 'record_start', 'record_stop']:
    p = execute('prepare', action=action)
    assert execute('commit', proposal_id=p['proposal_id'])['status'] == 'requested'
  assert state['records'] == [True, True, False]
  assert execute('open', action='restore_settings')['status'] == 'open_requested'
  assert not state['starts']
