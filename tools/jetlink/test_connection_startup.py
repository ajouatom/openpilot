import json
from types import SimpleNamespace as NS

import pytest

from openpilot.common import jetlink_status as status
from openpilot.selfdrive.modeld.jetlink import startup
from jetlink.transport.base import LinkError, LinkTimeout


@pytest.mark.parametrize('error', [TimeoutError('HELLO timeout'), LinkTimeout('HELLO timeout'),
                                 LinkError('gadget write had no reader for 3.000s'),
                                 LinkError('gadget write had no reader before deadline; link abandoned')])
@pytest.mark.parametrize('elapsed,ready,preparing,expected', [
  (0, False, False, True), (119.9, False, False, True),
  (120, False, False, False), (-1, False, False, False),
  (1, True, False, False), (1, False, True, False),
])
def test_only_bounded_first_handshake_timeout_is_waiting(error, elapsed, ready, preparing, expected):
  assert startup.initial_connection_wait(error, elapsed=elapsed, ready_once=ready, preparing=preparing) is expected


@pytest.mark.parametrize('error', [ValueError('signature'), LinkError('partial payload'), OSError('USB failure'),
                                 LinkError('hello response is not an object'), LinkError('server selected invalid protocol value 9'),
                                 RuntimeError('engine failed'), ConnectionError('disconnected')])
def test_real_failures_are_not_initial_waits(error):
  assert not startup.initial_connection_wait(error, elapsed=1, ready_once=False, preparing=False)


def test_next_cold_start_after_offroad_absence_gets_new_grace():
  cycle = startup.ConnectionStartup()
  cycle.begin(10)
  cycle.ready_once = True
  cycle.absent(offroad=True, onroad=False)
  cycle.begin(60000)
  assert cycle.waiting(LinkTimeout('HELLO'), now=60003, preparing=False)
  assert not cycle.waiting(LinkTimeout('HELLO'), now=60120, preparing=False)
  cycle.ready_once = True
  assert not cycle.waiting(LinkTimeout('lost session'), now=60010, preparing=False)


@pytest.mark.parametrize('offroad,onroad', [(False, True), (False, False), (True, True)])
def test_dropout_never_rearms_grace_without_confirmed_offroad(offroad, onroad):
  cycle = startup.ConnectionStartup()
  cycle.begin(10)
  cycle.ready_once = True
  cycle.absent(offroad=offroad, onroad=onroad)
  cycle.begin(20)
  assert not cycle.waiting(LinkTimeout('HELLO'), now=21, preparing=False)
  assert cycle.started == 10


def test_initial_onroad_detaches_cannot_extend_the_timeout_forever():
  cycle = startup.ConnectionStartup()
  cycle.begin(10)
  for now in (30, 90, 130):
    cycle.absent(offroad=False, onroad=True)
    cycle.begin(now)
  assert not cycle.waiting(LinkTimeout('HELLO'), now=130, preparing=False)


def test_new_cycle_still_reports_real_preparation_and_usb_errors():
  cycle = startup.ConnectionStartup()
  assert not cycle.waiting(LinkTimeout('HELLO'), now=1, preparing=False)
  cycle.absent(offroad=True, onroad=False)
  cycle.begin(1)
  assert not cycle.waiting(LinkTimeout('engine timeout'), now=2, preparing=True)
  assert not cycle.waiting(OSError('USB failure'), now=2, preparing=False)


def test_actual_boot_gate_handoff_is_expected_but_disconnect_is_not(monkeypatch):
  monkeypatch.setattr(startup, 'selected_release', lambda *a, **kw: {'source_commit': 'test'})
  peer = {'carrot_host': 'jetson', startup.CAPABILITY: True}
  calls = []
  client = NS(t=NS(send_json=lambda *args: calls.append(args)), _next_seq=lambda: 1,
              state=lambda: {'carrot_update': {'state': 'ready'}})
  with pytest.raises(startup.RuntimeHandoff):
    startup.wait_for_boot_update(client, peer, None, lambda: True, lambda *a, **kw: None)
  assert len(calls) == 1
  connected = iter([True, False])
  with pytest.raises(ConnectionError, match='disconnected') as error:
    startup.wait_for_boot_update(client, peer, None, lambda: next(connected), lambda *a, **kw: None)
  assert not isinstance(error.value, startup.RuntimeHandoff)


@pytest.fixture
def reports(tmp_path, monkeypatch):
  link, model = tmp_path / 'link', tmp_path / 'model'
  monkeypatch.setattr(status, 'LINK_STATUS', link)
  monkeypatch.setattr(status, 'MODEL_STATUS', model)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 20.)
  def write(state, *, error='', model_error='', active=False):
    link.write_text(json.dumps({'updated': 20., 'state': state, 'peer': {'carrot_host': 'jetson'},
                               'error': error, 'preparation': {'stage': 'waiting-host'}}))
    model.write_text(json.dumps({'updated': 20., 'active': active, 'error': model_error}))
  return write


def test_wait_and_model_preparation_are_progress_not_red(reports):
  reports('connecting', error='last unanswered initial HELLO')
  assert status.badge() == ('jetSON 연결 대기', 'loading')
  reports('loading')
  assert status.badge() == ('jetSON 모델 준비 중', 'loading')


@pytest.mark.parametrize('age', [3., 5., 7.9])
def test_bounded_blocking_hello_does_not_expire_as_an_error(reports, monkeypatch, age):
  reports('connecting')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 20. + age)
  assert status.badge() == ('jetSON 연결 대기', 'loading')


def test_stalled_handshake_eventually_becomes_an_error(reports, monkeypatch):
  reports('connecting')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 28.)
  assert status.badge() == ('jetSON ERROR', 'error')


@pytest.mark.parametrize('state', ['ready', 'loading', 'retrying', 'blocked'])
def test_other_link_states_keep_the_three_second_expiry(reports, monkeypatch, state):
  reports(state)
  monkeypatch.setattr(status.time, 'monotonic', lambda: 23.)
  assert status.badge() == ('jetSON ERROR', 'error')


def test_fresh_model_failure_overrides_extended_handshake_lease(reports, monkeypatch):
  reports('connecting')
  monkeypatch.setattr(status.time, 'monotonic', lambda: 25.)
  status.MODEL_STATUS.write_text(json.dumps({'updated': 25., 'active': False, 'error': 'inference timeout'}))
  assert status.badge() == ('jetSON ERROR', 'error')


@pytest.mark.parametrize('state,model_error', [
  ('blocked', ''), ('retrying', ''), ('connecting', 'inference failed'), ('loading', 'warp validation failed'),
])
def test_real_error_overrides_startup_progress(reports, state, model_error):
  reports(state, error='failed', model_error=model_error)
  assert status.badge() == ('jetSON ERROR', 'error')
