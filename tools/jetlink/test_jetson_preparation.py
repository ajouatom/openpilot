"""Resident-engine waits retain the transport and preserve setup admission."""
import json
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.modeld.jetlink import compat, mac
from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.transport.base import LinkError, LinkTimeout
from test_v3 import MemoryStream, packet

SPEC = mac.JETSON_SPEC


def peer(loaded=None):
  return {'protocol': 2, 'backend': 'trt', 'device': 'Orin-sm87', 'loaded': loaded}


@pytest.mark.parametrize('version', [2, 3])
@pytest.mark.parametrize('loaded', [None, 'a' * 64])
def test_onroad_wait_uses_state_then_exact_engine_on_same_stream(monkeypatch, version, loaded):
  messages = [(P.Msg.STATE_RESP, 1, {'loaded': loaded, 'engine_state': 'none'}),
              (P.Msg.STATE_RESP, 2, {'loaded': SPEC.sha256, 'engine_state': 'none'}),
              (P.Msg.ENGINE_RESP, 3, {'state': 'ready', 'spec': SPEC.to_dict()})]
  stream = MemoryStream(version, b''.join(packet(version, kind, seq, json.dumps(body).encode())
                                         for kind, seq, body in messages))
  client = JetlinkClient(stream) if version == 2 else compat.V3Client(stream)
  sleeps, reports = [], []
  monkeypatch.setattr(mac.time, 'sleep', sleeps.append)
  remote = peer(loaded)
  result = mac.prepare(client, remote, lambda: False, lambda: True, lambda *args: reports.append(args))
  assert result == SPEC and client.spec == SPEC and remote['loaded'] == SPEC.sha256
  assert client.t is stream and not stream.closed and sleeps == [.5, .5]
  assert sum(stage == 'resident-wait' for stage, _, _ in reports) == 4
  requests = MemoryStream(version, bytes(stream.out))
  assert [requests.recv(1).msg_type for _ in range(3)] == [P.Msg.STATE_REQ, P.Msg.STATE_REQ, P.Msg.ENGINE_REQ]
  assert not requests.incoming  # No second HELLO, upload or close/reopen.
  assert client.progress_cb is None and client._should_stop() is False


def test_offroad_transition_starts_setup_without_reconnection(monkeypatch):
  allowed, calls = [False], []
  transport = object()
  def state():
    allowed[0] = True
    return {'loaded': None, 'engine_state': 'none'}
  def ensure(*args, **kwargs):
    assert allowed[0] and getattr(client.t, 'transport', client.t) is transport
    calls.append(args)
    return SPEC
  client = NS(t=transport, state=state, ensure_engine=ensure)
  monkeypatch.setattr(mac.time, 'sleep', lambda _: None)
  assert mac.prepare(client, peer(), lambda: allowed[0], lambda: True, lambda *a: None) == SPEC
  assert len(calls) == 1 and client.t is transport


@pytest.mark.parametrize('initially_loaded', [False, True])
def test_ignition_or_stale_hello_deferral_keeps_session(monkeypatch, initially_loaded):
  allowed, calls, polls = [not initially_loaded], [], []
  transport = object()
  def state():
    polls.append(True)
    return {'loaded': SPEC.sha256, 'engine_state': 'ready'}
  def ensure(*args, **kwargs):
    calls.append(args)
    assert getattr(client.t, 'transport', client.t) is transport
    if len(calls) == 1:
      allowed[0] = False
      kwargs['should_stop']()
      pytest.fail('Pending build continued onroad')
    return SPEC
  client = NS(t=transport, state=state, ensure_engine=ensure)
  monkeypatch.setattr(mac.time, 'sleep', lambda _: None)
  assert mac.prepare(client, peer(SPEC.sha256 if initially_loaded else None),
                     lambda: allowed[0], lambda: True, lambda *a: None) == SPEC
  assert len(calls) == 2 and polls == [True] and client.t is transport
  assert client.progress_cb is None and client._should_stop() is False


@pytest.mark.parametrize('failure', ['timeout', 'failed', 'malformed', 'detached'])
def test_wait_preserves_real_failure_recovery(monkeypatch, failure):
  def state():
    if failure == 'timeout':
      raise LinkTimeout('no STATE response')
    if failure == 'failed':
      return {'loaded': None, 'engine_state': 'failed', 'detail': 'engine rejected'}
    return {}
  client = NS(t=object(), state=state, ensure_engine=lambda *a, **kw: pytest.fail('not ready'))
  monkeypatch.setattr(mac.time, 'sleep', lambda _: None)
  expected = mac.PreparationDeferred if failure == 'detached' else LinkError
  with pytest.raises(expected):
    mac.prepare(client, peer(), lambda: False, lambda: failure != 'detached', lambda *a: None)


def test_build_over_thirty_seconds_keeps_heartbeat_and_transport(monkeypatch):
  clock, reports = [0.], []
  class Transport:
    calls = 0
    def send_json(self, kind, seq, body):
      assert kind == P.Msg.ENGINE_REQ and seq == 1
    def recv(self, timeout):
      self.calls += 1
      if self.calls == 1:
        body, seq = {'state': 'building', 'sha256': SPEC.sha256}, 1
      elif clock[0] < 40.:
        clock[0] += timeout
        raise LinkTimeout('build still running')
      else:
        body, seq = {'state': 'ready', 'sha256': SPEC.sha256, 'spec': SPEC.to_dict()}, 0
      return NS(msg_type=P.Msg.ENGINE_RESP, seq=seq, payload=json.dumps(body).encode())
  transport = Transport()
  client = JetlinkClient(transport)
  monkeypatch.setattr(mac.time, 'monotonic', lambda: clock[0])
  assert mac.prepare(client, peer(), lambda: True, lambda: True,
                     lambda *args: reports.append(clock[0])) == SPEC
  assert clock[0] >= 40. and client.t is transport
  assert max(b - a for a, b in zip(reports, reports[1:])) <= 1.
  assert client.progress_cb is None and client._should_stop() is False


def test_build_failure_restores_transport_and_callbacks():
  stream = MemoryStream(2, packet(2, P.Msg.ENGINE_RESP, 1,
                                json.dumps({'state': 'failed', 'detail': 'bad engine'}).encode()))
  client = JetlinkClient(stream)
  with pytest.raises(LinkError, match='bad engine'):
    mac.prepare(client, peer(), lambda: True, lambda: True, lambda *args: None)
  assert client.t is stream and client.progress_cb is None and client._should_stop() is False


def test_slow_state_replies_refresh_throttled_status_before_three_second_expiry(monkeypatch):
  clock, reports, polls = [0.], [], []
  def progress(*args):
    if not reports or clock[0] - reports[-1] >= 1.:
      reports.append(clock[0])
  def state():
    polls.append(True)
    clock[0] += 2. if len(polls) % 2 == 0 else 0.
    return {'loaded': SPEC.sha256 if len(polls) == 6 else None, 'engine_state': 'none'}
  def sleep(duration):
    clock[0] += duration
  monkeypatch.setattr(mac.time, 'sleep', sleep)
  client = NS(t=object(), state=state, ensure_engine=lambda *a, **kw: SPEC)
  assert mac.prepare(client, peer(), lambda: False, lambda: True, progress) == SPEC
  assert len(polls) == 6 and max(b - a for a, b in zip(reports, reports[1:])) < 3.
