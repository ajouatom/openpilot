"""Negotiated N16 replies: real host/client/IPC/parser, deterministic CPU engine."""
from collections import deque
from dataclasses import replace
import json
import socket
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_nav_rl import SPEC, Parser, parse_outputs
from jetlink import output_projection as C, protocol as P
from jetlink.client import JetlinkClient
from jetlink.queues import StatefulQueues, DeviceStateQueues
from jetlink.server.session import Session
from jetlink.transport.base import Transport, Message, LinkError
from openpilot.selfdrive.modeld.jetlink import link


class Loopback(Transport):
  protocol_version = 2

  def __init__(self):
    self.replies = deque()
    self.requests = []
    self.handler = None

  def send(self, kind, seq, parts=(), flags=0, timeout=None):
    payload = bytearray(b''.join(memoryview(p).cast('B') for p in parts))
    message = Message(kind, seq, flags, memoryview(payload))
    if kind in (P.Msg.HELLO_REQ, P.Msg.INFER_REQ):
      self.requests.append(message)
      self.handler(message)
    else:
      self.replies.append(message)

  def recv(self, timeout=None):
    return self.replies.popleft()

  def close(self): pass


def pair():
  transport = Loopback()
  host = NS(telemetry=NS(read=lambda: {'temperature': 42}), backend=NS(describe=lambda: {}),
            status=lambda *a: {'state': 'ready'}, loaded_sha=lambda: SPEC.sha256,
            cache=NS(inventory=lambda: []), sleep_after=0, frame_stats=NS(record=lambda *a: None))
  session = Session(transport, host)
  inputs = {k: np.zeros(v, np.float32) for k, v in SPEC.input_shapes.items()}
  outputs = {k: np.zeros(v, np.float32) for k, v in SPEC.output_shapes.items()}
  queues = StatefulQueues(SPEC, inputs)
  loaded = NS(spec=SPEC, host_inputs=inputs, queues=queues,
              engine=NS(run=lambda: outputs, last_gpu_us=24000))
  def handle(message):
    if message.msg_type == P.Msg.HELLO_REQ:
      session._greet(message)
      session.on_hello(message)
    else:
      session._infer(loaded, message)
  transport.handler = handle
  client = JetlinkClient(transport, allow_output_projection=True)
  client.hello()
  client.spec = SPEC
  return client, session, loaded, outputs


def infer(client, frame=1, reset=False, state=False):
  return client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32),
                      frame, reset=reset, want_state=state)


def test_full_compact_full_preserves_outputs_and_all_feedback():
  client, _, loaded, outputs = pair()
  assert client.output_projection_available
  rng = np.random.default_rng(361)
  for frame in range(12):
    client.project_output = frame % 3 == 1
    for value in outputs.values():
      value[...] = rng.uniform(-1, 1, value.shape)
    result = infer(client, frame, reset=frame == 0, state=True)
    count = C.COUNT if client.project_output else SPEC.output_nelem
    assert result.tobytes() == outputs['outputs'].reshape(-1)[:count].tobytes()
    assert client.last_state == {'temperature': 42}
    for name, output in SPEC.state_pairs.items():
      assert loaded.host_inputs[name].tobytes() == outputs[output].tobytes()
    # Reusing the engine output must not mutate the returned response.
    saved = result.tobytes()
    outputs['outputs'].fill(0)
    assert result.tobytes() == saved


@pytest.mark.parametrize('kind', ['omitted_nan', 'state_nan', 'shape'])
def test_projection_never_hides_invalid_full_output_or_feedback(kind):
  client, _, loaded, outputs = pair()
  client.project_output = True
  if kind == 'omitted_nan':
    outputs['outputs'].reshape(-1)[C.COUNT + 4] = np.nan
  elif kind == 'state_nan':
    outputs[next(iter(SPEC.state_pairs.values()))].reshape(-1)[0] = np.nan
  else:
    outputs['outputs'] = np.zeros((1, C.COUNT), np.float32)
  with pytest.raises((LinkError, ValueError)):
    infer(client)
  assert not any(np.any(v) for v in loaded.queues.state.values())


@pytest.mark.parametrize('change', ['model', 'layout', 'legacy'])
def test_exact_model_and_layout_pin(change):
  assert C.eligible(SPEC)
  spec = (replace(SPEC, sha256='0' * 64) if change == 'model' else
          replace(SPEC, output_slices={**SPEC.output_slices, 'plan': slice(1, 496)}) if change == 'layout' else link.SPEC)
  assert not C.eligible(spec)


@pytest.mark.parametrize('bad', [None, True, {}, {**C.descriptor(), 'schema': True},
                                {**C.descriptor(), 'count': 2066.0}, {**C.descriptor(), 'count': 2065}])
def test_no_negotiation_for_partial_or_ambiguous_capabilities(bad):
  assert not C.matches(bad)


def test_new_host_old_client_and_rehello_do_not_trim():
  client, session, _, _ = pair()
  old = JetlinkClient(client.t)
  response = old.hello()
  old.spec = SPEC
  assert C.CAPABILITY not in response and not session._projection_negotiated
  assert not old.output_projection_available
  assert infer(old).size == C.FULL_COUNT
  old.project_output = True
  with pytest.raises(LinkError, match='not negotiated'):
    infer(old)


@pytest.mark.parametrize('version', [2, 3])
def test_old_host_and_v3_use_full_contract(version):
  transport = Loopback()
  transport.protocol_version = version
  transport.handler = lambda msg: transport.send(P.Msg.HELLO_RESP, msg.seq, [json.dumps({
    'protocol': version, 'stateful_v2': True}).encode()])
  client = JetlinkClient(transport, allow_output_projection=True)
  client.hello()
  client.spec = SPEC
  assert not client.output_projection_available and not client.project_output
  offer = json.loads(bytes(transport.requests[0].payload))['client']
  assert (C.CAPABILITY in offer) == (version == 2)


def test_host_rejects_unnegotiated_projection_before_queue_reset():
  client, session, loaded, _ = pair()
  session._projection_negotiated = False
  for value in loaded.queues.state.values(): value.fill(1)
  client.project_output = True
  with pytest.raises(LinkError, match='BAD_SHAPE'):
    infer(client, reset=True)
  assert all(np.all(v == 1) for v in loaded.queues.state.values())


@pytest.mark.parametrize('fault', ['marker', 'short', 'long', 'frame', 'nan', 'flags', 'telemetry'])
def test_compact_receiver_rejects_corrupt_responses(fault):
  client, _, _, _ = pair()
  client.project_output = True
  seq = client.infer_begin(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 17,
                           want_state=fault == 'telemetry')
  msg = client.t.replies[0]
  body = bytearray(msg.payload)
  if fault == 'marker': msg.flags = 0
  if fault == 'flags': msg.flags |= 64
  if fault == 'short': body = body[:-4]
  if fault == 'long': body += b'abcd'
  if fault == 'frame': body[:P.INFER_RESP_SIZE] = P.pack_infer_resp(18, P.Status.OK, 0, 0, 0)
  if fault == 'nan': np.frombuffer(body, np.float32, C.COUNT, P.INFER_RESP_SIZE)[0] = np.nan
  if fault == 'telemetry': body[-1] = ord('!')
  msg.payload = memoryview(body)
  with pytest.raises(LinkError): client.infer_end(seq)
  assert client.dead


@pytest.mark.parametrize('capable,raw', [(False, False), (True, False), (True, True)])
@pytest.mark.parametrize("enabled", [None, "0", "1"])
def test_local_ipc_negotiates_without_changing_legacy_requests(monkeypatch, capable, raw, enabled):
  if enabled is None: monkeypatch.delenv("JETLINK_COMPACT_REPLY", raising=False)
  else: monkeypatch.setenv("JETLINK_COMPACT_REPLY", enabled)
  if raw: monkeypatch.setenv('SEND_RAW_PRED', '1')
  else: monkeypatch.delenv('SEND_RAW_PRED', raising=False)
  hello = {'spec': SPEC.to_dict()}
  if capable: hello[C.IPC_CAPABILITY] = C.descriptor()
  sock = NS(settimeout=lambda *a: None, connect=lambda *a: None, close=lambda: None)
  # This test replaces the socket; no native Unix socket is opened on Windows.
  monkeypatch.setattr(link.socket, 'AF_UNIX', 1, raising=False)
  monkeypatch.setattr(link.socket, 'socket', lambda *a: sock)
  monkeypatch.setattr(link, 'receive', lambda *a: json.dumps(hello).encode())
  client = link.Client()
  compact = capable and not raw and enabled != "0"
  assert client.project_output == compact
  count = C.COUNT if compact else C.FULL_COUNT
  payload = bytearray(link.REPLY.pack(7, 24, 1, 25) + np.arange(count, dtype=np.float32).tobytes())
  client.reader = NS(receive=lambda *a: memoryview(payload))
  requests = []
  monkeypatch.setattr(link, 'send_parts', lambda sock, hdr, *a, **kw: requests.append(link.REQUEST.unpack(hdr)))
  result = client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 7, True)
  assert requests == [(7, 3 if compact else 1, 0)]
  assert result.tobytes() == payload[link.REPLY.size:]
  payload[link.REPLY.size:] = bytes(count * 4)
  assert result[-1] == count - 1  # Own the response before reader-buffer reuse.


def test_parser_all_retained_outputs_are_bitwise_equal():
  rng = np.random.default_rng(11)
  for _ in range(64):
    full = rng.uniform(-1, 1, SPEC.output_nelem).astype(np.float32)
    a = parse_outputs(Parser(), SPEC, full.copy())
    b = parse_outputs(Parser(), SPEC, full[:C.COUNT].copy())
    assert set(b) == set(a) - C.OMITTED
    for name in b:
      assert a[name].shape == b[name].shape
      assert a[name].tobytes() == b[name].tobytes(), name


@pytest.mark.parametrize('fault', [None, 'omitted_nan', 'device_state'])
def test_projection_keeps_gpu_state_validation_and_failure_reset(fault):
  client, _, loaded, outputs = pair()
  calls = []
  def validate():
    calls.append('validate')
    if fault == 'device_state':
      raise FloatingPointError('nonfinite GPU state')
  device = NS(input_names=set(SPEC.state_pairs), output_names=set(SPEC.state_pairs.values()),
              reset=lambda: calls.append('reset'), validate=validate)
  loaded.queues = DeviceStateQueues(SPEC, loaded.host_inputs, device)
  driving = outputs['outputs']
  outputs.clear()
  outputs['outputs'] = driving
  if fault == 'omitted_nan': driving.reshape(-1)[C.COUNT + 7] = np.nan
  calls.clear()
  client.project_output = True
  if fault:
    with pytest.raises(LinkError, match='NOT_FINITE'):
      infer(client)
    assert calls == (['reset'] if fault == 'omitted_nan' else ['validate', 'reset'])
  else:
    assert infer(client).size == C.COUNT
    assert calls == ['validate']


@pytest.mark.parametrize('compact', [False, True])
@pytest.mark.parametrize('fault', ['short', 'long', 'frame', 'nan', 'wrong_contract'])
def test_local_ipc_rejects_malformed_output(monkeypatch, compact, fault):
  monkeypatch.delenv('SEND_RAW_PRED', raising=False)
  monkeypatch.setattr(link.socket, 'AF_UNIX', 1, raising=False)
  monkeypatch.setattr(link.socket, 'socket', lambda *a: NS(settimeout=lambda *a: None, connect=lambda *a: None, close=lambda: None))
  hello = {'spec': SPEC.to_dict()}
  if compact: hello[C.IPC_CAPABILITY] = C.descriptor()
  monkeypatch.setattr(link, 'receive', lambda *a: json.dumps(hello).encode())
  monkeypatch.setattr(link, 'send_parts', lambda *a, **kw: None)
  client = link.Client()
  count = C.COUNT if compact else C.FULL_COUNT
  if fault == 'wrong_contract': count = C.FULL_COUNT if compact else C.COUNT
  values = np.zeros(count, np.float32)
  if fault == 'nan': values[0] = np.nan
  payload = link.REPLY.pack(8 if fault == 'frame' else 7, 24, 1, 25) + values.tobytes()
  if fault == 'short': payload = payload[:-4]
  if fault == 'long': payload += bytes(4)
  client.reader = NS(receive=lambda *a: memoryview(payload))
  with pytest.raises(ValueError):
    client.infer(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 7)


@pytest.mark.skipif(not hasattr(socket.socket, 'sendmsg'), reason='native POSIX socket framing')
def test_full_compact_switches_use_real_stream_framing():
  from jetlink.transport.tcp import TcpTransport
  _, original, loaded, outputs = pair()
  outputs['outputs'].reshape(-1)[:] = np.arange(C.FULL_COUNT, dtype=np.float32)
  ours, peer = socket.socketpair()
  server = TcpTransport(peer)
  session = Session(server, original.host)
  errors = []
  def serve():
    try:
      hello = server.recv(timeout=3)
      session._greet(hello)
      session.on_hello(hello)
      for _ in range(4): session._infer(loaded, server.recv(timeout=3))
    except BaseException as exc:
      errors.append(exc)
    finally:
      server.close()
  thread = threading.Thread(target=serve, daemon=True)
  thread.start()
  client = JetlinkClient(TcpTransport(ours), deadline=3, allow_output_projection=True)
  try:
    client.hello()
    client.spec = SPEC
    for frame, compact in enumerate((False, True, True, False)):
      client.project_output = compact
      result = infer(client, frame=frame, reset=frame == 0, state=True)
      count = C.COUNT if compact else C.FULL_COUNT
      assert result.tobytes() == np.arange(count, dtype=np.float32).tobytes()
      assert client.last_state == {'temperature': 42}
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


def test_future_full_2066_model_keeps_the_original_parser_contract():
  # 개수가 같아도 다른 정상 모델의 전체 출력을 N16 축소 응답으로 오인하면 안 된다.
  from openpilot.selfdrive.modeld.jetlink.contracts import validate_model_spec
  future = replace(SPEC, sha256='3' * 64,
                   output_shapes={**SPEC.output_shapes, 'outputs': (1, C.COUNT)},
                   output_slices={name: at for name, at in SPEC.output_slices.items() if name not in C.OMITTED})
  assert validate_model_spec(future) is future and not C.eligible(future)
  full = np.linspace(-1, 1, C.COUNT, dtype=np.float32)
  expected = Parser().parse_outputs({name: full[None, at] for name, at in future.output_slices.items()})
  actual = parse_outputs(Parser(), future, full)
  assert actual.keys() == expected.keys()
  for name in actual:
    assert actual[name].shape == expected[name].shape
    assert actual[name].tobytes() == expected[name].tobytes(), name
