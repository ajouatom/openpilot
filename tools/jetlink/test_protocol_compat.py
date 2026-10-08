from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import threading
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'third_party/jetlink'))

from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.spec import ModelSpec
from jetlink.transport.base import LinkError
from jetlink.transport.tcp import TcpTransport

REFERENCE_ROOT = Path(os.environ.get('JETLINK_REFERENCE_ROOT', ROOT.parent / 'jetlink-main')).resolve()

REFERENCE_V3_SESSION = r"""
import json
import socket
import sys

import numpy as np

sys.path.insert(0, sys.argv[1])
from jetlink import protocol as P
from jetlink.queues import PolicyQueues
from jetlink.spec import DRIVING_OUTPUT, ModelSpec
from jetlink.transport.tcp import TcpTransport

sock = socket.socket(fileno=int(sys.argv[2]))
spec = ModelSpec.from_dict(json.loads(sys.argv[3]))
t = TcpTransport(sock)
q = PolicyQueues(spec, dtype=np.float32)

hello = t.recv(timeout=5)
assert hello.msg_type == P.Msg.HELLO_REQ
client = json.loads(bytes(hello.payload))['client']
assert client['link'] == {'kind': 'usb', 'usb_speed': 'high-speed'}
t.send_json(P.Msg.HELLO_RESP, hello.seq, {'protocol': 3, 'device': 'reference'})

eng = t.recv(timeout=5)
assert eng.msg_type == P.Msg.ENGINE_REQ
engine_req = json.loads(bytes(eng.payload))
assert engine_req == {'sha256': spec.sha256, 'nbytes': spec.nbytes, 'frame_skip': spec.frame_skip}
t.send_json(P.Msg.ENGINE_RESP, eng.seq, {'state': 'ready', 'sha256': spec.sha256, 'spec': spec.to_dict()})

hidden = slice(*spec.hidden_range)
hidden_size = hidden.stop - hidden.start
expected_prev = np.zeros(spec.prev_feat_shape, np.float32)

for frame_id, reset_expected in ((101, False), (102, False), (103, True)):
  req = t.recv(timeout=5)
  assert req.msg_type == P.Msg.INFER_REQ
  wire_frame, flags = P.unpack_infer_req(req.payload)
  assert wire_frame == frame_id
  assert bool(flags & P.Flag.RESET_QUEUES) is reset_expected
  assert flags & P.Flag.WANT_HIDDEN
  assert req.payload.nbytes == P.INFER_REQ_SIZE + spec.warped_nbytes + 48
  if reset_expected:
    q.reset()
    expected_prev = np.zeros(spec.prev_feat_shape, np.float32)
  np.testing.assert_array_equal(q.prev_feat, expected_prev)
  off = P.INFER_REQ_SIZE
  warped = np.frombuffer(req.payload, np.uint8, spec.warped_nbytes, off).reshape(spec.warped_shape)
  assert warped.nbytes == 393216
  off += spec.warped_nbytes
  packed = np.frombuffer(req.payload, np.float32, 12, off)
  np.testing.assert_array_equal(packed, np.arange(1, 13, dtype=np.float32))
  q.step(warped, packed)
  out = np.arange(spec.output_nelem, dtype=np.float32)
  out[hidden] = frame_id
  q.after_run({DRIVING_OUTPUT: out})
  expected_prev = np.full(spec.prev_feat_shape, frame_id, np.float32)
  t.send(P.Msg.INFER_RESP, req.seq, (P.pack_infer_resp(frame_id, P.Status.OK, 1, 2, 3), out))

t.close()
"""


def _spec() -> ModelSpec:
  return ModelSpec(
    sha256='a' * 64,
    nbytes=393_216 + 48,
    frame_skip=4,
    input_shapes={
      'img': (1, 12, 128, 256),
      'big_img': (1, 12, 128, 256),
      'desire_pulse': (1, 33, 8),
      'traffic_convention': (1, 2),
      'action_t': (1, 2),
      'features_buffer': (1, 32, 32, 512),
    },
    output_shapes={'outputs': (1, 18452)},
    output_slices={'hidden_state': slice(2066, 18450)},
    checkpoint=None,
  )


def _inputs(spec: ModelSpec) -> tuple[np.ndarray, np.ndarray]:
  warped = np.zeros(spec.warped_shape, np.uint8)
  packed = np.zeros(spec.packed_nelem, np.float32)
  packed[:12] = np.arange(1, 13, dtype=np.float32)
  return warped, packed


def _reference_available() -> bool:
  return (REFERENCE_ROOT / 'jetlink/spec.py').is_file() and (REFERENCE_ROOT / 'jetlink/queues.py').is_file()


def test_protocol3_reference_session_uses_engine_handshake_and_queue_feedback():
  if not _reference_available():
    pytest.skip('latest Jetlink reference checkout is unavailable')

  ours, peer = socket.socketpair()
  spec = _spec()
  child = subprocess.Popen(
    [sys.executable, '-c', REFERENCE_V3_SESSION, str(REFERENCE_ROOT), str(peer.fileno()), json.dumps(spec.to_dict())],
    pass_fds=(peer.fileno(),), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
  peer.close()

  stream = TcpTransport(ours)
  stream.protocol_version = 3
  stream.link_info = lambda: {'kind': 'usb', 'usb_speed': 'high-speed'}

  class WrappedTransport:
    def __init__(self, inner):
      self.inner = inner

    def __getattr__(self, name):
      return getattr(self.inner, name)

  client = JetlinkClient(WrappedTransport(stream), deadline=5)
  warped, packed = _inputs(spec)
  try:
    hello = client.hello()
    assert hello['protocol'] == 3
    resolved = client.ensure_engine(spec.sha256, spec.nbytes, onnx_path=None, frame_skip=spec.frame_skip, build_timeout=5)
    assert resolved.packed_nbytes == spec.packed_nbytes
    hidden_start, hidden_stop = spec.output_slices['hidden_state'].start, spec.output_slices['hidden_state'].stop

    first = client.infer(warped, packed, frame_id=101)
    second = client.infer(warped, packed, frame_id=102)
    third = client.infer(warped, packed, frame_id=103, reset=True)

    assert first.nbytes == spec.output_nbytes
    assert second.nbytes == spec.output_nbytes
    assert third.nbytes == spec.output_nbytes
    assert np.all(first[hidden_start:hidden_stop] == 101)
    assert np.all(second[hidden_start:hidden_stop] == 102)
    assert np.all(third[hidden_start:hidden_stop] == 103)
    assert P.VERSION == 2
    stdout, stderr = child.communicate(timeout=10)
    assert child.returncode == 0, f'reference peer failed: {stdout}\n{stderr}'
  finally:
    client.close()
    if child.poll() is None:
      child.kill()
      child.communicate()


def test_v2_wire_keeps_legacy_packed_inputs_and_full_outputs():
  ours, peer = socket.socketpair()
  stream = TcpTransport(ours)
  server = TcpTransport(peer)
  spec = _spec()
  warped, packed = _inputs(spec)
  errors = []

  def serve():
    try:
      hello = server.recv(timeout=3)
      assert P.unpack_header(P.pack_header(P.Msg.PING, 0, 0))[1] == 2
      assert 'link' not in json.loads(bytes(hello.payload))['client']
      server.send_json(P.Msg.HELLO_RESP, hello.seq, {'protocol': 2})
      req = server.recv(timeout=3)
      assert req.msg_type == P.Msg.INFER_REQ
      frame_id, flags = P.unpack_infer_req(req.payload)
      assert not flags & P.Flag.WANT_HIDDEN
      assert req.payload.nbytes == P.INFER_REQ_SIZE + spec.warped_nbytes + spec.packed_nbytes
      packed_offset = P.INFER_REQ_SIZE + spec.warped_nbytes
      wire_packed = np.frombuffer(req.payload, np.float32, spec.packed_nelem, packed_offset)
      np.testing.assert_array_equal(wire_packed, packed)
      outputs = np.arange(spec.output_nelem, dtype=np.float32)
      response = P.pack_infer_resp(frame_id, P.Status.OK, 1, 2, 3) + outputs.tobytes()
      server.send(P.Msg.INFER_RESP, req.seq, (response,))
    except BaseException as e:
      errors.append(e)

  thread = threading.Thread(target=serve, daemon=True)
  thread.start()
  client = JetlinkClient(stream, deadline=3)
  client.spec = spec
  try:
    assert client.hello()['protocol'] == 2
    output = client.infer(warped, packed, frame_id=7)
    np.testing.assert_array_equal(output, np.arange(spec.output_nelem, dtype=np.float32))
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors
  assert P.VERSION == 2


@pytest.mark.parametrize(('version', 'payload_size'), [(2, 992), (3, 480)])
def test_short_packet_padding_uses_each_protocol_rule(version, payload_size):
  ours, peer = socket.socketpair()
  sender, receiver = TcpTransport(ours), TcpTransport(peer)
  sender.protocol_version = receiver.protocol_version = version
  try:
    sender.send(P.Msg.PING, 9, (bytes(payload_size),))
    msg = receiver.recv(timeout=2)
    assert msg.flags & P.Flag.PADDED
    assert msg.payload.nbytes == payload_size
    assert P.packet_multiple(version) == payload_size + P.HEADER_SIZE
  finally:
    sender.close()
    receiver.close()


def test_protocol_version_is_latched_after_first_message():
  ours, peer = socket.socketpair()
  sender, receiver = TcpTransport(ours), TcpTransport(peer)
  sender.protocol_version = receiver.protocol_version = 2
  try:
    sender.send(P.Msg.PING, 1)
    receiver.recv(timeout=1)
    sender.protocol_version = 2  # unchanged is allowed
    with pytest.raises(ValueError, match='locked to 2'):
      sender.protocol_version = 3
    receiver.protocol_version = 2
    with pytest.raises(ValueError, match='locked to 2'):
      receiver.protocol_version = 3
  finally:
    sender.close()
    receiver.close()


def test_unsupported_protocol_versions_are_rejected():
  ours, peer = socket.socketpair()
  stream = TcpTransport(ours)
  try:
    for version in (1, 4, True):
      with pytest.raises(ValueError):
        stream.protocol_version = version
    with pytest.raises(ValueError):
      P.pack_header(P.Msg.PING, 1, 0, version=4)
  finally:
    stream.close()
    peer.close()


def test_stream_metadata_reports_available_udc_speed(tmp_path, monkeypatch):
  from jetlink.transport import base

  speed_file = tmp_path / 'udc0' / 'current_speed'
  speed_file.parent.mkdir()
  speed_file.write_text('high-speed\n')
  read_speed = base.udc_speed
  monkeypatch.setattr(base, 'udc_speed', lambda udc=None: read_speed(udc, str(tmp_path)))

  ours, peer = socket.socketpair()
  stream = TcpTransport(ours)
  stream.bound_udc = 'udc0'
  try:
    assert stream.link_info() == {'kind': 'usb', 'usb_speed': 'high-speed'}
  finally:
    stream.close()
    peer.close()


def test_hello_rejects_protocol_type_that_is_not_int():
  ours, peer = socket.socketpair()
  client_stream, server_stream = TcpTransport(ours), TcpTransport(peer)
  client_stream.protocol_version = server_stream.protocol_version = 3

  def serve():
    req = server_stream.recv(timeout=2)
    server_stream.send_json(P.Msg.HELLO_RESP, req.seq, {'protocol': '3'})

  thread = threading.Thread(target=serve, daemon=True)
  thread.start()
  try:
    with pytest.raises(LinkError, match='invalid protocol value'):
      JetlinkClient(client_stream).hello(timeout=2)
  finally:
    client_stream.close()
    server_stream.close()
    thread.join(timeout=3)
  assert not thread.is_alive()


def test_hello_rejects_a_server_that_selected_another_protocol():
  ours, peer = socket.socketpair()
  client_stream, server_stream = TcpTransport(ours), TcpTransport(peer)
  client_stream.protocol_version = server_stream.protocol_version = 3

  def serve():
    req = server_stream.recv(timeout=2)
    server_stream.send_json(P.Msg.HELLO_RESP, req.seq, {'protocol': 2})

  thread = threading.Thread(target=serve, daemon=True)
  thread.start()
  try:
    with pytest.raises(LinkError, match='selected protocol 2'):
      JetlinkClient(client_stream).hello(timeout=2)
  finally:
    client_stream.close()
    server_stream.close()
    thread.join(timeout=3)
  assert not thread.is_alive()


@pytest.mark.parametrize('failure', ['wrong_frame', 'short_outputs', 'extra_bytes', 'non_finite'])
def test_protocol3_invalid_outputs_fail_closed(failure):
  ours, peer = socket.socketpair()
  client_stream, server_stream = TcpTransport(ours), TcpTransport(peer)
  client_stream.protocol_version = server_stream.protocol_version = 3
  spec = _spec()
  warped, packed = _inputs(spec)
  errors = []

  def serve():
    try:
      hello = server_stream.recv(timeout=3)
      server_stream.send_json(P.Msg.HELLO_RESP, hello.seq, {'protocol': 3})
      req = server_stream.recv(timeout=3)
      frame_id, _flags = P.unpack_infer_req(req.payload)
      response_id = frame_id + 1 if failure == 'wrong_frame' else frame_id
      outputs = np.zeros(spec.output_nelem, np.float32)
      if failure == 'non_finite':
        outputs[0] = np.nan
      payload = P.pack_infer_resp(response_id, P.Status.OK, 0, 0, 0) + outputs.tobytes()
      if failure == 'short_outputs':
        payload = payload[:-4]
      elif failure == 'extra_bytes':
        payload += b'x'
      server_stream.send(P.Msg.INFER_RESP, req.seq, (payload,))
    except BaseException as e:
      errors.append(e)

  thread = threading.Thread(target=serve, daemon=True)
  thread.start()
  client = JetlinkClient(client_stream, deadline=3)
  client.spec = spec
  try:
    client.hello(timeout=2)
    with pytest.raises(LinkError):
      client.infer(warped, packed, frame_id=55)
    assert client.dead
  finally:
    client.close()
    server_stream.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors
