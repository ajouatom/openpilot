from __future__ import annotations

import json
import socket
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
from jetlink.transport.base import LinkError, LinkTimeout
from jetlink.transport.tcp import TcpTransport


def _stateful_spec() -> ModelSpec:
  return ModelSpec(
    sha256='b' * 64,
    nbytes=1024,
    frame_skip=4,
    input_shapes={
      'new_img': (2, 6, 128, 256),
      'state_img': (1, 12, 128, 256),
      'state_features': (1, 32, 32, 512),
      'desire': (1, 8),
      'traffic_convention': (1, 2),
      'action_t': (1, 2),
    },
    output_shapes={
      'outputs': (1, 18452),
      'next_state_img': (1, 12, 128, 256),
      'next_state_features': (1, 32, 32, 512),
    },
    output_slices={'hidden_state': slice(2066, 18450)},
    checkpoint=None,
  )


def _queued_spec() -> ModelSpec:
  return ModelSpec(
    sha256='c' * 64,
    nbytes=1024,
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


def _run_server(sock: socket.socket, fn, version: int = 2):
  errors = []

  def run():
    server = TcpTransport(sock)
    server.protocol_version = version
    try:
      fn(server)
    except BaseException as e:
      errors.append(e)
    finally:
      server.close()

  thread = threading.Thread(target=run, daemon=True)
  thread.start()
  return thread, errors


def test_stateful_v3_loaded_engine_and_inference_are_fully_framed():
  spec = _stateful_spec()
  ours, peer = socket.socketpair()
  outputs = np.arange(spec.output_nelem, dtype=np.float32)
  packed = np.arange(1, 13, dtype=np.float32)
  warped = np.zeros(spec.warped_shape, np.uint8)

  def serve(server):
    probe = server.recv(timeout=3)
    assert probe.msg_type == P.Msg.ENGINE_REQ
    assert json.loads(bytes(probe.payload)) == {
      'sha256': spec.sha256, 'nbytes': 0, 'frame_skip': 4,
    }
    server.send_json(P.Msg.ENGINE_RESP, probe.seq, {
      'state': 'ready', 'sha256': spec.sha256, 'spec': spec.to_dict(),
    })

    for frame_id, reset in ((41, True), (42, False)):
      req = server.recv(timeout=3)
      assert req.msg_type == P.Msg.INFER_REQ
      actual_frame, flags = P.unpack_infer_req(req.payload)
      assert actual_frame == frame_id
      assert bool(flags & P.Flag.RESET_QUEUES) is reset
      assert flags & P.Flag.WANT_HIDDEN
      assert req.payload.nbytes == P.INFER_REQ_SIZE + 393216 + 48
      wire_packed = np.frombuffer(
        req.payload, np.float32, 12, P.INFER_REQ_SIZE + spec.warped_nbytes)
      np.testing.assert_array_equal(wire_packed, packed)
      response = P.pack_infer_resp(frame_id, P.Status.OK, 1, 2, 3) + outputs.tobytes()
      server.send(P.Msg.INFER_RESP, req.seq, (response,))

  thread, errors = _run_server(peer, serve, version=3)
  stream = TcpTransport(ours)
  stream.protocol_version = 3
  client = JetlinkClient(stream, deadline=3)
  try:
    assert spec.stateful
    assert spec.model_hw == (128, 256)
    assert spec.packed_shapes == {
      'desire': (8,), 'traffic_convention': (1, 2), 'action_t': (1, 2),
    }
    assert spec.packed_nbytes == 48
    assert spec.state_pairs == {
      'state_img': 'next_state_img',
      'state_features': 'next_state_features',
    }
    assert client.ensure_engine(spec.sha256, 0, frame_skip=4) == spec
    first = client.infer(warped, packed, frame_id=41, reset=True)
    second = client.infer(warped, packed, frame_id=42)
    np.testing.assert_array_equal(first, outputs)
    np.testing.assert_array_equal(second, outputs)
    assert np.isfinite(first).all() and np.isfinite(second).all()
    assert P.VERSION == 2
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


def test_zero_nbytes_engine_probe_keeps_actual_server_model_size():
  spec = _stateful_spec()
  ours, peer = socket.socketpair()

  def serve(server):
    req = server.recv(timeout=3)
    assert req.msg_type == P.Msg.ENGINE_REQ
    assert json.loads(bytes(req.payload)) == {
      'sha256': spec.sha256, 'nbytes': 0, 'frame_skip': 4,
    }
    server.send_json(P.Msg.ENGINE_RESP, req.seq, {
      'state': 'ready', 'sha256': spec.sha256, 'spec': spec.to_dict(),
    })

  thread, errors = _run_server(peer, serve)
  client = JetlinkClient(TcpTransport(ours), deadline=3)
  try:
    resolved = client.ensure_engine(spec.sha256, 0, frame_skip=4)
    assert resolved.nbytes == spec.nbytes == 1024
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


def test_zero_nbytes_engine_probe_does_not_upload_a_missing_model():
  spec = _stateful_spec()
  ours, peer = socket.socketpair()

  def serve(server):
    req = server.recv(timeout=3)
    assert req.msg_type == P.Msg.ENGINE_REQ
    assert json.loads(bytes(req.payload))['nbytes'] == 0
    server.send_json(P.Msg.ENGINE_RESP, req.seq, {
      'state': 'need_upload', 'sha256': spec.sha256, 'detail': 'not cached',
    })

  thread, errors = _run_server(peer, serve)
  client = JetlinkClient(TcpTransport(ours), deadline=3)
  try:
    with pytest.raises(LinkError, match='no cached engine'):
      client.ensure_engine(spec.sha256, 0, onnx_path='/does/not/exist', frame_skip=4)
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


@pytest.mark.parametrize('field,value', [('nbytes', 2048), ('frame_skip', True)])
def test_engine_spec_must_match_nonzero_request(field, value):
  spec = _queued_spec()
  response_spec = spec.to_dict()
  response_spec[field] = value
  ours, peer = socket.socketpair()

  def serve(server):
    req = server.recv(timeout=3)
    assert req.msg_type == P.Msg.ENGINE_REQ
    server.send_json(P.Msg.ENGINE_RESP, req.seq, {
      'state': 'ready', 'sha256': spec.sha256, 'spec': response_spec,
    })

  thread, errors = _run_server(peer, serve)
  client = JetlinkClient(TcpTransport(ours), deadline=3)
  try:
    with pytest.raises(LinkError):
      client.ensure_engine(spec.sha256, spec.nbytes, frame_skip=4)
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


@pytest.mark.parametrize('bad_value', [True, -1])
def test_engine_request_rejects_invalid_integer_types_before_sending(bad_value):
  ours, peer = socket.socketpair()
  client = JetlinkClient(TcpTransport(ours))
  try:
    with pytest.raises(LinkError, match='nbytes'):
      client.ensure_engine('a' * 64, bad_value)
    with pytest.raises(LinkError, match='frame_skip'):
      client.ensure_engine('a' * 64, 1, frame_skip=bad_value)
  finally:
    client.close()
    peer.close()


@pytest.mark.parametrize('failure', ['frame', 'nonfinite'])
def test_stateful_v3_rejects_bad_inference_output(failure):
  spec = _stateful_spec()
  ours, peer = socket.socketpair()

  def serve(server):
    req = server.recv(timeout=3)
    frame_id, _ = P.unpack_infer_req(req.payload)
    response_frame = frame_id + 1 if failure == 'frame' else frame_id
    output = np.zeros(spec.output_nelem, dtype=np.float32)
    if failure == 'nonfinite':
      output[0] = np.nan
    response = P.pack_infer_resp(response_frame, P.Status.OK, 1, 2, 3) + output.tobytes()
    server.send(P.Msg.INFER_RESP, req.seq, (response,))

  thread, errors = _run_server(peer, serve, version=3)
  stream = TcpTransport(ours)
  stream.protocol_version = 3
  client = JetlinkClient(stream, deadline=3)
  client.spec = spec
  try:
    with pytest.raises(LinkError, match='frame|non-finite'):
      client.infer(np.zeros(spec.warped_shape, np.uint8),
                   np.zeros(spec.packed_nelem, np.float32), frame_id=52)
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


def test_v2_rejects_stateful_model_before_inference_send():
  spec = _stateful_spec()
  ours, peer = socket.socketpair()
  verified = threading.Event()

  def serve(server):
    hello = server.recv(timeout=3)
    server.send_json(P.Msg.HELLO_RESP, hello.seq, {'protocol': 2})
    with pytest.raises(LinkTimeout):
      server.recv(timeout=0.1)
    verified.set()

  thread, errors = _run_server(peer, serve)
  client = JetlinkClient(TcpTransport(ours), deadline=3)
  client.spec = spec
  try:
    assert client.hello()['protocol'] == 2
    with pytest.raises(LinkError, match='require protocol 3'):
      client.infer(np.zeros(spec.warped_shape, np.uint8),
                   np.zeros(spec.packed_nelem, np.float32), frame_id=53)
    assert verified.wait(2)
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors


def test_v2_queued_inference_retains_legacy_prev_feat_tail():
  spec = _queued_spec()
  ours, peer = socket.socketpair()
  packed = np.arange(spec.packed_nelem, dtype=np.float32)

  def serve(server):
    hello = server.recv(timeout=3)
    server.send_json(P.Msg.HELLO_RESP, hello.seq, {'protocol': 2})
    req = server.recv(timeout=3)
    assert req.msg_type == P.Msg.INFER_REQ
    _, flags = P.unpack_infer_req(req.payload)
    assert not flags & P.Flag.WANT_HIDDEN
    assert req.payload.nbytes == P.INFER_REQ_SIZE + spec.warped_nbytes + 65584
    offset = P.INFER_REQ_SIZE + spec.warped_nbytes
    wire_packed = np.frombuffer(req.payload, np.float32, spec.packed_nelem, offset)
    np.testing.assert_array_equal(wire_packed, packed)
    output = np.arange(spec.output_nelem, dtype=np.float32)
    response = P.pack_infer_resp(54, P.Status.OK, 1, 2, 3) + output.tobytes()
    server.send(P.Msg.INFER_RESP, req.seq, (response,))

  thread, errors = _run_server(peer, serve)
  client = JetlinkClient(TcpTransport(ours), deadline=3)
  client.spec = spec
  try:
    assert 'prev_feat' in spec.packed_shapes
    assert spec.packed_nbytes == 65584
    assert client.hello()['protocol'] == 2
    output = client.infer(np.zeros(spec.warped_shape, np.uint8), packed, frame_id=54)
    np.testing.assert_array_equal(output, np.arange(spec.output_nelem, dtype=np.float32))
  finally:
    client.close()
    thread.join(timeout=5)
  assert not thread.is_alive()
  assert not errors
