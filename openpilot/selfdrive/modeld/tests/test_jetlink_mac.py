"""Comma-side protocol/guard tests; no claim of Mac USB or CoreML validation."""
from dataclasses import replace
import hashlib
import io
import json
import socket
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.common.jetlink_peer import is_mac_peer
from openpilot.selfdrive.modeld.jetlink import link, mac
from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.server.session import Session
from jetlink.transport.tcp import TcpTransport
from jetlink.transport.base import LinkError, LinkTimeout

PEER = {'protocol': 2, 'backend': 'ort', 'device': 'ane-Apple_M1_Pro'}


@pytest.mark.parametrize('tag', ['ane-Apple_M1_Pro', 'coreml-Apple-M2-Max', 'ane-whole-Apple_M4', 'ane-Apple_M3_Ultra'])
def test_mac_recognition_is_transport_and_backend_specific(tag):
  peer = dict(PEER, device=tag)
  assert is_mac_peer(peer)
  assert not is_mac_peer(peer, 'tcp')
  assert not is_mac_peer(dict(peer, carrot_host='jetson'))
  assert not is_mac_peer(dict(peer, backend='trt'))


@pytest.mark.parametrize('change', [dict(device='coreml-Apple_A17_Pro'), dict(device='ane-unknown'),
  dict(device='cpu-Apple_M1_Pro'), dict(device='ane-Apple_M1_extra'), dict(protocol=1),
  dict(protocol=True), dict(carrot_host=[]), dict(device=None)])
def test_unknown_or_conflicting_identity_is_not_mac(change):
  assert not is_mac_peer(dict(PEER, **change))


@pytest.fixture
def model(monkeypatch):
  body = b'controlled protocol fixture, not an ONNX model' * 17
  spec = replace(link.SPEC, sha256=hashlib.sha256(body).hexdigest(), nbytes=len(body))
  monkeypatch.setattr(link, 'SPEC', spec)
  monkeypatch.setattr(mac, 'SPEC', spec)
  return body, spec


def response(body, url=mac.MODEL_URL):
  obj = io.BytesIO(body)
  obj.geturl = lambda: url
  return obj


def test_download_cache_corruption_and_offline_reuse(tmp_path, monkeypatch, model):
  body, spec = model
  calls = []
  def fetch(url, **kwargs):
    calls.append(url)
    return response(body)
  monkeypatch.setattr(mac.urllib.request, 'urlopen', fetch)
  path = mac.model_file(lambda: True, lambda: True, lambda *a: None, tmp_path)
  assert path.read_bytes() == body and calls == [mac.MODEL_URL]
  monkeypatch.setattr(mac.urllib.request, 'urlopen', lambda *a, **kw: pytest.fail('cache should work offline'))
  assert mac.model_file(lambda: True, lambda: True, lambda *a: None, tmp_path) == path
  path.write_bytes(b'x' * len(body))
  monkeypatch.setattr(mac.urllib.request, 'urlopen', fetch)
  assert mac.model_file(lambda: True, lambda: True, lambda *a: None, tmp_path).read_bytes() == body


@pytest.mark.parametrize('failure', ['hash', 'short', 'oversize', 'redirect', 'ignition', 'disconnect'])
def test_bad_or_interrupted_download_never_installs(tmp_path, monkeypatch, model, failure):
  body, spec = model
  received = {'hash': b'x' * len(body), 'short': body[:-1], 'oversize': body + b'x'}.get(failure, body)
  monkeypatch.setattr(mac.urllib.request, 'urlopen', lambda *a, **kw:
                      response(received, 'https://example.invalid/model' if failure == 'redirect' else mac.MODEL_URL))
  allowed = [True]
  def progress(stage, fraction, message):
    if fraction and failure in ('ignition', 'disconnect'):
      allowed[0] = False
  with pytest.raises((ValueError, mac.PreparationDeferred)):
    mac.model_file(lambda: allowed[0] if failure == 'ignition' else True,
                   lambda: allowed[0] if failure == 'disconnect' else True, progress, tmp_path)
  assert not list(tmp_path.iterdir())


@pytest.mark.parametrize('peer', [{'carrot_host': 'jetson', 'backend': 'trt', 'device': 'Orin'},
                                 {'backend': 'trt', 'device': 'Orin'}, {'backend': 'ort', 'device': 'cuda-RTX'}])
def test_jetson_and_other_peers_keep_legacy_call_without_mac_side_effects(peer, monkeypatch):
  calls = []
  def ensure(*a, **kw):
    calls.append((a, kw))
    return link.SPEC
  def untouched(*a):
    pytest.fail('non-Mac must not invoke Mac callbacks or download')
  monkeypatch.setattr(mac, 'model_file', untouched)
  mac.prepare(SimpleNamespace(ensure_engine=ensure), peer, untouched, untouched, untouched)
  assert calls == [((link.SPEC.sha256, link.SPEC.nbytes), dict(frame_skip=4, build_timeout=30))]


def test_unprepared_mac_defers_before_any_engine_request():
  mac_client = SimpleNamespace(ensure_engine=lambda *a, **kw: pytest.fail('must defer first'))
  with pytest.raises(mac.PreparationDeferred, match='ignition off'):
    mac.prepare(mac_client, PEER, lambda: False, lambda: True, lambda *a: None)


class DesktopTransport(TcpTransport):
  def _write(self, bufs):
    # Windows has no sendmsg; framing remains the actual vendored transport.
    self._set_timeout(self._write_timeout())
    return self.sock.send(b''.join(bufs))


@pytest.mark.parametrize('bad_spec', [False, True])
def test_real_wire_upload_ready_inference_and_cached_reconnect(tmp_path, monkeypatch, model, bad_spec):
  body, spec = model
  downloads, uploaded, errors = [], bytearray(), []
  def fetch(url, **kw):
    downloads.append(url)
    return response(body)
  monkeypatch.setattr(mac.urllib.request, 'urlopen', fetch)
  for attempt in range(1 if bad_spec else 2):
    a, b = socket.socketpair()
    client, remote = JetlinkClient(DesktopTransport(a)), DesktopTransport(b)
    state = {'requests': 0}
    def host():
      try:
        # Use upstream HELLO itself: its engine_state is request-scoped and
        # starts at none even with an already-loaded model on reconnect.
        hello_host = SimpleNamespace(backend=SimpleNamespace(describe=lambda: dict(PEER)),
          telemetry=SimpleNamespace(read=lambda: {}), status=lambda *a: {'state': 'none'},
          loaded_sha=lambda: spec.sha256 if uploaded else None, sleep_after=0,
          cache=SimpleNamespace(inventory=lambda: []))
        hello_session = Session(remote, hello_host)
        while True:
          msg = remote.recv(timeout=3)
          if msg.msg_type == P.Msg.HELLO_REQ:
            hello_session.on_hello(msg)
          elif msg.msg_type == P.Msg.ENGINE_REQ:
            state['requests'] += 1
            request = json.loads(bytes(msg.payload))
            assert request == dict(sha256=spec.sha256, nbytes=len(body), frame_skip=4)
            remote.send_json(P.Msg.ENGINE_RESP, msg.seq,
                             dict(state='ready', spec=spec.to_dict()) if uploaded else dict(state='need_upload', chunk=64))
          elif msg.msg_type == P.Msg.UPLOAD_CHUNK:
            offset = int.from_bytes(msg.payload[:8], 'little')
            assert offset == len(uploaded)
            uploaded.extend(msg.payload[8:])
          elif msg.msg_type == P.Msg.UPLOAD_DONE:
            assert bytes(uploaded) == body
            remote.send_json(P.Msg.ENGINE_RESP, msg.seq, dict(state='building'))
            remote.send_json(P.Msg.PROGRESS, 0, dict(stage='build', frac=.5, msg='Fixture build'))
            ready = spec.to_dict()
            if bad_spec:
              ready['checkpoint'] = 'wrong checkpoint'
            remote.send_json(P.Msg.ENGINE_RESP, 0, dict(state='ready', spec=ready))
          elif msg.msg_type == P.Msg.INFER_REQ:
            frame, _ = P.unpack_infer_req(msg.payload)
            assert len(msg.payload) == P.INFER_REQ_SIZE + spec.warped_nbytes + spec.packed_nbytes
            remote.send(P.Msg.INFER_RESP, msg.seq, [P.pack_infer_resp(frame, P.Status.OK, 1, 2, 3),
                                                    np.zeros(spec.output_nelem, np.float32)])
      except LinkError:
        pass
      except BaseException as exc:
        errors.append(exc)
      finally:
        remote.close()
    worker = threading.Thread(target=host, daemon=True)
    worker.start()
    try:
      peer = client.hello()
      original = client.t
      if bad_spec:
        with pytest.raises(ValueError, match='contract mismatch'):
          mac.prepare(client, peer, lambda: True, lambda: True, lambda *a: None, tmp_path)
      else:
        # Cached, loaded Mac reconnects even after ignition starts, no download.
        mac.prepare(client, peer, lambda: attempt == 0, lambda: True, lambda *a: None, tmp_path)
        result = client.infer(np.zeros(spec.warped_shape, np.uint8), np.zeros(spec.packed_nelem, np.float32), 9)
        assert result.size == spec.output_nelem
      assert client.t is original
    finally:
      client.close()
      worker.join(4)
    assert not worker.is_alive() and not errors
  assert downloads == [mac.MODEL_URL]


def test_setup_recv_polls_cancellation_and_restores_transport(monkeypatch):
  calls = []
  class Silent:
    def recv(self, timeout):
      calls.append(timeout)
      raise LinkTimeout('silence')
  original = Silent()
  def ensure(*args, **kwargs):
    return client.t.recv(timeout=60)
  client = SimpleNamespace(t=original, ensure_engine=ensure)
  with pytest.raises(mac.PreparationDeferred):
    mac.prepare(client, PEER, lambda: len(calls) < 2, lambda: True, lambda *a: None)
  assert calls == [1., 1.] and client.t is original


def test_setup_transport_preserves_partial_message_across_poll_timeout():
  a, b = socket.socketpair()
  body = b'{"state":"building"}'
  packet = P.pack_header(P.Msg.ENGINE_RESP, 4, len(body)) + body
  checks = []
  original = DesktopTransport(a)
  wrapped = mac.PreparationTransport(original, lambda: checks.append(True))
  def drip():
    b.sendall(packet[:9])
    time.sleep(1.1)
    b.sendall(packet[9:])
  worker = threading.Thread(target=drip)
  worker.start()
  try:
    result = wrapped.recv(timeout=3)
    assert result.seq == 4 and bytes(result.payload) == body
    assert len(checks) >= 2
  finally:
    worker.join(4)
    a.close()
    b.close()
