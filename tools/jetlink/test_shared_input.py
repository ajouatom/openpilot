"""Real Linux memfd/Unix sockets; no device, GPU or driving processes required."""
import array
import gc
import json
import os
import socket
import threading
import time
from contextlib import contextmanager

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import link, shared_input as shm
from openpilot.selfdrive.modeld.jetlink.contracts import JETSON_SPEC as SPEC

pytestmark = pytest.mark.skipif(not shm.supported(), reason='Linux memfd required')


@contextmanager
def pair():
  a, b = socket.socketpair()
  a.settimeout(.5)
  b.settimeout(.5)
  reader = shm.Reader.offer(a, SPEC)
  link.send(a, reader.nonce)
  writer = shm.Writer.receive(b, SPEC, link.receive, time.monotonic() + .5)
  try:
    yield reader, writer
  finally:
    writer.close()
    reader.close()
    a.close()
    b.close()


def fill(writer, value=42):
  image = writer.begin()
  image[:] = value
  packed = np.arange(SPEC.packed_nelem, dtype=np.float32)
  return writer.token(image, packed, 7, 3, 123), packed


def test_exact_frame_and_reset_context_1000_cycles():
  with pair() as (reader, writer):
    for i in range(1000):
      token, packed = fill(writer, i % 256)
      frame, flags, sof, image, actual = reader.read(token)
      assert (frame, flags, sof) == (7, 3, 123)
      assert np.all(image == i % 256) and np.array_equal(actual, packed)
      assert not image.flags.writeable and not actual.flags.writeable
      writer.complete()


@pytest.mark.parametrize('field,value', [(0, b'INVALID!'), (1, b'x'*16), (2, 0), (2, 2), (3, 0), (3, 2**63)])
def test_reject_wrong_nonce_generation_or_age(field, value):
  with pair() as (reader, writer):
    token, _ = fill(writer)
    fields = list(shm.TOKEN.unpack(token))
    fields[field] = value
    with pytest.raises(ValueError):
      reader.read(shm.TOKEN.pack(*fields))


def test_busy_duplicate_and_retired_writer_cannot_reuse():
  with pair() as (reader, writer):
    token, _ = fill(writer)
    with pytest.raises(ConnectionError):
      writer.begin()
    reader.read(token)
    with pytest.raises(ValueError):
      reader.read(token)
    writer.close()
    with pytest.raises(ConnectionError):
      writer.begin()


def test_timeout_late_usb_reader_isolated_from_100_reconnections():
  with pair() as (old_reader, old_writer):
    old_token, _ = fill(old_writer, 137)
    old_image = old_reader.image
    old_writer.close()  # modeld timed out; old USB consumer is still alive
    for i in range(100):
      with pair() as (reader, writer):
        assert reader.nonce != old_reader.nonce
        token, _ = fill(writer, i)
        reader.read(token)
        assert np.all(old_image == 137)
        with pytest.raises(ValueError):
          reader.read(old_token)
        writer.complete()
    # Timeout cannot unmap a still-exported consumer view.
    old_reader.close()
    assert np.all(old_image == 137)


@pytest.mark.parametrize('kind', ['size', 'unsealed', 'extra', 'missing', 'truncated', 'nonce'])
def test_bad_descriptor_is_rejected_without_fd_leak(kind):
  before = len(os.listdir('/proc/self/fd'))
  a, b = socket.socketpair()
  a.settimeout(.5)
  b.settimeout(.5)
  import fcntl
  fd = os.memfd_create('test', os.MFD_ALLOW_SEALING | os.MFD_CLOEXEC)
  try:
    os.ftruncate(fd, shm.size_for(SPEC) + (1 if kind == 'size' else 0))
    if kind != 'unsealed':
      fcntl.fcntl(fd, fcntl.F_ADD_SEALS, shm._seals())
    count = 2 if kind == 'extra' else (32 if kind == 'truncated' else 1)
    ancillary = [] if kind == 'missing' else [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array.array('i', [fd]*count))]
    a.sendmsg([b'A'], ancillary)
    link.send(a, b'B'*16 if kind == 'nonce' else b'A'*16)
    with pytest.raises(ValueError):
      shm.Writer.receive(b, SPEC, link.receive, time.monotonic()+.5)
  finally:
    os.close(fd)
    a.close()
    b.close()
  assert len(os.listdir('/proc/self/fd')) == before


@contextmanager
def fake_usb_server(path, capable=True, error=None):
  listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  listener.bind(str(path))
  listener.listen(1)
  captured = []
  failures = []
  def serve():
    shared = None
    try:
      conn, _ = listener.accept()
      with conn:
        conn.settimeout(.5)
        link.send(conn, json.dumps({'spec': SPEC.to_dict(), shm.CAPABILITY: capable}).encode())
        request = link.receive(conn)
        if request == shm.SETUP:
          shared = shm.Reader.offer(conn, SPEC)
          link.send(conn, shared.nonce)
          request = link.receive(conn)
        if shared is not None:
          frame, flags, sof, image, packed = shared.read(request)
          captured.append((image.copy(), packed.copy(), frame, flags, sof))
        else:
          frame, flags, sof = link.REQUEST.unpack_from(request)
          captured.append((bytes(request), frame, flags, sof))
        if error == 'disconnect':
          return
        if error == 'timeout':
          time.sleep(.08)
          return
        output = np.zeros(SPEC.output_nelem, np.float32)
        if error == 'nan':
          output[0] = np.nan
        link.send_parts(conn, link.REPLY.pack(frame + (error == 'frame'), 1, 2, 3), output)
    except Exception as e:
      failures.append(repr(e))
    finally:
      if shared is not None:
        shared.close()
  thread = threading.Thread(target=serve)
  thread.start()
  try:
    yield captured
  finally:
    thread.join(2)
    listener.close()
    assert not thread.is_alive() and not failures, failures


@pytest.mark.parametrize('capable,enabled', [(False, True), (True, False), (True, True), (False, None), (True, None)])
def test_real_client_legacy_and_negotiated_exact_input(tmp_path, monkeypatch, capable, enabled):
  if enabled is None:
    monkeypatch.delenv('JETLINK_RAM_INPUT', raising=False)
  else:
    monkeypatch.setenv('JETLINK_RAM_INPUT', str(int(enabled)))
  path = tmp_path/'ipc'
  with fake_usb_server(path, capable) as captured:
    client = link.Client(str(path), timeout=.5)
    try:
      image = client.image_buffer()
      if image is None:
        image = np.empty(SPEC.warped_shape, np.uint8)
      image[:] = 79
      packed = np.arange(SPEC.packed_nelem, dtype=np.float32)
      client.infer(image, packed, 17, reset=True, source_sof=1234)
      if capable and enabled is not False:
        assert np.array_equal(captured[0][0], image) and np.array_equal(captured[0][1], packed)
        assert captured[0][2:] == (17, 1, 1234)
        assert not client.shared.busy
      else:
        assert captured[0][0] == link.REQUEST.pack(17, 1, 1234) + image.tobytes() + packed.tobytes()
    finally:
      client.close()


@pytest.mark.parametrize('error', ['disconnect', 'timeout', 'frame', 'nan'])
def test_reply_failure_permanently_retires_slot(tmp_path, monkeypatch, error):
  monkeypatch.setenv('JETLINK_RAM_INPUT', '1')
  with fake_usb_server(tmp_path/'ipc', error=error):
    client = link.Client(str(tmp_path/'ipc'), timeout=.5)
    client.timeout = .03
    image = client.image_buffer()
    image[:] = 67
    with pytest.raises((ConnectionError, TimeoutError, ValueError)):
      client.infer(image, np.zeros(SPEC.packed_nelem, np.float32), 1, True)
    assert client.shared.closed and client.sock.fileno() == -1
    assert np.all(image == 67)
    with pytest.raises(ConnectionError):
      client.image_buffer()


def test_100_connections_no_fd_or_mapping_leak():
  gc.collect()
  fds = len(os.listdir('/proc/self/fd'))
  def mappings():
    with open('/proc/self/maps') as f:
      return f.read().count('/memfd:jetlink-input')
  maps = mappings()
  for _ in range(100):
    with pair() as (r, w):
      token, _ = fill(w)
      r.read(token)
      w.complete()
  gc.collect()
  assert len(os.listdir('/proc/self/fd')) == fds and mappings() == maps


def test_frame_wrap_does_not_reuse_generation_and_reset_is_preserved():
  with pair() as (reader, writer):
    for frame, reset in ((0xFFFFFFFF, 0), (0, 1), (1, 0)):
      image = writer.begin()
      image[:] = frame % 256
      token = writer.token(image, np.zeros(SPEC.packed_nelem, np.float32), frame, reset, 77)
      actual = reader.read(token)
      assert actual[:3] == (frame, reset, 77)
      writer.complete()
    assert reader.generation == writer.generation == 3


def test_wrong_source_cannot_publish_reserved_slot():
  with pair() as (_, writer):
    writer.begin()
    with pytest.raises(ValueError, match='does not own'):
      writer.token(np.zeros(SPEC.warped_shape, np.uint8), np.zeros(SPEC.packed_nelem, np.float32), 1, 1, 0)


def test_invalid_context_closes_reserved_slot_without_a_request():
  with pair() as (_, writer):
    a, b = socket.socketpair()
    client = object.__new__(link.Client)
    client.sock, client.shared, client.spec = b, writer, SPEC
    image = client.image_buffer()
    try:
      with pytest.raises(ValueError, match='invalid recurrent'):
        client.infer(image, np.full(SPEC.packed_nelem, np.nan, np.float32), 1, True)
      assert writer.closed and a.recv(1) == b''
    finally:
      a.close()
      client.close()


@pytest.mark.parametrize('deferred', [False, True])
@pytest.mark.parametrize('shared_enabled', [False, True])
@pytest.mark.parametrize('projected', [False, True])
def test_actual_daemon_consumes_exact_inputs_before_matching_reply(tmp_path, monkeypatch, shared_enabled, projected, deferred):
  import hashlib
  import sys
  from types import SimpleNamespace as NS
  from openpilot.selfdrive.modeld.jetlink import daemon
  from jetlink import output_projection as projection
  monkeypatch.setenv('JETLINK_RAM_INPUT', str(int(shared_enabled)))
  monkeypatch.setenv('JETLINK_DEFER_NAV', str(int(deferred)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(put_bool_nonblocking=lambda *a: None)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(event=lambda *a, **kw: None)))
  monkeypatch.setattr(daemon, 'publish', lambda *a, **kw: None)
  monkeypatch.setattr(daemon, 'update_affinity', lambda: None)
  monkeypatch.setattr(daemon, 'check_loaded', lambda *a: None)
  stop = threading.Event()
  monkeypatch.setattr(daemon, 'host_attached', lambda: not stop.is_set())
  captured, failures = [], []
  class USB:
    spec = SPEC
    deferred_map_available = True
    flush_map = lambda self: 0
    output_projection_available = projected
    project_output = False
    last_state = None
    last_timings = (24000, 600, 25000)
    t = NS(last_receive={})
    def infer_begin(self, image, packed, frame, reset, want_state, **options):
      assert options == ({'nav_metadata': (0, 0, 0)} if deferred else {})
      captured.append((hashlib.sha256(image.tobytes() + packed.tobytes()).digest(), frame, reset))
      # Actual daemon must send the matching reply before a slot is reusable.
      return frame
    def infer_end(self, frame):
      return np.full(projection.COUNT if self.project_output else SPEC.output_nelem, frame, np.float32)
    def state(self): return None
  listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  path = tmp_path / 'daemon'
  listener.bind(str(path)); listener.listen(1); listener.settimeout(.1)
  def serve():
    try: daemon._serve_local(listener, USB(), {'carrot_host': 'jetson'}, None, NS(sent=lambda *a: None))
    except Exception as exc: failures.append(repr(exc))
  thread = threading.Thread(target=serve); thread.start()
  expected = []
  client = None
  try:
    client = link.Client(str(path), timeout=1.)
    assert (client.shared is not None) == shared_enabled
    assert client.project_output == projected
    for frame in range(1, 25):
      # Also accept a legacy payload on a negotiated connection when no slot
      # has been reserved, preserving existing Client.infer-only callers.
      image = client.image_buffer() if frame % 3 else None
      if image is None: image = np.empty(SPEC.warped_shape, np.uint8)
      image[:] = frame
      packed = np.arange(SPEC.packed_nelem, dtype=np.float32) * frame
      expected.append((hashlib.sha256(image.tobytes() + packed.tobytes()).digest(), frame, frame == 1))
      out = client.infer(image, packed, frame, reset=frame == 1)
      assert np.all(out == frame)
      if client.shared is not None: assert not client.shared.busy
    assert captured == expected
  finally:
    stop.set()
    if client is not None: client.close()
    thread.join(2); listener.close()
  assert not thread.is_alive() and not failures
