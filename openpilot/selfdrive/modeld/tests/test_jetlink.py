import json
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import link
from openpilot.selfdrive.modeld.jetlink.model import may_join
from openpilot.selfdrive.modeld.jetlink.warp import validated_warp
from openpilot.selfdrive.modeld.jetlink import warp as warp_module
from jetlink.spec import ModelSpec


@pytest.mark.parametrize('device', ['tici', 'tizi', 'pc', ''])
def test_fused_warp_leaves_unvalidated_devices_on_original_path(device):
  reference = object()
  assert validated_warp(reference, None, None, None, 4, device) is reference


@pytest.mark.parametrize('failure', [None, 'pixel', 'shape', 'dtype', 'build'])
def test_fused_warp_requires_exact_parity_and_restores_transforms(monkeypatch, failure):
  expected = np.arange(32, dtype=np.uint8).reshape(2, 4, 4)
  actual = expected.copy()
  if failure == 'pixel':
    actual[0, 0, 0] ^= 1
  elif failure == 'shape':
    actual = actual.reshape(-1)
  elif failure == 'dtype':
    actual = actual.astype(np.int16)
  class FakeTensor:
    def __init__(self, *args, **kwargs): pass
    def realize(self): return self
  def candidate(**kwargs):
    if failure == 'build':
      raise RuntimeError('compiler failure')
    return NS(numpy=lambda: actual)
  def reference(**kwargs):
    return NS(numpy=lambda: expected)
  failures = []
  monkeypatch.setitem(sys.modules, 'tinygrad', NS(Tensor=FakeTensor, TinyJit=lambda _: candidate))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog',
                      NS(cloudlog=NS(warning=lambda *args: None, exception=lambda *args: failures.append(args))))
  monkeypatch.setattr(warp_module, 'make_warp', lambda *args: None)
  monkeypatch.setattr(warp_module, 'validation_matrices', lambda: [np.ones((3, 3))])
  transforms = {key: np.eye(3, dtype=np.float32) for key in ('tfm', 'big_tfm')}
  selected = validated_warp(reference, NS(size=64), transforms, {}, 4, 'mici')
  assert selected is (candidate if failure is None else reference)
  assert bool(failures) == (failure is not None)
  assert all(np.array_equal(matrix, np.eye(3)) for matrix in transforms.values())


def test_join_requires_fresh_stopped_fully_disengaged_messages():
  messages = dict(carState=NS(standstill=True, vEgo=0.), selfdriveState=NS(enabled=False),
                  carControl=NS(latActive=False, longActive=False))
  valid = dict.fromkeys(messages, True)
  received = dict.fromkeys(messages, 100.)
  assert may_join(100.1, messages, valid, received)
  for key in messages:
    assert not may_join(100.1, messages, {**valid, key: False}, received)
    assert not may_join(100.1, messages, valid, {**received, key: 99.})
  for key, field in (('carState', 'vEgo'), ('selfdriveState', 'enabled'), ('carControl', 'latActive'), ('carControl', 'longActive')):
    original = getattr(messages[key], field)
    setattr(messages[key], field, 1)
    assert not may_join(100.1, messages, valid, received)
    setattr(messages[key], field, original)


def test_model_contract_checks_checkpoint_and_every_output_slice():
  link.validate_spec(link.SPEC)
  for field, change in [('checkpoint', 'different-trained-model'), ('frame_skip', 3), ('nbytes', 1)]:
    spec = link.SPEC.to_dict()
    spec[field] = change
    with pytest.raises(ValueError):
      link.validate_spec(ModelSpec.from_dict(spec))
  spec = link.SPEC.to_dict()
  spec['output_slices']['plan'][0] += 1
  with pytest.raises(ValueError):
    link.validate_spec(ModelSpec.from_dict(spec))


@pytest.mark.parametrize('failure', ['frame', 'nan', 'length', 'timeout'])
def test_ipc_rejects_stale_nonfinite_truncated_and_missing_results(tmp_path, failure):
  if not hasattr(socket, 'AF_UNIX'):
    pytest.skip('Unix sockets unavailable')
  path = str(tmp_path / 's')
  listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  listener.bind(path)
  listener.listen(1)
  def server():
    with listener.accept()[0] as conn:
      link.send(conn, json.dumps({'spec': link.SPEC.to_dict()}).encode())
      link.receive(conn)
      if failure == 'timeout':
        time.sleep(.2)
        return
      output = np.zeros(link.SPEC.output_nelem, np.float32)
      if failure == 'nan':
        output[100] = np.nan
      payload = link.REPLY.pack(8 if failure == 'frame' else 7, 0, 0, 0) + output.tobytes()
      link.send(conn, payload[:-4] if failure == 'length' else payload)
  worker = threading.Thread(target=server)
  worker.start()
  client = link.Client(path, timeout=.05)
  try:
    with pytest.raises((ValueError, TimeoutError, ConnectionError)):
      client.infer(np.zeros(link.SPEC.warped_shape, np.uint8), np.zeros(link.SPEC.packed_nelem, np.float32), 7)
  finally:
    client.close()
    worker.join()
    listener.close()


def test_ipc_rejects_oversized_packet_without_allocating():
  a, b = socket.socketpair()
  try:
    a.sendall(struct.pack('<I', link.MAX_PACKET + 1))
    with pytest.raises(ValueError):
      link.receive(b)
  finally:
    a.close()
    b.close()


def test_ipc_total_deadline_cannot_be_extended_by_partial_reads():
  a, b = socket.socketpair()
  def drip():
    try:
      a.sendall(struct.pack('<I', 100))
      for _ in range(10):
        a.sendall(b'x')
        time.sleep(.02)
    except OSError:
      pass
  worker = threading.Thread(target=drip)
  worker.start()
  start = time.monotonic()
  try:
    with pytest.raises((TimeoutError, ConnectionError)):
      link.receive(b, start + .05)
    assert time.monotonic() - start < .15
  finally:
    b.close()
    worker.join()
    a.close()


def test_scatter_send_handles_partial_writes_without_changing_wire_bytes():
  class PartialSocket:
    def __init__(self):
      self.data = bytearray()
    def sendmsg(self, parts):
      data = b''.join(parts)[:7]
      self.data.extend(data)
      return len(data)
  sock = PartialSocket()
  image = np.arange(48, dtype=np.uint8).reshape(2, 4, 6)
  floats = np.arange(12, dtype=np.float32)
  link.send_parts(sock, b'head', image, floats)
  expected = b'head' + image.tobytes() + floats.tobytes()
  assert sock.data == struct.pack('<I', len(expected)) + expected


def test_scatter_send_deadline_includes_partial_writes():
  class SlowSocket:
    def settimeout(self, timeout):
      assert timeout > 0
    def sendmsg(self, parts):
      time.sleep(.02)
      return 1
  start = time.monotonic()
  with pytest.raises(TimeoutError):
    link.send_parts(SlowSocket(), b'x' * 100, deadline=start + .05)
  assert time.monotonic() - start < .15


def test_reusable_reader_handles_fragmented_packets_and_preserves_bounds():
  a, b = socket.socketpair()
  reader = link.PacketReader(100)
  storage = reader.payload
  try:
    for payload in (b'x' * 100, b'abc'):
      wire = struct.pack('<I', len(payload)) + payload
      def sender():
        for pos in range(0, len(wire), 3):
          a.sendall(wire[pos:pos + 3])
      worker = threading.Thread(target=sender)
      worker.start()
      assert bytes(reader.receive(b, time.monotonic() + 1)) == payload
      worker.join()
      assert reader.payload is storage
    a.sendall(struct.pack('<I', 101))
    with pytest.raises(ValueError):
      reader.receive(b)
    assert reader.payload is storage
  finally:
    a.close()
    b.close()


def test_reusable_reader_abandons_partial_header_on_timeout():
  a, b = socket.socketpair()
  try:
    a.sendall(b'\x04')
    with pytest.raises(ConnectionError, match='partial'):
      link.PacketReader(100).receive(b, time.monotonic() + .02)
  finally:
    a.close()
    b.close()


def test_ipc_reuses_storage_without_overwriting_retained_output(tmp_path):
  if not hasattr(socket, 'AF_UNIX'):
    pytest.skip('Unix sockets unavailable')
  path = str(tmp_path / 's')
  listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  listener.bind(path)
  listener.listen(1)
  image = np.arange(link.SPEC.warped_nbytes, dtype=np.uint8).reshape(link.SPEC.warped_shape)
  packed = np.arange(link.SPEC.packed_nelem, dtype=np.float32)
  errors = []
  def server():
    try:
      with listener.accept()[0] as conn:
        link.send(conn, json.dumps({'spec': link.SPEC.to_dict()}).encode())
        reader = link.PacketReader(link.REQUEST.size + image.nbytes + packed.nbytes)
        for frame in (1, 2):
          payload = reader.receive(conn, time.monotonic() + 1)
          assert bytes(payload) == link.REQUEST.pack(frame, int(frame == 1), 123000000 + frame) + image.tobytes() + packed.tobytes()
          output = np.full(link.SPEC.output_nelem, frame, np.float32)
          link.send_parts(conn, link.REPLY.pack(frame, 1, 2, 3), output)
    except Exception as exc:
      errors.append(exc)
  worker = threading.Thread(target=server)
  worker.start()
  client = link.Client(path, timeout=1)
  try:
    first = client.infer(image, packed, 1, reset=True, source_sof=123000001)
    second = client.infer(image, packed, 2, source_sof=123000002)
    assert np.all(first == 1) and np.all(second == 2)
    assert client.timings == [1, 2, 3]
  finally:
    client.close()
    worker.join()
    listener.close()
  assert not errors
