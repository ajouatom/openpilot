import json
import socket
import struct
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import link
from openpilot.selfdrive.modeld.jetlink.model import may_join
from jetlink.spec import ModelSpec


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
