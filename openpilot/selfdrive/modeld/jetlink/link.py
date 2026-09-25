"""Bounded local IPC; the USB owner never executes on modeld's realtime thread."""
import json
from pathlib import Path
import socket
import struct
import time

import numpy as np

from openpilot.selfdrive.modeld.jetlink import VENDOR  # noqa: F401
from jetlink.spec import ModelSpec

SPEC = ModelSpec.from_dict(json.loads(Path(__file__).with_name('cinque_v2.json').read_text()))
SOCKET = '/dev/shm/carrot-jetlink.sock'
STATUS = Path('/dev/shm/carrot-jetlink.json')
FAULT = Path('/dev/shm/carrot-jetlink-fault')
REQUEST = struct.Struct('<II')
REPLY = struct.Struct('<I3I')
MAX_PACKET = 1 << 20


def read_exact(sock, size):
  result = bytearray(size)
  offset = 0
  while offset < size:
    try:
      n = sock.recv_into(memoryview(result)[offset:])
    except TimeoutError:
      if offset:
        raise ConnectionError('partial IPC packet timed out') from None
      raise
    if not n:
      raise ConnectionError('IPC disconnected')
    offset += n
  return result


def receive(sock):
  size, = struct.unpack('<I', read_exact(sock, 4))
  if not 0 < size <= MAX_PACKET:
    raise ValueError('invalid IPC packet size')
  try:
    return read_exact(sock, size)
  except TimeoutError:
    raise ConnectionError('IPC payload timed out') from None


def send(sock, data):
  if not 0 < len(data) <= MAX_PACKET:
    raise ValueError('invalid IPC packet size')
  sock.sendall(struct.pack('<I', len(data)) + data)


def state():
  try:
    value = json.loads(STATUS.read_text())
    if 0 <= time.monotonic() - value['updated'] < 3:
      return value
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return {'state': 'unavailable'}


def validate_spec(spec):
  if spec.to_dict() != SPEC.to_dict():
    raise ValueError('Jetlink model identity or input/output contract mismatch')


def fault_active():
  try:
    return 0 <= time.monotonic() - float(FAULT.read_text()) < 5
  except (OSError, ValueError):
    return False


def badge():
  """Separate Jetlink readiness from eGPU presence and from active inference."""
  link = state()
  try:
    model = json.loads(Path('/dev/shm/carrot-jetlink-model.json').read_text())
    if 0 <= time.monotonic() - model['updated'] < 3 and model['active']:
      return 'Jetlink v2', 'active'
  except (OSError, ValueError, KeyError, TypeError):
    pass
  status = link.get('state')
  if status == 'ready':
    return 'Jetlink READY', 'ready'
  if status in ('connecting', 'loading'):
    return 'Jetlink WAIT', 'loading'
  if status == 'retrying':
    return 'Jetlink RETRY', 'error'
  return None


class Client:
  def __init__(self, path=SOCKET, timeout=.15):
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    self.sock.settimeout(timeout)
    try:
      self.sock.connect(path)
      hello = json.loads(receive(self.sock))
      validate_spec(ModelSpec.from_dict(hello['spec']))
    except Exception:
      self.sock.close()
      raise
    self.timings = (0, 0, 0)

  def infer(self, image, packed, frame, reset=False):
    if image.dtype != np.uint8 or image.shape != SPEC.warped_shape:
      raise ValueError('invalid warped image')
    if packed.dtype != np.float32 or packed.size != SPEC.packed_nelem or not np.all(np.isfinite(packed)):
      raise ValueError('invalid recurrent input')
    send(self.sock, REQUEST.pack(frame, int(reset)) + image.tobytes() + packed.tobytes())
    reply = receive(self.sock)
    if len(reply) != REPLY.size + SPEC.output_nbytes:
      raise ValueError('invalid inference reply size')
    fid, *self.timings = REPLY.unpack_from(reply)
    if fid != frame:
      raise ValueError('stale inference reply')
    output = np.frombuffer(reply, np.float32, SPEC.output_nelem, REPLY.size).copy()
    if not np.all(np.isfinite(output)):
      raise ValueError('nonfinite inference reply')
    return output

  def close(self):
    self.sock.close()
