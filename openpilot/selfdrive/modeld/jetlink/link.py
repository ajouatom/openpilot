"""Bounded local IPC; the USB owner never executes on modeld's realtime thread."""
import json
from concurrent.futures import Future, InvalidStateError
import os
from pathlib import Path
import socket
import struct
import time
import threading

import numpy as np

from openpilot.selfdrive.modeld.jetlink import VENDOR  # noqa: F401
from openpilot.common.jetlink_status import badge  # noqa: F401
from jetlink.spec import ModelSpec
from openpilot.selfdrive.modeld.jetlink.contracts import DEFAULT_SPEC, validate_model_spec

SPEC = DEFAULT_SPEC
SOCKET = '/dev/shm/carrot-jetlink.sock'
STATUS = Path('/dev/shm/carrot-jetlink.json')
FAULT = Path('/dev/shm/carrot-jetlink-fault')
REQUEST = struct.Struct('<IIQ')
REPLY = struct.Struct('<I3I')
MAX_PACKET = 1 << 20


def read_exact(sock, size, deadline=None):
  result = bytearray(size)
  read_into(sock, memoryview(result), deadline)
  return result


def receive(sock, deadline=None):
  size, = struct.unpack('<I', read_exact(sock, 4, deadline))
  if not 0 < size <= MAX_PACKET:
    raise ValueError('invalid IPC packet size')
  try:
    return read_exact(sock, size, deadline)
  except TimeoutError:
    raise ConnectionError('IPC payload timed out') from None


def send_parts(sock, *parts, deadline=None):
  """Send one existing wire-format packet without concatenating image buffers."""
  views = [memoryview(part).cast('B') for part in parts]
  size = sum(part.nbytes for part in views)
  if not 0 < size <= MAX_PACKET:
    raise ValueError('invalid IPC packet size')
  views.insert(0, memoryview(struct.pack('<I', size)))
  while views:
    if deadline is not None:
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        raise TimeoutError('IPC send deadline exceeded')
      sock.settimeout(remaining)
    if hasattr(sock, 'sendmsg'):
      sent = sock.sendmsg(views)
    else:
      sent = sock.send(views[0])
    if sent <= 0:
      raise ConnectionError('IPC disconnected during send')
    while views and sent >= views[0].nbytes:
      sent -= views.pop(0).nbytes
    if sent:
      views[0] = views[0][sent:]


def send(sock, data):
  send_parts(sock, data)


class PacketReader:
  """Reuse bounded receive storage; a returned view lasts until the next read."""
  def __init__(self, capacity):
    if not 0 < capacity <= MAX_PACKET:
      raise ValueError('invalid IPC receive capacity')
    self.header = bytearray(4)
    self.payload = bytearray(capacity)

  def receive(self, sock, deadline=None):
    read_into(sock, memoryview(self.header), deadline)
    size, = struct.unpack('<I', self.header)
    if not 0 < size <= len(self.payload):
      raise ValueError('invalid IPC packet size')
    result = memoryview(self.payload)[:size]
    try:
      read_into(sock, result, deadline)
    except TimeoutError:
      raise ConnectionError('IPC payload timed out') from None
    return result


def read_into(sock, result, deadline=None):
  offset = 0
  while offset < result.nbytes:
    if deadline is not None:
      remaining = deadline - time.monotonic()
      if remaining <= 0:
        if offset:
          raise ConnectionError('partial IPC packet timed out')
        raise TimeoutError('IPC deadline exceeded')
      sock.settimeout(remaining)
    try:
      n = sock.recv_into(result[offset:])
    except TimeoutError:
      if offset:
        raise ConnectionError('partial IPC packet timed out') from None
      raise
    if not n:
      raise ConnectionError('IPC disconnected')
    offset += n


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


class Client:
  def __init__(self, path=SOCKET, timeout=.15):
    self.timeout = timeout
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    self.sock.settimeout(timeout)
    try:
      self.sock.connect(path)
      hello = json.loads(receive(self.sock, time.monotonic() + timeout))
      self.spec = validate_model_spec(ModelSpec.from_dict(hello['spec']))
    except Exception:
      self.sock.close()
      raise
    self.timings = (0, 0, 0)
    self.reader = PacketReader(REPLY.size + self.spec.output_nbytes)

  def infer(self, image, packed, frame, reset=False, source_sof=0):
    if image.dtype != np.uint8 or image.shape != self.spec.warped_shape or not image.flags.c_contiguous:
      raise ValueError('invalid warped image')
    if packed.dtype != np.float32 or packed.size != self.spec.packed_nelem or not packed.flags.c_contiguous or not np.all(np.isfinite(packed)):
      raise ValueError('invalid recurrent input')
    deadline = time.monotonic() + self.timeout
    self.sock.settimeout(self.timeout)
    send_parts(self.sock, REQUEST.pack(frame, int(reset), source_sof), image, packed, deadline=deadline)
    reply = self.reader.receive(self.sock, deadline)
    if len(reply) != REPLY.size + self.spec.output_nbytes:
      raise ValueError('invalid inference reply size')
    fid, *self.timings = REPLY.unpack_from(reply)
    if fid != frame:
      raise ValueError('stale inference reply')
    output = np.frombuffer(reply, np.float32, self.spec.output_nelem, REPLY.size).copy()
    if not np.all(np.isfinite(output)):
      raise ValueError('nonfinite inference reply')
    return output

  def close(self):
    self.sock.close()


class ClientConnection:
  """One bounded handshake, without waiting on the camera inference thread."""
  def __init__(self):
    self.future = Future()
    threading.Thread(target=self._connect, args=(self.future,), name='jetlink-connect', daemon=True).start()

  @staticmethod
  def _connect(future):
    client = None
    try:
      # A thread otherwise inherits modeld's realtime scheduling policy.
      if hasattr(os, 'sched_setscheduler'):
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
      client = Client()
      future.set_result(client)
    except InvalidStateError:
      if client is not None:
        client.close()  # The join opportunity expired during the handshake.
    except Exception as exc:
      try:
        future.set_exception(exc)
      except InvalidStateError:
        pass

  def close(self):
    if not self.future.cancel():
      try:
        client = self.future.result()
      except Exception:
        return
      client.close()
