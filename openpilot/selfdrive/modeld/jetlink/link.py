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

from openpilot.selfdrive.modeld.jetlink import shared_input
from openpilot.selfdrive.modeld.jetlink import VENDOR  # noqa: F401
from openpilot.common.jetlink_status import badge  # noqa: F401
from jetlink.spec import ModelSpec
from jetlink import output_projection as projection, deferred_map
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


def validate_spec(spec, expected=None):
  expected = SPEC if expected is None else expected
  if spec.to_dict() != expected.to_dict():
    raise ValueError('Jetlink model identity or input/output contract mismatch')


def fault_active():
  try:
    return 0 <= time.monotonic() - float(FAULT.read_text()) < 5
  except (OSError, ValueError):
    return False


class Client:
  def __init__(self, path=SOCKET, timeout=.15):
    self.timeout = timeout
    self.shared = None
    self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    self.sock.settimeout(timeout)
    try:
      self.sock.connect(path)
      hello = json.loads(receive(self.sock, time.monotonic() + timeout))
      self.spec = validate_model_spec(ModelSpec.from_dict(hello['spec']))
      self.project_output = (os.getenv('JETLINK_COMPACT_REPLY', '1') != '0' and
                             not os.getenv('SEND_RAW_PRED') and projection.eligible(self.spec) and
                             projection.matches(hello.get(projection.IPC_CAPABILITY)))
      self.deferred_nav = (os.getenv('JETLINK_DEFER_NAV', '1') == '1' and projection.eligible(self.spec) and
                           deferred_map.matches(hello.get(deferred_map.IPC_CAPABILITY)))
      if os.getenv('JETLINK_RAM_INPUT', '1') == '1' and shared_input.supported() and hello.get(shared_input.CAPABILITY) is True:
        deadline = time.monotonic() + timeout
        send_parts(self.sock, shared_input.SETUP, deadline=deadline)
        self.shared = shared_input.Writer.receive(self.sock, self.spec, receive, deadline)
    except Exception:
      self.sock.close()
      raise
    self.timings = (0, 0, 0)
    self.reader = PacketReader(REPLY.size + self.spec.output_nbytes)

  def image_buffer(self):
    return self.shared.begin() if self.shared is not None else None

  def infer(self, image, packed, frame, reset=False, source_sof=0, nav_metadata=None):
    try:
      return self._infer(image, packed, frame, reset, source_sof, nav_metadata)
    except Exception:
      # Neither late replies nor late USB readers may outlive a reused slot.
      self.close()
      raise

  def _infer(self, image, packed, frame, reset=False, source_sof=0, nav_metadata=None):
    if image.dtype != np.uint8 or image.shape != self.spec.warped_shape or not image.flags.c_contiguous:
      raise ValueError('invalid warped image')
    if packed.dtype != np.float32 or packed.size != self.spec.packed_nelem or not packed.flags.c_contiguous or not np.all(np.isfinite(packed)):
      raise ValueError('invalid recurrent input')
    deadline = time.monotonic() + self.timeout
    self.sock.settimeout(self.timeout)
    request_flags = int(reset) | (projection.IPC_FLAG if self.project_output else 0)
    map_parts = ()
    if getattr(self, 'deferred_nav', False):
      request_flags |= deferred_map.IPC_FLAG
      map_parts = (deferred_map.META.pack(*deferred_map.metadata(nav_metadata)),)
    shared_request = self.shared is not None and self.shared.busy
    if shared_request:
      token = self.shared.token(image, packed, frame, request_flags, source_sof)
      send_parts(self.sock, token, *map_parts, deadline=deadline)
    else:
      send_parts(self.sock, REQUEST.pack(frame, request_flags, source_sof), image, packed, *map_parts, deadline=deadline)
    reply = self.reader.receive(self.sock, deadline)
    count = projection.COUNT if self.project_output else self.spec.output_nelem
    if len(reply) != REPLY.size + count * 4:
      raise ValueError('invalid inference reply size')
    fid, *self.timings = REPLY.unpack_from(reply)
    if fid != frame:
      raise ValueError('stale inference reply')
    output = np.frombuffer(reply, np.float32, count, REPLY.size).copy()
    if not np.all(np.isfinite(output)):
      raise ValueError('nonfinite inference reply')
    if shared_request:
      self.shared.complete()
    return output

  def close(self):
    self.sock.close()
    if self.shared is not None:
      self.shared.close()


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
