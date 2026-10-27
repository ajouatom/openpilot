"""Connection-owned, synchronous RAM input slot. No pathname or cross-session reuse.

The daemon allocates a size-sealed memfd after explicit negotiation. One matching
inference reply releases the writer's slot. Any uncertain outcome retires the
entire connection: a late USB consumer still sees the old, unchanged mapping.
"""
import array
import mmap
import os
import secrets
import socket
import struct
import time

import numpy as np

CAPABILITY = 'ram_input_v1'
SETUP = b'JETLINK_RAM_INPUT_V1'
TOKEN = struct.Struct('<8s16sQQIIQ')
MAGIC = b'JLRAM001'
MAX_BYTES = 1 << 20


def supported():
  return hasattr(os, 'memfd_create') and hasattr(socket.socket, 'recvmsg')


def size_for(spec):
  size = spec.warped_nbytes + spec.packed_nbytes
  if not 0 < size <= MAX_BYTES or spec.warped_nbytes % 4:
    raise ValueError('invalid shared input size')
  return size


def _seals():
  import fcntl
  return fcntl.F_SEAL_GROW | fcntl.F_SEAL_SHRINK | fcntl.F_SEAL_SEAL


class Mapping:
  def __init__(self, mapping, spec, nonce):
    self.mapping, self.spec, self.nonce = mapping, spec, nonce
    self.image = np.frombuffer(mapping, np.uint8, spec.warped_nbytes).reshape(spec.warped_shape)
    self.packed = np.frombuffer(mapping, np.float32, spec.packed_nelem, offset=spec.warped_nbytes)
    self.generation = 0
    self.closed = False

  def close(self):
    self.closed = True
    self.image = self.packed = None
    mapping, self.mapping = self.mapping, None
    if mapping is not None:
      try:
        mapping.close()
      except BufferError:
        # Exported NumPy/memoryview leases keep only this retired mapping alive.
        # Its fd is already closed; it is never reused by another connection.
        pass


class Reader(Mapping):
  @classmethod
  def offer(cls, sock, spec):
    import fcntl
    fd = os.memfd_create('jetlink-input', os.MFD_CLOEXEC | os.MFD_ALLOW_SEALING)
    mapping = None
    try:
      os.ftruncate(fd, size_for(spec))
      fcntl.fcntl(fd, fcntl.F_ADD_SEALS, _seals())
      # Only the model process writes; daemon views cannot accidentally mutate it.
      mapping = mmap.mmap(fd, size_for(spec), access=mmap.ACCESS_READ)
      nonce = secrets.token_bytes(16)
      if sock.sendmsg([nonce[:1]], [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array.array('i', [fd]))]) != 1:
        raise ConnectionError('shared input descriptor send failed')
      # Remaining nonce is an ordinary bounded packet handled by the caller.
      return cls(mapping, spec, nonce)
    except BaseException:
      if mapping is not None:
        mapping.close()
      raise
    finally:
      os.close(fd)

  def read(self, token):
    if self.closed or len(token) != TOKEN.size:
      raise ValueError('invalid shared input token')
    magic, nonce, generation, ready, frame, flags, source_sof = TOKEN.unpack(token)
    age = time.monotonic_ns() - ready
    if magic != MAGIC or nonce != self.nonce or generation != self.generation + 1 or not 0 <= age <= 150_000_000:
      raise ValueError('stale or foreign shared input')
    self.generation = generation
    return frame, flags, source_sof, self.image, self.packed


class Writer(Mapping):
  @classmethod
  def receive(cls, sock, spec, receive_packet, deadline):
    import fcntl
    fds = []
    mapping = None
    try:
      sock.settimeout(max(0.000001, deadline - time.monotonic()))
      first, ancillary, flags, _ = sock.recvmsg(1, socket.CMSG_SPACE(16 * array.array('i').itemsize), socket.MSG_CMSG_CLOEXEC)
      unknown = False
      for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == socket.SCM_RIGHTS:
          received = array.array('i')
          received.frombytes(data[:len(data) - len(data) % received.itemsize])
          fds.extend(received)
        else:
          unknown = True
      if len(first) != 1 or flags & (socket.MSG_CTRUNC | socket.MSG_TRUNC) or unknown or len(fds) != 1:
        raise ValueError('invalid shared input descriptor')
      fd = fds[0]
      if os.fstat(fd).st_size != size_for(spec) or fcntl.fcntl(fd, fcntl.F_GET_SEALS) & _seals() != _seals():
        raise ValueError('unsealed or incorrectly sized shared input')
      nonce = bytes(receive_packet(sock, deadline))
      if len(nonce) != 16 or nonce[:1] != first:
        raise ValueError('invalid shared input session')
      mapping = mmap.mmap(fd, size_for(spec), access=mmap.ACCESS_WRITE)
      result = cls(mapping, spec, nonce)
      result.busy = False
      return result
    except BaseException:
      if mapping is not None:
        mapping.close()
      raise
    finally:
      for fd in fds:
        os.close(fd)

  def begin(self):
    if self.closed or self.busy:
      raise ConnectionError('shared input unavailable or already in flight')
    self.busy = True
    return self.image

  def token(self, image, packed, frame, flags, source_sof):
    if self.closed or not self.busy or image is not self.image:
      raise ValueError('shared input does not own this frame')
    np.copyto(self.packed, packed)
    self.generation += 1
    return TOKEN.pack(MAGIC, self.nonce, self.generation, time.monotonic_ns(), frame, flags, source_sof)

  def complete(self):
    if self.closed or not self.busy:
      raise ConnectionError('shared input completion without a request')
    self.busy = False
