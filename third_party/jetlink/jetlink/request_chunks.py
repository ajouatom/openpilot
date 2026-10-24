"""Negotiated, bounded request envelopes: each fits one small FunctionFS write.

Whole USB request replays remain whole framed messages, never bytes inserted
inside an inference payload. The original sequence is committed only once the
entire checked request is assembled. This does not pipeline model frames.
"""
import struct
import time
import zlib

from jetlink import protocol as P
from jetlink.transport.base import LinkError, LinkTimeout, Message

CAPABILITY = 'carrot_request_chunks_v1'
MESSAGE = 0x4020
WIRE_BYTES = 32 * 1024
HEADER = struct.Struct('<HHIII')  # original kind, flags, total, offset, crc32
DATA_BYTES = WIRE_BYTES - P.HEADER_SIZE - HEADER.size
MAX_BYTES = 1 << 20
MAX_AGE = .5
KINDS = frozenset((int(P.Msg.INFER_REQ), 0x4000, 0x4001, 0x4002))


def descriptor():
  return {'version': 1, 'wire_bytes': WIRE_BYTES, 'max_bytes': MAX_BYTES}


def supported(value):
  return (isinstance(value, dict) and value == descriptor() and
          all(type(value.get(k)) is int for k in descriptor()))


def send(send_message, kind, seq, parts, flags=0, timeout=None):
  raw = b''.join(memoryview(p).cast('B') for p in parts)
  if kind not in KINDS or not 0 < len(raw) <= MAX_BYTES or flags != 0:
    raise LinkError('unsupported chunked request')
  checksum = zlib.crc32(raw)
  deadline = time.monotonic() + (MAX_AGE if timeout is None else min(timeout, MAX_AGE))
  view = memoryview(raw)
  for offset in range(0, len(raw), DATA_BYTES):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
      raise LinkTimeout('chunked request send deadline exceeded')
    send_message(MESSAGE, seq, (HEADER.pack(kind, flags, len(raw), offset, checksum),
                               view[offset:offset + DATA_BYTES]), timeout=remaining)


def packets(kind, seq, parts, flags=0):
  """Exact protocol-2 gadget frames for the native same-request queue."""
  raw = b''.join(memoryview(p).cast('B') for p in parts)
  if kind not in KINDS or not 0 < len(raw) <= MAX_BYTES or flags != 0:
    raise LinkError('unsupported chunked request')
  checksum = zlib.crc32(raw)
  view = memoryview(raw)
  result = []
  for offset in range(0, len(raw), DATA_BYTES):
    body = view[offset:offset + DATA_BYTES]
    length = HEADER.size + len(body)
    pad = -(P.HEADER_SIZE + length) % P.GADGET_TX_ALIGN
    result.append(b''.join((P.pack_header(MESSAGE, seq, length, version=2),
                           HEADER.pack(kind, flags, len(raw), offset, checksum), body, bytes(pad))))
  return result


class Receiver:
  def __init__(self):
    self.clear()

  def clear(self):
    self.key = None
    self.data = None
    self.offset = 0
    self.started = 0.

  def accept(self, msg, last_seq):
    # A final-envelope replay after execution must never run inference again.
    if msg.seq <= last_seq:
      return None
    try:
      return self._accept(msg)
    except (ValueError, struct.error):
      self.clear()
      raise LinkError('invalid or expired chunked request') from None

  def _accept(self, msg):
    if msg.flags or not HEADER.size < len(msg.payload) <= HEADER.size + DATA_BYTES:
      raise ValueError('envelope')
    kind, flags, total, offset, checksum = HEADER.unpack_from(msg.payload)
    body = msg.payload[HEADER.size:]
    if (kind not in KINDS or flags or not 0 < total <= MAX_BYTES or
        offset % DATA_BYTES or offset >= total or len(body) != min(DATA_BYTES, total - offset)):
      raise ValueError('layout')
    key = (msg.seq, kind, flags, total, checksum)
    now = time.monotonic()
    if self.key is None:
      if offset:
        raise ValueError('missing first envelope')
      self.key, self.data, self.started = key, bytearray(total), now
    if key != self.key or now - self.started > MAX_AGE or now < self.started:
      raise ValueError('interleaved or stale request')
    if offset < self.offset:
      if self.data[offset:offset + len(body)] != body:
        raise ValueError('changed replay')
      return None
    if offset != self.offset:
      raise ValueError('missing envelope')
    self.data[offset:offset + len(body)] = body
    self.offset += len(body)
    if self.offset != total:
      return None
    if zlib.crc32(self.data) != checksum:
      raise ValueError('checksum')
    result = Message(kind, msg.seq, flags, memoryview(self.data))
    self.clear()
    return result
