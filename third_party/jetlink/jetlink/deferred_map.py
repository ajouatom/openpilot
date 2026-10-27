"""Negotiated N16 map staging during the preceding inference, bounded by age.

Only the map image is staged. Camera, navigation vector and recurrent controls
remain current. Missing/changed routes, resets and gaps use the inline input.
"""
import struct
import time
import zlib

from jetlink import output_projection, packed_codec

CAPABILITY = 'carrot_n16_deferred_map_v1'
IPC_CAPABILITY = 'carrot_n16_deferred_map_ipc_v1'
IPC_FLAG = 4
FLAG = 64
MESSAGE = 0x4021
META = struct.Struct('<QQQ')  # route identity, source image monotonic ns, expiry ns
REF = struct.Struct('<QQ')  # map message sequence, route identity
UPDATE = struct.Struct('<QQII')  # map sequence, route, conservative TTL ms, CRC32
MAX_TTL_MS = 350
TRANSIT_RESERVE_NS = 150_000_000
USE_RESERVE_NS = 50_000_000


def descriptor():
  return {'version': 1, 'model': output_projection.MODEL_SHA256,
          'spec': output_projection.SPEC_SHA256, 'pixels': 65536, 'max_ttl_ms': MAX_TTL_MS}


def matches(value):
  try:
    return type(value) is dict and output_projection.canonical(value) == output_projection.canonical(descriptor())
  except (TypeError, ValueError):
    return False


def metadata(value, now=None):
  if value is None:
    return (0, 0, 0)
  if len(value) != 3 or any(type(v) is not int or not 0 <= v < 1 << 64 for v in value):
    raise ValueError('invalid map metadata')
  route, image, expiry = value
  if (route, image, expiry) == (0, 0, 0):
    return (0, 0, 0)
  now = time.monotonic_ns() if now is None else now
  if not route or not 0 < image <= now or not image < expiry <= image + 500_000_000:
    raise ValueError('invalid map lifetime')
  return (route, image, expiry) if expiry > now else (0, 0, 0)


class Sender:
  def __init__(self, spec, *, clock=time.monotonic_ns):
    if not output_projection.eligible(spec):
      raise ValueError('deferred map requires the exact N16 contract')
    self.spec, self.clock = spec, clock
    self.offset = packed_codec._layout(spec)
    self.clear()

  def clear(self):
    self.cached = None
    self.pending = None
    self.previous_frame = None
    self.used = False

  def prepare(self, packed, frame, reset, value):
    now = self.clock()
    meta = metadata(value, now)
    route, image, expiry = meta
    raw = memoryview(packed).cast('B')
    if raw.nbytes != self.spec.packed_nbytes:
      raise ValueError('invalid deferred context size')
    if (reset or not route or self.previous_frame is None or
        frame != (self.previous_frame + 1) & 0xffffffff or
        (self.cached and (self.cached['meta'][0] != route or self.cached['meta'][1] > image))):
      self.cached = None
    self.previous_frame = frame
    self.pending = None
    self.used = bool(self.cached and now + USE_RESERVE_NS < min(self.cached['until'], expiry - TRANSIT_RESERVE_NS))
    if self.used:
      # Hold this synchronous input slot until flush(), after camera submission.
      # Do not convert the new image before the GPU has begun this frame.
      if self.cached['meta'][:2] != meta[:2]:
        self.pending = (raw, meta, False)
      reference = REF.pack(self.cached['sequence'], route)
      return bytes(raw[:self.offset]) + reference + bytes(raw[self.offset + 65536 * 4:]), FLAG | packed_codec.FLAG
    compact, encoded = packed_codec.encode(raw, self.spec)
    self.cached = None
    if route and encoded:
      self.pending = (compact, meta, True)
    return compact, packed_codec.FLAG if encoded else 0

  def flush(self, next_seq, send, timeout):
    pending, self.pending = self.pending, None
    if pending is None:
      return 0
    packed, meta, encoded = pending
    if not encoded:
      packed, encoded = packed_codec.encode(packed, self.spec)
      if not encoded:
        # No response for this frame may reach modeld after a rejected input.
        raise ValueError('noncanonical deferred map input')
    now = self.clock()
    ttl = min(MAX_TTL_MS, (meta[2] - now - TRANSIT_RESERVE_NS) // 1_000_000)
    if ttl <= USE_RESERVE_NS // 1_000_000:
      self.cached = None
      return 0
    pixels = bytes(packed[self.offset:self.offset + 65536])
    sequence = next_seq()
    body = UPDATE.pack(sequence, meta[0], ttl, zlib.crc32(pixels)) + pixels
    send(MESSAGE, sequence, (body,), timeout=timeout)
    # Starting the TTL before send is stricter than the receiver's arrival time.
    self.cached = {'sequence':sequence, 'meta':meta, 'until':now + ttl * 1_000_000}
    return len(body)


class Receiver:
  def __init__(self, *, clock=time.monotonic_ns):
    self.clock = clock
    self.clear()

  def clear(self):
    self.cached = None

  def accept(self, sequence, payload):
    if len(payload) != UPDATE.size + 65536:
      raise ValueError('invalid map update size')
    key, route, ttl, crc = UPDATE.unpack_from(payload)
    pixels = bytes(payload[UPDATE.size:])
    if not key or key != sequence or not route or not 0 < ttl <= MAX_TTL_MS or zlib.crc32(pixels) != crc:
      raise ValueError('invalid map update identity or lifetime')
    self.cached = (key, route, self.clock() + ttl * 1_000_000, pixels)

  def expand(self, packed, spec):
    if not output_projection.eligible(spec):
      raise ValueError('deferred map model changed')
    offset = packed_codec._layout(spec)
    if len(packed) != spec.packed_nbytes - 65536 * 4 + REF.size:
      raise ValueError('invalid deferred reference size')
    key, route = REF.unpack_from(packed, offset)
    cached = self.cached
    if cached is None or (key, route) != cached[:2] or self.clock() >= cached[2]:
      raise ValueError('missing or expired deferred map')
    return bytes(packed[:offset]) + cached[3] + bytes(packed[offset + REF.size:])
