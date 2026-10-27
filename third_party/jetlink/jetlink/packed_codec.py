"""Negotiated lossless transport of integer navigation-map pixels in v2."""
import numpy as np

CAPABILITY = 'carrot_nav_u8_v1'
FLAG = 1 << 3
MAP_PIXELS = 256 * 256


def _layout(spec):
  if not spec.stateful or spec.packed_shapes.get('nav_map') != (1, 1, 256, 256):
    raise ValueError('Unsupported compact navigation contract')
  offset = 0
  for name, size in zip(spec.packed_shapes, spec.packed_sizes, strict=True):
    if name == 'nav_map':
      return offset * 4
    offset += size
  raise ValueError('Missing navigation map')


def encode(packed, spec):
  raw = memoryview(packed).cast('B')
  offset = _layout(spec)
  if raw.nbytes != spec.packed_nbytes:
    raise ValueError('Invalid navigation context size')
  pixels = np.frombuffer(raw, np.float32, MAP_PIXELS, offset)
  if not np.isfinite(pixels).all() or np.any(pixels < 0) or np.any(pixels > 255):
    return raw, False
  compact = pixels.astype(np.uint8)
  # Preserve every bit, including negative zero; fractional/non-canonical
  # inputs use the original float32 packet. No quantization or model change.
  if not np.array_equal(compact.astype(np.float32).view(np.uint32), pixels.view(np.uint32)):
    return raw, False
  return bytes(raw[:offset]) + compact.tobytes() + bytes(raw[offset + MAP_PIXELS * 4:]), True


def decode(payload, spec):
  offset = _layout(spec)
  if len(payload) != spec.packed_nbytes - MAP_PIXELS * 3:
    raise ValueError('Invalid compact navigation context size')
  pixels = np.frombuffer(payload, np.uint8, MAP_PIXELS, offset).astype(np.float32)
  return bytes(payload[:offset]) + pixels.tobytes() + bytes(payload[offset + MAP_PIXELS:])
