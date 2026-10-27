"""Optional system LZ4 blocks; exact bytes, bounded allocation and raw fallback."""
import ctypes
import ctypes.util
from functools import lru_cache

CAPABILITY = 'carrot_lz4_v1'
FLAG = 1 << 4
MAX_BYTES = 2 * 1024 * 1024


@lru_cache(maxsize=1)
def library():
  try:
    name = ctypes.util.find_library('lz4')
    if name is None:
      return None
    lib = ctypes.CDLL(name)
    lib.LZ4_compressBound.argtypes = [ctypes.c_int]
    lib.LZ4_compressBound.restype = ctypes.c_int
    lib.LZ4_compress_default.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.LZ4_compress_default.restype = ctypes.c_int
    lib.LZ4_decompress_safe.argtypes = [ctypes.c_char_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.LZ4_decompress_safe.restype = ctypes.c_int
    return lib
  except (OSError, AttributeError):
    return None


def encode(parts):
  size = sum(memoryview(p).nbytes for p in parts)
  lib = library()
  if lib is None or not 0 < size <= MAX_BYTES:
    return None
  raw = b''.join(parts)
  capacity = lib.LZ4_compressBound(size)
  dest = ctypes.create_string_buffer(capacity)
  written = lib.LZ4_compress_default(raw, dest, size, capacity)
  # Avoid adding wire framing/decoding work when bytes barely shrink.
  return dest.raw[:written] if 0 < written < size - 1024 else None


def decode(payload, expected):
  lib = library()
  if lib is None or not 0 < expected <= MAX_BYTES:
    raise ValueError('Unsupported LZ4 frame')
  if not 0 < len(payload) <= lib.LZ4_compressBound(expected):
    raise ValueError('Invalid LZ4 frame size')
  dest = ctypes.create_string_buffer(expected)
  written = lib.LZ4_decompress_safe(bytes(payload), dest, len(payload), expected)
  if written != expected:
    raise ValueError('Invalid LZ4 frame contents')
  return dest.raw
