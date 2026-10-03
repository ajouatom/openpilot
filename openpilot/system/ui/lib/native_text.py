"""C3/C4 styled text batches using the loaded Raylib, with bounded layout reuse."""
import math
import os

import numpy as np

from openpilot.system.ui.lib import native_draw

_CACHE = os.getenv('CARROT_UI_TEXT_CACHE', '1') != '0'
_ENABLED = os.getenv('CARROT_UI_TEXT_NATIVE', '1') != '0'
_OFFSETS = np.array([(math.cos(math.radians(d)), math.sin(math.radians(d))) for d in range(0, 360, 45)], dtype=np.float64)
_FIELDS = {
  'Vector2': ('x', 'y'), 'Color': ('r', 'g', 'b', 'a'), 'Rectangle': ('x', 'y', 'width', 'height'),
  'Texture': ('id', 'width', 'height', 'mipmaps', 'format'), 'Image': ('data', 'width', 'height', 'mipmaps', 'format'),
  'GlyphInfo': ('value', 'offsetX', 'offsetY', 'advanceX', 'image'),
  'Font': ('baseSize', 'glyphCount', 'glyphPadding', 'texture', 'recs', 'glyphs'),
}
_raylib = None
_renderer = None


def active() -> bool:
  return _ENABLED and native_draw._ENABLED and _renderer is not None


def clear():
  if _renderer is not None:
    _renderer.clear()


def stats() -> dict:
  return _renderer.stats() if _renderer is not None else {}


def _backend(rl):
  global _raylib, _renderer
  native = native_draw._draw_native
  if not _ENABLED or not native_draw._ENABLED or native is None or not hasattr(native, 'TextRenderer'):
    return None
  if _raylib is not rl:
    _raylib, _renderer = rl, None
    try:
      # Includes pointer-bearing Font/GlyphInfo/Image layout on both x86/ARM.
      actual = {name: (rl.ffi.sizeof(name), tuple(rl.ffi.offsetof(name, f) for f in fields)) for name, fields in _FIELDS.items()}
      if actual != native.text_abi():
        return None
      addresses = [int(rl.ffi.cast('uintptr_t', rl.ffi.addressof(rl.rl, name)))
                   for name in ('DrawTextEx', 'DrawTexturePro', 'GetGlyphIndex', 'GetCodepointNext')]
      _renderer = native.TextRenderer(*addresses)
    except Exception:
      # CFFI raises its own CDefError for unknown typedefs. This optional ABI
      # probe must leave an unfamiliar binding on the original drawing path.
      return None
  return _renderer


def _rgba(color):
  return (color.r, color.g, color.b, color.a) if hasattr(color, 'r') else color


def try_text(rl, font, text: bytes, x: float, y: float, size: float, border: float, shadow: float,
             color, border_color, shadow_color) -> bool:
  renderer = _backend(rl)
  if renderer is None:
    return False
  address = int(rl.ffi.cast('uintptr_t', rl.ffi.addressof(font)))
  renderer.draw(address, text, x, y, size, _OFFSETS, border, shadow,
                _rgba(color), _rgba(border_color), _rgba(shadow_color), _CACHE)
  return True


def try_plain_text(rl, font, text, position, size, spacing, color) -> bool:
  renderer = _backend(rl)
  if renderer is None or not isinstance(text, (str, bytes)):
    return False
  encoded = text.encode('utf8') if isinstance(text, str) else text
  x, y = (position.x, position.y) if hasattr(position, 'x') else position
  address = int(rl.ffi.cast('uintptr_t', rl.ffi.addressof(font)))
  renderer.draw_plain(address, encoded, x, y, size, spacing, _rgba(color), _CACHE)
  return True
