"""Optional shared C3/C4 drawing batches, with the original Python fallback."""
import os

import numpy as np

try:
  from openpilot.system.ui.lib import _draw_native
except ImportError:
  _draw_native = None

_ENABLED = os.getenv('CARROT_UI_NATIVE', '1') != '0'
_raylib = None
_addresses: tuple[int, int] | None = None


def active() -> bool:
  return _ENABLED and _draw_native is not None and _addresses is not None


def _functions(rl) -> tuple[int, int] | None:
  global _raylib, _addresses
  if not _ENABLED or _draw_native is None:
    return None
  if _raylib is not rl:
    _raylib, _addresses = rl, None
    try:
      ffi = rl.ffi
      # Function pointers call the loaded binding's library, not another Raylib
      # instance. Verify the two small by-value structs before crossing the ABI.
      if (ffi.sizeof('Vector2'), ffi.sizeof('Color')) != (8, 4):
        return None
      if tuple(ffi.offsetof('Vector2', k) for k in ('x', 'y')) != (0, 4):
        return None
      if tuple(ffi.offsetof('Color', k) for k in ('r', 'g', 'b', 'a')) != (0, 1, 2, 3):
        return None
      _addresses = tuple(int(ffi.cast('uintptr_t', ffi.addressof(rl.rl, name)))
                         for name in ('DrawLineEx', 'DrawTriangleStrip'))
    except (AttributeError, TypeError, ValueError):
      return None
  return _addresses


def try_outline(rl, points: np.ndarray, color, width: float) -> bool:
  functions = _functions(rl)
  if functions is None:
    return False
  pts = np.ascontiguousarray(points, dtype=np.float32)
  rgba = (color.r, color.g, color.b, color.a) if hasattr(color, 'r') else color
  _draw_native.outline(pts, functions[0], width, *rgba)
  return True


def try_ribbon(rl, points: np.ndarray, color) -> bool:
  functions = _functions(rl)
  if functions is None:
    return False
  rgba = (color.r, color.g, color.b, color.a) if hasattr(color, 'r') else color
  _draw_native.ribbon(points, functions[1], *rgba)
  return True
