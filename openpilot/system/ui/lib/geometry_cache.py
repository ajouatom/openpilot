"""Bounded, content-keyed reuse for pure screen-coordinate calculations.

No model timestamps or object identities: in-place changes and all projection
inputs invalidate immediately. Colors, alerts and animation still run each frame.
"""
import os
import struct
from collections import OrderedDict
from functools import wraps

import numpy as np
from openpilot.system.ui.lib import native_draw

ENABLED = os.getenv('CARROT_UI_GEOMETRY_CACHE', '1') != '0'
MAX_ENTRIES = 64
MAX_BYTES = 512 * 1024


def _key(value):
  if isinstance(value, np.ndarray):
    if value.dtype.hasobject or value.nbytes > 64 * 1024:
      raise ValueError('large projection')
    return (value.dtype.str, value.shape, value.tobytes())
  if hasattr(value, 'width') and hasattr(value, 'x'):
    return tuple(_key(v) for v in (value.x, value.y, value.width, value.height))
  if isinstance(value, float):
    return struct.pack('!d', value)
  return value


def cached_projection(function):
  entries = OrderedDict()
  size = 0
  miss_streak = probe = 0

  @wraps(function)
  def calculate(*args, **kwargs):
    nonlocal size, miss_streak, probe
    if not ENABLED:
      return function(*args, **kwargs)
    # Model input normally changes every frame. Do not spend more on key/storage
    # work than the newly batched kernel saves. Probe occasionally to recover
    # reuse when the scene/input stops changing; this never delays fresh output.
    if miss_streak >= MAX_ENTRIES:
      probe = (probe + 1) % 16
      if probe:
        return function(*args, **kwargs)
    try:
      key = (native_draw._ENABLED, id(native_draw._draw_native),
             tuple(_key(a) for a in args), tuple((k, _key(v)) for k, v in sorted(kwargs.items())))
      hit = entries.get(key)
    except (TypeError, ValueError):
      return function(*args, **kwargs)
    if hit is not None:
      miss_streak = probe = 0
      entries.move_to_end(key)
      return hit[0]
    miss_streak += 1
    result = function(*args, **kwargs)
    # Small immutable arrays only; never retain message readers or mutable views.
    cost = result.nbytes + sum(a.nbytes for a in (*args, *kwargs.values()) if isinstance(a, np.ndarray)) + 512
    if cost <= MAX_BYTES:
      result = np.array(result, copy=True)
      result.flags.writeable = False
      entries[key] = (result, cost)
      size += cost
      while len(entries) > MAX_ENTRIES or size > MAX_BYTES:
        _, (_, removed) = entries.popitem(last=False)
        size -= removed
    return result

  def clear():
    nonlocal size, miss_streak, probe
    entries.clear()
    size = 0
    miss_streak = probe = 0

  calculate.clear_cache = clear
  calculate.cache_stats = lambda: {'entries': len(entries), 'bytes': size}
  return calculate
