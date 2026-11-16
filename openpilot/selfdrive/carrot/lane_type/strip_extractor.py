"""Bit-exact batched strip extraction. Only constant maps and scratch storage persist between frames."""
from collections import OrderedDict
import math
import sys

import cv2
import numpy as np

from openpilot.selfdrive.carrot.lane_type import profile_scanner as ps
try:
  from openpilot.selfdrive.carrot.lane_type import _strip_native as native
except ImportError:
  native = None  # optional Cython build (SCons); the numpy/OpenCV path below is the bit-exact reference

# Restart the daemon after changing a switch. Buffer reuse and allocator tuning can be disabled together.
REUSE_BUFFERS = True
TUNE_ALLOCATOR = True
PLANAR_UV = False
SHARE_GEOMETRY = True
NATIVE = True
MAP_CACHE_SIZE = 8
DISTANCES = np.arange(ps.NEAR_M, ps.FAR_M + .001, .02)
COLUMN_GRID = {c: (np.arange(c*ps.SUB) + .5)/(c*ps.SUB) for c in (128,)}  # the lane_strip_v4 strip; other widths computed per call


def tune_allocator():
  """Best effort, glibc/Linux only; a missing/failed mallopt must not prevent inference."""
  if not REUSE_BUFFERS or not TUNE_ALLOCATOR or not sys.platform.startswith('linux'):
    return False
  try:
    import ctypes
    import platform
    if platform.libc_ver()[0] != 'glibc':
      return False
    mallopt = ctypes.CDLL(None).mallopt
    mallopt.argtypes, mallopt.restype = [ctypes.c_int, ctypes.c_int], ctypes.c_int
    results = [mallopt(key, value) for key, value in ((-3, 64 << 20), (-1, 256 << 20), (-2, 16 << 20))]
    return all(results)
  except (AttributeError, OSError, TypeError, ValueError):
    return False


class Extractor:
  def __init__(self, *, buffers=None, planar=None, geometry=None, use_native=None):
    self.buffers = REUSE_BUFFERS if buffers is None else buffers
    self.planar = PLANAR_UV if planar is None else planar
    self.geometry = SHARE_GEOMETRY if geometry is None else geometry
    self.native = native if (NATIVE if use_native is None else use_native) else None
    if planar is None and not getattr(self.native, 'STRIDED_AREA4', False):
      self.planar = True  # Preserve the existing fast path with a missing/older optional binary.
    self.scratch, self.y_maps = {}, OrderedDict()
    self.layout = self.key = self.matrix = None

  def buf(self, tag, shape, dtype=np.uint8):
    if not self.buffers:
      return np.empty(shape, dtype)
    size = math.prod(shape)
    old = self.scratch.get(tag)
    if old is None or old.size < size or old.dtype != dtype:
      old = self.scratch[tag] = np.empty(size, dtype)
    # A linear capacity buffer gives OpenCV a contiguous destination even as the sampled row count shrinks.
    return old[:size].reshape(shape)

  @staticmethod
  def rows(line, matrix, height):
    x, y, z = (np.asarray(line[c], float) for c in 'xyz')
    if len(x) < 3 or not np.isfinite(x).all() or np.any(np.diff(x) <= 0) or x[0] > ps.NEAR_M or x[-1] < ps.FAR_M + 1:
      return None
    lat, zz = np.interp(DISTANCES, x, y), np.interp(DISTANCES, x, z)
    p = np.column_stack((DISTANCES, lat, zz)) @ matrix.T
    if (p[:, 2] <= .1).any():
      return None
    v = p[:, 1]/p[:, 2]
    if np.any(np.diff(v) >= 0):
      return None
    top = max(0, math.ceil(v[-1]))
    bottom = min(math.floor(v[0]), int(height*ps.HOOD_FRACTION) - 1, height - 1)
    if bottom - top < 8:
      return None
    ys = np.arange(top, bottom + 1)
    d = np.interp(ys, v[::-1], DISTANCES[::-1])
    return ys, d, np.interp(d, x, y), np.interp(d, x, z)

  def maps(self, row, matrix, half, cols):
    ys, d, lat, zz = row
    edges = []
    for off in (-half, half):
      q = np.column_stack((d, lat + off, zz)) @ matrix.T
      edges.append(q[:, 0]/q[:, 2])
    t = COLUMN_GRID.get(cols)
    if t is None:
      t = (np.arange(cols*ps.SUB) + .5)/(cols*ps.SUB)
    if self.native is not None:
      mx, mx_uv = np.empty((len(ys), len(t)), np.float32), np.empty((len(ys), len(t)), np.float32)
      self.native.column_map(edges[0], edges[1], t, mx, mx_uv)
    else:
      mx = (edges[0][:, None] + t[None]*(edges[1] - edges[0])[:, None]).astype(np.float32)
      mx_uv = mx*.5
    key = int(ys[0]), int(ys[-1]), cols
    pair = self.y_maps.get(key)
    if pair is None:
      my = np.repeat(ys[:, None].astype(np.float32), cols*ps.SUB, 1)
      pair = (my, my*.5)
      self.y_maps[key] = pair
      if len(self.y_maps) > MAP_CACHE_SIZE:
        self.y_maps.popitem(last=False)
    else:
      self.y_maps.move_to_end(key)
    return {'mx': mx, 'mx_uv': mx_uv, 'my': pair[0], 'my_uv': pair[1]}

  def remap(self, source, mx, my, tag):
    return cv2.remap(source, mx, my, cv2.INTER_LINEAR,
                     dst=self.buf(tag, mx.shape + source.shape[2:]), borderMode=cv2.BORDER_REPLICATE)

  def raw(self, y, uv, geo, tag):
    mx, my = geo['mx'], geo['my']
    lum = self.remap(y, mx, my, tag+'y')
    mx_uv = geo['mx_uv'] if 'mx_uv' in geo else mx*.5
    my_uv = geo['my_uv'] if 'my_uv' in geo else my*.5
    if isinstance(uv, tuple):
      return [lum] + [self.remap(p, mx_uv, my_uv, tag+str(i)) for i, p in enumerate(uv)]
    chroma = self.remap(uv, mx_uv, my_uv, tag+'uv')
    return [lum, chroma[..., 0], chroma[..., 1]]

  def finish(self, planes, cols, rows):
    image = self.buf('image', (len(planes[0]), cols, 3))
    if self.native is not None and ps.SUB == 4 and len(planes) == 3 and all(
        p.dtype == np.uint8 and p.ndim == 2 and p.shape == (len(image), 4*cols)
        and (p.flags.c_contiguous or getattr(self.native, 'STRIDED_AREA4', False)) for p in planes):
      # Exact 4:1 box sum >> 2 == float32 INTER_AREA mean truncated to uint8 (sums of 4 bytes are exact in float32).
      self.native.area4_interleave(*planes, image)
      planes = ()
    for i, plane in enumerate(planes):
      cast = self.buf('float', plane.shape, np.float32)
      np.copyto(cast, plane, casting='unsafe')
      area = cv2.resize(cast, (cols, len(plane)), dst=self.buf('area', (len(plane), cols), np.float32),
                        interpolation=cv2.INTER_AREA)
      # The box mean is bounded by its uint8 source. Assignment truncates exactly as the original float32 -> uint8 cast.
      image[..., i] = area
    image = cv2.resize(image, (cols, rows), interpolation=cv2.INTER_AREA if len(image) > rows else cv2.INTER_LINEAR)
    return image.transpose(2, 0, 1).astype(np.float32)  # caller-owned, never a scratch view

  def extract(self, y, uv, lines, rpy, k, widths=((1., 128, 128),)):
    """Unmirrored strips, width-major then side-major. No line geometry/pixels survive into the next call."""
    layout = y.shape, y.strides, uv.shape, uv.strides, tuple(widths)
    if layout != self.layout:
      self.layout = layout
      self.scratch.clear()
      self.y_maps.clear()
    if self.planar:
      uv = cv2.split(uv, [self.buf('plane0', uv.shape[:2]), self.buf('plane1', uv.shape[:2])])
    if self.geometry:
      key = tuple(rpy), np.asarray(k).tobytes()
      if key != self.key:
        self.key, self.matrix = key, ps.projection(rpy, k)
      common = [self.rows(line, self.matrix, y.shape[0]) for line in lines]
      geos = [[None if row is None else self.maps(row, self.matrix, h, c) for row in common] for h, c, _ in widths]
    else:
      geos = [[ps.strip_geometry(line, rpy, k, y.shape[1], y.shape[0], half_m=h, cols=c) for line in lines] for h, c, _ in widths]
    return [None if g is None else self.finish(self.raw(y, uv, g, 'remap'), cols, rows)
            for group, (_, cols, rows) in zip(geos, widths, strict=True) for g in group]
