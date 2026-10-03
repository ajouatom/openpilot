# cython: language_level=3, boundscheck=False, wraparound=False
# distutils: language=c++
"""Same Raylib primitives/order, with per-vertex work kept across one boundary."""
from libc.stdint cimport uintptr_t
from libc.limits cimport INT_MAX
from libcpp.vector cimport vector
from libc.math cimport fabs, INFINITY
from libc.string cimport memmove
import numpy as np

ctypedef fused coordinate:
  float
  double


def offset_sides(const coordinate[:, :] points, float left_y, float right_y, float left_z, float right_z):
  """Keep the original float32 offsets, including for float64 input points."""
  cdef Py_ssize_t i, n = points.shape[0]
  if points.shape[1] != 3:
    raise ValueError('expected (N,3) points')
  out = np.empty((2, n, 3), dtype=np.float32 if coordinate is float else np.float64)
  cdef coordinate[:, :, ::1] result = out
  for i in range(n):
    result[0, i, 0] = points[i, 0] + <coordinate>0.
    result[1, i, 0] = points[i, 0] + <coordinate>0.
    result[0, i, 1] = points[i, 1] + left_y
    result[1, i, 1] = points[i, 1] + right_y
    result[0, i, 2] = points[i, 2] + left_z
    result[1, i, 2] = points[i, 2] + right_z
  return out


def path_sides(const double[:, :] points, const double[:] y_offset, const double[:] z_offset):
  cdef Py_ssize_t i, n = points.shape[0]
  if points.shape[1] != 3 or y_offset.shape[0] != n or z_offset.shape[0] != n:
    raise ValueError('expected (N,3) points and N offsets')
  out = np.empty((2, n, 3), dtype=np.float64)
  cdef double[:, :, ::1] result = out
  for i in range(n):
    result[0, i, 0] = points[i, 0]
    result[1, i, 0] = points[i, 0]
    result[0, i, 1] = points[i, 1] - y_offset[i]
    result[1, i, 1] = points[i, 1] + y_offset[i]
    result[0, i, 2] = points[i, 2] + z_offset[i]
    result[1, i, 2] = points[i, 2] + z_offset[i]
  return out


cdef _clip_ribbon(const coordinate[:, :, :] projected, coordinate x_min, coordinate x_max,
                  coordinate y_min, coordinate y_max, bint allow_invert, const coordinate[:] forward):
  """Divide, clip both sides and compact in the original vertex order.

  NumPy retains ownership of matrix products: BLAS rounding at clip/hill
  boundaries must not change when the optional native backend is enabled.
  Accept strided views so path, tire, lane and barrier projections share this.
  """
  cdef Py_ssize_t i, n = projected.shape[2], count = 0
  cdef coordinate lz, rz, lx, ly, rx, ry, min_y = INFINITY
  cdef coordinate epsilon = 1e-6
  if projected.shape[0] != 3 or projected.shape[1] != 2:
    raise ValueError('expected (3,2,N) projected coordinates')
  out = np.empty((2*n, 2), dtype=np.float32)
  cdef float[:, ::1] result = out
  for i in range(n):
    if forward is not None and not forward[i] >= 0:
      continue
    lz = projected[2, 0, i]
    rz = projected[2, 1, i]
    if not (fabs(lz) >= epsilon and fabs(rz) >= epsilon):
      continue
    lx = projected[0, 0, i] / lz
    ly = projected[1, 0, i] / lz
    rx = projected[0, 1, i] / rz
    ry = projected[1, 1, i] / rz
    if not (x_min <= lx <= x_max and x_min <= rx <= x_max and y_min <= ly <= y_max and y_min <= ry <= y_max):
      continue
    if not allow_invert and ly > min_y:
      continue
    min_y = ly
    result[count, 0] = lx
    result[count, 1] = ly
    result[2*n-1-count, 0] = rx
    result[2*n-1-count, 1] = ry
    count += 1
  if count:
    memmove(&result[count, 0], &result[2*n-count, 0], count * 2 * sizeof(float))
  return out[:2*count]


def clip_ribbon(const coordinate[:, :, :] projected, coordinate x_min, coordinate x_max,
                coordinate y_min, coordinate y_max, bint allow_invert=True):
  return _clip_ribbon[coordinate](projected, x_min, x_max, y_min, y_max, allow_invert, None)


def clip_dashes(const coordinate[:, :, :] projected, const coordinate[:] forward, const long long[:] lengths,
                coordinate x_min, coordinate x_max, coordinate y_min, coordinate y_max):
  cdef Py_ssize_t i, start = 0, end = 0, n = projected.shape[2]
  if projected.shape[0] != 3 or projected.shape[1] != 2 or forward.shape[0] != n:
    raise ValueError('expected (3,2,N) projections and N forward coordinates')
  for i in range(lengths.shape[0]):
    if lengths[i] < 0 or lengths[i] > n - end:
      raise ValueError('invalid dash lengths')
    end += lengths[i]
  if end != n:
    raise ValueError('dash lengths must cover the projected points')
  result = []
  for i in range(lengths.shape[0]):
    end = start + lengths[i]
    polygon = _clip_ribbon(projected[:, :, start:end], x_min, x_max, y_min, y_max, True, forward[start:end])
    if polygon.size:
      result.append(polygon)
    start = end
  return result

cdef extern from *:
  """
  struct CarrotVector2 { float x, y; };
  struct CarrotColor { unsigned char r, g, b, a; };
  typedef void (*CarrotLine)(CarrotVector2, CarrotVector2, float, CarrotColor);
  typedef void (*CarrotStrip)(const CarrotVector2*, int, CarrotColor);
  """
  cdef struct CarrotVector2:
    float x
    float y
  cdef struct CarrotColor:
    unsigned char r, g, b, a
  ctypedef void (*CarrotLine)(CarrotVector2, CarrotVector2, float, CarrotColor) noexcept
  ctypedef void (*CarrotStrip)(const CarrotVector2*, int, CarrotColor) noexcept


def outline(const float[:, ::1] points, uintptr_t address, float width,
            unsigned char r, unsigned char g, unsigned char b, unsigned char a):
  cdef Py_ssize_t i, n = points.shape[0]
  cdef CarrotVector2 p, q
  cdef CarrotColor color
  cdef CarrotLine draw = <CarrotLine>address
  if points.shape[1] != 2 or address == 0:
    raise ValueError('expected (N,2) points and a valid Raylib function')
  if n < 2:
    return
  color.r = r; color.g = g; color.b = b; color.a = a
  for i in range(n):
    p.x = points[i, 0]; p.y = points[i, 1]
    q.x = points[(i + 1) % n, 0]; q.y = points[(i + 1) % n, 1]
    draw(p, q, width, color)


def ribbon(const float[:, ::1] points, uintptr_t address,
           unsigned char r, unsigned char g, unsigned char b, unsigned char a):
  cdef Py_ssize_t i, n = (points.shape[0] // 2) * 2
  cdef vector[CarrotVector2] strip
  cdef CarrotColor color
  cdef CarrotStrip draw = <CarrotStrip>address
  if points.shape[1] != 2 or address == 0 or n > INT_MAX:
    raise ValueError('expected (N,2) points and a valid Raylib function')
  if n == 0:
    return
  strip.resize(n)
  for i in range(n // 2):
    strip[2*i].x = points[i, 0]; strip[2*i].y = points[i, 1]
    strip[2*i+1].x = points[n-i-1, 0]; strip[2*i+1].y = points[n-i-1, 1]
  color.r = r; color.g = g; color.b = b; color.a = a
  draw(&strip[0], <int>n, color)
