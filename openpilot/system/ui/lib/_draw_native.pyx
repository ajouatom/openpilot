# cython: language_level=3, boundscheck=False, wraparound=False
# distutils: language=c++
"""Same Raylib primitives/order, with per-vertex work kept across one boundary."""
from libc.stdint cimport uintptr_t
from libc.limits cimport INT_MAX
from libcpp.vector cimport vector

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
