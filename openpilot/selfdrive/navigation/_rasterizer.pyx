# cython: boundscheck=False, wraparound=False, cdivision=True
# cython: language_level=3
# distutils: language=c++
"""Analytic stroke rasterization with the reference renderer's rounding/order."""
from libc.math cimport floor, ceil, hypot, nearbyint, fabs


def draw_segments(unsigned char[:, ::1] image, const double[:, ::1] starts,
                  const double[:, ::1] ends, const double[::1] radii,
                  int intensity, bint overlay, const unsigned char[:, ::1] caps):
  cdef Py_ssize_t i, x, y, x0, x1, y0, y1, row0, row1
  cdef double sx, sy, ex, ey, dx, dy, denom, px, py, along, t, ax, ay, alpha, reach, radius
  cdef double ta, tb, tx0, tx1
  cdef unsigned char value
  if starts.shape[0] != ends.shape[0] or starts.shape[0] != radii.shape[0] or starts.shape[0] != caps.shape[0]:
    raise ValueError('Inconsistent segment count')
  if starts.shape[1] != 2 or ends.shape[1] != 2 or caps.shape[1] != 2:
    raise ValueError('Expected two coordinates/caps per segment')
  with nogil:
    for i in range(starts.shape[0]):
      sx, sy, ex, ey = starts[i, 0], starts[i, 1], ends[i, 0], ends[i, 1]
      radius = radii[i]
      reach = radius + 1
      x0 = <Py_ssize_t>max(floor(min(sx, ex) - reach), 0)
      x1 = <Py_ssize_t>min(ceil(max(sx, ex) + reach), image.shape[1] - 1)
      y0 = <Py_ssize_t>max(floor(min(sy, ey) - reach), 0)
      y1 = <Py_ssize_t>min(ceil(max(sy, ey) + reach), image.shape[0] - 1)
      dx, dy = ex - sx, ey - sy
      denom = max(dx * dx + dy * dy, 1e-12)
      for y in range(y0, y1 + 1):
        py = y + (0.5 - sy)
        row0, row1 = x0, x1
        if fabs(dy) > 1e-12:
          # Intersect this row's expanded band with the segment before visiting
          # pixels. Keep an extra pixel around this conservative capsule bound.
          ta, tb = (py - reach) / dy, (py + reach) / dy
          tx0 = sx + min(max(min(ta, tb), 0), 1) * dx
          tx1 = sx + min(max(max(ta, tb), 0), 1) * dx
          row0 = max(x0, <Py_ssize_t>floor(min(tx0, tx1) - reach - 1))
          row1 = min(x1, <Py_ssize_t>ceil(max(tx0, tx1) + reach + 1))
        for x in range(row0, row1 + 1):
          px = x + (0.5 - sx)
          along = (px * dx + py * dy) / denom
          if (not caps[i, 0] and along < 0) or (not caps[i, 1] and along > 1):
            continue
          t = min(max(along, 0), 1)
          ax, ay = px - t * dx, py - t * dy
          # Most pixels in a diagonal segment's bounding box are far outside.
          # A generous bound avoids changing pixels at the analytic edge.
          if ax * ax + ay * ay > (radius + 1) * (radius + 1):
            continue
          alpha = min(max(radius + 0.5 - hypot(ax, ay), 0), 1)
          if overlay:
            image[y, x] = <unsigned char>nearbyint(image[y, x] * (1 - alpha) + intensity * alpha)
          else:
            value = <unsigned char>nearbyint(alpha * intensity)
            if value > image[y, x]:
              image[y, x] = value
