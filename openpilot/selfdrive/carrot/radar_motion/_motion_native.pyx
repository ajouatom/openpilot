# cython: language_level=3
# distutils: language=c++
"""Equivalent double-precision loops; policy and history remain in Python."""
from libc.math cimport fabs, isfinite
from libcpp.vector cimport vector
from libcpp.algorithm cimport stable_sort


cdef double median(vector[double]& values):
  cdef size_t n = values.size()
  if n == 0:
    return 0.0
  stable_sort(values.begin(), values.end())
  if n % 2:
    return values[n // 2]
  return (values[n // 2 - 1] + values[n // 2]) / 2.0


def motion(object observations, double window_s, str attribute, bint metrics,
           double jitter_travel=0.45, double jitter_fraction=0.50):
  cdef vector[double] times, values, slopes, start_values, end_values
  cdef double start, t, v, dt, slope, side, inward, progress, delta
  cdef double inward_travel = 0.0, outward_travel = 0.0
  cdef double total, consistency, net, a, b
  cdef size_t i, j, n, count
  if not observations:
    if metrics:
      return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
    return 0.0
  if window_s < 0.0:
    return None
  start = observations[-1].time_s - window_s
  if not isfinite(start):
    return None
  for observation in observations:
    t = observation.time_s
    if not isfinite(t):
      return None
    if t >= start:
      if times.size() and t < times.back():
        return None
      v = getattr(observation, attribute)
      if not isfinite(v):
        return None
      times.push_back(t)
      values.push_back(v)
  n = values.size()
  if metrics and n < 2:
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False)
  for i in range(n):
    for j in range(i + 1, n):
      dt = times[j] - times[i]
      if dt < 0.10:
        continue
      v = (values[j] - values[i]) / dt
      if not isfinite(v):
        return None
      slopes.push_back(v)
  slope = median(slopes)
  if not metrics:
    return slope
  v = values[n - 1]
  if v == 0.0:
    v = values[0]
  side = -1.0 if v < 0.0 else 1.0
  inward = max(0.0, -side * slope)
  count = min(<size_t>3, max(<size_t>1, n // 3))
  for i in range(count):
    start_values.push_back(fabs(values[i]))
  for i in range(n - min(<size_t>2, n), n):
    end_values.push_back(fabs(values[i]))
  a = median(start_values)
  b = median(end_values)
  progress = max(0.0, a - b)
  for i in range(n - 1):
    delta = fabs(values[i]) - fabs(values[i + 1])
    if fabs(delta) < 0.01:
      continue
    if delta > 0.0:
      inward_travel += delta
    else:
      outward_travel -= delta
  total = inward_travel + outward_travel
  if not isfinite(total) or not isfinite(progress) or not isfinite(slope):
    return None
  consistency = inward_travel / total if total > 1e-6 else 0.0
  net = progress / max(total, 1e-6)
  return slope, inward, progress, total, net, consistency, total >= jitter_travel and net < jitter_fraction


def nearest_segment(object segments, double x, double y):
  cdef double x0, y0, tx, ty, length, accumulated_s
  cdef double dx, dy, raw, ratio, cx, cy, ox, oy, distance
  cdef double best = float('inf')
  cdef object result = None
  if not isfinite(x) or not isfinite(y):
    return None
  for x0, y0, tx, ty, length, accumulated_s in segments:
    dx = tx * length
    dy = ty * length
    raw = ((x - x0) * dx + (y - y0) * dy) / (length * length)
    if not isfinite(raw):
      return None
    ratio = min(1.0, max(0.0, raw))
    cx = x0 + ratio * dx
    cy = y0 + ratio * dy
    ox = x - cx
    oy = y - cy
    distance = ox * ox + oy * oy
    if distance < best:
      best = distance
      result = (accumulated_s + ratio * length, cx, cy, tx, ty, -ty * ox + tx * oy)
  return result
