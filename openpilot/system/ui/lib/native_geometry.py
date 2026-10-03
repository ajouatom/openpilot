"""Shared C3/C4 ribbon geometry; NumPy projection with optional native packing."""
import numpy as np

from openpilot.system.ui.lib import native_draw
from openpilot.system.ui.lib.geometry_cache import cached_projection


def active() -> bool:
  # An older optional binary must still allow the Python UI to start.
  return native_draw._ENABLED and hasattr(native_draw._draw_native, 'clip_ribbon')


def offset_sides(points, left_y, right_y, left_z, right_z):
  if active() and points.dtype in (np.float32, np.float64):
    return native_draw._draw_native.offset_sides(points, left_y, right_y, left_z, right_z)
  offsets = np.array([[0, left_y, left_z], [0, right_y, right_z]], dtype=np.float32)
  return points[None, :, :] + offsets[:, None, :]


def clip_ribbon(projected, clip, allow_invert=True):
  if active() and projected.dtype in (np.float32, np.float64):
    return native_draw._draw_native.clip_ribbon(projected, clip.x, clip.x + clip.width,
                                               clip.y, clip.y + clip.height, allow_invert)
  depth_ok = np.abs(projected[2]) >= 1e-6
  xy = np.divide(projected[:2], projected[2:3], out=np.full_like(projected[:2], np.nan), where=depth_ok[None, :, :])
  valid = (depth_ok & (xy[0] >= clip.x) & (xy[0] <= clip.x + clip.width)
           & (xy[1] >= clip.y) & (xy[1] <= clip.y + clip.height)).all(axis=0)
  left, right = xy[:, 0, valid], xy[:, 1, valid]
  if not allow_invert and left.shape[1]:
    keep = left[1] == np.minimum.accumulate(left[1])
    left, right = left[:, keep], right[:, keep]
  return np.concatenate((left.T, right[:, ::-1].T)).astype(np.float32)


@cached_projection
def project_ribbon(line, half_width, z_offset, max_idx, transform, clip, allow_invert=True,
                   max_distance=None, y_shift=0., start_idx=0):
  """C3 adds an interpolated distance endpoint; C4 uses only recorded nodes."""
  if active() and hasattr(native_draw._draw_native, 'project_ribbon_batch') and line.dtype in (np.float32, np.float64):
    return native_draw._draw_native.project_ribbon_batch(line, half_width, z_offset, max_idx, transform, clip,
                                                        allow_invert, max_distance, y_shift, start_idx)
  points = line[start_idx:max_idx + 1]
  if max_distance is not None and 0 < max_idx < len(line) - 1:
    p0, p1 = line[max_idx:max_idx + 2]
    end = np.array([max_distance, np.interp(max_distance, [p0[0], p1[0]], [p0[1], p1[1]]),
                    np.interp(max_distance, [p0[0], p1[0]], [p0[2], p1[2]])], dtype=points.dtype)
    points = np.concatenate((points, end[None, :]))
  points = points[points[:, 0] >= 0]
  n = len(points)
  if n == 0:
    return np.empty((0, 2), dtype=np.float32)
  sides = offset_sides(points, -half_width + y_shift, half_width + y_shift, z_offset, z_offset)
  projected = (transform @ sides.reshape(2*n, 3).T).reshape(3, 2, n)
  return clip_ribbon(projected, clip, allow_invert)
