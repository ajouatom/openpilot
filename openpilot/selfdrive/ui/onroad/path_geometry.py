"""Batch the interpolation and projection of the Carrot path ribbon."""
import numpy as np


def sample_path(line, distances):
  line = np.asarray(line, dtype=np.float32)
  idxs = np.arange(len(line), dtype=np.float32)
  # Preserve the existing two-stage interpolation, including repeated x nodes.
  indices = np.interp(distances, np.maximum.accumulate(line[:, 0]), idxs)
  return np.column_stack((distances, np.interp(indices, idxs, line[:, 1]), np.interp(indices, idxs, line[:, 2])))


def project_path(line, width, z_start, z_end, transform, clip, allow_invert=True):
  if len(line) == 0:
    return np.empty((0, 2), dtype=np.float32)
  # Float64 matches the scalar interpolation/projection at clipping boundaries.
  points = np.asarray(line, dtype=np.float64)
  z_off = np.interp(points[:, 0], [0., 100.], [z_start, z_end])
  y_off = np.interp(z_off, [-3., 0., 3.], [1.5, .5, 1.5]) * width
  sides = np.broadcast_to(points, (2, *points.shape)).copy()
  sides[0, :, 1] -= y_off
  sides[1, :, 1] += y_off
  sides[:, :, 2] += z_off
  projected = sides @ transform.T
  valid_depth = np.abs(projected[:, :, 2]) >= 1e-6
  xy = np.divide(projected[:, :, :2], projected[:, :, 2:3],
                 out=np.full(projected[:, :, :2].shape, np.nan), where=valid_depth[:, :, None])
  valid = (valid_depth & (xy[:, :, 0] >= clip.x) & (xy[:, :, 0] <= clip.x + clip.width)
           & (xy[:, :, 1] >= clip.y) & (xy[:, :, 1] <= clip.y + clip.height)).all(axis=0)
  left, right = xy[:, valid]
  if not allow_invert and len(left):
    keep = left[:, 1] <= np.minimum.accumulate(left[:, 1])
    left, right = left[keep], right[keep]
  return np.concatenate((left, right[::-1])).astype(np.float32)
