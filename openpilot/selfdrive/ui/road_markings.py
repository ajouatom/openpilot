"""Lane dashes and blindspot geometry shared by c3 and mici displays."""

import math

import numpy as np

LANE_DASH_LENGTH_M = 5.2
LANE_DASH_GAP_M = 4.2


def lane_dash_segments(line: np.ndarray, max_distance: float) -> list[np.ndarray]:
  if line.shape[0] < 2:
    return []

  x = line[:, 0]
  start_distance = max(0.0, float(x[0]))
  end_distance = min(max_distance, float(x[-1]))
  if end_distance <= start_distance:
    return []

  segments = []
  cycle_distance = LANE_DASH_LENGTH_M + LANE_DASH_GAP_M
  cursor = math.floor(start_distance / cycle_distance) * cycle_distance
  while cursor < end_distance:
    dash_start = max(cursor, start_distance)
    dash_end = min(cursor + LANE_DASH_LENGTH_M, end_distance)
    if dash_end > dash_start:
      inside = line[(x > dash_start) & (x < dash_end)]
      start_point = np.array(
        [dash_start, *(np.interp(dash_start, x, line[:, axis]) for axis in (1, 2))],
        dtype=line.dtype,
      )
      end_point = np.array(
        [dash_end, *(np.interp(dash_end, x, line[:, axis]) for axis in (1, 2))],
        dtype=line.dtype,
      )
      segments.append(np.vstack((start_point, inside, end_point)))
    cursor += cycle_distance
  return segments


def project_blindspot_barrier(points: np.ndarray, y_shift: float, transform: np.ndarray, clip) -> np.ndarray:
  points = points[points[:, 0] >= 0]
  if points.shape[0] == 0:
    return np.empty((0, 2), dtype=np.float32)

  # Project the upper/lower barrier edges together. This preserves the old
  # point and clipping rules without two Python projection calls per point.
  offsets = np.array(
    [[0.0, y_shift, 1.15], [0.0, y_shift, 0.6]],
    dtype=np.float32,
  )
  points_3d = points[None, :, :] + offsets[:, None, :]
  # Keep the former three-term float32 dot-product order while applying the
  # projection to both edges and all model points at once.
  projected = (
    transform[:, 0, None, None] * points_3d[None, :, :, 0] +
    transform[:, 1, None, None] * points_3d[None, :, :, 1] +
    transform[:, 2, None, None] * points_3d[None, :, :, 2]
  )
  upper_projected = projected[:, 0, :]
  lower_projected = projected[:, 1, :]

  valid_depth = (np.abs(upper_projected[2]) >= 1e-6) & (np.abs(lower_projected[2]) >= 1e-6)
  if not np.any(valid_depth):
    return np.empty((0, 2), dtype=np.float32)

  upper_screen = upper_projected[:2, valid_depth] / upper_projected[2, valid_depth][None, :]
  lower_screen = lower_projected[:2, valid_depth] / lower_projected[2, valid_depth][None, :]

  x_min, x_max = clip.x, clip.x + clip.width
  y_min, y_max = clip.y, clip.y + clip.height
  upper_in_clip = (
    (upper_screen[0] >= x_min) & (upper_screen[0] <= x_max) &
    (upper_screen[1] >= y_min) & (upper_screen[1] <= y_max)
  )
  lower_in_clip = (
    (lower_screen[0] >= x_min) & (lower_screen[0] <= x_max) &
    (lower_screen[1] >= y_min) & (lower_screen[1] <= y_max)
  )
  both_in_clip = upper_in_clip & lower_in_clip
  if not np.any(both_in_clip):
    return np.empty((0, 2), dtype=np.float32)

  upper_screen = upper_screen[:, both_in_clip]
  lower_screen = lower_screen[:, both_in_clip]

  # Match the old hill/inversion filter: keep a point only when its upper
  # screen Y does not increase relative to the last accepted point.
  if upper_screen.shape[1] > 1:
    keep = upper_screen[1] == np.minimum.accumulate(upper_screen[1])
    upper_screen = upper_screen[:, keep]
    lower_screen = lower_screen[:, keep]

  if upper_screen.shape[1] == 0:
    return np.empty((0, 2), dtype=np.float32)
  return np.vstack((upper_screen.T, lower_screen[:, ::-1].T)).astype(np.float32)


def blindspot_barrier_quads(points: np.ndarray) -> np.ndarray:
  if points.size == 0:
    return np.empty((0, 4, 2), dtype=np.float32)

  count = points.shape[0]
  half = count // 2
  if half < 3:
    return np.empty((0, 4, 2), dtype=np.float32)

  starts = np.arange(0, half - 2, 2)
  if starts.size == 0:
    return np.empty((0, 4, 2), dtype=np.float32)

  quads = np.stack(
    (
      points[starts],
      points[starts + 1],
      points[count - starts - 3],
      points[count - starts - 2],
    ),
    axis=1,
  )
  centers = np.mean(quads, axis=1, keepdims=True)
  angles = np.arctan2(quads[:, :, 1] - centers[:, :, 1], quads[:, :, 0] - centers[:, :, 0])
  order = np.argsort(angles, axis=1)
  ordered_quads = np.take_along_axis(quads, order[:, :, None], axis=1)

  return ordered_quads

