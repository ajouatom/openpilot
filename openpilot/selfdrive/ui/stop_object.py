"""Display the outgoing synthetic stop object as a cone, never as a real car."""

import math
import time

from opendbc.car.hyundai.stop_object import STOP_OBJECT_DISTANCE

MAX_AGE = 0.2


def stop_object_visible(sm, *, now=None, started_frame=0):
  now = time.monotonic() if now is None else now
  for name in ('carOutput', 'carControl', 'carState', 'radarState'):
    if any(not getattr(sm, field, {}).get(name, False) for field in ('seen', 'alive', 'valid')):
      return False
    if getattr(sm, 'recv_frame', {}).get(name, -1) < started_frame:
      return False
    for stamp in (getattr(sm, 'recv_time', {}).get(name, float('nan')), getattr(sm, 'logMonoTime', {}).get(name, 0) * 1e-9):
      if not math.isfinite(stamp) or not 0 <= now - stamp <= MAX_AGE:
        return False
  cc, cs, rs = sm['carControl'], sm['carState'], sm['radarState']
  return bool(
    getattr(sm['carOutput'].actuatorsOutput, 'stopObjectActive', False)
    and getattr(cc.hudControl, 'signalStopObject', False)
    and cc.enabled
    and cc.longActive
    and cs.canValid
    and not cs.canTimeout
    and not (cs.gasPressed or cs.brakePressed)
    and not rs.leadOne.status
    and not rs.leadTwo.status
  )


def draw_stop_object(renderer, sm, *, started_frame=0):
  if not stop_object_visible(sm, started_frame=started_frame):
    return
  import pyray as rl

  path = renderer._path.raw_points
  if len(path) == 0:
    return
  index = renderer._get_path_length_idx(path[:, 0], STOP_OBJECT_DISTANCE)
  z = float(path[min(index, len(path) - 1), 2]) + renderer._path_offset_z
  point = renderer._map_to_screen(STOP_OBJECT_DISTANCE, 0.0, z)
  if point is None:
    # The ordinary lead projector clips far outside the camera view. A 3 m
    # ground point can be below that margin on a zoomed/tall camera; retain its
    # projection for the bounded edge marker, but never project behind camera.
    transform = getattr(renderer, '_car_space_transform', None)
    if transform is None:
      return
    projected = transform @ (STOP_OBJECT_DISTANCE, 0.0, z)
    if not all(math.isfinite(float(v)) for v in projected) or projected[2] <= 1e-6:
      return
    point = projected[:2] / projected[2]
  if point is None or not all(math.isfinite(float(v)) for v in point):
    return
  rect = renderer._rect
  height = min(110.0, rect.height * 0.20)
  width = height * 0.7
  # Close road points can lie below the camera crop. Keep the marker legible
  # at the lower boundary; it still represents the fixed three-metre command.
  x = min(rect.x + rect.width - width, max(rect.x + width, float(point[0])))
  y = min(rect.y + rect.height - 12.0, max(rect.y + height + 12.0, float(point[1])))
  orange, dark, white = rl.Color(255, 135, 20, 240), rl.Color(35, 35, 35, 240), rl.Color(250, 250, 250, 240)
  rl.draw_rectangle(int(x - width * 0.65), int(y - height * 0.08), int(width * 1.3), int(height * 0.12), dark)
  # Raylib expects counter-clockwise vertices in screen coordinates.
  rl.draw_triangle(rl.Vector2(x, y - height), rl.Vector2(x - width * 0.5, y), rl.Vector2(x + width * 0.5, y), orange)
  for top, bottom in ((0.38, 0.52), (0.69, 0.81)):
    a, b = width * 0.5 * top, width * 0.5 * bottom
    yl, yh = y - height + height * top, y - height + height * bottom
    tl, bl = rl.Vector2(x - a, yl), rl.Vector2(x - b, yh)
    tr, br = rl.Vector2(x + a, yl), rl.Vector2(x + b, yh)
    rl.draw_triangle(tl, bl, br, white)
    rl.draw_triangle(tl, br, tr, white)
