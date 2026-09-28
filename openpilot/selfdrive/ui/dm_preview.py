"""Read-only presentation helpers for the onroad DM inset."""
import math
from dataclasses import dataclass


PREVIEW_MAX_AGE = 0.5


@dataclass(frozen=True)
class PreviewState:
  kind: str
  label: str


def sample_fresh(now: float, timestamp: float) -> bool:
  return timestamp > 0 and 0 <= now - timestamp < PREVIEW_MAX_AGE


def preview_state(*, monitoring_fresh: bool, camera_unavailable: bool, driver_fresh: bool,
                  frame_fresh: bool, face_detected: bool, distracted: bool, lockout: bool, wheel_policy: bool = False) -> PreviewState:
  if not monitoring_fresh:
    return PreviewState('waiting', 'DM CHECK')
  if lockout:
    return PreviewState('warning', 'DM LIMIT')
  if camera_unavailable:
    return PreviewState('wheel', 'DM HANDS')
  if not driver_fresh or not frame_fresh:
    return PreviewState('waiting', 'DM CHECK')
  if not face_detected:
    return PreviewState('searching', 'DM FACE?')
  if distracted:
    return PreviewState('warning', 'DM ALERT')
  if wheel_policy:
    return PreviewState('wheel', 'DM HANDS')
  return PreviewState('tracking', 'DM FACE')


def preview_rect(x: float, y: float, width: float, height: float, compact: bool) -> tuple[float, float, float, float]:
  if compact:
    # D ends at x+373; the 60px side strip begins at x+476 on C4.
    return x + width - 154, y + height - 96, 84, 84
  return x + 40, y + 220, 260, 180


def face_crop(width: int, height: int, face_position, aspect: float) -> tuple[float, float, float, float] | None:
  """Approximate face projection used by the existing driver preview, in raw pixels.

  The display is mirrored; return an unmirrored source rectangle for a single
  horizontal flip by CameraView. This projection is display-only, never DM input.
  """
  if len(face_position) != 2 or not all(math.isfinite(v) for v in face_position):
    return None
  fx, fy = face_position
  mirrored_x = (1080.0 - 1714.0 * fx) / 2160.0 * width
  tici_y = -135.0 + 504.0 + abs(fx) * 112.0 + (1205.0 - abs(fx) * 724.0) * fy
  center_y = height / 2 + (tici_y - 540.0) * width / 2160.0
  if not (0 <= mirrored_x <= width and 0 <= center_y <= height):
    return None
  crop_h = min(height * 0.42, width / aspect)
  crop_w = crop_h * aspect
  left = min(max(width - mirrored_x - crop_w / 2, 0), width - crop_w)
  top = min(max(center_y - crop_h / 2, 0), height - crop_h)
  return left, top, crop_w, crop_h
