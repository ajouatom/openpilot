"""Read-only presentation helpers for the onroad DM eyes."""
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
                  face_detected: bool, distracted: bool, lockout: bool, wheel_policy: bool = False) -> PreviewState:
  if not monitoring_fresh:
    return PreviewState('waiting', 'DM CHECK')
  if lockout:
    return PreviewState('warning', 'DM LIMIT')
  if camera_unavailable:
    return PreviewState('wheel', 'DM HANDS')
  if not driver_fresh:
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


@dataclass(frozen=True)
class EyeExpression:
  x: float = 0.0
  y: float = 0.0
  left_open: float = 1.0
  right_open: float = 1.0
  left_known: bool = False
  right_known: bool = False


def eye_expression(driver) -> EyeExpression:
  """Mirror head pose into cartoon pupils; this is not measured ocular gaze.

  Eye confidence and sunglasses gate blink display only, never monitoring policy.
  Unknown eyes have no pupil and are distinct from a measured closed eyelid.
  """
  orientation, uncertainty = driver.faceOrientation, driver.faceOrientationStd
  pose_known = (len(orientation) == 3 and len(uncertainty) == 3 and
                all(math.isfinite(v) for v in (*orientation, *uncertainty)) and
                all(0 <= uncertainty[i] < 0.3 for i in (0, 1)))
  x = max(-1.0, min(1.0, -orientation[1] / 0.6)) if pose_known else 0.0
  y = max(-1.0, min(1.0, orientation[0] / 0.5)) if pose_known else 0.0
  glasses = driver.sunglassesProb

  def eyelid(confidence, blink):
    known = (math.isfinite(glasses) and 0 <= glasses < 0.9 and math.isfinite(confidence) and
             0.65 < confidence <= 1 and math.isfinite(blink) and 0 <= blink <= 1)
    return (max(0.08, 1.0 - blink), True) if known else (1.0, False)

  left_open, left_known = eyelid(driver.leftEyeProb, driver.leftBlinkProb)
  right_open, right_known = eyelid(driver.rightEyeProb, driver.rightBlinkProb)
  return EyeExpression(x, y, left_open, right_open, left_known, right_known)


def smooth_expression(previous: EyeExpression | None, target: EyeExpression, dt: float) -> EyeExpression:
  if previous is None:
    return target
  dt = min(max(dt, 0.0), 0.1)
  alpha = 1.0 - math.exp(-dt / 0.08)

  def eyelid(old, new, was_known, known):
    if not known or not was_known:
      return new
    # Closing follows a brief model blink quickly; opening softens over ~60 ms.
    weight = 1.0 - math.exp(-dt / (0.018 if new < old else 0.06))
    return old + weight * (new - old)

  return EyeExpression(previous.x + alpha * (target.x - previous.x), previous.y + alpha * (target.y - previous.y),
                       eyelid(previous.left_open, target.left_open, previous.left_known, target.left_known),
                       eyelid(previous.right_open, target.right_open, previous.right_known, target.right_known),
                       target.left_known, target.right_known)
