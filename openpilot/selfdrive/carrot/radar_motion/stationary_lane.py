"""Measured stopped-corner fallback near the end of a stopping trajectory.

The time-sampled stopping path can end before a visible car, with a nearly
sideways tail. It cannot then establish that car's lateral distance. Require
both confident inner lane lines, current visual position agreement and stable
measured corner motion instead. This supplies only an otherwise missing L2;
it does not change the path used by ordinary matching or CUT-IN prediction.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot, VisionLead


MAX_GAP_S = 0.15
MIN_MEASURED_S = 0.35
MIN_VISION_S = 0.10
MIN_VISION_SAMPLES = 3
MIN_VISION_PROB = 0.40
HOLD_VISION_PROB = 0.35
MAX_DISTANCE_M = 80.0


def _line(line: Any) -> tuple[tuple[float, ...], tuple[float, ...]] | None:
  xs = tuple(float(x) for x in getattr(line, "x", ()))
  ys = tuple(float(y) for y in getattr(line, "y", ()))
  if (len(xs) < 3 or len(xs) != len(ys)
      or not all(math.isfinite(v) for v in (*xs, *ys))
      or any(b <= a for a, b in zip(xs, xs[1:]))):
    return None
  return xs, ys


def _at(line: tuple[tuple[float, ...], tuple[float, ...]], x: float) -> float | None:
  xs, ys = line
  if not math.isfinite(x) or x < xs[0] or x > xs[-1]:
    return None
  i = min(max(1, bisect.bisect_left(xs, x)), len(xs) - 1)
  f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
  return ys[i - 1] + f * (ys[i] - ys[i - 1])


class _Lane:
  def __init__(self, left: Any, right: Any, std: float):
    self.left, self.right, self.margin = left, right, 0.3 + 2.0 * std

  def offset(self, x: float, radar_y: float) -> float | None:
    left, right = _at(self.left, x), _at(self.right, x)
    if left is None or right is None or not 2.5 <= right - left <= 4.3:
      return None
    # Model Y is right-positive; radar Y is left-positive.
    y = -radar_y
    if not left + self.margin <= y <= right - self.margin:
      return None
    offset = (left + right) * 0.5 - y
    return offset if abs(offset) <= 1.0 else None


def confident_ego_lane(model: Any) -> tuple[_Lane, float] | None:
  if str(getattr(getattr(model, "meta", None), "laneChangeState", "unknown")) != "off":
    return None
  path = _line(getattr(model, "position", None))
  lanes = tuple(getattr(model, "laneLines", ()))
  probs = tuple(getattr(model, "laneLineProbs", ()))
  stds = tuple(getattr(model, "laneLineStds", ()))
  if path is None or len(lanes) < 3 or len(probs) < 3 or len(stds) < 3:
    return None
  if (not all(math.isfinite(p) and p >= 0.90 for p in probs[1:3])
      or not all(math.isfinite(s) and 0.0 <= s <= 0.25 for s in stds[1:3])):
    return None
  left, right = _line(lanes[1]), _line(lanes[2])
  if left is None or right is None or path[0][-1] < 30.0:
    return None
  lane = _Lane(left, right, max(stds[1:3]))
  # The original path must agree with this ego lane before its stopping tail.
  # Never borrow the adjacent lane during a lane change or extrapolate a line.
  for x in (0.0, 10.0, 20.0, min(40.0, path[0][-1] - 10.0)):
    y = _at(path, x)
    if y is None or lane.offset(x, -y) is None:
      return None
  return lane, path[0][-1]


def _stopping_lane(model: Any) -> tuple[_Lane, float] | None:
  velocity = tuple(getattr(getattr(model, "velocity", None), "x", ()))
  if len(velocity) < 2 or not math.isfinite(velocity[-1]) or not -0.5 <= velocity[-1] <= 2.0:
    return None
  return confident_ego_lane(model)


@dataclass
class _History:
  point: RadarPointSnapshot
  time_s: float
  since_s: float
  vision_since_s: float | None = None
  vision_samples: int = 0
  last_strong_s: float | None = None
  confirmed: bool = False


class StoppingLaneCornerTracker:
  def __init__(self) -> None:
    self._history: dict[tuple[str, int], _History] = {}

  def reset(self) -> None:
    self._history.clear()

  def update(
    self, time_s: float, v_ego: float, model: Any,
    points: tuple[RadarPointSnapshot, ...], vision: VisionLead | None,
  ) -> tuple[RadarPointSnapshot, float] | None:
    context = _stopping_lane(model)
    if (context is None or vision is None or not math.isfinite(time_s)
        or not math.isfinite(v_ego) or v_ego < 5.0
        or not all(math.isfinite(v) for v in (
          vision.probability, vision.d_rel, vision.y_rel, vision.velocity,
        ))):
      self.reset()
      return None
    lane, end_x = context
    history = {}
    candidates = []
    matching_points = 0
    for point in points:
      if (not point.source.startswith("corner") or not point.measured
          or not all(math.isfinite(v) for v in (
            point.d_rel, point.y_rel, point.v_rel, point.yv_rel, point.v_lead,
          ))
          or not 30.0 <= point.d_rel <= 120.0 or abs(point.v_lead) > 2.0):
        continue
      offset = lane.offset(point.d_rel, point.y_rel)
      if offset is None:
        continue
      key = point.source, point.track_id
      old = self._history.get(key)
      dt = time_s - old.time_s if old is not None else math.inf
      continuous = bool(
        old is not None and 0.0 < dt <= MAX_GAP_S
        and abs(point.d_rel - old.point.d_rel - old.point.v_rel * dt) <= 1.0
        and abs(point.y_rel - old.point.y_rel - old.point.yv_rel * dt) <= 0.30
        and abs(point.v_lead - old.point.v_lead) <= 1.5
      )
      h = _History(point, time_s, old.since_s if continuous else time_s)
      # A lane estimate is only a geometric check. The current visual car must
      # also agree in raw range, side and speed; no radar-only promotion.
      supports = (
        vision.probability >= HOLD_VISION_PROB
        and lane.offset(vision.d_rel, vision.y_rel) is not None
        and min(point.d_rel, vision.d_rel) >= end_x - 10.0
        and max(point.d_rel, vision.d_rel) <= end_x + 25.0
        and abs(point.d_rel - vision.d_rel) <= min(8.0, point.d_rel * 0.12)
        and abs(point.y_rel - vision.y_rel) <= 1.25
        and -1.0 <= vision.velocity <= 8.0
        and abs(point.v_lead - vision.velocity) <= 8.0
      )
      if supports:
        # An equally plausible return just outside the output range still
        # makes association ambiguous; the 80 m cap cannot resolve identity.
        matching_points += 1
        strong = vision.probability >= MIN_VISION_PROB
        if (continuous and old.last_strong_s is not None
            and time_s - old.last_strong_s <= MAX_GAP_S):
          h.vision_since_s = old.vision_since_s
          h.vision_samples = old.vision_samples
          h.last_strong_s = old.last_strong_s
          h.confirmed = old.confirmed
        if strong:
          h.vision_since_s = time_s if h.vision_since_s is None else h.vision_since_s
          h.vision_samples += 1
          h.last_strong_s = time_s
        if (h.vision_since_s is not None and h.last_strong_s is not None
            and time_s - h.since_s >= MIN_MEASURED_S
            and time_s - h.vision_since_s >= MIN_VISION_S
            and h.vision_samples >= MIN_VISION_SAMPLES
            and time_s - h.last_strong_s <= MAX_GAP_S
            and point.d_rel <= MAX_DISTANCE_M
            and (strong or h.confirmed)):
          h.confirmed = True
          candidates.append((point, offset))
      history[key] = h
    self._history = history
    # Multiple matching stopped returns are ambiguous without a front anchor.
    return candidates[0] if len(candidates) == 1 and matching_points == 1 else None
