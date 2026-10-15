"""Corroborate a slowing visual lead with a corner already in the ego lane.

CUT-IN requires previous outside-lane motion. An object born inside the lane
needs a different contract: current lane/vision agreement and continuous radar
kinematics. This auxiliary lead never replaces a selected front/SCC primary.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.primary import (
  RADAR_ONLY_MOVING_CORNER_MAX_LONGITUDINAL_ERROR_RATE_MPS, RadarPointSnapshot, VisionLead,
)
from openpilot.selfdrive.carrot.radar_motion.stationary_lane import confident_ego_lane


@dataclass
class _History:
  point: RadarPointSnapshot
  time_s: float
  since_s: float
  vision_since_s: float | None = None
  vision_samples: int = 0
  lane_since_s: float | None = None
  braking_since_s: float | None = None
  braking_start_v: float = 0.0
  observations: tuple[tuple[float, float, float], ...] = ()


class InLaneCornerTracker:
  def __init__(self) -> None:
    self._history: dict[tuple[str, int], _History] = {}
    self._measurement_time_s: float | None = None

  def reset(self) -> None:
    self._history.clear()
    self._measurement_time_s = None

  def update(
    self, time_s: float, v_ego: float, model: Any,
    points: tuple[RadarPointSnapshot, ...], vision: VisionLead | None,
    *, measurement_time_s: float | None = None,
  ) -> tuple[RadarPointSnapshot, float] | None:
    if not math.isfinite(time_s) or not math.isfinite(v_ego) or v_ego < 5.0:
      self.reset()
      return None

    measurement_time_s = time_s if measurement_time_s is None else measurement_time_s
    if self._measurement_time_s is not None and measurement_time_s == self._measurement_time_s:
      return None
    if (not math.isfinite(measurement_time_s)
        or (self._measurement_time_s is not None and measurement_time_s < self._measurement_time_s)):
      self.reset()
      self._measurement_time_s = measurement_time_s
      return None
    self._measurement_time_s = measurement_time_s

    # Accumulate measured motion independently of an occasional missing visual
    # vote. Neither a retained history nor a previous lane grants an output.
    history = {}
    for point in points:
      if (not point.source.startswith("corner") or not point.measured
          or not all(math.isfinite(v) for v in (
            point.d_rel, point.y_rel, point.v_rel, point.yv_rel, point.v_lead, point.a_lead,
          )) or not 5.0 <= point.d_rel <= 120.0):
        continue
      key = point.source, point.track_id
      old = self._history.get(key)
      dt = time_s - old.time_s if old is not None else math.inf
      continuous = bool(
        old is not None and 0.0 < dt <= 0.15
        and abs(point.d_rel - old.point.d_rel - old.point.v_rel * dt) <= 1.0
        and abs(point.y_rel - old.point.y_rel - old.point.yv_rel * dt) <= 0.40
        and abs(point.v_lead - old.point.v_lead) <= 1.5
      )
      history[key] = _History(
        point, time_s, old.since_s if continuous else time_s,
        old.vision_since_s if continuous else None,
        old.vision_samples if continuous else 0,
        old.lane_since_s if continuous else None,
        old.braking_since_s if continuous else None,
        old.braking_start_v if continuous else point.v_lead,
        tuple(v for v in old.observations if time_s - v[0] <= 0.5) + ((time_s, point.d_rel, point.v_rel),)
        if continuous else ((time_s, point.d_rel, point.v_rel),),
      )
      item = history[key]
      if point.a_lead > -0.5 or point.v_rel >= -1.0:
        item.braking_since_s = None
      elif item.braking_since_s is None:
        item.braking_since_s = time_s
        item.braking_start_v = point.v_lead
    self._history = history

    context = confident_ego_lane(model)
    velocity = tuple(getattr(getattr(model, "velocity", None), "x", ()))
    # This is a braking backup, not a second general-purpose visual match.
    # Require the driving model to independently predict substantial slowing.
    slowing = (len(velocity) >= 2 and math.isfinite(velocity[-1])
               and -0.5 <= velocity[-1] <= 0.75 * v_ego)
    offsets = {}
    for key, item in history.items():
      offset = context[0].offset(item.point.d_rel, item.point.y_rel) if context is not None else None
      offsets[key] = offset
      if offset is None:
        item.lane_since_s = None
      elif item.lane_since_s is None:
        item.lane_since_s = time_s
    visual_ok = bool(
      slowing and context is not None and vision is not None
      and all(math.isfinite(v) for v in (
        vision.probability, vision.d_rel, vision.y_rel, vision.velocity,
        vision.x_std, vision.y_std, vision.v_std,
      ))
      and vision.probability >= 0.50
      and 0.0 < vision.x_std <= 15.0 and 0.0 < vision.y_std <= 2.0
      and 0.0 < vision.v_std <= 5.0
    )
    if not visual_ok:
      for item in history.values():
        item.vision_since_s, item.vision_samples = None, 0
      return None
    assert context is not None and vision is not None
    lane, _ = context
    visual_offset = lane.offset(vision.d_rel, vision.y_rel)
    matches = []
    for key, item in history.items():
      point = item.point
      offset = offsets[key]
      if (offset is not None and visual_offset is not None
          and abs(offset - visual_offset) <= 0.75
          and abs(point.d_rel - vision.d_rel) <= 10.0
          and abs(point.v_lead - vision.velocity) <= 4.0):
        matches.append((key, offset))
      else:
        item.vision_since_s, item.vision_samples = None, 0

    # Current ambiguity vetoes even a previously confirmed ID. A newly born
    # second return must not lose to the established one's longer history.
    if len(matches) != 1:
      for item in history.values():
        item.vision_since_s, item.vision_samples = None, 0
      return None
    key, offset = matches[0]
    item = history[key]
    if item.vision_since_s is None:
      item.vision_since_s = time_s
    item.vision_samples += 1
    point = item.point
    samples = item.observations
    displacement = sum(.5 * (a[2] + b[2]) * (b[0] - a[0]) for a, b in zip(samples, samples[1:], strict=False))
    duration = time_s - samples[0][0]
    range_consistent = (duration >= 0.30
                        and abs(point.d_rel - samples[0][1] - displacement)
                        <= RADAR_ONLY_MOVING_CORNER_MAX_LONGITUDINAL_ERROR_RATE_MPS * duration)
    if (not range_consistent or item.braking_start_v - point.v_lead < 0.30
        or item.braking_since_s is None or time_s - item.braking_since_s < 0.35
        or item.lane_since_s is None or time_s - item.lane_since_s < 0.35
        or time_s - item.since_s < 0.35 or item.vision_samples < 3
        or time_s - item.vision_since_s < 0.10 - 1e-6
        or not 10.0 <= point.d_rel <= 80.0
        or point.v_rel >= -1.0 or point.v_lead < -2.0):
      return None
    return point, offset
