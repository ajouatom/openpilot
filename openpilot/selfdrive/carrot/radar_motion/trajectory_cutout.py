"""Predict clearance of a previously vision-confirmed moving primary lead.

Tracking and measured lead kinematics remain owned by the primary matcher.
This signal only permits bounded future following-distance relief in ACC.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from openpilot.selfdrive.carrot.radar_motion.predictor import project_to_model_path
from openpilot.selfdrive.carrot.radar_motion.primary import RadarPointSnapshot, VisionLead


CLEARANCE_HALF_WIDTH_M = 2.15  # ego + target half widths, with 0.25 m margin
MAX_EXIT_TIME_S = 2.5
MAX_GAP_S = 0.15
HISTORY_S = 0.40
CONFIRMATION_S = 0.10
RAMP_S = 0.30


@dataclass(frozen=True)
class CutOutPrediction:
  time_s: float = 0.0
  confidence: float = 0.0


@dataclass(frozen=True)
class _Observation:
  time_s: float
  point: RadarPointSnapshot
  lateral: RadarPointSnapshot
  d_path: float


class TrajectoryCutOutTracker:
  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self._history: deque[_Observation] = deque()
    self._vision_since: float | None = None
    self._vision_anchor: float | None = None
    self._candidate_since: float | None = None

  def _inactive(self) -> CutOutPrediction:
    self._candidate_since = None
    return CutOutPrediction()

  def update(self, time_s: float, point: RadarPointSnapshot | None,
             lateral: RadarPointSnapshot | None, vision: VisionLead | None,
             path: tuple[tuple[float, float], ...], v_ego: float,
             yaw_rate: float) -> CutOutPrediction:
    if (point is None or lateral is None or not point.measured or not lateral.measured
        or point.source != "frontRadar" or point.track_id == 0 or point.radar_track_state < 2
        or not all(math.isfinite(v) for v in (
          time_s, v_ego, yaw_rate, point.d_rel, point.y_rel, point.v_rel,
          point.v_lead, point.a_lead, lateral.d_rel, lateral.y_rel, lateral.v_rel,
        ))
        or len(path) < 2 or abs(yaw_rate) >= 0.025 or not 5.0 <= v_ego <= 40.0
        or not 6.0 < point.d_rel <= 60.0 or point.v_lead <= 4.0 or point.a_lead < -2.5):
      self.reset()
      return CutOutPrediction()

    d_path = project_to_model_path(path, lateral.d_rel, lateral.y_rel).d_path
    front_path = project_to_model_path(path, point.d_rel, point.y_rel).d_path
    if (not math.isfinite(d_path) or abs(d_path - front_path) > 0.65
        or abs(d_path - lateral.y_rel) > 0.75):
      self.reset()
      return CutOutPrediction()

    previous = self._history[-1] if self._history else None
    if previous is not None:
      dt = time_s - previous.time_s
      if (not 0.0 < dt <= MAX_GAP_S
          or point.track_id != previous.point.track_id
          or (lateral.source, lateral.track_id) != (previous.lateral.source, previous.lateral.track_id)
          or abs(point.d_rel - previous.point.d_rel - previous.point.v_rel * dt) > 1.5
          or abs(point.v_lead - previous.point.v_lead) > 3.0
          or abs(lateral.y_rel - previous.lateral.y_rel) > 0.65):
        self.reset()
    self._history.append(_Observation(time_s, point, lateral, d_path))
    while self._history and time_s - self._history[0].time_s > HISTORY_S + MAX_GAP_S:
      self._history.popleft()

    vision_valid = vision is not None and all(math.isfinite(v) for v in (
      vision.probability, vision.d_rel, vision.y_rel, vision.x_std,
    )) and vision.probability >= 0.40 and vision.x_std >= 0.0
    same_vision = (vision_valid and abs(vision.d_rel - point.d_rel) <= 3.0
                   and abs(vision.y_rel - point.y_rel) <= 1.0 and abs(front_path) < 0.5)
    if same_vision:
      if self._vision_since is None:
        self._vision_since = time_s
      if time_s - self._vision_since >= 0.30:
        self._vision_anchor = time_s
    else:
      self._vision_since = None
    if (self._vision_anchor is None or time_s - self._vision_anchor > 2.0
        or not vision_valid or vision.d_rel - point.d_rel <= max(3.0, 0.25 * point.d_rel, vision.x_std)):
      return self._inactive()

    history = tuple(o for o in self._history if time_s - o.time_s <= HISTORY_S)
    recent = tuple(o for o in history if time_s - o.time_s <= 0.20)
    if len(history) < 5 or len(recent) < 3 or time_s - history[0].time_s < 0.25:
      return self._inactive()
    side = math.copysign(1.0, d_path)
    net = side * (d_path - history[0].d_path)
    travel = sum(abs(b.d_path - a.d_path) for a, b in zip(history, history[1:], strict=False))
    long_rate = net / (time_s - history[0].time_s)
    short_rate = side * (d_path - recent[0].d_path) / (time_s - recent[0].time_s)
    raw_progress = side * (lateral.y_rel - history[0].lateral.y_rel)
    front_progress = side * (point.y_rel - history[0].point.y_rel)
    latest_progress = side * (d_path - history[-2].d_path)
    # A paired corner supplies the fine lateral history; quantized front
    # azimuth may plateau, but must not contradict the observed departure.
    front_min_progress = -0.10 if lateral.source.startswith("corner") else 0.10
    if (not 0.35 <= abs(d_path) < CLEARANCE_HALF_WIDTH_M or net < 0.20
        or raw_progress < 0.15 or front_progress < front_min_progress
        or net < 0.75 * travel or latest_progress < -0.03
        or min(long_rate, short_rate) < 0.35):
      return self._inactive()
    exit_time = (CLEARANCE_HALF_WIDTH_M - abs(d_path)) / min(long_rate, short_rate)
    # Reserve time for imperfect clearance prediction. Assume ego accelerates
    # at 0.5 m/s² and the lead decelerates at least at 0.5 m/s².
    horizon = exit_time + 0.30
    gap_at_exit = point.d_rel + min(0.0, point.v_rel) * horizon + 0.5 * (min(-0.5, point.a_lead) - 0.5) * horizon**2
    if not 0.0 < exit_time <= MAX_EXIT_TIME_S or gap_at_exit <= 6.0:
      return self._inactive()
    if self._candidate_since is None:
      self._candidate_since = time_s
    confidence = min(1.0, max(0.0, (time_s - self._candidate_since - CONFIRMATION_S) / RAMP_S))
    return CutOutPrediction(exit_time, confidence)
