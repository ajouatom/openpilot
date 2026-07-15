#!/usr/bin/env python3
"""Shared fused-radar feature extraction and production inference helpers."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

try:
  from openpilot.selfdrive.carrot.radar_object_fusion import FusedRadarObject
except ModuleNotFoundError:
  from radar_object_fusion import FusedRadarObject


MODEL_HEADS = ("lead", "cutin", "external")
CUTIN_TEMPORAL_THRESHOLD_MAX = 0.82
CUTIN_ACTIVE_CURRENT_MIN = 0.20
EXTERNAL_ACTIVE_CURRENT_MIN = 0.20
HISTORY_OFFSETS = (1, 2, 4, 8, 12, 16, 20)
MAX_HISTORY_FRAMES = max(HISTORY_OFFSETS) + 2


@dataclass(frozen=True)
class VisionLeadContext:
  probability: float
  d_rel: float
  y_rel: float
  v: float
  a: float
  x_std: float
  y_std: float
  v_std: float


@dataclass(frozen=True)
class RadarLeadContext:
  time_s: float
  v_ego: float
  path: tuple[tuple[float, float], ...]
  lane_lines: tuple[tuple[tuple[float, float], ...], ...]
  lane_probs: tuple[float, ...]
  model_leads: tuple[VisionLeadContext, ...]


@dataclass(frozen=True)
class RadarLeadFeatures:
  object_id: str
  aliases: tuple[str, ...]
  radar_object: FusedRadarObject
  values: tuple[float, ...]
  track_age: int
  d_path: float
  d_path_future: float
  in_lane_prob: float


@dataclass(frozen=True)
class RadarLeadPrediction:
  features: RadarLeadFeatures
  lead_prob: float
  cutin_prob: float
  risk_prob: float
  external_prob: float = 0.0


@dataclass(frozen=True)
class RadarLeadDecision:
  lead_candidates: tuple[RadarLeadPrediction, ...]
  cutin_candidates: tuple[RadarLeadPrediction, ...]
  external_candidates: tuple[RadarLeadPrediction, ...] = ()


@dataclass(frozen=True)
class _Observation:
  time_s: float
  d_rel: float
  y_rel: float
  v_rel: float
  yv_rel: float
  d_path: float


@dataclass
class _ObjectHistory:
  object_id: str
  observations: deque[_Observation] = field(default_factory=lambda: deque(maxlen=MAX_HISTORY_FRAMES))
  last_frame: int = 0


@dataclass
class _DecisionState:
  lead_ema: float = 0.0
  cutin_ema: float = 0.0
  external_ema: float = 0.0
  lead_hits: int = 0
  cutin_hits: int = 0
  external_hits: int = 0
  lead_active_until: float = 0.0
  cutin_active_until: float = 0.0
  external_active_until: float = 0.0
  last_seen: float = 0.0


def _finite(value: float, fallback: float = 0.0) -> float:
  return float(value) if math.isfinite(float(value)) else fallback


def _line_y(points: Sequence[tuple[float, float]], distance: float) -> float:
  """Return model line position in radar coordinates (positive left)."""
  if not points:
    return 0.0
  if distance <= points[0][0]:
    return -float(points[0][1])
  for index in range(1, len(points)):
    x1, y1 = points[index]
    if distance <= x1:
      x0, y0 = points[index - 1]
      ratio = 0.0 if x1 == x0 else (distance - x0) / (x1 - x0)
      return -float(y0 + (y1 - y0) * ratio)
  return -float(points[-1][1])


def _feature_names() -> tuple[str, ...]:
  names = [
    "v_ego", "object_count", "track_age",
    "has_front", "has_corner", "has_scc", "corner180", "corner235",
    "trusted", "match_confidence", "pair_age",
    "d_rel", "y_rel", "v_rel", "a_rel", "yv_rel", "v_lead", "ttc",
    "front_d_present", "front_d_rel", "corner_d_present", "corner_d_rel", "sensor_d_delta",
    "front_y_present", "front_y_rel", "corner_y_present", "corner_y_rel", "sensor_y_delta",
    "front_v_present", "front_v_rel", "corner_v_present", "corner_v_rel", "sensor_v_delta",
    "path_y", "d_path", "lane_half_width", "in_lane_prob",
    "future_d_rel", "future_y_rel", "future_d_path", "future_in_lane_prob", "inward_speed",
  ]
  for lane_index in range(4):
    names.extend((f"lane{lane_index}_y", f"lane{lane_index}_prob"))
  for lead_index in range(1):
    names.extend((
      f"model{lead_index}_prob", f"model{lead_index}_d", f"model{lead_index}_y",
      f"model{lead_index}_v", f"model{lead_index}_a", f"model{lead_index}_x_std",
      f"model{lead_index}_y_std", f"model{lead_index}_v_std",
    ))
  for offset in HISTORY_OFFSETS:
    prefix = f"h{offset}"
    names.extend((
      f"{prefix}_present", f"{prefix}_dt", f"{prefix}_d_rel", f"{prefix}_y_rel",
      f"{prefix}_v_rel", f"{prefix}_yv_rel", f"{prefix}_d_path",
      f"{prefix}_d_rate", f"{prefix}_y_rate",
    ))
  return tuple(names)


MODEL_FEATURE_NAMES = _feature_names()


def object_aliases(obj: FusedRadarObject) -> tuple[str, ...]:
  aliases: list[str] = []
  if obj.front_track_id is not None:
    aliases.append(f"front:{obj.front_track_id}")
  if obj.corner_track_id is not None:
    aliases.append(f"corner:{obj.corner_track_id}")
  if obj.scc_track_id is not None:
    aliases.append(f"scc:{obj.scc_track_id}")
  return tuple(aliases) or (obj.object_id,)


def object_contains_track(obj: FusedRadarObject, track_id: int) -> bool:
  return track_id in (obj.front_track_id, obj.corner_track_id, obj.scc_track_id)


class RadarLeadFeatureBuilder:
  """Stateful feature builder shared by log export and on-device inference."""

  def __init__(self) -> None:
    self.frame_index = 0
    self._aliases: dict[str, _ObjectHistory] = {}

  def _history_for(self, aliases: tuple[str, ...]) -> _ObjectHistory:
    matches = {id(state): state for alias in aliases if (state := self._aliases.get(alias)) is not None}
    if matches:
      state = max(matches.values(), key=lambda candidate: len(candidate.observations))
      for other in matches.values():
        if other is state:
          continue
        for observation in other.observations:
          if not state.observations or observation.time_s > state.observations[-1].time_s:
            state.observations.append(observation)
        for alias, candidate in tuple(self._aliases.items()):
          if candidate is other:
            self._aliases[alias] = state
    else:
      state = _ObjectHistory(aliases[0])
    for alias in aliases:
      self._aliases[alias] = state
    return state

  @staticmethod
  def _lane_state(context: RadarLeadContext, distance: float, y_rel: float) -> tuple[float, float, float, float]:
    path_y = _line_y(context.path, distance)
    d_path = y_rel - path_y
    if len(context.lane_lines) < 3 or not context.lane_lines[1] or not context.lane_lines[2]:
      return path_y, d_path, 1.8, max(0.0, 1.0 - abs(d_path) / 1.8)
    left = _line_y(context.lane_lines[1], distance)
    right = _line_y(context.lane_lines[2], distance)
    lane_center = 0.5 * (left + right)
    lane_half_width = max(0.5, 0.5 * abs(left - right))
    lane_d_path = y_rel - lane_center
    return path_y, lane_d_path, lane_half_width, max(0.0, 1.0 - abs(lane_d_path) / lane_half_width)

  def _values(
    self,
    context: RadarLeadContext,
    objects: Sequence[FusedRadarObject],
    obj: FusedRadarObject,
    history: _ObjectHistory,
  ) -> tuple[tuple[float, ...], float, float, float]:
    path_y, d_path, lane_half_width, in_lane_prob = self._lane_state(context, obj.d_rel, obj.y_rel)
    future_d = max(0.0, obj.d_rel + obj.v_rel)
    future_y = obj.y_rel + obj.yv_rel
    _, future_d_path, future_half_width, future_in_lane = self._lane_state(context, future_d, future_y)
    ttc = min(99.0, obj.d_rel / max(-obj.v_rel, 0.1)) if obj.v_rel < -0.1 else 99.0
    corner180 = obj.lateral_source == "corner180"
    corner235 = obj.lateral_source == "corner235"
    raw = {
      "v_ego": context.v_ego,
      "object_count": float(len(objects)),
      "track_age": float(min(len(history.observations) + 1, 40)),
      "has_front": float(obj.front_track_id is not None),
      "has_corner": float(obj.corner_track_id is not None),
      "has_scc": float(obj.scc_track_id is not None),
      "corner180": float(corner180),
      "corner235": float(corner235),
      "trusted": float(obj.trusted_for_control),
      "match_confidence": obj.match_confidence,
      "pair_age": float(obj.pair_age),
      "d_rel": obj.d_rel,
      "y_rel": obj.y_rel,
      "v_rel": obj.v_rel,
      "a_rel": obj.a_rel,
      "yv_rel": obj.yv_rel,
      "v_lead": obj.v_lead,
      "ttc": ttc,
      "front_d_present": float(obj.front_d_rel is not None),
      "front_d_rel": obj.front_d_rel or 0.0,
      "corner_d_present": float(obj.corner_d_rel is not None),
      "corner_d_rel": obj.corner_d_rel or 0.0,
      "sensor_d_delta": (obj.front_d_rel - obj.corner_d_rel) if obj.front_d_rel is not None and obj.corner_d_rel is not None else 0.0,
      "front_y_present": float(obj.front_y_rel is not None),
      "front_y_rel": obj.front_y_rel or 0.0,
      "corner_y_present": float(obj.corner_y_rel is not None),
      "corner_y_rel": obj.corner_y_rel or 0.0,
      "sensor_y_delta": (obj.front_y_rel - obj.corner_y_rel) if obj.front_y_rel is not None and obj.corner_y_rel is not None else 0.0,
      "front_v_present": float(obj.front_v_rel is not None),
      "front_v_rel": obj.front_v_rel or 0.0,
      "corner_v_present": float(obj.corner_v_rel is not None),
      "corner_v_rel": obj.corner_v_rel or 0.0,
      "sensor_v_delta": (obj.front_v_rel - obj.corner_v_rel) if obj.front_v_rel is not None and obj.corner_v_rel is not None else 0.0,
      "path_y": path_y,
      "d_path": d_path,
      "lane_half_width": lane_half_width,
      "in_lane_prob": in_lane_prob,
      "future_d_rel": future_d,
      "future_y_rel": future_y,
      "future_d_path": future_d_path,
      "future_in_lane_prob": max(0.0, 1.0 - abs(future_d_path) / future_half_width),
      "inward_speed": abs(d_path) - abs(future_d_path),
    }
    for lane_index in range(4):
      line = context.lane_lines[lane_index] if lane_index < len(context.lane_lines) else ()
      raw[f"lane{lane_index}_y"] = _line_y(line, obj.d_rel) if line else 0.0
      raw[f"lane{lane_index}_prob"] = context.lane_probs[lane_index] if lane_index < len(context.lane_probs) else 0.0
    for lead_index in range(1):
      lead = context.model_leads[lead_index] if lead_index < len(context.model_leads) else None
      values = (
        lead.probability, lead.d_rel, lead.y_rel, lead.v, lead.a,
        lead.x_std, lead.y_std, lead.v_std,
      ) if lead is not None else (0.0,) * 8
      for name, value in zip(("prob", "d", "y", "v", "a", "x_std", "y_std", "v_std"), values, strict=True):
        raw[f"model{lead_index}_{name}"] = value
    observations = history.observations
    for offset in HISTORY_OFFSETS:
      prefix = f"h{offset}"
      observation = observations[-offset] if len(observations) >= offset else None
      raw[f"{prefix}_present"] = float(observation is not None)
      if observation is None:
        for name in ("dt", "d_rel", "y_rel", "v_rel", "yv_rel", "d_path", "d_rate", "y_rate"):
          raw[f"{prefix}_{name}"] = 0.0
        continue
      dt = max(1e-3, context.time_s - observation.time_s)
      raw[f"{prefix}_dt"] = dt
      raw[f"{prefix}_d_rel"] = observation.d_rel
      raw[f"{prefix}_y_rel"] = observation.y_rel
      raw[f"{prefix}_v_rel"] = observation.v_rel
      raw[f"{prefix}_yv_rel"] = observation.yv_rel
      raw[f"{prefix}_d_path"] = observation.d_path
      raw[f"{prefix}_d_rate"] = (obj.d_rel - observation.d_rel) / dt
      raw[f"{prefix}_y_rate"] = (obj.y_rel - observation.y_rel) / dt
    return tuple(_finite(raw[name]) for name in MODEL_FEATURE_NAMES), d_path, future_d_path, in_lane_prob

  def update(
    self, context: RadarLeadContext, objects: Iterable[FusedRadarObject],
  ) -> tuple[RadarLeadFeatures, ...]:
    current = tuple(objects)
    output: list[RadarLeadFeatures] = []
    for obj in current:
      aliases = object_aliases(obj)
      history = self._history_for(aliases)
      values, d_path, future_d_path, in_lane_prob = self._values(context, current, obj, history)
      output.append(RadarLeadFeatures(
        object_id=history.object_id,
        aliases=aliases,
        radar_object=obj,
        values=values,
        track_age=min(len(history.observations) + 1, 40),
        d_path=d_path,
        d_path_future=future_d_path,
        in_lane_prob=in_lane_prob,
      ))
      history.observations.append(_Observation(
        context.time_s, obj.d_rel, obj.y_rel, obj.v_rel, obj.yv_rel, d_path,
      ))
      history.last_frame = self.frame_index
    self.frame_index += 1
    for alias, history in tuple(self._aliases.items()):
      if self.frame_index - history.last_frame > MAX_HISTORY_FRAMES * 2:
        self._aliases.pop(alias, None)
    return tuple(output)


class RadarLeadModel:
  """Small three-head MLP with the same NumPy path used during training."""

  def __init__(self, path: Path) -> None:
    import numpy as np

    self.np = np
    with np.load(path, allow_pickle=False) as model:
      names = tuple(str(value) for value in model["feature_names"].tolist())
      heads = tuple(str(value) for value in model["head_names"].tolist())
      if names != MODEL_FEATURE_NAMES or heads != MODEL_HEADS:
        raise RuntimeError("radar lead model schema mismatch")
      self.mean = model["mean"].astype(np.float32)
      self.std = model["std"].astype(np.float32)
      self.w1 = model["w1"].astype(np.float32)
      self.b1 = model["b1"].astype(np.float32)
      self.w2 = model["w2"].astype(np.float32)
      self.b2 = model["b2"].astype(np.float32)
      self.w3 = model["w3"].astype(np.float32)
      self.b3 = model["b3"].astype(np.float32)
      self.calibration = model["calibration"].astype(np.float32)
      self.thresholds = model["thresholds"].astype(np.float32)

  def predict(self, features: Sequence[RadarLeadFeatures]) -> tuple[RadarLeadPrediction, ...]:
    if not features:
      return ()
    np = self.np
    matrix = np.asarray([sample.values for sample in features], dtype=np.float32)
    normalized = (matrix - self.mean) / self.std
    hidden1 = np.maximum(normalized @ self.w1 + self.b1, 0.0)
    hidden2 = np.maximum(hidden1 @ self.w2 + self.b2, 0.0)
    logits = hidden2 @ self.w3 + self.b3
    logits = logits * self.calibration[:, 0] + self.calibration[:, 1]
    probabilities = 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))
    output = []
    for sample, (lead_prob, cutin_prob, external_prob) in zip(features, probabilities, strict=True):
      closing = max(0.0, -sample.radar_object.v_rel)
      ttc_risk = min(1.0, closing / 8.0) * min(1.0, 25.0 / max(sample.radar_object.d_rel, 1.0))
      risk_prob = float(max(cutin_prob, lead_prob * ttc_risk, external_prob * ttc_risk))
      output.append(RadarLeadPrediction(
        sample, float(lead_prob), float(cutin_prob), risk_prob, float(external_prob),
      ))
    return tuple(output)


class RadarLeadDecisionFilter:
  """Identity-aware hysteresis for stable lead and cut-in decisions."""

  def __init__(
    self, lead_threshold: float = 0.65, cutin_threshold: float = 0.70, external_threshold: float = 0.65,
  ) -> None:
    self.lead_threshold = lead_threshold
    self.cutin_threshold = cutin_threshold
    self.external_threshold = external_threshold
    self._states: dict[str, _DecisionState] = {}

  @staticmethod
  def _reliable(prediction: RadarLeadPrediction) -> bool:
    sample = prediction.features
    obj = sample.radar_object
    return obj.trusted_for_control or (sample.track_age >= 5 and obj.corner_track_id is not None) or (
      sample.track_age >= 5 and obj.front_track_id is not None and abs(sample.d_path) < 1.8
    )

  def update(self, time_s: float, predictions: Iterable[RadarLeadPrediction]) -> RadarLeadDecision:
    # Sensor aliases can briefly merge while front/corner associations change.
    # Process only the strongest current sample for an identity so sticky state
    # cannot be applied to several different radar points in the same frame.
    strongest: dict[str, RadarLeadPrediction] = {}
    for prediction in predictions:
      object_id = prediction.features.object_id
      previous = strongest.get(object_id)
      if previous is None or max(prediction.lead_prob, prediction.cutin_prob, prediction.external_prob) > max(
        previous.lead_prob, previous.cutin_prob, previous.external_prob,
      ):
        strongest[object_id] = prediction
    current = tuple(strongest.values())
    seen: set[str] = set()
    lead_active: list[RadarLeadPrediction] = []
    cutin_active: list[RadarLeadPrediction] = []
    external_active: list[RadarLeadPrediction] = []
    for prediction in current:
      object_id = prediction.features.object_id
      seen.add(object_id)
      state = self._states.setdefault(object_id, _DecisionState())
      state.last_seen = time_s
      state.lead_ema = max(prediction.lead_prob, 0.65 * state.lead_ema + 0.35 * prediction.lead_prob)
      state.cutin_ema = max(prediction.cutin_prob, 0.60 * state.cutin_ema + 0.40 * prediction.cutin_prob)
      state.external_ema = max(prediction.external_prob, 0.65 * state.external_ema + 0.35 * prediction.external_prob)
      reliable = self._reliable(prediction)
      state.lead_hits = state.lead_hits + 1 if reliable and state.lead_ema >= self.lead_threshold else 0
      state.external_hits = state.external_hits + 1 if reliable and state.external_ema >= self.external_threshold else 0
      obj = prediction.features.radar_object
      inward = abs(prediction.features.d_path_future) + 0.15 < abs(prediction.features.d_path)
      close_future_limit = 2.5 if obj.d_rel < 8.0 else 2.2
      close_geometry = (
        obj.d_rel < 12.0 and inward and abs(prediction.features.d_path_future) < close_future_limit
      )
      close_threshold = 0.62 if obj.d_rel < 8.0 else 0.70
      cutin_threshold = min(self.cutin_threshold, close_threshold) if close_geometry else self.cutin_threshold
      state.cutin_hits = state.cutin_hits + 1 if reliable and state.cutin_ema >= cutin_threshold else 0
      required_cutin_hits = 2 if close_geometry else 3
      urgent = (
        reliable and close_geometry and obj.d_rel < 4.5
        and prediction.cutin_prob >= max(0.85, cutin_threshold + 0.15)
      )
      if state.lead_hits >= 2:
        state.lead_active_until = time_s + 0.35
      if state.cutin_hits >= required_cutin_hits or urgent:
        state.cutin_active_until = time_s + 0.45
      if state.external_hits >= 2:
        state.external_active_until = time_s + 0.35
      if state.lead_ema < 0.25:
        state.lead_active_until = min(state.lead_active_until, time_s + 0.10)
      if state.cutin_ema < 0.25 or abs(prediction.features.d_path_future) > abs(prediction.features.d_path) + 0.5:
        state.cutin_active_until = min(state.cutin_active_until, time_s + 0.10)
      if state.external_ema < 0.25:
        state.external_active_until = min(state.external_active_until, time_s + 0.10)
      if time_s <= state.lead_active_until:
        lead_active.append(prediction)
      # Sticky state belongs to a fused object identity, but a sensor track ID can
      # be reassociated. Never expose a newly associated low-probability point as
      # a cut-in solely because the previous object was active.
      if time_s <= state.cutin_active_until and prediction.cutin_prob >= CUTIN_ACTIVE_CURRENT_MIN:
        cutin_active.append(prediction)
      if time_s <= state.external_active_until and prediction.external_prob >= EXTERNAL_ACTIVE_CURRENT_MIN:
        external_active.append(prediction)
    for object_id, state in tuple(self._states.items()):
      if object_id not in seen and time_s - state.last_seen > 0.5:
        self._states.pop(object_id, None)
    return RadarLeadDecision(
      lead_candidates=tuple(sorted(lead_active, key=lambda item: (-item.lead_prob, item.features.radar_object.d_rel))[:2]),
      cutin_candidates=tuple(sorted(cutin_active, key=lambda item: (-item.risk_prob, item.features.radar_object.d_rel))[:2]),
      external_candidates=tuple(sorted(
        external_active, key=lambda item: (-item.external_prob, item.features.radar_object.d_rel),
      )[:2]),
    )
