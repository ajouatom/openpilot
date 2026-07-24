#!/usr/bin/env python3
"""Shared fused-radar feature extraction and production inference helpers."""

from __future__ import annotations

import math
from collections import deque
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path

try:
  from openpilot.selfdrive.carrot.radar.radar_object_fusion import FusedRadarObject
except ModuleNotFoundError:
  from radar_object_fusion import FusedRadarObject


MODEL_HEADS = ("lead", "cutin", "external")
CUTIN_TEMPORAL_THRESHOLD_MAX = 0.82
CUTIN_ACTIVE_CURRENT_MIN = 0.20
CUTIN_STICKY_MAX_S = 0.75
CUTIN_VEHICLE_HALF_WIDTH_M = 0.90
EXTERNAL_ACTIVE_CURRENT_MIN = 0.20
CORNER_ONLY_CUTIN_MAX_DREL_M = 30.0
FRONT_ONLY_CUTIN_MIN_DREL_M = 5.0
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
  base_cutin_prob: float = 0.0
  temporal_cutin_prob: float = 0.0


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
  cutin_evidence_time: float = 0.0
  cutin_hit_aliases: frozenset[str] = field(default_factory=frozenset)
  external_active_until: float = 0.0
  last_seen: float = 0.0
  cutin_aliases: frozenset[str] = field(default_factory=frozenset)


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
MODEL_FEATURE_INDICES = {name: index for index, name in enumerate(MODEL_FEATURE_NAMES)}
H4_DT_INDEX = MODEL_FEATURE_NAMES.index("h4_dt")
H8_DT_INDEX = MODEL_FEATURE_NAMES.index("h8_dt")
H4_D_PATH_INDEX = MODEL_FEATURE_NAMES.index("h4_d_path")
H8_D_PATH_INDEX = MODEL_FEATURE_NAMES.index("h8_d_path")
H4_PRESENT_INDEX = MODEL_FEATURE_NAMES.index("h4_present")
H8_PRESENT_INDEX = MODEL_FEATURE_NAMES.index("h8_present")
H12_PRESENT_INDEX = MODEL_FEATURE_NAMES.index("h12_present")
LANE_HALF_WIDTH_INDEX = MODEL_FEATURE_NAMES.index("lane_half_width")
LANE1_PROB_INDEX = MODEL_FEATURE_NAMES.index("lane1_prob")
LANE2_PROB_INDEX = MODEL_FEATURE_NAMES.index("lane2_prob")

ANTICIPATORY_FEATURE_NAMES = (
  "d_rel", "v_lead", "v_rel", "track_age", "has_front", "has_corner", "trusted", "match_confidence",
  "abs_y_rel", "abs_d_path", "abs_future_d_path", "d_path_reduction", "instant_inward",
  "h8_lane_inward", "h12_lane_inward", "lane1_prob", "lane2_prob", "model0_prob", "model0_y_std",
)


def _temporal_feature_names() -> tuple[str, ...]:
  names = [
    "v_ego", "object_count", "track_age",
    "has_front", "has_corner", "has_scc", "trusted", "match_confidence", "pair_age",
    "d_rel", "v_rel", "yv_rel", "v_lead", "ttc",
    "abs_y_rel", "abs_d_path", "abs_future_d_path", "d_path_reduction", "instant_inward",
    "lane_half_width", "in_lane_prob", "future_in_lane_prob", "lane1_prob", "lane2_prob",
    "model0_prob", "model0_d_delta", "model0_y_delta", "model0_x_std", "model0_y_std",
    "sensor_d_delta", "sensor_y_delta", "sensor_v_delta",
  ]
  for offset in HISTORY_OFFSETS:
    prefix = f"h{offset}"
    names.extend((
      f"{prefix}_present", f"{prefix}_dt", f"{prefix}_abs_d_path",
      f"{prefix}_lane_inward", f"{prefix}_lateral_inward",
      f"{prefix}_yv_inward", f"{prefix}_d_rate",
    ))
  return tuple(names)


TEMPORAL_CUTIN_FEATURE_NAMES = _temporal_feature_names()


def temporal_cutin_feature_matrix(matrix, np):
  """Return compact current and multi-scale trajectory features for cut-in inference."""
  def column(name: str):
    return matrix[:, MODEL_FEATURE_INDICES[name]]

  d_path = column("d_path")
  abs_d_path = np.abs(d_path)
  abs_future_d_path = np.abs(column("future_d_path"))
  side = np.where(d_path >= 0.0, 1.0, -1.0)
  values = [
    column("v_ego"), column("object_count"), column("track_age"),
    column("has_front"), column("has_corner"), column("has_scc"), column("trusted"),
    column("match_confidence"), column("pair_age"),
    column("d_rel"), column("v_rel"), column("yv_rel"), column("v_lead"), column("ttc"),
    np.abs(column("y_rel")), abs_d_path, abs_future_d_path, abs_d_path - abs_future_d_path,
    -side * column("yv_rel"), column("lane_half_width"), column("in_lane_prob"),
    column("future_in_lane_prob"), column("lane1_prob"), column("lane2_prob"),
    column("model0_prob"), column("model0_d") - column("d_rel"),
    column("model0_y") - column("y_rel"), column("model0_x_std"), column("model0_y_std"),
    column("sensor_d_delta"), column("sensor_y_delta"), column("sensor_v_delta"),
  ]
  for offset in HISTORY_OFFSETS:
    prefix = f"h{offset}"
    dt = np.maximum(column(f"{prefix}_dt"), 1e-3)
    values.extend((
      column(f"{prefix}_present"), column(f"{prefix}_dt"), np.abs(column(f"{prefix}_d_path")),
      (np.abs(column(f"{prefix}_d_path")) - abs_d_path) / dt,
      -side * column(f"{prefix}_y_rate"),
      -side * column(f"{prefix}_yv_rel"),
      column(f"{prefix}_d_rate"),
    ))
  return np.column_stack(values).astype(np.float32)


def _temporal_cutin_eligibility_from_values(matrix, values, np):
  """Apply only physical sensor and lane sanity; leave scenario classification to the MLP."""
  distance = values[:, 9]
  track_age = values[:, 2]
  v_lead = values[:, 12]
  abs_d_path = values[:, 15]
  abs_future_d_path = values[:, 16]
  lane_reliable = (values[:, 22] > 0.35) & (values[:, 23] > 0.35)
  h4_base = 32 + HISTORY_OFFSETS.index(4) * 7
  h8_base = 32 + HISTORY_OFFSETS.index(8) * 7
  h4_present = values[:, h4_base] > 0.5
  h8_present = values[:, h8_base] > 0.5
  h4_inward = values[:, h4_base + 3]
  h8_inward = values[:, h8_base + 3]
  future_inward = values[:, 17]
  plausible_motion = (
    ((h4_inward > 0.06) & (h8_inward > 0.06))
    | ((future_inward > 0.08) & ((h4_inward > 0.02) | (h8_inward > 0.02)))
  )
  return (
    (distance > 2.0) & (distance < 65.0) & (track_age >= 5.0)
    & ((v_lead > 1.0) | ((distance < 12.0) & (v_lead > -0.5)))
    & (abs_d_path > 0.65) & (abs_d_path < 4.8) & (abs_future_d_path < 3.4)
    & h4_present & h8_present & ((distance < 12.0) | lane_reliable)
    & plausible_motion
  )


def temporal_cutin_eligibility(matrix, np):
  values = temporal_cutin_feature_matrix(matrix, np)
  return _temporal_cutin_eligibility_from_values(matrix, values, np)


def anticipatory_feature_matrix(matrix, np):
  """Return compact, engineered early cut-in features from base model rows."""
  def column(name: str):
    return matrix[:, MODEL_FEATURE_INDICES[name]]

  d_path = column("d_path")
  abs_d_path = np.abs(d_path)
  abs_future_d_path = np.abs(column("future_d_path"))
  side = np.where(d_path >= 0.0, 1.0, -1.0)
  h8_dt = np.maximum(column("h8_dt"), 1e-3)
  h12_dt = np.maximum(column("h12_dt"), 1e-3)
  return np.column_stack((
    column("d_rel"), column("v_lead"), column("v_rel"), column("track_age"),
    column("has_front"), column("has_corner"), column("trusted"), column("match_confidence"),
    np.abs(column("y_rel")), abs_d_path, abs_future_d_path, abs_d_path - abs_future_d_path,
    -side * column("yv_rel"),
    (np.abs(column("h8_d_path")) - abs_d_path) / h8_dt,
    (np.abs(column("h12_d_path")) - abs_d_path) / h12_dt,
    column("lane1_prob"), column("lane2_prob"), column("model0_prob"), column("model0_y_std"),
  )).astype(np.float32)


def _anticipatory_eligibility_from_values(matrix, values, np):
  distance = values[:, 0]
  v_lead = values[:, 1]
  track_age = values[:, 3]
  abs_d_path = values[:, 9]
  abs_future_d_path = values[:, 10]
  reduction = values[:, 11]
  h8_lane_inward = values[:, 13]
  h12_lane_inward = values[:, 14]
  lane_reliable = (values[:, 15] > 0.5) & (values[:, 16] > 0.5)
  return (
    (distance > 2.5) & (distance < 60.0) & (v_lead > 2.0) & (track_age >= 7.0)
    & (abs_d_path > 1.15) & (abs_d_path < 4.2)
    & (abs_future_d_path < 2.5) & (reduction > 0.10)
    & (matrix[:, H8_PRESENT_INDEX] > 0.5) & (matrix[:, H12_PRESENT_INDEX] > 0.5)
    & ((distance < 12.0) | lane_reliable)
    & (h8_lane_inward > 0.12) & (h8_lane_inward < 3.2)
    & (h12_lane_inward > 0.12) & (h12_lane_inward < 3.2)
  )


def anticipatory_eligibility(matrix, np):
  """Limit the auxiliary classifier to stable, lane-inward radar motion."""
  return _anticipatory_eligibility_from_values(matrix, anticipatory_feature_matrix(matrix, np), np)


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

  @staticmethod
  def _history_is_continuous(
    history: _ObjectHistory, context: RadarLeadContext, obj: FusedRadarObject,
  ) -> bool:
    if not history.observations:
      return True
    previous = history.observations[-1]
    dt = context.time_s - previous.time_s
    if dt <= 0.0 or dt > 0.6:
      return False
    predicted_d = previous.d_rel + previous.v_rel * dt
    predicted_y = previous.y_rel + previous.yv_rel * dt
    d_tolerance = max(8.0, 2.0 + 8.0 * dt)
    y_tolerance = max(3.0, 1.0 + 4.0 * dt)
    return abs(obj.d_rel - predicted_d) <= d_tolerance and abs(obj.y_rel - predicted_y) <= y_tolerance

  def _history_for(
    self, aliases: tuple[str, ...], context: RadarLeadContext, obj: FusedRadarObject,
  ) -> _ObjectHistory:
    matches = {id(state): state for alias in aliases if (state := self._aliases.get(alias)) is not None}
    continuous = {
      identity: state for identity, state in matches.items()
      if self._history_is_continuous(state, context, obj)
    }
    if continuous:
      state = max(continuous.values(), key=lambda candidate: len(candidate.observations))
      for other in continuous.values():
        if other is state:
          continue
        for observation in other.observations:
          if not state.observations or observation.time_s > state.observations[-1].time_s:
            state.observations.append(observation)
        for alias, candidate in tuple(self._aliases.items()):
          if candidate is other:
            self._aliases[alias] = state
    elif matches:
      # Radar track IDs are finite and can be reused for another physical target.
      # A generation suffix also isolates the temporal decision state.
      state = _ObjectHistory(f"{aliases[0]}@{self.frame_index}")
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
      history = self._history_for(aliases, context, obj)
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
      self.sensor_mode = str(model["sensor_mode"].item()) if "sensor_mode" in model.files else "fused"
      self.external_threshold = float(self.thresholds[2])
      self.stationary_model = None
      if "stationary_w1" in model.files:
        self.stationary_model = tuple(model[f"stationary_{name}"].astype(np.float32) for name in (
          "mean", "std", "w1", "b1", "w2", "b2", "w3", "b3", "calibration",
        ))
        self.stationary_threshold = float(model["stationary_threshold"].reshape(-1)[0])
        self.thresholds[2] = 0.5
      self.anticipatory_model = None
      if "anticipatory_w" in model.files:
        anticipatory_names = tuple(str(value) for value in model["anticipatory_feature_names"].tolist())
        if anticipatory_names != ANTICIPATORY_FEATURE_NAMES:
          raise RuntimeError("anticipatory radar lead model schema mismatch")
        self.anticipatory_model = tuple(model[f"anticipatory_{name}"].astype(np.float32) for name in (
          "mean", "std", "w", "b", "calibration",
        ))
        self.anticipatory_threshold = float(model["anticipatory_threshold"].reshape(-1)[0])
      self.temporal_cutin_model = None
      self.cutin_temporal_only = bool(model["cutin_temporal_only"].reshape(-1)[0]) if "cutin_temporal_only" in model.files else False
      if "temporal_cutin_w1" in model.files:
        temporal_names = tuple(str(value) for value in model["temporal_cutin_feature_names"].tolist())
        if temporal_names != TEMPORAL_CUTIN_FEATURE_NAMES:
          raise RuntimeError("temporal cut-in radar lead model schema mismatch")
        self.temporal_cutin_model = tuple(model[f"temporal_cutin_{name}"].astype(np.float32) for name in (
          "mean", "std", "w1", "b1", "w2", "b2", "w3", "b3", "calibration",
        ))
        self.temporal_cutin_threshold = float(model["temporal_cutin_threshold"].reshape(-1)[0])

  @staticmethod
  def _relative_probability(probability, threshold):
    import numpy as np

    probability = np.clip(probability, 1e-6, 1.0 - 1e-6)
    threshold = float(np.clip(threshold, 1e-6, 1.0 - 1e-6))
    relative_logit = np.log(probability / (1.0 - probability)) - math.log(threshold / (1.0 - threshold))
    return 1.0 / (1.0 + np.exp(-np.clip(relative_logit, -30.0, 30.0)))

  @staticmethod
  def _remap_probability(probability, source_threshold: float, target_threshold: float):
    import numpy as np

    probability = np.clip(probability, 1e-6, 1.0 - 1e-6)
    source_threshold = float(np.clip(source_threshold, 1e-6, 1.0 - 1e-6))
    target_threshold = float(np.clip(target_threshold, 1e-6, 1.0 - 1e-6))
    logits = np.log(probability / (1.0 - probability))
    logits -= math.log(source_threshold / (1.0 - source_threshold))
    logits += math.log(target_threshold / (1.0 - target_threshold))
    return 1.0 / (1.0 + np.exp(-np.clip(logits, -30.0, 30.0)))

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
    if self.anticipatory_model is not None:
      mean, std, weights, bias, calibration = self.anticipatory_model
      anticipatory_values = anticipatory_feature_matrix(matrix, np)
      anticipatory_logits = ((anticipatory_values - mean) / std) @ weights + bias.reshape(-1)[0]
      calibration = calibration.reshape(-1)
      anticipatory_logits = anticipatory_logits * calibration[0] + calibration[1]
      anticipatory_probabilities = 1.0 / (1.0 + np.exp(-np.clip(anticipatory_logits, -30.0, 30.0)))
      anticipatory_probabilities = self._remap_probability(
        anticipatory_probabilities, self.anticipatory_threshold, CUTIN_TEMPORAL_THRESHOLD_MAX,
      )
      anticipatory_probabilities = np.where(
        _anticipatory_eligibility_from_values(matrix, anticipatory_values, np), anticipatory_probabilities, 0.0,
      )
      probabilities[:, 1] = np.maximum(probabilities[:, 1], anticipatory_probabilities)
    base_cutin_probabilities = probabilities[:, 1].copy()
    temporal_probabilities = np.zeros(len(features), dtype=np.float32)
    if self.temporal_cutin_model is not None:
      mean, std, w1, b1, w2, b2, w3, b3, calibration = self.temporal_cutin_model
      temporal_values = temporal_cutin_feature_matrix(matrix, np)
      temporal_hidden = np.maximum(((temporal_values - mean) / std) @ w1 + b1, 0.0)
      temporal_hidden = np.maximum(temporal_hidden @ w2 + b2, 0.0)
      temporal_logits = temporal_hidden @ w3 + b3.reshape(-1)[0]
      calibration = calibration.reshape(-1)
      temporal_logits = temporal_logits * calibration[0] + calibration[1]
      temporal_probabilities = 1.0 / (1.0 + np.exp(-np.clip(temporal_logits, -30.0, 30.0)))
      temporal_probabilities = self._remap_probability(
        temporal_probabilities, self.temporal_cutin_threshold, CUTIN_TEMPORAL_THRESHOLD_MAX,
      )
      temporal_probabilities = np.where(
        _temporal_cutin_eligibility_from_values(matrix, temporal_values, np), temporal_probabilities, 0.0,
      )
      probabilities[:, 1] = (
        temporal_probabilities
        if self.cutin_temporal_only
        else np.maximum(probabilities[:, 1], temporal_probabilities)
      )
    if self.stationary_model is not None:
      mean, std, w1, b1, w2, b2, w3, b3, calibration = self.stationary_model
      stationary_hidden = np.maximum(((matrix - mean) / std) @ w1 + b1, 0.0)
      stationary_hidden = np.maximum(stationary_hidden @ w2 + b2, 0.0)
      stationary_logits = stationary_hidden @ w3 + b3
      stationary_logits = stationary_logits * calibration[:, 0] + calibration[:, 1]
      stationary_probabilities = 1.0 / (1.0 + np.exp(-np.clip(stationary_logits[:, 2], -30.0, 30.0)))
      external_probabilities = self._relative_probability(probabilities[:, 2], self.external_threshold)
      v_lead_index = MODEL_FEATURE_NAMES.index("v_lead")
      stopped = np.abs(matrix[:, v_lead_index]) < 1.8
      external_probabilities[stopped] = np.maximum(
        external_probabilities[stopped],
        self._relative_probability(stationary_probabilities[stopped], self.stationary_threshold),
      )
      probabilities[:, 2] = external_probabilities
    output = []
    for sample, (lead_prob, cutin_prob, external_prob), base_cutin_prob, temporal_cutin_prob in zip(
      features, probabilities, base_cutin_probabilities, temporal_probabilities, strict=True,
    ):
      closing = max(0.0, -sample.radar_object.v_rel)
      ttc_risk = min(1.0, closing / 8.0) * min(1.0, 25.0 / max(sample.radar_object.d_rel, 1.0))
      risk_prob = float(max(cutin_prob, lead_prob * ttc_risk, external_prob * ttc_risk))
      output.append(RadarLeadPrediction(
        sample, float(lead_prob), float(cutin_prob), risk_prob, float(external_prob),
        float(base_cutin_prob), float(temporal_cutin_prob),
      ))
    return tuple(output)


class RadarLeadDecisionFilter:
  """Model-owned decisions with small physical gates and identity hysteresis."""

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
      sample.track_age >= 5 and (obj.front_track_id is not None or obj.scc_track_id is not None)
      and abs(sample.d_path) < 1.8
    )

  @staticmethod
  def _group_predictions(predictions: Iterable[RadarLeadPrediction]) -> tuple[RadarLeadPrediction, ...]:
    grouped: dict[str, list[RadarLeadPrediction]] = {}
    for prediction in predictions:
      grouped.setdefault(prediction.features.object_id, []).append(prediction)

    current: list[RadarLeadPrediction] = []
    for samples in grouped.values():
      owner = max(samples, key=lambda item: max(item.lead_prob, item.base_cutin_prob, item.external_prob))
      temporal_prob = max(item.temporal_cutin_prob for item in samples)
      if temporal_prob > owner.temporal_cutin_prob:
        owner = replace(
          owner,
          temporal_cutin_prob=temporal_prob,
          cutin_prob=max(owner.cutin_prob, temporal_prob),
          risk_prob=max(owner.risk_prob, temporal_prob),
        )
      current.append(owner)
    return tuple(current)

  @staticmethod
  def _cutin_geometry(prediction: RadarLeadPrediction) -> tuple[bool, bool, bool, bool, bool]:
    """Return (usable, strong, holdable, near-corner, predicted-entry)."""
    sample = prediction.features
    obj = sample.radar_object
    current_d_path = abs(sample.d_path)
    future_d_path = abs(sample.d_path_future)
    lane_half_width = min(2.2, max(1.5, sample.values[LANE_HALF_WIDTH_INDEX]))
    body_entry_limit = lane_half_width + CUTIN_VEHICLE_HALF_WIDTH_M
    lane_reliable = (
      sample.values[LANE1_PROB_INDEX] > 0.35
      and sample.values[LANE2_PROB_INDEX] > 0.35
    )

    has_front = obj.front_track_id is not None or obj.scc_track_id is not None
    corner_only = obj.corner_track_id is not None and not has_front
    front_only = has_front and obj.corner_track_id is None
    distance_sane = 1.0 < obj.d_rel < 65.0 and (
      not corner_only or obj.d_rel < CORNER_ONLY_CUTIN_MAX_DREL_M
    ) and (not front_only or obj.d_rel >= FRONT_ONLY_CUTIN_MIN_DREL_M)
    speed_sane = obj.v_lead > 1.0 or (obj.d_rel < 12.0 and obj.v_lead > -0.5)
    if not distance_sane or not speed_sane or sample.track_age < 5:
      return False, False, False, False, False
    if obj.d_rel >= 12.0 and not lane_reliable:
      return False, False, False, False, False

    side = 1.0 if sample.d_path >= 0.0 else -1.0
    instant_inward = -side * obj.yv_rel
    history_inward: list[float] = []
    for present_index, dt_index, d_path_index in (
      (H4_PRESENT_INDEX, H4_DT_INDEX, H4_D_PATH_INDEX),
      (H8_PRESENT_INDEX, H8_DT_INDEX, H8_D_PATH_INDEX),
    ):
      if sample.values[present_index] > 0.5:
        dt = max(1e-3, sample.values[dt_index])
        history_inward.append((abs(sample.values[d_path_index]) - current_d_path) / dt)

    future_inward = current_d_path - future_d_path
    history_supports_inward = bool(history_inward) and max(history_inward) > 0.05
    history_strongly_outward = bool(history_inward) and max(history_inward) < -0.25
    inward = future_inward > 0.08 or instant_inward > 0.08 or history_supports_inward
    strongly_outward = (
      future_inward < -0.40
      and instant_inward < -0.10
      and (not history_inward or history_strongly_outward)
    )
    body_entering_lane = min(current_d_path, future_d_path) <= body_entry_limit + 0.15
    near_corner_anticipation = (
      corner_only
      and 5.0 <= obj.d_rel < 12.0
      and current_d_path <= body_entry_limit + 0.80
      and len(history_inward) >= 2
      and min(history_inward) > 0.25
    )
    predicted_entry = (
      current_d_path > body_entry_limit + 0.15
      and future_d_path <= body_entry_limit
      and inward
    )
    center_in_lane = current_d_path <= lane_half_width + 0.15
    holdable = (
      current_d_path <= body_entry_limit + 0.60
      and future_d_path <= current_d_path + 0.35
      and not strongly_outward
    )
    usable = (
      (body_entering_lane or near_corner_anticipation)
      and not strongly_outward
      and (inward or center_in_lane)
    )

    # Front radar lateral/range noise is largest in the immediate near field.
    # Require established history there; fused or corner-confirmed targets do not
    # need this extra sensor-quality gate.
    front_only_near = front_only and obj.d_rel < 8.0
    if front_only_near and (sample.track_age < 8 or len(history_inward) < 2):
      usable = False

    strong = (
      usable
      and inward
      and future_d_path <= body_entry_limit
      and sample.track_age >= 8
      and (obj.d_rel < 12.0 or lane_reliable)
    )
    return usable, strong, holdable, near_corner_anticipation, predicted_entry

  def update(self, time_s: float, predictions: Iterable[RadarLeadPrediction]) -> RadarLeadDecision:
    current = self._group_predictions(predictions)
    seen: set[str] = set()
    lead_active: list[RadarLeadPrediction] = []
    cutin_active: list[RadarLeadPrediction] = []
    external_active: list[RadarLeadPrediction] = []

    for prediction in current:
      object_id = prediction.features.object_id
      seen.add(object_id)
      state = self._states.setdefault(object_id, _DecisionState())
      state.last_seen = time_s
      reliable = self._reliable(prediction)

      state.lead_ema = max(prediction.lead_prob, 0.65 * state.lead_ema + 0.35 * prediction.lead_prob)
      state.external_ema = max(
        prediction.external_prob, 0.65 * state.external_ema + 0.35 * prediction.external_prob,
      )
      state.lead_hits = state.lead_hits + 1 if reliable and state.lead_ema >= self.lead_threshold else 0
      state.external_hits = (
        state.external_hits + 1
        if reliable and state.external_ema >= self.external_threshold
        else 0
      )

      (
        cutin_usable,
        strong_cutin_geometry,
        holdable_cutin_geometry,
        near_corner_anticipation,
        predicted_cutin_entry,
      ) = self._cutin_geometry(prediction)
      obj = prediction.features.radar_object
      side = 1.0 if prediction.features.d_path >= 0.0 else -1.0
      close_history_inward = []
      for present_index, dt_index, d_path_index in (
        (H4_PRESENT_INDEX, H4_DT_INDEX, H4_D_PATH_INDEX),
        (H8_PRESENT_INDEX, H8_DT_INDEX, H8_D_PATH_INDEX),
      ):
        if prediction.features.values[present_index] > 0.5:
          dt = max(1e-3, prediction.features.values[dt_index])
          close_history_inward.append(
            (abs(prediction.features.values[d_path_index]) - abs(prediction.features.d_path)) / dt
          )
      close_corner_entry = (
        obj.corner_track_id is not None
        and obj.front_track_id is None
        and obj.scc_track_id is None
        and obj.d_rel < 6.0
        and obj.v_lead > 2.0
        and strong_cutin_geometry
        and -side * obj.yv_rel > 0.25
        and len(close_history_inward) >= 2
        and min(close_history_inward) > 0.25
        and prediction.base_cutin_prob >= 0.95
      )
      effective_cutin_prob = max(
        prediction.cutin_prob,
        self.cutin_threshold if close_corner_entry else 0.0,
      )
      state.cutin_ema = max(effective_cutin_prob, 0.65 * state.cutin_ema)
      current_aliases = frozenset(prediction.features.aliases)
      cutin_entry_threshold = min(
        self.cutin_threshold,
        0.50 if near_corner_anticipation else 0.75 if predicted_cutin_entry else self.cutin_threshold,
      )
      if cutin_usable and effective_cutin_prob >= cutin_entry_threshold:
        state.cutin_hits = (
          state.cutin_hits + 1
          if state.cutin_hits > 0 and state.cutin_hit_aliases.intersection(current_aliases)
          else 1
        )
        state.cutin_hit_aliases = current_aliases
      else:
        state.cutin_hits = 0
        state.cutin_hit_aliases = frozenset()

      if state.lead_hits >= 2:
        state.lead_active_until = time_s + 0.35
      immediate_cutin = (
        strong_cutin_geometry and prediction.cutin_prob >= (
          min(self.cutin_threshold, 0.75)
          if predicted_cutin_entry
          else max(0.92, self.cutin_threshold + 0.08)
        )
      )
      if state.cutin_hits >= 2 or immediate_cutin:
        state.cutin_active_until = time_s + 0.45
        state.cutin_evidence_time = time_s
        state.cutin_aliases = current_aliases
      if state.external_hits >= 2:
        state.external_active_until = time_s + 0.35

      if state.lead_ema < 0.25:
        state.lead_active_until = min(state.lead_active_until, time_s + 0.10)
      if not holdable_cutin_geometry:
        state.cutin_active_until = min(state.cutin_active_until, time_s + 0.10)
      if state.external_ema < 0.25:
        state.external_active_until = min(state.external_active_until, time_s + 0.10)

      if time_s < state.lead_active_until:
        lead_active.append(prediction)

      same_cutin_sensor = bool(state.cutin_aliases.intersection(current_aliases))
      sticky_cutin = (
        same_cutin_sensor
        and holdable_cutin_geometry
        and time_s <= state.cutin_evidence_time + CUTIN_STICKY_MAX_S
      )
      cutin_is_active = time_s < state.cutin_active_until and (
        effective_cutin_prob >= CUTIN_ACTIVE_CURRENT_MIN or sticky_cutin
      )
      if cutin_is_active:
        if sticky_cutin:
          state.cutin_active_until = min(
            max(state.cutin_active_until, time_s + 0.20),
            state.cutin_evidence_time + CUTIN_STICKY_MAX_S,
          )
        active_cutin_prob = max(effective_cutin_prob, state.cutin_ema)
        cutin_active.append(replace(
          prediction,
          cutin_prob=active_cutin_prob,
          risk_prob=max(prediction.risk_prob, active_cutin_prob),
        ))

      if time_s < state.external_active_until and prediction.external_prob >= EXTERNAL_ACTIVE_CURRENT_MIN:
        external_active.append(prediction)

    for object_id, state in tuple(self._states.items()):
      if object_id not in seen and time_s - state.last_seen > 0.5:
        self._states.pop(object_id, None)

    return RadarLeadDecision(
      lead_candidates=tuple(sorted(
        lead_active, key=lambda item: (-item.lead_prob, item.features.radar_object.d_rel),
      )[:2]),
      cutin_candidates=tuple(sorted(
        cutin_active, key=lambda item: (-item.risk_prob, item.features.radar_object.d_rel),
      )[:2]),
      external_candidates=tuple(sorted(
        external_active, key=lambda item: (-item.external_prob, item.features.radar_object.d_rel),
      )[:2]),
    )
