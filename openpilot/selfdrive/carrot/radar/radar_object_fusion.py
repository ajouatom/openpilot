#!/usr/bin/env python3
"""Stateful front/corner radar object fusion for offline model development."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Protocol


class RadarPointLike(Protocol):
  track_id: int
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  v_lead: float
  a_lead: float
  j_lead: float
  measured: bool
  source: str


@dataclass(frozen=True)
class FusedRadarObject:
  object_id: str
  d_rel: float
  y_rel: float
  v_rel: float
  a_rel: float
  yv_rel: float
  v_lead: float
  front_track_id: int | None
  corner_track_id: int | None
  scc_track_id: int | None
  front_d_rel: float | None
  corner_d_rel: float | None
  front_y_rel: float | None
  corner_y_rel: float | None
  front_v_rel: float | None
  corner_v_rel: float | None
  distance_source: str
  lateral_source: str
  match_confidence: float
  pair_age: int
  a_lead: float = 0.0
  j_lead: float = 0.0

  @property
  def trusted_for_control(self) -> bool:
    return (
      self.front_track_id is not None
      and self.corner_track_id is not None
      and self.pair_age >= 5
      and self.match_confidence >= 0.65
    )


@dataclass
class _PairState:
  hits: int = 0
  quality_hits: int = 0
  misses: int = 0
  distance_offset: float = 0.0
  fused_d_rel: float | None = None
  fused_v_rel: float = 0.0
  last_time_s: float | None = None
  quality: float = 0.0


def _finite_or(value: float, fallback: float) -> float:
  return value if math.isfinite(value) else fallback


def _corner_distance_weight(source: str, distance: float) -> float:
  if source == "corner180":
    if distance < 10.0:
      return 0.82
    if distance < 20.0:
      return 0.72
    if distance < 40.0:
      return 0.55
  else:
    if distance < 10.0:
      return 0.58
    if distance < 20.0:
      return 0.55
    if distance < 40.0:
      return 0.45
  if distance < 80.0:
    return 0.25
  return 0.10


class RadarObjectFusion:
  def __init__(self, include_scc: bool = False, confirm_frames: int = 3) -> None:
    self.include_scc = include_scc
    self.confirm_frames = max(1, confirm_frames)
    self._pairs: dict[tuple[int, int], _PairState] = {}

  @staticmethod
  def _match_cost(front: RadarPointLike, corner: RadarPointLike, previous: _PairState | None) -> float | None:
    distance_delta = front.d_rel - corner.d_rel
    velocity_delta = front.v_rel - corner.v_rel
    lateral_delta = front.y_rel - corner.y_rel
    if abs(distance_delta) > 6.0 or abs(velocity_delta) > 1.5 or abs(lateral_delta) > 2.4:
      return None
    if abs(front.y_rel) > 0.8 and abs(corner.y_rel) > 0.8 and front.y_rel * corner.y_rel < 0.0:
      return None
    expected_offset = previous.distance_offset if previous is not None and previous.hits >= 2 else 1.25
    persistence_bonus = 0.45 if previous is not None and previous.misses <= 1 else 0.0
    return (
      abs(distance_delta - expected_offset) / 5.0
      + abs(velocity_delta) / 1.5
      + abs(lateral_delta) / 2.4
      - persistence_bonus
    )

  def _matched_pairs(
    self, fronts: list[RadarPointLike], corners: list[RadarPointLike],
  ) -> list[tuple[RadarPointLike, RadarPointLike, _PairState]]:
    front_by_id = {point.track_id: point for point in fronts}
    corner_by_id = {point.track_id: point for point in corners}
    matched: list[tuple[RadarPointLike, RadarPointLike, _PairState]] = []
    used_front: set[int] = set()
    used_corner: set[int] = set()
    matched_keys: set[tuple[int, int]] = set()

    # Preserve established associations before considering new geometry matches.
    for key, state in sorted(self._pairs.items(), key=lambda item: item[1].hits, reverse=True):
      if state.quality_hits < self.confirm_frames:
        continue
      front = front_by_id.get(key[0])
      corner = corner_by_id.get(key[1])
      if front is None or corner is None or front.track_id in used_front or corner.track_id in used_corner:
        continue
      cost = self._match_cost(front, corner, state)
      if cost is None or cost > 2.2:
        continue
      self._update_pair_state(front, corner, state)
      matched.append((front, corner, state))
      matched_keys.add(key)
      used_front.add(front.track_id)
      used_corner.add(corner.track_id)

    candidates: list[tuple[float, RadarPointLike, RadarPointLike]] = []
    for front in fronts:
      if front.track_id in used_front:
        continue
      for corner in corners:
        if corner.track_id in used_corner:
          continue
        state = self._pairs.get((front.track_id, corner.track_id))
        cost = self._match_cost(front, corner, state)
        if cost is not None:
          candidates.append((cost, front, corner))

    for _, front, corner in sorted(candidates, key=lambda item: item[0]):
      if front.track_id in used_front or corner.track_id in used_corner:
        continue
      key = (front.track_id, corner.track_id)
      state = self._pairs.setdefault(key, _PairState(distance_offset=front.d_rel - corner.d_rel))
      self._update_pair_state(front, corner, state)
      matched.append((front, corner, state))
      matched_keys.add(key)
      used_front.add(front.track_id)
      used_corner.add(corner.track_id)

    for key, state in tuple(self._pairs.items()):
      if key in matched_keys:
        continue
      state.misses += 1
      if state.misses > 4:
        self._pairs.pop(key, None)
    return matched

  @staticmethod
  def _update_pair_state(front: RadarPointLike, corner: RadarPointLike, state: _PairState) -> None:
    if state.misses:
      state.hits = max(0, state.hits - state.misses * 2)
      state.quality_hits = 0
    state.hits += 1
    state.misses = 0
    distance_delta = front.d_rel - corner.d_rel
    distance_quality = max(0.0, 1.0 - abs(distance_delta - state.distance_offset) / 3.0)
    velocity_quality = max(0.0, 1.0 - abs(front.v_rel - corner.v_rel) / 1.5)
    lateral_quality = max(0.0, 1.0 - abs(front.y_rel - corner.y_rel) / 2.4)
    instant_quality = 0.25 * distance_quality + 0.45 * velocity_quality + 0.30 * lateral_quality
    state.quality = instant_quality if state.hits == 1 else 0.82 * state.quality + 0.18 * instant_quality
    state.quality_hits = state.quality_hits + 1 if instant_quality >= 0.55 else 0
    if abs(distance_delta - state.distance_offset) < 2.0 and instant_quality > 0.55:
      state.distance_offset = state.distance_offset * 0.97 + distance_delta * 0.03

  def _pair_confirmed(self, state: _PairState) -> bool:
    return state.quality_hits >= self.confirm_frames and state.quality >= 0.55 and state.misses == 0

  @staticmethod
  def _robust_weight(base_weight: float, innovation: float, scale: float) -> float:
    return base_weight / (1.0 + (innovation / max(scale, 0.05)) ** 2)

  def _fuse_pair(
    self, time_s: float, front: RadarPointLike, corner: RadarPointLike, state: _PairState,
  ) -> FusedRadarObject:
    aligned_corner_d = corner.d_rel + state.distance_offset
    corner_weight = _corner_distance_weight(corner.source, min(front.d_rel, aligned_corner_d))
    front_weight = 1.0 - corner_weight
    if state.fused_d_rel is None or state.last_time_s is None:
      fused_d_rel = front.d_rel * front_weight + aligned_corner_d * corner_weight
    else:
      dt = min(max(time_s - state.last_time_s, 0.0), 0.2)
      predicted_d_rel = state.fused_d_rel + state.fused_v_rel * dt
      front_weight = self._robust_weight(front_weight, front.d_rel - predicted_d_rel, 0.55)
      corner_weight = self._robust_weight(corner_weight, aligned_corner_d - predicted_d_rel, 0.35)
      total_weight = max(front_weight + corner_weight, 1e-3)
      measurement = (front.d_rel * front_weight + aligned_corner_d * corner_weight) / total_weight
      fused_d_rel = predicted_d_rel + 0.72 * (measurement - predicted_d_rel)
      fused_d_rel = min(
        max(fused_d_rel, min(front.d_rel, aligned_corner_d) - 0.75),
        max(front.d_rel, aligned_corner_d) + 0.75,
      )
    fused_v_rel = front.v_rel * 0.75 + corner.v_rel * 0.25
    state.fused_d_rel = fused_d_rel
    state.fused_v_rel = fused_v_rel
    state.last_time_s = time_s
    confidence = min(1.0, state.quality_hits / max(self.confirm_frames + 2, 1)) * state.quality
    distance_source = "corner-weighted" if corner_weight > front_weight else "front-weighted"
    return FusedRadarObject(
      object_id=f"fc:{front.track_id}:{corner.track_id}",
      d_rel=fused_d_rel,
      y_rel=corner.y_rel,
      v_rel=fused_v_rel,
      a_rel=_finite_or(front.a_rel, corner.a_rel),
      yv_rel=corner.yv_rel,
      v_lead=front.v_lead * 0.75 + corner.v_lead * 0.25,
      front_track_id=front.track_id,
      corner_track_id=corner.track_id,
      scc_track_id=None,
      front_d_rel=front.d_rel,
      corner_d_rel=corner.d_rel,
      front_y_rel=front.y_rel,
      corner_y_rel=corner.y_rel,
      front_v_rel=front.v_rel,
      corner_v_rel=corner.v_rel,
      distance_source=distance_source,
      lateral_source=corner.source,
      match_confidence=confidence,
      pair_age=state.quality_hits,
      a_lead=_finite_or(getattr(front, "a_lead", 0.0), 0.0),
      j_lead=_finite_or(getattr(front, "j_lead", 0.0), 0.0),
    )

  @staticmethod
  def _single(point: RadarPointLike) -> FusedRadarObject:
    is_corner = point.source.startswith("corner")
    is_scc = point.source == "scc"
    return FusedRadarObject(
      object_id=f"{point.source}:{point.track_id}",
      d_rel=point.d_rel,
      y_rel=point.y_rel,
      v_rel=point.v_rel,
      a_rel=point.a_rel,
      yv_rel=point.yv_rel,
      v_lead=point.v_lead,
      front_track_id=None if is_corner or is_scc else point.track_id,
      corner_track_id=point.track_id if is_corner else None,
      scc_track_id=point.track_id if is_scc else None,
      front_d_rel=None if is_corner or is_scc else point.d_rel,
      corner_d_rel=point.d_rel if is_corner else None,
      front_y_rel=None if is_corner or is_scc else point.y_rel,
      corner_y_rel=point.y_rel if is_corner else None,
      front_v_rel=None if is_corner or is_scc else point.v_rel,
      corner_v_rel=point.v_rel if is_corner else None,
      distance_source=point.source,
      lateral_source=point.source,
      match_confidence=0.35,
      pair_age=1,
      a_lead=_finite_or(getattr(point, "a_lead", 0.0), 0.0),
      j_lead=_finite_or(getattr(point, "j_lead", 0.0), 0.0),
    )

  def update(self, time_s: float, points: Iterable[RadarPointLike]) -> tuple[FusedRadarObject, ...]:
    measured = [point for point in points if point.measured and point.d_rel > 0.2]
    fronts = [point for point in measured if point.source == "frontRadar"]
    corners = [point for point in measured if point.source.startswith("corner")]
    scc_points = [point for point in measured if point.source == "scc"]
    matched = self._matched_pairs(fronts, corners)
    used_front = {front.track_id for front, _, state in matched if self._pair_confirmed(state)}
    used_corner = {corner.track_id for _, corner, state in matched if self._pair_confirmed(state)}
    objects = [
      self._fuse_pair(time_s, front, corner, state)
      for front, corner, state in matched
      if self._pair_confirmed(state)
    ]
    objects.extend(self._single(front) for front in fronts if front.track_id not in used_front)
    objects.extend(self._single(corner) for corner in corners if corner.track_id not in used_corner)

    if self.include_scc:
      for scc in scc_points:
        duplicate = min(
          objects,
          key=lambda obj: abs(obj.d_rel - scc.d_rel) + 0.5 * abs(obj.v_rel - scc.v_rel),
          default=None,
        )
        if duplicate is None or abs(duplicate.d_rel - scc.d_rel) > 3.0 or abs(duplicate.v_rel - scc.v_rel) > 1.5:
          if not fronts:
            objects.append(self._single(scc))
    return tuple(sorted(objects, key=lambda obj: obj.d_rel))
