#!/usr/bin/env python3
"""Small staged control policy over path-compensated radar motion evidence.

V3 deliberately does not estimate motion from raw dPath history. Ego motion,
curvature, delayed samples, and ID continuity belong to the observation layer.
This module only converts compensated physical evidence into control stages.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from enum import IntEnum

from openpilot.selfdrive.carrot.radar_motion.occupancy_v2 import (
  OccupancyEstimate,
  OccupancyStage,
)


DOWNGRADE_HOLD_S = 0.40
MISSING_HOLD_S = 0.40


class V3Stage(IntEnum):
  CLEAR = 0
  WATCH = 1
  CAUTION = 2
  PREDECEL = 3
  LEAD = 4


@dataclass(frozen=True)
class V3Evidence:
  occupancy: OccupancyEstimate

  @property
  def identity(self) -> tuple[str, int]:
    return self.occupancy.evidence.identity


@dataclass(frozen=True)
class V3Estimate:
  evidence: V3Evidence
  stage: V3Stage
  extension_stage: V3Stage
  score: float
  reason: str


@dataclass
class _StageState:
  stage: V3Stage = V3Stage.CLEAR
  pending_stage: V3Stage = V3Stage.CLEAR
  pending_since_s: float = 0.0
  downgrade_since_s: float | None = None
  reason: str = "baseline"
  last_seen_s: float = -math.inf


def _base_stage(stage: OccupancyStage) -> V3Stage:
  if stage >= OccupancyStage.LEAD:
    return V3Stage.LEAD
  if stage >= OccupancyStage.LIMIT:
    return V3Stage.PREDECEL
  if stage >= OccupancyStage.WATCH:
    return V3Stage.WATCH
  return V3Stage.CLEAR


def _inward_components(
  estimate: OccupancyEstimate,
) -> tuple[float, float, float]:
  evidence = estimate.evidence
  inward_sign = -math.copysign(1.0, evidence.d_path)
  return (
    inward_sign * evidence.d_path_rate_short,
    inward_sign * evidence.d_path_rate_long,
    inward_sign * evidence.reported_normal_speed,
  )


def _entry_is_reachable(estimate: OccupancyEstimate) -> bool:
  overlap_s = estimate.time_to_overlap_s
  collision_s = estimate.time_to_collision_s
  return (
    overlap_s is not None
    and (collision_s is None or overlap_s <= collision_s + 0.35)
  )


def _front_position_history_stage(
  estimate: OccupancyEstimate,
) -> V3Stage:
  """Recover real front cut-ins when instantaneous lateral speed is absent."""
  evidence = estimate.evidence
  if not (
    evidence.source == "frontRadar"
    and 8.0 <= evidence.d_rel <= 20.0
    and evidence.v_rel < -0.5
    and estimate.time_to_collision_s is not None
    and estimate.time_to_collision_s > 1.2
    and _entry_is_reachable(estimate)
    and evidence.history_count >= 12
    and evidence.motion_consistency >= 0.80
    and evidence.recent_motion_support >= 0.65
  ):
    return V3Stage.CLEAR

  short, long, _reported = _inward_components(estimate)
  displacement = evidence.directional_inward_displacement_m
  consistency = evidence.directional_consistency
  sample_ratio = evidence.directional_inward_sample_ratio
  if (
    displacement >= 0.60
    and consistency >= 0.75
    and sample_ratio >= 0.60
    and short >= 0.60
    and long >= 0.20
  ):
    return V3Stage.LEAD
  if (
    displacement >= 0.50
    and consistency >= 0.70
    and sample_ratio >= 0.55
    and short >= 0.50
    and long >= 0.15
  ):
    return V3Stage.PREDECEL
  if (
    displacement >= 0.35
    and consistency >= 0.62
    and sample_ratio >= 0.50
    and short >= 0.35
    and long >= 0.10
  ):
    return V3Stage.CAUTION
  return V3Stage.CLEAR


def _fused_corner_stage(estimate: OccupancyEstimate) -> V3Stage:
  """Stage a corroborated corner target before ordinary L2 confirmation."""
  evidence = estimate.evidence
  overlap_s = estimate.time_to_overlap_s
  if not (
    evidence.source.startswith("corner")
    and evidence.cross_sensor_confirmed
    and not evidence.current_path_occupancy
    and 8.0 <= evidence.d_rel <= 45.0
    and evidence.v_rel <= -1.5
    and overlap_s is not None
    and 0.50 <= overlap_s <= 3.0
    and _entry_is_reachable(estimate)
    and evidence.directional_inward_displacement_m >= 0.30
    and evidence.directional_consistency >= 0.90
    and evidence.directional_inward_sample_ratio >= 0.75
    and evidence.motion_consistency >= 0.80
    and evidence.recent_motion_support >= 0.80
    and estimate.confidence >= 0.80
    and estimate.control_urgency >= 0.70
  ):
    return V3Stage.CLEAR

  short, long, reported = _inward_components(estimate)
  if min(short, long) < 0.35 or reported < 0.40:
    return V3Stage.CLEAR
  if estimate.risk_score >= 0.78 and overlap_s <= 1.50:
    return V3Stage.LEAD
  return V3Stage.PREDECEL


def _extension_stage(
  estimate: OccupancyEstimate,
) -> tuple[V3Stage, str]:
  front = _front_position_history_stage(estimate)
  corner = _fused_corner_stage(estimate)
  if front >= corner and front > V3Stage.CLEAR:
    return front, "front-position-history"
  if corner > V3Stage.CLEAR:
    return corner, "fused-corner-motion"
  return V3Stage.CLEAR, "baseline"


class RadarOccupancyModelV3:
  """A short staged policy; tracking and path compensation stay upstream."""

  def __init__(self) -> None:
    self._states: dict[tuple[str, int], _StageState] = {}

  def update(
    self,
    time_s: float,
    occupancy_values: Iterable[OccupancyEstimate],
  ) -> tuple[V3Estimate, ...]:
    estimates = []
    seen = set()
    for occupancy in occupancy_values:
      evidence = V3Evidence(occupancy)
      seen.add(evidence.identity)
      state = self._states.setdefault(evidence.identity, _StageState())
      state.last_seen_s = time_s
      baseline = _base_stage(occupancy.stage)
      extension, extension_reason = _extension_stage(occupancy)
      raw = max(baseline, extension)
      raw_reason = extension_reason if extension > baseline else "baseline"

      if baseline > state.stage:
        state.stage = baseline
        state.reason = "baseline"
        state.pending_stage = baseline
        state.pending_since_s = time_s
        state.downgrade_since_s = None

      if raw > state.stage:
        if state.pending_stage <= state.stage:
          state.pending_since_s = time_s
        state.pending_stage = raw
        confirmation_s = (
          0.0
          if raw_reason == "baseline"
          or raw <= V3Stage.PREDECEL
          or state.stage >= V3Stage.PREDECEL
          else 0.10
        )
        if time_s - state.pending_since_s + 1e-6 >= confirmation_s:
          state.stage = raw
          state.reason = raw_reason
          state.downgrade_since_s = None
      elif raw < state.stage:
        if state.downgrade_since_s is None:
          state.downgrade_since_s = time_s
        if time_s - state.downgrade_since_s + 1e-6 >= DOWNGRADE_HOLD_S:
          state.stage = raw
          state.reason = raw_reason
          state.pending_stage = raw
          state.pending_since_s = time_s
      else:
        state.pending_stage = raw
        state.pending_since_s = time_s
        state.downgrade_since_s = None
        if raw_reason != "baseline":
          state.reason = raw_reason

      score = max(
        occupancy.risk_score,
        occupancy.lead_score,
        0.20 * int(extension >= V3Stage.CAUTION),
        0.50 * int(extension >= V3Stage.PREDECEL),
        0.82 * int(extension >= V3Stage.LEAD),
      )
      estimates.append(V3Estimate(
        evidence=evidence,
        stage=state.stage,
        extension_stage=extension,
        score=min(score, 1.0),
        reason=state.reason,
      ))

    stale = (
      identity for identity, state in self._states.items()
      if identity not in seen and time_s - state.last_seen_s > MISSING_HOLD_S
    )
    for identity in tuple(stale):
      self._states.pop(identity, None)
    return tuple(estimates)


__all__ = (
  "RadarOccupancyModelV3",
  "V3Estimate",
  "V3Evidence",
  "V3Stage",
)
