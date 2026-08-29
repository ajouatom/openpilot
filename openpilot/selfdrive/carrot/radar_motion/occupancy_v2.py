#!/usr/bin/env python3
"""Compact uncertainty-aware lane occupancy state model.

This module intentionally owns no radar decoding, path projection, object-ID
association, or lead selection policy.  It consumes continuous physical track
estimates and turns them into two independent decisions:

* LIMIT: the object is likely enough to enter that acceleration should stop.
* LEAD: the object is committed enough to become a control lead.

Keeping these decisions separate lets early evidence produce a bounded response
without making every adjacent-lane drift a braking target.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from enum import IntEnum


EGO_HALF_WIDTH_M = 0.90
TARGET_HALF_WIDTH_M = 0.90
BASE_REFLECTION_MARGIN_M = 0.18
CROSS_SENSOR_REFLECTION_MARGIN_M = 0.22
VISION_REFLECTION_MARGIN_M = 0.12

WATCH_ENTER_SCORE = 0.16
LIMIT_ENTER_SCORE = 0.31
LIMIT_EXIT_SCORE = 0.23
LEAD_ENTER_SCORE = 0.48
LEAD_EXIT_SCORE = 0.38

LIMIT_CONFIRMATION_S = 0.05
LEAD_CONFIRMATION_S = 0.10
DOWNGRADE_HOLD_S = 0.45
MISSING_HOLD_S = 0.40

RISK_LOOKAHEAD_S = 3.25
LEAD_LOOKAHEAD_S = 1.65


class OccupancyStage(IntEnum):
  CLEAR = 0
  WATCH = 1
  LIMIT = 2
  LEAD = 3
  OCCUPIED = 4


@dataclass(frozen=True)
class OccupancyEvidence:
  source: str
  track_id: int
  continuity_id: int
  d_rel: float
  v_rel: float
  v_lead: float
  v_ego: float
  d_path: float
  d_path_rate_short: float
  d_path_rate_long: float
  reported_normal_speed: float
  normal_speed_disagreement: float
  directional_inward_displacement_m: float
  directional_consistency: float
  directional_inward_sample_ratio: float
  motion_consistency: float
  recent_motion_support: float
  history_count: int
  uncertainty: float
  current_path_occupancy: bool
  cross_sensor_confirmed: bool = False
  vision_supported: bool = False

  @property
  def identity(self) -> tuple[str, int]:
    return self.source, self.continuity_id


@dataclass(frozen=True)
class OccupancyEstimate:
  evidence: OccupancyEvidence
  stage: OccupancyStage
  occupancy_score: float
  risk_score: float
  lead_score: float
  intent_score: float
  confidence: float
  path_clearance_m: float
  inward_rate_mps: float
  time_to_overlap_s: float | None
  time_gap_s: float | None
  time_to_collision_s: float | None
  control_urgency: float


@dataclass
class _StageState:
  stage: OccupancyStage = OccupancyStage.CLEAR
  pending_stage: OccupancyStage = OccupancyStage.CLEAR
  pending_since_s: float = 0.0
  last_upgrade_s: float = -math.inf
  last_seen_s: float = -math.inf


def _clamp01(value: float) -> float:
  return min(max(float(value), 0.0), 1.0)


def _smoothstep(low: float, high: float, value: float) -> float:
  if high <= low:
    return float(value >= high)
  x = _clamp01((float(value) - low) / (high - low))
  return x * x * (3.0 - 2.0 * x)


def _reverse_smoothstep(low: float, high: float, value: float) -> float:
  return 1.0 - _smoothstep(low, high, value)


def _geometric_mean(values: Iterable[float]) -> float:
  values = tuple(max(float(value), 1e-6) for value in values)
  if not values:
    return 0.0
  return math.exp(sum(math.log(value) for value in values) / len(values))


def _inward_rate(evidence: OccupancyEvidence) -> float:
  if abs(evidence.d_path) <= 1e-6:
    return 0.0
  inward_sign = -math.copysign(1.0, evidence.d_path)
  rates = sorted((
    max(0.0, inward_sign * evidence.d_path_rate_short),
    max(0.0, inward_sign * evidence.d_path_rate_long),
    max(0.0, inward_sign * evidence.reported_normal_speed),
  ))
  # The median rejects one quantized or delayed velocity channel without
  # introducing a source-specific hard gate.
  return rates[1]


def _reflection_margin(evidence: OccupancyEvidence) -> float:
  return (
    BASE_REFLECTION_MARGIN_M
    + CROSS_SENSOR_REFLECTION_MARGIN_M
    * float(evidence.cross_sensor_confirmed)
    + VISION_REFLECTION_MARGIN_M * float(evidence.vision_supported)
  )


def _track_confidence(evidence: OccupancyEvidence) -> float:
  age = _smoothstep(3.0, 14.0, evidence.history_count)
  direction = _geometric_mean((
    _smoothstep(0.48, 0.92, evidence.directional_consistency),
    _smoothstep(0.42, 0.82, evidence.directional_inward_sample_ratio),
  ))
  motion = _geometric_mean((
    _smoothstep(0.42, 0.90, evidence.motion_consistency),
    _smoothstep(0.42, 0.90, evidence.recent_motion_support),
  ))
  sensor_support = min(
    1.0,
    0.62
    + 0.24 * float(evidence.cross_sensor_confirmed)
    + 0.14 * float(evidence.vision_supported),
  )
  uncertainty = _reverse_smoothstep(0.55, 2.20, evidence.uncertainty)
  normal_speed_agreement = _reverse_smoothstep(
    0.08, 0.35, evidence.normal_speed_disagreement,
  )
  position_history_supported = (
    evidence.directional_inward_displacement_m >= 0.50
    and evidence.directional_consistency >= 0.82
    and evidence.directional_inward_sample_ratio >= 0.78
    and evidence.recent_motion_support >= 0.70
  )
  if position_history_supported:
    # Front radar commonly quantizes its lateral velocity to zero. Sustained
    # monotonic position history is independent support for real motion.
    normal_speed_agreement = max(normal_speed_agreement, 0.85)
  physical_quality = _geometric_mean((
    max(age, 0.10),
    max(direction, 0.10),
    max(motion, 0.10),
    max(uncertainty, 0.15),
  ))
  # A single radar may still reach LEAD with clean motion, but corroborated
  # tracks earn confidence sooner. Position-derived and reported lateral
  # velocities must also agree; reflection hopping commonly breaks this.
  agreement_factor = 0.45 + 0.55 * normal_speed_agreement
  return _clamp01(physical_quality * sensor_support * agreement_factor)


def occupancy_scores(
  evidence: OccupancyEvidence,
) -> tuple[float, float, float, float, float, float | None]:
  """Return occupancy components, intent, confidence, and overlap geometry."""
  inward_rate = _inward_rate(evidence)
  overlap_half_width = (
    EGO_HALF_WIDTH_M
    + TARGET_HALF_WIDTH_M
    + _reflection_margin(evidence)
  )
  clearance = max(0.0, abs(evidence.d_path) - overlap_half_width)
  time_to_overlap = (
    clearance / inward_rate
    if clearance > 0.0 and inward_rate > 0.03
    else 0.0 if clearance <= 0.0 else None
  )

  rate_score = _smoothstep(0.06, 0.52, inward_rate)
  displacement_score = _smoothstep(
    0.10, 0.52, evidence.directional_inward_displacement_m,
  )
  direction_score = _geometric_mean((
    _smoothstep(0.48, 0.92, evidence.directional_consistency),
    _smoothstep(0.42, 0.82, evidence.directional_inward_sample_ratio),
  ))
  intent = _geometric_mean((
    max(rate_score, 0.02),
    max(displacement_score, 0.02),
    max(direction_score, 0.02),
  ))
  confidence = _track_confidence(evidence)

  if evidence.current_path_occupancy:
    future_risk = 1.0
    near_lead = 1.0
  elif time_to_overlap is None:
    future_risk = 0.0
    near_lead = 0.0
  else:
    future_risk = _reverse_smoothstep(
      0.55, RISK_LOOKAHEAD_S, time_to_overlap,
    )
    near_lead = _reverse_smoothstep(
      0.35, LEAD_LOOKAHEAD_S, time_to_overlap,
    )

  proximity = _reverse_smoothstep(0.0, 1.45, clearance)
  support_bonus = min(
    0.18,
    0.10 * float(evidence.cross_sensor_confirmed)
    + 0.08 * float(evidence.vision_supported),
  )
  risk = _clamp01(
    intent * confidence * (0.66 * future_risk + 0.34 * proximity)
    + support_bonus * future_risk * intent
  )
  lead = _clamp01(
    intent * confidence * (0.72 * near_lead + 0.28 * proximity)
    + support_bonus * near_lead * intent
  )
  if evidence.current_path_occupancy:
    lead = max(lead, confidence * max(0.55, direction_score))
  return risk, lead, intent, confidence, clearance, time_to_overlap


def _longitudinal_control_urgency(
  evidence: OccupancyEvidence,
) -> tuple[float | None, float | None, float]:
  time_gap = (
    evidence.d_rel / evidence.v_ego
    if evidence.d_rel > 0.0 and evidence.v_ego > 0.5
    else None
  )
  closing_speed = max(0.0, -evidence.v_rel)
  time_to_collision = (
    evidence.d_rel / closing_speed
    if evidence.d_rel > 0.0 and closing_speed > 0.2
    else None
  )
  headway_urgency = (
    _reverse_smoothstep(0.65, 2.80, time_gap)
    if time_gap is not None else 0.0
  )
  approaching = _reverse_smoothstep(-0.2, 2.0, evidence.v_rel)
  headway_urgency *= 0.20 + 0.80 * approaching
  closing_urgency = (
    _reverse_smoothstep(1.25, 8.00, time_to_collision)
    if time_to_collision is not None else 0.0
  )
  return (
    time_gap,
    time_to_collision,
    _clamp01(0.35 * headway_urgency + 0.65 * closing_urgency),
  )


def _raw_stage(
  evidence: OccupancyEvidence,
  occupancy_score: float,
  risk_score: float,
  lead_score: float,
) -> OccupancyStage:
  if (
    evidence.current_path_occupancy
    and lead_score >= LEAD_EXIT_SCORE
  ):
    return OccupancyStage.OCCUPIED
  if lead_score >= LEAD_ENTER_SCORE:
    return OccupancyStage.LEAD
  if risk_score >= LIMIT_ENTER_SCORE:
    return OccupancyStage.LIMIT
  if occupancy_score >= WATCH_ENTER_SCORE:
    return OccupancyStage.WATCH
  return OccupancyStage.CLEAR


class RadarOccupancyModelV2:
  """Temporal state machine over calibrated continuous occupancy evidence."""

  def __init__(self) -> None:
    self._states: dict[tuple[str, int], _StageState] = {}

  def reset(self) -> None:
    self._states.clear()

  @staticmethod
  def _confirmation_s(stage: OccupancyStage) -> float:
    if stage >= OccupancyStage.LEAD:
      return LEAD_CONFIRMATION_S
    if stage >= OccupancyStage.LIMIT:
      return LIMIT_CONFIRMATION_S
    return 0.0

  def _update_stage(
    self,
    time_s: float,
    evidence: OccupancyEvidence,
    occupancy_score: float,
    risk_score: float,
    lead_score: float,
  ) -> OccupancyStage:
    identity = evidence.identity
    state = self._states.setdefault(identity, _StageState())
    state.last_seen_s = time_s
    raw_stage = _raw_stage(
      evidence, occupancy_score, risk_score, lead_score,
    )

    if raw_stage > state.stage:
      if state.pending_stage <= state.stage:
        state.pending_since_s = time_s
      state.pending_stage = max(state.pending_stage, raw_stage)
      # Continuous strong evidence may satisfy more than one staged timer
      # between sparse replay samples. Real-time frames still expose LIMIT
      # before LEAD; replay must not lose that elapsed evidence.
      while state.stage < raw_stage:
        next_stage = OccupancyStage(int(state.stage) + 1)
        if (
          time_s - state.pending_since_s + 1e-6
          < self._confirmation_s(next_stage)
        ):
          break
        state.stage = next_stage
        state.last_upgrade_s = time_s
    elif raw_stage == state.stage:
      state.pending_stage = raw_stage
      state.pending_since_s = time_s
    else:
      hold_threshold_met = (
        state.stage >= OccupancyStage.LEAD
        and lead_score >= LEAD_EXIT_SCORE
      ) or (
        state.stage == OccupancyStage.LIMIT
        and risk_score >= LIMIT_EXIT_SCORE
      )
      if not hold_threshold_met and time_s - state.last_upgrade_s >= DOWNGRADE_HOLD_S:
        state.stage = raw_stage
        state.pending_stage = raw_stage
        state.pending_since_s = time_s
    return state.stage

  def update(
    self,
    time_s: float,
    evidence_values: Iterable[OccupancyEvidence],
  ) -> tuple[OccupancyEstimate, ...]:
    time_s = float(time_s)
    estimates = []
    seen: set[tuple[str, int]] = set()
    for evidence in evidence_values:
      seen.add(evidence.identity)
      (
        occupancy_risk,
        occupancy_lead,
        intent,
        confidence,
        clearance,
        time_to_overlap,
      ) = occupancy_scores(evidence)
      time_gap, time_to_collision, control_urgency = (
        _longitudinal_control_urgency(evidence)
      )
      risk = occupancy_risk * (0.35 + 0.65 * control_urgency)
      lead = occupancy_lead * (0.45 + 0.55 * control_urgency)
      stage = self._update_stage(
        time_s, evidence, occupancy_risk, risk, lead,
      )
      estimates.append(OccupancyEstimate(
        evidence=evidence,
        stage=stage,
        occupancy_score=occupancy_risk,
        risk_score=risk,
        lead_score=lead,
        intent_score=intent,
        confidence=confidence,
        path_clearance_m=clearance,
        inward_rate_mps=_inward_rate(evidence),
        time_to_overlap_s=time_to_overlap,
        time_gap_s=time_gap,
        time_to_collision_s=time_to_collision,
        control_urgency=control_urgency,
      ))

    for identity, state in tuple(self._states.items()):
      if identity not in seen and time_s - state.last_seen_s > MISSING_HOLD_S:
        self._states.pop(identity)
    return tuple(estimates)


def early_control_eligible(estimate: OccupancyEstimate) -> bool:
  """Bound V2-only control additions to independently supported urgency.

  Existing V1 control remains responsible for front-only, distant, and
  stationary handoffs. This gate defines where V2 may safely act earlier than
  V1 while the new occupancy model is expanded through validation.
  """
  evidence = estimate.evidence
  return (
    evidence.source.startswith("corner")
    and evidence.cross_sensor_confirmed
    and evidence.v_rel < 0.5
    and evidence.d_rel <= 12.0
    and abs(evidence.d_path) <= 3.30
  )
