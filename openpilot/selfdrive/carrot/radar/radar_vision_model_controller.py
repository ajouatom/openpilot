#!/usr/bin/env python3
"""Independent vision-matched primary lead plus model secondary leads."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

from openpilot.selfdrive.carrot.radar.radar_lead_model import RadarLeadPrediction, VisionLeadContext
from openpilot.selfdrive.carrot.radar.radar_lead_runtime import RadarLeadRuntime


RADAR_TO_CAMERA = 1.52
VISION_LEAD_MIN_PROB = 0.5
VISION_LEAD_HOLD_MIN_PROB = 0.35
VISION_LEAD_HOLD_MAX_FRAMES = 10
VISION_MATCH_DISTANCE_HYSTERESIS_M = 2.0
VISION_MATCH_LONG_RANGE_M = 45.0
VISION_MATCH_FRESH_MAX_DPATH_M = 3.0
VISION_MATCH_HELD_MAX_DPATH_M = 4.0
VISION_MATCH_CORNER_HOLD_MAX_DPATH_M = 2.2
VISION_MATCH_CORNER_HOLD_MIN_SCORE_RATIO = 0.15
CORNER_STATIONARY_MIN_VISION_PROB = 0.65
CORNER_STATIONARY_MIN_TRACK_AGE = 8
CORNER_STATIONARY_MAX_DPATH_M = 0.75
CORNER_STATIONARY_MIN_IN_LANE_PROB = 0.65
CORNER_STATIONARY_MAX_VLEAD_MPS = 4.0
STEALTH_LEAD_MIN_TRACK_AGE = 7
STEALTH_LEAD_MAX_DREL_M = 50.0
STEALTH_LEAD_MAX_DPATH_M = 1.0
STEALTH_LEAD_MIN_IN_LANE_PROB = 0.55
STEALTH_LEAD_HOLD_S = 0.5
PRIMARY_STEALTH_HOLD_S = 0.75
PRIMARY_STEALTH_MAX_DREL_M = 60.0
PRIMARY_STEALTH_MAX_DPATH_M = 2.2
PRIMARY_STEALTH_MIN_LEAD_PROB = 0.8
CUTIN_REPORT_MIN_DREL_M = 1.0
LOW_SPEED_CUTIN_MAX_EGO_MPS = 3.0
LOW_SPEED_CUTIN_MAX_DPATH_M = 1.5


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError, IndexError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def cutin_is_ahead_of_primary(cutin_d_rel: float, primary_d_rel: float | None) -> bool:
  return primary_d_rel is None or cutin_d_rel <= primary_d_rel


def _first(values: Any, fallback: float = 0.0) -> float:
  try:
    return _finite(values[0], fallback)
  except (TypeError, IndexError):
    return fallback


def _laplacian(value: float, mean: float, scale: float) -> float:
  scale = max(abs(scale), 0.1)
  return math.exp(-abs(value - mean) / scale) / (2.0 * scale)


@dataclass(frozen=True)
class VisionMatch:
  prediction: RadarLeadPrediction
  probability: float
  score: float


@dataclass(frozen=True)
class RadarLeadModelOutput:
  available: bool
  error: str = ""
  lead_one: dict[str, Any] | None = None
  lead_two: dict[str, Any] | None = None
  lead_cutin: dict[str, Any] | None = None
  lead_external: dict[str, Any] | None = None
  lead_left: dict[str, Any] | None = None
  lead_right: dict[str, Any] | None = None
  leads_left: tuple[dict[str, Any], ...] = ()
  leads_center: tuple[dict[str, Any], ...] = ()
  leads_right: tuple[dict[str, Any], ...] = ()
  leads_cutin: tuple[dict[str, Any], ...] = ()
  leads_left2: tuple[dict[str, Any], ...] = ()
  leads_right2: tuple[dict[str, Any], ...] = ()


class VisionRadarMatcher:
  """Use vision only to identify a front/SCC radar object."""

  def __init__(self) -> None:
    self.last_aliases: frozenset[str] = frozenset()
    self.low_probability_hold_frames = 0

  @staticmethod
  def _vision_lead(model: Any) -> tuple[float, float, float, float, float, float, float] | None:
    leads = getattr(model, "leadsV3", ())
    if not leads:
      return None
    lead = leads[0]
    probability = _finite(getattr(lead, "prob", 0.0))
    if not getattr(lead, "x", ()) or not getattr(lead, "y", ()) or not getattr(lead, "v", ()):
      return None
    d_rel = _first(lead.x) - RADAR_TO_CAMERA
    if d_rel <= 0.5:
      return None
    return (
      probability,
      d_rel,
      -_first(lead.y),
      _first(lead.v),
      _first(getattr(lead, "xStd", ()), 1.0),
      _first(getattr(lead, "yStd", ()), 1.0),
      _first(getattr(lead, "vStd", ()), 1.0),
    )

  @staticmethod
  def _radar_values(prediction: RadarLeadPrediction, v_ego: float) -> tuple[float, float, float]:
    obj = prediction.features.radar_object
    d_rel = obj.front_d_rel if obj.front_d_rel is not None else obj.d_rel
    v_rel = obj.front_v_rel if obj.front_v_rel is not None else obj.v_rel
    return float(d_rel), float(obj.y_rel), float(obj.v_lead if obj.front_v_rel is None else v_ego + v_rel)

  @staticmethod
  def _corner_corroborated(
    prediction: RadarLeadPrediction, predictions: tuple[RadarLeadPrediction, ...],
  ) -> bool:
    """Return whether a front target is also observed by a corner radar."""
    obj = prediction.features.radar_object
    if obj.corner_track_id is not None:
      return True
    if obj.front_track_id is None:
      return False
    return any(
      other is not prediction
      and other.features.radar_object.corner_track_id is not None
      and abs(other.features.radar_object.d_rel - obj.d_rel) < 4.0
      and abs(other.features.radar_object.y_rel - obj.y_rel) < 2.5
      and abs(other.features.radar_object.v_lead - obj.v_lead) < 4.0
      for other in predictions
    )

  @staticmethod
  def _corner_stationary_vehicle_evidence(
    prediction: RadarLeadPrediction,
    predictions: tuple[RadarLeadPrediction, ...],
    vision_lead: VisionLeadContext,
    held_identity: bool = False,
  ) -> bool:
    """Accept slow corner data only when a second radar sees the same object."""
    features = prediction.features
    obj = features.radar_object
    if (
      obj.corner_track_id is None
      or vision_lead.probability < (
        VISION_LEAD_HOLD_MIN_PROB + 0.20 if held_identity else CORNER_STATIONARY_MIN_VISION_PROB
      )
      or features.track_age < CORNER_STATIONARY_MIN_TRACK_AGE
      or not 8.0 < obj.d_rel < 110.0
      or not -1.0 < obj.v_lead < CORNER_STATIONARY_MAX_VLEAD_MPS
      or abs(features.d_path) >= CORNER_STATIONARY_MAX_DPATH_M
      or features.in_lane_prob < CORNER_STATIONARY_MIN_IN_LANE_PROB
      or abs(obj.d_rel - vision_lead.d_rel) >= max(8.0, min(24.0, 1.5 * abs(vision_lead.x_std)))
      or abs(obj.y_rel - vision_lead.y_rel) >= max(2.0, min(3.0, 1.5 * abs(vision_lead.y_std)))
    ):
      return False
    if obj.front_track_id is not None or obj.scc_track_id is not None:
      return True
    return any(
      other is not prediction
      and (other.features.radar_object.front_track_id is not None or other.features.radar_object.scc_track_id is not None)
      and abs(other.features.radar_object.d_rel - obj.d_rel) < 6.0
      and abs(other.features.radar_object.y_rel - obj.y_rel) < 2.4
      and -1.0 < other.features.radar_object.v_lead < 5.0
      for other in predictions
    )

  def match_context(
    self,
    vision_lead: VisionLeadContext | None,
    predictions: Iterable[RadarLeadPrediction],
    v_ego: float,
  ) -> VisionMatch | None:
    if vision_lead is None or vision_lead.d_rel <= 0.5:
      self.last_aliases = frozenset()
      self.low_probability_hold_frames = 0
      return None
    high_probability = vision_lead.probability > VISION_LEAD_MIN_PROB
    holding_previous = (
      not high_probability
      and vision_lead.probability > VISION_LEAD_HOLD_MIN_PROB
      and bool(self.last_aliases)
      and self.low_probability_hold_frames < VISION_LEAD_HOLD_MAX_FRAMES
    )
    if not high_probability and not holding_previous:
      self.last_aliases = frozenset()
      self.low_probability_hold_frames = 0
      return None
    vision = (
      vision_lead.probability,
      vision_lead.d_rel,
      vision_lead.y_rel,
      vision_lead.v,
      vision_lead.x_std,
      vision_lead.y_std,
      vision_lead.v_std,
    )
    probability, vision_d, vision_y, vision_v, x_std, y_std, v_std = vision

    prediction_list = tuple(predictions)
    corner_stationary_ids: set[int] = set()
    candidates: list[tuple[RadarLeadPrediction, float, float, float, float]] = []
    for prediction in prediction_list:
      obj = prediction.features.radar_object
      aliases = frozenset(prediction.features.aliases)
      held_identity = bool(aliases & self.last_aliases)
      corner_stationary = self._corner_stationary_vehicle_evidence(
        prediction, prediction_list, vision_lead, held_identity,
      )
      if obj.front_track_id is None and obj.scc_track_id is None and not corner_stationary:
        continue
      radar_d, radar_y, radar_v = self._radar_values(prediction, v_ego)
      if not 0.5 < radar_d < 180.0:
        continue
      score = (
        _laplacian(radar_d, vision_d, x_std)
        * _laplacian(radar_y, vision_y, y_std)
        * _laplacian(radar_v, vision_v, v_std)
      )
      if (
        radar_d > VISION_MATCH_LONG_RANGE_M
        and abs(prediction.features.d_path) > (
          VISION_MATCH_HELD_MAX_DPATH_M if held_identity else VISION_MATCH_FRESH_MAX_DPATH_M
        )
      ):
        continue
      if holding_previous and not aliases & self.last_aliases:
        continue
      if corner_stationary:
        corner_stationary_ids.add(id(prediction))
      candidates.append((prediction, score, radar_d, radar_y, radar_v))

    if not candidates:
      self.last_aliases = frozenset()
      self.low_probability_hold_frames = 0
      return None

    velocity_tolerance = max(
      5.0,
      abs(vision_v) * (0.3 + 0.2 * min(max((probability - 0.8) / 0.18, 0.0), 1.0)),
    )

    def velocity_sane(candidate: tuple[RadarLeadPrediction, float, float, float, float]) -> bool:
      prediction, _, _, _, radar_v = candidate
      if id(prediction) in corner_stationary_ids:
        return True
      velocity_error = abs(radar_v - vision_v)
      if velocity_error < velocity_tolerance:
        return True
      # Vision velocity is less reliable than distance. Preserve the existing
      # moving-target allowance, but retain a hard physical guard and lane gate.
      return (
        radar_v > 3.0
        and velocity_error < max(velocity_tolerance * 3.0, 20.0)
        and prediction.features.in_lane_prob >= 0.25
      )

    usable = [
      candidate for candidate in candidates
      if abs(candidate[2] - vision_d) < (
        max(5.0, vision_d * 0.25)
        + (VISION_MATCH_DISTANCE_HYSTERESIS_M if frozenset(candidate[0].features.aliases) & self.last_aliases else 0.0)
      )
      and abs(candidate[3] - vision_y) < (
        3.0 if (
          id(candidate[0]) in corner_stationary_ids
          or (
            frozenset(candidate[0].features.aliases) & self.last_aliases
            and abs(candidate[0].features.d_path) < VISION_MATCH_CORNER_HOLD_MAX_DPATH_M
            and self._corner_corroborated(candidate[0], prediction_list)
          )
        ) else 2.0
      )
      and velocity_sane(candidate)
    ]
    if not usable:
      self.last_aliases = frozenset()
      self.low_probability_hold_frames = 0
      return None

    ranked = sorted(candidates, key=lambda candidate: (candidate[1], -abs(candidate[2] - vision_d)), reverse=True)
    selected = max(usable, key=lambda candidate: (
      bool(
        id(candidate[0]) in corner_stationary_ids
        and frozenset(candidate[0].features.aliases) & self.last_aliases
      ),
      candidate[1],
      -abs(candidate[2] - vision_d),
    ))
    # A small vehicle can make the vision distance alternate between two radar
    # returns. Keep the previous identity when front and corner radar still
    # corroborate it and the vision match remains sane.
    held_corner_candidates = [
      candidate for candidate in usable
      if (
        frozenset(candidate[0].features.aliases) & self.last_aliases
        and abs(candidate[0].features.d_path) < VISION_MATCH_CORNER_HOLD_MAX_DPATH_M
        and self._corner_corroborated(candidate[0], prediction_list)
      )
    ]
    if held_corner_candidates:
      held_corner = max(held_corner_candidates, key=lambda candidate: candidate[1])
      if held_corner[1] >= selected[1] * VISION_MATCH_CORNER_HOLD_MIN_SCORE_RATIO:
        selected = held_corner
    selected_is_held_corner = bool(
      frozenset(selected[0].features.aliases) & self.last_aliases
      and self._corner_corroborated(selected[0], prediction_list)
    )
    # Mirror radard's closer second-match rule for small targets whose radar
    # distance can sit just outside the normal vision-distance gate.
    if not selected_is_held_corner and len(ranked) > 1 and ranked[0][0] is selected[0]:
      closer = ranked[1]
      closer_features = closer[0].features
      if (
        closer_features.track_age > 5
        and closer_features.in_lane_prob > 0.3
        and vision_d * 0.5 < closer[2] < selected[2]
        and abs(closer[3] - vision_y) < 2.0
        and velocity_sane(closer)
      ):
        selected = closer
    self.last_aliases = frozenset(selected[0].features.aliases)
    self.low_probability_hold_frames = self.low_probability_hold_frames + 1 if holding_previous else 0
    return VisionMatch(selected[0], probability, selected[1])

  def match(self, model: Any, predictions: Iterable[RadarLeadPrediction], v_ego: float) -> VisionMatch | None:
    values = self._vision_lead(model)
    vision = None if values is None else VisionLeadContext(
      probability=values[0], d_rel=values[1], y_rel=values[2], v=values[3], a=0.0,
      x_std=values[4], y_std=values[5], v_std=values[6],
    )
    return self.match_context(vision, predictions, v_ego)


class VisionModelRadarController:
  """Conventional vision/radar primary selection with model cut-in/external output."""

  def __init__(self) -> None:
    self.runtime = RadarLeadRuntime()
    self.last_runtime_result = None
    self.matcher = VisionRadarMatcher()
    self.stealth_aliases: frozenset[str] = frozenset()
    self.stealth_hold_until = 0.0
    self.primary_aliases: frozenset[str] = frozenset()
    self.primary_hold_until = 0.0
    self.displaced_primary_aliases: frozenset[str] = frozenset()
    self.displaced_primary_hold_until = 0.0

  @staticmethod
  def _lead_from_prediction(
    prediction: RadarLeadPrediction,
    probability: float,
    v_ego: float,
    prefer_front: bool = False,
  ) -> dict[str, Any] | None:
    obj = prediction.features.radar_object
    track_id = next((
      track_id for track_id in (obj.front_track_id, obj.scc_track_id, obj.corner_track_id)
      if track_id is not None
    ), None)
    if track_id is None:
      return None

    front_values = prefer_front and obj.front_track_id is not None
    d_rel = obj.front_d_rel if front_values and obj.front_d_rel is not None else obj.d_rel
    v_rel = obj.front_v_rel if front_values and obj.front_v_rel is not None else obj.v_rel
    v_lead = v_ego + v_rel if front_values else obj.v_lead
    return {
      "dRel": float(d_rel),
      "yRel": float(obj.y_rel),
      "dPath": float(prediction.features.d_path),
      "vRel": float(v_rel),
      "aRel": float(obj.a_rel),
      "vLead": float(v_lead),
      "vLeadK": float(v_lead),
      "aLead": float(obj.a_lead),
      "aLeadK": float(obj.a_lead),
      "aLeadTau": 1.5,
      "jLead": float(obj.j_lead),
      "vLat": float(obj.yv_rel),
      "status": True,
      "fcw": False,
      "modelProb": float(probability),
      "radar": True,
      "radarTrackId": int(track_id),
      "score": float(prediction.risk_prob),
    }

  @staticmethod
  def _external_control_usable(prediction: RadarLeadPrediction) -> bool:
    obj = prediction.features.radar_object
    if obj.front_track_id is not None or obj.scc_track_id is not None:
      # Unmatched stationary front returns are commonly tunnel/gantry ghosts.
      # A stopped object still becomes leadOne when vision matches it; do not
      # independently feed a radar-only stationary ghost to MPC as leadTwo.
      return (
        1.0 < obj.d_rel < 60.0 and obj.v_lead > 2.0
        and prediction.features.track_age >= STEALTH_LEAD_MIN_TRACK_AGE
        and abs(prediction.features.d_path) < 1.5
        and prediction.features.in_lane_prob >= 0.40
        and abs(prediction.features.d_path_future) <= abs(prediction.features.d_path) + 0.35
      )
    if obj.d_rel <= 2.0 or obj.d_rel >= 120.0:
      return False
    if obj.v_lead > 2.0:
      return abs(prediction.features.d_path) < 2.4
    return (
      prediction.features.track_age >= 7
      and abs(obj.v_lead) < 1.8
      and abs(obj.yv_rel) < 0.8
      and abs(prediction.features.d_path) < (1.0 if obj.d_rel < 35.0 else 0.75)
    )

  @staticmethod
  def _stealth_control_usable(prediction: RadarLeadPrediction) -> bool:
    obj = prediction.features.radar_object
    return (
      (obj.front_track_id is not None or obj.scc_track_id is not None)
      and 1.0 < obj.d_rel < STEALTH_LEAD_MAX_DREL_M
      and obj.v_lead > 2.0
      and prediction.features.track_age >= STEALTH_LEAD_MIN_TRACK_AGE
      and abs(prediction.features.d_path) < STEALTH_LEAD_MAX_DPATH_M
      and prediction.features.in_lane_prob >= STEALTH_LEAD_MIN_IN_LANE_PROB
    )

  @staticmethod
  def _primary_hold_usable(prediction: RadarLeadPrediction) -> bool:
    obj = prediction.features.radar_object
    return (
      (obj.front_track_id is not None or obj.scc_track_id is not None)
      and 1.0 < obj.d_rel < PRIMARY_STEALTH_MAX_DREL_M
      and prediction.features.track_age >= STEALTH_LEAD_MIN_TRACK_AGE
      and abs(prediction.features.d_path) < PRIMARY_STEALTH_MAX_DPATH_M
      and abs(prediction.features.d_path_future) <= abs(prediction.features.d_path) + 0.35
      and prediction.lead_prob >= PRIMARY_STEALTH_MIN_LEAD_PROB
    )

  @classmethod
  def _cutin_control_usable(cls, prediction: RadarLeadPrediction, v_ego: float) -> bool:
    obj = prediction.features.radar_object
    return (
      cls._cutin_report_usable(prediction, v_ego)
      and 2.0 < obj.d_rel < 60.0 and obj.v_lead > 2.0
    )

  @staticmethod
  def _cutin_report_usable(prediction: RadarLeadPrediction, v_ego: float) -> bool:
    obj = prediction.features.radar_object
    if obj.d_rel <= CUTIN_REPORT_MIN_DREL_M:
      return False
    return not (
      v_ego < LOW_SPEED_CUTIN_MAX_EGO_MPS
      and abs(prediction.features.d_path) > LOW_SPEED_CUTIN_MAX_DPATH_M
    )

  @staticmethod
  def _pick_side(leads: list[dict[str, Any]]) -> dict[str, Any] | None:
    return min(
      (lead for lead in leads if lead["dRel"] > 5.0 and abs(lead["dPath"]) < 3.5),
      key=lambda lead: lead["dRel"],
      default=None,
    )

  @staticmethod
  def _pick_two(leads: list[dict[str, Any]]) -> tuple[dict[str, Any], ...]:
    usable = sorted((
      lead for lead in leads
      if lead.get("vLead", 0.0) > 2.0 and abs(lead.get("dPath", 0.0)) < 4.2 and lead.get("dRel", 0.0) > 2.0
    ), key=lambda lead: lead["dRel"])
    if not usable:
      return ()
    second = next((lead for lead in usable[1:] if lead["dRel"] - usable[0]["dRel"] >= 5.0), None)
    return (usable[0],) if second is None else (usable[0], second)

  def update(self, time_s: float, v_ego: float, points: Any, model: Any) -> RadarLeadModelOutput:
    result = self.runtime.update(time_s, v_ego, points, model)
    self.last_runtime_result = result
    if not result.available:
      return RadarLeadModelOutput(False, result.error)

    vision_match = self.matcher.match(model, result.predictions, v_ego)
    lead_one = (
      self._lead_from_prediction(vision_match.prediction, vision_match.probability, v_ego, prefer_front=True)
      if vision_match is not None else None
    )

    primary_aliases = frozenset(vision_match.prediction.features.aliases) if vision_match is not None else frozenset()
    if primary_aliases:
      if self.primary_aliases and not primary_aliases & self.primary_aliases:
        self.displaced_primary_aliases = self.primary_aliases
        self.displaced_primary_hold_until = time_s + PRIMARY_STEALTH_HOLD_S
      self.primary_aliases = primary_aliases
      self.primary_hold_until = time_s + PRIMARY_STEALTH_HOLD_S

    def matches_primary(prediction: RadarLeadPrediction) -> bool:
      return bool(primary_aliases & frozenset(prediction.features.aliases))

    cutin_pairs = [
      (prediction, self._lead_from_prediction(prediction, prediction.cutin_prob, v_ego))
      for prediction in result.decision.cutin_candidates
    ]
    primary_d_rel = lead_one.get("dRel") if lead_one is not None else None
    relevant_cutin_pairs = [
      (prediction, lead) for prediction, lead in cutin_pairs
      if (
        lead is not None
        and cutin_is_ahead_of_primary(float(lead["dRel"]), primary_d_rel)
        and self._cutin_report_usable(prediction, v_ego)
      )
    ]
    external_pairs = [
      (prediction, self._lead_from_prediction(prediction, prediction.external_prob, v_ego))
      for prediction in result.decision.external_candidates
      if self._external_control_usable(prediction)
    ]
    stealth_pairs = [
      (prediction, self._lead_from_prediction(prediction, prediction.lead_prob, v_ego, prefer_front=True))
      for prediction in result.decision.lead_candidates
      if not matches_primary(prediction) and self._stealth_control_usable(prediction)
    ]
    stealth_pairs.sort(key=lambda pair: pair[0].features.radar_object.d_rel)
    if stealth_pairs:
      self.stealth_aliases = frozenset(stealth_pairs[0][0].features.aliases)
      self.stealth_hold_until = time_s + STEALTH_LEAD_HOLD_S
    elif self.stealth_aliases and time_s <= self.stealth_hold_until:
      held_prediction = next((
        prediction for prediction in result.predictions
        if self.stealth_aliases & frozenset(prediction.features.aliases)
        and not matches_primary(prediction)
        and self._stealth_control_usable(prediction)
      ), None)
      if held_prediction is not None:
        held_lead = self._lead_from_prediction(held_prediction, held_prediction.lead_prob, v_ego, prefer_front=True)
        if held_lead is not None:
          stealth_pairs = [(held_prediction, held_lead)]
    else:
      self.stealth_aliases = frozenset()
      self.stealth_hold_until = 0.0
    primary_hold_pair = None
    hold_aliases = self.displaced_primary_aliases if primary_aliases else self.primary_aliases
    hold_until = self.displaced_primary_hold_until if primary_aliases else self.primary_hold_until
    if hold_aliases and time_s <= hold_until:
      held_primary = next((
        prediction for prediction in result.predictions
        if hold_aliases & frozenset(prediction.features.aliases)
        and not matches_primary(prediction)
        and self._primary_hold_usable(prediction)
      ), None)
      if held_primary is not None:
        held_lead = self._lead_from_prediction(held_primary, held_primary.lead_prob, v_ego, prefer_front=True)
        if held_lead is not None:
          primary_hold_pair = (held_primary, held_lead)
    # Report independent cut-in decisions even when they are not safe leadTwo
    # control inputs, but do not re-report the vision-matched primary object as
    # a separate cut-in.
    cutin_leads = tuple(lead for prediction, lead in relevant_cutin_pairs if not matches_primary(prediction))
    external_leads = tuple(lead for _, lead in external_pairs if lead is not None)
    lead_two = next((
      lead for prediction, lead in relevant_cutin_pairs
      if not matches_primary(prediction) and self._cutin_control_usable(prediction, v_ego)
    ), None)
    if lead_two is None:
      lead_two = next((
        lead for prediction, lead in external_pairs
        if lead is not None and not matches_primary(prediction)
      ), None)
    if lead_two is None and primary_hold_pair is not None:
      lead_two = primary_hold_pair[1]
    if lead_two is None:
      lead_two = next((lead for _, lead in stealth_pairs if lead is not None), None)

    left: list[dict[str, Any]] = []
    center: list[dict[str, Any]] = []
    right: list[dict[str, Any]] = []
    for prediction in result.predictions:
      lead = self._lead_from_prediction(
        prediction,
        max(prediction.lead_prob, prediction.cutin_prob, prediction.external_prob),
        v_ego,
      )
      if lead is None:
        continue
      if abs(prediction.features.d_path) < 1.8:
        center.append(lead)
      elif prediction.features.radar_object.y_rel > 0.0:
        left.append(lead)
      else:
        right.append(lead)
    left.sort(key=lambda lead: lead["dRel"])
    center.sort(key=lambda lead: lead["dRel"])
    right.sort(key=lambda lead: lead["dRel"])

    return RadarLeadModelOutput(
      available=True,
      lead_one=lead_one,
      lead_two=lead_two,
      lead_cutin=cutin_leads[0] if cutin_leads else None,
      lead_external=external_leads[0] if external_leads else None,
      lead_left=self._pick_side(left),
      lead_right=self._pick_side(right),
      leads_left=tuple(left),
      leads_center=tuple(center),
      leads_right=tuple(right),
      leads_cutin=cutin_leads,
      leads_left2=self._pick_two(left),
      leads_right2=self._pick_two(right),
    )
