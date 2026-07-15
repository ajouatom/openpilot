#!/usr/bin/env python3
"""Standalone model-based radar lead selection for radard."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_lead_model import RadarLeadPrediction
from openpilot.selfdrive.carrot.radar_lead_runtime import RadarLeadRuntime


def _cloudlog(level: str, message: str) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    getattr(cloudlog, level)(message)
  except (ImportError, ModuleNotFoundError):
    pass


RADAR_TO_CAMERA = 1.52


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError, IndexError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def _first_finite(values: Any, fallback: float = 0.0) -> float:
  try:
    return _finite(values[0], fallback)
  except (TypeError, IndexError):
    return fallback


@dataclass(frozen=True)
class RadarLeadModelOutput:
  available: bool
  error: str = ""
  lead_one: dict[str, Any] | None = None
  lead_two: dict[str, Any] | None = None
  lead_cutin: dict[str, Any] | None = None
  lead_left: dict[str, Any] | None = None
  lead_right: dict[str, Any] | None = None
  leads_left: tuple[dict[str, Any], ...] = ()
  leads_center: tuple[dict[str, Any], ...] = ()
  leads_right: tuple[dict[str, Any], ...] = ()
  leads_cutin: tuple[dict[str, Any], ...] = ()
  leads_left2: tuple[dict[str, Any], ...] = ()
  leads_right2: tuple[dict[str, Any], ...] = ()


class RadarLeadModelController:
  """Owns model inference, temporal decisions, and radarState selection."""

  def __init__(self) -> None:
    self.runtime = RadarLeadRuntime()
    self.frames = 0
    self.time_ms = 0.0
    self.last_error = ""

  @staticmethod
  def _lead_from_prediction(
    prediction: RadarLeadPrediction,
    probability: float,
  ) -> dict[str, Any] | None:
    obj = prediction.features.radar_object
    track_id = next((
      track_id for track_id in (obj.front_track_id, obj.corner_track_id, obj.scc_track_id)
      if track_id is not None
    ), None)
    if track_id is None:
      return None
    return {
      "dRel": float(obj.d_rel),
      "yRel": float(obj.y_rel),
      "dPath": float(prediction.features.d_path),
      "vRel": float(obj.v_rel),
      "aRel": float(obj.a_rel),
      "vLead": float(obj.v_lead),
      "vLeadK": float(obj.v_lead),
      "aLead": float(obj.a_lead),
      "aLeadK": float(obj.a_lead),
      "aLeadTau": 1.5,
      "jLead": float(obj.j_lead),
      "vLat": float(obj.yv_rel),
      "status": True,
      "fcw": probability > 0.9,
      "modelProb": float(probability),
      "radar": True,
      "radarTrackId": int(track_id),
      "score": float(prediction.risk_prob),
    }

  @staticmethod
  def _lead_from_vision(model: Any, v_ego: float) -> dict[str, Any] | None:
    leads = getattr(model, "leadsV3", ())
    if not leads:
      return None
    lead = leads[0]
    prob = _finite(getattr(lead, "prob", 0.0))
    if prob < 0.5 or not getattr(lead, "x", ()) or not getattr(lead, "y", ()) or not getattr(lead, "v", ()):
      return None
    d_rel = _finite(lead.x[0]) - RADAR_TO_CAMERA
    if d_rel <= 0.5:
      return None
    y_rel = -_finite(lead.y[0])
    model_v_ego = _first_finite(getattr(getattr(model, "velocity", None), "x", ()), v_ego)
    v_rel = _finite(lead.v[0]) - model_v_ego
    a_lead = _finite(lead.a[0]) if getattr(lead, "a", ()) else 0.0
    path_y = 0.0
    position = getattr(model, "position", None)
    if position is not None and getattr(position, "x", ()) and getattr(position, "y", ()):
      xs = tuple(_finite(value) for value in position.x)
      ys = tuple(_finite(value) for value in position.y)
      for index in range(1, min(len(xs), len(ys))):
        if d_rel <= xs[index]:
          x0, x1 = xs[index - 1], xs[index]
          y0, y1 = ys[index - 1], ys[index]
          ratio = 0.0 if x1 == x0 else (d_rel - x0) / (x1 - x0)
          path_y = y0 + (y1 - y0) * ratio
          break
    return {
      "dRel": float(d_rel),
      "yRel": float(y_rel),
      "dPath": float(y_rel + path_y),
      "vRel": float(v_rel),
      "aRel": float(a_lead),
      "vLead": float(v_ego + v_rel),
      "vLeadK": float(v_ego + v_rel),
      "aLead": float(a_lead),
      "aLeadK": float(a_lead),
      "aLeadTau": 0.3,
      "jLead": 0.0,
      "vLat": 0.0,
      "status": True,
      "fcw": False,
      "modelProb": float(prob),
      "radar": False,
      "radarTrackId": -1,
      "score": float(prob),
    }

  @staticmethod
  def _pick_side(leads: list[dict[str, Any]]) -> dict[str, Any] | None:
    return min(
      (lead for lead in leads if lead["dRel"] > 5.0 and abs(lead["dPath"]) < 3.5),
      key=lambda lead: lead["dRel"],
      default=None,
    )

  @staticmethod
  def _pick_two(leads: list[dict[str, Any]], min_gap: float = 5.0) -> tuple[dict[str, Any], ...]:
    usable = sorted((
      lead for lead in leads
      if lead.get("vLead", 0.0) > 2.0 and abs(lead.get("dPath", 0.0)) < 4.2 and lead.get("dRel", 0.0) > 2.0
    ), key=lambda lead: lead["dRel"])
    if not usable:
      return ()
    second = next((lead for lead in usable[1:] if lead["dRel"] - usable[0]["dRel"] >= min_gap), None)
    return (usable[0],) if second is None else (usable[0], second)

  @staticmethod
  def _external_control_usable(prediction: RadarLeadPrediction) -> bool:
    obj = prediction.features.radar_object
    return obj.front_track_id is not None or (obj.d_rel > 2.0 and obj.v_lead > 2.0)

  @staticmethod
  def _lead_one_fallback_prediction(predictions: tuple[RadarLeadPrediction, ...]) -> RadarLeadPrediction | None:
    candidates = [
      prediction for prediction in predictions
      if (
        prediction.features.radar_object.front_track_id is not None
        or prediction.features.radar_object.scc_track_id is not None
      )
      and 0.5 < prediction.features.radar_object.d_rel < 160.0
      and abs(prediction.features.d_path) < 2.4
      and prediction.lead_prob >= 0.15
    ]
    return min(candidates, key=lambda prediction: (-prediction.lead_prob, prediction.features.radar_object.d_rel), default=None)

  def update(
    self,
    time_s: float,
    v_ego: float,
    points: Any,
    model: Any,
  ) -> RadarLeadModelOutput:
    result = self.runtime.update(time_s, v_ego, points, model)
    self.frames += 1
    self.time_ms += result.elapsed_ms
    if result.error and result.error != self.last_error:
      _cloudlog("error", f"radar lead model error: {result.error}")
    self.last_error = result.error
    if self.frames % 100 == 0:
      _cloudlog("info",
        f"radar lead model available={result.available} avg={self.time_ms / 100.0:.3f}ms "
        f"points={len(result.predictions)} lead={len(result.decision.lead_candidates)} "
        f"cutin={len(result.decision.cutin_candidates)}"
      )
      self.time_ms = 0.0
    if not result.available:
      return RadarLeadModelOutput(False, result.error, lead_one=self._lead_from_vision(model, v_ego))

    left: list[dict[str, Any]] = []
    center: list[dict[str, Any]] = []
    right: list[dict[str, Any]] = []
    for prediction in result.predictions:
      lead = self._lead_from_prediction(
        prediction, max(prediction.lead_prob, prediction.cutin_prob),
      )
      if lead is None:
        continue
      if abs(prediction.features.d_path) < 1.8:
        center.append(lead)
      elif prediction.features.radar_object.y_rel > 0.0:
        left.append(lead)
      else:
        right.append(lead)

    def selected(prediction: RadarLeadPrediction, probability: float) -> dict[str, Any] | None:
      lead = self._lead_from_prediction(prediction, probability)
      if lead is None:
        return None
      return lead

    lead_one_prediction = next((
      prediction for prediction in result.decision.lead_candidates
      if (
        prediction.features.radar_object.front_track_id is not None
        or prediction.features.radar_object.scc_track_id is not None
      )
    ), None)
    if lead_one_prediction is None:
      lead_one_prediction = self._lead_one_fallback_prediction(result.predictions)
    lead_one = selected(lead_one_prediction, lead_one_prediction.lead_prob) if lead_one_prediction else None
    if lead_one is None:
      lead_one = self._lead_from_vision(model, v_ego)
    lead_one_object = lead_one_prediction.features.object_id if lead_one_prediction else None

    cutin_pairs = [
      (prediction, selected(prediction, prediction.cutin_prob))
      for prediction in result.decision.cutin_candidates
    ]
    cutin_leads = tuple(lead for _, lead in cutin_pairs if lead is not None)

    lead_two = next((
      lead for prediction, lead in cutin_pairs
      if lead is not None and prediction.features.object_id != lead_one_object
      and self._external_control_usable(prediction)
    ), None)
    if lead_two is None:
      lead_two_prediction = next((
        prediction for prediction in result.decision.lead_candidates
        if prediction.features.object_id != lead_one_object
        and self._external_control_usable(prediction)
      ), None)
      if lead_two_prediction is not None:
        lead_two = selected(lead_two_prediction, lead_two_prediction.lead_prob)

    left.sort(key=lambda lead: lead["dRel"])
    center.sort(key=lambda lead: lead["dRel"])
    right.sort(key=lambda lead: lead["dRel"])
    return RadarLeadModelOutput(
      available=True,
      lead_one=lead_one,
      lead_two=lead_two,
      lead_cutin=cutin_leads[0] if cutin_leads else None,
      lead_left=self._pick_side(left),
      lead_right=self._pick_side(right),
      leads_left=tuple(left),
      leads_center=tuple(center),
      leads_right=tuple(right),
      leads_cutin=cutin_leads,
      leads_left2=self._pick_two(left),
      leads_right2=self._pick_two(right),
    )
