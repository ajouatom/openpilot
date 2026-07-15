#!/usr/bin/env python3
"""Standalone model-based radar lead selection for radard."""

from __future__ import annotations

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
      "aLead": 0.0,
      "aLeadK": 0.0,
      "aLeadTau": 1.5,
      "jLead": 0.0,
      "vLat": float(obj.yv_rel),
      "status": True,
      "fcw": probability > 0.9,
      "modelProb": float(probability),
      "radar": True,
      "radarTrackId": int(track_id),
      "score": float(prediction.risk_prob),
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
      return RadarLeadModelOutput(False, result.error)

    converted: dict[str, dict[str, Any]] = {}
    left: list[dict[str, Any]] = []
    center: list[dict[str, Any]] = []
    right: list[dict[str, Any]] = []
    for prediction in result.predictions:
      lead = self._lead_from_prediction(
        prediction, max(prediction.lead_prob, prediction.cutin_prob),
      )
      if lead is None:
        continue
      converted[prediction.features.object_id] = lead
      if abs(prediction.features.d_path) < 1.8:
        center.append(lead)
      elif prediction.features.radar_object.y_rel > 0.0:
        left.append(lead)
      else:
        right.append(lead)

    def selected(prediction: RadarLeadPrediction, probability: float) -> dict[str, Any] | None:
      lead = converted.get(prediction.features.object_id)
      if lead is None:
        return None
      output = lead.copy()
      output["modelProb"] = float(probability)
      return output

    lead_one_prediction = next((
      prediction for prediction in result.decision.lead_candidates
      if prediction.features.object_id in converted
    ), None)
    lead_one = selected(lead_one_prediction, lead_one_prediction.lead_prob) if lead_one_prediction else None
    lead_one_object = lead_one_prediction.features.object_id if lead_one_prediction else None

    cutin_pairs = [
      (prediction, selected(prediction, prediction.cutin_prob))
      for prediction in result.decision.cutin_candidates
      if prediction.features.object_id in converted
    ]
    cutin_leads = tuple(lead for _, lead in cutin_pairs if lead is not None)

    lead_two = next((
      lead for prediction, lead in cutin_pairs
      if lead is not None and prediction.features.object_id != lead_one_object
    ), None)
    if lead_two is None:
      lead_two_prediction = next((
        prediction for prediction in result.decision.lead_candidates
        if prediction.features.object_id != lead_one_object and prediction.features.object_id in converted
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
