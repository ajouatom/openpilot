#!/usr/bin/env python3
"""On-device fused radar model runtime."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from openpilot.selfdrive.carrot.radar_lead_model import (
  CUTIN_TEMPORAL_THRESHOLD_MAX,
  RadarLeadContext,
  RadarLeadDecision,
  RadarLeadDecisionFilter,
  RadarLeadFeatureBuilder,
  RadarLeadModel,
  RadarLeadPrediction,
  VisionLeadContext,
)
from openpilot.selfdrive.carrot.radar_object_fusion import RadarObjectFusion


DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_lead_multitask.npz"


@dataclass(frozen=True)
class RuntimeRadarPoint:
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
class RadarLeadRuntimeResult:
  available: bool
  decision: RadarLeadDecision
  predictions: tuple[RadarLeadPrediction, ...]
  elapsed_ms: float
  error: str = ""


EMPTY_DECISION = RadarLeadDecision((), ())


def _finite(value: Any, fallback: float = 0.0) -> float:
  try:
    parsed = float(value)
  except (TypeError, ValueError):
    return fallback
  return parsed if math.isfinite(parsed) else fallback


def _source(point: Any) -> str:
  try:
    source = str(point.radarSource)
  except (AttributeError, TypeError):
    source = "frontRadar"
  track_id = int(point.trackId)
  if source == "frontRadar":
    if 200 <= track_id < 220:
      return "corner235"
    if 240 <= track_id < 250:
      return "corner180"
  return source


def runtime_points(points: Iterable[Any]) -> tuple[RuntimeRadarPoint, ...]:
  return tuple(RuntimeRadarPoint(
    track_id=int(point.trackId),
    d_rel=_finite(point.dRel),
    y_rel=_finite(point.yRel),
    v_rel=_finite(point.vRel),
    a_rel=_finite(point.aRel),
    yv_rel=_finite(point.yvRel),
    v_lead=_finite(point.vLead),
    a_lead=_finite(getattr(point, "aLead", 0.0)),
    j_lead=_finite(getattr(point, "jLead", 0.0)),
    measured=bool(point.measured),
    source=_source(point),
  ) for point in points)


def _xy(data: Any) -> tuple[tuple[float, float], ...]:
  return tuple((_finite(data.x[index]), _finite(data.y[index])) for index in range(min(len(data.x), len(data.y))))


def runtime_context(time_s: float, v_ego: float, model: Any) -> RadarLeadContext:
  leads = []
  for lead in model.leadsV3[:2]:
    if not lead.x or not lead.y or not lead.v or not lead.a:
      continue
    leads.append(VisionLeadContext(
      probability=_finite(lead.prob),
      d_rel=_finite(lead.x[0]) - 1.52,
      y_rel=-_finite(lead.y[0]),
      v=_finite(lead.v[0]),
      a=_finite(lead.a[0]),
      x_std=_finite(lead.xStd[0], 1.0) if lead.xStd else 1.0,
      y_std=_finite(lead.yStd[0], 1.0) if lead.yStd else 1.0,
      v_std=_finite(lead.vStd[0], 1.0) if lead.vStd else 1.0,
    ))
  return RadarLeadContext(
    time_s=time_s,
    v_ego=v_ego,
    path=_xy(model.position),
    lane_lines=tuple(_xy(line) for line in model.laneLines),
    lane_probs=tuple(_finite(probability) for probability in model.laneLineProbs),
    model_leads=tuple(leads),
  )


class RadarLeadRuntime:
  def __init__(self, model_path: Path = DEFAULT_MODEL_PATH, include_scc: bool = True) -> None:
    self.model_path = model_path
    self.fusion = RadarObjectFusion(include_scc=include_scc)
    self.features = RadarLeadFeatureBuilder()
    self.model: RadarLeadModel | None = None
    self.decisions: RadarLeadDecisionFilter | None = None
    self.load_error = ""

  def _load(self) -> bool:
    if self.model is not None:
      return True
    try:
      self.model = RadarLeadModel(self.model_path)
      self.decisions = RadarLeadDecisionFilter(
        lead_threshold=max(0.5, float(self.model.thresholds[0])),
        cutin_threshold=max(0.5, min(CUTIN_TEMPORAL_THRESHOLD_MAX, float(self.model.thresholds[1]))),
      )
      self.load_error = ""
      return True
    except Exception as exc:
      self.load_error = f"{type(exc).__name__}: {exc}"
      return False

  def update(self, time_s: float, v_ego: float, points: Iterable[Any], model: Any) -> RadarLeadRuntimeResult:
    started = time.perf_counter()
    if not self._load():
      return RadarLeadRuntimeResult(False, EMPTY_DECISION, (), (time.perf_counter() - started) * 1e3, self.load_error)
    try:
      fused = self.fusion.update(time_s, runtime_points(points))
      features = self.features.update(runtime_context(time_s, v_ego, model), fused)
      predictions = self.model.predict(features)
      assert self.decisions is not None
      decision = self.decisions.update(time_s, predictions)
      return RadarLeadRuntimeResult(True, decision, predictions, (time.perf_counter() - started) * 1e3)
    except Exception as exc:
      return RadarLeadRuntimeResult(
        False, EMPTY_DECISION, (), (time.perf_counter() - started) * 1e3, f"{type(exc).__name__}: {exc}",
      )
