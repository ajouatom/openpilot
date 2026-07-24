#!/usr/bin/env python3
"""On-device independent front/corner radar model runtime."""

from __future__ import annotations

import math
import time
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from openpilot.selfdrive.carrot.radar.radar_lead_model import (
  CUTIN_TEMPORAL_THRESHOLD_MAX,
  RadarLeadContext,
  RadarLeadDecision,
  RadarLeadDecisionFilter,
  RadarLeadFeatureBuilder,
  RadarLeadModel,
  RadarLeadPrediction,
  VisionLeadContext,
)
from openpilot.selfdrive.carrot.radar.radar_sensor_objects import independent_radar_objects


DEFAULT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_lead_multitask.npz"
DEFAULT_FRONT_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_lead_front.npz"
DEFAULT_CORNER_MODEL_PATH = Path(__file__).resolve().parent / "models" / "radar_lead_corner.npz"


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


EMPTY_DECISION = RadarLeadDecision((), ())


@dataclass(frozen=True)
class RadarLeadRuntimeResult:
  available: bool
  decision: RadarLeadDecision
  predictions: tuple[RadarLeadPrediction, ...]
  elapsed_ms: float
  error: str = ""
  front_predictions: tuple[RadarLeadPrediction, ...] = ()
  corner_predictions: tuple[RadarLeadPrediction, ...] = ()
  front_decision: RadarLeadDecision = EMPTY_DECISION
  corner_decision: RadarLeadDecision = EMPTY_DECISION


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
    if 300 <= track_id < 412:
      return "corner430"
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
  # Cap'n Proto DynamicListReader accepts integer indexes, but not Python slices.
  for index in range(min(1, len(model.leadsV3))):
    lead = model.leadsV3[index]
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
  def __init__(
    self,
    model_path: Path | None = None,
    include_scc: bool = True,
    *,
    front_model_path: Path | None = None,
    corner_model_path: Path | None = None,
    corner_radar_enabled: bool = False,
    enable_radar_tracks: int = 1,
  ) -> None:
    # model_path is retained for replay/backward compatibility. Production uses
    # independent model files as soon as both source-specific artifacts exist.
    legacy_path = Path(model_path) if model_path is not None else None
    default_front = DEFAULT_FRONT_MODEL_PATH if DEFAULT_FRONT_MODEL_PATH.is_file() else DEFAULT_MODEL_PATH
    default_corner = DEFAULT_CORNER_MODEL_PATH if DEFAULT_CORNER_MODEL_PATH.is_file() else DEFAULT_MODEL_PATH
    self.front_model_path = Path(front_model_path) if front_model_path is not None else (legacy_path or default_front)
    self.corner_model_path = Path(corner_model_path) if corner_model_path is not None else (legacy_path or default_corner)
    self.model_path = self.front_model_path
    self.include_scc = include_scc
    self.corner_radar_enabled = corner_radar_enabled
    self.enable_radar_tracks = enable_radar_tracks
    self.front_features = RadarLeadFeatureBuilder()
    self.corner_features = RadarLeadFeatureBuilder()
    self.front_model: RadarLeadModel | None = None
    self.corner_model: RadarLeadModel | None = None
    # Compatibility for existing replay diagnostics. This always names the
    # front model and is not shared for inference.
    self.model: RadarLeadModel | None = None
    self.front_decisions: RadarLeadDecisionFilter | None = None
    self.corner_decisions: RadarLeadDecisionFilter | None = None
    self.load_error = ""

  def _load(self) -> bool:
    if self.front_model is not None and self.corner_model is not None:
      return True
    try:
      self.front_model = RadarLeadModel(self.front_model_path)
      self.corner_model = RadarLeadModel(self.corner_model_path)
      if self.front_model.sensor_mode not in ("front", "fused"):
        raise RuntimeError(f"front model has {self.front_model.sensor_mode} sensor inputs")
      if self.corner_model.sensor_mode not in ("corner", "fused"):
        raise RuntimeError(f"corner model has {self.corner_model.sensor_mode} sensor inputs")
      self.model = self.front_model
      front_thresholds = dict(
        lead_threshold=max(0.5, float(self.front_model.thresholds[0])),
        cutin_threshold=max(0.5, min(CUTIN_TEMPORAL_THRESHOLD_MAX, float(self.front_model.thresholds[1]))),
        external_threshold=max(0.5, float(self.front_model.thresholds[2])),
      )
      corner_thresholds = dict(
        lead_threshold=max(0.5, float(self.corner_model.thresholds[0])),
        cutin_threshold=max(0.5, min(CUTIN_TEMPORAL_THRESHOLD_MAX, float(self.corner_model.thresholds[1]))),
        external_threshold=max(0.5, float(self.corner_model.thresholds[2])),
      )
      self.front_decisions = RadarLeadDecisionFilter(**front_thresholds)
      self.corner_decisions = RadarLeadDecisionFilter(**corner_thresholds)
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
      objects = independent_radar_objects(
        runtime_points(points),
        include_scc=self.include_scc,
        enable_radar_tracks=self.enable_radar_tracks,
      )
      context = runtime_context(time_s, v_ego, model)
      assert self.front_model is not None and self.corner_model is not None
      front_predictions = self.front_model.predict(self.front_features.update(context, objects.front))
      corner_predictions = self.corner_model.predict(self.corner_features.update(context, objects.corner))
      assert self.front_decisions is not None and self.corner_decisions is not None
      front_decision = self.front_decisions.update(time_s, front_predictions)
      corner_decision = self.corner_decisions.update(time_s, corner_predictions)
      # Front inference always runs for primary vision matching and to provide
      # control-quality kinematics when a corner object can be associated with
      # it. Secondary detection is owned by the cleaner corner stream when the
      # vehicle has corner radar; front is the complete fallback otherwise.
      decision = corner_decision if self.corner_radar_enabled else front_decision
      predictions = front_predictions + corner_predictions
      return RadarLeadRuntimeResult(
        True, decision, predictions, (time.perf_counter() - started) * 1e3, "",
        front_predictions, corner_predictions, front_decision, corner_decision,
      )
    except Exception as exc:
      return RadarLeadRuntimeResult(
        False, EMPTY_DECISION, (), (time.perf_counter() - started) * 1e3, f"{type(exc).__name__}: {exc}",
      )
