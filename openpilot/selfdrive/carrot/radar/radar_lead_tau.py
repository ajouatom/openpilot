#!/usr/bin/env python3
"""Per-object lead acceleration decay used by model radar controllers."""

from __future__ import annotations

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.carrot.radar.radar_lead_model import RadarLeadPrediction


LEAD_ACCEL_TAU = 1.5
MODEL_DT_S = 0.05


class RadarLeadTauTracker:
  def __init__(self, reaction_factor: float = 1.0) -> None:
    self.reaction_factor = max(0.0, min(2.0, float(reaction_factor)))
    self._filters: dict[str, FirstOrderFilter] = {}
    self._values: dict[str, float] = {}
    self._last_seen: dict[str, float] = {}

  def update(self, time_s: float, predictions: tuple[RadarLeadPrediction, ...]) -> None:
    seen: set[str] = set()
    for prediction in predictions:
      object_id = prediction.features.object_id
      if object_id in seen:
        continue
      seen.add(object_id)
      obj = prediction.features.radar_object
      tau_filter = self._filters.setdefault(
        object_id, FirstOrderFilter(LEAD_ACCEL_TAU, 0.45, MODEL_DT_S),
      )
      if abs(obj.a_lead) < 0.5 * self.reaction_factor and abs(obj.j_lead) < 0.5:
        tau_filter.x = LEAD_ACCEL_TAU * self.reaction_factor
      else:
        tau_filter.update(0.0)
      self._values[object_id] = float(tau_filter.x)
      self._last_seen[object_id] = time_s

    for object_id, last_seen in tuple(self._last_seen.items()):
      if object_id not in seen and time_s - last_seen > 0.5:
        self._filters.pop(object_id, None)
        self._values.pop(object_id, None)
        self._last_seen.pop(object_id, None)

  def value(self, prediction: RadarLeadPrediction) -> float:
    return self._values.get(
      prediction.features.object_id, LEAD_ACCEL_TAU * self.reaction_factor,
    )
