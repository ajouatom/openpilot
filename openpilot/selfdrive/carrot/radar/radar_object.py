"""Shared radar-object value type used by independent source models."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RadarObject:
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
