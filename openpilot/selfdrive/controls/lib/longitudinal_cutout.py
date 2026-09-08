"""Bounded future headway relief for a measured lead predicted to cut out."""

import math
from typing import Any

import numpy as np


def cutout_obstacle_relief(lead: Any, v_ego: float, horizons: np.ndarray,
                           t_follow: float, stop_distance: float) -> np.ndarray:
  relief = np.zeros_like(horizons, dtype=float)
  exit_time = float(getattr(lead, "cutOutTime", 0.0))
  confidence = float(getattr(lead, "cutOutConfidence", 0.0))
  if (not lead.status or not lead.radar or lead.radarTrackId < 0
      or not all(math.isfinite(v) for v in (
        exit_time, confidence, v_ego, t_follow, stop_distance,
        lead.dRel, lead.vRel, lead.vLead, lead.aLeadK,
      ))
      or not 0.0 < exit_time <= 2.5 or not 0.0 < confidence <= 1.0
      or v_ego < 5.0 or lead.vLead <= 4.0 or lead.aLeadK < -2.5 or t_follow <= 0.0):
    return relief
  clearance_time = exit_time + 0.30
  remaining_gap = (lead.dRel + min(0.0, lead.vRel) * clearance_time
                   + 0.5 * (min(-0.5, lead.aLeadK) - 0.5) * clearance_time**2)
  if remaining_gap <= max(6.0, stop_distance):
    return relief
  # Keep every pre-clearance obstacle intact. Afterwards relax at most half
  # the configured time gap, capped at 0.5 seconds / 8 metres. This does not
  # delete the obstacle or change the measured range, speed or FCW trajectory.
  credit = min(8.0, min(0.5, 0.5 * t_follow) * v_ego) * confidence
  return credit * np.clip((horizons - clearance_time) / 0.50, 0.0, 1.0)
