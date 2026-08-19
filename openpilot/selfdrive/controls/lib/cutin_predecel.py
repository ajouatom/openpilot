"""Bounded longitudinal response to a strong early corner-radar CUT-IN risk."""

from __future__ import annotations

import math
from typing import Any


CUTIN_PREDECEL_MIN_SCORE = 0.15
CUTIN_PREDECEL_MIN_TTC_S = 2.5
CUTIN_PREDECEL_MAX_TTC_S = 8.0
CUTIN_PREDECEL_MIN_ACCEL_LIMIT = -0.65
CUTIN_PREDECEL_MAX_ACCEL_LIMIT = -0.25
CUTIN_PREDECEL_MAX_JERK_STEP = 0.15
NORMAL_MAX_ACCEL_STEP = 0.05


def get_cutin_predecel_accel_limit(radar_state: Any) -> float | None:
  """Return a bounded negative acceleration ceiling for strong early risk."""
  lead = getattr(radar_state, "leadCutInRisk", None)
  if (
    lead is None
    or not bool(getattr(lead, "status", False))
    or not bool(getattr(lead, "radar", False))
    or float(getattr(lead, "score", 0.0)) < CUTIN_PREDECEL_MIN_SCORE
  ):
    return None
  d_rel = float(getattr(lead, "dRel", 0.0))
  v_rel = float(getattr(lead, "vRel", 0.0))
  if not (
    math.isfinite(d_rel)
    and math.isfinite(v_rel)
    and d_rel > 0.0
    and v_rel < 0.0
  ):
    return None
  ttc_s = min(
    CUTIN_PREDECEL_MAX_TTC_S,
    max(CUTIN_PREDECEL_MIN_TTC_S, d_rel / max(-v_rel, 0.1)),
  )
  fraction = (
    (ttc_s - CUTIN_PREDECEL_MIN_TTC_S)
    / (CUTIN_PREDECEL_MAX_TTC_S - CUTIN_PREDECEL_MIN_TTC_S)
  )
  return (
    CUTIN_PREDECEL_MIN_ACCEL_LIMIT
    + fraction * (
      CUTIN_PREDECEL_MAX_ACCEL_LIMIT
      - CUTIN_PREDECEL_MIN_ACCEL_LIMIT
    )
  )


def apply_cutin_predecel_accel_limit(
  maximum_accel: float,
  a_desired: float,
  predecel_limit: float | None,
) -> float:
  """Apply the risk ceiling with a bounded per-plan acceleration step."""
  requested = (
    float(maximum_accel)
    if predecel_limit is None
    else min(float(maximum_accel), float(predecel_limit))
  )
  maximum_step = (
    NORMAL_MAX_ACCEL_STEP
    if predecel_limit is None
    else CUTIN_PREDECEL_MAX_JERK_STEP
  )
  return max(requested, float(a_desired) - maximum_step)
