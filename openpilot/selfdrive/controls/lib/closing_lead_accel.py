"""No catch-up acceleration toward a slower lead (K7 0x238 radar).

When the lead is far but slower than the car, the MPC still accelerates toward
the set speed because the gap is larger than desired, then brakes once the gap
closes (2026-10-03: 88 -> 99 km/h toward an 87 km/h lead at 80 m, then -1.9 m/s^2).
This ceiling only removes acceleration; it never requests braking.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


CLOSING_LEAD_MIN_CLOSING_MPS = 0.5     # below this closing speed the ceiling is off
CLOSING_LEAD_FULL_CLOSING_MPS = 2.0    # at/above this closing speed no acceleration is allowed
CLOSING_LEAD_MAX_DISTANCE_M = 150.0
CLOSING_LEAD_ACCEL_HORIZON_S = 2.0     # an accelerating lead counts as this much faster
CLOSING_LEAD_MAX_ACCEL_STEP = 0.05     # per plan step, same as the normal ceiling ramp


def get_closing_lead_accel_limit(lead: Any, v_ego: float, maximum_accel: float) -> float | None:
  """Acceleration ceiling (>= 0) while closing on a slower lead, or None."""
  if lead is None or not bool(getattr(lead, "status", False)):
    return None
  d_rel = float(getattr(lead, "dRel", 0.0))
  v_lead = float(getattr(lead, "vLead", 0.0))
  a_lead = float(getattr(lead, "aLeadK", 0.0))
  if not (math.isfinite(d_rel) and math.isfinite(v_lead) and math.isfinite(a_lead)):
    return None
  if not 0.0 < d_rel <= CLOSING_LEAD_MAX_DISTANCE_M:
    return None
  closing = float(v_ego) - (v_lead + max(a_lead, 0.0) * CLOSING_LEAD_ACCEL_HORIZON_S)
  if closing <= CLOSING_LEAD_MIN_CLOSING_MPS:
    return None
  ceiling = float(np.interp(closing, [CLOSING_LEAD_MIN_CLOSING_MPS, CLOSING_LEAD_FULL_CLOSING_MPS],
                            [max(float(maximum_accel), 0.0), 0.0]))
  return ceiling


def apply_closing_lead_accel_limit(maximum_accel: float, a_desired: float, limit: float | None) -> float:
  """Lower the ceiling gradually from the current plan so acceleration fades out smoothly."""
  if limit is None:
    return float(maximum_accel)
  return min(float(maximum_accel), max(limit, float(a_desired) - CLOSING_LEAD_MAX_ACCEL_STEP))
