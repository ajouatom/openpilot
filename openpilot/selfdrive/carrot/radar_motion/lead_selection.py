"""Lead-role selection for the production dPath radar mode."""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.carrot.radar_motion.predictor import (
  POSITION_ONLY_MAX_ABS_VLEAD_MPS,
)


CONTROL_RELEVANCE_HORIZON_S = 2.0
CONTROL_RELEVANCE_MIN_DREL_M = 20.0
CONTROL_RELEVANCE_BUFFER_M = 10.0
CUTIN_MAX_DREL_M = 80.0
PRIMARY_DUPLICATE_MAX_DREL_DELTA_M = 3.5
PRIMARY_DUPLICATE_MAX_YREL_DELTA_M = 1.4


@dataclass(frozen=True)
class DPathLeadSelection:
  cutins: tuple[dict[str, Any], ...]
  lead_two: dict[str, Any] | None


def dpath_control_max_d_rel(v_ego: float) -> float:
  """Limit leadTwo to distance ego can reach over the prediction horizon."""
  return min(
    CUTIN_MAX_DREL_M,
    max(
      CONTROL_RELEVANCE_MIN_DREL_M,
      max(0.0, float(v_ego)) * CONTROL_RELEVANCE_HORIZON_S
      + CONTROL_RELEVANCE_BUFFER_M,
    ),
  )


def lead_duplicates_primary(
  lead: dict[str, Any],
  primary: dict[str, Any] | None,
) -> bool:
  if primary is None or not primary.get("status"):
    return False
  lead_track_id = int(lead.get("radarTrackId", -1))
  primary_track_id = int(primary.get("radarTrackId", -1))
  if (
    lead_track_id >= 0
    and primary_track_id >= 0
    and lead_track_id == primary_track_id
  ):
    return True
  return (
    abs(float(lead.get("dRel", 0.0)) - float(primary.get("dRel", 0.0)))
    < PRIMARY_DUPLICATE_MAX_DREL_DELTA_M
    and abs(
      float(lead.get("yRel", 0.0)) - float(primary.get("yRel", 0.0))
    )
    < PRIMARY_DUPLICATE_MAX_YREL_DELTA_M
  )


def select_dpath_lead_two(
  primary: dict[str, Any] | None,
  candidates: Iterable[dict[str, Any]],
  v_ego: float,
) -> DPathLeadSelection:
  """Choose an independent confirmed CUT-IN after leadOne is known."""
  maximum_d_rel = dpath_control_max_d_rel(v_ego)
  primary_d_rel = math.inf
  if primary is not None and primary.get("status"):
    value = float(primary.get("dRel", math.inf))
    if math.isfinite(value):
      primary_d_rel = value

  cutins = tuple(sorted(
    (
      lead for lead in candidates
      if (
        lead.get("status")
        and lead.get("radar")
        and 0.8 < float(lead.get("dRel", 0.0)) <= maximum_d_rel
        and float(lead.get("dRel", 0.0)) < primary_d_rel
        and float(lead.get("vLead", 0.0))
        >= POSITION_ONLY_MAX_ABS_VLEAD_MPS
        and not lead_duplicates_primary(lead, primary)
      )
    ),
    key=lambda lead: float(lead["dRel"]),
  ))
  return DPathLeadSelection(
    cutins=cutins,
    lead_two=cutins[0] if cutins else None,
  )
