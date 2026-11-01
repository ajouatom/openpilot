"""Directional boundary exclusions for manual indication only, never steering."""
import math


def adjacent_lane(cs, md, direction, now):
  try:
    if direction not in (1, 2):
      return False
    if direction == 1:
      stamp = cs.laneLineInfoMonoTime / 1e9
      # Only the left side uses OEM marking classification. Unknown/stale
      # classification cannot establish that this is not a centre line.
      return stamp > 0 and 0 <= now - stamp <= .25 and cs.leftLaneLine in (10, 11, 14, 15, 17)
    # The right side uses geometric road-edge clearance only. Neither the OEM
    # marking colour nor a visible outer lane line is required. This is not a
    # free-space or traffic check: a wide shoulder can also pass this exclusion.
    for index in (0, 4, 8):
      boundary, edge = md.laneLines[2].y[index], md.roadEdges[1].y[index]
      if not all(math.isfinite(v) for v in (boundary, edge)):
        return False
      if edge - boundary < 2.5:
        return False
    return True
  except (AttributeError, IndexError, TypeError, ValueError):
    return False
