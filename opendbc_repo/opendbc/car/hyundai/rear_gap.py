"""Rear observations for the optional TBT indicator trigger, not a TTC estimate."""

import math

REAR_GAP_MIN_M = 20.0
# Both display messages plateau at raw 249; 250 also occurs as a special value.
# Neither is evidence of a measured gap or a zero relative speed.
REAR_GAP_SATURATION_M = 24.9
REAR_INPUT_MAX_AGE_NS = 150_000_000
REAR_SOURCES = {
  "ADRV_0x1ea": (0x1EA, frozenset((1, 2, 3))),
  "CCNC_0x162": (0x162, frozenset(range(1, 15))),
}


def _fresh(parser, name, address, size, now):
  if parser is None:
    return False
  stamp = max(parser.ts_nanos.get(name, {}).values(), default=0)
  return (not parser.bus_timeout and stamp > 0 and
          0 <= now - stamp <= REAR_INPUT_MAX_AGE_NS and
          len(parser.dat.get(address, b"")) == size)


def rear_distance_blocks(values, side, detected_states):
  """A fresh 'none' adds no obstacle; unknown/hidden is never an all-clear."""
  try:
    state = float(values[f"{side}_DETECT"])
    distance = float(values[f"{side}_DETECT_DISTANCE"])
  except (KeyError, TypeError, ValueError):
    return True
  if not math.isfinite(state) or not math.isfinite(distance):
    return True
  if state == 0 and distance == 0:
    return False
  return not (state in detected_states and REAR_GAP_MIN_M <= distance < REAR_GAP_SATURATION_M)


def rear_lane_change_blocks(camera, bsm, sources):
  """Combine received OEM BSD and independently validated rear display slots.

  `sources` contains only camera messages discovered on this vehicle. Once a
  source is discovered, losing it blocks admission instead of granting a gap.
  A false result only removes this veto; ordinary lane/front/driver gates remain.
  """
  if not sources:
    return False, False
  now = camera._last_update_nanos
  if bsm is not None:
    now = max(now, bsm._last_update_nanos)
  name = "BLINDSPOTS_REAR_CORNERS"
  if not _fresh(bsm, name, 0x1BA, 24, now):
    return True, True
  values = bsm.vl[name]
  blocked = [any(values.get(key, None) != 0 for key in keys) for keys in (
    ("FL_INDICATOR", "INDICATOR_LEFT_TWO", "INDICATOR_LEFT_FOUR"),
    ("FR_INDICATOR", "INDICATOR_RIGHT_TWO", "INDICATOR_RIGHT_FOUR"),
  )]
  for name in sources:
    address, states = REAR_SOURCES[name]
    if not _fresh(camera, name, address, 32, now):
      return True, True
    for index, side in enumerate(("LR", "RR")):
      blocked[index] |= rear_distance_blocks(camera.vl[name], side, states)
  return tuple(blocked)
