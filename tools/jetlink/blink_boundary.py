"""One fresh model-owned lamp-only edge through the existing USB2CAN tap path."""

import math


def vehicle_allowed(cs, direction):
  try:
    signed = cs.steeringTorque * (1 if direction == 1 else -1)
    return (
      direction in (1, 2)
      and cs.canValid
      and str(cs.gearShifter) == 'drive'
      and math.isfinite(cs.vEgo)
      and cs.vEgo >= 30 / 3.6
      and cs.steeringPressed
      and math.isfinite(signed)
      and signed > 0
      and not cs.trailerConnected
      and not cs.steerFaultTemporary
      and not cs.steerFaultPermanent
      and not cs.leftBlinker
      and not cs.rightBlinker
      and not cs.blinkerLever
    )
  except (AttributeError, TypeError, ValueError):
    return False


class BoundaryEvents:
  def __init__(self):
    self.last = None

  def read(self, now, direction, stamp, *, allowed, record):
    previous, self.last = self.last, stamp
    if (
      previous is None
      or stamp == previous
      or direction not in (1, 2)
      or not allowed
      or type(stamp) not in (int, float)
      or not math.isfinite(stamp)
      or not 0 <= now - stamp < 0.2
    ):
      return []
    # Before sending, persist origin for a modeld restart. A failed write forbids output.
    try:
      record({'time': stamp, 'direction': direction})
    except OSError:
      return []
    return [{'id': 'boundary:' + str(int(stamp * 1e9)), 'time': stamp, 'direction': 'left' if direction == 1 else 'right'}]
