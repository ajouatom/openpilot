"""One fresh model-owned lamp-only edge through the existing USB2CAN tap path."""

import math


def vehicle_allowed(cs, direction):
  try:
    return (
      direction in (1, 2)
      and cs.canValid
      and str(cs.gearShifter) == 'drive'
      and math.isfinite(cs.vEgo)
      and cs.vEgo >= 30 / 3.6
      # The fresh model-owned edge already proved recent driver intent. Requiring
      # pressed torque again here discards real crossings after torque release.
      and math.isfinite(cs.steeringTorque)
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
  def __init__(self, journal=None):
    self.last = None
    self.journal = journal if isinstance(journal, dict) else {}

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
      prior_time = self.journal.get('time')
      prior_mask = self.journal.get('mask', self.journal.get('direction'))
      mask = direction
      # Preserve both origins across rapid opposite requests and modeld restarts.
      if (type(prior_time) in (int, float) and math.isfinite(prior_time) and 0 <= stamp-prior_time < 20 and
          type(prior_mask) is int and prior_mask in (1, 2, 3)):
        mask |= prior_mask
      value = {'time': stamp, 'direction': direction}
      if mask != direction:
        value['mask'] = mask
      record(value)
      self.journal = value
    except OSError:
      return []
    return [{'id': 'boundary:' + str(int(stamp * 1e9)), 'time': stamp, 'direction': 'left' if direction == 1 else 'right'}]
