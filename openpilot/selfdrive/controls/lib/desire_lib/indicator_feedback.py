"""Bounded recognition of our one-touch echo, never a general stalk mask.

Stock neutral frames can interleave the 3/4 transmitted tap frames, so one
request may produce several rising edges. The receiver still independently
aborts on a changed physical stalk; same-direction input during the brief
echo window cannot be uniquely attributed from these aggregate fields alone.
"""
import math


class TapEcho:
  def __init__(self):
    self.ident = None
    self.since = None
    self.finished = False

  def matches(self, now, cs, direction, request_time, value):
    try:
      tap = value['tap']
      if (not isinstance(tap['id'], str) or not tap['id'] or
          tap['direction'] != direction or abs(tap['request']-request_time) > 1e-6 or
          not all(math.isfinite(x) for x in (now, value['stamp'], tap['start'], tap['request'])) or
          not 0 <= now-value['stamp'] < .15 or not 0 <= now-tap['start'] < .6):
        return False
      if tap['id'] != self.ident:
        self.ident, self.since, self.finished = tap['id'], None, False
      if not cs.blinkerLever:
        return False
      counts = (cs.leftBlinkerStalkCount, cs.rightBlinkerStalkCount)
      baseline = tap['counts']
      i = direction-1
      if (direction not in (1, 2) or len(baseline) != 2 or self.finished or cs.blinkerLever != 1 or
          any(type(x) is not int or not 0 <= x < 256 for x in (*counts, *baseline)) or
          not 1 <= (counts[i]-baseline[i]) % 256 <= (3 if direction == 1 else 4) or counts[1-i] != baseline[1-i] or
          (cs.rightBlinker if direction == 1 else cs.leftBlinker)):
        self.finished = True
        return False
      self.since = now if self.since is None else self.since
      return 0 <= now-self.since < .25
    except (AttributeError, KeyError, IndexError, TypeError, ValueError):
      return False
