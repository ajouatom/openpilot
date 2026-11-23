"""Short native TBT lamp permission; the persistent lamp-isolation mask remains."""
import math


def lane_request(journal, now):
  stamp = journal.get('time')
  request = journal.get('tbt_lane_request')
  mask = journal.get('mask', 0)
  if (type(stamp) in (int, float) and math.isfinite(stamp) and 0 <= now-stamp < .2 and
      journal.get('direction') == 2 and type(mask) is int and mask in (2, 3) and
      isinstance(request, str) and 0 < len(request) <= 100):
    return request
  return None
