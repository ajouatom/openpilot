"""Map rendering demand from a fresh, actively running Jetlink nav model."""
import json
import math
from pathlib import Path
import time

STATUS = Path('/dev/shm/carrot-jetlink-model.json')


def jetlink_map_demand(path=STATUS, now=None):
  try:
    value = json.loads(path.read_text())
    updated = value.get('updated')
    now = time.monotonic() if now is None else now
    return (value.get('active') is True and value.get('nav_model') is True and
            type(updated) in (int, float) and math.isfinite(updated) and 0 <= now - updated < 2.5)
  except (OSError, ValueError, TypeError, AttributeError):
    return False
