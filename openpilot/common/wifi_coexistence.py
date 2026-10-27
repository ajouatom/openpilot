"""Secret-free hint for keeping optional background discovery off the HUD radio."""
import json
from pathlib import Path
import time

STATUS = Path('/dev/shm/carrot-jetlink-direct-wifi.json')


def direct_ap_active():
  try:
    value = json.loads(STATUS.read_text())
    return (value.get('state') == 'active' and value.get('interface') == 'wljetson0'
            and 0 <= time.monotonic() - value['updated'] < 8.)
  except (OSError, ValueError, KeyError, TypeError):
    return False
