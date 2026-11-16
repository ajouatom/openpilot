"""Jetson USB CAN link state for the onroad HUD's small yellow "CAN" badge (joongyu01 11-17). Display only.

tools/jetlink/blink_protocol.Sender runs on the comma and learns each 50 ms whether the Jetson reports its USB CAN
adapter available to this comma (token matches, report younger than 0.5 s). It publishes that here every 0.5 s; the
UI reads it on its 5 s parameter refresh and shows the badge while the last report says available and is fresh.
"""
import json
import os
import time
from pathlib import Path

PATH = Path('/dev/shm/carrot-can-link.json')
STALE_S = 7.  # longer than the UI's 5 s refresh, so a live link never blinks off between refreshes

_next_write = 0.


def publish(now, available, path=PATH):
  global _next_write
  if now < _next_write:
    return
  _next_write = now + .5
  try:
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'stamp': now, 'available': bool(available)}), encoding='utf8')
    os.replace(temporary, path)
  except OSError:
    pass  # the badge is optional; never disturb the CAN sender


def connected(now=None, path=PATH):
  try:
    value = json.loads(path.read_text(encoding='utf8'))
  except (OSError, ValueError):
    return False
  now = time.monotonic() if now is None else now
  stamp = value.get('stamp') if isinstance(value, dict) else None
  return (value.get('available') is True and isinstance(stamp, (int, float)) and not isinstance(stamp, bool)
          and 0 <= now - stamp < STALE_S)
