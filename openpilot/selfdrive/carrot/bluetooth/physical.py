"""Fresh opt-in hold lease for the existing Hyundai 0x1EA indicator fields."""
import json
import math
import time
from pathlib import Path

HOLD_PATH = Path('/dev/shm/carrot-bluetooth/physical-hold.json')


def native_hold_supported(cp):
  from opendbc.car.hyundai.values import HyundaiExtFlags
  return (getattr(cp, 'brand', '') == 'hyundai' and not getattr(cp, 'passive', True) and getattr(cp, 'openpilotLongitudinalControl', False) and
          bool(getattr(cp, 'extFlags', 0) & HyundaiExtFlags.CANFD_INDICATOR_HOLD))


def lease_value(value, now, fingerprint):
  stamp = value.get('stamp')
  if (type(stamp) not in (int, float) or not math.isfinite(stamp) or not 0 <= now-stamp < .35 or
      value.get('fingerprint') != fingerprint or value.get('mode') not in (1, 2)):
    return 1 if value.get('fingerprint') == fingerprint and value.get('override') is True and type(stamp) in (int,float) and 0 <= now-stamp < 1.35 else 0
  if value.get('override') is not True:
    return 0
  return {'left':2, 'right':3}.get(value.get('direction'), 1)


class HoldReader:
  def __init__(self, cp, params, path=HOLD_PATH):
    self.supported = native_hold_supported(cp)
    self.fingerprint = cp.carFingerprint
    self.params, self.path = params, path
    self.next_mode = self.next_read = 0.
    self.mode, self.value = 0, {}

  def read(self, can_valid, now=None):
    if not self.supported:
      return 0
    now = time.monotonic() if now is None else now
    if now >= self.next_mode:
      self.mode = self.params.get_int('AutoBlinkerControl')
      self.next_mode = now+.2
    if self.mode not in (1, 2):
      return 0  # Preserve the existing lane-change hold when the feature is off.
    if not can_valid:
      return 1
    if now >= self.next_read:
      try:
        with self.path.open() as stream:
          value = json.loads(stream.read(2049))
        self.value = value if isinstance(value, dict) else {}
      except (OSError, ValueError):
        self.value = {}
      self.next_read = now+.05
    return lease_value(self.value, now, self.fingerprint)  # Recheck age even between reads.
