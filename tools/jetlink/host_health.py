"""Read-only host health. All sysfs/network work runs outside inference."""
import ipaddress
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

STATUS = Path('/dev/shm/carrot-jetlink-health.json')
THERMAL = Path('/sys/class/thermal')


def storage_health(value, status=Path('/run/carrot-storage.json'), marker=Path('/etc/carrot-jetlink-protected.json')):
  if not marker.exists():
    value['storage_mode'] = 'legacy-writable'
    return
  try:
    data = json.loads(status.read_text())
    mode = data['state']
    if data.get('system_read_only') is not True or mode not in ('protected', 'base-recovery'):
      raise ValueError('Unknown protected storage state')
  except (OSError, ValueError, KeyError, TypeError):
    mode = 'unknown'
  value['storage_mode'] = mode
  if mode != 'protected':
    issue = 'Storage recovery: factory runtime, check SD DATA' if mode == 'base-recovery' else 'Storage protection status unavailable'
    value['reason'] = '; '.join(filter(None, [value.get('reason'), issue]))[:240]
    # Existing UI reserves warning/HOT for temperature. Storage recovery is a
    # separate fault, not a thermal warning or proof that inference is invalid.
    value.update(severity='error', storage_error=True)


def thermal_health(root=THERMAL):
  sensors = []
  for zone in sorted(root.glob('thermal_zone*')):
    try:
      name = (zone / 'type').read_text().strip()
      temperature = int((zone / 'temp').read_text()) / 1000
      if not math.isfinite(temperature) or not -20 <= temperature <= 150:
        continue
    except (OSError, ValueError, TypeError):
      continue
    limits = []
    for trip in zone.glob('trip_point_*_type'):
      try:
        kind = trip.read_text().strip()
        limit = int(trip.with_name(trip.name.replace('_type', '_temp')).read_text()) / 1000
        index = trip.name.split('_')[2]
        cooling = []
        for binding in zone.glob('cdev*_trip_point'):
          if binding.read_text().strip() == index:
            cooling.append((binding.with_name(binding.name.replace('_trip_point', '')) / 'type').read_text().strip())
        # JetPack also has a 70C passive hot-surface notification. It is not
        # a processor throttle trip and must not become an internal fault.
        if kind == 'passive' and cooling and all(c == 'hot-surface-alert' for c in cooling):
          continue
        if kind in ('passive', 'hot', 'critical') and 0 < limit <= 150:
          limits.append((kind, limit))
      except (OSError, ValueError, TypeError):
        continue
    sensors.append({'name': name, 'temp_c': temperature, 'limits': limits})
  issues = []
  for sensor in sensors:
    for kind, limit in sensor['limits']:
      temp = sensor['temp_c']
      if temp >= limit - 5:
        severity = 'error' if temp >= limit else 'warning'
        issues.append((severity, f"{sensor['name']} {temp:.1f}C / {kind} {limit:.1f}C"))
  severity = 'error' if any(i[0] == 'error' for i in issues) else 'warning' if issues else 'ok' if sensors else 'unknown'
  reason = '; '.join(i[1] for i in issues)[:240]
  trips = [(limit - sensor['temp_c'], sensor['temp_c'], limit)
           for sensor in sensors for _, limit in sensor['limits']]
  nearest = min(trips) if trips else None
  # Preserve thermal identity when storage/runtime health overrides severity.
  return {'severity': severity, 'reason': reason,
          'thermal_severity': severity, 'thermal_reason': reason,
          'thermal_trip': {'temp_c': nearest[1], 'limit_c': nearest[2]} if nearest else None,
          'temp_c': max((s['temp_c'] for s in sensors), default=None), 'sensors': sensors}


def network_addresses():
  result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'scope', 'global'],
                          capture_output=True, text=True, timeout=1, check=True)
  addresses = []
  for interface in json.loads(result.stdout):
    for info in interface.get('addr_info', []):
      try:
        address = ipaddress.ip_address(info.get('local', ''))
      except ValueError:
        continue
      if not address.is_loopback and not address.is_link_local and not address.is_unspecified:
        addresses.append({'interface': interface['ifname'], 'address': str(address)})
  return addresses


class HostHealth:
  def __init__(self):
    self.error = None
    self.snapshot = {}
    threading.Thread(target=self._run, name='carrot-host-health', daemon=True).start()

  def record_error(self, error, detail=''):
    self.error = (time.monotonic(), f'{error}: {detail}'[:240])

  def read(self):
    value = self.snapshot
    age = time.monotonic() - value.get('updated', float('-inf'))
    # A blocked sensor worker must not make an old temperature look current.
    return {**value, 'age_s': age} if 0 <= age < 3 else {}

  def _run(self):
    while True:
      started = time.monotonic()
      try:
        value = thermal_health()
        storage_health(value)
        try:
          value['addresses'] = network_addresses()
        except (OSError, ValueError, subprocess.SubprocessError):
          value['addresses'] = []
        error = self.error
        if error and 0 <= started - error[0] < 30:
          value.update(severity='error', reason=error[1], runtime_error=True)
        value['updated'] = started
        self.snapshot = value
        temporary = STATUS.with_suffix('.tmp')
        temporary.write_text(json.dumps(value))
        os.replace(temporary, STATUS)
      except Exception:
        pass  # Expiry exposes failure; diagnostics cannot terminate inference.
      time.sleep(1)


_health = None


def health_worker():
  global _health
  if _health is None:
    _health = HostHealth()
  return _health
