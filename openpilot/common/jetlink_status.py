"""Read-only display identity; independent of model/runtime imports."""
import json
import ipaddress
import math
from pathlib import Path
import time

LINK_STATUS = Path('/dev/shm/carrot-jetlink.json')
MODEL_STATUS = Path('/dev/shm/carrot-jetlink-model.json')
HOST_LABELS = {'jetson': 'jetSON', 'mac': 'MAC'}


def _read(path):
  try:
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def host_label(peer):
  if not isinstance(peer, dict):
    return 'Jetlink'
  host = peer.get('carrot_host')
  if isinstance(host, str) and host in HOST_LABELS:
    return HOST_LABELS[host]
  # Compatibility with the already-installed server, before host metadata.
  device = str(peer.get('device', '')).lower()
  if peer.get('backend') == 'trt' and 'orin' in device:
    return 'jetSON'
  if peer.get('backend') == 'ort' and 'coreml' in device:
    return 'MAC'
  return 'Jetlink'


def _fresh(path, now):
  try:
    value = _read(path)
    if isinstance(value, dict) and 0 <= now - value['updated'] < 3:
      return value
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return {}


def diagnostics():
  """Only current telemetry can report health or an address; HELLO is identity."""
  now = time.monotonic()
  saved = _read(LINK_STATUS)
  link = _fresh(LINK_STATUS, now)
  model = _fresh(MODEL_STATUS, now)
  if not saved.get('peer') and saved.get('state') not in ('connecting', 'loading', 'retrying') and not model.get('active'):
    return None
  result = {'label': host_label(saved.get('peer')), 'state': link.get('state', 'disconnected'),
            'severity': 'unknown', 'reason': '', 'addresses': [], 'temp_c': None,
            'active': bool(model.get('active')), 'fresh': False}
  model_error = str(model.get('error') or '')[:240] if not model.get('active') else ''
  if model_error:
    result.update(severity='error', reason=model_error)
  if result['state'] in ('retrying', 'stopped', 'disconnected', 'waiting'):
    result.update(severity='error', reason=str(link.get('error') or 'Host connection unavailable')[:240])
    return result
  telemetry = link.get('telemetry') or {}
  health = telemetry.get('carrot_health') if isinstance(telemetry, dict) else None
  try:
    age = now - float(link['telemetry_updated']) + float(health['age_s'])
    if not 0 <= age < 3:
      return result
  except (KeyError, TypeError, ValueError):
    return result
  result.update(fresh=True, severity=health.get('severity', 'unknown'), reason=str(health.get('reason', ''))[:240])
  if result['severity'] not in ('ok', 'warning', 'error', 'unknown'):
    result['severity'] = 'unknown'
  temperature = health.get('temp_c')
  if isinstance(temperature, (int, float)) and math.isfinite(temperature) and -20 <= temperature <= 150:
    result['temp_c'] = temperature
  entries = health.get('addresses')
  for entry in (entries[:8] if isinstance(entries, list) else []):
    try:
      address = ipaddress.ip_address(entry['address'])
      if not address.is_loopback and not address.is_unspecified and not address.is_link_local:
        result['addresses'].append(str(address))
    except (KeyError, TypeError, ValueError):
      continue
  if model_error:
    result.update(severity='error', reason=model_error)
  return result


def badge():
  now = time.monotonic()
  link = _fresh(LINK_STATUS, now)
  model = _fresh(MODEL_STATUS, now)
  label = host_label(link.get('peer'))
  health = diagnostics()
  if health:
    label = health['label']
    if health['severity'] == 'error':
      return f'{label} ERROR', 'error'
    if health['severity'] == 'warning':
      return f'{label} HOT', 'loading'
    if label == 'jetSON' and link.get('state') == 'ready' and (not health['fresh'] or health['severity'] == 'unknown'):
      return f'{label} DATA?', 'loading'
  if model.get('active'):
    return label, 'active'
  state = link.get('state')
  if state == 'ready':
    return f'{label} READY', 'ready'
  if state in ('connecting', 'loading'):
    return f'{label} WAIT', 'loading'
  if state == 'retrying':
    return f'{label} RETRY', 'error'
  return None
