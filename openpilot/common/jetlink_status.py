"""Read-only display identity; independent of model/runtime imports."""
import json
from pathlib import Path
import time

LINK_STATUS = Path('/dev/shm/carrot-jetlink.json')
MODEL_STATUS = Path('/dev/shm/carrot-jetlink-model.json')
HOST_LABELS = {'jetson': 'jetSON', 'mac': 'MAC'}


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
    value = json.loads(path.read_text())
    if isinstance(value, dict) and 0 <= now - value['updated'] < 3:
      return value
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return {}


def badge():
  now = time.monotonic()
  link = _fresh(LINK_STATUS, now)
  model = _fresh(MODEL_STATUS, now)
  label = host_label(link.get('peer'))
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
