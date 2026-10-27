"""Reviewed external models and a selection frozen for the entire device boot.

The setting is deliberately separate from downloaded/loaded model status. Saving
a choice must never change a running model, including during worker recovery.
"""
from contextlib import contextmanager
from functools import wraps
import json
import threading
import os
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DIRECTORY = ROOT / 'openpilot/selfdrive/modeld/model_catalog'
DEFAULT = 'n16'


_LOCK = threading.RLock()


@contextmanager
def library_lock():
  with _LOCK:
    storage().mkdir(parents=True, exist_ok=True)
    with (storage() / '.library.lock').open('a') as f:
      if os.name != 'nt':
        import fcntl
        fcntl.flock(f, fcntl.LOCK_EX)
      yield


def synchronized(fn):
  @wraps(fn)
  def call(*args, **kwargs):
    with library_lock():
      return fn(*args, **kwargs)
  return call


def catalog():
  return json.loads((DIRECTORY / 'catalog.json').read_text(encoding='utf-8'))


def available():
  """Curator-owned choices; retired metadata stays for safe cache cleanup."""
  models = {key: entry for key, entry in catalog().items() if entry.get('retired') is not True}
  if DEFAULT not in models:
    raise ValueError('catalog must retain an available default model')
  return models


def storage():
  override = os.getenv('CARROT_BIG_MODEL_DIR')
  return Path(override) if override else Path('/data/media/0/carrot/models') if Path('/TICI').is_file() else Path.home() / '.comma/models'


def read_json(path):
  try:
    value = json.loads(path.read_text(encoding='utf-8'))
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def atomic_json(path, value):
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, name = tempfile.mkstemp(prefix='.selection-', dir=path.parent)
  try:
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
      json.dump(value, f, indent=2)
      f.write('\n')
      f.flush()
      os.fsync(f.fileno())
    os.replace(name, path)
  finally:
    if os.path.exists(name):
      os.unlink(name)


def boot_id():
  try:
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  except OSError:
    return 'desktop'


def preferences():
  value = read_json(storage() / 'selection.json')
  models = available()
  return {'selected': value.get('selected') if isinstance(value.get('selected'), str) and value['selected'] in models else DEFAULT}


def effective_id():
  value = read_json(storage() / 'boot-selection.json')
  # Missing/corrupt boot state always keeps the branch's known default. Only
  # launch, before delivery/modeld/jetlinkd, may accept the pending preference.
  return value['selected'] if value.get('boot') == boot_id() and isinstance(value.get('selected'), str) and value['selected'] in catalog() else DEFAULT


@synchronized
def freeze_boot():
  path = storage() / 'boot-selection.json'
  value = read_json(path)
  if value.get('boot') != boot_id() or not isinstance(value.get('selected'), str) or value['selected'] not in catalog():
    value = {'boot': boot_id(), 'selected': preferences()['selected']}
    atomic_json(path, value)
  return value['selected']


def selected():
  return catalog()[effective_id()]


@synchronized
def select(key):
  if key not in available():
    raise ValueError('model is not available')
  value = preferences()
  value['selected'] = key
  atomic_json(storage() / 'selection.json', value)


def model_spec(key):
  return json.loads((DIRECTORY / catalog()[key]['jetson_spec']).read_text())


def noo(key):
  inputs = model_spec(key)['input_shapes']
  return inputs.get('nav_map') == [1, 1, 256, 256] and inputs.get('navigation') == [1, 47]


if __name__ == '__main__':
  print(freeze_boot())
