"""Offroad model-library maintenance, separate from the inference loop."""
from contextlib import contextmanager
import os
from pathlib import Path
import shutil
import time

from openpilot.common import big_model_catalog as C


@contextmanager
def delivery_lock():
  # Same lock as launch/background delivery; never remove a file being installed.
  path = Path('/tmp/big_model_update.lock') if os.name != 'nt' else C.storage() / '.delivery.lock'
  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open('a') as f:
    if os.name != 'nt':
      import fcntl
      fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    yield


def protected_ids():
  protected = {C.effective_id(), C.preferences()['selected']}
  active = C.read_json(C.storage() / 'state.json').get('active') or {}
  protected.update(key for key, entry in C.catalog().items() if entry['egpu']['sha256'] == active.get('sha256'))
  return protected


def paths(key):
  item = C.catalog()[key]['egpu']
  stem, ext = os.path.splitext(item['filename'])
  model = C.storage() / f"{stem}-{item['sha256'][:16]}{ext}"
  return [model, model.with_suffix(model.suffix + '.part'), C.storage() / 'precompiled' / item['sha256']]


def unique_bytes(items):
  seen, total = set(), 0
  for item in items:
    for path in item.rglob('*') if item.is_dir() and not item.is_symlink() else [item]:
      if path.is_file() and not path.is_symlink():
        stat = path.stat()
        identity = (stat.st_dev, stat.st_ino)
        if identity not in seen:
          total += stat.st_size
          seen.add(identity)
  return total


@C.synchronized
def _remove_cache(key, offroad=None):
  if key not in C.catalog():
    raise ValueError('unknown model')
  with delivery_lock():
    if offroad is not None and not offroad():
      raise ValueError('offroad state changed')
    if key in protected_ids():
      raise ValueError('running, installed active or selected model is protected')
    root = C.storage().resolve()
    for path in paths(key):
      if not path.resolve().is_relative_to(root) or path.is_symlink():
        raise ValueError('model cache path escapes storage')
    for path in paths(key):
      if path.is_dir():
        shutil.rmtree(path)
      else:
        path.unlink(missing_ok=True)
    _SIZES.pop(key, None)
    queue = C.read_json(C.storage() / 'jetson-delete.json')
    keys = set(queue.get('models', [])) | {key}
    C.atomic_json(C.storage() / 'jetson-delete.json', {'models': sorted(keys)})


def reconcile_retired(params):
  """Apply reviewed catalog retirements only while actually offroad."""
  def offroad():
    return params.get_bool('IsOffroad') and not params.get_bool('IsOnroad')
  if not offroad():
    return []
  removed = []
  for key, entry in C.catalog().items():
    if entry.get('retired') is not True:
      continue
    if not offroad():
      break
    try:
      _remove_cache(key, offroad)
      removed.append(key)
    except (ValueError, OSError):
      # Running/selected/active or delivery busy: retry after transition.
      continue
  return removed


_SIZES = {}


def cached_bytes(key):
  now = time.monotonic()
  stamp, size = _SIZES.get(key, (0, 0))
  if now - stamp > 30:
    size = unique_bytes(paths(key))
    _SIZES[key] = (now, size)
  return size


@C.synchronized
def status():
  prefs, protected = C.preferences(), protected_ids()
  host = C.read_json(C.storage() / 'jetson-library.json')
  host_fresh = host.get('boot') == C.boot_id() and 0 <= time.monotonic() - host.get('updated', 0) < 30
  queue = C.read_json(C.storage() / 'jetson-delete.json').get('models', [])
  rows = []
  for key, entry in C.available().items():
    sha = C.model_spec(key)['sha256']
    rows.append({'id': key, 'name': entry['name'], 'noo': C.noo(key),
                 'protected': key in protected, 'egpu_bytes': cached_bytes(key),
                 'download_bytes': entry['egpu']['size'], 'jetson_sha256': sha,
                 'jetson_bytes': host.get('models', {}).get(sha) if host_fresh else None,
                 'delete_pending': key in queue})
  return {'models': rows, 'effective': C.effective_id(), 'selected': prefs['selected'],
          'reboot_required': C.effective_id() != prefs['selected'], 'internal': 'CD210',
          'effective_name': C.catalog()[C.effective_id()]['name'], 'effective_noo': C.noo(C.effective_id()),
          'jetson_inventory_fresh': host_fresh}
