"""Device-local smoke-test receipts; artifact integrity is still checked on every boot."""
import hashlib
from importlib.metadata import version
import json
import os
from pathlib import Path
import platform
import sys
import tempfile

from openpilot.common.basedir import BASEDIR


def camera_sizes(device_type: str) -> tuple[tuple[int, int], ...]:
  if device_type == 'mici':
    return ((1344, 760),)
  if device_type in ('tici', 'tizi'):
    return ((1928, 1208),)
  return ((1928, 1208), (1344, 760))


def device_context() -> dict:
  # Missing hardware/OS identity disables reuse rather than sharing a generic receipt.
  return {
    'device': Path('/sys/firmware/devicetree/base/model').read_text().strip('\x00').split('comma ')[-1],
    'machine_id': Path('/etc/machine-id').read_text().strip(),
    'os': Path('/VERSION').read_text().strip(),
    'kernel': platform.release(),
    'machine': platform.machine(),
    'python': sys.version,
    'numpy': version('numpy'),
  }


def validation_key(path: Path, device: dict, sizes: tuple[tuple[int, int], ...]) -> str | None:
  if device.get('device') not in ('mici', 'tici', 'tizi') or not device.get('machine_id') or not device.get('os'):
    return None
  catalog = json.loads((path.parent / 'installed.json').read_text())
  # Hash only runtime/validation inputs, not the repository HEAD or unrelated modeld changes.
  sources = [
    'openpilot/selfdrive/modeld/' + name for name in (
      'precompiled_validation.py', 'precompiled_runner.py', 'precompiled_worker.py', 'precompiled_model.py',
      'generic_model_runtime.py', 'local_gpu_warp.py', 'helpers.py', 'parse_model_outputs.py', 'constants.py',
    )
  ] + ['openpilot/common/file_chunker.py', 'openpilot/system/camerad/cameras/nv12_info.py']
  inputs = {name: hashlib.sha256((Path(BASEDIR) / name).read_bytes()).hexdigest() for name in sources}
  runtime = path.parent / catalog['runtime_directory']
  if not runtime.is_dir():
    raise FileNotFoundError(runtime)
  # Detect edits/corruption of extracted runtime code too. Ignore bytecode and
  # generated warp PKLs, which are outputs of the pinned runtime rather than its source.
  runtime_sources = {p.relative_to(runtime).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                     for p in runtime.rglob('*') if p.is_file() and p.suffix in ('.py', '.cl', '.so', '.h', '.c', '.cpp', '.s')}
  identity = {'schema': 1, 'catalog': catalog, 'device': device, 'camera_sizes': sizes, 'sources': inputs, 'runtime_sources': runtime_sources}
  return hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()


def validation_cached(path: Path, key: str | None) -> bool:
  if key is None or (path.parent / 'rejected').exists():
    return False
  try:
    return json.loads((path.parent / 'boot_validation.json').read_text()) == {'key': key}
  except (OSError, ValueError):
    return False


def save_validation(path: Path, key: str | None) -> None:
  if key is None:
    return
  # Only the caller's successful subprocess may issue a receipt. A crash cannot
  # turn a partially written file into a valid receipt.
  temporary = None
  try:
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='boot-validation-', delete=False) as f:
      temporary = Path(f.name)
      json.dump({'key': key}, f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(temporary, path.parent / 'boot_validation.json')
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)
