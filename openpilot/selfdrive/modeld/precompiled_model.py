"""Verified, optional precompiled eGPU artifacts, independent of the local compiler."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import tarfile
import tempfile
import time
from urllib.parse import urljoin, urlparse
from urllib.request import Request, urlopen

from openpilot.selfdrive.modeld.big_model import active_manifest, model_cache_dir

PROTOCOL = 1
MAX_CATALOG = 64 * 1024
SHA256 = re.compile(r'^[0-9a-f]{64}$')


def sha256(path: Path) -> str:
  with path.open('rb') as f:
    return hashlib.file_digest(f, 'sha256').hexdigest()


def validate_catalog(value: dict, model_sha: str, catalog_url: str) -> dict:
  if value.get('protocol') != PROTOCOL or value.get('onnx_sha256') != model_sha:
    raise ValueError('precompiled model does not match the selected ONNX/protocol')
  if value.get('format') != 'comma-run-model' or value.get('gpu_arch') != 'gfx1200':
    raise ValueError('unsupported precompiled model format/GPU')
  if value.get('frame_skip') != 4 or value.get('camera_resolutions') != [[1928, 1208], [1344, 760]]:
    raise ValueError('incompatible precompiled model inputs')
  for name, limit in [('pickle', 4 * 1024**3), ('runtime', 128 * 1024**2)]:
    artifact = value[name]
    if not SHA256.fullmatch(artifact.get('sha256', '')):
      raise ValueError('invalid artifact hash')
    if type(artifact.get('size')) is not int or not 0 < artifact['size'] <= limit:
      raise ValueError('invalid artifact size')
    artifact['url'] = urljoin(catalog_url, artifact['url'])
    if urlparse(artifact['url']).scheme != 'https' or urlparse(artifact['url']).netloc != urlparse(catalog_url).netloc:
      raise ValueError('artifact must use the model server HTTPS origin')
  return value


def download(artifact: dict, target: Path, progress=None) -> None:
  if target.is_file() and target.stat().st_size == artifact['size'] and sha256(target) == artifact['sha256']:
    return
  partial = target.with_suffix(target.suffix + '.part')
  offset = partial.stat().st_size if partial.exists() else 0
  if offset > artifact['size']:
    partial.unlink()
    offset = 0
  if offset == artifact['size']:
    if sha256(partial) == artifact['sha256']:
      os.replace(partial, target)
      return
    partial.unlink()
    offset = 0
  if shutil.disk_usage(target.parent).free < artifact['size'] - offset + 256 * 1024**2:
    raise OSError('insufficient storage for precompiled model')
  headers = {'Accept-Encoding': 'identity', 'User-Agent': 'carrot-precompiled/1'}
  if offset:
    headers['Range'] = f'bytes={offset}-'
  with urlopen(Request(artifact['url'], headers=headers), timeout=30) as response:
    append = offset > 0 and response.status == 206
    if append and not response.headers.get('Content-Range', '').startswith(f'bytes {offset}-'):
      raise OSError('incorrect artifact resume response')
    if not append:
      if response.status != 200:
        raise OSError(f'unexpected download status {response.status}')
      offset = 0
    with partial.open('ab' if append else 'wb') as f:
      while data := response.read(1024 * 1024):
        offset += len(data)
        if offset > artifact['size']:
          raise ValueError('artifact exceeds declared size')
        f.write(data)
        if progress:
          progress(offset, artifact['size'])
      f.flush()
      os.fsync(f.fileno())
  if offset != artifact['size']:
    raise OSError('incomplete precompiled artifact')
  if sha256(partial) != artifact['sha256']:
    partial.unlink()
    raise ValueError('precompiled artifact hash mismatch')
  os.replace(partial, target)


def installed(model=None, cache_dir: Path | None = None) -> Path | None:
  model = model or active_manifest()
  if model is None:
    return None
  root = (cache_dir or model_cache_dir()) / 'precompiled' / model.sha256
  try:
    value = json.loads((root / 'installed.json').read_text())
    validate_catalog(value, model.sha256, value['catalog_url'])
    if (root / 'rejected').exists():
      return None
    if (root / 'model.pkl').stat().st_size != value['pickle']['size']:
      return None
    runtime = root / ('runtime-' + value['runtime']['sha256'][:16])
    if value.get('runtime_directory') != runtime.name:
      return None
    if not (runtime / 'model_runtime.py').is_file() or not (runtime / 'tinygrad' / '__init__.py').is_file():
      return None
    return root / 'model.pkl'
  except (OSError, ValueError, KeyError, TypeError):
    return None


def ensure_precompiled(model=None, cache_dir: Path | None = None, progress=None) -> Path | None:
  model = model or active_manifest()
  if model is None:
    return None
  root = (cache_dir or model_cache_dir()) / 'precompiled' / model.sha256
  existing = installed(model, cache_dir)
  if existing:
    value = json.loads((root / 'installed.json').read_text())
    if sha256(existing) == value['pickle']['sha256'] and sha256(root / 'runtime.tar.gz') == value['runtime']['sha256']:
      return existing
  catalog_url = urljoin(model.url, 'precompiled.json')
  with urlopen(Request(catalog_url, headers={'User-Agent': 'carrot-precompiled/1'}), timeout=8) as response:
    data = response.read(MAX_CATALOG + 1)
  if len(data) > MAX_CATALOG:
    raise ValueError('precompiled catalog too large')
  value = validate_catalog(json.loads(data), model.sha256, catalog_url)
  # A runtime rejected on this device must use the local compiler until the artifact changes.
  if (root / 'rejected').exists() and (root / 'rejected').read_text() == value['pickle']['sha256']:
    return None
  root.mkdir(parents=True, exist_ok=True)
  download(value['pickle'], root / 'model.pkl', progress)
  download(value['runtime'], root / 'runtime.tar.gz')
  runtime = root / ('runtime-' + value['runtime']['sha256'][:16])
  if not runtime.exists():
    with tempfile.TemporaryDirectory(dir=root) as staging:
      with tarfile.open(root / 'runtime.tar.gz') as archive:
        members = archive.getmembers()
        if sum(m.size for m in members) > 256 * 1024**2:
          raise ValueError('runtime archive too large')
        for member in members:
          if not member.isfile() and not member.isdir():
            raise ValueError('runtime archive contains links/special files')
        archive.extractall(staging, filter='data')
      os.replace(staging, runtime)
  # The worker uses this verified immutable directory, never the repository's tinygrad.
  value['runtime_directory'] = runtime.name
  value['catalog_url'] = catalog_url
  marker = root / 'installed.json.tmp'
  marker.write_text(json.dumps(value, indent=2))
  os.replace(marker, root / 'installed.json')
  (root / 'rejected').unlink(missing_ok=True)
  return root / 'model.pkl'


def reject(path: Path) -> None:
  value = json.loads((path.parent / 'installed.json').read_text())
  (path.parent / 'rejected').write_text(value['pickle']['sha256'])


def record_failure(path: Path, error: BaseException | str, phase: str) -> bool:
  """Keep transient device failures retryable; persist why an artifact was rejected."""
  from openpilot.selfdrive.modeld.helpers import usbgpu_pcie_not_ready
  detail = str(error)
  transient = (usbgpu_pcie_not_ready(error) or isinstance(error, (TimeoutError, BrokenPipeError)) or
               'precompiled eGPU worker timed out' in detail or 'precompiled eGPU worker exited' in detail)
  value = json.loads((path.parent / 'installed.json').read_text())
  failure = {'time': time.time(), 'phase': phase, 'rejected': not transient,  # noqa: TID251 - correlate persisted failures with boot logs
             'pickle_sha256': value['pickle']['sha256'], 'error': detail[-16384:]}
  target = path.parent / 'last_failure.json'
  temporary = target.with_suffix('.json.tmp')
  temporary.write_text(json.dumps(failure, indent=2))
  os.replace(temporary, target)
  if not transient:
    reject(path)
  return not transient
