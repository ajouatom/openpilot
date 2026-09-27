"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Which large models exist, which one is which file, and how to get the bytes.

One stdlib-only package every platform can run: a Jetson prefetching a model
over its own network, a Mac app driving it through the control channel, and
`jetlink-models` on a laptop all go through here. It knows the cache layout
`jetlink.server.cache` defines and nothing about backends, sessions or the
wire, so importing it never pulls in an inference runtime.

State lives in `<cache>/registry/`:

    catalog.json       {'fetched_at', 'url', 'raw'} as fetched, refreshed hourly
    pointers.json      {ref: {'oid', 'size'}}; a commit's tree never changes, so this is kept forever
    local-models.json  [{'sha256', 'bytes', 'name', 'added_at'}] for models imported from disk

Each is written atomically, because a control server, a CLI and a download
thread can all be writing while a comma is asking for a model.
"""
from __future__ import annotations

import hashlib
import json
import logging
import os
import re
import shutil
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

from jetlink.registry.catalog import (CATALOG_URL, DEFAULT_BIG_MODEL_REF, REQUIRED_SELECTOR_VERSION, CatalogModel, NetworkError, RegistryError,
                                      VerifyError, fetch_catalog, is_ref, is_sha256, parse_catalog)
from jetlink.registry.lfs import (LFS_ENDPOINTS, POINTER_URL, Pointer, ProgressFn, StopFn, fetch_pointer, lfs_download, lfs_resolve,
                                  parse_pointer_text)
from jetlink.server.cache import LAST_LOADED, EngineCache

log = logging.getLogger('jetlink.registry')

__all__ = ['CATALOG_URL', 'DEFAULT_BIG_MODEL_REF', 'LFS_ENDPOINTS', 'POINTER_URL', 'REQUIRED_SELECTOR_VERSION', 'CatalogModel', 'LocalModel',
           'NetworkError', 'Pointer', 'ProgressFn', 'Registry', 'RegistryError', 'StopFn', 'VerifyError', 'fetch_catalog', 'fetch_pointer',
           'is_ref', 'is_sha256', 'lfs_download', 'lfs_resolve', 'parse_catalog', 'parse_pointer_text']

CATALOG_MAX_AGE = 3600.0
_SHA16 = re.compile(r'[0-9a-f]{16}')
HASH_CHUNK = 1 << 20
COPY_CHUNK = 4 << 20

# The suffix each backend gives its artifact, so a sidecar can be paired with
# the thing it describes without importing the backend (and so a Jetson's
# sidecars are readable on a Mac).
ARTIFACT_SUFFIXES = {'trt': '.plan', 'tinygrad': '.pkl', 'ort': '.ortcache', 'fake': '.fake'}


@dataclass(frozen=True)
class LocalModel:
  sha256: str
  bytes: int
  name: str
  added_at: float


class Registry:
  def __init__(self, cache_root: Path):
    self.root = Path(cache_root)
    self.state = self.root / 'registry'
    self.models = self.root / 'models'
    self.engines = self.root / 'engines'
    for directory in (self.state, self.models):
      directory.mkdir(parents=True, exist_ok=True)
    self.catalog_path = self.state / 'catalog.json'
    self.pointers_path = self.state / 'pointers.json'
    self.local_path = self.state / 'local-models.json'
    self._lock = threading.Lock()

  # --- catalog ---------------------------------------------------------------

  def catalog(self, refresh: bool = False, max_age: float = CATALOG_MAX_AGE, opener=None) -> dict:
    """The `catalog` event payload, from the cache when it is fresh enough.

    Pointers are not resolved here: that is one request per model, thirteen of
    them today, and it belongs on a worker the caller owns (`resolve_missing`).
    A failed refresh is not fatal; the previous list comes back with `error`.
    """
    cached = self._read_json(self.catalog_path, {})
    raw = cached.get('raw') if isinstance(cached, dict) else None
    fetched_at = cached.get('fetched_at') if isinstance(cached, dict) else None
    fetched_at = float(fetched_at) if isinstance(fetched_at, (int, float)) else None
    error = None

    if refresh or raw is None or fetched_at is None or time.time() - fetched_at > max_age:
      try:
        fresh = fetch_catalog(CATALOG_URL, opener=opener)
      except RegistryError as e:
        error = str(e)
      else:
        raw, fetched_at = fresh, time.time()
        self._write_json(self.catalog_path, {'fetched_at': fetched_at, 'url': CATALOG_URL, 'raw': raw})

    pointers = self._pointers()
    models = []
    for model in parse_catalog(raw or {}):
      pointer = pointers.get(model.ref) or {}
      models.append({'name': model.name, 'short_name': model.short_name, 'ref': model.ref, 'build_time': model.build_time,
                     'index': model.index, 'sha256': pointer.get('oid'), 'bytes': int(pointer['size']) if pointer.get('size') else None})
    return {'fetched_at': fetched_at, 'url': CATALOG_URL, 'default_ref': DEFAULT_BIG_MODEL_REF, 'error': error, 'models': models}

  def resolve(self, ref: str, opener=None) -> Pointer:
    """The pointer behind a catalog ref, fetched the first time and kept for good."""
    if not is_ref(ref):
      raise RegistryError(f"{ref!r} is not a 40 character commit")
    known = self._pointers().get(ref)
    if known:
      return Pointer(str(known['oid']), int(known['size']))
    pointer = fetch_pointer(ref, opener=opener)
    self._save_pointers({ref: pointer})
    log.info("%s is %s, %d MB", ref[:10], pointer.oid[:16], pointer.size >> 20)
    return pointer

  def resolve_missing(self, refs: list[str], workers: int = 8, opener=None) -> dict[str, Pointer | Exception]:
    """Resolve several refs at once. Failures come back as the exception, per ref."""
    known = self._pointers()
    out: dict[str, Pointer | Exception] = {}
    todo = []
    for ref in dict.fromkeys(refs):
      if not is_ref(ref):
        out[ref] = RegistryError(f"{ref!r} is not a 40 character commit")
      elif ref in known:
        out[ref] = Pointer(str(known[ref]['oid']), int(known[ref]['size']))
      else:
        todo.append(ref)

    if todo:
      with ThreadPoolExecutor(max_workers=max(1, min(workers, len(todo)))) as pool:
        futures = {pool.submit(fetch_pointer, ref, opener=opener): ref for ref in todo}
        for future, ref in futures.items():
          try:
            out[ref] = future.result()
          except Exception as e:   # one bad ref must not lose the other twelve
            out[ref] = e
      fresh = {ref: p for ref, p in out.items() if isinstance(p, Pointer) and ref in todo}
      if fresh:
        self._save_pointers(fresh)
    return out

  def name_for(self, sha256: str) -> tuple[str | None, str | None]:
    """A human name and the catalog ref for a model identity, either may be None."""
    for ref, pointer in self._pointers().items():
      if pointer.get('oid') == sha256:
        for model in parse_catalog((self._read_json(self.catalog_path, {}) or {}).get('raw') or {}):
          if model.ref == ref:
            return model.name, ref
        return None, ref
    for local in self.local_models():
      if local.sha256 == sha256:
        return local.name, None
    return None, None

  # --- models on disk --------------------------------------------------------

  def model_path(self, sha256: str) -> Path:
    """Where the server looks for an uploaded model. The rule is EngineCache's."""
    EngineCache._validate_sha256(sha256)
    return self.models / f"{sha256[:16]}.onnx"

  def fetch(self, ref_or_sha256: str, progress: ProgressFn | None = None, should_stop: StopFn | None = None, opener=None) -> Path:
    """Materialise one model's ONNX, from whichever LFS server has it."""
    pointer = self._pointer_for(ref_or_sha256, opener=opener)
    dest = self.model_path(pointer.oid)
    if dest.is_file() and dest.stat().st_size == pointer.size:
      return dest

    for endpoint in LFS_ENDPOINTS:
      href = lfs_resolve(endpoint, pointer, opener=opener)
      if href is None:
        continue
      log.info("fetching %s (%d MB) from %s", pointer.oid[:16], pointer.size >> 20, endpoint)
      return lfs_download(href, pointer, dest, progress=progress, should_stop=should_stop, opener=opener)
    raise NetworkError(f"no LFS server has {pointer.oid[:16]}")

  def import_model(self, path: Path, name: str | None = None, progress: ProgressFn | None = None,
                   should_stop: StopFn | None = None) -> LocalModel:
    """Take a model from disk into the cache under its own identity.

    Progress covers the whole operation: the hashing pass is the first half and
    the copy the second, because both read the file end to end.
    """
    path = Path(path)
    if path.suffix.lower() != '.onnx':
      raise RegistryError(f"{path.name} is not an .onnx file")
    if not path.is_file():
      raise RegistryError(f"{path} does not exist")

    total = path.stat().st_size
    sha256, nbytes = self._hash_file(path, total, progress, should_stop)
    dest = self.model_path(sha256)
    if not (dest.is_file() and dest.stat().st_size == nbytes):
      self._copy_file(path, dest, total, progress, should_stop)
    if progress is not None:
      progress(1.0)

    local = LocalModel(sha256=sha256, bytes=nbytes, name=name or path.stem, added_at=time.time())
    with self._lock:
      records = [r for r in self._local_records() if r.get('sha256') != sha256]
      records.append({'sha256': local.sha256, 'bytes': local.bytes, 'name': local.name, 'added_at': local.added_at})
      self._write_json(self.local_path, records)
    return local

  def local_models(self) -> list[LocalModel]:
    out = []
    for record in self._local_records():
      try:
        out.append(LocalModel(sha256=str(record['sha256']), bytes=int(record['bytes']), name=str(record.get('name') or ''),
                              added_at=float(record.get('added_at') or 0.0)))
      except (KeyError, TypeError, ValueError):
        continue
    return out

  # --- inventory and removal -------------------------------------------------

  def inventory(self, cache: EngineCache | None = None) -> dict:
    """The `inventory` event payload: what is on disk and what it belongs to.

    A model file is named by the first 16 characters of its identity, so a file
    whose full identity is in no pointer, no local record and no sidecar is
    listed with a 16 character `sha256`: the prefix is all anyone knows.
    """
    known = self._known_shas()
    models = []
    models_bytes = 0
    for path in sorted(self.models.glob('*.onnx')):
      stem = path.stem
      if _SHA16.fullmatch(stem) is None:
        continue
      try:
        nbytes = path.stat().st_size
      except OSError:
        continue
      sha256 = known.get(stem, stem)
      name, ref = self.name_for(sha256) if len(sha256) == 64 else (None, None)
      models.append({'sha256': sha256, 'bytes': nbytes, 'path': str(path), 'name': name, 'ref': ref})
      models_bytes += nbytes

    artifacts = []
    engines_bytes = 0
    for meta_path in sorted(self.engines.glob('*.json')):
      entry = self._artifact_entry(meta_path, cache)
      if entry is None:
        continue
      artifacts.append(entry)
      engines_bytes += entry['bytes']

    last = EngineCache(self.root).last_loaded()
    try:
      free = shutil.disk_usage(self.root).free
    except OSError:
      free = 0
    return {'loaded': None, 'last_loaded': last[0] if last else None, 'models': models, 'artifacts': artifacts,
            'disk': {'models_bytes': models_bytes, 'engines_bytes': engines_bytes, 'free_bytes': free}}

  def remove(self, sha256: str, artifacts: bool, model: bool) -> None:
    """Delete what was asked for. A file that is already gone is not an error.

    The loaded engine is not this class's business: a control server unloads
    first, and a CLI is not running one.
    """
    EngineCache._validate_sha256(sha256)
    sha16 = sha256[:16]
    if artifacts:
      for path in sorted(self.engines.glob(f"{sha16}.*")):
        try:
          if path.is_dir():
            shutil.rmtree(path, ignore_errors=True)
          else:
            path.unlink(missing_ok=True)
        except OSError:
          log.warning("could not remove %s", path)
      last = EngineCache(self.root).last_loaded()
      if last and last[0] == sha256:
        (self.root / LAST_LOADED).unlink(missing_ok=True)
    if model:
      path = self.model_path(sha256)
      path.unlink(missing_ok=True)
      path.with_name(path.name + '.part').unlink(missing_ok=True)
      # The local record is what gives an imported model its name. Left behind,
      # it names a file that is gone and the app offers to prepare it.
      with self._lock:
        records = self._local_records()
        kept = [r for r in records if r.get('sha256') != sha256]
        if len(kept) != len(records):
          self._write_json(self.local_path, kept)

  # --- internals -------------------------------------------------------------

  def _pointer_for(self, ref_or_sha256: str, opener=None) -> Pointer:
    if is_ref(ref_or_sha256):
      return self.resolve(ref_or_sha256, opener=opener)
    if is_sha256(ref_or_sha256):
      for pointer in self._pointers().values():
        if pointer.get('oid') == ref_or_sha256:
          return Pointer(ref_or_sha256, int(pointer['size']))
      raise RegistryError(f"size for {ref_or_sha256[:16]} unknown; fetch by catalog ref")
    raise RegistryError(f"{ref_or_sha256!r} is neither a 40 character ref nor a 64 character sha256")

  def _hash_file(self, path: Path, total: int, progress: ProgressFn | None, should_stop: StopFn | None) -> tuple[str, int]:
    digest = hashlib.sha256()
    read = 0
    with open(path, 'rb') as f:
      while chunk := f.read(HASH_CHUNK):
        if should_stop is not None and should_stop():
          raise RegistryError('import cancelled')
        digest.update(chunk)
        read += len(chunk)
        if progress is not None and total:
          progress(0.5 * min(1.0, read / total))
    return digest.hexdigest(), read

  def _copy_file(self, path: Path, dest: Path, total: int, progress: ProgressFn | None, should_stop: StopFn | None) -> None:
    part = dest.with_name(dest.name + '.part')
    written = 0
    try:
      with open(path, 'rb') as src, open(part, 'wb') as out:
        while chunk := src.read(COPY_CHUNK):
          if should_stop is not None and should_stop():
            raise RegistryError('import cancelled')
          out.write(chunk)
          written += len(chunk)
          if progress is not None and total:
            progress(0.5 + 0.5 * min(1.0, written / total))
    except RegistryError:
      part.unlink(missing_ok=True)
      raise
    except OSError as e:
      part.unlink(missing_ok=True)
      raise RegistryError(f"could not copy {path} into the cache: {e}") from e
    part.replace(dest)

  def _artifact_entry(self, meta_path: Path, cache: EngineCache | None) -> dict | None:
    try:
      meta = json.loads(meta_path.read_text())
      sha256 = meta['spec']['sha256']
    except (OSError, ValueError, KeyError, TypeError):
      return None
    if not is_sha256(sha256):
      return None
    key = meta_path.stem
    backend = str(meta.get('backend') or '')
    artifact = self._artifact_path(key, backend, meta_path)
    if artifact is None:
      return None
    return {
      'sha256': sha256,
      'key': key,
      'path': str(artifact),
      'bytes': _size_of(artifact),
      'backend': backend,
      'runtime_version': meta.get('onnxruntime') or meta.get('tinygrad') or meta.get('trt_version'),
      'device': str(meta.get('device') or ''),
      'built_at': meta.get('built_at'),
      'build_seconds': meta.get('build_seconds'),
      'checkpoint': (meta.get('spec') or {}).get('checkpoint'),
      'current': self._is_current(cache, sha256, meta_path),
    }

  def _artifact_path(self, key: str, backend: str, meta_path: Path) -> Path | None:
    suffix = ARTIFACT_SUFFIXES.get(backend)
    if suffix is not None:
      candidate = self.engines / f"{key}{suffix}"
      if candidate.exists():
        return candidate
    for path in sorted(self.engines.iterdir()):
      if path != meta_path and path.name.startswith(f"{key}."):
        return path
    return None

  @staticmethod
  def _is_current(cache: EngineCache | None, sha256: str, meta_path: Path) -> bool:
    if cache is None:
      return False
    try:
      # `cache.backend` selects a runtime, which on a machine without one raises.
      return cache.entry(sha256).meta_path == meta_path
    except Exception:
      return False

  def _known_shas(self) -> dict[str, str]:
    """prefix16 to full identity, from every place a full identity is written down."""
    out = {}
    for pointer in self._pointers().values():
      oid = pointer.get('oid')
      if is_sha256(oid):
        out[oid[:16]] = oid
    for local in self.local_models():
      if is_sha256(local.sha256):
        out[local.sha256[:16]] = local.sha256
    for meta_path in self.engines.glob('*.json'):
      try:
        sha256 = json.loads(meta_path.read_text())['spec']['sha256']
      except (OSError, ValueError, KeyError, TypeError):
        continue
      if is_sha256(sha256):
        out[sha256[:16]] = sha256
    return out

  def _pointers(self) -> dict[str, dict]:
    known = self._read_json(self.pointers_path, {})
    if not isinstance(known, dict):
      return {}
    return {ref: p for ref, p in known.items() if isinstance(p, dict) and p.get('oid') and p.get('size')}

  def _save_pointers(self, fresh: dict[str, Pointer]) -> None:
    with self._lock:
      known = self._pointers()
      known.update({ref: {'oid': p.oid, 'size': p.size} for ref, p in fresh.items()})
      self._write_json(self.pointers_path, known)

  def _local_records(self) -> list[dict]:
    records = self._read_json(self.local_path, [])
    return [r for r in records if isinstance(r, dict)] if isinstance(records, list) else []

  @staticmethod
  def _read_json(path: Path, default):
    try:
      return json.loads(path.read_text())
    except (OSError, ValueError):
      return default

  @staticmethod
  def _write_json(path: Path, value) -> None:
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(json.dumps(value))
    os.replace(tmp, path)


def _size_of(path: Path) -> int:
  """A file's size, or everything under a directory artifact."""
  try:
    if path.is_file():
      return path.stat().st_size
    return sum(p.stat().st_size for p in path.rglob('*') if p.is_file())
  except OSError:
    return 0
