"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Where built engines and uploaded models live.

    <root>/engines/<sha16>.<backend tag><suffix>     the artifact
    <root>/engines/<sha16>.<backend tag>.json        its sidecar: build facts and the model spec
    <root>/models/<sha16>.onnx                       the model as uploaded, never pruned

The key is the model's identity plus the backend's tag, so a laptop with two
backends installed keeps two artifacts per model and each backend sees only its
own. The tag is the backend's to define; for TensorRT it is byte for byte what
it was before there were backends, so a Jetson's existing plans keep loading.
"""
from __future__ import annotations

import json
import logging
import re
import shutil
import time
from dataclasses import dataclass
from pathlib import Path

from jetlink.server.backends.base import Backend
from jetlink.server.platform import default_cache_dir

log = logging.getLogger('jetlink.cache')

# One per registry entry: a rebuild costs minutes and a plan under 2 GB of a
# 900 GB disk, so a smaller cap makes an A/B rebuild on every switch.
KEEP_PLANS = 6

# What the server loaded last, so a fresh process can preload it. Beside the
# caches rather than in them: it describes the server, not a plan.
LAST_LOADED = 'last-loaded.json'

_SHA256 = re.compile(r'[0-9a-f]{64}')


@dataclass
class CacheEntry:
  path: Path        # the artifact: a file for TensorRT and tinygrad, a directory for onnxruntime
  meta_path: Path

  @property
  def exists(self) -> bool:
    return self.path.exists() and self.meta_path.is_file()

  def meta(self) -> dict:
    return json.loads(self.meta_path.read_text())

  def write_meta(self, meta: dict) -> None:
    self.meta_path.write_text(json.dumps(meta, indent=2))

  def remove(self) -> None:
    """Both halves, whichever exist. A directory artifact goes whole."""
    if self.path.is_dir():
      shutil.rmtree(self.path, ignore_errors=True)
    else:
      self.path.unlink(missing_ok=True)
    self.meta_path.unlink(missing_ok=True)


class EngineCache:
  def __init__(self, root: str | Path | None = None, backend: Backend | None = None):
    self.root = Path(root) if root is not None else default_cache_dir()
    self._backend = backend
    self.engines = self.root / 'engines'
    self.models = self.root / 'models'
    for d in (self.engines, self.models):
      d.mkdir(parents=True, exist_ok=True)

  @property
  def backend(self) -> Backend:
    # Resolved late so a cache can be opened before the backend is chosen, and
    # so tooling that only wants model_path() never imports a runtime.
    if self._backend is None:
      from jetlink.server.backends import select
      self._backend = select('auto')
    return self._backend

  def key(self, model_sha256: str) -> str:
    self._validate_sha256(model_sha256)
    return f"{model_sha256[:16]}.{self.backend.tag()}"

  def entry(self, model_sha256: str) -> CacheEntry:
    k = self.key(model_sha256)
    return CacheEntry(self.engines / f"{k}{self.backend.suffix}", self.engines / f"{k}.json")

  def model_path(self, model_sha256: str) -> Path:
    self._validate_sha256(model_sha256)
    return self.models / f"{model_sha256[:16]}.onnx"

  @staticmethod
  def _validate_sha256(value: str) -> None:
    # Model identities arrive from the peer and become filesystem paths.
    if _SHA256.fullmatch(value) is None:
      raise ValueError('model identity must be a lowercase SHA-256 digest')

  def inventory(self) -> list[str]:
    """Model identities with artifacts this backend can load on this device."""
    found = []
    for meta in self.engines.glob('*.json'):
      try:
        sha = json.loads(meta.read_text())['spec']['sha256']
        if (isinstance(sha, str) and _SHA256.fullmatch(sha)
            and self.entry(sha).meta_path == meta and self.entry(sha).exists):
          found.append(sha)
      except (OSError, ValueError, KeyError, TypeError):
        continue
    return sorted(set(found))

  def remember_loaded(self, sha256: str, frame_skip: int) -> None:
    """Record what is loaded, for the next process to preload.

    frame_skip goes with it: the spec served is stamped with it, so preloading
    under another value hands the next client a spec it did not ask for. The
    backend is recorded for the log; the key already keeps artifacts apart.
    """
    try:
      (self.root / LAST_LOADED).write_text(json.dumps(
        {'sha256': sha256, 'frame_skip': frame_skip, 'backend': self.backend.name}))
    except OSError:
      pass   # a read-only cache still serves; it just cannot preload next time

  def last_loaded(self) -> tuple[str, int] | None:
    try:
      d = json.loads((self.root / LAST_LOADED).read_text())
      sha, skip = d['sha256'], int(d['frame_skip'])
    except (OSError, ValueError, KeyError, TypeError):
      return None
    return (sha, skip) if _SHA256.fullmatch(sha) else None

  def prune(self, keep: int = KEEP_PLANS, protect: Path | None = None) -> None:
    """Keep the newest few artifacts of this backend's kind; each is ~770 MB.

    `protect` is never pruned whatever its mtime says: the Jetson boots at 1970
    without NTP, so a plan built offroad looks older than everything on disk and
    a fresh build would be the first one deleted. Other backends' artifacts are
    not touched: on a laptop with two runtimes each keeps its own set.
    """
    found = [p for p in self.engines.glob(f'*{self.backend.suffix}') if protect is None or p != protect]
    found.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    for p in found[max(keep - (protect is not None), 0):]:
      CacheEntry(p, p.with_suffix('.json')).remove()

  def sweep_temp(self, max_age: float = 6 * 3600) -> None:
    """Drop build directories a crashed or killed build left behind.

    Builds stage their artifact in a TemporaryDirectory inside engines/, which
    prune() does not glob.
    """
    now = time.time()
    for d in self.engines.glob('tmp*'):
      if not d.is_dir():
        continue
      try:
        if now - d.stat().st_mtime < max_age:
          continue
        shutil.rmtree(d, ignore_errors=True)
      except OSError:
        pass
