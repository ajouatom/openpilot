"""Content fingerprints and compressed sidecars for local static assets.

JS/CSS fingerprints are derived from contained regular files and cached by
resolved path metadata. Requested and startup-precompressed sidecars share the
same asset lock, stable source snapshot, and atomic replacement path so a
downstream static handler never observes a partial gzip or Brotli file.
"""
from __future__ import annotations

import asyncio  # noqa: ANYIO_OK
from contextlib import suppress
from functools import cache
import gzip
import hashlib
import importlib
import os
from pathlib import Path
import stat
import tempfile
import threading
from typing import Final, Protocol, runtime_checkable
import zlib

from aiohttp import web
from aiohttp.typedefs import Handler, Middleware


@runtime_checkable
class _BrotliCodec(Protocol):
  error: type[Exception]

  def compress(self, data: bytes, *, quality: int) -> bytes: ...
  def decompress(self, data: bytes) -> bytes: ...


IMMUTABLE_STATIC_PREFIXES: Final = ("/js/", "/css/", "/assets/")
_VENDOR_STATIC_PREFIXES: Final = ("/js/vendor/", "/css/vendor/")
_ONE_YEAR: Final = 31536000
_COMPRESS_EXTS: Final = (".js", ".css")
_SOURCE_READ_ATTEMPTS: Final = 3


def create_static_cache_middleware(web_dir: str) -> Middleware:
  @web.middleware
  async def static_cache_middleware(request: web.Request, handler: Handler) -> web.StreamResponse:
    path = request.path
    is_static_method = request.method in ("GET", "HEAD")
    is_static_request = is_static_method and any(
      path.startswith(prefix) for prefix in IMMUTABLE_STATIC_PREFIXES
    )
    if is_static_request and path.endswith(_COMPRESS_EXTS):
      await asyncio.to_thread(_refresh_requested_gzip, web_dir, path)

    response = await handler(request)
    if is_static_request:
      versions = request.query.getall("v", [])
      is_vendor = any(path.startswith(prefix) for prefix in _VENDOR_STATIC_PREFIXES)
      has_current_version = bool(versions) if is_vendor else (
        len(versions) == 1 and fingerprint_static_asset(web_dir, path) == versions[0]
      )
      if has_current_version:
        response.headers.setdefault("Cache-Control", f"public, max-age={_ONE_YEAR}, immutable")
    return response

  return static_cache_middleware


def _resolve_local_asset(
  web_dir: str | os.PathLike[str],
  asset_path: str | os.PathLike[str],
) -> Path | None:
  raw_path = os.fspath(asset_path)
  if not raw_path.endswith(_COMPRESS_EXTS) or raw_path.startswith(("//", "\\\\")):
    return None
  relative_path = raw_path[1:] if raw_path.startswith("/") else raw_path
  try:
    web_root = Path(web_dir).resolve(strict=True)
    if not web_root.is_dir():
      return None
    source = (web_root / relative_path).resolve(strict=True)
    source.relative_to(web_root)
    source_stat = source.stat()
  except (OSError, RuntimeError, ValueError):
    return None
  if not stat.S_ISREG(source_stat.st_mode):
    return None
  return source


@cache
def _cached_content_hash(resolved_path: str, _mtime_ns: int, _size: int) -> str:
  with open(resolved_path, "rb") as source_file:
    return hashlib.sha256(source_file.read()).hexdigest()


def fingerprint_static_asset(
  web_root: str | os.PathLike[str],
  asset_path: str | os.PathLike[str],
) -> str | None:
  """Return a SHA-256 fingerprint for a contained regular JS/CSS file."""
  source = _resolve_local_asset(web_root, asset_path)
  if source is None:
    return None
  for _attempt in range(_SOURCE_READ_ATTEMPTS):
    try:
      source_stat = source.stat()
      fingerprint = _cached_content_hash(str(source), source_stat.st_mtime_ns, source_stat.st_size)
      current_stat = source.stat()
    except OSError:
      return None
    if (current_stat.st_mtime_ns, current_stat.st_size) == (source_stat.st_mtime_ns, source_stat.st_size):
      return fingerprint
  return None


@cache
def _asset_lock(resolved_path: str) -> threading.Lock:
  return threading.Lock()


def _source_key(source: Path) -> tuple[int, int] | None:
  try:
    source_stat = source.stat()
  except OSError:
    return None
  if not stat.S_ISREG(source_stat.st_mode):
    return None
  return source_stat.st_mtime_ns, source_stat.st_size


def _read_stable_source(source: Path) -> tuple[bytes, tuple[int, int]] | None:
  before = _source_key(source)
  if before is None:
    return None
  try:
    data = source.read_bytes()
  except OSError:
    return None
  after = _source_key(source)
  if after != before or len(data) != before[1]:
    return None
  return data, before


def _gzip_matches(sidecar: Path, source_data: bytes) -> bool:
  try:
    return gzip.decompress(sidecar.read_bytes()) == source_data
  except (EOFError, OSError, zlib.error):
    return False


def _brotli_matches(sidecar: Path, source_data: bytes, codec: _BrotliCodec) -> bool:
  try:
    return codec.decompress(sidecar.read_bytes()) == source_data
  except (OSError, codec.error):
    return False


def _atomic_replace(destination: Path, payload: bytes) -> None:
  descriptor, temporary_name = tempfile.mkstemp(
    dir=destination.parent,
    prefix=f".{destination.name}.",
    suffix=".tmp",
  )
  os.close(descriptor)
  temporary = Path(temporary_name)
  try:
    temporary.write_bytes(payload)
    os.replace(temporary, destination)
  finally:
    with suppress(FileNotFoundError):
      temporary.unlink()


def _remove_sidecar(sidecar: Path) -> None:
  with suppress(FileNotFoundError):
    sidecar.unlink()


def _refresh_static_asset(
  web_dir: str | os.PathLike[str],
  asset_path: str | os.PathLike[str],
  codec: _BrotliCodec | None,
) -> None:
  source = _resolve_local_asset(web_dir, asset_path)
  if source is None:
    return
  gzip_path = source.with_name(f"{source.name}.gz")
  brotli_path = source.with_name(f"{source.name}.br")
  with _asset_lock(str(source)):
    for _attempt in range(_SOURCE_READ_ATTEMPTS):
      snapshot = _read_stable_source(source)
      if snapshot is None:
        continue
      source_data, source_key = snapshot
      gzip_payload = None if _gzip_matches(gzip_path, source_data) else gzip.compress(source_data, 9)
      brotli_payload = None
      if codec is not None and not _brotli_matches(brotli_path, source_data, codec):
        brotli_payload = codec.compress(source_data, quality=11)
      if _source_key(source) != source_key:
        continue
      if gzip_payload is not None:
        _atomic_replace(gzip_path, gzip_payload)
      if _source_key(source) != source_key:
        continue
      if codec is None:
        _remove_sidecar(brotli_path)
      elif brotli_payload is not None:
        _atomic_replace(brotli_path, brotli_payload)
      if _source_key(source) == source_key:
        return
    _remove_sidecar(gzip_path)
    _remove_sidecar(brotli_path)


def _load_brotli() -> _BrotliCodec | None:
  try:
    module = importlib.import_module("brotli")
  except ImportError:
    return None
  return module if isinstance(module, _BrotliCodec) else None


def _refresh_requested_gzip(web_dir: str, request_path: str) -> None:
  """Refresh requested gzip and optional Brotli sidecars before static lookup."""
  _refresh_static_asset(web_dir, request_path, _load_brotli())


def precompress_static_assets(web_dir: str) -> None:
  """Generate matching gzip/Brotli siblings for JS/CSS under ``web_dir``."""
  codec = _load_brotli()
  for subdirectory in ("js", "css"):
    root = os.path.join(web_dir, subdirectory)
    if not os.path.isdir(root):
      continue
    for directory, _subdirectories, files in os.walk(root):
      for name in files:
        if not name.endswith(_COMPRESS_EXTS):
          continue
        try:
          source_path = os.path.relpath(os.path.join(directory, name), web_dir)
          _refresh_static_asset(web_dir, source_path, codec)
        except OSError:
          continue


def start_precompress(web_dir: str) -> asyncio.Task[None]:
  """Kick precompression off the event loop without blocking startup."""
  async def _run() -> None:
    await asyncio.to_thread(precompress_static_assets, web_dir)

  return asyncio.create_task(_run())
