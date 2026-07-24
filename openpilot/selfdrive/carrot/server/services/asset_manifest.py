from __future__ import annotations

import hashlib
import json
import logging
import os
from pathlib import Path, PurePosixPath
import re
import stat
import threading
from typing import Final, Literal, NotRequired, TypedDict


class AssetRecord(TypedDict):
  id: str
  kind: Literal["bundle", "worker"]
  path: str
  hash: str
  source: NotRequired[str]


class AssetManifest(TypedDict):
  schemaVersion: int
  assets: list[AssetRecord]


class AssetManifestError(RuntimeError):
  pass


_MANIFEST_RELATIVE_PATH: Final = Path("generated") / "asset-manifest.json"
_LOGICAL_ID_RE: Final = re.compile(r"^[a-z][a-z0-9]*(?:[._-][a-z0-9]+)*$")
_HASH_RE: Final = re.compile(r"^[0-9a-f]{64}$")
_READ_ATTEMPTS: Final = 3
_LOGGER = logging.getLogger(__name__)


def _asset_content_hash(payload: bytes) -> str:
  return hashlib.sha256(payload.replace(b"\r\n", b"\n")).hexdigest()


def _contained_file(web_root: Path, relative_path: str) -> Path:
  pure_path = PurePosixPath(relative_path)
  if pure_path.is_absolute() or ".." in pure_path.parts or "\\" in relative_path:
    raise AssetManifestError("Asset path must be relative")
  try:
    candidate = (web_root / Path(*pure_path.parts)).resolve(strict=True)
    candidate.relative_to(web_root)
    metadata = candidate.stat()
  except (OSError, RuntimeError, ValueError) as error:
    raise AssetManifestError("Asset path is unavailable") from error
  if not stat.S_ISREG(metadata.st_mode):
    raise AssetManifestError("Asset path is not a regular file")
  return candidate


def _parse_asset(
  web_root: Path,
  raw_asset: object,
  *,
  repair_content_hash: bool,
) -> AssetRecord:
  if not isinstance(raw_asset, dict):
    raise AssetManifestError("Asset entry must be an object")
  asset_id = raw_asset.get("id")
  kind = raw_asset.get("kind")
  path = raw_asset.get("path")
  content_hash = raw_asset.get("hash")
  source = raw_asset.get("source")
  if not isinstance(asset_id, str) or _LOGICAL_ID_RE.fullmatch(asset_id) is None:
    raise AssetManifestError("Asset id is invalid")
  if kind not in ("bundle", "worker"):
    raise AssetManifestError("Asset kind is invalid")
  if not isinstance(path, str) or not isinstance(content_hash, str):
    raise AssetManifestError("Asset path or hash is invalid")
  if _HASH_RE.fullmatch(content_hash) is None:
    raise AssetManifestError("Asset hash is invalid")
  if (kind == "bundle") != isinstance(source, str):
    raise AssetManifestError("Bundle source contract is invalid")

  asset_path = _contained_file(web_root, path)
  actual_hash = _asset_content_hash(asset_path.read_bytes())
  if actual_hash != content_hash:
    if not repair_content_hash:
      raise AssetManifestError("Asset content hash does not match")
    _LOGGER.warning(
      "Asset manifest hash is stale for %s; using the current file content",
      asset_id,
    )
  asset: AssetRecord = {"id": asset_id, "kind": kind, "path": path, "hash": actual_hash}
  if isinstance(source, str):
    asset["source"] = source
  return asset


def _parse_manifest(
  web_root: Path,
  payload: bytes,
  *,
  repair_content_hash: bool,
) -> AssetManifest:
  try:
    raw_manifest = json.loads(payload)
  except (UnicodeDecodeError, json.JSONDecodeError) as error:
    raise AssetManifestError("Asset manifest JSON is invalid") from error
  if not isinstance(raw_manifest, dict) or set(raw_manifest) != {"schemaVersion", "assets"}:
    raise AssetManifestError("Asset manifest shape is invalid")
  if raw_manifest.get("schemaVersion") != 1 or not isinstance(raw_manifest.get("assets"), list):
    raise AssetManifestError("Asset manifest schema is invalid")

  assets = [
    _parse_asset(web_root, raw_asset, repair_content_hash=repair_content_hash)
    for raw_asset in raw_manifest["assets"]
  ]
  if len({asset["id"] for asset in assets}) != len(assets):
    raise AssetManifestError("Asset ids must be unique")
  if len({asset["path"] for asset in assets}) != len(assets):
    raise AssetManifestError("Asset paths must be unique")
  return {"schemaVersion": 1, "assets": assets}


class AssetManifestLoader:
  def __init__(self) -> None:
    self._cache_key: tuple[str, int, int, bool] | None = None
    self._manifest: AssetManifest | None = None
    self._asset_keys: tuple[tuple[str, int, int], ...] = ()
    self._lock = threading.Lock()

  @staticmethod
  def _asset_key(web_root: Path, asset: AssetRecord) -> tuple[str, int, int] | None:
    try:
      asset_path = _contained_file(web_root, asset["path"])
      metadata = asset_path.stat()
    except (AssetManifestError, OSError):
      return None
    return str(asset_path), metadata.st_mtime_ns, metadata.st_size

  def _cached_assets_unchanged(self, web_root: Path) -> bool:
    if self._manifest is None or len(self._asset_keys) != len(self._manifest["assets"]):
      return False
    return all(
      self._asset_key(web_root, asset) == expected
      for asset, expected in zip(self._manifest["assets"], self._asset_keys, strict=True)
    )

  def load(self, web_dir: str | os.PathLike[str]) -> AssetManifest:
    with self._lock:
      return self._load(web_dir, repair_content_hash=True)

  def validate(self, web_dir: str | os.PathLike[str]) -> AssetManifest:
    """Strictly validate build output without applying runtime hash recovery."""
    with self._lock:
      return self._load(web_dir, repair_content_hash=False)

  def _load(
    self,
    web_dir: str | os.PathLike[str],
    *,
    repair_content_hash: bool,
  ) -> AssetManifest:
    try:
      web_root = Path(web_dir).resolve(strict=True)
      manifest_path = (web_root / _MANIFEST_RELATIVE_PATH).resolve(strict=True)
      manifest_path.relative_to(web_root)
    except (OSError, RuntimeError, ValueError) as error:
      raise AssetManifestError("Asset manifest is missing") from error

    for _attempt in range(_READ_ATTEMPTS):
      try:
        before = manifest_path.stat()
        cache_key = (
          str(manifest_path),
          before.st_mtime_ns,
          before.st_size,
          repair_content_hash,
        )
        if (
          cache_key == self._cache_key
          and self._manifest is not None
          and self._cached_assets_unchanged(web_root)
        ):
          return self._manifest
        payload = manifest_path.read_bytes()
        after = manifest_path.stat()
      except OSError as error:
        raise AssetManifestError("Asset manifest cannot be read") from error
      if (after.st_mtime_ns, after.st_size) != (before.st_mtime_ns, before.st_size):
        continue
      manifest = _parse_manifest(
        web_root,
        payload,
        repair_content_hash=repair_content_hash,
      )
      asset_keys = tuple(self._asset_key(web_root, asset) for asset in manifest["assets"])
      if any(asset_key is None for asset_key in asset_keys):
        raise AssetManifestError("Asset path changed after validation")
      self._cache_key = cache_key
      self._manifest = manifest
      self._asset_keys = tuple(asset_key for asset_key in asset_keys if asset_key is not None)
      return manifest
    raise AssetManifestError("Asset manifest changed while reading")


def inject_asset_manifest(html: str, manifest: AssetManifest) -> str:
  placeholder = '<script id="carrotAssetManifest" type="application/json"></script>'
  if html.count(placeholder) != 1:
    raise AssetManifestError("Asset manifest placeholder is missing or duplicated")
  payload = json.dumps(manifest, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c")
  replacement = f'<script id="carrotAssetManifest" type="application/json">{payload}</script>'
  return html.replace(placeholder, replacement, 1)
