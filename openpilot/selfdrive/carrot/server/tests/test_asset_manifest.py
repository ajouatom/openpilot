from __future__ import annotations

import asyncio
import hashlib
import json

import pytest

from openpilot.selfdrive.carrot.server.services.asset_manifest import (
  AssetManifestError,
  AssetManifestLoader,
)


def _content_hash(payload: bytes) -> str:
  return hashlib.sha256(payload.replace(b"\r\n", b"\n")).hexdigest()


def _write_manifest(web_dir, payload: bytes) -> None:
  generated = web_dir / "generated"
  generated.mkdir(exist_ok=True)
  manifest = {
    "schemaVersion": 1,
    "assets": [{
      "id": "app.runtime",
      "kind": "bundle",
      "source": "src/entries/app.js",
      "path": "js/generated/app.js",
      "hash": _content_hash(payload),
    }],
  }
  (generated / "asset-manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def test_runtime_loader_repairs_stale_hash_and_strict_validation_reports_it(tmp_path):
  web_dir = tmp_path / "web"
  asset_dir = web_dir / "js" / "generated"
  asset_dir.mkdir(parents=True)
  asset_path = asset_dir / "app.js"
  original = b"original asset\n"
  asset_path.write_bytes(original)
  _write_manifest(web_dir, original)

  loader = AssetManifestLoader()
  assert loader.load(web_dir)["assets"][0]["hash"] == _content_hash(original)

  updated = b"updated asset with a different size\n"
  asset_path.write_bytes(updated)
  assert loader.load(web_dir)["assets"][0]["hash"] == _content_hash(updated)
  with pytest.raises(AssetManifestError, match="content hash"):
    loader.validate(web_dir)


def test_loader_recovers_after_manifest_and_asset_finish_updating(tmp_path):
  web_dir = tmp_path / "web"
  asset_dir = web_dir / "js" / "generated"
  asset_dir.mkdir(parents=True)
  asset_path = asset_dir / "app.js"
  original = b"original asset\n"
  updated = b"updated asset\n"
  asset_path.write_bytes(original)
  _write_manifest(web_dir, original)

  loader = AssetManifestLoader()
  loader.load(web_dir)

  asset_path.write_bytes(updated)
  assert loader.load(web_dir)["assets"][0]["hash"] == _content_hash(updated)
  with pytest.raises(AssetManifestError):
    loader.validate(web_dir)

  _write_manifest(web_dir, updated)
  manifest = loader.validate(web_dir)
  assert manifest["assets"][0]["hash"] == _content_hash(updated)


def test_index_falls_back_to_empty_manifest_when_manifest_is_unavailable(tmp_path, monkeypatch):
  from openpilot.selfdrive.carrot.server.features import static

  web_dir = tmp_path / "web"
  web_dir.mkdir()
  (web_dir / "index.html").write_text(
    '<!doctype html><script id="carrotAssetManifest" type="application/json"></script>',
    encoding="utf-8",
  )
  monkeypatch.setattr(static, "WEB_DIR", str(web_dir))
  monkeypatch.setattr(static, "_INDEX_RETRY_DELAYS", (0.0,))
  monkeypatch.setattr(static, "_ASSET_MANIFEST_LOADER", AssetManifestLoader())

  loaded = asyncio.run(static._load_index_after_update())

  assert loaded is not None
  html, degraded = loaded
  assert degraded is True
  assert '"schemaVersion":1,"assets":[]' in html


def test_runtime_hash_recovery_does_not_allow_paths_outside_web_root(tmp_path):
  web_dir = tmp_path / "web"
  generated = web_dir / "generated"
  generated.mkdir(parents=True)
  outside = tmp_path / "outside.js"
  outside.write_text("outside", encoding="utf-8")
  manifest = {
    "schemaVersion": 1,
    "assets": [{
      "id": "outside.worker",
      "kind": "worker",
      "path": "../outside.js",
      "hash": _content_hash(outside.read_bytes()),
    }],
  }
  (generated / "asset-manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

  with pytest.raises(AssetManifestError, match="relative"):
    AssetManifestLoader().load(web_dir)
