from __future__ import annotations

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


def test_cached_manifest_is_invalidated_when_asset_changes(tmp_path):
  web_dir = tmp_path / "web"
  asset_dir = web_dir / "js" / "generated"
  asset_dir.mkdir(parents=True)
  asset_path = asset_dir / "app.js"
  original = b"original asset\n"
  asset_path.write_bytes(original)
  _write_manifest(web_dir, original)

  loader = AssetManifestLoader()
  assert loader.load(web_dir)["assets"][0]["hash"] == _content_hash(original)

  asset_path.write_bytes(b"updated asset with a different size\n")
  with pytest.raises(AssetManifestError, match="content hash"):
    loader.load(web_dir)


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
  with pytest.raises(AssetManifestError):
    loader.load(web_dir)

  _write_manifest(web_dir, updated)
  manifest = loader.load(web_dir)
  assert manifest["assets"][0]["hash"] == _content_hash(updated)
