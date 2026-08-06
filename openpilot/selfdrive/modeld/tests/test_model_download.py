import hashlib
import io
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.modeld.model_download import ModelDownloadError, ensure_rdf_model, load_model_manifest


def write_manifest(path: Path, payload: bytes, url: str = "https://example.test/model.onnx") -> None:
  path.write_text(json.dumps({
    "version": 1,
    "url": url,
    "size": len(payload),
    "sha256": hashlib.sha256(payload).hexdigest(),
  }), encoding="utf-8")


def test_existing_model_is_not_downloaded(tmp_path: Path):
  payload = b"rdf-model"
  model_path = tmp_path / "model.onnx"
  manifest_path = tmp_path / "model.json"
  model_path.write_bytes(payload)
  write_manifest(manifest_path, payload)

  def fail_open(*args, **kwargs):
    raise AssertionError("download should not be attempted")

  assert not ensure_rdf_model(model_path, manifest_path, urlopen_fn=fail_open)


def test_download_is_verified_and_installed_atomically(tmp_path: Path):
  payload = b"new-rdf-model"
  model_path = tmp_path / "model.onnx"
  manifest_path = tmp_path / "model.json"
  model_path.write_bytes(b"old-model")
  write_manifest(manifest_path, payload)
  progress = []

  assert ensure_rdf_model(model_path, manifest_path,
                          progress=lambda downloaded, total: progress.append((downloaded, total)),
                          urlopen_fn=lambda request, timeout: io.BytesIO(payload))
  assert model_path.read_bytes() == payload
  assert progress[-1] == (len(payload), len(payload))
  assert not list(tmp_path.glob(".*.download-*"))


def test_bad_download_preserves_existing_model(tmp_path: Path):
  payload = b"expected-model"
  model_path = tmp_path / "model.onnx"
  manifest_path = tmp_path / "model.json"
  model_path.write_bytes(b"keep-me")
  write_manifest(manifest_path, payload)

  with pytest.raises(ModelDownloadError):
    ensure_rdf_model(model_path, manifest_path,
                     urlopen_fn=lambda request, timeout: io.BytesIO(b"wrong"),
                     attempts=1)
  assert model_path.read_bytes() == b"keep-me"
  assert not list(tmp_path.glob(".*.download-*"))


def test_https_mirror_override(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
  payload = b"mirrored-rdf-model"
  model_path = tmp_path / "model.onnx"
  manifest_path = tmp_path / "model.json"
  write_manifest(manifest_path, payload)
  monkeypatch.setenv("RDF_MODEL_URL", "https://nas.example.test/rdf.onnx")
  requested_urls = []

  def open_mirror(request, timeout):
    requested_urls.append(request.full_url)
    return io.BytesIO(payload)

  assert ensure_rdf_model(model_path, manifest_path, urlopen_fn=open_mirror)
  assert requested_urls == ["https://nas.example.test/rdf.onnx"]


@pytest.mark.parametrize("patch", [
  {"version": 2},
  {"url": "http://example.test/model.onnx"},
  {"size": 0},
  {"sha256": "not-a-hash"},
])
def test_invalid_manifest_is_rejected(tmp_path: Path, patch: dict):
  path = tmp_path / "model.json"
  data = {
    "version": 1,
    "url": "https://example.test/model.onnx",
    "size": 1,
    "sha256": "0" * 64,
  }
  data.update(patch)
  path.write_text(json.dumps(data), encoding="utf-8")

  with pytest.raises(ModelDownloadError):
    load_model_manifest(path)
