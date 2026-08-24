import hashlib
import io
import json
from pathlib import Path

import pytest

import openpilot.selfdrive.modeld.big_model as big_model
from openpilot.selfdrive.modeld.big_model import BigModelManifest


class FakeResponse:
  def __init__(self, data: bytes, status: int = 200, headers=None):
    self._stream = io.BytesIO(data)
    self.status = status
    self.headers = headers or {}

  def __enter__(self):
    return self

  def __exit__(self, *_args):
    return False

  def read(self, size=-1):
    return self._stream.read(size)

  def getcode(self):
    return self.status


def manifest_for(data: bytes, model_id: str = "big-400") -> dict:
  return {
    "model_id": model_id,
    "filename": "big_driving_supercombo.onnx",
    "size": len(data),
    "sha256": hashlib.sha256(data).hexdigest(),
    "url": "big_driving_supercombo.onnx",
  }


def test_manifest_resolves_relative_https_url():
  manifest = BigModelManifest.from_dict(manifest_for(b"model"), "https://example.com/models/manifest.json")
  assert manifest.url == "https://example.com/models/big_driving_supercombo.onnx"
  assert manifest.cache_filename.endswith(".onnx")


@pytest.mark.parametrize("field,value", [
  ("model_id", "../bad"),
  ("filename", "../model.onnx"),
  ("size", 0),
  ("sha256", "bad"),
  ("url", "http://example.com/model.onnx"),
])
def test_manifest_rejects_unsafe_values(field, value):
  raw = manifest_for(b"model")
  raw[field] = value
  with pytest.raises(ValueError):
    BigModelManifest.from_dict(raw)


def test_ensure_downloads_verifies_and_activates(monkeypatch, tmp_path: Path):
  data = b"verified model data"
  raw_manifest = manifest_for(data)
  manifest_url = "https://example.com/models/manifest.json"

  def fake_urlopen(request, timeout):
    if request.full_url == manifest_url:
      return FakeResponse(json.dumps(raw_manifest).encode())
    assert request.full_url == "https://example.com/models/big_driving_supercombo.onnx"
    return FakeResponse(data)

  monkeypatch.setattr(big_model, "urlopen", fake_urlopen)
  path, changed = big_model.ensure_big_model(manifest_url, tmp_path)
  assert changed
  assert path.read_bytes() == data
  assert big_model.active_model_path(tmp_path) == path

  _, changed = big_model.ensure_big_model(manifest_url, tmp_path)
  assert not changed


def test_download_resumes_partial_file(monkeypatch, tmp_path: Path):
  data = b"0123456789"
  manifest = BigModelManifest.from_dict(manifest_for(data), "https://example.com/models/manifest.json")
  final_path = big_model.model_path(manifest, tmp_path)
  partial_path = final_path.with_suffix(final_path.suffix + ".part")
  tmp_path.mkdir(exist_ok=True)
  partial_path.write_bytes(data[:4])

  def fake_urlopen(request, timeout):
    assert request.headers["Range"] == "bytes=4-"
    return FakeResponse(data[4:], status=206, headers={"Content-Range": "bytes 4-9/10"})

  monkeypatch.setattr(big_model, "urlopen", fake_urlopen)
  assert big_model._download_model(manifest, tmp_path).read_bytes() == data


def test_bad_hash_never_becomes_active(monkeypatch, tmp_path: Path):
  expected = b"good"
  manifest = BigModelManifest.from_dict(manifest_for(expected), "https://example.com/models/manifest.json")
  monkeypatch.setattr(big_model, "urlopen", lambda request, timeout: FakeResponse(b"baad"))
  with pytest.raises(OSError, match="sha256 mismatch"):
    big_model._download_model(manifest, tmp_path)
  assert not big_model.model_path(manifest, tmp_path).exists()
  assert not big_model.model_path(manifest, tmp_path).with_suffix(".onnx.part").exists()
