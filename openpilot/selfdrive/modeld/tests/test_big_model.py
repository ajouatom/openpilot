import hashlib
import io
import json
from pathlib import Path

import pytest

import openpilot.selfdrive.modeld.big_model as big_model
from openpilot.selfdrive.modeld.big_model import BigModelManifest
from openpilot.selfdrive.modeld.big_model_status import BigModelStatusReporter, read_big_model_status, write_big_model_status


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


class FakeParams:
  def __init__(self, remembered=False):
    self.remembered = remembered
    self.puts = []

  def get_bool(self, key):
    assert key == "UsbGpuHardwareSeen"
    return self.remembered

  def put_bool(self, key, value):
    assert key == "UsbGpuHardwareSeen"
    self.puts.append((key, value))
    self.remembered = value


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


def test_default_manifest_is_pinned_to_tgc(monkeypatch):
  monkeypatch.setattr(big_model, "urlopen", lambda *_args, **_kwargs: pytest.fail("default manifest must be built in"))
  manifest = big_model.fetch_manifest()
  assert manifest.model_id == "comma-pr38739-tgc-a2e422ee-1791d594"
  assert manifest.size == 765_950_064
  assert manifest.sha256 == "1791d5940b2c048d0639813426dd2cf1d6f2a6727ed51e17c8bcea8bbe754123"
  assert manifest.url == "https://upload.shind0.synology.me/models/comma4-big-tgc/big_driving_supercombo.onnx"


def test_tgc_tinygrad_custom_op_is_supported():
  source = (Path(big_model.__file__).parents[3] / "tinygrad_repo/tinygrad/nn/onnx.py").read_text(encoding="utf-8")
  assert 'TINYGRAD = "org.tinygrad"' in source
  assert "Contiguous = {OpSetId(Domain.TINYGRAD, 1):contiguous_1}" in source


def test_explicit_manifest_override_is_fetched(monkeypatch):
  manifest_url = "https://example.com/models/manifest.json"
  raw_manifest = manifest_for(b"model")

  def fake_urlopen(request, timeout):
    assert request.full_url == manifest_url
    return FakeResponse(json.dumps(raw_manifest).encode())

  monkeypatch.setattr(big_model, "urlopen", fake_urlopen)
  manifest = big_model.fetch_manifest(manifest_url)
  assert manifest.model_id == raw_manifest["model_id"]
  assert manifest.url == "https://example.com/models/big_driving_supercombo.onnx"


class FakeConnection:
  def __init__(self):
    self.closed = False

  def close(self):
    self.closed = True


def test_wait_for_manifest_network_connects_to_manifest_host(monkeypatch):
  connection = FakeConnection()
  calls = []

  def fake_create_connection(address, timeout):
    calls.append((address, timeout))
    return connection

  monkeypatch.setattr(big_model.socket, "create_connection", fake_create_connection)
  assert big_model.wait_for_manifest_network("https://example.com:8443/models/manifest.json", timeout=0)
  assert calls == [(('example.com', 8443), 0.1)]
  assert connection.closed


def test_wait_for_manifest_network_times_out(monkeypatch):
  def fail_connection(_address, timeout):
    assert timeout == 0.1
    raise OSError("offline")

  monkeypatch.setattr(big_model.socket, "create_connection", fail_connection)
  assert not big_model.wait_for_manifest_network("https://example.com/manifest.json", timeout=0)


@pytest.mark.parametrize("remembered,present,compiled,expected,write", [
  (False, False, False, False, False),
  (False, True, False, True, True),
  (False, False, True, True, True),
  (True, False, False, True, False),
])
def test_remember_usbgpu_connection(remembered, present, compiled, expected, write):
  params = FakeParams(remembered)
  assert big_model.remember_usbgpu_connection(params, present, compiled) is expected
  assert bool(params.puts) is write


def test_background_update_does_not_treat_cached_onnx_as_hardware_history():
  source = Path(big_model.__file__).read_text(encoding="utf-8")
  assert "remember_usbgpu_connection(Params(), present, usbgpu_compiled())" in source
  assert "remember_usbgpu_connection(Params(), present, active_manifest() is not None)" not in source


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


def test_download_reports_resumed_progress_and_verification(monkeypatch, tmp_path: Path):
  data = b"0123456789"
  manifest = BigModelManifest.from_dict(manifest_for(data), "https://example.com/models/manifest.json")
  partial_path = big_model.model_path(manifest, tmp_path).with_suffix(".onnx.part")
  tmp_path.mkdir(exist_ok=True)
  partial_path.write_bytes(data[:4])
  progress = []
  phases = []

  monkeypatch.setattr(big_model, "urlopen", lambda _request, timeout: FakeResponse(
    data[4:], status=206, headers={"Content-Range": "bytes 4-9/10"}))
  big_model._download_model(manifest, tmp_path,
                            progress_callback=lambda _manifest, current, total: progress.append((current, total)),
                            phase_callback=lambda state, _manifest: phases.append(state))

  assert progress[0] == (4, 10)
  assert progress[-1] == (10, 10)
  assert phases == ["verifying"]


def test_status_is_atomic_and_preserves_stage_start(tmp_path: Path):
  first = write_big_model_status(tmp_path, "downloading", downloaded_bytes=10, total_bytes=100)
  second = write_big_model_status(tmp_path, "downloading", downloaded_bytes=20, total_bytes=100)
  assert second["started_at"] == first["started_at"]
  assert read_big_model_status(tmp_path)["downloaded_bytes"] == 20
  assert not list(tmp_path.glob(".status-*.json"))


def test_status_reporter_rate_limits_download_updates(monkeypatch, tmp_path: Path):
  manifest = BigModelManifest.from_dict(manifest_for(b"model"), "https://example.com/models/manifest.json")
  reporter = BigModelStatusReporter(tmp_path, min_interval=60.0)
  reporter.download_progress(manifest, 1, manifest.size)
  reporter.download_progress(manifest, 2, manifest.size)
  assert read_big_model_status(tmp_path)["downloaded_bytes"] == 1
  reporter.download_progress(manifest, manifest.size, manifest.size)
  assert read_big_model_status(tmp_path)["downloaded_bytes"] == manifest.size


def test_web_surface_is_hidden_without_egpu_history_and_restart_is_gated():
  source = (Path(big_model.__file__).parents[1] / "carrot/server/features/egpu_model.py").read_text(encoding="utf-8")
  assert 'return {"ok": True, "available": False}' in source
  assert '"UsbGpuHardwareSeen"' in source
  assert "if not hardware_seen:" in source
  assert "hardware_seen or manifest is not None" not in source
  assert 'payload.get("engaged")' in source
  assert 'if not usbgpu_present()' in source
  assert 'check_usbgpu(' not in source
  assert 'params.put_bool("DoReboot", True)' in source


def test_background_download_runs_at_low_cpu_and_io_priority():
  launcher = (Path(big_model.__file__).parents[3] / "launch_chffrplus.sh").read_text(encoding="utf-8")
  update = launcher[launcher.index("function start_big_model_update {"):]
  update = update[:update.index("\n}")]
  assert "ionice -c 3 nice -n 10 python3" in update
  assert "nice -n 10 python3" in update


def test_bad_hash_never_becomes_active(monkeypatch, tmp_path: Path):
  expected = b"good"
  manifest = BigModelManifest.from_dict(manifest_for(expected), "https://example.com/models/manifest.json")
  monkeypatch.setattr(big_model, "urlopen", lambda request, timeout: FakeResponse(b"baad"))
  with pytest.raises(OSError, match="sha256 mismatch"):
    big_model._download_model(manifest, tmp_path)
  assert not big_model.model_path(manifest, tmp_path).exists()
  assert not big_model.model_path(manifest, tmp_path).with_suffix(".onnx.part").exists()
