import hashlib
import importlib.util
from pathlib import Path
import sys
import types

import pytest

try:
  import requests
except ModuleNotFoundError:
  requests = types.ModuleType("requests")

  class RequestException(Exception):
    pass

  requests.ConnectionError = RequestException
  requests.exceptions = types.SimpleNamespace(RequestException=RequestException)
  sys.modules["requests"] = requests

from openpilot.common.basedir import BASEDIR


def _load_agnos_module():
  path = Path(BASEDIR) / "openpilot/system/hardware/tici/agnos.py"
  spec = importlib.util.spec_from_file_location("agnos_download_cache_test", path)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


class FakeLog:
  def info(self, _message: str) -> None:
    pass

  def warning(self, _message: str) -> None:
    pass


class FakeResponse:
  def __init__(self, data: bytes, status_code: int, *, fail_after: bool = False):
    self.data = data
    self.status_code = status_code
    self.headers = {"Content-Length": str(len(data))}
    self.fail_after = fail_after
    self.closed = False

  def raise_for_status(self) -> None:
    pass

  def iter_content(self, chunk_size: int):
    assert chunk_size > 0
    yield self.data
    if self.fail_after:
      raise requests.ConnectionError("connection dropped")

  def close(self) -> None:
    self.closed = True


def _partition(data: bytes) -> dict:
  return {
    "name": "system",
    "url": "https://example.test/system.img.xz",
    "compressed_hash": hashlib.sha256(data).hexdigest(),
    "compressed_size": len(data),
  }


def _partial_path(module, partition: dict) -> Path:
  final = module.DOWNLOAD_CACHE_DIR / f"system-{partition['compressed_hash']}.img.xz"
  return final.with_suffix(final.suffix + ".part")


def test_cached_download_resumes_with_http_range(monkeypatch, tmp_path: Path) -> None:
  module = _load_agnos_module()
  data = b"0123456789abcdef"
  partition = _partition(data)
  monkeypatch.setattr(module, "DOWNLOAD_CACHE_DIR", tmp_path)
  partial = _partial_path(module, partition)
  partial.write_bytes(data[:6])
  response = FakeResponse(data[6:], 206)
  requested_headers = {}

  def fake_get(_url, **kwargs):
    requested_headers.update(kwargs["headers"])
    return response

  monkeypatch.setattr(module.requests, "get", fake_get, raising=False)
  result = module.download_to_cache(partition, FakeLog())

  assert requested_headers["Range"] == "bytes=6-"
  assert result.read_bytes() == data
  assert not partial.exists()
  assert response.closed


def test_cached_download_restarts_when_server_ignores_range(monkeypatch, tmp_path: Path) -> None:
  module = _load_agnos_module()
  data = b"complete download"
  partition = _partition(data)
  monkeypatch.setattr(module, "DOWNLOAD_CACHE_DIR", tmp_path)
  partial = _partial_path(module, partition)
  partial.write_bytes(b"stale prefix")
  response = FakeResponse(data, 200)
  monkeypatch.setattr(module.requests, "get", lambda *_args, **_kwargs: response, raising=False)

  result = module.download_to_cache(partition, FakeLog())
  assert result.read_bytes() == data


def test_cached_download_keeps_partial_file_after_disconnect(monkeypatch, tmp_path: Path) -> None:
  module = _load_agnos_module()
  data = b"eventual complete download"
  partition = _partition(data)
  monkeypatch.setattr(module, "DOWNLOAD_CACHE_DIR", tmp_path)
  response = FakeResponse(data[:8], 200, fail_after=True)
  monkeypatch.setattr(module.requests, "get", lambda *_args, **_kwargs: response, raising=False)

  with pytest.raises(requests.ConnectionError):
    module.download_to_cache(partition, FakeLog())

  assert _partial_path(module, partition).read_bytes() == data[:8]
  assert response.closed


def test_complete_partial_file_is_promoted_without_network(monkeypatch, tmp_path: Path) -> None:
  module = _load_agnos_module()
  data = b"already complete"
  partition = _partition(data)
  monkeypatch.setattr(module, "DOWNLOAD_CACHE_DIR", tmp_path)
  partial = _partial_path(module, partition)
  partial.write_bytes(data)
  monkeypatch.setattr(module.requests, "get", lambda *_args, **_kwargs: pytest.fail("unexpected network request"), raising=False)

  result = module.download_to_cache(partition, FakeLog())
  assert result.read_bytes() == data
  assert not partial.exists()
