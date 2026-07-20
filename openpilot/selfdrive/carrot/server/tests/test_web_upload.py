import asyncio
import json
from pathlib import Path
from typing import cast

import pytest
from aiohttp import web

from openpilot.selfdrive.carrot import web_upload
from openpilot.selfdrive.carrot.server.features.dashcam import routes, upload
from openpilot.selfdrive.carrot.server.services import web_settings


def clear_upload_env(monkeypatch):
  for key in (
    "CARROT_WEB_UPLOAD_URL",
    "CARROT_WEB_UPLOAD_TOKEN",
    "CARROT_TOSS_UPLOAD_URL",
    "CARROT_TOSS_UPLOAD_TOKEN",
    "CARROT_TMUX_WEB_UPLOAD_URL",
  ):
    monkeypatch.delenv(key, raising=False)


def test_carrot_runtime_contains_no_legacy_ftp_code():
  carrot_root = Path(__file__).resolve().parents[2]
  legacy_terms = (
    "ft" + "plib",
    "carrot_" + "ftp",
    "ftp" + "://",
    "ftp" + "_ok",
    "upload_folder_to_" + "ftp",
  )
  findings = []
  for path in carrot_root.rglob("*"):
    if not path.is_file() or path.suffix.lower() not in {".py", ".js", ".sh"}:
      continue
    if "tests" in path.parts or "generated" in path.parts or "vendor" in path.parts:
      continue
    text = path.read_text(encoding="utf-8", errors="ignore").lower()
    if any(term in text for term in legacy_terms):
      findings.append(str(path.relative_to(carrot_root)))
  assert findings == []


def test_upload_target_defaults_to_carrot_and_keeps_credentials_isolated(monkeypatch):
  clear_upload_env(monkeypatch)
  settings = {
    "web_upload_url": "https://carrot.example/",
    "web_upload_token": "carrot-token",
    "toss_upload_url": "https://toss.example/",
    "toss_upload_token": "toss-token",
  }
  assert web_upload.selected_upload_settings(settings) == (
    "carrot", "https://carrot.example", "carrot-token",
  )
  assert web_upload.selected_upload_settings({**settings, "log_upload_target": "toss"}) == (
    "toss", "https://toss.example", "toss-token",
  )

  monkeypatch.setenv("CARROT_WEB_UPLOAD_URL", "https://carrot-env.example/root/")
  monkeypatch.setenv("CARROT_WEB_UPLOAD_TOKEN", "carrot-env-token")
  monkeypatch.setenv("CARROT_TOSS_UPLOAD_URL", "https://toss-env.example/root/")
  monkeypatch.setenv("CARROT_TOSS_UPLOAD_TOKEN", "toss-env-token")
  assert web_upload.selected_upload_settings(settings) == (
    "carrot", "https://carrot-env.example/root", "carrot-env-token",
  )
  assert web_upload.selected_upload_settings({**settings, "log_upload_target": "toss"}) == (
    "toss", "https://toss-env.example/root", "toss-env-token",
  )


def test_web_api_url_quotes_every_path_component():
  assert web_upload.api_url(
    "https://upload.example/",
    "upload",
    "car name/id",
    "route|0",
    "qlog.zst",
  ) == "https://upload.example/api/v1/upload/car%20name%2Fid/route%7C0/qlog.zst"


def test_tmux_target_uses_authenticated_web_api_when_token_exists(monkeypatch):
  clear_upload_env(monkeypatch)
  url, headers = web_upload.tmux_web_target({
    "web_upload_url": "https://upload.example",
    "web_upload_token": "secret",
  })
  assert url == "https://upload.example/api/v1/tmux/upload"
  assert headers == {"Authorization": "Bearer secret"}


def test_toss_tmux_uses_selected_server_and_token(monkeypatch):
  clear_upload_env(monkeypatch)
  url, headers = web_upload.tmux_web_target({
    "log_upload_target": "toss",
    "web_upload_url": "https://carrot.example",
    "web_upload_token": "carrot-token",
    "toss_upload_url": "https://toss.example",
    "toss_upload_token": "toss-token",
  })
  assert url == "https://toss.example/api/v1/tmux/upload"
  assert headers == {"Authorization": "Bearer toss-token"}


def test_tmux_target_falls_back_to_direct_web_endpoint_without_token(monkeypatch):
  clear_upload_env(monkeypatch)
  monkeypatch.setenv("CARROT_TMUX_WEB_UPLOAD_URL", "https://tmux.example/upload/")
  assert web_upload.tmux_web_target({}) == ("https://tmux.example/upload", {})


def test_toss_tmux_never_falls_back_to_carrot_endpoint_without_token(monkeypatch):
  clear_upload_env(monkeypatch)
  monkeypatch.setenv("CARROT_TMUX_WEB_UPLOAD_URL", "https://tmux.example/upload/")
  with pytest.raises(ValueError, match="Toss upload token"):
    web_upload.tmux_web_target({
      "log_upload_target": "toss",
      "toss_upload_url": "https://toss.example",
      "toss_upload_token": "",
    })


def test_tmux_web_post_sends_multipart_and_closes_files(tmp_path: Path):
  tmux_path = tmp_path / "tmux.log"
  settings_path = tmp_path / "toggle_values.json"
  tmux_path.write_bytes(b"tmux-data")
  settings_path.write_bytes(b'{"enabled": true}')
  captured = {}

  def fake_post(url, *, headers, data, files, timeout):
    captured.update({"url": url, "headers": headers, "data": data, "files": files, "timeout": timeout})
    captured["contents"] = [item[1][1].read() for item in files]
    return "response"

  response = web_upload.post_tmux_web(
    "https://upload.example/api/v1/tmux/upload",
    {"Authorization": "Bearer token"},
    {"tmux_why": "exception"},
    str(tmux_path),
    str(settings_path),
    fake_post,
  )

  assert response == "response"
  assert captured["headers"] == {"Authorization": "Bearer token"}
  assert captured["data"] == {"tmux_why": "exception"}
  assert [item[0] for item in captured["files"]] == ["files[0]", "files[1]"]
  assert captured["contents"] == [b"tmux-data", b'{"enabled": true}']
  assert captured["timeout"] == 30
  assert all(item[1][1].closed for item in captured["files"])


def test_web_settings_preserve_default_carrot_and_separate_toss_credentials():
  defaults = web_settings.sanitize_web_settings({})
  assert defaults["log_upload_target"] == "carrot"

  settings = web_settings.sanitize_web_settings({
    "log_upload_target": "toss",
    "web_upload_url": "https://carrot.example/",
    "web_upload_token": "carrot-token",
    "toss_upload_url": "https://toss.example/",
    "toss_upload_token": "toss-token",
  })
  assert settings["log_upload_target"] == "toss"
  assert settings["web_upload_url"] == "https://carrot.example"
  assert settings["web_upload_token"] == "carrot-token"
  assert settings["toss_upload_url"] == "https://toss.example"
  assert settings["toss_upload_token"] == "toss-token"


def test_dashcam_target_resolution_and_test_route_are_unique(monkeypatch):
  monkeypatch.setattr(upload, "read_web_settings", lambda: {
    "log_upload_target": "toss",
    "web_upload_url": "https://carrot.example",
    "web_upload_token": "carrot-token",
    "toss_upload_url": "https://toss.example",
    "toss_upload_token": "toss-token",
  })
  assert upload.resolve_upload_target() == {
    "kind": "toss",
    "base_url": "https://toss.example",
    "token": "toss-token",
  }

  app = web.Application()
  routes.register(app)
  matching = [
    route for route in app.router.routes()
    if route.method == "POST" and getattr(route.resource, "canonical", "") == "/api/dashcam/upload/test"
  ]
  assert len(matching) == 1

  async def fake_health(base_url, token):
    assert (base_url, token) == ("https://toss.example", "toss-token")
    return {"ok": True, "status": 200}

  monkeypatch.setattr(routes, "check_web_upload_health", fake_health)
  response = asyncio.run(routes.api_dashcam_upload_test(cast(web.Request, None)))
  assert json.loads(response.text or "") == {
    "target": "toss",
    "url": "https://toss.example",
    "ok": True,
    "status": 200,
  }


class FakeResponse:
  def __init__(self, status: int, payload: dict):
    self.status = status
    self._payload = payload

  async def text(self):
    return json.dumps(self._payload)


class FakeRequestContext:
  def __init__(self, session, url, data):
    self.session = session
    self.url = url
    self.data = data

  async def __aenter__(self):
    content = bytearray()
    async for chunk in self.data:
      content.extend(chunk)
    self.session.requests.append((self.url, bytes(content)))
    size_delta = self.session.size_deltas.pop(0) if self.session.size_deltas else 0
    return FakeResponse(200, {"ok": True, "size": len(content) + size_delta})

  async def __aexit__(self, exc_type, exc, tb):
    return False


class FakeSession:
  instances = []
  size_deltas = []

  def __init__(self, *args, headers=None, **kwargs):
    self.headers = headers or {}
    self.requests = []
    self.size_deltas = list(type(self).size_deltas)
    type(self).instances.append(self)

  async def __aenter__(self):
    return self

  async def __aexit__(self, exc_type, exc, tb):
    return False

  def put(self, url, data):
    return FakeRequestContext(self, url, data)


def test_dashcam_web_upload_streams_and_verifies_every_file(tmp_path: Path, monkeypatch):
  (tmp_path / "fcamera.hevc").write_bytes(b"camera-data")
  (tmp_path / "qlog.zst").write_bytes(b"log-data")
  FakeSession.instances = []
  FakeSession.size_deltas = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "car name dongle/id",
    "2026-07-20--00-00-00|0",
    "https://upload.example",
    "token",
  ))

  session = FakeSession.instances[-1]
  assert session.headers["Authorization"] == "Bearer token"
  assert [request[0] for request in session.requests] == [
    "https://upload.example/api/v1/upload/car%20name%20dongle%2Fid/2026-07-20--00-00-00%7C0/fcamera.hevc",
    "https://upload.example/api/v1/upload/car%20name%20dongle%2Fid/2026-07-20--00-00-00%7C0/qlog.zst",
  ]
  assert [request[1] for request in session.requests] == [b"camera-data", b"log-data"]


def test_dashcam_web_upload_retries_size_mismatch(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"retry-me")
  FakeSession.instances = []
  FakeSession.size_deltas = [1, 0]
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "device",
    "route|0",
    "https://upload.example",
    "token",
  ))
  assert len(FakeSession.instances[-1].requests) == 2


def test_dashcam_web_upload_requires_token_before_network(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"data")
  FakeSession.instances = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  with pytest.raises(RuntimeError, match="token is not configured"):
    asyncio.run(web_upload.upload_folder_to_web(
      str(tmp_path),
      "device",
      "route|0",
      "https://upload.example",
      "",
    ))
  assert FakeSession.instances == []


def test_dashcam_web_upload_honors_cancellation_before_network(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"data")
  FakeSession.instances = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  with pytest.raises(RuntimeError, match="upload canceled"):
    asyncio.run(web_upload.upload_folder_to_web(
      str(tmp_path),
      "device",
      "route|0",
      "https://upload.example",
      "token",
      lambda: True,
    ))
  assert FakeSession.instances == []
