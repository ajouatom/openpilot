import asyncio
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot import web_upload
from openpilot.selfdrive.carrot.server.services import web_settings


def clear_upload_env(monkeypatch):
  for key in ("CARROT_WEB_UPLOAD_URL", "CARROT_WEB_UPLOAD_TOKEN", "CARROT_TMUX_WEB_UPLOAD_URL"):
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


def test_carrot_man_sends_diagnostics_to_dsm_and_carrot_logs():
  carrot_man = (Path(__file__).resolve().parents[2] / "carrot_man.py").read_text(encoding="utf-8")
  assert "def send_tmux_web(" in carrot_man
  assert "def send_tmux_carrot_logs(" in carrot_man
  assert 'self.send_tmux_carrot_logs("onroad", send_settings = True)' in carrot_man
  assert "self.send_tmux_carrot_logs(pending_tmux_reason, send_settings = False)" in carrot_man
  assert 'self.send_tmux_carrot_logs("tmux_send")' in carrot_man
  assert "using tmux web fallback" not in carrot_man


def test_web_upload_settings_support_legacy_values_and_environment_override(monkeypatch):
  clear_upload_env(monkeypatch)
  assert web_upload.web_upload_settings({
    "toss_upload_url": "https://legacy.example/",
    "toss_upload_token": "legacy-token",
  }) == ("https://legacy.example", "")

  monkeypatch.setenv("CARROT_WEB_UPLOAD_URL", "https://env.example/root/")
  monkeypatch.setenv("CARROT_WEB_UPLOAD_TOKEN", "env-token")
  assert web_upload.web_upload_settings({
    "web_upload_url": "https://setting.example",
    "web_upload_token": "setting-token",
  }) == ("https://env.example/root", "env-token")


def test_web_api_url_quotes_every_path_component():
  assert web_upload.api_url(
    "https://upload.example/",
    "upload",
    "car name/id",
    "route|0",
    "qlog.zst",
  ) == "https://upload.example/api/v1/upload/car%20name%2Fid/route%7C0/qlog.zst"


def test_tmux_target_uses_automatic_session_token(monkeypatch):
  clear_upload_env(monkeypatch)
  url, headers = web_upload.tmux_web_target({
    "web_upload_url": "https://upload.example",
  }, "automatic-session")
  assert url == "https://upload.example/api/v1/tmux/upload"
  assert headers == {"Authorization": "Bearer automatic-session"}


def test_tmux_target_falls_back_to_direct_web_endpoint_without_token(monkeypatch):
  clear_upload_env(monkeypatch)
  monkeypatch.setenv("CARROT_TMUX_WEB_UPLOAD_URL", "https://tmux.example/upload/")
  assert web_upload.tmux_web_target({}) == ("https://tmux.example/upload", {})


def test_carrot_logs_target_is_independent_from_dsm_token(monkeypatch):
  clear_upload_env(monkeypatch)
  monkeypatch.setenv("CARROT_WEB_UPLOAD_TOKEN", "dsm-token")
  monkeypatch.setenv("CARROT_TMUX_WEB_UPLOAD_URL", "https://tmux.example/upload/")
  assert web_upload.carrot_logs_web_target() == ("https://tmux.example/upload", {})


def test_sync_session_is_issued_automatically_from_device_metadata():
  captured = {}

  class Response:
    status_code = 200
    text = '{"ok":true}'

    @staticmethod
    def json():
      return {"ok": True, "token": "short-lived-session"}

  def fake_post(url, *, json, timeout):
    captured.update({"url": url, "json": json, "timeout": timeout})
    return Response()

  token = web_upload.create_web_upload_session_sync(
    "https://upload.example",
    {"dongle_id": "0123456789abcdef", "car_name": "TEST"},
    fake_post,
  )
  assert token == "short-lived-session"
  assert captured == {
    "url": "https://upload.example/api/v1/session",
    "json": {
      "dongle_id": "0123456789abcdef",
      "car_name": "TEST",
      "deviceId": "0123456789abcdef",
      "purpose": "tmux",
    },
    "timeout": 12,
  }


def test_async_session_is_issued_automatically(monkeypatch):
  captured = {}

  class Response:
    status = 200

    async def text(self):
      return '{"ok":true,"token":"dashcam-session"}'

  class Context:
    async def __aenter__(self):
      return Response()

    async def __aexit__(self, exc_type, exc, tb):
      return False

  class Session:
    def __init__(self, *args, **kwargs):
      pass

    async def __aenter__(self):
      return self

    async def __aexit__(self, exc_type, exc, tb):
      return False

    def post(self, url, *, json):
      captured.update({"url": url, "json": json})
      return Context()

  monkeypatch.setattr(web_upload, "ClientSession", Session)
  token = asyncio.run(web_upload.create_web_upload_session(
    "https://upload.example", {"dongleId": "0123456789abcdef"}, "dashcam",
  ))
  assert token == "dashcam-session"
  assert captured["json"]["deviceId"] == "0123456789abcdef"
  assert captured["json"]["purpose"] == "dashcam"


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


def test_web_settings_migrate_previous_upload_keys():
  settings = web_settings.sanitize_web_settings({
    "toss_upload_url": "https://legacy.example/",
    "toss_upload_token": "legacy-token",
  })
  assert settings["web_upload_url"] == "https://legacy.example"
  assert "web_upload_token" not in settings
  assert "toss_upload_url" not in settings
  assert "toss_upload_token" not in settings


@pytest.mark.parametrize("previous_url", [
  "https://op.wjcloud.kr",
  "https://shind0.synology.me",
  "https://SHIND0.synology.me",
])
def test_web_settings_migrate_previous_default_server(previous_url):
  settings = web_settings.sanitize_web_settings({"web_upload_url": previous_url})
  assert settings["web_upload_url"] == web_upload.DEFAULT_WEB_UPLOAD_URL


class FakeResponse:
  def __init__(self, status: int, payload: dict):
    self.status = status
    self._payload = payload

  async def text(self):
    return json.dumps(self._payload)


class FakeRequestContext:
  def __init__(self, session, url, data, headers):
    self.session = session
    self.url = url
    self.data = data
    self.headers = headers

  async def __aenter__(self):
    content = bytearray()
    async for chunk in self.data:
      content.extend(chunk)
    self.session.requests.append((self.url, bytes(content), self.headers))
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

  def put(self, url, data, headers=None):
    return FakeRequestContext(self, url, data, headers or {})


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
  assert [request[2]["X-File-Size"] for request in session.requests] == ["11", "8"]


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


def test_dashcam_web_upload_requires_session_before_network(tmp_path: Path, monkeypatch):
  (tmp_path / "qlog.zst").write_bytes(b"data")
  FakeSession.instances = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  with pytest.raises(RuntimeError, match="session is not configured"):
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
