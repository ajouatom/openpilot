import json

from openpilot.selfdrive.carrot.recovery import server as recovery


class _Response:
  status = 204

  @staticmethod
  def read(_limit=-1):
    return b""

  def __enter__(self):
    return self

  def __exit__(self, _exc_type, _exc, _tb):
    return False


def test_recovery_tmux_discord_upload_attaches_log(monkeypatch, tmp_path):
  tmux_log = tmp_path / "tmux.log"
  tmux_log.write_bytes(b"recovery tmux output")
  request_capture = {}

  monkeypatch.setattr(recovery, "TMUX_LOG_PATH", str(tmux_log))
  monkeypatch.setattr(recovery, "_exception_webhook_url", lambda: "https://discord.example/webhook")
  monkeypatch.setattr(recovery, "_read_param", lambda key, default="": {
    "GitBranch": "jominki354/recovery",
    "GitCommit": "0123456789abcdef",
  }.get(key, default))

  def fake_urlopen(request, timeout):
    request_capture["request"] = request
    request_capture["timeout"] = timeout
    return _Response()

  monkeypatch.setattr(recovery.urllib.request, "urlopen", fake_urlopen)

  result = recovery._send_tmux_discord("tmux_send")

  assert result == {"configured": True, "ok": True, "status": 204}
  assert request_capture["timeout"] == 12
  request = request_capture["request"]
  assert request.full_url == "https://discord.example/webhook"
  assert request.get_header("User-agent") == "CarrotRecovery/2.0"
  assert b'name="files[0]"' in request.data
  assert b"recovery tmux output" in request.data
  assert b"tmux_send-" in request.data
  payload_start = request.data.index(b'{"username"')
  payload_end = request.data.index(b"\r\n", payload_start)
  payload = json.loads(request.data[payload_start:payload_end])
  assert payload["username"] == "Carrot Exception"
  assert payload["flags"] == 4


def test_server_tmux_log_reports_actual_discord_result(monkeypatch):
  calls = []
  monkeypatch.setattr(recovery, "_capture_tmux_log", lambda: (0, ""))
  monkeypatch.setattr(recovery, "_send_tmux_destinations", lambda reason: calls.append(reason) or {
    "ok": False,
    "partial": False,
    "destinations": {},
    "error": "all destinations failed",
  })

  result = recovery._tool_action("server_tmux_log", {})

  assert calls == ["tmux_send"]
  assert result == {
    "ok": False,
    "partial": False,
    "destinations": {},
    "error": "all destinations failed",
    "file": "/download/tmux.log",
  }


def test_recovery_tmux_sends_to_all_main_server_destinations(monkeypatch, tmp_path):
  tmux_log = tmp_path / "tmux.log"
  tmux_log.write_bytes(b"tmux-data")
  calls = []
  payload = {"tmux_why": "tmux_send", "dongle_id": "device-id"}

  monkeypatch.setattr(recovery, "TMUX_LOG_PATH", str(tmux_log))
  monkeypatch.setattr(recovery, "_tmux_upload_payload", lambda reason: payload)
  monkeypatch.setattr(recovery, "_send_tmux_dsm", lambda sent_payload, raw: calls.append(("dsm", sent_payload, raw)) or {"ok": True, "status": 200})
  monkeypatch.setattr(
    recovery,
    "_send_tmux_carrot_logs",
    lambda sent_payload, raw: calls.append(("carrot_logs", sent_payload, raw)) or {"ok": True, "status": 200},
  )
  monkeypatch.setattr(recovery, "_send_tmux_discord", lambda reason, raw, web: calls.append(("discord", reason, raw, web)) or {"ok": True, "status": 204})

  result = recovery._send_tmux_destinations("tmux_send")

  assert result["ok"] is True
  assert result["partial"] is False
  assert list(result["destinations"]) == ["dsm", "carrot_logs", "discord"]
  assert calls == [
    ("dsm", payload, b"tmux-data"),
    ("carrot_logs", payload, b"tmux-data"),
    ("discord", "tmux_send", b"tmux-data", {"ok": True, "status": 200}),
  ]


def test_recovery_dsm_upload_uses_automatic_session(monkeypatch):
  calls = []
  payload = {"tmux_why": "tmux_send", "dongle_id": "device-id", "car_name": "TEST"}

  monkeypatch.delenv("CARROT_WEB_UPLOAD_TOKEN", raising=False)
  monkeypatch.setattr(recovery, "_web_upload_base_url", lambda: "https://upload.example")
  monkeypatch.setattr(recovery, "_post_json", lambda url, body: calls.append(("session", url, body)) or {
    "ok": True,
    "status": 200,
    "body": {"ok": True, "token": "session-token"},
  })
  monkeypatch.setattr(recovery, "_post_tmux_upload", lambda url, headers, body, raw: calls.append(("upload", url, headers, body, raw)) or {
    "ok": True,
    "status": 200,
  })

  result = recovery._send_tmux_dsm(payload, b"tmux-data")

  assert result == {"ok": True, "status": 200}
  assert calls[0] == ("session", "https://upload.example/api/v1/session", {
    "tmux_why": "tmux_send",
    "dongle_id": "device-id",
    "car_name": "TEST",
    "deviceId": "device-id",
    "purpose": "tmux",
  })
  assert calls[1] == (
    "upload",
    "https://upload.example/api/v1/tmux/upload",
    {"Authorization": "Bearer session-token"},
    payload,
    b"tmux-data",
  )
