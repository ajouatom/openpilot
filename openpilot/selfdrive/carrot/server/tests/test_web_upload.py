import asyncio
import json
from pathlib import Path

import pytest
from aiohttp import web

from openpilot.selfdrive.carrot import web_upload
from openpilot.selfdrive.carrot.server.features.dashcam import catalog
from openpilot.selfdrive.carrot.server.features.dashcam import upload
from openpilot.selfdrive.carrot.server.features.dashcam import upload_jobs
from openpilot.selfdrive.carrot.server.services import dashcam_upload_report
from openpilot.selfdrive.carrot.server.services import web_settings


def clear_upload_env(monkeypatch):
  for key in ("CARROT_WEB_UPLOAD_URL", "CARROT_WEB_UPLOAD_TOKEN", "CARROT_TMUX_WEB_UPLOAD_URL"):
    monkeypatch.delenv(key, raising=False)


class FakeUploadTask:
  def __init__(self, *, done=False):
    self._done = done
    self.cancel_called = False

  def done(self):
    return self._done

  def cancel(self):
    self.cancel_called = True


def test_dashcam_stale_upload_job_is_failed_and_released():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0"])
  task = FakeUploadTask()
  job["_task"] = task
  job["_activity_at"] = 100.0

  upload_jobs.expire_stale_jobs(now=100.0 + upload_jobs.UPLOAD_JOB_STALE_SECONDS)

  assert task.cancel_called is True
  assert job["status"] == "failed"
  assert job["error"] == "upload job expired after 30 minutes without activity"
  assert upload_jobs.running_job() is None
  upload_jobs.jobs().clear()


def test_dashcam_finished_task_cannot_leave_running_job():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0"])
  job["_task"] = FakeUploadTask(done=True)

  upload_jobs.expire_stale_jobs(now=job["_activity_at"])

  assert job["status"] == "failed"
  assert job["error"] == "upload task ended without a final state"
  upload_jobs.jobs().clear()


def test_dashcam_upload_job_exposes_stable_phase_codes():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0"])

  assert upload_jobs.snapshot(job)["phase"] == "queued"

  upload_jobs.progress(job, current=1, total=1, phase=upload_jobs.UPLOAD_PHASE_UPLOADING)
  assert upload_jobs.snapshot(job)["phase"] == "uploading"

  upload_jobs.finish(job, ok=True, result={"ok": True})
  snapshot = upload_jobs.snapshot(job)
  assert snapshot["status"] == "done"
  assert snapshot["phase"] == "complete"
  upload_jobs.jobs().clear()


def test_dashcam_upload_progress_is_monotonic_and_revisioned():
  upload_jobs.jobs().clear()
  job = upload_jobs.create_job(["route--0", "route--1"])
  assert upload_jobs.snapshot(job)["revision"] == 0

  upload_jobs.progress(
    job,
    phase=upload_jobs.UPLOAD_PHASE_PREPARING,
    current=1,
    total=2,
    phase_current=1,
    phase_total=2,
    percent=4,
  )
  preparing = upload_jobs.snapshot(job)
  assert preparing["progress"] == 4
  assert preparing["phase_current"] == 1
  assert preparing["phase_total"] == 2
  assert preparing["revision"] == 1

  upload_jobs.progress(
    job,
    phase=upload_jobs.UPLOAD_PHASE_UPLOADING,
    percent=2,
    bytes_current=128,
    bytes_total=1024,
    bytes_per_second=512,
  )
  uploading = upload_jobs.snapshot(job)
  assert uploading["progress"] == 4
  assert uploading["bytes_current"] == 128
  assert uploading["bytes_total"] == 1024
  assert uploading["bytes_per_second"] == 512
  assert uploading["revision"] == 2

  upload_jobs.finish(job, ok=True, result={"ok": True})
  complete = upload_jobs.snapshot(job)
  assert complete["progress"] == 100
  assert complete["revision"] == 3
  upload_jobs.jobs().clear()


def test_dashcam_upload_cancel_and_failure_keep_visible_progress():
  upload_jobs.jobs().clear()

  canceled_job = upload_jobs.create_job(["route--0"])
  upload_jobs.progress(
    canceled_job,
    phase=upload_jobs.UPLOAD_PHASE_UPLOADING,
    percent=41,
  )
  canceled = upload_jobs.cancel_job(canceled_job["id"])
  assert canceled["phase"] == "canceling"
  assert canceled["progress"] == 41
  upload_jobs.finish(
    canceled_job,
    ok=False,
    status="canceled",
    result={"ok": False, "canceled": True},
  )
  canceled = upload_jobs.snapshot(canceled_job)
  assert canceled["phase"] == "canceled"
  assert canceled["progress"] == 41

  failed_job = upload_jobs.create_job(["route--1"])
  upload_jobs.progress(
    failed_job,
    phase=upload_jobs.UPLOAD_PHASE_UPLOADING,
    percent=62,
  )
  upload_jobs.finish(
    failed_job,
    ok=False,
    error="network failed",
    result={"ok": False, "error": "network failed"},
  )
  failed = upload_jobs.snapshot(failed_job)
  assert failed["phase"] == "failed"
  assert failed["progress"] == 62
  upload_jobs.jobs().clear()


def test_dashcam_start_job_finalizes_unhandled_task_failure(monkeypatch):
  upload_jobs.jobs().clear()

  async def scenario():
    async def crash(_job):
      raise RuntimeError("unexpected task failure")

    monkeypatch.setattr(upload_jobs, "run_job", crash)
    job = upload_jobs.create_job(["route--0"])
    task = upload_jobs.start_job(job)
    with pytest.raises(RuntimeError, match="unexpected task failure"):
      await task
    await asyncio.sleep(0)
    return job

  job = asyncio.run(scenario())
  assert job["status"] == "failed"
  assert job["error"] == "unexpected task failure"
  upload_jobs.jobs().clear()


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


def test_dashcam_upload_report_links_public_segment_and_quotes_storage_directory():
  payload = {
    "uploadedAt": "2026-07-23 11:17:18",
    "remoteBasePath": "https://upload.example/routes/HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3/",
    "meta": {
      "carName": "HYUNDAI_IONIQ_5_PE",
      "dongleId": "8b06424f3adf2bd3",
      "commit": "79a2a542",
    },
    "results": [{
      "segment": "00000cfb--69588de3d7--10",
      "route": "00000cfb--69588de3d7",
      "segmentIndex": 10,
      "ok": True,
      "remotePath": "https://upload.example/routes/HYUNDAI_IONIQ_5_PE 8b06424f3adf2bd3/00000cfb--69588de3d7--10",
    }],
  }

  report = dashcam_upload_report.upload_share_text(payload)
  assert "HYUNDAI_IONIQ_5_PE%208b06424f3adf2bd3" in report
  assert "[00000cfb--69588de3d7--10 OK · Open](https://upload.example/routes/" in report
  assert "### Open & Analyze" not in report


def test_dashcam_upload_report_adds_one_slice_link_for_consecutive_segments():
  base = "https://upload.example/routes/TEST CAR 0123456789abcdef"
  results = [
    {
      "segment": f"00000cfb--69588de3d7--{index}",
      "route": "00000cfb--69588de3d7",
      "segmentIndex": index,
      "ok": True,
      "remotePath": f"{base}/00000cfb--69588de3d7--{index}",
    }
    for index in (10, 11, 12)
  ]

  report = dashcam_upload_report.upload_share_text({"remoteBasePath": f"{base}/", "results": results})
  assert "### Open & Analyze" in report
  assert "Segments 10–12 (3 logs) · Web/Video/Tools" in report
  assert "https://upload.example/routes/TEST%20CAR%200123456789abcdef/00000cfb--69588de3d7--10:13" in report
  assert report.count(" OK · Open]") == 3


def test_dashcam_upload_report_does_not_merge_nonconsecutive_segments():
  base = "https://upload.example/routes/TEST CAR 0123456789abcdef"
  results = [
    {
      "segment": f"00000cfb--69588de3d7--{index}",
      "route": "00000cfb--69588de3d7",
      "segmentIndex": index,
      "ok": True,
      "remotePath": f"{base}/00000cfb--69588de3d7--{index}",
    }
    for index in (10, 12)
  ]

  report = dashcam_upload_report.upload_share_text({"remoteBasePath": f"{base}/", "results": results})
  assert "### Open & Analyze" not in report


def test_dashcam_upload_completion_notifies_web_server_and_discord(monkeypatch):
  segment = "00000cfb--69588de3d7--10"
  notifications = []
  uploaded_files = []
  progress_snapshots = []

  async def fake_upload_folder(*args, **kwargs):
    uploaded_files.extend(kwargs["filenames"])
    on_progress = kwargs.get("on_progress")
    if on_progress:
      on_progress("qcamera.ts", 12, 12, 12)
      on_progress("rlog.zst", 34, 34, 34)
    return True

  async def fake_web_complete(base_url, token, payload):
    notifications.append(("web", base_url, token, payload["results"][0]["segment"]))
    return {"ok": True, "status": 200}

  async def fake_discord(webhook_url, payload):
    notifications.append(("discord", webhook_url, payload["shareText"]))
    return {"configured": True, "ok": True, "status": 204}

  monkeypatch.setattr(upload_jobs, "HAS_PARAMS", False)
  monkeypatch.setattr(upload, "upload_target_settings", lambda: ("https://upload.example", "session-token"))
  monkeypatch.setattr(upload, "upload_metadata", lambda params: {
    "carName": "TEST_CAR",
    "dongleId": "0123456789abcdef",
  })
  monkeypatch.setattr(upload, "upload_share_text", lambda payload: "shared upload report")
  monkeypatch.setattr(upload, "discord_webhook_url", lambda params: "https://discord.example/webhook")
  monkeypatch.setattr(upload, "send_discord_webhook", fake_discord)
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda value: "/tmp/segment")
  monkeypatch.setattr(upload_jobs, "segment_file_summary", lambda value: [
    {"kind": "qcamera", "name": "qcamera.ts", "size": 12},
    {"kind": "rlog", "name": "rlog.zst", "size": 34},
  ])
  monkeypatch.setattr(upload_jobs, "upload_folder_to_web", fake_upload_folder)
  monkeypatch.setattr(upload_jobs, "send_web_upload_complete", fake_web_complete)
  real_progress = upload_jobs.progress

  def capture_progress(job, **kwargs):
    real_progress(job, **kwargs)
    progress_snapshots.append(upload_jobs.snapshot(job))

  monkeypatch.setattr(upload_jobs, "progress", capture_progress)

  upload_jobs.jobs().clear()
  job = upload_jobs.create_job([segment])
  result = asyncio.run(upload_jobs.run_upload_segments([segment], job))

  assert result["ok"] is True
  assert result["webComplete"] == {"ok": True, "status": 200}
  assert result["discord"] == {"configured": True, "ok": True, "status": 204}
  assert notifications == [
    ("web", "https://upload.example", "session-token", segment),
    ("discord", "https://discord.example/webhook", "shared upload report"),
  ]
  assert uploaded_files == ["qcamera.ts", "rlog.zst"]
  snapshot = upload_jobs.snapshot(job)
  assert snapshot["bytes_current"] == 46
  assert snapshot["bytes_total"] == 46
  assert snapshot["bytes_per_second"] >= 0
  assert snapshot["step_current"] == 1
  assert snapshot["step_total"] == 1
  assert snapshot["progress"] == 99
  assert snapshot["phase"] == "notifying"
  assert snapshot["phase_current"] == 2
  assert snapshot["phase_total"] == 2
  assert [item["progress"] for item in progress_snapshots] == sorted(
    item["progress"] for item in progress_snapshots
  )
  assert {"preparing", "uploading", "notifying"} <= {
    item["phase"] for item in progress_snapshots
  }
  assert any(
    item["phase"] == "preparing" and 0 < item["progress"] <= upload_jobs.UPLOAD_PREPARING_END_PERCENT
    for item in progress_snapshots
  )
  upload_jobs.jobs().clear()


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


def test_dashcam_web_upload_reports_chunk_progress(tmp_path: Path, monkeypatch):
  (tmp_path / "qcamera.ts").write_bytes(b"camera-data")
  FakeSession.instances = []
  FakeSession.size_deltas = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)
  updates = []

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "device",
    "route|0",
    "https://upload.example",
    "token",
    on_progress=lambda *args: updates.append(args),
  ))

  assert updates == [
    ("qcamera.ts", 0, 11, 0),
    ("qcamera.ts", 11, 11, 11),
  ]


def test_dashcam_upload_summary_selects_only_original_qcamera_and_rlog(tmp_path: Path):
  (tmp_path / "qcamera.ts").write_bytes(b"original-video")
  (tmp_path / "qcamera.mp4").write_bytes(b"converted-video")
  (tmp_path / "rlog.zst").write_bytes(b"original-rlog")
  (tmp_path / "rlog.bz2").write_bytes(b"fallback-rlog")
  (tmp_path / "qlog.zst").write_bytes(b"reduced-log")
  (tmp_path / "fcamera.hevc").write_bytes(b"auxiliary-video")

  files = catalog.segment_file_summary(str(tmp_path))

  assert [(item["kind"], item["name"], item["size"]) for item in files] == [
    ("qcamera", "qcamera.ts", len(b"original-video")),
    ("rlog", "rlog.zst", len(b"original-rlog")),
  ]


def test_dashcam_upload_summary_allows_rlog_without_qcamera(tmp_path: Path):
  (tmp_path / "rlog.bz2").write_bytes(b"original-rlog")

  files = catalog.segment_file_summary(str(tmp_path))

  assert [(item["kind"], item["name"]) for item in files] == [("rlog", "rlog.bz2")]


def test_dashcam_upload_summary_requires_rlog(tmp_path: Path):
  (tmp_path / "qcamera.ts").write_bytes(b"original-video")

  with pytest.raises(web.HTTPNotFound) as exc_info:
    catalog.segment_file_summary(str(tmp_path))
  assert exc_info.value.text == "rlog not found"


def test_dashcam_web_upload_honors_explicit_file_selection(tmp_path: Path, monkeypatch):
  (tmp_path / "qcamera.ts").write_bytes(b"camera-data")
  (tmp_path / "rlog.zst").write_bytes(b"log-data")
  (tmp_path / "qlog.zst").write_bytes(b"excluded-data")
  FakeSession.instances = []
  FakeSession.size_deltas = []
  monkeypatch.setattr(web_upload, "ClientSession", FakeSession)

  assert asyncio.run(web_upload.upload_folder_to_web(
    str(tmp_path),
    "device",
    "route|0",
    "https://upload.example",
    "token",
    filenames=("qcamera.ts", "rlog.zst"),
  ))

  session = FakeSession.instances[-1]
  assert [request[0].rsplit("/", 1)[-1] for request in session.requests] == [
    "qcamera.ts",
    "rlog.zst",
  ]


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
