import asyncio
import json
import re
import sys
import types
from unittest import mock

import pytest
from aiohttp import web
from aiohttp.test_utils import TestServer

try:
  import msgq  # noqa: F401
except ImportError:  # desktop without the compiled msgq: the upload path never uses it
  class _Stub(types.ModuleType):
    def __getattr__(self, name):
      return mock.MagicMock(name=name)
  for _name in ("msgq", "msgq.visionipc", "msgq.ipc_pyx"):
    sys.modules[_name] = _Stub(_name)

from openpilot.selfdrive.carrot import drive_upload
from openpilot.selfdrive.carrot.server.features.dashcam import routes, upload_jobs


class FakeDrive:
  """Just enough of the Drive v3 + OAuth token API: folders, files.list by name/parent, resumable upload."""

  def __init__(self, token_ok=True, fail_puts=0):
    self.token_ok, self.fail_puts = token_ok, fail_puts
    self.items: dict[str, dict] = {}  # id -> {name, parent, folder, data}
    self.sessions: dict[str, dict] = {}
    self.puts = 0
    app = web.Application()
    app.router.add_post("/token", self.token)
    app.router.add_get("/files", self.list)
    app.router.add_post("/files", self.create_folder)
    app.router.add_post("/upload", self.start)
    app.router.add_patch("/upload/{id}", self.start)
    app.router.add_put("/session/{sid}", self.put)
    self.server = TestServer(app)

  def new_id(self) -> str:
    return f"id{len(self.items) + len(self.sessions)}"

  async def token(self, request):
    if not self.token_ok:
      return web.json_response({"error": "invalid_grant"}, status=400)
    return web.json_response({"access_token": "t"})

  async def list(self, request):
    query = request.query["q"]
    name = re.search(r"name = '([^']*)'", query).group(1)
    parent = re.search(r"'([^']*)' in parents", query).group(1)
    folder = "mimeType" in query
    files = [{"id": i, "size": str(len(it["data"]))} for i, it in self.items.items()
             if it["name"] == name and it["parent"] == parent and it["folder"] == folder]
    return web.json_response({"files": files})

  async def create_folder(self, request):
    body = await request.json()
    fid = self.new_id()
    self.items[fid] = {"name": body["name"], "parent": body["parents"][0], "folder": True, "data": b""}
    return web.json_response({"id": fid})

  async def start(self, request):
    body = await request.json()
    sid = self.new_id()
    target = request.match_info.get("id")
    self.sessions[sid] = {"target": target, "name": body.get("name"), "parent": (body.get("parents") or [None])[0],
                          "size": int(request.headers["X-Upload-Content-Length"]), "data": b""}
    return web.Response(headers={"Location": str(self.server.make_url(f"/session/{sid}"))})

  async def put(self, request):
    self.puts += 1
    s = self.sessions[request.match_info["sid"]]
    chunk = await request.read()
    if chunk and self.fail_puts:
      self.fail_puts -= 1
      return web.Response(status=503)
    m = re.match(r"bytes (\d+)-(\d+)/(\d+)", request.headers["Content-Range"])
    if m:
      start = int(m.group(1))
      s["data"] = s["data"][:start] + chunk
    if len(s["data"]) < s["size"]:
      headers = {"Range": f"bytes=0-{len(s['data']) - 1}"} if s["data"] else {}
      return web.Response(status=308, headers=headers)
    if s["target"]:
      self.items[s["target"]]["data"] = s["data"]
    else:
      self.items[self.new_id()] = {"name": s["name"], "parent": s["parent"], "folder": False, "data": s["data"]}
    return web.json_response({"id": "done"})

  def path(self, *names) -> str | None:
    parent = "root"
    for name in names:
      found = [i for i, it in self.items.items() if it["name"] == name and it["parent"] == parent]
      if not found:
        return None
      parent = found[0]
    return parent


@pytest.fixture
def drive(monkeypatch, tmp_path):
  fakes = []

  def make(**kwargs):
    fake = FakeDrive(**kwargs)
    fakes.append(fake)
    asyncio.get_event_loop().run_until_complete(fake.server.start_server())
    monkeypatch.setattr(drive_upload, "TOKEN_URL", str(fake.server.make_url("/token")))
    monkeypatch.setattr(drive_upload, "FILES_URL", str(fake.server.make_url("/files")))
    monkeypatch.setattr(drive_upload, "UPLOAD_URL", str(fake.server.make_url("/upload")))
    monkeypatch.setattr(drive_upload, "CHUNK_SIZE", 4)
    monkeypatch.setattr(drive_upload, "RETRY_BASE_S", 0)
    config = tmp_path / "google_drive.json"
    config.write_text(json.dumps({"client_id": "c", "client_secret": "s", "refresh_token": "r"}))
    monkeypatch.setenv("CARROT_GOOGLE_DRIVE_CONFIG", str(config))
    return fake
  loop = asyncio.new_event_loop()
  asyncio.set_event_loop(loop)
  yield make
  for fake in fakes:
    loop.run_until_complete(fake.server.close())
  loop.close()


def segment(tmp_path, name, files=(("rlog.zst", b"0123456789"), ("qcamera.ts", b"abcde"))):
  folder = tmp_path / name
  folder.mkdir()
  for fname, data in files:
    (folder / fname).write_bytes(data)
  return str(folder)


async def upload(fake, folder, seg, names=("rlog.zst", "qcamera.ts"), on_progress=None):
  async with drive_upload.new_session() as session:
    client = drive_upload.DriveClient(session, drive_upload.load_config())
    return await drive_upload.upload_folder_to_drive(client, folder, seg.rsplit("--", 1)[0], seg, list(names),
                                                     on_progress=on_progress)


def test_upload_makes_route_folders_and_resumes_after_a_failed_chunk(drive, tmp_path):
  fake = drive(fail_puts=1)
  seen = {}
  folder = segment(tmp_path, "r1--0")
  ok = asyncio.get_event_loop().run_until_complete(
    upload(fake, folder, "r1--0", on_progress=lambda name, sent, size, delta: seen.__setitem__(name, sent)))
  assert ok
  seg = fake.path("openpilot", "r1", "r1--0")
  assert seg is not None
  data = {it["name"]: it["data"] for it in fake.items.values() if it["parent"] == seg}
  assert data == {"rlog.zst": b"0123456789", "qcamera.ts": b"abcde"}
  assert seen == {"rlog.zst": 10, "qcamera.ts": 5}


def test_same_name_and_size_is_skipped(drive, tmp_path):
  fake = drive()
  folder = segment(tmp_path, "r1--0")
  loop = asyncio.get_event_loop()
  loop.run_until_complete(upload(fake, folder, "r1--0"))
  puts = fake.puts
  loop.run_until_complete(upload(fake, folder, "r1--0"))
  assert fake.puts == puts
  (tmp_path / "r1--0" / "rlog.zst").write_bytes(b"changed!!!!!")  # different size: replaced in place
  loop.run_until_complete(upload(fake, folder, "r1--0"))
  seg = fake.path("openpilot", "r1", "r1--0")
  files = [it for it in fake.items.values() if it["parent"] == seg and it["name"] == "rlog.zst"]
  assert [f["data"] for f in files] == [b"changed!!!!!"]


def test_parallel_segments_share_one_route_folder(drive, tmp_path):
  fake = drive()
  a, b = segment(tmp_path, "r1--0"), segment(tmp_path, "r1--1")

  async def both():
    async with drive_upload.new_session() as session:
      client = drive_upload.DriveClient(session, drive_upload.load_config())
      await asyncio.gather(*(drive_upload.upload_folder_to_drive(client, f, "r1", s, ["rlog.zst"])
                             for f, s in ((a, "r1--0"), (b, "r1--1"))))
  asyncio.get_event_loop().run_until_complete(both())
  assert sum(1 for it in fake.items.values() if it["name"] == "r1" and it["folder"]) == 1
  assert sum(1 for it in fake.items.values() if it["name"] == "openpilot") == 1


def test_bad_refresh_token_names_the_fix(drive, tmp_path):
  fake = drive(token_ok=False)
  with pytest.raises(drive_upload.DriveError, match="refresh token"):
    asyncio.get_event_loop().run_until_complete(upload(fake, segment(tmp_path, "r1--0"), "r1--0"))


def test_config_needs_all_three_values(tmp_path, monkeypatch):
  path = tmp_path / "google_drive.json"
  monkeypatch.setenv("CARROT_GOOGLE_DRIVE_CONFIG", str(path))
  assert not drive_upload.is_configured()
  path.write_text(json.dumps({"client_id": "c", "client_secret": "s"}))
  assert not drive_upload.is_configured()
  path.write_text(json.dumps({"client_id": "c", "client_secret": "s", "refresh_token": "r"}))
  assert drive_upload.is_configured()


@pytest.mark.parametrize("body, configured, expected", [
  ({}, False, "web"),
  ({"target": "web"}, False, "web"),
  ({"target": "drive"}, True, "drive"),
  ({"target": "drive"}, False, web.HTTPConflict),
  ({"target": "ftp"}, True, web.HTTPBadRequest),
])
def test_upload_target_request(monkeypatch, body, configured, expected):
  class Request:
    async def json(self):
      return body
  monkeypatch.setattr(drive_upload, "is_configured", lambda: configured)
  if isinstance(expected, str):
    assert asyncio.run(routes.request_upload_target(Request())) == expected
  else:
    with pytest.raises(expected):
      asyncio.run(routes.request_upload_target(Request()))


def test_drive_job_uploads_without_touching_the_web_server(drive, tmp_path, monkeypatch):
  fake = drive()
  folder = segment(tmp_path, "r1--0")
  monkeypatch.setattr(upload_jobs, "segment_dir", lambda seg: folder)
  monkeypatch.setattr(upload_jobs.upload, "upload_metadata", lambda params: {"dongleId": "d", "carName": "c"})

  async def no_web(*args, **kwargs):
    raise AssertionError("web upload server must not be used for Drive")
  monkeypatch.setattr(upload_jobs, "create_web_upload_session", no_web)
  monkeypatch.setattr(upload_jobs, "send_web_upload_complete", no_web)
  monkeypatch.setattr(upload_jobs, "upload_folder_to_web", no_web)
  job = upload_jobs.create_job(["r1--0"], target="drive")
  result = asyncio.get_event_loop().run_until_complete(
    upload_jobs.run_upload_segments(["r1--0"], job, target="drive"))
  assert result["ok"] and result["target"] == "drive" and job["upload_target"] == "drive"
  assert result["results"][0]["remotePath"] == "Google Drive/openpilot/r1/r1--0"
  assert fake.path("openpilot", "r1", "r1--0") is not None


def test_drive_job_without_config_fails_clearly(tmp_path, monkeypatch):
  monkeypatch.setenv("CARROT_GOOGLE_DRIVE_CONFIG", str(tmp_path / "missing.json"))
  with pytest.raises(RuntimeError, match="not configured"):
    asyncio.run(upload_jobs.run_upload_segments(["r1--0"], target="drive"))
