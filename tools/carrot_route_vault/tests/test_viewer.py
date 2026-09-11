import asyncio
from dataclasses import replace
from pathlib import Path
import re
from types import SimpleNamespace
from urllib.parse import quote, urlparse

from aiohttp.test_utils import TestClient, TestServer

from .. import viewer as viewer_module
from ..server import Config, create_app


ADMIN_KEY = "test-admin-key-with-enough-entropy"
DEVICE = "0123456789abcdef"
DIRECTORY = f"TEST CAR {DEVICE}"
ROUTE = "2026-07-20--00-00-00"
OTHER_ROUTE = "2026-07-21--00-00-00"


def viewer_config(tmp_path: Path) -> Config:
  return Config(
    storage_root=tmp_path / "uploads",
    db_path=tmp_path / "state" / "uploads.sqlite3",
    daily_device_quota=1024 * 1024,
    daily_ip_quota=4 * 1024 * 1024,
    max_file_bytes=1024 * 1024,
    max_tmux_bytes=1024 * 1024,
    min_free_bytes=0,
    session_ttl_seconds=600,
    concurrent_per_device=3,
    concurrent_global=16,
    route_admin_key=ADMIN_KEY,
    public_base_url="https://routes.example",
    video_cache_root=tmp_path / "state" / "video-cache",
  )


def seed_route(tmp_path: Path, route: str, segment_indexes: tuple[int, ...]) -> None:
  root = tmp_path / "uploads" / "routes" / DIRECTORY
  for index in segment_indexes:
    segment = root / f"{route}--{index}"
    segment.mkdir(parents=True)
    (segment / "rlog.zst").write_bytes(f"rlog-{route}-{index}".encode())
    (segment / "qlog.zst").write_bytes(f"qlog-{route}-{index}".encode())
    (segment / "fcamera.hevc").write_bytes(f"fcamera-{route}-{index}".encode())
    (segment / "dcamera.hevc").write_bytes(f"dcamera-{route}-{index}".encode())
    (segment / "qcamera.ts").write_bytes(f"qcamera-ts-{route}-{index}".encode())
    (segment / "qcamera.mp4").write_bytes(f"mp4-{route}-{index}".encode())
    (segment / "private.txt").write_text("must not be shared", encoding="utf-8")


def admin_headers() -> dict[str, str]:
  return {"Authorization": f"Bearer {ADMIN_KEY}"}


def test_precompiled_models_support_download_and_resume_without_exposing_other_files(tmp_path):
  async def run():
    root = tmp_path / 'uploads' / 'models' / 'cinque-v2'
    root.mkdir(parents=True)
    for name in ['precompiled.json', 'big_driving_tinygrad.pkl', 'precompiled-runtime.tar.gz', 'private.txt']:
      (root / name).write_bytes(b'0123456789')
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      for name in ['precompiled.json', 'big_driving_tinygrad.pkl', 'precompiled-runtime.tar.gz']:
        response = await client.get(f'/models/cinque-v2/{name}')
        assert response.status == 200
        assert await response.read() == b'0123456789'
      response = await client.get('/models/cinque-v2/big_driving_tinygrad.pkl', headers={'Range': 'bytes=4-'})
      assert response.status == 206
      assert await response.read() == b'456789'
      assert response.headers['Content-Range'] == 'bytes 4-9/10'
      assert (await client.get('/models/cinque-v2/private.txt')).status == 404
      assert (await client.get('/models/cinque-v2/missing.pkl')).status == 404
  asyncio.run(run())


async def create_share(client: TestClient, route: str = ROUTE) -> dict:
  response = await client.post(
    "/api/admin/shares",
    json={"directory": DIRECTORY, "route": route},
    headers=admin_headers(),
  )
  assert response.status == 200, await response.text()
  return await response.json()


def test_admin_catalog_login_and_route_details(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (0, 1))
    seed_route(tmp_path, OTHER_ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      denied = await client.get("/api/admin/routes")
      assert denied.status == 401

      wrong_login = await client.post("/api/admin/login", json={"key": "wrong"})
      assert wrong_login.status == 401
      login = await client.post("/api/admin/login", json={"key": ADMIN_KEY})
      assert login.status == 200
      cookie = login.headers["Set-Cookie"]
      assert "HttpOnly" in cookie
      assert "Secure" in cookie
      assert "SameSite=Strict" in cookie

      catalog = await client.get("/api/admin/routes", headers=admin_headers())
      assert catalog.status == 200
      catalog_body = await catalog.json()
      assert catalog_body["total"] == 2
      assert {item["route"] for item in catalog_body["routes"]} == {ROUTE, OTHER_ROUTE}
      assert all(item["canonicalRoute"].startswith(f"{DEVICE}|") for item in catalog_body["routes"])
      route_item = next(item for item in catalog_body["routes"] if item["route"] == ROUTE)
      assert route_item["driveId"] == ROUTE.split("--", 1)[0]
      assert [item["name"] for item in route_item["segments"]] == [f"{ROUTE}--0", f"{ROUTE}--1"]

      details = await client.get(
        "/api/admin/route",
        params={"directory": DIRECTORY, "route": ROUTE},
        headers=admin_headers(),
      )
      assert details.status == 200
      details_body = await details.json()
      assert details_body["segmentCount"] == 2
      assert details_body["canonicalRoute"] == f"{DEVICE}|{ROUTE}"
      assert details_body["segments"][0]["previewUrl"].endswith("/0.jpg")
      assert "private.txt" not in {
        item["name"]
        for segment in details_body["segments"]
        for item in segment["files"]
      }
      assert "dcamera.hevc" not in {
        item["name"]
        for segment in details_body["segments"]
        for item in segment["files"]
      }
      denied_dcamera = await client.get(
        f"/admin/files/{DIRECTORY}/{ROUTE}/0/dcamera.hevc",
        headers=admin_headers(),
      )
      assert denied_dcamera.status == 404

      admin_page = await client.get("/admin")
      assert admin_page.status == 200
      admin_html = await admin_page.text()
      assert "Carrot Routes" in admin_html
      assert "Route 보관함" in admin_html
      assert "공유 링크" in admin_html
      assert admin_page.headers["X-Robots-Tag"].startswith("noindex")

      home_page = await client.get("/")
      home_html = await home_page.text()
      assert home_page.status == 200
      assert "나의 주행 기록" in home_html
      assert ROUTE not in home_html
      assert DIRECTORY not in home_html

  asyncio.run(run())


def test_route_share_is_scoped_and_streams_files_and_video(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (0, 1))
    seed_route(tmp_path, OTHER_ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      share = await create_share(client)
      token = share["token"]
      assert share["shareUrl"] == f"https://routes.example/s/{token}"
      assert share["apiHost"] == f"https://routes.example/s/{token}/api"
      assert f'{DEVICE}/{ROUTE}' in share["cabanaCommand"]
      assert share["plotJugglerCommand"].endswith(f'"{DEVICE}/{ROUTE}/0:2/r"')
      assert share["jotPlugglerCommand"].endswith(f'"{DEVICE}/{ROUTE}/0:2/r"')

      page = await client.get(f"/s/{token}")
      assert page.status == 200
      share_html = await page.text()
      assert "분석 도구 연결" in share_html
      assert "Cabana" in share_html
      assert "PlotJuggler" in share_html
      assert "JotPluggler" in share_html
      assert page.headers["X-Robots-Tag"].startswith("noindex")

      manifest_response = await client.get(f"/s/{token}/manifest")
      assert manifest_response.status == 200
      manifest = await manifest_response.json()
      assert manifest["route"] == ROUTE
      assert manifest["canonicalRoute"] == f"{DEVICE}|{ROUTE}"
      assert manifest["segmentCount"] == 2
      assert "directory" not in manifest
      assert all(segment["name"].startswith(f"{ROUTE}--") for segment in manifest["segments"])
      assert manifest["segments"][0]["previewUrl"].endswith("/preview/0.jpg")
      assert "dcamera.hevc" not in {
        item["name"]
        for segment in manifest["segments"]
        for item in segment["files"]
      }

      qlog_url = next(
        item["url"]
        for item in manifest["segments"][0]["files"]
        if item["name"] == "qlog.zst"
      )
      qlog_path = urlparse(qlog_url).path
      ranged = await client.get(qlog_path, headers={"Range": "bytes=0-3"})
      assert ranged.status == 206
      assert await ranged.read() == b"qlog"
      assert ranged.headers["Content-Type"] == "application/zstd"

      video_path = urlparse(manifest["segments"][0]["videoUrl"]).path
      video = await client.get(video_path, headers={"Range": "bytes=0-3"})
      assert video.status == 206
      assert await video.read() == b"mp4-"
      assert video.headers["Content-Type"] == "video/mp4"

      other_file = f"/s/{token}/files/{DEVICE}/{OTHER_ROUTE}/0/qlog.zst"
      denied = await client.get(other_file)
      assert denied.status == 404

      denied_dcamera = await client.get(
        f"/s/{token}/files/{DEVICE}/{ROUTE}/0/dcamera.hevc",
      )
      assert denied_dcamera.status == 404

      unknown = await client.get("/s/not-a-real-share")
      assert unknown.status == 404

  asyncio.run(run())


def test_plotjuggler_command_targets_the_existing_rlog_segment(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (3,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      share = await create_share(client)
      assert share["plotJugglerCommand"].endswith(f'"{DEVICE}/{ROUTE}/3/r"')

      manifest_response = await client.get(f'/s/{share["token"]}/manifest')
      assert manifest_response.status == 200
      manifest = await manifest_response.json()
      assert manifest["plotJugglerCommand"].endswith(f'"{DEVICE}/{ROUTE}/3/r"')
      assert manifest["cabanaCommand"].endswith(f'"{DEVICE}/{ROUTE}/3"')
      assert manifest["jotPlugglerCommand"].endswith(f'"{DEVICE}/{ROUTE}/3/r"')

  asyncio.run(run())


def test_public_upload_link_opens_exact_segment_with_web_video_and_tools(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (10, 11, 12))
    encoded_directory = quote(DIRECTORY, safe="")
    selection = f"{ROUTE}--10"
    page_path = f"/routes/{encoded_directory}/{selection}"
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      page = await client.get(page_path)
      assert page.status == 200
      page_html = await page.text()
      assert "Cabana" in page_html
      assert "PlotJuggler" in page_html
      assert "JotPluggler" in page_html
      assert page.headers["X-Robots-Tag"].startswith("noindex")

      manifest_response = await client.get(f"{page_path}/manifest")
      assert manifest_response.status == 200
      manifest = await manifest_response.json()
      assert [segment["index"] for segment in manifest["segments"]] == [10]
      assert manifest["segments"][0]["previewUrl"].endswith("/preview/10.jpg")
      assert manifest["publicUrl"] == f"https://routes.example/routes/{encoded_directory}/{selection}"
      assert manifest["apiHost"] == f"{manifest['publicUrl']}/api"
      assert manifest["cabanaCommand"].endswith(f'"{DEVICE}/{ROUTE}/10"')
      rlog_target = f'"{DEVICE}/{ROUTE}/10/r"'
      assert manifest["plotJugglerCommand"].endswith(rlog_target)
      assert manifest["jotPlugglerCommand"].endswith(rlog_target)
      assert "dcamera.hevc" not in {
        item["name"]
        for segment in manifest["segments"]
        for item in segment["files"]
      }

      video = await client.get(urlparse(manifest["segments"][0]["videoUrl"]).path, headers={"Range": "bytes=0-3"})
      assert video.status == 206
      assert await video.read() == b"mp4-"

      denied_other_segment = await client.get(
        f"{page_path}/files/{DEVICE}/{ROUTE}/11/rlog.zst",
      )
      assert denied_other_segment.status == 404

  asyncio.run(run())


def test_public_page_generates_and_caches_scoped_qcamera_preview(tmp_path: Path, monkeypatch):
  async def run():
    seed_route(tmp_path, ROUTE, (10, 11))
    fake_ffmpeg = tmp_path / "ffmpeg"
    fake_ffmpeg.write_bytes(b"executable")
    calls = 0

    def fake_run(command, **_kwargs):
      nonlocal calls
      calls += 1
      Path(command[-1]).write_bytes(b"\xff\xd8preview\xff\xd9")
      return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr(viewer_module.subprocess, "run", fake_run)
    config = replace(viewer_config(tmp_path), ffmpeg_binary=str(fake_ffmpeg))
    encoded_directory = quote(DIRECTORY, safe="")
    selection = f"{ROUTE}--10"
    page_path = f"/routes/{encoded_directory}/{selection}"

    async with TestClient(TestServer(create_app(config, start_cleanup=False))) as client:
      page = await client.get(page_path)
      page_html = await page.text()
      assert '<video id="video" autoplay muted playsinline preload="metadata">' in page_html
      assert 'id="videoSeek"' not in page_html
      assert "attachRadarReview(video)" in page_html
      assert "setVideo(featured.videoUrl,featured.previewUrl,featured.index)" in page_html
      assert "video.play().catch(()=>{})" in page_html

      manifest_response = await client.get(f"{page_path}/manifest")
      manifest = await manifest_response.json()
      preview_path = urlparse(manifest["segments"][0]["previewUrl"]).path

      first = await client.get(preview_path)
      assert first.status == 200
      assert first.headers["Content-Type"] == "image/jpeg"
      assert first.headers["Cache-Control"] == "no-store"
      assert await first.read() == b"\xff\xd8preview\xff\xd9"

      second = await client.get(preview_path)
      assert second.status == 200
      assert await second.read() == b"\xff\xd8preview\xff\xd9"
      assert calls == 1

      denied_other_segment = await client.get(f"{page_path}/preview/11.jpg")
      assert denied_other_segment.status == 404

  asyncio.run(run())


def test_public_upload_range_scopes_contiguous_logs_and_comma_api(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (10, 11, 12, 13))
    encoded_directory = quote(DIRECTORY, safe="")
    selection = f"{ROUTE}--10:13"
    page_path = f"/routes/{encoded_directory}/{selection}"
    prefix = f"{page_path}/api/v1"
    canonical = f"{DEVICE}|{ROUTE}"
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      manifest_response = await client.get(f"{page_path}/manifest")
      assert manifest_response.status == 200
      manifest = await manifest_response.json()
      assert [segment["index"] for segment in manifest["segments"]] == [10, 11, 12]
      assert manifest["cabanaCommand"].endswith(f'"{DEVICE}/{ROUTE}/10:13"')
      rlog_target = f'"{DEVICE}/{ROUTE}/10:13/r"'
      assert manifest["plotJugglerCommand"].endswith(rlog_target)
      assert manifest["jotPlugglerCommand"].endswith(rlog_target)

      route_files = await client.get(f"{prefix}/route/{canonical}/files")
      assert route_files.status == 200
      files = await route_files.json()
      assert len(files["logs"]) == 3
      assert all(any(f"/{index}/rlog.zst" in url for url in files["logs"]) for index in (10, 11, 12))
      assert not any("/13/" in url for urls in files.values() for url in urls)

      metadata = await client.get(f"{prefix}/route/{canonical}")
      assert metadata.status == 200
      assert await metadata.json() == {
        "fullname": canonical,
        "maxqlog": 12,
        "maxrlog": 12,
        "segmentCount": 3,
      }

      double_slash = await client.get(f"{page_path}/api//v1/route/{canonical}")
      assert double_slash.status == 200

  asyncio.run(run())


def test_public_upload_range_rejects_missing_segment(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (10, 12))
    encoded_directory = quote(DIRECTORY, safe="")
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      response = await client.get(f"/routes/{encoded_directory}/{ROUTE}--10:13/manifest")
      assert response.status == 404

  asyncio.run(run())


def test_share_exposes_existing_cabana_and_plotjuggler_api_contract(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (0, 1, 2))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      share = await create_share(client)
      token = share["token"]
      canonical = f"{DEVICE}|{ROUTE}"
      prefix = f"/s/{token}/api/v1"

      route_files = await client.get(f"{prefix}/route/{canonical}/files")
      assert route_files.status == 200
      files = await route_files.json()
      assert len(files["logs"]) == 3
      assert len(files["qlogs"]) == 3
      assert len(files["cameras"]) == 3
      assert len(files["qcameras"]) == 3
      assert "dcameras" not in files
      assert all(re.search(rf"/{DEVICE}/{re.escape(ROUTE)}/\d+/[^/]+$", url) for urls in files.values() for url in urls)

      route_meta = await client.get(f"{prefix}/route/{canonical}")
      assert route_meta.status == 200
      assert await route_meta.json() == {
        "fullname": canonical,
        "maxqlog": 2,
        "maxrlog": 2,
        "segmentCount": 3,
      }

      # PlotJuggler's negative/open-ended segment lookup currently emits a
      # double slash when API_HOST contains a path. The viewer accepts both.
      double_slash_meta = await client.get(f"/s/{token}/api//v1/route/{canonical}")
      assert double_slash_meta.status == 200
      assert (await double_slash_meta.json())["maxqlog"] == 2

      devices = await client.get(f"{prefix}/me/devices/")
      assert devices.status == 200
      assert (await devices.json())[0]["dongle_id"] == DEVICE

      routes = await client.get(f"{prefix}/devices/{DEVICE}/routes_segments")
      assert routes.status == 200
      assert (await routes.json())[0]["fullname"] == canonical

      wrong_route = await client.get(f"{prefix}/route/{DEVICE}|{OTHER_ROUTE}/files")
      assert wrong_route.status == 404

  asyncio.run(run())


def test_admin_can_list_and_revoke_share_without_storing_plain_token(tmp_path: Path):
  async def run():
    seed_route(tmp_path, ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      share = await create_share(client)
      token = share["token"]

      shares = await client.get("/api/admin/shares", headers=admin_headers())
      body = await shares.json()
      assert body["shares"][0]["id"] == share["id"]
      assert body["shares"][0]["active"] is True
      assert token not in await shares.text()

      revoke = await client.post(
        f"/api/admin/shares/{share['id']}/revoke",
        headers=admin_headers(),
      )
      assert revoke.status == 200
      revoked_page = await client.get(f"/s/{token}")
      assert revoked_page.status == 404

  asyncio.run(run())
