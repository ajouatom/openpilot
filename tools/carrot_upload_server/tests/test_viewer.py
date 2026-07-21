import asyncio
from pathlib import Path
import re
from urllib.parse import urlparse

from aiohttp.test_utils import TestClient, TestServer

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
    (segment / "qcamera.ts").write_bytes(f"qcamera-ts-{route}-{index}".encode())
    (segment / "qcamera.mp4").write_bytes(f"mp4-{route}-{index}".encode())
    (segment / "private.txt").write_text("must not be shared", encoding="utf-8")


def admin_headers() -> dict[str, str]:
  return {"Authorization": f"Bearer {ADMIN_KEY}"}


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

      details = await client.get(
        "/api/admin/route",
        params={"directory": DIRECTORY, "route": ROUTE},
        headers=admin_headers(),
      )
      assert details.status == 200
      details_body = await details.json()
      assert details_body["segmentCount"] == 2
      assert details_body["canonicalRoute"] == f"{DEVICE}|{ROUTE}"
      assert "private.txt" not in {
        item["name"]
        for segment in details_body["segments"]
        for item in segment["files"]
      }

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
      assert f'{DEVICE}/{ROUTE}' in share["plotJugglerCommand"]

      page = await client.get(f"/s/{token}")
      assert page.status == 200
      share_html = await page.text()
      assert "분석 도구 연결" in share_html
      assert "Cabana" in share_html
      assert "PlotJuggler" in share_html
      assert page.headers["X-Robots-Tag"].startswith("noindex")

      manifest_response = await client.get(f"/s/{token}/manifest")
      assert manifest_response.status == 200
      manifest = await manifest_response.json()
      assert manifest["route"] == ROUTE
      assert manifest["canonicalRoute"] == f"{DEVICE}|{ROUTE}"
      assert manifest["segmentCount"] == 2
      assert "directory" not in manifest
      assert all(segment["name"].startswith(f"{ROUTE}--") for segment in manifest["segments"])

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

      unknown = await client.get("/s/not-a-real-share")
      assert unknown.status == 404

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
