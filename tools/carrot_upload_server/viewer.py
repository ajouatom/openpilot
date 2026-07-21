# ruff: noqa: E501
from __future__ import annotations

import asyncio
import hashlib
import hmac
import html
import logging
import os
import re
import secrets
import shutil
import sqlite3
import subprocess
from collections import defaultdict
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path
from typing import Any
from urllib.parse import quote

from aiohttp import web


ROUTE_RE = re.compile(r"^[A-Za-z0-9_.|-]{20}$")
SEGMENT_RE = re.compile(r"^(?P<route>[A-Za-z0-9_.|-]{20})--(?P<index>\d+)$")
DEVICE_RE = re.compile(r"(?P<device>[a-z0-9]{16})$", re.IGNORECASE)
SHARE_ID_RE = re.compile(r"^[A-Za-z0-9_-]{8,32}$")
SHARE_SECRET_RE = re.compile(r"^[A-Za-z0-9_-]{24,96}$")

VIEWABLE_FILES = frozenset({
  "rlog.zst", "rlog.bz2", "rlog",
  "qlog.zst", "qlog.bz2", "qlog",
  "fcamera.hevc", "dcamera.hevc", "ecamera.hevc",
  "qcamera.ts", "qcamera.mp4",
})
FILE_GROUPS = {
  "rlog.zst": "logs", "rlog.bz2": "logs", "rlog": "logs",
  "qlog.zst": "qlogs", "qlog.bz2": "qlogs", "qlog": "qlogs",
  "fcamera.hevc": "cameras",
  "dcamera.hevc": "dcameras",
  "ecamera.hevc": "ecameras",
  "qcamera.ts": "qcameras",
}
CONTENT_TYPES = {
  "rlog.zst": "application/zstd",
  "qlog.zst": "application/zstd",
  "rlog.bz2": "application/x-bzip2",
  "qlog.bz2": "application/x-bzip2",
  "fcamera.hevc": "video/h265",
  "dcamera.hevc": "video/h265",
  "ecamera.hevc": "video/h265",
  "qcamera.ts": "video/mp2t",
  "qcamera.mp4": "video/mp4",
}

ADMIN_COOKIE = "carrot_route_admin"
ADMIN_COOKIE_MESSAGE = b"carrot-route-admin-v1"
ROUTE_VIEWER_KEY = web.AppKey("route_viewer", object)


def _human_size(size: int) -> str:
  value = float(max(0, size))
  for unit in ("B", "KB", "MB", "GB", "TB"):
    if value < 1024 or unit == "TB":
      return f"{value:.0f} {unit}" if unit == "B" else f"{value:.1f} {unit}"
    value /= 1024
  return f"{size} B"


def _component(value: Any, *, route: bool = False) -> str:
  text = str(value or "").strip()
  if not text or text in {".", ".."} or "/" in text or "\\" in text or "\0" in text:
    raise web.HTTPBadRequest(text="invalid route component")
  if route and not ROUTE_RE.fullmatch(text):
    raise web.HTTPBadRequest(text="invalid route")
  return text


def _segment_index(value: Any) -> int:
  try:
    index = int(value)
  except (TypeError, ValueError) as exc:
    raise web.HTTPBadRequest(text="invalid segment index") from exc
  if index < 0 or index > 1000000:
    raise web.HTTPBadRequest(text="invalid segment index")
  return index


def _small_json_body(request: web.Request, limit: int = 16 * 1024) -> Any:
  if request.content_length is not None and request.content_length > limit:
    raise web.HTTPRequestEntityTooLarge(max_size=limit, actual_size=request.content_length)
  return request.json()


class RouteViewer:
  def __init__(
    self,
    *,
    storage_root: Path,
    db_path: Path,
    admin_key: str = "",
    public_base_url: str = "",
    video_cache_root: Path | None = None,
    ffmpeg_binary: str = "ffmpeg",
    video_cache_max_bytes: int = 10 * 1024 * 1024 * 1024,
  ) -> None:
    self.storage_root = Path(storage_root)
    self.db_path = Path(db_path)
    self.admin_key = str(admin_key or "").strip()
    self.public_base_url = str(public_base_url or "").strip().rstrip("/")
    self.video_cache_root = Path(video_cache_root or (self.db_path.parent / "route_video_cache"))
    self.ffmpeg_binary = str(ffmpeg_binary or "ffmpeg")
    self.video_cache_max_bytes = max(128 * 1024 * 1024, int(video_cache_max_bytes))
    self._db_lock = asyncio.Lock()
    self._video_locks: dict[str, asyncio.Lock] = {}
    self._init_db()

  @property
  def routes_root(self) -> Path:
    return self.storage_root / "routes"

  def _connect(self) -> sqlite3.Connection:
    connection = sqlite3.connect(self.db_path, timeout=30)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA journal_mode=WAL")
    return connection

  def _init_db(self) -> None:
    self.db_path.parent.mkdir(parents=True, exist_ok=True)
    with self._connect() as connection:
      connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS route_shares (
          share_id TEXT PRIMARY KEY,
          token_hash TEXT NOT NULL,
          directory TEXT NOT NULL,
          route TEXT NOT NULL,
          created_at INTEGER NOT NULL,
          expires_at INTEGER,
          revoked_at INTEGER
        );
        CREATE INDEX IF NOT EXISTS route_shares_route
          ON route_shares(directory, route, created_at DESC);
        """,
      )

  @staticmethod
  def _token_hash(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()

  def _admin_cookie_value(self) -> str:
    if not self.admin_key:
      return ""
    return hmac.new(self.admin_key.encode("utf-8"), ADMIN_COOKIE_MESSAGE, hashlib.sha256).hexdigest()

  def _admin_authorized(self, request: web.Request) -> bool:
    if not self.admin_key:
      return False
    authorization = request.headers.get("Authorization", "")
    supplied = authorization[7:].strip() if authorization.startswith("Bearer ") else ""
    if supplied and hmac.compare_digest(supplied, self.admin_key):
      return True
    cookie = str(request.cookies.get(ADMIN_COOKIE) or "")
    expected = self._admin_cookie_value()
    return bool(cookie and expected and hmac.compare_digest(cookie, expected))

  def _require_admin(self, request: web.Request) -> None:
    if not self.admin_key:
      raise web.HTTPServiceUnavailable(text="route viewer admin key is not configured")
    if not self._admin_authorized(request):
      raise web.HTTPUnauthorized(text="invalid admin key")

  def _public_base(self, request: web.Request) -> str:
    return self.public_base_url or f"{request.scheme}://{request.host}"

  def _directory_path(self, directory: Any) -> Path:
    name = _component(directory)
    root = self.routes_root.resolve()
    path = root / name
    try:
      if path.is_symlink() or not path.is_dir():
        raise web.HTTPNotFound(text="route directory not found")
      resolved = path.resolve()
    except OSError as exc:
      raise web.HTTPNotFound(text="route directory not found") from exc
    if root not in resolved.parents:
      raise web.HTTPBadRequest(text="invalid route directory")
    return resolved

  @staticmethod
  def _device_id(directory: str) -> str:
    match = DEVICE_RE.search(directory)
    return match.group("device").lower() if match else ""

  def _segment_path(self, directory: Any, route: Any, index: Any) -> Path:
    route_name = _component(route, route=True)
    segment = f"{route_name}--{_segment_index(index)}"
    directory_path = self._directory_path(directory)
    path = directory_path / segment
    if path.is_symlink() or not path.is_dir():
      raise web.HTTPNotFound(text="segment not found")
    resolved = path.resolve()
    if directory_path not in resolved.parents:
      raise web.HTTPBadRequest(text="invalid segment path")
    return resolved

  def _file_path(self, directory: Any, route: Any, index: Any, filename: Any) -> Path:
    name = _component(filename)
    if name not in VIEWABLE_FILES:
      raise web.HTTPNotFound(text="route file not found")
    segment_path = self._segment_path(directory, route, index)
    path = segment_path / name
    if path.is_symlink() or not path.is_file():
      raise web.HTTPNotFound(text="route file not found")
    resolved = path.resolve()
    if segment_path not in resolved.parents:
      raise web.HTTPBadRequest(text="invalid route file")
    return resolved

  def _catalog_sync(self) -> list[dict[str, Any]]:
    root = self.routes_root
    if not root.is_dir():
      return []
    routes: list[dict[str, Any]] = []
    for directory_path in root.iterdir():
      try:
        if directory_path.is_symlink() or not directory_path.is_dir():
          continue
        grouped: dict[str, list[tuple[int, float]]] = defaultdict(list)
        for segment_path in directory_path.iterdir():
          if segment_path.is_symlink() or not segment_path.is_dir():
            continue
          match = SEGMENT_RE.fullmatch(segment_path.name)
          if not match:
            continue
          grouped[match.group("route")].append((int(match.group("index")), segment_path.stat().st_mtime))
        device_id = self._device_id(directory_path.name)
        for route, segments in grouped.items():
          ordered = sorted(segments)
          routes.append({
            "directory": directory_path.name,
            "deviceId": device_id,
            "route": route,
            "canonicalRoute": f"{device_id}|{route}" if device_id else "",
            "toolCompatible": bool(device_id),
            "segmentCount": len(ordered),
            "firstSegment": ordered[0][0],
            "lastSegment": ordered[-1][0],
            "modifiedEpoch": int(max(item[1] for item in ordered)),
          })
      except OSError:
        continue
    routes.sort(key=lambda item: (int(item["modifiedEpoch"]), item["directory"], item["route"]), reverse=True)
    return routes

  def _route_exists_sync(self, directory: str, route: str) -> bool:
    try:
      directory_path = self._directory_path(directory)
      return any(
        path.is_dir() and not path.is_symlink()
        for path in directory_path.glob(f"{route}--*")
        if (match := SEGMENT_RE.fullmatch(path.name)) and match.group("route") == route
      )
    except web.HTTPException:
      return False

  def _manifest_sync(
    self,
    directory: str,
    route: str,
    file_url: Callable[[str, int, str], str],
    video_url: Callable[[int], str],
  ) -> dict[str, Any]:
    directory = _component(directory)
    route = _component(route, route=True)
    directory_path = self._directory_path(directory)
    device_id = self._device_id(directory)
    segments: list[dict[str, Any]] = []
    total_size = 0
    modified_epoch = 0
    for segment_path in directory_path.glob(f"{route}--*"):
      try:
        if segment_path.is_symlink() or not segment_path.is_dir():
          continue
        match = SEGMENT_RE.fullmatch(segment_path.name)
        if not match or match.group("route") != route:
          continue
        index = int(match.group("index"))
        files: list[dict[str, Any]] = []
        for path in sorted(segment_path.iterdir(), key=lambda item: item.name):
          if path.name not in VIEWABLE_FILES or path.is_symlink() or not path.is_file():
            continue
          stat = path.stat()
          total_size += stat.st_size
          modified_epoch = max(modified_epoch, int(stat.st_mtime))
          files.append({
            "name": path.name,
            "size": stat.st_size,
            "sizeLabel": _human_size(stat.st_size),
            "contentType": CONTENT_TYPES.get(path.name, "application/octet-stream"),
            "url": file_url(device_id, index, path.name),
          })
        if files:
          names = {item["name"] for item in files}
          segments.append({
            "name": segment_path.name,
            "index": index,
            "files": files,
            "videoUrl": video_url(index) if names.intersection({"qcamera.ts", "qcamera.mp4"}) else "",
          })
      except OSError:
        continue
    segments.sort(key=lambda item: int(item["index"]))
    if not segments:
      raise web.HTTPNotFound(text="route not found")
    canonical = f"{device_id}|{route}" if device_id else ""
    return {
      "ok": True,
      "deviceId": device_id,
      "route": route,
      "canonicalRoute": canonical,
      "toolCompatible": bool(canonical),
      "segmentCount": len(segments),
      "firstSegment": int(segments[0]["index"]),
      "lastSegment": int(segments[-1]["index"]),
      "modifiedEpoch": modified_epoch,
      "totalSize": total_size,
      "totalSizeLabel": _human_size(total_size),
      "segments": segments,
    }

  @staticmethod
  def _share_token_parts(token: str) -> tuple[str, str]:
    try:
      share_id, secret = str(token or "").split(".", 1)
    except ValueError as exc:
      raise web.HTTPNotFound(text="share not found") from exc
    if not SHARE_ID_RE.fullmatch(share_id) or not SHARE_SECRET_RE.fullmatch(secret):
      raise web.HTTPNotFound(text="share not found")
    return share_id, secret

  async def _share(self, token: str) -> sqlite3.Row:
    share_id, _secret = self._share_token_parts(token)
    async with self._db_lock:
      with self._connect() as connection:
        row = connection.execute("SELECT * FROM route_shares WHERE share_id = ?", (share_id,)).fetchone()
    now = int(datetime.now(UTC).timestamp())
    if (
      row is None
      or not hmac.compare_digest(str(row["token_hash"]), self._token_hash(token))
      or row["revoked_at"] is not None
      or (row["expires_at"] is not None and int(row["expires_at"]) < now)
    ):
      raise web.HTTPNotFound(text="share not found")
    if not await asyncio.to_thread(self._route_exists_sync, str(row["directory"]), str(row["route"])):
      raise web.HTTPNotFound(text="shared route not found")
    return row

  async def _share_for_route(self, request: web.Request, canonical_route: str) -> tuple[sqlite3.Row, str, str]:
    row = await self._share(request.match_info.get("token", ""))
    expected_device = self._device_id(str(row["directory"]))
    expected_route = str(row["route"])
    value = str(canonical_route or "").replace("/", "|")
    if value != f"{expected_device}|{expected_route}":
      raise web.HTTPNotFound(text="route not found")
    return row, expected_device, expected_route

  def _share_file_url(self, request: web.Request, token: str, device: str, route: str, index: int, filename: str) -> str:
    prefix = f"{self._public_base(request)}/s/{quote(token, safe='._-')}/files"
    return f"{prefix}/{quote(device, safe='')}/{quote(route, safe='|-')}/{index}/{quote(filename, safe='.-')}"

  async def api_admin_login(self, request: web.Request) -> web.Response:
    if not self.admin_key:
      raise web.HTTPServiceUnavailable(text="route viewer admin key is not configured")
    body = await _small_json_body(request)
    supplied = str((body or {}).get("key") or "") if isinstance(body, dict) else ""
    if not hmac.compare_digest(supplied, self.admin_key):
      raise web.HTTPUnauthorized(text="invalid admin key")
    response = web.json_response({"ok": True})
    response.set_cookie(
      ADMIN_COOKIE,
      self._admin_cookie_value(),
      httponly=True,
      secure=request.secure or self.public_base_url.startswith("https://"),
      samesite="Strict",
      max_age=12 * 60 * 60,
      path="/",
    )
    return response

  async def api_admin_logout(self, _request: web.Request) -> web.Response:
    response = web.json_response({"ok": True})
    response.del_cookie(ADMIN_COOKIE, path="/")
    return response

  async def api_admin_routes(self, request: web.Request) -> web.Response:
    self._require_admin(request)
    routes = await asyncio.to_thread(self._catalog_sync)
    return web.json_response({"ok": True, "routes": routes, "total": len(routes)})

  async def api_admin_route(self, request: web.Request) -> web.Response:
    self._require_admin(request)
    directory = _component(request.query.get("directory"))
    route = _component(request.query.get("route"), route=True)
    base = self._public_base(request)

    def file_url(device: str, index: int, filename: str) -> str:
      prefix = f"{base}/admin/files/{quote(directory, safe='')}/{quote(route, safe='|-')}"
      return f"{prefix}/{index}/{quote(filename, safe='.-')}"

    def video_url(index: int) -> str:
      return f"{base}/admin/video/{quote(directory, safe='')}/{quote(route, safe='|-')}/{index}.mp4"

    manifest = await asyncio.to_thread(self._manifest_sync, directory, route, file_url, video_url)
    return web.json_response(manifest)

  async def api_admin_shares(self, request: web.Request) -> web.Response:
    self._require_admin(request)
    async with self._db_lock:
      with self._connect() as connection:
        rows = connection.execute(
          "SELECT share_id, directory, route, created_at, expires_at, revoked_at FROM route_shares ORDER BY created_at DESC",
        ).fetchall()
    now = int(datetime.now(UTC).timestamp())
    return web.json_response({
      "ok": True,
      "shares": [
        {
          "id": row["share_id"],
          "directory": row["directory"],
          "route": row["route"],
          "createdAt": row["created_at"],
          "expiresAt": row["expires_at"],
          "revokedAt": row["revoked_at"],
          "active": row["revoked_at"] is None and (row["expires_at"] is None or int(row["expires_at"]) >= now),
        }
        for row in rows
      ],
    })

  async def api_admin_share_create(self, request: web.Request) -> web.Response:
    self._require_admin(request)
    body = await _small_json_body(request)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    directory = _component(body.get("directory"))
    route = _component(body.get("route"), route=True)
    if not await asyncio.to_thread(self._route_exists_sync, directory, route):
      raise web.HTTPNotFound(text="route not found")
    try:
      expires_days = int(body.get("expiresDays") or 0)
    except (TypeError, ValueError) as exc:
      raise web.HTTPBadRequest(text="invalid expiration") from exc
    if expires_days < 0 or expires_days > 3650:
      raise web.HTTPBadRequest(text="invalid expiration")
    now = int(datetime.now(UTC).timestamp())
    expires_at = now + expires_days * 86400 if expires_days else None
    share_id = secrets.token_urlsafe(9)
    token = f"{share_id}.{secrets.token_urlsafe(32)}"
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute(
          "INSERT INTO route_shares(share_id, token_hash, directory, route, created_at, expires_at) VALUES(?,?,?,?,?,?)",
          (share_id, self._token_hash(token), directory, route, now, expires_at),
        )
    device_id = self._device_id(directory)
    canonical = f"{device_id}|{route}" if device_id else ""
    base = self._public_base(request)
    share_url = f"{base}/s/{token}"
    api_host = f"{share_url}/api"
    return web.json_response({
      "ok": True,
      "id": share_id,
      "token": token,
      "shareUrl": share_url,
      "apiHost": api_host,
      "canonicalRoute": canonical,
      "expiresAt": expires_at,
      "cabanaCommand": f'API_HOST="{api_host}" ./cabana "{device_id}/{route}"' if canonical else "",
      "plotJugglerCommand": (
        f'API_HOST="{api_host}" python3 -m openpilot.tools.plotjuggler.juggle "{device_id}/{route}"'
        if canonical else ""
      ),
    })

  async def api_admin_share_revoke(self, request: web.Request) -> web.Response:
    self._require_admin(request)
    share_id = _component(request.match_info.get("share_id"))
    if not SHARE_ID_RE.fullmatch(share_id):
      raise web.HTTPBadRequest(text="invalid share id")
    async with self._db_lock:
      with self._connect() as connection:
        changed = connection.execute(
          "UPDATE route_shares SET revoked_at = ? WHERE share_id = ? AND revoked_at IS NULL",
          (int(datetime.now(UTC).timestamp()), share_id),
        ).rowcount
    if not changed:
      raise web.HTTPNotFound(text="share not found")
    return web.json_response({"ok": True, "id": share_id})

  async def api_share_manifest(self, request: web.Request) -> web.Response:
    token = request.match_info.get("token", "")
    row = await self._share(token)
    directory = str(row["directory"])
    route = str(row["route"])
    base = self._public_base(request)
    device_id = self._device_id(directory)

    def file_url(device: str, index: int, filename: str) -> str:
      return self._share_file_url(request, token, device, route, index, filename)

    def video_url(index: int) -> str:
      return f"{base}/s/{quote(token, safe='._-')}/video/{index}.mp4"

    manifest = await asyncio.to_thread(self._manifest_sync, directory, route, file_url, video_url)
    manifest["shareId"] = row["share_id"]
    manifest["expiresAt"] = row["expires_at"]
    manifest["apiHost"] = f"{base}/s/{quote(token, safe='._-')}/api"
    manifest["cabanaCommand"] = (
      f'API_HOST="{manifest["apiHost"]}" ./cabana "{device_id}/{route}"' if device_id else ""
    )
    manifest["plotJugglerCommand"] = (
      f'API_HOST="{manifest["apiHost"]}" python3 -m openpilot.tools.plotjuggler.juggle "{device_id}/{route}"'
      if device_id else ""
    )
    return web.json_response(manifest)

  async def api_share_route_files(self, request: web.Request) -> web.Response:
    row, device_id, route = await self._share_for_route(request, request.match_info.get("canonical", ""))
    token = request.match_info.get("token", "")

    def file_url(device: str, index: int, filename: str) -> str:
      return self._share_file_url(request, token, device, route, index, filename)

    manifest = await asyncio.to_thread(
      self._manifest_sync,
      str(row["directory"]),
      route,
      file_url,
      lambda _index: "",
    )
    groups: dict[str, list[str]] = {name: [] for name in FILE_GROUPS.values()}
    for segment in manifest["segments"]:
      for item in segment["files"]:
        group = FILE_GROUPS.get(item["name"])
        if group:
          groups[group].append(item["url"])
    return web.json_response(groups)

  async def api_share_route_metadata(self, request: web.Request) -> web.Response:
    row, device_id, route = await self._share_for_route(request, request.match_info.get("canonical", ""))
    manifest = await asyncio.to_thread(
      self._manifest_sync,
      str(row["directory"]),
      route,
      lambda _device, _index, _filename: "",
      lambda _index: "",
    )
    return web.json_response({
      "fullname": f"{device_id}|{route}",
      "maxqlog": int(manifest["lastSegment"]),
      "maxrlog": int(manifest["lastSegment"]),
      "segmentCount": int(manifest["segmentCount"]),
    })

  async def api_share_devices(self, request: web.Request) -> web.Response:
    row = await self._share(request.match_info.get("token", ""))
    device_id = self._device_id(str(row["directory"]))
    return web.json_response([
      {"dongle_id": device_id, "alias": "Shared route", "device_type": "shared"},
    ] if device_id else [])

  async def api_share_device_routes(self, request: web.Request) -> web.Response:
    row = await self._share(request.match_info.get("token", ""))
    device_id = self._device_id(str(row["directory"]))
    if request.match_info.get("device", "").lower() != device_id:
      raise web.HTTPNotFound(text="device not found")
    route = str(row["route"])
    manifest = await asyncio.to_thread(
      self._manifest_sync,
      str(row["directory"]),
      route,
      lambda _device, _index, _filename: "",
      lambda _index: "",
    )
    end_ms = int(manifest["modifiedEpoch"]) * 1000
    start_ms = max(0, end_ms - int(manifest["segmentCount"]) * 60 * 1000)
    return web.json_response([{
      "fullname": f"{device_id}|{route}",
      "start_time_utc_millis": start_ms,
      "end_time_utc_millis": end_ms,
      "maxqlog": int(manifest["lastSegment"]),
    }])

  async def share_file(self, request: web.Request) -> web.StreamResponse:
    row = await self._share(request.match_info.get("token", ""))
    device = request.match_info.get("device", "").lower()
    route = _component(request.match_info.get("route"), route=True)
    expected_device = self._device_id(str(row["directory"]))
    if device != expected_device or route != row["route"]:
      raise web.HTTPNotFound(text="route file not found")
    filename = request.match_info.get("filename", "")
    path = await asyncio.to_thread(
      self._file_path,
      str(row["directory"]),
      route,
      request.match_info.get("index"),
      filename,
    )
    return web.FileResponse(path, headers={"Content-Type": CONTENT_TYPES.get(filename, "application/octet-stream")})

  async def admin_file(self, request: web.Request) -> web.StreamResponse:
    self._require_admin(request)
    filename = request.match_info.get("filename", "")
    path = await asyncio.to_thread(
      self._file_path,
      request.match_info.get("directory"),
      request.match_info.get("route"),
      request.match_info.get("index"),
      filename,
    )
    return web.FileResponse(path, headers={"Content-Type": CONTENT_TYPES.get(filename, "application/octet-stream")})

  def _prune_video_cache_sync(self) -> None:
    if not self.video_cache_root.is_dir():
      return
    entries: list[tuple[float, int, Path]] = []
    total = 0
    for path in self.video_cache_root.glob("*.mp4"):
      try:
        stat = path.stat()
      except OSError:
        continue
      total += stat.st_size
      entries.append((stat.st_mtime, stat.st_size, path))
    for _mtime, size, path in sorted(entries):
      if total <= self.video_cache_max_bytes:
        break
      try:
        path.unlink()
        total -= size
      except OSError:
        pass

  def _prepare_video_sync(self, directory: str, route: str, index: int) -> Path:
    segment_path = self._segment_path(directory, route, index)
    direct = segment_path / "qcamera.mp4"
    if direct.is_file() and not direct.is_symlink() and direct.stat().st_size > 0:
      return direct.resolve()
    source = segment_path / "qcamera.ts"
    if source.is_symlink() or not source.is_file() or source.stat().st_size <= 0:
      raise web.HTTPNotFound(text="qcamera video not found")
    stat = source.stat()
    cache_key = hashlib.sha256(
      f"{directory}\0{route}\0{index}\0{stat.st_size}\0{stat.st_mtime_ns}".encode(),
    ).hexdigest()
    self.video_cache_root.mkdir(parents=True, exist_ok=True)
    output = self.video_cache_root / f"{cache_key}.mp4"
    if output.is_file() and output.stat().st_size > 0:
      os.utime(output, None)
      return output
    ffmpeg = shutil.which(self.ffmpeg_binary) if not os.path.isabs(self.ffmpeg_binary) else self.ffmpeg_binary
    if not ffmpeg or not os.path.isfile(ffmpeg):
      raise web.HTTPServiceUnavailable(text="ffmpeg is not available")
    temp = self.video_cache_root / f".{cache_key}.{secrets.token_hex(6)}.part.mp4"
    try:
      result = subprocess.run(
        [
          ffmpeg, "-hide_banner", "-loglevel", "error", "-y",
          "-i", str(source), "-map", "0:v:0", "-map", "0:a?",
          "-c", "copy", "-movflags", "+faststart", str(temp),
        ],
        capture_output=True,
        text=True,
        timeout=180,
      )
      if result.returncode != 0 or not temp.is_file() or temp.stat().st_size <= 0:
        logging.warning("route video remux failed: %s", (result.stderr or "unknown ffmpeg error")[-500:])
        raise web.HTTPServiceUnavailable(text="video remux failed")
      os.replace(temp, output)
      self._prune_video_cache_sync()
      return output
    finally:
      temp.unlink(missing_ok=True)

  async def _video(self, directory: str, route: str, index: int) -> web.StreamResponse:
    key = f"{directory}\0{route}\0{index}"
    lock = self._video_locks.setdefault(key, asyncio.Lock())
    async with lock:
      path = await asyncio.to_thread(self._prepare_video_sync, directory, route, index)
    return web.FileResponse(path, headers={"Content-Type": "video/mp4"})

  async def share_video(self, request: web.Request) -> web.StreamResponse:
    row = await self._share(request.match_info.get("token", ""))
    index_text = str(request.match_info.get("index", ""))
    if index_text.endswith(".mp4"):
      index_text = index_text[:-4]
    return await self._video(str(row["directory"]), str(row["route"]), _segment_index(index_text))

  async def admin_video(self, request: web.Request) -> web.StreamResponse:
    self._require_admin(request)
    index_text = str(request.match_info.get("index", ""))
    if index_text.endswith(".mp4"):
      index_text = index_text[:-4]
    return await self._video(
      _component(request.match_info.get("directory")),
      _component(request.match_info.get("route"), route=True),
      _segment_index(index_text),
    )

  async def admin_page(self, _request: web.Request) -> web.Response:
    return _html_response(ADMIN_HTML)

  async def home_page(self, _request: web.Request) -> web.Response:
    return _html_response(HOME_HTML)

  async def share_page(self, request: web.Request) -> web.Response:
    row = await self._share(request.match_info.get("token", ""))
    title = html.escape(str(row["route"]))
    return _html_response(SHARE_HTML.replace("__ROUTE_TITLE__", title))

  def register(self, app: web.Application) -> None:
    app.router.add_get("/", self.home_page)
    app.router.add_get("/admin", self.admin_page)
    app.router.add_post("/api/admin/login", self.api_admin_login)
    app.router.add_post("/api/admin/logout", self.api_admin_logout)
    app.router.add_get("/api/admin/routes", self.api_admin_routes)
    app.router.add_get("/api/admin/route", self.api_admin_route)
    app.router.add_get("/api/admin/shares", self.api_admin_shares)
    app.router.add_post("/api/admin/shares", self.api_admin_share_create)
    app.router.add_post("/api/admin/shares/{share_id}/revoke", self.api_admin_share_revoke)
    app.router.add_get("/admin/files/{directory}/{route}/{index}/{filename}", self.admin_file)
    app.router.add_get("/admin/video/{directory}/{route}/{index}", self.admin_video)

    app.router.add_get("/s/{token}", self.share_page)
    app.router.add_get("/s/{token}/manifest", self.api_share_manifest)
    app.router.add_get("/s/{token}/files/{device}/{route}/{index}/{filename}", self.share_file)
    app.router.add_get("/s/{token}/video/{index}", self.share_video)
    for prefix in ("/s/{token}/api/v1", "/s/{token}/api//v1"):
      app.router.add_get(f"{prefix}/route/{{canonical}}/files", self.api_share_route_files)
      app.router.add_get(f"{prefix}/route/{{canonical}}", self.api_share_route_metadata)
      app.router.add_get(f"{prefix}/me/devices/", self.api_share_devices)
      app.router.add_get(f"{prefix}/devices/{{device}}/routes_segments", self.api_share_device_routes)
      app.router.add_get(f"{prefix}/devices/{{device}}/routes/preserved", self.api_share_device_routes)


def _html_response(source: str) -> web.Response:
  return web.Response(
    text=source,
    content_type="text/html",
    charset="utf-8",
    headers={
      "Content-Security-Policy": "; ".join([
        "default-src 'self'",
        "style-src 'unsafe-inline'",
        "script-src 'unsafe-inline'",
        "media-src 'self'",
        "img-src 'self' data:",
        "connect-src 'self'",
        "frame-ancestors 'none'",
      ]),
      "X-Robots-Tag": "noindex, nofollow, noarchive",
    },
  )


COMMON_STYLE = """
  :root{color-scheme:dark;--bg:#080b0d;--panel:#101518;--panel2:#151c20;--line:#253036;--text:#f3f6f4;--muted:#95a29b;--orange:#ff8a36;--orange2:#ffb15f;--green:#86d993;--red:#ff7d7d;--shadow:0 22px 70px rgba(0,0,0,.34);font-family:Inter,"Noto Sans KR",ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;background:var(--bg);color:var(--text)}
  *{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;min-height:100vh;background:radial-gradient(circle at 8% -10%,rgba(255,138,54,.13),transparent 30rem),radial-gradient(circle at 92% 0,rgba(134,217,147,.07),transparent 24rem),var(--bg);line-height:1.5}button,input,select{font:inherit}button,a{touch-action:manipulation}button{cursor:pointer}a{color:inherit}.hidden{display:none!important}.grow{flex:1}.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}.stack{display:grid;gap:12px}.muted{color:var(--muted)}.tiny{font-size:12px}.error{color:var(--red)}.success{color:var(--green)}
  .shell{width:min(1240px,calc(100% - 36px));margin:0 auto;padding:26px 0 64px}.topbar{display:flex;align-items:center;gap:14px;margin-bottom:34px}.brand{display:flex;align-items:center;gap:11px;text-decoration:none;font-weight:840;letter-spacing:-.03em}.brand-mark{width:38px;height:38px;display:grid;place-items:center;border-radius:12px;background:linear-gradient(145deg,var(--orange2),var(--orange));box-shadow:0 9px 25px rgba(255,138,54,.24)}.brand-mark svg{width:22px;height:22px}.brand-copy{font-size:18px}.badge{display:inline-flex;align-items:center;gap:7px;border:1px solid var(--line);background:rgba(16,21,24,.78);color:#b8c2bd;border-radius:999px;padding:6px 10px;font-size:12px;font-weight:700}.badge-dot{width:7px;height:7px;border-radius:99px;background:var(--green);box-shadow:0 0 0 4px rgba(134,217,147,.1)}
  h1,h2,h3,p{margin-top:0}h1{font-size:clamp(31px,5vw,58px);line-height:1.03;letter-spacing:-.055em;margin-bottom:16px}h2{font-size:21px;letter-spacing:-.025em;margin-bottom:14px}h3{font-size:15px;margin-bottom:5px}.eyebrow{color:var(--orange2);font-size:12px;font-weight:820;letter-spacing:.13em;text-transform:uppercase;margin-bottom:12px}.lede{color:#aeb9b3;font-size:17px;max-width:660px;margin-bottom:0}.card{background:linear-gradient(145deg,rgba(21,28,32,.96),rgba(14,19,22,.96));border:1px solid var(--line);border-radius:20px;box-shadow:var(--shadow)}.card-pad{padding:22px}.soft-card{background:rgba(17,23,26,.74);border:1px solid var(--line);border-radius:15px;padding:16px}
  button,.button{border:1px solid transparent;border-radius:11px;background:linear-gradient(145deg,var(--orange2),var(--orange));color:#17120e;padding:10px 14px;font-weight:800;text-decoration:none;display:inline-flex;align-items:center;justify-content:center;gap:7px;transition:transform .16s ease,filter .16s ease,border-color .16s ease}button:hover,.button:hover{filter:brightness(1.06);transform:translateY(-1px)}button:disabled{opacity:.55;cursor:wait;transform:none}.secondary{background:#1b252a!important;color:var(--text)!important;border-color:#304047!important}.ghost{background:transparent!important;color:#c2cbc6!important;border-color:var(--line)!important}.danger{background:#2c1b1d!important;color:#ffb1b1!important;border-color:#553036!important}.icon-button{width:40px;height:40px;padding:0}
  input,select{width:100%;border:1px solid #344148;border-radius:12px;background:#0b1012;color:var(--text);padding:12px 14px;outline:none;transition:border-color .16s,box-shadow .16s}input:focus,select:focus{border-color:var(--orange);box-shadow:0 0 0 4px rgba(255,138,54,.1)}label{display:grid;gap:7px;color:#c4cdc8;font-size:13px;font-weight:700}
  .hero{padding:42px 0 34px}.hero-grid{display:grid;grid-template-columns:minmax(0,1.2fr) minmax(290px,.8fr);align-items:end;gap:44px}.privacy-card{padding:24px;position:relative;overflow:hidden}.privacy-card:after{content:"";position:absolute;width:160px;height:160px;border:34px solid rgba(255,138,54,.08);border-radius:50%;right:-66px;bottom:-80px}.privacy-icon{width:43px;height:43px;border-radius:13px;background:#202b2f;display:grid;place-items:center;color:var(--green);margin-bottom:18px}.privacy-list{list-style:none;padding:0;margin:16px 0 0;display:grid;gap:10px}.privacy-list li{display:flex;gap:9px;color:#aeb8b3;font-size:14px}.privacy-list li:before{content:"✓";color:var(--green);font-weight:900}
  .login-wrap{min-height:calc(100vh - 150px);display:grid;place-items:center}.login-card{width:min(470px,100%);padding:34px}.login-mark{width:52px;height:52px;margin-bottom:26px}.login-card h1{font-size:32px}.login-card form{display:grid;gap:12px;margin-top:26px}.login-card button{width:100%;padding:12px}.login-note{display:flex;gap:9px;margin-top:18px;color:var(--muted);font-size:12px}
  .admin-head{display:flex;gap:22px;align-items:end;justify-content:space-between;margin-bottom:22px}.admin-head h1{font-size:42px;margin-bottom:7px}.toolbar{display:grid;grid-template-columns:minmax(240px,1fr) auto;gap:10px;margin:18px 0 24px}.toolbar #mobileLogout{display:none}.search-wrap{position:relative}.search-wrap:before{content:"⌕";position:absolute;left:14px;top:8px;color:var(--muted);font-size:20px}.search-wrap input{padding-left:40px}.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-bottom:24px}.stat{padding:18px}.stat strong{display:block;font-size:25px;letter-spacing:-.035em}.stat span{color:var(--muted);font-size:12px}.tabs{display:flex;gap:4px;border-bottom:1px solid var(--line);margin-bottom:18px}.tab{background:transparent!important;color:var(--muted)!important;border:0!important;border-radius:0;padding:10px 13px;transform:none!important}.tab.active{color:var(--text)!important;box-shadow:inset 0 -2px var(--orange)}
  .route-list{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:13px}.route-card{padding:20px;box-shadow:none;transition:border-color .18s,transform .18s}.route-card:hover{border-color:#405159;transform:translateY(-2px)}.route-date{font-size:20px;letter-spacing:-.03em;font-weight:820}.route-id{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;color:#bdc8c2;font-size:13px;margin-top:3px}.route-meta{display:flex;gap:7px;flex-wrap:wrap;margin:17px 0}.pill{display:inline-flex;align-items:center;border:1px solid #2c3a40;background:#121a1e;border-radius:999px;color:#aab6af;padding:5px 9px;font-size:11px}.route-actions{display:flex;gap:8px}.route-actions button{flex:1}.empty{padding:54px 20px;text-align:center;color:var(--muted);border:1px dashed #354248;border-radius:18px;grid-column:1/-1}
  .drawer{position:fixed;inset:0;z-index:20;background:rgba(0,0,0,.68);backdrop-filter:blur(8px);display:grid;justify-items:end}.drawer-panel{height:100%;width:min(760px,100%);overflow:auto;background:#0c1113;border-left:1px solid var(--line);padding:24px;box-shadow:-25px 0 70px rgba(0,0,0,.35)}.drawer-head{display:flex;align-items:start;gap:16px;margin-bottom:20px}.drawer-head h2{font-size:23px;margin:0}.video-stage{position:relative;display:grid;place-items:center;aspect-ratio:16/9;background:#030405;border:1px solid var(--line);border-radius:17px;overflow:hidden;margin-bottom:20px}.video-stage video{width:100%;height:100%;object-fit:contain}.video-empty{position:absolute;color:#66726c;font-size:13px;pointer-events:none}.segment{margin-bottom:10px}.segment summary{list-style:none;cursor:pointer;display:flex;align-items:center;gap:12px;padding:15px}.segment summary::-webkit-details-marker{display:none}.segment summary:after{content:"＋";color:var(--muted)}.segment[open] summary:after{content:"−"}.segment-body{padding:0 15px 15px}.files{display:flex;gap:7px;flex-wrap:wrap}.file{display:inline-flex;align-items:center;gap:6px;border:1px solid #2c3a40;background:#10181b;border-radius:10px;padding:7px 9px;color:#bac6bf;text-decoration:none;font-size:12px}.file:hover{border-color:#4b5e66;color:white}.file-size{color:#75837c}
  .modal{position:fixed;inset:0;z-index:30;background:rgba(0,0,0,.72);backdrop-filter:blur(9px);display:grid;place-items:center;padding:18px}.modal-card{width:min(560px,100%);max-height:calc(100vh - 36px);overflow:auto;padding:26px}.modal-card h2{font-size:25px}.modal-actions{display:flex;justify-content:flex-end;gap:9px;margin-top:20px}.command{position:relative;background:#090d0f;border:1px solid #29363b;border-radius:13px;padding:14px 46px 14px 14px;overflow-wrap:anywhere;font:12px/1.55 ui-monospace,SFMono-Regular,Consolas,monospace;color:#bfe4c5;margin:8px 0 15px}.copy{position:absolute;right:7px;top:7px;width:32px;height:32px;padding:0;background:#1e292d!important;color:#bec9c3!important;border-color:#334249!important}.share-url{font-size:13px;color:#ffc88f}.share-list{display:grid;gap:10px}.share-row{padding:17px;display:grid;grid-template-columns:minmax(0,1fr) auto;gap:14px;align-items:center}.status-chip{display:inline-flex;border-radius:99px;padding:4px 8px;font-size:10px;font-weight:800;background:rgba(134,217,147,.1);color:var(--green)}.status-chip.off{color:#c48f8f;background:rgba(255,125,125,.08)}
  .share-hero{padding:30px 0 22px}.share-hero h1{font-size:clamp(28px,5vw,48px);font-family:ui-monospace,SFMono-Regular,Consolas,monospace;overflow-wrap:anywhere}.summary-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:22px 0}.summary-item{padding:16px;border-left:2px solid var(--orange)}.summary-item strong{display:block;font-size:20px}.summary-item span{font-size:11px;color:var(--muted)}.section-head{display:flex;align-items:end;justify-content:space-between;gap:16px;margin:32px 0 13px}.section-head h2{margin:0}.tools-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.tool-card{padding:19px}.tool-name{display:flex;align-items:center;gap:9px;font-size:17px;font-weight:830}.tool-mark{width:34px;height:34px;border-radius:10px;background:#222d31;display:grid;place-items:center;color:var(--orange2)}.notice{border:1px solid rgba(255,138,54,.22);background:rgba(255,138,54,.06);border-radius:14px;padding:13px 15px;color:#bea994;font-size:12px}.public-segments{display:grid;gap:10px}.footer{border-top:1px solid var(--line);margin-top:44px;padding-top:20px;color:#69766f;font-size:12px;display:flex;justify-content:space-between;gap:14px;flex-wrap:wrap}
  @media(max-width:800px){.shell{width:min(100% - 24px,1240px);padding-top:17px}.hero-grid{grid-template-columns:1fr;gap:25px}.privacy-card{display:none}.route-list,.tools-grid{grid-template-columns:1fr}.stats{grid-template-columns:1fr 1fr}.stats .stat:last-child{grid-column:1/-1}.toolbar{grid-template-columns:1fr auto auto}.toolbar #mobileLogout{display:inline-flex}.admin-head #logout{display:none}.admin-head{align-items:start}.admin-head h1{font-size:34px}.share-row{grid-template-columns:1fr}.drawer-panel{padding:16px}.summary-grid{grid-template-columns:1fr 1fr}.summary-item:last-child{grid-column:1/-1}.topbar{margin-bottom:20px}}
  @media(prefers-reduced-motion:reduce){*{scroll-behavior:auto!important;transition:none!important}}
"""


BRAND = """<a class="brand" href="/"><span class="brand-mark"><svg viewBox="0 0 24 24" aria-hidden="true"><path fill="#1b130d" d="M14.9 5.2c1.3-2.5 3.7-3.4 5.8-3.1-.2 2.2-1.6 4.1-4.2 4.8 2.1.3 3.5 1.3 4.5 2.6-2 1-4.4.8-6.3-.9C13.9 15.4 11.1 21 8.4 21c-2.5 0-5.2-4.5-5.2-7.8 0-3.4 2.8-5.9 6.1-5.9 1.6 0 3.1.6 4.2 1.7.2-1.4.7-2.7 1.4-3.8Z"/></svg></span><span class="brand-copy">Carrot Routes</span></a>"""


HOME_HTML = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#080b0d">
<title>Carrot Routes</title><style>__COMMON_STYLE__</style></head><body><main class="shell">
<header class="topbar">__BRAND__<span class="grow"></span><span class="badge"><span class="badge-dot"></span>Private vault</span></header>
<section class="hero"><div class="hero-grid"><div><div class="eyebrow">Private driving archive</div><h1>나의 주행 기록을<br>안전하게, 필요한 만큼만.</h1><p class="lede">공유받은 링크가 있다면 그 링크로 바로 접속하세요. 이 첫 화면에서는 어떤 route나 파일도 공개하지 않습니다.</p><div class="row" style="margin-top:27px"><a class="button" href="/admin">관리자 보관함 열기 →</a><span class="muted tiny">관리자 키 필요</span></div></div>
<aside class="card privacy-card"><div class="privacy-icon">◆</div><h2>링크 기반 비공개 공유</h2><p class="muted">각 공유 링크는 선택한 route 하나에만 연결됩니다.</p><ul class="privacy-list"><li>공개 route 목록 없음</li><li>원본 로그와 영상 Range 전송</li><li>Cabana · PlotJuggler 호환</li></ul></aside></div></section>
<footer class="footer"><span>Self-hosted on Synology</span><span>Carrot Routes · private by design</span></footer></main></body></html>""".replace("__COMMON_STYLE__", COMMON_STYLE).replace("__BRAND__", BRAND)


ADMIN_HTML = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#080b0d">
<title>Carrot Routes Admin</title><style>__COMMON_STYLE__</style></head><body><main class="shell">
<header class="topbar">__BRAND__<span class="grow"></span><span class="badge"><span class="badge-dot"></span>Owner access</span></header>
<section id="login" class="login-wrap"><div class="card login-card"><div class="brand-mark login-mark"><svg viewBox="0 0 24 24"><path fill="#1b130d" d="M14.9 5.2c1.3-2.5 3.7-3.4 5.8-3.1-.2 2.2-1.6 4.1-4.2 4.8 2.1.3 3.5 1.3 4.5 2.6-2 1-4.4.8-6.3-.9C13.9 15.4 11.1 21 8.4 21c-2.5 0-5.2-4.5-5.2-7.8 0-3.4 2.8-5.9 6.1-5.9 1.6 0 3.1.6 4.2 1.7.2-1.4.7-2.7 1.4-3.8Z"/></svg></div><div class="eyebrow">Owner only</div><h1>나의 Routes</h1><p class="muted">관리자 키로 전체 주행 기록과 공유 링크를 관리합니다.</p><form id="loginForm"><label>관리자 키<input id="key" type="password" autocomplete="current-password" placeholder="CARROT_ROUTE_ADMIN_KEY" autofocus></label><button id="loginButton">보관함 열기 →</button></form><div id="loginStatus" class="error tiny"></div><div class="login-note"><span>◆</span><span>키는 브라우저 저장소에 보관되지 않으며, 로그인 쿠키는 12시간 뒤 만료됩니다.</span></div></div></section>
<section id="catalog" class="hidden"><div class="admin-head"><div><div class="eyebrow">Synology archive</div><h1>Route 보관함</h1><div id="catalogMeta" class="muted">불러오는 중…</div></div><button id="logout" class="ghost">로그아웃</button></div>
<div class="stats"><div class="card stat"><strong id="routeCount">—</strong><span>전체 routes</span></div><div class="card stat"><strong id="segmentCount">—</strong><span>전체 segments</span></div><div class="card stat"><strong>Private</strong><span>링크를 만든 route만 공유</span></div></div>
<div class="toolbar"><div class="search-wrap"><input id="search" type="search" placeholder="날짜, 차량, dongle ID 검색"></div><button id="refresh" class="secondary">새로고침</button><button id="mobileLogout" class="ghost">로그아웃</button></div>
<nav class="tabs"><button class="tab active" data-tab="routes">Routes</button><button class="tab" data-tab="shares">공유 링크</button></nav><section id="routesPanel"><div id="routes" class="route-list"></div></section><section id="sharesPanel" class="hidden"><div id="shares" class="share-list"></div></section><div id="status" class="tiny muted" style="margin-top:14px"></div></section>
</main>
<section id="detail" class="drawer hidden"><div class="drawer-panel"><div class="drawer-head"><div class="grow"><div class="eyebrow">Route detail</div><h2 id="detailTitle"></h2><div id="detailMeta" class="muted tiny"></div></div><button id="closeDetail" class="ghost icon-button" aria-label="닫기">×</button></div><div class="video-stage"><span id="videoEmpty" class="video-empty">세그먼트의 재생 버튼을 선택하세요</span><video id="video" controls playsinline preload="metadata"></video></div><div id="segments"></div></div></section>
<section id="shareModal" class="modal hidden"><div class="card modal-card"><div class="eyebrow">Create secure link</div><h2>Route 공유</h2><p id="shareTarget" class="muted"></p><label>링크 만료<select id="expires"><option value="7">7일</option><option value="30" selected>30일</option><option value="90">90일</option><option value="0">만료 없음</option></select></label><div class="notice" style="margin-top:15px">링크를 아는 사람은 이 route의 로그와 영상을 볼 수 있습니다. 다른 route에는 접근할 수 없습니다.</div><div class="modal-actions"><button id="cancelShare" class="ghost">취소</button><button id="createShare">공유 링크 만들기</button></div></div></section>
<section id="shareResult" class="modal hidden"><div class="card modal-card"><div class="eyebrow">Link ready</div><h2>공유 링크가 준비됐습니다</h2><p class="muted">이 창을 닫으면 전체 토큰은 다시 표시되지 않습니다.</p><label>공유 URL<div id="shareUrl" class="command share-url"><span id="shareUrlText"></span><button class="copy" data-copy-target="shareUrlText" aria-label="복사">⧉</button></div></label><div id="toolCommands"></div><div class="modal-actions"><a id="shareLink" class="button secondary" target="_blank" rel="noreferrer">공유 화면 열기</a><button id="closeShare">완료</button></div></div></section>
<script>
const $=s=>document.querySelector(s);let routesData=[],shareTarget=null;
const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const routeDate=r=>{const [date,time]=r.split('--');return `${date} · ${(time||'').replaceAll('-' , ':')}`};
const pack=(d,r)=>encodeURIComponent(JSON.stringify([d,r]));
async function api(path,opt={}){const r=await fetch(path,{credentials:'same-origin',...opt,headers:{'Content-Type':'application/json',...(opt.headers||{})}});if(!r.ok){const body=await r.json().catch(()=>({}));throw new Error(body.error||`HTTP ${r.status}`)}return r.json()}
function setBusy(button,busy){button.disabled=busy;button.dataset.label??=button.textContent;button.textContent=busy?'처리 중…':button.dataset.label}
function showError(error){$('#status').textContent=error.message||String(error);$('#status').className='tiny error'}
function renderRoutes(){const query=$('#search').value.trim().toLowerCase();const found=routesData.filter(r=>`${r.route} ${r.directory} ${r.deviceId}`.toLowerCase().includes(query));$('#routes').innerHTML=found.map(r=>`<article class="card route-card"><div class="route-date">${esc(routeDate(r.route))}</div><div class="route-id">${esc(r.route)}</div><div class="route-meta"><span class="pill">${r.segmentCount} segments</span><span class="pill">${esc(r.directory)}</span>${r.toolCompatible?'<span class="pill">Tools ready</span>':''}</div><div class="route-actions"><button class="secondary" data-view="${pack(r.directory,r.route)}">내용 보기</button><button data-share="${pack(r.directory,r.route)}">링크 공유</button></div></article>`).join('')||'<div class="empty">검색 조건에 맞는 route가 없습니다.</div>'}
async function load(){try{const data=await api('/api/admin/routes');routesData=data.routes;$('#login').classList.add('hidden');$('#catalog').classList.remove('hidden');$('#routeCount').textContent=data.total.toLocaleString();$('#segmentCount').textContent=data.routes.reduce((n,r)=>n+r.segmentCount,0).toLocaleString();$('#catalogMeta').textContent=`최근 업로드 ${data.routes[0]?new Date(data.routes[0].modifiedEpoch*1000).toLocaleString():'없음'}`;renderRoutes();$('#status').textContent=''}catch(e){$('#login').classList.remove('hidden');$('#catalog').classList.add('hidden');$('#loginStatus').textContent=e.message}}
async function detail(d,r){const data=await api(`/api/admin/route?directory=${encodeURIComponent(d)}&route=${encodeURIComponent(r)}`);$('#detail').classList.remove('hidden');$('#detailTitle').textContent=routeDate(r);$('#detailMeta').textContent=`${data.segmentCount} segments · ${data.totalSizeLabel} · ${data.deviceId}`;$('#video').removeAttribute('src');$('#video').load();$('#videoEmpty').classList.remove('hidden');$('#segments').innerHTML=data.segments.map((s,i)=>`<details class="soft-card segment" ${i===0?'open':''}><summary><strong class="grow">Segment ${s.index}</strong><span class="pill">${s.files.length} files</span>${s.videoUrl?`<button class="secondary" data-play="${esc(s.videoUrl)}">▶ 재생</button>`:''}</summary><div class="segment-body"><div class="files">${s.files.map(f=>`<a class="file" href="${esc(f.url)}" target="_blank" rel="noreferrer"><span>${esc(f.name)}</span><span class="file-size">${esc(f.sizeLabel)}</span></a>`).join('')}</div></div></details>`).join('')}
function openShare(d,r){shareTarget=[d,r];$('#shareTarget').textContent=`${routeDate(r)} · ${d}`;$('#shareModal').classList.remove('hidden')}
async function createShare(){if(!shareTarget)return;const button=$('#createShare');setBusy(button,true);try{const [directory,route]=shareTarget;const data=await api('/api/admin/shares',{method:'POST',body:JSON.stringify({directory,route,expiresDays:Number($('#expires').value)})});$('#shareModal').classList.add('hidden');$('#shareResult').classList.remove('hidden');$('#shareLink').href=data.shareUrl;$('#shareUrlText').textContent=data.shareUrl;const commands=[['Cabana',data.cabanaCommand],['PlotJuggler',data.plotJugglerCommand]].filter(x=>x[1]);$('#toolCommands').innerHTML=commands.map(([name,command],i)=>`<label>${name}<div id="command${i}" class="command">${esc(command)}<button class="copy" data-copy-target="command${i}" aria-label="복사">⧉</button></div></label>`).join('');await loadShares()}finally{setBusy(button,false)}}
async function loadShares(){const data=await api('/api/admin/shares');$('#shares').innerHTML=data.shares.map(s=>`<article class="soft-card share-row"><div><div class="row"><strong>${esc(routeDate(s.route))}</strong><span class="status-chip ${s.active?'':'off'}">${s.active?'활성':'종료'}</span></div><div class="muted tiny">${esc(s.directory)} · 생성 ${new Date(s.createdAt*1000).toLocaleString()}${s.expiresAt?` · 만료 ${new Date(s.expiresAt*1000).toLocaleDateString()}`:' · 만료 없음'}</div></div>${s.active?`<button class="danger" data-revoke="${esc(s.id)}">링크 폐기</button>`:''}</article>`).join('')||'<div class="empty">생성된 공유 링크가 없습니다.</div>'}
$('#loginForm').addEventListener('submit',async e=>{e.preventDefault();const button=$('#loginButton');setBusy(button,true);try{await api('/api/admin/login',{method:'POST',body:JSON.stringify({key:$('#key').value})});$('#key').value='';await load()}catch(err){$('#loginStatus').textContent=err.message}finally{setBusy(button,false)}});
$('#routes').addEventListener('click',e=>{const view=e.target.closest('[data-view]'),share=e.target.closest('[data-share]');if(view)detail(...JSON.parse(decodeURIComponent(view.dataset.view))).catch(showError);if(share)openShare(...JSON.parse(decodeURIComponent(share.dataset.share)))});
$('#segments').addEventListener('click',e=>{const b=e.target.closest('[data-play]');if(b){e.preventDefault();$('#video').src=b.dataset.play;$('#videoEmpty').classList.add('hidden');$('#video').play().catch(()=>{})}});
$('#shares').addEventListener('click',async e=>{const b=e.target.closest('[data-revoke]');if(!b||!confirm('이 공유 링크를 즉시 폐기할까요?'))return;setBusy(b,true);try{await api(`/api/admin/shares/${encodeURIComponent(b.dataset.revoke)}/revoke`,{method:'POST',body:'{}'});await loadShares()}catch(err){showError(err);setBusy(b,false)}});
document.addEventListener('click',e=>{const b=e.target.closest('[data-copy-target]');if(!b)return;const target=$(`#${b.dataset.copyTarget}`);const text=[...target.childNodes].filter(n=>n.nodeType===Node.TEXT_NODE).map(n=>n.textContent).join('').trim();navigator.clipboard.writeText(text).then(()=>{b.textContent='✓';setTimeout(()=>b.textContent='⧉',1200)})});
document.querySelectorAll('[data-tab]').forEach(tab=>tab.onclick=()=>{document.querySelectorAll('[data-tab]').forEach(x=>x.classList.toggle('active',x===tab));const routes=tab.dataset.tab==='routes';$('#routesPanel').classList.toggle('hidden',!routes);$('#sharesPanel').classList.toggle('hidden',routes);if(!routes)loadShares().catch(showError)});
$('#search').oninput=renderRoutes;$('#refresh').onclick=()=>load();async function logout(){await api('/api/admin/logout',{method:'POST',body:'{}'});location.reload()}$('#logout').onclick=logout;$('#mobileLogout').onclick=logout;$('#closeDetail').onclick=()=>{$('#video').pause();$('#detail').classList.add('hidden')};$('#detail').onclick=e=>{if(e.target===$('#detail'))$('#closeDetail').click()};$('#cancelShare').onclick=()=>$('#shareModal').classList.add('hidden');$('#shareModal').onclick=e=>{if(e.target===$('#shareModal'))$('#cancelShare').click()};$('#createShare').onclick=()=>createShare().catch(showError);$('#closeShare').onclick=()=>$('#shareResult').classList.add('hidden');load();
</script></body></html>""".replace("__COMMON_STYLE__", COMMON_STYLE).replace("__BRAND__", BRAND)


SHARE_HTML = """<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#080b0d">
<title>Shared Carrot Route · __ROUTE_TITLE__</title><style>__COMMON_STYLE__</style></head><body><main class="shell">
<header class="topbar">__BRAND__<span class="grow"></span><span class="badge"><span class="badge-dot"></span>Shared route</span></header>
<section class="share-hero"><div class="eyebrow">Driving archive</div><h1 id="title">Shared Route</h1><p id="meta" class="lede">route 정보를 불러오는 중…</p><div id="summary" class="summary-grid hidden"></div></section>
<section class="card card-pad"><div class="video-stage" style="margin:0"><span id="videoEmpty" class="video-empty">아래 세그먼트에서 영상을 선택하세요</span><video id="video" controls playsinline preload="metadata"></video></div></section>
<section id="tools" class="hidden"><div class="section-head"><div><div class="eyebrow">Open with</div><h2>분석 도구 연결</h2></div><span class="muted tiny">도구 소스 수정 없이 API_HOST로 연결</span></div><div class="notice" style="margin-bottom:12px">명령을 복사해 openpilot 저장소에서 실행하세요. 이 공유 링크가 허용한 route 하나만 조회됩니다.</div><div id="commands" class="tools-grid"></div></section>
<section><div class="section-head"><div><div class="eyebrow">Route contents</div><h2>세그먼트와 파일</h2></div><span id="fileCount" class="muted tiny"></span></div><div id="segments" class="public-segments"></div></section><div id="status" class="muted"></div>
<footer class="footer"><span>링크를 전달받은 사람만 접근할 수 있습니다.</span><span>Carrot Routes · hosted on Synology</span></footer></main>
<script>
const $=s=>document.querySelector(s);const esc=v=>String(v??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fileKind=n=>n.includes('camera')?'VIDEO':n.includes('log')?'LOG':'FILE';
async function copy(text,button){await navigator.clipboard.writeText(text);button.textContent='복사됨';setTimeout(()=>button.textContent='복사',1300)}
async function load(){try{const r=await fetch(location.pathname+'/manifest');if(!r.ok)throw new Error(r.status===404?'공유 링크가 만료되었거나 폐기되었습니다.':`HTTP ${r.status}`);const d=await r.json();$('#title').textContent=d.route;$('#meta').textContent=`${d.deviceId} · 안전하게 공유된 단일 route`;$('#summary').classList.remove('hidden');$('#summary').innerHTML=`<div class="card summary-item"><strong>${d.segmentCount}</strong><span>SEGMENTS</span></div><div class="card summary-item"><strong>${esc(d.totalSizeLabel)}</strong><span>ROUTE SIZE</span></div><div class="card summary-item"><strong>${d.firstSegment}–${d.lastSegment}</strong><span>SEGMENT RANGE</span></div>`;const commands=[['Cabana','CAN 메시지를 탐색하고 신호를 분석합니다.',d.cabanaCommand],['PlotJuggler','시간축으로 주행 데이터를 시각화합니다.',d.plotJugglerCommand]].filter(x=>x[2]);if(commands.length){$('#tools').classList.remove('hidden');$('#commands').innerHTML=commands.map(([name,desc,command],i)=>`<article class="card tool-card"><div class="tool-name"><span class="tool-mark">${name==='Cabana'?'C':'P'}</span>${name}</div><p class="muted tiny" style="margin:10px 0">${desc}</p><div class="command" id="cmd${i}">${esc(command)}<button class="copy" data-command="${encodeURIComponent(command)}">복사</button></div></article>`).join('')}const totalFiles=d.segments.reduce((n,s)=>n+s.files.length,0);$('#fileCount').textContent=`${totalFiles} files`;$('#segments').innerHTML=d.segments.map((s,i)=>`<details class="soft-card segment" ${i===0?'open':''}><summary><strong class="grow">Segment ${s.index}</strong><span class="pill">${s.files.length} files</span>${s.videoUrl?`<button data-play="${esc(s.videoUrl)}">▶ 영상 재생</button>`:''}</summary><div class="segment-body"><div class="files">${s.files.map(f=>`<a class="file" href="${esc(f.url)}" target="_blank" rel="noreferrer"><span class="tiny muted">${fileKind(f.name)}</span><span>${esc(f.name)}</span><span class="file-size">${esc(f.sizeLabel)}</span></a>`).join('')}</div></div></details>`).join('')}catch(e){$('#status').textContent=e.message;$('#status').className='card card-pad error';$('#segments').innerHTML=''}}
document.addEventListener('click',e=>{const play=e.target.closest('[data-play]');if(play){e.preventDefault();$('#video').src=play.dataset.play;$('#videoEmpty').classList.add('hidden');$('#video').play().catch(()=>{});scrollTo({top:0,behavior:'smooth'})}const button=e.target.closest('[data-command]');if(button)copy(decodeURIComponent(button.dataset.command),button)});load();
</script></body></html>""".replace("__COMMON_STYLE__", COMMON_STYLE).replace("__BRAND__", BRAND)
