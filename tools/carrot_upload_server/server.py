from __future__ import annotations

import asyncio
import hashlib
import ipaddress
import json
import logging
import os
import re
import secrets
import shutil
import sqlite3
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from aiohttp import web


GIB = 1024 * 1024 * 1024
MIB = 1024 * 1024
DEVICE_RE = re.compile(r"^[A-Za-z0-9_-]{8,64}$")
SEGMENT_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.|-]{0,127}$")
FILENAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")
BLOCKED_EXTENSIONS = {
  ".apk", ".bat", ".cgi", ".cmd", ".com", ".dll", ".exe", ".html",
  ".htm", ".jar", ".js", ".php", ".ps1", ".py", ".sh", ".so",
}
KST = timezone(timedelta(hours=9))


def _env_int(name: str, default: int, minimum: int = 0) -> int:
  try:
    return max(minimum, int(os.environ.get(name, str(default))))
  except Exception:
    return default


@dataclass(frozen=True)
class Config:
  storage_root: Path
  db_path: Path
  daily_device_quota: int = GIB
  daily_ip_quota: int = 8 * GIB
  max_file_bytes: int = 512 * MIB
  max_tmux_bytes: int = 16 * MIB
  min_free_bytes: int = 10 * GIB
  session_ttl_seconds: int = 4 * 60 * 60
  concurrent_per_device: int = 3
  concurrent_global: int = 16
  session_issue_limit: int = 30
  session_issue_window_seconds: int = 10 * 60
  trusted_proxy_networks: tuple[str, ...] = ("127.0.0.0/8", "::1/128", "172.16.0.0/12")

  @classmethod
  def from_env(cls) -> Config:
    root = Path(os.environ.get("CARROT_UPLOAD_ROOT", "/data/uploads"))
    return cls(
      storage_root=root,
      db_path=Path(os.environ.get("CARROT_UPLOAD_DB", "/data/state/uploads.sqlite3")),
      daily_device_quota=_env_int("CARROT_DAILY_DEVICE_QUOTA_BYTES", GIB, MIB),
      daily_ip_quota=_env_int("CARROT_DAILY_IP_QUOTA_BYTES", 8 * GIB, MIB),
      max_file_bytes=_env_int("CARROT_MAX_FILE_BYTES", 512 * MIB, MIB),
      max_tmux_bytes=_env_int("CARROT_MAX_TMUX_BYTES", 16 * MIB, MIB),
      min_free_bytes=_env_int("CARROT_MIN_FREE_BYTES", 10 * GIB),
      session_ttl_seconds=_env_int("CARROT_SESSION_TTL_SECONDS", 4 * 60 * 60, 60),
      concurrent_per_device=_env_int("CARROT_CONCURRENT_PER_DEVICE", 3, 1),
      concurrent_global=_env_int("CARROT_CONCURRENT_GLOBAL", 16, 1),
    )


class UploadService:
  def __init__(self, config: Config):
    self.config = config
    self.config.storage_root.mkdir(parents=True, exist_ok=True)
    self.config.db_path.parent.mkdir(parents=True, exist_ok=True)
    self._db_lock = asyncio.Lock()
    self._active_lock = asyncio.Lock()
    self._active_global = 0
    self._active_by_device: dict[str, int] = defaultdict(int)
    self._session_issues: dict[str, deque[float]] = defaultdict(deque)
    self._trusted_proxies = tuple(ipaddress.ip_network(value) for value in config.trusted_proxy_networks)
    self._init_db()

  def _connect(self) -> sqlite3.Connection:
    connection = sqlite3.connect(self.config.db_path, timeout=30)
    connection.row_factory = sqlite3.Row
    connection.execute("PRAGMA journal_mode=WAL")
    connection.execute("PRAGMA foreign_keys=ON")
    return connection

  def _init_db(self) -> None:
    with self._connect() as connection:
      connection.executescript(
        """
        CREATE TABLE IF NOT EXISTS sessions (
          token_hash TEXT PRIMARY KEY,
          device_id TEXT NOT NULL,
          source_ip TEXT NOT NULL,
          purpose TEXT NOT NULL,
          car_name TEXT NOT NULL DEFAULT 'none',
          git_branch TEXT NOT NULL DEFAULT 'unknown',
          tmux_reason TEXT NOT NULL DEFAULT 'tmux',
          created_at INTEGER NOT NULL,
          expires_at INTEGER NOT NULL
        );
        CREATE INDEX IF NOT EXISTS sessions_expiry ON sessions(expires_at);
        CREATE TABLE IF NOT EXISTS daily_usage (
          usage_day TEXT NOT NULL,
          scope TEXT NOT NULL,
          scope_id TEXT NOT NULL,
          committed_bytes INTEGER NOT NULL DEFAULT 0,
          reserved_bytes INTEGER NOT NULL DEFAULT 0,
          PRIMARY KEY (usage_day, scope, scope_id)
        );
        """
      )
      columns = {str(row[1]) for row in connection.execute("PRAGMA table_info(sessions)")}
      for name, definition in (
        ("car_name", "TEXT NOT NULL DEFAULT 'none'"),
        ("git_branch", "TEXT NOT NULL DEFAULT 'unknown'"),
        ("tmux_reason", "TEXT NOT NULL DEFAULT 'tmux'"),
      ):
        if name not in columns:
          connection.execute(f"ALTER TABLE sessions ADD COLUMN {name} {definition}")

  @staticmethod
  def _token_hash(token: str) -> str:
    return hashlib.sha256(token.encode("utf-8")).hexdigest()

  @staticmethod
  def _usage_day() -> str:
    return datetime.now(UTC).strftime("%Y-%m-%d")

  @staticmethod
  def _clean_metadata(value: Any) -> str:
    text = str(value or "").strip().replace("\r", " ").replace("\n", " ")
    return text[:160]

  @classmethod
  def _storage_component(cls, value: Any, default: str, *, branch: bool = False) -> str:
    text = cls._clean_metadata(value)
    if branch:
      text = text.replace("/", "__").replace("\\", "__")
    text = re.sub(r"[^A-Za-z0-9_. -]+", "_", text).strip(" .")[:96]
    return text if text and text not in {".", ".."} else default

  @staticmethod
  def _storage_directory(session: sqlite3.Row) -> str:
    return f"{session['car_name']} {session['device_id']}"

  def source_ip(self, request: web.Request) -> str:
    remote = str(request.remote or "unknown")
    try:
      remote_ip = ipaddress.ip_address(remote)
      trusted = any(remote_ip in network for network in self._trusted_proxies)
    except ValueError:
      trusted = False
    if trusted:
      # DSM/nginx appends the real client to any inbound X-Forwarded-For
      # values. Walk from the proxy-facing end so a caller cannot spoof the
      # first value to evade IP quotas or session binding.
      forwarded = request.headers.get("X-Forwarded-For", "")
      for value in reversed(forwarded.split(",")):
        try:
          candidate = ipaddress.ip_address(value.strip())
        except ValueError:
          continue
        if not any(candidate in network for network in self._trusted_proxies):
          return str(candidate)
    return remote[:64]

  def _validate_device(self, value: Any) -> str:
    device = str(value or "").strip()
    if not DEVICE_RE.fullmatch(device) or device.lower() in {"unknown", "none"}:
      raise web.HTTPBadRequest(text="invalid device id")
    return device

  @staticmethod
  def _validate_segment(value: Any) -> str:
    segment = str(value or "").strip()
    if not SEGMENT_RE.fullmatch(segment) or segment in {".", ".."}:
      raise web.HTTPBadRequest(text="invalid segment")
    return segment

  @staticmethod
  def _validate_filename(value: Any) -> str:
    filename = str(value or "").strip()
    if not FILENAME_RE.fullmatch(filename) or filename in {".", ".."}:
      raise web.HTTPBadRequest(text="invalid filename")
    if Path(filename).suffix.lower() in BLOCKED_EXTENSIONS:
      raise web.HTTPBadRequest(text="file type is not allowed")
    return filename

  def _path(self, *parts: str) -> Path:
    root = self.config.storage_root.resolve()
    path = root.joinpath(*parts).resolve()
    if path != root and root not in path.parents:
      raise web.HTTPBadRequest(text="invalid upload path")
    return path

  def _has_disk_space(self, expected: int = 0) -> bool:
    free = shutil.disk_usage(self.config.storage_root).free
    return free - max(0, expected) >= self.config.min_free_bytes

  def _check_session_rate(self, source_ip: str) -> None:
    now = time.monotonic()
    cutoff = now - self.config.session_issue_window_seconds
    bucket = self._session_issues[source_ip]
    while bucket and bucket[0] < cutoff:
      bucket.popleft()
    if len(bucket) >= self.config.session_issue_limit:
      raise web.HTTPTooManyRequests(text="too many session requests")
    bucket.append(now)

  @staticmethod
  async def _json_body(request: web.Request, limit: int) -> Any:
    content_length = request.content_length
    if content_length is not None and content_length > limit:
      raise web.HTTPRequestEntityTooLarge(max_size=limit, actual_size=content_length)
    data = bytearray()
    async for chunk in request.content.iter_chunked(64 * 1024):
      data.extend(chunk)
      if len(data) > limit:
        raise web.HTTPRequestEntityTooLarge(max_size=limit, actual_size=len(data))
    try:
      return json.loads(data.decode("utf-8"))
    except Exception as exc:
      raise web.HTTPBadRequest(text="invalid JSON") from exc

  async def create_session(self, request: web.Request) -> web.Response:
    body = await self._json_body(request, 16 * 1024)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    device = self._validate_device(body.get("deviceId") or body.get("dongleId"))
    purpose = str(body.get("purpose") or "upload").strip().lower()
    if purpose not in {"dashcam", "tmux", "test"}:
      raise web.HTTPBadRequest(text="invalid purpose")
    car_name = self._storage_component(body.get("carName") or body.get("car_name"), "none")
    git_branch = self._storage_component(
      body.get("branch") or body.get("gitBranch") or body.get("git_branch"), "unknown", branch=True,
    )
    tmux_reason = self._storage_component(body.get("tmux_why") or body.get("reason"), "tmux")
    source_ip = self.source_ip(request)
    self._check_session_rate(source_ip)
    token = secrets.token_urlsafe(32)
    now = int(datetime.now(UTC).timestamp())
    expires = now + self.config.session_ttl_seconds
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("DELETE FROM sessions WHERE expires_at < ?", (now,))
        connection.execute(
          """INSERT INTO sessions(
               token_hash, device_id, source_ip, purpose, car_name, git_branch, tmux_reason, created_at, expires_at
             ) VALUES(?,?,?,?,?,?,?,?,?)""",
          (self._token_hash(token), device, source_ip, purpose, car_name, git_branch, tmux_reason, now, expires),
        )
    return web.json_response({
      "ok": True,
      "token": token,
      "expiresAt": expires,
      "expiresIn": self.config.session_ttl_seconds,
      "dailyQuotaBytes": self.config.daily_device_quota,
      "maxFileBytes": self.config.max_file_bytes,
    })

  async def authenticate(
    self,
    request: web.Request,
    *,
    device: str | None = None,
    purposes: set[str] | None = None,
  ) -> sqlite3.Row:
    authorization = request.headers.get("Authorization", "")
    if not authorization.startswith("Bearer "):
      raise web.HTTPUnauthorized(text="missing upload session")
    token = authorization[7:].strip()
    if not token:
      raise web.HTTPUnauthorized(text="missing upload session")
    async with self._db_lock:
      with self._connect() as connection:
        row = connection.execute(
          "SELECT * FROM sessions WHERE token_hash = ?", (self._token_hash(token),),
        ).fetchone()
    if row is None or int(row["expires_at"]) < int(datetime.now(UTC).timestamp()):
      raise web.HTTPUnauthorized(text="expired upload session")
    if row["source_ip"] != self.source_ip(request):
      raise web.HTTPForbidden(text="upload session IP mismatch")
    if device is not None and row["device_id"] != device:
      raise web.HTTPForbidden(text="upload session device mismatch")
    if purposes is not None and row["purpose"] not in purposes:
      raise web.HTTPForbidden(text="upload session purpose mismatch")
    return row

  async def _reserve(self, device: str, source_ip: str, amount: int) -> str:
    day = self._usage_day()
    amount = max(0, int(amount))
    async with self._db_lock:
      with self._connect() as connection:
        connection.execute("BEGIN IMMEDIATE")
        for scope, scope_id, quota in (
          ("device", device, self.config.daily_device_quota),
          ("ip", source_ip, self.config.daily_ip_quota),
        ):
          connection.execute(
            "INSERT OR IGNORE INTO daily_usage(usage_day, scope, scope_id) VALUES(?,?,?)",
            (day, scope, scope_id),
          )
          row = connection.execute(
            "SELECT committed_bytes, reserved_bytes FROM daily_usage WHERE usage_day=? AND scope=? AND scope_id=?",
            (day, scope, scope_id),
          ).fetchone()
          if int(row["committed_bytes"]) + int(row["reserved_bytes"]) + amount > quota:
            raise web.HTTPRequestEntityTooLarge(max_size=quota, actual_size=int(row["committed_bytes"]) + amount)
        for scope, scope_id, _quota in (("device", device, 0), ("ip", source_ip, 0)):
          connection.execute(
            "UPDATE daily_usage SET reserved_bytes=reserved_bytes+? WHERE usage_day=? AND scope=? AND scope_id=?",
            (amount, day, scope, scope_id),
          )
    return day

  async def _finish_reservation(
    self, device: str, source_ip: str, day: str, reserved: int, committed_delta: int,
  ) -> None:
    async with self._db_lock:
      with self._connect() as connection:
        for scope, scope_id in (("device", device), ("ip", source_ip)):
          connection.execute(
            """UPDATE daily_usage
               SET reserved_bytes=MAX(0, reserved_bytes-?),
                   committed_bytes=MAX(0, committed_bytes+?)
               WHERE usage_day=? AND scope=? AND scope_id=?""",
            (reserved, committed_delta, day, scope, scope_id),
          )

  async def _enter_upload(self, device: str) -> None:
    async with self._active_lock:
      if self._active_global >= self.config.concurrent_global:
        raise web.HTTPServiceUnavailable(text="upload server is busy")
      if self._active_by_device[device] >= self.config.concurrent_per_device:
        raise web.HTTPTooManyRequests(text="too many concurrent uploads for device")
      self._active_global += 1
      self._active_by_device[device] += 1

  async def _leave_upload(self, device: str) -> None:
    async with self._active_lock:
      self._active_global = max(0, self._active_global - 1)
      self._active_by_device[device] = max(0, self._active_by_device[device] - 1)
      if not self._active_by_device[device]:
        self._active_by_device.pop(device, None)

  async def health(self, _request: web.Request) -> web.Response:
    ready = self._has_disk_space()
    return web.json_response({
      "ok": ready,
      "service": "carrot-upload",
      "dailyQuotaBytes": self.config.daily_device_quota,
      "maxFileBytes": self.config.max_file_bytes,
      "bandwidthLimit": None,
    }, status=200 if ready else 507)

  async def upload_file(self, request: web.Request) -> web.Response:
    device = self._validate_device(request.match_info.get("device"))
    segment = self._validate_segment(request.match_info.get("segment"))
    filename = self._validate_filename(request.match_info.get("filename"))
    session = await self.authenticate(request, device=device, purposes={"dashcam"})
    try:
      expected = int(request.headers.get("X-File-Size", ""))
    except ValueError as exc:
      raise web.HTTPLengthRequired(text="X-File-Size is required") from exc
    if expected < 0 or expected > self.config.max_file_bytes:
      raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_file_bytes, actual_size=expected)
    if not self._has_disk_space(expected):
      raise web.HTTPInsufficientStorage(text="not enough free storage")

    target = self._path("routes", self._storage_directory(session), segment, filename)
    target.parent.mkdir(parents=True, exist_ok=True)
    # The daily quota measures network usage, not the final storage delta.
    # Re-uploading the same path must therefore consume the full request size
    # again instead of allowing repeated overwrites to bypass the limit.
    reserved = expected
    await self._enter_upload(device)
    reservation_created = False
    reservation_day = self._usage_day()
    temp = target.with_name(f".{filename}.{secrets.token_hex(8)}.part")
    try:
      reservation_day = await self._reserve(device, session["source_ip"], reserved)
      reservation_created = True
      written = 0
      with temp.open("xb") as output:
        async for chunk in request.content.iter_chunked(MIB):
          written += len(chunk)
          if written > expected or written > self.config.max_file_bytes:
            raise web.HTTPRequestEntityTooLarge(max_size=expected, actual_size=written)
          output.write(chunk)
        output.flush()
        os.fsync(output.fileno())
      if written != expected:
        raise web.HTTPBadRequest(text=f"file size mismatch: expected {expected}, received {written}")
      os.replace(temp, target)
      await self._finish_reservation(device, session["source_ip"], reservation_day, reserved, written)
      reservation_created = False
      return web.json_response({"ok": True, "size": written})
    finally:
      if temp.exists():
        temp.unlink(missing_ok=True)
      if reservation_created:
        await self._finish_reservation(device, session["source_ip"], reservation_day, reserved, 0)
      await self._leave_upload(device)

  async def complete(self, request: web.Request) -> web.Response:
    session = await self.authenticate(request, purposes={"dashcam"})
    body = await self._json_body(request, 512 * 1024)
    if not isinstance(body, dict):
      raise web.HTTPBadRequest(text="JSON object is required")
    meta = body.get("meta") if isinstance(body, dict) else {}
    meta_device = str((meta or {}).get("dongleId") or "").strip()
    if meta_device.lower() in {"", "unknown", "none"}:
      meta_device = ""
    body_device = meta_device or str(body.get("deviceId") or "").strip()
    if body_device and body_device != session["device_id"]:
      raise web.HTTPForbidden(text="completion device mismatch")
    state_root = self.config.db_path.parent.resolve()
    directory = state_root / "manifests" / session["device_id"]
    if state_root not in directory.resolve().parents:
      raise web.HTTPBadRequest(text="invalid manifest path")
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
    path = directory / f"{stamp}.json"
    safe_body = dict(body) if isinstance(body, dict) else {}
    safe_body["receivedAt"] = datetime.now(UTC).isoformat()
    encoded = json.dumps(safe_body, ensure_ascii=False, indent=2).encode("utf-8")
    reservation_day = await self._reserve(session["device_id"], session["source_ip"], len(encoded))
    reserved = True
    temp = path.with_suffix(".json.part")
    try:
      with temp.open("xb") as output:
        output.write(encoded)
        output.flush()
        os.fsync(output.fileno())
      os.replace(temp, path)
      await self._finish_reservation(
        session["device_id"], session["source_ip"], reservation_day, len(encoded), len(encoded),
      )
      reserved = False
      return web.json_response({"ok": True})
    finally:
      temp.unlink(missing_ok=True)
      if reserved:
        await self._finish_reservation(
          session["device_id"], session["source_ip"], reservation_day, len(encoded), 0,
        )

  async def tmux_upload(self, request: web.Request) -> web.Response:
    session = await self.authenticate(request, purposes={"tmux"})
    device = session["device_id"]
    if not self._has_disk_space(self.config.max_tmux_bytes):
      raise web.HTTPInsufficientStorage(text="not enough free storage")
    await self._enter_upload(device)
    destination = self._path("tmux", session["git_branch"], self._storage_directory(session))
    destination.mkdir(parents=True, exist_ok=True)
    temp_dir = destination / f".carrot-incoming-{secrets.token_hex(12)}"
    temp_dir.mkdir(parents=True, exist_ok=False)
    total = 0
    metadata: dict[str, str] = {}
    files: list[tuple[str, Path, int]] = []
    reserved = 0
    reservation_created = False
    reservation_day = self._usage_day()
    try:
      reader = await request.multipart()
      async for part in reader:
        if part.filename:
          filename = "tmux.log" if part.name == "files[0]" else "toggle_values.json" if part.name == "files[1]" else ""
          if not filename:
            raise web.HTTPBadRequest(text="unexpected upload file")
          path = temp_dir / filename
          size = 0
          with path.open("xb") as output:
            while chunk := await part.read_chunk(MIB):
              size += len(chunk)
              total += len(chunk)
              if total > self.config.max_tmux_bytes:
                raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_tmux_bytes, actual_size=total)
              output.write(chunk)
          files.append((filename, path, size))
        else:
          value = bytearray()
          while chunk := await part.read_chunk(4096):
            value.extend(chunk)
            total += len(chunk)
            if len(value) > 4096 or total > self.config.max_tmux_bytes:
              raise web.HTTPRequestEntityTooLarge(max_size=self.config.max_tmux_bytes, actual_size=total)
          metadata[str(part.name or "")[:64]] = self._clean_metadata(value.decode("utf-8", errors="replace"))
      if not files or files[0][0] != "tmux.log":
        raise web.HTTPBadRequest(text="tmux.log is required")
      reservation_day = await self._reserve(device, session["source_ip"], total)
      reserved = total
      reservation_created = True
      stamp = datetime.now(KST).strftime("%Y%m%d-%H%M%S")
      branch = session["git_branch"]
      reason = session["tmux_reason"]
      for filename, path, _size in files:
        target_name = (
          f"{reason}-{stamp}-{branch}.txt" if filename == "tmux.log" else f"toggles-{stamp}.json"
        )
        os.replace(path, destination / target_name)
      await self._finish_reservation(device, session["source_ip"], reservation_day, reserved, total)
      reservation_created = False
      return web.json_response({"ok": True, "size": total, "files": len(files)})
    finally:
      if reservation_created:
        await self._finish_reservation(device, session["source_ip"], reservation_day, reserved, 0)
      if temp_dir.exists():
        shutil.rmtree(temp_dir, ignore_errors=True)
      await self._leave_upload(device)

  async def cleanup(self) -> dict[str, int]:
    # The DSM root already contains historical FTP uploads and source folders.
    # Never scan or delete that tree. Only discard expired session/quota state.
    now = int(datetime.now(UTC).timestamp())
    oldest_usage_day = (datetime.now(UTC) - timedelta(days=2)).strftime("%Y-%m-%d")
    async with self._db_lock:
      with self._connect() as connection:
        expired = connection.execute("DELETE FROM sessions WHERE expires_at < ?", (now,)).rowcount
        usage = connection.execute("DELETE FROM daily_usage WHERE usage_day < ?", (oldest_usage_day,)).rowcount
    return {"sessions": expired, "usageRows": usage}


UPLOAD_SERVICE_KEY = web.AppKey("upload_service", UploadService)
CLEANUP_TASK_KEY = web.AppKey("cleanup_task", asyncio.Task[None])


async def _cleanup_loop(app: web.Application) -> None:
  service = app[UPLOAD_SERVICE_KEY]
  while True:
    try:
      await service.cleanup()
    except Exception:
      pass
    await asyncio.sleep(6 * 60 * 60)


async def _start_cleanup(app: web.Application) -> None:
  app[CLEANUP_TASK_KEY] = asyncio.create_task(_cleanup_loop(app))


async def _stop_cleanup(app: web.Application) -> None:
  task = app.get(CLEANUP_TASK_KEY)
  if task:
    task.cancel()
    try:
      await task
    except asyncio.CancelledError:
      pass


@web.middleware
async def security_headers(request: web.Request, handler):
  try:
    response = await handler(request)
  except web.HTTPException as exc:
    response = web.json_response({"ok": False, "error": exc.text or exc.reason}, status=exc.status)
  except Exception:
    logging.exception("unhandled upload request error")
    response = web.json_response({"ok": False, "error": "internal server error"}, status=500)
  response.headers["Cache-Control"] = "no-store"
  response.headers["X-Content-Type-Options"] = "nosniff"
  response.headers["Referrer-Policy"] = "no-referrer"
  return response


def create_app(config: Config | None = None, *, start_cleanup: bool = True) -> web.Application:
  service = UploadService(config or Config.from_env())
  app = web.Application(client_max_size=service.config.max_file_bytes + MIB, middlewares=[security_headers])
  app[UPLOAD_SERVICE_KEY] = service
  app.router.add_get("/api/v1/health", service.health)
  app.router.add_post("/api/v1/session", service.create_session)
  app.router.add_put("/api/v1/upload/{device}/{segment}/{filename}", service.upload_file)
  app.router.add_post("/api/v1/complete", service.complete)
  app.router.add_post("/api/v1/tmux/upload", service.tmux_upload)
  if start_cleanup:
    app.on_startup.append(_start_cleanup)
    app.on_cleanup.append(_stop_cleanup)
  return app


def main() -> None:
  web.run_app(
    create_app(),
    host=os.environ.get("CARROT_UPLOAD_HOST", "0.0.0.0"),
    port=_env_int("CARROT_UPLOAD_PORT", 8080, 1),
    access_log_format='%a %t "%r" %s %b %Tf',
  )


if __name__ == "__main__":
  main()
