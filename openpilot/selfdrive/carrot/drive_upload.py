"""Dashcam segment upload to the owner's own Google Drive (OAuth scope drive.file).

The credentials (client_id, client_secret, refresh_token) live in one JSON file on the device and never leave
this module: the web API only learns whether Drive is configured. Files go to openpilot/<route>/<segment>/.
"""
from __future__ import annotations

import asyncio
import json
import os
from collections.abc import Callable
from typing import Any

from aiohttp import ClientError, ClientSession, ClientTimeout

CONFIG_PATH = "/data/carrot/google_drive.json"
TOKEN_URL = "https://oauth2.googleapis.com/token"
FILES_URL = "https://www.googleapis.com/drive/v3/files"
UPLOAD_URL = "https://www.googleapis.com/upload/drive/v3/files"
FOLDER_MIME = "application/vnd.google-apps.folder"
ROOT_FOLDER = "openpilot"
CHUNK_SIZE = 8 * 1024 * 1024  # resumable chunks must be multiples of 256 KiB
RETRIES = 5
RETRY_BASE_S = 1.0
TIMEOUT = ClientTimeout(total=None, connect=30, sock_read=120)


class DriveError(Exception):
  pass


def config_path() -> str:
  return os.environ.get("CARROT_GOOGLE_DRIVE_CONFIG", "").strip() or CONFIG_PATH


def load_config(path: str | None = None) -> dict[str, str] | None:
  try:
    with open(path or config_path(), encoding="utf-8") as f:
      data = json.load(f)
  except (OSError, ValueError):
    return None
  keys = ("client_id", "client_secret", "refresh_token")
  if not isinstance(data, dict) or not all(isinstance(data.get(k), str) and data[k].strip() for k in keys):
    return None
  return {k: data[k].strip() for k in keys}


def is_configured() -> bool:
  return load_config() is not None


def _quote(value: str) -> str:
  return value.replace("\\", "\\\\").replace("'", "\\'")


def _range_end(headers: Any) -> int:
  """Next byte to send from a 308 response's Range header ("bytes=0-N"); 0 when nothing was stored."""
  value = str(headers.get("Range") or "")
  try:
    return int(value.rsplit("-", 1)[1]) + 1 if value.startswith("bytes=") else 0
  except (IndexError, ValueError):
    return 0


class DriveClient:
  def __init__(self, session: ClientSession, config: dict[str, str]):
    self.session, self.config = session, config
    self.token = ""
    self.folders: dict[tuple[str, str], str] = {}
    self.folder_locks: dict[tuple[str, str], asyncio.Lock] = {}

  async def refresh(self) -> None:
    data = {
      "client_id": self.config["client_id"],
      "client_secret": self.config["client_secret"],
      "refresh_token": self.config["refresh_token"],
      "grant_type": "refresh_token",
    }
    async with self.session.post(TOKEN_URL, data=data) as resp:
      body = await resp.json(content_type=None)
    if resp.status != 200 or not isinstance(body, dict) or not body.get("access_token"):
      error = body.get("error", "") if isinstance(body, dict) else ""
      raise DriveError(f"Google token refresh failed ({resp.status} {error}); issue a new refresh token")
    self.token = body["access_token"]

  async def call(self, method: str, url: str, *, ok=(200,), headers: dict[str, str] | None = None,
                 **kwargs) -> tuple[int, Any, Any]:
    """One Drive API request: refresh on 401 once, back off on 429/5xx and connection errors."""
    refreshed = False
    for attempt in range(RETRIES + 1):
      if not self.token:
        await self.refresh()
      try:
        async with self.session.request(method, url, headers={"Authorization": f"Bearer {self.token}", **(headers or {})},
                                        **kwargs) as resp:
          status, resp_headers = resp.status, resp.headers
          try:
            body = await resp.json(content_type=None)
          except ValueError:
            body = None
      except (ClientError, asyncio.TimeoutError) as exc:
        if attempt == RETRIES:
          raise DriveError(f"Drive request failed: {exc}") from exc
        await asyncio.sleep(RETRY_BASE_S * 2 ** attempt)
        continue
      if status in ok:
        return status, resp_headers, body
      if status == 401 and not refreshed:
        self.token, refreshed = "", True
        continue
      if (status == 429 or status >= 500) and attempt < RETRIES:
        await asyncio.sleep(RETRY_BASE_S * 2 ** attempt)
        continue
      raise DriveError(f"Drive {method} failed ({status}): {body}")
    raise DriveError("Drive request retries exhausted")

  async def find(self, name: str, parent: str, *, folder: bool = False) -> dict[str, Any] | None:
    query = f"name = '{_quote(name)}' and '{_quote(parent)}' in parents and trashed = false"
    if folder:
      query += f" and mimeType = '{FOLDER_MIME}'"
    _, _, body = await self.call("GET", FILES_URL, params={"q": query, "fields": "files(id,size)", "spaces": "drive"})
    files = (body or {}).get("files") or []
    return files[0] if files else None

  async def folder(self, name: str, parent: str = "root") -> str:
    key = (parent, name)
    async with self.folder_locks.setdefault(key, asyncio.Lock()):  # segments of one route upload in parallel
      if key not in self.folders:
        found = await self.find(name, parent, folder=True)
        if found is None:
          _, _, found = await self.call("POST", FILES_URL, params={"fields": "id"},
                                        json={"name": name, "mimeType": FOLDER_MIME, "parents": [parent]})
        self.folders[key] = found["id"]
      return self.folders[key]

  async def upload_file(self, path: str, parent: str, on_progress: Callable[[int, int], None] | None = None,
                        should_cancel: Callable[[], bool] | None = None) -> str:
    """Resumable upload of one file; returns "uploaded" or "skipped" (same name and size already there)."""
    name, size = os.path.basename(path), os.path.getsize(path)
    existing = await self.find(name, parent)
    if existing is not None and int(existing.get("size") or -1) == size:
      if on_progress:
        on_progress(size, 0)
      return "skipped"
    for _ in range(2):  # a session URL can expire (404/410): start one new session
      if existing is None:
        _, headers, _ = await self.call("POST", UPLOAD_URL, params={"uploadType": "resumable"},
                                        json={"name": name, "parents": [parent]},
                                        headers={"X-Upload-Content-Length": str(size)})
      else:
        _, headers, _ = await self.call("PATCH", f"{UPLOAD_URL}/{existing['id']}", params={"uploadType": "resumable"},
                                        json={}, headers={"X-Upload-Content-Length": str(size)})
      if await self._send(headers["Location"], path, size, on_progress, should_cancel):
        return "uploaded"
    raise DriveError(f"Drive upload session for {name} kept expiring")

  async def _send(self, session_url: str, path: str, size: int, on_progress, should_cancel) -> bool:
    offset, failures = 0, 0
    with open(path, "rb") as f:
      while True:
        if should_cancel and should_cancel():
          raise DriveError("upload canceled")
        f.seek(offset)
        chunk = f.read(CHUNK_SIZE)
        content_range = f"bytes {offset}-{offset + len(chunk) - 1}/{size}" if chunk else f"bytes */{size}"
        try:
          async with self.session.put(session_url, data=chunk, headers={"Content-Range": content_range}) as resp:
            status, headers = resp.status, resp.headers
            await resp.read()
        except (ClientError, asyncio.TimeoutError):
          status, headers = None, {}
        if status in (200, 201):
          if on_progress:
            on_progress(size, size - offset)
          return True
        if status == 308:
          sent = _range_end(headers)
          if on_progress:
            on_progress(sent, max(0, sent - offset))
          offset, failures = sent, 0
          continue
        if status in (404, 410):
          return False
        failures += 1
        if failures > RETRIES or (status is not None and status < 500 and status != 429):
          raise DriveError(f"Drive chunk upload failed ({status})")
        await asyncio.sleep(RETRY_BASE_S * 2 ** (failures - 1))
        # ask where the server stands before resending
        try:
          async with self.session.put(session_url, headers={"Content-Range": f"bytes */{size}"}) as resp:
            if resp.status in (200, 201):
              return True
            if resp.status == 308:
              offset = _range_end(resp.headers)
        except (ClientError, asyncio.TimeoutError):
          pass


async def upload_folder_to_drive(client: DriveClient, folder: str, route: str, segment: str, filenames: list[str],
                                 should_cancel: Callable[[], bool] | None = None,
                                 on_progress: Callable[[str, int, int, int], None] | None = None) -> bool:
  parent = await client.folder(segment, await client.folder(route, await client.folder(ROOT_FOLDER)))
  for name in filenames:
    path = os.path.join(folder, name)
    size = os.path.getsize(path)

    def file_progress(sent: int, delta: int, name=name, size=size) -> None:
      if on_progress:
        on_progress(name, sent, size, delta)

    await client.upload_file(path, parent, file_progress, should_cancel)
  return True


def new_session() -> ClientSession:
  return ClientSession(timeout=TIMEOUT)
