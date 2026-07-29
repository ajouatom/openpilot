from __future__ import annotations

import json
import os
import time
import urllib.parse
from collections.abc import Callable, Mapping, Sequence
from contextlib import ExitStack
from typing import Any

from aiohttp import ClientSession, ClientTimeout


DEFAULT_WEB_UPLOAD_URL = "https://upload.shind0.synology.me"
DEFAULT_TMUX_WEB_UPLOAD_URL = "https://tmux.carrotpilot.app/upload"


def normalize_base_url(value: Any, default: str = "") -> str:
  url = str(value or "").strip().rstrip("/")
  if not url:
    url = str(default or "").strip().rstrip("/")
  if url and not url.startswith(("http://", "https://")):
    raise ValueError("web upload URL must start with http:// or https://")
  return url


def web_upload_settings(settings: Mapping[str, Any] | None = None) -> tuple[str, str]:
  settings = settings or {}
  base_url = (
    os.environ.get("CARROT_WEB_UPLOAD_URL", "").strip()
    or str(settings.get("web_upload_url") or "").strip()
    or str(settings.get("toss_upload_url") or "").strip()
    or DEFAULT_WEB_UPLOAD_URL
  )
  # Normal users never configure a token. The server issues a short-lived
  # session automatically. Keep only an environment override for private
  # deployments that deliberately use a static service token.
  token = os.environ.get("CARROT_WEB_UPLOAD_TOKEN", "").strip()
  return normalize_base_url(base_url), token


def api_url(base_url: str, *parts: str) -> str:
  base_url = normalize_base_url(base_url)
  if not base_url:
    raise ValueError("web upload URL is not configured")
  quoted = "/".join(urllib.parse.quote(str(part), safe="") for part in parts)
  return f"{base_url}/api/v1/{quoted}"


def tmux_web_target(
  settings: Mapping[str, Any] | None = None,
  session_token: str = "",
) -> tuple[str, dict[str, str]]:
  base_url, token = web_upload_settings(settings)
  token = str(session_token or token).strip()
  if token:
    return api_url(base_url, "tmux", "upload"), {"Authorization": f"Bearer {token}"}

  direct_url = normalize_base_url(
    os.environ.get("CARROT_TMUX_WEB_UPLOAD_URL", ""),
    DEFAULT_TMUX_WEB_UPLOAD_URL,
  )
  return direct_url, {}


def carrot_logs_web_target() -> tuple[str, dict[str, str]]:
  """Return the independent Carrot Logs receiver used by the Discord forum.

  This target must not depend on the DSM upload token. Diagnostics are sent to
  both destinations, so a configured DSM token must never redirect this copy
  away from the Carrot Logs service.
  """
  direct_url = normalize_base_url(
    os.environ.get("CARROT_TMUX_WEB_UPLOAD_URL", ""),
    DEFAULT_TMUX_WEB_UPLOAD_URL,
  )
  return direct_url, {}


def upload_device_id(metadata: Mapping[str, Any]) -> str:
  for key in ("dongleId", "dongle_id", "deviceId", "device_id", "serial", "device_serial"):
    value = str(metadata.get(key) or "").strip()
    if value and value.lower() not in {"unknown", "none"}:
      return value
  return "unknown"


def _session_payload(metadata: Mapping[str, Any], purpose: str) -> dict[str, str]:
  payload = {str(key): str(value or "")[:160] for key, value in metadata.items()}
  payload["deviceId"] = upload_device_id(metadata)
  payload["purpose"] = purpose
  return payload


def _session_token(body: Any) -> str:
  token = str((body or {}).get("token") or "").strip() if isinstance(body, Mapping) else ""
  if not token:
    raise RuntimeError("upload server did not issue a session")
  return token


async def create_web_upload_session(
  base_url: str,
  metadata: Mapping[str, Any],
  purpose: str = "dashcam",
) -> str:
  timeout = ClientTimeout(total=12)
  async with ClientSession(timeout=timeout) as session:
    async with session.post(api_url(base_url, "session"), json=_session_payload(metadata, purpose)) as resp:
      text = await resp.text()
      try:
        body = json.loads(text)
      except Exception:
        body = None
      if not 200 <= resp.status < 300 or not (body or {}).get("ok"):
        error = str((body or {}).get("error") or text or "")[:300]
        raise RuntimeError(f"upload session HTTP {resp.status}: {error}")
      return _session_token(body)


def create_web_upload_session_sync(
  base_url: str,
  metadata: Mapping[str, Any],
  post: Callable[..., Any],
  purpose: str = "tmux",
) -> str:
  response = post(
    api_url(base_url, "session"),
    json=_session_payload(metadata, purpose),
    timeout=12,
  )
  try:
    body = response.json()
  except Exception:
    body = None
  status = int(getattr(response, "status_code", 0) or 0)
  if not 200 <= status < 300 or not (body or {}).get("ok"):
    text = str(getattr(response, "text", "") or "")[:300]
    error = str((body or {}).get("error") or text)[:300]
    raise RuntimeError(f"upload session HTTP {status}: {error}")
  return _session_token(body)


def post_tmux_web(
  url: str,
  headers: Mapping[str, str],
  payload: Mapping[str, Any],
  tmux_path: str,
  settings_path: str | None = None,
  post: Callable[..., Any] | None = None,
):
  if post is None:
    raise ValueError("web POST function is required")
  with ExitStack() as stack:
    tmux_file = stack.enter_context(open(tmux_path, "rb"))
    files = [("files[0]", ("tmux.log", tmux_file, "text/plain"))]
    if settings_path and os.path.isfile(settings_path):
      settings_file = stack.enter_context(open(settings_path, "rb"))
      files.append(("files[1]", ("toggle_values.json", settings_file, "application/json")))
    return post(
      url,
      headers=dict(headers),
      data=dict(payload),
      files=files,
      timeout=30,
    )


async def check_web_upload_health(base_url: str, token: str) -> dict[str, Any]:
  started = time.monotonic()

  def elapsed_ms() -> int:
    return int((time.monotonic() - started) * 1000)

  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      headers = {"Authorization": f"Bearer {token}"} if token else {}
      async with session.get(
        api_url(base_url, "health"),
        headers=headers,
      ) as resp:
        text = await resp.text()
        if resp.status == 200:
          return {"ok": True, "status": resp.status, "elapsed_ms": elapsed_ms()}
        return {"ok": False, "status": resp.status, "error": text[:300], "elapsed_ms": elapsed_ms()}
  except Exception as e:
    return {"ok": False, "error": str(e), "elapsed_ms": elapsed_ms()}


async def upload_folder_to_web(
  local_folder: str,
  directory: str,
  remote_path: str,
  base_url: str,
  token: str,
  should_cancel: Callable[[], bool] | None = None,
  filenames: Sequence[str] | None = None,
  on_progress: Callable[[str, int, int, int], None] | None = None,
) -> bool:
  def check_cancel() -> None:
    if should_cancel and should_cancel():
      raise RuntimeError("upload canceled")

  check_cancel()
  if filenames is None:
    try:
      entries = sorted(entry.name for entry in os.scandir(local_folder) if entry.is_file(follow_symlinks=False))
    except OSError as e:
      raise RuntimeError(f"cannot read segment folder: {e}") from e
  else:
    entries = []
    seen: set[str] = set()
    for raw_name in filenames:
      filename = str(raw_name or "")
      if not filename or filename in (".", "..") or "/" in filename or "\\" in filename:
        raise RuntimeError("invalid upload filename")
      if filename in seen:
        continue
      local_path = os.path.join(local_folder, filename)
      if not os.path.isfile(local_path):
        raise RuntimeError(f"upload file not found: {filename}")
      seen.add(filename)
      entries.append(filename)
  if not token:
    raise RuntimeError("upload session is not configured")

  headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/octet-stream"}
  timeout = ClientTimeout(total=None, connect=20, sock_read=180)
  async with ClientSession(timeout=timeout, headers=headers) as session:
    for filename in entries:
      local_path = os.path.join(local_folder, filename)
      url = api_url(base_url, "upload", directory, remote_path, filename)
      file_size = os.path.getsize(local_path)
      sent = 0

      async def send_file(
        path: str = local_path,
        upload_name: str = filename,
        upload_size: int = file_size,
      ):
        nonlocal sent
        sent = 0
        if on_progress:
          on_progress(upload_name, sent, upload_size, 0)
        check_cancel()
        with open(path, "rb") as f:
          while True:
            check_cancel()
            chunk = f.read(1024 * 1024)
            if not chunk:
              break
            sent += len(chunk)
            if on_progress:
              on_progress(upload_name, sent, upload_size, len(chunk))
            yield chunk

      last_error: Exception | None = None
      for _attempt in range(2):
        check_cancel()
        try:
          async with session.put(url, data=send_file(), headers={"X-File-Size": str(file_size)}) as resp:
            text = await resp.text()
            try:
              body = json.loads(text)
            except Exception:
              body = None
            if not 200 <= resp.status < 300 or not (body or {}).get("ok"):
              error = str((body or {}).get("error") or text or "")[:200]
              raise RuntimeError(f"HTTP {resp.status}: {error}")
            raw_size = (body or {}).get("size")
            remote_size = int(raw_size) if raw_size is not None else -1
            if remote_size != sent:
              raise RuntimeError(f"size mismatch for {filename}: sent {sent}, remote {remote_size}")
          last_error = None
          break
        except Exception as e:
          check_cancel()
          last_error = e
      if last_error is not None:
        raise RuntimeError(f"{filename}: {last_error}") from last_error
      check_cancel()
  return True


async def send_web_upload_complete(base_url: str, token: str, payload: dict[str, Any]) -> dict[str, Any]:
  if not token:
    return {"ok": False, "error": "upload session is not configured"}
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      async with session.post(
        api_url(base_url, "complete"),
        json=payload,
        headers={"Authorization": f"Bearer {token}"},
      ) as resp:
        text = await resp.text()
        if 200 <= resp.status < 300:
          return {"ok": True, "status": resp.status}
        return {"ok": False, "status": resp.status, "error": text[:300]}
  except Exception as e:
    return {"ok": False, "error": str(e)}
