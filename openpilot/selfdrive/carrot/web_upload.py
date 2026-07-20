from __future__ import annotations

import json
import os
import time
import urllib.parse
from collections.abc import Callable, Mapping
from contextlib import ExitStack
from typing import Any

from aiohttp import ClientSession, ClientTimeout


DEFAULT_WEB_UPLOAD_URL = "https://op.wjcloud.kr"
DEFAULT_TOSS_UPLOAD_URL = "https://op.wjcloud.kr"
DEFAULT_TMUX_WEB_UPLOAD_URL = "https://tmux.carrotpilot.app/upload"
UPLOAD_TARGETS = {"carrot", "toss"}


def normalize_base_url(value: Any, default: str = "") -> str:
  url = str(value or "").strip().rstrip("/")
  if not url:
    url = str(default or "").strip().rstrip("/")
  if url and not url.startswith(("http://", "https://")):
    raise ValueError("web upload URL must start with http:// or https://")
  return url


def web_upload_settings(settings: Mapping[str, Any] | None = None) -> tuple[str, str]:
  """Return the upstream Carrot Web API settings.

  Toss credentials intentionally do not fall back into this path: the selected
  target owns its own URL/token so changing one target cannot redirect the other.
  """
  settings = settings or {}
  base_url = (
    os.environ.get("CARROT_WEB_UPLOAD_URL", "").strip()
    or str(settings.get("web_upload_url") or "").strip()
    or DEFAULT_WEB_UPLOAD_URL
  )
  token = (
    os.environ.get("CARROT_WEB_UPLOAD_TOKEN", "").strip()
    or str(settings.get("web_upload_token") or "").strip()
  )
  return normalize_base_url(base_url), token


def toss_upload_settings(settings: Mapping[str, Any] | None = None) -> tuple[str, str]:
  settings = settings or {}
  base_url = (
    os.environ.get("CARROT_TOSS_UPLOAD_URL", "").strip()
    or str(settings.get("toss_upload_url") or "").strip()
    or DEFAULT_TOSS_UPLOAD_URL
  )
  token = (
    os.environ.get("CARROT_TOSS_UPLOAD_TOKEN", "").strip()
    or str(settings.get("toss_upload_token") or "").strip()
  )
  return normalize_base_url(base_url), token


def selected_upload_settings(settings: Mapping[str, Any] | None = None) -> tuple[str, str, str]:
  settings = settings or {}
  target = str(settings.get("log_upload_target") or "carrot").strip().lower()
  target = target if target in UPLOAD_TARGETS else "carrot"
  base_url, token = toss_upload_settings(settings) if target == "toss" else web_upload_settings(settings)
  return target, base_url, token


def api_url(base_url: str, *parts: str) -> str:
  base_url = normalize_base_url(base_url)
  if not base_url:
    raise ValueError("web upload URL is not configured")
  quoted = "/".join(urllib.parse.quote(str(part), safe="") for part in parts)
  return f"{base_url}/api/v1/{quoted}"


def tmux_web_target(settings: Mapping[str, Any] | None = None) -> tuple[str, dict[str, str]]:
  target, base_url, token = selected_upload_settings(settings)
  if token:
    return api_url(base_url, "tmux", "upload"), {"Authorization": f"Bearer {token}"}
  if target == "toss":
    raise ValueError("Toss upload token is not configured")

  # Preserve the upstream Carrot tmux fallback only for the default Carrot
  # target. Toss must never leak diagnostics to the Carrot endpoint.
  direct_url = normalize_base_url(
    os.environ.get("CARROT_TMUX_WEB_UPLOAD_URL", ""),
    DEFAULT_TMUX_WEB_UPLOAD_URL,
  )
  return direct_url, {}


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

  if not token:
    return {"ok": False, "error": "web upload token is not configured", "elapsed_ms": elapsed_ms()}
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      async with session.get(
        api_url(base_url, "health"),
        headers={"Authorization": f"Bearer {token}"},
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
) -> bool:
  def check_cancel() -> None:
    if should_cancel and should_cancel():
      raise RuntimeError("upload canceled")

  check_cancel()
  try:
    entries = sorted(entry.name for entry in os.scandir(local_folder) if entry.is_file(follow_symlinks=False))
  except OSError as e:
    raise RuntimeError(f"cannot read segment folder: {e}") from e
  if not token:
    raise RuntimeError("web upload token is not configured")

  headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/octet-stream"}
  timeout = ClientTimeout(total=None, connect=20, sock_read=180)
  async with ClientSession(timeout=timeout, headers=headers) as session:
    for filename in entries:
      local_path = os.path.join(local_folder, filename)
      url = api_url(base_url, "upload", directory, remote_path, filename)
      sent = 0

      async def send_file(path: str = local_path):
        nonlocal sent
        sent = 0
        check_cancel()
        with open(path, "rb") as f:
          while True:
            check_cancel()
            chunk = f.read(1024 * 1024)
            if not chunk:
              break
            sent += len(chunk)
            yield chunk

      last_error: Exception | None = None
      for _attempt in range(2):
        check_cancel()
        try:
          async with session.put(url, data=send_file()) as resp:
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
    return {"ok": False, "error": "web upload token is not configured"}
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
