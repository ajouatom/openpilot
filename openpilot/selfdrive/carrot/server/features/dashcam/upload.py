import base64
import json
import os
import subprocess
import urllib.parse
from ftplib import FTP
from typing import Any, Callable

from aiohttp import ClientSession, ClientTimeout

from openpilot.system.hardware import HARDWARE

from ...config import DASHCAM_DEFAULT_DISCORD_KEY, DASHCAM_DEFAULT_DISCORD_WEBHOOK
from ...services.web_settings import read_web_settings


def param_text(params: Any, key: str, default: str = "unknown") -> str:
  try:
    if not params:
      return default
    value = params.get(key)
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    value = str(value or "").strip()
    return value or default
  except Exception:
    return default


def repo_dir() -> str:
  return os.environ.get("CARROT_REPO_DIR", "/data/openpilot")


def git_text(args: list[str], default: str = "") -> str:
  try:
    result = subprocess.run(
      ["git", *args],
      cwd=repo_dir(),
      capture_output=True,
      text=True,
      timeout=4,
    )
    if result.returncode == 0:
      value = (result.stdout or "").strip()
      return value or default
  except Exception:
    pass
  return default


def device_serial(params: Any) -> str:
  for key in ("HardwareSerial", "DeviceSerial", "Serial", "CarrotSerial"):
    value = param_text(params, key, "")
    if value:
      return value
  for env_key in ("CARROT_DEVICE_SERIAL", "DEVICE_SERIAL", "SERIAL"):
    value = os.environ.get(env_key, "").strip()
    if value:
      return value
  try:
    getter = getattr(HARDWARE, "get_serial", None)
    if callable(getter):
      value = str(getter() or "").strip()
      if value:
        return value
  except Exception:
    pass
  return "unknown"


def upload_metadata(params: Any) -> dict[str, str]:
  return {
    "carName": param_text(params, "CarName", "none"),
    "dongleId": param_text(params, "DongleId", "unknown"),
    "serial": device_serial(params),
    "branch": git_text(["branch", "--show-current"], "unknown"),
    "commit": git_text(["rev-parse", "--short", "HEAD"], "unknown"),
    "commitDate": git_text(["show", "-s", "--date=format:%Y-%m-%d %H:%M:%S", "--format=%cd", "HEAD"], "unknown"),
  }


def decode_obfuscated(value: str, key: str) -> str:
  try:
    token = str(value or "").strip()
    key_bytes = str(key or "").encode("utf-8")
    if not token or not key_bytes:
      return ""
    raw = base64.urlsafe_b64decode(token + "=" * (-len(token) % 4))
    decoded = bytes(raw[i] ^ key_bytes[i % len(key_bytes)] for i in range(len(raw)))
    return decoded.decode("utf-8", errors="ignore").strip()
  except Exception:
    return ""


def discord_webhook_url(params: Any) -> str:
  for key in ("CARROT_DISCORD_WEBHOOK_URL", "DISCORD_WEBHOOK_URL"):
    value = os.environ.get(key, "").strip()
    if value:
      return value
  for key in ("CarrotDiscordWebhookUrl", "CarrotDiscordWebhookURL", "DiscordWebhookUrl", "DiscordWebhookURL"):
    value = param_text(params, key, "")
    if value:
      return value
  if os.environ.get("CARROT_DISCORD_WEBHOOK_DISABLE", "").strip().lower() in {"1", "true", "yes", "on"}:
    return ""
  return decode_obfuscated(DASHCAM_DEFAULT_DISCORD_WEBHOOK, DASHCAM_DEFAULT_DISCORD_KEY)


def upload_message_lines(payload: dict[str, Any], max_results: int | None = None) -> list[str]:
  meta = payload.get("meta") or {}
  commit = str(meta.get("commit") or "").strip()
  commit_date = meta.get("commitDate") or "unknown"
  commit_text = (
    f"[{commit}](https://github.com/ajouatom/openpilot/commit/{commit})"
    if commit and commit != "unknown"
    else "unknown"
  )
  uploaded = [item for item in payload.get("results") or [] if item.get("ok")]
  failed = [item for item in payload.get("results") or [] if not item.get("ok")]
  lines = [
    "# Carrot Dashcam Upload",
    "### Upload",
    f"- Time: {payload.get('uploadedAt') or ''}",
    f"- Server: {'Toss' if payload.get('target') == 'toss' else 'Carrot'}",
    f"- Path: {payload.get('remoteBasePath') or ''}",
    "### Device",
    f"- Car name: {meta.get('carName') or 'none'}",
    f"- DongleId: {meta.get('dongleId') or 'unknown'}",
    f"- Serial: {meta.get('serial') or 'unknown'}",
    f"- Branch: {meta.get('branch') or 'unknown'}",
    f"- Commit: {commit_text} ({commit_date})",
    "### Result",
  ]

  result_items = uploaded + failed
  visible_items = result_items if max_results is None else result_items[:max_results]
  for item in visible_items:
    if item.get("ok"):
      lines.append(f"- {item.get('segment')} OK")
    else:
      error = str(item.get("error") or "").strip()
      suffix = f": {error}" if error else ""
      lines.append(f"- {item.get('segment')} FAILED{suffix}")

  hidden_count = len(result_items) - len(visible_items)
  if hidden_count > 0:
    lines.append(f"- ... +{hidden_count} more")
  if not result_items:
    lines.append("- none")
  return lines


def upload_share_text(payload: dict[str, Any]) -> str:
  return "\n".join(upload_message_lines(payload)).strip()


def discord_content(payload: dict[str, Any]) -> str:
  content = "\n".join(upload_message_lines(payload, max_results=24)).strip()
  if len(content) <= 1900:
    return content

  content = "\n".join(upload_message_lines(payload, max_results=10)).strip()
  if len(content) <= 1900:
    return content

  return "\n".join(upload_message_lines(payload, max_results=3)).strip()[:1900]


async def send_discord_webhook(url: str, payload: dict[str, Any]) -> dict[str, Any]:
  url = (url or "").strip()
  if not url:
    return {"configured": False, "ok": False, "skipped": True}
  if not url.startswith(("http://", "https://")):
    return {"configured": True, "ok": False, "error": "invalid webhook url"}
  body = {
    "username": "Carrot Dashcam",
    "content": discord_content(payload),
    "allowed_mentions": {"parse": []},
    "flags": 4,
  }
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      async with session.post(url, json=body) as resp:
        text = await resp.text()
        if 200 <= resp.status < 300:
          return {"configured": True, "ok": True, "status": resp.status}
        return {"configured": True, "ok": False, "status": resp.status, "error": text[:500]}
  except Exception as e:
    return {"configured": True, "ok": False, "error": str(e)}


def upload_folder_to_ftp(
  local_folder: str,
  directory: str,
  remote_path: str,
  should_cancel: Callable[[], bool] | None = None,
) -> bool:
  def check_cancel() -> None:
    if should_cancel and should_cancel():
      raise RuntimeError("upload canceled")

  ftp_server = os.environ.get("CARROT_FTP_SERVER", "shind0.synology.me")
  ftp_port = int(os.environ.get("CARROT_FTP_PORT", "8021"))
  ftp_username = os.environ.get("CARROT_FTP_USERNAME", "carrotpilot")
  ftp_password = os.environ.get("CARROT_FTP_PASSWORD", "Ekdrmsvkdlffjt7710")

  check_cancel()
  ftp = FTP()
  ftp.connect(ftp_server, ftp_port, timeout=20)
  check_cancel()
  ftp.login(ftp_username, ftp_password)
  try:
    check_cancel()
    ftp.cwd("routes")
    routes_root = ftp.pwd()

    def cwd_or_create(path: str) -> None:
      check_cancel()
      ftp.cwd(routes_root)
      for part in [p for p in path.split("/") if p]:
        check_cancel()
        try:
          ftp.cwd(part)
        except Exception:
          ftp.mkd(part)
          ftp.cwd(part)

    base_path = f"{directory}/{remote_path}".strip("/")
    for root, _, files in os.walk(local_folder):
      check_cancel()
      rel_dir = os.path.relpath(root, local_folder)
      remote_dir = base_path if rel_dir == "." else f"{base_path}/{rel_dir.replace(os.sep, '/')}"
      cwd_or_create(remote_dir)
      for filename in files:
        check_cancel()
        local_path = os.path.join(root, filename)
        with open(local_path, "rb") as f:
          # 1MB chunks (vs ftplib's 8KB default) cut per-chunk Python/syscall
          # overhead for large camera files.
          ftp.storbinary(f"STOR {filename}", f, blocksize=1024 * 1024)
        check_cancel()
    return True
  finally:
    try:
      ftp.quit()
    except Exception:
      pass


def toss_settings() -> tuple[str, str]:
  try:
    settings = read_web_settings()
  except Exception:
    settings = {}
  base_url = str(settings.get("toss_upload_url") or "").strip().rstrip("/")
  token = str(settings.get("toss_upload_token") or "").strip()
  return base_url, token


def resolve_upload_target() -> dict[str, Any]:
  try:
    settings = read_web_settings()
  except Exception:
    settings = {}
  if str(settings.get("log_upload_target") or "").strip().lower() != "toss":
    return {"kind": "carrot"}
  base_url, token = toss_settings()
  if not base_url:
    raise RuntimeError("Toss server URL is not configured")
  if not token:
    raise RuntimeError("Toss server token is not configured")
  return {"kind": "toss", "base_url": base_url, "token": token}


def _toss_url(base_url: str, *parts: str) -> str:
  quoted = "/".join(urllib.parse.quote(part, safe="") for part in parts)
  return f"{base_url}/api/v1/{quoted}"


async def check_toss_health(base_url: str, token: str) -> dict[str, Any]:
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      url = f"{base_url}/api/v1/health"
      async with session.get(url, headers={"Authorization": f"Bearer {token}"}) as resp:
        text = await resp.text()
        if resp.status == 200:
          return {"ok": True, "status": resp.status}
        return {"ok": False, "status": resp.status, "error": text[:300]}
  except Exception as e:
    return {"ok": False, "error": str(e)}


async def upload_folder_to_toss(
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

  # The toss API stores a flat file list per segment
  # (PUT /upload/{directory}/{segment}/{filename}); segment folders only
  # contain top-level files in practice.
  check_cancel()
  try:
    entries = sorted(e.name for e in os.scandir(local_folder) if e.is_file())
  except OSError as e:
    raise RuntimeError(f"cannot read segment folder: {e}") from e

  headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/octet-stream"}
  timeout = ClientTimeout(total=None, connect=20, sock_read=180)
  async with ClientSession(timeout=timeout, headers=headers) as session:
    for filename in entries:
      local_path = os.path.join(local_folder, filename)
      local_size = os.path.getsize(local_path)
      url = _toss_url(base_url, "upload", directory, remote_path, filename)

      async def send_file(path: str = local_path):
        check_cancel()
        with open(path, "rb") as f:
          while True:
            check_cancel()
            chunk = f.read(1024 * 1024)
            if not chunk:
              break
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
            if resp.status != 200 or not (body or {}).get("ok"):
              error = str((body or {}).get("error") or text or "")[:200]
              raise RuntimeError(f"HTTP {resp.status}: {error}")
            remote_size = int((body or {}).get("size") or -1)
            if remote_size != local_size:
              raise RuntimeError(f"size mismatch for {filename}: local {local_size}, remote {remote_size}")
          last_error = None
          break
        except Exception as e:
          check_cancel()
          last_error = e
      if last_error is not None:
        raise RuntimeError(f"{filename}: {last_error}") from last_error
      check_cancel()
  return True


async def send_toss_complete(base_url: str, token: str, payload: dict[str, Any]) -> dict[str, Any]:
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      url = f"{base_url}/api/v1/complete"
      async with session.post(url, json=payload, headers={"Authorization": f"Bearer {token}"}) as resp:
        text = await resp.text()
        if 200 <= resp.status < 300:
          return {"ok": True, "status": resp.status}
        return {"ok": False, "status": resp.status, "error": text[:300]}
  except Exception as e:
    return {"ok": False, "error": str(e)}
