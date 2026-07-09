from __future__ import annotations

import base64
import os
import socket
import subprocess
import time
from typing import Any

from aiohttp import ClientSession, ClientTimeout

from .params import get_param_values

try:
  from openpilot.system.hardware import HARDWARE
except Exception:
  HARDWARE = None

_OBFUSCATION_KEY = b"carrot-support-v1"
_OBFUSCATED_WEBHOOK_URL = (
  "CxUGAhxOAlwRGQMMHQZJWFIMDF0THx0CBBASGAAdH15ZAFZTRkBdRxpLQ0BEWUVFFUYEUE4ZJA0lf0Am"
  "BhgINy1EJ39XAzlEOwNJHxwmJjUkGkYxVwFYQDVXDVQmEyYfHCI6AAZ0KjQZACAbRhIRHBMVGQwcBAY5CQ=="
)


def _decode_obfuscated_webhook_url() -> str:
  try:
    data = base64.b64decode(_OBFUSCATED_WEBHOOK_URL)
    decoded = bytes(
      byte ^ _OBFUSCATION_KEY[index % len(_OBFUSCATION_KEY)]
      for index, byte in enumerate(data)
    )
    return decoded.decode("utf-8").strip()
  except Exception:
    return ""


def support_discord_webhook_url() -> str:
  for key in ("CARROT_SUPPORT_DISCORD_WEBHOOK_URL", "CARROT_DISCORD_WEBHOOK_URL", "DISCORD_WEBHOOK_URL"):
    value = os.environ.get(key, "").strip()
    if value:
      return value
  return _decode_obfuscated_webhook_url()


def _repo_dir() -> str:
  return os.environ.get("CARROT_REPO_DIR", "/data/openpilot")


def _git_text(args: list[str], default: str = "unknown") -> str:
  try:
    result = subprocess.run(
      ["git", *args],
      cwd=_repo_dir(),
      capture_output=True,
      text=True,
      timeout=4,
    )
    if result.returncode == 0:
      return (result.stdout or "").strip() or default
  except Exception:
    pass
  return default


def _device_serial(params: dict[str, Any]) -> str:
  for key in ("HardwareSerial", "DeviceSerial", "Serial", "CarrotSerial"):
    value = str(params.get(key) or "").strip()
    if value:
      return value
  for key in ("CARROT_DEVICE_SERIAL", "DEVICE_SERIAL", "SERIAL"):
    value = os.environ.get(key, "").strip()
    if value:
      return value
  try:
    getter = getattr(HARDWARE, "get_serial", None) if HARDWARE is not None else None
    if callable(getter):
      value = str(getter() or "").strip()
      if value:
        return value
  except Exception:
    pass
  return "unknown"


def support_metadata() -> dict[str, str]:
  params = get_param_values(
    ["CarName", "DongleId", "HardwareSerial", "DeviceSerial", "Serial", "CarrotSerial"],
    {
      "CarName": "none",
      "DongleId": "unknown",
      "HardwareSerial": "",
      "DeviceSerial": "",
      "Serial": "",
      "CarrotSerial": "",
    },
  )
  return {
    "carName": str(params.get("CarName") or "none").strip() or "none",
    "dongleId": str(params.get("DongleId") or "unknown").strip() or "unknown",
    "serial": _device_serial(params),
    "branch": _git_text(["branch", "--show-current"]),
    "commit": _git_text(["rev-parse", "--short", "HEAD"]),
    "commitDate": _git_text(["show", "-s", "--date=format:%Y-%m-%d %H:%M:%S", "--format=%cd", "HEAD"]),
    "host": socket.gethostname(),
  }


def _support_message(payload: dict[str, Any]) -> str:
  meta = payload.get("meta") or {}
  note = str(payload.get("note") or "").strip()
  commit = str(meta.get("commit") or "").strip()
  commit_date = meta.get("commitDate") or "unknown"
  commit_text = (
    f"[{commit}](https://github.com/ajouatom/openpilot/commit/{commit})"
    if commit and commit != "unknown"
    else "unknown"
  )
  permission_mode = str(payload.get("permissionMode") or "approve_each")
  permission_text = {
    "approve_each": "항상 확인",
    "allow_all": "전체 허용",
  }.get(permission_mode, permission_mode)
  expires_text = payload.get("ttl_minutes") or 30
  if expires_text == "unlimited":
    expires_text = "unlimited"
  else:
    expires_text = f"{expires_text} min"
  lines = [
    "# Carrot Remote Terminal",
    "### Session",
    f"- Time: {payload.get('createdAt') or time.strftime('%Y-%m-%d %H:%M:%S')}",
    f"- Session ID: {payload.get('sessionId') or 'unknown'}",
    "### Access",
    f"- Link: {payload.get('url') or ''}",
    f"- PIN: **__{payload.get('pin') or ''}__**",
    f"- Expires: {expires_text}",
    f"- Permission: {permission_text}",
  ]
  if permission_mode == "approve_each":
    lines.append(f"- Approval timeout: {payload.get('commandTimeoutSeconds') or 30} sec")
  if note:
    lines.extend([
      "### Issue",
      f"- Note: {note[:500]}",
    ])
  lines.extend([
    "### Device",
    f"- Car name: {meta.get('carName') or 'none'}",
    f"- DongleId: {meta.get('dongleId') or 'unknown'}",
    f"- Serial: {meta.get('serial') or 'unknown'}",
    f"- Branch: {meta.get('branch') or 'unknown'}",
    f"- Commit: {commit_text} ({commit_date})",
  ])
  return "\n".join(lines)[:1900]


async def send_support_webhook(session: ClientSession | None, payload: dict[str, Any]) -> dict[str, Any]:
  if os.environ.get("CARROT_SUPPORT_DISCORD_WEBHOOK_DISABLE", "").strip().lower() in {"1", "true", "yes", "on"}:
    return {"configured": True, "ok": False, "skipped": True, "disabled": True}
  url = support_discord_webhook_url()
  if not url:
    return {"configured": False, "ok": False, "skipped": True}
  if not url.startswith(("http://", "https://")):
    return {"configured": True, "ok": False, "error": "invalid webhook url"}

  body = {
    "username": "Carrot Support",
    "content": _support_message(payload),
    "allowed_mentions": {"parse": []},
    "flags": 4,
  }
  owns_session = session is None
  if session is None:
    session = ClientSession(timeout=ClientTimeout(total=12))
  try:
    async with session.post(url, json=body) as resp:
      text = await resp.text()
      if 200 <= resp.status < 300:
        return {"configured": True, "ok": True, "status": resp.status}
      return {"configured": True, "ok": False, "status": resp.status, "error": text[:500]}
  except Exception as exc:
    return {"configured": True, "ok": False, "error": str(exc)}
  finally:
    if owns_session:
      await session.close()
