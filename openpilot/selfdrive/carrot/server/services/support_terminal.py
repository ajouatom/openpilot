from __future__ import annotations

import asyncio
import base64
import json
import os
import re
import secrets
import socket
import time
from dataclasses import dataclass, field
from typing import Any

from aiohttp import web, WSMsgType

from ..config import TMUX_WEB_SESSION
from ..terminal_commands import translate_meta_command
from . import tmux
from .support_discord import send_support_webhook, support_metadata
from .support_tunnel import TunnelHandle, cloudflared_status, start_quick_tunnel


SUPPORT_TTL_SECONDS = int(os.environ.get("CARROT_SUPPORT_TTL_SECONDS", "1800"))
COMMAND_TIMEOUT_SECONDS = int(os.environ.get("CARROT_SUPPORT_COMMAND_TIMEOUT_SECONDS", "30"))
ALLOWED_TTL_SECONDS = {900, 1800, 3600}
ALLOWED_PERMISSION_MODES = {"approve_each", "allow_all"}
ALLOWED_COMMAND_TIMEOUT_SECONDS = {15, 30, 60, 120}
SCREEN_POLL_SECONDS = 0.25
PIN_FAILURE_LIMIT = 5
PUBLIC_URL_START_DELAY = float(os.environ.get("CARROT_SUPPORT_LINK_START_DELAY_SECONDS", "3.0"))
TMUX_ATTACH_RE = re.compile(r"^\s*tmux\s+(?:a|attach|attach-session)(?:\s*)$", re.IGNORECASE)
TMUX_ATTACH_TARGET_RE = re.compile(r"^\s*tmux\s+(?:a|attach|attach-session)\s+-t\s+\S+\s*$", re.IGNORECASE)


def _now() -> float:
  return time.time()


def _new_pin() -> str:
  return f"{secrets.randbelow(1_000_000):06d}"


def _new_session_id() -> str:
  return secrets.token_urlsafe(16)


def _new_csp_nonce() -> str:
  return base64.b64encode(secrets.token_bytes(16)).decode("ascii")


def _translate_support_terminal_line(line: str) -> str:
  translated = translate_meta_command(line)
  if translated:
    return translated
  text = str(line or "")
  if TMUX_ATTACH_RE.match(text):
    return "TMUX= tmux a -t comma"
  if TMUX_ATTACH_TARGET_RE.match(text):
    return f"TMUX= {text.strip()}"
  return text


def _csp_connect_sources(host: str) -> str:
  safe_host = "".join(ch for ch in str(host or "") if ch.isalnum() or ch in ".-:[]")
  if not safe_host:
    return "'self'"
  return f"'self' ws://{safe_host} wss://{safe_host}"


def _find_free_port() -> int:
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind(("127.0.0.1", 0))
    return int(sock.getsockname()[1])


def _sanitize_ttl_seconds(value: Any) -> int:
  try:
    seconds = int(value)
  except Exception:
    seconds = SUPPORT_TTL_SECONDS
  if seconds not in ALLOWED_TTL_SECONDS:
    seconds = SUPPORT_TTL_SECONDS if SUPPORT_TTL_SECONDS in ALLOWED_TTL_SECONDS else 1800
  return seconds


def _sanitize_permission_mode(value: Any) -> str:
  mode = str(value or "approve_each").strip()
  return mode if mode in ALLOWED_PERMISSION_MODES else "approve_each"


def _sanitize_command_timeout_seconds(value: Any) -> int:
  try:
    seconds = int(value)
  except Exception:
    seconds = COMMAND_TIMEOUT_SECONDS
  if seconds not in ALLOWED_COMMAND_TIMEOUT_SECONDS:
    seconds = COMMAND_TIMEOUT_SECONDS if COMMAND_TIMEOUT_SECONDS in ALLOWED_COMMAND_TIMEOUT_SECONDS else 30
  return seconds


@dataclass
class PendingCommand:
  id: str
  line: str
  created_at: float
  status: str = "pending"
  control_action: str = ""


@dataclass
class SupportSession:
  id: str
  pin: str
  note: str
  created_at: float
  expires_at: float
  ttl_seconds: int
  command_timeout_seconds: int
  state: str = "starting"
  permission_mode: str = "approve_each"
  tunnel_url: str = ""
  local_url: str = ""
  local_origin_url: str = ""
  discord: dict[str, Any] = field(default_factory=dict)
  error: str = ""
  status_detail: str = ""
  pin_failures: int = 0
  guest_count: int = 0
  owner_count: int = 0
  pending_commands: dict[str, PendingCommand] = field(default_factory=dict)
  tunnel: TunnelHandle | None = None
  local_runner: web.AppRunner | None = None
  screen_task: asyncio.Task | None = None
  expiry_task: asyncio.Task | None = None
  owner_sockets: set[web.WebSocketResponse] = field(default_factory=set)
  guest_sockets: set[web.WebSocketResponse] = field(default_factory=set)
  last_screen: str = ""

  def is_expired(self) -> bool:
    if self.expires_at <= 0:
      return False
    return _now() >= self.expires_at


class SupportTerminalManager:
  def __init__(self) -> None:
    self._lock = asyncio.Lock()
    self._session: SupportSession | None = None

  def current(self) -> SupportSession | None:
    return self._session

  def _is_current(self, session: SupportSession) -> bool:
    return self._session is session

  async def _finish_orphaned_start(self, session: SupportSession) -> dict[str, Any]:
    await self._cleanup_session(session)
    return self.snapshot(None)

  async def _abort_if_not_current(self, session: SupportSession) -> bool:
    if self._is_current(session):
      return False
    await self._cleanup_session(session)
    return True

  def snapshot(self, session: SupportSession | None = None, include_secret: bool = False) -> dict[str, Any]:
    session = session or self._session
    if session is None:
      return {"ok": True, "active": False, "state": "idle"}
    remaining = None if session.expires_at <= 0 else max(0, int(session.expires_at - _now()))
    data = {
      "ok": True,
      "active": session.state in {"starting", "sharing"},
      "id": session.id,
      "state": session.state,
      "url": session.tunnel_url,
      "local_url": session.local_url,
      "expires_at": session.expires_at,
      "expires_in": remaining,
      "ttl_seconds": session.ttl_seconds,
      "permission_mode": session.permission_mode,
      "command_timeout_seconds": session.command_timeout_seconds,
      "guest_count": len(session.guest_sockets),
      "owner_count": len(session.owner_sockets),
      "owner_present": bool(session.owner_sockets),
      "discord": session.discord,
      "error": session.error,
      "status_detail": session.status_detail,
      "pending_commands": [
        {"id": command.id, "line": command.line, "status": command.status, "created_at": command.created_at}
        for command in session.pending_commands.values()
        if command.status == "pending"
      ],
    }
    if include_secret:
      data["pin"] = session.pin
    return data

  async def start(
    self,
    app: web.Application,
    note: str = "",
    ttl_seconds: Any = None,
    permission_mode: Any = None,
    command_timeout_seconds: Any = None,
  ) -> dict[str, Any]:
    async with self._lock:
      if self._session and self._session.state in {"starting", "sharing"} and not self._session.is_expired():
        return self.snapshot(self._session)
      if self._session:
        await self._stop_locked("restart")

      ttl = _sanitize_ttl_seconds(ttl_seconds)
      permission = _sanitize_permission_mode(permission_mode)
      command_timeout = _sanitize_command_timeout_seconds(command_timeout_seconds)
      session = SupportSession(
        id=_new_session_id(),
        pin=_new_pin(),
        note=note[:500],
        created_at=_now(),
        expires_at=0 if ttl <= 0 else _now() + ttl,
        ttl_seconds=ttl,
        permission_mode=permission,
        command_timeout_seconds=command_timeout,
      )
      self._session = session

    try:
      await self._set_status(session, "Preparing terminal session")
      await asyncio.to_thread(tmux.ensure_session, TMUX_WEB_SESSION)
      if await self._abort_if_not_current(session):
        return self.snapshot(None)
      await self._set_status(session, "Starting support page")
      await self._start_local_guest_server(session)
      if await self._abort_if_not_current(session):
        return self.snapshot(None)
      cloudflared = cloudflared_status()
      if cloudflared.get("installed"):
        await self._set_status(session, "Starting secure tunnel")
      elif cloudflared.get("auto_install") and cloudflared.get("download_supported"):
        await self._set_status(session, "Downloading cloudflared")
      else:
        await self._set_status(session, "cloudflared unavailable")
      session.tunnel = await start_quick_tunnel(session.local_origin_url)
      if await self._abort_if_not_current(session):
        return self.snapshot(None)
      session.tunnel_url = self._public_session_url(session.tunnel.url, session.id)
      session.state = "sharing"
      await self._set_status(session, "Secure tunnel ready")
      if PUBLIC_URL_START_DELAY > 0:
        await asyncio.sleep(PUBLIC_URL_START_DELAY)
      if await self._abort_if_not_current(session):
        return self.snapshot(None)
      session.screen_task = asyncio.create_task(self._screen_loop(session))
      if session.expires_at > 0:
        session.expiry_task = asyncio.create_task(self._expiry_loop(session))
      await self._set_status(session, "Sending Carrot server notification")
      metadata = await asyncio.to_thread(support_metadata)
      if await self._abort_if_not_current(session):
        return self.snapshot(None)
      session.discord = await send_support_webhook(app.get("http"), {
        "server": "Carrot",
        "device": os.environ.get("CARROT_DEVICE_NAME", socket.gethostname()),
        "sessionId": session.id,
        "createdAt": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(session.created_at)),
        "url": session.tunnel_url,
        "pin": session.pin,
        "ttl_minutes": "unlimited" if session.ttl_seconds <= 0 else max(1, session.ttl_seconds // 60),
        "permissionMode": session.permission_mode,
        "commandTimeoutSeconds": session.command_timeout_seconds,
        "tmuxSession": TMUX_WEB_SESSION,
        "meta": metadata,
        "note": session.note,
      })
      if await self._abort_if_not_current(session):
        return self.snapshot(None)
      await self._set_status(session, "Ready")
      await self.broadcast_owner({"type": "session_status", **self.snapshot(session)})
      return self.snapshot(session)
    except Exception as exc:
      if self._is_current(session):
        session.state = "error"
        session.error = str(exc)
        session.status_detail = "Start failed"
        await self._cleanup_session(session)
        await self.broadcast_owner({"type": "error", "message": session.error})
        return self.snapshot(session)
      return await self._finish_orphaned_start(session)

  async def stop(self, reason: str = "stopped") -> dict[str, Any]:
    async with self._lock:
      await self._stop_locked(reason)
      return self.snapshot(None)

  async def _stop_locked(self, reason: str) -> None:
    session = self._session
    if session is None:
      return
    session.state = "stopped" if reason != "expired" else "expired"
    await self.broadcast_all({"type": "session_closed", "reason": reason})
    self._session = None
    asyncio.create_task(self._cleanup_session(session))

  async def _cleanup_session(self, session: SupportSession) -> None:
    for task in (session.screen_task, session.expiry_task):
      if task:
        task.cancel()
    for ws in list(session.owner_sockets | session.guest_sockets):
      try:
        await ws.close()
      except Exception:
        pass
    session.owner_sockets.clear()
    session.guest_sockets.clear()
    if session.tunnel:
      await session.tunnel.stop()
      session.tunnel = None
    if session.local_runner:
      try:
        await session.local_runner.cleanup()
      except Exception:
        pass
      session.local_runner = None

  async def _start_local_guest_server(self, session: SupportSession) -> None:
    app = web.Application()
    app.router.add_get("/", self.handle_guest_page)
    app.router.add_get("/support/terminal/{session_id}", self.handle_guest_page)
    app.router.add_get("/ws/support_terminal/{session_id}", self.handle_guest_ws)
    runner = web.AppRunner(app)
    await runner.setup()
    port = _find_free_port()
    site = web.TCPSite(runner, "127.0.0.1", port)
    await site.start()
    session.local_runner = runner
    session.local_origin_url = f"http://127.0.0.1:{port}"
    session.local_url = f"{session.local_origin_url}/support/terminal/{session.id}"

  def _public_session_url(self, tunnel_url: str, session_id: str) -> str:
    base = str(tunnel_url or "").rstrip("/")
    if "/support/terminal/" in base:
      return base
    return f"{base}/support/terminal/{session_id}"

  async def _set_status(self, session: SupportSession, detail: str) -> None:
    session.status_detail = detail
    if self._session is session:
      await self.broadcast_owner({"type": "session_status", **self.snapshot(session)})

  async def _expiry_loop(self, session: SupportSession) -> None:
    while not session.is_expired():
      await self.broadcast_owner({"type": "session_status", **self.snapshot(session)})
      await asyncio.sleep(1.0)
    async with self._lock:
      if self._session is session:
        await self._stop_locked("expired")

  async def _screen_loop(self, session: SupportSession) -> None:
    while session.state in {"starting", "sharing"}:
      try:
        screen = await asyncio.to_thread(tmux.capture, TMUX_WEB_SESSION)
        if screen != session.last_screen:
          session.last_screen = screen
          await self.broadcast_guests({"type": "screen", "text": screen})
      except Exception as exc:
        await self.broadcast_guests({"type": "error", "message": str(exc)})
      await asyncio.sleep(SCREEN_POLL_SECONDS)

  async def broadcast_owner(self, payload: dict[str, Any]) -> None:
    session = self._session
    if session is None:
      return
    await self._broadcast(session.owner_sockets, payload)

  async def broadcast_guests(self, payload: dict[str, Any]) -> None:
    session = self._session
    if session is None:
      return
    await self._broadcast(session.guest_sockets, payload)

  async def broadcast_all(self, payload: dict[str, Any]) -> None:
    session = self._session
    if session is None:
      return
    await self._broadcast(session.owner_sockets | session.guest_sockets, payload)

  async def _broadcast(self, sockets: set[web.WebSocketResponse], payload: dict[str, Any]) -> None:
    dead: list[web.WebSocketResponse] = []
    text = json.dumps(payload, ensure_ascii=False)
    for ws in list(sockets):
      if ws.closed:
        dead.append(ws)
        continue
      try:
        await ws.send_str(text)
      except Exception:
        dead.append(ws)
    for ws in dead:
      sockets.discard(ws)

  async def handle_owner_ws(self, request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse(heartbeat=20, compress=False)
    await ws.prepare(request)
    session = self._session
    if session is None:
      await ws.send_str(json.dumps({"type": "session_status", **self.snapshot(None)}))
      await ws.close()
      return ws
    session.owner_sockets.add(ws)
    await ws.send_str(json.dumps({"type": "session_status", **self.snapshot(session)}, ensure_ascii=False))
    await self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_sockets)})
    await self.broadcast_guests({"type": "owner_presence", "active": True})
    try:
      async for msg in ws:
        if msg.type == WSMsgType.TEXT:
          try:
            data = json.loads(msg.data)
          except Exception:
            continue
          if data.get("type") == "refresh":
            await ws.send_str(json.dumps({"type": "session_status", **self.snapshot(session)}, ensure_ascii=False))
    finally:
      session.owner_sockets.discard(ws)
      await self.broadcast_guests({"type": "owner_presence", "active": bool(session.owner_sockets)})
    return ws

  async def handle_guest_page(self, request: web.Request) -> web.Response:
    nonce = _new_csp_nonce()
    html = (
      GUEST_HTML
      .replace("__SESSION_ID__", request.match_info.get("session_id", ""))
      .replace("__NONCE__", nonce)
    )
    csp = (
      "default-src 'none'; "
      f"script-src 'nonce-{nonce}'; "
      f"style-src 'nonce-{nonce}'; "
      f"connect-src {_csp_connect_sources(request.host)}; "
      "base-uri 'none'; form-action 'self'; frame-ancestors 'none'; "
      "require-trusted-types-for 'script'; trusted-types 'none'"
    )
    headers = {
      "Cache-Control": "no-store",
      "Content-Security-Policy": csp,
      "Permissions-Policy": "camera=(), microphone=(), geolocation=(), payment=(), usb=(), serial=(), hid=(), bluetooth=()",
      "Referrer-Policy": "no-referrer",
      "X-Content-Type-Options": "nosniff",
      "X-Frame-Options": "DENY",
    }
    return web.Response(text=html, content_type="text/html", headers=headers)

  async def handle_guest_ws(self, request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse(heartbeat=20, compress=False)
    await ws.prepare(request)
    session = self._session
    session_id = request.match_info.get("session_id", "")
    if session is None or session.id != session_id or session.is_expired():
      await ws.send_str(json.dumps({"type": "error", "message": "session unavailable"}))
      await ws.close()
      return ws

    authed = False
    try:
      async for msg in ws:
        if msg.type != WSMsgType.TEXT:
          continue
        try:
          data = json.loads(msg.data)
        except Exception:
          continue
        typ = data.get("type")
        if not authed:
          if typ != "auth":
            continue
          pin = str(data.get("pin") or "").strip()
          if pin != session.pin:
            session.pin_failures += 1
            await ws.send_str(json.dumps({"type": "auth_failed", "remaining": max(0, PIN_FAILURE_LIMIT - session.pin_failures)}))
            if session.pin_failures >= PIN_FAILURE_LIMIT:
              await ws.close()
              return ws
            continue
          authed = True
          session.guest_sockets.add(ws)
          expires_in = None if session.expires_at <= 0 else max(0, int(session.expires_at - _now()))
          await ws.send_str(json.dumps({
            "type": "auth_ok",
            "permission_mode": session.permission_mode,
            "command_timeout_seconds": session.command_timeout_seconds,
            "expires_in": expires_in,
            "owner_present": bool(session.owner_sockets),
          }))
          if session.last_screen:
            await ws.send_str(json.dumps({"type": "screen", "text": session.last_screen}))
          await self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_sockets)})
          continue

        if typ == "typing":
          text = str(data.get("text") or "")[:160]
          await self.broadcast_owner({"type": "guest_typing", "active": bool(data.get("active")), "text": text})
        elif typ == "close_session":
          await self.stop("guest_closed")
          return ws
        elif typ == "control":
          action = str(data.get("action") or "").strip()
          if action in {"ctrl_c", "clear"}:
            label = "Ctrl+C" if action == "ctrl_c" else "clear"
            await self.queue_guest_command(session, label, ws, control_action=action)
        elif typ == "input":
          line = str(data.get("data") or "").strip()
          if line:
            await self.queue_guest_command(session, line, ws)
    finally:
      session.guest_sockets.discard(ws)
      await self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_sockets)})
    return ws

  async def queue_guest_command(self, session: SupportSession, line: str, ws: web.WebSocketResponse | None = None, control_action: str = "") -> dict[str, Any]:
    if not session.owner_sockets:
      if ws is not None and not ws.closed:
        await ws.send_str(json.dumps({"type": "owner_absent"}))
      return {"ok": False, "error": "owner not present"}
    if session.permission_mode == "allow_all":
      command = PendingCommand(id=secrets.token_urlsafe(8), line=line[:4000], created_at=_now(), status="approved", control_action=control_action)
      if ws is not None and not ws.closed:
        await ws.send_str(json.dumps({"type": "command_approved", "id": command.id}))
      result = await self._run_command(session, command)
      await self.broadcast_owner({"type": "command_auto_run", "id": command.id, "line": command.line, "created_at": command.created_at})
      return result
    command = PendingCommand(id=secrets.token_urlsafe(8), line=line[:4000], created_at=_now(), control_action=control_action)
    session.pending_commands[command.id] = command
    payload = {"type": "command_request", "id": command.id, "line": command.line, "created_at": command.created_at, "control_action": command.control_action}
    await self.broadcast_owner(payload)
    if ws is not None and not ws.closed:
      await ws.send_str(json.dumps({"type": "command_waiting_approval", "id": command.id}))
    asyncio.create_task(self._expire_command(session, command.id))
    return {"ok": True, "command": payload}

  async def _expire_command(self, session: SupportSession, command_id: str) -> None:
    await asyncio.sleep(session.command_timeout_seconds)
    command = session.pending_commands.get(command_id)
    if command is None or command.status != "pending":
      return
    command.status = "expired"
    session.pending_commands.pop(command_id, None)
    await self.broadcast_all({"type": "command_expired", "id": command_id})

  async def approve_command(self, command_id: str) -> dict[str, Any]:
    session = self._session
    if session is None:
      return {"ok": False, "error": "no active session"}
    command = session.pending_commands.get(command_id)
    if command is None or command.status != "pending":
      return {"ok": False, "error": "command not pending"}
    command.status = "approved"
    session.pending_commands.pop(command_id, None)
    await self.broadcast_all({"type": "command_approved", "id": command.id})
    return await self._run_command(session, command)

  async def _run_command(self, session: SupportSession, command: PendingCommand) -> dict[str, Any]:
    try:
      if command.control_action == "ctrl_c":
        await asyncio.to_thread(tmux.ctrl_c, TMUX_WEB_SESSION)
      elif command.control_action == "clear":
        await asyncio.to_thread(tmux.clear, TMUX_WEB_SESSION)
      else:
        await asyncio.to_thread(tmux.send_line, TMUX_WEB_SESSION, _translate_support_terminal_line(command.line))
      await self.broadcast_all({"type": "command_running", "id": command.id})
      return {"ok": True}
    except Exception as exc:
      await self.broadcast_all({"type": "command_failed", "id": command.id, "message": str(exc)})
      return {"ok": False, "error": str(exc)}

  async def reject_command(self, command_id: str) -> dict[str, Any]:
    session = self._session
    if session is None:
      return {"ok": False, "error": "no active session"}
    command = session.pending_commands.get(command_id)
    if command is None or command.status != "pending":
      return {"ok": False, "error": "command not pending"}
    command.status = "rejected"
    session.pending_commands.pop(command_id, None)
    await self.broadcast_all({"type": "command_rejected", "id": command.id})
    return {"ok": True}


GUEST_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <title>Carrot Remote Terminal</title>
  <style nonce="__NONCE__">
    :root {
      color-scheme: dark;
      --md-surface: #0b0f14;
      --md-surface-cont: #141a21;
      --md-on-surface: #f4f6f8;
      --md-on-surface-var: #c4ccd4;
      --md-outline: #98a3ad;
      --md-primary: #c57a3d;
      --md-stroke-soft: #54606c;
      --control-radius: 8px;
      --terminal-inline: 12px;
      --popup-item-radius: var(--control-radius);
      --popup-item-border: color-mix(in srgb, var(--md-outline) 46%, transparent);
      --popup-item-bg: var(--md-surface-cont);
      --shadow-1: 0 1px 2px rgba(0, 0, 0, 0.18), 0 4px 10px rgba(0, 0, 0, 0.16);
      --font-mono: ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      background: var(--md-surface);
      color: var(--md-on-surface);
    }
    * { box-sizing: border-box; }
    html, body { width: 100%; height: 100%; }
    body { margin: 0; overflow: hidden; background: var(--md-surface); }
    button, input { font: inherit; }
    .page--terminal {
      position: fixed;
      inset: 0;
      width: 100dvw;
      height: 100dvh;
      display: flex;
      flex-direction: column;
      min-height: 0;
      overflow: hidden;
    }
    .terminal-shell {
      width: 100%;
      height: 100%;
      min-height: 0;
      display: flex;
      flex-direction: column;
      background: color-mix(in srgb, var(--md-surface) 92%, #000);
      overflow: hidden;
      position: relative;
    }
    .terminal-auth {
      position: absolute;
      inset: 0;
      z-index: 5;
      display: grid;
      place-items: center;
      padding: clamp(18px, 6dvw, 56px);
      background: color-mix(in srgb, var(--md-surface) 94%, #000);
      opacity: 1;
      transform: scale(1);
      transition: opacity 180ms ease, transform 180ms ease, visibility 180ms ease;
      visibility: visible;
    }
    .terminal-auth.is-hiding {
      opacity: 0;
      transform: scale(0.985);
      visibility: hidden;
    }
    .terminal-auth[hidden] {
      display: none !important;
    }
    .terminal-auth__panel {
      --auth-control-height: clamp(58px, 9dvh, 82px);
      --auth-input-size: clamp(24px, 5.2dvw, 34px);
      --auth-button-size: clamp(16px, 2.8dvw, 20px);
      width: min(92dvw, 560px);
      display: grid;
      gap: clamp(16px, 3dvh, 26px);
      padding: 0;
      border: 0;
      border-radius: 0;
      background: transparent;
      box-shadow: none;
    }
    .terminal-auth__status {
      min-height: 28px;
      color: var(--md-on-surface);
      font-size: clamp(18px, 3.2dvw, 24px);
      font-weight: 900;
      line-height: 1.25;
      text-align: center;
    }
    .terminal-auth__form {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 0;
      align-items: center;
      border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 64%, transparent);
      border-radius: var(--control-radius);
      background: color-mix(in srgb, var(--md-surface-cont) 88%, #000);
      overflow: hidden;
      box-shadow: 0 12px 30px color-mix(in srgb, #000 26%, transparent);
    }
    .terminal-auth__input {
      min-width: 0;
      min-height: var(--auth-control-height);
      padding: 12px clamp(16px, 4dvw, 28px);
      border: 0;
      background: transparent;
      color: var(--md-on-surface);
      font-family: var(--font-mono);
      font-size: var(--auth-input-size);
      font-weight: 800;
      letter-spacing: 0.08em;
      outline: none;
      text-align: center;
    }
    .terminal-auth__input::placeholder { color: var(--md-outline); }
    .terminal-auth__button {
      min-height: var(--auth-control-height);
      padding: 0 clamp(24px, 5dvw, 38px);
      border: 0;
      border-left: 1px solid color-mix(in srgb, var(--md-stroke-soft) 44%, transparent);
      background: color-mix(in srgb, var(--md-primary) 86%, #000 14%);
      color: #fff;
      font-size: var(--auth-button-size);
      font-weight: 850;
      cursor: pointer;
    }
    .terminal-head {
      flex: 0 0 auto;
      background: color-mix(in srgb, var(--md-surface) 92%, #000);
      box-shadow: 0 8px 18px color-mix(in srgb, #000 22%, transparent);
      z-index: 2;
    }
    .terminal-meta {
      margin: 0;
      padding: 12px var(--terminal-inline) 8px;
      color: var(--md-on-surface);
      font-size: 15px;
      font-weight: 800;
    }
    .terminal-toolbar {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 8px;
      align-items: center;
      padding: 8px var(--terminal-inline);
      border-top: 1px solid color-mix(in srgb, var(--md-stroke-soft) 18%, transparent);
      border-bottom: 1px solid color-mix(in srgb, var(--md-stroke-soft) 42%, transparent);
    }
    .terminal-sessionMeta {
      min-width: 0;
      color: var(--md-outline);
      font-family: var(--font-mono);
      font-size: 12px;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
    }
    .terminal-toolbar__actions {
      display: inline-flex;
      flex-wrap: wrap;
      gap: 6px;
      justify-content: flex-end;
      min-width: 0;
    }
    .terminal-toolbar__actions .smallBtn {
      align-self: auto;
      min-height: 34px;
      padding: 6px 12px;
      border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 54%, transparent);
      border-radius: 8px;
      background: color-mix(in srgb, var(--md-surface-cont-h) 88%, transparent);
      color: var(--md-on-surface);
      font-size: 12px;
      white-space: nowrap;
    }
    .status-chip {
      min-height: 26px;
      display: inline-flex;
      align-items: center;
      padding: 2px 9px;
      border-radius: 999px;
      background: color-mix(in srgb, var(--md-surface-cont) 72%, transparent);
      color: var(--md-on-surface-var);
      font-size: 12px;
      font-weight: 750;
      white-space: nowrap;
    }
    .terminal-screen {
      flex: 1 1 auto;
      min-height: 0;
      overflow: auto;
      padding: 8px var(--terminal-inline);
      overscroll-behavior: contain;
    }
    .terminal-output {
      margin: 0;
      min-height: 100%;
      display: inline-block;
      width: max-content;
      min-width: 100%;
      background: transparent;
      color: var(--md-on-surface);
      font-family: var(--font-mono);
      font-size: 13px;
      line-height: 1.45;
      white-space: pre;
      word-break: normal;
    }
    .terminal-form {
      flex: 0 0 auto;
      display: grid;
      grid-template-columns: auto minmax(0, 1fr) auto;
      gap: 8px;
      align-items: center;
      margin: 6px var(--terminal-inline) calc(8px + env(safe-area-inset-bottom, 0px));
      padding-left: 12px;
      border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 56%, transparent);
      border-radius: var(--control-radius);
      background: color-mix(in srgb, var(--md-surface-cont) 92%, #000);
      overflow: hidden;
    }
    .terminal-form:focus-within {
      border-color: color-mix(in srgb, var(--md-on-surface-var) 72%, transparent);
    }
    .terminal-form__prompt {
      color: var(--md-primary);
      font-family: var(--font-mono);
      font-size: 14px;
      font-weight: 800;
    }
    .terminal-commandNotice {
      flex: 0 0 auto;
      justify-self: start;
      max-width: calc(100% - var(--terminal-inline) * 2);
      margin: 0 var(--terminal-inline) 6px;
      padding: 7px 10px;
      border: 1px solid var(--popup-item-border, color-mix(in srgb, var(--md-stroke-soft) 48%, transparent));
      border-radius: var(--popup-item-radius, var(--control-radius));
      background: var(--popup-item-bg, color-mix(in srgb, var(--md-surface-cont) 92%, #000));
      color: var(--md-on-surface-var);
      font-size: 12px;
      font-weight: 850;
      line-height: 1.2;
      white-space: nowrap;
      overflow: hidden;
      text-overflow: ellipsis;
      box-shadow: var(--shadow-1, 0 10px 24px color-mix(in srgb, #000 22%, transparent));
    }
    .terminal-form__mode {
      min-height: 26px;
      display: inline-flex;
      align-items: center;
      padding: 2px 8px;
      border: 1px solid color-mix(in srgb, var(--md-stroke-soft) 44%, transparent);
      border-radius: 999px;
      background: color-mix(in srgb, var(--md-surface-cont-h) 72%, transparent);
      color: var(--md-on-surface-var);
      font-size: 11px;
      font-weight: 850;
      white-space: nowrap;
    }
    .terminal-form__input {
      min-width: 0;
      min-height: 38px;
      padding: 8px 0;
      border: 0;
      background: transparent;
      color: var(--md-on-surface);
      font-family: var(--font-mono);
      font-size: 14px;
      font-weight: 650;
      outline: none;
    }
    .terminal-form__input::placeholder { color: var(--md-outline); }
    .smallBtn {
      align-self: stretch;
      min-height: 100%;
      padding: 0 16px;
      border: 0;
      border-left: 1px solid color-mix(in srgb, var(--md-stroke-soft) 44%, transparent);
      border-radius: 0;
      background: color-mix(in srgb, var(--md-primary) 16%, transparent);
      color: var(--md-primary);
      font-size: 13px;
      font-weight: 800;
      cursor: pointer;
    }
    .hidden { display: none !important; }
    .terminal-form {
      opacity: 1;
      transform: translateY(0);
      transition: opacity 180ms ease, transform 180ms ease;
    }
    .terminal-form.is-entering {
      opacity: 0;
      transform: translateY(6px);
    }
    @media (max-width: 520px) {
      :root { --terminal-inline: 10px; }
      .terminal-output { font-size: 11px; line-height: 1.35; }
      .terminal-form__input { font-size: 13px; }
      .smallBtn { padding: 0 13px; font-size: 12px; }
      .terminal-toolbar__actions .smallBtn { min-height: 32px; padding: 6px 8px; font-size: 11px; }
      .status-chip { min-height: 24px; padding: 2px 7px; font-size: 11px; }
    }
  </style>
</head>
<body>
  <main class="page--terminal">
    <div class="terminal-shell">
      <div class="terminal-head">
        <div class="terminal-meta">Carrot Remote Terminal</div>
        <div class="terminal-toolbar">
          <div id="status" class="terminal-sessionMeta">PIN required</div>
          <div class="terminal-toolbar__actions">
            <button id="btnCtrlC" class="smallBtn" type="button">Ctrl+C</button>
            <button id="btnClear" class="smallBtn" type="button">Clear</button>
            <button id="btnStop" class="smallBtn" type="button">End</button>
          </div>
        </div>
      </div>
      <form id="auth" class="terminal-auth">
        <div class="terminal-auth__panel">
          <div id="authStatus" class="terminal-auth__status">PIN required</div>
          <div class="terminal-auth__form">
            <input id="pin" class="terminal-auth__input" inputmode="numeric" autocomplete="one-time-code" maxlength="6" placeholder="000000">
            <button id="connectBtn" class="terminal-auth__button" type="submit">Connect</button>
          </div>
        </div>
      </form>
      <div id="terminalScreen" class="terminal-screen">
        <pre id="screen" class="terminal-output">Not connected</pre>
      </div>
      <div id="commandNotice" class="terminal-commandNotice hidden"></div>
      <form id="cmd" class="terminal-form hidden">
        <span id="permission" class="terminal-form__mode hidden">Owner approval required</span>
        <span class="terminal-form__prompt">$</span>
        <input id="line" class="terminal-form__input" autocomplete="off" placeholder="">
        <button id="sendBtn" class="smallBtn" type="submit">Send</button>
      </form>
    </div>
  </main>
  <script nonce="__NONCE__">
    const strings = {
      en: {
        pinRequired: "PIN required",
        approval: "Owner approval required",
        approvalAllowAll: "Allow all",
        connect: "Connect",
        ctrlC: "Ctrl+C",
        clear: "Clear",
        endSession: "End Remote",
        send: "Send",
        notConnected: "Not connected",
        commandPlaceholder: "Command waits for owner approval",
        commandPlaceholderAllowAll: "Command runs immediately",
        connected: "Connected - commands require owner approval",
        connectedAllowAll: "Connected - commands run immediately",
        connectedWithTime: "Connected · {time}",
        connectedUnlimited: "Connected · unlimited",
        timeUnlimited: "unlimited",
        hostAway: "Host is not viewing terminal",
        hostAwayWithTime: "Host is not viewing terminal · {time}",
        hostAwayUnlimited: "Host is not viewing terminal · unlimited",
        approvalWithTimeout: "Owner approval · {seconds}s",
        pinFailed: "PIN failed - {remaining} tries left",
        waiting: "Waiting for owner approval",
        approved: "Approved · sending to terminal",
        running: "Sent to terminal",
        failed: "Failed to send command",
        rejected: "Command rejected",
        expired: "Command approval expired",
        closed: "Session closed",
        disconnected: "Disconnected",
        consoleBlocked: "Developer tools detected - enter PIN again",
        consoleWarningTitle: "Warning",
        consoleWarningBody: "Console use is prohibited on this page.",
        error: "Error"
      },
      ko: {
        pinRequired: "PIN 필요",
        approval: "소유자 승인 필요",
        approvalAllowAll: "전체 허용",
        connect: "연결",
        ctrlC: "Ctrl+C",
        clear: "Clear",
        endSession: "원격 종료",
        send: "전송",
        notConnected: "연결되지 않음",
        commandPlaceholder: "명령은 소유자 승인 후 실행됩니다",
        commandPlaceholderAllowAll: "명령이 즉시 실행됩니다",
        connected: "연결됨 - 명령은 소유자 승인이 필요합니다",
        connectedAllowAll: "연결됨 - 명령이 즉시 실행됩니다",
        connectedWithTime: "연결됨 · {time}",
        connectedUnlimited: "연결됨 · 무제한",
        timeUnlimited: "무제한",
        hostAway: "호스트가 터미널 페이지를 보고 있지 않습니다",
        hostAwayWithTime: "호스트가 터미널 페이지를 보고 있지 않습니다 · {time}",
        hostAwayUnlimited: "호스트가 터미널 페이지를 보고 있지 않습니다 · 무제한",
        approvalWithTimeout: "소유자 승인 · {seconds}초",
        pinFailed: "PIN 실패 - {remaining}회 남음",
        waiting: "소유자 승인 대기 중",
        approved: "승인됨 · 터미널로 전송 중",
        running: "터미널로 전송됨",
        failed: "명령 전송 실패",
        rejected: "명령 거절됨",
        expired: "명령 승인 만료",
        closed: "세션 닫힘",
        disconnected: "연결 끊김",
        consoleBlocked: "개발자 도구 감지됨 - PIN을 다시 입력하세요",
        consoleWarningTitle: "경고",
        consoleWarningBody: "콘솔 사용이 금지되어 있습니다.",
        error: "오류"
      },
      zh: {
        pinRequired: "需要 PIN",
        approval: "需要车主批准",
        approvalAllowAll: "全部允许",
        connect: "连接",
        ctrlC: "Ctrl+C",
        clear: "清除",
        endSession: "结束远程",
        send: "发送",
        notConnected: "未连接",
        commandPlaceholder: "命令需要车主批准",
        commandPlaceholderAllowAll: "命令会立即运行",
        connected: "已连接 - 命令需要车主批准",
        connectedAllowAll: "已连接 - 命令会立即运行",
        connectedWithTime: "已连接 · {time}",
        connectedUnlimited: "已连接 · 无限",
        timeUnlimited: "无限",
        hostAway: "车主未查看终端页面",
        hostAwayWithTime: "车主未查看终端页面 · {time}",
        hostAwayUnlimited: "车主未查看终端页面 · 无限",
        approvalWithTimeout: "车主批准 · {seconds}秒",
        pinFailed: "PIN 错误 - 剩余 {remaining} 次",
        waiting: "等待车主批准",
        approved: "已批准 · 正在发送到终端",
        running: "已发送到终端",
        failed: "命令发送失败",
        rejected: "命令已拒绝",
        expired: "命令批准已过期",
        closed: "会话已关闭",
        disconnected: "已断开",
        consoleBlocked: "检测到开发者工具 - 请重新输入 PIN",
        consoleWarningTitle: "警告",
        consoleWarningBody: "此页面禁止使用控制台。",
        error: "错误"
      }
    };
    const language = (navigator.language || "").toLowerCase();
    const lang = language.startsWith("ko") ? "ko" : language.startsWith("zh") ? "zh" : "en";
    function tx(key, vars) {
      let value = (strings[lang] && strings[lang][key]) || strings.en[key] || key;
      if (vars) Object.keys(vars).forEach((name) => { value = value.replaceAll(`{${name}}`, String(vars[name])); });
      return value;
    }
    const sessionId = "__SESSION_ID__" || location.pathname.split("/").pop();
    const statusEl = document.getElementById("status");
    const authStatusEl = document.getElementById("authStatus");
    const permissionEl = document.getElementById("permission");
    const authForm = document.getElementById("auth");
    const pinInput = document.getElementById("pin");
    const connectBtn = document.getElementById("connectBtn");
    const btnCtrlC = document.getElementById("btnCtrlC");
    const btnClear = document.getElementById("btnClear");
    const btnStop = document.getElementById("btnStop");
    const sendBtn = document.getElementById("sendBtn");
    const screenEl = document.getElementById("screen");
    const screenWrap = document.getElementById("terminalScreen");
    const commandNoticeEl = document.getElementById("commandNotice");
    const cmdForm = document.getElementById("cmd");
    const lineInput = document.getElementById("line");
    let ws = null;
    let typingTimer = 0;
    let remainingTimer = 0;
    let remainingDeadlineMs = 0;
    let allowAllMode = false;
    let commandTimeoutSeconds = 30;
    let ownerPresent = false;
    let commandNoticeMessage = "";
    let commandNoticeTimer = 0;
    let sessionClosed = false;
    let consoleBlocked = false;
    function setStatus(text, updateAuth = true) {
      statusEl.textContent = text;
      if (updateAuth) authStatusEl.textContent = text;
    }
    function formatRemaining(seconds) {
      if (seconds == null) return tx("timeUnlimited");
      const total = Math.max(0, Math.ceil(Number(seconds || 0)));
      const min = Math.floor(total / 60);
      const sec = total % 60;
      if (min <= 0) return `${sec}s`;
      return `${min}:${String(sec).padStart(2, "0")}`;
    }
    function clearRemainingTimer() {
      if (!remainingTimer) return;
      clearInterval(remainingTimer);
      remainingTimer = 0;
    }
    function clearCommandNoticeTimer() {
      if (!commandNoticeTimer) return;
      clearTimeout(commandNoticeTimer);
      commandNoticeTimer = 0;
    }
    function socketReady() {
      return ws && ws.readyState === WebSocket.OPEN;
    }
    function currentRemainingSeconds() {
      if (!remainingDeadlineMs) return null;
      return Math.max(0, Math.ceil((remainingDeadlineMs - Date.now()) / 1000));
    }
    function syncGuestControls() {
      const commandVisible = !cmdForm.classList.contains("hidden");
      const enabled = commandVisible && ownerPresent && socketReady();
      cmdForm.classList.toggle("is-disabled", commandVisible && !enabled);
      lineInput.disabled = !enabled;
      sendBtn.disabled = !enabled;
      btnCtrlC.disabled = !enabled;
      btnClear.disabled = !enabled;
      permissionEl.textContent = ownerPresent
        ? (allowAllMode ? tx("approvalAllowAll") : tx("approvalWithTimeout", { seconds: commandTimeoutSeconds }))
        : tx("hostAway");
      const notice = ownerPresent ? commandNoticeMessage : tx("hostAway");
      commandNoticeEl.hidden = !commandVisible || !notice;
      commandNoticeEl.classList.toggle("hidden", !commandVisible || !notice);
      if (commandVisible && notice) commandNoticeEl.textContent = notice;
    }
    function setCommandNotice(message, durationMs = 0) {
      clearCommandNoticeTimer();
      commandNoticeMessage = String(message || "");
      if (!message) {
        commandNoticeEl.hidden = true;
        commandNoticeEl.classList.add("hidden");
        commandNoticeEl.textContent = "";
        return;
      }
      commandNoticeEl.textContent = message;
      commandNoticeEl.hidden = false;
      commandNoticeEl.classList.remove("hidden");
      if (durationMs > 0) {
        commandNoticeTimer = setTimeout(() => setCommandNotice(""), durationMs);
      }
    }
    function showAuthPanel() {
      authForm.hidden = false;
      authForm.classList.remove("hidden");
      requestAnimationFrame(() => authForm.classList.remove("is-hiding"));
    }
    function hideAuthPanel() {
      authForm.classList.add("is-hiding");
      window.setTimeout(() => {
        if (authForm.classList.contains("is-hiding")) authForm.hidden = true;
      }, 190);
    }
    function showCommandForm() {
      cmdForm.classList.remove("hidden");
      cmdForm.classList.add("is-entering");
      requestAnimationFrame(() => cmdForm.classList.remove("is-entering"));
    }
    function hideCommandForm() {
      cmdForm.classList.add("hidden");
      cmdForm.classList.remove("is-entering");
    }
    function updateRemainingStatus() {
      const seconds = currentRemainingSeconds();
      if (!ownerPresent) {
        setStatus(seconds == null ? tx("hostAwayUnlimited") : tx("hostAwayWithTime", { time: formatRemaining(seconds) }), false);
        syncGuestControls();
        return;
      }
      if (seconds == null) {
        setStatus(tx("connectedUnlimited"), false);
        syncGuestControls();
        return;
      }
      setStatus(tx("connectedWithTime", { time: formatRemaining(seconds) }), false);
      syncGuestControls();
      if (seconds <= 0) resetToPin(tx("expired"));
    }
    function startRemainingTimer(expiresIn) {
      clearRemainingTimer();
      remainingDeadlineMs = expiresIn == null ? 0 : Date.now() + Math.max(0, Number(expiresIn || 0)) * 1000;
      updateRemainingStatus();
      remainingTimer = setInterval(updateRemainingStatus, 1000);
    }
    function showConsoleWarning() {
      try {
        console.log(`%c${tx("consoleWarningTitle")}`, "color:#ff4d4f;font-size:42px;font-weight:900;");
        console.log(`%c${tx("consoleWarningBody")}`, "color:#ffb86b;font-size:16px;font-weight:700;");
      } catch {}
    }
    function resetToPin(message) {
      clearTimeout(typingTimer);
      clearCommandNoticeTimer();
      clearRemainingTimer();
      typingTimer = 0;
      remainingDeadlineMs = 0;
      allowAllMode = false;
      commandTimeoutSeconds = 30;
      ownerPresent = false;
      commandNoticeMessage = "";
      sessionClosed = true;
      if (ws) {
        try {
          if (ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "typing", active: false, text: "" }));
          ws.close();
        } catch {}
      }
      ws = null;
      lineInput.value = "";
      pinInput.value = "";
      hideCommandForm();
      commandNoticeEl.hidden = true;
      commandNoticeEl.classList.add("hidden");
      syncGuestControls();
      showAuthPanel();
      setStatus(message || tx("pinRequired"));
      pinInput.focus();
    }
    function blockForConsole() {
      if (consoleBlocked) return;
      consoleBlocked = true;
      resetToPin(tx("consoleBlocked"));
    }
    function isDevToolsLikelyOpen() {
      const widthGap = window.outerWidth > 0 ? Math.abs(window.outerWidth - window.innerWidth) : 0;
      const heightGap = window.outerHeight > 0 ? Math.abs(window.outerHeight - window.innerHeight) : 0;
      return widthGap > 170 || heightGap > 170;
    }
    function checkDevToolsOpen() {
      if (isDevToolsLikelyOpen()) {
        blockForConsole();
        return true;
      } else if (consoleBlocked) {
        consoleBlocked = false;
        setStatus(tx("pinRequired"));
      }
      return false;
    }
    function localizeStaticText() {
      setStatus(tx("pinRequired"));
      permissionEl.textContent = tx("approval");
      connectBtn.textContent = tx("connect");
      btnCtrlC.textContent = tx("ctrlC");
      btnClear.textContent = tx("clear");
      btnStop.textContent = tx("endSession");
      sendBtn.textContent = tx("send");
      screenEl.textContent = tx("notConnected");
      lineInput.placeholder = "";
    }
    function connect(pin) {
      if (consoleBlocked || checkDevToolsOpen()) return;
      sessionClosed = false;
      if (ws) {
        try { ws.close(); } catch {}
      }
      const proto = location.protocol === "https:" ? "wss" : "ws";
      ws = new WebSocket(`${proto}://${location.host}/ws/support_terminal/${encodeURIComponent(sessionId)}`);
      ws.onopen = () => {
        if (consoleBlocked || checkDevToolsOpen()) return;
        ws.send(JSON.stringify({ type: "auth", pin }));
      };
      ws.onmessage = (event) => {
        let data;
        try { data = JSON.parse(event.data); } catch { return; }
        if (data.type === "auth_ok") {
          if (consoleBlocked || checkDevToolsOpen()) return;
          allowAllMode = data.permission_mode === "allow_all";
          commandTimeoutSeconds = Number(data.command_timeout_seconds || 30);
          ownerPresent = Boolean(data.owner_present);
          hideAuthPanel();
          showCommandForm();
          lineInput.placeholder = "";
          startRemainingTimer(data.expires_in == null ? null : Number(data.expires_in || 0));
          setCommandNotice("");
          syncGuestControls();
          if (ownerPresent) lineInput.focus();
        } else if (data.type === "auth_failed") {
          setStatus(tx("pinFailed", { remaining: data.remaining }));
        } else if (data.type === "screen") {
          screenEl.textContent = data.text || " ";
          screenWrap.scrollTop = screenWrap.scrollHeight;
        } else if (data.type === "owner_presence") {
          ownerPresent = Boolean(data.active);
          updateRemainingStatus();
          if (ownerPresent) lineInput.focus();
        } else if (data.type === "owner_absent") {
          ownerPresent = false;
          updateRemainingStatus();
        } else if (data.type === "command_waiting_approval") {
          setCommandNotice(tx("waiting"));
        } else if (data.type === "command_approved") {
          setCommandNotice(tx("approved"), 1600);
        } else if (data.type === "command_running") {
          setCommandNotice(tx("running"), 1800);
        } else if (data.type === "command_failed") {
          setCommandNotice(tx("failed"), 2600);
        } else if (data.type === "command_rejected") {
          setCommandNotice(tx("rejected"), 2200);
        } else if (data.type === "command_expired") {
          setCommandNotice(tx("expired"), 2200);
        } else if (data.type === "session_closed") {
          sessionClosed = true;
          resetToPin(tx("closed"));
          if (ws) ws.close();
        } else if (data.type === "error") {
          setStatus(data.message || tx("error"));
        }
      };
      ws.onclose = () => {
        ownerPresent = false;
        syncGuestControls();
        if (!sessionClosed) resetToPin(tx("disconnected"));
      };
    }
    function sendControl(action) {
      if (!socketReady() || !ownerPresent) {
        updateRemainingStatus();
        return false;
      }
      ws.send(JSON.stringify({ type: "control", action }));
      return true;
    }
    authForm.addEventListener("submit", (event) => {
      event.preventDefault();
      if (consoleBlocked || checkDevToolsOpen()) return;
      const pin = pinInput.value.trim();
      if (pin) connect(pin);
    });
    btnCtrlC.addEventListener("click", () => {
      sendControl("ctrl_c");
    });
    btnClear.addEventListener("click", () => {
      screenEl.textContent = " ";
      sendControl("clear");
    });
    btnStop.addEventListener("click", () => {
      if (!ws || ws.readyState !== WebSocket.OPEN) return;
      ws.send(JSON.stringify({ type: "close_session" }));
      resetToPin(tx("closed"));
    });
    lineInput.addEventListener("input", () => {
      if (!socketReady() || !ownerPresent) return;
      ws.send(JSON.stringify({ type: "typing", active: true, text: lineInput.value }));
      clearTimeout(typingTimer);
      typingTimer = setTimeout(() => {
        if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "typing", active: false, text: "" }));
      }, 1400);
    });
    cmdForm.addEventListener("submit", (event) => {
      event.preventDefault();
      const line = lineInput.value.trim();
      if (!line || !socketReady()) return;
      if (!ownerPresent) {
        updateRemainingStatus();
        return;
      }
      ws.send(JSON.stringify({ type: "typing", active: false, text: "" }));
      ws.send(JSON.stringify({ type: "input", data: line }));
      lineInput.value = "";
    });
    window.addEventListener("resize", checkDevToolsOpen);
    window.setInterval(checkDevToolsOpen, 1200);
    showConsoleWarning();
    localizeStaticText();
    syncGuestControls();
  </script>
</body>
</html>"""


manager = SupportTerminalManager()
