from __future__ import annotations

import asyncio
import html
import json
import os
import secrets
import socket
import time
from dataclasses import dataclass, field
from typing import Any

from aiohttp import web, WSMsgType

from ..config import WEB_DIR
from ..terminal_commands import translate_meta_command
from .support_discord import send_support_webhook, support_metadata
from .terminal_pty import PTY_SESSION
from .support_tunnel import TunnelHandle, cloudflared_status, start_quick_tunnel


SUPPORT_TTL_SECONDS = int(os.environ.get("CARROT_SUPPORT_TTL_SECONDS", "1800"))
COMMAND_TIMEOUT_SECONDS = int(os.environ.get("CARROT_SUPPORT_COMMAND_TIMEOUT_SECONDS", "30"))
ALLOWED_TTL_SECONDS = {900, 1800, 3600}
ALLOWED_PERMISSION_MODES = {"approve_each", "allow_all"}
ALLOWED_COMMAND_TIMEOUT_SECONDS = {15, 30, 60, 120}
PIN_FAILURE_LIMIT = 5
PUBLIC_URL_START_DELAY = float(os.environ.get("CARROT_SUPPORT_LINK_START_DELAY_SECONDS", "3.0"))
SUPPORT_GUEST_DIR = os.path.join(WEB_DIR, "support_terminal")
SUPPORT_GUEST_HTML_PATH = os.path.join(SUPPORT_GUEST_DIR, "guest.html")
SUPPORT_GUEST_ASSETS = {
  "design-tokens.css": os.path.join(WEB_DIR, "css", "generated", "design-tokens.css"),
  "design-system.css": os.path.join(WEB_DIR, "css", "generated", "design-system.css"),
  "tokens.css": os.path.join(WEB_DIR, "css", "tokens.css"),
  "layout_tokens.css": os.path.join(WEB_DIR, "css", "layout_tokens.css"),
  "base.css": os.path.join(WEB_DIR, "css", "base.css"),
  "layout.css": os.path.join(WEB_DIR, "css", "layout.css"),
  "components.css": os.path.join(WEB_DIR, "css", "components.css"),
  "terminal.css": os.path.join(WEB_DIR, "css", "generated", "terminal.css"),
  "terminal_typing_indicator.js": os.path.join(WEB_DIR, "js", "generated", "terminal-shared.js"),
  "guest.css": os.path.join(WEB_DIR, "css", "generated", "terminal-guest.css"),
  "guest.js": os.path.join(WEB_DIR, "js", "generated", "terminal-guest.js"),
  "xterm.css": os.path.join(WEB_DIR, "css", "vendor", "xterm.css"),
  "xterm.js": os.path.join(WEB_DIR, "js", "vendor", "xterm.js"),
  "xterm-addon-shim.js": os.path.join(WEB_DIR, "js", "vendor", "xterm-addon-shim.js"),
  "xterm-addon-webgl.js": os.path.join(WEB_DIR, "js", "vendor", "xterm-addon-webgl.js"),
  "xterm-addon-canvas.js": os.path.join(WEB_DIR, "js", "vendor", "xterm-addon-canvas.js"),
}


def _now() -> float:
  return time.time()


def _new_pin() -> str:
  return f"{secrets.randbelow(1_000_000):06d}"


def _new_session_id() -> str:
  return secrets.token_urlsafe(16)


def _translate_support_terminal_line(line: str) -> str:
  translated = translate_meta_command(line)
  if translated:
    return translated
  return str(line or "")


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


def _sanitize_typing_text(value: Any) -> str:
  # Collaboration indicators are text-only. Terminal control bytes never belong
  # in a human-facing input preview.
  return "".join(ch for ch in str(value or "") if ch.isprintable())[:160]


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
  expiry_task: asyncio.Task | None = None
  owner_sockets: set[web.WebSocketResponse] = field(default_factory=set)
  guest_sockets: set[web.WebSocketResponse] = field(default_factory=set)
  controller_socket: web.WebSocketResponse | None = None

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
      "controller_present": session.controller_socket is not None and not session.controller_socket.closed,
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
      await self._set_status(session, "Preparing shared terminal session")
      await PTY_SESSION.ensure()
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
        "terminalSession": PTY_SESSION.session,
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
    for task in (session.expiry_task,):
      if task:
        task.cancel()
    for ws in list(session.guest_sockets):
      await PTY_SESSION.detach(ws)
    for ws in list(session.owner_sockets | session.guest_sockets):
      try:
        await ws.close()
      except Exception:
        pass
    session.owner_sockets.clear()
    session.guest_sockets.clear()
    session.controller_socket = None
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
    app.router.add_get("/support-terminal-assets/{asset_name}", self.handle_guest_asset)
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

  async def _sync_guest_control_roles(self, session: SupportSession) -> None:
    if session.permission_mode != "allow_all":
      session.controller_socket = None
    elif session.controller_socket is None or session.controller_socket.closed or session.controller_socket not in session.guest_sockets:
      session.controller_socket = next((guest for guest in session.guest_sockets if not guest.closed), None)
    for guest in list(session.guest_sockets):
      if guest.closed:
        continue
      try:
        await guest.send_str(json.dumps({
          "type": "control_role",
          "granted": session.permission_mode != "allow_all" or guest is session.controller_socket,
        }))
      except Exception:
        pass

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
          elif data.get("type") == "typing":
            text = _sanitize_typing_text(data.get("text"))
            await self.broadcast_guests({"type": "host_typing", "active": bool(data.get("active")), "text": text})
    finally:
      session.owner_sockets.discard(ws)
      await self.broadcast_guests({"type": "owner_presence", "active": bool(session.owner_sockets)})
    return ws

  async def handle_guest_page(self, request: web.Request) -> web.Response:
    session_id = html.escape(request.match_info.get("session_id", ""), quote=True)
    with open(SUPPORT_GUEST_HTML_PATH, "r", encoding="utf-8") as f:
      page_html = f.read().replace("__SESSION_ID__", session_id)
    csp = (
      "default-src 'none'; "
      "script-src 'self'; "
      "style-src 'self' 'unsafe-inline'; "
      f"connect-src {_csp_connect_sources(request.host)}; "
      "img-src 'self' data:; font-src 'self'; "
      "base-uri 'none'; form-action 'self'; frame-ancestors 'none'"
    )
    headers = {
      "Cache-Control": "no-store",
      "Content-Security-Policy": csp,
      "Permissions-Policy": "camera=(), microphone=(), geolocation=(), payment=(), usb=(), serial=(), hid=(), bluetooth=()",
      "Referrer-Policy": "no-referrer",
      "X-Content-Type-Options": "nosniff",
      "X-Frame-Options": "DENY",
    }
    return web.Response(text=page_html, content_type="text/html", headers=headers)

  async def handle_guest_asset(self, request: web.Request) -> web.StreamResponse:
    asset_name = str(request.match_info.get("asset_name") or "")
    path = SUPPORT_GUEST_ASSETS.get(asset_name)
    if not path or not os.path.isfile(path):
      raise web.HTTPNotFound()
    return web.FileResponse(path, headers={
      "Cache-Control": "no-store",
      "X-Content-Type-Options": "nosniff",
      "Referrer-Policy": "no-referrer",
    })

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
          if session.permission_mode == "allow_all" and (session.controller_socket is None or session.controller_socket.closed):
            session.controller_socket = ws
          expires_in = None if session.expires_at <= 0 else max(0, int(session.expires_at - _now()))
          await ws.send_str(json.dumps({
            "type": "auth_ok",
            "permission_mode": session.permission_mode,
            "command_timeout_seconds": session.command_timeout_seconds,
            "expires_in": expires_in,
            "owner_present": bool(session.owner_sockets),
            "control_granted": session.permission_mode != "allow_all" or ws is session.controller_socket,
          }))
          try:
            await PTY_SESSION.attach(ws, primary_eligible=False)
          except Exception as exc:
            session.guest_sockets.discard(ws)
            await ws.send_str(json.dumps({"type": "error", "message": str(exc)}))
            await ws.close()
            return ws
          await self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_sockets)})
          await self._sync_guest_control_roles(session)
          continue

        if typ == "typing":
          text = _sanitize_typing_text(data.get("text"))
          await self.broadcast_owner({"type": "guest_typing", "active": bool(data.get("active")), "text": text})
        elif typ == "close_session":
          await self.stop("guest")
          break
        elif typ == "disconnect":
          break
        elif typ == "control":
          if session.permission_mode == "allow_all" and ws is not session.controller_socket:
            await ws.send_str(json.dumps({"type": "input_denied", "reason": "viewer only"}))
            continue
          action = str(data.get("action") or "").strip()
          if action in {"ctrl_c", "clear"}:
            label = "Ctrl+C" if action == "ctrl_c" else "clear"
            await self.queue_guest_command(session, label, ws, control_action=action)
        elif typ == "input":
          if session.permission_mode == "allow_all" and ws is not session.controller_socket:
            await ws.send_str(json.dumps({"type": "input_denied", "reason": "viewer only"}))
            continue
          line = str(data.get("data") or "").strip()
          if line:
            await self.queue_guest_command(session, line, ws)
        elif typ == "raw":
          if not session.owner_sockets:
            await ws.send_str(json.dumps({"type": "owner_absent"}))
            continue
          if session.permission_mode != "allow_all":
            await ws.send_str(json.dumps({"type": "input_denied", "reason": "approval required"}))
            continue
          if ws is not session.controller_socket:
            await ws.send_str(json.dumps({"type": "input_denied", "reason": "viewer only"}))
            continue
          text = str(data.get("data") or "")[:4096]
          if text:
            try:
              await PTY_SESSION.write_text(text)
            except Exception as exc:
              await ws.send_str(json.dumps({"type": "error", "message": str(exc)}))
    finally:
      await PTY_SESSION.detach(ws)
      session.guest_sockets.discard(ws)
      if session.controller_socket is ws:
        session.controller_socket = None
      await self._sync_guest_control_roles(session)
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
        await PTY_SESSION.write(b"\x03")
      elif command.control_action == "clear":
        await PTY_SESSION.clear_history()
        await PTY_SESSION.write(b"clear\r")
      else:
        line = _translate_support_terminal_line(command.line)
        await PTY_SESSION.write_text(line + "\r")
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

manager = SupportTerminalManager()
