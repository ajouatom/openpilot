import asyncio
import base64
import json
import os
import re
import signal
import struct
import subprocess
from typing import Optional

from aiohttp import web, WSMsgType

from ..config import TMUX_WEB_SESSION
from ..services import tmux
from ..terminal_commands import translate_meta_command

try:
  import fcntl
  import pty
  import termios
except Exception:
  fcntl = None
  pty = None
  termios = None


TMUX_ATTACH_RE = re.compile(r"^\s*tmux\s+(?:a|attach|attach-session)(?:\s*)$", re.IGNORECASE)
TMUX_ATTACH_TARGET_RE = re.compile(r"^\s*tmux\s+(?:a|attach|attach-session)\s+-t\s+\S+\s*$", re.IGNORECASE)
PTY_HISTORY_LIMIT = 512 * 1024
PTY_FIXED_COLS = 100
PTY_FIXED_ROWS = 30


def _translate_terminal_line(line: str, *, nested_tmux: bool = False) -> str:
  translated = translate_meta_command(line)
  if translated:
    return translated
  if not nested_tmux:
    return str(line or "")
  text = str(line or "")
  if TMUX_ATTACH_RE.match(text):
    return "TMUX= tmux a -t comma"
  if TMUX_ATTACH_TARGET_RE.match(text):
    return f"TMUX= {text.strip()}"
  return text


def _set_pty_size(fd: int, rows: int, cols: int) -> None:
  if fcntl is None or termios is None:
    return
  rows = max(8, min(int(rows or 24), 200))
  cols = max(20, min(int(cols or 80), 400))
  fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))


def _write_all(fd: int, data: bytes) -> None:
  view = memoryview(data)
  while view:
    written = os.write(fd, view)
    if written <= 0:
      raise OSError("pty write failed")
    view = view[written:]


class PersistentPtySession:
  def __init__(self) -> None:
    self.session = "login-shell"
    self.master_fd = -1
    self.proc: Optional[subprocess.Popen] = None
    self.reader_task: asyncio.Task | None = None
    self.clients: set[web.WebSocketResponse] = set()
    self.primary_client: web.WebSocketResponse | None = None
    self.history = bytearray()
    self.rows = PTY_FIXED_ROWS
    self.cols = PTY_FIXED_COLS
    self.lock = asyncio.Lock()

  def _alive_locked(self) -> bool:
    return self.master_fd >= 0 and self.proc is not None and self.proc.poll() is None

  def _snapshot_locked(self) -> dict:
    alive = self._alive_locked()
    return {
      "alive": alive,
      "pid": self.proc.pid if alive and self.proc else None,
      "clients": len(self.clients),
      "primary": self.primary_client is not None and self.primary_client in self.clients and not self.primary_client.closed,
      "rows": self.rows,
      "cols": self.cols,
      "history_bytes": len(self.history),
      "session": self.session,
    }

  async def snapshot(self) -> dict:
    async with self.lock:
      return self._snapshot_locked()

  def _terminate_proc_locked(self) -> None:
    proc = self.proc
    self.proc = None
    if self.reader_task:
      self.reader_task.cancel()
      self.reader_task = None
    if proc is not None and proc.poll() is None:
      try:
        os.killpg(proc.pid, signal.SIGHUP)
      except Exception:
        try:
          proc.terminate()
        except Exception:
          pass

  async def terminate(self) -> None:
    # Kill the current login shell (and its process group) and drop the
    # scrollback so the next attach spawns a brand-new session — this is what
    # the Reconnect button uses to mean "end this session and start fresh".
    async with self.lock:
      self._terminate_proc_locked()
      self._close_fds_locked()
      self.history.clear()

  async def ensure(self, rows: int, cols: int) -> bool:
    async with self.lock:
      if self._alive_locked():
        return False

      self.rows = PTY_FIXED_ROWS
      self.cols = PTY_FIXED_COLS

      self._close_fds_locked()
      self.history.clear()
      if pty is None:
        raise RuntimeError("PTY terminal is only available on POSIX devices")

      master_fd, slave_fd = pty.openpty()
      try:
        _set_pty_size(master_fd, self.rows, self.cols)
        env = os.environ.copy()
        env.pop("TMUX", None)
        env.setdefault("TERM", "xterm-256color")
        env.setdefault("COLORTERM", "truecolor")
        shell = os.environ.get("SHELL") or "/bin/bash"
        proc = subprocess.Popen(
          [shell, "-lc", tmux.start_command()],
          stdin=slave_fd,
          stdout=slave_fd,
          stderr=slave_fd,
          close_fds=True,
          env=env,
          start_new_session=True,
        )
      except Exception:
        try:
          os.close(master_fd)
        except Exception:
          pass
        raise
      finally:
        try:
          os.close(slave_fd)
        except Exception:
          pass

      self.master_fd = master_fd
      self.proc = proc
      self.reader_task = asyncio.create_task(self._read_loop())
      return True

  async def attach(self, ws: web.WebSocketResponse, rows: int, cols: int) -> bool:
    created = await self.ensure(rows, cols)
    async with self.lock:
      self.clients.add(ws)
      if self.primary_client is None or self.primary_client.closed or self.primary_client not in self.clients:
        self.primary_client = ws
      history = bytes(self.history)
      snapshot = self._snapshot_locked()
      session = snapshot["session"]
      is_primary = ws is self.primary_client
    await ws.send_str(json.dumps({
      "type": "meta",
      "mode": "pty",
      "session": session,
      "created": created,
      "user": "comma",
      "pid": snapshot["pid"],
      "clients": snapshot["clients"],
      "primary": is_primary,
      "rows": snapshot["rows"],
      "cols": snapshot["cols"],
    }))
    if history:
      await ws.send_str(json.dumps({
        "type": "pty_output",
        "session": session,
        "b64": base64.b64encode(history).decode("ascii"),
        "replay": True,
      }))
    return created

  async def detach(self, ws: web.WebSocketResponse) -> None:
    async with self.lock:
      self.clients.discard(ws)
      if self.primary_client is ws:
        self.primary_client = next((client for client in self.clients if not client.closed), None)

  async def write_text(self, text: str) -> None:
    data = str(text or "").encode("utf-8", errors="replace")
    if data:
      await self.write(data)

  async def write(self, data: bytes) -> None:
    async with self.lock:
      if not self._alive_locked():
        raise RuntimeError("terminal session is not running")
      fd = self.master_fd
    await asyncio.to_thread(_write_all, fd, data)

  async def clear_history(self) -> None:
    async with self.lock:
      self.history.clear()

  async def resize(self, ws: web.WebSocketResponse, rows: int, cols: int) -> None:
    # Columns stay locked at PTY_FIXED_COLS so line-wrapping is identical for
    # every viewer (the "same view" guarantee). Only the row count follows the
    # client so the grid fills the screen height and full-screen apps (btop/vim)
    # draw to the full height instead of leaving an empty band below.
    new_rows = max(8, min(int(rows or PTY_FIXED_ROWS), 200))
    async with self.lock:
      if not self._alive_locked() or new_rows == self.rows:
        return
      self.rows = new_rows
      _set_pty_size(self.master_fd, self.rows, self.cols)
      proc = self.proc
    if proc is not None and proc.poll() is None:
      try:
        os.killpg(proc.pid, signal.SIGWINCH)
      except Exception:
        pass

  def _append_history(self, chunk: bytes) -> None:
    self.history.extend(chunk)
    if len(self.history) > PTY_HISTORY_LIMIT:
      del self.history[:len(self.history) - PTY_HISTORY_LIMIT]

  async def _broadcast(self, payload: dict) -> None:
    async with self.lock:
      clients = list(self.clients)
    stale = []
    text = json.dumps(payload)
    for client in clients:
      if client.closed:
        stale.append(client)
        continue
      try:
        await client.send_str(text)
      except Exception:
        stale.append(client)
    if stale:
      async with self.lock:
        for client in stale:
          self.clients.discard(client)

  async def _read_loop(self) -> None:
    try:
      while True:
        async with self.lock:
          if not self._alive_locked():
            break
          fd = self.master_fd
        try:
          chunk = await asyncio.to_thread(os.read, fd, 4096)
        except asyncio.CancelledError:
          raise
        except OSError:
          break
        if not chunk:
          break
        async with self.lock:
          self._append_history(chunk)
          session = self.session
        await self._broadcast({
          "type": "pty_output",
          "session": session,
          "b64": base64.b64encode(chunk).decode("ascii"),
        })
    finally:
      async with self.lock:
        exit_code = self.proc.poll() if self.proc else None
        session = self.session
        clients = list(self.clients)
        self.clients.clear()
        self._close_fds_locked()
        self.proc = None
        self.reader_task = None
      payload = json.dumps({
        "type": "pty_exit",
        "session": session,
        "exit_code": exit_code,
      })
      for client in clients:
        if client.closed:
          continue
        try:
          await client.send_str(payload)
          await client.close()
        except Exception:
          pass

  def _close_fds_locked(self) -> None:
    if self.master_fd >= 0:
      try:
        os.close(self.master_fd)
      except Exception:
        pass
      self.master_fd = -1


PTY_SESSION = PersistentPtySession()


async def ws_terminal(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20, compress=False)
  await ws.prepare(request)

  session = (request.query.get("session") or TMUX_WEB_SESSION).strip() or TMUX_WEB_SESSION
  last_screen = None

  try:
    created = await asyncio.to_thread(tmux.ensure_session, session)
    await ws.send_str(json.dumps({
      "type": "meta",
      "session": session,
      "created": created,
      "user": "comma",
    }))
  except Exception as e:
    await ws.send_str(json.dumps({
      "type": "error",
      "error": str(e),
      "session": session,
    }))
    await ws.close()
    return ws

  async def push_screen(force: bool = False, delay: float = 0.0) -> None:
    nonlocal last_screen
    if delay > 0:
      await asyncio.sleep(delay)
    screen = await asyncio.to_thread(tmux.capture, session)
    if force or screen != last_screen:
      last_screen = screen
      await ws.send_str(json.dumps({
        "type": "screen",
        "session": session,
        "text": screen,
      }))

  async def pump_screen():
    while not ws.closed:
      try:
        await push_screen(force=False)
      except asyncio.CancelledError:
        raise
      except Exception as e:
        await ws.send_str(json.dumps({
          "type": "error",
          "error": str(e),
          "session": session,
        }))
        break
      await asyncio.sleep(0.18)

  pump_task = asyncio.create_task(pump_screen())

  try:
    await push_screen(force=True, delay=0.02)
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          data = json.loads(msg.data)
        except Exception:
          continue

        typ = data.get("type")
        try:
          if typ == "input":
            line = str(data.get("data") or "")
            await asyncio.to_thread(tmux.send_line, session, _translate_terminal_line(line, nested_tmux=True))
            await push_screen(force=True, delay=0.03)
          elif typ == "control":
            action = (data.get("action") or "").strip()
            if action == "ctrl_c":
              await asyncio.to_thread(tmux.ctrl_c, session)
              await push_screen(force=True, delay=0.03)
            elif action == "clear":
              await asyncio.to_thread(tmux.clear, session)
              await push_screen(force=True, delay=0.05)
            elif action == "refresh":
              await push_screen(force=True)
            elif action == "new_session":
              created = await asyncio.to_thread(tmux.ensure_session, session)
              await ws.send_str(json.dumps({
                "type": "meta",
                "session": session,
                "created": created,
                "user": "comma",
              }))
              await push_screen(force=True, delay=0.08)
        except Exception as e:
          await ws.send_str(json.dumps({
            "type": "error",
            "error": str(e),
            "session": session,
          }))
      elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSING):
        break
  finally:
    pump_task.cancel()
    try:
      await pump_task
    except Exception:
      pass
    try:
      await ws.close()
    except Exception:
      pass
  return ws


async def ws_terminal_pty(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20, compress=False)
  await ws.prepare(request)

  rows = int(request.query.get("rows") or 28)
  cols = int(request.query.get("cols") or 100)
  reset = request.query.get("reset") in ("1", "true", "yes")

  try:
    if reset:
      await PTY_SESSION.terminate()
    await PTY_SESSION.attach(ws, rows, cols)
  except Exception as e:
    await ws.send_str(json.dumps({
      "type": "error",
      "error": str(e),
      "session": PTY_SESSION.session,
    }))
    await ws.close()
    return ws

  try:
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          data = json.loads(msg.data)
        except Exception:
          continue
        typ = data.get("type")
        try:
          if typ == "input":
            line = _translate_terminal_line(str(data.get("data") or ""))
            await PTY_SESSION.write_text(line + "\r")
          elif typ == "raw":
            text = str(data.get("data") or "")
            if text:
              await PTY_SESSION.write_text(text)
          elif typ == "resize":
            await PTY_SESSION.resize(ws, int(data.get("rows") or rows), int(data.get("cols") or cols))
          elif typ == "control":
            action = (data.get("action") or "").strip()
            if action == "ctrl_c":
              await PTY_SESSION.write(b"\x03")
            elif action == "clear":
              await PTY_SESSION.clear_history()
              await PTY_SESSION.write(b"clear\r")
            elif action == "refresh":
              await PTY_SESSION.write(b"\x0c")
            elif action == "detach":
              # AGNOS tmux prefix is backtick, not Ctrl-B, so detach = ` then d.
              await PTY_SESSION.write(b"\x60d")
        except Exception as e:
          await ws.send_str(json.dumps({
            "type": "error",
            "error": str(e),
            "session": PTY_SESSION.session,
          }))
      elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSING):
        break
  finally:
    await PTY_SESSION.detach(ws)
    try:
      await ws.close()
    except Exception:
      pass
  return ws


async def handle_terminal_pty_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **await PTY_SESSION.snapshot()})


async def handle_download_tmux(request: web.Request) -> web.Response:
  path = "/data/media/tmux.log"
  if not os.path.exists(path):
    return web.json_response({"ok": False, "error": "file not found"}, status=404)

  return web.FileResponse(
    path,
    headers={
      "Content-Disposition": "attachment; filename=tmux.log"
    }
  )


def register(app: web.Application) -> None:
  app.router.add_get("/api/terminal_pty/status", handle_terminal_pty_status)
  app.router.add_get("/ws/terminal", ws_terminal)
  app.router.add_get("/ws/terminal_pty", ws_terminal_pty)
  app.router.add_get("/download/tmux.log", handle_download_tmux)
