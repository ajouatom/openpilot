from __future__ import annotations

import asyncio
import base64
import json
import os
import signal
import struct
import subprocess
from typing import Optional

from aiohttp import web

from . import tmux

try:
  import fcntl
  import pty
  import termios
except Exception:
  fcntl = None
  pty = None
  termios = None


PTY_HISTORY_LIMIT = 512 * 1024
PTY_FIXED_COLS = 100
PTY_FIXED_ROWS = 30


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
  """One persistent login PTY shared by the local and support terminals."""

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
    async with self.lock:
      self._terminate_proc_locked()
      self._close_fds_locked()
      self.history.clear()

  async def ensure(self, rows: int = PTY_FIXED_ROWS, cols: int = PTY_FIXED_COLS) -> bool:
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

  async def attach(
    self,
    ws: web.WebSocketResponse,
    rows: int = PTY_FIXED_ROWS,
    cols: int = PTY_FIXED_COLS,
    *,
    primary_eligible: bool = True,
  ) -> bool:
    created = await self.ensure(rows, cols)
    async with self.lock:
      self.clients.add(ws)
      if primary_eligible and (self.primary_client is None or self.primary_client.closed or self.primary_client not in self.clients):
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
    # The local terminal owns shared PTY geometry. Support viewers never call
    # this method, which prevents differently-sized browsers fighting over rows.
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
    await self._broadcast({
      "type": "pty_resize",
      "session": self.session,
      "rows": self.rows,
      "cols": self.cols,
    })

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
          if self.primary_client is client:
            self.primary_client = None

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
        self.primary_client = None
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
