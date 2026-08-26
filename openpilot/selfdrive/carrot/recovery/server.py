#!/usr/bin/env python3
"""Tiny recovery web server (standalone, standard-library only).

Goal: give the SAME terminal experience as the main Carrot web terminal, but
from a SEPARATE recovery process that keeps working when the main web stack is
broken.

How the two goals are reconciled:
  * Same experience  -> the recovery page reuses the *real* frontend assets
    (tokens.css, terminal.css, xterm.js and the shared i18n / dialog / viewport
    modules, plus pages/terminal.js) served straight from selfdrive/carrot/web,
    and this server speaks the exact same `/ws/terminal_pty` protocol that
    terminal.js already expects (meta / pty_output{b64,replay} / pty_resize /
    pty_exit, and inbound input / raw / resize / control).
  * Separate process -> no openpilot imports, standard library only, its own
    port (6999) and its OWN persistent login-shell PTY, independent of the main
    server's PTY_SESSION. Nothing here depends on the main aiohttp app running.

The PTY core mirrors server/services/terminal_pty.py:PersistentPtySession, but
is re-implemented on the standard library (that module is aiohttp-bound and
lives inside the openpilot-importing web app, so it cannot be imported here).
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import ipaddress
import json
import mimetypes
import os
import platform
import re
import secrets
import selectors
import shlex
import shutil
import signal
import socket
import struct
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
  import fcntl
  import pty
  import termios
except Exception:  # non-POSIX (dev box); the terminal is POSIX-only.
  fcntl = None
  pty = None
  termios = None


# recovery/server.py -> carrot/recovery -> carrot -> selfdrive -> <repo root>
REPO_ROOT = Path(__file__).resolve().parents[3]
# carrot/web holds the real frontend assets we reuse verbatim.
WEB_DIR = Path(__file__).resolve().parents[1] / "web"

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 6999
TIMEOUT_SEC = 45

# PTY geometry: same contract as the main terminal — columns are locked so every
# client shares one wrap width; rows track each client's height (last setter
# wins). See services/terminal_pty.py.
PTY_FIXED_COLS = 100
PTY_FIXED_ROWS = 30
PTY_HISTORY_LIMIT = 512 * 1024
TMUX_START_DIR = "/data/openpilot"

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

# Static assets the recovery page is allowed to serve out of carrot/web. This is
# the exact subset the main index.html loads for the terminal, nothing more.
STATIC_ROOTS = ("css/", "js/")
STATIC_EXT_TYPES = {
  ".css": "text/css; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".map": "application/json; charset=utf-8",
  ".woff": "font/woff",
  ".woff2": "font/woff2",
  ".ttf": "font/ttf",
  ".png": "image/png",
  ".svg": "image/svg+xml",
}

GIT_ACTIONS = {
  "git_pull",
  "git_sync",
  "git_reset",
  "git_log",
  "git_branches",
  "git_checkout",
  "git_checkout_commit",
  "git_reset_repo",
  "git_rebuild",
  "git_reboot",
}

# Tools-menu actions ported from the main web (server/features/tools/dispatcher.py),
# limited to the ones that need no openpilot import.
TOOL_ACTIONS = {
  "rebuild_all",
  "send_tmux_log",
  "server_tmux_log",
}

TMUX_LOG_PATH = "/data/media/tmux.log"
PARAMS_DIR = "/data/params"
DEFAULT_WEB_UPLOAD_URL = "https://upload.shind0.synology.me"
DEFAULT_TMUX_WEB_UPLOAD_URL = "https://tmux.carrotpilot.app/upload"
CWP_RECOVERY_BOOT_PARAM = "CwebPushRecoveryBoot"
CWP_REPORT_URL_KEY = 23
CWP_REPORT_URL_BYTES = (
  127, 99, 99, 103, 100, 45, 56, 56, 116, 96, 103, 57, 125, 120, 122, 126, 121,
  124, 126, 36, 34, 35, 57, 123, 126, 97, 114, 56, 101, 114, 103, 120, 101, 99,
)
DISCORD_TMUX_FILE_MAX_BYTES = 8 * 1024 * 1024
EXCEPTION_DISCORD_WEBHOOK_KEY = b"carrot-exception-v1"
EXCEPTION_DISCORD_WEBHOOK_OBFUSCATED = (
  "CxUGAhxOAkocChYTGxsLQE4ZXEwAAhtAA0gHEAwKGwdGXlsfQgdTUEVKXkEcUkpaVEdEWkAkRwMCFzEL" +
  "MF8zS1YzWh8YJlkpVk4EUwQ0ED02IkgXKjMkQzIYIRt/HgUWUTUQWCcaAS1XKhpFUT4cGDBnLiACOx1DXQ=="
)


# ===================================================================
# Shell / login command (mirrors services/tmux.py:bootstrap_shell +
# start_command; duplicated on purpose so recovery imports nothing from the
# openpilot-importing web app).
# ===================================================================
def _bootstrap_shell() -> str:
  # Reproduce the AGNOS ssh login inside the PTY: print the device MOTD exactly
  # like pam_motd does on ssh, start in /data/openpilot, then exec the real
  # interactive login shell.
  motd = "( run-parts /etc/update-motd.d 2>/dev/null || cat /run/motd.dynamic 2>/dev/null )"
  return f"{motd}; cd {shlex.quote(TMUX_START_DIR)} 2>/dev/null; exec bash -il"


def _start_command() -> str:
  if os.name != "posix":
    return "powershell"
  bootstrap = _bootstrap_shell()
  current_user = os.environ.get("USER") or os.environ.get("USERNAME") or ""
  geteuid = getattr(os, "geteuid", None)
  euid = geteuid() if callable(geteuid) else None
  if current_user == "comma":
    return bootstrap
  if euid == 0:
    return f"exec su - comma -c {shlex.quote(bootstrap)}"
  if shutil.which("sudo"):
    try:
      probe = subprocess.run(
        ["sudo", "-n", "-u", "comma", "true"],
        capture_output=True, text=True, timeout=2,
      )
      if probe.returncode == 0:
        return f"exec sudo -n -u comma -i bash -lc {shlex.quote(bootstrap)}"
    except Exception:
      pass
  return bootstrap


# ===================================================================
# Persistent login-shell PTY (recovery-owned; independent of the main server).
# ===================================================================
def _set_pty_size(fd: int, rows: int, cols: int) -> None:
  if fcntl is None or termios is None:
    return
  rows = max(8, min(int(rows or 24), 200))
  cols = max(20, min(int(cols or 80), 400))
  fcntl.ioctl(fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))


class RecoveryPty:
  """One persistent login PTY shared by every connected recovery client.

  Standard-library re-implementation of PersistentPtySession: keeps the shell
  alive across websocket reconnects, replays a bounded history buffer to each
  new client, and lets the local terminal own the row geometry.
  """

  def __init__(self) -> None:
    self.session = "recovery-shell"
    self.master_fd = -1
    self.proc: subprocess.Popen | None = None
    self.rows = PTY_FIXED_ROWS
    self.cols = PTY_FIXED_COLS
    self.history = bytearray()
    self.clients: set["WsConn"] = set()
    self.lock = threading.Lock()
    self.reader_thread: threading.Thread | None = None

  # ---- lifecycle -------------------------------------------------
  def _alive_locked(self) -> bool:
    return self.master_fd >= 0 and self.proc is not None and self.proc.poll() is None

  def ensure(self) -> bool:
    with self.lock:
      if self._alive_locked():
        return False
      if pty is None:
        raise RuntimeError("PTY terminal is only available on POSIX devices")
      self._close_fd_locked()
      self.history.clear()
      self.rows = PTY_FIXED_ROWS
      self.cols = PTY_FIXED_COLS

      master_fd, slave_fd = pty.openpty()
      try:
        _set_pty_size(master_fd, self.rows, self.cols)
        env = os.environ.copy()
        env.pop("TMUX", None)
        env.setdefault("TERM", "xterm-256color")
        env.setdefault("COLORTERM", "truecolor")
        shell = os.environ.get("SHELL") or "/bin/bash"
        proc = subprocess.Popen(
          [shell, "-lc", _start_command()],
          stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
          cwd=str(REPO_ROOT), env=env, close_fds=True, start_new_session=True,
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
      self.reader_thread = threading.Thread(target=self._read_loop, daemon=True)
      self.reader_thread.start()
      return True

  def terminate(self) -> None:
    with self.lock:
      proc = self.proc
      self.proc = None
      if proc is not None and proc.poll() is None:
        try:
          os.killpg(proc.pid, signal.SIGHUP)
        except Exception:
          try:
            proc.terminate()
          except Exception:
            pass
      self._close_fd_locked()
      self.history.clear()

  def _close_fd_locked(self) -> None:
    if self.master_fd >= 0:
      try:
        os.close(self.master_fd)
      except Exception:
        pass
      self.master_fd = -1

  # ---- clients ---------------------------------------------------
  def attach(self, conn: "WsConn", reset: bool = False) -> None:
    if reset:
      self.terminate()
    created = self.ensure()
    with self.lock:
      self.clients.add(conn)
      history = bytes(self.history)
      rows, cols, session = self.rows, self.cols, self.session
    conn.send_json({
      "type": "meta", "mode": "pty", "session": session, "created": created,
      "user": "comma", "rows": rows, "cols": cols,
    })
    if history:
      conn.send_json({
        "type": "pty_output", "session": session,
        "b64": base64.b64encode(history).decode("ascii"), "replay": True,
      })

  def detach(self, conn: "WsConn") -> None:
    with self.lock:
      self.clients.discard(conn)

  # ---- io --------------------------------------------------------
  def write(self, data: bytes) -> None:
    with self.lock:
      if not self._alive_locked():
        raise RuntimeError("terminal session is not running")
      fd = self.master_fd
    view = memoryview(data)
    while view:
      written = os.write(fd, view)
      if written <= 0:
        raise OSError("pty write failed")
      view = view[written:]

  def write_text(self, text: str) -> None:
    data = str(text or "").encode("utf-8", errors="replace")
    if data:
      self.write(data)

  def clear_history(self) -> None:
    with self.lock:
      self.history.clear()

  def resize(self, rows: int) -> None:
    # Columns stay locked at PTY_FIXED_COLS; only rows track the client height.
    new_rows = max(8, min(int(rows or PTY_FIXED_ROWS), 200))
    with self.lock:
      if not self._alive_locked() or new_rows == self.rows:
        return
      self.rows = new_rows
      _set_pty_size(self.master_fd, self.rows, self.cols)
      proc = self.proc
      rows, cols, session = self.rows, self.cols, self.session
    if proc is not None and proc.poll() is None:
      try:
        os.killpg(proc.pid, signal.SIGWINCH)
      except Exception:
        pass
    self._broadcast({"type": "pty_resize", "session": session, "rows": rows, "cols": cols})

  def _append_history(self, chunk: bytes) -> None:
    self.history.extend(chunk)
    if len(self.history) > PTY_HISTORY_LIMIT:
      del self.history[:len(self.history) - PTY_HISTORY_LIMIT]

  def _broadcast(self, payload: dict) -> None:
    with self.lock:
      clients = list(self.clients)
    stale = []
    for conn in clients:
      if not conn.send_json(payload):
        stale.append(conn)
    if stale:
      with self.lock:
        for conn in stale:
          self.clients.discard(conn)

  def _read_loop(self) -> None:
    sel = selectors.DefaultSelector()
    try:
      with self.lock:
        fd = self.master_fd
      sel.register(fd, selectors.EVENT_READ)
      while True:
        with self.lock:
          if not self._alive_locked():
            break
          fd = self.master_fd
        events = sel.select(timeout=0.5)
        if not events:
          continue
        try:
          chunk = os.read(fd, 4096)
        except OSError:
          break
        if not chunk:
          break
        with self.lock:
          self._append_history(chunk)
          session = self.session
        self._broadcast({
          "type": "pty_output", "session": session,
          "b64": base64.b64encode(chunk).decode("ascii"),
        })
    finally:
      try:
        sel.close()
      except Exception:
        pass
      with self.lock:
        exit_code = self.proc.poll() if self.proc else None
        session = self.session
        clients = list(self.clients)
        self.clients.clear()
        self._close_fd_locked()
        self.proc = None
      for conn in clients:
        conn.send_json({"type": "pty_exit", "session": session, "exit_code": exit_code})
        conn.close()


PTY = RecoveryPty()


# ===================================================================
# Minimal RFC 6455 WebSocket connection (server side, standard library).
# ===================================================================
class WsClosed(Exception):
  pass


class WsConn:
  """One websocket connection. Reads run on the request thread; writes may also
  come from the shared PTY reader thread, so every send holds a lock."""

  def __init__(self, handler: BaseHTTPRequestHandler) -> None:
    self.handler = handler
    self.rfile = handler.rfile
    self.wfile = handler.wfile
    self.send_lock = threading.Lock()
    self.closed = False

  @staticmethod
  def accept_key(key: str) -> str:
    digest = hashlib.sha1((key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")

  def handshake(self, key: str) -> None:
    accept = self.accept_key(key)
    headers = (
      "HTTP/1.1 101 Switching Protocols\r\n"
      "Upgrade: websocket\r\n"
      "Connection: Upgrade\r\n"
      f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
    )
    with self.send_lock:
      self.wfile.write(headers.encode("ascii"))
      self.wfile.flush()

  # ---- framing ---------------------------------------------------
  def _send_frame(self, payload: bytes, opcode: int = 0x1) -> bool:
    header = bytearray()
    header.append(0x80 | opcode)  # FIN + opcode
    n = len(payload)
    if n < 126:
      header.append(n)
    elif n < 65536:
      header.append(126)
      header += struct.pack("!H", n)
    else:
      header.append(127)
      header += struct.pack("!Q", n)
    with self.send_lock:
      if self.closed:
        return False
      try:
        self.wfile.write(bytes(header) + payload)
        self.wfile.flush()
        return True
      except Exception:
        self.closed = True
        return False

  def send_json(self, payload: dict) -> bool:
    try:
      data = json.dumps(payload).encode("utf-8")
    except Exception:
      return False
    return self._send_frame(data, 0x1)

  def close(self) -> None:
    if self.closed:
      return
    self._send_frame(b"", 0x8)
    self.closed = True

  def _read_exact(self, n: int) -> bytes:
    data = self.rfile.read(n)
    if data is None or len(data) < n:
      raise WsClosed()
    return data

  def read_message(self) -> str | None:
    """Return the next complete text message, or None on close/ping handled."""
    payload = bytearray()
    while True:
      b0, b1 = self._read_exact(2)
      fin = b0 & 0x80
      opcode = b0 & 0x0F
      masked = b1 & 0x80
      length = b1 & 0x7F
      if length == 126:
        length = struct.unpack("!H", self._read_exact(2))[0]
      elif length == 127:
        length = struct.unpack("!Q", self._read_exact(8))[0]
      mask = self._read_exact(4) if masked else b"\x00\x00\x00\x00"
      raw = bytearray(self._read_exact(length)) if length else bytearray()
      if masked:
        for i in range(len(raw)):
          raw[i] ^= mask[i & 3]

      if opcode == 0x8:  # close
        raise WsClosed()
      if opcode == 0x9:  # ping -> pong
        self._send_frame(bytes(raw), 0xA)
        continue
      if opcode == 0xA:  # pong
        continue
      # 0x0 continuation, 0x1 text, 0x2 binary
      payload += raw
      if fin:
        try:
          return payload.decode("utf-8", errors="replace")
        except Exception:
          return ""


def _serve_terminal_ws(handler: BaseHTTPRequestHandler, query: dict) -> None:
  key = handler.headers.get("Sec-WebSocket-Key")
  if not key:
    handler.send_error(400, "missing websocket key")
    return
  conn = WsConn(handler)
  handler.close_connection = True
  conn.handshake(key)

  reset = (query.get("reset") or "") in ("1", "true", "yes")
  try:
    PTY.attach(conn, reset=reset)
  except Exception as exc:
    conn.send_json({"type": "error", "error": str(exc), "session": PTY.session})
    conn.close()
    return

  try:
    while True:
      message = conn.read_message()
      if message is None:
        continue
      try:
        data = json.loads(message)
      except Exception:
        continue
      typ = data.get("type")
      try:
        if typ == "input":
          PTY.write_text(str(data.get("data") or "") + "\r")
        elif typ == "raw":
          text = str(data.get("data") or "")
          if text:
            PTY.write_text(text)
        elif typ == "resize":
          PTY.resize(int(data.get("rows") or PTY_FIXED_ROWS))
        elif typ == "control":
          action = (data.get("action") or "").strip()
          if action == "ctrl_c":
            PTY.write(b"\x03")
          elif action == "clear":
            PTY.clear_history()
            PTY.write(b"clear\r")
          elif action == "refresh":
            PTY.write(b"\x0c")
          elif action == "detach":
            # AGNOS tmux prefix is backtick, so detach = ` then d.
            PTY.write(b"\x60d")
      except Exception as exc:
        conn.send_json({"type": "error", "error": str(exc), "session": PTY.session})
  except (WsClosed, OSError, ConnectionError):
    pass
  finally:
    PTY.detach(conn)


# ===================================================================
# Git recovery actions (unchanged behaviour; injected into the PTY terminal).
# ===================================================================
def _run_exec(args: list[str], timeout: float = TIMEOUT_SEC) -> tuple[int, str]:
  # Force UTF-8 decode (errors="replace") instead of the platform locale so a
  # non-UTF-8 console (e.g. a Windows cp949 dev box) can't crash the reader on
  # Korean commit messages; the device is UTF-8 either way.
  proc = subprocess.run(
    args, cwd=str(REPO_ROOT), capture_output=True, timeout=timeout, check=False,
    encoding="utf-8", errors="replace",
  )
  return proc.returncode, (proc.stdout or "") + (proc.stderr or "")


def _git_branches() -> dict:
  _run_exec(["git", "fetch", "--all", "--prune"], 180)
  current = (_run_exec(["git", "branch", "--show-current"], 15)[1] or "").strip()
  rc, out = _run_exec(
    ["git", "for-each-ref",
     "--format=%(refname:short)\t%(refname)\t%(upstream:short)",
     "refs/heads/", "refs/remotes/"],
    30,
  )
  if rc != 0:
    return {"ok": False, "error": out}
  branches: list[dict] = []
  for line in out.splitlines():
    parts = line.split("\t")
    if len(parts) < 2:
      continue
    name = parts[0]
    fullref = parts[1]
    tracking = parts[2] if len(parts) > 2 else ""
    if name.endswith("/HEAD"):
      continue
    if fullref.startswith("refs/heads/"):
      kind = "local"
    elif fullref.startswith("refs/remotes/"):
      kind = "remote"
    else:
      continue
    branches.append({"name": name, "kind": kind, "tracking": tracking})
  return {"ok": True, "current": current, "branches": branches}


def _git_reboot() -> dict:
  for cmd in (["sudo", "reboot"], ["reboot"]):
    try:
      subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
      return {"ok": True, "rebooting": True}
    except FileNotFoundError:
      continue
    except Exception as exc:
      return {"ok": False, "error": str(exc)}
  return {"ok": False, "error": "reboot not available"}


def _git_command(action: str, payload: dict) -> str | None:
  """Returns a shell command string to inject into the terminal. None if invalid."""
  if action == "git_pull":
    return "git reset --hard && git pull"
  if action == "git_sync":
    current = (_run_exec(["git", "branch", "--show-current"], 15)[1] or "").strip()
    rc, out = _run_exec(["git", "branch", "--format=%(refname:short)"], 30)
    parts: list[str] = []
    if rc == 0:
      for line in (out or "").splitlines():
        b = line.strip()
        if b and b != current:
          parts.append(f"git branch -D {shlex.quote(b)}")
    parts.append("git fetch --all --prune")
    return " && ".join(parts)
  if action == "git_reset":
    return "git reset --hard HEAD"
  if action == "git_log":
    return "git log --oneline --decorate -30"
  if action == "git_checkout":
    branch = str(payload.get("branch") or "").strip()
    if not branch:
      return None
    if branch.startswith("origin/"):
      local = branch.split("/", 1)[1]
      return (
        "git fetch --all --prune && "
        f"git switch -C {shlex.quote(local)} --track {shlex.quote(branch)}"
      )
    return f"git fetch --all --prune && git switch {shlex.quote(branch)}"
  if action == "git_checkout_commit":
    commit = str(payload.get("commit") or "").strip()
    if not commit:
      return None
    return f"git checkout {shlex.quote(commit)}"
  if action == "git_reset_repo":
    branch = str(payload.get("branch") or "").strip()
    if not branch:
      return None
    b = shlex.quote(branch)
    return (
      # robust factory reset (matches the tools server): clear stale locks +
      # abort any in-progress op (';' so failures are ignored), rebuild remote
      # refs so a corrupt ref can't block fetch, then FORCE the branch to the
      # remote ('&&' chain stops only on a real failure).
      "find .git -type f -name '*.lock' -delete 2>/dev/null; "
      "git remote set-url origin https://github.com/ajouatom/openpilot.git 2>/dev/null; "
      "git merge --abort 2>/dev/null; git rebase --abort 2>/dev/null; "
      "git cherry-pick --abort 2>/dev/null; git revert --abort 2>/dev/null; "
      "git am --abort 2>/dev/null; git bisect reset 2>/dev/null; "
      "git pack-refs --all 2>/dev/null; rm -rf .git/refs/remotes/origin 2>/dev/null; "
      "git fetch origin --prune --force && "
      f"git checkout -f -B {b} origin/{b} && "
      f"git reset --hard origin/{b} && "
      "git clean -xfd"
    )
  if action == "git_rebuild":
    return (
      "cd /data/openpilot && "
      "scons -c && "
      "rm -f .sconsign.dblite && "
      "rm -rf /tmp/scons_cache && "
      "rm -f prebuilt"
    )
  return None


def _git_log_output() -> dict:
  rc, out = _run_exec(["git", "log", "--oneline", "-30"], 30)
  if rc != 0:
    return {"ok": False, "error": out}
  rc_head, out_head = _run_exec(["git", "rev-parse", "--short", "HEAD"], 10)
  current = out_head.strip() if rc_head == 0 else ""
  commits: list[dict] = []
  for line in (out or "").splitlines():
    line = line.strip()
    if not line:
      continue
    parts = line.split(" ", 1)
    commits.append({"hash": parts[0], "message": parts[1] if len(parts) > 1 else ""})
  return {"ok": True, "commits": commits, "current": current}


def _git_action(action: str, payload: dict) -> dict:
  if action not in GIT_ACTIONS:
    return {"ok": False, "error": f"unknown action: {action}"}
  if action == "git_branches":
    return _git_branches()
  if action == "git_reboot":
    return _git_reboot()
  if action == "git_log":
    return _git_log_output()
  cmd = _git_command(action, payload)
  if cmd is None:
    return {"ok": False, "error": "invalid action or missing parameters"}
  return {"ok": True, "command": cmd}


# ===================================================================
# Tools-menu actions (ported from dispatcher.py, openpilot-free subset).
# ===================================================================
def _capture_tmux_log() -> tuple[int, str]:
  """Capture the current tmux pane to /data/media/tmux.log (mirrors
  dispatcher.capture_tmux_log_sync). No -t: captures the most recent tmux
  session (e.g. `comma` after `tmux a`)."""
  try:
    os.remove(TMUX_LOG_PATH)
  except FileNotFoundError:
    pass
  except OSError as exc:
    return 1, str(exc)
  try:
    proc = subprocess.run(
      ["tmux", "capture-pane", "-pq", "-S-1000"],
      capture_output=True, encoding="utf-8", errors="replace", check=False, timeout=10,
    )
  except FileNotFoundError:
    return 1, "tmux not available"
  except Exception as exc:
    return 1, str(exc)
  if proc.returncode != 0:
    return proc.returncode, (proc.stderr or proc.stdout or "tmux capture failed").strip()
  os.makedirs(os.path.dirname(TMUX_LOG_PATH), exist_ok=True)
  with open(TMUX_LOG_PATH, "w", encoding="utf-8") as f:
    f.write(proc.stdout or "")
  return 0, ""


def _tool_action(action: str, payload: dict) -> dict:
  if action == "rebuild_all":
    # Same as the tools button: clean build + drop prebuilt, then reboot.
    # Returned as a command so it runs in the visible PTY (like the git menu).
    return {"ok": True, "command": "cd /data/openpilot && scons -c && rm -rf prebuilt && sudo reboot"}
  if action == "send_tmux_log":
    rc, err = _capture_tmux_log()
    if rc != 0:
      return {"ok": False, "error": err or "tmux capture failed"}
    return {"ok": True, "file": "/download/tmux.log"}
  if action == "server_tmux_log":
    rc, err = _capture_tmux_log()
    if rc != 0:
      return {"ok": False, "error": err or "tmux capture failed"}
    result = _send_tmux_destinations("tmux_send")
    result["file"] = "/download/tmux.log"
    return result
  return {"ok": False, "error": f"unknown action: {action}"}


# ===================================================================
# Remote support (ported from server/services/support_terminal.py + support_tunnel.py
# + support_discord.py). Standalone stdlib reimplementation: shares recovery's PTY,
# runs a SEPARATE guest HTTP server (so the public tunnel only exposes the restricted
# guest page, not the recovery owner UI) + a cloudflared quick tunnel, and delivers
# URL+PIN via the same Discord webhook (openpilot metadata degraded to git+hostname).
# ===================================================================
CARROT_DATA_DIR = os.environ.get("CARROT_DATA_DIR", "/data/carrot")
SUPPORT_GUEST_DIR = str(WEB_DIR / "support_terminal")
SUPPORT_GUEST_HTML_PATH = os.path.join(SUPPORT_GUEST_DIR, "guest.html")
SUPPORT_GUEST_ASSETS = {
  "design-tokens.css": str(WEB_DIR / "css" / "generated" / "design-tokens.css"),
  "design-system.css": str(WEB_DIR / "css" / "generated" / "design-system.css"),
  "tokens.css": str(WEB_DIR / "css" / "tokens.css"),
  "layout_tokens.css": str(WEB_DIR / "css" / "layout_tokens.css"),
  "base.css": str(WEB_DIR / "css" / "base.css"),
  "layout.css": str(WEB_DIR / "css" / "layout.css"),
  "components.css": str(WEB_DIR / "css" / "components.css"),
  "terminal.css": str(WEB_DIR / "css" / "generated" / "terminal.css"),
  "terminal_typing_indicator.js": str(WEB_DIR / "js" / "generated" / "terminal-shared.js"),
  "guest.css": str(WEB_DIR / "css" / "generated" / "terminal-guest.css"),
  "guest.js": str(WEB_DIR / "js" / "generated" / "terminal-guest.js"),
  "xterm.css": str(WEB_DIR / "css" / "vendor" / "xterm.css"),
  "xterm.js": str(WEB_DIR / "js" / "vendor" / "xterm.js"),
  "xterm-addon-shim.js": str(WEB_DIR / "js" / "vendor" / "xterm-addon-shim.js"),
  "xterm-addon-webgl.js": str(WEB_DIR / "js" / "vendor" / "xterm-addon-webgl.js"),
  "xterm-addon-canvas.js": str(WEB_DIR / "js" / "vendor" / "xterm-addon-canvas.js"),
}

SUPPORT_TTL_SECONDS = int(os.environ.get("CARROT_SUPPORT_TTL_SECONDS", "1800"))
SUPPORT_COMMAND_TIMEOUT_SECONDS = int(os.environ.get("CARROT_SUPPORT_COMMAND_TIMEOUT_SECONDS", "30"))
ALLOWED_TTL_SECONDS = {900, 1800, 3600}
ALLOWED_PERMISSION_MODES = {"approve_each", "allow_all"}
ALLOWED_COMMAND_TIMEOUT_SECONDS = {15, 30, 60, 120}
PIN_FAILURE_LIMIT = 5

TRYCLOUDFLARE_RE = re.compile(r"https://[a-zA-Z0-9.-]+\.trycloudflare\.com")
CLOUDFLARED_DOWNLOADS = {
  "x86_64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64",
  "amd64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64",
  "aarch64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64",
  "arm64": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64",
  "armv7l": "https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm",
}


def _ws_upgrade(handler: BaseHTTPRequestHandler) -> "WsConn | None":
  key = handler.headers.get("Sec-WebSocket-Key")
  if not key:
    handler.send_error(400, "missing websocket key")
    return None
  conn = WsConn(handler)
  handler.close_connection = True
  conn.handshake(key)
  return conn


# --- Discord webhook (obfuscation copied from support_discord.py; base64+xor only) ---
_SUPPORT_OBFUSCATION_KEY = b"carrot-support-v1"
_SUPPORT_OBFUSCATED_WEBHOOK_URL = (
  "CxUGAhxOAlwRGQMMHQZJWFIMDF0THx0CBBASGAAdH15ZAFZTRkBdRxpLQ0BEWUVFFUYEUE4ZJA0lf0Am"
  "BhgINy1EJ39XAzlEOwNJHxwmJjUkGkYxVwFYQDVXDVQmEyYfHCI6AAZ0KjQZACAbRhIRHBMVGQwcBAY5CQ=="
)


def _support_webhook_url() -> str:
  for key in ("CARROT_SUPPORT_DISCORD_WEBHOOK_URL", "CARROT_DISCORD_WEBHOOK_URL", "DISCORD_WEBHOOK_URL"):
    v = os.environ.get(key, "").strip()
    if v:
      return v
  try:
    data = base64.b64decode(_SUPPORT_OBFUSCATED_WEBHOOK_URL)
    decoded = bytes(b ^ _SUPPORT_OBFUSCATION_KEY[i % len(_SUPPORT_OBFUSCATION_KEY)] for i, b in enumerate(data))
    return decoded.decode("utf-8").strip()
  except Exception:
    return ""


def _read_param(key: str, default: str = "") -> str:
  try:
    with open(os.path.join(PARAMS_DIR, "d", key), "r", encoding="utf-8", errors="replace") as f:
      return f.read().strip() or default
  except Exception:
    return default


def _write_param(key: str, value: str) -> None:
  param_dir = os.path.join(PARAMS_DIR, "d")
  if not os.path.isdir(param_dir):
    raise FileNotFoundError(f"params unavailable: {param_dir}")
  target = os.path.join(param_dir, key)
  temp = os.path.join(param_dir, f".tmp_{key}_{secrets.token_hex(6)}")
  lock_fd = None
  try:
    if fcntl is not None:
      lock_fd = os.open(os.path.join(PARAMS_DIR, ".lock"), os.O_CREAT | os.O_RDWR, 0o664)
      fcntl.flock(lock_fd, fcntl.LOCK_EX)
    fd = os.open(temp, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o664)
    try:
      os.write(fd, str(value).encode("utf-8"))
      os.fsync(fd)
    finally:
      os.close(fd)
    os.replace(temp, target)
    dir_fd = os.open(param_dir, os.O_RDONLY)
    try:
      os.fsync(dir_fd)
    finally:
      os.close(dir_fd)
  finally:
    try:
      os.unlink(temp)
    except FileNotFoundError:
      pass
    if lock_fd is not None:
      fcntl.flock(lock_fd, fcntl.LOCK_UN)
      os.close(lock_fd)


def _cwp_device_id() -> str:
  for value in (_read_param("DongleId"), _read_param("HardwareSerial"), socket.gethostname()):
    text = str(value or "").strip()
    if text and text.lower() not in {"unknown", "none", "null", "unregistereddevice"}:
      return text
  return "comma"


def _cwp_url(path: str) -> str:
  report_url = os.environ.get("CWEB_PUSH_REPORT_URL", "").strip()
  if not report_url:
    report_url = "".join(chr(value ^ CWP_REPORT_URL_KEY) for value in CWP_REPORT_URL_BYTES)
  base = report_url[:-len("/report")] if report_url.endswith("/report") else report_url.rstrip("/")
  return base + path


def _cwp_request(path: str, payload: dict, timeout: int = 4) -> dict:
  headers = {
    "Content-Type": "application/json",
    "Accept": "application/json",
    "User-Agent": "CarrotRecovery/2.0",
  }
  token = os.environ.get("CWP_REPORT_TOKEN", "").strip()
  if token:
    headers["Authorization"] = f"Bearer {token}"
  request = urllib.request.Request(
    _cwp_url(path), data=json.dumps(payload).encode("utf-8"), headers=headers, method="POST",
  )
  return _request_result(request, timeout)


def _cwp_status() -> dict:
  enabled = _read_param(CWP_RECOVERY_BOOT_PARAM) == "1"
  result = _cwp_request("/recovery/status", {"deviceId": _cwp_device_id()})
  body = result.get("body") if isinstance(result.get("body"), dict) else {}
  if not result.get("ok") or not body.get("ok"):
    return {
      "ok": False,
      "enabled": enabled,
      "registered": None,
      "state": "unavailable",
      "error": result.get("error") or "CWP unavailable",
    }
  registered = bool(body.get("registered"))
  if enabled and not registered:
    _write_param(CWP_RECOVERY_BOOT_PARAM, "0")
    enabled = False
  return {
    "ok": True,
    "enabled": enabled,
    "registered": registered,
    "state": "ready" if registered else "unregistered",
  }


def _cwp_set_enabled(enabled: bool) -> dict:
  status = _cwp_status()
  if enabled:
    if not status.get("ok"):
      return status
    if not status.get("registered"):
      return {**status, "ok": False, "error": "Not registered"}
  _write_param(CWP_RECOVERY_BOOT_PARAM, "1" if enabled else "0")
  return {**status, "ok": True, "enabled": enabled}


def _cwp_boot_worker(port: int = DEFAULT_PORT) -> None:
  if _read_param(CWP_RECOVERY_BOOT_PARAM) != "1":
    return
  deadline = time.monotonic() + 120.0
  candidate = ""
  while time.monotonic() < deadline:
    local_ip = _local_ip()
    if not local_ip or local_ip != candidate:
      candidate = local_ip
      time.sleep(2.0)
      continue
    result = _cwp_request("/recovery/boot", {
      "deviceId": _cwp_device_id(),
      "ip": local_ip,
      "port": int(port),
    })
    body = result.get("body") if isinstance(result.get("body"), dict) else {}
    if result.get("ok") and body.get("ok"):
      pushed = int(body.get("pushed") or 0)
      registered = bool(body.get("registered"))
      if not registered:
        _write_param(CWP_RECOVERY_BOOT_PARAM, "0")
      print(f"[recovery] CWP boot registered={registered} pushed={pushed}", flush=True)
      return
    time.sleep(5.0)
  print("[recovery] CWP boot unavailable", flush=True)


def _exception_webhook_url() -> str:
  if os.environ.get("CARROT_EXCEPTION_DISCORD_WEBHOOK_DISABLE", "").strip().lower() in {"1", "true", "yes", "on"}:
    return ""
  for key in ("CARROT_EXCEPTION_DISCORD_WEBHOOK_URL", "CARROT_DISCORD_WEBHOOK_URL", "DISCORD_WEBHOOK_URL"):
    value = os.environ.get(key, "").strip()
    if value:
      return value
  for key in (
    "CarrotExceptionDiscordWebhookUrl",
    "CarrotDiscordWebhookUrl",
    "CarrotDiscordWebhookURL",
    "DiscordWebhookUrl",
    "DiscordWebhookURL",
  ):
    value = _read_param(key)
    if value:
      return value
  try:
    data = base64.b64decode(EXCEPTION_DISCORD_WEBHOOK_OBFUSCATED)
    decoded = bytes(
      byte ^ EXCEPTION_DISCORD_WEBHOOK_KEY[index % len(EXCEPTION_DISCORD_WEBHOOK_KEY)]
      for index, byte in enumerate(data)
    )
    return decoded.decode("utf-8").strip()
  except Exception:
    return ""


def _exception_repo_url() -> str:
  remote = _read_param("GitRemote")
  if remote.startswith("git@github.com:"):
    remote = "https://github.com/" + remote[len("git@github.com:"):]
  if remote.startswith(("https://github.com/", "http://github.com/")):
    return remote.removesuffix(".git").replace("http://github.com/", "https://github.com/", 1)
  github_user = _read_param("GithubUsername")
  return f"https://github.com/{github_user}/openpilot" if github_user else "https://github.com/ajouatom/openpilot"


def _local_ip() -> str:
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  try:
    sock.connect(("8.8.8.8", 80))
    value = sock.getsockname()[0]
    return value if ipaddress.ip_address(value).is_private else ""
  except Exception:
    return ""
  finally:
    sock.close()


def _tmux_upload_payload(reason: str) -> dict[str, str]:
  return {
    "tmux_why": reason,
    "car_name": _read_param("CarName"),
    "git_branch": _read_param("GitBranch"),
    "github_id": _read_param("GithubUsername"),
    "git_remote": _read_param("GitRemote"),
    "git_commit": _read_param("GitCommit"),
    "git_commit_date": _read_param("GitCommitDate"),
    "dongle_id": _read_param("DongleId"),
    "device_serial": _read_param("HardwareSerial"),
    "local_ip": _local_ip(),
  }


def _web_upload_base_url() -> str:
  configured = os.environ.get("CARROT_WEB_UPLOAD_URL", "").strip()
  if not configured:
    state_dir = os.path.join(os.environ.get("CARROT_DATA_DIR", "/data/carrot"), "state")
    try:
      with open(os.path.join(state_dir, "web_settings.json"), encoding="utf-8") as file:
        settings = json.load(file)
      configured = str(settings.get("web_upload_url") or settings.get("toss_upload_url") or "").strip()
    except Exception:
      configured = ""
  url = (configured or DEFAULT_WEB_UPLOAD_URL).rstrip("/")
  if not url.startswith(("http://", "https://")):
    raise ValueError("web upload URL must start with http:// or https://")
  return url


def _carrot_logs_url() -> str:
  url = (os.environ.get("CARROT_TMUX_WEB_UPLOAD_URL", "").strip() or DEFAULT_TMUX_WEB_UPLOAD_URL).rstrip("/")
  if not url.startswith(("http://", "https://")):
    raise ValueError("Carrot Logs URL must start with http:// or https://")
  return url


def _exception_discord_content(reason: str, web_result: dict | None = None) -> str:
  branch = _read_param("GitBranch", "unknown")
  commit = _read_param("GitCommit", "unknown")
  commit_date = _read_param("GitCommitDate", "unknown")
  repo_url = _exception_repo_url()
  commit_text = f"[{commit[:8]}]({repo_url}/commit/{commit})" if commit and commit != "unknown" else "unknown"
  web_result = web_result or {}
  web_status = web_result.get("status")
  web_text = "ok" if web_result.get("ok") else "failed"
  if web_status is not None:
    web_text += f" ({web_status})"
  return "\n".join((
    "# Carrot Exception",
    "### Upload",
    f"- Time: {time.strftime('%Y-%m-%d %H:%M:%S')}",
    f"- Reason: {reason}",
    f"- Web: {web_text}",
    "### Device",
    f"- Car name: {_read_param('CarName', 'none')}",
    f"- DongleId: {_read_param('DongleId', 'unknown')}",
    f"- Serial: {_read_param('HardwareSerial', 'unknown')}",
    f"- GitHub: {repo_url}",
    f"- Branch: {branch}",
    f"- Commit: {commit_text} ({commit_date})",
  ))[:1900]


def _multipart_form(fields: dict[str, str], files: list[tuple[str, str, str, bytes]]) -> tuple[bytes, str]:
  boundary = f"----CarrotRecovery{secrets.token_hex(16)}"
  body = bytearray()

  def add(value: bytes) -> None:
    body.extend(value)
    body.extend(b"\r\n")

  for name, value in fields.items():
    add(f"--{boundary}".encode())
    add(f'Content-Disposition: form-data; name="{name}"'.encode())
    add(b"")
    add(str(value).encode("utf-8"))
  for name, filename, content_type, file_data in files:
    add(f"--{boundary}".encode())
    add(f'Content-Disposition: form-data; name="{name}"; filename="{filename}"'.encode())
    add(f"Content-Type: {content_type}".encode())
    add(b"")
    add(file_data)
  body.extend(f"--{boundary}--\r\n".encode())
  return bytes(body), boundary


def _discord_multipart(payload: dict, filename: str, file_data: bytes) -> tuple[bytes, str]:
  return _multipart_form(
    {"payload_json": json.dumps(payload, ensure_ascii=False)},
    [("files[0]", filename, "text/plain", file_data)],
  )


def _request_result(request: urllib.request.Request, timeout: int) -> dict:
  try:
    with urllib.request.urlopen(request, timeout=timeout) as response:
      status = int(response.status)
      response_data = response.read(1024 * 1024)
      try:
        response_body = json.loads(response_data.decode("utf-8")) if response_data else None
      except Exception:
        response_body = None
      return {"configured": True, "ok": 200 <= status < 300, "status": status, "body": response_body}
  except urllib.error.HTTPError as exc:
    try:
      detail = exc.read().decode("utf-8", errors="replace")[:500]
    except Exception:
      detail = ""
    return {"configured": True, "ok": False, "status": exc.code, "error": detail or str(exc)}
  except Exception as exc:
    return {"configured": True, "ok": False, "error": str(exc)}


def _post_json(url: str, payload: dict, timeout: int = 12) -> dict:
  request = urllib.request.Request(url, data=json.dumps(payload).encode(), headers={
    "Content-Type": "application/json",
    "Accept": "application/json",
    "User-Agent": "CarrotRecovery/2.0",
  }, method="POST")
  return _request_result(request, timeout)


def _post_tmux_upload(url: str, headers: dict[str, str], payload: dict[str, str], raw: bytes) -> dict:
  body, boundary = _multipart_form(payload, [("files[0]", "tmux.log", "text/plain", raw)])
  request = urllib.request.Request(url, data=body, headers={
    **headers,
    "Content-Type": f"multipart/form-data; boundary={boundary}",
    "Accept": "application/json",
    "User-Agent": "CarrotRecovery/2.0",
  }, method="POST")
  return _request_result(request, 30)


def _send_tmux_dsm(payload: dict[str, str], raw: bytes) -> dict:
  try:
    base_url = _web_upload_base_url()
    token = os.environ.get("CARROT_WEB_UPLOAD_TOKEN", "").strip()
    if not token:
      session_payload = {key: str(value or "")[:160] for key, value in payload.items()}
      session_payload["deviceId"] = payload.get("dongle_id") or payload.get("device_serial") or "unknown"
      session_payload["purpose"] = "tmux"
      session = _post_json(f"{base_url}/api/v1/session", session_payload)
      body = session.get("body") if isinstance(session.get("body"), dict) else {}
      token = str(body.get("token") or "").strip()
      if not session.get("ok") or not body.get("ok") or not token:
        return {**session, "ok": False, "error": session.get("error") or "upload server did not issue a session"}
    return _post_tmux_upload(
      f"{base_url}/api/v1/tmux/upload",
      {"Authorization": f"Bearer {token}"},
      payload,
      raw,
    )
  except Exception as exc:
    return {"configured": True, "ok": False, "error": str(exc)}


def _send_tmux_carrot_logs(payload: dict[str, str], raw: bytes) -> dict:
  try:
    return _post_tmux_upload(_carrot_logs_url(), {}, payload, raw)
  except Exception as exc:
    return {"configured": True, "ok": False, "error": str(exc)}


def _send_tmux_discord(reason: str, raw: bytes | None = None, web_result: dict | None = None) -> dict:
  url = _exception_webhook_url()
  if not url or not url.startswith(("http://", "https://")):
    return {"configured": bool(url), "ok": False, "error": "Discord webhook is not configured"}
  if raw is None:
    try:
      raw = Path(TMUX_LOG_PATH).read_bytes()
    except Exception as exc:
      return {"configured": True, "ok": False, "error": f"tmux log read failed: {exc}"}

  branch = _read_param("GitBranch", "unknown").replace("/", "__").replace("\\", "__")
  stamp = time.strftime("%Y%m%d-%H%M%S")
  filename = f"{reason}-{stamp}-{branch}.txt"
  if len(raw) > DISCORD_TMUX_FILE_MAX_BYTES:
    marker = b"\n\n===== DISCORD TMUX TRUNCATED =====\n\n"
    keep_each = (DISCORD_TMUX_FILE_MAX_BYTES - len(marker)) // 2
    raw = raw[:keep_each] + marker + raw[-keep_each:]
    filename = f"{reason}-{stamp}-{branch}-truncated.txt"

  payload = {
    "username": "Carrot Exception",
    "content": _exception_discord_content(reason, web_result),
    "allowed_mentions": {"parse": []},
    "flags": 4,
  }
  body, boundary = _discord_multipart(payload, filename, raw)
  request = urllib.request.Request(url, data=body, headers={
    "Content-Type": f"multipart/form-data; boundary={boundary}",
    "Accept": "application/json",
    "User-Agent": "CarrotRecovery/2.0",
  }, method="POST")
  result = _request_result(request, 12)
  result.pop("body", None)
  return result


def _send_tmux_destinations(reason: str) -> dict:
  try:
    raw = Path(TMUX_LOG_PATH).read_bytes()
  except Exception as exc:
    return {"ok": False, "error": f"tmux log read failed: {exc}"}
  payload = _tmux_upload_payload(reason)
  dsm = _send_tmux_dsm(payload, raw)
  carrot_logs = _send_tmux_carrot_logs(payload, raw)
  discord = _send_tmux_discord(reason, raw, dsm)
  destinations = {"dsm": dsm, "carrot_logs": carrot_logs, "discord": discord}
  ok = any(result.get("ok") for result in destinations.values())
  failed = [name for name, result in destinations.items() if not result.get("ok")]
  return {
    "ok": ok,
    "partial": ok and bool(failed),
    "destinations": destinations,
    "error": "" if ok else "; ".join(
      f"{name}: {result.get('error') or result.get('status') or 'failed'}"
      for name, result in destinations.items()
    ),
  }


def _support_metadata() -> dict:
  return {
    "carName": _read_param("CarName", "none") or "none",
    "dongleId": _read_param("DongleId", "unknown") or "unknown",
    "serial": _read_param("HardwareSerial") or _read_param("DeviceSerial") or _read_param("Serial") or "unknown",
    "branch": (_run_exec(["git", "branch", "--show-current"], 6)[1] or "").strip() or "unknown",
    "commit": (_run_exec(["git", "rev-parse", "--short", "HEAD"], 6)[1] or "").strip() or "unknown",
    "host": socket.gethostname(),
  }


def _support_message(payload: dict) -> str:
  meta = payload.get("meta") or {}
  note = str(payload.get("note") or "").strip()
  commit = str(meta.get("commit") or "").strip()
  commit_text = (f"[{commit}](https://github.com/ajouatom/openpilot/commit/{commit})"
                 if commit and commit != "unknown" else "unknown")
  permission_text = {"approve_each": "항상 확인", "allow_all": "전체 허용"}.get(
    str(payload.get("permissionMode") or "approve_each"), str(payload.get("permissionMode")))
  ttl = payload.get("ttl_minutes") or 30
  expires_text = "unlimited" if ttl == "unlimited" else f"{ttl} min"
  lines = [
    "# Carrot Remote Terminal (recovery)",
    "### Session",
    f"- Time: {payload.get('createdAt') or time.strftime('%Y-%m-%d %H:%M:%S')}",
    f"- Session ID: {payload.get('sessionId') or 'unknown'}",
    "### Access",
    f"- Link: {payload.get('url') or ''}",
    f"- PIN: **__{payload.get('pin') or ''}__**",
    f"- Expires: {expires_text}",
    f"- Permission: {permission_text}",
  ]
  if note:
    lines += ["### Issue", f"- Note: {note[:500]}"]
  lines += [
    "### Device",
    f"- Car name: {meta.get('carName') or 'none'}",
    f"- DongleId: {meta.get('dongleId') or 'unknown'}",
    f"- Serial: {meta.get('serial') or 'unknown'}",
    f"- Branch: {meta.get('branch') or 'unknown'}",
    f"- Commit: {commit_text}",
  ]
  return "\n".join(lines)[:1900]


def _send_support_webhook(payload: dict) -> dict:
  if os.environ.get("CARROT_SUPPORT_DISCORD_WEBHOOK_DISABLE", "").strip().lower() in {"1", "true", "yes", "on"}:
    return {"configured": True, "ok": False, "skipped": True, "disabled": True}
  url = _support_webhook_url()
  if not url or not url.startswith(("http://", "https://")):
    return {"configured": bool(url), "ok": False, "skipped": True}
  body = json.dumps({
    "username": "Carrot Support",
    "content": _support_message(payload),
    "allowed_mentions": {"parse": []},
    "flags": 4,
  }).encode("utf-8")
  req = urllib.request.Request(url, data=body, headers={
    "Content-Type": "application/json",
    "Accept": "application/json",
    # Discord rejects urllib's default Python-urllib user agent with HTTP 403.
    "User-Agent": "CarrotRecovery/2.0",
  }, method="POST")
  try:
    with urllib.request.urlopen(req, timeout=12) as resp:
      return {"configured": True, "ok": 200 <= resp.status < 300, "status": resp.status}
  except urllib.error.HTTPError as exc:
    try:
      detail = exc.read().decode("utf-8", errors="replace")[:500]
    except Exception:
      detail = ""
    return {
      "configured": True,
      "ok": False,
      "status": exc.code,
      "error": detail or str(exc),
    }
  except Exception as exc:
    return {"configured": True, "ok": False, "error": str(exc)}


# --- cloudflared quick tunnel ---
def _cloudflared_path() -> str:
  configured = os.environ.get("CARROT_CLOUDFLARED_BIN", "").strip()
  if configured:
    return configured
  found = shutil.which("cloudflared")
  if found:
    return found
  local = os.path.join(CARROT_DATA_DIR, "bin", "cloudflared")
  if os.path.isfile(local) and os.access(local, os.X_OK):
    return local
  return ""


def _download_cloudflared() -> str:
  if platform.system().lower() != "linux":
    raise RuntimeError(f"cloudflared unavailable on {platform.system()} {platform.machine()}")
  url = CLOUDFLARED_DOWNLOADS.get(platform.machine().lower(), "")
  if not url:
    raise RuntimeError(f"cloudflared auto-install unsupported on {platform.machine()}")
  target = os.path.join(CARROT_DATA_DIR, "bin", "cloudflared")
  os.makedirs(os.path.dirname(target), exist_ok=True)
  tmp = f"{target}.tmp"
  with urllib.request.urlopen(url, timeout=60) as resp, open(tmp, "wb") as out:
    shutil.copyfileobj(resp, out)
  os.chmod(tmp, 0o755)
  os.replace(tmp, target)
  return target


class Tunnel:
  def __init__(self, url: str, proc: subprocess.Popen | None = None):
    self.url = url
    self.proc = proc

  def stop(self) -> None:
    if self.proc and self.proc.poll() is None:
      try:
        self.proc.terminate()
        try:
          self.proc.wait(timeout=4)
        except Exception:
          self.proc.kill()
      except Exception:
        pass


def _start_quick_tunnel(local_url: str, timeout_s: float = 25.0) -> Tunnel:
  fake = os.environ.get("CARROT_SUPPORT_FAKE_TUNNEL_URL", "").strip()
  if fake:
    return Tunnel(url=fake)
  binary = _cloudflared_path() or _download_cloudflared()
  proc = subprocess.Popen(
    [binary, "tunnel", "--url", local_url],
    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1,
  )
  url = ""
  deadline = time.time() + timeout_s
  while time.time() < deadline:
    line = proc.stdout.readline() if proc.stdout else ""
    if not line:
      if proc.poll() is not None:
        break
      continue
    match = TRYCLOUDFLARE_RE.search(line)
    if match:
      url = match.group(0)
      break
  if not url:
    Tunnel(url="", proc=proc).stop()
    raise RuntimeError("cloudflared did not provide a trycloudflare URL")
  # Keep draining stdout so the OS pipe buffer can't fill and stall cloudflared.
  threading.Thread(target=lambda: [None for _ in iter(proc.stdout.readline, "")], daemon=True).start()
  return Tunnel(url=url, proc=proc)


# --- session + manager ---
def _now() -> float:
  return time.time()


def _new_pin() -> str:
  return f"{secrets.randbelow(1_000_000):06d}"


def _sanitize_ttl(v) -> int:
  try:
    s = int(v)
  except Exception:
    s = SUPPORT_TTL_SECONDS
  return s if s in ALLOWED_TTL_SECONDS else (SUPPORT_TTL_SECONDS if SUPPORT_TTL_SECONDS in ALLOWED_TTL_SECONDS else 1800)


def _sanitize_permission(v) -> str:
  m = str(v or "approve_each").strip()
  return m if m in ALLOWED_PERMISSION_MODES else "approve_each"


def _sanitize_cmd_timeout(v) -> int:
  try:
    s = int(v)
  except Exception:
    s = SUPPORT_COMMAND_TIMEOUT_SECONDS
  return s if s in ALLOWED_COMMAND_TIMEOUT_SECONDS else (SUPPORT_COMMAND_TIMEOUT_SECONDS if SUPPORT_COMMAND_TIMEOUT_SECONDS in ALLOWED_COMMAND_TIMEOUT_SECONDS else 30)


def _sanitize_typing(v) -> str:
  return "".join(ch for ch in str(v or "") if ch.isprintable())[:160]


class SupportSession:
  def __init__(self, sid, pin, note, ttl, permission_mode, command_timeout):
    self.id = sid
    self.pin = pin
    self.note = note
    self.created_at = _now()
    self.ttl_seconds = ttl
    self.expires_at = 0 if ttl <= 0 else _now() + ttl
    self.command_timeout_seconds = command_timeout
    self.permission_mode = permission_mode
    self.state = "starting"
    self.tunnel_url = ""
    self.local_url = ""
    self.error = ""
    self.status_detail = ""
    self.discord: dict = {}
    self.pin_failures = 0
    self.pending: dict = {}          # id -> command dict
    self.tunnel: Tunnel | None = None
    self.guest_httpd: ThreadingHTTPServer | None = None
    self.guest_port = 0
    self.owner_conns: set = set()    # WsConn
    self.guest_conns: set = set()    # WsConn
    self.controller = None           # WsConn

  def is_expired(self) -> bool:
    return self.expires_at > 0 and _now() >= self.expires_at


class SupportManager:
  def __init__(self) -> None:
    self.lock = threading.RLock()
    self.session: SupportSession | None = None

  def snapshot(self, session: SupportSession | None = None) -> dict:
    session = session or self.session
    if session is None:
      return {"ok": True, "active": False, "state": "idle"}
    remaining = None if session.expires_at <= 0 else max(0, int(session.expires_at - _now()))
    return {
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
      "guest_count": len(session.guest_conns),
      "owner_count": len(session.owner_conns),
      "owner_present": bool(session.owner_conns),
      "controller_present": session.controller is not None and not session.controller.closed,
      "discord": session.discord,
      "error": session.error,
      "status_detail": session.status_detail,
      "pending_commands": [
        {"id": c["id"], "line": c["line"], "status": c["status"], "created_at": c["created_at"]}
        for c in session.pending.values() if c["status"] == "pending"
      ],
    }

  def _broadcast(self, conns: set, payload: dict) -> None:
    for conn in list(conns):
      if conn.closed or not conn.send_json(payload):
        conns.discard(conn)

  def broadcast_owner(self, payload: dict) -> None:
    if self.session:
      self._broadcast(self.session.owner_conns, payload)

  def broadcast_guests(self, payload: dict) -> None:
    if self.session:
      self._broadcast(self.session.guest_conns, payload)

  def broadcast_all(self, payload: dict) -> None:
    if self.session:
      self._broadcast(self.session.owner_conns, payload)
      self._broadcast(self.session.guest_conns, payload)

  def _set_status(self, session: SupportSession, detail: str) -> None:
    session.status_detail = detail
    if self.session is session:
      self.broadcast_owner({"type": "session_status", **self.snapshot(session)})

  def start(self, note="", ttl_seconds=None, permission_mode=None, command_timeout_seconds=None) -> dict:
    with self.lock:
      if self.session and self.session.state in {"starting", "sharing"} and not self.session.is_expired():
        return self.snapshot(self.session)
      if self.session:
        self._stop_locked("restart")
      session = SupportSession(
        sid=secrets.token_urlsafe(16),
        pin=_new_pin(),
        note=str(note or "")[:500],
        ttl=_sanitize_ttl(ttl_seconds),
        permission_mode=_sanitize_permission(permission_mode),
        command_timeout=_sanitize_cmd_timeout(command_timeout_seconds),
      )
      self.session = session
    try:
      PTY.ensure()
      self._set_status(session, "Starting support page")
      self._start_guest_server(session)
      self._set_status(session, "Starting secure tunnel")
      session.tunnel = _start_quick_tunnel(f"http://127.0.0.1:{session.guest_port}")
      base = session.tunnel.url.rstrip("/")
      session.tunnel_url = base if "/support/terminal/" in base else f"{base}/support/terminal/{session.id}"
      session.state = "sharing"
      self._set_status(session, "Secure tunnel ready")
      if session.expires_at > 0:
        threading.Thread(target=self._expiry_loop, args=(session,), daemon=True).start()

      def _notify():
        session.discord = _send_support_webhook({
          "sessionId": session.id,
          "createdAt": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(session.created_at)),
          "url": session.tunnel_url, "pin": session.pin,
          "ttl_minutes": "unlimited" if session.ttl_seconds <= 0 else max(1, session.ttl_seconds // 60),
          "permissionMode": session.permission_mode,
          "commandTimeoutSeconds": session.command_timeout_seconds,
          "meta": _support_metadata(), "note": session.note,
        })
        self.broadcast_owner({"type": "session_status", **self.snapshot(session)})
      threading.Thread(target=_notify, daemon=True).start()
      self._set_status(session, "Ready")
      return self.snapshot(session)
    except Exception as exc:
      if self.session is session:
        session.state = "error"
        session.error = str(exc)
        session.status_detail = "Start failed"
        self._cleanup(session)
        self.broadcast_owner({"type": "error", "message": session.error})
        return self.snapshot(session)
      self._cleanup(session)
      return self.snapshot(None)

  def stop(self, reason="stopped") -> dict:
    with self.lock:
      self._stop_locked(reason)
      return self.snapshot(None)

  def _stop_locked(self, reason: str) -> None:
    session = self.session
    if session is None:
      return
    session.state = "stopped" if reason != "expired" else "expired"
    self.broadcast_all({"type": "session_closed", "reason": reason})
    self.session = None
    threading.Thread(target=self._cleanup, args=(session,), daemon=True).start()

  def _cleanup(self, session: SupportSession) -> None:
    for conn in list(session.guest_conns):
      PTY.detach(conn)
    for conn in list(session.owner_conns | session.guest_conns):
      try:
        conn.close()
      except Exception:
        pass
    session.owner_conns.clear()
    session.guest_conns.clear()
    session.controller = None
    if session.tunnel:
      session.tunnel.stop()
      session.tunnel = None
    if session.guest_httpd:
      try:
        session.guest_httpd.shutdown()
        session.guest_httpd.server_close()
      except Exception:
        pass
      session.guest_httpd = None

  def _start_guest_server(self, session: SupportSession) -> None:
    httpd = ThreadingHTTPServer(("127.0.0.1", 0), SupportGuestHandler)
    httpd.daemon_threads = True
    session.guest_httpd = httpd
    session.guest_port = httpd.server_address[1]
    session.local_url = f"http://127.0.0.1:{session.guest_port}/support/terminal/{session.id}"
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

  def _expiry_loop(self, session: SupportSession) -> None:
    while not session.is_expired():
      if self.session is not session:
        return
      self.broadcast_owner({"type": "session_status", **self.snapshot(session)})
      time.sleep(1.0)
    with self.lock:
      if self.session is session:
        self._stop_locked("expired")

  def _sync_control_roles(self, session: SupportSession) -> None:
    if session.permission_mode != "allow_all":
      session.controller = None
    elif session.controller is None or session.controller.closed or session.controller not in session.guest_conns:
      session.controller = next((g for g in session.guest_conns if not g.closed), None)
    for g in list(session.guest_conns):
      if g.closed:
        continue
      g.send_json({"type": "control_role",
                   "granted": session.permission_mode != "allow_all" or g is session.controller})

  # ---- owner side ----
  def owner_connected(self, conn) -> bool:
    session = self.session
    if session is None:
      conn.send_json({"type": "session_status", **self.snapshot(None)})
      return False
    session.owner_conns.add(conn)
    conn.send_json({"type": "session_status", **self.snapshot(session)})
    self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_conns)})
    self.broadcast_guests({"type": "owner_presence", "active": True})
    return True

  def owner_message(self, conn, data: dict) -> None:
    session = self.session
    if session is None:
      return
    if data.get("type") == "refresh":
      conn.send_json({"type": "session_status", **self.snapshot(session)})
    elif data.get("type") == "typing":
      self.broadcast_guests({"type": "host_typing", "active": bool(data.get("active")), "text": _sanitize_typing(data.get("text"))})

  def owner_disconnected(self, conn) -> None:
    session = self.session
    if session is None:
      return
    session.owner_conns.discard(conn)
    self.broadcast_guests({"type": "owner_presence", "active": bool(session.owner_conns)})

  # ---- guest side ----
  def guest_auth(self, conn, session: SupportSession, pin: str):
    if pin != session.pin:
      session.pin_failures += 1
      conn.send_json({"type": "auth_failed", "remaining": max(0, PIN_FAILURE_LIMIT - session.pin_failures)})
      if session.pin_failures >= PIN_FAILURE_LIMIT:
        conn.close()
        return "closed"
      return False
    session.guest_conns.add(conn)
    if session.permission_mode == "allow_all" and (session.controller is None or session.controller.closed):
      session.controller = conn
    expires_in = None if session.expires_at <= 0 else max(0, int(session.expires_at - _now()))
    conn.send_json({
      "type": "auth_ok",
      "permission_mode": session.permission_mode,
      "command_timeout_seconds": session.command_timeout_seconds,
      "expires_in": expires_in,
      "owner_present": bool(session.owner_conns),
      "control_granted": session.permission_mode != "allow_all" or conn is session.controller,
    })
    try:
      PTY.attach(conn)
    except Exception as exc:
      session.guest_conns.discard(conn)
      conn.send_json({"type": "error", "message": str(exc)})
      conn.close()
      return "closed"
    self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_conns)})
    self._sync_control_roles(session)
    return True

  def guest_message(self, conn, session: SupportSession, data: dict) -> None:
    typ = data.get("type")
    if typ == "typing":
      self.broadcast_owner({"type": "guest_typing", "active": bool(data.get("active")), "text": _sanitize_typing(data.get("text"))})
    elif typ == "close_session":
      self.stop("guest")
      raise WsClosed()
    elif typ == "disconnect":
      raise WsClosed()
    elif typ == "control":
      if session.permission_mode == "allow_all" and conn is not session.controller:
        conn.send_json({"type": "input_denied", "reason": "viewer only"})
        return
      action = str(data.get("action") or "").strip()
      if action in {"ctrl_c", "clear"}:
        label = "Ctrl+C" if action == "ctrl_c" else "clear"
        self.queue_command(session, label, conn, control_action=action)
    elif typ == "input":
      if session.permission_mode == "allow_all" and conn is not session.controller:
        conn.send_json({"type": "input_denied", "reason": "viewer only"})
        return
      line = str(data.get("data") or "").strip()
      if line:
        self.queue_command(session, line, conn)
    elif typ == "raw":
      if not session.owner_conns:
        conn.send_json({"type": "owner_absent"})
        return
      if session.permission_mode != "allow_all":
        conn.send_json({"type": "input_denied", "reason": "approval required"})
        return
      if conn is not session.controller:
        conn.send_json({"type": "input_denied", "reason": "viewer only"})
        return
      text = str(data.get("data") or "")[:4096]
      if text:
        try:
          PTY.write_text(text)
        except Exception as exc:
          conn.send_json({"type": "error", "message": str(exc)})

  def guest_disconnected(self, conn, session: SupportSession) -> None:
    PTY.detach(conn)
    session.guest_conns.discard(conn)
    if session.controller is conn:
      session.controller = None
    self._sync_control_roles(session)
    self.broadcast_owner({"type": "guest_presence", "count": len(session.guest_conns)})

  # ---- command approval ----
  def queue_command(self, session: SupportSession, line: str, conn=None, control_action="") -> None:
    if not session.owner_conns:
      if conn is not None and not conn.closed:
        conn.send_json({"type": "owner_absent"})
      return
    if session.permission_mode == "allow_all":
      cmd = {"id": secrets.token_urlsafe(8), "line": line[:4000], "created_at": _now(), "status": "approved", "control_action": control_action}
      if conn is not None and not conn.closed:
        conn.send_json({"type": "command_approved", "id": cmd["id"]})
      self._run_command(session, cmd)
      self.broadcast_owner({"type": "command_auto_run", "id": cmd["id"], "line": cmd["line"], "created_at": cmd["created_at"]})
      return
    cmd = {"id": secrets.token_urlsafe(8), "line": line[:4000], "created_at": _now(), "status": "pending", "control_action": control_action}
    session.pending[cmd["id"]] = cmd
    self.broadcast_owner({"type": "command_request", "id": cmd["id"], "line": cmd["line"], "created_at": cmd["created_at"], "control_action": control_action})
    if conn is not None and not conn.closed:
      conn.send_json({"type": "command_waiting_approval", "id": cmd["id"]})
    threading.Timer(session.command_timeout_seconds, self._expire_command, args=(session, cmd["id"])).start()

  def _expire_command(self, session: SupportSession, cmd_id: str) -> None:
    cmd = session.pending.get(cmd_id)
    if cmd is None or cmd["status"] != "pending":
      return
    cmd["status"] = "expired"
    session.pending.pop(cmd_id, None)
    self.broadcast_all({"type": "command_expired", "id": cmd_id})

  def approve_command(self, cmd_id: str) -> dict:
    session = self.session
    if session is None:
      return {"ok": False, "error": "no active session"}
    cmd = session.pending.get(cmd_id)
    if cmd is None or cmd["status"] != "pending":
      return {"ok": False, "error": "command not pending"}
    cmd["status"] = "approved"
    session.pending.pop(cmd_id, None)
    self.broadcast_all({"type": "command_approved", "id": cmd_id})
    return self._run_command(session, cmd)

  def reject_command(self, cmd_id: str) -> dict:
    session = self.session
    if session is None:
      return {"ok": False, "error": "no active session"}
    cmd = session.pending.get(cmd_id)
    if cmd is None or cmd["status"] != "pending":
      return {"ok": False, "error": "command not pending"}
    cmd["status"] = "rejected"
    session.pending.pop(cmd_id, None)
    self.broadcast_all({"type": "command_rejected", "id": cmd_id})
    return {"ok": True}

  def _run_command(self, session: SupportSession, cmd: dict) -> dict:
    try:
      if cmd["control_action"] == "ctrl_c":
        PTY.write(b"\x03")
      elif cmd["control_action"] == "clear":
        PTY.clear_history()
        PTY.write(b"clear\r")
      else:
        PTY.write_text(cmd["line"] + "\r")
      self.broadcast_all({"type": "command_running", "id": cmd["id"]})
      return {"ok": True}
    except Exception as exc:
      self.broadcast_all({"type": "command_failed", "id": cmd["id"], "message": str(exc)})
      return {"ok": False, "error": str(exc)}


SUPPORT = SupportManager()


def _serve_support_owner_ws(handler: BaseHTTPRequestHandler) -> None:
  conn = _ws_upgrade(handler)
  if conn is None:
    return
  if not SUPPORT.owner_connected(conn):
    conn.close()
    return
  try:
    while True:
      message = conn.read_message()
      if message is None:
        continue
      try:
        data = json.loads(message)
      except Exception:
        continue
      SUPPORT.owner_message(conn, data)
  except (WsClosed, OSError, ConnectionError):
    pass
  finally:
    SUPPORT.owner_disconnected(conn)


def _serve_support_guest_ws(handler: BaseHTTPRequestHandler, session_id: str) -> None:
  conn = _ws_upgrade(handler)
  if conn is None:
    return
  session = SUPPORT.session
  if session is None or session.id != session_id or session.is_expired():
    conn.send_json({"type": "error", "message": "session unavailable"})
    conn.close()
    return
  authed = False
  try:
    while True:
      message = conn.read_message()
      if message is None:
        continue
      try:
        data = json.loads(message)
      except Exception:
        continue
      if not authed:
        if data.get("type") != "auth":
          continue
        result = SUPPORT.guest_auth(conn, session, str(data.get("pin") or "").strip())
        if result == "closed":
          return
        authed = bool(result)
        continue
      SUPPORT.guest_message(conn, session, data)
  except (WsClosed, OSError, ConnectionError):
    pass
  finally:
    SUPPORT.guest_disconnected(conn, session)


def _support_csp(host: str) -> str:
  safe = "".join(ch for ch in str(host or "") if ch.isalnum() or ch in ".-:[]")
  connect = "'self'" if not safe else f"'self' ws://{safe} wss://{safe}"
  return (
    "default-src 'none'; script-src 'self'; style-src 'self' 'unsafe-inline'; "
    f"connect-src {connect}; img-src 'self' data:; font-src 'self'; "
    "base-uri 'none'; form-action 'self'; frame-ancestors 'none'"
  )


class SupportGuestHandler(BaseHTTPRequestHandler):
  """Restricted server exposed via the public tunnel — guest page/assets/WS only."""
  server_version = "CarrotRecoveryGuest/1.0"
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt: str, *args) -> None:
    pass

  def _send(self, status: int, ctype: str, data: bytes, extra: dict | None = None) -> None:
    self.send_response(status)
    self.send_header("Content-Type", ctype)
    self.send_header("Content-Length", str(len(data)))
    self.send_header("Cache-Control", "no-store")
    self.send_header("X-Content-Type-Options", "nosniff")
    self.send_header("Referrer-Policy", "no-referrer")
    for k, v in (extra or {}).items():
      self.send_header(k, v)
    self.end_headers()
    if self.command != "HEAD":
      self.wfile.write(data)

  def do_GET(self) -> None:
    path = urlparse(self.path).path
    if path.startswith("/ws/support_terminal/"):
      if self.headers.get("Upgrade", "").lower() == "websocket":
        _serve_support_guest_ws(self, path.rsplit("/", 1)[-1])
      else:
        self.send_error(400, "expected websocket upgrade")
      return
    if path == "/" or path.startswith("/support/terminal/"):
      sid = path.rsplit("/", 1)[-1] if path.startswith("/support/terminal/") else ""
      sid = "".join(c for c in sid if c.isalnum() or c in "-_")
      try:
        with open(SUPPORT_GUEST_HTML_PATH, "r", encoding="utf-8") as f:
          page = f.read().replace("__SESSION_ID__", sid)
      except Exception:
        self.send_error(404)
        return
      self._send(200, "text/html; charset=utf-8", page.encode("utf-8"), {
        "Content-Security-Policy": _support_csp(self.headers.get("Host", "")),
        "X-Frame-Options": "DENY",
        "Permissions-Policy": "camera=(), microphone=(), geolocation=(), usb=(), serial=(), hid=(), bluetooth=()",
      })
      return
    if path.startswith("/support-terminal-assets/"):
      asset = SUPPORT_GUEST_ASSETS.get(path.rsplit("/", 1)[-1])
      if not asset or not os.path.isfile(asset):
        self.send_error(404)
        return
      ctype = STATIC_EXT_TYPES.get(Path(asset).suffix.lower(), "application/octet-stream")
      self._send(200, ctype, Path(asset).read_bytes())
      return
    self.send_error(404)

  def do_HEAD(self) -> None:
    self.do_GET()


# ===================================================================
# Recovery page — reuses the real terminal assets from selfdrive/carrot/web.
# ===================================================================
HTML_PAGE = """<!doctype html>
<html class="notranslate" translate="no">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover, interactive-widget=resizes-content">
<title>Carrot Recovery</title>
<link rel="stylesheet" href="/css/generated/design-tokens.css">
<link rel="stylesheet" href="/css/tokens.css">
<link rel="stylesheet" href="/css/layout_tokens.css">
<link rel="stylesheet" href="/css/base.css">
<link rel="stylesheet" href="/css/generated/design-system.css">
<link rel="stylesheet" href="/css/components.css">
<link rel="stylesheet" href="/css/vendor/xterm.css">
<link rel="stylesheet" href="/css/generated/terminal.css">
<style>
/* recovery has none of the app's nav chrome (side rail / bottom bar). The reused
   terminal.css reserves space for them (left: var(--nav-rail-width) on wide
   screens, a bottom gap of var(--nav-bar-height) on narrow ones), which shows up
   as empty margins here. Zero the nav sizing tokens so the terminal fills the
   viewport correctly in every layout (wide / narrow / foldable). */
:root {
  --nav-rail-width: 0px;
  --nav-bar-height: 0px;
  --nav-bar-height-desktop: 0px;
  --app-nav-bottom-gap: 0px;
}
/* recovery-only: the Git recovery menu that lives in the terminal toolbar. It
   reuses the app tokens (.smallBtn / popup surface), only the dropdown layout
   is local. */
.rc-menu-wrap { position: relative; }
.rc-menu {
  position: absolute; top: calc(100% + 6px); right: 0; z-index: var(--z-popover, 150);
  min-width: 208px; display: flex; flex-direction: column; gap: 2px; padding: 6px;
  background: var(--popup-bg); border: 1px solid var(--popup-border-color);
  border-radius: var(--popup-radius); box-shadow: var(--popup-shadow);
}
.rc-menu[hidden] { display: none; }
.rc-menu button {
  text-align: left; background: transparent; color: var(--md-on-surface); border: 0;
  border-radius: var(--control-radius); padding: 10px 12px; font: inherit; font-weight: 700;
  cursor: pointer;
}
.rc-menu button:hover { background: var(--md-surface-cont-h); }
.rc-menu button.danger { color: var(--md-danger); }
.rc-menu button.danger:hover { background: var(--md-danger-cont); }
.rc-cwp {
  display: flex; align-items: center; justify-content: space-between; gap: 12px;
  padding: 10px 12px; color: var(--md-on-surface); border-radius: var(--control-radius);
  font-weight: 700; cursor: pointer;
}
.rc-cwp:hover { background: var(--md-surface-cont-h); }
.rc-cwp:has(input:disabled) { opacity: .55; cursor: default; }
.rc-cwp:has(input:disabled):hover { background: transparent; }
.rc-cwp__control { display: inline-flex; align-items: center; gap: 8px; }
.rc-cwp__state { color: var(--md-on-surface-variant); font-size: 10px; font-weight: 600; }
.rc-cwp input { width: 18px; height: 18px; accent-color: var(--md-primary); }
.rc-brand { color: var(--md-primary); font-weight: 800; }
</style>
</head>
<body data-page="terminal">
  <div id="pageTerminal" class="page page--full page--terminal">
    <div class="terminal-shell">
      <div class="terminal-head">
        <div class="muted terminal-meta" id="terminalMeta">connecting...</div>
        <div class="terminal-toolbar">
          <div id="terminalSessionMeta" class="terminal-sessionMeta"><span class="rc-brand">carrot recovery</span></div>
          <div class="terminal-toolbar__actions">
            <div class="rc-menu-wrap">
              <button id="rcGitBtn" class="smallBtn" type="button" aria-haspopup="true" aria-expanded="false">Git &#x25BE;</button>
              <div id="rcGitMenu" class="rc-menu" hidden>
                <button data-act="git_pull">git pull</button>
                <button data-act="git_sync">git sync</button>
                <button data-act="git_reset">git reset</button>
                <button data-act="git_log">git log</button>
                <button data-act="git_branches">change branch</button>
                <button data-act="git_reset_repo">reset repo</button>
              </div>
            </div>
            <div class="rc-menu-wrap">
              <button id="rcToolsBtn" class="smallBtn" type="button" aria-haspopup="true" aria-expanded="false">Tools &#x25BE;</button>
              <div id="rcToolsMenu" class="rc-menu" hidden>
                <label class="rc-cwp" for="rcCwpPush">
                  <span>CWP Push</span>
                  <span class="rc-cwp__control"><span id="rcCwpState" class="rc-cwp__state" hidden></span><input id="rcCwpPush" type="checkbox" disabled></span>
                </label>
                <button data-tool="send_tmux_log">download tmux log</button>
                <button data-tool="server_tmux_log">send tmux log</button>
                <button data-tool="rebuild_all" class="danger">rebuild</button>
                <button data-act="git_reboot" class="danger">reboot</button>
              </div>
            </div>
            <button id="btnSupportTerminalOpen" class="smallBtn terminal-support-open" type="button">Support</button>
            <button id="btnTerminalReconnect" class="smallBtn" type="button">Reconnect</button>
          </div>
        </div>
      </div>
      <div id="terminalScreen" class="terminal-screen"><pre id="terminalOutput" class="terminal-output"> </pre></div>
      <div id="terminalXterm" class="terminal-xterm" hidden></div>
      <div id="supportTerminalTypingHost" class="terminal-typing-host" aria-hidden="true"></div>
      <div id="supportTerminalApprovalHost" class="terminal-approval-host" hidden></div>
      <div id="terminalKeys" class="terminal-keys" role="group" aria-label="Terminal keys">
        <button class="smallBtn terminal-key" type="button" data-key="esc">Esc</button>
        <button class="smallBtn terminal-key" type="button" data-key="ctrl">Ctrl</button>
        <button class="smallBtn terminal-key" type="button" data-key="ctrl_c">Ctrl+C</button>
        <button class="smallBtn terminal-key" type="button" data-key="ctrl_d">Ctrl+D</button>
        <button id="btnTerminalClear" class="smallBtn terminal-key" type="button">Clear</button>
        <button class="smallBtn terminal-key" type="button" data-key="detach">Detach</button>
        <button class="smallBtn terminal-key" type="button" data-key="tab">Tab</button>
        <button class="smallBtn terminal-key" type="button" data-key="home">Home</button>
        <button class="smallBtn terminal-key" type="button" data-key="end">End</button>
        <button class="smallBtn terminal-key" type="button" data-key="page_up">PgUp</button>
        <button class="smallBtn terminal-key" type="button" data-key="page_down">PgDn</button>
        <button class="smallBtn terminal-key" type="button" data-key="up" aria-label="Up">&#x2191;</button>
        <button class="smallBtn terminal-key" type="button" data-key="down" aria-label="Down">&#x2193;</button>
        <button class="smallBtn terminal-key" type="button" data-key="left" aria-label="Left">&#x2190;</button>
        <button class="smallBtn terminal-key" type="button" data-key="right" aria-label="Right">&#x2192;</button>
      </div>
    </div>
  </div>

  <div id="appToastHost" class="app-toast-host" aria-live="polite" aria-atomic="true"></div>

  <div id="appDialog" class="app-dialog" hidden>
    <button id="appDialogBackdrop" class="app-dialog__backdrop" type="button" aria-label="Close dialog"></button>
    <div class="app-dialog__sheet" role="dialog" aria-modal="true" aria-labelledby="appDialogTitle" aria-describedby="appDialogBody">
      <div class="app-dialog__head">
        <div id="appDialogTitle" class="app-dialog__title">Notice</div>
      </div>
      <div id="appDialogBody" class="app-dialog__body">-</div>
      <div id="appDialogChoices" class="app-dialog__choices" hidden></div>
      <div id="appDialogInputWrap" class="app-dialog__inputWrap" hidden>
        <input id="appDialogInput" type="text" class="input-text app-dialog__input" />
        <div id="appDialogInputError" class="app-dialog__inputError" role="alert" hidden></div>
      </div>
      <div class="app-dialog__actions">
        <button id="appDialogCopy" class="smallBtn" type="button" hidden>Copy</button>
        <button id="appDialogDefault" class="smallBtn" type="button" hidden>Default</button>
        <button id="appDialogCancel" class="smallBtn" type="button">Cancel</button>
        <button id="appDialogConfirm" class="smallBtn btn--filled" type="button">OK</button>
      </div>
    </div>
  </div>

  <script src="/js/translations/registry.js"></script>
  <script src="/js/translations/ko.js"></script>
  <script src="/js/translations/en.js"></script>
  <script src="/js/translations/zh.js"></script>
  <script src="/js/shared/constants.js"></script>
  <script src="/js/shared/dom.js"></script>
  <script src="/js/shared/utils.js"></script>
  <script src="/js/shared/i18n.js"></script>
  <script src="/js/generated/app.js"></script>
  <script src="/js/shared/ui/viewport.js"></script>
  <script src="/js/vendor/xterm-addon-shim.js"></script>
  <script src="/js/vendor/xterm.js"></script>
  <script src="/js/vendor/xterm-addon-webgl.js"></script>
  <script src="/js/vendor/xterm-addon-canvas.js"></script>
  <script src="/js/generated/terminal.js"></script>
  <script src="/js/generated/terminal-shared.js"></script>
  <script src="/recovery-api.js"></script>
  <script src="/js/generated/terminal-support.js"></script>
  <script src="/recovery.js"></script>
</body>
</html>
"""


# Recovery-owned transport for shared UI modules. The main app normally
# provides getJson/postJson from shared/api.js; recovery intentionally does not
# load that app-wide API surface. Keep requests same-origin so they always
# resolve to this standalone server (port 6999), never the main Carrot service.
RECOVERY_API_JS = """\"use strict\";
(function () {
  async function requestJson(url, options) {
    var response = await fetch(url, options || {});
    var text = await response.text();
    var payload = {};

    if (text) {
      try {
        payload = JSON.parse(text);
      } catch (err) {
        throw new Error(text.slice(0, 500) || ("HTTP " + response.status));
      }
    }

    if (!response.ok || (payload && payload.ok === false)) {
      var error = new Error((payload && (payload.error || payload.out)) || ("HTTP " + response.status));
      error.status = response.status;
      error.payload = payload || null;
      throw error;
    }
    return payload;
  }

  function getJson(url) {
    return requestJson(url, { cache: "no-store" });
  }

  function postJson(url, body) {
    return requestJson(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
  }

  var api = Object.freeze({ requestJson: requestJson, getJson: getJson, postJson: postJson });
  window.CarrotRecoveryApi = api;

  // Compatibility contract for the shared support_terminal.js component.
  // These globals exist only in the recovery document and are backed by the
  // recovery-owned, same-origin transport above.
  window.getJson = getJson;
  window.postJson = postJson;
})();
"""


# recovery-only glue: wires the Git recovery menu to the reused app dialogs and
# runs recovered commands in the shared PTY, then boots the reused terminal.
RECOVERY_JS = """\"use strict\";
(function () {
  var menuWraps = Array.prototype.slice.call(document.querySelectorAll(\".rc-menu-wrap\"));
  function closeMenus() {
    menuWraps.forEach(function (wrap) {
      var m = wrap.querySelector(\".rc-menu\");
      var b = wrap.querySelector(\"button[aria-haspopup]\");
      if (m) m.hidden = true;
      if (b) b.setAttribute(\"aria-expanded\", \"false\");
    });
  }
  menuWraps.forEach(function (wrap) {
    var b = wrap.querySelector(\"button[aria-haspopup]\");
    var m = wrap.querySelector(\".rc-menu\");
    if (!b || !m) return;
    b.addEventListener(\"click\", function (e) {
      e.stopPropagation();
      var willOpen = m.hidden;
      closeMenus();
      m.hidden = !willOpen;
      b.setAttribute(\"aria-expanded\", String(willOpen));
    });
  });
  document.addEventListener(\"click\", function (e) {
    if (!e.target.closest(\".rc-menu-wrap\")) closeMenus();
  });

  function toast(msg, tone) {
    if (typeof showAppToast === \"function\") showAppToast(msg, tone ? { tone: tone } : {});
  }

  async function callAction(action, payload) {
    var body = Object.assign({ action: action }, payload || {});
    var r = await fetch(\"/api/action\", {
      method: \"POST\", headers: { \"Content-Type\": \"application/json\" }, body: JSON.stringify(body),
    });
    return r.json();
  }
  var cwpPush = document.getElementById(\"rcCwpPush\");
  var cwpState = document.getElementById(\"rcCwpState\");
  function showCwp(data) {
    if (!cwpPush || !cwpState) return;
    cwpPush.checked = !!(data && data.enabled);
    cwpPush.disabled = !(data && data.ok && data.registered);
    cwpState.hidden = !!(data && data.registered);
    cwpState.textContent = data && data.state === \"unregistered\" ? \"Not registered\" : \"Unavailable\";
  }
  async function refreshCwp() {
    try {
      showCwp(await CarrotRecoveryApi.getJson(\"/api/recovery/cwp\"));
    } catch (err) {
      showCwp((err && err.payload) || {});
    }
  }
  if (cwpPush) cwpPush.addEventListener(\"change\", async function (event) {
    event.stopPropagation();
    var enabled = cwpPush.checked;
    cwpPush.disabled = true;
    try {
      showCwp(await CarrotRecoveryApi.postJson(\"/api/recovery/cwp\", { enabled: enabled }));
      toast(\"CWP Push \" + (enabled ? \"on\" : \"off\"), \"success\");
    } catch (err) {
      toast((err && err.message) || \"CWP failed\", \"error\");
      await refreshCwp();
    }
  });
  async function runInTerminal(command) {
    await fetch(\"/api/terminal/input\", {
      method: \"POST\", headers: { \"Content-Type\": \"application/json\" }, body: JSON.stringify({ data: command }),
    });
  }
  async function dispatchGit(action, payload) {
    var data = await callAction(action, payload);
    if (data && data.rebooting) return data;
    if (data && data.ok === false) { toast(data.error || \"failed\", \"error\"); return data; }
    if (data && data.command) await runInTerminal(data.command);
    return data;
  }

  var CONFIRMS = {
    git_pull:    { title: \"git pull\",  message: \"Pull latest commits from remote.\\nLocal changes will be reset first.\", confirmLabel: \"Pull\" },
    git_sync:    { title: \"git sync\",  message: \"Delete all local branches except the current one, then fetch from remote.\", confirmLabel: \"Sync\" },
    git_reset:   { title: \"git reset\", message: \"Reset the current branch to the last commit.\\nAll local changes will be lost.\", confirmLabel: \"Reset\" },
    git_rebuild: { title: \"rebuild\",   message: \"Clean all build cache. Will rebuild on next boot.\", confirmLabel: \"Clean\" },
  };

  async function pickBranch(title, message, selected, confirmLabel) {
    var data = await callAction(\"git_branches\");
    if (!data || !data.ok) { toast((data && data.error) || \"failed to list branches\", \"error\"); return null; }
    var want = selected || data.current;
    var choices = data.branches.map(function (b) {
      return { label: (b.kind === \"remote\" ? \"\\u2197 \" : \"   \") + b.name, value: b.name, current: b.name === want };
    });
    return openAppDialog({ mode: \"choice\", title: title, message: message, choices: choices, confirmLabel: confirmLabel });
  }

  async function onGit(action) {
    closeMenus();
    if (CONFIRMS[action]) {
      var c = CONFIRMS[action];
      if (await appConfirm(c.message, { title: c.title, confirmLabel: c.confirmLabel })) await dispatchGit(action);
      return;
    }
    if (action === \"git_log\") {
      var log = await callAction(\"git_log\");
      if (!log || !log.ok) { toast((log && log.error) || \"failed\", \"error\"); return; }
      var commits = log.commits || [];
      if (!commits.length) { appAlert(\"(no commits)\", { title: \"git log\" }); return; }
      var logChoices = commits.map(function (c) {
        return { label: c.hash + \"  \" + (c.message || \"\"), value: c.hash, current: !!(log.current && c.hash.indexOf(log.current) === 0) };
      });
      var sel = await openAppDialog({ mode: \"choice\", title: \"git log\", message: \"Select a commit to checkout.\", choices: logChoices, confirmLabel: \"Checkout\" });
      if (!sel) return;
      if (log.current && sel.indexOf(log.current) === 0) return;
      if (await appConfirm(\"Move to the selected commit.\\n\\n\" + sel, { title: \"Checkout commit\", confirmLabel: \"Checkout\" })) {
        await dispatchGit(\"git_checkout_commit\", { commit: sel });
      }
      return;
    }
    if (action === \"git_branches\") {
      var branch = await pickBranch(\"Change branch\", \"Select a branch to switch to.\", null, \"Switch\");
      if (branch) await dispatchGit(\"git_checkout\", { branch: branch });
      return;
    }
    if (action === \"git_reset_repo\") {
      var target = await pickBranch(\"Reset repo\", \"Fetch the selected branch fresh.\\nAll local changes and untracked files will be lost.\", \"c3-wip\", \"Next\");
      if (!target) return;
      if (await appConfirm(\"Branch: \" + target + \"\\nRemote: ajouatom/openpilot.git\\n\\nThis cannot be undone.\", { title: \"Confirm reset\", confirmLabel: \"Reset\" })) {
        await dispatchGit(\"git_reset_repo\", { branch: target });
      }
      return;
    }
    if (action === \"git_reboot\") {
      if (!(await appConfirm(\"The device will restart immediately.\", { title: \"Reboot\", confirmLabel: \"Next\" }))) return;
      if (!(await appConfirm(\"Really reboot the device?\", { title: \"Confirm reboot\", confirmLabel: \"Reboot\" }))) return;
      await dispatchGit(\"git_reboot\");
      return;
    }
  }

  async function onTool(action) {
    closeMenus();
    if (action === \"rebuild_all\") {
      if (await appConfirm(\"Clean the build cache and reboot.\\n\\n\\u2022 scons -c\\n\\u2022 rm -rf prebuilt\\n\\u2022 sudo reboot\", { title: \"rebuild\", confirmLabel: \"Rebuild\" })) {
        await dispatchGit(\"rebuild_all\");
      }
      return;
    }
    if (action === \"send_tmux_log\") {
      var r = await callAction(\"send_tmux_log\");
      if (r && r.ok) { toast(\"tmux log captured\", \"success\"); window.open(r.file || \"/download/tmux.log\", \"_blank\"); }
      else toast((r && r.error) || \"capture failed\", \"error\");
      return;
    }
    if (action === \"server_tmux_log\") {
      var r2 = await callAction(\"server_tmux_log\");
      if (r2 && r2.ok) toast(r2.partial ? \"tmux log sent to server (partial)\" : \"tmux log sent to server\", \"success\");
      else toast((r2 && r2.error) || \"failed\", \"error\");
      return;
    }
  }

  document.querySelectorAll(\"[data-act]\").forEach(function (btn) {
    btn.addEventListener(\"click\", function (e) { e.stopPropagation(); onGit(btn.dataset.act); });
  });
  document.querySelectorAll(\"[data-tool]\").forEach(function (btn) {
    btn.addEventListener(\"click\", function (e) { e.stopPropagation(); onTool(btn.dataset.tool); });
  });
  refreshCwp();

  // Boot the reused generated terminal runtime.
  function boot() {
    if (typeof initTerminalPage === \"function\") initTerminalPage();
    else setTimeout(boot, 30);
  }
  if (document.readyState === \"loading\") document.addEventListener(\"DOMContentLoaded\", boot);
  else boot();
})();
"""


# ===================================================================
# HTTP handler
# ===================================================================
def _resolve_static(path: str) -> Path | None:
  rel = path.lstrip("/")
  if not any(rel.startswith(root) for root in STATIC_ROOTS):
    return None
  target = (WEB_DIR / rel).resolve()
  try:
    target.relative_to(WEB_DIR.resolve())
  except ValueError:
    return None  # path traversal
  if not target.is_file():
    return None
  if target.suffix.lower() not in STATIC_EXT_TYPES:
    return None
  return target


class RecoveryHandler(BaseHTTPRequestHandler):
  server_version = "CarrotRecovery/2.0"
  protocol_version = "HTTP/1.1"

  def log_message(self, fmt: str, *args) -> None:
    print("[recovery]", self.address_string(), fmt % args)

  # ---- helpers ---------------------------------------------------
  def _send_bytes(self, status: int, content_type: str, data: bytes, cache: bool = False) -> None:
    self.send_response(status)
    self.send_header("Content-Type", content_type)
    self.send_header("Content-Length", str(len(data)))
    self.send_header("Cache-Control", "public, max-age=300" if cache else "no-store")
    self.end_headers()
    if self.command != "HEAD":
      self.wfile.write(data)

  def _send_json(self, status: int, payload: dict) -> None:
    self._send_bytes(status, "application/json; charset=utf-8",
                     json.dumps(payload, ensure_ascii=False).encode("utf-8"))

  def _read_json(self) -> dict:
    length = int(self.headers.get("Content-Length", "0") or "0")
    if length > 65536:
      raise ValueError("request too large")
    body = self.rfile.read(length).decode("utf-8") if length else ""
    return json.loads(body or "{}")

  # ---- routes ----------------------------------------------------
  def do_GET(self) -> None:
    parsed = urlparse(self.path)
    path = parsed.path
    query = dict(p.split("=", 1) if "=" in p else (p, "") for p in parsed.query.split("&") if p)

    if path == "/ws/terminal_pty":
      if (self.headers.get("Upgrade", "").lower() == "websocket"):
        _serve_terminal_ws(self, query)
      else:
        self.send_error(400, "expected websocket upgrade")
      return

    if path == "/ws/support_terminal/owner":
      if self.headers.get("Upgrade", "").lower() == "websocket":
        _serve_support_owner_ws(self)
      else:
        self.send_error(400, "expected websocket upgrade")
      return

    if path == "/api/support_terminal/status":
      self._send_json(200, SUPPORT.snapshot())
      return

    if path == "/api/recovery/cwp":
      status = _cwp_status()
      self._send_json(200 if status.get("ok") else 503, status)
      return

    if path in ("/", "/index.html"):
      self._send_bytes(200, "text/html; charset=utf-8", HTML_PAGE.encode("utf-8"))
      return

    if path == "/recovery.js":
      self._send_bytes(200, "text/javascript; charset=utf-8", RECOVERY_JS.encode("utf-8"))
      return

    if path == "/recovery-api.js":
      self._send_bytes(200, "text/javascript; charset=utf-8", RECOVERY_API_JS.encode("utf-8"))
      return

    if path == "/download/tmux.log":
      if not os.path.isfile(TMUX_LOG_PATH):
        self._send_json(404, {"ok": False, "error": "file not found"})
        return
      data = Path(TMUX_LOG_PATH).read_bytes()
      self.send_response(200)
      self.send_header("Content-Type", "text/plain; charset=utf-8")
      self.send_header("Content-Length", str(len(data)))
      self.send_header("Content-Disposition", "attachment; filename=tmux.log")
      self.send_header("Cache-Control", "no-store")
      self.end_headers()
      if self.command != "HEAD":
        self.wfile.write(data)
      return

    static = _resolve_static(path)
    if static is not None:
      ctype = STATIC_EXT_TYPES.get(static.suffix.lower()) or (mimetypes.guess_type(static.name)[0] or "application/octet-stream")
      self._send_bytes(200, ctype, static.read_bytes(), cache=True)
      return

    self._send_json(404, {"ok": False, "error": "not found"})

  def do_HEAD(self) -> None:
    self.do_GET()

  def do_POST(self) -> None:
    path = urlparse(self.path).path
    try:
      payload = self._read_json()
    except Exception as exc:
      self._send_json(400, {"ok": False, "error": str(exc)})
      return

    if path == "/api/action":
      action = str(payload.get("action") or "").strip()
      handler = _tool_action if action in TOOL_ACTIONS else _git_action
      self._send_json(200, handler(action, payload))
      return

    if path == "/api/recovery/cwp":
      enabled = payload.get("enabled")
      if not isinstance(enabled, bool):
        self._send_json(400, {"ok": False, "error": "enabled must be boolean"})
        return
      result = _cwp_set_enabled(enabled)
      self._send_json(200 if result.get("ok") else 409, result)
      return

    if path == "/api/terminal/input":
      # Used by the git panel to run a recovered command in the shared PTY.
      try:
        PTY.write_text(str(payload.get("data") or "") + "\r")
        self._send_json(200, {"ok": True})
      except Exception as exc:
        self._send_json(200, {"ok": False, "error": str(exc)})
      return

    if path == "/api/support_terminal/start":
      self._send_json(200, SUPPORT.start(
        note=payload.get("note") or "",
        ttl_seconds=payload.get("ttl_seconds"),
        permission_mode=payload.get("permission_mode"),
        command_timeout_seconds=payload.get("command_timeout_seconds"),
      ))
      return

    if path == "/api/support_terminal/stop":
      self._send_json(200, SUPPORT.stop("user"))
      return

    support_cmd = re.match(r"^/api/support_terminal/commands/([^/]+)/(approve|reject)$", path)
    if support_cmd:
      cmd_id, act = support_cmd.group(1), support_cmd.group(2)
      result = SUPPORT.approve_command(cmd_id) if act == "approve" else SUPPORT.reject_command(cmd_id)
      self._send_json(200 if result.get("ok") else 400, result)
      return

    self._send_json(404, {"ok": False, "error": "not found"})


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--host", default=DEFAULT_HOST)
  parser.add_argument("--port", type=int, default=DEFAULT_PORT)
  args = parser.parse_args()

  httpd = ThreadingHTTPServer((args.host, args.port), RecoveryHandler)
  httpd.daemon_threads = True
  print(f"[recovery] serving http://{args.host}:{args.port} cwd={REPO_ROOT} web={WEB_DIR}")
  threading.Thread(target=_cwp_boot_worker, args=(args.port,), daemon=True).start()
  try:
    httpd.serve_forever()
  except KeyboardInterrupt:
    pass
  finally:
    httpd.server_close()


if __name__ == "__main__":
  main()
