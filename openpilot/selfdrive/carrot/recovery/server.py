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
import json
import mimetypes
import os
import selectors
import shlex
import shutil
import signal
import struct
import subprocess
import threading
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


def _put_param(key: str, value: bytes) -> None:
  """Write an openpilot param the same way the C++ Params does: stage in
  /data/params/d_tmp then atomic-rename into /data/params/d. Lets recovery
  trigger `server_tmux_log` (CarrotException=tmux_send) without importing
  openpilot; a running carrot_man consumes it (no-op if openpilot is down)."""
  d = os.path.join(PARAMS_DIR, "d")
  d_tmp = os.path.join(PARAMS_DIR, "d_tmp")
  os.makedirs(d, exist_ok=True)
  os.makedirs(d_tmp, exist_ok=True)
  tmp = os.path.join(d_tmp, key)
  with open(tmp, "wb") as f:
    f.write(value)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, os.path.join(d, key))


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
    try:
      _put_param("CarrotException", b"tmux_send")
      return {"ok": True}
    except Exception as exc:
      return {"ok": False, "error": str(exc)}
  return {"ok": False, "error": f"unknown action: {action}"}


# ===================================================================
# Recovery page — reuses the real terminal assets from selfdrive/carrot/web.
# ===================================================================
HTML_PAGE = """<!doctype html>
<html class="notranslate" translate="no">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover, interactive-widget=resizes-content">
<title>Carrot Recovery</title>
<link rel="stylesheet" href="/css/tokens.css">
<link rel="stylesheet" href="/css/layout_tokens.css">
<link rel="stylesheet" href="/css/base.css">
<link rel="stylesheet" href="/css/components.css">
<link rel="stylesheet" href="/css/vendor/xterm.css">
<link rel="stylesheet" href="/css/pages/terminal.css">
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
                <button data-tool="send_tmux_log">send tmux log</button>
                <button data-tool="server_tmux_log">server tmux log</button>
                <button data-tool="rebuild_all" class="danger">rebuild</button>
                <button data-act="git_reboot" class="danger">reboot</button>
              </div>
            </div>
            <button id="btnTerminalReconnect" class="smallBtn" type="button">Reconnect</button>
          </div>
        </div>
      </div>
      <div id="terminalScreen" class="terminal-screen"><pre id="terminalOutput" class="terminal-output"> </pre></div>
      <div id="terminalXterm" class="terminal-xterm" hidden></div>
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
  <script src="/js/shared/ui/dialog.js"></script>
  <script src="/js/shared/ui/viewport.js"></script>
  <script src="/js/vendor/xterm-addon-shim.js"></script>
  <script src="/js/vendor/xterm.js"></script>
  <script src="/js/vendor/xterm-addon-webgl.js"></script>
  <script src="/js/vendor/xterm-addon-canvas.js"></script>
  <script src="/js/pages/terminal.js"></script>
  <script src="/recovery.js"></script>
</body>
</html>
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
      if (r2 && r2.ok) toast(\"tmux log send triggered\", \"success\");
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

  // Boot the reused terminal (pages/terminal.js defines initTerminalPage).
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

    if path in ("/", "/index.html"):
      self._send_bytes(200, "text/html; charset=utf-8", HTML_PAGE.encode("utf-8"))
      return

    if path == "/recovery.js":
      self._send_bytes(200, "text/javascript; charset=utf-8", RECOVERY_JS.encode("utf-8"))
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

    if path == "/api/terminal/input":
      # Used by the git panel to run a recovered command in the shared PTY.
      try:
        PTY.write_text(str(payload.get("data") or "") + "\r")
        self._send_json(200, {"ok": True})
      except Exception as exc:
        self._send_json(200, {"ok": False, "error": str(exc)})
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
  try:
    httpd.serve_forever()
  except KeyboardInterrupt:
    pass
  finally:
    httpd.server_close()


if __name__ == "__main__":
  main()
