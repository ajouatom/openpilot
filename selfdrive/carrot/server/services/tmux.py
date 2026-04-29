import getpass
import os
import shlex
import shutil
import subprocess
import time
from typing import List

from ..config import TMUX_CAPTURE_LINES, TMUX_START_DIR, TMUX_WEB_SESSION


def run(args: List[str], timeout: float = 5.0, check: bool = False) -> subprocess.CompletedProcess:
  return subprocess.run(
    args,
    capture_output=True,
    text=True,
    timeout=timeout,
    check=check,
  )


def has_session(session: str) -> bool:
  p = run(["tmux", "has-session", "-t", session], timeout=2.5)
  return p.returncode == 0


def send_keys(session: str, *keys: str, literal: bool = False) -> None:
  cmd = ["tmux", "send-keys", "-t", session]
  if literal:
    cmd.append("-l")
  cmd.extend(keys)
  run(cmd, timeout=4.0, check=True)


def bootstrap_shell(cwd: str = TMUX_START_DIR) -> str:
  return f"cd {shlex.quote(cwd)} && exec bash -il"


def start_command() -> str:
  if os.name != "posix":
    return "powershell"

  current_user = (
    os.environ.get("USER")
    or os.environ.get("USERNAME")
    or getpass.getuser()
  )
  geteuid = getattr(os, "geteuid", None)
  euid = geteuid() if callable(geteuid) else None
  bootstrap = bootstrap_shell()

  if current_user == "comma":
    return bootstrap

  if euid == 0:
    return f"exec su - comma -c {shlex.quote(bootstrap)}"

  if shutil.which("sudo"):
    try:
      probe = subprocess.run(
        ["sudo", "-n", "-u", "comma", "true"],
        capture_output=True,
        text=True,
        timeout=2,
      )
      if probe.returncode == 0:
        return f"exec sudo -n -u comma -i bash -lc {shlex.quote(bootstrap)}"
    except Exception:
      pass

  return bootstrap


def ensure_session(session: str = TMUX_WEB_SESSION) -> bool:
  created = False
  if not has_session(session):
    run(
      ["tmux", "new-session", "-d", "-s", session, start_command()],
      timeout=5.0,
      check=True,
    )
    created = True
  return created


def capture(session: str = TMUX_WEB_SESSION, lines: int = TMUX_CAPTURE_LINES) -> str:
  p = run(
    ["tmux", "capture-pane", "-p", "-J", "-t", session, "-S", f"-{max(lines, 40)}"],
    timeout=4.0,
    check=False,
  )
  if p.returncode != 0:
    return ""
  return (p.stdout or "").rstrip() or " "


def send_line(session: str, line: str) -> None:
  if line:
    send_keys(session, line, literal=True)
  send_keys(session, "Enter")


def ctrl_c(session: str) -> None:
  send_keys(session, "C-c")


def clear(session: str) -> None:
  send_line(session, "clear")
  time.sleep(0.04)
  run(["tmux", "clear-history", "-t", session], timeout=4.0, check=False)
