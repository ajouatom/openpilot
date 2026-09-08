"""Cooperative device checkout lock and conservative orphaned-index recovery.

The launcher, web tools and updater share this lock. Never unlink the lock file:
flock ownership, unlike a sentinel file, is released when its owner exits.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
from contextvars import ContextVar
import os
from pathlib import Path
import stat
import subprocess
import time


LOCK_PATH = os.environ.get("CARROT_REPO_LOCK_PATH", "/tmp/carrot_repo_update.lock")
STALE_INDEX_SECONDS = 60.0
_lock_fd: ContextVar[int | None] = ContextVar("carrot_repo_lock_fd", default=None)


class RepoBusyError(RuntimeError):
  pass


def _flock(fd: int, *, unlock: bool = False) -> None:
  if os.name == "nt":
    import msvcrt
    os.lseek(fd, 0, os.SEEK_SET)
    msvcrt.locking(fd, msvcrt.LK_UNLCK if unlock else msvcrt.LK_NBLCK, 1)
  else:
    import fcntl
    fcntl.flock(fd, fcntl.LOCK_UN if unlock else fcntl.LOCK_EX | fcntl.LOCK_NB)


def child_lock_kwargs() -> dict:
  fd = _lock_fd.get()
  return {"pass_fds": (fd,)} if fd is not None and os.name != "nt" else {}


@contextmanager
def repo_lock():
  # Do not make this reentrant: separate asyncio tasks must also exclude each
  # other. Callers take one lock around the complete multi-command transaction.
  fd = os.open(LOCK_PATH, os.O_CREAT | os.O_RDWR, 0o600)
  try:
    try:
      _flock(fd)
    except OSError as exc:
      raise RepoBusyError("Build or another Git operation is running; retry when it finishes.") from exc
    token = _lock_fd.set(fd)
    try:
      yield
    finally:
      _lock_fd.reset(token)
      # close, rather than LOCK_UN, keeps children protected if the caller dies.
  finally:
    os.close(fd)


def release_boot_lock() -> None:
  """Manager releases the launcher's inherited lock only after initialization."""
  value = os.environ.pop("CARROT_BOOT_LOCK_FD", "")
  if not value:
    return
  fd = int(value)
  if not os.path.samestat(os.fstat(fd), os.stat(LOCK_PATH)):
    raise RuntimeError("Unexpected boot repository lock descriptor")
  _flock(fd, unlock=True)
  os.close(fd)


def git_process_running() -> bool:
  """Fail closed if we cannot inspect processes. Any Git process defers repair."""
  proc_root = Path("/proc")
  if not proc_root.is_dir():
    return True
  try:
    for entry in proc_root.iterdir():
      if not entry.name.isdigit():
        continue
      try:
        name = (entry / "comm").read_text().strip()
      except FileNotFoundError:
        continue  # process exited while scanning
      if name == "git" or name.startswith("git-"):
        return True
    return False
  except OSError:
    return True


def recover_stale_index_lock(repo_dir: str) -> bool:
  """Called under repo_lock, before starting any child Git command.

  Only index.lock is recoverable here. Ref/config locks need explicit diagnosis.
  Age alone is never evidence that an index lock is abandoned.
  """
  result = subprocess.run(
    ["git", "rev-parse", "--path-format=absolute", "--git-path", "index.lock"], cwd=repo_dir,
    capture_output=True, text=True, timeout=10, **child_lock_kwargs(),
  )
  if result.returncode:
    raise RuntimeError(result.stderr.strip() or "Unable to locate Git index lock")
  path = Path(repo_dir) / result.stdout.strip()
  try:
    before = path.lstat()
  except FileNotFoundError:
    return False
  if not stat.S_ISREG(before.st_mode):
    raise RepoBusyError("Git index lock is not a regular file; manual inspection required.")
  if time.time() - before.st_mtime < STALE_INDEX_SECONDS or git_process_running():  # noqa: TID251 -- filesystem mtime is wall-clock time
    raise RepoBusyError("Git index is in use; waiting before retrying.")
  time.sleep(0.1)
  try:
    after = path.lstat()
  except FileNotFoundError:
    return False
  if (before.st_dev, before.st_ino, before.st_mtime_ns, before.st_size) != (
    after.st_dev, after.st_ino, after.st_mtime_ns, after.st_size
  ) or git_process_running():
    raise RepoBusyError("Git index lock changed or Git is running; retry later.")
  path.unlink()
  print("[repo_update] removed abandoned index.lock after checking Git processes", flush=True)
  return True


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--repo", default="/data/openpilot")
  parser.add_argument("command", nargs=argparse.REMAINDER)
  args = parser.parse_args()
  try:
    with repo_lock():
      recover_stale_index_lock(args.repo)
      if args.command:
        return subprocess.call(args.command, cwd=args.repo, **child_lock_kwargs())
      return 0
  except RepoBusyError as exc:
    print(str(exc), flush=True)
    return 75


if __name__ == "__main__":
  raise SystemExit(main())
