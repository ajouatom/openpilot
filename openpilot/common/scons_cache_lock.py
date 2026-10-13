"""Process-lifetime exclusion for users and cleaners of a shared SCons cache."""
import fcntl
import os
from pathlib import Path
import stat
import sys
import time


class CacheBusyError(RuntimeError):
  pass


def guard_path(cache_dir: Path) -> Path:
  # A sibling of the cache, so deleting the cache never splits the lock.
  return cache_dir.with_name(cache_dir.name + '.build.lock')


def _open_guard(lock_path: Path) -> int:
  # Read-only: flock needs no write access, so a guard created by another user
  # (e.g. by `sudo scons`) still works for everyone. It is never unlinked.
  try:
    return os.open(lock_path, os.O_RDONLY | os.O_NOFOLLOW)
  except FileNotFoundError:
    os.close(os.open(lock_path, os.O_CREAT | os.O_WRONLY | os.O_NOFOLLOW, 0o666))
    return os.open(lock_path, os.O_RDONLY | os.O_NOFOLLOW)


def acquire_cache_lock(cache_dir: Path, inherited_fd: str | None = None, timeout: float = 0.0):
  """Builders pass a timeout and wait for each other; cleaners never wait."""
  lock_path = guard_path(cache_dir)
  if inherited_fd is not None:
    fd = os.dup(int(inherited_fd))
    try:
      inherited = os.fstat(fd)
      expected = os.stat(lock_path, follow_symlinks=False)
      if not stat.S_ISREG(expected.st_mode) or (inherited.st_dev, inherited.st_ino) != (expected.st_dev, expected.st_ino):
        raise OSError('Inherited cache guard does not match the cache path')
    except BaseException:
      os.close(fd)
      raise
  else:
    fd = _open_guard(lock_path)
  handle = os.fdopen(fd, 'r')
  try:
    if not stat.S_ISREG(os.fstat(handle.fileno()).st_mode):
      raise OSError(f'Cache guard is not a regular file: {lock_path}')
    deadline = time.monotonic() + timeout
    reported_wait = False
    while True:
      try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return handle
      except BlockingIOError as exc:
        if time.monotonic() >= deadline:
          raise CacheBusyError(f'SCons cache is in use: {cache_dir}') from exc
        if not reported_wait:
          print(f"Waiting for another SCons build to release {cache_dir} (up to {timeout:g}s)", file=sys.stderr, flush=True)
          reported_wait = True
        time.sleep(0.2)
  except BaseException:
    handle.close()
    raise
