"""One USB panel owner; the regular HUD takes priority over boot diagnostics."""
from contextlib import contextmanager
import os
import json
from pathlib import Path
import time

# Runtime updates also work on legacy images without a new systemd drop-in.
DIRECTORY = Path('/dev/shm') / ('carrot-jetlink-display-' + str(getattr(os, 'getuid', lambda: 0)()))


def process_identity(pid):
  # A reused PID from a dead HUD must not suppress diagnostics indefinitely.
  value = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
  return [Path('/proc/sys/kernel/random/boot_id').read_text().strip(), value[19]]


def hud_requested(directory=DIRECTORY):
  try:
    request = json.loads((directory / 'hud-request').read_text())
    pid = int(request['pid'])
    if pid <= 1:
      return False
    return request['identity'] == process_identity(pid)
  except (OSError, ValueError, KeyError, TypeError, IndexError):
    return False


@contextmanager
def panel_owner(hud=False, directory=DIRECTORY):
  import fcntl
  directory.mkdir(mode=0o700, parents=True, exist_ok=True)
  request = directory / 'hud-request'
  lock = (directory / 'owner.lock').open('a')
  acquired = False
  try:
    if hud:
      temporary = request.with_suffix('.new')
      temporary.write_text(json.dumps({'pid': os.getpid(), 'identity': process_identity(os.getpid())}))
      temporary.replace(request)
    while True:
      try:
        if not hud and hud_requested(directory):
          break
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        acquired = True
        break
      except BlockingIOError:
        if not hud:
          break
        time.sleep(.1)
    if hud:
      request.unlink(missing_ok=True)
    yield acquired
  finally:
    if hud:
      request.unlink(missing_ok=True)
    if acquired:
      fcntl.flock(lock, fcntl.LOCK_UN)
    lock.close()
