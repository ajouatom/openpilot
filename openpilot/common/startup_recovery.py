"""Startup-only update action. No Params, cereal, hardware or UI imports."""
import os
from collections import deque
from pathlib import Path
import subprocess
import threading
import time

from openpilot.common.repo_update import child_lock_kwargs, recover_stale_index_lock, repo_lock
from openpilot.selfdrive.carrot.server.services.git_config import prepare_git_pull


def capture_startup_output(source, destination, log_path: Path):
  """Forward output while keeping at most 64 KiB of startup/error history."""
  chunks = deque()
  size = 0
  last_write = -float('inf')
  while data := source.readline(4096):
    destination.write(data)
    destination.flush()
    chunks.append(data)
    size += len(data)
    while size > 65536:
      size -= len(chunks.popleft())
    if time.monotonic() - last_write >= 1:
      log_path.write_bytes(b''.join(chunks))
      last_write = time.monotonic()
  log_path.write_bytes(b''.join(chunks))


def pull_current_branch(repo: Path) -> tuple[str, bool]:
  def git(*args):
    result = subprocess.run(['git', *args], cwd=repo, capture_output=True, text=True,
                            encoding='utf-8', errors='replace', timeout=180, **child_lock_kwargs())
    if result.returncode:
      raise RuntimeError(result.stderr.strip() or result.stdout.strip() or 'Git failed')
    return result.stdout.strip()

  # Caller owns repo_lock for the complete update and reboot request.
  recover_stale_index_lock(str(repo))
  if not git('symbolic-ref', '--quiet', '--short', 'HEAD'):
    raise RuntimeError('Select a branch before updating.')
  if git('status', '--porcelain', '--untracked-files=no'):
    raise RuntimeError('Local changes found. Resolve them in the recovery terminal; no files were reset.')
  before = git('rev-parse', 'HEAD')
  rc, detail, target = prepare_git_pull(str(repo))
  if rc or not target:
    raise RuntimeError(detail or 'Unable to fetch the current branch.')
  git('merge', '--ff-only', target)
  if git('rev-parse', 'HEAD') != target:
    raise RuntimeError('Local commits differ from the update. Resolve them in the recovery terminal.')
  return target, target != before


def reboot_device():
  if not (Path('/AGNOS').exists() or Path('/TICI').exists()):
    raise RuntimeError('Device reboot is disabled on this computer.')
  subprocess.run(['sudo', '-n', 'reboot'], check=True, capture_output=True, timeout=15)


class RecoveryUpdate:
  def __init__(self, repo: Path, reboot=reboot_device):
    self.repo = repo
    self.reboot = reboot
    self._lock = threading.Lock()
    self._state = ('idle', '')
    self._pending_reboot = None

  @property
  def state(self):
    with self._lock:
      return self._state

  def _set(self, state, detail=''):
    with self._lock:
      self._state = (state, detail)

  def start(self, automatic=False):
    with self._lock:
      if self._state[0] in ('updating', 'rebooting'):
        return False
      self._state = ('updating', '')
    threading.Thread(target=self._run, args=(automatic,), name='startup-recovery-update', daemon=True).start()
    return True

  def _run(self, automatic=False):
    try:
      # No credential prompts or detached Git maintenance on an error screen.
      os.environ['GIT_TERMINAL_PROMPT'] = '0'
      with repo_lock():
        target, changed = pull_current_branch(self.repo)
        if automatic and not changed and target != self._pending_reboot:
          self._set('waiting', 'No new commit. Waiting for a fix or network connection.')
          return
        self._pending_reboot = target
        self._set('rebooting', target[:10])
        self.reboot()
    except Exception as exc:
      print(f'Startup recovery update failed: {exc}', flush=True)
      self._set('failed', str(exc)[-2000:])


if __name__ == '__main__':
  # The launcher calls this standard-library-only fallback if even Raylib/the
  # display is unavailable. It never needs to import a graphics dependency.
  import argparse
  import sys
  parser = argparse.ArgumentParser()
  action = parser.add_mutually_exclusive_group(required=True)
  action.add_argument('--repo', type=Path)
  action.add_argument('--capture-log', type=Path)
  args = parser.parse_args()
  if args.capture_log:
    capture_startup_output(sys.stdin.buffer, sys.stdout.buffer, args.capture_log)
    raise SystemExit(0)
  if os.environ.get('CARROT_STARTUP_RECOVERY') != '1':
    raise SystemExit('Only the failed-startup launcher may request automatic recovery.')
  update = RecoveryUpdate(args.repo)
  update._run(automatic=True)
  print(*update.state, flush=True)
