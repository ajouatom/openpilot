import os
import io
from pathlib import Path
import shlex
import subprocess
import sys
import threading

import pytest

from openpilot.common import repo_update, startup_recovery
from openpilot.common import text_window


def git(repo, *args):
  return subprocess.run(['git', '-C', str(repo), *args], check=True, capture_output=True, text=True).stdout.strip()


@pytest.fixture
def checkouts(tmp_path, monkeypatch):
  remote, author, device = (tmp_path / name for name in ('remote', 'author', 'device'))
  subprocess.run(['git', 'init', '--bare', '-b', 'main', str(remote)], check=True, capture_output=True)
  subprocess.run(['git', 'clone', str(remote), str(author)], check=True, capture_output=True)
  git(author, 'config', 'user.name', 'Recovery test')
  git(author, 'config', 'user.email', 'recovery@example.invalid')
  (author / 'source.txt').write_text('old\n')
  git(author, 'add', 'source.txt')
  git(author, 'commit', '-m', 'initial')
  git(author, 'push', '-u', 'origin', 'main')
  subprocess.run(['git', 'clone', str(remote), str(device)], check=True, capture_output=True)
  (author / 'source.txt').write_text('fixed\n')
  git(author, 'commit', '-am', 'fix startup')
  git(author, 'push')
  monkeypatch.setattr(repo_update, 'LOCK_PATH', str(tmp_path / 'update.lock'))
  monkeypatch.setattr(repo_update, 'git_process_running', lambda: False)
  monkeypatch.setenv('GIT_TERMINAL_PROMPT', '0')
  return device, git(author, 'rev-parse', 'HEAD')


def test_update_current_branch_then_request_reboot(checkouts):
  device, target = checkouts
  calls = []
  update = startup_recovery.RecoveryUpdate(device, reboot=lambda: calls.append(git(device, 'rev-parse', 'HEAD')))
  update._run()
  assert calls == [target] and update.state == ('rebooting', target[:10])
  assert (device / 'source.txt').read_text() == 'fixed\n'
  assert git(device, 'branch', '--show-current') == 'main'


@pytest.mark.parametrize('reason', ['dirty', 'diverged', 'network', 'busy', 'reboot'])
def test_failure_preserves_checkout_and_allows_retry(checkouts, monkeypatch, reason):
  device, _ = checkouts
  old = git(device, 'rev-parse', 'HEAD')
  calls = []
  if reason == 'dirty':
    (device / 'source.txt').write_text('my changes\n')
  elif reason == 'diverged':
    git(device, 'config', 'user.name', 'Recovery test')
    git(device, 'config', 'user.email', 'recovery@example.invalid')
    (device / 'local.txt').write_text('keep\n')
    git(device, 'add', 'local.txt')
    git(device, 'commit', '-m', 'local work')
    old = git(device, 'rev-parse', 'HEAD')
  elif reason == 'network':
    git(device, 'remote', 'set-url', 'origin', str(device / 'missing-remote'))
  elif reason == 'busy':
    def locked():
      raise repo_update.RepoBusyError('Build still running')
    monkeypatch.setattr(startup_recovery, 'repo_lock', locked)

  def reboot():
    if reason == 'reboot':
      raise OSError('Reboot command failed')
    calls.append(True)

  update = startup_recovery.RecoveryUpdate(device, reboot=reboot)
  update._run()
  assert update.state[0] == 'failed' and update.state[1]
  assert calls == []
  if reason != 'reboot':
    assert git(device, 'rev-parse', 'HEAD') == old
  if reason == 'dirty':
    assert (device / 'source.txt').read_text() == 'my changes\n'


def test_repeated_taps_do_not_start_concurrent_updates(monkeypatch, tmp_path):
  entered, finish = threading.Event(), threading.Event()
  calls = []

  def run(self, automatic=False):
    calls.append(True)
    entered.set()
    finish.wait(3)
    self._set('failed', 'Retry available')

  monkeypatch.setattr(startup_recovery.RecoveryUpdate, '_run', run)
  update = startup_recovery.RecoveryUpdate(tmp_path)
  assert update.start() and entered.wait(2)
  assert not update.start()
  finish.set()
  assert len(calls) == 1


def test_automatic_recovery_never_reboots_the_same_failed_revision(checkouts):
  device, target = checkouts
  reboots = []
  first = startup_recovery.RecoveryUpdate(device, reboot=lambda: reboots.append(True))
  first._run(automatic=True)
  assert first.state == ('rebooting', target[:10]) and reboots == [True]
  # Simulate rebooting into that commit and failing the build again.
  after_reboot = startup_recovery.RecoveryUpdate(device, reboot=lambda: reboots.append(True))
  after_reboot._run(automatic=True)
  assert after_reboot.state[0] == 'waiting' and reboots == [True]
  # Manual action may explicitly retry the existing revision.
  after_reboot._run()
  assert reboots == [True, True]


def test_reboot_command_failure_can_retry_after_successful_pull(checkouts):
  device, target = checkouts

  def fail_reboot():
    raise OSError('reboot failed')

  update = startup_recovery.RecoveryUpdate(device, reboot=fail_reboot)
  update._run(automatic=True)
  assert update.state[0] == 'failed'
  assert git(device, 'rev-parse', 'HEAD') == target
  calls = []
  update.reboot = lambda: calls.append(True)
  update._run(automatic=True)
  assert update.state[0] == 'rebooting' and calls == [True]


def test_network_restoration_applies_the_pending_fix(checkouts):
  device, target = checkouts
  remote = git(device, 'remote', 'get-url', 'origin')
  git(device, 'remote', 'set-url', 'origin', str(device / 'offline'))
  calls = []
  update = startup_recovery.RecoveryUpdate(device, reboot=lambda: calls.append(True))
  update._run(automatic=True)
  assert update.state[0] == 'failed' and not calls
  git(device, 'remote', 'set-url', 'origin', remote)
  update._run(automatic=True)
  assert update.state == ('rebooting', target[:10]) and calls == [True]


@pytest.mark.parametrize('returncode', [0, 1, 2, -9])
def test_text_window_does_not_wait_forever_after_child_exit(monkeypatch, returncode):
  class Process:
    def poll(self):
      return returncode

    def terminate(self):
      pass

  process = Process()
  process.returncode = returncode
  monkeypatch.setattr(text_window.subprocess, 'Popen', lambda *a, **k: process)
  monkeypatch.setattr(text_window.time, 'sleep', lambda *a: pytest.fail('Exited window must not loop'))
  with text_window.TextWindow('error') as window:
    window.wait_for_exit()


def test_launcher_captures_failed_command_and_releases_lock_before_recovery(tmp_path):
  root = Path(__file__).resolve().parents[3]
  source = (root / 'launch_chffrplus.sh').read_text(encoding='utf8')
  recovery = source[source.index('function run_startup_command {'):source.index('function launch {')]
  assert recovery.index('flock -u 9') < recovery.index('startup_recovery.py')
  assert recovery.index('exec 9>&-') < recovery.index('startup_recovery.py')
  assert 'unset CARROT_BOOT_LOCK_FD' in recovery
  assert 'while true; do' in recovery  # a crashed display is relaunched, never a silent wait
  assert 'python3 -m openpilot.common.startup_recovery --repo "$DIR"' in recovery
  assert 'CARROT_STARTUP_RECOVERY' in (root / 'openpilot/system/manager/build.py').read_text()
  assert 'CARROT_STARTUP_RECOVERY' in (root / 'openpilot/system/manager/manager.py').read_text()
  bash = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else 'bash'
  command = recovery[:recovery.index('function show_startup_failure')]
  command = command.replace('/tmp/carrot_startup_failure.log', 'failure.log')
  command = command.replace('python3', shlex.quote(sys.executable.replace('\\', '/')))
  probe = command + '\nrun_startup_command bash -c "echo failed; exit 17"\nexit $?\n'
  result = subprocess.run([bash, '-c', probe], cwd=tmp_path, capture_output=True, text=True, timeout=5,
                          env={**os.environ, 'PYTHONPATH': str(root)})
  assert result.returncode == 17 and 'failed' in (tmp_path / 'failure.log').read_text()


def test_startup_capture_is_bounded_and_keeps_the_last_error(tmp_path):
  data = b'build output\n' * 20000 + b'last error\n'
  destination = io.BytesIO()
  path = tmp_path / 'failure.log'
  startup_recovery.capture_startup_output(io.BytesIO(data), destination, path)
  assert destination.getvalue() == data
  assert path.stat().st_size <= 65536
  assert path.read_bytes().endswith(b'last error\n')
