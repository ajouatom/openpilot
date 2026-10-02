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
  assert not update.start_rebuild()
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


def test_error_log_prioritizes_diagnostic_without_losing_context(tmp_path):
  path = tmp_path / 'failure.log'
  path.write_text('Compiling file.cc\n\x1b[31mfile.cc:12: error: missing header\x1b[0m\n' +
                  '#include <missing.h>\nscons: building terminated because of errors.\n')
  text = startup_recovery.read_startup_error(path)
  assert text.startswith('file.cc:12: error: missing header\n#include <missing.h>')
  assert 'Compiling file.cc' in text and '\x1b' not in text


def test_error_log_handles_tracebacks_missing_and_large_logs(tmp_path):
  path = tmp_path / 'failure.log'
  assert startup_recovery.read_startup_error(path) == ''
  assert startup_recovery.read_startup_error(None) == ''
  path.write_bytes(b'old output\n' * 10000 + b'Traceback (most recent call last):\n' +
                   b'  File "manager.py", line 1\nModuleNotFoundError: missing_native\n')
  text = startup_recovery.read_startup_error(path)
  assert text.startswith('ModuleNotFoundError: missing_native')
  assert 'File "manager.py"' in text and len(text) <= 65536


@pytest.mark.parametrize('clean_fails', [False, True])
def test_rebuild_cleans_offline_and_reboots_only_on_success(tmp_path, monkeypatch, clean_fails):
  monkeypatch.setattr(repo_update, 'LOCK_PATH', str(tmp_path / 'update.lock'))
  (tmp_path / 'prebuilt').touch()
  (tmp_path / 'source.txt').write_text('local source changes')
  calls = []

  def run(command, **kwargs):
    assert command[:2] == ['scons', '-c']
    assert kwargs['cwd'] == tmp_path
    # Cleanup must own the same lock used by Git and the launcher.
    with pytest.raises(repo_update.RepoBusyError), repo_update.repo_lock():
      pytest.fail('cleanup did not hold repository lock')
    calls.append('clean')
    return subprocess.CompletedProcess(command, int(clean_fails), '', 'cleanup error' if clean_fails else '')

  monkeypatch.setattr(startup_recovery.subprocess, 'run', run)
  update = startup_recovery.RecoveryUpdate(tmp_path, reboot=lambda: calls.append('reboot'))
  update._rebuild()
  assert (tmp_path / 'source.txt').read_text() == 'local source changes'
  assert (tmp_path / 'prebuilt').exists() == clean_fails
  assert calls == (['clean'] if clean_fails else ['clean', 'reboot'])
  assert update.state[0] == ('failed' if clean_fails else 'rebuild_rebooting')
  if clean_fails:
    assert 'cleanup error' in update.state[1]


def test_rebuild_and_git_exclude_each_other(tmp_path, monkeypatch):
  entered, finish, done = threading.Event(), threading.Event(), threading.Event()

  def rebuild(self):
    entered.set()
    finish.wait(3)
    self._set('failed', 'Retry available')
    done.set()

  monkeypatch.setattr(startup_recovery.RecoveryUpdate, '_rebuild', rebuild)
  update = startup_recovery.RecoveryUpdate(tmp_path)
  assert update.start_rebuild() and entered.wait(2)
  try:
    assert not update.start_rebuild()
    assert not update.start(automatic=True)
    assert not update.start()
  finally:
    finish.set()
  assert done.wait(2)


@pytest.mark.parametrize('failure', ['busy', 'timeout', 'reboot'])
def test_rebuild_failure_is_retryable(tmp_path, monkeypatch, failure):
  monkeypatch.setattr(repo_update, 'LOCK_PATH', str(tmp_path / 'update.lock'))
  calls = []

  def clean(repo):
    calls.append('clean')
    if failure == 'timeout':
      raise subprocess.TimeoutExpired('scons', 180)

  def reboot():
    calls.append('reboot')
    raise OSError('reboot failed')

  monkeypatch.setattr(startup_recovery, 'clean_startup_build', clean)
  update = startup_recovery.RecoveryUpdate(tmp_path, reboot=reboot)
  if failure == 'busy':
    with repo_update.repo_lock():
      update._rebuild()
  else:
    update._rebuild()
  assert update.state[0] == 'failed' and update.state[1]
  assert calls == {'busy': [], 'timeout': ['clean'], 'reboot': ['clean', 'reboot']}[failure]


def test_error_pages_wrap_without_losing_long_paths(monkeypatch):
  pytest.importorskip('pyray')
  from openpilot.system.ui import startup_recovery as ui
  monkeypatch.setattr(ui.rl, 'measure_text_ex', lambda font, text, size, spacing: ui.rl.Vector2(len(text) * 8, 12))
  error = '/long/path/' * 50 + ': error: missing header'
  pages = ui.error_pages(None, error, 1)
  assert len(pages) > 1
  assert ''.join(line for page in pages for line in page) == error
  assert all(len(page) <= 4 and all(len(line) <= 63 for line in page) for page in pages)


@pytest.mark.parametrize('status', startup_recovery.BUSY_STATES + ('idle', 'waiting', 'failed'))
def test_display_keeps_original_error_in_every_update_state(monkeypatch, status):
  pytest.importorskip('pyray')
  from openpilot.system.ui import startup_recovery as ui
  drawn = []
  monkeypatch.setattr(ui.rl, 'measure_text_ex', lambda font, text, size, spacing: ui.rl.Vector2(len(text) * size / 2, size))
  monkeypatch.setattr(ui.rl, 'draw_text_ex', lambda font, text, *args: drawn.append(text))
  monkeypatch.setattr(ui.rl, 'clear_background', lambda *args: None)
  monkeypatch.setattr(ui.rl, 'draw_rectangle_rounded', lambda *args: None)
  rebuild, update, error = ui.draw_screen(None, 536, 240, (status, 'Update status'), 'Build failed', 'host:6999',
                                         [['fatal error: missing header']])
  assert 'fatal error: missing header' in drawn and 'Update status' in drawn
  assert ui.REBUILD_KO in drawn and ui.BUTTON_KO in drawn
  assert error.y + error.height <= rebuild.y
  assert rebuild.x + rebuild.width < update.x
  assert update.x + update.width <= 536 and update.y + update.height <= 240
