import json
import subprocess

import pytest

from openpilot.system.manager import update_status


def git(repo, *args):
  return subprocess.run(['git', '-c', 'user.name=Update Test', '-c', 'user.email=update@example.invalid', *args],
                        cwd=repo, capture_output=True, text=True, check=True).stdout.strip()


@pytest.fixture
def repo(tmp_path):
  git(tmp_path, 'init', '-q')
  git(tmp_path, 'commit', '--allow-empty', '-qm', 'running')
  return tmp_path


def test_checkout_change_revert_and_restart(repo):
  monitor = update_status.UpdateStatus(str(repo))
  running = monitor.running_commit
  assert not monitor.update(0)
  # Pull/checkout changes HEAD while this manager remains alive.
  git(repo, 'commit', '--allow-empty', '-qm', 'downloaded')
  assert update_status.read_checkout_commit(repo) != running
  assert not monitor.update(5)
  assert not monitor.update(9)  # Cached; not a second confirmation.
  assert monitor.update(10)
  assert monitor.update(30)
  assert monitor.running_commit == running
  # A manager restart captures the installed checkout and clears the notice.
  restarted = update_status.UpdateStatus(str(repo))
  assert not restarted.update(30)
  assert not restarted.update(35)
  # Reverting to the actually running version also clears the notice.
  git(repo, 'checkout', '--detach', running)
  assert not monitor.update(35)


def test_branch_name_and_dirty_files_do_not_require_reboot(repo):
  monitor = update_status.UpdateStatus(str(repo))
  git(repo, 'checkout', '-qb', 'different-name')
  (repo / 'uncommitted.txt').write_text('local edit', encoding='utf-8')
  assert not monitor.update(0)
  assert not monitor.update(5)


def test_linked_worktree_head(repo, tmp_path):
  worktree = tmp_path / 'linked'
  git(repo, 'worktree', 'add', '-q', '-b', 'linked-test', str(worktree))
  assert (worktree / '.git').is_file()
  monitor = update_status.UpdateStatus(str(worktree))
  git(worktree, 'commit', '--allow-empty', '-qm', 'linked update')
  assert not monitor.update(0)
  assert monitor.update(5)


def test_packaged_release_uses_metadata_without_git(tmp_path):
  metadata = tmp_path / 'build.json'
  metadata.write_text(json.dumps({'openpilot': {'git_commit': 'a' * 40}}), encoding='utf-8')
  monitor = update_status.UpdateStatus(str(tmp_path))
  metadata.write_text(json.dumps({'openpilot': {'git_commit': 'b' * 40}}), encoding='utf-8')
  assert not monitor.update(0)
  assert monitor.update(5)


def test_missing_startup_identity_is_not_recaptured(monkeypatch):
  monkeypatch.setattr(update_status, 'read_checkout_commit', lambda repo: None)
  monitor = update_status.UpdateStatus('.')
  monkeypatch.setattr(update_status, 'read_checkout_commit', lambda repo: 'a' * 40)
  assert not monitor.update(0)
  assert not monitor.update(5)
  assert monitor.running_commit is None


def test_transient_change_and_read_failure_do_not_confirm_update(monkeypatch):
  commits = iter(['a' * 40, 'b' * 40, 'a' * 40, 'b' * 40, None, 'b' * 40, 'b' * 40])
  monkeypatch.setattr(update_status, 'read_checkout_commit', lambda repo: next(commits))
  monitor = update_status.UpdateStatus('.')
  assert [monitor.update(t) for t in (0, 5, 10, 15, 20, 25)] == [False, False, False, False, False, True]


@pytest.mark.parametrize('contents', ['{}', 'null', '{', '{"openpilot":{"git_commit":"unknown"}}'])
def test_invalid_metadata_is_unknown(tmp_path, contents):
  (tmp_path / 'build.json').write_text(contents, encoding='utf-8')
  assert update_status.read_checkout_commit(tmp_path) is None


@pytest.mark.parametrize('failure', [OSError('missing git'), subprocess.TimeoutExpired('git', 1)])
def test_git_probe_failure_is_bounded_and_ignored(tmp_path, monkeypatch, failure):
  (tmp_path / '.git').mkdir()

  def fail(*args, **kwargs):
    assert kwargs['timeout'] == 1
    raise failure

  monkeypatch.setattr(update_status.subprocess, 'run', fail)
  assert update_status.read_checkout_commit(tmp_path) is None
