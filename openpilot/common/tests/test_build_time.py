from pathlib import Path
import subprocess
from unittest.mock import Mock  # noqa: TID251 -- pytest tests; mock clock-setting subprocesses only

import pytest

from openpilot.common import build_time


COMMIT = 1791158400


@pytest.fixture
def commands(monkeypatch):
  run = Mock(return_value=subprocess.CompletedProcess([], 0, stdout=f'{COMMIT}\n'))
  monkeypatch.setattr(build_time.subprocess, 'run', run)
  return run


@pytest.mark.parametrize('now', [COMMIT, COMMIT + 1, COMMIT + 86400])
def test_current_or_future_clock_is_untouched(monkeypatch, commands, now):
  monkeypatch.setattr(build_time.time, 'time', lambda: now)
  build_time.ensure_build_time(Path('checkout'))
  assert commands.call_count == 1
  assert commands.call_args.args[0] == ['git', '-C', 'checkout', 'show', '-s', '--format=%ct', 'HEAD']


def test_old_clock_advances_offline_and_is_verified(monkeypatch, commands, capsys):
  monkeypatch.setattr(build_time.time, 'time', Mock(side_effect=[COMMIT - 86400, COMMIT - 86400, COMMIT + 1]))
  build_time.ensure_build_time(Path('checkout'))
  assert commands.call_count == 2
  assert commands.call_args.args[0] == ['sudo', '-n', 'date', '-u', '-s', f'@{COMMIT + 1}']
  assert commands.call_args.kwargs['timeout'] == 10
  log = capsys.readouterr().out
  assert 'device=' in log and 'HEAD=' in log and 'After correction:' in log


def test_ntp_catches_up_before_set(monkeypatch, commands):
  monkeypatch.setattr(build_time.time, 'time', Mock(side_effect=[COMMIT - 100, COMMIT + 100]))
  build_time.ensure_build_time(Path('checkout'))
  assert commands.call_count == 1


def test_successful_command_with_unchanged_clock_fails(monkeypatch, commands):
  monkeypatch.setattr(build_time.time, 'time', lambda: COMMIT - 100)
  assert build_time.main() == 1
  assert commands.call_count == 2


@pytest.mark.parametrize('error', [
  subprocess.CalledProcessError(1, ['sudo'], stderr='permission denied'),
  subprocess.TimeoutExpired(['sudo'], 10),
  FileNotFoundError('sudo'),
])
def test_failed_set_does_not_report_success(monkeypatch, commands, error):
  monkeypatch.setattr(build_time.time, 'time', lambda: COMMIT - 100)
  commands.side_effect = [commands.return_value, error]
  assert build_time.main() == 1


@pytest.mark.parametrize('stamp', ['', 'invalid', '-1', '0', '99999999999999999999999'])
def test_invalid_commit_never_sets_clock(commands, stamp):
  commands.return_value.stdout = stamp
  assert build_time.main() == 1
  assert commands.call_count == 1


def test_missing_git_metadata_never_sets_clock(commands):
  commands.side_effect = subprocess.CalledProcessError(128, ['git'], stderr='not a git repository')
  assert build_time.main() == 1
  assert commands.call_count == 1
