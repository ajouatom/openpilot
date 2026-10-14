"""Crash diagnostics: no live vehicle, external uploads or native device imports."""
import ast
import json
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
from types import SimpleNamespace

import pytest

from openpilot.system.manager import process_crash as crash


def event(index=1):
  return {'id': f'1-2-{index}', 'name': 'carrot_man', 'pid': 2, 'exitcode': -int(signal.SIGABRT),
          'git_branch': 'test', 'git_commit': 'test-commit', 'dongle_id': 'test-device'}


def collector(tmp_path, **kwargs):
  return crash.CrashCollector(root=tmp_path, capture=lambda _: 'original traceback',
                              payload=lambda _: {'tmux_why': 'process_crash'}, **kwargs)


def wait_until(predicate, timeout=5):
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if predicate():
      return
    time.sleep(0.01)
  assert predicate()


def test_sender_bounded_retry_and_no_success_exit_report(monkeypatch):
  delivered = []
  available = False

  class Socket:
    def __enter__(self):
      return self

    def __exit__(self, *_):
      pass

    def setblocking(self, value):
      assert value is False

    def sendto(self, data, address):
      assert address == 'test-socket'
      if not available:
        raise BlockingIOError()
      delivered.append(json.loads(data))

  monkeypatch.setattr(crash.socket, 'socket', lambda *_: Socket())
  monkeypatch.setattr(crash.socket, 'AF_UNIX', 1, raising=False)
  sender = crash.CrashSender('test-socket')
  for code in (None, 0):
    sender.report('worker', 1, code)
  assert not sender.pending
  for pid in range(1, 50):
    sender.report('worker', pid, -6)
  assert len(sender.pending) == crash.MAX_PENDING
  available = True
  sender.flush()
  assert not sender.pending
  assert [e['pid'] for e in delivered] == list(range(18, 50))


def test_diagnostics_failure_cannot_break_manager(monkeypatch):
  def fail(*_):
    raise RuntimeError('diagnostics broken')
  monkeypatch.setattr(crash._sender, 'report', fail)
  monkeypatch.setattr(crash._sender, 'flush', fail)
  crash.report_process_exit('worker', 1, -6)
  crash.flush_crash_reports()


def test_snapshot_bounded_and_exit_metadata_survives_missing_params(tmp_path):
  c = collector(tmp_path)
  c.capture = lambda _: 'x' * (crash.MAX_TEXT + 100)
  c.payload = lambda _: (_ for _ in ()).throw(OSError('params unavailable'))
  for index in range(crash.MAX_REPORTS + 3):
    c.record(event(index))
  reports = sorted(tmp_path.glob('*.json'))
  assert len(reports) == len(list(tmp_path.glob('*.txt'))) == crash.MAX_REPORTS
  assert not (tmp_path / '1-2-0.json').exists()
  data = json.loads(reports[-1].read_text())
  assert data['event']['signal'] == 'SIGABRT'
  assert data['payload']['git_commit'] == 'test-commit'
  assert data['payload']['tmux_why'] == 'process_crash'
  text = reports[-1].with_suffix('.txt').read_text()
  assert text.startswith('PROCESS_EXIT ')
  assert len(text.split('\n\n', 1)[1]) == crash.MAX_TEXT


@pytest.mark.parametrize('field,value', [('id', '../escape'), ('id', '1' * 300), ('name', 'a/b'), ('exitcode', '-6')])
def test_bad_event_rejected(tmp_path, field, value):
  e = event()
  e[field] = value
  with pytest.raises(ValueError):
    collector(tmp_path).record(e)
  assert not list(tmp_path.iterdir())


def test_saved_capture_retry_and_per_destination_ack_survive_restart(tmp_path):
  calls = []
  online = False

  def upload(target, payload, path):
    calls.append((target, path.read_text()))
    assert payload['tmux_why'] == 'process_crash'
    if target == 'dsm' and not online:
      raise OSError('offline')
    return True

  c = collector(tmp_path, upload=upload)
  c.record(event())
  original = (tmp_path / '1-2-1.txt').read_text()
  c.upload_pending(now=0)
  assert [x[0] for x in calls] == ['dsm', 'carrot_logs']
  c.capture = lambda _: 'new pane contents'
  c.record(event())  # Duplicate cannot replace evidence or acknowledgements.
  c.upload_pending(now=1)
  assert len(calls) == 2
  online = True
  restarted = collector(tmp_path, upload=upload)
  restarted.upload_pending(now=2)
  restarted.upload_pending(now=100)
  assert [x[0] for x in calls] == ['dsm', 'carrot_logs', 'dsm']
  assert all(x[1] == original for x in calls)
  assert json.loads((tmp_path / '1-2-1.json').read_text())['sent'] == ['carrot_logs', 'dsm']


def test_tmux_capture_failure_still_persists_exit(tmp_path, monkeypatch):
  def fail(*args, **kwargs):
    assert kwargs['timeout'] == 3
    raise subprocess.TimeoutExpired(args[0], 3)
  monkeypatch.setattr(crash.subprocess, 'run', fail)
  c = collector(tmp_path)
  c.capture = crash.capture_tmux
  c.record(event())
  assert 'TimeoutExpired' in (tmp_path / '1-2-1.txt').read_text()
  assert json.loads((tmp_path / '1-2-1.json').read_text())['event']['exitcode'] == -int(signal.SIGABRT)


def test_launcher_prints_exception_before_sentry(capsys):
  source = Path(crash.__file__).with_name('process.py')
  definition = next(n for n in ast.parse(source.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'launcher')

  def fail_import(_):
    raise RuntimeError('original child failure')

  def sentry_failure():
    assert 'RuntimeError: original child failure' in capsys.readouterr().err
    raise OSError('sentry unavailable')

  ns = {'enable_fault_handler': lambda: None, 'traceback': traceback,
        'importlib': SimpleNamespace(import_module=fail_import), 'sentry': SimpleNamespace(capture_exception=sentry_failure)}
  exec(compile(ast.Module(body=[definition], type_ignores=[]), str(source), 'exec'), ns)
  with pytest.raises(OSError, match='sentry unavailable'):
    ns['launcher']('test_worker', 'worker')


@pytest.mark.skipif(os.name != 'posix', reason='device transport requires POSIX Unix datagrams')
def test_real_socket_captures_while_uploader_is_blocked(tmp_path):
  blocked, release = threading.Event(), threading.Event()

  def upload(*_):
    blocked.set()
    release.wait(5)
    return True

  c = collector(tmp_path / 'reports', address=str(tmp_path / 'socket'), upload=upload).start()
  sender = crash.CrashSender(c.address)
  try:
    sender.report('worker', 101, -6)
    assert blocked.wait(3)
    started = time.monotonic()
    sender.report('worker', 102, -9)
    assert time.monotonic() - started < 0.5
    wait_until(lambda: len(list(c.root.glob('*.json'))) == 2)
    data = [json.loads(p.read_text())['event'] for p in c.root.glob('*.json')]
    assert {e['signal'] for e in data} == {'SIGABRT', 'SIGKILL'}
    assert not sender.pending
    assert os.stat(c.address).st_mode & 0o777 == 0o600
  finally:
    release.set()
    c.close()


@pytest.mark.skipif(os.name != 'posix' or not shutil.which('tmux'), reason='Linux tmux integration')
def test_fatal_signal_stack_is_in_real_tmux_snapshot(tmp_path, monkeypatch):
  # An isolated tmux server runs a disposable Python child, never the vehicle.
  server = f'crash-test-{os.getpid()}'
  prefix = ['tmux', '-L', server]
  real_run = subprocess.run
  code = ('import resource, os; resource.setrlimit(resource.RLIMIT_CORE, (0, 0)); '
          + 'from openpilot.system.manager.process_crash import enable_fault_handler; '
          + 'enable_fault_handler(); os.abort()')
  root = Path(crash.__file__).resolve().parents[3]
  command = f'cd {shlex.quote(str(root))} && {shlex.quote(sys.executable)} -c {shlex.quote(code)}; sleep 15'
  try:
    real_run(prefix + ['new-session', '-d', '-s', 'crash', command], check=True, capture_output=True, timeout=5)

    def isolated_run(cmd, **kwargs):
      assert cmd[0] == 'tmux'
      return real_run(prefix + cmd[1:], **kwargs)

    monkeypatch.setattr(crash.subprocess, 'run', isolated_run)
    e = event()
    e['pane'] = 'crash:0.0'
    wait_until(lambda: 'Fatal Python error: Aborted' in crash.capture_tmux(e))
    c = collector(tmp_path)
    c.capture = crash.capture_tmux
    c.record(e)
    text = (tmp_path / '1-2-1.txt').read_text()
    assert 'SIGABRT' in text and 'Fatal Python error: Aborted' in text
    assert 'File "<string>", line 1' in text
  finally:
    real_run(prefix + ['kill-server'], capture_output=True, timeout=5)


def test_upload_uses_existing_tmux_destinations_and_snapshot(tmp_path, monkeypatch):
  import requests
  from openpilot.selfdrive.carrot import web_upload
  from openpilot.selfdrive.carrot.server.services import web_settings

  path = tmp_path / 'crash.txt'
  path.write_text('saved crash snapshot')
  calls = []

  def post(url, **kwargs):
    calls.append(url)
    if url.endswith('/session'):
      assert kwargs['json']['purpose'] == 'tmux'
      return SimpleNamespace(status_code=200, json=lambda: {'ok': True, 'token': 'test-session'})
    assert kwargs['files'][0][1][1].read() == b'saved crash snapshot'
    assert kwargs['data']['tmux_why'] == 'process_crash'
    assert kwargs['timeout'] == 30
    return SimpleNamespace(status_code=200)

  monkeypatch.delenv('FAKEUPLOAD', raising=False)
  monkeypatch.delenv('CARROT_WEB_UPLOAD_TOKEN', raising=False)
  monkeypatch.setenv('CARROT_WEB_UPLOAD_URL', 'https://nas.invalid')
  monkeypatch.setenv('CARROT_TMUX_WEB_UPLOAD_URL', 'https://logs.invalid/upload')
  monkeypatch.setattr(web_settings, 'read_web_settings', dict)
  monkeypatch.setattr(requests, 'post', post)
  payload = {'tmux_why': 'process_crash', 'dongle_id': 'test'}
  assert crash.upload_report('dsm', payload, path)
  assert crash.upload_report('carrot_logs', payload, path)
  assert calls == ['https://nas.invalid/api/v1/session', 'https://nas.invalid/api/v1/tmux/upload',
                   web_upload.carrot_logs_web_target()[0]]
