import ast
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from openpilot.common import egpu_diagnostics as diag


@pytest.fixture
def evidence(tmp_path, monkeypatch):
  cache, params, usb = (tmp_path / name for name in ('models', 'params', 'usb'))
  for path in (cache, params, usb):
    path.mkdir()
  monkeypatch.setattr(diag, 'command_tail', lambda argv, **kw: 'exit=0\nusb 1-1: reset SuperSpeed USB device\nunrelated line')
  return cache, params, usb, tmp_path, tmp_path / 'update.log'


def test_rejected_artifact_retains_failure_and_usb_evidence(evidence):
  cache, params, usb, _, _ = evidence
  sha = 'a' * 64
  root = cache / 'precompiled' / sha
  root.mkdir(parents=True)
  (cache / 'state.json').write_text(json.dumps({'active': {'sha256': sha, 'filename': 'model.pkl', 'size': 3}}))
  (cache / f'model-{sha[:16]}.pkl').write_bytes(b'abc')
  (root / 'model.pkl').write_bytes(b'abc')
  (root / 'installed.json').write_text(json.dumps({'pickle': {'sha256': sha, 'size': 3}}))
  (root / 'rejected').write_text(sha)
  failure = {'phase': 'inference', 'rejected': True, 'error': 'libusb_control_transfer: Input/Output Error',
             'worker': {'stage': 'output_read', 'schedstat': '100 200 3'}, 'boot_id': 'current-boot'}
  (root / 'last_failure.json').write_text(json.dumps(failure))
  (params / 'UsbGpuCompiled').write_text('0')
  device = usb / '1-1'
  device.mkdir()
  for name, value in {'idVendor': 'add1', 'idProduct': '0001', 'speed': '5000'}.items():
    (device / name).write_text(value)
  report = diag.collect_report(*evidence)
  assert report['artifacts'][0]['rejected'] is True
  assert report['artifacts'][0]['last_failure'] == failure
  assert report['active_source'] == {'exists': True, 'bytes': 3, 'expected_bytes': 3}
  assert report['usb_devices'][0]['speed'] == '5000'
  assert report['params']['UsbGpuCompiled'] == '0'
  assert 'unrelated line' not in report['kernel_usb_tail']
  assert (root / 'rejected').read_text() == sha  # diagnostics never repair or un-reject


def test_missing_and_corrupt_metadata_remain_diagnosable(evidence):
  cache = evidence[0]
  (cache / 'state.json').write_text('{bad')
  report = diag.collect_report(*evidence)
  assert 'invalid JSON' in report['active_model']
  assert report['artifacts'] == []
  assert report['usb_devices'] == []
  assert 'unavailable' in report['params']['UsbGpuCompiled']


def test_report_attached_to_tmux_and_saved_without_credentials(tmp_path, monkeypatch):
  original = {'failure': 'libusb error\nAuthorization: Bearer PRIVATE\nhttps://host/path?key=PRIVATE',
              'artifacts': [{'rejected': True}]}
  monkeypatch.setattr(diag, 'collect_report', lambda *a: original)
  path = tmp_path / 'tmux.log'
  path.write_text('original tmux\n')
  assert diag.append_report(path)
  text = path.read_text()
  assert text.startswith('original tmux\n')
  assert 'PRIVATE' not in text
  payload = json.loads((tmp_path / 'egpu_diagnostics.json').read_text())
  assert payload['artifacts'][0]['rejected']
  assert 'libusb error' in payload['failure']
  assert json.dumps(payload, ensure_ascii=True, indent=2) in text


def test_capture_survives_collection_failure(tmp_path, monkeypatch):
  def fail(*args):
    raise PermissionError('private details')
  monkeypatch.setattr(diag, 'collect_report', fail)
  path = tmp_path / 'tmux.log'
  path.write_text('original')
  assert not diag.append_report(path)
  assert 'original' in path.read_text()
  assert 'PermissionError' in path.read_text()
  assert 'private details' not in path.read_text()


def test_command_output_and_runtime_are_bounded(monkeypatch):
  def run(argv, **kwargs):
    assert kwargs['timeout'] == 2
    kwargs['stdout'].write(b'x' * (diag.LIMIT * 2) + b'last')
    return SimpleNamespace(returncode=0)
  monkeypatch.setattr(diag.subprocess, 'run', run)
  result = diag.command_tail(['dmesg'])
  assert result.endswith('last') and len(result) < diag.LIMIT + 30
  def timeout(*args, **kwargs):
    raise subprocess.TimeoutExpired('dmesg', 2)
  monkeypatch.setattr(diag.subprocess, 'run', timeout)
  assert 'TimeoutExpired' in diag.command_tail(['dmesg'])


def test_main_web_capture_upload_file_contains_report(tmp_path, monkeypatch):
  # Exercise the actual capture function without importing platform-only web/Params dependencies.
  import os
  source = Path('openpilot/selfdrive/carrot/server/features/tools/dispatcher.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'capture_tmux_log_sync')
  function.returns = None
  path = tmp_path / 'tmux.log'
  namespace = {'os': os, 'subprocess': SimpleNamespace(run=lambda *a, **kw: SimpleNamespace(returncode=0, stdout='pane', stderr='')),
               'TMUX_LOG_PATH': str(path)}
  monkeypatch.setattr(diag, 'collect_report', lambda *a: {'test': 'attached'})
  exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
  assert namespace['capture_tmux_log_sync']() == (0, '')
  assert 'pane' in path.read_text() and 'attached' in path.read_text()


def test_recovery_capture_uses_standalone_reporter_and_survives_timeout(tmp_path):
  import os
  source = Path('openpilot/selfdrive/carrot/recovery/server.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_capture_tmux_log')
  calls = []
  def run(argv, **kwargs):
    calls.append(argv)
    if argv[0] == 'tmux':
      return SimpleNamespace(returncode=0, stdout='recovery pane', stderr='')
    assert argv[1].endswith('egpu_diagnostics.py') and kwargs['timeout'] == 8
    raise subprocess.TimeoutExpired(argv, 8)
  path = tmp_path / 'tmux.log'
  namespace = {'os': os, 'subprocess': SimpleNamespace(run=run, DEVNULL=subprocess.DEVNULL, TimeoutExpired=subprocess.TimeoutExpired),
               'REPO_ROOT': Path('openpilot'), 'TMUX_LOG_PATH': str(path)}
  exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
  assert namespace['_capture_tmux_log']() == (0, '')
  assert len(calls) == 2 and path.read_text() == 'recovery pane'


@pytest.mark.parametrize('present,compiled,failed,pending,expected', [
  (True, False, False, '', 'egpu_error'),
  (True, True, True, '', 'egpu_error'),
  (True, False, False, 'spi_error', 'spi_error'),
  (True, True, False, '', ''),
  (False, False, False, '', ''),
])
def test_startup_report_request_without_gpu_work(present, compiled, failed, pending, expected):
  source = Path('openpilot/selfdrive/modeld/modeld.py')
  tree = ast.parse(source.read_text(encoding='utf-8'))
  queue = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'queue_usbgpu_error_tmux')
  main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
  # Run the real startup block, stopping before camera subscriptions and realtime setup.
  stop = next(i for i, n in enumerate(main.body)
              if isinstance(n, ast.Expr) and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name)
              and n.value.func.id == 'config_realtime_process')
  main.body = main.body[:stop]
  values = {'CarrotException': pending, 'UsbGpuStartupFailed': failed}
  class Params:
    def get(self, key, **kwargs):
      return values.get(key)
    def get_bool(self, key):
      return bool(values.get(key))
    def put(self, key, value):
      values[key] = value
    put_bool = put
  namespace = {'Params': Params, 'cloudlog': SimpleNamespace(warning=lambda *a: None, exception=lambda *a: None),
               'USBGPU_TMUX_ERROR_REASON': 'egpu_error', 'usbgpu_present': lambda: present,
               'usbgpu_compiled_path': lambda: 'model.pkl' if compiled else None}
  exec(compile(ast.Module(body=[queue, main], type_ignores=[]), str(source), 'exec'), namespace)
  namespace['main']()
  assert values['CarrotException'] == expected
  assert values['UsbGpuActive'] is False
