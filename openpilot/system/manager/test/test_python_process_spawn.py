"""Exercise the production PythonProcess without importing device-only services."""
import ast
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock  # noqa: TID251 - use pytest with stdlib call assertions

import pytest


@pytest.fixture
def factories():
  path = Path(__file__).parents[1] / 'process.py'
  tree = ast.parse(path.read_text(encoding='utf-8'))
  definition = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'PythonProcess')
  default, spawned = Mock(), Mock()
  namespace = {'ManagerProcess': type('Base', (), {'proc': None, 'shutting_down': False}),
               'Process': default, 'get_context': Mock(return_value=SimpleNamespace(Process=spawned)),
               'launcher': Mock(), 'cloudlog': Mock()}
  exec(compile(ast.Module(body=[definition], type_ignores=[]), str(path), 'exec'), namespace)
  return namespace, default, spawned


@pytest.mark.parametrize('spawn', [False, True])
def test_start_uses_selected_context_and_does_not_duplicate_child(factories, spawn):
  ns, default, spawned = factories
  process = ns['PythonProcess']('worker', 'worker.module', lambda *_: True, spawn=spawn)
  process.start()
  process.start()
  selected, unused = (spawned, default) if spawn else (default, spawned)
  selected.assert_called_once_with(name='worker', target=ns['launcher'], args=('worker.module', 'worker'))
  selected.return_value.start.assert_called_once_with()
  unused.assert_not_called()
  if spawn:
    ns['get_context'].assert_called_once_with('spawn')
  else:
    ns['get_context'].assert_not_called()


def test_deferred_and_restart_sensitive_services_select_spawn():
  path = Path(__file__).parents[1] / 'process_config.py'
  calls = [n for n in ast.walk(ast.parse(path.read_text(encoding='utf-8')))
           if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == 'PythonProcess']
  selected = [n.args[0].value for n in calls if any(k.arg == 'spawn' and ast.literal_eval(k.value) for k in n.keywords)]
  assert selected == ['signalcolord', 'carrot_man', 'xiaoge_data']


@pytest.mark.parametrize('legacy_fork', [False, True])
def test_real_crash_restart_and_supervisor_responsiveness(tmp_path, legacy_fork):
  if legacy_fork and not hasattr(os, 'fork'):
    pytest.skip('fork comparison requires POSIX')
  probe = Path(__file__).with_name('process_lifecycle_probe.py')
  result = subprocess.run([sys.executable, str(probe), str(tmp_path), str(int(legacy_fork))],
                          capture_output=True, text=True, timeout=20)
  assert result.returncode == 0, result.stdout + result.stderr
  data = json.loads(result.stdout)
  assert data['pids'][0] != data['pids'][1]
  assert data['crash_exitcode'] != 0
  assert data['max_supervision_s'] < 2.0
  assert data['fork_callbacks'] == (2 if legacy_fork else 0)
  assert data['inherited_parent_state'] == [legacy_fork, legacy_fork]
  assert data['stopped']
