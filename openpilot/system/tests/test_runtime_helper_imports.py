"""Keep startup-loaded helpers usable when an update replaces checkout files.

Run in a fresh interpreter/package tree. Native callers use their actual method
and module-level helper import, without importing hardware or starting services.
Helper bodies are inert: this tests import lifetime, not reboot/CAN execution.
"""
import ast
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[3]
CASES = [
  ('openpilot/system/hardware/tici/hardware.py', 'Tici', 'reboot',
   'openpilot.common.reboot', 'reboot_device', 'caller.reboot(None)'),
  ('openpilot/selfdrive/carrot/signal_display.py', None, 'display_requested',
   'openpilot.selfdrive.carrot.signal_mode', 'flag_override', 'assert caller.display_requested() is False'),
  ('openpilot/selfdrive/carrot/carrot_man.py', 'CarrotMan', 'make_tmux_data',
   'openpilot.common.egpu_diagnostics', 'append_report', 'assert caller.make_tmux_data(None) is True'),
  ('openpilot/selfdrive/carrot/server/features/tools/dispatcher.py', None, 'capture_tmux_log_sync',
   'openpilot.common.egpu_diagnostics', 'append_report', 'assert caller.capture_tmux_log_sync() == (0, "")'),
]


@pytest.mark.parametrize('replacement', ['missing', 'different_version'])
@pytest.mark.parametrize('source,cls,method,helper,function,invoke', CASES, ids=['reboot', 'display', 'tmux', 'web_tmux'])
def test_helper_survives_checkout_change(tmp_path, replacement, source, cls, method, helper, function, invoke):
  source_text = (ROOT / source).read_text(encoding='utf-8')
  if method != 'display_requested':
    tree = ast.parse(source_text)
    body = next(n.body for n in tree.body if isinstance(n, ast.ClassDef) and n.name == cls) if cls else tree.body
    target = next(n for n in body if isinstance(n, ast.FunctionDef) and n.name == method)
    imports = [n for n in tree.body if isinstance(n, ast.ImportFrom) and n.module == helper]
    source_text = ast.unparse(ast.Module(body=imports + [target], type_ignores=[]))
  (tmp_path / 'caller.py').write_text('from __future__ import annotations\n' + source_text, encoding='utf-8')

  helper_path = tmp_path.joinpath(*helper.split('.')).with_suffix('.py')
  helper_path.parent.mkdir(parents=True)
  parent = helper_path.parent
  while parent != tmp_path:
    (parent / '__init__.py').touch()
    parent = parent.parent
  marker = tmp_path / 'calls.txt'
  helper_path.write_text(
    f'from pathlib import Path\ndef {function}(*args):\n'
    f'  with Path({str(marker)!r}).open("a") as f: f.write("startup helper\\n")\n'
    '  return False\n', encoding='utf-8')

  runner = tmp_path / 'run.py'
  runner.write_text(f'''
import importlib
import os
from pathlib import Path
import sys
from types import SimpleNamespace
sys.path.insert(0, {str(tmp_path)!r})
import caller
assert not Path({str(marker)!r}).exists(), 'loading must not execute the helper'
# No real tmux, hardware commands, or vehicle paths are touched.
caller.subprocess = SimpleNamespace(run=lambda *a, **kw: SimpleNamespace(returncode=0, stdout='capture', stderr=''))
caller.os = os
caller.TMUX_LOG_PATH = {str(tmp_path / 'tmux.log')!r}
helper_path = Path({str(helper_path)!r})
if {replacement!r} == 'missing':
  helper_path.unlink()
else:
  helper_path.write_text('raise RuntimeError("different checkout helper loaded")', encoding='utf-8')
importlib.invalidate_caches()
{invoke}
{invoke}
assert Path({str(marker)!r}).read_text().splitlines() == ['startup helper', 'startup helper']
''', encoding='utf-8')
  result = subprocess.run([sys.executable, '-I', '-B', str(runner)], cwd=tmp_path,
                          capture_output=True, text=True, timeout=20)
  assert result.returncode == 0, result.stdout + result.stderr
