"""Exercise retry reporting without loading the native Params extension."""
import ast
from pathlib import Path
import traceback

import pytest


def report(exc):
  source = Path(__file__).resolve().parents[1] / 'cluster_autorun.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  names = {'_is_usb_disconnect_error', '_report_cluster_failure'}
  nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names
           or isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'USB_DISCONNECT_TEXT' for t in n.targets)]
  scope = {'traceback': traceback, 'RETRY_INTERVAL_S': 5}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
  scope['_report_cluster_failure'](exc)




@pytest.mark.parametrize('cause', [ValueError('bad encoder input'), OSError('unrelated encoder IO failure')])
def test_unexpected_pipeline_failure_keeps_traceback(capsys, cause):
  try:
    raise RuntimeError('asynchronous USB JPEG pipeline failed') from cause
  except RuntimeError as exc:
    report(exc)
  captured = capsys.readouterr()
  assert 'cluster HUD failed' in captured.out
  assert 'Traceback' in captured.err and str(cause) in captured.err




def test_usb_hot_unplug_remains_concise(capsys):
  report(RuntimeError('USB display disconnected'))
  captured = capsys.readouterr()
  assert captured.out == '[cluster_autorun] USB display disconnected; retrying in 5s\n'
  assert captured.err == ''
