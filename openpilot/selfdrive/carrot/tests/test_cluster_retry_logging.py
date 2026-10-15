"""Exercise retry reporting without loading the native Params extension."""
import ast
from pathlib import Path
import traceback

import pytest


def report(exc):
  source = Path(__file__).resolve().parents[1] / 'cluster_autorun.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  names = {'_is_usb_disconnect_error', '_s3_transport_failure_reason', '_report_cluster_failure'}
  nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names
           or isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'USB_DISCONNECT_TEXT' for t in n.targets)]
  scope = {'traceback': traceback, 'RETRY_INTERVAL_S': 5}
  exec(compile(ast.Module(body=nodes, type_ignores=[]), str(source), 'exec'), scope)
  scope['_report_cluster_failure'](exc)


@pytest.mark.parametrize('cause', [TimeoutError('timed out'), ConnectionResetError('peer reset'), ConnectionError('S3 display is closed')])
def test_s3_transport_failure_is_one_line_with_cause_and_retry(capsys, cause):
  transport = RuntimeError('S3 display disconnected; reconnect required')
  transport.__cause__ = cause
  pipeline = RuntimeError('asynchronous USB JPEG pipeline failed')
  pipeline.__cause__ = transport
  report(pipeline)
  captured = capsys.readouterr()
  assert captured.out.splitlines() == [f'[cluster_autorun] S3 Wi-Fi disconnected ({type(cause).__name__}: {cause}); retrying in 5s']
  assert captured.err == ''


@pytest.mark.parametrize('cause', [ValueError('bad encoder input'), OSError('unrelated encoder IO failure')])
def test_unexpected_pipeline_failure_keeps_traceback(capsys, cause):
  try:
    raise RuntimeError('asynchronous USB JPEG pipeline failed') from cause
  except RuntimeError as exc:
    report(exc)
  captured = capsys.readouterr()
  assert 'cluster HUD failed' in captured.out
  assert 'Traceback' in captured.err and str(cause) in captured.err


def test_s3_wrapper_does_not_hide_programming_error(capsys):
  try:
    raise RuntimeError('S3 display connection failed; waiting for rediscovery') from ValueError('invalid payload')
  except RuntimeError as exc:
    report(exc)
  assert 'ValueError: invalid payload' in capsys.readouterr().err


def test_usb_hot_unplug_remains_concise(capsys):
  report(RuntimeError('USB display disconnected'))
  captured = capsys.readouterr()
  assert captured.out == '[cluster_autorun] USB display disconnected; retrying in 5s\n'
  assert captured.err == ''


def test_cyclic_chain_and_multiline_detail_are_bounded(capsys):
  transport = RuntimeError('S3 display connection failed; waiting for rediscovery')
  error = TimeoutError('timed out\n' + 'x' * 1000)
  transport.__cause__ = error
  error.__context__ = transport
  report(transport)
  captured = capsys.readouterr()
  assert len(captured.out.splitlines()) == 1 and len(captured.out) < 260
  assert captured.err == ''
