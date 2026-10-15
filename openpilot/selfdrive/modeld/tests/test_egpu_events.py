import json

from openpilot.selfdrive.modeld.egpu_events import error_summary, log_event


def test_error_is_bounded_and_event_is_one_stderr_line(capsys):
  assert error_summary(RuntimeError('Traceback\nRuntimeError: AMD failed with 1\n')) == 'RuntimeError: AMD failed with 1'
  assert len(error_summary(ValueError('x' * 10000))) == 600
  log_event('failure', error='one\ntwo', retry=False)
  captured = capsys.readouterr()
  assert not captured.out and len(captured.err.splitlines()) == 1
  value = json.loads(captured.err.removeprefix('[eGPU] '))
  assert value['error'] == 'one\ntwo' and value['retry'] is False


def test_closed_diagnostic_stream_cannot_disable_model(monkeypatch):
  def broken_print(*args, **kwargs):
    raise BrokenPipeError('closed tmux stream')
  monkeypatch.setattr('builtins.print', broken_print)
  log_event('retry_started', attempt=1)
