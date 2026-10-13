import json

import pytest

from openpilot.common.startup_touch_log import TouchLog


def test_touch_records_survive_reopen_and_rotate(tmp_path):
  path = tmp_path / 'touch.jsonl'
  logger = TouchLog(path, max_bytes=1)
  assert logger.record('press', sync=True, x=143, y=207, hit='rebuild')
  assert TouchLog(path, max_bytes=1).record('reboot', sync=True, request_exists=True)
  assert json.loads(path.read_text())['request_exists']
  previous = json.loads(path.with_suffix('.jsonl.1').read_text())
  assert (previous['x'], previous['y'], previous['hit']) == (143, 207, 'rebuild')


def test_logging_failure_does_not_block_recovery(tmp_path):
  path = tmp_path / 'directory'
  path.mkdir()
  assert not TouchLog(path).record('reboot', sync=True)


@pytest.mark.parametrize('backend,expected', [('comma', 1), ('desktop', 0), ('', 0)])
def test_poll_loaded_backend_independently_of_evdev(monkeypatch, backend, expected):
  # Exercise the input poll helper without opening a native window.
  from openpilot.system.ui import startup_recovery as ui
  calls = []
  monkeypatch.setattr(ui, 'RAYLIB_BACKEND', backend)
  monkeypatch.setattr(ui.rl, 'poll_input_events', lambda: calls.append('poll'))
  ui.poll_recovery_input()
  assert calls == ['poll'] * expected
