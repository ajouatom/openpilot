import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from hud import require_usb_display, wait_for_usb_display


def test_host_switch_is_independent_but_ignition_and_preferences_are_forwarded(monkeypatch):
  import base64
  import hud
  settings = {'ClusterHud': '0', 'IsOnroad': '0', 'ClusterHudDebug': '0', 'ClusterHudBrightness': '42'}
  monkeypatch.setattr(hud, 'read_snapshot', lambda: (1., {'params': {
    key: base64.b64encode(value.encode()).decode() for key, value in settings.items()}}))
  params = hud.DisplayParams()
  assert params.get_int('ClusterHud') == 1
  assert not params.get_bool('IsOnroad') and params.get_int('ClusterHudDebug') == 0
  assert params.get_int('ClusterHudBrightness') == 42
  settings['IsOnroad'] = '1'
  params.next_read = 0.
  assert params.get_bool('IsOnroad') and params.get_int('ClusterHud') == 1


def test_absent_display_waits_until_it_arrives_without_window_fallback():
  replies = iter([None, None, 0x0092])
  sleeps = []
  expected = []
  def scan(product):
    expected.append(product)
    return next(replies)
  wait_for_usb_display(scan, 0x0092, sleeps.append)
  assert sleeps == [1., 1.] and expected == [0x0092] * 3


def test_unplug_between_scan_and_open_retries_instead_of_window_mode():
  with pytest.raises(RuntimeError, match='disappeared'):
    require_usb_display(lambda expected: None, 0x0092)
  assert require_usb_display(lambda expected: expected, 0x0092) == 0x0092
