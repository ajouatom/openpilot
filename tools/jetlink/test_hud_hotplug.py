import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from hud import require_usb_display, wait_for_usb_display


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
