import unittest
from types import SimpleNamespace

from selfdrive.car.openpilot_toggle import CRUISE_MAIN_LONG_PRESS_SECONDS, CruiseMainOpenpilotToggle


MAIN_BUTTON = "mainCruise"
CANCEL_BUTTON = "cancel"


def button_event(pressed: bool, button_type=MAIN_BUTTON):
  return SimpleNamespace(pressed=pressed, type=button_type)


class TestCruiseMainOpenpilotToggle(unittest.TestCase):
  def test_long_press_triggers_at_threshold(self):
    toggle = CruiseMainOpenpilotToggle(MAIN_BUTTON)

    self.assertFalse(toggle.update([button_event(True)], engaged=False, now=1.0))
    self.assertTrue(toggle.update([], engaged=False, now=1.0 + CRUISE_MAIN_LONG_PRESS_SECONDS))
    self.assertFalse(toggle.update([], engaged=False, now=10.0))
    self.assertFalse(toggle.update([button_event(False)], engaged=False, now=10.1))

  def test_short_press_does_not_trigger(self):
    toggle = CruiseMainOpenpilotToggle(MAIN_BUTTON)

    self.assertFalse(toggle.update([button_event(True)], engaged=False, now=1.0))
    self.assertFalse(toggle.update([button_event(False)], engaged=False, now=1.5))

  def test_long_press_is_blocked_while_engaged(self):
    toggle = CruiseMainOpenpilotToggle(MAIN_BUTTON)

    self.assertFalse(toggle.update([button_event(True)], engaged=True, now=1.0))
    self.assertFalse(toggle.update([], engaged=True, now=1.1 + CRUISE_MAIN_LONG_PRESS_SECONDS))
    self.assertFalse(toggle.update([button_event(False)], engaged=True, now=10.0))

  def test_other_buttons_are_ignored(self):
    toggle = CruiseMainOpenpilotToggle(MAIN_BUTTON)

    self.assertFalse(toggle.update([button_event(True, CANCEL_BUTTON)], engaged=False, now=1.0))
    self.assertFalse(toggle.update([button_event(False, CANCEL_BUTTON)], engaged=False, now=10.0))


if __name__ == "__main__":
  unittest.main()
