from __future__ import annotations

import time


CRUISE_MAIN_LONG_PRESS_SECONDS = 2.0


class CruiseMainOpenpilotToggle:
  def __init__(self, main_button_type):
    self.main_button_type = main_button_type
    self._pressed_at: float | None = None
    self._triggered = False

  def update(self, button_events, engaged: bool, now: float | None = None) -> bool:
    """Return True once when cruise MAIN has been held long enough while disengaged."""
    now = time.monotonic() if now is None else now

    for event in button_events:
      if event.type != self.main_button_type:
        continue

      if event.pressed:
        if self._pressed_at is None:
          self._pressed_at = now
          self._triggered = False
      else:
        self._pressed_at = None
        self._triggered = False

    if self._pressed_at is not None and not self._triggered and \
       (now - self._pressed_at) >= CRUISE_MAIN_LONG_PRESS_SECONDS and not engaged:
      self._triggered = True
      return True

    return False
