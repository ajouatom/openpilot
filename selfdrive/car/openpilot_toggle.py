from __future__ import annotations

import time


CRUISE_MAIN_LONG_PRESS_SECONDS = 2.0


class CruiseMainOpenpilotToggle:
  def __init__(self, main_button_type):
    self.main_button_type = main_button_type
    self._pressed_at: float | None = None

  def update(self, button_events, engaged: bool, now: float | None = None) -> bool:
    """Return True once when cruise MAIN is released after a long press while disengaged."""
    now = time.monotonic() if now is None else now

    for event in button_events:
      if event.type != self.main_button_type:
        continue

      if event.pressed:
        if self._pressed_at is None:
          self._pressed_at = now
      else:
        pressed_at = self._pressed_at
        self._pressed_at = None
        return pressed_at is not None and (now - pressed_at) >= CRUISE_MAIN_LONG_PRESS_SECONDS and not engaged

    return False
