"""DM pacing without accumulated catch-up after camera poll timeouts."""
import time


class DmRatekeeper:
  def __init__(self, interval):
    self.frame = 0
    self._interval = interval
    self._next_frame_time = time.monotonic() + interval

  def keep_time(self):
    now = time.monotonic()
    if now < self._next_frame_time:
      time.sleep(self._next_frame_time - now)
    # A missing camera already consumed a whole poll timeout, plus processing.
    # Discard missed deadlines instead of accumulating debt for a later burst.
    # Include sleep overshoot so scheduler delays cannot accumulate either.
    self._next_frame_time = max(self._next_frame_time, time.monotonic()) + self._interval
    self.frame += 1
