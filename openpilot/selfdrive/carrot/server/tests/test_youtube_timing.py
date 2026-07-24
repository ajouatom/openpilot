from __future__ import annotations

from openpilot.selfdrive.carrot.server.services.youtube_live import (
  ACTIVE_LOOP_PERIOD_SECONDS,
  IDLE_LOOP_PERIOD_SECONDS,
  _loop_delay,
)
from openpilot.selfdrive.carrot.server.services.youtube_live_muxer import _cfr_timestamp_ms


def test_active_loop_period_includes_work_time():
  assert _loop_delay(10.0, 10.0, active=True) == ACTIVE_LOOP_PERIOD_SECONDS
  assert _loop_delay(10.0, 10.004, active=True) < ACTIVE_LOOP_PERIOD_SECONDS
  assert _loop_delay(10.0, 10.020, active=True) == 0.0


def test_idle_loop_keeps_the_existing_low_poll_rate():
  assert _loop_delay(20.0, 20.0, active=False) == IDLE_LOOP_PERIOD_SECONDS
  assert round(_loop_delay(20.0, 20.1, active=False), 3) == 0.15


def test_cfr_timestamps_are_exact_and_strictly_increasing():
  timestamps = [_cfr_timestamp_ms(index, 20) for index in range(60)]

  assert timestamps[:4] == [0, 50, 100, 150]
  assert timestamps[-1] == 2_950
  assert all(current > previous for previous, current in zip(timestamps[:-1], timestamps[1:], strict=True))
