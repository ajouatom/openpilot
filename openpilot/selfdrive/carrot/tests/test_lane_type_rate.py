# CcncLaneTypeDetection picks the StripNet rate (HDA1 CCNC lane display): mapping, frame gate, colour latch per rate
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.carrot.lane_type import marking
from openpilot.selfdrive.carrot.lane_type.marking_daemon import Recogniser, due

MS = 1_000_000


@pytest.mark.parametrize(("value", "rate", "mode"), ((0, 0, 0), (1, 5, 2), (2, 10, 2), (3, 20, 2), (4, 0, 0), (-1, 0, 0)))
def test_setting_maps_to_rate(value, rate, mode):
  params = SimpleNamespace(get_int=lambda key: value)
  assert marking.rate_of(params) == rate and marking.mode_of(params) == mode


def test_unreadable_setting_is_off():
  def bad(key):
    raise ValueError
  assert marking.rate_of(SimpleNamespace(get_int=bad)) == 0


@pytest.mark.parametrize(("rate", "processed"), ((20, 20), (10, 10), (5, 5)))
def test_due_runs_rate_frames_per_second(rate, processed):
  last, runs = None, 0
  for k in range(20):  # 1 s of modelV2 at 20 Hz with camera jitter
    stamp = k * 50 * MS + (3 * MS if k % 2 else -3 * MS)
    if due(last, stamp, rate):
      last, runs = stamp, runs + 1
  assert runs == processed


def test_due_retries_a_lost_frame_on_the_next_one():
  # 5 Hz: frame at 200 ms lost (never reached due), the 250 ms one runs instead of waiting for 400 ms
  assert not due(0, 150 * MS, 5) and due(0, 250 * MS, 5)
  assert due(None, 0, 5) and due(5_000 * MS, 0, 5)  # first frame, and a stamp going backwards


@pytest.mark.parametrize(("rate", "frames"), ((20, 10), (10, 5), (5, 3)))
def test_colour_latch_keeps_about_half_a_second(rate, frames):
  assert Recogniser(2, model=object(), lanechange='C2', rate_hz=rate).strip_filter.color_latch_frames == frames
