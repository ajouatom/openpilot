from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld.camera_sync import receive_camera_pair


class Camera:
  def __init__(self, frames):
    self.frames = iter(frames)
    self.frame_id = self.timestamp_sof = self.timestamp_eof = 0

  def recv(self):
    frame = next(self.frames, None)
    if frame is None:
      return None
    self.frame_id, self.timestamp_sof = frame
    self.timestamp_eof = self.timestamp_sof + 1_000_000
    return SimpleNamespace(frame_id=self.frame_id)


def test_alternating_exposure_intervals_do_not_drop_valid_pairs():
  # Ioniq 5 f79: short/long exposure spacing must still produce every pair.
  stamps = [1_000_000_000]
  for interval in [23_107_000, 77_653_000, 22_292_000, 75_255_000, 24_781_000, 75_103_000, 26_025_000]:
    stamps.append(stamps[-1] + interval)
  main = Camera(enumerate(stamps))
  extra = Camera((i, t + 100_000) for i, t in enumerate(stamps))
  for i in range(len(stamps)):
    buf, meta, wide, wide_meta = receive_camera_pair(main, extra)
    assert buf.frame_id == meta.frame_id == wide.frame_id == wide_meta.frame_id == i


def test_current_pair_resync_advances_only_older_camera():
  main = Camera([(10, 100_000_000), (11, 150_000_000)])
  extra = Camera([(20, 50_000_000), (21, 150_000_000)])
  _, meta, _, wide_meta = receive_camera_pair(main, extra)
  assert meta.frame_id == 11 and wide_meta.frame_id == 21


def test_timeout_never_reuses_a_stale_buffer():
  main = Camera([(0, 100_000_000)])
  extra = Camera([(0, 0)])
  assert receive_camera_pair(main, extra) is None
  assert receive_camera_pair(main, extra) is None


def test_single_camera_accepts_each_new_frame():
  main = Camera([(0, 100), (1, 200)])
  for i in range(2):
    buf, meta, extra, extra_meta = receive_camera_pair(main)
    assert buf is extra and meta is extra_meta and meta.frame_id == i


def test_resync_is_bounded_even_if_timestamps_never_match():
  main = Camera((i, i * 100_000_000) for i in range(100))
  extra = Camera((i, i * 100_000_000 + 50_000_000) for i in range(100))
  assert receive_camera_pair(main, extra) is None
  assert main.frame_id + extra.frame_id <= 10


@pytest.mark.parametrize('frame_id,main_sof,wide_sof', [
  (12212, 688_852_340_000, 688_865_358_000),
  (12232, 689_852_248_000, 689_864_863_000),
  (12266, 691_552_169_000, 691_565_355_000),
  (12793, 717_914_040_000, 717_927_115_000),
])
def test_ev9_complete_camera_streams_do_not_create_model_frame_gaps(frame_id, main_sof, wide_sof):
  # EV9 000002cc--03d0a44f7d--10: the old 10 ms limit skipped these
  # main frames, publishing invalid odometry despite complete camera streams.
  main = Camera([(frame_id - 1, main_sof - 50_000_000), (frame_id, main_sof),
                 (frame_id + 1, main_sof + 50_000_000)])
  wide = Camera([(frame_id, main_sof - 50_000_000), (frame_id + 1, wide_sof),
                 (frame_id + 2, main_sof + 50_000_000)])
  selected = [receive_camera_pair(main, wide)[1].frame_id for _ in range(3)]
  assert selected == [frame_id - 1, frame_id, frame_id + 1]


@pytest.mark.parametrize('skew', [-20_000_000, 20_000_000])
def test_small_skew_is_symmetric_and_bounded(skew):
  main = Camera([(1, 100_000_000)])
  wide = Camera([(1, 100_000_000 + skew)])
  assert receive_camera_pair(main, wide) is not None


@pytest.mark.parametrize('skew', [-20_000_001, 20_000_001])
def test_outside_skew_limit_requires_a_fresh_frame(skew):
  main = Camera([(1, 100_000_000)])
  wide = Camera([(1, 100_000_000 + skew)])
  assert receive_camera_pair(main, wide) is None


@pytest.mark.parametrize('missing', [1, 3])
def test_missing_camera_frames_still_create_a_real_gap(missing):
  # Ioniq f8c/f8d: phase slip / IFE recovery cannot be made valid by
  # accepting a camera pair separated by one or more complete periods.
  main = Camera((i, 1_000_000_000 + i * 50_000_000) for i in range(missing + 3))
  wide = Camera((i, 1_000_000_000 + i * 50_000_000) for i in [0, missing + 1, missing + 2])
  selected = [receive_camera_pair(main, wide)[1].frame_id for _ in range(3)]
  assert selected == [0, missing + 1, missing + 2]
