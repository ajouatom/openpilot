from types import SimpleNamespace

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
