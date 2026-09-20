"""Pair current camera frames by exposure time, without assuming a fixed cadence."""

# SOF timestamps are taken in the kernel IRQ callback. EV9 2cc--10 records
# 12.6-13.2 ms skew with both camera streams complete; a 10 ms hard limit
# discarded four usable main frames. Allow bounded IRQ timing jitter while
# staying below half the nominal 50 ms camera period. Do not accept the
# 50-150 ms stale pairs observed during Ioniq camera recovery.
MAX_CAMERA_SKEW_NS = 20_000_000


class FrameMeta:
  frame_id: int = 0
  timestamp_sof: int = 0
  timestamp_eof: int = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id, self.timestamp_sof, self.timestamp_eof = vipc.frame_id, vipc.timestamp_sof, vipc.timestamp_eof


def receive_camera_pair(main, extra=None):
  """Return a fresh exposure pair, or None after a timeout/bounded resync.

  Camera intervals can alternate between 23 and 77 ms at 20 Hz. Comparing
  against the previous exposure plus half a nominal period drops valid frames.
  Instead, compare the two current exposures and advance only the older one.
  """
  main_buf = main.recv()
  main_meta = FrameMeta(main)
  if main_buf is None:
    return None
  if extra is None:
    return main_buf, main_meta, main_buf, main_meta
  extra_buf = extra.recv()
  extra_meta = FrameMeta(extra)
  for _ in range(10):
    if main_buf is None or extra_buf is None:
      return None
    delta = main_meta.timestamp_sof - extra_meta.timestamp_sof
    if abs(delta) <= MAX_CAMERA_SKEW_NS:
      return main_buf, main_meta, extra_buf, extra_meta
    if delta < 0:
      main_buf = main.recv()
      main_meta = FrameMeta(main)
    else:
      extra_buf = extra.recv()
      extra_meta = FrameMeta(extra)
  return None
