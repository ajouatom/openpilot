"""Low-priority read-only vehicle display publisher, independent of inference."""
import ctypes
import os
from pathlib import Path
import signal
import sys
import time

from hud_protocol import SnapshotBuilder, HEADER


def main(path):
  from openpilot.common.display_scheduling import DisplayScheduler
  from openpilot.common.params import Params
  ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0)
  if os.getppid() == 1:
    return
  def stop(*_):
    raise SystemExit
  signal.signal(signal.SIGTERM, stop)
  scheduler = DisplayScheduler(7, enabled=True)
  params = Params()
  builder = SnapshotBuilder()
  next_warning = 0.
  try:
    while True:
      start = time.monotonic()
      scheduler.update(params.get_bool('IsOnroad'))
      try:
        packet = builder.packet()
        if packet is not None:
          temporary = path.with_suffix('.tmp')
          temporary.write_bytes(HEADER.pack(time.monotonic()) + packet)
          os.replace(temporary, path)
      except Exception as exc:
        # A transient display/size failure expires the last snapshot. Retry in
        # this low-priority worker without disturbing the active model session.
        if start >= next_warning:
          print(f'Jetlink display snapshot unavailable: {exc}', file=sys.stderr, flush=True)
          next_warning = start + 2
      time.sleep(max(0, start + .1 - time.monotonic()))
  finally:
    builder.close()
    path.unlink(missing_ok=True)
    path.with_suffix('.tmp').unlink(missing_ok=True)


if __name__ == '__main__':
  main(Path(sys.argv[1]))
