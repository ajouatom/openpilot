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
  try:
    while True:
      start = time.monotonic()
      scheduler.update(params.get_bool('IsOnroad'))
      packet = builder.packet()
      if packet is not None:
        temporary = path.with_suffix('.tmp')
        temporary.write_bytes(HEADER.pack(time.monotonic()) + packet)
        os.replace(temporary, path)
      time.sleep(max(0, start + .1 - time.monotonic()))
  finally:
    builder.close()
    path.unlink(missing_ok=True)
    path.with_suffix('.tmp').unlink(missing_ok=True)


if __name__ == '__main__':
  main(Path(sys.argv[1]))
