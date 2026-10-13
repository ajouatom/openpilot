"""Rebuild All request, consumed by launch_chffrplus.sh:consume_rebuild_request."""
import os
from pathlib import Path
import time

# Outside the repo so git reset/clean cannot drop it.
REBUILD_REQUEST_PATH = Path('/data/rebuild_all.request')


def request_full_rebuild(path: Path = REBUILD_REQUEST_PATH):
  # Only a file write: must keep working when SCons or Python deps are broken,
  # which is exactly when a full rebuild is needed.
  tmp = path.with_name(path.name + '.tmp')
  with tmp.open('w') as f:
    f.write(f'{time.monotonic()}\n')
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)
