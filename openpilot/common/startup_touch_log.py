"""Rotate recovery input records; logging failures must not block recovery."""
import json
import os
from pathlib import Path
import time
import threading

class TouchLog:
  def __init__(self, path, max_bytes=262144):
    self._lock = threading.Lock()
    self.path = Path(path)
    self.max_bytes = max_bytes
    try:
      self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
      self.boot_id = ''

  def record(self, event, sync=False, **fields):
    with self._lock:
      return self._record(event, sync, fields)

  def _record(self, event, sync, fields):
    try:
      self.path.parent.mkdir(parents=True, exist_ok=True)
      if self.path.exists() and self.path.stat().st_size >= self.max_bytes:
        os.replace(self.path, self.path.with_suffix(self.path.suffix + '.1'))
      with self.path.open('a') as output:
        output.write(json.dumps(dict(event=event, time=time.time(), monotonic=time.monotonic(), boot_id=self.boot_id, **fields)) + '\n')  # noqa: TID251 -- wall time correlates diagnostics across boots
        if sync:
          output.flush()
          os.fsync(output.fileno())
      return True
    except (OSError, ValueError, TypeError):
      return False
