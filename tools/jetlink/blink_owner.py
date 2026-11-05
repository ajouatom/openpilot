"""First-ready physical indicator ownership shared by comma BLE and USB sources."""
import json
import math
import os
from pathlib import Path
import secrets
import time

DIRECTORY = Path('/dev/shm/carrot-bluetooth')
GRACE = .9  # Longer than the old receiver's command lease and in-flight send.


def current(record, now):
  until = record.get('until') if isinstance(record, dict) else None
  return type(until) in (int, float) and math.isfinite(until) and 0 < until-now <= GRACE+1


class Owner:
  def __init__(self, name, directory=DIRECTORY, clock=time.monotonic):
    self.name, self.directory, self.clock = name, directory, clock
    self.token = secrets.token_hex(16)
    self.held, self.renewed = False, 0.

  def acquire(self, ready):
    if not ready:
      self.release()
      return False
    # A stalled holder must stop its policy before competing again. It cannot
    # silently resume an old intention after another adapter acquired ownership.
    if self.held and self.clock()-self.renewed >= .5:
      self.release()
      return False
    try:
      return self._update(False)
    except OSError:
      self.held = False
      return False

  def release(self):
    if self.held:
      try:
        self._update(True)
      except OSError:
        pass  # The last bounded lease expires without a renewal.
    self.held = False

  def _update(self, release):
    import fcntl
    self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
    path = self.directory/'indicator-owner.json'
    with (self.directory/'indicator-owner.lock').open('a') as lock:
      fcntl.flock(lock, fcntl.LOCK_EX)
      now = self.clock()
      try:
        record = json.loads(path.read_text())
      except (OSError, ValueError):
        record = {}
      ours = isinstance(record, dict) and record.get('token') == self.token
      if release:
        if not ours:
          return False
        value = {'name':'switching', 'token':None, 'until':now+GRACE}
      else:
        if current(record, now) and not ours:
          self.held = False
          return False
        value = {'name':self.name, 'token':self.token, 'until':now+GRACE}
      temporary = path.with_suffix('.tmp')
      temporary.write_text(json.dumps(value, separators=(',', ':')))
      os.replace(temporary, path)
      self.held, self.renewed = not release, now
      return self.held
