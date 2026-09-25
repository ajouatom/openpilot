"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Powering the Jetson off on the comma's say-so.

hardwared shuts the comma down below 11.8 V or after 30 hours parked, and a
Jetson on an always-on feed draws even asleep, so it goes too. Off is off: only
a DC cycle or the button brings it back, which is why this is separate from
sleeping (see sleep.py) and only the comma's shutdown path sends it.

The container cannot power the host off, so it drops a flag file for a host-side
path unit (scripts/jetlink-poweroff.path). Not a boot loop: the host script
deletes the flag before powering off and ignores one older than the boot.
"""
from __future__ import annotations

import json
import logging
import time
from pathlib import Path

log = logging.getLogger('jetlink.power')

FLAG_NAME = 'poweroff'


def flag_path(cache_root: str | Path) -> Path:
  return Path(cache_root) / FLAG_NAME


def request_poweroff(cache_root: str | Path, reason: str = '') -> bool:
  """Leave the flag for the host. True if it was written."""
  path = flag_path(cache_root)
  try:
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps({'reason': reason, 'time': time.time()}))
    tmp.replace(path)
  except OSError as e:
    log.error("could not write %s: %s", path, e)
    return False
  log.warning("poweroff flag written to %s", path)
  return True


def clear_stale_flag(cache_root: str | Path) -> None:
  """At startup: a flag that survived a boot means the host unit is not
  installed, and one installed later must not honour it."""
  path = flag_path(cache_root)
  try:
    if path.exists():
      path.unlink()
      log.warning("removed a stale poweroff flag; is jetlink-poweroff.path installed on the host?")
  except OSError as e:
    log.error("could not remove %s: %s", path, e)
