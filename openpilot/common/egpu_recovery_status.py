"""Ephemeral recovery status; READY means a tested worker, not USB presence."""
import json
import os
from pathlib import Path
import time

STATUS = Path('/dev/shm/carrot-egpu-recovery.json')
BUDGET = Path('/dev/shm/carrot-egpu-recovery-budget.json')


def write_status(value, path=STATUS):
  temporary = path.with_suffix('.tmp')
  temporary.write_text(json.dumps(value))
  os.replace(temporary, path)


def read_status(path=STATUS, now=None):
  try:
    value = json.loads(path.read_text())
    age = (time.monotonic() if now is None else now) - value['updated']
    if not 0 <= age < 3 or value['state'] not in ('waiting', 'recovering', 'ready', 'stopped'):
      return {}
    return value
  except (OSError, ValueError, TypeError, KeyError):
    return {}


def recovery_badge(value):
  return {'waiting': ('eGPU RETRY', 'loading'), 'recovering': ('eGPU RETRY', 'loading'),
          'ready': ('eGPU READY', 'ready'), 'stopped': ('eGPU ERROR', 'error')}.get(value.get('state'))
