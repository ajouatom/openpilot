"""One small tmpfs image shared by Web and USB display (never model input)."""
import os
import json
from pathlib import Path
import time

PREVIEW_PATH = Path('/dev/shm/carrot_nav_map.png')
PREVIEW_MAX_AGE = 2.
WEB_DEMAND_PATH = Path('/dev/shm/carrot_nav_web')
STATUS_PATH = Path('/dev/shm/carrot_nav_status.json')


def preview_fresh(path=PREVIEW_PATH):
  try:
    return 0 <= time.time() - path.stat().st_mtime < PREVIEW_MAX_AGE  # noqa: TID251 - filesystem mtime is wall clock
  except OSError:
    return False


def write_preview(image, path=PREVIEW_PATH, *, reason='ready'):
  from PIL import Image
  temporary = path.with_suffix('.tmp')
  Image.fromarray(image).save(temporary, format='PNG', compress_level=1)
  os.replace(temporary, path)
  status = path.with_name(STATUS_PATH.name)
  temporary = status.with_suffix('.tmp')
  temporary.write_text(json.dumps({'reason': reason}), encoding='utf-8')
  os.replace(temporary, status)


def preview_reason():
  try:
    return str(json.loads(STATUS_PATH.read_text(encoding='utf-8'))['reason'])
  except (OSError, ValueError, KeyError):
    return 'waiting'
