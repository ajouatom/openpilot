import hashlib
import os
import time

from aiohttp import web

from ...config import DASHCAM_ROOT, DASHCAM_CACHE_DIR


def safe_segment(segment: str) -> str:
  segment = (segment or "").strip()
  if not segment or "/" in segment or "\\" in segment or segment in {".", ".."}:
    raise web.HTTPBadRequest(text="bad segment")
  parts = segment.split("--")
  if len(parts) < 2 or not parts[-1].isdigit():
    raise web.HTTPBadRequest(text="bad segment")
  return segment


def segment_index(segment: str) -> int:
  try:
    return int(segment.split("--")[-1])
  except Exception:
    return 0


def route_name(segment: str) -> str:
  try:
    return "--".join(str(segment or "").split("--")[:-1])
  except Exception:
    return str(segment or "")


def file_size_label(size: int) -> str:
  try:
    n = float(size)
  except Exception:
    return "-"
  if n < 1024:
    return f"{int(n)} B"
  if n < 1024 * 1024:
    return f"{n / 1024:.1f} KB"
  if n < 1024 * 1024 * 1024:
    return f"{n / (1024 * 1024):.1f} MB"
  return f"{n / (1024 * 1024 * 1024):.1f} GB"


def segment_dir(segment: str) -> str:
  segment = safe_segment(segment)
  root = os.path.abspath(DASHCAM_ROOT)
  path = os.path.abspath(os.path.join(root, segment))
  if not path.startswith(root + os.sep):
    raise web.HTTPBadRequest(text="bad segment path")
  if not os.path.isdir(path):
    raise web.HTTPNotFound(text="segment not found")
  return path


def cache_path(kind: str, segment: str, ext: str) -> str:
  token = hashlib.sha1(segment.encode("utf-8", errors="ignore")).hexdigest()[:24]
  directory = os.path.join(DASHCAM_CACHE_DIR, kind)
  os.makedirs(directory, exist_ok=True)
  return os.path.join(directory, f"{token}{ext}")


def route_date_label(route: str) -> str:
  try:
    if "-" in route and "--" in route:
      parts = route.split("--")
      if len(parts) >= 2:
        date = parts[0]
        t = parts[1].split("-")
        if len(t) >= 2:
          return f"{date} {t[0]}:{t[1]}"
        return date
    compact = route.split("--")
    if len(compact) >= 2:
      raw_date, raw_time = compact[0], compact[1]
      if len(raw_date) >= 8 and len(raw_time) >= 4:
        return f"{raw_date[:4]}-{raw_date[4:6]}-{raw_date[6:8]} {raw_time[:2]}:{raw_time[2:4]}"
  except Exception:
    pass
  return route


def relative_time(epoch_seconds: int) -> str:
  if epoch_seconds <= 0:
    return "-"
  delta = max(0, int(time.time()) - int(epoch_seconds))
  if delta < 60:
    return "방금 전"
  if delta < 3600:
    return f"{delta // 60}분 전"
  if delta < 86400:
    return f"{delta // 3600}시간 전"
  return f"{delta // 86400}일 전"
