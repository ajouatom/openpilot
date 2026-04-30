import hashlib
import os
from datetime import datetime
from typing import Any

from aiohttp import web

from ...config import SCREEN_RECORDING_DIRS, SCREEN_RECORDING_EXTS
from ..dashcam.ffmpeg import run_ffmpeg
from ..dashcam.paths import cache_path, relative_time


def file_id(path: str) -> str:
  return hashlib.sha1(os.path.abspath(path).encode("utf-8", errors="ignore")).hexdigest()[:24]


def date_label(epoch_seconds: int) -> str:
  try:
    return datetime.fromtimestamp(epoch_seconds).strftime("%Y-%m-%d %H:%M")
  except Exception:
    return "-"


def build_videos() -> list[dict[str, Any]]:
  videos: list[dict[str, Any]] = []
  seen: set[str] = set()
  for folder in SCREEN_RECORDING_DIRS:
    if not os.path.isdir(folder):
      continue
    try:
      with os.scandir(folder) as it:
        for entry in it:
          try:
            name = entry.name
            if not entry.is_file(follow_symlinks=False):
              continue
            if not name.lower().endswith(SCREEN_RECORDING_EXTS):
              continue
            stat = entry.stat(follow_symlinks=False)
            if stat.st_size <= 0:
              continue
            path = os.path.abspath(entry.path)
            real = os.path.realpath(path)
            if real in seen:
              continue
            seen.add(real)
            modified = int(stat.st_mtime)
            videos.append({
              "id": file_id(path),
              "name": name,
              "folder": folder,
              "size": int(stat.st_size),
              "modifiedEpoch": modified,
              "modifiedLabel": date_label(modified),
              "relativeModifiedLabel": relative_time(modified),
              "ext": os.path.splitext(name)[1].lower().lstrip("."),
            })
          except Exception:
            continue
    except Exception:
      continue
  videos.sort(key=lambda item: (item.get("modifiedEpoch", 0), item.get("name", "")), reverse=True)
  return videos


def find_file(file_id_in: str) -> str:
  file_id_in = (file_id_in or "").strip()
  if not file_id_in or "/" in file_id_in or "\\" in file_id_in or len(file_id_in) > 64:
    raise web.HTTPBadRequest(text="bad file id")
  for item in build_videos():
    folder = str(item.get("folder") or "")
    name = str(item.get("name") or "")
    path = os.path.abspath(os.path.join(folder, name))
    if file_id(path) == file_id_in and os.path.isfile(path):
      return path
  raise web.HTTPNotFound(text="screen recording not found")


def thumbnail_path(file_id_in: str) -> str:
  path = find_file(file_id_in)
  out = cache_path("screen_thumb", file_id_in, ".jpg")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  result = run_ffmpeg(["-ss", "1", "-i", path, "-vframes", "1", "-vf", "scale=320:-1", out])
  if result.returncode != 0 or not os.path.isfile(out) or os.path.getsize(out) <= 0:
    raise web.HTTPInternalServerError(text=result.stderr or result.stdout or "screenrecord thumbnail generation failed")
  return out
