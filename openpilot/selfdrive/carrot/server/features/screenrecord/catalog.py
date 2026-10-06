import hashlib
import os
import re
import threading
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
            entry_info = {
              "id": file_id(path),
              "name": name,
              "folder": folder,
              "size": int(stat.st_size),
              "modifiedEpoch": modified,
              "modifiedLabel": date_label(modified),
              "relativeModifiedLabel": relative_time(modified),
              "ext": os.path.splitext(name)[1].lower().lstrip("."),
            }
            start_epoch = _name_start_epoch(name)
            if start_epoch is not None:
              # Device-side start time from the file name: clients use this for
              # the recording elapsed timer so their own timezone/clock cannot
              # skew it.
              entry_info["startEpoch"] = start_epoch
              entry_info["startLabel"] = date_label(start_epoch)
            videos.append(entry_info)
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
  # Very short recordings have no frame at 1 s, so fall back to the first frame
  # instead of failing the request.
  last_error = ""
  for seek in ("1", "0"):
    if os.path.isfile(out):
      try:
        os.remove(out)
      except OSError:
        pass
    result = run_ffmpeg(["-ss", seek, "-i", path, "-vframes", "1", "-vf", "scale=320:-1", out])
    if result.returncode == 0 and os.path.isfile(out) and os.path.getsize(out) > 0:
      return out
    last_error = result.stderr or result.stdout or last_error
  raise web.HTTPInternalServerError(text=last_error or "screenrecord thumbnail generation failed")


# --- Video details (MP4/ISO-BMFF box parsing, no external dependencies) -------

_INFO_CACHE: dict[str, tuple[float, int, dict[str, Any]]] = {}
_INFO_CACHE_LOCK = threading.Lock()


def clear_video_info_cache() -> None:
  with _INFO_CACHE_LOCK:
    _INFO_CACHE.clear()


def _name_start_epoch(name: str) -> int | None:
  match = re.match(r"(\d{8})-(\d{6})", name or "")
  if not match:
    return None
  try:
    return int(datetime.strptime(match.group(1) + match.group(2), "%Y%m%d%H%M%S").timestamp())
  except Exception:
    return None


def _read_box(f, offset: int, end: int):
  if offset + 8 > end:
    return None
  f.seek(offset)
  header = f.read(8)
  if len(header) != 8:
    return None
  size = int.from_bytes(header[0:4], "big")
  box_type = header[4:8]
  header_size = 8
  if size == 1:
    extended = f.read(8)
    if len(extended) != 8:
      return None
    size = int.from_bytes(extended, "big")
    header_size = 16
  elif size == 0:
    size = end - offset
  if size < header_size:
    return None
  return box_type, offset + header_size, min(end, offset + size)


def _iter_boxes(f, start: int, end: int):
  offset = start
  for _ in range(4096):
    box = _read_box(f, offset, end)
    if box is None:
      return
    yield box
    if box[2] <= offset:
      return
    offset = box[2]


def _find_path(f, start: int, end: int, path: tuple[bytes, ...]):
  for name, payload_start, box_end in _iter_boxes(f, start, end):
    if name != path[0]:
      continue
    if len(path) == 1:
      return payload_start, box_end
    found = _find_path(f, payload_start, box_end, path[1:])
    if found:
      return found
  return None


def _read_timescale_duration(f, payload_start: int) -> tuple[int, int]:
  f.seek(payload_start)
  version = f.read(1)
  if not version:
    return 0, 0
  f.read(3)
  if version[0] == 1:
    f.read(16)
    timescale = int.from_bytes(f.read(4), "big")
    duration = int.from_bytes(f.read(8), "big")
  else:
    f.read(8)
    timescale = int.from_bytes(f.read(4), "big")
    duration = int.from_bytes(f.read(4), "big")
  return timescale, duration


def _parse_video_info(path: str, stat: os.stat_result) -> dict[str, Any]:
  info: dict[str, Any] = {
    "size": int(stat.st_size),
    "modifiedEpoch": int(stat.st_mtime),
    "modifiedLabel": date_label(int(stat.st_mtime)),
  }
  name = os.path.basename(path)
  start_epoch = _name_start_epoch(name)
  if start_epoch is not None:
    info["startEpoch"] = start_epoch
    info["startLabel"] = date_label(start_epoch)
  try:
    with open(path, "rb") as f:
      end = int(stat.st_size)
      moov = _find_path(f, 0, end, (b"moov",))
      if not moov:
        return info
      moov_start, moov_end = moov

      mvhd = _find_path(f, moov_start, moov_end, (b"mvhd",))
      movie_seconds = None
      if mvhd:
        timescale, duration = _read_timescale_duration(f, mvhd[0])
        if timescale > 0:
          movie_seconds = duration / timescale

      for box_type, track_start, track_end in _iter_boxes(f, moov_start, moov_end):
        if box_type != b"trak":
          continue
        hdlr = _find_path(f, track_start, track_end, (b"mdia", b"hdlr"))
        if not hdlr:
          continue
        f.seek(hdlr[0] + 8)  # version/flags + pre_defined, then handler_type
        if f.read(4) != b"vide":
          continue

        track_seconds = None
        mdhd = _find_path(f, track_start, track_end, (b"mdia", b"mdhd"))
        if mdhd:
          timescale, duration = _read_timescale_duration(f, mdhd[0])
          if timescale > 0:
            track_seconds = duration / timescale

        frame_count = 0
        stsz = _find_path(f, track_start, track_end, (b"mdia", b"minf", b"stbl", b"stsz"))
        if stsz:
          f.seek(stsz[0] + 4)
          f.read(4)  # sample_size
          frame_count = int.from_bytes(f.read(4), "big")

        width = height = 0
        stsd = _find_path(f, track_start, track_end, (b"mdia", b"minf", b"stbl", b"stsd"))
        if stsd:
          f.seek(stsd[0] + 40)  # version/flags + entry_count + avc1 header + VisualSampleEntry fields
          dimensions = f.read(4)
          if len(dimensions) == 4:
            width = int.from_bytes(dimensions[0:2], "big")
            height = int.from_bytes(dimensions[2:4], "big")

        seconds = track_seconds or movie_seconds
        info.update({
          "width": width,
          "height": height,
          "frameCount": frame_count,
          "durationSeconds": round(seconds, 3) if seconds else None,
          "fps": round(frame_count / seconds, 2) if (frame_count and seconds) else None,
          "codec": "H.264",
        })
        break
  except Exception:
    pass
  return info


def probe_video_info(path: str) -> dict[str, Any]:
  stat = os.stat(path)
  key = os.path.abspath(path)
  with _INFO_CACHE_LOCK:
    cached = _INFO_CACHE.get(key)
    if cached and cached[0] == stat.st_mtime and cached[1] == stat.st_size:
      return dict(cached[2])
  info = _parse_video_info(path, stat)
  with _INFO_CACHE_LOCK:
    _INFO_CACHE[key] = (stat.st_mtime, stat.st_size, dict(info))
  return info


def delete_recording(path: str) -> None:
  real = os.path.realpath(path)
  allowed = False
  for folder in SCREEN_RECORDING_DIRS:
    try:
      folder_real = os.path.realpath(folder)
    except Exception:
      continue
    if real == folder_real or real.startswith(folder_real + os.sep):
      allowed = True
      break
  if not allowed:
    raise PermissionError("path not allowed")
  os.remove(real)
  clear_video_info_cache()
