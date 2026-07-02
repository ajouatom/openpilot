import os
from typing import Any

from aiohttp import web

from ...config import DASHCAM_ROOT
from .paths import (
  file_size_label,
  relative_time,
  route_date_label,
  segment_index,
)


def has_source_video(segment_dir_path: str) -> bool:
  for name in ("qcamera.mp4", "qcamera.ts"):
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return True
  return False


def source_video(segment_dir_path: str) -> tuple[str, str]:
  # Prefer MP4 for browser playback, but keep TS as the canonical logger output.
  for name in ("qcamera.mp4", "qcamera.ts"):
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return path, name
  raise web.HTTPNotFound(text="qcamera video not found")


def build_routes() -> list[dict[str, Any]]:
  if not os.path.isdir(DASHCAM_ROOT):
    return []

  route_segments: dict[str, list[str]] = {}
  route_modified: dict[str, int] = {}
  with os.scandir(DASHCAM_ROOT) as it:
    for entry in it:
      try:
        if not entry.is_dir(follow_symlinks=False) or "--" not in entry.name:
          continue
        if not has_source_video(entry.path):
          continue
        parts = entry.name.split("--")
        if len(parts) < 2 or not parts[-1].isdigit():
          continue
        route = "--".join(parts[:-1])
        route_segments.setdefault(route, []).append(entry.name)
        modified = int(entry.stat(follow_symlinks=False).st_mtime)
        if modified > route_modified.get(route, 0):
          route_modified[route] = modified
      except Exception:
        continue

  routes: list[dict[str, Any]] = []
  for route, segments in route_segments.items():
    sorted_segments = sorted(segments, key=lambda s: (segment_index(s), s))
    latest = route_modified.get(route, 0)
    routes.append({
      "route": route,
      "title": route.lstrip("0") or route,
      "dateLabel": route_date_label(route),
      "segmentFolders": sorted_segments,
      "segmentCount": len(sorted_segments),
      "latestModifiedEpoch": latest,
      "latestModifiedLabel": relative_time(latest),
    })
  routes.sort(key=lambda r: (r.get("route", ""), r.get("latestModifiedEpoch", 0)), reverse=True)
  return routes


def segment_file_summary(segment_dir_path: str) -> list[dict[str, Any]]:
  out: list[dict[str, Any]] = []
  for name in ("qcamera.mp4", "qcamera.ts", "rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path):
      try:
        size = os.path.getsize(path)
      except OSError:
        size = 0
      out.append({"name": name, "size": size, "sizeLabel": file_size_label(size)})
  return out
