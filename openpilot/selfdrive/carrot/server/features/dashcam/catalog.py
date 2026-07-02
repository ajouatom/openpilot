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


DASHCAM_SEGMENT_SECONDS = 60


def source_video_end_epoch(segment_dir_path: str) -> int:
  """Return the recording file's latest write time.

  qcamera.ts is the canonical logger output. Prefer it over a later-created MP4
  so browser conversion or other post-processing does not move the timestamp.
  """
  for name in ("qcamera.ts", "qcamera.mp4"):
    path = os.path.join(segment_dir_path, name)
    try:
      if os.path.isfile(path) and os.path.getsize(path) > 0:
        return int(os.stat(path, follow_symlinks=False).st_mtime)
    except OSError:
      continue
  return 0


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
  route_end_epochs: dict[str, dict[str, int]] = {}
  with os.scandir(DASHCAM_ROOT) as it:
    for entry in it:
      try:
        if not entry.is_dir(follow_symlinks=False) or "--" not in entry.name:
          continue
        parts = entry.name.split("--")
        if len(parts) < 2 or not parts[-1].isdigit():
          continue
        end_epoch = source_video_end_epoch(entry.path)
        if end_epoch <= 0:
          continue
        route = "--".join(parts[:-1])
        route_segments.setdefault(route, []).append(entry.name)
        route_end_epochs.setdefault(route, {})[entry.name] = end_epoch
      except Exception:
        continue

  routes: list[dict[str, Any]] = []
  for route, segments in route_segments.items():
    sorted_segments = sorted(segments, key=lambda s: (segment_index(s), s))
    end_epochs = route_end_epochs.get(route, {})
    segment_times: dict[str, dict[str, int]] = {}
    previous_index: int | None = None
    previous_end = 0
    for segment in sorted_segments:
      index = segment_index(segment)
      end_epoch = int(end_epochs.get(segment) or 0)
      if end_epoch <= 0:
        continue
      contiguous = (
        previous_end > 0
        and end_epoch >= previous_end
        and previous_index is not None
        and index == previous_index + 1
      )
      start_epoch = previous_end if contiguous else end_epoch - DASHCAM_SEGMENT_SECONDS
      start_epoch = max(0, min(start_epoch, end_epoch))
      segment_times[segment] = {"startEpoch": start_epoch, "endEpoch": end_epoch}
      previous_index = index
      previous_end = end_epoch

    first_time = segment_times.get(sorted_segments[0], {}) if sorted_segments else {}
    last_time = segment_times.get(sorted_segments[-1], {}) if sorted_segments else {}
    route_start = int(first_time.get("startEpoch") or 0)
    route_end = int(last_time.get("endEpoch") or 0)
    routes.append({
      "route": route,
      "title": route.lstrip("0") or route,
      "dateLabel": route_date_label(route),
      "segmentFolders": sorted_segments,
      "segmentCount": len(sorted_segments),
      "segmentTimes": segment_times,
      "routeStartEpoch": route_start,
      "routeEndEpoch": route_end,
      "latestModifiedEpoch": route_end,
      "latestModifiedLabel": relative_time(route_end),
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
