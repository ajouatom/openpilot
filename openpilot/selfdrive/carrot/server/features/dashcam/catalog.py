import os
import re
import threading
from typing import Any

from aiohttp import web

from ...config import DASHCAM_ROOT
from .paths import (
  file_size_label,
  route_date_label,
  segment_index,
)


DASHCAM_SEGMENT_SECONDS = 60
QCAMERA_UPLOAD_NAMES = ("qcamera.ts", "qcamera.mp4")
RLOG_SOURCE_NAMES = ("rlog.zst", "rlog.bz2", "rlog")
QLOG_SOURCE_NAMES = ("qlog.zst", "qlog.bz2", "qlog")
UPLOAD_SOURCE_GROUPS = (
  ("qcamera", QCAMERA_UPLOAD_NAMES),
  ("rlog", RLOG_SOURCE_NAMES),
)
_MODERN_ROUTE_PATTERN = re.compile(r"^([0-9a-fA-F]{8})--([0-9a-fA-F]{10})$")

# A finished segment's recording end time never changes, so once we've stat'd a
# segment's qcamera file we can reuse the epoch on every later list view and
# pagination step (0 disk touches on repeat). The cache is dropped whenever the
# route set changes (a segment added/removed) via invalidate_segment_time_cache,
# which the route cache calls on every rebuild — that also clears the entry for
# the one segment that was still recording when first seen. Missing/empty videos
# (epoch <= 0) are never cached so a just-created segment is re-checked cheaply.
_end_epoch_cache: dict[str, int] = {}
_end_epoch_cache_lock = threading.Lock()


def invalidate_segment_time_cache() -> None:
  with _end_epoch_cache_lock:
    _end_epoch_cache.clear()


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


def source_rlog(segment_dir_path: str) -> tuple[str, str]:
  """Return the canonical recorded log without parsing or decompressing it."""
  for name in RLOG_SOURCE_NAMES:
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return path, name
  raise web.HTTPNotFound(text="rlog not found")


def source_qlog(segment_dir_path: str) -> tuple[str, str]:
  """Return the reduced-frequency recorded log without parsing it."""
  for name in QLOG_SOURCE_NAMES:
    path = os.path.join(segment_dir_path, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return path, name
  raise web.HTTPNotFound(text="qlog not found")


def route_creation_key(route: str) -> tuple[int, int, str, str]:
  """Stable logger creation order for modern and legacy route identifiers.

  Modern loggerd routes start with an eight-digit hexadecimal RouteCount and a
  random suffix (for example ``0000000d--c4ed939d0d``). RouteCount, not the
  random suffix or a filesystem timestamp, is the creation sequence. Legacy
  date-based identifiers retain lexical chronological ordering.
  """
  value = str(route or "")
  match = _MODERN_ROUTE_PATTERN.fullmatch(value)
  if match:
    return (1, int(match.group(1), 16), "", value)
  return (0, 0, value, value)


def segment_creation_key(segment: str) -> tuple[int, int, str, str, int, str]:
  parts = str(segment or "").split("--")
  route = "--".join(parts[:-1]) if len(parts) >= 2 else str(segment or "")
  return (*route_creation_key(route), segment_index(segment), str(segment or ""))


def segment_is_complete(segment: str) -> bool:
  """Return true only for an unlocked segment with a finalized log source."""
  segment_path = os.path.join(DASHCAM_ROOT, str(segment or ""))
  has_rlog = False
  try:
    with os.scandir(segment_path) as entries:
      for entry in entries:
        if entry.name.endswith(".lock"):
          return False
        if entry.name not in RLOG_SOURCE_NAMES:
          continue
        try:
          has_rlog = entry.is_file(follow_symlinks=False) and entry.stat(follow_symlinks=False).st_size > 0
        except OSError:
          continue
  except OSError:
    return False
  return has_rlog


def build_routes() -> list[dict[str, Any]]:
  """Enumerate routes/segments from the top-level directory only.

  This is deliberately a NAME-ONLY index: it never stat()s the per-segment
  qcamera files. On the device the recording directory can hold thousands of
  1-minute segment folders, and stat-ing every segment just to list routes cost
  one syscall per segment (seconds on eMMC/NVMe). Segment recording times are
  hydrated lazily for the visible page only (see compute_segment_times /
  route_time_bounds, consumed by routes.route_with_segment_page).

  Modern route folders encode loggerd's increasing RouteCount; older folders
  encode their date. Both are ordered by route_creation_key without touching
  per-segment timestamps.
  """
  if not os.path.isdir(DASHCAM_ROOT):
    return []

  route_segments: dict[str, list[str]] = {}
  with os.scandir(DASHCAM_ROOT) as it:
    for entry in it:
      try:
        # entry.is_dir() uses the readdir d_type on Linux, so this stays a pure
        # directory read with no extra stat syscall per segment.
        if not entry.is_dir(follow_symlinks=False) or "--" not in entry.name:
          continue
        parts = entry.name.split("--")
        if len(parts) < 2 or not parts[-1].isdigit():
          continue
        route = "--".join(parts[:-1])
        route_segments.setdefault(route, []).append(entry.name)
      except Exception:
        continue

  routes: list[dict[str, Any]] = []
  for route, segments in route_segments.items():
    sorted_segments = sorted(segments, key=lambda s: (segment_index(s), s))
    routes.append({
      "route": route,
      "title": route.lstrip("0") or route,
      "dateLabel": route_date_label(route),
      "segmentFolders": sorted_segments,
      "segmentCount": len(sorted_segments),
    })
  routes.sort(key=lambda r: route_creation_key(r.get("route", "")), reverse=True)
  return routes


def _segment_end_epoch_by_name(segment_name: str) -> int:
  """Recording end epoch for a single segment. Memoized: a real recording time is
  cached and reused until the route set changes, so repeat list views/pagination
  cost no disk. Only positive epochs are cached (a still-recording or missing
  video keeps getting re-checked)."""
  with _end_epoch_cache_lock:
    cached = _end_epoch_cache.get(segment_name)
  if cached is not None:
    return cached
  epoch = source_video_end_epoch(os.path.join(DASHCAM_ROOT, segment_name))
  if epoch > 0:
    with _end_epoch_cache_lock:
      _end_epoch_cache[segment_name] = epoch
  return epoch


def compute_segment_times(
  segments_asc: list[str],
  seed_segment: str | None = None,
) -> dict[str, dict[str, int]]:
  """Build {segment: {startEpoch, endEpoch}} for the given ascending-ordered
  segments, stat-ing ONLY those segments (plus an optional preceding seed so the
  contiguity chain survives a page boundary). Cost is O(len(segments)) stats,
  independent of how much footage is stored on the device.

  Segments whose video file is missing or empty (end epoch <= 0, e.g. a segment
  still being recorded) are simply omitted from the result; callers already
  tolerate a missing time entry.
  """
  times: dict[str, dict[str, int]] = {}
  previous_index: int | None = None
  previous_end = 0

  if seed_segment:
    seed_end = _segment_end_epoch_by_name(seed_segment)
    if seed_end > 0:
      previous_index = segment_index(seed_segment)
      previous_end = seed_end

  for name in segments_asc:
    index = segment_index(name)
    end_epoch = _segment_end_epoch_by_name(name)
    if end_epoch <= 0:
      previous_index = None
      previous_end = 0
      continue
    contiguous = (
      previous_end > 0
      and end_epoch >= previous_end
      and previous_index is not None
      and index == previous_index + 1
    )
    start_epoch = previous_end if contiguous else end_epoch - DASHCAM_SEGMENT_SECONDS
    start_epoch = max(0, min(start_epoch, end_epoch))
    times[name] = {"startEpoch": start_epoch, "endEpoch": end_epoch}
    previous_index = index
    previous_end = end_epoch
  return times


def route_time_bounds(segments_asc: list[str]) -> tuple[int, int]:
  """(routeStartEpoch, routeEndEpoch) for a route using only its first and last
  segment — two stats per route regardless of segment count. The first segment
  is never contiguous, so its start is approximated as end - 60s (same rule the
  full chain uses for a non-contiguous head)."""
  if not segments_asc:
    return 0, 0
  first = segments_asc[0]
  last = segments_asc[-1]
  first_end = _segment_end_epoch_by_name(first)
  last_end = first_end if last == first else _segment_end_epoch_by_name(last)
  route_start = max(0, first_end - DASHCAM_SEGMENT_SECONDS) if first_end > 0 else 0
  route_end = last_end if last_end > 0 else 0
  return route_start, route_end


def segment_file_summary(segment_dir_path: str) -> list[dict[str, Any]]:
  """Return the original files selected for a segment upload.

  Upload exactly one qcamera source and one rlog source. Prefer logger output
  over browser-oriented derivatives and never include reduced or auxiliary
  artifacts. An rlog is required because the uploaded segment cannot be
  analyzed without it; qcamera is optional so log-only segments remain useful.
  """
  out: list[dict[str, Any]] = []
  for kind, names in UPLOAD_SOURCE_GROUPS:
    for name in names:
      path = os.path.join(segment_dir_path, name)
      try:
        if not os.path.isfile(path):
          continue
        size = os.path.getsize(path)
      except OSError:
        continue
      if size <= 0:
        continue
      out.append({
        "kind": kind,
        "name": name,
        "size": size,
        "sizeLabel": file_size_label(size),
      })
      break
  if not any(item["kind"] == "rlog" for item in out):
    raise web.HTTPNotFound(text="rlog not found")
  return out
