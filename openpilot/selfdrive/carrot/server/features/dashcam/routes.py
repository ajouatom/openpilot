import asyncio
import mimetypes
import os
import threading
import time
from urllib.parse import quote

from aiohttp import web

from openpilot.selfdrive.carrot.web_upload import check_web_upload_health, create_web_upload_session

from ...config import DASHCAM_ROOT
from . import upload, upload_jobs
from .catalog import (
  build_routes,
  compute_segment_times,
  invalidate_segment_time_cache,
  route_time_bounds,
  segment_file_summary,
  segment_is_complete,
  source_qlog,
  source_rlog,
  source_video,
)
from .ffmpeg import browser_video, ensure_preview, ensure_thumbnail
from .report import build_route_report
from .read_state import read_dashcam_read_state, write_dashcam_recent_segment
from .summary_sources import build_route_summary_source
from .paths import (
  file_size_label,
  relative_time,
  route_name,
  safe_segment,
  segment_dir,
  segment_index,
)

# Safety net only. The route-name index is normally invalidated precisely by the
# recording directory's mtime (adding/removing a segment folder bumps realdata's
# mtime), so fresh segments show up immediately and an idle logs view never
# rescans. This max age just forces an eventual rebuild on the rare filesystem
# that fails to update a directory mtime.
ROUTE_CACHE_MAX_AGE = 300.0
DASHCAM_ROUTE_LIMIT_DEFAULT = 40
DASHCAM_ROUTE_LIMIT_MAX = 200
DASHCAM_SEGMENT_LIMIT_DEFAULT = 10
DASHCAM_SEGMENT_LIMIT_MAX = 2000
DASHCAM_OFFSET_MAX = 1000000
DASHCAM_RECENT_UPLOAD_LIMITS = frozenset((2, 5, 10))
_route_cache_lock = threading.Lock()
# "sig" starts as a unique sentinel so it never equals a real signature tuple and
# the first call always builds. "routes" stays None until populated so an empty
# index (no footage) is still cached instead of rebuilt every request.
_route_cache: dict = {"time": 0.0, "sig": object(), "routes": None}


def client_replay_source_description(segment: str) -> dict:
  """Describe raw replay inputs without running replay preparation or ffmpeg."""
  segment = safe_segment(segment)
  segment_path = segment_dir(segment)
  rlog_path, rlog_name = source_rlog(segment_path)
  video_path, video_name = source_video(segment_path)
  encoded_segment = quote(segment, safe="")
  return {
    "ok": True,
    "mode": "client",
    "segment": segment,
    "segmentIndex": segment_index(segment),
    "rlog": {
      "name": rlog_name,
      "size": os.path.getsize(rlog_path),
      "compression": "zstd" if rlog_name.endswith(".zst") else ("bzip2" if rlog_name.endswith(".bz2") else "none"),
      "url": f"/api/dashcam/replay-source/{encoded_segment}/rlog",
    },
    "video": {
      "name": video_name,
      "size": os.path.getsize(video_path),
      "container": "mp4" if video_name.endswith(".mp4") else "mpegts",
      "url": f"/api/dashcam/replay-source/{encoded_segment}/video",
    },
  }


async def request_upload_segments(request: web.Request) -> list[str]:
  try:
    body = await request.json()
  except Exception:
    body = {}
  segments = body.get("segments")
  if not isinstance(segments, list):
    one = body.get("segment")
    segments = [one] if one else []
  segments = [safe_segment(str(seg)) for seg in segments if seg]
  if not segments:
    raise web.HTTPBadRequest(text="missing segments")
  incomplete = await asyncio.to_thread(
    lambda: [segment for segment in segments if not segment_is_complete(segment)]
  )
  if incomplete:
    raise web.HTTPConflict(text=f"segment is still recording or incomplete: {incomplete[0]}")
  return segments


def _realdata_signature() -> tuple | None:
  """Cheap change token for the recording directory (one stat). It changes
  whenever a segment folder is added or removed, because POSIX updates a
  directory's mtime when its entries change. Writes to files *inside* a segment
  do not affect it, which is exactly right: the index only holds folder names,
  and per-segment times are hydrated live on every request."""
  try:
    st = os.stat(DASHCAM_ROOT)
  except OSError:
    return None
  return (st.st_mtime_ns, st.st_size)


def cached_dashcam_routes() -> list[dict]:
  sig = _realdata_signature()
  now = time.monotonic()
  with _route_cache_lock:
    cached = _route_cache.get("routes")
    fresh = (now - float(_route_cache.get("time") or 0.0)) < ROUTE_CACHE_MAX_AGE
    if cached is not None and fresh and sig == _route_cache.get("sig"):
      return list(cached)

  # The segment set changed (or first build), so any memoized end epochs for the
  # previous state — including the segment that was still recording last time —
  # must be dropped before the fresh page hydration re-stats what it needs.
  invalidate_segment_time_cache()
  routes = build_routes()
  with _route_cache_lock:
    _route_cache["time"] = time.monotonic()
    _route_cache["sig"] = sig
    _route_cache["routes"] = routes
  return list(routes)


def _newest_first_segments(routes: list[dict]):
  for entry in routes:
    yield from reversed(entry.get("segmentFolders") or [])


def visible_dashcam_routes() -> list[dict]:
  """Return the cached catalog without the currently-writing tail.

  Completeness is deliberately evaluated outside the five-minute name cache:
  removing rlog.lock changes the segment directory, not the realdata root, so a
  completed segment must become visible on the next frontend refresh.
  """
  routes = cached_dashcam_routes()
  hidden: set[str] = set()
  for segment in _newest_first_segments(routes):
    if segment_is_complete(segment):
      break
    hidden.add(segment)

  if not hidden:
    return routes

  visible: list[dict] = []
  for entry in routes:
    segments = [segment for segment in entry.get("segmentFolders") or [] if segment not in hidden]
    if not segments:
      continue
    item = dict(entry)
    item["segmentFolders"] = segments
    item["segmentCount"] = len(segments)
    visible.append(item)
  return visible


def recent_completed_dashcam_segments(limit: int) -> list[str]:
  """Newest completed segments across route groups, independent of UI sort."""
  count = int(limit)
  if count not in DASHCAM_RECENT_UPLOAD_LIMITS:
    raise ValueError("unsupported recent log limit")

  completed: list[str] = []
  for segment in _newest_first_segments(cached_dashcam_routes()):
    if not segment_is_complete(segment):
      continue
    completed.append(segment)
    if len(completed) >= count:
      break
  return completed


def bounded_query_int(request: web.Request, name: str, default: int, maximum: int) -> int:
  try:
    value = int(request.query.get(name, str(default)) or default)
  except (TypeError, ValueError):
    value = default
  return max(0 if name == "offset" else 1, min(maximum, value))


def normalized_sort(request: web.Request) -> str:
  value = (request.query.get("sort") or "asc").strip().lower()
  return "desc" if value == "desc" else "asc"


def route_with_segment_page(entry: dict, segment_offset: int = 0, segment_limit: int = DASHCAM_SEGMENT_LIMIT_DEFAULT, sort: str = "asc") -> dict:
  # NOTE: this stat()s the qcamera files for the page's segments (plus the route
  # head/tail for the time range), so it must run off the event loop — callers
  # invoke it inside asyncio.to_thread.
  segments_asc = list(entry.get("segmentFolders") or [])  # ascending by segment index
  total = len(segments_asc)
  offset = max(0, min(segment_offset, total))
  limit = max(1, min(DASHCAM_SEGMENT_LIMIT_MAX, segment_limit))
  end = min(offset + limit, total)

  # Map the requested display window back to an ascending slice so we only stat
  # the segments actually returned. "desc" shows newest-first, i.e. the tail of
  # the ascending list.
  if sort == "desc":
    asc_start = max(0, total - end)
    asc_end = max(0, total - offset)
  else:
    asc_start = offset
    asc_end = end
  page_asc = segments_asc[asc_start:asc_end]
  seed = segments_asc[asc_start - 1] if asc_start > 0 else None
  page_times = compute_segment_times(page_asc, seed_segment=seed)
  page_segments = list(reversed(page_asc)) if sort == "desc" else page_asc

  route_start, route_end = route_time_bounds(segments_asc)

  result = dict(entry)
  result["segmentFolders"] = page_segments
  result["segmentTimes"] = {
    segment: page_times[segment]
    for segment in page_segments
    if segment in page_times
  }
  result["segmentCount"] = int(entry.get("segmentCount") or total)
  result["segmentOffset"] = offset
  result["segmentLimit"] = limit
  result["segmentsNextOffset"] = end if end < total else None
  result["segmentsHasMore"] = end < total
  result["routeStartEpoch"] = route_start
  result["routeEndEpoch"] = route_end
  result["latestModifiedEpoch"] = route_end
  result["latestModifiedLabel"] = relative_time(route_end)
  return result


def find_dashcam_route(routes: list[dict], route: str) -> dict | None:
  if not route or "/" in route or "\\" in route or route in (".", ".."):
    return None
  for entry in routes:
    if entry.get("route") == route:
      return entry
  return None


def _routes_page_payload(offset: int, limit: int, segment_limit: int, sort: str) -> dict:
  """Build the /api/dashcam/routes response. Runs in a worker thread because the
  cheap name-only index build and the per-page segment stat()s both touch disk."""
  routes = visible_dashcam_routes()
  total = len(routes)
  end = min(offset + limit, total)
  return {
    "ok": True,
    "routes": [
      route_with_segment_page(entry, 0, segment_limit, sort)
      for entry in routes[offset:end]
    ],
    "root": DASHCAM_ROOT,
    "offset": offset,
    "limit": limit,
    "segmentLimit": segment_limit,
    "total": total,
    "nextOffset": end if end < total else None,
    "hasMore": end < total,
  }


async def api_dashcam_routes(request: web.Request) -> web.Response:
  try:
    offset = bounded_query_int(request, "offset", 0, DASHCAM_OFFSET_MAX)
    limit = bounded_query_int(request, "limit", DASHCAM_ROUTE_LIMIT_DEFAULT, DASHCAM_ROUTE_LIMIT_MAX)
    segment_limit = bounded_query_int(
      request,
      "segment_limit",
      DASHCAM_SEGMENT_LIMIT_DEFAULT,
      DASHCAM_SEGMENT_LIMIT_MAX,
    )
    sort = normalized_sort(request)
    payload = await asyncio.to_thread(_routes_page_payload, offset, limit, segment_limit, sort)
    return web.json_response(payload)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


def _segments_page_payload(route: str, offset: int, limit: int, sort: str) -> dict | None:
  """Build the /api/dashcam/segments response, or None when the route is gone.
  Runs in a worker thread for the same disk-access reason as the routes page."""
  routes = visible_dashcam_routes()
  entry = find_dashcam_route(routes, route)
  if not entry:
    return None
  page = route_with_segment_page(entry, offset, limit, sort)
  return {
    "ok": True,
    "route": route,
    "segments": page["segmentFolders"],
    "segmentTimes": page["segmentTimes"],
    "offset": page["segmentOffset"],
    "limit": page["segmentLimit"],
    "total": page["segmentCount"],
    "nextOffset": page["segmentsNextOffset"],
    "hasMore": page["segmentsHasMore"],
  }


async def api_dashcam_segments(request: web.Request) -> web.Response:
  try:
    route = request.match_info.get("route", "")
    offset = bounded_query_int(request, "offset", 0, DASHCAM_OFFSET_MAX)
    limit = bounded_query_int(request, "limit", DASHCAM_SEGMENT_LIMIT_DEFAULT, DASHCAM_SEGMENT_LIMIT_MAX)
    sort = normalized_sort(request)
    payload = await asyncio.to_thread(_segments_page_payload, route, offset, limit, sort)
    if payload is None:
      return web.json_response({"ok": False, "error": "route not found"}, status=404)
    return web.json_response(payload)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_report(request: web.Request) -> web.Response:
  try:
    route = request.match_info.get("route", "")
    prefer_rlog = request.query.get("source", "rlog") != "qlog"
    report = await asyncio.to_thread(build_route_report, route, prefer_rlog)
    status = 200 if report.get("ok") else 404
    return web.json_response(report, status=status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_summary_source(request: web.Request) -> web.Response:
  """Describe route logs for client-side summary analysis without parsing them."""
  try:
    route = request.match_info.get("route", "")

    def build_payload():
      entry = find_dashcam_route(visible_dashcam_routes(), route)
      if entry is None:
        return None
      return build_route_summary_source(route, list(entry.get("segmentFolders") or []))

    payload = await asyncio.to_thread(build_payload)
    if payload is None:
      return web.json_response({"ok": False, "error": "route not found"}, status=404)
    return web.json_response(payload, headers={"Cache-Control": "no-store"})
  except web.HTTPException as e:
    return web.json_response({"ok": False, "error": e.text or e.reason}, status=e.status)
  except Exception:
    return web.json_response({"ok": False, "error": "summary source unavailable"}, status=500)


async def api_dashcam_recent_segments(request: web.Request) -> web.Response:
  try:
    limit = int(request.query.get("limit", "") or 0)
  except (TypeError, ValueError):
    limit = 0
  if limit not in DASHCAM_RECENT_UPLOAD_LIMITS:
    return web.json_response({"ok": False, "error": "limit must be one of 2, 5, 10"}, status=400)
  try:
    segments = await asyncio.to_thread(recent_completed_dashcam_segments, limit)
    return web.json_response({
      "ok": True,
      "requested": limit,
      "count": len(segments),
      "segments": segments,
    })
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_thumbnail(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  path = await asyncio.to_thread(ensure_thumbnail, segment)
  return web.FileResponse(path, headers={"Cache-Control": "public, max-age=86400"})


async def api_dashcam_preview(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  path = await asyncio.to_thread(ensure_preview, segment)
  return web.FileResponse(path, headers={"Cache-Control": "public, max-age=86400"})


async def api_dashcam_video(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  path, content_type = await asyncio.to_thread(browser_video, segment)
  headers = {
    "Content-Type": content_type,
    "Cache-Control": "private, max-age=3600",
  }
  if request.query.get("download"):
    ext = os.path.splitext(path)[1] or ".mp4"
    headers["Content-Disposition"] = f'attachment; filename="{segment}{ext}"'
  return web.FileResponse(path, headers=headers)


async def api_dashcam_replay_source(request: web.Request) -> web.Response:
  """Return only cheap filesystem metadata for client-side replay processing."""
  try:
    payload = client_replay_source_description(request.match_info.get("segment", ""))
    return web.json_response(payload, headers={"Cache-Control": "no-store"})
  except web.HTTPException as e:
    return web.json_response({"ok": False, "error": e.text or e.reason}, status=e.status, headers={"Cache-Control": "no-store"})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500, headers={"Cache-Control": "no-store"})


async def api_dashcam_replay_source_file(request: web.Request) -> web.StreamResponse:
  """Stream an untouched replay input; no LogReader, decompression, or ffmpeg."""
  segment = safe_segment(request.match_info.get("segment", ""))
  segment_path = segment_dir(segment)
  kind = (request.match_info.get("kind", "") or "").strip()
  if kind == "rlog":
    path, name = source_rlog(segment_path)
    content_type = "application/zstd" if name.endswith(".zst") else (
      "application/x-bzip2" if name.endswith(".bz2") else "application/octet-stream"
    )
  elif kind == "qlog":
    path, name = source_qlog(segment_path)
    content_type = "application/zstd" if name.endswith(".zst") else (
      "application/x-bzip2" if name.endswith(".bz2") else "application/octet-stream"
    )
  elif kind == "video":
    path, name = source_video(segment_path)
    content_type = "video/mp4" if name.endswith(".mp4") else "video/mp2t"
  else:
    raise web.HTTPNotFound(text="replay source not found")
  return web.FileResponse(path, headers={
    "Content-Type": content_type,
    "Cache-Control": "no-store",
    "X-Content-Type-Options": "nosniff",
  })


async def api_dashcam_download(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  kind = (request.match_info.get("kind", "") or "").strip()
  segment_path = segment_dir(segment)
  allowed = {
    "qcamera": ("qcamera.ts", "qcamera.mp4"),
    "rlog": ("rlog.zst", "rlog.bz2", "rlog"),
    "qlog": ("qlog.zst", "qlog.bz2", "qlog"),
  }
  for name in allowed.get(kind, ()):
    path = os.path.join(segment_path, name)
    if os.path.isfile(path):
      mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
      return web.FileResponse(
        path,
        headers={
          "Content-Type": mime,
          "Content-Disposition": f'attachment; filename="{segment}--{name}"',
        },
      )
  raise web.HTTPNotFound(text="artifact not found")


async def api_dashcam_upload_summary(request: web.Request) -> web.Response:
  try:
    segments = await request_upload_segments(request)

    summaries = []
    for segment in segments:
      segment_path = segment_dir(segment)
      files = await asyncio.to_thread(segment_file_summary, segment_path)
      total_size = sum(int(item.get("size") or 0) for item in files)
      summaries.append({
        "segment": segment,
        "route": route_name(segment),
        "segmentIndex": segment_index(segment),
        "files": files,
        "totalSize": total_size,
        "totalSizeLabel": file_size_label(total_size),
      })
    return web.json_response({"ok": True, "summaries": summaries})
  except web.HTTPException as e:
    return web.json_response({"ok": False, "error": e.text or e.reason}, status=e.status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_upload(request: web.Request) -> web.Response:
  try:
    segments = await request_upload_segments(request)
    return web.json_response(await upload_jobs.run_upload_segments(segments))
  except web.HTTPException as e:
    return web.json_response({"ok": False, "error": e.text or e.reason}, status=e.status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_upload_start(request: web.Request) -> web.Response:
  try:
    segments = await request_upload_segments(request)
    running = upload_jobs.running_job()
    if running:
      return web.json_response({
        "ok": False,
        "error": "upload already running",
        "job_id": running.get("id"),
        "job": upload_jobs.snapshot(running),
      }, status=409)
    job = upload_jobs.create_job(segments)
    upload_jobs.start_job(job)
    return web.json_response({"ok": True, "job_id": job["id"], "status": job["status"]})
  except web.HTTPException as e:
    return web.json_response({"ok": False, "error": e.text or e.reason}, status=e.status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_upload_test(request: web.Request) -> web.Response:
  try:
    base_url, token = upload.upload_target_settings()
    result = await check_web_upload_health(base_url, token)
    if result.get("ok") and not token:
      await create_web_upload_session(base_url, upload.current_upload_metadata(), "test")
      result["session"] = "automatic"
    status = 200 if result.get("ok") else 502
    return web.json_response({"target": "web", "url": base_url, **result}, status=status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_upload_job(request: web.Request) -> web.Response:
  job_id = (request.query.get("id") or request.match_info.get("job_id") or "").strip()
  if not job_id:
    return web.json_response({"ok": False, "error": "missing job id"}, status=400)
  upload_jobs.expire_stale_jobs()
  job = upload_jobs.jobs().get(job_id)
  if not job:
    return web.json_response({"ok": False, "error": "job not found"}, status=404)
  return web.json_response(upload_jobs.snapshot(job))


async def api_dashcam_upload_cancel(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  job_id = str(body.get("id") or body.get("job_id") or "").strip()
  if not job_id:
    return web.json_response({"ok": False, "error": "missing job id"}, status=400)
  result = upload_jobs.cancel_job(job_id)
  status = 200 if result.get("ok") else 404
  return web.json_response(result, status=status)


async def api_dashcam_read_state(request: web.Request) -> web.Response:
  state = await asyncio.to_thread(read_dashcam_read_state)
  return web.json_response({"ok": True, **state})


async def api_dashcam_read_state_update(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  if not isinstance(body, dict):
    return web.json_response({"ok": False, "error": "bad request"}, status=400)
  try:
    state = await asyncio.to_thread(write_dashcam_recent_segment, body.get("recentSegment"))
  except ValueError as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)
  return web.json_response({"ok": True, **state})


def register(app: web.Application) -> None:
  app.router.add_get("/api/dashcam/routes", api_dashcam_routes)
  app.router.add_get("/api/dashcam/read-state", api_dashcam_read_state)
  app.router.add_post("/api/dashcam/read-state", api_dashcam_read_state_update)
  app.router.add_get("/api/dashcam/segments/{route}", api_dashcam_segments)
  app.router.add_get("/api/dashcam/report/{route}", api_dashcam_report)
  app.router.add_get("/api/dashcam/summary-source/{route}", api_dashcam_summary_source)
  app.router.add_get("/api/dashcam/recent", api_dashcam_recent_segments)
  app.router.add_get("/api/dashcam/thumbnail/{segment}", api_dashcam_thumbnail)
  app.router.add_get("/api/dashcam/preview/{segment}", api_dashcam_preview)
  app.router.add_get("/api/dashcam/video/{segment}", api_dashcam_video)
  app.router.add_get("/api/dashcam/replay-source/{segment}", api_dashcam_replay_source)
  app.router.add_get("/api/dashcam/replay-source/{segment}/{kind}", api_dashcam_replay_source_file)
  app.router.add_get("/api/dashcam/download/{segment}/{kind}", api_dashcam_download)
  app.router.add_post("/api/dashcam/upload/summary", api_dashcam_upload_summary)
  app.router.add_post("/api/dashcam/upload/test", api_dashcam_upload_test)
  app.router.add_post("/api/dashcam/upload/start", api_dashcam_upload_start)
  app.router.add_get("/api/dashcam/upload/job", api_dashcam_upload_job)
  app.router.add_post("/api/dashcam/upload/cancel", api_dashcam_upload_cancel)
  app.router.add_post("/api/dashcam/upload", api_dashcam_upload)
