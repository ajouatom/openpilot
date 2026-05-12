import asyncio
import mimetypes
import os
import threading
import time

from aiohttp import web

from ...config import DASHCAM_ROOT
from . import upload_jobs
from .catalog import build_routes, segment_file_summary
from .ffmpeg import browser_video, ensure_preview, ensure_thumbnail
from .paths import (
  file_size_label,
  route_name,
  safe_segment,
  segment_dir,
  segment_index,
)

ROUTE_CACHE_TTL = 3.0
DASHCAM_ROUTE_LIMIT_DEFAULT = 40
DASHCAM_ROUTE_LIMIT_MAX = 200
DASHCAM_SEGMENT_LIMIT_DEFAULT = 10
DASHCAM_SEGMENT_LIMIT_MAX = 80
DASHCAM_OFFSET_MAX = 1000000
_route_cache_lock = threading.Lock()
_route_cache = {"time": 0.0, "routes": []}


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
  return segments


def cached_dashcam_routes() -> list[dict]:
  now = time.monotonic()
  with _route_cache_lock:
    if now - float(_route_cache.get("time") or 0.0) < ROUTE_CACHE_TTL:
      return list(_route_cache.get("routes") or [])

  routes = build_routes()
  with _route_cache_lock:
    _route_cache["time"] = time.monotonic()
    _route_cache["routes"] = routes
  return list(routes)


def bounded_query_int(request: web.Request, name: str, default: int, maximum: int) -> int:
  try:
    value = int(request.query.get(name, str(default)) or default)
  except (TypeError, ValueError):
    value = default
  return max(0 if name == "offset" else 1, min(maximum, value))


def route_with_segment_page(entry: dict, segment_offset: int = 0, segment_limit: int = DASHCAM_SEGMENT_LIMIT_DEFAULT) -> dict:
  segments = list(entry.get("segmentFolders") or [])
  total = len(segments)
  offset = max(0, min(segment_offset, total))
  limit = max(1, min(DASHCAM_SEGMENT_LIMIT_MAX, segment_limit))
  end = min(offset + limit, total)
  result = dict(entry)
  result["segmentFolders"] = segments[offset:end]
  result["segmentCount"] = int(entry.get("segmentCount") or total)
  result["segmentOffset"] = offset
  result["segmentLimit"] = limit
  result["segmentsNextOffset"] = end if end < total else None
  result["segmentsHasMore"] = end < total
  return result


def find_dashcam_route(routes: list[dict], route: str) -> dict | None:
  if not route or "/" in route or "\\" in route or route in (".", ".."):
    return None
  for entry in routes:
    if entry.get("route") == route:
      return entry
  return None


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
    routes = await asyncio.to_thread(cached_dashcam_routes)
    total = len(routes)
    end = min(offset + limit, total)
    return web.json_response({
      "ok": True,
      "routes": [
        route_with_segment_page(entry, 0, segment_limit)
        for entry in routes[offset:end]
      ],
      "root": DASHCAM_ROOT,
      "offset": offset,
      "limit": limit,
      "segmentLimit": segment_limit,
      "total": total,
      "nextOffset": end if end < total else None,
      "hasMore": end < total,
    })
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_segments(request: web.Request) -> web.Response:
  try:
    route = request.match_info.get("route", "")
    offset = bounded_query_int(request, "offset", 0, DASHCAM_OFFSET_MAX)
    limit = bounded_query_int(request, "limit", DASHCAM_SEGMENT_LIMIT_DEFAULT, DASHCAM_SEGMENT_LIMIT_MAX)
    routes = await asyncio.to_thread(cached_dashcam_routes)
    entry = find_dashcam_route(routes, route)
    if not entry:
      return web.json_response({"ok": False, "error": "route not found"}, status=404)
    page = route_with_segment_page(entry, offset, limit)
    return web.json_response({
      "ok": True,
      "route": route,
      "segments": page["segmentFolders"],
      "offset": page["segmentOffset"],
      "limit": page["segmentLimit"],
      "total": page["segmentCount"],
      "nextOffset": page["segmentsNextOffset"],
      "hasMore": page["segmentsHasMore"],
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
    asyncio.create_task(upload_jobs.run_job(job))
    return web.json_response({"ok": True, "job_id": job["id"], "status": job["status"]})
  except web.HTTPException as e:
    return web.json_response({"ok": False, "error": e.text or e.reason}, status=e.status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_upload_job(request: web.Request) -> web.Response:
  job_id = (request.query.get("id") or request.match_info.get("job_id") or "").strip()
  if not job_id:
    return web.json_response({"ok": False, "error": "missing job id"}, status=400)
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


def register(app: web.Application) -> None:
  app.router.add_get("/api/dashcam/routes", api_dashcam_routes)
  app.router.add_get("/api/dashcam/segments/{route}", api_dashcam_segments)
  app.router.add_get("/api/dashcam/thumbnail/{segment}", api_dashcam_thumbnail)
  app.router.add_get("/api/dashcam/preview/{segment}", api_dashcam_preview)
  app.router.add_get("/api/dashcam/video/{segment}", api_dashcam_video)
  app.router.add_get("/api/dashcam/download/{segment}/{kind}", api_dashcam_download)
  app.router.add_post("/api/dashcam/upload/summary", api_dashcam_upload_summary)
  app.router.add_post("/api/dashcam/upload/start", api_dashcam_upload_start)
  app.router.add_get("/api/dashcam/upload/job", api_dashcam_upload_job)
  app.router.add_post("/api/dashcam/upload/cancel", api_dashcam_upload_cancel)
  app.router.add_post("/api/dashcam/upload", api_dashcam_upload)
