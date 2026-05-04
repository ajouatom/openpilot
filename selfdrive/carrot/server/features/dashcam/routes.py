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


async def api_dashcam_routes(request: web.Request) -> web.Response:
  try:
    offset = max(0, int(request.query.get("offset", "0") or 0))
    limit = max(1, min(200, int(request.query.get("limit", "80") or 80)))
    routes = await asyncio.to_thread(cached_dashcam_routes)
    total = len(routes)
    end = min(offset + limit, total)
    return web.json_response({
      "ok": True,
      "routes": routes[offset:end],
      "root": DASHCAM_ROOT,
      "offset": offset,
      "limit": limit,
      "total": total,
      "nextOffset": end if end < total else None,
      "hasMore": end < total,
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
    if upload_jobs.has_running_job():
      return web.json_response({"ok": False, "error": "upload already running"}, status=409)
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


def register(app: web.Application) -> None:
  app.router.add_get("/api/dashcam/routes", api_dashcam_routes)
  app.router.add_get("/api/dashcam/thumbnail/{segment}", api_dashcam_thumbnail)
  app.router.add_get("/api/dashcam/preview/{segment}", api_dashcam_preview)
  app.router.add_get("/api/dashcam/video/{segment}", api_dashcam_video)
  app.router.add_get("/api/dashcam/download/{segment}/{kind}", api_dashcam_download)
  app.router.add_post("/api/dashcam/upload/summary", api_dashcam_upload_summary)
  app.router.add_post("/api/dashcam/upload/start", api_dashcam_upload_start)
  app.router.add_get("/api/dashcam/upload/job", api_dashcam_upload_job)
  app.router.add_post("/api/dashcam/upload", api_dashcam_upload)
