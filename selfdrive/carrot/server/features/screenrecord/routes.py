import asyncio
import mimetypes
import os
import threading
import time

from aiohttp import web

from ...config import SCREEN_RECORDING_DIRS
from .catalog import build_videos, find_file, thumbnail_path

VIDEO_CACHE_TTL = 3.0
_video_cache_lock = threading.Lock()
_video_cache = {"time": 0.0, "videos": []}


def cached_screenrecord_videos() -> list[dict]:
  now = time.monotonic()
  with _video_cache_lock:
    if now - float(_video_cache.get("time") or 0.0) < VIDEO_CACHE_TTL:
      return list(_video_cache.get("videos") or [])

  videos = build_videos()
  with _video_cache_lock:
    _video_cache["time"] = time.monotonic()
    _video_cache["videos"] = videos
  return list(videos)


async def api_screenrecord_videos(request: web.Request) -> web.Response:
  try:
    offset = max(0, int(request.query.get("offset", "0") or 0))
    limit = max(1, min(200, int(request.query.get("limit", "80") or 80)))
    videos = await asyncio.to_thread(cached_screenrecord_videos)
    total = len(videos)
    end = min(offset + limit, total)
    folders = [folder for folder in SCREEN_RECORDING_DIRS if os.path.isdir(folder)]
    return web.json_response({
      "ok": True,
      "videos": videos[offset:end],
      "folders": folders,
      "offset": offset,
      "limit": limit,
      "total": total,
      "nextOffset": end if end < total else None,
      "hasMore": end < total,
    })
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_screenrecord_thumbnail(request: web.Request) -> web.StreamResponse:
  file_id_in = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(thumbnail_path, file_id_in)
  return web.FileResponse(path, headers={"Cache-Control": "public, max-age=86400"})


async def api_screenrecord_video(request: web.Request) -> web.StreamResponse:
  file_id_in = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(find_file, file_id_in)
  mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
  headers = {
    "Content-Type": mime,
    "Cache-Control": "private, max-age=3600",
  }
  if request.query.get("download"):
    filename = os.path.basename(path)
    safe_filename = "".join(ch if 32 <= ord(ch) < 127 and ch not in {'"', "\\"} else "_" for ch in filename)
    headers["Content-Disposition"] = f'attachment; filename="{safe_filename or "screenrecord"}"'
  return web.FileResponse(path, headers=headers)


async def api_screenrecord_download(request: web.Request) -> web.StreamResponse:
  file_id_in = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(find_file, file_id_in)
  filename = os.path.basename(path)
  safe_filename = "".join(ch if 32 <= ord(ch) < 127 and ch not in {'"', "\\"} else "_" for ch in filename)
  mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
  return web.FileResponse(
    path,
    headers={
      "Content-Type": mime,
      "Content-Disposition": f'attachment; filename="{safe_filename or "screenrecord"}"',
    },
  )


def register(app: web.Application) -> None:
  app.router.add_get("/api/screenrecord/videos", api_screenrecord_videos)
  app.router.add_get("/api/screenrecord/thumbnail/{file_id}", api_screenrecord_thumbnail)
  app.router.add_get("/api/screenrecord/video/{file_id}", api_screenrecord_video)
  app.router.add_get("/api/screenrecord/download/{file_id}", api_screenrecord_download)
