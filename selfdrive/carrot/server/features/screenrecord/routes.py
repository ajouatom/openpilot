import asyncio
import mimetypes
import os

from aiohttp import web

from ...config import SCREEN_RECORDING_DIRS
from .catalog import build_videos, find_file, thumbnail_path


async def api_screenrecord_videos(request: web.Request) -> web.Response:
  try:
    videos = await asyncio.to_thread(build_videos)
    folders = [folder for folder in SCREEN_RECORDING_DIRS if os.path.isdir(folder)]
    return web.json_response({"ok": True, "videos": videos, "folders": folders})
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
  return web.FileResponse(
    path,
    headers={
      "Content-Type": mime,
      "Cache-Control": "private, max-age=3600",
    },
  )


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
