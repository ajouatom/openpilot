import asyncio
import mimetypes
import os
from datetime import datetime

from aiohttp import web

from ...config import DASHCAM_ROOT
from ...services.params import HAS_PARAMS, Params
from . import upload
from .catalog import build_routes, segment_file_summary
from .ffmpeg import browser_video, ensure_preview, ensure_thumbnail
from .paths import (
  route_name,
  safe_segment,
  segment_dir,
  segment_index,
)


async def api_dashcam_routes(request: web.Request) -> web.Response:
  try:
    routes = await asyncio.to_thread(build_routes)
    return web.json_response({"ok": True, "routes": routes, "root": DASHCAM_ROOT})
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
  return web.FileResponse(
    path,
    headers={
      "Content-Type": content_type,
      "Cache-Control": "private, max-age=3600",
    },
  )


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


async def api_dashcam_upload(request: web.Request) -> web.Response:
  try:
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
      return web.json_response({"ok": False, "error": "missing segments"}, status=400)

    params = Params() if HAS_PARAMS else None
    meta = upload.upload_metadata(params)
    car_selected = meta.get("carName") or "none"
    dongle_id = meta.get("dongleId") or "unknown"
    directory = f"{car_selected} {dongle_id}".strip()
    remote_base_path = f"routes/{directory}/".replace("\\", "/")

    results = []
    for segment in segments:
      try:
        segment_path = segment_dir(segment)
        ok = await asyncio.to_thread(
          upload.upload_folder_to_ftp,
          segment_path,
          directory,
          segment,
        )
        results.append({
          "segment": segment,
          "route": route_name(segment),
          "segmentIndex": segment_index(segment),
          "ok": bool(ok),
          "remotePath": f"{remote_base_path}{segment}",
          "files": segment_file_summary(segment_path),
        })
      except Exception as e:
        results.append({
          "segment": segment,
          "route": route_name(segment),
          "segmentIndex": segment_index(segment),
          "ok": False,
          "remotePath": f"{remote_base_path}{segment}",
          "error": str(e),
        })

    ok_count = sum(1 for item in results if item["ok"])
    uploaded_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    response_payload = {
      "ok": ok_count == len(results),
      "uploaded": ok_count,
      "total": len(results),
      "uploadedAt": uploaded_at,
      "remoteBasePath": remote_base_path,
      "meta": meta,
      "results": results,
      "message": f"{ok_count}/{len(results)} uploaded",
    }
    response_payload["shareText"] = upload.upload_share_text(response_payload)
    response_payload["discord"] = await upload.send_discord_webhook(
      upload.discord_webhook_url(params),
      response_payload,
    )
    return web.json_response(response_payload)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


def register(app: web.Application) -> None:
  app.router.add_get("/api/dashcam/routes", api_dashcam_routes)
  app.router.add_get("/api/dashcam/thumbnail/{segment}", api_dashcam_thumbnail)
  app.router.add_get("/api/dashcam/preview/{segment}", api_dashcam_preview)
  app.router.add_get("/api/dashcam/video/{segment}", api_dashcam_video)
  app.router.add_get("/api/dashcam/download/{segment}/{kind}", api_dashcam_download)
  app.router.add_post("/api/dashcam/upload", api_dashcam_upload)
