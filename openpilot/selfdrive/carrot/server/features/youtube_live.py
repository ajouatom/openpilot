from __future__ import annotations

import asyncio

from aiohttp import web

from ..services.youtube_live import YouTubeLiveService


YOUTUBE_LIVE_APP_KEY = "youtube_live_service"


def _service(request: web.Request) -> YouTubeLiveService:
  service = request.app.get(YOUTUBE_LIVE_APP_KEY)
  if not isinstance(service, YouTubeLiveService):
    raise web.HTTPServiceUnavailable(text="youtube live service unavailable")
  return service


async def youtube_live_context(app: web.Application):
  service = YouTubeLiveService()
  app[YOUTUBE_LIVE_APP_KEY] = service
  await service.start()
  try:
    yield
  finally:
    await service.stop()
    app.pop(YOUTUBE_LIVE_APP_KEY, None)


async def api_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **_service(request).status()})


async def api_set_stream_key(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)
  key = body.get("stream_key", body.get("key", ""))
  try:
    status = _service(request).set_stream_key(str(key or ""))
  except ValueError as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=400)
  return web.json_response({"ok": True, **status})


async def api_get_stream_key(request: web.Request) -> web.Response:
  stream_key = _service(request).get_stream_key()
  return web.json_response({"ok": True, "configured": bool(stream_key), "stream_key": stream_key})


async def api_clear_stream_key(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **_service(request).clear_stream_key()})


async def api_test(request: web.Request) -> web.Response:
  result = await asyncio.to_thread(_service(request).test_config)
  return web.json_response(result, status=200 if result.get("ok") else 409)


async def api_validate_stream_key(request: web.Request) -> web.Response:
  body = {}
  if request.can_read_body:
    try:
      body = await request.json()
    except Exception:
      return web.json_response({"ok": False, "error": "invalid json"}, status=400)
  key = body.get("stream_key", body.get("key")) if isinstance(body, dict) else None
  result = await asyncio.to_thread(_service(request).validate_stream_key, key)
  return web.json_response(result, status=200 if result.get("ok") else 409)


async def api_diagnostics(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, "diagnostics": _service(request).diagnostics()})


def register(app: web.Application) -> None:
  app.cleanup_ctx.append(youtube_live_context)
  app.router.add_get("/api/youtube_live/status", api_status)
  app.router.add_get("/api/youtube_live/diagnostics", api_diagnostics)
  app.router.add_get("/api/youtube_live/stream_key", api_get_stream_key)
  app.router.add_post("/api/youtube_live/stream_key", api_set_stream_key)
  app.router.add_post("/api/youtube_live/stream_key/validate", api_validate_stream_key)
  app.router.add_delete("/api/youtube_live/stream_key", api_clear_stream_key)
  app.router.add_post("/api/youtube_live/test", api_test)
