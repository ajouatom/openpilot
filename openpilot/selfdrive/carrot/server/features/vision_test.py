import asyncio

from aiohttp import web

from ..services.vision_test import get_status


async def api_vision_test_status(_request: web.Request) -> web.Response:
  status = await asyncio.to_thread(get_status)
  return web.json_response({"ok": True, **status})


def register(app: web.Application) -> None:
  app.router.add_get("/api/vision_test/status", api_vision_test_status)
