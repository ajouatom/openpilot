from aiohttp import web

from ..services.web_settings import read_web_settings, update_web_settings


async def get_web_settings(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, "settings": read_web_settings()})


async def set_web_settings(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  if not isinstance(body, dict):
    return web.json_response({"ok": False, "error": "bad request"}, status=400)
  settings = update_web_settings(body)
  return web.json_response({"ok": True, "settings": settings})


def register(app: web.Application) -> None:
  app.router.add_get("/api/web_settings", get_web_settings)
  app.router.add_post("/api/web_settings", set_web_settings)
