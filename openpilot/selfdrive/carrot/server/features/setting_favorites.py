from aiohttp import web

from ..services.setting_favorites import read_setting_favorites, update_setting_favorites


async def get_setting_favorites(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **read_setting_favorites()})


async def set_setting_favorites(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  if not isinstance(body, dict):
    return web.json_response({"ok": False, "error": "bad request"}, status=400)
  settings = update_setting_favorites(body)
  return web.json_response({"ok": True, **settings})


def register(app: web.Application) -> None:
  app.router.add_get("/api/setting_favorites", get_setting_favorites)
  app.router.add_post("/api/setting_favorites", set_setting_favorites)
