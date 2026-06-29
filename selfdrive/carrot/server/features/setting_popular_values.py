from __future__ import annotations

from aiohttp import web

from ..services.popular_values import (
  get_popular_value_detail,
  read_popular_values_memory,
  refresh_popular_values_once,
  schedule_popular_value_refresh,
)


async def api_setting_popular_values(request: web.Request) -> web.Response:
  # Kick a throttled, download-only refresh so opening settings reflects the
  # latest fleet values (non-blocking — returns the current cache immediately).
  schedule_popular_value_refresh(request.app)
  return web.json_response(read_popular_values_memory())


async def api_setting_popular_value_detail(request: web.Request) -> web.Response:
  name = request.query.get("name", "")
  cache = read_popular_values_memory()
  return web.json_response({
    "ok": True,
    "car_key_type": cache.get("car_key_type", "CarSelected3"),
    "car_key": cache.get("car_key", ""),
    "param_name": name,
    "detail": get_popular_value_detail(name),
  })


async def api_setting_popular_values_refresh(request: web.Request) -> web.Response:
  session = request.app.get("http")
  if session is None:
    return web.json_response(read_popular_values_memory())
  cache = await refresh_popular_values_once(session, upload=True)
  return web.json_response(cache)


def register(app: web.Application) -> None:
  app.router.add_get("/api/setting_popular_values", api_setting_popular_values)
  app.router.add_get("/api/setting_popular_values/detail", api_setting_popular_value_detail)
  app.router.add_post("/api/setting_popular_values/refresh", api_setting_popular_values_refresh)
