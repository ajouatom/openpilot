import os

from aiohttp import web

from ..config import WEB_DIR


async def handle_index(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(WEB_DIR, "index.html"))


def register(app: web.Application) -> None:
  app.router.add_get("/", handle_index)
