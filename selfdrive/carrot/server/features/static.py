import os

from aiohttp import web

from ..config import WEB_DIR


async def handle_index(request: web.Request) -> web.Response:
  response = web.FileResponse(os.path.join(WEB_DIR, "index.html"))
  response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
  response.headers["Pragma"] = "no-cache"
  response.headers["Expires"] = "0"
  return response


def register(app: web.Application) -> None:
  app.router.add_get("/", handle_index)
