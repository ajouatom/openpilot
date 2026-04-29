from aiohttp import web

from . import (
  cars,
  dashcam,
  params,
  screenrecord,
  settings,
  static,
  stream,
  system,
  terminal,
  tools,
  ws,
)


def register_all(app: web.Application) -> None:
  static.register(app)
  stream.register(app)
  ws.register(app)
  settings.register(app)
  params.register(app)
  cars.register(app)
  system.register(app)
  terminal.register(app)
  dashcam.register(app)
  screenrecord.register(app)
  tools.register(app)
