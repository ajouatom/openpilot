from aiohttp import web

from . import (
  carrot_navi,
  cars,
  dashcam,
  egpu_model,
  intro,
  mapbox_tokens,
  params,
  screenrecord,
  settings,
  setting_favorites,
  setting_popular_values,
  setting_profiles,
  ssh_keys,
  static,
  stream,
  support_terminal,
  system,
  terminal,
  tools,
  vision_diag,
  vision_test,
  web_sound,
  web_settings,
  ws,
  youtube_live,
)


def register_all(app: web.Application) -> None:
  static.register(app)
  intro.register(app)
  carrot_navi.register(app)
  stream.register(app)
  support_terminal.register(app)
  ws.register(app)
  settings.register(app)
  params.register(app)
  setting_favorites.register(app)
  setting_popular_values.register(app)
  setting_profiles.register(app)
  web_settings.register(app)
  ssh_keys.register(app)
  cars.register(app)
  system.register(app)
  terminal.register(app)
  dashcam.register(app)
  egpu_model.register(app)
  screenrecord.register(app)
  tools.register(app)
  mapbox_tokens.register(app)
  youtube_live.register(app)
  vision_test.register(app)
  vision_diag.register(app)
  web_sound.register(app)
