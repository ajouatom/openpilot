from aiohttp import web

from .core import log_mw, on_startup, on_cleanup, WEB_DIR
from . import routes_static, routes_api, routes_ws


def make_app() -> web.Application:
  app = web.Application(middlewares=[log_mw])
  app.on_startup.append(on_startup)
  app.on_cleanup.append(on_cleanup)

  # static-like routes
  app.router.add_get("/", routes_static.handle_index)
  app.router.add_get("/app.js", routes_static.handle_appjs)

  # api
  app.router.add_get("/api/settings", routes_api.api_settings)
  app.router.add_get("/api/params_bulk", routes_api.api_params_bulk)
  app.router.add_post("/api/param_set", routes_api.api_param_set)
  app.router.add_get("/api/cars", routes_api.api_cars)
  app.router.add_post("/api/reboot", routes_api.api_reboot)
  app.router.add_post("/api/tools", routes_api.api_tools)
  app.router.add_post("/api/tools/start", routes_api.api_tools_start)
  app.router.add_get("/api/tools/job", routes_api.api_tools_job)
  app.router.add_post("/api/params_restore", routes_api.api_params_restore)
  app.router.add_get("/api/heartbeat_status", routes_api.api_heartbeat_status)
  app.router.add_get("/api/live_runtime", routes_api.api_live_runtime)
  app.router.add_post("/api/time_sync", routes_api.api_time_sync)
  app.router.add_get("/api/dashcam/routes", routes_api.api_dashcam_routes)
  app.router.add_get("/api/dashcam/thumbnail/{segment}", routes_api.api_dashcam_thumbnail)
  app.router.add_get("/api/dashcam/preview/{segment}", routes_api.api_dashcam_preview)
  app.router.add_get("/api/dashcam/video/{segment}", routes_api.api_dashcam_video)
  app.router.add_get("/api/dashcam/download/{segment}/{kind}", routes_api.api_dashcam_download)
  app.router.add_post("/api/dashcam/upload", routes_api.api_dashcam_upload)
  app.router.add_get("/api/screenrecord/videos", routes_api.api_screenrecord_videos)
  app.router.add_get("/api/screenrecord/thumbnail/{file_id}", routes_api.api_screenrecord_thumbnail)
  app.router.add_get("/api/screenrecord/video/{file_id}", routes_api.api_screenrecord_video)
  app.router.add_get("/api/screenrecord/download/{file_id}", routes_api.api_screenrecord_download)
  app.router.add_post("/stream", routes_api.proxy_stream)

  # ws
  app.router.add_get("/ws/raw_multiplex", routes_ws.ws_raw_multiplex)
  app.router.add_get("/ws/raw/{service}", routes_ws.ws_raw)
  app.router.add_get("/ws/camera/{camera}", routes_ws.ws_camera)
  app.router.add_get("/ws/terminal", routes_ws.ws_terminal)

  # downloads
  app.router.add_get("/download/tmux.log", routes_api.handle_download_tmux)
  app.router.add_get("/download/params_backup.json", routes_api.handle_download_params_backup)

  # foldered static assets
  app.router.add_static("/", str(WEB_DIR), show_index=True)
  return app
