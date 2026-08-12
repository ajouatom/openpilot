from __future__ import annotations

import json
from aiohttp import WSMsgType, web

from .bridge import CarrotNaviWebBridge
from .protocol import MEDIA_WIRE_VERSION, SESSION_BUSY_CODE, SESSION_WIRE_VERSION


APP_KEY = "carrot_navi_web_bridge"


def _bridge(request: web.Request) -> CarrotNaviWebBridge:
  bridge = request.app.get(APP_KEY)
  if not isinstance(bridge, CarrotNaviWebBridge):
    raise web.HTTPServiceUnavailable(text="Carrot Navi web bridge unavailable")
  return bridge


async def bridge_context(app: web.Application):
  bridge = CarrotNaviWebBridge()
  app[APP_KEY] = bridge
  try:
    yield
  finally:
    await bridge.stop()
    app.pop(APP_KEY, None)


async def api_capabilities(_request: web.Request) -> web.Response:
  return web.json_response({
    "ok": True,
    "feature": "carrotNavi",
    "stateProtocolVersion": 1,
    "mediaProtocolVersion": MEDIA_WIRE_VERSION,
    "codec": "avc1.42E01E",
    "mapStream": "render:map_main",
    "overlayStreams": "image:*",
    "requiresWebCodecs": False,
    "supportsMediaSource": True,
    "supportsWebRTC": False,
    "browserPipeline": "server-fmp4-v1",
    "sessionProtocolVersion": SESSION_WIRE_VERSION,
    "sessionPolicy": "single-viewer-last-entry-wins",
    "sessionTakeoverChannel": "state",
    "sessionBusyCode": SESSION_BUSY_CODE,
    "mapProfiles": ["default", "cavdy_hud"],
  })


async def api_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **_bridge(request).status()})


async def api_client_diagnostic(request: web.Request) -> web.Response:
  raw = await request.read()
  if len(raw) > 8192:
    raise web.HTTPRequestEntityTooLarge(max_size=8192, actual_size=len(raw))
  try:
    payload = json.loads(raw)
  except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
    raise web.HTTPBadRequest(text="invalid Carrot Navi client diagnostic") from None
  if not isinstance(payload, dict):
    raise web.HTTPBadRequest(text="Carrot Navi client diagnostic must be an object")
  _bridge(request).record_client_diagnostic(request.remote or "-", payload)
  return web.json_response({"ok": True})


async def _serve_ws(request: web.Request, media: bool) -> web.WebSocketResponse:
  bridge = _bridge(request)
  if not bridge.stream_allowed(force=True):
    raise web.HTTPConflict(text="Carrot Navi web stream is unavailable while Cluster HUD is active")
  raw_client_id = str(request.query.get("client_id", "")).strip()[:128]
  client_id = f"client:{raw_client_id}" if raw_client_id else f"remote:{request.remote or '-'}"
  takeover = str(request.query.get("takeover", "0")).lower() in ("1", "true", "yes")
  ws = web.WebSocketResponse(heartbeat=20, max_msg_size=8 * 1024 * 1024, compress=False)
  await ws.prepare(request)
  if media:
    include_map = request.query.get("map", "1").lower() not in ("0", "false", "no")
    hud_map_profile = (
      raw_client_id == "cavdy-navdy"
      and request.query.get("profile", "").strip().lower() == "cavdy_hud"
    )
    registered = await bridge.register_media(ws, client_id, include_map=include_map)
  else:
    registered = await bridge.register_state(ws, client_id, takeover=takeover)
  if not registered:
    return ws
  if media and hud_map_profile:
    bridge.set_hud_map_profile(True)
  try:
    async for message in ws:
      if message.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    await bridge.unregister(ws)
    if media and hud_map_profile:
      bridge.set_hud_map_profile(False)
  return ws


async def ws_state(request: web.Request) -> web.WebSocketResponse:
  return await _serve_ws(request, False)


async def ws_media(request: web.Request) -> web.WebSocketResponse:
  return await _serve_ws(request, True)


def register(app: web.Application) -> None:
  app.cleanup_ctx.append(bridge_context)
  app.router.add_get("/api/carrot_navi/capabilities", api_capabilities)
  app.router.add_get("/api/carrot_navi/status", api_status)
  app.router.add_post("/api/carrot_navi/client_diagnostic", api_client_diagnostic)
  app.router.add_get("/ws/carrot_navi/state", ws_state)
  app.router.add_get("/ws/carrot_navi/media", ws_media)
