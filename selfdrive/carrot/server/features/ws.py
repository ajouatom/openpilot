import json

from aiohttp import WSMsgType, web

from ...realtime.raw_protocol import build_raw_hello, build_raw_multiplex_hello
from ...realtime.transports import CameraWsHub, RawWsHub


async def ws_raw(request: web.Request) -> web.WebSocketResponse:
  service = (request.match_info.get("service") or "").strip()
  hub: RawWsHub | None = request.app.get("realtime_raw_hub")
  if hub is None:
    raise web.HTTPServiceUnavailable(text="realtime raw hub unavailable")
  if not service or not hub.is_allowed_service(service):
    raise web.HTTPNotFound(text=f"unknown raw service: {service}")

  ws = web.WebSocketResponse(heartbeat=20, max_msg_size=8 * 1024 * 1024, compress=False)
  await ws.prepare(request)
  await ws.send_str(json.dumps(build_raw_hello(service=service), separators=(",", ":")))
  await hub.register(service, ws)
  try:
    async for msg in ws:
      if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    await hub.unregister_client(ws)

  return ws


async def ws_raw_multiplex(request: web.Request) -> web.WebSocketResponse:
  hub: RawWsHub | None = request.app.get("realtime_raw_hub")
  if hub is None:
    raise web.HTTPServiceUnavailable(text="realtime raw hub unavailable")

  services_param = request.query.get("services", "")
  services = [service.strip() for service in services_param.split(",") if service.strip()]
  if not services:
    raise web.HTTPBadRequest(text="missing raw services")
  invalid = [service for service in services if not hub.is_allowed_service(service)]
  if invalid:
    raise web.HTTPNotFound(text=f"unknown raw services: {','.join(invalid)}")

  ws = web.WebSocketResponse(heartbeat=20, max_msg_size=8 * 1024 * 1024, compress=False)
  await ws.prepare(request)
  await ws.send_str(json.dumps(build_raw_multiplex_hello(services=services), separators=(",", ":")))
  await hub.register_many(services, ws)
  try:
    async for msg in ws:
      if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    await hub.unregister_client(ws)

  return ws


async def ws_camera(request: web.Request) -> web.WebSocketResponse:
  hub: CameraWsHub | None = request.app.get("realtime_camera_hub")
  if hub is None:
    raise web.HTTPServiceUnavailable(text="realtime camera hub unavailable")
  return await hub.ws_camera(request)


def register(app: web.Application) -> None:
  app.router.add_get("/ws/raw_multiplex", ws_raw_multiplex)
  app.router.add_get("/ws/raw/{service}", ws_raw)
  app.router.add_get("/ws/camera/{camera}", ws_camera)
