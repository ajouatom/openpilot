from __future__ import annotations

from aiohttp import web

from ..services.support_terminal import manager


async def api_start(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  note = str(body.get("note") or "") if isinstance(body, dict) else ""
  ttl_seconds = body.get("ttl_seconds") if isinstance(body, dict) else None
  permission_mode = body.get("permission_mode") if isinstance(body, dict) else None
  command_timeout_seconds = body.get("command_timeout_seconds") if isinstance(body, dict) else None
  payload = await manager.start(
    request.app,
    note=note,
    ttl_seconds=ttl_seconds,
    permission_mode=permission_mode,
    command_timeout_seconds=command_timeout_seconds,
  )
  status = 500 if payload.get("state") == "error" else 200
  return web.json_response(payload, status=status)


async def api_stop(request: web.Request) -> web.Response:
  payload = await manager.stop("user")
  return web.json_response(payload)


async def api_status(request: web.Request) -> web.Response:
  return web.json_response(manager.snapshot())


async def api_approve(request: web.Request) -> web.Response:
  command_id = request.match_info.get("command_id", "")
  payload = await manager.approve_command(command_id)
  return web.json_response(payload, status=200 if payload.get("ok") else 400)


async def api_reject(request: web.Request) -> web.Response:
  command_id = request.match_info.get("command_id", "")
  payload = await manager.reject_command(command_id)
  return web.json_response(payload, status=200 if payload.get("ok") else 400)


async def ws_owner(request: web.Request) -> web.WebSocketResponse:
  return await manager.handle_owner_ws(request)


async def ws_guest(request: web.Request) -> web.WebSocketResponse:
  return await manager.handle_guest_ws(request)


async def guest_page(request: web.Request) -> web.Response:
  return await manager.handle_guest_page(request)


async def guest_asset(request: web.Request) -> web.StreamResponse:
  return await manager.handle_guest_asset(request)


def register(app: web.Application) -> None:
  app.router.add_post("/api/support_terminal/start", api_start)
  app.router.add_post("/api/support_terminal/stop", api_stop)
  app.router.add_get("/api/support_terminal/status", api_status)
  app.router.add_post("/api/support_terminal/commands/{command_id}/approve", api_approve)
  app.router.add_post("/api/support_terminal/commands/{command_id}/reject", api_reject)
  app.router.add_get("/ws/support_terminal/owner", ws_owner)
  app.router.add_get("/ws/support_terminal/{session_id}", ws_guest)
  app.router.add_get("/support/terminal/{session_id}", guest_page)
  app.router.add_get("/support-terminal-assets/{asset_name}", guest_asset)
