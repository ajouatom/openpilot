import asyncio

from aiohttp import web

from ..services.vision_diag import get_server_diagnostic_snapshot, upload_diagnostic_bundle_to_discord


async def api_vision_diag_server_snapshot(_request: web.Request) -> web.Response:
  snapshot = await asyncio.to_thread(get_server_diagnostic_snapshot)
  return web.json_response({"ok": True, "snapshot": snapshot})


async def api_vision_diag_upload_discord(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except web.HTTPRequestEntityTooLarge:
    return web.json_response({"ok": False, "error": "diagnostic upload too large"}, status=413)
  except Exception as exc:
    return web.json_response({"ok": False, "error": f"invalid diagnostic upload json: {exc}"}, status=400)
  bundle_text = str(body.get("bundle") or body.get("text") or "")
  if not bundle_text.strip():
    return web.json_response({"ok": False, "error": "missing diagnostic bundle"}, status=400)
  result = await upload_diagnostic_bundle_to_discord(
    bundle_text=bundle_text,
    filename=body.get("filename"),
    console_text=str(body.get("console") or ""),
    console_filename=body.get("consoleFilename") or body.get("console_filename"),
    source=str(body.get("source") or "web"),
  )
  status = 200 if result.get("ok") or result.get("skipped") else 502
  return web.json_response({"ok": bool(result.get("ok")), "discord": result}, status=status)


def register(app: web.Application) -> None:
  app.router.add_get("/api/vision_diag/server_snapshot", api_vision_diag_server_snapshot)
  app.router.add_post("/api/vision_diag/upload_discord", api_vision_diag_upload_discord)
