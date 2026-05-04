from aiohttp import web

from ..services.ssh_keys import (
  SshKeyError,
  add_github_ssh_keys,
  clear_ssh_keys,
  get_ssh_key_status,
)


async def api_ssh_keys_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **get_ssh_key_status()})


async def api_ssh_keys(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  action = str(body.get("action") or "").strip().lower()
  try:
    if action == "remove":
      return web.json_response({"ok": True, **clear_ssh_keys()})
    if action == "add":
      session = request.app.get("http")
      if session is None:
        return web.json_response({"ok": False, "error": "http client unavailable"}, status=500)
      return web.json_response({"ok": True, **await add_github_ssh_keys(session, body.get("username", ""))})
  except SshKeyError as e:
    return web.json_response({"ok": False, "error": str(e)}, status=e.status)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e) or "GitHub request failed"}, status=502)

  return web.json_response({"ok": False, "error": "bad action"}, status=400)


def register(app: web.Application) -> None:
  app.router.add_get("/api/ssh_keys", api_ssh_keys_status)
  app.router.add_post("/api/ssh_keys", api_ssh_keys)
