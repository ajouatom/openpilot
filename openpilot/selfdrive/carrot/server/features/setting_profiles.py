from aiohttp import web

from ..services.setting_profiles import (
  apply_setting_profile,
  create_setting_profile,
  delete_setting_profile,
  preview_setting_profile,
  read_setting_profiles,
  update_setting_profile,
)


async def get_setting_profiles(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **read_setting_profiles()})


async def create_setting_profile_route(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  try:
    profile = create_setting_profile(body.get("name", ""))
    return web.json_response({"ok": True, "profile": profile, **read_setting_profiles()})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


async def update_setting_profile_route(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  profile_id = str(body.get("id") or "").strip()
  if not profile_id:
    return web.json_response({"ok": False, "error": "missing profile id"}, status=400)
  try:
    profile = update_setting_profile(profile_id, body)
    return web.json_response({"ok": True, "profile": profile, **read_setting_profiles()})
  except KeyError as e:
    return web.json_response({"ok": False, "error": str(e)}, status=404)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


async def delete_setting_profile_route(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  profile_id = str(body.get("id") or "").strip()
  if not profile_id:
    return web.json_response({"ok": False, "error": "missing profile id"}, status=400)
  try:
    delete_setting_profile(profile_id)
    return web.json_response({"ok": True, **read_setting_profiles()})
  except KeyError as e:
    return web.json_response({"ok": False, "error": str(e)}, status=404)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


async def preview_setting_profile_route(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  profile_id = str(body.get("id") or "").strip()
  if not profile_id:
    return web.json_response({"ok": False, "error": "missing profile id"}, status=400)
  try:
    preview = preview_setting_profile(profile_id, body.get("values") if isinstance(body, dict) else None)
    return web.json_response({"ok": True, "preview": preview})
  except KeyError as e:
    return web.json_response({"ok": False, "error": str(e)}, status=404)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


async def apply_setting_profile_route(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  profile_id = str(body.get("id") or "").strip()
  if not profile_id:
    return web.json_response({"ok": False, "error": "missing profile id"}, status=400)
  try:
    result = apply_setting_profile(profile_id, body.get("values") if isinstance(body, dict) else None)
    return web.json_response({"ok": True, **result})
  except KeyError as e:
    return web.json_response({"ok": False, "error": str(e)}, status=404)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=400)


def register(app: web.Application) -> None:
  app.router.add_get("/api/setting_profiles", get_setting_profiles)
  app.router.add_post("/api/setting_profiles", create_setting_profile_route)
  app.router.add_post("/api/setting_profiles/update", update_setting_profile_route)
  app.router.add_post("/api/setting_profiles/delete", delete_setting_profile_route)
  app.router.add_post("/api/setting_profiles/preview", preview_setting_profile_route)
  app.router.add_post("/api/setting_profiles/apply", apply_setting_profile_route)
