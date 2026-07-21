import asyncio
import os

from aiohttp import web

from ..config import UNIT_CYCLE
from ..services.device_info import (
  get_device_network_snapshot,
  get_device_setting_group_names,
  get_device_setting_values,
)
from ..services.param_changes import observe_param_values
from ..services.params import HAS_PARAMS, ParamKeyType, Params, get_param_values
from ..services.popular_values import read_popular_values_memory, schedule_popular_value_refresh
from ..services.setting_favorites import read_setting_favorites
from ..services.setting_profiles import read_setting_profiles
from ..services.setting_unit_index import read_setting_unit_index, update_setting_unit_index
from ..services.settings import get_settings_cached, settings_cache
from ..services.ssh_keys import get_ssh_key_status


def _settings_catalog_payload(cache_parts: tuple | None = None) -> dict:
  path = settings_cache["path"]
  data, groups, _by_name, groups_list = cache_parts or get_settings_cached()
  return {
    "path": path,
    "apilot": data.get("apilot"),
    "groups": groups_list,
    "items_by_group": dict(groups),
    "categories": settings_cache.get("categories"),
    "unit_cycle": UNIT_CYCLE,
    "has_params": HAS_PARAMS,
    "has_param_type": bool(ParamKeyType is not None and hasattr(Params(), "get_type")) if HAS_PARAMS else False,
  }


def _settings_snapshot_payload() -> dict:
  cache_parts = get_settings_cached()
  _data, _groups, by_name, _groups_list = cache_parts
  values = get_param_values(
    list(by_name),
    {name: meta.get("default", 0) for name, meta in by_name.items()},
  )
  # Entering settings is exactly when a value changed elsewhere would confuse
  # the user, so this is the read worth comparing against what we last knew.
  observe_param_values(values)
  ssh_status = get_ssh_key_status()
  return {
    "ok": True,
    "settings": _settings_catalog_payload(cache_parts),
    "values": values,
    "device_values": get_device_setting_values(ssh_status),
    # The web reads its Device tab parameter names from here rather than
    # restating them, so the two lists cannot drift apart.
    "device_groups": get_device_setting_group_names(),
    "device_network": get_device_network_snapshot(),
    "device_ssh": ssh_status,
    "favorites": read_setting_favorites().get("favorites", []),
    "profiles": read_setting_profiles().get("profiles", []),
    "popular": read_popular_values_memory(),
    # Shipped with the snapshot so restoring the step multipliers costs no
    # extra round trip on first entry.
    "unit_index": read_setting_unit_index().get("units", {}),
  }


def _compressed_json_response(payload: dict) -> web.Response:
  response = web.json_response(payload)
  response.headers["Cache-Control"] = "no-store"
  response.enable_compression()
  return response


def _missing_settings_response() -> web.Response | None:
  path = settings_cache["path"]
  if os.path.exists(path):
    return None
  return web.json_response({"ok": False, "error": f"settings file not found: {path}"}, status=404)


async def api_settings(request: web.Request) -> web.Response:
  missing = _missing_settings_response()
  if missing is not None:
    return missing

  try:
    return _compressed_json_response({"ok": True, **_settings_catalog_payload()})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_settings_snapshot(request: web.Request) -> web.Response:
  """Return all data required for the first settings entry in one request.

  The catalog is process-cached by services.settings. Favorites, profiles and
  popular values remain live per-device data, but are read together so the
  client does not create a four-request render barrier on its first visit.
  """
  missing = _missing_settings_response()
  if missing is not None:
    return missing

  try:
    schedule_popular_value_refresh(request.app)
    return _compressed_json_response(await asyncio.to_thread(_settings_snapshot_payload))
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_setting_unit_index(request: web.Request) -> web.Response:
  units = await asyncio.to_thread(read_setting_unit_index)
  return web.json_response({"ok": True, "units": units.get("units", {})})


async def api_setting_unit_index_update(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  try:
    saved = await asyncio.to_thread(update_setting_unit_index, body)
    return web.json_response({"ok": True, "units": saved.get("units", {})})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


def register(app: web.Application) -> None:
  app.router.add_get("/api/settings", api_settings)
  app.router.add_get("/api/settings/snapshot", api_settings_snapshot)
  app.router.add_get("/api/setting_unit_index", api_setting_unit_index)
  app.router.add_post("/api/setting_unit_index", api_setting_unit_index_update)
