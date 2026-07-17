"""Carrot intro HTTP routes.

  GET  /api/intro/state          is the intro due? (also exposed via bootstrap)
  POST /api/intro/complete       record completion
  POST /api/intro/apply_preset   apply one driving-control preset in one shot
  POST /api/intro/reset          clear the flag (debug / "show intro again")

Everything else the intro needs already exists:
  GET  /api/cars                 features/cars.py
  POST /api/param_set            features/params.py      (car, HDA)
  POST /api/web_settings         features/web_settings.py (language)
  POST /api/params_restore       features/params.py      (backup restore)
  POST /api/reboot               features/system.py
"""
from __future__ import annotations

import asyncio
from typing import Any, Dict

from aiohttp import web

from ...services.params import HAS_PARAMS, clamp_numeric, set_param_value
from ...services.settings import get_settings_cached
from .presets import PRESET_NAMES, get_preset
from .state import (
  REASON_FINISHED,
  intro_bootstrap,
  mark_completed,
  read_intro_state,
)


def _clamped(name: str, value: Any) -> Any:
  """Same clamp features/params.py:api_param_set applies, so a preset can
  never write a value the settings UI would reject."""
  try:
    _, _, by_name, _ = get_settings_cached()
    p = by_name.get(name)
  except Exception:
    p = None
  if p is None:
    return value, None
  try:
    if isinstance(p.get("min"), (int, float)) and isinstance(p.get("max"), (int, float)):
      fv = clamp_numeric(float(value), p)
      if all(isinstance(p.get(k), int) for k in ("min", "max", "default")):
        return int(round(fv)), p
      return fv, p
  except Exception:
    pass
  return value, p


def _apply_preset_sync(name: str) -> Dict[str, Any]:
  """Write every param of the preset. Runs in a thread — Params writes hit
  the filesystem and would otherwise block the event loop."""
  values = get_preset(name)
  if values is None:
    raise ValueError(f"unknown preset: {name}")

  applied: Dict[str, Any] = {}
  failed: Dict[str, str] = {}
  for param, raw in values.items():
    try:
      value, p = _clamped(param, raw)
      set_param_value(param, value, p)
      applied[param] = value
    except Exception as exc:
      failed[param] = str(exc)
  return {"applied": applied, "failed": failed}


async def api_state(_request: web.Request) -> web.Response:
  return web.json_response({"ok": True, **intro_bootstrap(), "state": read_intro_state()})


async def api_complete(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    body = {}
  reason = str((body or {}).get("reason") or REASON_FINISHED)
  return web.json_response({"ok": True, "state": mark_completed(reason)})


async def api_reset(_request: web.Request) -> web.Response:
  """Clear the flag so the intro runs again. Used by the settings entry point
  and while developing — without it a device can only ever see the intro once."""
  from .state import CARROT_INTRO_STATE_PATH
  import os

  try:
    if os.path.exists(CARROT_INTRO_STATE_PATH):
      os.remove(CARROT_INTRO_STATE_PATH)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)
  return web.json_response({"ok": True, **intro_bootstrap()})


async def api_apply_preset(request: web.Request) -> web.Response:
  if not HAS_PARAMS:
    return web.json_response({"ok": False, "error": "Params not available"}, status=500)

  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  preset = str((body or {}).get("preset") or "").strip()
  if preset not in PRESET_NAMES:
    return web.json_response(
      {"ok": False, "error": f"unknown preset: {preset}", "known": list(PRESET_NAMES)},
      status=400,
    )

  try:
    result = await asyncio.to_thread(_apply_preset_sync, preset)
  except Exception as exc:
    return web.json_response({"ok": False, "error": str(exc)}, status=500)

  # Report partial failure honestly rather than claiming success — these are
  # vehicle control params and the caller needs to know if one did not land.
  if result["failed"]:
    return web.json_response(
      {"ok": False, "error": "some params failed", "preset": preset, **result},
      status=500,
    )
  return web.json_response({"ok": True, "preset": preset, **result})


def register(app: web.Application) -> None:
  app.router.add_get("/api/intro/state", api_state)
  app.router.add_post("/api/intro/complete", api_complete)
  app.router.add_post("/api/intro/reset", api_reset)
  app.router.add_post("/api/intro/apply_preset", api_apply_preset)
