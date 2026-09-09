from __future__ import annotations

from typing import Any

from aiohttp import web

from openpilot.selfdrive.modeld.big_model import active_manifest, active_model_compiled, active_model_path, model_cache_dir
from openpilot.selfdrive.modeld.big_model_status import read_big_model_status, write_big_model_status

from ..services.params import HAS_PARAMS, Params


def _params_bool(params: Any, key: str) -> bool:
  try:
    return bool(params.get_bool(key))
  except Exception:
    return False


def build_status_payload(params: Any | None = None) -> dict[str, Any]:
  params = params if params is not None else (Params() if HAS_PARAMS and Params is not None else None)
  hardware_seen = _params_bool(params, "UsbGpuHardwareSeen") if params is not None else False
  if not hardware_seen:
    return {"ok": True, "available": False}

  status = read_big_model_status(model_cache_dir()) or {}
  manifest = active_manifest()
  # Status describes the model being updated; active_manifest can still refer
  # to the previous model until the new download passes verification.
  same_model = manifest is not None and (not status or status.get("sha256") == manifest.sha256)
  phase = status.get("state")
  updating = phase in {"checking", "downloading", "verifying", "compiling", "error"}
  try:
    compiled = active_model_compiled() if same_model and not updating else False
  except Exception:
    compiled = False
  if updating:
    state = phase
  elif compiled:
    state = "compiled"
  elif same_model:
    state = "waiting_for_ignition" if phase == "waiting_for_ignition" else "ready"
  else:
    state = "checking"
  fallback_size = manifest.size if same_model else 0
  downloaded = int(status.get("downloaded_bytes", fallback_size))
  total = int(status.get("total_bytes", fallback_size))
  progress = round(min(100.0, max(0.0, downloaded * 100.0 / total)), 1) if total > 0 else None
  engaged = _params_bool(params, "IsEngaged") if params is not None else False

  return {
    "ok": True,
    "available": True,
    "state": state,
    "model_id": status.get("model_id") if status else (manifest.model_id if manifest is not None else None),
    "sha256": status.get("sha256") if status else (manifest.sha256 if manifest is not None else None),
    "downloaded_bytes": downloaded,
    "total_bytes": total,
    "progress": progress,
    "detail": status.get("detail"),
    "started_at": status.get("started_at"),
    "updated_at": status.get("updated_at"),
    "compiled": compiled,
    "engaged": engaged,
    "can_restart": same_model and not compiled and not engaged and state not in {
      "checking", "downloading", "verifying", "compiling",
    },
  }


async def api_status(_request: web.Request) -> web.Response:
  return web.json_response(build_status_payload())


async def api_compile_restart(_request: web.Request) -> web.Response:
  if not HAS_PARAMS or Params is None:
    return web.json_response({"ok": False, "error": "params unavailable"}, status=500)

  params = Params()
  payload = build_status_payload(params)
  if not payload.get("available"):
    return web.json_response({"ok": False, "error": "eGPU has never been connected"}, status=404)
  if payload.get("engaged"):
    return web.json_response({"ok": False, "error": "disengage openpilot and park before restarting"}, status=409)
  if not payload.get("can_restart"):
    return web.json_response({"ok": False, "error": "model is not ready for compilation", "state": payload.get("state")}, status=409)

  from openpilot.selfdrive.modeld.helpers import usbgpu_present

  if not usbgpu_present():
    return web.json_response({"ok": False, "error": "turn ignition on and wait for eGPU power"}, status=409)

  manifest = active_manifest()
  if manifest is None or active_model_path() is None:
    return web.json_response({"ok": False, "error": "verified model is unavailable"}, status=409)
  write_big_model_status(model_cache_dir(), "waiting_for_ignition", model_id=manifest.model_id,
                         sha256=manifest.sha256, downloaded_bytes=manifest.size, total_bytes=manifest.size,
                         detail="restart requested; compilation will begin during boot")
  params.put_bool("DoReboot", True)
  return web.json_response({"ok": True, "reboot_requested": True})


def register(app: web.Application) -> None:
  app.router.add_get("/api/egpu/model", api_status)
  app.router.add_post("/api/egpu/model/compile-restart", api_compile_restart)
