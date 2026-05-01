import asyncio
import os
import subprocess
from typing import Any

from aiohttp import web

from ..live_runtime.broker import RealtimeBroker
from ..live_runtime.normalize import to_transport_safe
from ..config import OFFROAD_ASSETS_DIR
from ..services.device_info import get_device_info
from ..services.params import HAS_PARAMS, Params, set_param_value
from ..services.time_sync import TIME_SYNC_DEBUG_DEFAULT, sync_system_time_from_browser


_LIVE_RUNTIME_SERVICE_NAMES = (
  "selfdriveState",
  "carState",
  "controlsState",
  "deviceState",
  "peripheralState",
  "longitudinalPlan",
  "lateralPlan",
  "radarState",
  "carrotMan",
)


def _select_live_runtime_services(snapshot: dict[str, Any]) -> dict[str, Any]:
  services = snapshot.get("services")
  if not isinstance(services, dict):
    return {}
  out: dict[str, Any] = {}
  for name in _LIVE_RUNTIME_SERVICE_NAMES:
    value = services.get(name)
    if isinstance(value, dict):
      out[name] = value
  return out


def _is_drive_engaged(request: web.Request) -> bool:
  broker: RealtimeBroker | None = request.app.get("realtime_broker")
  snapshot = broker.last_snapshot if broker is not None and isinstance(broker.last_snapshot, dict) else {}
  services = snapshot.get("services") if isinstance(snapshot, dict) else {}
  if not isinstance(services, dict):
    return False

  for service_name in ("selfdriveState", "controlsState"):
    service = services.get(service_name)
    if isinstance(service, dict) and bool(service.get("enabled")):
      return True
  return False


def _reject_if_engaged(request: web.Request) -> web.Response | None:
  if not _is_drive_engaged(request):
    return None
  return web.json_response({"ok": False, "error": "Disengage first"}, status=409)


async def api_heartbeat_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, "hb": request.app.get("hb_last")})


async def api_live_runtime(request: web.Request) -> web.Response:
  broker: RealtimeBroker | None = request.app.get("realtime_broker")
  broker_error = request.app.get("realtime_broker_error")
  if broker is None:
    return web.json_response({"ok": False, "error": broker_error or "realtime broker unavailable"}, status=503)

  force = request.query.get("force") == "1"
  runtime = broker.last_snapshot.get("runtime") if isinstance(broker.last_snapshot, dict) else None
  if not isinstance(runtime, dict):
    runtime = {}

  age_ms = broker.snapshot_age_ms()
  if force or age_ms is None or age_ms > 250 or not runtime.get("params"):
    try:
      await asyncio.to_thread(broker.poll, 0)
    except Exception as exc:
      return web.json_response({"ok": False, "error": str(exc)}, status=500)
    runtime = broker.last_snapshot.get("runtime") if isinstance(broker.last_snapshot, dict) else {}

  meta = broker.last_snapshot.get("meta") if isinstance(broker.last_snapshot, dict) else {}
  services = _select_live_runtime_services(broker.last_snapshot if isinstance(broker.last_snapshot, dict) else {})
  return web.json_response(to_transport_safe({
    "ok": True,
    "meta": meta if isinstance(meta, dict) else {},
    "runtime": runtime if isinstance(runtime, dict) else {},
    "services": services,
    "snapshotAgeMs": broker.snapshot_age_ms(),
  }))


async def api_reboot(request: web.Request) -> web.Response:
  blocked = _reject_if_engaged(request)
  if blocked is not None:
    return blocked
  try:
    if HAS_PARAMS:
      Params().put_bool("DoReboot", True)
    else:
      # Params-less development fallback only.
      subprocess.Popen(["sudo", "reboot"])
    return web.json_response({"ok": True})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_time_sync(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception as e:
    return web.json_response({"ok": False, "error": f"bad json: {e}"}, status=400)

  epoch_ms = body.get("epoch_ms")
  timezone_name = (body.get("timezone") or "").strip()
  debug = bool(body.get("debug", False))
  client_iso = body.get("client_iso")

  if not isinstance(epoch_ms, (int, float)):
    return web.json_response({"ok": False, "error": "epoch_ms required"}, status=400)

  if not timezone_name:
    timezone_name = "UTC"

  effective_debug = debug or TIME_SYNC_DEBUG_DEFAULT
  if effective_debug:
    print(f"[time_sync] client={request.remote} timezone={timezone_name} client_iso={client_iso} debug={debug}")

  result = await asyncio.to_thread(
    sync_system_time_from_browser,
    int(epoch_ms),
    timezone_name,
    effective_debug,
  )

  if effective_debug:
    print(
      f"[time_sync] result ok={result.get('ok')} "
      f"applied={result.get('applied')} "
      f"diff_sec={result.get('diff_sec')} "
      f"message={result.get('message')}"
    )

  status = 200 if result.get("ok") else 500
  return web.json_response(result, status=status)


async def api_device_info(request: web.Request) -> web.Response:
  try:
    info = await asyncio.to_thread(get_device_info)
    return web.json_response({"ok": True, **info})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_regulatory(request: web.Request) -> web.Response:
  try:
    path = os.path.join(OFFROAD_ASSETS_DIR, "fcc.html")
    if not os.path.isfile(path):
      return web.json_response({"ok": False, "error": "regulatory info unavailable"}, status=404)

    def read_file() -> str:
      with open(path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()

    html = await asyncio.to_thread(read_file)
    return web.json_response({"ok": True, "html": html})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_poweroff(request: web.Request) -> web.Response:
  blocked = _reject_if_engaged(request)
  if blocked is not None:
    return blocked
  if not HAS_PARAMS:
    return web.json_response({"ok": False, "error": "params unavailable"}, status=500)
  try:
    Params().put_bool("DoShutdown", True)
    return web.json_response({"ok": True})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_recalibrate(request: web.Request) -> web.Response:
  blocked = _reject_if_engaged(request)
  if blocked is not None:
    return blocked
  if not HAS_PARAMS:
    return web.json_response({"ok": False, "error": "params unavailable"}, status=500)
  try:
    params = Params()
    for key in ("CalibrationParams", "LiveTorqueParameters",
                "LiveParameters", "LiveParametersV2", "LiveDelay"):
      try:
        params.remove(key)
      except Exception:
        pass
    # Request onroad cycle restart if available
    try:
      params.put_bool("OnroadCycleRequested", True)
    except Exception:
      pass
    try:
      params.put_bool("DoReboot", True)
    except Exception:
      pass
    return web.json_response({"ok": True})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_set_default(request: web.Request) -> web.Response:
  if not HAS_PARAMS:
    return web.json_response({"ok": False, "error": "params unavailable"}, status=500)
  try:
    Params().put_int("SoftRestartTriggered", 2)
    return web.json_response({"ok": True})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


def register(app: web.Application) -> None:
  app.router.add_get("/api/heartbeat_status", api_heartbeat_status)
  app.router.add_get("/api/live_runtime", api_live_runtime)
  app.router.add_get("/api/device_info", api_device_info)
  app.router.add_get("/api/regulatory", api_regulatory)
  app.router.add_post("/api/reboot", api_reboot)
  app.router.add_post("/api/poweroff", api_poweroff)
  app.router.add_post("/api/recalibrate", api_recalibrate)
  app.router.add_post("/api/set_default", api_set_default)
  app.router.add_post("/api/time_sync", api_time_sync)
