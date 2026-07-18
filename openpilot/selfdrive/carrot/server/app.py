"""
Carrot web server composition root.

Wires the aiohttp Application together:
- request log middleware
- startup/cleanup hooks (broker, hubs, heartbeat, malloc trim)
- feature route registration
- static asset fallback
"""
from __future__ import annotations

import asyncio
import time
import traceback

from aiohttp import ClientSession, web

from openpilot.cereal import messaging

from ..realtime.transports import CameraWsHub, RawWsHub
from . import features
from .config import SELFDRIVE_ASSETS_DIR, WEB_DIR, migrate_legacy_carrot_state
from .live_runtime.broker import RealtimeBroker
from .services.auto_update import auto_update_loop
from .services.git_status import git_status_loop
from .services.heartbeat import heartbeat_loop
from .services.params import HAS_PARAMS, Params
from .services.popular_values import start_popular_value_upload
from .services.settings import get_settings_cached
from .services.static_assets import create_static_cache_middleware, start_precompress

VISION_DIAG_UPLOAD_MAX_BYTES = 16 * 1024 * 1024


# ===== request log middleware =====
@web.middleware
async def log_mw(request, handler):
  ua = request.headers.get("User-Agent", "")
  ip = request.remote
  t0 = time.time()
  try:
    resp = await handler(request)
    return resp
  finally:
    #dt = (time.time() - t0) * 1000
    #print(f"[REQ] {ip} {request.method} {request.path_qs} {dt:.1f}ms UA={ua[:80]}")
    pass


def _do_gc_and_trim() -> None:
  """gc.collect + malloc_trim ??runs in thread pool (GIL acquired there)."""
  import gc as _gc
  _gc.collect()
  try:
    import ctypes
    libc = ctypes.CDLL("libc.so.6")
    libc.malloc_trim(0)
  except Exception:
    pass


async def _malloc_trim_loop(app: web.Application):
  """Periodic gc + malloc_trim to reclaim leaked objects and return C heap.
  Runs via to_thread so the event loop is never blocked."""
  while True:
    await asyncio.sleep(30.0)
    raw_hub = app.get("realtime_raw_hub")
    if raw_hub is not None and raw_hub.client_count() > 0:
      continue
    params = app.get("params")
    try:
      if params is not None and params.get_bool("CarrotVisionActive"):
        continue
    except Exception:
      pass
    await asyncio.to_thread(_do_gc_and_trim)


async def _warm_settings_cache() -> None:
  """Build the /api/settings cache (read + parse the ~100 KB settings file and
  the menu tree) at startup so the first settings-page open doesn't pay for it
  on its critical path. Best-effort: the file may not exist yet."""
  try:
    await asyncio.to_thread(get_settings_cached)
  except Exception:
    pass


async def on_startup(app: web.Application) -> None:
  app["http"] = ClientSession()
  app["params"] = Params() if HAS_PARAMS and Params is not None else None
  app["hb_last"] = {"ok": None, "msg": "not yet", "ts": 0}
  # Keep only route metadata plus the server-side engagement safety signal
  # outside the compact HUD/overlay relay.
  try:
    broker = RealtimeBroker(
      repo_flavor="c3",
      include_optional=("navInstructionCarrot", "navRoute"),
      exclude_services=(
        "carState", "controlsState", "longitudinalPlan",
        "liveCalibration", "modelV2", "roadCameraState", "deviceState",
      ),
    )
    app["realtime_broker"] = broker
    app["realtime_broker_error"] = None
  except Exception as exc:
    app["realtime_broker"] = None
    app["realtime_broker_error"] = str(exc)
  # Serializes broker.poll() across concurrent /api/live_runtime requests.
  # SubMaster (msgq) is not thread-safe, so two parallel polls can crash it
  # and take the whole server down.
  app["realtime_broker_poll_lock"] = asyncio.Lock()
  app["realtime_camera_hub"] = CameraWsHub(messaging)
  app["realtime_raw_hub"] = RawWsHub(messaging)
  if HAS_PARAMS:
    app["hb_task"] = asyncio.create_task(heartbeat_loop(app))
  app["git_status_task"] = asyncio.create_task(git_status_loop())
  app["auto_update_task"] = asyncio.create_task(auto_update_loop())
  app["popular_value_upload_task"] = start_popular_value_upload(app)
  app["malloc_trim_task"] = asyncio.create_task(_malloc_trim_loop(app))
  app["settings_warm_task"] = asyncio.create_task(_warm_settings_cache())
  app["precompress_task"] = start_precompress(str(WEB_DIR))


async def on_cleanup(app: web.Application) -> None:
  settings_warm_task = app.get("settings_warm_task")
  if settings_warm_task:
    settings_warm_task.cancel()
    try:
      await settings_warm_task
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  precompress_task = app.get("precompress_task")
  if precompress_task:
    precompress_task.cancel()
    try:
      await precompress_task
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  malloc_trim_task = app.get("malloc_trim_task")
  if malloc_trim_task:
    malloc_trim_task.cancel()
    try:
      await malloc_trim_task
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  realtime_camera_hub = app.get("realtime_camera_hub")
  if realtime_camera_hub is not None:
    try:
      await realtime_camera_hub.stop_all()
    except Exception:
      traceback.print_exc()

  realtime_raw_hub = app.get("realtime_raw_hub")
  if realtime_raw_hub is not None:
    try:
      await realtime_raw_hub.stop_all()
    except Exception:
      traceback.print_exc()

  t = app.get("hb_task")
  if t:
    t.cancel()
    try:
      await t
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  git_status_task = app.get("git_status_task")
  if git_status_task:
    git_status_task.cancel()
    try:
      await git_status_task
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  auto_update_task = app.get("auto_update_task")
  if auto_update_task:
    auto_update_task.cancel()
    try:
      await auto_update_task
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  popular_value_upload_task = app.get("popular_value_upload_task")
  if popular_value_upload_task:
    popular_value_upload_task.cancel()
    try:
      await popular_value_upload_task
    except asyncio.CancelledError:
      pass
    except Exception:
      pass

  sess = app.get("http")
  if sess:
    await sess.close()


def make_app() -> web.Application:
  # Bring forward user state (web settings, YouTube stream key, favorites) from
  # the old in-repo location before any service reads it, so upgrading devices
  # keep their settings instead of seeing defaults once.
  migrate_legacy_carrot_state()
  app = web.Application(
    middlewares=[log_mw, create_static_cache_middleware(str(WEB_DIR))],
    client_max_size=VISION_DIAG_UPLOAD_MAX_BYTES,
  )
  app.on_startup.append(on_startup)
  app.on_cleanup.append(on_cleanup)

  features.register_all(app)

  # Cluster and web HUDs consume one canonical set of icons/fonts. Register the
  # shared tree before the web-root fallback so the URL cannot be shadowed.
  app.router.add_static("/shared-assets/", str(SELFDRIVE_ASSETS_DIR), show_index=False)

  # foldered static assets ??must come after explicit routes so /api/...,
  # /ws/..., /download/... win the match.
  app.router.add_static("/", str(WEB_DIR), show_index=True)
  return app
