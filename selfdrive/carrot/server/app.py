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

from cereal import messaging

from ..realtime.transports import CameraWsHub, RawWsHub
from . import features
from .config import WEB_DIR
from .live_runtime.broker import RealtimeBroker
from .services.git_status import git_status_loop
from .services.heartbeat import heartbeat_loop
from .services.params import HAS_PARAMS


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
  """gc.collect + malloc_trim — runs in thread pool (GIL acquired there)."""
  import gc as _gc
  _gc.collect()
  try:
    import ctypes
    libc = ctypes.CDLL("libc.so.6")
    libc.malloc_trim(0)
  except Exception:
    pass


async def _malloc_trim_loop():
  """Periodic gc + malloc_trim to reclaim leaked objects and return C heap.
  Runs via to_thread so the event loop is never blocked."""
  while True:
    await asyncio.sleep(30.0)
    await asyncio.to_thread(_do_gc_and_trim)


async def on_startup(app: web.Application) -> None:
  app["http"] = ClientSession()
  app["hb_last"] = {"ok": None, "msg": "not yet", "ts": 0}
  # Eager broker creation — single SubMaster via RealtimeBroker
  try:
    broker = RealtimeBroker(repo_flavor="c3")
    app["realtime_broker"] = broker
    app["realtime_broker_error"] = None
  except Exception as exc:
    app["realtime_broker"] = None
    app["realtime_broker_error"] = str(exc)
  app["realtime_camera_hub"] = CameraWsHub(messaging)
  app["realtime_raw_hub"] = RawWsHub(messaging)
  if HAS_PARAMS:
    app["hb_task"] = asyncio.create_task(heartbeat_loop(app))
  app["git_status_task"] = asyncio.create_task(git_status_loop())
  asyncio.create_task(_malloc_trim_loop())


async def on_cleanup(app: web.Application) -> None:
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

  sess = app.get("http")
  if sess:
    await sess.close()


def make_app() -> web.Application:
  app = web.Application(middlewares=[log_mw])
  app.on_startup.append(on_startup)
  app.on_cleanup.append(on_cleanup)

  features.register_all(app)

  # foldered static assets — must come after explicit routes so /api/...,
  # /ws/..., /download/... win the match.
  app.router.add_static("/", str(WEB_DIR), show_index=True)
  return app
