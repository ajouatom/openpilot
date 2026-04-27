#!/usr/bin/env python3
# /data/openpilot/selfdrive/carrot/carrot_server.py
#
# aiohttp dashboard:
# - Home / Setting
# - loads carrot_settings.json
# - group buttons
# - bulk values load (fast on phone)
# - typed param set (ParamKeyType 기반) with fallback inference
#
# Run:
#   python3 /data/openpilot/selfdrive/carrot/carrot_server.py --host 0.0.0.0 --port 7000
#
# Open:
#   http://<device_ip>:7000/

import argparse
import base64
import json
import os
import math
import time
from datetime import datetime
import asyncio
import glob
import subprocess
import traceback
import numpy as np
from typing import Dict, Any, Tuple, Optional, List

from aiohttp import web, ClientSession, ClientTimeout, WSMsgType
from cereal import messaging
from opendbc.car import structs
import shlex
import shutil
import socket
import urllib.request
import urllib.error
import ssl
import getpass
import uuid
import hashlib
import mimetypes
from ftplib import FTP
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware import HARDWARE

from ..realtime.raw_protocol import build_raw_hello, build_raw_multiplex_hello
from ..realtime.transports import CameraWsHub, RawWsHub
from .live_compat.broker import RealtimeBroker
from .live_compat.normalize import to_transport_safe

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)

DEFAULT_SETTINGS_PATH = "/data/openpilot/selfdrive/carrot_settings.json"
CARROT_DATA_DIR = "/data/openpilot/selfdrive/carrot/data"
CARROT_STATE_DIR = os.path.join(CARROT_DATA_DIR, "state")
CARROT_GIT_STATE_PATH = os.path.join(CARROT_STATE_DIR, "git.json")
DASHCAM_ROOT = "/data/media/0/realdata"
DASHCAM_CACHE_DIR = os.path.join(CARROT_DATA_DIR, "cache", "dashcam")
SCREEN_RECORDING_DIRS = (
  "/data/media/0/videos",
  "/data/media/0/screenrecord",
  "/data/media/0/screen_recordings",
  "/data/media/0/screenrecords",
  "/data/media/0/ScreenRecords",
  "/data/media/0/Movies",
  "/sdcard/Movies",
)
SCREEN_RECORDING_EXTS = (".mp4", ".mkv", ".avi", ".mov", ".ts", ".hevc")
DASHCAM_DEFAULT_DISCORD_WEBHOOK = (
  "CxUGAhxOAkMLDhACHQALWk4DAkgCERtdGBFPBAAICBJdQ1tNFV1aU1ZSS0JeRhVY"
  "Vl9RVV0WHD8eGyw3CCkTJQoeGyVCJTosGiEfMhgPVwJbCwEQVxBqCQBXJQk4BB9Z"
  "RUEoVxYELSNfWUgCOBUiF0s4HBpsIjcyLw"
)
DASHCAM_DEFAULT_DISCORD_KEY = "carrot-log"

WEB_DIR = os.path.join(ROOT_DIR, "web")
CSS_DIR = os.path.join(WEB_DIR, "css")
JS_DIR = os.path.join(WEB_DIR, "js")
ASSETS_DIR = os.path.join(WEB_DIR, "assets")
PAGES_DIR = os.path.join(WEB_DIR, "pages")

UNIT_CYCLE = [1, 2, 5, 10, 50, 100]

GearShifter = structs.CarState.GearShifter

# -----------------------
# Optional openpilot Params
# -----------------------
HAS_PARAMS = False
Params = None
ParamKeyType = None

try:
  from openpilot.common.params import Params as _Params
  Params = _Params
  HAS_PARAMS = True
except Exception:
  pass

# ParamKeyType는 fork/버전에 따라 위치가 다를 수 있어서 방어적으로 처리
if HAS_PARAMS:
  try:
    # 일부 환경에서는 openpilot.common.params에 ParamKeyType가 있을 수 있음
    from openpilot.common.params import ParamKeyType as _ParamKeyType
    ParamKeyType = _ParamKeyType
  except Exception:
    ParamKeyType = None


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


WEBRTCD_URL = "http://127.0.0.1:5001/stream"
TMUX_WEB_SESSION = "carrot-web"
TMUX_CAPTURE_LINES = 160
TMUX_START_DIR = "/data/openpilot"

def _get_local_ip() -> str:
  try:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
      s.connect(("8.8.8.8", 80))
      return s.getsockname()[0]
  except Exception:
    # fallback: hostname 방식(가끔 127.0.1.1 나올 수 있음)
    try:
      return socket.gethostbyname(socket.gethostname())
    except Exception:
      return "0.0.0.0"


def _register_my_ip_sync(params: "Params") -> tuple[bool, str]:
  """
  기존 carrot_man.py의 register_my_ip()를 그대로 옮긴 버전 (동기)
  """
  try:
    token = "12345678"
    local_ip = _get_local_ip()
    version = params.get("Version")
    github_id = params.get("GithubUsername")
    port = 7000
    is_onroad = params.get_bool("IsOnroad")
    url = "https://shind0.synology.me/carrot/api_heartbeat.php"
    timeout_s = 3.5

    payload = {
      "github_id": github_id,
      "token": token,
      "local_ip": local_ip,
      "port": int(port),
      "version": version,
      "is_onroad": bool(is_onroad),
      "ts": int(time.time()),
    }

    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
      url=url,
      data=data,
      headers={"Content-Type": "application/json"},
      method="POST",
    )

    ctx = ssl._create_unverified_context()
    with urllib.request.urlopen(req, timeout=timeout_s, context=ctx) as resp:
      body = resp.read().decode("utf-8", errors="replace")
      return (200 <= resp.status < 300), body

  except urllib.error.HTTPError as e:
    try:
      body = e.read().decode("utf-8", errors="replace")
    except Exception:
      body = ""
    return False, f"HTTPError {e.code}: {body}"
  except Exception as e:
    return False, f"Exception: {e}"


async def heartbeat_loop(app: web.Application):
  """
  aiohttp startup에서 create_task로 돌릴 백그라운드 루프
  - 이벤트 루프 블로킹 방지 위해 to_thread 사용
  """
  if not HAS_PARAMS:
    app["hb_last"] = {"ok": False, "msg": "Params not available"}
    return

  params = Params()
  interval_s = 30.0  # 기존: frame%(20*30) = 30초
  while True:
    try:
      ok, msg = await asyncio.to_thread(_register_my_ip_sync, params)
      app["hb_last"] = {
        "ok": bool(ok),
        "msg": str(msg)[:800],
        "ts": time.time(),
        "local_ip": _get_local_ip(),
      }
      # 원하면 로그
      # print(f"[heartbeat] ok:{ok}, msg:{msg}")
    except asyncio.CancelledError:
      break
    except Exception as e:
      app["hb_last"] = {"ok": False, "msg": f"Exception: {e}", "ts": time.time()}
    await asyncio.sleep(interval_s)


async def proxy_stream(request: web.Request) -> web.StreamResponse:
  body = await request.read()
  ct = request.headers.get("Content-Type", "application/json")

  sess: ClientSession = request.app["http"]

  try:
    async with sess.post(WEBRTCD_URL, data=body, headers={"Content-Type": ct},
                         timeout=ClientTimeout(total=15)) as resp:
      resp_body = await resp.read()
      out = web.Response(body=resp_body, status=resp.status)
      rct = resp.headers.get("Content-Type")
      if rct:
        out.headers["Content-Type"] = rct
      return out
  except asyncio.TimeoutError:
    return web.json_response({"ok": False, "error": "webrtcd timeout"}, status=504)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=502)

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


def _dashcam_safe_segment(segment: str) -> str:
  segment = (segment or "").strip()
  if not segment or "/" in segment or "\\" in segment or segment in {".", ".."}:
    raise web.HTTPBadRequest(text="bad segment")
  parts = segment.split("--")
  if len(parts) < 2 or not parts[-1].isdigit():
    raise web.HTTPBadRequest(text="bad segment")
  return segment


def _dashcam_segment_index(segment: str) -> int:
  try:
    return int(segment.split("--")[-1])
  except Exception:
    return 0


def _dashcam_route_name(segment: str) -> str:
  try:
    return "--".join(str(segment or "").split("--")[:-1])
  except Exception:
    return str(segment or "")


def _dashcam_file_size_label(size: int) -> str:
  try:
    n = float(size)
  except Exception:
    return "-"
  if n < 1024:
    return f"{int(n)} B"
  if n < 1024 * 1024:
    return f"{n / 1024:.1f} KB"
  if n < 1024 * 1024 * 1024:
    return f"{n / (1024 * 1024):.1f} MB"
  return f"{n / (1024 * 1024 * 1024):.1f} GB"


def _dashcam_segment_dir(segment: str) -> str:
  segment = _dashcam_safe_segment(segment)
  root = os.path.abspath(DASHCAM_ROOT)
  path = os.path.abspath(os.path.join(root, segment))
  if not path.startswith(root + os.sep):
    raise web.HTTPBadRequest(text="bad segment path")
  if not os.path.isdir(path):
    raise web.HTTPNotFound(text="segment not found")
  return path


def _dashcam_has_source_video(segment_dir: str) -> bool:
  for name in ("qcamera.mp4", "qcamera.ts"):
    path = os.path.join(segment_dir, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return True
  return False


def _dashcam_cache_path(kind: str, segment: str, ext: str) -> str:
  token = hashlib.sha1(segment.encode("utf-8", errors="ignore")).hexdigest()[:24]
  directory = os.path.join(DASHCAM_CACHE_DIR, kind)
  os.makedirs(directory, exist_ok=True)
  return os.path.join(directory, f"{token}{ext}")


def _dashcam_source_video(segment_dir: str) -> tuple[str, str]:
  # Prefer MP4 for browser playback, but keep TS as the canonical logger output.
  for name in ("qcamera.mp4", "qcamera.ts"):
    path = os.path.join(segment_dir, name)
    if os.path.isfile(path) and os.path.getsize(path) > 0:
      return path, name
  raise web.HTTPNotFound(text="qcamera video not found")


def _dashcam_route_date_label(route: str) -> str:
  try:
    if "-" in route and "--" in route:
      parts = route.split("--")
      if len(parts) >= 2:
        date = parts[0]
        t = parts[1].split("-")
        if len(t) >= 2:
          return f"{date} {t[0]}:{t[1]}"
        return date
    compact = route.split("--")
    if len(compact) >= 2:
      raw_date, raw_time = compact[0], compact[1]
      if len(raw_date) >= 8 and len(raw_time) >= 4:
        return f"{raw_date[:4]}-{raw_date[4:6]}-{raw_date[6:8]} {raw_time[:2]}:{raw_time[2:4]}"
  except Exception:
    pass
  return route


def _dashcam_relative_time(epoch_seconds: int) -> str:
  if epoch_seconds <= 0:
    return "-"
  delta = max(0, int(time.time()) - int(epoch_seconds))
  if delta < 60:
    return "방금 전"
  if delta < 3600:
    return f"{delta // 60}분 전"
  if delta < 86400:
    return f"{delta // 3600}시간 전"
  return f"{delta // 86400}일 전"


def _dashcam_build_routes() -> list[dict[str, Any]]:
  if not os.path.isdir(DASHCAM_ROOT):
    return []

  route_segments: dict[str, list[str]] = {}
  route_modified: dict[str, int] = {}
  with os.scandir(DASHCAM_ROOT) as it:
    for entry in it:
      try:
        if not entry.is_dir(follow_symlinks=False) or "--" not in entry.name:
          continue
        if not _dashcam_has_source_video(entry.path):
          continue
        parts = entry.name.split("--")
        if len(parts) < 2 or not parts[-1].isdigit():
          continue
        route = "--".join(parts[:-1])
        route_segments.setdefault(route, []).append(entry.name)
        modified = int(entry.stat(follow_symlinks=False).st_mtime)
        if modified > route_modified.get(route, 0):
          route_modified[route] = modified
      except Exception:
        continue

  routes: list[dict[str, Any]] = []
  for route, segments in route_segments.items():
    sorted_segments = sorted(segments, key=lambda s: (_dashcam_segment_index(s), s))
    latest = route_modified.get(route, 0)
    routes.append({
      "route": route,
      "title": route.lstrip("0") or route,
      "dateLabel": _dashcam_route_date_label(route),
      "segmentFolders": sorted_segments,
      "segmentCount": len(sorted_segments),
      "latestModifiedEpoch": latest,
      "latestModifiedLabel": _dashcam_relative_time(latest),
    })
  routes.sort(key=lambda r: (r.get("route", ""), r.get("latestModifiedEpoch", 0)), reverse=True)
  return routes


def _dashcam_run_ffmpeg(args: list[str], timeout: float = 90.0) -> subprocess.CompletedProcess:
  if not shutil.which("ffmpeg"):
    raise web.HTTPServiceUnavailable(text="ffmpeg not available")
  return subprocess.run(
    ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", *args],
    capture_output=True,
    text=True,
    timeout=timeout,
  )


def _dashcam_placeholder_svg(token: str = "dashcam") -> str:
  out = _dashcam_cache_path("placeholder", token, ".svg")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  svg = """<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">
<rect width="640" height="360" fill="#10161d"/>
<rect x="1" y="1" width="638" height="358" fill="none" stroke="#354252" stroke-width="2"/>
<path d="M296 126h48l20 24h44a24 24 0 0 1 24 24v72a24 24 0 0 1-24 24H232a24 24 0 0 1-24-24v-72a24 24 0 0 1 24-24h44z" fill="#253241"/>
<circle cx="320" cy="210" r="42" fill="#111820" stroke="#5d6c7d" stroke-width="8"/>
<path d="M306 188v44l38-22z" fill="#ffb268"/>
<text x="320" y="306" text-anchor="middle" fill="#9aa6b2" font-family="Arial, sans-serif" font-size="24" font-weight="700">NO THUMBNAIL</text>
</svg>"""
  with open(out, "w", encoding="utf-8") as f:
    f.write(svg)
  return out


def _dashcam_ensure_thumbnail(segment: str) -> str:
  segment_dir = _dashcam_segment_dir(segment)
  source, _ = _dashcam_source_video(segment_dir)
  out = _dashcam_cache_path("thumb", segment, ".jpg")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  attempts = (
    ["-ss", "2", "-i", source, "-vframes", "1", "-vf", "scale=640:-1", out],
    ["-ss", "0.2", "-i", source, "-vframes", "1", "-vf", "scale=640:-1", out],
  )
  for args in attempts:
    result = _dashcam_run_ffmpeg(args)
    if result.returncode == 0 and os.path.isfile(out) and os.path.getsize(out) > 0:
      return out
    try:
      if os.path.exists(out):
        os.remove(out)
    except OSError:
      pass
  return _dashcam_placeholder_svg(segment)


def _dashcam_ensure_preview(segment: str) -> str:
  segment_dir = _dashcam_segment_dir(segment)
  source, _ = _dashcam_source_video(segment_dir)
  out = _dashcam_cache_path("preview", segment, ".gif")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  result = _dashcam_run_ffmpeg([
    "-ss", "1",
    "-t", "2.4",
    "-i", source,
    "-vf", "fps=4,scale=360:-1:flags=lanczos",
    "-loop", "0",
    out,
  ], timeout=120.0)
  if result.returncode != 0 or not os.path.isfile(out) or os.path.getsize(out) <= 0:
    try:
      if os.path.exists(out):
        os.remove(out)
    except OSError:
      pass
    return _dashcam_ensure_thumbnail(segment)
  return out


def _dashcam_browser_video(segment: str) -> tuple[str, str]:
  segment_dir = _dashcam_segment_dir(segment)
  source, source_name = _dashcam_source_video(segment_dir)
  if source_name.endswith(".mp4"):
    return source, "video/mp4"

  out = _dashcam_cache_path("video", segment, ".mp4")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out, "video/mp4"

  result = _dashcam_run_ffmpeg(["-i", source, "-c", "copy", "-an", "-movflags", "+faststart", out], timeout=180.0)
  if result.returncode == 0 and os.path.isfile(out) and os.path.getsize(out) > 0:
    return out, "video/mp4"
  try:
    if os.path.exists(out):
      os.remove(out)
  except OSError:
    pass

  # Last-resort fallback: some browsers can still handle TS, and this preserves access.
  return source, "video/mp2t"


def _dashcam_param_text(params: "Params", key: str, default: str = "unknown") -> str:
  try:
    if not params:
      return default
    value = params.get(key)
    if isinstance(value, bytes):
      value = value.decode("utf-8", errors="replace")
    value = str(value or "").strip()
    return value or default
  except Exception:
    return default


def _dashcam_repo_dir() -> str:
  return os.environ.get("CARROT_REPO_DIR", "/data/openpilot")


def _dashcam_git_text(args: list[str], default: str = "") -> str:
  try:
    result = subprocess.run(
      ["git", *args],
      cwd=_dashcam_repo_dir(),
      capture_output=True,
      text=True,
      timeout=4,
    )
    if result.returncode == 0:
      value = (result.stdout or "").strip()
      return value or default
  except Exception:
    pass
  return default


def _dashcam_device_serial(params: Any) -> str:
  for key in ("HardwareSerial", "DeviceSerial", "Serial", "CarrotSerial"):
    value = _dashcam_param_text(params, key, "")
    if value:
      return value
  for env_key in ("CARROT_DEVICE_SERIAL", "DEVICE_SERIAL", "SERIAL"):
    value = os.environ.get(env_key, "").strip()
    if value:
      return value
  try:
    getter = getattr(HARDWARE, "get_serial", None)
    if callable(getter):
      value = str(getter() or "").strip()
      if value:
        return value
  except Exception:
    pass
  return "unknown"


def _dashcam_upload_metadata(params: Any) -> dict[str, str]:
  return {
    "carName": _dashcam_param_text(params, "CarName", "none"),
    "dongleId": _dashcam_param_text(params, "DongleId", "unknown"),
    "serial": _dashcam_device_serial(params),
    "branch": _dashcam_git_text(["branch", "--show-current"], "unknown"),
    "commit": _dashcam_git_text(["rev-parse", "--short", "HEAD"], "unknown"),
    "commitDate": _dashcam_git_text(["show", "-s", "--date=format:%Y-%m-%d %H:%M:%S", "--format=%cd", "HEAD"], "unknown"),
  }


def _dashcam_discord_webhook_url(params: Any) -> str:
  for key in ("CARROT_DISCORD_WEBHOOK_URL", "DISCORD_WEBHOOK_URL"):
    value = os.environ.get(key, "").strip()
    if value:
      return value
  for key in ("CarrotDiscordWebhookUrl", "CarrotDiscordWebhookURL", "DiscordWebhookUrl", "DiscordWebhookURL"):
    value = _dashcam_param_text(params, key, "")
    if value:
      return value
  if os.environ.get("CARROT_DISCORD_WEBHOOK_DISABLE", "").strip().lower() in {"1", "true", "yes", "on"}:
    return ""
  return _dashcam_decode_obfuscated(DASHCAM_DEFAULT_DISCORD_WEBHOOK, DASHCAM_DEFAULT_DISCORD_KEY)


def _dashcam_decode_obfuscated(value: str, key: str) -> str:
  try:
    token = str(value or "").strip()
    key_bytes = str(key or "").encode("utf-8")
    if not token or not key_bytes:
      return ""
    raw = base64.urlsafe_b64decode(token + "=" * (-len(token) % 4))
    decoded = bytes(raw[i] ^ key_bytes[i % len(key_bytes)] for i in range(len(raw)))
    return decoded.decode("utf-8", errors="ignore").strip()
  except Exception:
    return ""


def _dashcam_segment_file_summary(segment_dir: str) -> list[dict[str, Any]]:
  out: list[dict[str, Any]] = []
  for name in ("qcamera.mp4", "qcamera.ts", "rlog.zst", "rlog.bz2", "rlog", "qlog.zst", "qlog.bz2", "qlog"):
    path = os.path.join(segment_dir, name)
    if os.path.isfile(path):
      try:
        size = os.path.getsize(path)
      except OSError:
        size = 0
      out.append({"name": name, "size": size, "sizeLabel": _dashcam_file_size_label(size)})
  return out


def _dashcam_upload_share_text(payload: dict[str, Any]) -> str:
  meta = payload.get("meta") or {}
  uploaded = [item for item in payload.get("results") or [] if item.get("ok")]
  failed = [item for item in payload.get("results") or [] if not item.get("ok")]
  lines = [
    "# Carrot Dashcam Upload",
    "## Upload",
    f"- Time: {payload.get('uploadedAt') or ''}",
    f"- Path: {payload.get('remoteBasePath') or ''}",
    "## Device",
    f"- Car name: {meta.get('carName') or 'none'}",
    f"- DongleId: {meta.get('dongleId') or 'unknown'}",
    f"- Serial: {meta.get('serial') or 'unknown'}",
    f"- Branch: {meta.get('branch') or 'unknown'}",
    f"- Commit: {meta.get('commit') or 'unknown'} ({meta.get('commitDate') or 'unknown'})",
    "",
    "## Result",
  ]
  for item in uploaded:
    lines.append(f"- {item.get('segment')} OK")
  if failed:
    lines.extend(["## Failed", f"- {len(failed)}"])
  return "\n".join(lines).strip()


def _dashcam_discord_content(payload: dict[str, Any]) -> str:
  meta = payload.get("meta") or {}
  uploaded = [item for item in payload.get("results") or [] if item.get("ok")]
  failed = [item for item in payload.get("results") or [] if not item.get("ok")]
  detail_lines = [f"- {item.get('segment')} OK" for item in uploaded[:24]]
  if len(uploaded) > len(detail_lines):
    detail_lines.append(f"- ... +{len(uploaded) - len(detail_lines)} more")
  if not detail_lines:
    detail_lines.append("- none")
  failed_line = f"\n- Failed: **{len(failed)}**" if failed else ""
  content = (
    "# Carrot Dashcam Upload\n"
    "## Upload\n"
    f"- Time: **{payload.get('uploadedAt') or ''}**\n"
    f"- Path: **{payload.get('remoteBasePath') or ''}**\n"
    "## Device\n"
    f"- Car name: **{meta.get('carName') or 'none'}**\n"
    f"- DongleId: **{meta.get('dongleId') or 'unknown'}**\n"
    f"- Serial: **{meta.get('serial') or 'unknown'}**\n"
    f"- Branch: **{meta.get('branch') or 'unknown'}**\n"
    f"- Commit: **{meta.get('commit') or 'unknown'}** ({meta.get('commitDate') or 'unknown'})"
    f"{failed_line}\n"
    "## Result\n"
    + "\n".join(detail_lines)
  )
  if len(content) <= 1900:
    return content
  trimmed = detail_lines[:10]
  if len(uploaded) > len(trimmed):
    trimmed.append(f"- ... +{len(uploaded) - len(trimmed)} more")
  return (
    "# Carrot Dashcam Upload\n"
    "## Upload\n"
    f"- Time: **{payload.get('uploadedAt') or ''}**\n"
    f"- Path: **{payload.get('remoteBasePath') or ''}**\n"
    "## Device\n"
    f"- Car name: **{meta.get('carName') or 'none'}**\n"
    f"- DongleId: **{meta.get('dongleId') or 'unknown'}**\n"
    f"- Serial: **{meta.get('serial') or 'unknown'}**\n"
    f"- Branch: **{meta.get('branch') or 'unknown'}**\n"
    f"- Commit: **{meta.get('commit') or 'unknown'}** ({meta.get('commitDate') or 'unknown'})"
    f"{failed_line}\n"
    "## Result\n"
    + "\n".join(trimmed)
  )


async def _dashcam_send_discord_webhook(url: str, payload: dict[str, Any]) -> dict[str, Any]:
  url = (url or "").strip()
  if not url:
    return {"configured": False, "ok": False, "skipped": True}
  if not url.startswith(("http://", "https://")):
    return {"configured": True, "ok": False, "error": "invalid webhook url"}
  body = {
    "username": "Carrot Dashcam",
    "content": _dashcam_discord_content(payload),
    "allowed_mentions": {"parse": []},
  }
  try:
    timeout = ClientTimeout(total=12)
    async with ClientSession(timeout=timeout) as session:
      async with session.post(url, json=body) as resp:
        text = await resp.text()
        if 200 <= resp.status < 300:
          return {"configured": True, "ok": True, "status": resp.status}
        return {"configured": True, "ok": False, "status": resp.status, "error": text[:500]}
  except Exception as e:
    return {"configured": True, "ok": False, "error": str(e)}


def _dashcam_upload_folder_to_ftp(local_folder: str, directory: str, remote_path: str) -> bool:
  ftp_server = os.environ.get("CARROT_FTP_SERVER", "shind0.synology.me")
  ftp_port = int(os.environ.get("CARROT_FTP_PORT", "8021"))
  ftp_username = os.environ.get("CARROT_FTP_USERNAME", "carrotpilot")
  ftp_password = os.environ.get("CARROT_FTP_PASSWORD", "Ekdrmsvkdlffjt7710")

  ftp = FTP()
  ftp.connect(ftp_server, ftp_port, timeout=20)
  ftp.login(ftp_username, ftp_password)
  try:
    ftp.cwd("routes")
    routes_root = ftp.pwd()

    def cwd_or_create(path: str) -> None:
      ftp.cwd(routes_root)
      for part in [p for p in path.split("/") if p]:
        try:
          ftp.cwd(part)
        except Exception:
          ftp.mkd(part)
          ftp.cwd(part)

    base_path = f"{directory}/{remote_path}".strip("/")
    for root, _, files in os.walk(local_folder):
      rel_dir = os.path.relpath(root, local_folder)
      remote_dir = base_path if rel_dir == "." else f"{base_path}/{rel_dir.replace(os.sep, '/')}"
      cwd_or_create(remote_dir)
      for filename in files:
        local_path = os.path.join(root, filename)
        with open(local_path, "rb") as f:
          ftp.storbinary(f"STOR {filename}", f)
    return True
  finally:
    try:
      ftp.quit()
    except Exception:
      pass


async def api_dashcam_routes(request: web.Request) -> web.Response:
  try:
    routes = await asyncio.to_thread(_dashcam_build_routes)
    return web.json_response({"ok": True, "routes": routes, "root": DASHCAM_ROOT})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_dashcam_thumbnail(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  path = await asyncio.to_thread(_dashcam_ensure_thumbnail, segment)
  return web.FileResponse(path, headers={"Cache-Control": "public, max-age=86400"})


async def api_dashcam_preview(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  path = await asyncio.to_thread(_dashcam_ensure_preview, segment)
  return web.FileResponse(path, headers={"Cache-Control": "public, max-age=86400"})


async def api_dashcam_video(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  path, content_type = await asyncio.to_thread(_dashcam_browser_video, segment)
  return web.FileResponse(
    path,
    headers={
      "Content-Type": content_type,
      "Cache-Control": "private, max-age=3600",
    },
  )


async def api_dashcam_download(request: web.Request) -> web.StreamResponse:
  segment = request.match_info.get("segment", "")
  kind = (request.match_info.get("kind", "") or "").strip()
  segment_dir = _dashcam_segment_dir(segment)
  allowed = {
    "qcamera": ("qcamera.ts", "qcamera.mp4"),
    "rlog": ("rlog.zst", "rlog.bz2", "rlog"),
    "qlog": ("qlog.zst", "qlog.bz2", "qlog"),
  }
  for name in allowed.get(kind, ()):
    path = os.path.join(segment_dir, name)
    if os.path.isfile(path):
      mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
      return web.FileResponse(
        path,
        headers={
          "Content-Type": mime,
          "Content-Disposition": f'attachment; filename="{segment}--{name}"',
        },
      )
  raise web.HTTPNotFound(text="artifact not found")


async def api_dashcam_upload(request: web.Request) -> web.Response:
  try:
    try:
      body = await request.json()
    except Exception:
      body = {}
    segments = body.get("segments")
    if not isinstance(segments, list):
      one = body.get("segment")
      segments = [one] if one else []
    segments = [_dashcam_safe_segment(str(segment)) for segment in segments if segment]
    if not segments:
      return web.json_response({"ok": False, "error": "missing segments"}, status=400)

    params = Params() if HAS_PARAMS else None
    meta = _dashcam_upload_metadata(params)
    car_selected = meta.get("carName") or "none"
    dongle_id = meta.get("dongleId") or "unknown"
    directory = f"{car_selected} {dongle_id}".strip()
    remote_base_path = f"routes/{directory}/".replace("\\", "/")

    results = []
    for segment in segments:
      try:
        segment_dir = _dashcam_segment_dir(segment)
        ok = await asyncio.to_thread(
          _dashcam_upload_folder_to_ftp,
          segment_dir,
          directory,
          segment,
        )
        results.append({
          "segment": segment,
          "route": _dashcam_route_name(segment),
          "segmentIndex": _dashcam_segment_index(segment),
          "ok": bool(ok),
          "remotePath": f"{remote_base_path}{segment}",
          "files": _dashcam_segment_file_summary(segment_dir),
        })
      except Exception as e:
        results.append({
          "segment": segment,
          "route": _dashcam_route_name(segment),
          "segmentIndex": _dashcam_segment_index(segment),
          "ok": False,
          "remotePath": f"{remote_base_path}{segment}",
          "error": str(e),
        })

    ok_count = sum(1 for item in results if item["ok"])
    uploaded_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    response_payload = {
      "ok": ok_count == len(results),
      "uploaded": ok_count,
      "total": len(results),
      "uploadedAt": uploaded_at,
      "remoteBasePath": remote_base_path,
      "meta": meta,
      "results": results,
      "message": f"{ok_count}/{len(results)} uploaded",
    }
    response_payload["shareText"] = _dashcam_upload_share_text(response_payload)
    response_payload["discord"] = await _dashcam_send_discord_webhook(
      _dashcam_discord_webhook_url(params),
      response_payload,
    )
    return web.json_response(response_payload)
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


def _screenrecord_file_id(path: str) -> str:
  return hashlib.sha1(os.path.abspath(path).encode("utf-8", errors="ignore")).hexdigest()[:24]


def _screenrecord_date_label(epoch_seconds: int) -> str:
  try:
    return datetime.fromtimestamp(epoch_seconds).strftime("%Y-%m-%d %H:%M")
  except Exception:
    return "-"


def _screenrecord_build_videos() -> list[dict[str, Any]]:
  videos: list[dict[str, Any]] = []
  seen: set[str] = set()
  for folder in SCREEN_RECORDING_DIRS:
    if not os.path.isdir(folder):
      continue
    try:
      with os.scandir(folder) as it:
        for entry in it:
          try:
            name = entry.name
            if not entry.is_file(follow_symlinks=False):
              continue
            if not name.lower().endswith(SCREEN_RECORDING_EXTS):
              continue
            stat = entry.stat(follow_symlinks=False)
            if stat.st_size <= 0:
              continue
            path = os.path.abspath(entry.path)
            real = os.path.realpath(path)
            if real in seen:
              continue
            seen.add(real)
            modified = int(stat.st_mtime)
            videos.append({
              "id": _screenrecord_file_id(path),
              "name": name,
              "folder": folder,
              "size": int(stat.st_size),
              "modifiedEpoch": modified,
              "modifiedLabel": _screenrecord_date_label(modified),
              "relativeModifiedLabel": _dashcam_relative_time(modified),
              "ext": os.path.splitext(name)[1].lower().lstrip("."),
            })
          except Exception:
            continue
    except Exception:
      continue
  videos.sort(key=lambda item: (item.get("modifiedEpoch", 0), item.get("name", "")), reverse=True)
  return videos


def _screenrecord_find_file(file_id: str) -> str:
  file_id = (file_id or "").strip()
  if not file_id or "/" in file_id or "\\" in file_id or len(file_id) > 64:
    raise web.HTTPBadRequest(text="bad file id")
  for item in _screenrecord_build_videos():
    folder = str(item.get("folder") or "")
    name = str(item.get("name") or "")
    path = os.path.abspath(os.path.join(folder, name))
    if _screenrecord_file_id(path) == file_id and os.path.isfile(path):
      return path
  raise web.HTTPNotFound(text="screen recording not found")


def _screenrecord_thumbnail_path(file_id: str) -> str:
  path = _screenrecord_find_file(file_id)
  out = _dashcam_cache_path("screen_thumb", file_id, ".jpg")
  if os.path.isfile(out) and os.path.getsize(out) > 0:
    return out
  result = _dashcam_run_ffmpeg(["-ss", "1", "-i", path, "-vframes", "1", "-vf", "scale=320:-1", out])
  if result.returncode != 0 or not os.path.isfile(out) or os.path.getsize(out) <= 0:
    raise web.HTTPInternalServerError(text=result.stderr or result.stdout or "screenrecord thumbnail generation failed")
  return out


async def api_screenrecord_videos(request: web.Request) -> web.Response:
  try:
    videos = await asyncio.to_thread(_screenrecord_build_videos)
    folders = [folder for folder in SCREEN_RECORDING_DIRS if os.path.isdir(folder)]
    return web.json_response({"ok": True, "videos": videos, "folders": folders})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


async def api_screenrecord_thumbnail(request: web.Request) -> web.StreamResponse:
  file_id = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(_screenrecord_thumbnail_path, file_id)
  return web.FileResponse(path, headers={"Cache-Control": "public, max-age=86400"})


async def api_screenrecord_video(request: web.Request) -> web.StreamResponse:
  file_id = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(_screenrecord_find_file, file_id)
  mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
  return web.FileResponse(
    path,
    headers={
      "Content-Type": mime,
      "Cache-Control": "private, max-age=3600",
    },
  )


async def api_screenrecord_download(request: web.Request) -> web.StreamResponse:
  file_id = request.match_info.get("file_id", "")
  path = await asyncio.to_thread(_screenrecord_find_file, file_id)
  filename = os.path.basename(path)
  safe_filename = "".join(ch if 32 <= ord(ch) < 127 and ch not in {'"', "\\"} else "_" for ch in filename)
  mime = mimetypes.guess_type(path)[0] or "application/octet-stream"
  return web.FileResponse(
    path,
    headers={
      "Content-Type": mime,
      "Content-Disposition": f'attachment; filename="{safe_filename or "screenrecord"}"',
    },
  )


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

async def on_startup(app: web.Application):
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
  asyncio.create_task(_malloc_trim_loop())

async def on_cleanup(app: web.Application):
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
    except Exception:
      pass

  sess = app.get("http")
  if sess:
    await sess.close()

# -----------------------
# Settings cache (mtime based)
# -----------------------
_settings_cache = {
  "path": DEFAULT_SETTINGS_PATH,
  "mtime": 0,
  "data": None,      # full json
  "groups": None,    # {group: [param,...]}
  "by_name": None,   # {name: param}
  "groups_list": None,  # [{group, egroup, count}, ...]
}

def _read_settings_file(path: str) -> Dict[str, Any]:
  with open(path, "r", encoding="utf-8") as f:
    return json.load(f)

def _group_index(settings: Dict[str, Any]) -> Tuple[Dict[str, list], Dict[str, Dict[str, Any]], List[Dict[str, Any]]]:
  groups: Dict[str, list] = {}
  by_name: Dict[str, Dict[str, Any]] = {}
  groups_list: List[Dict[str, Any]] = []

  params = settings.get("params", [])
  for p in params:
    g = p.get("group", "기타")
    if g == "기타":
        if "egroup" not in p: p["egroup"] = "Other"
        if "cgroup" not in p: p["cgroup"] = "其他"

    groups.setdefault(g, []).append(p)
    n = p.get("name")
    if n:
      by_name[n] = p

  # group list with egroup/cgroup guess
  for g, items in groups.items():
    egroup = None
    cgroup = None
    for it in items:
      if not egroup and it.get("egroup"):
        egroup = it.get("egroup")
      if not cgroup and it.get("cgroup"):
        cgroup = it.get("cgroup")
      if egroup and cgroup:
        break
    groups_list.append({"group": g, "egroup": egroup, "cgroup": cgroup, "count": len(items)})

  return groups, by_name, groups_list

def _get_settings_cached() -> Tuple[Dict[str, Any], Dict[str, list], Dict[str, Dict[str, Any]], List[Dict[str, Any]]]:
  path = _settings_cache["path"]
  st = os.stat(path)
  mtime = int(st.st_mtime)
  if _settings_cache["data"] is None or _settings_cache["mtime"] != mtime:
    data = _read_settings_file(path)
    groups, by_name, groups_list = _group_index(data)
    _settings_cache.update({
      "mtime": mtime,
      "data": data,
      "groups": groups,
      "by_name": by_name,
      "groups_list": groups_list,
    })
  return _settings_cache["data"], _settings_cache["groups"], _settings_cache["by_name"], _settings_cache["groups_list"]


# -----------------------
# Param helpers
# -----------------------
_mem_store: Dict[str, str] = {}  # if Params not available

def _infer_type_from_setting(p: Optional[Dict[str, Any]]) -> str:
  """
  Fallback when get_type/ParamKeyType unavailable.
  returns one of: "bool","int","float","string","json","time"
  """
  if not p:
    return "string"
  mn, mx, d = p.get("min"), p.get("max"), p.get("default")

  # bool heuristic: min=0 max=1 and default is 0/1
  if mn in (0, 0.0) and mx in (1, 1.0) and d in (0, 1, 0.0, 1.0):
    return "bool"

  # int vs float
  if isinstance(mn, int) and isinstance(mx, int) and isinstance(d, int):
    return "int"

  if isinstance(mn, (int, float)) and isinstance(mx, (int, float)) and isinstance(d, (int, float)):
    # if any float exists
    if any(isinstance(x, float) for x in (mn, mx, d)):
      return "float"
    return "int"

  return "string"

def _clamp_numeric(value: float, p: Optional[Dict[str, Any]]) -> float:
  if not p:
    return value
  mn = p.get("min")
  mx = p.get("max")
  try:
    if mn is not None:
      value = max(value, float(mn))
    if mx is not None:
      value = min(value, float(mx))
  except Exception:
    pass
  return value

def _read_git_state() -> Dict[str, Any]:
  try:
    with open(CARROT_GIT_STATE_PATH, "r", encoding="utf-8") as f:
      data = json.load(f)
    return data if isinstance(data, dict) else {}
  except Exception:
    return {}

def _write_git_state(data: Dict[str, Any]) -> None:
  try:
    os.makedirs(CARROT_STATE_DIR, exist_ok=True)
    tmp_path = f"{CARROT_GIT_STATE_PATH}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
      json.dump(data, f, ensure_ascii=True, separators=(",", ":"))
    os.replace(tmp_path, CARROT_GIT_STATE_PATH)
  except Exception:
    pass

def _read_custom_meta_value(name: str) -> Optional[str]:
  if name != "GitPullTime":
    return None

  try:
    value = _read_git_state().get("git_pull_time")
    if value is None:
      return None
    return str(value).strip()
  except Exception:
    return None

def _write_git_pull_time(ts: Optional[int] = None) -> None:
  value = int(ts if ts is not None else time.time())
  data = _read_git_state()
  data["git_pull_time"] = value
  data["git_pull_ok"] = True
  _write_git_state(data)

def _did_git_pull_update(output: str) -> bool:
  body = str(output or "").strip().lower()
  if not body:
    return False
  if "already up to date" in body or "already up-to-date" in body:
    return False
  return (
    "fast-forward" in body or
    "merge made by" in body or
    "updating " in body or
    bool(re.search(r"[0-9]+\s+files?\s+changed", body))
  )

def _get_param_value(name: str, default: Any) -> Any:
  custom_value = _read_custom_meta_value(name)
  if custom_value is not None:
    return custom_value

  if not HAS_PARAMS:
    # mem store (string) fallback
    s = _mem_store.get(name, None)
    return default if s is None else s

  params = Params()
  try:
    t = params.get_type(name)

    if t == ParamKeyType.BOOL:
      return bool(params.get_bool(name))

    if t == ParamKeyType.INT:
      return int(params.get_int(name))

    if t == ParamKeyType.FLOAT:
      return float(params.get_float(name))

    # STRING / TIME / 기타는 raw string
    v = params.get(name)
    if v is None:
      return default if default is not None else ""
    if isinstance(v, (bytes, bytearray, memoryview)):
      return v.decode("utf-8", errors="replace")
    return str(v)

  except Exception:
    pass

  # fallback: raw get + minimal decode
  try:
    v = params.get(name)
    if v is None:
      return default if default is not None else ""
    return v.decode("utf-8", errors="replace")
  except Exception:
    return default if default is not None else ""

def _put_typed(params: "Params", key: str, value: Any) -> None:
  try:
      t = params.get_type(key)

      # BOOL
      if t == ParamKeyType.BOOL:
        v = value in ("1", "true", "True", "on", "yes") if isinstance(value, str) else bool(value)
        params.put_bool(key, v)
        return

      # INT
      if t == ParamKeyType.INT:
        params.put_int(key, int(float(value)))
        return

      # FLOAT
      if t == ParamKeyType.FLOAT:
        params.put_float(key, float(value))
        return

      # TIME (string ISO)
      if t == ParamKeyType.TIME:
        params.put(key, str(value))
        return

      # STRING
      if t == ParamKeyType.STRING:
        params.put(key, str(value))
        return

      # JSON
      if t == ParamKeyType.JSON:
        obj = json.loads(value) if isinstance(value, str) else value
        params.put(key, obj)

      # BYTES 등은 일단 스킵
      raise RuntimeError(f"Unsupported ParamKeyType for {key}: {t}")

  except Exception:
    # fall through to inference
    pass


def _set_param_value(name: str, value: Any) -> None:
  if not HAS_PARAMS:
    _mem_store[name] = str(value)
    return
  params = Params()
  _put_typed(params, name, value)


# -----------------------
# Web handlers
# -----------------------
async def handle_index(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(WEB_DIR, "index.html"))

# Legacy direct-file routes kept for backward compatibility.
async def handle_appjs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(JS_DIR, "app_core.js"))

async def handle_appcss(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(CSS_DIR, "app.css"))

async def handle_appcorejs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(JS_DIR, "app_core.js"))

async def handle_apppagesjs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(JS_DIR, "app_pages.js"))

async def handle_apprealtimejs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(JS_DIR, "app_realtime.js"))

async def api_settings(request: web.Request) -> web.Response:
  path = _settings_cache["path"]
  if not os.path.exists(path):
    return web.json_response({"ok": False, "error": f"settings file not found: {path}"}, status=404)

  try:
    data, groups, by_name, groups_list = _get_settings_cached()
    # keep insertion order of groups
    items_by_group = {g: items for g, items in groups.items()}
    return web.json_response({
      "ok": True,
      "path": path,
      "apilot": data.get("apilot"),
      "groups": groups_list,
      "items_by_group": items_by_group,
      "unit_cycle": UNIT_CYCLE,
      "has_params": HAS_PARAMS,
      "has_param_type": bool(ParamKeyType is not None and hasattr(Params(), "get_type")) if HAS_PARAMS else False,
    })
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)

async def api_params_bulk(request: web.Request) -> web.Response:
  names = request.query.get("names", "")
  if not names:
    return web.json_response({"ok": False, "error": "missing names"}, status=400)

  req_names = [n for n in names.split(",") if n]
  try:
    _, _, by_name, _ = _get_settings_cached()
  except Exception:
    by_name = {}

  values = {}
  for n in req_names:
    if n == "DeviceType":
      try:
        from openpilot.system.hardware import HARDWARE
        values[n] = HARDWARE.get_device_type()
      except Exception:
        values[n] = "unknown"
    else:
      default = by_name.get(n, {}).get("default", 0)
      values[n] = _get_param_value(n, default)

  return web.json_response({"ok": True, "values": values})

async def api_param_set(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  name = body.get("name")
  value = body.get("value")

  if not name:
    return web.json_response({"ok": False, "error": "missing name"}, status=400)

  # clamp using settings if numeric
  p = None
  try:
    _, _, by_name, _ = _get_settings_cached()
    p = by_name.get(name)
  except Exception:
    pass

  # If value numeric -> clamp
  try:
    if p is not None and isinstance(p.get("min"), (int, float)) and isinstance(p.get("max"), (int, float)):
      fv = float(value)
      fv = _clamp_numeric(fv, p)
      # keep int if setting looks int-ish
      if isinstance(p.get("min"), int) and isinstance(p.get("max"), int) and isinstance(p.get("default"), int):
        value = int(round(fv))
      else:
        value = fv
  except Exception:
    # ignore clamp errors (string values etc.)
    pass

  try:
    _set_param_value(name, value)
    return web.json_response({"ok": True, "name": name, "value": value, "has_params": HAS_PARAMS})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)

SUPPORTED_CAR_GLOB = "/data/params/d/SupportedCars*"

def _load_supported_cars() -> Tuple[List[str], Dict[str, List[str]]]:
  files = sorted(glob.glob(SUPPORTED_CAR_GLOB))
  makers: Dict[str, set] = {}

  for fp in files:
    try:
      with open(fp, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
          line = line.strip()
          if not line:
            continue
          parts = line.split(" ", 1)
          if len(parts) < 2:
            continue
          maker, rest = parts[0], parts[1].strip()
          full = f"{maker} {rest}"
          makers.setdefault(maker, set()).add(full)
    except Exception:
      continue

  makers_sorted: Dict[str, List[str]] = {}
  for mk, s in makers.items():
    makers_sorted[mk] = sorted(s)

  return [os.path.basename(x) for x in files], makers_sorted


async def api_cars(request: web.Request) -> web.Response:
  try:
    sources, makers = _load_supported_cars()
    return web.json_response({
      "ok": True,
      "sources": sources,
      "makers": makers,
    })
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)

async def api_reboot(request: web.Request) -> web.Response:
  try:
    # 보안 최소조치(권장): 로컬/사설 대역만 허용 등
    # ip = request.remote
    # if not (ip.startswith("192.168.") or ip.startswith("10.") or ip in ("127.0.0.1", "::1")):
    #   return web.json_response({"ok": False, "error": "forbidden"}, status=403)

    # 즉시 반환하고 리붓은 백그라운드로
    subprocess.Popen(["sudo", "reboot"])
    return web.json_response({"ok": True})
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)


TOOL_JOB_MAX_LOG_CHARS = 180000
TOOL_JOB_KEEP_COUNT = 24
_tool_jobs: Dict[str, Dict[str, Any]] = {}


def _tool_job_touch(job: Dict[str, Any]) -> None:
  job["updated_at"] = time.time()


def _tool_job_trim_log(job: Dict[str, Any]) -> None:
  text = job.get("log") or ""
  if len(text) <= TOOL_JOB_MAX_LOG_CHARS:
    return
  job["log"] = text[-TOOL_JOB_MAX_LOG_CHARS:]


def _tool_job_append(job: Dict[str, Any], text: Any) -> None:
  if text is None:
    return
  chunk = str(text).replace("\r\n", "\n").replace("\r", "\n")
  if not chunk:
    return
  cur = job.get("log") or ""
  if cur and not cur.endswith("\n") and not chunk.startswith("\n"):
    cur += "\n"
  job["log"] = cur + chunk
  _tool_job_trim_log(job)
  _tool_job_touch(job)


def _tool_job_progress(job: Dict[str, Any], *, message: Optional[str] = None,
                       current: Optional[int] = None, total: Optional[int] = None,
                       percent: Optional[int] = None) -> None:
  if message is not None:
    job["message"] = str(message)
  if current is not None:
    job["step_current"] = max(0, int(current))
  if total is not None:
    job["step_total"] = max(0, int(total))
  if percent is None:
    c = job.get("step_current")
    t = job.get("step_total")
    if isinstance(c, int) and isinstance(t, int) and t > 0:
      percent = int(max(0, min(100, round((c / t) * 100))))
  job["progress"] = percent
  _tool_job_touch(job)


def _tool_job_snapshot(job: Dict[str, Any]) -> Dict[str, Any]:
  result = job.get("result")
  return {
    "ok": True,
    "id": job["id"],
    "action": job["action"],
    "status": job["status"],
    "done": job["status"] in ("done", "failed"),
    "log": job.get("log") or "",
    "progress": job.get("progress"),
    "message": job.get("message") or "",
    "step_current": job.get("step_current"),
    "step_total": job.get("step_total"),
    "error": job.get("error"),
    "error_code": job.get("error_code"),
    "error_detail": job.get("error_detail"),
    "created_at": job.get("created_at"),
    "updated_at": job.get("updated_at"),
    "result": result,
  }


def _tool_job_finish(job: Dict[str, Any], *, ok: bool, result: Optional[Dict[str, Any]] = None,
                     error: Optional[str] = None, error_code: Optional[str] = None,
                     error_detail: Optional[str] = None) -> None:
  job["status"] = "done" if ok else "failed"
  job["result"] = result or {"ok": bool(ok)}
  if error is None and result:
    error = result.get("error") or (None if result.get("ok", ok) else result.get("out"))
  job["error"] = error
  job["error_code"] = error_code or (result.get("error_code") if result else None)
  job["error_detail"] = error_detail or (result.get("error_detail") if result else None)
  if ok:
    job["progress"] = 100
  _tool_job_touch(job)
  _tool_job_prune()


def _tool_job_prune() -> None:
  finished = [
    item for item in _tool_jobs.values()
    if item.get("status") in ("done", "failed")
  ]
  if len(finished) <= TOOL_JOB_KEEP_COUNT:
    return
  finished.sort(key=lambda item: float(item.get("updated_at") or 0), reverse=True)
  for old in finished[TOOL_JOB_KEEP_COUNT:]:
    _tool_jobs.pop(old["id"], None)


async def _tool_stream_exec(job: Dict[str, Any], cmd: List[str], *, cwd: Optional[str] = None,
                            timeout: Optional[float] = None) -> int:
  proc = await asyncio.create_subprocess_exec(
    *cmd,
    cwd=cwd,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.STDOUT,
  )

  async def _consume() -> int:
    assert proc.stdout is not None
    while True:
      chunk = await proc.stdout.read(1024)
      if not chunk:
        break
      _tool_job_append(job, chunk.decode("utf-8", errors="replace"))
    return await proc.wait()

  try:
    if timeout is not None:
      return await asyncio.wait_for(_consume(), timeout=timeout)
    return await _consume()
  except asyncio.TimeoutError:
    try:
      proc.kill()
    except Exception:
      pass
    try:
      await proc.wait()
    except Exception:
      pass
    _tool_job_append(job, "\n[timeout]\n")
    raise


async def _tool_capture_exec(cmd: List[str], *, cwd: Optional[str] = None,
                             timeout: Optional[float] = None) -> Tuple[int, str]:
  proc = await asyncio.create_subprocess_exec(
    *cmd,
    cwd=cwd,
    stdout=asyncio.subprocess.PIPE,
    stderr=asyncio.subprocess.STDOUT,
  )
  try:
    stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=timeout) if timeout is not None else await proc.communicate()
  except asyncio.TimeoutError:
    try:
      proc.kill()
    except Exception:
      pass
    try:
      await proc.wait()
    except Exception:
      pass
    raise
  return proc.returncode, (stdout or b"").decode("utf-8", errors="replace").strip()


def _tool_result_from_log(job: Dict[str, Any], rc: int, **extra: Any) -> Dict[str, Any]:
  out = (job.get("log") or "").strip() or "(no output)"
  return {"ok": rc == 0, "rc": rc, "out": out, **extra}


def _get_branch_prefix() -> str:
  try:
    return "c4" if HARDWARE.get_device_type() == "mici" else "c3"
  except Exception:
    return "c3"

def _parse_remote_urls(remote_urls_out: str) -> dict[str, str]:
  remote_urls: dict[str, str] = {}
  for remote_line in (remote_urls_out or "").splitlines():
    parts = remote_line.split()
    if len(parts) >= 2 and parts[0] not in remote_urls:
      remote_urls[parts[0]] = parts[1]
  return remote_urls

def _match_remote_ref(ref: str, remotes: list[str]) -> Optional[tuple[str, str]]:
  for remote in sorted(remotes, key=len, reverse=True):
    prefix = f"{remote}/"
    if not ref.startswith(prefix):
      continue
    name = ref[len(prefix):].strip()
    if name and name != "HEAD":
      return remote, name
  return None

def _build_branch_items(local_refs_out: str, remote_refs_out: str, remotes: list[str]) -> list[dict[str, Any]]:
  items: list[dict[str, Any]] = []
  seen: set[tuple[str, str, str]] = set()

  for line in (local_refs_out or "").splitlines():
    name = line.strip()
    if not name:
      continue
    key = ("local", "", name)
    if key in seen:
      continue
    seen.add(key)
    items.append({
      "kind": "local",
      "ref": name,
      "name": name,
      "label": name,
    })

  for line in (remote_refs_out or "").splitlines():
    ref = line.strip()
    if not ref:
      continue
    match = _match_remote_ref(ref, remotes)
    if match is None:
      continue
    remote, name = match
    key = ("remote", remote, name)
    if key in seen:
      continue
    seen.add(key)
    items.append({
      "kind": "remote",
      "ref": f"{remote}/{name}",
      "remote": remote,
      "name": name,
      "label": name,
    })

  return sorted(items, key=lambda item: (
    0 if item.get("kind") == "local" else 1,
    str(item.get("remote") or "").lower(),
    str(item.get("name") or item.get("ref") or "").lower(),
  ))

async def _run_tool_job(job: Dict[str, Any]) -> None:
  action = job["action"]
  body = job.get("payload") or {}
  repo_dir = "/data/openpilot"

  try:
    if action == "git_pull":
      _tool_job_progress(job, message="git reset --hard", current=1, total=2)
      _tool_job_append(job, "$ git reset --hard\n")
      rc_reset = await _tool_stream_exec(job, ["git", "reset", "--hard"], cwd=repo_dir, timeout=120)
      if rc_reset != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_reset))
        return

      _tool_job_append(job, "\n$ git pull\n")
      _tool_job_progress(job, message="git pull", current=2, total=2)
      rc = await _tool_stream_exec(job, ["git", "pull"], cwd=repo_dir, timeout=180)
      if rc == 0 and _did_git_pull_update(job.get("log") or ""):
        _write_git_pull_time()
      result = _tool_result_from_log(job, rc)
      _tool_job_finish(job, ok=rc == 0, result=result)
      return

    if action == "git_sync":
      _tool_job_progress(job, message="delete local branches", current=1, total=2)
      rc1 = await _tool_stream_exec(
        job,
        ["bash", "-lc", "git branch | grep -v '^\\*' | xargs -r git branch -D"],
        cwd=repo_dir,
        timeout=120,
      )
      if rc1 != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc1))
        return

      _tool_job_progress(job, message="fetch --all --prune", current=2, total=2)
      rc2 = await _tool_stream_exec(job, ["git", "fetch", "--all", "--prune"], cwd=repo_dir, timeout=180)
      _tool_job_finish(job, ok=rc2 == 0, result=_tool_result_from_log(job, rc2))
      return

    if action == "git_reset":
      mode = (body.get("mode") or "hard").strip()
      target = (body.get("target") or "HEAD").strip()
      if mode not in ("hard", "soft", "mixed"):
        _tool_job_finish(
          job,
          ok=False,
          result={"ok": False, "error": "bad mode", "error_code": "INVALID_RESET_MODE"},
          error="bad mode",
          error_code="INVALID_RESET_MODE",
        )
        return

      _tool_job_progress(job, message=f"git reset --{mode} {target}", current=1, total=1)
      rc = await _tool_stream_exec(job, ["git", "reset", f"--{mode}", target], cwd=repo_dir, timeout=120)
      _tool_job_finish(job, ok=rc == 0, result=_tool_result_from_log(job, rc))
      return

    if action == "git_checkout":
      branch = (body.get("branch") or "").strip()
      kind = str(body.get("kind") or "").strip()
      item_name = str(body.get("name") or "").strip()
      item_remote = str(body.get("remote") or "").strip()
      if not branch and not item_name:
        _tool_job_finish(
          job,
          ok=False,
          result={"ok": False, "error": "missing branch", "error_code": "MISSING_BRANCH"},
          error="missing branch",
          error_code="MISSING_BRANCH",
        )
        return

      _tool_job_progress(job, message="fetch --all --prune", current=1, total=2)
      rc_fetch = await _tool_stream_exec(job, ["git", "fetch", "--all", "--prune"], cwd=repo_dir, timeout=180)
      if rc_fetch != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_fetch))
        return

      _tool_job_progress(job, message=f"switch {branch}", current=2, total=2)

      rc_remotes, remotes_out = await _tool_capture_exec(["git", "remote"], cwd=repo_dir, timeout=30)
      known_remotes = remotes_out.split() if rc_remotes == 0 else ["origin"]

      if kind == "local":
        local_branch = item_name or branch
        script = f"git switch {shlex.quote(local_branch)}"
      elif kind == "remote":
        if not item_remote or not item_name:
          _tool_job_finish(job, ok=False, result={"ok": False, "error": "missing remote branch info"}, error="missing remote branch info")
          return
        if item_remote not in known_remotes:
          _tool_job_finish(job, ok=False, result={"ok": False, "error": f"unknown remote: {item_remote}"}, error=f"unknown remote: {item_remote}")
          return
        branch = f"{item_remote}/{item_name}"
        local_branch = item_name
        script = (
          f"if git show-ref --verify --quiet {shlex.quote(f'refs/heads/{local_branch}')}; "
          f"then git switch {shlex.quote(local_branch)}; "
          f"else git switch -c {shlex.quote(local_branch)} --track {shlex.quote(branch)}; fi"
        )
      else:
        # Backward compatibility for older clients that only send a branch string.
        remote_prefix = None
        for r in known_remotes:
          if branch.startswith(f"{r}/"):
            remote_prefix = r
            break

        if remote_prefix is not None:
          local_branch = branch[len(remote_prefix) + 1:]
          script = (
            f"if git show-ref --verify --quiet {shlex.quote(f'refs/heads/{local_branch}')}; "
            f"then git switch {shlex.quote(local_branch)}; "
            f"else git switch -c {shlex.quote(local_branch)} --track {shlex.quote(branch)}; fi"
          )
        else:
          script = (
            f"git switch {shlex.quote(branch)} || "
            f"git switch -c {shlex.quote(branch)} --track {shlex.quote(f'origin/{branch}')}"
          )
      rc = await _tool_stream_exec(job, ["bash", "-lc", script], cwd=repo_dir, timeout=180)
      _tool_job_finish(job, ok=rc == 0, result=_tool_result_from_log(job, rc))
      return

    if action == "git_remote_set":
      url = str(job.get("payload", {}).get("url") or "").strip()
      if not url:
        _tool_job_finish(job, ok=False, result={"ok": False, "error": "missing url"}, error="missing url")
        return
      
      _tool_job_progress(job, message=f"set-url origin {url}", current=1, total=2)
      rc_set = await _tool_stream_exec(job, ["git", "remote", "set-url", "origin", url], cwd=repo_dir, timeout=30)
      if rc_set != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_set))
        return

      _tool_job_progress(job, message="fetch origin", current=2, total=2)
      rc_fetch = await _tool_stream_exec(job, ["git", "fetch", "--progress", "origin"], cwd=repo_dir, timeout=180)
      _tool_job_finish(job, ok=rc_fetch == 0, result=_tool_result_from_log(job, rc_fetch))
      return

    if action == "git_branch_list":
      _tool_job_progress(job, message="fetch --all --prune", current=1, total=2)
      rc_fetch = await _tool_stream_exec(job, ["git", "fetch", "--all", "--prune"], cwd=repo_dir, timeout=180)
      if rc_fetch != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_fetch))
        return

      _tool_job_progress(job, message="git refs", current=2, total=2)
      rc_local, local_refs_out = await _tool_capture_exec(
        ["git", "for-each-ref", "--format=%(refname:short)", "refs/heads"],
        cwd=repo_dir,
        timeout=30,
      )
      rc_remote, remote_refs_out = await _tool_capture_exec(
        ["git", "for-each-ref", "--format=%(refname:short)", "refs/remotes"],
        cwd=repo_dir,
        timeout=30,
      )
      if local_refs_out or remote_refs_out:
        _tool_job_append(job, "\n$ git refs\n")
        if local_refs_out:
          _tool_job_append(job, "[local]\n" + local_refs_out + "\n")
        if remote_refs_out:
          _tool_job_append(job, "[remote]\n" + remote_refs_out + "\n")
      if rc_local != 0 or rc_remote != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_local if rc_local != 0 else rc_remote))
        return

      rc_current, current_branch = await _tool_capture_exec(
        ["git", "branch", "--show-current"],
        cwd=repo_dir,
        timeout=15,
      )
      if rc_current != 0:
        current_branch = ""
      current_branch = (current_branch or "").strip()

      rc_remotes, remotes_out = await _tool_capture_exec(["git", "remote"], cwd=repo_dir, timeout=15)
      remotes = remotes_out.split() if rc_remotes == 0 else ["origin"]
      rc_remote_urls, remote_urls_out = await _tool_capture_exec(["git", "remote", "-v"], cwd=repo_dir, timeout=15)
      remote_urls = _parse_remote_urls(remote_urls_out) if rc_remote_urls == 0 else {}
      branch_items = _build_branch_items(local_refs_out, remote_refs_out, remotes)
      branches = sorted({str(item.get("ref") or "") for item in branch_items if item.get("ref")})

      result = {
        "ok": True,
        "branches": branches,
        "branch_items": branch_items,
        "current_branch": current_branch,
        "fetch": (job.get("log") or "").strip(),
        "device_type": HARDWARE.get_device_type(),
        "branch_prefix": _get_branch_prefix(),
        "remotes": remotes,
        "remote_urls": remote_urls,
      }
      _tool_job_finish(job, ok=True, result=result)
      return
    if action == "git_remote_add":
      name = str(body.get("name") or "").strip()
      url = str(body.get("url") or "").strip()
      if not name or not url:
        _tool_job_finish(job, ok=False, result={"ok": False, "error": "missing name or url"}, error="missing name or url")
        return

      rc_remotes, remotes_out = await _tool_capture_exec(["git", "remote"], cwd=repo_dir, timeout=15)
      remotes = remotes_out.split() if rc_remotes == 0 else []
      remote_exists = name in remotes

      setup_cmd = ["git", "remote", "set-url", name, url] if remote_exists else ["git", "remote", "add", name, url]
      setup_label = "set-url" if remote_exists else "add"
      _tool_job_progress(job, message=f"git remote {setup_label} {name}", current=1, total=2)
      rc_setup = await _tool_stream_exec(job, setup_cmd, cwd=repo_dir, timeout=30)
      if rc_setup != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_setup))
        return

      _tool_job_progress(job, message=f"git fetch --prune {name}", current=2, total=2)
      rc_fetch = await _tool_stream_exec(job, ["git", "fetch", "--prune", "--progress", name], cwd=repo_dir, timeout=180)

      rc_remote_urls, remote_urls_out = await _tool_capture_exec(["git", "remote", "-v"], cwd=repo_dir, timeout=15)
      if rc_remote_urls == 0 and remote_urls_out:
        _tool_job_append(job, "\n$ git remote -v\n")
        _tool_job_append(job, remote_urls_out + "\n")
      _tool_job_finish(job, ok=rc_fetch == 0, result=_tool_result_from_log(job, rc_fetch))
      return

    if action == "git_log":
      count = min(int(body.get("count") or 20), 50)
      _tool_job_progress(job, message="git log", current=1, total=1)
      rc, out = await _tool_capture_exec(
        ["git", "log", f"--oneline", f"-{count}"],
        cwd=repo_dir,
        timeout=30,
      )
      rc_head, out_head = await _tool_capture_exec(
        ["git", "rev-parse", "--short", "HEAD"],
        cwd=repo_dir,
        timeout=10,
      )
      current_commit = out_head.strip() if rc_head == 0 else ""
      if out:
        _tool_job_append(job, out)
      commits = []
      for line in (out or "").splitlines():
        line = line.strip()
        if not line:
          continue
        parts = line.split(" ", 1)
        commits.append({"hash": parts[0], "message": parts[1] if len(parts) > 1 else ""})
      result = {"ok": rc == 0, "commits": commits, "current_commit": current_commit, "out": out}
      _tool_job_finish(job, ok=rc == 0, result=result)
      return

    if action == "git_reset_repo_fetch":
      url = "https://github.com/ajouatom/openpilot.git"
      # Phase 1: ensure origin points to the correct URL
      _tool_job_progress(job, message="configuring origin remote", current=1, total=4)

      # Try set-url first (works if origin exists)
      rc_set, _ = await _tool_capture_exec(
        ["git", "remote", "set-url", "origin", url], cwd=repo_dir, timeout=15
      )
      if rc_set != 0:
        # origin doesn't exist; remove any stale one then add fresh
        _tool_job_append(job, "origin not found, adding new remote\n")
        await _tool_capture_exec(["git", "remote", "remove", "origin"], cwd=repo_dir, timeout=10)
        rc_add, out_add = await _tool_capture_exec(
          ["git", "remote", "add", "origin", url], cwd=repo_dir, timeout=15
        )
        if rc_add != 0:
          _tool_job_append(job, f"failed to add origin: {out_add}\n")
          _tool_job_finish(job, ok=False, result={"ok": False, "error": f"failed to configure remote: {out_add}"})
          return
      _tool_job_append(job, f"origin → {url}\n")

      # Phase 2: remove ALL other remotes (so only origin remains)
      _tool_job_progress(job, message="cleaning other remotes", current=2, total=4)
      rc_remotes, out_remotes = await _tool_capture_exec(
        ["git", "remote"], cwd=repo_dir, timeout=10
      )
      for remote_name in (out_remotes or "").splitlines():
        remote_name = remote_name.strip()
        if remote_name and remote_name != "origin":
          _tool_job_append(job, f"removing remote: {remote_name}\n")
          await _tool_capture_exec(
            ["git", "remote", "remove", remote_name], cwd=repo_dir, timeout=10
          )

      # Phase 3: fetch from origin only
      _tool_job_progress(job, message="git fetch origin --prune", current=3, total=4)
      rc_fetch = await _tool_stream_exec(
        job, ["git", "fetch", "origin", "--prune"], cwd=repo_dir, timeout=300
      )
      if rc_fetch != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_fetch))
        return

      # Phase 4: list remote branches (origin/* only)
      _tool_job_progress(job, message="listing branches", current=4, total=4)
      rc_br, out_br = await _tool_capture_exec(
        ["git", "branch", "-r"], cwd=repo_dir, timeout=15
      )
      branches = []
      for line in (out_br or "").splitlines():
        line = line.strip()
        if not line or "->" in line:
          continue
        # Only include origin/* branches
        if not line.startswith("origin/"):
          continue
        # "origin/c3-wip" → "c3-wip"
        branches.append(line.split("/", 1)[1])
      branches = sorted(set(branches))
      _tool_job_append(job, f"found {len(branches)} branches\n")

      result = {"ok": True, "branches": branches, "out": (job.get("log") or "").strip()}
      _tool_job_finish(job, ok=True, result=result)
      return

    if action == "git_reset_repo_checkout":
      branch = str(body.get("branch") or "").strip()
      if not branch:
        _tool_job_finish(job, ok=False, result={"ok": False, "error": "missing branch"}, error="missing branch")
        return

      steps = [
        (f"git checkout -B {branch} origin/{branch}", ["git", "checkout", "-B", branch, f"origin/{branch}"]),
        (f"git reset --hard origin/{branch}", ["git", "reset", "--hard", f"origin/{branch}"]),
        ("git clean -xfd", ["git", "clean", "-xfd"]),
      ]
      for i, (msg, cmd) in enumerate(steps):
        _tool_job_progress(job, message=msg, current=i+1, total=len(steps))
        rc = await _tool_stream_exec(job, cmd, cwd=repo_dir, timeout=120)
        if rc != 0:
          _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc))
          return

      _tool_job_finish(job, ok=True, result=_tool_result_from_log(job, 0))
      return

    if action == "delete_all_videos":
      _tool_job_progress(job, message="delete videos", current=1, total=1)
      deleted = 0
      for path in ["/data/media/0/videos"]:
        if not os.path.isdir(path):
          continue
        for fn in glob.glob(os.path.join(path, "*")):
          try:
            os.remove(fn)
            deleted += 1
            _tool_job_append(job, f"deleted: {os.path.basename(fn)}")
          except Exception as e:
            _tool_job_append(job, f"delete error: {e}")
      result = {"ok": True, "out": f"deleted files: {deleted}"}
      _tool_job_finish(job, ok=True, result=result)
      return

    if action == "delete_all_logs":
      _tool_job_progress(job, message="delete logs", current=1, total=1)
      deleted = 0
      for path in ["/data/media/0/realdata"]:
        if not os.path.isdir(path):
          continue
        for name in os.listdir(path):
          full_path = os.path.join(path, name)
          try:
            if os.path.isfile(full_path) or os.path.islink(full_path):
              os.remove(full_path)
            elif os.path.isdir(full_path):
              shutil.rmtree(full_path)
            else:
              continue
            deleted += 1
            _tool_job_append(job, f"deleted: {name}")
          except Exception as e:
            _tool_job_append(job, f"delete error: {e}")
      result = {"ok": True, "out": f"deleted entries: {deleted}"}
      _tool_job_finish(job, ok=True, result=result)
      return

    if action == "send_tmux_log":
      _tool_job_progress(job, message="capture tmux", current=1, total=1)
      cmd = "rm -f /data/media/tmux.log && tmux capture-pane -pq -S-1000 > /data/media/tmux.log"
      rc = await asyncio.to_thread(
        lambda: subprocess.run(cmd, shell=True, capture_output=True, text=True).returncode
      )
      if rc != 0:
        _tool_job_finish(
          job,
          ok=False,
          result={"ok": False, "error": "tmux capture failed", "error_code": "TMUX_CAPTURE_FAIL"},
          error="tmux capture failed",
          error_code="TMUX_CAPTURE_FAIL",
        )
        return
      result = {"ok": True, "out": "tmux log captured", "file": "/download/tmux.log"}
      _tool_job_finish(job, ok=True, result=result)
      return

    if action == "server_tmux_log":
      _tool_job_progress(job, message="send tmux", current=1, total=1)
      params = Params()
      params.put_nonblocking("CarrotException", "tmux_send")
      _tool_job_finish(job, ok=True, result={"ok": True, "out": "tmux send triggered"})
      return

    if action == "install_required":
      import importlib.util

      packages = [
        {"pip": "flask", "import": "flask"},
        {"pip": "shapely", "import": "shapely"},
        {"pip": "kaitaistruct", "import": "kaitaistruct"},
      ]
      results = []
      installed_any = False

      for idx, item in enumerate(packages, start=1):
        pip_name = item["pip"]
        import_name = item["import"]
        _tool_job_progress(job, message=f"checking {pip_name}", current=idx - 1, total=len(packages))

        if importlib.util.find_spec(import_name) is not None:
          results.append({"package": pip_name, "status": "already_installed"})
          _tool_job_append(job, f"{pip_name}: already installed")
          continue

        _tool_job_progress(job, message=f"installing {pip_name}", current=idx, total=len(packages))
        _tool_job_append(job, f"$ pip install {pip_name}")
        rc = await _tool_stream_exec(job, ["pip", "install", pip_name], timeout=300)
        results.append({"package": pip_name, "status": "installed" if rc == 0 else "failed", "returncode": rc})
        if rc != 0:
          _tool_job_finish(
            job,
            ok=False,
            result={
              "ok": False,
              "error": f"pip install failed: {pip_name}",
              "results": results,
              "need_reboot": False,
            },
            error=f"pip install failed: {pip_name}",
          )
          return
        installed_any = True

      result = {
        "ok": True,
        "out": "required packages installed. reboot is required to apply changes." if installed_any else "all required packages are already installed.",
        "results": results,
        "need_reboot": installed_any,
      }
      _tool_job_finish(job, ok=True, result=result)
      return

    if action == "backup_settings":
      if not HAS_PARAMS or ParamKeyType is None:
        _tool_job_finish(
          job,
          ok=False,
          result={"ok": False, "error": "Params/ParamKeyType not available"},
          error="Params/ParamKeyType not available",
        )
        return

      _tool_job_progress(job, message="backup settings", current=1, total=1)
      values = _get_all_param_values_for_backup()
      os.makedirs(os.path.dirname(PARAMS_BACKUP_PATH), exist_ok=True)
      with open(PARAMS_BACKUP_PATH, "w", encoding="utf-8") as f:
        json.dump(values, f, ensure_ascii=False, indent=2)
      result = {"ok": True, "out": f"backup saved ({len(values)} keys)", "file": "/download/params_backup.json"}
      _tool_job_finish(job, ok=True, result=result)
      return

    if action == "reset_calib":
      _tool_job_progress(job, message="reset calibration", current=1, total=1)
      import glob as _glob
      for f in ["/data/params/d_tmp/CalibrationParams", "/data/params/d/CalibrationParams"]:
        try:
          os.remove(f)
          _tool_job_append(job, f"removed {f}")
        except FileNotFoundError:
          pass
        except Exception as e:
          _tool_job_append(job, f"error removing {f}: {e}")
      _tool_job_finish(job, ok=True, result={"ok": True, "out": "calibration reset"})
      await asyncio.sleep(1)
      subprocess.Popen(["sudo", "reboot"])
      return

    if action == "reboot":
      _tool_job_progress(job, message="request reboot", current=1, total=1)
      subprocess.Popen(["sudo", "reboot"])
      _tool_job_finish(job, ok=True, result={"ok": True, "out": "reboot requested"})
      return

    if action == "rebuild_all":
      _tool_job_progress(job, message="rebuild all", current=1, total=1)
      cmd = "cd /data/openpilot && scons -c && rm -rf prebuilt && sudo reboot"
      subprocess.Popen(cmd, shell=True)
      _tool_job_finish(job, ok=True, result={"ok": True, "out": "rebuild_all requested (clean + remove prebuilt + reboot)"})
      return

    if action == "shell_cmd":
      cmd_str = (body.get("cmd") or "").strip()
      if not cmd_str:
        _tool_job_finish(job, ok=False, result={"ok": False, "error": "missing cmd"}, error="missing cmd")
        return
      try:
        argv = shlex.split(cmd_str)
      except Exception:
        _tool_job_finish(job, ok=False, result={"ok": False, "error": "bad cmd format"}, error="bad cmd format")
        return
      if not argv:
        _tool_job_finish(job, ok=False, result={"ok": False, "error": "empty cmd"}, error="empty cmd")
        return

      alias_map = {
        "pull": ["git", "pull"],
        "status": ["git", "status"],
        "branch": ["git", "branch"],
        "log": ["git", "log"],
      }
      if argv[0] in alias_map:
        argv = alias_map[argv[0]] + argv[1:]

      allowed_top = {"git", "df", "free", "uptime", "scons", "rm", "echo", "sleep", "sudo", "reboot", "cat", "ls"}
      if argv[0] not in allowed_top:
        _tool_job_finish(
          job,
          ok=False,
          result={
            "ok": False,
            "error": f"not allowed: {argv[0]}",
            "error_code": "CMD_NOT_ALLOWED",
            "error_detail": argv[0],
          },
          error=f"not allowed: {argv[0]}",
          error_code="CMD_NOT_ALLOWED",
          error_detail=argv[0],
        )
        return

      _tool_job_progress(job, message=cmd_str, current=1, total=1)
      _tool_job_append(job, f"$ {cmd_str}")
      try:
        rc = await _tool_stream_exec(job, argv, cwd="/data/openpilot", timeout=10)
      except asyncio.TimeoutError:
        _tool_job_finish(
          job,
          ok=False,
          result={"ok": False, "error": "timeout", "error_code": "CMD_TIMEOUT"},
          error="timeout",
          error_code="CMD_TIMEOUT",
        )
        return
      result = {
        "ok": rc == 0,
        "out": (job.get("log") or "").strip() or "(no output)",
        "returncode": rc,
      }
      _tool_job_finish(job, ok=rc == 0, result=result)
      return

    _tool_job_finish(job, ok=False, result={"ok": False, "error": f"unknown action: {action}"}, error=f"unknown action: {action}")
  except asyncio.TimeoutError:
    _tool_job_finish(
      job,
      ok=False,
      result={"ok": False, "error": "timeout", "error_code": "CMD_TIMEOUT"},
      error="timeout",
      error_code="CMD_TIMEOUT",
    )
  except Exception as e:
    _tool_job_append(job, f"\n{traceback.format_exc()}")
    _tool_job_finish(job, ok=False, result={"ok": False, "error": str(e)}, error=str(e))


async def api_tools_start(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  action = body.get("action")
  if not action:
    return web.json_response({"ok": False, "error": "missing action"}, status=400)

  job_id = uuid.uuid4().hex[:12]
  job = {
    "id": job_id,
    "action": str(action),
    "payload": dict(body),
    "status": "running",
    "log": "",
    "progress": 0,
    "message": "",
    "step_current": 0,
    "step_total": 0,
    "error": None,
    "error_code": None,
    "error_detail": None,
    "result": None,
    "created_at": time.time(),
    "updated_at": time.time(),
  }
  _tool_jobs[job_id] = job
  _tool_job_prune()
  asyncio.create_task(_run_tool_job(job))
  return web.json_response({"ok": True, "job_id": job_id, "status": job["status"]})


async def api_tools_job(request: web.Request) -> web.Response:
  job_id = (request.query.get("id") or request.match_info.get("job_id") or "").strip()
  if not job_id:
    return web.json_response({"ok": False, "error": "missing job id"}, status=400)

  job = _tool_jobs.get(job_id)
  if not job:
    return web.json_response({"ok": False, "error": "job not found"}, status=404)

  return web.json_response(_tool_job_snapshot(job))

async def api_tools(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  action = body.get("action")
  if not action:
    return web.json_response({"ok": False, "error": "missing action"}, status=400)

  # 최소 보안: 사설대역만 허용 (권장)
  #ip = request.remote or ""
  #if not (ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172.16.") or ip.startswith("172.17.") or ip in ("127.0.0.1", "::1")):
  #  return web.json_response({"ok": False, "error": "forbidden"}, status=403)

  def run(cmd: List[str], cwd: Optional[str] = None) -> Tuple[int, str]:
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    out = (p.stdout or "") + (("\n" + p.stderr) if p.stderr else "")
    return p.returncode, out.strip()

  try:
    # repo 위치는 당신 환경에 맞게 조정
    REPO_DIR = "/data/openpilot"

    if action == "git_pull":
      rc, out = run(["git", "pull"], cwd=REPO_DIR)
      if rc == 0 and _did_git_pull_update(out):
        _write_git_pull_time()
      return web.json_response({"ok": rc == 0, "rc": rc, "out": out})

    if action == "git_sync":
      # 목적: 현재 체크아웃된 브랜치만 남기고 로컬 브랜치 모두 삭제 후 fetch/prune
      rc1, out1 = run(["bash", "-lc", "git branch | grep -v '^\\*' | xargs -r git branch -D"], cwd=REPO_DIR)
      if rc1 != 0:
        return web.json_response({"ok": False, "rc": rc1, "out": out1})

      rc2, out2 = run(["git", "fetch", "--all", "--prune"], cwd=REPO_DIR)
      out = (out1 + "\n\n" + out2).strip()
      return web.json_response({"ok": rc2 == 0, "rc": rc2, "out": out})


    if action == "git_reset":
      mode = (body.get("mode") or "hard").strip()
      target = (body.get("target") or "HEAD").strip()
      if mode not in ("hard", "soft", "mixed"):
        return web.json_response({"ok": False, "error": "bad mode"}, status=400)
      rc, out = run(["git", "reset", f"--{mode}", target], cwd=REPO_DIR)
      return web.json_response({"ok": rc == 0, "rc": rc, "out": out})

    if action == "git_checkout":
      branch = (body.get("branch") or "").strip()
      kind = str(body.get("kind") or "").strip()
      item_name = str(body.get("name") or "").strip()
      item_remote = str(body.get("remote") or "").strip()
      if not branch and not item_name:
        return web.json_response({"ok": False, "error": "missing branch"}, status=400)

      rc_fetch, out_fetch = run(["git", "fetch", "--all", "--prune"], cwd=REPO_DIR)
      if rc_fetch != 0:
        return web.json_response({"ok": False, "rc": rc_fetch, "out": out_fetch})

      rc_remotes, out_remotes = run(["git", "remote"], cwd=REPO_DIR)
      known_remotes = out_remotes.split() if rc_remotes == 0 else ["origin"]
      remote_prefix = None
      for remote in known_remotes:
        if branch.startswith(f"{remote}/"):
          remote_prefix = remote
          break

      try:
        if kind == "local":
          local_branch = item_name or branch
          rc, out = run(["git", "switch", local_branch], cwd=REPO_DIR)
        elif kind == "remote":
          if not item_remote or not item_name:
            return web.json_response({"ok": False, "error": "missing remote branch info"}, status=400)
          if item_remote not in known_remotes:
            return web.json_response({"ok": False, "error": f"unknown remote: {item_remote}"}, status=400)
          branch = f"{item_remote}/{item_name}"
          local_branch = item_name
          rc_check, _ = run(["git", "show-ref", "--verify", "--quiet", f"refs/heads/{local_branch}"], cwd=REPO_DIR)
          if rc_check == 0:
            rc, out = run(["git", "switch", local_branch], cwd=REPO_DIR)
          else:
            rc, out = run(
              ["git", "switch", "-c", local_branch, "--track", branch],
              cwd=REPO_DIR
            )
        elif remote_prefix is not None:
          local_branch = branch[len(remote_prefix) + 1:]
          rc_check, _ = run(["git", "show-ref", "--verify", "--quiet", f"refs/heads/{local_branch}"], cwd=REPO_DIR)
          if rc_check == 0:
            rc, out = run(["git", "switch", local_branch], cwd=REPO_DIR)
          else:
            rc, out = run(
              ["git", "switch", "-c", local_branch, "--track", branch],
              cwd=REPO_DIR
            )
        else:
          rc, out = run(["git", "switch", branch], cwd=REPO_DIR)
          if rc != 0:
            rc2, out2 = run(
              ["git", "switch", "-c", branch, "--track", f"origin/{branch}"],
              cwd=REPO_DIR
            )
            rc, out = rc2, out2
        return web.json_response({"ok": rc == 0, "rc": rc, "out": out})
      except Exception as e:
        return web.json_response({"ok": False, "error": str(e)}, status=500)
  
    if action == "git_branch_list":
      rc0, out0 = run(["git", "fetch", "--all", "--prune"], cwd=REPO_DIR)
      if rc0 != 0:
        return web.json_response({"ok": False, "rc": rc0, "out": out0})

      rc_local, out_local = run(
        ["git", "for-each-ref", "--format=%(refname:short)", "refs/heads"],
        cwd=REPO_DIR
      )
      rc_remote, out_remote = run(
        ["git", "for-each-ref", "--format=%(refname:short)", "refs/remotes"],
        cwd=REPO_DIR
      )
      if rc_local != 0 or rc_remote != 0:
        merged = (out0 + "\n\n" + out_local + "\n" + out_remote).strip()
        return web.json_response({"ok": False, "rc": rc_local if rc_local != 0 else rc_remote, "out": merged})

      rc_current, out_current = run(["git", "branch", "--show-current"], cwd=REPO_DIR)
      current_branch = out_current.strip() if rc_current == 0 else ""

      rc_remotes, out_remotes = run(["git", "remote"], cwd=REPO_DIR)
      remotes = out_remotes.split() if rc_remotes == 0 else ["origin"]
      rc_remote_urls, out_remote_urls = run(["git", "remote", "-v"], cwd=REPO_DIR)
      remote_urls = _parse_remote_urls(out_remote_urls) if rc_remote_urls == 0 else {}
      branch_items = _build_branch_items(out_local, out_remote, remotes)
      branches = sorted({str(item.get("ref") or "") for item in branch_items if item.get("ref")})

      return web.json_response({
        "ok": True,
        "branches": branches,
        "branch_items": branch_items,
        "current_branch": current_branch,
        "fetch": out0.strip(),
        "device_type": HARDWARE.get_device_type(),
        "branch_prefix": _get_branch_prefix(),
        "remotes": remotes,
        "remote_urls": remote_urls,
      })
    

    if action == "git_remote_add":
      name = (body.get("name") or "").strip()
      url = (body.get("url") or "").strip()
      if not name or not url:
        return web.json_response({"ok": False, "error": "missing name or url"}, status=400)

      rc_remotes, out_remotes = run(["git", "remote"], cwd=REPO_DIR)
      remotes = out_remotes.split() if rc_remotes == 0 else []
      remote_exists = name in remotes
      setup_cmd = ["git", "remote", "set-url", name, url] if remote_exists else ["git", "remote", "add", name, url]
      rc_setup, out_setup = run(setup_cmd, cwd=REPO_DIR)
      if rc_setup != 0:
        return web.json_response({"ok": False, "rc": rc_setup, "out": out_setup})

      rc_fetch, out_fetch = run(["git", "fetch", "--prune", name], cwd=REPO_DIR)
      rc_remote_urls, out_remote_urls = run(["git", "remote", "-v"], cwd=REPO_DIR)
      out = (out_setup + "\n" + out_fetch + "\n\n> git remote -v\n" + (out_remote_urls if rc_remote_urls == 0 else "")).strip()
      return web.json_response({"ok": rc_fetch == 0, "rc": rc_fetch, "out": out})

    if action == "git_log":
      count = min(int(body.get("count") or 20), 50)
      rc, out = run(["git", "log", "--oneline", f"-{count}"], cwd=REPO_DIR)
      rc_head, out_head = run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_DIR)
      current_commit = out_head.strip() if rc_head == 0 else ""
      commits = []
      for line in (out or "").splitlines():
        line = line.strip()
        if not line:
          continue
        parts = line.split(" ", 1)
        commits.append({"hash": parts[0], "message": parts[1] if len(parts) > 1 else ""})
      return web.json_response({"ok": rc == 0, "commits": commits, "current_commit": current_commit, "out": out})

    if action == "git_reset_repo_fetch":
      url = "https://github.com/ajouatom/openpilot.git"
      out_all = ""

      # Configure origin
      rc_set, out_set = run(["git", "remote", "set-url", "origin", url], cwd=REPO_DIR)
      if rc_set != 0:
        run(["git", "remote", "remove", "origin"], cwd=REPO_DIR)
        rc_add, out_add = run(["git", "remote", "add", "origin", url], cwd=REPO_DIR)
        out_all += f"> git remote add origin {url}\n{out_add}\n\n"
        if rc_add != 0:
          return web.json_response({"ok": False, "error": f"failed to configure remote: {out_add}"})
      else:
        out_all += f"> git remote set-url origin {url}\n{out_set}\n\n"

      # Remove ALL other remotes
      rc_rem, out_rem = run(["git", "remote"], cwd=REPO_DIR)
      for rname in (out_rem or "").splitlines():
        rname = rname.strip()
        if rname and rname != "origin":
          run(["git", "remote", "remove", rname], cwd=REPO_DIR)
          out_all += f"> removed remote: {rname}\n"

      # Fetch origin only
      rc_fetch, out_fetch = run(["git", "fetch", "origin", "--prune"], cwd=REPO_DIR)
      out_all += f"> git fetch origin --prune\n{out_fetch}\n\n"
      if rc_fetch != 0:
        return web.json_response({"ok": False, "rc": rc_fetch, "out": out_all.strip()})

      # List origin/* branches only
      rc_br, out_br = run(["git", "branch", "-r"], cwd=REPO_DIR)
      branches = []
      for line in (out_br or "").splitlines():
        line = line.strip()
        if not line or "->" in line:
          continue
        if not line.startswith("origin/"):
          continue
        branches.append(line.split("/", 1)[1])
      branches = sorted(set(branches))
      return web.json_response({"ok": True, "branches": branches, "out": out_all.strip()})

    if action == "git_reset_repo_checkout":
      branch = str(body.get("branch") or "").strip()
      if not branch:
        return web.json_response({"ok": False, "error": "missing branch"}, status=400)
      commands = [
        ["git", "checkout", "-B", branch, f"origin/{branch}"],
        ["git", "reset", "--hard", f"origin/{branch}"],
        ["git", "clean", "-xfd"],
      ]
      out_all = ""
      for c in commands:
        rc, out = run(c, cwd=REPO_DIR)
        out_all += f"> {' '.join(c)}\n{out}\n\n"
        if rc != 0:
          return web.json_response({"ok": False, "rc": rc, "out": out_all.strip()})
      return web.json_response({"ok": True, "out": out_all.strip()})

    if action == "delete_all_videos":
      # 경로는 환경 맞춰 조정
      # openpilot device: /data/media/0/videos
      paths = ["/data/media/0/videos"]
      deleted = 0
      for pth in paths:
        if not os.path.isdir(pth):
          continue
        for fn in glob.glob(os.path.join(pth, "*")):
          try:
            os.remove(fn)
            deleted += 1
          except Exception:
            pass
      return web.json_response({"ok": True, "out": f"deleted files: {deleted}"})

    if action == "delete_all_logs":
      # 경로는 환경 맞춰 조정
      # openpilot device: /data/media/0/realdata
      paths = ["/data/media/0/realdata"]
      deleted = 0
      for pth in paths:
        if not os.path.isdir(pth):
          continue

        for name in os.listdir(pth):
          full_path = os.path.join(pth, name)
          try:
            if os.path.isfile(full_path) or os.path.islink(full_path):
              os.remove(full_path)
              deleted += 1
            elif os.path.isdir(full_path):
              shutil.rmtree(full_path)
              deleted += 1
          except Exception as e:
            print("delete error:", e)

      return web.json_response({"ok": True, "out": f"deleted entries: {deleted}"})



    if action == "send_tmux_log":
      log_path = "/data/media/tmux.log"

      cmd = (
        "rm -f /data/media/tmux.log && "
        "tmux capture-pane -pq -S-1000 > /data/media/tmux.log"
      )

      p = subprocess.run(
        cmd,
        shell=True,
        capture_output=True,
        text=False
      )

      if p.returncode != 0:
        return web.json_response({
          "ok": False,
          "error": "tmux capture failed"
        })

      return web.json_response({
        "ok": True,
        "out": "tmux log captured",
        "file": "/download/tmux.log"
      })
    
    if action == "server_tmux_log":
      params = Params()
      params.put_nonblocking("CarrotException", "tmux_send")
      return web.json_response({"ok": True, "out": "tmux send triggered"})

    if action == "install_required":
      import importlib.util

      packages = [
        {"pip": "flask", "import": "flask"},
        {"pip": "shapely", "import": "shapely"},
        {"pip": "kaitaistruct", "import": "kaitaistruct"},
      ]

      results = []
      installed_any = False

      for item in packages:
        pip_name = item["pip"]
        import_name = item["import"]

        try:
          if importlib.util.find_spec(import_name) is not None:
            results.append({
              "package": pip_name,
              "status": "already_installed",
            })
            continue

          cmd = ["pip", "install", pip_name]
          p = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=300
          )

          results.append({
            "package": pip_name,
            "status": "installed" if p.returncode == 0 else "failed",
            "returncode": p.returncode,
            "stdout": (p.stdout or "")[-2000:],
            "stderr": (p.stderr or "")[-2000:],
          })

          if p.returncode != 0:
            return web.json_response({
              "ok": False,
              "error": f"pip install failed: {pip_name}",
              "results": results,
              "need_reboot": False,
            }, status=500)

          installed_any = True

        except Exception as e:
          return web.json_response({
            "ok": False,
            "error": f"exception while checking/installing {pip_name}: {str(e)}",
            "results": results,
            "need_reboot": False,
          }, status=500)

      if installed_any:
        return web.json_response({
          "ok": True,
          "out": "required packages installed. reboot is required to apply changes.",
          "results": results,
          "need_reboot": True,
        })

      return web.json_response({
        "ok": True,
        "out": "all required packages are already installed.",
        "results": results,
        "need_reboot": False,
      })    
    if action == "backup_settings":
      if not HAS_PARAMS or ParamKeyType is None:
        return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

      # 사설대역 제한
      #ip = request.remote or ""
      #if not (ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172.16.") or ip.startswith("172.17.") or ip in ("127.0.0.1", "::1")):
      #  return web.json_response({"ok": False, "error": "forbidden"}, status=403)

      try:
        values = _get_all_param_values_for_backup()

        os.makedirs(os.path.dirname(PARAMS_BACKUP_PATH), exist_ok=True)
        with open(PARAMS_BACKUP_PATH, "w", encoding="utf-8") as f:
          json.dump(values, f, ensure_ascii=False, indent=2)

        return web.json_response({"ok": True, "out": f"backup saved ({len(values)} keys)", "file": "/download/params_backup.json"})
      except Exception as e:
        return web.json_response({"ok": False, "error": str(e)}, status=500)

    if action == "reset_calib":
      import glob as _glob
      out_msg = []
      for f in ["/data/params/d_tmp/CalibrationParams", "/data/params/d/CalibrationParams"]:
        try:
          os.remove(f)
          out_msg.append(f"removed {f}")
        except FileNotFoundError:
          pass
        except Exception as e:
          out_msg.append(f"error removing {f}: {e}")
      # start reboot async
      subprocess.Popen("sleep 1 && sudo reboot", shell=True)
      return web.json_response({"ok": True, "out": "\n".join(out_msg) or "calibration reset"})

    if action == "reboot":
      subprocess.Popen(["sudo", "reboot"])
      return web.json_response({"ok": True, "out": "reboot requested"})

    if action == "rebuild_all":
      # cd /data/openpilot
      # scons -c
      # rm -rf prebuilt
      # sudo reboot
      cmd = "cd /data/openpilot && scons -c && rm -rf prebuilt && sudo reboot"
      subprocess.Popen(cmd, shell=True)
      return web.json_response({"ok": True, "out": "rebuild_all requested (clean + remove prebuilt + reboot)"})

    if action == "shell_cmd":
      cmd_str = (body.get("cmd") or "").strip()
      if not cmd_str:
        return web.json_response({"ok": False, "error": "missing cmd"}, status=400)

      # 화이트리스트: "첫 토큰" 기준 + git은 서브커맨드 제한
      try:
        argv = shlex.split(cmd_str)
      except Exception:
        return web.json_response({"ok": False, "error": "bad cmd format"}, status=400)

      if not argv:
        return web.json_response({"ok": False, "error": "empty cmd"}, status=400)

      alias_map = {
        "pull": ["git", "pull"],
        "status": ["git", "status"],
        "branch": ["git", "branch"],
        "log": ["git", "log"],
      }
      if argv[0] in alias_map:
        argv = alias_map[argv[0]] + argv[1:]

      allowed_top = {"git", "df", "free", "uptime", "scons", "rm", "echo", "sleep", "sudo", "reboot", "cat", "ls"}
      if argv[0] not in allowed_top:
        return web.json_response({"ok": False, "error": f"not allowed: {argv[0]}"}, status=403)

      """
      # git subcommand 제한
      if argv[0] == "git":
        if len(argv) < 2:
          return web.json_response({"ok": False, "error": "git needs subcommand"}, status=400)
        allowed_git = {"pull", "status", "branch", "log", "rev-parse"}
        if argv[1] not in allowed_git:
          return web.json_response({"ok": False, "error": f"git subcommand not allowed: {argv[1]}"}, status=403)
      """
      # 실행 (shell=False 유지)
      try:
        p = subprocess.run(
          argv,
          cwd="/data/openpilot",     # 필요시 조정
          capture_output=True,
          text=True,
          timeout=10
        )
        out = ""
        if p.stdout: out += p.stdout
        if p.stderr: out += ("\n" + p.stderr if out else p.stderr)
        out = out.strip() or "(no output)"
        return web.json_response({"ok": True, "out": out, "returncode": p.returncode})
      except subprocess.TimeoutExpired:
        return web.json_response({"ok": False, "error": "timeout"}, status=504)
      except Exception as e:
        return web.json_response({"ok": False, "error": str(e)}, status=500)



    return web.json_response({"ok": False, "error": f"unknown action: {action}"}, status=400)

  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)

async def ws_raw(request: web.Request) -> web.WebSocketResponse:
  service = (request.match_info.get("service") or "").strip()
  hub: RawWsHub | None = request.app.get("realtime_raw_hub")
  if hub is None:
    raise web.HTTPServiceUnavailable(text="realtime raw hub unavailable")
  if not service or not hub.is_allowed_service(service):
    raise web.HTTPNotFound(text=f"unknown raw service: {service}")

  ws = web.WebSocketResponse(heartbeat=20, max_msg_size=8 * 1024 * 1024, compress=False)
  await ws.prepare(request)
  await ws.send_str(json.dumps(build_raw_hello(service=service), separators=(",", ":")))
  await hub.register(service, ws)
  try:
    async for msg in ws:
      if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    await hub.unregister_client(ws)

  return ws


async def ws_raw_multiplex(request: web.Request) -> web.WebSocketResponse:
  hub: RawWsHub | None = request.app.get("realtime_raw_hub")
  if hub is None:
    raise web.HTTPServiceUnavailable(text="realtime raw hub unavailable")

  services_param = request.query.get("services", "")
  services = [service.strip() for service in services_param.split(",") if service.strip()]
  if not services:
    raise web.HTTPBadRequest(text="missing raw services")
  invalid = [service for service in services if not hub.is_allowed_service(service)]
  if invalid:
    raise web.HTTPNotFound(text=f"unknown raw services: {','.join(invalid)}")

  ws = web.WebSocketResponse(heartbeat=20, max_msg_size=8 * 1024 * 1024, compress=False)
  await ws.prepare(request)
  await ws.send_str(json.dumps(build_raw_multiplex_hello(services=services), separators=(",", ":")))
  await hub.register_many(services, ws)
  try:
    async for msg in ws:
      if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.ERROR):
        break
  finally:
    await hub.unregister_client(ws)

  return ws


async def ws_camera(request: web.Request) -> web.WebSocketResponse:
  hub: CameraWsHub | None = request.app.get("realtime_camera_hub")
  if hub is None:
    raise web.HTTPServiceUnavailable(text="realtime camera hub unavailable")
  return await hub.ws_camera(request)

def _tmux_run(args: List[str], timeout: float = 5.0, check: bool = False) -> subprocess.CompletedProcess:
  return subprocess.run(
    args,
    capture_output=True,
    text=True,
    timeout=timeout,
    check=check,
  )

def _tmux_has_session(session: str) -> bool:
  p = _tmux_run(["tmux", "has-session", "-t", session], timeout=2.5)
  return p.returncode == 0

def _tmux_send_keys(session: str, *keys: str, literal: bool = False) -> None:
  cmd = ["tmux", "send-keys", "-t", session]
  if literal:
    cmd.append("-l")
  cmd.extend(keys)
  _tmux_run(cmd, timeout=4.0, check=True)

def _tmux_bootstrap_shell(cwd: str = TMUX_START_DIR) -> str:
  return f"cd {shlex.quote(cwd)} && exec bash -il"

def _tmux_start_command() -> str:
  if os.name != "posix":
    return "powershell"

  current_user = (
    os.environ.get("USER")
    or os.environ.get("USERNAME")
    or getpass.getuser()
  )
  geteuid = getattr(os, "geteuid", None)
  euid = geteuid() if callable(geteuid) else None
  bootstrap = _tmux_bootstrap_shell()

  if current_user == "comma":
    return bootstrap

  if euid == 0:
    return f"exec su - comma -c {shlex.quote(bootstrap)}"

  if shutil.which("sudo"):
    try:
      probe = subprocess.run(
        ["sudo", "-n", "-u", "comma", "true"],
        capture_output=True,
        text=True,
        timeout=2,
      )
      if probe.returncode == 0:
        return f"exec sudo -n -u comma -i bash -lc {shlex.quote(bootstrap)}"
    except Exception:
      pass

  return bootstrap

def _tmux_ensure_session(session: str = TMUX_WEB_SESSION) -> bool:
  created = False
  if not _tmux_has_session(session):
    _tmux_run(
      ["tmux", "new-session", "-d", "-s", session, _tmux_start_command()],
      timeout=5.0,
      check=True,
    )
    created = True
  return created

def _tmux_capture(session: str = TMUX_WEB_SESSION, lines: int = TMUX_CAPTURE_LINES) -> str:
  p = _tmux_run(
    ["tmux", "capture-pane", "-p", "-J", "-t", session, "-S", f"-{max(lines, 40)}"],
    timeout=4.0,
    check=False,
  )
  if p.returncode != 0:
    return ""
  return (p.stdout or "").rstrip() or " "

def _tmux_send_line(session: str, line: str) -> None:
  if line:
    _tmux_send_keys(session, line, literal=True)
  _tmux_send_keys(session, "Enter")

def _tmux_ctrl_c(session: str) -> None:
  _tmux_send_keys(session, "C-c")

def _tmux_clear(session: str) -> None:
  _tmux_send_line(session, "clear")
  time.sleep(0.04)
  _tmux_run(["tmux", "clear-history", "-t", session], timeout=4.0, check=False)

async def ws_terminal(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20, compress=False)
  await ws.prepare(request)

  session = (request.query.get("session") or TMUX_WEB_SESSION).strip() or TMUX_WEB_SESSION
  last_screen = None

  try:
    created = await asyncio.to_thread(_tmux_ensure_session, session)
    await ws.send_str(json.dumps({
      "type": "meta",
      "session": session,
      "created": created,
      "user": "comma",
    }))
  except Exception as e:
    await ws.send_str(json.dumps({
      "type": "error",
      "error": str(e),
      "session": session,
    }))
    await ws.close()
    return ws

  async def push_screen(force: bool = False, delay: float = 0.0) -> None:
    nonlocal last_screen
    if delay > 0:
      await asyncio.sleep(delay)
    screen = await asyncio.to_thread(_tmux_capture, session)
    if force or screen != last_screen:
      last_screen = screen
      await ws.send_str(json.dumps({
        "type": "screen",
        "session": session,
        "text": screen,
      }))

  async def pump_screen():
    while not ws.closed:
      try:
        await push_screen(force=False)
      except asyncio.CancelledError:
        raise
      except Exception as e:
        await ws.send_str(json.dumps({
          "type": "error",
          "error": str(e),
          "session": session,
        }))
        break
      await asyncio.sleep(0.18)

  pump_task = asyncio.create_task(pump_screen())

  try:
    await push_screen(force=True, delay=0.02)
    async for msg in ws:
      if msg.type == WSMsgType.TEXT:
        try:
          data = json.loads(msg.data)
        except Exception:
          continue

        typ = data.get("type")
        try:
          if typ == "input":
            await asyncio.to_thread(_tmux_send_line, session, str(data.get("data") or ""))
            await push_screen(force=True, delay=0.03)
          elif typ == "control":
            action = (data.get("action") or "").strip()
            if action == "ctrl_c":
              await asyncio.to_thread(_tmux_ctrl_c, session)
              await push_screen(force=True, delay=0.03)
            elif action == "clear":
              await asyncio.to_thread(_tmux_clear, session)
              await push_screen(force=True, delay=0.05)
            elif action == "refresh":
              await push_screen(force=True)
            elif action == "new_session":
              await asyncio.to_thread(_tmux_run, ["tmux", "kill-session", "-t", session], 3.0, False)
              created = await asyncio.to_thread(_tmux_ensure_session, session)
              await ws.send_str(json.dumps({
                "type": "meta",
                "session": session,
                "created": created,
                "user": "comma",
              }))
              await push_screen(force=True, delay=0.08)
        except Exception as e:
          await ws.send_str(json.dumps({
            "type": "error",
            "error": str(e),
            "session": session,
          }))
      elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE, WSMsgType.CLOSING):
        break
  finally:
    pump_task.cancel()
    try:
      await pump_task
    except Exception:
      pass
    try:
      await ws.close()
    except Exception:
      pass
  return ws

async def handle_download_tmux(request: web.Request) -> web.Response:
  path = "/data/media/tmux.log"
  if not os.path.exists(path):
    return web.json_response({"ok": False, "error": "file not found"}, status=404)

  return web.FileResponse(
    path,
    headers={
      "Content-Disposition": "attachment; filename=tmux.log"
    }
  )

PARAMS_BACKUP_PATH = "/data/media/params_backup.json"
def _get_all_param_values_for_backup() -> Dict[str, str]:
  if not HAS_PARAMS or ParamKeyType is None:
    raise RuntimeError("Params/ParamKeyType not available")

  params = Params()
  out: Dict[str, str] = {}

  for k in params.all_keys():
    # key normalize
    if isinstance(k, (bytes, bytearray, memoryview)):
      try:
        key = k.decode("utf-8")
      except Exception:
        continue
    else:
      key = str(k)

    # type
    try:
      t = params.get_type(key)
    except Exception:
      continue

    # skip heavy/unsupported
    if t in (ParamKeyType.BYTES, ParamKeyType.JSON):
      continue

    # default 없는 키 제외(당신 로직 유지)
    try:
      dv = params.get_default_value(key)
    except Exception:
      continue
    if dv is None:
      continue

    # read current
    try:
      v = params.get(key, block=False, return_default=False)
    except Exception:
      v = None

    if v is None:
      v = dv

    # stringify for JSON file
    if isinstance(v, (dict, list)):
      out[key] = json.dumps(v, ensure_ascii=False)
    else:
      out[key] = str(v)

  return out

def _restore_param_values_from_backup(values: Dict[str, Any]) -> Dict[str, Any]:
  if not HAS_PARAMS or ParamKeyType is None:
    raise RuntimeError("Params/ParamKeyType not available")

  params = Params()
  ok_cnt = 0
  fail_cnt = 0
  fails = []

  for key, value in values.items():
    try:
      t = params.get_type(key)

      if t == ParamKeyType.BOOL:
        v = value in ("1", "true", "True", "on", "yes") if isinstance(value, str) else bool(value)
        params.put_bool(key, v)

      elif t == ParamKeyType.INT:
        params.put_int(key, int(float(value)))

      elif t == ParamKeyType.FLOAT:
        params.put_float(key, float(value))

      elif t == ParamKeyType.TIME:
        params.put(key, str(value))

      elif t == ParamKeyType.STRING:
        params.put(key, str(value))

      # JSON/BYTES는 백업에서 제외했지만, 혹시 들어오면 skip
      else:
        continue

      ok_cnt += 1

    except Exception as e:
      fail_cnt += 1
      fails.append({"key": key, "err": str(e)})

  return {"ok_cnt": ok_cnt, "fail_cnt": fail_cnt, "fails": fails[:30]}

async def handle_download_params_backup(request: web.Request) -> web.Response:
  path = PARAMS_BACKUP_PATH
  if not os.path.exists(path):
    return web.json_response({"ok": False, "error": "file not found"}, status=404)

  return web.FileResponse(
    path,
    headers={"Content-Disposition": "attachment; filename=params_backup.json"}
  )

async def api_params_restore(request: web.Request) -> web.Response:
  if not HAS_PARAMS or ParamKeyType is None:
    return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

  try:
    reader = await request.multipart()
    part = await reader.next()
    if part is None or part.name != "file":
      return web.json_response({"ok": False, "error": "missing file field"}, status=400)

    data = await part.read(decode=False)
    text = data.decode("utf-8", errors="replace")
    j = json.loads(text)

    if not isinstance(j, dict):
      return web.json_response({"ok": False, "error": "bad json format (must be object)"}, status=400)

    values = j
    res = _restore_param_values_from_backup(values)
    return web.json_response({"ok": True, "result": res})

  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=500)



# -----------------------
# Browser -> server time sync
# -----------------------
TIME_SYNC_THRESHOLD_SEC = 10
TIME_SYNC_DEBUG_DEFAULT = True


def _run_cmd_debug(cmd: list[str]) -> dict:
  try:
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return {
      "cmd": cmd,
      "returncode": proc.returncode,
      "stdout": (proc.stdout or "").strip(),
      "stderr": (proc.stderr or "").strip(),
    }
  except Exception as e:
    return {
      "cmd": cmd,
      "returncode": -1,
      "stdout": "",
      "stderr": str(e),
    }


def sync_system_time_from_browser(epoch_ms: int, timezone_name: str, debug: bool = False) -> dict:
  import os
  import time
  import datetime
  import subprocess

  server_epoch = int(time.time())
  target_epoch = int(epoch_ms // 1000)
  diff_sec = target_epoch - server_epoch

  localtime_path = "/data/etc/localtime"
  timezone_name = (timezone_name or "").strip() or "UTC"
  zoneinfo_path = f"/usr/share/zoneinfo/{timezone_name}"

  result: dict[str, Any] = {
    "ok": True,
    "applied": False,
    "server_epoch": server_epoch,
    "target_epoch": target_epoch,
    "diff_sec": diff_sec,
    "timezone": timezone_name,
    "threshold_sec": TIME_SYNC_THRESHOLD_SEC,
    "steps": [],
  }

  def log(msg: str):
    if debug or TIME_SYNC_DEBUG_DEFAULT:
      print(f"[time_sync] {msg}")

  log(f"request tz={timezone_name} target_epoch={target_epoch} server_epoch={server_epoch} diff_sec={diff_sec}")

  # 1) timezone 링크 설정
  if timezone_name:
    if not os.path.exists(zoneinfo_path):
      result["ok"] = False
      result["message"] = f"zoneinfo not found: {zoneinfo_path}"
      log(result["message"])
      return result

    current_target = ""
    try:
      if os.path.exists(localtime_path) or os.path.islink(localtime_path):
        current_target = os.path.realpath(localtime_path)
    except Exception as e:
      log(f"failed to read current localtime link: {e}")

    if current_target == zoneinfo_path:
      result["steps"].append({"timezone": "already matched"})
      log(f"timezone already matched: {timezone_name}")
    else:
      try:
        if os.path.exists(localtime_path) or os.path.islink(localtime_path):
          subprocess.run(["sudo", "rm", "-f", localtime_path], check=True)
          result["steps"].append({"remove_localtime": localtime_path})
          log(f"removed existing localtime: {localtime_path}")

        subprocess.run(["sudo", "ln", "-s", zoneinfo_path, localtime_path], check=True)
        result["steps"].append({"set_timezone_link": zoneinfo_path})
        log(f"timezone set to: {timezone_name}")
      except subprocess.CalledProcessError as e:
        result["ok"] = False
        result["message"] = f"failed to set timezone: {e}"
        log(result["message"])
        return result

  # 2) timezone이 없었는지 확인
  no_timezone = False
  try:
    if os.path.getsize(localtime_path) == 0:
      no_timezone = True
  except (FileNotFoundError, OSError):
    no_timezone = True

  # 3) diff가 작고 timezone도 있으면 스킵
  if abs(diff_sec) <= TIME_SYNC_THRESHOLD_SEC and not no_timezone:
    log(f"skip date set: within threshold ({diff_sec}s) and timezone exists")
    result["message"] = "skip: time diff within threshold"
    return result

  # 4) 시간 세팅 시도
  # epoch는 UTC 기준이므로 UTC로 해석해서 넣는 것이 안전
  new_time_utc = datetime.datetime.utcfromtimestamp(target_epoch)
  formatted_time = new_time_utc.strftime("%Y-%m-%d %H:%M:%S")

  try:
    cmd = f"TZ=UTC sudo date -s '{formatted_time}'"
    result["steps"].append({"date_cmd": cmd})
    log(f"run: {cmd}")
    subprocess.run(cmd, shell=True, check=True)
    result["applied"] = True
    result["message"] = "time updated"
    result["server_epoch_after"] = int(time.time())
    log(f"time set success: {formatted_time} UTC")
    return result
  except subprocess.CalledProcessError as e:
    result["ok"] = False
    result["message"] = f"failed to set date: {e}"
    log(result["message"])
    return result


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
