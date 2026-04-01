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

from aiohttp import web, ClientSession, WSMsgType
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
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware import HARDWARE

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(BASE_DIR)

DEFAULT_SETTINGS_PATH = "/data/openpilot/selfdrive/carrot_settings.json"

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
    async with sess.post(WEBRTCD_URL, data=body, headers={"Content-Type": ct}) as resp:
      resp_body = await resp.read()
      # 그대로 전달
      out = web.Response(body=resp_body, status=resp.status)
      rct = resp.headers.get("Content-Type")
      if rct:
        out.headers["Content-Type"] = rct
      return out
  except Exception as e:
    return web.json_response({"ok": False, "error": str(e)}, status=502)

async def api_heartbeat_status(request: web.Request) -> web.Response:
  return web.json_response({"ok": True, "hb": request.app.get("hb_last")})

async def on_startup(app: web.Application):
  app["http"] = ClientSession()
  app["hb_last"] = {"ok": None, "msg": "not yet", "ts": 0}
  if HAS_PARAMS:
    app["hb_task"] = asyncio.create_task(heartbeat_loop(app))
  global _ws_carstate_task, _ws_carstate_lock
  _ws_carstate_lock = asyncio.Lock()
  _ws_carstate_task = asyncio.create_task(carstate_updater(app))
  
async def on_cleanup(app: web.Application):
  global _ws_carstate_task

  for ws in list(_ws_carstate_clients):
    try:
      await ws.close()
    except Exception:
      pass
  _ws_carstate_clients.clear()

  if _ws_carstate_task is not None:
    _ws_carstate_task.cancel()
    try:
      await _ws_carstate_task
    except asyncio.CancelledError:
      pass
    except Exception:
      traceback.print_exc()
    _ws_carstate_task = None
    
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

def _get_param_value(name: str, default: Any) -> Any:
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

async def handle_hudjs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(JS_DIR, "hud_card.js"))

async def handle_hudcss(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(CSS_DIR, "hud_card.css"))

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

def _filter_branch_list(branches: list[str]) -> list[str]:
  prefix = _get_branch_prefix()

  filtered = []
  for branch in branches:
    name = branch.strip()
    if not name:
      continue

    # local branch: c3-xxx / c4-xxx
    # remote branch: origin/c3-xxx, ajouatom/c3-xxx, etc.
    branch_name = name.split("/", 1)[-1] if "/" in name else name
    if branch_name.startswith(prefix):
      filtered.append(name)

  return sorted(set(filtered))

async def _run_tool_job(job: Dict[str, Any]) -> None:
  action = job["action"]
  body = job.get("payload") or {}
  repo_dir = "/data/openpilot"

  try:
    if action == "git_pull":
      _tool_job_progress(job, message="git pull", current=1, total=1)
      rc = await _tool_stream_exec(job, ["git", "pull"], cwd=repo_dir, timeout=180)
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
      if not branch:
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
      if branch.startswith("origin/"):
        local_branch = branch.replace("origin/", "", 1)
        script = (
          f"if git rev-parse --verify {shlex.quote(local_branch)} >/dev/null 2>&1; "
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

    if action == "git_branch_list":
      _tool_job_progress(job, message="fetch --all --prune", current=1, total=2)
      rc_fetch = await _tool_stream_exec(job, ["git", "fetch", "--all", "--prune"], cwd=repo_dir, timeout=180)
      if rc_fetch != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc_fetch))
        return

      _tool_job_progress(job, message="git branch -a", current=2, total=2)
      rc, out = await _tool_capture_exec(
        ["git", "branch", "-a", "--format=%(refname:short)"],
        cwd=repo_dir,
        timeout=30,
      )
      if out:
        _tool_job_append(job, "\n" + out + "\n")
      if rc != 0:
        _tool_job_finish(job, ok=False, result=_tool_result_from_log(job, rc))
        return

      rc_current, current_branch = await _tool_capture_exec(
        ["git", "branch", "--show-current"],
        cwd=repo_dir,
        timeout=15,
      )
      if rc_current != 0:
        current_branch = ""
      current_branch = (current_branch or "").strip()

      branches: list[str] = []
      for line in out.splitlines():
        line = line.strip()
        if not line or "->" in line:
          continue
        if line.startswith("remotes/"):
          line = line.replace("remotes/", "", 1)
        branches.append(line)

      branches = _filter_branch_list(branches)

      result = {
        "ok": True,
        "branches": branches,
        "current_branch": current_branch,
        "fetch": (job.get("log") or "").strip(),
        "device_type": HARDWARE.get_device_type(),
        "branch_prefix": _get_branch_prefix(),
      }
      _tool_job_finish(job, ok=True, result=result)
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

      allowed_top = {"git", "df", "free", "uptime", "scons"}
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
      if not branch:
        return web.json_response({"ok": False, "error": "missing branch"}, status=400)

      rc_fetch, out_fetch = run(["git", "fetch", "--all", "--prune"], cwd=REPO_DIR)
      if rc_fetch != 0:
        return web.json_response({"ok": False, "rc": rc_fetch, "out": out_fetch})

      is_remote = branch.startswith("origin/")
      try:
        if is_remote:
          local_branch = branch.replace("origin/", "", 1)
          rc_check, _ = run(["git", "rev-parse", "--verify", local_branch], cwd=REPO_DIR)
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

      rc, out = run(
        ["git", "branch", "-a", "--format=%(refname:short)"],
        cwd=REPO_DIR
      )
      if rc != 0:
        merged = (out0 + "\n\n" + out).strip()
        return web.json_response({"ok": False, "rc": rc, "out": merged})

      rc_current, out_current = run(["git", "branch", "--show-current"], cwd=REPO_DIR)
      current_branch = out_current.strip() if rc_current == 0 else ""

      branches: list[str] = []
      for line in out.splitlines():
        line = line.strip()
        if not line:
          continue
        if "->" in line:
          continue
        if line.startswith("remotes/"):
          line = line.replace("remotes/", "", 1)
        branches.append(line)

      branches = _filter_branch_list(branches)

      return web.json_response({
        "ok": True,
        "branches": branches,
        "current_branch": current_branch,
        "fetch": out0.strip(),
        "device_type": HARDWARE.get_device_type(),
        "branch_prefix": _get_branch_prefix(),
      })
    

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

      allowed_top = {"git", "df", "free", "uptime", "scons"}
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

async def ws_state(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20)
  await ws.prepare(request)

  while True:
    payload = {
      "ts": time.time(),
      "pid": os.getpid(),
      "has_params": HAS_PARAMS,
      "settings_path": _settings_cache["path"],
      "settings_exists": os.path.exists(_settings_cache["path"]),
    }
    try:
      await ws.send_str(json.dumps(payload))
    except Exception:
      break
    await asyncio.sleep(2.0)  # 폰에서 부담 줄이려고 2초

  try:
    await ws.close()
  except Exception:
    pass
  return ws

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
  ws = web.WebSocketResponse(heartbeat=20)
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

_ws_carstate_task = None
_ws_carstate_payload_json = "{}"
_ws_carstate_lock = None
_ws_carstate_clients = set()
async def carstate_updater(app: web.Application):
  global _ws_carstate_payload_json, _ws_carstate_lock

  sm = messaging.SubMaster([
    'carState',
    'carControl',
    'deviceState',
    'longitudinalPlan',
    'carrotMan',
    'peripheralState',
  ])

  params = Params() if HAS_PARAMS else None

  last_toggle_t = 0.0
  show_volt = False

  last_tf_gap_read_t = 0.0
  cached_tf_gap = None

  while True:
    try:
      sm.update(0)
      now = time.time()

      if now - last_toggle_t > 3.2:
        last_toggle_t = now
        show_volt = not show_volt

      # Params는 매 루프마다 읽지 말고 1초에 1번만
      if params is not None and (now - last_tf_gap_read_t > 1.0):
        last_tf_gap_read_t = now
        try:
          cached_tf_gap = int(params.get_int("LongitudinalPersonality") or 0) + 1
        except Exception:
          cached_tf_gap = None

      v_ego = None
      v_cruise = None
      gear = None
      temp = None
      gps_ok = None

      cpu_temp_c = None
      mem_pct = None
      disk_pct = None
      volt_v = None
      drive_mode_obj = None
      temp_speed = None

      if sm.alive['carState'] and sm.alive['carControl']:
        CS = sm['carState']
        CC = sm['carControl']
        CM = sm['carrotMan']
        lp = sm['longitudinalPlan']
        ps = sm['peripheralState']
        ds = sm['deviceState']

        v_ego = CS.vEgoCluster
        v_cruise = CS.vCruiseCluster

        gs = CS.gearShifter
        step = CS.gearStep
        if gs == GearShifter.unknown:
          gear = "U"
        elif gs == GearShifter.park:
          gear = "P"
        elif gs == GearShifter.drive:
          gear = str(step) if step > 0 else "D"
        elif gs == GearShifter.neutral:
          gear = "N"
        elif gs == GearShifter.reverse:
          gear = "R"
        elif gs == GearShifter.low:
          gear = "L"
        elif gs == GearShifter.sport:
          gear = "S"
        else:
          gear = "X"

        apply_speed = CM.desiredSpeed
        apply_source = CM.desiredSource
        temp_speed = {
          "speed": apply_speed,
          "source": apply_source if v_cruise is not None and apply_speed >= v_cruise else "",
          "is_decel": True if v_cruise is not None and apply_speed < v_cruise else False,
        }

        drive_mode = lp.myDrivingMode
        if drive_mode == 1:
          drive_mode_obj = {"name": "Eco", "kind": "eco"}
        elif drive_mode == 2:
          drive_mode_obj = {"name": "Safe", "kind": "safe"}
        elif drive_mode == 4:
          drive_mode_obj = {"name": "Sport", "kind": "sport"}
        else:
          drive_mode_obj = {"name": "Normal", "kind": "normal"}

        gps_ok = True

        if sm.alive['deviceState']:
          c = ds.cpuTempC
          cpu_temp_c = np.mean(c)
        else:
          cpu_temp_c = 0
        mem_pct = ds.memoryUsagePercent
        free_pct = ds.freeSpacePercent
        if math.isfinite(free_pct):
          disk_pct = 100.0 - free_pct

        volt_v = ps.voltage / 1000.0 if ps.voltage is not None else None

      payload = {
        "ts": now,
        "vEgo": v_ego,
        "vSetKph": v_cruise,
        "gear": gear,
        "gpsOk": gps_ok,

        "cpuTempC": cpu_temp_c,
        "memPct": mem_pct,
        "diskPct": (volt_v if show_volt else disk_pct),
        "diskLabel": ("VOLT" if show_volt else "DISK"),

        "tfGap": cached_tf_gap,
        "tfBars": cached_tf_gap,
        "driveMode": drive_mode_obj,

        "tlight": "off",
        "redDot": False,
        "temp": temp_speed,
        "speedLimitKph": None,
        "speedLimitOver": False,
        "apm": " ",
      }

      payload_json = json.dumps(payload, separators=(",", ":"))

      async with _ws_carstate_lock:
        _ws_carstate_payload_json = payload_json

    except asyncio.CancelledError:
      raise
    except Exception:
      traceback.print_exc()

    await asyncio.sleep(0.2)
    
async def ws_carstate(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20)
  await ws.prepare(request)

  _ws_carstate_clients.add(ws)

  try:
    while True:
      async with _ws_carstate_lock:
        payload_json = _ws_carstate_payload_json

      try:
        await ws.send_str(payload_json)
      except (asyncio.CancelledError, GeneratorExit):
        raise
      except (ConnectionResetError, BrokenPipeError, web.HTTPException):
        break
      except Exception as e:
        if isinstance(e, (aiohttp.client_exceptions.ClientConnectionResetError,)):
          break
        if "Cannot write to closing transport" in str(e):
          break
        break

      await asyncio.sleep(0.1)

  except asyncio.CancelledError:
    raise
  except Exception:
    traceback.print_exc()

  try:
    _ws_carstate_clients.discard(ws)
    await ws.close()
  except Exception:
    pass

  return ws

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
