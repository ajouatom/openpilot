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
from typing import Dict, Any, Tuple, Optional, List

from aiohttp import web, ClientSession
from cereal import messaging
from opendbc.car import structs
import shlex

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

DEFAULT_SETTINGS_PATH = "/data/openpilot/selfdrive/carrot_settings.json"

WEB_DIR = os.path.join(BASE_DIR, "web")

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

async def on_startup(app: web.Application):
  app["http"] = ClientSession()

async def on_cleanup(app: web.Application):
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
    g = p.get("group", "UNGROUPED")
    groups.setdefault(g, []).append(p)
    n = p.get("name")
    if n:
      by_name[n] = p

  # group list with egroup guess
  for g, items in groups.items():
    egroup = None
    for it in items:
      if it.get("egroup"):
        egroup = it.get("egroup")
        break
    groups_list.append({"group": g, "egroup": egroup, "count": len(items)})

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

async def handle_appjs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(WEB_DIR, "app.js"))

async def handle_hudjs(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(WEB_DIR, "hud_card.js"))

async def handle_hudcss(request: web.Request) -> web.Response:
  return web.FileResponse(os.path.join(WEB_DIR, "hud_card.css"))

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

async def api_tools(request: web.Request) -> web.Response:
  try:
    body = await request.json()
  except Exception:
    return web.json_response({"ok": False, "error": "invalid json"}, status=400)

  action = body.get("action")
  if not action:
    return web.json_response({"ok": False, "error": "missing action"}, status=400)

  # 최소 보안: 사설대역만 허용 (권장)
  ip = request.remote or ""
  if not (ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172.16.") or ip.startswith("172.17.") or ip in ("127.0.0.1", "::1")):
    return web.json_response({"ok": False, "error": "forbidden"}, status=403)

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
      rc, out = run(["git", "checkout", "-f", branch], cwd=REPO_DIR)
      return web.json_response({"ok": rc == 0, "rc": rc, "out": out})

    if action == "git_branch_list":
      rc, out = run(
        ["git", "branch", "-a", "--format=%(refname:short)"],
        cwd=REPO_DIR
      )
      if rc != 0:
        return web.json_response({"ok": False, "rc": rc, "out": out})

      branches = []
      for line in out.splitlines():
        line = line.strip()
        if not line:
          continue
        if line.startswith("remotes/"):
          line = line.replace("remotes/", "", 1)
        branches.append(line)

      # 중복 제거 + 정렬
      branches = sorted(set(branches))
      return web.json_response({"ok": True, "branches": branches})


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
        for fn in glob.glob(os.path.join(pth, "*")):
          try:
            os.remove(fn)
            deleted += 1
          except Exception:
            pass
      return web.json_response({"ok": True, "out": f"deleted files: {deleted}"})


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

    if action == "backup_settings":
      if not HAS_PARAMS or ParamKeyType is None:
        return web.json_response({"ok": False, "error": "Params/ParamKeyType not available"}, status=500)

      # 사설대역 제한
      ip = request.remote or ""
      if not (ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172.16.") or ip.startswith("172.17.") or ip in ("127.0.0.1", "::1")):
        return web.json_response({"ok": False, "error": "forbidden"}, status=403)

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

      allowed_top = {"git", "df", "free", "uptime"}
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

async def ws_carstate(request: web.Request) -> web.WebSocketResponse:
  ws = web.WebSocketResponse(heartbeat=20)
  await ws.prepare(request)

  sm = messaging.SubMaster(['carState', 'carControl', 'deviceState', 'longitudinalPlan', 'carrotMan', 'peripheralState'])

  # for gap/driving mode (same as your drawHud: Params reads)
  params = Params() if HAS_PARAMS else None
  last_toggle_t = 0.0
  show_volt = False

  try:
    while True:
      sm.update(0)  # non-blocking
      now = time.time()

      # toggle DISK/VOLT display every ~3s (like disp_timer)
      if now - last_toggle_t > 3.2:
        last_toggle_t = now
        show_volt = not show_volt

      v_ego = None
      v_cruise = None
      gear = None
      temp = None
      gps_ok = None

      cpu_temp_c = None
      mem_pct = None
      disk_pct = None
      volt_v = None
      tf_gap = None
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
        temp_speed = { "speed": apply_speed, "source": apply_source if apply_speed >= v_cruise else "", "is_decel": True if apply_speed < v_cruise else False}
        drive_mode = lp.myDrivingMode
        if drive_mode == 1:
          drive_mode_obj = {"name": "연비", "kind": "eco"}
        elif drive_mode == 2:
          drive_mode_obj = {"name": "안전", "kind": "safe"}
        elif drive_mode == 4:
          drive_mode_obj = {"name": "고속", "kind": "sport"}
        else:
          drive_mode_obj = {"name": "일반", "kind": "normal"}


        gps_ok = True

        # deviceState
        ds = sm['deviceState']
        # cpuTempC can be list; use max
        c = ds.cpuTempC
        if c is not None:
          if isinstance(c, (list, tuple)) and len(c) > 0:
            cpu_temp_c = float(max(c))

        mem_pct = ds.memoryUsagePercent
        free_pct = ds.freeSpacePercent
        if math.isfinite(free_pct):
          disk_pct = 100.0 - free_pct

        volt_v = ps.voltage

        # gap/driving mode from Params (same as your C++)
        tf_gap = int(params.get_int("LongitudinalPersonality") or 0) + 1


      payload = {
        "ts": now,
        "vEgo": v_ego,              # m/s
        "vSetKph": v_cruise,
        "gear": gear,
        "gpsOk": gps_ok,

        "cpuTempC": cpu_temp_c,
        "memPct": mem_pct,
        "diskPct": (volt_v if show_volt else disk_pct),
        "diskLabel": ("VOLT" if show_volt else "DISK"),

        "tfGap": tf_gap,
        "tfBars": tf_gap,
        "driveMode": drive_mode_obj,

        # placeholders (fill later from your sources)
        "tlight": "off",
        "redDot": False,
        "temp": temp_speed,
        "speedLimitKph": None,
        "speedLimitOver": False,
        "apm": " ",
      }

      await ws.send_str(json.dumps(payload))
      await asyncio.sleep(0.1)  # 10Hz
  except Exception:
    #traceback.print_exc()
    pass

  try:
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

  # 최소 보안: 사설대역만 허용 (api_tools와 동일하게)
  ip = request.remote or ""
  if not (ip.startswith("192.168.") or ip.startswith("10.") or ip.startswith("172.16.") or ip.startswith("172.17.") or ip in ("127.0.0.1", "::1")):
    return web.json_response({"ok": False, "error": "forbidden"}, status=403)

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


def make_app() -> web.Application:
  app = web.Application(middlewares=[log_mw])
  app.on_startup.append(on_startup)
  app.on_cleanup.append(on_cleanup)
  
  # static-like routes
  app.router.add_get("/", handle_index)
  app.router.add_get("/app.js", handle_appjs)
  app.router.add_get("/hud_card.js", handle_hudjs)
  app.router.add_get("/hud_card.css", handle_hudcss)

  # api
  app.router.add_get("/api/settings", api_settings)
  app.router.add_get("/api/params_bulk", api_params_bulk)
  app.router.add_post("/api/param_set", api_param_set)
  app.router.add_get("/api/cars", api_cars)
  app.router.add_post("/api/reboot", api_reboot)
  app.router.add_post("/api/tools", api_tools)
  app.router.add_post("/stream", proxy_stream)
  # ws
  app.router.add_get("/ws/state", ws_state)

  app.router.add_get("/ws/carstate", ws_carstate)
  app.router.add_get("/download/tmux.log", handle_download_tmux)

  app.router.add_get("/download/params_backup.json", handle_download_params_backup)
  app.router.add_post("/api/params_restore", api_params_restore)

  app.router.add_static("/", str(WEB_DIR), show_index=True)
  return app


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--host", type=str, default="0.0.0.0")
  parser.add_argument("--port", type=int, default=7000)
  parser.add_argument("--settings", type=str, default=DEFAULT_SETTINGS_PATH,
                      help="path to carrot_settings.json")
  args = parser.parse_args()

  _settings_cache["path"] = args.settings

  if not os.path.isdir(WEB_DIR):
    raise RuntimeError(f"web dir not found: {WEB_DIR}")
  if not os.path.exists(_settings_cache["path"]):
    print(f"[WARN] settings file not found: {_settings_cache['path']}")

  import logging
  logging.getLogger("aiohttp.access").setLevel(logging.WARNING)
  web.run_app(make_app(), host=args.host, port=args.port)


if __name__ == "__main__":
  main()
