import asyncio
import json
import socket
import ssl
import time
import urllib.error
import urllib.request

from aiohttp import web

from .params import HAS_PARAMS, Params


def get_local_ip() -> str:
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


def register_my_ip_sync(params: "Params") -> tuple[bool, str]:
  """
  기존 carrot_man.py의 register_my_ip()를 그대로 옮긴 버전 (동기)
  """
  try:
    token = "12345678"
    local_ip = get_local_ip()
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
  aiohttp startup에서 create_task로 돌릴 백그라운드 루프.
  - 이벤트 루프 블로킹 방지 위해 to_thread 사용
  """
  if not HAS_PARAMS:
    app["hb_last"] = {"ok": False, "msg": "Params not available"}
    return

  params = Params()
  interval_s = 30.0  # 기존: frame%(20*30) = 30초
  while True:
    try:
      ok, msg = await asyncio.to_thread(register_my_ip_sync, params)
      app["hb_last"] = {
        "ok": bool(ok),
        "msg": str(msg)[:800],
        "ts": time.time(),
        "local_ip": get_local_ip(),
      }
    except asyncio.CancelledError:
      break
    except Exception as e:
      app["hb_last"] = {"ok": False, "msg": f"Exception: {e}", "ts": time.time()}
    await asyncio.sleep(interval_s)
