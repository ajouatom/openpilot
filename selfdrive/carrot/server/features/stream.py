import asyncio
import json
import time

from aiohttp import ClientSession, ClientTimeout, web

from ..config import WEBRTCD_URL
from ..services.vision_diag import record_stream_proxy_event


def _request_summary(body: bytes) -> dict:
  try:
    payload = json.loads(body.decode("utf-8", errors="replace"))
    return {
      "cameras": payload.get("cameras"),
      "bridge_services_in": payload.get("bridge_services_in"),
      "bridge_services_out": payload.get("bridge_services_out"),
      "sdp_bytes": len(str(payload.get("sdp") or "")),
    }
  except Exception:
    return {}


async def proxy_stream(request: web.Request) -> web.StreamResponse:
  body = await request.read()
  ct = request.headers.get("Content-Type", "application/json")
  started_at = time.monotonic()
  base_event = {
    "client": request.remote or "",
    "content_type": ct,
    "request_bytes": len(body),
    **_request_summary(body),
  }

  sess: ClientSession = request.app["http"]

  try:
    async with sess.post(WEBRTCD_URL, data=body, headers={"Content-Type": ct},
                         timeout=ClientTimeout(total=5)) as resp:
      resp_body = await resp.read()
      out = web.Response(body=resp_body, status=resp.status)
      rct = resp.headers.get("Content-Type")
      if rct:
        out.headers["Content-Type"] = rct
      record_stream_proxy_event({
        **base_event,
        "ok": 200 <= resp.status < 300,
        "status": resp.status,
        "response_bytes": len(resp_body),
        "elapsed_ms": round((time.monotonic() - started_at) * 1000, 1),
      })
      return out
  except asyncio.TimeoutError:
    record_stream_proxy_event({
      **base_event,
      "ok": False,
      "status": 504,
      "error": "webrtcd timeout",
      "elapsed_ms": round((time.monotonic() - started_at) * 1000, 1),
    })
    return web.json_response({"ok": False, "error": "webrtcd timeout"}, status=504)
  except Exception as e:
    record_stream_proxy_event({
      **base_event,
      "ok": False,
      "status": 502,
      "error": str(e),
      "elapsed_ms": round((time.monotonic() - started_at) * 1000, 1),
    })
    return web.json_response({"ok": False, "error": str(e)}, status=502)


def register(app: web.Application) -> None:
  app.router.add_post("/stream", proxy_stream)
