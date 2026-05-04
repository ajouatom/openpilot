import asyncio

from aiohttp import ClientSession, ClientTimeout, web

from ..config import WEBRTCD_URL


async def proxy_stream(request: web.Request) -> web.StreamResponse:
  body = await request.read()
  ct = request.headers.get("Content-Type", "application/json")

  sess: ClientSession = request.app["http"]

  try:
    async with sess.post(WEBRTCD_URL, data=body, headers={"Content-Type": ct},
                         timeout=ClientTimeout(total=5)) as resp:
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


def register(app: web.Application) -> None:
  app.router.add_post("/stream", proxy_stream)
