"""Serve ONNX diagnostics through Carrot Web; inference stays in xiaoge_data."""

from pathlib import Path
from urllib.parse import urlsplit

from aiohttp import ClientConnectorError, ClientError, ClientTimeout, ConnectionTimeoutError, web


XIAOGE_URL = "http://127.0.0.1:8082"
PAGE_PATH = Path(__file__).resolve().parents[2] / "xiaoge" / "v_asm_web.html"
MAX_REQUEST_BYTES = 65536
MAX_RESPONSE_BYTES = 4 * 1024 * 1024
PROXY_TIMEOUT = ClientTimeout(total=5, sock_connect=1)


async def index(request: web.Request) -> web.StreamResponse:
  if not request.path.endswith("/"):
    raise web.HTTPFound(location=str(request.rel_url.with_path("/xiaoge/", keep_query=True)))
  return web.FileResponse(PAGE_PATH, headers={"Cache-Control": "no-store"})


def unavailable(code: str, status: int) -> web.Response:
  return web.json_response({"error": code, "code": code}, status=status, headers={"Cache-Control": "no-store"})


async def proxy_api(request: web.Request) -> web.Response:
  # Register only the existing diagnostic operations. Neither a URL nor a host
  # supplied by the caller can turn this into a general-purpose proxy.
  endpoint = request.path.rsplit("/", 1)[-1]
  body = None
  headers = {}
  if request.method in ("POST", "DELETE"):
    origin = request.headers.get("Origin")
    if origin and urlsplit(origin).netloc != request.host:
      raise web.HTTPForbidden(text="cross-origin diagnostic changes are not allowed")
  if request.method == "POST":
    if request.content_type != "application/json":
      raise web.HTTPUnsupportedMediaType(text="expected application/json")
    body = await request.clone(client_max_size=MAX_REQUEST_BYTES + 1).read()
    if len(body) > MAX_REQUEST_BYTES:
      raise web.HTTPRequestEntityTooLarge(max_size=MAX_REQUEST_BYTES, actual_size=len(body))
    headers["Content-Type"] = "application/json"

  try:
    async with request.app["http"].request(
      request.method, f"{XIAOGE_URL}/api/{endpoint}", params=request.query,
      data=body, headers=headers, timeout=PROXY_TIMEOUT, allow_redirects=False,
    ) as response:
      if 300 <= response.status < 400:
        return unavailable("vision_bad_response", 502)
      chunks = bytearray()
      async for chunk in response.content.iter_chunked(65536):
        chunks.extend(chunk)
        if len(chunks) > MAX_RESPONSE_BYTES:
          return unavailable("vision_bad_response", 502)
      return web.Response(body=bytes(chunks), status=response.status, headers={
        "Content-Type": response.headers.get("Content-Type", "application/octet-stream"),
        "Cache-Control": "no-store",
      })
  except (ClientConnectorError, ConnectionTimeoutError):
    return unavailable("vision_unavailable", 503)
  except TimeoutError:
    return unavailable("vision_timeout", 504)
  except ClientError:
    return unavailable("vision_unavailable", 503)


def register(app: web.Application) -> None:
  app.router.add_get("/xiaoge", index)
  app.router.add_get("/xiaoge/", index)
  for endpoint, methods in (
    ("status", ("GET",)), ("snapshot", ("GET",)),
    ("config", ("GET", "POST", "DELETE")), ("settings", ("POST",)),
  ):
    for method in methods:
      app.router.add_route(method, f"/xiaoge/api/{endpoint}", proxy_api)
