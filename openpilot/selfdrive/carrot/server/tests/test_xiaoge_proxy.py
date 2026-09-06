import asyncio
from contextlib import asynccontextmanager
import importlib.util
from pathlib import Path

from aiohttp import ClientSession, ClientTimeout, web
from aiohttp.test_utils import TestClient, TestServer


# This feature has no device dependencies. Test its real HTTP routes without
# starting the application's camera, messaging, or background service workers.
spec = importlib.util.spec_from_file_location("xiaoge_proxy", Path(__file__).parents[1] / "features/xiaoge.py")
xiaoge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(xiaoge)


@asynccontextmanager
async def proxy_client():
  app = web.Application()
  async with ClientSession() as session:
    app["http"] = session
    xiaoge.register(app)
    async with TestClient(TestServer(app)) as client:
      yield client


def test_page_opens_without_inference_and_preserves_language_on_redirect():
  async def check():
    async with proxy_client() as client:
      response = await client.get("/xiaoge?lang=ko", allow_redirects=False)
      assert response.status == 302
      assert response.headers["Location"] == "/xiaoge/?lang=ko"
      response = await client.get("/xiaoge/?lang=ko")
      assert response.status == 200
      assert response.headers["Cache-Control"] == "no-store"
      assert "ONNX 차선·BSD 진단" in await response.text()
  asyncio.run(check())


def test_proxy_relays_status_snapshots_and_configuration_only_to_fixed_service(monkeypatch):
  async def check():
    received = []
    async def upstream(request):
      received.append((request.method, request.path, dict(request.query), await request.read()))
      if request.path == "/api/snapshot":
        return web.Response(body=b"jpeg-bytes", content_type="image/jpeg", headers={"Set-Cookie": "private=upstream"})
      if request.path == "/api/settings":
        return web.json_response({"error": "invalid threshold"}, status=400)
      return web.json_response({"ok": True})
    upstream_app = web.Application()
    upstream_app.router.add_route("*", "/{path:.*}", upstream)
    async with TestServer(upstream_app) as upstream_server:
      monkeypatch.setattr(xiaoge, "XIAOGE_URL", str(upstream_server.make_url("")).rstrip("/"))
      async with proxy_client() as client:
        response = await client.get("/xiaoge/api/status?url=http://example.invalid/")
        assert await response.json() == {"ok": True}
        response = await client.get("/xiaoge/api/snapshot?stream=road&t=123")
        assert await response.read() == b"jpeg-bytes"
        assert response.content_type == "image/jpeg"
        assert response.headers["Cache-Control"] == "no-store"
        assert "Set-Cookie" not in response.headers
        response = await client.post("/xiaoge/api/config", json={"poly_left": [[1, 2]]})
        assert response.status == 200
        response = await client.delete("/xiaoge/api/config")
        assert response.status == 200
        response = await client.post("/xiaoge/api/settings", json={"threshold": -1})
        assert response.status == 400
        assert await response.json() == {"error": "invalid threshold"}
    assert [(method, path) for method, path, _, _ in received] == [
      ("GET", "/api/status"), ("GET", "/api/snapshot"), ("POST", "/api/config"),
      ("DELETE", "/api/config"), ("POST", "/api/settings"),
    ]
    assert received[1][2] == {"stream": "road", "t": "123"}
    assert received[2][3] == b'{"poly_left": [[1, 2]]}'
  asyncio.run(check())


def test_proxy_rejects_unregistered_operations_cross_origin_and_oversize_posts(monkeypatch):
  async def check():
    async with proxy_client() as client:
      def unexpected_forward(*args, **kwargs):
        raise AssertionError("invalid operations must not reach the vision service")
      monkeypatch.setattr(client.app["http"], "request", unexpected_forward)
      for method, path, kwargs, expected in (
        ("GET", "/xiaoge/api/terminal", {}, 404),
        ("POST", "/xiaoge/api/status", {"json": {}}, 405),
        ("DELETE", "/xiaoge/api/settings", {}, 405),
        ("POST", "/xiaoge/api/config", {"data": "{}"}, 415),
        ("POST", "/xiaoge/api/config", {"json": {}, "headers": {"Origin": "http://example.invalid"}}, 403),
        ("DELETE", "/xiaoge/api/config", {"headers": {"Origin": "null"}}, 403),
        ("POST", "/xiaoge/api/config", {"data": b" " * 65537, "headers": {"Content-Type": "application/json"}}, 413),
      ):
        response = await client.request(method, path, **kwargs)
        assert response.status == expected, (method, path, await response.text())
  asyncio.run(check())


def test_stopped_service_returns_actionable_error_and_page_stays_available(monkeypatch):
  async def check():
    server = TestServer(web.Application())
    await server.start_server()
    monkeypatch.setattr(xiaoge, "XIAOGE_URL", str(server.make_url("")).rstrip("/"))
    await server.close()
    async with proxy_client() as client:
      response = await client.get("/xiaoge/api/status")
      assert response.status == 503
      assert (await response.json())["code"] == "vision_unavailable"
      assert (await client.get("/xiaoge/")).status == 200
  asyncio.run(check())


def test_proxy_times_out_and_does_not_follow_upstream_redirects(monkeypatch):
  async def check():
    async def upstream(request):
      if request.path == "/api/status":
        await asyncio.sleep(0.1)
      else:
        raise web.HTTPFound("http://example.invalid/")
      return web.json_response({})
    app = web.Application()
    app.router.add_route("*", "/api/{endpoint}", upstream)
    async with TestServer(app) as server:
      monkeypatch.setattr(xiaoge, "XIAOGE_URL", str(server.make_url("")).rstrip("/"))
      monkeypatch.setattr(xiaoge, "PROXY_TIMEOUT", ClientTimeout(total=0.02))
      async with proxy_client() as client:
        response = await client.get("/xiaoge/api/status")
        assert response.status == 504
        assert (await response.json())["code"] == "vision_timeout"
        monkeypatch.setattr(xiaoge, "PROXY_TIMEOUT", ClientTimeout(total=5))
        response = await client.get("/xiaoge/api/config")
        assert response.status == 502
        assert (await response.json())["code"] == "vision_bad_response"
  asyncio.run(check())


def test_proxy_limits_upstream_response_size(monkeypatch):
  async def check():
    async def upstream(request):
      return web.Response(body=b"x" * 33)
    app = web.Application()
    app.router.add_get("/api/snapshot", upstream)
    async with TestServer(app) as server:
      monkeypatch.setattr(xiaoge, "XIAOGE_URL", str(server.make_url("")).rstrip("/"))
      monkeypatch.setattr(xiaoge, "MAX_RESPONSE_BYTES", 32)
      async with proxy_client() as client:
        response = await client.get("/xiaoge/api/snapshot")
        assert response.status == 502
        assert (await response.json())["code"] == "vision_bad_response"
  asyncio.run(check())
