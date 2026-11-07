# ruff: noqa: TID251
# HTTP expiry is a cross-host Unix timestamp, not a local monotonic deadline.
import importlib.util
import sys
import tempfile
import time
import types
import pytest
from pathlib import Path
import pytest_asyncio

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from openpilot.selfdrive.carrot.server.services.assistant_settings import AssistantSettings


def load_routes(feature='assistant_settings'):
  # Import the real feature without starting unrelated native msgq features on Windows.
  package = 'openpilot.selfdrive.carrot.server.features'
  directory = Path(__file__).resolve().parents[1] / 'features'
  namespace = types.ModuleType(package)
  namespace.__path__ = [str(directory)]
  spec = importlib.util.spec_from_file_location(package + '.' + feature, directory / (feature + '.py'))
  module = importlib.util.module_from_spec(spec)
  with pytest.MonkeyPatch.context() as patch:
    patch.setitem(sys.modules, package, namespace)
    spec.loader.exec_module(module)
  return module


@pytest.mark.asyncio
class TestAssistantSettingsHttp:
  @pytest_asyncio.fixture(autouse=True)
  async def setup(self):
    self.routes = load_routes()
    self.directory = tempfile.TemporaryDirectory()
    self.value = 50
    service = AssistantSettings(
      Path(self.directory.name) / 'grants.json',
      lambda: {'ShowCustomBrightness': {'min': 0, 'max': 100, 'unit': 1}},
      lambda _: self.value,
      lambda _, value: setattr(self, 'value', value),
      lambda: True,
      lambda *args: None,
    )
    app = web.Application()
    app[self.routes.STATE] = service
    self.routes.register(app)
    self.client = TestClient(TestServer(app))
    await self.client.start_server()
    self.origin = str(self.client.make_url('/')).rstrip('/')
    self.token = service.enable(self.origin)

    yield
    await self.client.close()
    self.directory.cleanup()

  def headers(self, **extra):
    return {'Origin': self.origin, 'Authorization': f'Bearer {self.token}', **extra}

  async def test_origin_json_and_permission_are_mandatory(self):
    route = '/api/assistant/settings/execute'
    assert (await self.client.post(route, json={})).status == 403
    assert (await self.client.post(route, json={}, headers=self.headers(Origin='http://evil.test'))).status == 403
    assert (await self.client.post(route, data='{}', headers=self.headers())).status == 415
    command = {'id': 'r' * 43, 'operation': 'read', 'name': 'ShowCustomBrightness', 'expires_at': time.time() * 1000 + 20000}
    response = await self.client.post(route, json={'command': command}, headers=self.headers(Authorization='Bearer wrong'))
    assert response.status == 403
    assert (await self.client.post('/api/assistant/settings/enable', json={'confirmed': False}, headers=self.headers())).status == 400

  async def test_stale_and_oversized_commands_never_apply(self):
    command = {'id': 'r' * 43, 'operation': 'read', 'name': 'ShowCustomBrightness', 'expires_at': time.time() * 1000 - 1}
    response = await self.client.post('/api/assistant/settings/execute', json={'command': command}, headers=self.headers())
    assert response.status == 400
    response = await self.client.post('/api/assistant/settings/execute', json={'padding': 'x' * 5000}, headers=self.headers())
    assert response.status == 413
    assert self.value == 50

  async def test_http_prepare_commit_readback_and_revoke(self):
    async def execute(identity, operation, **fields):
      command = {'id': identity * 43, 'operation': operation, 'expires_at': time.time() * 1000 + 20000, **fields}
      return await self.client.post('/api/assistant/settings/execute', json={'command': command}, headers=self.headers())

    response = await execute('p', 'prepare', name='ShowCustomBrightness', mode='percent', value=10)
    proposed = await response.json()
    assert proposed['proposed'] == 55
    assert self.value == 50
    result = await (await execute('c', 'commit', proposal_id=proposed['proposal_id'])).json()
    assert result['status'] == 'applied'
    assert result['current'] == 55
    assert (await self.client.post('/api/assistant/settings/disable', json={}, headers=self.headers())).status == 200
    assert (await execute('r', 'read', name='ShowCustomBrightness')).status == 403
