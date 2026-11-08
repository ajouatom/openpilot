import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.server.tests.test_assistant_settings_http import load_routes
from openpilot.selfdrive.carrot.server.services.assistant_connection import proof_id
from openpilot.selfdrive.carrot.server.services.assistant_settings import AssistantSettings


@pytest.mark.asyncio
async def test_existing_permission_rebind_http(tmp_path):
  routes = load_routes('assistant_connection')
  store = AssistantSettings(tmp_path / 'grants.json', dict, lambda _: 0, lambda *_: None, lambda: True, lambda *_: None)
  old, new = 'http://10.225.84.19:7000', 'http://192.168.0.178:7000'
  token = store.enable(old)
  routes.stores = lambda _: {'settings': (store, None)}
  app = web.Application()
  routes.register(app)
  async with TestClient(TestServer(app)) as client:
    headers = {'Host': '192.168.0.178:7000', 'Origin': new}
    body = {'previous_origin': old, 'nonce': 'n' * 43, 'identifiers': {'settings': proof_id(store._digest(token))}}
    path = '/api/assistant/connection/'
    assert (await client.post(path + 'proof', json=body)).status == 403
    assert (await client.post(path + 'proof', json=body, headers={**headers, 'Sec-Fetch-Site': 'cross-site'})).status == 403
    assert (await client.post(path + 'proof', data='{}', headers=headers)).status == 415
    assert (await client.post(path + 'proof', json={'padding': 'x' * 5000}, headers=headers)).status == 413
    response = await client.post(path + 'proof', json=body, headers=headers)
    assert response.status == 200
    assert set((await response.json())['proofs']) == {'settings'}
    assert response.headers['Cache-Control'] == 'no-store'
    move = {'previous_origin': old, 'credentials': {'settings': token}}
    assert (await client.post(path + 'move', json=move, headers=headers)).status == 200
    store.authorize(token, new)
    with pytest.raises(PermissionError):
      store.authorize(token, old)
    assert (await client.post(path + 'move', json={**move, 'credentials': {'drive': token}}, headers=headers)).status == 403
    assert (await client.post(path + 'bind', json={}, headers=headers)).status == 400
