# ruff: noqa: TID251
import time

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.server.services.assistant_tools import AssistantTools
from openpilot.selfdrive.carrot.server.tests.test_assistant_settings_http import load_routes


@pytest.mark.asyncio
async def test_tools_http_enforces_origin_grant_expiry_and_confirmation_flow(tmp_path):
  routes = load_routes('assistant_tools')
  starts = []
  def start(body, guard):
    guard()
    starts.append(body)
    return 'a' * 12
  service = AssistantTools(tmp_path / 'grants.json', lambda: True, start, lambda _: None, lambda _: None)
  app = web.Application()
  app[routes.STATE] = service
  routes.register(app)
  async with TestClient(TestServer(app)) as client:
    origin = str(client.make_url('/')).rstrip('/')
    headers = {'Origin': origin}
    assert (await client.post('/api/assistant/tools/enable', json={'confirmed': True})).status == 403
    assert (await client.post('/api/assistant/tools/enable', json={'confirmed': False}, headers=headers)).status == 400
    allowed = await (await client.post('/api/assistant/tools/enable', json={'confirmed': True}, headers=headers)).json()
    headers['Authorization'] = f'Bearer {allowed["token"]}'
    command = {'id': 'p' * 43, 'operation': 'prepare', 'action': 'git_pull', 'expires_at': time.time() * 1000 + 20000}
    route = '/api/assistant/tools/execute'
    assert (await client.post(route, json={'command': command}, headers={**headers, 'Origin': 'http://evil'})).status == 403
    assert (await client.post(route, data='{}', headers=headers)).status == 415
    assert (await client.post(route, json={'padding': 'x' * 5000}, headers=headers)).status == 413
    for expiry in [True, 0, time.time() * 1000 + 60000]:
      assert (await client.post(route, json={'command': {**command, 'expires_at': expiry}}, headers=headers)).status == 400
    prepared = await (await client.post(route, json={'command': command}, headers=headers)).json()
    assert prepared['status'] == 'prepared' and not starts
    commit = {**command, 'id': 'c' * 43, 'operation': 'commit', 'proposal_id': prepared['proposal_id']}
    result = await (await client.post(route, json={'command': commit}, headers=headers)).json()
    assert result['status'] == 'running' and starts == [{'action': 'git_pull'}]
    await client.post(route, json={'command': commit}, headers=headers)
    assert len(starts) == 1
    assert (await client.post('/api/assistant/tools/disable', json={}, headers=headers)).status == 200
    assert (await client.post(route, json={'command': commit}, headers=headers)).status == 403
