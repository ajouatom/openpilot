# ruff: noqa: TID251 -- HTTP uses cross-host Unix expiry.
import time

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.server.services.assistant_drive import AssistantDrive
from openpilot.selfdrive.carrot.server.tests.test_assistant_settings_http import load_routes
from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json


@pytest.mark.asyncio
async def test_http_drive_permission_expiry_delivery_and_revocation(tmp_path):
  routes = load_routes('assistant_drive')
  parked = [False]
  service = AssistantDrive(tmp_path / 'grants.json', lambda: parked[0], tmp_path / 'runtime')
  app = web.Application()
  app[routes.STATE] = service
  routes.register(app)
  async with TestClient(TestServer(app)) as client:
    origin = str(client.make_url('/')).rstrip('/')
    headers = {'Origin': origin}
    base = '/api/assistant/drive/'
    assert (await client.post(base + 'enable', json={'confirmed': True})).status == 403
    assert (await client.post(base + 'enable', json={'confirmed': True}, headers=headers)).status == 400
    parked[0] = True
    assert (await client.post(base + 'enable', json={'confirmed': False}, headers=headers)).status == 400
    response = await client.post(base + 'enable', json={'confirmed': True}, headers=headers)
    assert response.status == 200
    headers['Authorization'] = 'Bearer ' + (await response.json())['token']
    command = {'id': 'a' * 43, 'operation': 'execute', 'action': 'speed_set', 'value': 100, 'expires_at': time.time() * 1000 + 4000}
    assert (await client.post(base + 'execute', json={'command': command}, headers={**headers, 'Origin': 'http://evil.test'})).status == 403
    assert (await client.post(base + 'execute', json={'command': command}, headers={**headers, 'Authorization': 'Bearer wrong'})).status == 403
    assert (await client.post(base + 'execute', json={'command': {**command, 'expires_at': 1}}, headers=headers)).status == 400
    assert (await client.post(base + 'execute', json={'padding': 'x' * 3000}, headers=headers)).status == 413
    assert not (service.root / 'speed/request.json').exists()
    parked[0] = False  # Initial grant is parked-only; controller decides subsequent requests.
    response = await client.post(base + 'execute', json={'command': command}, headers=headers)
    assert response.status == 200 and (await response.json())['status'] == 'waiting'
    sent = read_json(service.root / 'speed/request.json')
    atomic_json(service.root / 'speed/result.json', {'id': command['id'], 'status': 'rejected', 'reason': 'cruise_inactive', 'time': time.monotonic()})
    result = await (await client.post(base + 'execute', json={'command': command}, headers=headers)).json()
    assert result['status'] == 'rejected'
    assert read_json(service.root / 'speed/request.json') == sent
    assert (await client.post(base + 'disable', json={}, headers=headers)).status == 200
    assert (await client.post(base + 'execute', json={'command': command}, headers=headers)).status == 403
