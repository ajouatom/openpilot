# ruff: noqa: TID251 -- HTTP uses cross-host Unix expiry.
import asyncio
import time
from types import SimpleNamespace

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
    started = time.perf_counter()
    response = await client.post(base + 'execute', json={'command': command}, headers=headers)
    assert response.status == 200 and (await response.json())['status'] == 'waiting'
    print(f'mock device HTTP round trip (waiting, ms): {(time.perf_counter() - started) * 1000:.1f}')
    sent = read_json(service.root / 'speed/request.json')
    atomic_json(service.root / 'speed/result.json', {'id': command['id'], 'status': 'rejected', 'reason': 'cruise_inactive', 'time': time.monotonic()})
    started = time.perf_counter()
    result = await (await client.post(base + 'execute', json={'command': command}, headers=headers)).json()
    print(f'mock device HTTP round trip (result ready, ms): {(time.perf_counter() - started) * 1000:.1f}')
    assert result['status'] == 'rejected'
    assert read_json(service.root / 'speed/request.json') == sent
    assert (await client.post(base + 'speeds', json={}, headers={**headers, 'Authorization': 'Bearer invalid'})).status == 403
    unavailable = await (await client.post(base + 'speeds', json={}, headers=headers)).json()
    assert unavailable == {'ok': True, 'status': 'unavailable', 'reason': 'no_fresh_data'}
    assert (await client.post(base + 'disable', json={}, headers=headers)).status == 200
    assert (await client.post(base + 'execute', json={'command': command}, headers=headers)).status == 403
    assert (await client.post(base + 'speeds', json={}, headers=headers)).status == 403


@pytest.mark.asyncio
async def test_speeds_fresh_can_only_and_unset_target():
  routes = load_routes('assistant_drive')

  class MockSubMaster(dict):
    pass
  car = SimpleNamespace(vEgo=18., vCruise=60., canValid=True)
  sm = MockSubMaster(carState=car)
  sm.valid = {'carState': True}
  sm.alive = {'carState': True}
  sm.logMonoTime = {'carState': time.monotonic_ns()}
  app = {'realtime_broker': SimpleNamespace(sm=sm, poll=lambda _: None), 'realtime_broker_poll_lock': asyncio.Lock()}
  reading = await routes.read_speeds(app)
  assert (reading['vehicle_speed_kph'], reading['set_speed_kph']) == (64.8, 60)
  car.vEgo, car.vCruise = 0., 255
  reading = await routes.read_speeds(app)
  assert (reading['vehicle_speed_kph'], reading['set_speed_kph']) == (0, None)
  car.canValid = False
  assert (await routes.read_speeds(app))['status'] == 'unavailable'
  car.canValid = True
  sm.logMonoTime['carState'] -= 2_000_000_000
  assert (await routes.read_speeds(app))['status'] == 'unavailable'
  sm.logMonoTime['carState'] = time.monotonic_ns() + 2_000_000_000
  assert (await routes.read_speeds(app))['status'] == 'unavailable'
  sm.logMonoTime['carState'] = time.monotonic_ns()
  for field in (sm.valid, sm.alive):
    field['carState'] = False
    assert (await routes.read_speeds(app))['status'] == 'unavailable'
    field['carState'] = True
  car.vEgo, car.vCruise = float('nan'), 60.
  reading = await routes.read_speeds(app)
  assert (reading['vehicle_speed_kph'], reading['set_speed_kph']) == (None, 60)
  car.vEgo, car.vCruise = 18., float('inf')
  reading = await routes.read_speeds(app)
  assert (reading['vehicle_speed_kph'], reading['set_speed_kph']) == (64.8, None)
  car.vEgo = float('nan')
  assert (await routes.read_speeds(app))['status'] == 'unavailable'
