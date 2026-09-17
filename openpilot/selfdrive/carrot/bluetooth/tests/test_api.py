import asyncio
import importlib.util
from pathlib import Path
import time

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.bluetooth.model import DEFAULT_MAPPING, atomic_json

MAC = '66:C0:0C:7B:6E:71'


def test_stationary_origin_mapping_and_no_http_command_endpoint(tmp_path, monkeypatch):
  spec = importlib.util.spec_from_file_location('bluetooth_feature_test', Path(__file__).parents[2] / 'server/features/bluetooth.py')
  feature = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(feature)
  monkeypatch.setattr(feature, 'RUNTIME', tmp_path)
  monkeypatch.setattr(feature, 'CONFIG_PATH', tmp_path / 'config.json')
  monkeypatch.setattr(feature, 'config', lambda: {'version': 1, 'devices': {}})

  class Client:
    async def snapshot(self):
      return {'devices': [{'address': MAC, 'paired': True}]}

    async def close(self):
      pass

  monkeypatch.setattr(feature, 'Bluez', Client)
  async def enabled():
    return True
  monkeypatch.setattr(feature, 'radio_enabled', enabled)

  async def run():
    app = web.Application()
    feature.register(app)
    async with TestClient(TestServer(app)) as client:
      result = await client.get('/api/bluetooth')
      assert result.status == 200
      assert (await result.json())['radioEnabled']
      payload = {'devices': {MAC: {'profile': 'yiser-j6', 'enabled': True, 'mapping': DEFAULT_MAPPING}}}
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 409
      atomic_json(tmp_path / 'status.json', {'time': time.monotonic(), 'stationary': True})
      assert (await client.post('/api/bluetooth/config', json=payload, headers={'Origin': 'https://evil.example'})).status == 403
      assert (await client.post('/api/bluetooth/config', data='{}')).status == 415
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 200
      assert (await client.post('/api/bluetooth/fire', json={'action': 'accelCruise'})).status == 404
      payload['devices'][MAC]['mapping'] = {'up': 'unrestrictedCommand'}
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 400
      atomic_json(tmp_path / 'status.json', {'time': time.monotonic() - 3, 'stationary': True})
      assert (await client.post('/api/bluetooth/config', json=payload)).status == 409
  asyncio.run(run())
