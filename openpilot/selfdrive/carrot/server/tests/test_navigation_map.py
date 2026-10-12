import asyncio
import json
from contextlib import asynccontextmanager
from types import SimpleNamespace as NS

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.server.features import navigation


def test_destination_lifecycle_and_validation(monkeypatch):
  values = {}
  monkeypatch.setattr(navigation, 'get_param_value', lambda key, default: values.get(key, default))
  monkeypatch.setattr(navigation, 'set_param_value', lambda key, value: values.__setitem__(key, value))

  async def run():
    app = web.Application()
    navigation.register(app)
    async with TestClient(TestServer(app)) as client:
      response = await client.post('/api/navigation/destination', json={'latitude': 37., 'longitude': 127., 'place_name': '<script>name</script>'})
      assert response.status == 200
      assert json.loads(values['NavDestination'])['place_name'] == '<script>name</script>'
      response = await client.get('/api/navigation/destination')
      assert (await response.json())['destination']['latitude'] == 37.
      for payload in ([], {}, {'latitude': 91, 'longitude': 127}, {'latitude': None, 'longitude': 0}):
        assert (await client.post('/api/navigation/destination', json=payload)).status == 400
      assert (await client.get('/api/navigation/search?q=Seoul')).status == 409
      assert (await client.delete('/api/navigation/destination')).status == 200
      assert values['NavDestination'] == ''
  asyncio.run(run())


def test_navigation_page_is_served():
  async def run():
    app = web.Application()
    navigation.register(app)
    async with TestClient(TestServer(app)) as client:
      response = await client.get('/navigation')
      assert response.status == 200
      assert 'navigation_map.js' in await response.text()
  asyncio.run(run())


def test_search_reads_fragmented_response_and_keeps_token_server_side(monkeypatch):
  monkeypatch.setattr(navigation, 'get_param_value', lambda key, default: 'pk.private-test')
  payload = json.dumps({'features': [{'geometry': {'coordinates': [127., 37.]},
                                     'properties': {'full_address': 'Test destination'}}]}).encode()
  async def chunks(_):
    for i in range(0, len(payload), 7):
      yield payload[i:i + 7]
  @asynccontextmanager
  async def get(url, **kwargs):
    assert url == 'https://api.mapbox.com/search/geocode/v6/forward'
    assert kwargs['params']['access_token'] == 'pk.private-test'
    assert kwargs['params']['permanent'] == 'true'
    yield NS(status=200, content=NS(iter_chunked=chunks))
  async def run():
    app = web.Application()
    app['http'] = NS(get=get)
    navigation.register(app)
    async with TestClient(TestServer(app)) as client:
      response = await client.get('/api/navigation/search?q=Seoul')
      body = await response.text()
      assert response.status == 200 and 'pk.private-test' not in body
      assert json.loads(body)['results'][0]['latitude'] == 37.
      assert (await client.get('/api/navigation/search?q=Seoul')).status == 429
  asyncio.run(run())
