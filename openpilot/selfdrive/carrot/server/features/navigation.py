"""Mapbox search/destination UI and the shared nav-model preview."""
import asyncio
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
import time

import aiohttp
from aiohttp import web

from ..config import WEB_DIR
from ..services.params import get_param_value, set_param_value
from openpilot.selfdrive.navigation.preview import PREVIEW_PATH, WEB_DEMAND_PATH, preview_fresh, preview_reason

MAX_RESPONSE = 1024 * 1024


@dataclass
class SearchState:
  lock: asyncio.Lock = field(default_factory=asyncio.Lock)
  next_request: float = 0.


SEARCH_STATE = web.AppKey('navigation_search', SearchState)


def destination(value):
  if not isinstance(value, dict):
    raise ValueError('invalid destination')
  lat, lon = float(value['latitude']), float(value['longitude'])
  if not math.isfinite(lat) or not math.isfinite(lon) or abs(lat) > 85.05112878 or abs(lon) > 180:
    raise ValueError('invalid coordinates')
  return {'latitude': lat, 'longitude': lon, 'place_name': str(value.get('place_name', 'Destination'))[:256]}


async def page(request):
  return web.FileResponse(str(Path(WEB_DIR) / 'navigation.html'), headers={'Cache-Control': 'no-cache'})


async def search(request):
  query = request.query.get('q', '').strip()
  if not 2 <= len(query) <= 200:
    return web.json_response({'error': 'Enter 2–200 characters.'}, status=400)
  token = str(get_param_value('MapboxPublicKey', '') or '').strip()
  if not token:
    return web.json_response({'error': 'Set the Mapbox Public key in Settings → Network.'}, status=409)
  state = request.app[SEARCH_STATE]
  if state.lock.locked() or time.monotonic() < state.next_request:
    return web.json_response({'error': 'Please wait before searching again.'}, status=429)
  async with state.lock:
    state.next_request = time.monotonic() + 1.
    try:
      params = {'q': query, 'access_token': token, 'limit': '5', 'autocomplete': 'false', 'permanent': 'true'}
      async with request.app['http'].get('https://api.mapbox.com/search/geocode/v6/forward', params=params,
                                         timeout=aiohttp.ClientTimeout(total=8)) as response:
        if response.status != 200:
          return web.json_response({'error': f'Mapbox search failed (HTTP {response.status}).'}, status=502)
        body = bytearray()
        async for chunk in response.content.iter_chunked(65536):
          body.extend(chunk)
          if len(body) > MAX_RESPONSE:
            raise ValueError('oversized response')
        results = []
        for feature in json.loads(body).get('features', [])[:5]:
          lon, lat = feature['geometry']['coordinates'][:2]
          properties = feature.get('properties', {})
          results.append(destination({'latitude': lat, 'longitude': lon,
                                      'place_name': properties.get('full_address') or properties.get('name') or query}))
        return web.json_response({'results': results})
    except (aiohttp.ClientError, TimeoutError, ValueError, KeyError, TypeError, IndexError, AttributeError):
      return web.json_response({'error': 'Mapbox search is unavailable.'}, status=502)


async def get_destination(request):
  try:
    value = get_param_value('NavDestination', '')
    current = destination(json.loads(value)) if value else None
  except (ValueError, KeyError, TypeError):
    current = None
  return web.json_response({'destination': current, 'map_ready': preview_fresh() and preview_reason() == 'ready',
                            'key_configured': bool(get_param_value('MapboxPublicKey', ''))})


async def set_destination(request):
  try:
    value = destination(await request.json())
  except (ValueError, KeyError, TypeError):
    return web.json_response({'error': 'Invalid destination.'}, status=400)
  set_param_value('NavDestination', json.dumps(value, ensure_ascii=False))
  set_param_value('NavDestinationWaypoints', '')
  return web.json_response({'destination': value})


async def clear_destination(request):
  set_param_value('NavDestination', '')
  set_param_value('NavDestinationWaypoints', '')
  return web.json_response({'destination': None})


async def preview(request):
  try:
    WEB_DEMAND_PATH.touch()
  except OSError:
    pass
  if not preview_fresh():
    reason = 'waiting' if get_param_value('IsOnroad', False) else 'offroad'
    return web.Response(status=204, headers={'Cache-Control': 'no-store', 'X-Nav-Map-Status': reason})
  return web.FileResponse(PREVIEW_PATH, headers={'Cache-Control': 'no-store', 'X-Nav-Map-Status': preview_reason()})


def register(app):
  app[SEARCH_STATE] = SearchState()
  app.router.add_get('/navigation', page)
  app.router.add_get('/api/navigation/search', search)
  app.router.add_get('/api/navigation/destination', get_destination)
  app.router.add_post('/api/navigation/destination', set_destination)
  app.router.add_delete('/api/navigation/destination', clear_destination)
  app.router.add_get('/api/navigation/map.png', preview)
