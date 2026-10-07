import asyncio
import time

from aiohttp import web

from openpilot.common.jetson_maintenance import PENDING, parked, status


async def get_status(_request):
  from openpilot.common.params import Params
  return web.json_response({'ok': True, **status(Params())})


async def request_wait(request):
  from openpilot.common.params import Params
  try:
    body = await request.json()
  except Exception:
    return web.json_response({'ok': False, 'error': 'invalid json'}, status=400)
  if not isinstance(body, dict) or type(body.get('enabled')) is not bool:
    return web.json_response({'ok': False, 'error': 'enabled must be boolean'}, status=400)
  params = Params()
  if not body['enabled']:
    params.put_bool(PENDING, False)
    return web.json_response({'ok': True, **status(params)})
  if params.get_bool(PENDING):
    return web.json_response({'ok': True, **status(params)})
  host = status(params)
  if not host['connected'] or host['migrated']:
    return web.json_response({'ok': False, 'error_code': 'jetson_unavailable'}, status=409)
  # Observe a full second, ending at the write; cached Params are not vehicle state.
  from openpilot.cereal import messaging
  sm = messaging.SubMaster(['carState', 'selfdriveState', 'carControl'])
  ready_since = None
  deadline = time.monotonic() + 3
  while time.monotonic() < deadline:
    sm.update(0)
    now = time.monotonic()
    if parked(sm):
      if ready_since is None:
        ready_since = now
      if now - ready_since >= 1:
        params.put_bool(PENDING, True)
        return web.json_response({'ok': True, **status(params)})
    else:
      ready_since = None
    await asyncio.sleep(.05)
  return web.json_response({'ok': False, 'error_code': 'park_required'}, status=409)
