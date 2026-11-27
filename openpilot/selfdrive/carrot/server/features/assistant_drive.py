"""Same-origin driving permission, independent of setting-write permission."""
import asyncio
import json
import math
import time
from pathlib import Path

from aiohttp import web
from ..config import CARROT_STATE_DIR
from ..services.assistant_drive import AssistantDrive
from .bluetooth import runtime

STATE = web.AppKey('assistant_drive', AssistantDrive)


async def read_speeds(app):
  unavailable = {'status': 'unavailable', 'reason': 'no_fresh_data'}
  broker = app.get('realtime_broker')
  if broker is None:
    return unavailable
  async with app['realtime_broker_poll_lock']:
    await asyncio.to_thread(broker.poll, 0)
    sm = broker.sm
    age = time.monotonic() - sm.logMonoTime['carState'] / 1e9
    if not sm.valid['carState'] or not sm.alive['carState'] or not 0 <= age <= 1:
      return unavailable
    car = sm['carState']
    if not car.canValid:
      return unavailable
    actual = round(car.vEgo * 3.6, 1) if math.isfinite(car.vEgo) else None
    target = round(car.vCruise, 1) if 0 < car.vCruise < 255 else None  # 0: no target; 255: V_CRUISE_UNSET
    if actual is None and target is None:
      return unavailable
    return {'status': 'ok', 'vehicle_speed_kph': actual, 'set_speed_kph': target, 'sample_age_ms': math.ceil(age * 1000)}


async def handle(request):
  origin = request.headers.get('Origin')
  if origin != f'{request.scheme}://{request.host}' or request.headers.get('Sec-Fetch-Site') == 'cross-site':
    raise web.HTTPForbidden(text='Same-origin requests only')
  if request.content_type != 'application/json':
    raise web.HTTPUnsupportedMediaType(text='JSON required')
  raw = await request.content.read(2049)
  if len(raw) > 2048:
    raise web.HTTPRequestEntityTooLarge(max_size=2048, actual_size=len(raw))
  try:
    body = json.loads(raw)
    if not isinstance(body, dict):
      raise ValueError('Object required')
    if STATE not in request.app:
      request.app[STATE] = AssistantDrive(Path(CARROT_STATE_DIR) / 'assistant_drive_grants.json', lambda: runtime()['stationary'])
    service = request.app[STATE]
    operation = request.match_info['operation']
    token = request.headers.get('Authorization', '').removeprefix('Bearer ')
    if operation == 'enable':
      if body.get('confirmed') is not True:
        raise ValueError('Explicit driving permission required')
      result = {'token': service.enable(origin)}
    elif operation == 'disable':
      service.revoke(token, origin)
      result = {'revoked': True}
    elif operation == 'speeds':
      service.permissions.authorize(token, origin)
      result = await read_speeds(request.app)
    elif operation == 'execute':
      command = body.get('command')
      if not isinstance(command, dict):
        raise ValueError('Command required')
      expiry = command.get('expires_at')
      if type(expiry) not in (int, float) or not math.isfinite(expiry) or not 0 < expiry - time.time() * 1000 <= 5000:  # noqa: TID251
        raise ValueError('Driving request expired; do not retry it automatically')
      result = service.execute(token, origin, command)
      # One tool call can include the local controller acknowledgement.
      for _ in range(4):
        if result['status'] != 'waiting':
          break
        await asyncio.sleep(.05)
        result = service.execute(token, origin, {**command, 'operation': 'status'})
    else:
      raise ValueError('Unknown operation')
    return web.json_response({'ok': True, **result}, headers={'Cache-Control': 'no-store'})
  except PermissionError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=403)
  except (ValueError, TypeError, KeyError) as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=400)


def register(app):
  app.router.add_post('/api/assistant/drive/{operation}', handle)
