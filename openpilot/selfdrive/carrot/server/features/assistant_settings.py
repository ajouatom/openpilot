"""Local authorization and validation for the account-bound voice setting bridge."""
import json
import math
import time
from pathlib import Path
from types import SimpleNamespace

from aiohttp import web

from ..config import CARROT_STATE_DIR
from ..services import params
from ..services.assistant_settings import AssistantSettings
from ..services.param_changes import append_param_change
from ..services.settings import get_settings_cached
from .bluetooth import runtime
from .system import is_drive_engaged

STATE = web.AppKey('assistant_settings', AssistantSettings)


def _read(name):
  if not params.HAS_PARAMS or params.Params is None:
    raise ValueError('Device Params unavailable')
  raw = params.Params().get(name)
  if raw is None:
    return None
  value = float(raw)
  if not math.isfinite(value) or not value.is_integer():
    raise ValueError('Invalid current device value')
  return int(value)


def _write(name, value):
  if not params.HAS_PARAMS:
    raise ValueError('Device Params unavailable')
  params.set_param_value(name, value, get_settings_cached()[2][name])


async def handle(request):
  origin = request.headers.get('Origin')
  if origin != f'{request.scheme}://{request.host}' or request.headers.get('Sec-Fetch-Site') == 'cross-site':
    raise web.HTTPForbidden(text='Same-origin requests only')
  if request.content_type != 'application/json':
    raise web.HTTPUnsupportedMediaType(text='JSON required')
  raw = await request.content.read(4097)
  if len(raw) > 4096:
    raise web.HTTPRequestEntityTooLarge(max_size=4096, actual_size=len(raw))
  try:
    body = json.loads(raw)
    if not isinstance(body, dict):
      raise ValueError('Object required')
    if STATE not in request.app:
      request.app[STATE] = AssistantSettings(Path(CARROT_STATE_DIR) / 'assistant_grants.json',
        lambda: get_settings_cached()[2], _read, _write, lambda: runtime()['stationary'],
        lambda name, before, after, app=request.app: append_param_change(
          name, before, after, source='voice_assistant', engaged=is_drive_engaged(SimpleNamespace(app=app))))
    service = request.app[STATE]
    operation = request.match_info['operation']
    if operation == 'enable':
      if body.get('confirmed') is not True:
        raise ValueError('Explicit local permission confirmation required')
      result = {'token': service.enable(origin)}
    else:
      token = request.headers.get('Authorization', '').removeprefix('Bearer ')
      if operation == 'disable':
        service.revoke(token, origin)
        result = {'revoked': True}
      elif operation == 'execute':
        command = body.get('command')
        if not isinstance(command, dict):
          raise ValueError('Command required')
        expires = command.get('expires_at')
        # Relay expiry is a cross-host Unix timestamp; local proposals use monotonic time.
        now_ms = time.time() * 1000  # noqa: TID251
        if type(expires) not in (int, float) or not math.isfinite(expires) or not 0 < expires - now_ms <= 30000:
          raise ValueError('Voice request expired; ask again')
        result = service.execute(token, origin, command)
      else:
        raise ValueError('Unknown operation')
    return web.json_response({'ok': True, **result}, headers={'Cache-Control': 'no-store'})
  except PermissionError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=403)
  except (ValueError, TypeError, KeyError) as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=400)


def register(app):
  app.router.add_post('/api/assistant/settings/{operation}', handle)
