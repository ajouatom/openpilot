"""Optional restoration of existing local permissions after the app switches addresses."""
import json
from pathlib import Path

from aiohttp import web

from ..config import CARROT_STATE_DIR
from ..services.assistant_connection import AssistantConnection
from ..services.assistant_settings import AssistantSettings


def stores(app):
  from . import assistant_settings, assistant_drive, assistant_tools
  entries = {'settings': ('assistant_grants.json', assistant_settings.STATE),
             'drive': ('assistant_drive_grants.json', assistant_drive.STATE),
             'tools': ('assistant_tools_grants.json', assistant_tools.STATE)}
  result = {}
  for role, (name, key) in entries.items():
    active = app.get(key)
    if active:
      permissions = active if role == 'settings' else active.permissions
      result[role] = (permissions, lambda owner, current=active, kind=role: cancel_pending(current, kind, owner))
    else:
      permission = AssistantSettings(Path(CARROT_STATE_DIR) / name, dict, lambda _: None, lambda *_: None, lambda: True, lambda *_: None)
      result[role] = (permission, lambda owner, kind=role: cancel_pending(None, kind, owner))
  return result


def cancel_pending(active, role, owner):
  if role == 'tools' and active:
    active.proposals = {k: v for k, v in active.proposals.items() if v['owner'] != owner}
    active.results = {k: v for k, v in active.results.items() if k[0] != owner}
  if role == 'drive':
    from openpilot.selfdrive.carrot.bluetooth.model import atomic_json, read_json
    from openpilot.selfdrive.controls.lib.desire_lib.assistant_lane import ROOT, IDENTITY
    for root in (Path(ROOT), Path(ROOT) / 'speed'):
      pending = read_json(root / 'request.json', {})
      if isinstance(pending, dict) and pending.get('owner') == owner and IDENTITY.fullmatch(str(pending.get('id', ''))):
        atomic_json(root / 'cancel.json', {'id': pending['id']})


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
    available = stores(request.app)
    operation = request.match_info['operation']
    service = AssistantConnection(available)
    if operation == 'proof':
      result = service.proof(origin, body.get('previous_origin'), body.get('nonce'), body.get('identifiers'))
    elif operation == 'move':
      result = service.move(origin, body.get('previous_origin'), body.get('credentials'))
    else:
      raise ValueError('Unknown operation')
    return web.json_response({'ok': True, **result}, headers={'Cache-Control': 'no-store'})
  except PermissionError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=403)
  except (ValueError, TypeError, KeyError) as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=400)


def register(app):
  app.router.add_post('/api/assistant/connection/{operation}', handle)
