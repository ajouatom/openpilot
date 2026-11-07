"""Same-origin Tools permission, with bounded requests and existing job dispatch."""
import json
import math
import time
from pathlib import Path

from aiohttp import web

from ..config import CARROT_STATE_DIR
from ..services.assistant_tools import AssistantTools
from .bluetooth import runtime

STATE = web.AppKey('assistant_tools', AssistantTools)


def make_service():
  # Lazy imports keep the API testable without starting a device job dispatcher.
  from .tools.routes import start_job
  from .tools import jobs
  from ..services.params import Params

  def begin(body, guard):
    if any(job.get('status') == 'running' for job in jobs.jobs().values()):
      raise ValueError('다른 도구 작업이 진행 중입니다. 결과를 확인한 뒤 요청하세요.')
    return start_job(body, guard)

  def snapshot(identity):
    job = jobs.jobs().get(identity)
    if not job:
      return None
    # The legacy deletion dispatcher reports success even with per-file errors.
    if 'delete error:' in job.get('log', '') or 'error removing ' in job.get('log', ''):
      return {'status': 'failed'}
    return job

  def record(enabled):
    params = Params()
    params.put_bool('ScreenRecord', enabled)
    if params.get_bool('ScreenRecord') != enabled:
      raise ValueError('화면 녹화 설정 저장을 확인하지 못했습니다.')

  return AssistantTools(Path(CARROT_STATE_DIR) / 'assistant_tools_grants.json',
                        lambda: runtime()['stationary'], begin, snapshot, record)


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
      request.app[STATE] = make_service()
    service = request.app[STATE]
    operation = request.match_info['operation']
    if operation == 'enable':
      if body.get('confirmed') is not True:
        raise ValueError('Explicit local permission required')
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
        now_ms = time.time() * 1000  # noqa: TID251
        if type(expires) not in (int, float) or not math.isfinite(expires) or not 0 < expires - now_ms <= 30000:
          raise ValueError('Voice request expired; do not retry automatically')
        result = service.execute(token, origin, command)
      else:
        raise ValueError('Unknown operation')
    return web.json_response({'ok': True, **result}, headers={'Cache-Control': 'no-store'})
  except PermissionError as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=403)
  except (ValueError, TypeError, KeyError) as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=400)


def register(app):
  app.router.add_post('/api/assistant/tools/{operation}', handle)
