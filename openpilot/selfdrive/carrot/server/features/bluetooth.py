"""Stationary-only Bluetooth setup. HTTP can configure mappings, never fire them."""
import asyncio
import time
from urllib.parse import urlsplit

from aiohttp import web

from openpilot.selfdrive.carrot.bluetooth.bluez import Bluez
from openpilot.selfdrive.carrot.bluetooth.model import CONFIG_PATH, RUNTIME, ACTIONS, DEFAULT_MAPPING, address, atomic_json, config, read_json, validate_config

CLIENT = web.AppKey('bluetooth', Bluez)
LOCK = web.AppKey('bluetooth_lock', asyncio.Lock)


async def radio_enabled():
  # BlueZ deliberately keeps the parent directory private because it contains bonds.
  process = await asyncio.create_subprocess_exec('sudo', '-n', 'test', '-f', '/data/bluetooth/ENABLED',
    stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
  try:
    return await asyncio.wait_for(process.wait(), 3) == 0
  except TimeoutError:
    process.kill()
    await process.wait()
    return False


def runtime():
  state = read_json(RUNTIME / 'status.json', {})
  if not isinstance(state, dict):
    state = {}
  stamp = state.get('time', 0)
  state['alive'] = isinstance(stamp, (float, int)) and 0 <= time.monotonic() - stamp < 2 and not state.get('stopped')
  state['stationary'] = bool(state['alive'] and state.get('stationary'))
  return state


def guard(request):
  origin = request.headers.get('Origin')
  if (origin and urlsplit(origin).netloc != request.host) or request.headers.get('Sec-Fetch-Site') == 'cross-site':
    raise web.HTTPForbidden(text='same-origin requests only')
  if request.content_type != 'application/json':
    raise web.HTTPUnsupportedMediaType(text='application/json required')
  if not runtime()['stationary']:
    raise web.HTTPConflict(text='setup requires fresh stationary and disengaged state')


async def status(request):
  result = {'runtime': runtime(), 'config': config(), 'actions': ACTIONS, 'defaults': DEFAULT_MAPPING,
            'radioEnabled': await radio_enabled()}
  try:
    result.update(await request.app[CLIENT].snapshot())
    result['available'] = True
  except Exception as exc:
    result.update(available=False, error=str(exc), devices=[], adapters=[])
  return web.json_response(result)


async def mutate(request):
  guard(request)
  if request.content_length is not None and request.content_length > 32768:
    raise web.HTTPRequestEntityTooLarge(max_size=32768, actual_size=request.content_length)
  raw = await request.content.read(32769)
  if len(raw) > 32768:
    raise web.HTTPRequestEntityTooLarge(max_size=32768, actual_size=len(raw))
  import json
  try:
    body = json.loads(raw)
    if not isinstance(body, dict):
      raise ValueError('object required')
    client = request.app[CLIENT]
    operation = request.match_info['operation']
    async with request.app[LOCK]:
      guard(request)
      if operation == 'scan':
        await client.scan()
      elif operation == 'pair':
        await client.start_pair(address(body.get('address')))
      elif operation == 'cancel':
        await client.cancel_pair()
      elif operation == 'answer':
        client.respond(body.get('id'), body.get('value'))
      elif operation in ('connect', 'disconnect', 'forget'):
        mac = address(body.get('address'))
        await client.device_action(mac, operation)
        if operation == 'forget':
          settings = config()
          settings['devices'].pop(mac, None)
          atomic_json(CONFIG_PATH, settings)
      elif operation == 'config':
        settings = validate_config(body)
        paired = {d['address'] for d in (await client.snapshot())['devices'] if d['paired']}
        if any(mac not in paired for mac in settings['devices']):
          raise ValueError('pair devices before configuring input')
        atomic_json(CONFIG_PATH, settings)
      elif operation == 'learn':
        mac = address(body.get('address'))
        if mac not in config()['devices']:
          raise ValueError('save the input profile first')
        if type(body.get('enabled')) is not bool:
          raise ValueError('enabled must be boolean')
        atomic_json(RUNTIME / 'learn.json', {'address': mac, 'until': time.monotonic() + 120} if body['enabled'] else {})
        # Clear a pending action when entering test mode.
        for channel in ('cruise', 'lane'):
          atomic_json(RUNTIME / f'{channel}.json', {})
      elif operation == 'radio':
        if type(body.get('enabled')) is not bool:
          raise ValueError('enabled must be boolean')
        await client.close()
        commands = ([['sudo', '-n', 'mkdir', '-p', '/data/bluetooth'],
                     ['sudo', '-n', 'touch', '/data/bluetooth/ENABLED'],
                     ['sudo', '-n', 'systemctl', 'start', 'carrot-bluetooth-radio']] if body['enabled'] else
                    [['sudo', '-n', 'rm', '-f', '/data/bluetooth/ENABLED'],
                     ['sudo', '-n', 'systemctl', 'stop', 'carrot-bluetooth-radio'],
                     ['sudo', '-n', 'systemctl', 'stop', 'bluetooth']])
        for command in commands:
          process = await asyncio.create_subprocess_exec(*command, stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.PIPE)
          try:
            _, error = await asyncio.wait_for(process.communicate(), 20)
          except TimeoutError:
            process.kill()
            await process.wait()
            raise ValueError('radio operation timed out') from None
          if process.returncode:
            raise ValueError(error.decode(errors='replace')[:500])
      else:
        raise web.HTTPNotFound()
    return web.json_response({'ok': True})
  except (ValueError, TypeError, KeyError) as exc:
    raise web.HTTPBadRequest(text=str(exc)) from exc
  except web.HTTPException:
    raise
  except Exception as exc:
    raise web.HTTPBadGateway(text=str(exc)) from exc


def register(app):
  app[CLIENT] = Bluez()
  app[LOCK] = asyncio.Lock()
  app.router.add_get('/api/bluetooth', status)
  app.router.add_post('/api/bluetooth/{operation}', mutate)

  async def cleanup(app):
    await app[CLIENT].close()
  app.on_cleanup.append(cleanup)
