"""BlueZ D-Bus client and application-scoped pairing agent (no shell parsing)."""
import asyncio
from contextlib import suppress
import uuid

from jeepney import DBusAddress, HeaderFields, MatchRule, new_method_call, new_method_return, new_error
from jeepney.io.asyncio import open_dbus_router
from jeepney.wrappers import unwrap_msg

from .model import address

AGENT = '/org/carrot/BluetoothAgent'
DEVICE = 'org.bluez.Device1'
ADAPTER = 'org.bluez.Adapter1'


def properties(values):
  return {key: value[1] for key, value in values.items()}


class Bluez:
  def __init__(self):
    self.context = self.router = self.agent_task = self.pair_task = self.scan_task = None
    self.lock = asyncio.Lock()
    self.agent_registered = False
    self.owner = None
    self.target = None
    self.prompt = None
    self.answer = None
    self.pair = {'state': 'idle'}

  async def call(self, path, interface, method, signature=None, body=(), reply_timeout=10):
    message = new_method_call(DBusAddress(path, bus_name='org.bluez', interface=interface), method, signature, body)
    return unwrap_msg(await asyncio.wait_for(self.router.send_and_get_reply(message), reply_timeout))

  async def ensure(self):
    async with self.lock:
      if self.router is None:
        self.context = open_dbus_router('SYSTEM')
        self.router = await self.context.__aenter__()
        self.agent_task = asyncio.create_task(self.serve_agent())

  async def objects(self):
    await self.ensure()
    return (await self.call('/', 'org.freedesktop.DBus.ObjectManager', 'GetManagedObjects'))[0]

  async def locate(self, mac):
    mac = address(mac)
    for path, interfaces in (await self.objects()).items():
      if DEVICE in interfaces and properties(interfaces[DEVICE]).get('Address', '').upper() == mac:
        return path
    raise ValueError('device not found; scan again')

  async def snapshot(self):
    adapters, devices = [], []
    for _path, interfaces in (await self.objects()).items():
      if ADAPTER in interfaces:
        props = properties(interfaces[ADAPTER])
        adapters.append({'address': props.get('Address'), 'powered': props.get('Powered', False),
                         'discovering': props.get('Discovering', False)})
      if DEVICE in interfaces:
        props = properties(interfaces[DEVICE])
        devices.append({'address': props.get('Address'), 'name': props.get('Name', props.get('Alias', '')),
                        'paired': props.get('Paired', False), 'connected': props.get('Connected', False),
                        'trusted': props.get('Trusted', False), 'uuids': props.get('UUIDs', []),
                        'rssi': props.get('RSSI'), 'battery': properties(interfaces.get('org.bluez.Battery1', {})).get('Percentage')})
    return {'adapters': adapters, 'devices': devices, 'pair': self.pair, 'prompt': self.prompt}

  async def scan(self):
    if self.scan_task and not self.scan_task.done():
      return
    adapters = [p for p, interfaces in (await self.objects()).items() if ADAPTER in interfaces]
    if not adapters:
      raise ValueError('Bluetooth adapter unavailable')
    path = adapters[0]
    await self.call(path, ADAPTER, 'StartDiscovery')

    async def stop():
      try:
        await asyncio.sleep(30)
      finally:
        with suppress(Exception):
          await self.call(path, ADAPTER, 'StopDiscovery')
    self.scan_task = asyncio.create_task(stop())

  async def device_action(self, mac, action):
    path = await self.locate(mac)
    if action == 'forget':
      await self.call(path.rsplit('/', 1)[0], ADAPTER, 'RemoveDevice', 'o', (path,))
    else:
      await self.call(path, DEVICE, {'connect': 'Connect', 'disconnect': 'Disconnect'}[action], reply_timeout=30)

  async def start_pair(self, mac):
    if self.pair_task and not self.pair_task.done():
      raise ValueError('pairing already in progress')
    path = await self.locate(mac)
    if not self.agent_registered:
      owner_request = new_method_call(DBusAddress('/org/freedesktop/DBus', bus_name='org.freedesktop.DBus',
        interface='org.freedesktop.DBus'), 'GetNameOwner', 's', ('org.bluez',))
      self.owner = unwrap_msg(await asyncio.wait_for(self.router.send_and_get_reply(owner_request), 5))[0]
      await self.call('/org/bluez', 'org.bluez.AgentManager1', 'RegisterAgent', 'os', (AGENT, 'KeyboardDisplay'))
      self.agent_registered = True
    self.target = path
    self.pair = {'state': 'pairing', 'address': address(mac)}

    async def run():
      try:
        await self.call(path, DEVICE, 'Pair', reply_timeout=90)
        await self.call(path, 'org.freedesktop.DBus.Properties', 'Set', 'ssv', (DEVICE, 'Trusted', ('b', True)))
        self.pair = {'state': 'paired', 'address': address(mac)}
        try:
          await self.call(path, DEVICE, 'Connect', reply_timeout=20)
        except Exception as exc:
          self.pair['error'] = str(exc)
      except asyncio.CancelledError:
        self.pair = {'state': 'cancelled', 'address': address(mac)}
        raise
      except Exception as exc:
        self.pair = {'state': 'error', 'address': address(mac), 'error': str(exc)}
      finally:
        self.target = self.prompt = None
        if self.answer and not self.answer.done():
          self.answer.cancel()
        self.answer = None
    self.pair_task = asyncio.create_task(run())

  async def cancel_pair(self):
    if self.target:
      with suppress(Exception):
        await self.call(self.target, DEVICE, 'CancelPairing')
    if self.pair_task and not self.pair_task.done():
      self.pair_task.cancel()
      with suppress(asyncio.CancelledError):
        await self.pair_task

  def respond(self, prompt_id, value):
    if not self.prompt or self.prompt['id'] != prompt_id or not self.answer or self.answer.done():
      raise ValueError('pairing prompt expired')
    kind = self.prompt['kind']
    if value is not False:
      if kind in ('RequestConfirmation', 'RequestAuthorization', 'AuthorizeService') and value is not True:
        raise ValueError('confirmation must be boolean')
      if kind == 'RequestPasskey' and (not str(value).isdigit() or len(str(value)) > 6):
        raise ValueError('passkey must contain 1 to 6 digits')
      if kind == 'RequestPinCode' and (not isinstance(value, str) or not 1 <= len(value) <= 16):
        raise ValueError('PIN must contain 1 to 16 characters')
    self.answer.set_result(value)

  async def serve_agent(self):
    tasks = set()
    try:
      with self.router.filter(MatchRule(type='method_call', path=AGENT), bufsize=16) as queue:
        while True:
          task = asyncio.create_task(self.agent_reply(await queue.get()))
          tasks.add(task)
          task.add_done_callback(tasks.discard)
    finally:
      for task in tasks:
        task.cancel()
      await asyncio.gather(*tasks, return_exceptions=True)

  async def agent_reply(self, message):
    method = message.header.fields.get(HeaderFields.member)
    try:
      if message.header.fields.get(HeaderFields.sender) != self.owner or message.header.fields.get(HeaderFields.interface) != 'org.bluez.Agent1':
        reply = new_error(message, 'org.bluez.Error.Rejected')
      elif method in ('Cancel', 'Release'):
        if self.answer and not self.answer.done():
          self.answer.set_result(False)
        self.prompt = None
        reply = new_method_return(message)
      elif not self.target or not message.body or message.body[0] != self.target:
        reply = new_error(message, 'org.bluez.Error.Rejected')
      elif method in ('DisplayPinCode', 'DisplayPasskey'):
        self.prompt = {'id': uuid.uuid4().hex, 'kind': method, 'value': str(message.body[1]).zfill(6) if method == 'DisplayPasskey' else str(message.body[1])}
        reply = new_method_return(message)
      elif method in ('RequestConfirmation', 'RequestAuthorization', 'AuthorizeService', 'RequestPinCode', 'RequestPasskey'):
        if self.answer and not self.answer.done():
          raise ValueError('another pairing prompt is pending')
        self.answer = asyncio.get_running_loop().create_future()
        self.prompt = {'id': uuid.uuid4().hex, 'kind': method,
                       'value': str(message.body[1]).zfill(6) if method == 'RequestConfirmation' else ''}
        value = await asyncio.wait_for(self.answer, 60)
        self.prompt = None
        if value is False:
          reply = new_error(message, 'org.bluez.Error.Rejected')
        elif method == 'RequestPinCode':
          reply = new_method_return(message, 's', (value,))
        elif method == 'RequestPasskey':
          reply = new_method_return(message, 'u', (int(value),))
        else:
          reply = new_method_return(message)
      else:
        reply = new_error(message, 'org.bluez.Error.Rejected')
    except (Exception, asyncio.CancelledError):
      reply = new_error(message, 'org.bluez.Error.Canceled')
    with suppress(Exception):
      await self.router.send(reply)

  async def close(self):
    await self.cancel_pair()
    for task in (self.scan_task, self.agent_task):
      if task:
        task.cancel()
        with suppress(Exception, asyncio.CancelledError):
          await task
    if self.context:
      with suppress(Exception):
        await self.context.__aexit__(None, None, None)
    self.context = self.router = None
    self.agent_registered = False
