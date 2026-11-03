"""Authenticated BLE output for the existing indicator policy. No raw CAN over BLE."""
import asyncio
from collections import deque
from contextlib import suppress
import hmac
from pathlib import Path
import struct
import time

from .bluez import Bluez, DEVICE, properties
from .model import CONFIG_PATH, address, read_json

ENROLLMENT = CONFIG_PATH.with_name('b1-monitor.json')
SERVICE = 'db340a90-804c-4ead-af37-8a0e6af754a1'
META = 'db340a92-804c-4ead-af37-8a0e6af754a1'
CONTROL = 'db340a93-804c-4ead-af37-8a0e6af754a1'
GATT = 'org.bluez.GattCharacteristic1'
STATES = ('missing', 'ready', 'sending', 'waiting', 'driver_override', 'error')


def radio_present():
  return Path('/sys/class/bluetooth/hci0').exists()


def compatible(device):
  return device.get('name') == 'Carrot-B1' and SERVICE in device.get('uuids', [])


def command(key, nonce, stamp, seq, op, ttl):
  body = struct.pack('<IIHBB', nonce, stamp, seq, op, min(70, max(1, int(ttl * 200))))
  return body + hmac.digest(key, body, 'sha256')[:8]


class Intent:
  def __init__(self):
    self.started = time.monotonic()
    self.seen = deque(maxlen=256)
    self.inflight = None
    self.queued = None
    self.seq = 0

  def choose(self, source, state, ack, now):
    if not source or not source.get('ok') or not 0 <= now-source['stamp'] < .20 or state in ('error','driver_override','missing'):
      self.inflight = self.queued = None
      return 3, None, .10
    guard = source['stamp'] + .35 - now
    hold = source.get('hold')
    devices = source.get('devices', [])
    candidates = [(e, False) for e in source.get('events', [])]
    if hold:
      candidates.append((hold, True))
    for event, held in candidates:
      ident = event['id']
      if ident in self.seen or not self.started <= event['time'] <= now or now-event['time'] >= .4:
        continue
      self.seen.append(ident)
      if state == 'ready' or (self.inflight and state in ('sending','waiting')):
        lifetime = 3.2 if self.inflight and state != 'ready' else .4
        self.queued = (event, held, event['time'] + lifetime)
    if self.queued:
      event, held, until = self.queued
      if now >= until or (held and (not hold or hold['id'] != event['id'])) or (event.get('device') and event['device'] not in devices):
        self.queued = None
      elif state == 'ready' and (not self.inflight or ack == self.inflight[0]):
        self.queued = None
        return (1 if event['direction'] == 'left' else 2), event['id'], min(guard, until-now)
    if (source.get('b1_hold') and hold and self.inflight and ack == self.inflight[0] and
        hold['id'] == self.inflight[1] and now-hold['time'] < 300):
      return (4 if hold['direction']=='left' else 5), None, guard
    return 0, None, guard


class Monitor:
  def __init__(self):
    self.client = Bluez()
    self.state = 'idle'
    self.owned = None
    self.worker = None
    self.intent = Intent()
    self.attrs = None
    self.session = None
    self.next_start = 0.
    self.source = None
    self.params = None
    self.sent_sequences = deque(maxlen=6)

  def stop_source(self):
    if self.worker:
      self.worker.close()
      self.worker = None
    self.source = None

  async def release(self):
    self.stop_source()
    if self.owned:
      with suppress(Exception):
        await self.client.device_action(self.owned, 'disconnect')
    self.owned = self.attrs = self.session = None
    self.intent = Intent()
    await self.client.close()

  async def discover(self, mac):
    objects = await self.client.objects()
    for path, interfaces in objects.items():
      device = properties(interfaces.get(DEVICE, {}))
      if device.get('Address', '').upper() != mac or not compatible({'name':device.get('Name'), 'uuids':device.get('UUIDs', [])}):
        continue
      if not device.get('Connected'):
        await self.client.device_action(mac, 'connect')
        self.owned = mac
        return 'connecting'
      self.owned = mac
      attrs = {}
      for attr, interfaces in objects.items():
        char = properties(interfaces.get(GATT, {}))
        parent = properties(objects.get(char.get('Service'), {}).get('org.bluez.GattService1', {}))
        if parent.get('Device') == path and parent.get('UUID') == SERVICE:
          attrs[char.get('UUID')] = attr
      if META in attrs and CONTROL in attrs:
        self.attrs = attrs
        return 'connected'
      return 'waiting for control firmware'
    await self.client.scan()
    return 'scanning'

  async def exchange(self, key):
    from tools.jetlink.blink_protocol import Process  # noqa: TID251 -- shared standalone indicator worker
    from openpilot.common.params import Params
    if self.params is None:
      self.params = Params()
    # Conservative board-clock sample: GATT latency can only shorten the lease.
    raw = await self.client.call(self.attrs[META], GATT, 'ReadValue', 'a{sv}', ({},), reply_timeout=.35)
    nonce, stamp, ack, state_id, lamps, version, last_seq = struct.unpack('<IIHBBBH', bytes(raw[0]))
    if not nonce or version != 2 or state_id >= len(STATES):
      raise ValueError('unsupported B1 report')
    state = STATES[state_id]
    now = time.monotonic()
    if nonce != self.session:
      self.stop_source()
      self.session, self.intent = nonce, Intent()
      self.intent.seq = last_seq
      self.sent_sequences.clear()
      self.next_start = now + .65
    # The SPAS option owns its own output, and an offroad/stopped manager has no control lease.
    authenticated = last_seq in self.sent_sequences
    available = authenticated and state not in ('missing','error') and self.params.get_bool('IsOnroad') and not self.params.get_bool('CanfdBlinkerControl')
    if self.worker and (not available or self.worker.process.poll() is not None):
      self.stop_source()
      self.intent.inflight = self.intent.queued = None
      self.next_start = now + 1
    if not self.worker and available and now >= self.next_start:
      self.worker = Process('blink_source.py', 'ble')
      self.next_start = now + 2
    pending = self.intent.inflight
    if pending and state != 'sending' and ack != pending[0] and now-pending[2] > .45:
      self.intent.inflight = self.intent.queued = pending = None
    if self.worker:
      self.worker.send({'stamp':now, 'state':state, 'accepted_id': pending[1] if pending and ack==pending[0] else None})
      latest = self.worker.latest()
      if latest is not None:
        self.source = latest
    op, ident, ttl = self.intent.choose(self.source, state, ack, now)
    self.intent.seq = (self.intent.seq + 1) % 65536 or 1
    packet = command(key, nonce, stamp, self.intent.seq, op, ttl)
    await self.client.call(self.attrs[CONTROL], GATT, 'WriteValue', 'aya{sv}', (packet, {'type':('s','request')}), reply_timeout=.35)
    self.sent_sequences.append(self.intent.seq)
    if ident:
      self.intent.inflight = self.intent.seq, ident, now
    self.state = f'{state} · L={lamps & 1} R={(lamps >> 1) & 1}' if authenticated else 'verifying board key'

  async def run(self):
    registration = None
    next_config = 0.
    try:
      while True:
        now = time.monotonic()
        if now >= next_config:
          record = read_json(ENROLLMENT, {})
          new = None
          if isinstance(record, dict) and record.get('key'):
            with suppress(ValueError, TypeError):
              key = bytes.fromhex(record['key'])
              if len(key)==32:
                new = address(record.get('address')), key
          if new != registration:
            await self.release()
            registration = new
          next_config = now + .25
        if not registration or not radio_present():
          if self.owned or self.worker:
            await self.release()
          self.state = 'waiting for radio' if registration else 'idle'
          await asyncio.sleep(.5)
          continue
        try:
          if self.attrs:
            await self.exchange(registration[1])
            await asyncio.sleep(.035)
          else:
            self.state = await self.discover(registration[0])
            await asyncio.sleep(.5 if self.owned else 3)
        except Exception:
          await self.release()
          self.state = 'waiting to reconnect'
          await asyncio.sleep(2)
    finally:
      await self.release()
