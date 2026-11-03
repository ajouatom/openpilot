"""Authenticated BLE output for the existing indicator policy. No raw CAN over BLE."""
import asyncio
from collections import deque
from contextlib import suppress
import hashlib
import hmac
import secrets
from pathlib import Path
import struct
import time

from .bluez import Bluez, DEVICE, properties, radio_enabled
from .model import CONFIG_PATH, RUNTIME, address, atomic_json, read_json

ENROLLMENT = CONFIG_PATH.with_name('b1-monitor.json')
SERVICE = 'db340a90-804c-4ead-af37-8a0e6af754a1'
META = 'db340a92-804c-4ead-af37-8a0e6af754a1'
CONTROL = 'db340a93-804c-4ead-af37-8a0e6af754a1'
CLAIM = 'db340a95-804c-4ead-af37-8a0e6af754a1'
SETUP = CONFIG_PATH.with_name('b1-discovery.json')
STAGED = CONFIG_PATH.with_name('b1-pending.json')
BADGE = Path('/dev/shm/carrot-b1-link.json')
GATT = 'org.bluez.GattCharacteristic1'
STATES = ('missing', 'ready', 'sending', 'waiting', 'driver_override', 'error')


OTA_LOCK = RUNTIME / 'b1-ota.lock'


def maintenance_active():
  # Keep the BLE connection, but leave the ATT channel to the parked updater.
  import fcntl
  try:
    with OTA_LOCK.open('rb') as lock:
      fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
  except FileNotFoundError:
    return False
  except BlockingIOError:
    return True
  return False


def radio_present():
  return Path('/sys/class/bluetooth/hci0').exists()


def parked():
  state = read_json(RUNTIME / 'status.json', {})
  return (state.get('stationary') is True and not state.get('stopped') and
          0 <= time.monotonic()-state.get('time', 0) < 2)


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
      return 3, None, .35
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
    self.exchange_lock = asyncio.Lock()
    self.last_signal_at = time.monotonic()
    self.state = 'idle'
    self.firmware_state = 'waiting'
    self.owned = None
    self.worker = None
    self.intent = Intent()
    self.attrs = None
    self.session = None
    self.next_start = 0.
    self.source = None
    self.params = None
    self.sent_sequences = deque(maxlen=6)
    self.next_badge = 0.
    self.last_lamps = None
    self.proven_at = -10.
    self.last_command = None
    self.next_discovery = 0.

  def publish_badge(self, state, now, lamps, accepted_id):
    if lamps != self.last_lamps or now >= self.next_badge:
      self.last_lamps = lamps
      self.next_badge = now + .35
      with suppress(OSError):
        atomic_json(BADGE, {'state':state, 'time':now, 'lamps':lamps, 'accepted_id':accepted_id,
                            'source_ready':bool(self.source and self.source.get('ok') and 0 <= now-self.source['stamp'] < .2),
                            'last_command':self.last_command})

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
        if device.get('RSSI') is None:
          await self.client.scan()
          return 'scanning'
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
      self.last_signal_at = now
      self.proven_at = -10.
      self.last_command = None
      self.next_start = now + .65
    # The SPAS option owns its own output, and an offroad/stopped manager has no control lease.
    if last_seq in self.sent_sequences:
      self.proven_at = now
    authenticated = now-self.proven_at < 1.
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
    # The worker can publish a newer timestamp while latest() drains its socket.
    now = time.monotonic()
    op, ident, ttl = self.intent.choose(self.source, state, ack, now)
    if (lamps or state not in ('ready', 'missing') or ident or self.intent.queued or
        (self.source and (self.source.get('hold') or self.source.get('events')))):
      self.last_signal_at = now
    self.intent.seq = (self.intent.seq + 1) % 65536 or 1
    packet = command(key, nonce, stamp, self.intent.seq, op, ttl)
    await self.client.call(self.attrs[CONTROL], GATT, 'WriteValue', 'aya{sv}', (packet, {'type':('s','request')}), reply_timeout=.35)
    self.sent_sequences.append(self.intent.seq)
    if ident:
      self.last_command = {'id':ident, 'seq':self.intent.seq, 'time':now}
      self.intent.inflight = self.intent.seq, ident, now
    self.state = f'{state} · L={lamps & 1} R={(lamps >> 1) & 1}' if authenticated else 'verifying board key'
    self.publish_badge(state if authenticated else 'auth', now, lamps, pending[1] if pending and pending[0]==ack else None)

  async def enroll(self):
    # Setup is automatic while parked. Only an explicitly selected device can
    # break a tie. Scanning never takes ownership of a HID remote.
    setup = read_json(SETUP, {})
    if setup.get('disabled') or not parked() or not radio_present() or not await radio_enabled():
      return
    await self.client.scan()
    await asyncio.sleep(5)
    if not parked():
      return
    snapshot = await self.client.snapshot()
    selected = setup.get('selected')
    candidates = [d for d in snapshot['devices'] if compatible(d) and (d.get('rssi') is not None or d.get('connected'))]
    if selected:
      candidates = [d for d in candidates if d['address']==selected]
    if len(candidates)!=1:
      self.state = 'multiple Carrot-B1 devices' if candidates else 'scanning for Carrot-B1'
      return
    device = candidates[0]
    mac = address(device['address'])
    if not device.get('paired'):
      await self.client.start_pair(mac, headless=True)
      await self.client.pair_task
      if self.client.pair.get('state') != 'paired':
        self.state = 'waiting for encrypted pairing'
        return
    if not parked():
      return
    for _ in range(10):
      await self.discover(mac)
      if self.attrs or not parked():
        break
      await asyncio.sleep(.5)
    if not parked() or not self.attrs or CLAIM not in self.attrs:
      return
    # Save the proposed key before writing anything to the board. Retrying
    # after power loss uses the same seed until the board proves ownership.
    staged = read_json(STAGED, {})
    if staged.get('address') != mac or not isinstance(staged.get('seed'),str) or len(staged['seed'])!=32:
      staged = {'address':mac,'seed':secrets.token_hex(16)}
      atomic_json(STAGED, staged)
    seed = bytes.fromhex(staged['seed'])
    key = hashlib.sha256(seed).digest()
    async def report():
      raw = await self.client.call(self.attrs[META], GATT, 'ReadValue', 'a{sv}', ({},))
      return struct.unpack('<IIHBBBH',bytes(raw[0]))
    nonce, *_ = await report()
    if not nonce or not parked():
      return
    await self.client.call(self.attrs[CLAIM], GATT, 'WriteValue', 'aya{sv}',
                           (struct.pack('<I',nonce)+seed, {'type':('s','request')}))
    await asyncio.sleep(.12)
    sid, stamp, _, _, _, version, seq = await report()
    if sid != nonce or version!=2:
      return
    seq = (seq+1)%65536 or 1
    await self.client.call(self.attrs[CONTROL], GATT, 'WriteValue', 'aya{sv}',
                           (command(key, sid, stamp, seq, 3, .35), {'type':('s','request')}))
    await asyncio.sleep(.12)
    proof = await report()
    if proof[0]==sid and proof[-1]==seq and parked() and read_json(SETUP,{})==setup:
      atomic_json(ENROLLMENT, {'address':mac,'name':'Carrot-B1','key':key.hex()})
    else:
      self.state = 'board already enrolled or first-use window closed'

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
          if not registration and now >= self.next_discovery:
            self.next_discovery = now + 30
            try:
              await self.enroll()
            except Exception:
              self.state = 'waiting for name discovery'
            finally:
              await self.release()
          elif registration:
            self.state = 'waiting for radio'
          await asyncio.sleep(.5)
          continue
        try:
          if self.attrs:
            async with self.exchange_lock:
              if maintenance_active():
                self.stop_source()
                self.intent.inflight = self.intent.queued = None
                self.state = 'firmware update'
                self.publish_badge('updating', time.monotonic(), 0, None)
              else:
                await self.exchange(registration[1])
            await asyncio.sleep(.035 if self.state != 'firmware update' else .15)
          else:
            self.state = await self.discover(registration[0])
            await asyncio.sleep(.5 if self.owned else 3)
        except Exception:
          await self.release()
          self.state = 'waiting to reconnect'
          await asyncio.sleep(2)
    finally:
      await self.release()
