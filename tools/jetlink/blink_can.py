"""Experimental PEAK/B1 manual one-touch output. Fresh input/CAN guard, no retries."""
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import time

from blink_protocol import CONFIG, PROFILE, LIMIT, configuration, encode, finite

FRAME = struct.Struct('=IB3x8s')
TIMINGS = {'left': (0., .037063, .086550, .217974, .257611, .297840),
           'right': (0., .036767, .076723, .113991, .137291, .167372, .210180)}


def crc(data):
  value = 0
  for byte in data:
    value ^= byte
    for _ in range(8):
      value = ((value << 1) ^ (0x1d if value & 0x80 else 0)) & 255
  return value ^ 0x65


def inactive(data):
  return len(data) == 8 and not (data[3] & 0x40 or data[4] & 0x51) and crc(data[1:]) == data[0]


def lamps(data):
  if len(data) != 8:
    raise ValueError('invalid lamp feedback')
  return [bool(data[7] & 8), bool(data[7] & 32)] if data[7] & 64 else [bool(data[2] & 16), bool(data[2] & 64)]


def frames(original, direction, counter):
  if not inactive(original) or direction not in TIMINGS or type(counter) is not int or not 0 <= counter < 15:
    raise ValueError('invalid observed B1 state')
  tap, release = bytearray(original), bytearray(original)
  tap[1] = ((counter+1) % 15 << 4) | (tap[1] & 15)
  tap[3] = (tap[3] & ~0x40) | (0x40 if direction == 'left' else 0)
  tap[4] = (tap[4] & ~0x51) | (0x10 if direction == 'left' else 0x41)
  release[1] = ((counter+2) % 15 << 4) | (release[1] & 15)
  tap[0], release[0] = crc(tap[1:]), crc(release[1:])
  count = 3 if direction == 'left' else 4
  return [(offset, bytes(tap if i < count else release)) for i, offset in enumerate(TIMINGS[direction])]


def identity(iface):
  if not iface.startswith('can') or not iface[3:].isdigit():
    raise ValueError('invalid CAN interface')
  path = Path('/sys/class/net') / iface
  usb = (path/'device').resolve().parent
  if ((path/'device/driver').resolve().name != 'peak_usb' or
      (usb/'idVendor').read_text().strip() != '0c72' or (usb/'idProduct').read_text().strip() != '000c'):
    raise ValueError('profile requires the verified PEAK PCAN-USB; native CAN is not supported')
  return (path/'ifindex').read_text().strip()


WATCHDOG = '''import os,pathlib,select,subprocess,sys
p=pathlib.Path('/sys/class/net')/sys.argv[1]
while select.select([0],[],[],0.75)[0]:
 if not os.read(0,256):break
if p.exists() and (p/'ifindex').read_text().strip()==sys.argv[2] and (p/'device/driver').resolve().name=='peak_usb':
 subprocess.run(['ip','link','set','dev',sys.argv[1],'down'],timeout=3)
 subprocess.run(['ip','link','set','dev',sys.argv[1],'type','can','listen-only','on','one-shot','off'],timeout=3)
'''


class Bus:
  def __init__(self, iface):
    import fcntl
    self.iface, self.sock, self.watch = iface, None, None
    self.latest = {}
    self.lock = (CONFIG.parent/'blinker.lock').open('a')
    fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    self.index = identity(iface)
    status = json.loads(self.run('ip', '-json', 'link', 'show', 'dev', iface))[0]
    if 'UP' in status['flags']:
      self.lock.close()
      raise ValueError('CAN interface already in use; refusing to take ownership')
    self.watch = subprocess.Popen(['sudo', '-n', 'python3', '-c', WATCHDOG, iface, self.index],
                                  stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    self.next_lease = 0.
    try:
      self.mode(False)
    except BaseException:
      self.close()
      raise

  @staticmethod
  def run(*args):
    return subprocess.run(args, capture_output=True, text=True, timeout=3, check=True).stdout

  def mode(self, transmit):
    if identity(self.iface) != self.index:
      raise ValueError('CAN adapter changed')
    if self.sock is not None:
      self.sock.close()
      self.sock = None
    self.run('sudo', '-n', 'ip', 'link', 'set', 'dev', self.iface, 'down')
    self.run('sudo', '-n', 'ip', 'link', 'set', 'dev', self.iface, 'type', 'can', 'bitrate', '500000',
             'listen-only', 'off' if transmit else 'on', 'one-shot', 'on' if transmit else 'off', 'restart-ms', '0')
    info = json.loads(self.run('ip', '-details', '-json', 'link', 'show', 'dev', self.iface))[0]['linkinfo']['info_data']
    mode = info.get('ctrlmode', [])
    if (info['bittiming']['bitrate'] != 500000 or ('ONE-SHOT' in mode) != transmit or
        ('LISTEN-ONLY' in mode) == transmit or 'LOOPBACK' in mode):
      raise ValueError('CAN mode did not match the bounded transmission contract')
    self.run('sudo', '-n', 'ip', 'link', 'set', 'dev', self.iface, 'up')
    self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0)
    self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b''.join(struct.pack('=II', n, 0xc00007ff) for n in (0x3c1, 0x413)))
    self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack('=I', 0x1fffffff))
    self.sock.bind((self.iface,))
    self.sock.setblocking(False)

  def receive(self):
    for _ in range(64):
      try:
        raw = self.sock.recv(FRAME.size)
      except BlockingIOError:
        break
      ident, length, data = FRAME.unpack(raw)
      if ident & 0x20000000 or length != 8:
        raise ValueError('CAN error or unexpected feedback length')
      self.latest[ident] = (time.monotonic(), data)
    now = time.monotonic()
    if now >= self.next_lease:
      self.watch.stdin.write(b'1')
      self.watch.stdin.flush()
      self.next_lease = now+.1

  def neutral(self, now):
    if any(k not in self.latest or not 0 <= now-self.latest[k][0] < .4 for k in (0x3c1, 0x413)):
      return None
    original, feedback = self.latest[0x3c1][1], self.latest[0x413][1]
    return original if inactive(original) and not any(lamps(feedback)) and not feedback[1] & 5 else None

  def send(self, data):
    if self.sock.send(FRAME.pack(0x3c1, 8, data)) != FRAME.size:
      raise OSError('incomplete CAN write')

  def close(self):
    if self.sock is not None:
      self.sock.close()
      self.sock = None
    try:
      if identity(self.iface) == self.index:
        self.run('sudo', '-n', 'ip', 'link', 'set', 'dev', self.iface, 'down')
        self.run('sudo', '-n', 'ip', 'link', 'set', 'dev', self.iface, 'type', 'can', 'listen-only', 'on', 'one-shot', 'off')
    finally:
      if self.watch is not None:
        self.watch.stdin.close()
        self.watch.wait(timeout=8)
      self.lock.close()


class Controller:
  def __init__(self, bus, token, state_path, clock=time.monotonic):
    from collections import deque
    self.bus, self.token, self.state_path, self.clock = bus, token, state_path, clock
    self.seen = deque(maxlen=256)
    self.pending = self.queued = self.owned = None
    self.guard_deadline = self.ready_after = 0.
    self.started = clock()
    self.status = {'ready': False, 'state': 'waiting', 'sent': 0, 'accepted_id':None}

  def unchanged(self, now):
    current = self.bus.latest.get(0x3c1)
    return (self.owned is not None and now < self.owned['until'] and current is not None and
            0 <= now-current[0] < .4 and current[1] == self.owned['original'])

  def accept(self, value):
    now = self.clock()
    if not isinstance(value, dict) or value.get('token') != self.token:
      return
    deadline = value.get('guard_deadline')
    self.guard_deadline = deadline if value.get('ok') is True and finite(deadline) and 0 < deadline-now <= .35 else 0.
    connected = value.get('devices', [])
    if self.queued and self.queued.get('device') and self.queued['device'] not in connected:
      self.queued = None
    hold = value.get('hold')
    if self.queued and self.queued.get('hold') and (not isinstance(hold, dict) or hold.get('id') != self.queued['id']):
      self.queued = None
    events = list(value.get('events', [])[:4])
    if isinstance(hold, dict):
      events.append({**hold, 'hold':True})
    for event in events:
      if not isinstance(event, dict) or not isinstance(event.get('id'), str) or event['id'] in self.seen:
        continue
      self.seen.append(event['id'])
      expires = event.get('deadline')
      if (now-self.started < .6 or self.guard_deadline <= now or not finite(expires) or
          not 0 < expires-now <= .4 or event.get('direction') not in TIMINGS):
        continue
      if self.pending or self.unchanged(now):
        # Keep only the newest fresh intention while our previous one-touch ends.
        # Driver input, disconnect or stale vehicle state revokes this local wait.
        self.queued = {**event, 'until':now+3.2}
        self.status['state'] = 'waiting_previous_signal'
      elif self.bus.neutral(now) is not None:
        self.begin(event, now)

  def begin(self, event, now):
    original = self.bus.neutral(now)
    if original is None:
      return
    counter = original[1] >> 4
    try:
      saved = json.loads(self.state_path.read_text())
      if saved.get('neutral') == original.hex():
        counter = saved['counter']
    except (OSError, ValueError, KeyError):
      pass
    schedule = frames(original, event['direction'], counter)
    temporary = self.state_path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'neutral':original.hex(), 'counter':schedule[-1][1][1] >> 4}))
    os.replace(temporary, self.state_path)
    self.bus.mode(True)
    start = self.clock()
    if start >= min(event.get('until',event['deadline']), self.guard_deadline):
      self.bus.mode(False)
      return
    self.pending = {'schedule':schedule, 'original':original, 'start':start, 'index':0,
                    'direction':event['direction'], 'id':event['id']}
    self.owned = {'original':original, 'until':start+300}
    self.status.update(state='sending', direction=event['direction'], accepted_id=None)

  def abort(self, reason):
    if self.pending:
      self.bus.mode(False)
    self.pending = self.queued = self.owned = None
    self.status.update(state=reason, ready=False, accepted_id=None)

  def tick(self):
    now = self.clock()
    if now >= self.guard_deadline:
      self.abort('guard_expired')
      return
    if self.owned and not self.unchanged(now):
      self.abort('driver_override')
      return
    p = self.pending
    if p is not None:
      offset, data = p['schedule'][p['index']]
      if now >= p['start']+offset:
        if now-(p['start']+offset) >= .05:
          self.abort('late_request')
          return
        self.bus.send(data)
        self.status['sent'] += 1
        p['index'] += 1
        if p['index'] == len(p['schedule']):
          self.pending = None
          self.bus.mode(False)
          self.status.update(state='waiting', accepted_id=p['id'])
    if self.queued and (now >= self.queued['until'] or not self.unchanged(now)):
      self.queued = None
    if self.pending is None and self.queued and self.bus.neutral(now) is not None:
      event, self.queued = self.queued, None
      self.begin(event, now)
    self.status['ready'] = self.pending is None and self.bus.neutral(now) is not None
    if self.status['ready']:
      self.status['state'] = 'ready'
    elif self.queued:
      self.status['state'] = 'waiting_previous_signal'


def main(fd, token):
  sock = socket.socket(fileno=fd)
  sock.setblocking(False)
  bus = None
  def stop(*_):
    raise SystemExit
  signal.signal(signal.SIGTERM, stop)
  signal.signal(signal.SIGINT, stop)
  try:
    config = configuration()
    if config is None:
      return
    CONFIG.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    bus = Bus(config['interface'])
    controller = Controller(bus, token, CONFIG.parent/'blinker-counter.json')
    next_status = next_config = 0.
    while True:
      bus.receive()
      for _ in range(8):
        try:
          raw = sock.recv(LIMIT+1)
        except BlockingIOError:
          break
        if not raw:
          return
        if len(raw) <= LIMIT:
          controller.accept(json.loads(raw))
      controller.tick()
      now = time.monotonic()
      if now >= next_config:
        if configuration() != config:
          return
        next_config = now+1
      if now >= next_status:
        try:
          sock.send(encode({**controller.status, 'updated':now}))
        except BlockingIOError:
          pass
        next_status = now+.2
      select.select([sock, bus.sock], [], [], .005)
  except Exception as exc:
    try:
      sock.send(encode({'ready':False, 'state':'error', 'error':str(exc)[:200]}))
    except OSError:
      pass
  finally:
    if bus is not None:
      bus.close()
    sock.close()


if __name__ == '__main__':
  if len(sys.argv) == 3 and sys.argv[1] == '--configure':
    identity(sys.argv[2])
    CONFIG.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    CONFIG.write_text(json.dumps({'enabled':True, 'profile':PROFILE, 'interface':sys.argv[2]}, indent=2)+'\n')
    os.chmod(CONFIG, 0o600)
    print('Configured manual PCAN-USB/B1 profile. Reconnect the Jetlink session to activate.')
  else:
    main(int(sys.argv[1]), sys.argv[2])
