"""Session-authenticated, encrypted WLAN HUD transport, isolated from inference.

Separate snapshot/media TCP streams; stop-and-wait bounded records, no USB fallback.
The key only arrives through an inherited local socket, never argv/status/telemetry.
"""
import ctypes
import hmac
import ipaddress
import json
import os
from pathlib import Path
import secrets
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

from Crypto.Cipher import AES

from hud_protocol import HUD_PACKET, MAX_HUD_BYTES, HEADER as SNAPSHOT_HEADER
from hud_navi import CHUNK, HEADER as MEDIA_HEADER, HostForwarder
from hud_wifi_route import select_link

MAGIC = b'CHW1'
HEADER = struct.Struct('!QIf')  # sequence, plaintext length, source age seconds
LIMITS = {1: MAX_HUD_BYTES, 2: CHUNK + MEDIA_HEADER.size}
BUDGET = .4
STATUS = Path('/dev/shm/carrot-jetlink-hud-wifi.json')


def read_exact(sock, size, deadline):
  data = bytearray()
  while len(data) < size:
    left = deadline - time.monotonic()
    if left <= 0:
      raise TimeoutError('HUD record deadline')
    sock.settimeout(left)
    part = sock.recv(size - len(data))
    if not part:
      raise ConnectionError('HUD connection closed')
    data.extend(part)
  return bytes(data)


def send_exact(sock, data, deadline):
  left = deadline - time.monotonic()
  if left <= 0:
    raise TimeoutError('HUD send deadline')
  sock.settimeout(left)
  sock.sendall(data)


def authenticate(sock, key, channel=None):
  deadline = time.monotonic() + BUDGET
  if channel is not None:
    hello = MAGIC + bytes([channel]) + secrets.token_bytes(32)
    send_exact(sock, hello, deadline)
    answer = read_exact(sock, 64, deadline)
    context = hello + answer[:32]
    if not hmac.compare_digest(answer[32:], hmac.digest(key, b'server' + context, 'sha256')):
      raise ValueError('HUD peer authentication')
    send_exact(sock, hmac.digest(key, b'client' + context, 'sha256'), deadline)
  else:
    hello = read_exact(sock, 37, deadline)
    if hello[:4] != MAGIC or hello[4] not in LIMITS:
      raise ValueError('HUD protocol')
    channel = hello[4]
    nonce = secrets.token_bytes(32)
    context = hello + nonce
    send_exact(sock, nonce + hmac.digest(key, b'server' + context, 'sha256'), deadline)
    signature = read_exact(sock, 32, deadline)
    if not hmac.compare_digest(signature, hmac.digest(key, b'client' + context, 'sha256')):
      raise ValueError('HUD client authentication')
  return channel, hmac.digest(key, b'records' + context, 'sha256')


def seal(key, sequence, direction, header, data):
  cipher = AES.new(key, AES.MODE_GCM, nonce=struct.pack('!IQ', direction, sequence))
  cipher.update(header)
  encrypted, tag = cipher.encrypt_and_digest(data)
  return encrypted + tag


def unseal(key, sequence, direction, header, data):
  cipher = AES.new(key, AES.MODE_GCM, nonce=struct.pack('!IQ', direction, sequence))
  cipher.update(header)
  return cipher.decrypt_and_verify(data[:-16], data[-16:])


def wlan_addresses():
  rows = json.loads(subprocess.check_output(['ip', '-j', '-4', 'addr', 'show'], timeout=1., text=True))
  result = []
  for row in rows:
    if not row['ifname'].startswith('wl'):
      continue
    for item in row.get('addr_info', []):
      address = ipaddress.ip_address(item.get('local', '0.0.0.0'))
      if address.version == 4 and address.is_private and not (address.is_loopback or address.is_link_local or address.is_unspecified):
        result.append((row['ifname'], str(address)))
  return result


def target_addresses():
  value = json.loads(Path('/dev/shm/carrot-jetlink.json').read_text())
  if not 0 <= time.monotonic() - value['updated'] < 3 or value.get('peer', {}).get('carrot_host') != 'jetson':
    return ()
  if not 0 <= time.monotonic() - value.get('telemetry_updated', 0.) < 5:
    return ()
  result = []
  for item in value.get('telemetry', {}).get('carrot_health', {}).get('addresses', []):
    address = ipaddress.ip_address(item['address'])
    if (item['interface'].startswith('wl') and address.version == 4 and address.is_private
        and not (address.is_loopback or address.is_link_local or address.is_unspecified)):
      result.append(str(address))
  return result


class Runtime:
  def __init__(self, role, key, port, control):
    self.role, self.key, self.port, self.control = role, key, port, control
    self.stop = threading.Event()
    self.network = ()
    self.link = None
    self.state = {1: 'waiting_wifi', 2: 'waiting_wifi'}
    self.bytes = {1: 0, 2: 0}
    self.channel_locks = {1: threading.Lock(), 2: threading.Lock()}
    self.slots = threading.BoundedSemaphore(4)
    self.threads = []
    self.publisher = None

  def start_thread(self, fn, *args):
    thread = threading.Thread(target=fn, args=args, daemon=True)
    self.threads = [t for t in self.threads if t.is_alive()]
    self.threads.append(thread)
    thread.start()

  def run(self):
    if self.role == 'send':
      from hud_protocol import Publisher
      self.publisher = Publisher(navi=True, nav_map=True)
      for channel in LIMITS:
        self.start_thread(self.sender, channel)
    else:
      self.start_thread(self.listener)
    self.control.send(b'ready')
    try:
      next_network = 0.
      while not self.stop.is_set():
        if select.select([self.control], [], [], .1)[0]:
          if not self.control.recv(64):
            break
        if time.monotonic() >= next_network:
          try:
            self.network = tuple(wlan_addresses())
            self.link = select_link(self.network, target_addresses()) if self.role == 'send' else None
          except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
            self.network, self.link = (), None
          next_network = time.monotonic() + 1.
          # Diagnostic status contains no key, payload, vehicle data or SSID.
          temporary = STATUS.with_suffix('.tmp')
          try:
            temporary.write_text(json.dumps({'updated': time.monotonic(), 'role': self.role,
                                            'snapshot': self.state[1], 'media': self.state[2], 'bytes': self.bytes,
                                            'interface': self.link[0] if self.link else None}))
            os.replace(temporary, STATUS)
          except OSError:
            pass
    finally:
      self.stop.set()
      for thread in self.threads:
        thread.join(timeout=1.)
      if self.publisher is not None:
        self.publisher.close()
      if self.role == 'receive':
        HUD_PACKET.unlink(missing_ok=True)
      STATUS.unlink(missing_ok=True)

  def sender(self, channel):
    while not self.stop.is_set():
      self.state[channel] = 'waiting_wifi'
      if channel == 2:
        # Never replay navigation fragments queued while the WLAN was absent.
        # The producer socket is bounded; draining is also bounded per retry.
        for _ in range(64):
          if not self.publisher.media_packet():
            break
      link = self.link
      if not link:
        self.stop.wait(.2)
        continue
      try:
        interface, source, target = link
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
          sock.settimeout(BUDGET)
          sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
          sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 64 * 1024)
          sock.bind((source, 0))
          sock.connect((target, self.port))
          _, key = authenticate(sock, self.key, channel)
          sequence, heartbeat = 0, 0.
          media_start = True
          while not self.stop.is_set() and self.link == link:
            data = self.publisher.packet() if channel == 1 else self.publisher.media_packet()
            if data and channel == 2 and media_start:
              if len(data) <= MEDIA_HEADER.size or MEDIA_HEADER.unpack_from(data)[2] != 0:
                continue
              media_start = False
            age = 0.
            if data and channel == 1:
              age = time.monotonic() - json.loads(data)['sent']
              if not 0 <= age < .3:
                continue
            if not data and time.monotonic() < heartbeat:
              # Snapshot generation is 10 Hz; do not re-read/decode the same
              # tmpfs file 100 times/s. Media wakes on its local socket instead
              # of polling, preserving ordered H.264 fragments without busy work.
              delay = min(.05 if channel == 1 else .1, max(0., heartbeat - time.monotonic()))
              media_socket = getattr(self.publisher, 'media_socket', None)
              if channel == 2 and media_socket is not None:
                select.select([media_socket], [], [], delay)
              else:
                self.stop.wait(delay)
              continue
            data = data or b''
            if len(data) > LIMITS[channel]:
              raise ValueError('HUD record too large')
            sequence += 1
            header = HEADER.pack(sequence, len(data), age)
            deadline = time.monotonic() + BUDGET
            send_exact(sock, header + seal(key, sequence, 0, header, data), deadline)
            ack = read_exact(sock, 16, deadline)
            unseal(key, sequence, 1, header, ack)
            self.state[channel] = 'connected'
            self.bytes[channel] += len(data)
            heartbeat = time.monotonic() + .1
      except (OSError, ValueError, KeyError, TypeError, subprocess.SubprocessError):
        self.state[channel] = 'reconnecting'
        self.stop.wait(.2)

  def listener(self):
    while not self.stop.is_set():
      network = self.network
      if not network:
        self.stop.wait(.2)
        continue
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
          listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
          listener.bind((network[0][1], self.port))
          listener.listen(4)
          listener.settimeout(.2)
          while not self.stop.is_set() and self.network == network:
            try:
              sock, _ = listener.accept()
            except TimeoutError:
              continue
            if self.slots.acquire(blocking=False):
              self.start_thread(self.receiver, sock, network)
            else:
              sock.close()
      except OSError:
        self.stop.wait(.5)

  def receiver(self, sock, network):
    lock = None
    forwarder = None
    channel = None
    try:
      with sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        channel, key = authenticate(sock, self.key)
        candidate = self.channel_locks[channel]
        if not candidate.acquire(blocking=False):
          return
        lock = candidate
        forwarder = HostForwarder() if channel == 2 else None
        sequence, previous = 0, time.monotonic()
        while not self.stop.is_set() and self.network == network:
          deadline = previous + BUDGET
          header = read_exact(sock, HEADER.size, deadline)
          number, size, age = HEADER.unpack(header)
          if number != sequence + 1 or size > LIMITS[channel] or not 0 <= age < .3:
            raise ValueError('HUD record contract')
          data = unseal(key, number, 0, header, read_exact(sock, size + 16, deadline))
          now = time.monotonic()
          elapsed = now - previous  # Conservative bound since previous acknowledged record.
          if data and channel == 1:
            if json.loads(data).get('version') != 1:
              raise ValueError('HUD snapshot version')
            if age + elapsed < .5:
              temporary = HUD_PACKET.with_suffix('.wifi.tmp')
              temporary.write_bytes(SNAPSHOT_HEADER.pack(now - age - elapsed) + data)
              os.replace(temporary, HUD_PACKET)
          elif data:
            forwarder.send(data)
          send_exact(sock, seal(key, number, 1, header, b''), deadline)
          sequence, previous = number, time.monotonic()
          self.state[channel] = 'connected'
          self.bytes[channel] += len(data)
    except (OSError, ValueError, KeyError, TypeError):
      pass
    finally:
      if forwarder is not None:
        forwarder.close()
      if lock is not None:
        lock.release()
        self.state[channel] = 'waiting_wifi'
      self.slots.release()


def main():
  role, fd = sys.argv[1], int(sys.argv[2])
  if role not in ('send', 'receive'):
    raise ValueError('HUD worker role')
  ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0)
  if os.getppid() == 1:
    return
  os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
  # Only this display worker and its children are demoted. Never lower the
  # USB/IPC owner's priority or use realtime scheduling for display traffic.
  if role == 'send':
    os.setpriority(os.PRIO_PROCESS, 0, 19)
  else:
    os.nice(5)
  control = socket.socket(fileno=fd)
  control.settimeout(2.)
  config = json.loads(control.recv(512))
  key = bytes.fromhex(config['key'])
  if len(key) != 32 or config['port'] != 7716:
    raise ValueError('HUD worker configuration')
  runtime = Runtime(role, key, config['port'], control)
  signal.signal(signal.SIGTERM, lambda *_: runtime.stop.set())
  try:
    runtime.run()
  finally:
    control.close()


if __name__ == '__main__':
  main()
