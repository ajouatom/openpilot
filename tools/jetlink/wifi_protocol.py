"""Private USB provisioning channel, independent of HUD and model readiness.

Physical USB attachment authorizes importing the attached comma's Wi-Fi profiles.
Never put this payload into Params, HUD snapshots, telemetry, or diagnostic logs.
"""
import json
import os
from pathlib import Path
import re
import socket
import struct
import subprocess
import sys
import time

CAPABILITY = 'carrot_wifi_v1'
MESSAGE = 0x4002
LIMIT = 16384
PACKET = Path('/dev/shm/carrot-jetlink-wifi.packet')
CONTROL = Path('/dev/shm/carrot-jetlink-bootstrap.json')
HEADER = struct.Struct('<d')


def validate(value):
  if not isinstance(value, dict) or value.get('version') != 1:
    raise ValueError('Invalid provisioning version')
  if not isinstance(value.get('owner'), str) or not re.fullmatch('[0-9a-f]{64}', value['owner']):
    raise ValueError('Invalid provisioning owner')
  profiles = value.get('profiles')
  if not isinstance(profiles, list) or len(profiles) > 8:
    raise ValueError('Invalid profile count')
  seen = set()
  for p in profiles:
    if not isinstance(p, dict) or set(p) != {'id', 'ssid', 'security', 'password', 'hidden', 'active'}:
      raise ValueError('Invalid profile fields')
    if not isinstance(p['id'], str) or not re.fullmatch('[0-9a-f-]{36}', p['id']) or p['id'] in seen:
      raise ValueError('Invalid profile identifier')
    seen.add(p['id'])
    if not isinstance(p['ssid'], str) or not 1 <= len(p['ssid'].encode()) <= 32:
      raise ValueError('Invalid SSID')
    if not isinstance(p['password'], str) or any(c in p['ssid'] + p['password'] for c in '\x00\r\n'):
      raise ValueError('Invalid profile strings')
    if type(p['hidden']) is not bool or type(p['active']) is not bool:
      raise ValueError('Invalid hidden flag')
    security, password = p['security'], p['password']
    if security == 'open':
      valid = password == ''
    elif security == 'wpa-psk':
      valid = 8 <= len(password.encode()) <= 63 or bool(re.fullmatch('[0-9a-fA-F]{64}', password))
    elif security == 'sae':
      valid = 1 <= len(password.encode()) <= 63
    else:
      valid = False
    if not valid:
      raise ValueError('Unsupported Wi-Fi security or invalid secret')
  if value.get('onroad') is not None and type(value['onroad']) is not bool:
    raise ValueError('Invalid road state')
  return value


def private_write(path, data):
  temporary = path.with_suffix('.new')
  fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC | getattr(os, 'O_NOFOLLOW', 0), 0o600)
  with os.fdopen(fd, 'wb') as stream:
    os.chmod(temporary, 0o600)
    stream.write(data)
  os.replace(temporary, path)


def receive(payload):
  if not 0 < len(payload) <= LIMIT:
    return
  try:
    value = validate(json.loads(bytes(payload)))
    stamp = time.monotonic()
    private_write(PACKET, HEADER.pack(stamp) + bytes(payload))
    # Only this secret-free subset is visible to the updater before an engine
    # is ready. A missing/invalid onroad flag never authorizes downloading.
    control = {k: value[k] for k in ('onroad', 'jetson_release') if k in value}
    private_write(CONTROL, json.dumps(dict(control, received=stamp)).encode())
  except (OSError, ValueError, TypeError):
    pass


def read_packet():
  try:
    with PACKET.open('rb') as stream:
      data = stream.read(LIMIT + HEADER.size + 1)
    if not HEADER.size < len(data) <= LIMIT + HEADER.size:
      return None
    stamp, = HEADER.unpack_from(data)
    if not 0 <= time.monotonic() - stamp < 15:
      return None
    return stamp, validate(json.loads(data[HEADER.size:]))
  except (OSError, ValueError, TypeError, struct.error):
    return None


class Publisher:
  def __init__(self):
    self.socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    self.socket.setblocking(False)
    try:
      self.process = subprocess.Popen(['chrt', '--other', '0', sys.executable,
        str(Path(__file__).with_name('wifi_publisher.py')), str(child.fileno())],
        pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    finally:
      child.close()

  def send(self, client, initial=False):
    import select
    if initial:
      select.select([self.socket], [], [], 4)
    latest = None
    for _ in range(4):
      try:
        data = self.socket.recv(LIMIT + HEADER.size + 1)
        if HEADER.size < len(data) <= LIMIT + HEADER.size:
          latest = data
      except BlockingIOError:
        break
    if latest is not None and 0 <= time.monotonic() - HEADER.unpack_from(latest)[0] < 3:
      client.t.send(MESSAGE, client._next_seq(), [latest[HEADER.size:]])

  def close(self):
    self.process.terminate()
    try:
      self.process.wait(timeout=1)
    except subprocess.TimeoutExpired:
      self.process.kill()
      self.process.wait(timeout=1)
    self.socket.close()
