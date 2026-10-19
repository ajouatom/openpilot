"""Version 1 S3 display transport. No Wi-Fi credentials travel on this protocol."""
from __future__ import annotations

import hashlib
import hmac
import json
import os
from pathlib import Path
import secrets
import socket
import struct
import time

PORT = 3333
MAX_PAYLOAD = 1024 * 1024
HEADER = struct.Struct('!6I')
MAGIC = 0x53335531
ACK = 0x53334131
DISCOVER = b'S3D1'
FOUND = b'S3R1'
HELLO = b'S3H1'
CAP_QUERY = b'S3C1'
CAP_REPLY = b'S3C2'
CAP_H264 = 1
DEFAULT_CONFIG = '/data/carrot/cluster_wifi.json'


def config_path() -> Path:
  return Path(os.environ.get('CARROT_S3_CONFIG', DEFAULT_CONFIG))


def load_config(path: Path | None = None) -> dict | None:
  path = path or config_path()
  if not path.exists():
    return None
  try:
    data = json.loads(path.read_text(encoding='utf8'))
    if not isinstance(data, dict):
      raise ValueError()
    if data.get('version') != 1 or len(bytes.fromhex(data['device'])) != 6 or len(bytes.fromhex(data['key'])) != 32:
      raise ValueError()
    fps, quality = data.get('fps', 15), data.get('quality', 50)
    if type(fps) is not int or type(quality) is not int or not 1 <= fps <= 20 or not 20 <= quality <= 90:
      raise ValueError()
    if data.get('codec', 'auto') not in ('auto', 'jpeg', 'h264'):
      raise ValueError()
    return data
  except (ValueError, KeyError, TypeError) as exc:
    raise RuntimeError('Invalid S3 registration file; register the display again') from exc


def save_config(data: dict, path: Path | None = None) -> None:
  path = path or config_path()
  # Whitelist fields: never persist the hotspot password on the display sender.
  clean = {k: data[k] for k in ('version', 'device', 'key')}
  for k in ('fps', 'quality', 'codec'):
    if k in data:
      clean[k] = data[k]
  path.parent.mkdir(parents=True, exist_ok=True)
  temp = path.with_name(path.name + '.' + secrets.token_hex(4) + '.tmp')
  try:
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w', encoding='utf8') as stream:
      json.dump(clean, stream)
      stream.flush()
      os.fsync(stream.fileno())
    load_config(temp)
    os.replace(temp, path)
  finally:
    temp.unlink(missing_ok=True)


def mac(key: bytes, message: bytes) -> bytes:
  return hmac.digest(key, message, hashlib.sha256)


def receive(sock: socket.socket, size: int) -> bytes:
  result = bytearray()
  while len(result) < size:
    chunk = sock.recv(size - len(result))
    if not chunk:
      raise ConnectionError('S3 display disconnected')
    result.extend(chunk)
  return bytes(result)


def discover(config: dict, timeout: float = 2.0, targets: tuple[str, ...] = ('255.255.255.255',)) -> str:
  key, device = bytes.fromhex(config['key']), bytes.fromhex(config['device'])
  nonce = secrets.token_bytes(16)
  request = DISCOVER + device + nonce
  request += mac(key, request)
  deadline = time.monotonic() + timeout
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for target in targets:
      sock.sendto(request, (target, PORT))
    while time.monotonic() < deadline:
      # Broadcasts can be lost during AP/DHCP transitions. Retry within the
      # existing deadline instead of making the entire HUD restart wait for
      # another lifecycle retry after just one missing discovery packet.
      sock.settimeout(max(.01, min(.25, deadline - time.monotonic())))
      try:
        reply, address = sock.recvfrom(128)
      except TimeoutError:
        if time.monotonic() < deadline:
          for target in targets:
            sock.sendto(request, (target, PORT))
        continue
      if len(reply) == 58 and reply[:26] == FOUND + device + nonce and hmac.compare_digest(reply[26:], mac(key, reply[:26])):
        return address[0]
  raise TimeoutError('Registered S3 display not found on this Wi-Fi')


def capabilities(config: dict, timeout: float = 1.0, targets: tuple[str, ...] = ('255.255.255.255',)) -> int:
  """Authenticated feature query. Older firmware safely remains JPEG-only."""
  key, device = bytes.fromhex(config['key']), bytes.fromhex(config['device'])
  nonce = secrets.token_bytes(16)
  request = CAP_QUERY + device + nonce
  request += mac(key, request)
  deadline = time.monotonic() + timeout
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    for target in targets:
      sock.sendto(request, (target, PORT))
    while time.monotonic() < deadline:
      sock.settimeout(max(.01, min(.25, deadline - time.monotonic())))
      try:
        reply, _ = sock.recvfrom(128)
      except TimeoutError:
        if time.monotonic() < deadline:
          for target in targets:
            sock.sendto(request, (target, PORT))
        continue
      if len(reply) == 62 and reply[:26] == CAP_REPLY + device + nonce and hmac.compare_digest(reply[30:], mac(key, reply[:30])):
        return struct.unpack('!I', reply[26:30])[0]
  return 0


def select_codec(config: dict, *, jpeg_fallback: bool = False) -> str:
  requested = config.get('codec', 'auto')
  if requested == 'jpeg' or (requested == 'auto' and jpeg_fallback):
    return 'jpeg'
  if capabilities(config) & CAP_H264:
    return 'h264'
  if requested == 'h264':
    raise RuntimeError('S3 H.264 support not confirmed; update firmware to 1.2.0 or check Wi-Fi')
  return 'jpeg'


class Connection:
  """Bounded, authenticated packet stream; callers own send/receive serialization."""
  def __init__(self, config: dict, host: str | None = None, port: int = PORT):
    self.sock = socket.create_connection((host or discover(config), port), timeout=3)
    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    self.sequence = 0
    try:
      hello = receive(self.sock, 42)
      key = bytes.fromhex(config['key'])
      if hello[:10] != HELLO + bytes.fromhex(config['device']):
        raise ConnectionError('S3 device identity mismatch')
      nonce = secrets.token_bytes(32)
      self.session_key = mac(key, b'session' + hello + nonce)
      self.sock.sendall(nonce + mac(key, b'client' + hello + nonce))
      if not hmac.compare_digest(receive(self.sock, 32), mac(key, b'server' + hello + nonce)):
        raise ConnectionError('S3 authentication failed')
    except BaseException:
      self.close()
      raise

  def send(self, payload: bytes, *, expect_reply: bool = True) -> int:
    if not 512 <= len(payload) <= MAX_PAYLOAD:
      raise ValueError('S3 payload must be 512..1048576 bytes')
    self.sequence += 1
    header = HEADER.pack(MAGIC, self.sequence, len(payload), int(expect_reply), 0, 0)
    self.sock.sendall(header + mac(self.session_key, header + payload) + payload)
    return self.sequence

  def result(self) -> tuple[int, bytes, int]:
    header = receive(self.sock, HEADER.size)
    magic, seq, size, status, usb_us, reserved = HEADER.unpack(header)
    if magic != ACK or size > 512 or reserved:
      raise ConnectionError('Invalid S3 acknowledgement')
    signature = receive(self.sock, 32)
    response = receive(self.sock, size)
    if not hmac.compare_digest(signature, mac(self.session_key, header + response)):
      raise ConnectionError('Invalid S3 acknowledgement signature')
    if status:
      raise ConnectionError(f'S3 USB transaction failed (status {status})')
    return seq, response, usb_us

  def close(self) -> None:
    try:
      self.sock.shutdown(socket.SHUT_RDWR)
    except OSError:
      pass
    self.sock.close()
