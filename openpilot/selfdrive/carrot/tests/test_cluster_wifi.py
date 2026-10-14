import importlib.util
import json
from pathlib import Path
import socket
import sys
import threading
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / 'cluster'
sys.path.insert(0, str(CLUSTER))
import cluster_wifi_protocol as protocol
from cluster_wifi_display import S3WifiDisplay, create_display
from cluster_usb_display import TuringUsbDisplay

CONFIG = {'version': 1, 'device': '010203040506', 'key': '42' * 32}
KEY = bytes.fromhex(CONFIG['key'])


def serve_once(handler):
  server = socket.socket()
  server.bind(('127.0.0.1', 0))
  server.listen()
  errors = []
  def run():
    try:
      conn, _ = server.accept()
      with conn:
        conn.settimeout(3)
        handler(conn)
    except Exception as exc:
      errors.append(exc)
    finally:
      server.close()
  thread = threading.Thread(target=run, daemon=True)
  thread.start()
  return server.getsockname()[1], thread, errors


def server_auth(sock, key=KEY):
  hello = protocol.HELLO + bytes.fromhex(CONFIG['device']) + b'c' * 32
  for part in (hello[:3], hello[3:11], hello[11:]):
    sock.sendall(part)
  client = protocol.receive(sock, 64)
  assert client[32:] == protocol.mac(key, b'client' + hello + client[:32])
  sock.sendall(protocol.mac(key, b'server' + hello + client[:32]))
  return protocol.mac(key, b'session' + hello + client[:32])


def test_authenticated_fragmented_packets_and_usb_reply():
  payload = b'p' * 60000
  def handler(sock):
    key = server_auth(sock)
    header = protocol.receive(sock, 24)
    signature = protocol.receive(sock, 32)
    data = protocol.receive(sock, len(payload))
    assert signature == protocol.mac(key, header + data)
    assert data == payload
    assert protocol.HEADER.unpack(header) == (protocol.MAGIC, 1, len(payload), 1, 0, 0)
    reply = b'\x65\xc8' + bytes(510)
    ack = protocol.HEADER.pack(protocol.ACK, 1, 512, 0, 50000, 0)
    packet = ack + protocol.mac(key, ack + reply) + reply
    for pos in range(0, len(packet), 17):
      sock.sendall(packet[pos:pos + 17])
  port, thread, errors = serve_once(handler)
  connection = protocol.Connection(CONFIG, '127.0.0.1', port)
  assert connection.send(payload) == 1
  seq, response, usb_us = connection.result()
  assert (seq, response[:2], usb_us) == (1, b'\x65\xc8', 50000)
  connection.close()
  thread.join(3)
  assert not thread.is_alive() and not errors


@pytest.mark.parametrize('fault', ['signature', 'oversize', 'status', 'truncated'])
def test_rejects_bad_ack(fault):
  def handler(sock):
    key = server_auth(sock)
    if fault == 'truncated':
      sock.sendall(b'bad')
      return
    header = protocol.HEADER.pack(protocol.ACK, 1, 513 if fault == 'oversize' else 0,
                                  1 if fault == 'status' else 0, 0, 0)
    sock.sendall(header + (bytes(32) if fault == 'signature' else protocol.mac(key, header)))
  port, thread, errors = serve_once(handler)
  connection = protocol.Connection(CONFIG, '127.0.0.1', port)
  with pytest.raises((ConnectionError, OSError)):
    connection.result()
  connection.close()
  thread.join(3)
  assert not errors


def test_wrong_server_key_is_rejected():
  def handler(sock):
    hello = protocol.HELLO + bytes.fromhex(CONFIG['device']) + bytes(32)
    sock.sendall(hello)
    protocol.receive(sock, 64)
    sock.sendall(bytes(32))
  port, thread, errors = serve_once(handler)
  with pytest.raises(ConnectionError, match='authentication'):
    protocol.Connection(CONFIG, '127.0.0.1', port)
  thread.join(3)
  assert not errors


def test_discovery_rejects_unsigned_wrong_device_and_replayed_nonce(monkeypatch):
  server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  server.bind(('127.0.0.1', 0))
  monkeypatch.setattr(protocol, 'PORT', server.getsockname()[1])
  def run():
    request, peer = server.recvfrom(128)
    assert request[26:] == protocol.mac(KEY, request[:26])
    reply = protocol.FOUND + request[4:26]
    server.sendto(reply + bytes(32), peer)
    old = reply[:10] + bytes(16)
    server.sendto(old + protocol.mac(KEY, old), peer)
    server.sendto(reply + protocol.mac(KEY, reply), peer)
    server.close()
  thread = threading.Thread(target=run, daemon=True)
  thread.start()
  assert protocol.discover(CONFIG, targets=('127.0.0.1',)) == '127.0.0.1'
  thread.join(3)


def test_registration_is_atomic_and_never_copies_wifi_password(tmp_path):
  path = tmp_path / 'pair.json'
  protocol.save_config({**CONFIG, 'password': 'private', 'ssid': 'private'}, path)
  assert protocol.load_config(path) == CONFIG
  assert 'private' not in path.read_text()
  with pytest.raises(RuntimeError):
    protocol.save_config({**CONFIG, 'key': 'bad'}, path)
  assert protocol.load_config(path) == CONFIG
  assert list(tmp_path.iterdir()) == [path]


def test_default_usb_and_registered_wifi_factory(tmp_path, monkeypatch):
  path = tmp_path / 'pair.json'
  monkeypatch.setenv('CARROT_S3_CONFIG', str(path))
  assert type(create_display()) is TuringUsbDisplay
  protocol.save_config(CONFIG, path)
  assert isinstance(create_display(), S3WifiDisplay)
  path.write_text('{}')
  with pytest.raises(RuntimeError):
    create_display()


@pytest.mark.parametrize('data', [[], None, {**CONFIG, 'fps': 1.5}, {**CONFIG, 'quality': True}])
def test_registration_rejects_invalid_shape_and_noninteger_policy(tmp_path, data):
  path = tmp_path / 'pair.json'
  path.write_text(json.dumps(data))
  with pytest.raises(RuntimeError, match='Invalid S3 registration'):
    protocol.load_config(path)


def test_three_frame_window_drains_before_settings_and_disconnect():
  display = S3WifiDisplay(CONFIG)
  sent, acked = [], []
  def send(data, **kwargs):
    sent.append(data)
    return len(sent)
  def result():
    acked.append(len(acked) + 1)
    return acked[-1], b'ok', 100
  display.connection = SimpleNamespace(send=send, result=result, close=lambda: None)
  for _ in range(4):
    display._transaction(bytes(512), frame=True)
  assert len(sent) == 4 and len(acked) == 1 and len(display.pending) == 3
  assert display._transaction(bytes(512)) == b'ok'
  assert len(acked) == 5 and not display.pending
  display.connection.result = lambda: (_ for _ in ()).throw(ConnectionError('lost'))
  with pytest.raises(RuntimeError, match='disconnected'):
    display._transaction(bytes(512))
  assert display.connection is None


def test_provisioning_validates_bytes_and_does_not_reset_uart():
  path = Path(__file__).resolve().parents[4] / 'tools/esp32s3_display/provision.py'
  spec = importlib.util.spec_from_file_location('s3_provision', path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  module.validate_wifi('한글 WiFi', '12345678')
  for ssid, password in [('한' * 11, '12345678'), ('ok', 'short'), ('x\0y', '12345678')]:
    with pytest.raises(ValueError):
      module.validate_wifi(ssid, password)
  source = path.read_text()
  assert 'port.dtr = port.rts = False' in source


@pytest.mark.parametrize('size', [0, 511, protocol.MAX_PAYLOAD + 1])
def test_payload_bounds_before_socket_write(size):
  connection = protocol.Connection.__new__(protocol.Connection)
  with pytest.raises(ValueError):
    connection.send(bytes(size))
