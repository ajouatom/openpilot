import importlib.util
import ast
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


def test_wifi_geometry_is_available_before_opening_idle_timed_connection(monkeypatch):
  monkeypatch.setattr('cluster_wifi_display.Connection', lambda *a: pytest.fail('connected before renderer ready'))
  display = S3WifiDisplay(CONFIG)
  display.set_orientation(1)
  assert (display.landscape_width, display.landscape_height) == (1920, 462)
  assert display.connection is None and display.dev is None
  assert display.open_after_renderer
  assert not getattr(TuringUsbDisplay(), 'open_after_renderer', False)


@pytest.mark.parametrize('allowed', [True, False])
def test_registration_wakes_usb_wait_without_device_event(tmp_path, monkeypatch, allowed):
  path = tmp_path / 'pair.json'
  monkeypatch.setenv('CARROT_S3_CONFIG', str(path))
  source = CLUSTER.parent / 'cluster_autorun.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_wait_for_supported_usb_device')
  clock, closed = [0.0], []
  def wait(*args):
    protocol.save_config(CONFIG, path)
    clock[0] += 1
    return False
  scope = dict(Params=object, time=SimpleNamespace(monotonic=lambda: clock[0]),
               _read_hud_mode=lambda params: 1, _hud_output_allowed=lambda params: allowed or clock[0] == 0,
               _open_usb_uevent_socket=lambda: SimpleNamespace(close=lambda: closed.append(True)),
               _wait_for_usb_uevent=wait, USB_FALLBACK_SCAN_INTERVAL_S=30, HUD_CHECK_INTERVAL_S=1,
               HUD_PARAM='ClusterHud', HUD_DEBUG_PARAM='ClusterHudDebug', IS_ONROAD_PARAM='IsOnroad')
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
  monkeypatch.setattr('cluster_usb_display.find_supported_usb_product', lambda *a: pytest.fail('unnecessary USB scan'))
  assert scope['_wait_for_supported_usb_device'](object(), 0x0092, 'test') is None
  assert clock[0] == 1 and closed == [True]


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


def test_automatic_uart_selection_refuses_ambiguity_without_opening_ports(monkeypatch):
  path = Path(__file__).resolve().parents[4] / 'tools/esp32s3_display/provision.py'
  spec = importlib.util.spec_from_file_location('s3_auto_provision', path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  ports = []
  monkeypatch.setattr('serial.tools.list_ports.comports', lambda: ports)
  with pytest.raises(RuntimeError, match='UART candidates: none'):
    module.automatic_port()
  ports.extend([SimpleNamespace(vid=0x10c4, device='COM14'), SimpleNamespace(vid=0x9999, device='COM15')])
  assert module.automatic_port() == 'COM14'
  ports.append(SimpleNamespace(vid=0x1a86, device='COM16'))
  with pytest.raises(RuntimeError, match='COM14, COM16'):
    module.automatic_port()


@pytest.mark.parametrize('size', [0, 511, protocol.MAX_PAYLOAD + 1])
def test_payload_bounds_before_socket_write(size):
  connection = protocol.Connection.__new__(protocol.Connection)
  with pytest.raises(ValueError):
    connection.send(bytes(size))


@pytest.mark.parametrize('fail', [False, True])
def test_close_syncs_and_confirms_off_but_releases_transport_on_failure(monkeypatch, fail):
  display = S3WifiDisplay(CONFIG)
  events = []
  display.connection = SimpleNamespace(close=lambda: events.append('closed'))
  def command(number, name, fields=None, **kwargs):
    events.append((number, fields))
    assert kwargs.get('expect_response', True)
    if fail and number == 14:
      raise RuntimeError('USB response lost')
  monkeypatch.setattr(display, '_send_command', command)
  monkeypatch.setattr('cluster_wifi_display.time.sleep', lambda delay: events.append(delay))
  display.close()
  assert events == [(10, None), .2, (14, {8: 0}), 'closed']
  assert display.connection is None and not display.pending


@pytest.mark.parametrize('registered', [False, True])
def test_offroad_dims_registered_s3_without_local_usb(tmp_path, monkeypatch, registered):
  path = tmp_path / 'pair.json'
  monkeypatch.setenv('CARROT_S3_CONFIG', str(path))
  if registered:
    protocol.save_config(CONFIG, path)
  events = []
  def scan(pid):
    events.append('scan')
    return None
  class Display:
    def __init__(self, config, **kwargs):
      assert config == CONFIG and kwargs['brightness'] == 0
    def open(self): events.append('open-off')
    def close(self): events.append('close')
  monkeypatch.setattr('cluster_usb_display.find_supported_usb_product', scan)
  monkeypatch.setattr('cluster_wifi_display.S3WifiDisplay', Display)
  source = CLUSTER.parent / 'cluster_autorun.py'
  node = next(n for n in ast.parse(source.read_text(encoding='utf8')).body
              if isinstance(n, ast.FunctionDef) and n.name == '_turn_off_supported_usb_device')
  scope = {'_configure_autorun_locale': lambda: None}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
  assert scope[node.name](0x0092, 'test') is registered
  assert events == (['open-off', 'close'] if registered else ['scan'])
