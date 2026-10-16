import ast
from pathlib import Path
import queue
import socket
import struct
import sys
import threading
import time
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / 'cluster'
sys.path.insert(0, str(CLUSTER))
import cluster_wifi_protocol as protocol
from cluster_wifi_display import S3WifiDisplay

CONFIG = {'version': 1, 'device': '010203040506', 'key': '42' * 32}


def test_capability_query_rejects_forgery_replay_and_wrong_device(monkeypatch):
  server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  server.bind(('127.0.0.1', 0))
  server.settimeout(3)
  monkeypatch.setattr(protocol, 'PORT', server.getsockname()[1])
  key = bytes.fromhex(CONFIG['key'])
  errors = []

  def serve():
    try:
      request, peer = server.recvfrom(128)
      assert request[:4] == protocol.CAP_QUERY
      assert request[26:] == protocol.mac(key, request[:26])
      reply = protocol.CAP_REPLY + request[4:26] + struct.pack('!I', protocol.CAP_H264)
      server.sendto(reply + bytes(32), peer)
      for wrong in (reply[:10] + bytes(16) + reply[26:], reply[:4] + bytes(6) + reply[10:]):
        server.sendto(wrong + protocol.mac(key, wrong), peer)
      server.sendto(reply + protocol.mac(key, reply), peer)
    except BaseException as exc:
      errors.append(exc)
    finally:
      server.close()

  thread = threading.Thread(target=serve)
  thread.start()
  assert protocol.capabilities(CONFIG, targets=('127.0.0.1',)) == protocol.CAP_H264
  thread.join(3)
  assert not errors and not thread.is_alive()


@pytest.mark.parametrize('codec,supported,want', [('auto', 0, 'jpeg'), ('auto', 1, 'h264'), ('jpeg', 1, 'jpeg'), ('h264', 1, 'h264')])
def test_codec_selection_preserves_old_firmware(monkeypatch, codec, supported, want):
  monkeypatch.setattr(protocol, 'capabilities', lambda config: supported)
  assert protocol.select_codec({**CONFIG, 'codec': codec}) == want


def test_explicit_h264_cannot_use_unconfirmed_firmware(monkeypatch):
  monkeypatch.setattr(protocol, 'capabilities', lambda config: 0)
  with pytest.raises(RuntimeError, match='1.2.0'):
    protocol.select_codec({**CONFIG, 'codec': 'h264'})


def test_auto_encoder_can_fall_back_to_jpeg_without_restarting_failed_h264(monkeypatch):
  monkeypatch.setattr(protocol, 'capabilities', lambda config: protocol.CAP_H264)
  assert protocol.select_codec(CONFIG, jpeg_fallback=True) == 'jpeg'
  assert protocol.select_codec({**CONFIG, 'codec': 'h264'}, jpeg_fallback=True) == 'h264'


def test_codec_registration_roundtrip_and_invalid_value(tmp_path):
  path = tmp_path / 'config.json'
  protocol.save_config({**CONFIG, 'codec': 'h264'}, path)
  assert protocol.load_config(path)['codec'] == 'h264'
  with pytest.raises(RuntimeError):
    protocol.save_config({**CONFIG, 'codec': 'hevc'}, path)
  assert protocol.load_config(path)['codec'] == 'h264'


def test_h264_uses_wifi_without_usb_endpoints_and_preserves_order():
  display = S3WifiDisplay(CONFIG)
  sent = []
  received = []
  display._build_h264_chunk_payload = lambda chunk, **kw: chunk
  display.connection = SimpleNamespace(
    send=lambda data, **kw: sent.append((data, kw)) or len(sent),
    result=lambda: (received.append(len(received) + 1) or received[-1], b'ok', 100),
    close=lambda: None,
  )
  for i in range(4):
    display.send_h264_chunk(bytes([i]) * 512)
  assert len(received) == 1 and len(display.pending) == 3
  display.send_h264_chunk(b'last' * 128, wait_for_ack=True)
  assert [r[0][0] for r in sent[:4]] == [0, 1, 2, 3]
  assert not display.pending and len(received) == 5
  assert sent[0][1]['expect_reply'] is False and sent[-1][1]['expect_reply'] is True
  assert display._ep_out is None and display._ep_in is None


def test_h264_shutdown_stops_decoder_before_sync_and_off():
  display = S3WifiDisplay(CONFIG)
  events = []
  display.connection = SimpleNamespace(close=lambda: events.append('close'))
  display._h264_active = True
  display.stop_h264_stream = lambda: events.append('stop')
  display._send_command = lambda command, *a, **kw: events.append(command)
  display.close()
  assert events == ['stop', 10, 14, 'close'] and not display._h264_active


def wifi_queue_fixture():
  source = CLUSTER / 'cluster_h264_pipeline.py'
  tree = ast.parse(source.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'H264UsbPipeline')
  node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_enqueue_native_access_unit')
  scope = {'queue': queue, 'time': time, '_NativeAccessUnit': lambda chunks, codec_config, keyframe: (chunks, codec_config, keyframe)}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
  pending = queue.Queue(3)
  obj = SimpleNamespace(
    _closing=False, _packet_queue=pending, usb_display=SimpleNamespace(h264_preserve_reference_frames=True),
    _record_h264_queue_depth=lambda depth: None, check_error=lambda: None,
  )
  send = scope['_enqueue_native_access_unit']
  for i in range(3):
    send(obj, [bytes([i])], codec_config=i == 0, keyframe=i == 1)
  return obj, pending, send


def test_wifi_native_queue_preserves_references_across_short_network_stall():
  obj, pending, send = wifi_queue_fixture()
  output = []
  def consume():
    time.sleep(0.6)  # Beyond the old 250ms restart threshold.
    output.append(pending.get_nowait()[0][0])
  thread = threading.Thread(target=consume)
  thread.start()
  send(obj, [b'fourth'], codec_config=False, keyframe=False)
  thread.join(2)
  assert not thread.is_alive()
  output.extend(pending.get_nowait()[0][0] for _ in range(3))
  assert output == [b'\0', b'\1', b'\2', b'fourth']


def test_wifi_native_queue_remains_bounded_on_dead_sender():
  obj, pending, send = wifi_queue_fixture()
  with pytest.raises(RuntimeError, match='reconnect required') as caught:
    send(obj, [b'fourth'], codec_config=False, keyframe=False)
  assert isinstance(caught.value.__cause__, ConnectionError)
  assert 'reference queue stalled for 3s' in str(caught.value.__cause__)
  assert [pending.get_nowait()[0][0] for _ in range(3)] == [b'\0', b'\1', b'\2']


def test_wifi_native_queue_wait_stops_on_close_or_transport_error():
  for closing in (True, False):
    obj, pending, send = wifi_queue_fixture()
    def cancel(closing=closing, obj=obj):
      if closing:
        obj._closing = True
      else:
        raise ConnectionError('sender failed')
    obj.check_error = cancel
    if closing:
      send(obj, [b'fourth'], codec_config=False, keyframe=False)
    else:
      with pytest.raises(ConnectionError, match='sender failed'):
        send(obj, [b'fourth'], codec_config=False, keyframe=False)
    assert pending.qsize() == 3


def test_native_sender_exits_when_closed_without_queue_sentinel():
  source = CLUSTER / 'cluster_h264_pipeline.py'
  tree = ast.parse(source.read_text())
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'H264UsbPipeline')
  node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_send_queued_packets')
  scope = {'queue': queue}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
  obj = SimpleNamespace(_packet_queue=queue.Queue(3), chunk_size=65536, _closing=False, _condition=threading.Condition(), _error=None)
  thread = threading.Thread(target=scope['_send_queued_packets'], args=(obj,))
  thread.start()
  obj._closing = True
  thread.join(1)
  assert not thread.is_alive() and obj._error is None


def test_h264_stop_is_not_repeated_on_close(monkeypatch):
  from cluster_usb_display import TuringUsbDisplay

  events = []
  monkeypatch.setattr(TuringUsbDisplay, 'stop_h264_stream', lambda _: events.append('stop'))
  display = S3WifiDisplay(CONFIG)
  display.connection = SimpleNamespace(close=lambda: events.append('close'))
  display._h264_active = True
  display._send_command = lambda command, *a, **kw: events.append(command)
  display.stop_h264_stream()
  display.close()
  assert events == ['stop', 10, 14, 'close']
