import json
import socket
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
sys.path.insert(0, str(Path(__file__).parent))
import hud_wifi as policy
import hud_wifi_worker as wire
from hud_wifi_route import select_link


@pytest.mark.parametrize('capability', [None, False, True])
def test_jetson_never_uses_usb_display_even_on_setup_failure(monkeypatch, capability):
  legacy = Mock(side_effect=AssertionError('USB HUD selected'))
  monkeypatch.setattr(policy, 'WifiPublisher', Mock(side_effect=OSError('offline')))
  peer = {'carrot_host': 'jetson', 'carrot_hud_v1': True, 'carrot_navi_v1': True, policy.CAPABILITY: capability}
  assert policy.publisher(None, peer, legacy) is None
  legacy.assert_not_called()


def test_other_hosts_keep_their_existing_display(monkeypatch):
  legacy = Mock(return_value='legacy')
  assert policy.publisher(None, {'carrot_host': 'mac', 'carrot_hud_v1': True}, legacy) == 'legacy'
  legacy.assert_called_once_with(navi=False)


def test_explicit_usb_receiver_uses_navigation_and_model_preview(monkeypatch):
  legacy = Mock(return_value='usb')
  monkeypatch.setattr(policy, 'WifiPublisher', Mock(side_effect=AssertionError('Wi-Fi started')))
  peer = {'carrot_host': 'jetson', policy.USB_CAPABILITY: True, 'carrot_hud_v1': True,
          'carrot_navi_v1': True, 'carrot_navi_pump_v1': True}
  assert policy.publisher(None, peer, legacy) == 'usb'
  legacy.assert_called_once_with(navi=True, nav_map=True)


@pytest.mark.parametrize('missing', ['carrot_hud_v1', 'carrot_navi_v1', 'carrot_navi_pump_v1'])
def test_incomplete_usb_receiver_does_not_send_display(missing):
  legacy = Mock(side_effect=AssertionError('incomplete USB receiver'))
  peer = {'carrot_host': 'jetson', policy.USB_CAPABILITY: True, 'carrot_hud_v1': True,
          'carrot_navi_v1': True, 'carrot_navi_pump_v1': True}
  del peer[missing]
  assert policy.publisher(None, peer, legacy) is None


@pytest.mark.parametrize('negotiated', [False, True])
def test_jetson_rejects_both_usb_display_messages(tmp_path, monkeypatch, negotiated):
  import server
  monkeypatch.setattr(server, 'HUD_PACKET', tmp_path/'snapshot')
  session = object.__new__(server.CarrotSession)
  session.last_seq = 0
  session.usb_display = False
  session.hud_wifi = object() if negotiated else None
  session.navi = Mock()
  for seq, kind in enumerate((server.HUD_MESSAGE, server.NAVI_MESSAGE), 1):
    session.handle(NS(msg_type=kind, seq=seq, payload=b'old USB data'))
  assert session.last_seq == 2 and not (tmp_path/'snapshot').exists()
  session.navi.send.assert_not_called()


def test_real_authenticated_tcp_records_reject_tampering_and_replay(tmp_path, monkeypatch):
  monkeypatch.setattr(wire, 'HUD_PACKET', tmp_path/'snapshot')
  listener = socket.socket()
  listener.bind(('127.0.0.1', 0)); listener.listen(1)
  client = socket.create_connection(listener.getsockname())
  remote, _ = listener.accept(); listener.close()
  runtime = wire.Runtime('receive', b'x'*32, 7716, None)
  runtime.network = (('wlan0', '127.0.0.1'),)
  assert runtime.slots.acquire(False)
  thread = threading.Thread(target=runtime.receiver, args=(remote, runtime.network))
  thread.start()
  try:
    _, key = wire.authenticate(client, b'x'*32, 1)
    data = json.dumps({'version': 1, 'params': {}}).encode()
    header = wire.HEADER.pack(1, len(data), 0.)
    frame = header + wire.seal(key, 1, 0, header, data)
    client.sendall(frame)
    ack = wire.read_exact(client, 16, time.monotonic()+1)
    assert wire.unseal(key, 1, 1, header, ack) == b''
    assert (tmp_path/'snapshot').read_bytes()[wire.SNAPSHOT_HEADER.size:] == data
    with pytest.raises(ValueError):
      wire.unseal(key, 1, 0, header, frame[wire.HEADER.size:-1]+bytes([frame[-1]^1]))
    client.sendall(frame)
    client.settimeout(1)
    try:
      assert client.recv(1) == b''  # duplicate sequence closes the stream
    except ConnectionResetError:
      pass  # Windows resets a TCP connection closed with unread payload.
  finally:
    runtime.stop.set(); client.close(); thread.join(2)
  assert not thread.is_alive() and runtime.bytes[1] == len(data)


def test_private_ap_is_preferred_and_usb_route_is_rejected():
  networks = [('wlan0','192.168.1.2'), ('wljetson0','172.29.77.1')]
  def route(target, source):
    return [{'dev': 'wljetson0' if source.startswith('172.') else 'usb0'}]
  assert select_link(networks, ['172.29.77.2'], route) == ('wljetson0','172.29.77.1','172.29.77.2')
  assert select_link(networks[:1], ['172.29.77.2'], route) is None


def test_ap_creation_is_offroad_only(monkeypatch):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  import wifi_direct
  wireless = Mock()
  monkeypatch.setattr(wifi_direct, 'offroad', lambda: False)
  assert wifi_direct.AccessPoint(wireless).start() is False
  wireless.create.assert_not_called()


def test_subnet_conflicts_cannot_steal_existing_routes(monkeypatch):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  import wifi_direct
  assert wifi_direct.choose_subnet([{'dst':'172.29.77.0/24','dev':'wlan0'}]) == '172.29.78.0/24'
  with pytest.raises(StopIteration):
    wifi_direct.choose_subnet([{'dst':'172.29.0.0/16','dev':'wlan0'}, {'dst':'10.0.0.0/8','dev':'eth0'}])
