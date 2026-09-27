import json
import os
from pathlib import Path
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import wifi_protocol as protocol
import wifi_apply as host
import wifi_publisher as sender


def config(owner='a', password='synthetic pass'):
  return dict(version=1, owner=owner * 64, onroad=False,
              profiles=[dict(id='12345678-1234-1234-1234-123456789abc',
                             ssid=' synthetic:한글 wifi ', security='wpa-psk',
                             password=password, hidden=False, active=True)])


def test_private_channel_excludes_credentials_from_update_control(tmp_path, monkeypatch):
  monkeypatch.setattr(protocol, 'PACKET', tmp_path/'wifi.packet')
  monkeypatch.setattr(protocol, 'CONTROL', tmp_path/'control.json')
  value = config()
  value['jetson_release'] = {'synthetic': True}
  protocol.receive(json.dumps(value).encode())
  assert protocol.read_packet()[1] == value
  control = json.loads(protocol.CONTROL.read_text())
  assert set(control) == {'onroad', 'received', 'jetson_release'}
  if sys.platform != 'win32':
    assert protocol.PACKET.stat().st_mode & 0o777 == 0o600
  monkeypatch.setattr(protocol.time, 'monotonic', lambda: control['received'] + 16)
  assert protocol.read_packet() is None


@pytest.mark.parametrize('change', [dict(ssid='a\n[wifi-security]'), dict(password='bad\nsecret'),
                                  dict(security='wpa-eap'), dict(hidden=1), dict(id='../../x')])
def test_invalid_or_unsupported_profiles_are_rejected(change):
  value = config()
  value['profiles'][0].update(change)
  with pytest.raises(ValueError):
    protocol.validate(value)


def test_move_to_another_comma_and_password_change_preserves_manual_profiles(tmp_path):
  manual = tmp_path/'personal.nmconnection'
  manual.write_text('untouched')
  calls = []
  run = lambda *args: calls.append(args)
  first = host.install(config(), tmp_path, run)
  path = next(tmp_path.glob('carrot-usb-*'))
  assert 'ssid=\\ssynthetic:한글\\swifi\\s' in path.read_text()
  host.install(config(password='changed synthetic'), tmp_path, run)
  assert 'psk=changed\\ssynthetic' in path.read_text()
  second = host.install(config('b'), tmp_path, run)
  assert first != second and not path.exists()
  assert ('connection', 'delete', 'uuid', first[0]) in calls
  assert manual.read_text() == 'untouched'
  count = len(calls)
  host.install(config('b'), tmp_path, run)
  assert len(calls) == count  # Periodic retransmission does not reconnect/rewrite.


def test_open_and_sae_and_psk_hex():
  for security, password in [('open', ''), ('sae', 'short'), ('wpa-psk', 'f' * 64)]:
    value = config(password=password)
    value['profiles'][0]['security'] = security
    protocol.validate(value)
    _, data = host.render(value['owner'], value['profiles'][0], 100)
    assert ('[wifi-security]' in data) == (security != 'open')


def test_sender_uses_saved_secrets_and_excludes_hotspot(monkeypatch):
  uid = config()['profiles'][0]['id']
  hotspot = '22345678-1234-1234-1234-123456789abc'
  def nm(*args):
    if args[-1] == '--active':
      return uid
    if args[-1] == 'show':
      return f'{hotspot}:802-11-wireless\n{uid}:802-11-wireless'
    if args[-1] == hotspot:
      return 'hotspot\nap\nno\nwpa-psk\nsynthetic'
    assert '--show-secrets' in args
    return 'phone\ninfrastructure\nno\nwpa-psk\nsynthetic'
  monkeypatch.setattr(sender, 'nm', nm)
  profiles = sender.collect()
  assert len(profiles) == 1 and profiles[0]['id'] == uid


def test_connected_wifi_is_not_interrupted():
  calls = []
  def nm(*args):
    calls.append(args)
    return 'wlan0:wifi:connected'
  assert host.connect(['synthetic'], nm) == 'connected'
  assert len(calls) == 1


def test_comma_active_network_change_is_applied():
  calls = []
  def nm(*args):
    calls.append(args)
    if args[-1] == 'status':
      return 'wlan0:wifi:connected'
    if args[-1] == '--active':
      return 'previous'
    return ''
  assert host.connect(['new', 'previous'], nm, preferred='new') == 'connected'
  assert ('--wait', '12', 'connection', 'up', 'uuid', 'new') in calls


def test_unavailable_network_retries_without_leaking_output():
  def nm(*args):
    if args[0] == '--wait':
      raise RuntimeError('NetworkManager operation failed')
    return 'wlan0:wifi:disconnected'
  assert host.connect(['synthetic'], nm) == 'retrying'


def test_bootstrap_can_stage_before_model_and_fails_closed(tmp_path, monkeypatch):
  import update_host as update
  import hud_protocol
  control = tmp_path/'control.json'
  real_read = Path.read_text
  monkeypatch.setattr(Path, 'read_text', lambda p, *a, **k:
    real_read(control) if p.name == 'carrot-jetlink-bootstrap.json' else real_read(p, *a, **k))
  monkeypatch.setattr(hud_protocol, 'read_snapshot', lambda: pytest.fail('fresh bootstrap must win'))
  calls = []
  monkeypatch.setattr(update, 'stage_manifest', calls.append)
  for onroad in (True, None, False):
    control.write_text(json.dumps(dict(received=time.monotonic(), onroad=onroad, jetson_release={'pin': 1})))
    update.automatic_stage()
  assert calls == [{'pin': 1}]


def test_failed_profile_load_is_retried(tmp_path):
  def fail(*args):
    raise RuntimeError('synthetic failure')
  with pytest.raises(RuntimeError):
    host.install(config(), tmp_path, fail)
  assert not list(tmp_path.glob('carrot-usb-*'))
  calls = []
  host.install(config(), tmp_path, lambda *args: calls.append(args))
  assert len(calls) == 1


@pytest.mark.parametrize('raw,expected', [(True, True), (False, False), (b'0', False),
                                         (b'1', True), ('0', False), ('1', True), (None, None), ('', None)])
def test_typed_and_legacy_road_state(raw, expected):
  assert sender.road_state(raw) is expected


def test_stale_sender_buffer_is_never_forwarded(monkeypatch):
  from types import SimpleNamespace
  class Socket:
    def __init__(self, packet):
      self.packet = packet
    def recv(self, size):
      if self.packet is None:
        raise BlockingIOError()
      data, self.packet = self.packet, None
      return data
  publisher = object.__new__(protocol.Publisher)
  publisher.socket = Socket(protocol.HEADER.pack(time.monotonic() - 10) + b'stale')
  client = SimpleNamespace(t=SimpleNamespace(send=lambda *a: pytest.fail('stale packet sent')))
  publisher.send(client)


def test_server_accepts_provisioning_without_engine(tmp_path, monkeypatch):
  from types import SimpleNamespace
  from server import CarrotSession
  monkeypatch.setattr(protocol, 'PACKET', tmp_path/'wifi.packet')
  monkeypatch.setattr(protocol, 'CONTROL', tmp_path/'control.json')
  session = object.__new__(CarrotSession)
  session.last_seq = 0
  session.handle(SimpleNamespace(msg_type=protocol.MESSAGE, seq=1, payload=memoryview(json.dumps(config()).encode())))
  assert protocol.read_packet()[1] == config()
  # Replaying an earlier USB sequence cannot overwrite the current owner.
  session.handle(SimpleNamespace(msg_type=protocol.MESSAGE, seq=1, payload=memoryview(json.dumps(config('b')).encode())))
  assert protocol.read_packet()[1]['owner'] == 'a' * 64
