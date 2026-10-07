"""Safe auto arbitration; desktop framing is not a physical USB claim test."""
import json
import socket
import threading
from types import SimpleNamespace
from unittest.mock import Mock  # noqa: TID251 - mock only; pytest runs the tests

import pytest

from openpilot.common import jetlink_status
from openpilot.common.jetlink_peer import classify_auto_peer, may_provision
from openpilot.common.jetlink_status import host_label
from openpilot.selfdrive.modeld.jetlink import daemon, mobile
from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.transport.base import LinkError, LinkTimeout
from jetlink.transport.tcp import TcpTransport
from jetlink.transport.ffs import FfsTransport

MAC = {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_M4'}
ANDROID = {'protocol': 3, 'backend': 'litert', 'device': 'gpu-Tensor_G4'}
IPHONE = {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_A17_Pro'}
IPAD = {'protocol': 3, 'backend': 'ort', 'device': 'ane-Apple_M4'}
JETSON = {'protocol': 2, 'backend': 'trt', 'device': 'orin', 'carrot_host': 'jetson'}


def test_real_bootstrap_hello_is_accepted_before_wifi_and_signed_update(monkeypatch):
  import boot_update
  from openpilot.selfdrive.modeld.jetlink import startup
  sent = []
  bootstrap = boot_update.BootstrapSession(SimpleNamespace(send_json=lambda *args: sent.append(args)), None)
  bootstrap.handle(SimpleNamespace(msg_type=P.Msg.HELLO_REQ, seq=1))
  peer = sent[0][2]
  assert 'backend' not in peer and 'device' not in peer
  assert daemon.legacy_usb_peer(peer)
  assert peer[daemon.WIFI_CAPABILITY] is True
  actions = []
  client = SimpleNamespace(t=SimpleNamespace(send_json=lambda *args: actions.append('signed-release')),
                           _next_seq=lambda: 2, state=lambda: {'carrot_update': {'state': 'ready'}})
  wifi = SimpleNamespace(send=lambda *args: actions.append('wifi'))
  with pytest.raises(ConnectionError, match='reconnecting to runtime'):
    startup.wait_for_boot_update(client, peer, wifi, lambda: True, lambda *a, **kw: None)
  assert actions == ['signed-release', 'wifi']


@pytest.mark.parametrize('changes', [
  {'protocol': True}, {'protocol': 4}, {'carrot_host': 'mac'}, {'carrot_boot_update_v1': 'true'},
])
def test_incomplete_bootstrap_identity_is_not_accepted(changes):
  peer = {'protocol': 2, 'carrot_host': 'jetson', 'carrot_boot_update_v1': True, **changes}
  assert not daemon.legacy_usb_peer(peer)


@pytest.mark.parametrize('peer,path,expected', [
  (MAC, 'usb', 'usb'), (ANDROID, 'usb', 'android'),
  (IPHONE, 'tcp', 'ios'), (IPAD, 'tcp', 'ios'),
  (IPHONE, 'usb', None), (ANDROID, 'tcp', None), (JETSON, 'usb', None),
  (dict(MAC, carrot_host='jetson'), 'usb', None),
  (dict(ANDROID, protocol=2), 'usb', None),
  (dict(ANDROID, device='cpu-Tensor_G4'), 'usb', None),
  (dict(MAC, protocol=True), 'usb', None),
  ({}, 'usb', None), (None, 'usb', None),
])
def test_classify_only_supported_assertions_on_selected_path(peer, path, expected):
  assert classify_auto_peer(peer, path) == expected


def test_auto_labels_are_path_specific():
  assert host_label(MAC, 'auto') == 'MAC'
  assert host_label(ANDROID, 'auto') == 'Android'
  assert host_label(IPAD, 'ios') == 'iOS'
  assert may_provision(MAC, 'auto') and may_provision(ANDROID, 'auto')
  assert not may_provision(IPHONE, 'auto')


@pytest.mark.parametrize('peer,selected', [(MAC, 'usb'), (ANDROID, 'android'), (IPHONE, 'ios')])
def test_auto_usb2_only_recognized_apps_warn(peer, selected, caplog):
  daemon.validate_speed('high-speed', 'auto', peer=peer, selected=selected)
  assert 'USB 2' in caplog.text
  with pytest.raises(RuntimeError):
    daemon.validate_speed('full-speed', 'auto', peer=peer, selected=selected)
  with pytest.raises(RuntimeError):
    daemon.validate_speed('high-speed', 'usb', peer=peer)


@pytest.mark.parametrize('peer', [JETSON, {}, None, dict(MAC, device='cpu-Apple_M4')])
def test_auto_legacy_and_unknown_peers_keep_superspeed_requirement(peer):
  with pytest.raises(RuntimeError):
    daemon.validate_speed('high-speed', 'auto', peer=peer)
  daemon.validate_speed('super-speed', 'auto', peer=peer)


def test_auto_wait_never_opens_or_writes_bulk_or_closes_owner():
  owner = Mock()
  cable = SimpleNamespace(accept=Mock(side_effect=[LinkTimeout('not dialed'), LinkTimeout('still absent'), object()]))
  automatic = mobile.AutoTransport(owner, cable)
  for _ in range(2):
    with pytest.raises(LinkTimeout):
      automatic.accept(timeout=.01)
  transport, mode = automatic.accept(timeout=.01)
  assert mode == 'ios' and transport is not owner
  assert not owner.mock_calls
  assert 'No concurrent HELLO' in automatic.diagnostic


def test_sequential_grace_prioritizes_tcp_then_authorizes_only_one_usb(monkeypatch):
  now = [0.]
  monkeypatch.setattr(mobile.time, 'monotonic', lambda: now[0])
  owner = SimpleNamespace(_configured=Mock(return_value=True))
  cable = SimpleNamespace(accept=Mock(side_effect=LinkTimeout('no dial')))
  automatic = mobile.AutoTransport(owner, cable)
  with pytest.raises(LinkTimeout):
    automatic.accept()
  now[0] = 5.
  assert automatic.accept() == (owner, 'usb')
  with pytest.raises(LinkTimeout):
    automatic.accept()
  assert owner._configured.call_count == 1
  disabled = mobile.AutoTransport(owner, cable, allow_usb=False)
  now[0] = 100.
  with pytest.raises(LinkTimeout):
    disabled.accept()
  assert owner._configured.call_count == 1
  phone = object()
  cable.accept.side_effect = None
  cable.accept.return_value = phone
  assert disabled.accept() == (phone, 'ios')


def test_grace_expiry_never_opens_endpoints_before_configuration(monkeypatch):
  owner = SimpleNamespace(_configured=lambda: False)
  cable = SimpleNamespace(accept=Mock(side_effect=LinkTimeout('no dial')))
  automatic = mobile.AutoTransport(owner, cable)
  monkeypatch.setattr(mobile.time, 'monotonic', lambda: automatic.deadline + 1)
  with pytest.raises(LinkTimeout):
    automatic.accept()


def test_late_usb_retry_keeps_ncm_undisturbed_for_longer_grace(monkeypatch):
  now = [0.]
  monkeypatch.setattr(mobile.time, 'monotonic', lambda: now[0])
  owner = SimpleNamespace(_configured=Mock(return_value=True))
  cable = SimpleNamespace(accept=Mock(side_effect=LinkTimeout('no dial')))
  automatic = mobile.AutoTransport(owner, cable, grace=mobile.AutoTransport.RETRY_GRACE)
  now[0] = 29.
  with pytest.raises(LinkTimeout):
    automatic.accept()
  assert not owner._configured.called
  now[0] = 30.
  assert automatic.accept() == (owner, 'usb')


@pytest.mark.parametrize('failure', [False, True])
def test_usb_probe_bounds_hello_and_clears_budget_afterwards(failure):
  transport = SimpleNamespace()
  def hello(timeout):
    assert timeout == 3.
    assert transport._probe_deadline > daemon.time.monotonic()
    if failure:
      raise LinkTimeout('watchdog aborted USB write')
    return MAC
  client = SimpleNamespace(t=transport, hello=hello)
  choice = daemon.ProtocolChoice('auto')
  if failure:
    with pytest.raises(LinkTimeout):
      daemon.probe_usb(client, choice)
    assert choice.version == 2
  else:
    assert daemon.probe_usb(client, choice) == MAC
  assert transport._probe_deadline is None


def test_ffs_probe_caps_write_and_endpoint_readiness_without_unbind(monkeypatch):
  owner = daemon.CarrotTransport.__new__(daemon.CarrotTransport)
  owner._send_deadline = None
  owner._probe_deadline = 13.
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: 10.)
  assert owner._write_timeout(15.) == 3.
  receive = Mock(return_value=object())
  monkeypatch.setattr(FfsTransport, 'recv', receive)
  owner.recv(timeout=5.)
  receive.assert_called_once_with(3.)
  monkeypatch.setattr(daemon.time, 'monotonic', lambda: 14.)
  with pytest.raises(LinkTimeout):
    owner._configured()
  with pytest.raises(LinkTimeout):
    owner._write_timeout(15.)
  with pytest.raises(LinkTimeout):
    owner.recv(timeout=3.)
  assert not owner._wait_for_host_ready()


@pytest.mark.parametrize('data_role,attached', [('ufp', True), ('dfp', False), ('', False)])
def test_auto_requires_data_role_even_if_power_role_source(tmp_path, monkeypatch, data_role, attached):
  data, power = tmp_path / 'data', tmp_path / 'power'
  data.write_text(data_role)
  power.write_text('Source attached (high)\n')
  monkeypatch.setattr(daemon, 'DATA_ROLE', data)
  monkeypatch.setattr(daemon, 'ROLE', power)
  monkeypatch.setattr(daemon, 'TRANSPORT_MODE', 'auto')
  assert daemon.host_attached() is attached


def test_auto_fallback_only_when_data_role_attribute_missing(tmp_path, monkeypatch):
  power = tmp_path / 'power'
  monkeypatch.setattr(daemon, 'DATA_ROLE', tmp_path / 'absent')
  monkeypatch.setattr(daemon, 'ROLE', power)
  monkeypatch.setattr(daemon, 'TRANSPORT_MODE', 'auto')
  power.write_text('Source attached\n')
  assert daemon.host_attached()
  power.write_text('Sink attached\n')
  assert not daemon.host_attached()
  monkeypatch.setattr(daemon, 'DATA_ROLE', SimpleNamespace(read_text=Mock(side_effect=PermissionError('denied'))))
  power.write_text('Source attached\n')
  assert not daemon.host_attached()


def test_active_session_udc_disconnect_is_latched_for_platform_reset(tmp_path, monkeypatch):
  state = tmp_path / 'controller' / 'state'
  state.parent.mkdir()
  state.write_text('configured')
  monkeypatch.setattr(daemon, 'ACTIVE_UDC', 'controller')
  monkeypatch.setattr(daemon, 'SESSION_DETACHED', False)
  monkeypatch.setattr(daemon, '_role_attached', lambda: True)
  monkeypatch.setattr(daemon, 'Path', lambda value: tmp_path if value == '/sys/class/udc' else type(tmp_path)(value))
  assert daemon.host_attached()
  state.write_text('not attached')
  assert not daemon.host_attached()
  assert daemon.SESSION_DETACHED
  state.write_text('configured')
  assert daemon.host_attached()
  assert daemon.SESSION_DETACHED  # even a quick replug resets session history


def test_cleanup_logs_all_failures_without_throwing_and_requests_blocking(caplog):
  unsafe = SimpleNamespace(close=Mock(side_effect=RuntimeError('unsafe network ownership')))
  good = SimpleNamespace(close=Mock())
  error = daemon.close_resources(unsafe, good)
  assert 'Unsafe Jetlink cleanup' in error and 'offroad' in error
  assert 'unsafe network ownership' in caplog.text
  good.close.assert_called_once()
  assert daemon.close_resources(None, good) is None


@pytest.mark.parametrize('bound,reader_alive,remembered_udc,unsafe', [
  ('', False, 'controller', False), ('controller', False, 'controller', True),
  ('', True, 'controller', True), ('controller', False, None, True),
])
def test_native_owner_cleanup_is_verified_not_silently_retried(tmp_path, monkeypatch, bound, reader_alive, remembered_udc, unsafe):
  (tmp_path / 'UDC').write_text(bound)
  owner = daemon.CarrotTransport.__new__(daemon.CarrotTransport)
  owner.gadget, owner.bound_udc = str(tmp_path), remembered_udc
  owner._reader = SimpleNamespace(is_alive=lambda: reader_alive)
  monkeypatch.setattr(FfsTransport, 'close', lambda self: None)
  if unsafe:
    with pytest.raises(LinkError, match='refusing to rebind'):
      owner.close()
  else:
    owner.close()


def test_blocked_cleanup_is_visible_even_before_any_peer_hello(tmp_path, monkeypatch):
  report = tmp_path / 'link'
  monkeypatch.setattr(jetlink_status, 'LINK_STATUS', report)
  monkeypatch.setattr(jetlink_status, 'MODEL_STATUS', tmp_path / 'absent-model')
  monkeypatch.setattr(jetlink_status.time, 'monotonic', lambda: 10.)
  report.write_text(json.dumps({'state': 'blocked', 'updated': 10., 'transport': 'auto',
                                  'peer': None, 'error': 'Unsafe Jetlink cleanup'}))
  assert jetlink_status.badge() == ('Jetlink ERROR', 'error')
  assert jetlink_status.diagnostics()['reason'] == 'Unsafe Jetlink cleanup'


def test_real_socket_auto_claim_precedes_single_client_hello_and_reconnect():
  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server.bind(('127.0.0.1', 0))
  server.listen(1)
  server.settimeout(2)
  owner = Mock()
  owner.bound_udc = None
  errors = []

  def host(peer):
    remote = TcpTransport.connect('127.0.0.1', server.getsockname()[1], timeout=2)
    remote.protocol_version = 3
    try:
      request = remote.recv(timeout=2)
      assert request.msg_type == P.Msg.HELLO_REQ
      remote.send(P.Msg.HELLO_RESP, request.seq, [json.dumps(peer).encode()])
      with pytest.raises(LinkError):
        remote.recv(timeout=2)
    except BaseException as exc:
      errors.append(exc)
    finally:
      remote.close()

  try:
    # Each dial is a new client/session; the ep0 owner is unchanged.
    for peer in (IPHONE, IPAD):
      thread = threading.Thread(target=host, args=(peer,))
      thread.start()
      def accept(timeout):
        connection, address = server.accept()
        return mobile.CableTransport(connection, owner, 'loopback-test', address)
      automatic = mobile.AutoTransport(owner, SimpleNamespace(accept=accept))
      transport, selected = automatic.accept(timeout=2)
      transport.protocol_version = 3
      client = JetlinkClient(transport, name='auto-test')
      assert selected == 'ios'
      actual = client.hello(timeout=2)
      assert classify_auto_peer(actual, 'tcp') == 'ios'
      client.close()
      thread.join(timeout=3)
      assert not thread.is_alive()
    assert not errors
    assert not owner.mock_calls
  finally:
    server.close()
