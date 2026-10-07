"""Desktop-only lifecycle/framing checks; not a C3/C4 or physical phone trial."""
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import Mock  # noqa: TID251 - mock only; pytest runs the tests
import uuid

import pytest

from openpilot.selfdrive.modeld.jetlink import mobile
from jetlink import protocol as P
from jetlink.transport.base import LinkError, LinkTimeout
from jetlink.transport.tcp import TcpTransport


@pytest.fixture
def scratch():
  # Do not let pytest's default tmp_path write outside this project.
  root = Path('.analysis/scratch') / f'mobile-tests-{uuid.uuid4().hex}'
  root.mkdir(parents=True)
  try:
    yield root.resolve()
  finally:
    shutil.rmtree(root)


@pytest.fixture
def cable(monkeypatch, scratch):
  monkeypatch.setenv('JETLINK_TRANSPORT', 'ios')
  gadget = scratch / 'gadget'
  function = gadget / 'functions' / mobile.NCM_FUNCTION
  function.mkdir(parents=True)
  (function / 'ifname').write_text('usb7\n')
  net = scratch / 'net'
  (net / 'usb7').mkdir(parents=True)
  (net / 'usb7' / 'ifindex').write_text('17\n')
  # A separate modem interface must not be guessed from its name.
  (net / 'usb0').mkdir()
  (net / 'usb0' / 'ifindex').write_text('1\n')
  owner = SimpleNamespace(bound_udc='test-udc', ep_in=-1, ep_out=-1, close=Mock(),
                          send=Mock(side_effect=AssertionError('FFS must not send in iOS')),
                          recv=Mock(side_effect=AssertionError('FFS must not receive in iOS')))
  root = Mock()
  listener = Mock()
  monkeypatch.setattr(mobile, '_root', root)
  monkeypatch.setattr(mobile.socket, 'socket', Mock(return_value=listener))
  return mobile.MobileCable(owner, gadget=gadget, net_class=net), root, listener


@pytest.mark.parametrize('env,expected', [
  ({}, 'auto'), ({'JETLINK_TRANSPORT': 'auto'}, 'auto'), ({'JETLINK_TRANSPORT': 'usb'}, 'usb'),
  ({'JETLINK_TRANSPORT': 'android'}, 'android'), ({'JETLINK_TRANSPORT': 'ios'}, 'ios'),
])
def test_explicit_mode(env, expected):
  assert mobile.transport_mode(env) == expected
  assert mobile.mode(env) == expected


@pytest.mark.parametrize('value', ['auto', 'usb', 'android', 'ios'])
def test_persistent_mode_and_environment_precedence(scratch, value):
  config = scratch / 'mode'
  config.write_text(value + '\n')
  assert mobile.transport_mode({}, config) == value
  assert mobile.mode({}, config) == value
  assert mobile.mode({'JETLINK_TRANSPORT': 'usb'}, config) == 'usb'


@pytest.mark.parametrize('value', ['', '\n', 'IOS\n', 'ios\nandroid\n'])
def test_invalid_persistent_mode(scratch, value):
  config = scratch / 'mode'
  config.write_text(value)
  with pytest.raises(ValueError, match='usb, android or ios'):
    mobile.mode({}, config)
  assert mobile.mode({'JETLINK_TRANSPORT': 'android'}, config) == 'android'


def test_missing_config_only_is_harmless(scratch, monkeypatch):
  assert mobile.mode({}, scratch / 'missing') == 'auto'
  read = Mock(side_effect=PermissionError('unreadable mode'))
  monkeypatch.setattr(Path, 'read_text', read)
  with pytest.raises(PermissionError, match='unreadable'):
    mobile.mode({}, scratch / 'mode')
  assert mobile.mode({'JETLINK_TRANSPORT': 'ios'}, scratch / 'mode') == 'ios'


def test_environment_invalid_override_is_not_hidden_by_file(scratch):
  config = scratch / 'mode'
  config.write_text('ios\n')
  with pytest.raises(ValueError):
    mobile.mode({'JETLINK_TRANSPORT': ''}, config)


@pytest.mark.parametrize('mode', ['', 'tcp', 'lan', 'IOS', ' ios ', None])
def test_invalid_mode_fails(mode):
  with pytest.raises(ValueError, match='usb, android or ios'):
    mobile.transport_mode({'JETLINK_TRANSPORT': mode})


@pytest.mark.parametrize('mode,call', [
  ('usb', ('setup_gadget.sh',)), ('android', ('setup_gadget.sh',)), ('ios', ('setup_mobile.sh', 'gadget')),
  ('auto', ('setup_mobile.sh', 'gadget')),
])
def test_setup_before_binding_preserves_default(monkeypatch, mode, call):
  root = Mock()
  monkeypatch.setattr(mobile, '_root', root)
  assert mobile.setup_gadget(mode) == mode
  root.assert_called_once_with(*call)


@pytest.mark.parametrize('value', ['usb', 'android'])
def test_plain_setup_removes_previous_owned_ncm_first(monkeypatch, scratch, value):
  gadget = scratch / 'gadget'
  (gadget / 'functions' / mobile.NCM_FUNCTION).mkdir(parents=True)
  (gadget / 'configs' / 'c.1').mkdir(parents=True)
  (gadget / 'configs' / 'c.1' / mobile.NCM_FUNCTION).symlink_to(gadget / 'functions' / mobile.NCM_FUNCTION)
  monkeypatch.setattr(mobile, 'GADGET', gadget)
  root = Mock()
  monkeypatch.setattr(mobile, '_root', root)
  mobile.setup_gadget(value)
  assert [entry.args for entry in root.call_args_list] == [
    ('setup_mobile.sh', '--teardown'), ('setup_gadget.sh',),
  ]


def test_plain_setup_does_not_continue_after_unowned_or_bound_ncm(monkeypatch, scratch):
  gadget = scratch / 'gadget'
  (gadget / 'functions' / mobile.NCM_FUNCTION).mkdir(parents=True)
  (gadget / 'configs' / 'c.1').mkdir(parents=True)
  (gadget / 'configs' / 'c.1' / mobile.NCM_FUNCTION).symlink_to(gadget / 'functions' / mobile.NCM_FUNCTION)
  monkeypatch.setattr(mobile, 'GADGET', gadget)
  root = Mock(side_effect=mobile.MobileLinkError('NCM is still bound'))
  monkeypatch.setattr(mobile, '_root', root)
  with pytest.raises(mobile.MobileLinkError, match='bound'):
    mobile.setup_gadget('usb')
  root.assert_called_once_with('setup_mobile.sh', '--teardown')


@pytest.mark.parametrize('value', ['usb', 'android'])
def test_plain_setup_leaves_retained_detached_ncm_alone(monkeypatch, scratch, value):
  gadget = scratch / 'gadget'
  (gadget / 'functions' / mobile.NCM_FUNCTION).mkdir(parents=True)
  monkeypatch.setattr(mobile, 'GADGET', gadget)
  root = Mock()
  monkeypatch.setattr(mobile, '_root', root)
  mobile.setup_gadget(value)
  root.assert_called_once_with('setup_gadget.sh')


def test_root_helper_uses_noninteractive_sudo_and_reports_stderr(monkeypatch):
  monkeypatch.setattr(mobile.os, 'geteuid', lambda: 1000)
  run = Mock(side_effect=subprocess.CalledProcessError(1, 'helper', stderr='kernel has no NCM'))
  monkeypatch.setattr(mobile.subprocess, 'run', run)
  with pytest.raises(mobile.MobileLinkError, match='kernel has no NCM'):
    mobile._root('setup_mobile.sh', 'net')
  args, kwargs = run.call_args
  assert args[0][:5] == ['sudo', '-n', 'env', 'JETLINK_TRANSPORT=ios', 'bash']
  assert args[0][-1] == 'net'
  assert kwargs['env']['JETLINK_TRANSPORT'] == 'ios'
  assert kwargs['check'] and kwargs['timeout'] == 15


@pytest.mark.parametrize('failure', [FileNotFoundError('sudo missing'), subprocess.TimeoutExpired('helper', 15)])
def test_missing_or_timed_out_helper(monkeypatch, failure):
  monkeypatch.setattr(mobile.subprocess, 'run', Mock(side_effect=failure))
  with pytest.raises(mobile.MobileLinkError, match='unavailable'):
    mobile._root('setup_mobile.sh', 'net')


def test_requires_explicit_opt_in_and_ep0_only(monkeypatch):
  owner = SimpleNamespace(bound_udc='udc', ep_in=-1, ep_out=-1)
  monkeypatch.setenv('JETLINK_TRANSPORT', 'usb')
  with pytest.raises(mobile.MobileLinkError, match='explicit'):
    mobile.MobileCable(owner)
  monkeypatch.setenv('JETLINK_TRANSPORT', 'ios')
  owner.bound_udc = None
  with pytest.raises(mobile.MobileLinkError, match='bind'):
    mobile.MobileCable(owner)
  owner.bound_udc, owner.ep_in = 'udc', 7
  with pytest.raises(mobile.MobileLinkError, match='bulk endpoints'):
    mobile.MobileCable(owner)


def test_auto_opens_only_ep0_and_root_helper_is_ios(monkeypatch):
  monkeypatch.setenv('JETLINK_TRANSPORT', 'auto')
  owner = SimpleNamespace(bound_udc='udc', ep_in=-1, ep_out=-1)
  assert mobile.MobileCable(owner).owner is owner
  monkeypatch.setattr(mobile.os, 'geteuid', lambda: 1000)
  run = Mock()
  monkeypatch.setattr(mobile.subprocess, 'run', run)
  mobile._root('setup_mobile.sh', 'gadget')
  args, kwargs = run.call_args
  assert args[0][:5] == ['sudo', '-n', 'env', 'JETLINK_TRANSPORT=ios', 'bash']
  assert kwargs['env']['JETLINK_TRANSPORT'] == 'ios'


def test_listener_is_address_and_actual_interface_isolated(cable):
  link, root, listener = cable
  assert link.start() is link
  root.assert_called_once_with('setup_mobile.sh', 'net')
  listener.setsockopt.assert_any_call(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'usb7\0')
  listener.bind.assert_called_once_with(('192.168.60.1', 5599))
  assert (link.ifname, link.ifindex) == ('usb7', 17)
  link.start()
  assert root.call_count == 1
  link.close()
  link.close()
  listener.close.assert_called_once()
  assert root.call_count == 2
  root.assert_called_with('setup_mobile.sh', 'net-down')
  link.owner.send.assert_not_called()
  link.owner.recv.assert_not_called()
  link.owner.close.assert_not_called()


def test_netdev_can_appear_only_after_bind(cable, monkeypatch):
  link, _, _ = cable
  path = link.gadget / 'functions' / mobile.NCM_FUNCTION / 'ifname'
  path.unlink()
  sleeps = []

  def appear(delay):
    sleeps.append(delay)
    path.write_text('usb7\n')

  monkeypatch.setattr(mobile.time, 'sleep', appear)
  link.start()
  assert len(sleeps) == 1
  link.close()


def test_unsupported_or_absent_ncm_times_out(cable):
  link, root, listener = cable
  (link.gadget / 'functions' / mobile.NCM_FUNCTION / 'ifname').unlink()
  with pytest.raises(mobile.MobileLinkError, match='CONFIG_USB_CONFIGFS_NCM'):
    link.start(timeout=.001)
  root.assert_not_called()
  listener.bind.assert_not_called()


@pytest.mark.parametrize('where', ['network', 'socket'])
def test_failed_start_cleans_dhcp_and_keeps_ep0(cable, where):
  link, root, listener = cable
  if where == 'network':
    root.side_effect = [mobile.MobileLinkError('DHCP failed'), None]
  else:
    listener.bind.side_effect = PermissionError('not permitted')
  with pytest.raises(mobile.MobileLinkError):
    link.start()
  root.assert_called_with('setup_mobile.sh', 'net-down')
  assert link.listener is None and not link._network_started
  link.owner.close.assert_not_called()


def test_cleanup_failure_is_not_silent_and_can_retry(cable):
  link, root, _ = cable
  link.start()
  root.side_effect = [mobile.MobileLinkError('not helper-owned'), None]
  with pytest.raises(mobile.MobileLinkError, match='not helper-owned'):
    link.close()
  assert link._network_started
  link.close()
  assert not link._network_started


def peer_socket(ip='192.168.60.2', local=mobile.ADDRESS, interface=b'usb7\0'):
  connection = Mock()
  connection.getsockname.return_value = (local, 5599)
  connection.getsockopt.return_value = interface
  return connection, (ip, 12345)


def test_accept_returns_tcp_with_metadata_without_touching_ffs(cable, monkeypatch, scratch):
  link, _, listener = cable
  link.start()
  connection, peer = peer_socket()
  listener.accept.return_value = connection, peer
  udc = scratch / 'udc'
  (udc / 'test-udc').mkdir(parents=True)
  (udc / 'test-udc' / 'current_speed').write_text('super-speed\n')
  monkeypatch.setattr(mobile, 'UDC_CLASS', udc)
  transport = link.accept(timeout=.5)
  assert isinstance(transport, TcpTransport)
  assert transport.tx_align == transport.rx_align == 0
  assert transport.owner is link.owner
  assert transport.link_info() == {'kind': 'cable', 'transport': 'tcp', 'mode': 'ios', 'interface': 'usb7',
                                   'local': '192.168.60.1', 'peer': '192.168.60.2', 'usb_speed': 'super-speed'}
  transport.close()
  connection.close.assert_called_once()
  link.close()
  link.owner.send.assert_not_called()
  link.owner.recv.assert_not_called()


@pytest.mark.parametrize('bad', [
  peer_socket(ip='10.0.0.2'), peer_socket(ip='192.168.61.2'), peer_socket(ip='192.168.60.0'),
  peer_socket(ip='192.168.60.255'), peer_socket(ip='192.168.60.1'), peer_socket(local='0.0.0.0'),
])
def test_rejects_non_cable_peers_with_total_accept_deadline(cable, bad):
  link, _, listener = cable
  link.start()
  bad[0].reset_mock()
  listener.accept.side_effect = [bad, TimeoutError()]
  with pytest.raises(LinkTimeout):
    link.accept(timeout=.05)
  bad[0].close.assert_called_once()
  assert listener.settimeout.call_args_list[1].args[0] <= listener.settimeout.call_args_list[0].args[0]
  link.close()


def test_unprivileged_listener_uses_verified_firewall_not_cap_net_raw(cable, caplog):
  link, root, listener = cable
  listener.setsockopt.side_effect = [None, PermissionError('CAP_NET_RAW unavailable')]
  link.start()
  root.assert_called_once_with('setup_mobile.sh', 'net')
  listener.bind.assert_called_once_with((mobile.ADDRESS, mobile.PORT))
  assert 'verified helper-owned INPUT firewall isolation' in caplog.text
  connection, peer = peer_socket(interface=b'')
  listener.accept.return_value = connection, peer
  assert link.accept().sock is connection
  link.close()


@pytest.mark.parametrize('change', ['ifname', 'ifindex', 'disappear'])
def test_rebind_invalidates_listener_and_closes_accepted_socket(cable, change):
  link, _, listener = cable
  link.start()
  connection, peer = peer_socket()

  def accept():
    if change == 'ifname':
      (link.gadget / 'functions' / mobile.NCM_FUNCTION / 'ifname').write_text('usb0')
    elif change == 'ifindex':
      (link.net_class / 'usb7' / 'ifindex').write_text('18')
    else:
      (link.net_class / 'usb7' / 'ifindex').unlink()
    return connection, peer

  listener.accept.side_effect = accept
  with pytest.raises(mobile.MobileLinkError, match='restart'):
    link.accept()
  connection.close.assert_called_once()
  link.close()


@pytest.mark.parametrize('value', [0, -1, float('inf'), float('nan'), True, None])
def test_accept_and_start_require_bounded_positive_timeout(cable, value):
  link, _, _ = cable
  with pytest.raises(ValueError, match='finite positive'):
    link.start(timeout=value)
  with pytest.raises(ValueError, match='finite positive'):
    link.accept(timeout=value)


def test_accept_timeout_preserves_listener_and_owner(cable):
  link, root, listener = cable
  link.start()
  listener.accept.side_effect = TimeoutError()
  with pytest.raises(LinkTimeout):
    link.accept(timeout=.01)
  assert link.listener is listener and root.call_count == 1
  link.owner.close.assert_not_called()
  link.close()


def test_teardown_uses_owned_mobile_helper(monkeypatch):
  root = Mock()
  monkeypatch.setattr(mobile, '_root', root)
  mobile.teardown_gadget()
  root.assert_called_once_with('setup_mobile.sh', '--teardown')


def test_actual_loopback_tcp_framing_metadata_and_disconnect():
  # Only this test uses loopback; the production listener has no LAN/loopback mode.
  server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  server.bind(('127.0.0.1', 0))
  server.listen(1)
  server.settimeout(1)
  remote = TcpTransport.connect('127.0.0.1', server.getsockname()[1], timeout=1)
  connection, peer = server.accept()
  owner = SimpleNamespace(bound_udc=None, close=Mock())
  local = mobile.CableTransport(connection, owner, 'test-loopback', peer)
  try:
    assert local.tx_align == local.rx_align == 0
    for size in (0, 1, 31, 513, 65537):
      body = b'x' * size
      local.send(P.Msg.PING, size + 1, [body] if body else [])
      received = remote.recv(timeout=1)
      assert received.seq == size + 1 and bytes(received.payload) == body
      remote.send(P.Msg.PONG, size + 1, [body] if body else [])
      received = local.recv(timeout=1)
      assert received.seq == size + 1 and bytes(received.payload) == body
    assert local.link_info()['transport'] == 'tcp'
    remote.close()
    with pytest.raises(LinkError, match='closed'):
      local.recv(timeout=1)
  finally:
    local.close()
    remote.close()
    server.close()
  owner.close.assert_not_called()


def test_root_script_syntax_and_policy():
  script = mobile.TOOLS / 'setup_mobile.sh'
  subprocess.run(['bash', '-n', str(script)], check=True)
  source = script.read_text()
  assert 'bash "$SCRIPT_DIR/setup_gadget.sh"' in source
  assert '--dhcp-option=3 --dhcp-option=6' in source and '--port=0' in source
  assert 'functions/$FUNCTION/ifname' in source
  assert 'pidfd_send_signal' in source and 'saved !=' in source
  for forbidden in ('killall', 'pkill', '/power_supply/', 'adb ', 'sysctl ', 'ip address flush'):
    assert forbidden not in source


def test_root_script_rejects_non_opted_in_mode(scratch):
  env = dict(os.environ, JETLINK_TRANSPORT='usb', JETLINK_MOBILE_STATE=str(scratch / 'state'))
  result = subprocess.run(['bash', str(mobile.TOOLS / 'setup_mobile.sh'), 'net'], env=env, capture_output=True, text=True)
  assert result.returncode != 0
  assert 'explicit' in result.stderr or 'root' in result.stderr
  assert not (scratch / 'state').exists()


@pytest.mark.parametrize('pid', ['invalid', '1', str(os.getpid())])
def test_dhcp_helper_refuses_unowned_pid_without_signaling(scratch, monkeypatch, pid):
  source = (mobile.TOOLS / 'setup_mobile.sh').read_text().split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
  (scratch / 'dnsmasq.pid').write_text(pid)
  kill = Mock(side_effect=AssertionError('must not signal an unowned process'))
  monkeypatch.setattr(os, 'kill', kill)
  monkeypatch.setattr(signal, 'pidfd_send_signal', kill, raising=False)
  monkeypatch.setattr(sys, 'argv', ['helper', str(scratch), 'stop', str(scratch / 'leases')])
  with pytest.raises(SystemExit, match='refusing DHCP'):
    exec(compile(source, 'setup_mobile.sh:dhcp_process', 'exec'), {})
  kill.assert_not_called()


def test_dhcp_helper_cleans_exited_pid_without_signaling(scratch, monkeypatch):
  source = (mobile.TOOLS / 'setup_mobile.sh').read_text().split("<<'PY'\n", 1)[1].split('\nPY\n', 1)[0]
  (scratch / 'dnsmasq.pid').write_text('2147483647')
  (scratch / 'dnsmasq.owner').write_text('{"pid":2147483647,"start":"0"}')
  kill = Mock(side_effect=AssertionError('must not signal an exited process'))
  monkeypatch.setattr(os, 'kill', kill)
  monkeypatch.setattr(signal, 'pidfd_send_signal', kill, raising=False)
  monkeypatch.setattr(sys, 'argv', ['helper', str(scratch), 'stop', str(scratch / 'leases')])
  with pytest.raises(SystemExit) as result:
    exec(compile(source, 'setup_mobile.sh:dhcp_process', 'exec'), {})
  assert result.value.code == 0
  assert not (scratch / 'dnsmasq.pid').exists() and not (scratch / 'dnsmasq.owner').exists()
  kill.assert_not_called()


@pytest.mark.parametrize('iptables_fails', [False, True])
def test_firewall_is_exact_scoped_owned_and_failure_is_explicit(scratch, iptables_fails):
  source = (mobile.TOOLS / 'setup_mobile.sh').read_text()
  functions = source.split('firewall_backend() {', 1)[1].split('\nnet_down() {', 1)[0]
  functions = 'firewall_backend() {' + functions
  harness = r'''
set -euo pipefail
STATE=$1
ADDRESS=192.168.60.1
PORT=5599
FIREWALL_COMMENT=jetlink-mobile-owned-5599
dev=usb7
fail() { echo "$*" >&2; exit 1; }
iptables() {
  printf '%s\n' "$*" >> "$STATE/commands"
  [[ "$2" != 2 ]] && return 99
  if [[ "$3" == -I ]]; then
    [[ "$FAIL_INSERT" == 0 ]] || return 3
    echo 1 > "$STATE/rule-present"
  elif [[ "$3" == -C ]]; then
    [[ -f "$STATE/rule-present" ]]
  elif [[ "$3" == -D ]]; then
    rm "$STATE/rule-present"
  fi
}
'''
  env = dict(os.environ, FAIL_INSERT='1' if iptables_fails else '0')
  result = subprocess.run(['bash', '-c', harness + functions + '\nfirewall_up\nfirewall_down\n',
                           'mobile-firewall-test', str(scratch)], env=env, capture_output=True, text=True)
  commands = (scratch / 'commands').read_text().splitlines()
  exact = '-d 192.168.60.1/32 -p tcp --dport 5599 ! -i usb7 -m comment --comment jetlink-mobile-owned-5599 -j DROP'
  assert f'-w 2 -I INPUT 1 {exact}' in commands
  if iptables_fails:
    assert result.returncode != 0
    assert (scratch / 'firewall-interface').exists()
  else:
    assert result.returncode == 0, result.stderr
    assert f'-w 2 -C INPUT {exact}' in commands
    assert f'-w 2 -D INPUT {exact}' in commands
    assert not (scratch / 'firewall-interface').exists()
    assert not (scratch / 'rule-present').exists()
  assert all('OUTPUT' not in entry and 'FORWARD' not in entry and 'flush' not in entry for entry in commands)


def test_dhcp_leases_have_dedicated_privilege_drop_access(scratch):
  source = (mobile.TOOLS / 'setup_mobile.sh').read_text()
  function = 'leases_prepare() {' + source.split('leases_prepare() {', 1)[1].split('\nnet_down() {', 1)[0]
  harness = r'''
set -euo pipefail
STATE=$1/control
LEASE_DIR=$1/leases
fail() { echo "$*" >&2; exit 1; }
id() {
  if [[ "$1" == -u && "$2" == nobody ]]; then echo 65534;
  elif [[ "$1" == -gn && "$2" == nobody ]]; then echo nogroup;
  else return 1; fi
}
install() { printf '%s\n' "$*" >> "$STATE/install-command"; }
mkdir "$STATE"
chmod 0700 "$STATE"
'''
  result = subprocess.run(['bash', '-c', harness + function + '\nleases_prepare\n',
                           'mobile-lease-test', str(scratch)], capture_output=True, text=True)
  assert result.returncode == 0, result.stderr
  assert (scratch / 'control/install-command').read_text().splitlines() == [
    f'-d -o nobody -g nogroup -m 0700 {scratch}/leases',
    f'-o nobody -g nogroup -m 0600 /dev/null {scratch}/leases/dnsmasq.leases',
  ]
  assert (scratch / 'control').stat().st_mode & 0o777 == 0o700
  assert '--dhcp-leasefile="$LEASE_DIR/dnsmasq.leases"' in source
  assert '--user="$dhcp_user" --group="$dhcp_group"' in source
