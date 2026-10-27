import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from wifi_direct_forward import COMMENT, Sharing


class Firewall:
  def __init__(self, default=4):
    self.default = default
    self.rules = {('unrelated',)}
    self.calls = []

  def __call__(self, args):
    self.calls.append(args)
    if args[0] == 'iptables':
      if self.default is None:
        raise FileNotFoundError()
      if self.default:
        return self.default
    operation = args[5]
    rule = tuple(args[7:])
    if operation == '-S':
      return 0
    if operation == '-C':
      return 0 if rule in self.rules else 1
    if operation == '-A':
      self.rules.add(rule)
      return 0
    if operation == '-D':
      self.rules.remove(rule)
      return 0
    raise AssertionError(args)


@pytest.mark.parametrize('default,backend', [(4, 'iptables-legacy'), (None, 'iptables-legacy'), (0, 'iptables')])
def test_working_backend_idempotence_and_owned_cleanup(default, backend):
  firewall = Firewall(default)
  sharing = Sharing(firewall)
  sharing.ensure('172.29.77.0/24')
  sharing.ensure('172.29.77.0/24')
  assert sharing.backend == backend
  assert len(firewall.rules) == 2
  owned = next(r for r in firewall.rules if COMMENT in r)
  assert owned[:5] == ('-s', '172.29.77.0/24', '!', '-d', '172.29.77.0/24')
  assert sum('-A' in c for c in firewall.calls) == 1
  sharing.close()
  sharing.close()
  assert firewall.rules == {('unrelated',)}
  assert not any('-F' in c or '-P' in c for c in firewall.calls)


def test_failed_inspection_does_not_blindly_add_or_delete():
  command = Mock(side_effect=[0, 4])
  sharing = Sharing(command)
  with pytest.raises(RuntimeError, match='inspection'):
    sharing.ensure('172.29.77.0/24')
  command.side_effect = [4]
  with pytest.raises(RuntimeError, match='inspection'):
    sharing.close()
  assert not any('-A' in c.args[0] or '-D' in c.args[0] for c in command.call_args_list)
  assert sharing.rule is not None


def test_failed_addition_and_verification_are_reported():
  for results in ([0, 1, 4], [0, 1, 0, 1]):
    with pytest.raises(RuntimeError):
      Sharing(Mock(side_effect=results)).ensure('172.29.77.0/24')


@pytest.mark.parametrize('subnet', ['0.0.0.0/0', '8.8.8.0/24', '172.29.77.1/24', '::/0'])
def test_invalid_scope_never_touches_firewall(subnet):
  command = Mock()
  with pytest.raises(ValueError):
    Sharing(command).ensure(subnet)
  command.assert_not_called()


def test_onroad_never_changes_sharing_and_failure_preserves_ap(monkeypatch):
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  import wifi_direct
  ap = wifi_direct.AccessPoint(Mock())
  ap.value = {'subnet': '172.29.77.0/24'}
  ap.sharing = Mock()
  monkeypatch.setattr(wifi_direct, 'offroad', lambda: False)
  ap.share_internet()
  ap.sharing.ensure.assert_not_called()
  monkeypatch.setattr(wifi_direct, 'offroad', lambda: True)
  ap.sharing.ensure.side_effect = RuntimeError('backend unavailable')
  ap.share_internet()
  assert ap.sharing_state == 'unavailable' and ap.value is not None
  ap.sharing.close.assert_not_called()


@pytest.mark.skipif(os.environ.get('CARROT_WIFI_NAT_NAMESPACE') != '1',
                    reason='Requires the root CI job in an isolated mount/network namespace')
def test_real_packets_require_nat_and_cleanup_preserves_unrelated_rule():
  def command(*args, check=True):
    return subprocess.run(args, check=check, capture_output=True, text=True, timeout=10)
  sharing = Sharing()
  namespaces = []
  try:
    for name in ('carrot-nat-client', 'carrot-nat-server'):
      command('ip', 'netns', 'add', name)
      namespaces.append(name)
      command('ip', '-n', name, 'link', 'set', 'lo', 'up')
    for interface, peer, namespace, local, remote in (
      ('wljetson0', 'client', namespaces[0], '172.29.77.1/24', '172.29.77.2/24'),
      ('wlan0', 'server', namespaces[1], '192.0.2.1/24', '192.0.2.2/24'),
    ):
      command('ip', 'link', 'add', interface, 'type', 'veth', 'peer', 'name', peer)
      command('ip', 'link', 'set', peer, 'netns', namespace)
      command('ip', 'addr', 'add', local, 'dev', interface)
      command('ip', 'link', 'set', interface, 'up')
      command('ip', '-n', namespace, 'addr', 'add', remote, 'dev', peer)
      command('ip', '-n', namespace, 'link', 'set', peer, 'up')
    command('ip', '-n', namespaces[0], 'route', 'add', 'default', 'via', '172.29.77.1')
    command('sysctl', '-w', 'net.ipv4.ip_forward=1')
    def ping():
      return command('ip', 'netns', 'exec', namespaces[0], 'ping', '-n', '-c', '1', '-W', '1',
                     '192.0.2.2', check=False).returncode
    assert ping() != 0  # The server has no route back to the AP's private subnet.
    sharing.ensure('172.29.77.0/24')
    backend = sharing.backend
    unrelated = ('-s', '203.0.113.0/24', '-m', 'comment', '--comment', 'unrelated-test', '-j', 'RETURN')
    command(backend, '-t', 'nat', '-A', 'POSTROUTING', *unrelated)
    assert ping() == 0
    sharing.ensure('172.29.77.0/24')
    sharing.close()
    command(backend, '-t', 'nat', '-C', 'POSTROUTING', *unrelated)
    # New ping identifier creates a fresh conntrack flow after rule removal.
    assert ping() != 0
  finally:
    sharing.close()
    for namespace in reversed(namespaces):
      command('ip', 'netns', 'delete', namespace, check=False)
