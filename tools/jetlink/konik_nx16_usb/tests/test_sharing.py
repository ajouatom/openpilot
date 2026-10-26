"""실기/방화벽을 쓰지 않는 서비스 경계와 무해한 자식 회수 검사."""
import copy
import errno
import importlib.util
import ipaddress
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
from types import SimpleNamespace as NS

import pytest

if os.name != 'posix':
  pytest.skip('POSIX USB 공유 서비스 검사', allow_module_level=True)

HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location('usb_internet_candidate', HERE.parent / 'sharing.py')
service = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(service)


def snapshot(**changes):
  return dict(ncm='usb0', ncm_index=7, uplink='wlan0', uplink_index=3,
              dns=['192.0.2.53'], dns_input_rules=False) | changes


class FakeRunner:
  def __init__(self):
    self.paths = {'dnsmasq': '/usr/sbin/dnsmasq'}
    self.calls = []
    self.rules = [('nat', 'POSTROUTING', ['-s', '172.28.99.0/24', '-m', 'comment', '--comment',
                                        'carrot-jetson-direct-owned-nat', '-j', 'MASQUERADE'])]
    self.policies = dict(INPUT='ACCEPT', FORWARD='ACCEPT', POSTROUTING='ACCEPT')
    self.fail_add_after = None
    self.added = 0
    self.check_error = False
    self.dns = '192.0.2.53\n'
    self.route_device = 'wlan0'
    self.extra_routes = []
    self.extra_addresses = []

  def run(self, name, args, *, deadline=None):
    self.calls.append((name, list(args)))
    if name == 'iptables-legacy':
      assert args[:2] == ['-w', '2'] and args[2] == '-t'
      table, action, chain = args[3:6]
      body = args[6:]
      if action == '-S':
        text = '-P ' + chain + ' ' + self.policies[chain] + '\n'
        text += '\n'.join(shlex.join(['-A', chain, *entry[2]]) for entry in self.rules if entry[:2] == (table, chain))
        return NS(returncode=0, stdout=text)
      if action == '-I':
        assert body[0] == '1'
        body = body[1:]
      item = (table, chain, list(body))
      if action == '-C':
        return NS(returncode=2 if self.check_error else (0 if item in self.rules else 1), stdout='')
      if action in ('-A', '-I'):
        if self.fail_add_after is not None and self.added == self.fail_add_after:
          return NS(returncode=2, stdout='')
        if action == '-I':
          location = next((index for index, previous in enumerate(self.rules) if previous[:2] == (table, chain)), len(self.rules))
          self.rules.insert(location, item)
        else:
          self.rules.append(item)
        self.added += 1
        return NS(returncode=0, stdout='')
      if action == '-D':
        if item not in self.rules:
          return NS(returncode=1, stdout='')
        self.rules.remove(item)
        return NS(returncode=0, stdout='')
      raise AssertionError('허용 밖 방화벽 명령')
    if name == 'nmcli':
      assert args == ['-g', 'IP4.DNS', 'device', 'show', self.route_device]
      return NS(returncode=0, stdout=self.dns)
    assert name == 'ip'
    if args == ['-j', 'link', 'show']:
      value = [dict(ifname='usb0', ifindex=7, flags=['UP']),
               dict(ifname='wlan0', ifindex=3, flags=['UP']), dict(ifname='wwan0', ifindex=4, flags=['UP'])]
    elif args == ['-j', '-4', 'address', 'show', 'dev', 'usb0']:
      value = [dict(addr_info=[dict(local='192.168.60.1', prefixlen=24), *self.extra_addresses])]
    elif args == ['-j', '-4', 'route', 'show', 'table', 'all']:
      value = [dict(dst='192.168.60.0/24', dev='usb0'), *self.extra_routes]
    elif args == ['-j', '-4', 'route', 'show', 'default']:
      value = [dict(dst='default', dev=self.route_device)]
    elif args[:4] == ['-j', '-4', 'route', 'get']:
      value = [dict(dev=self.route_device)]
    else:
      raise AssertionError('조회 외 IP 명령')
    return NS(returncode=0, stdout=json.dumps(value))


class FakeDns:
  def __init__(self):
    self.starts, self.stops = [], []
    self.alive = False
    self.fail = False

  def start(self, value):
    if self.fail:
      raise service.OwnershipError('injected_dns_failure')
    self.starts.append(copy.deepcopy(value))
    self.alive = True
    return dict(pid=1000 + len(self.starts), start='77', argv_sha256='test', args=['test'])

  def stop(self, record, value):
    if record is not None:
      self.stops.append(record)
    self.alive = False

  def healthy(self, record):
    return self.alive and record is not None


class FakeProbe:
  def __init__(self):
    self.value = snapshot()
    self.reason = None
    self.port_busy = False

  def discover(self, deadline):
    if self.reason:
      raise service.NotReady(self.reason)
    return copy.deepcopy(self.value)

  def dns_port_free(self):
    if self.port_busy:
      raise service.NotReady('dns_port_owned_by_another_process')


@pytest.fixture
def fixture(tmp_path):
  store = service.PrivateStore(tmp_path / 'state', owner=os.getuid())
  runner, dns, probe = FakeRunner(), FakeDns(), FakeProbe()
  controller = service.Controller(probe, service.Firewall(runner), dns, store, 'fixture-boot')
  return NS(store=store, runner=runner, dns=dns, probe=probe, controller=controller)


def own_rules(value):
  return [rule for rule in value.runner.rules if any('carrot-usb-internet-' in item for item in rule[2])]


def input_verdict(runner, *, incoming, destination, protocol='udp', port=53, source='192.168.60.2'):
  for table, chain, args in runner.rules:
    if table != 'filter' or chain != 'INPUT':
      continue
    matched, negated, index, target = True, False, 0, None
    while index < len(args):
      flag = args[index]
      if flag == '!':
        negated = True
        index += 1
        continue
      value = args[index + 1]
      if flag == '-j':
        target = value
        break
      if flag == '-i':
        condition = incoming == value
      elif flag == '-s':
        condition = ipaddress.ip_address(source) in ipaddress.ip_network(value)
      elif flag == '-d':
        condition = ipaddress.ip_address(destination) in ipaddress.ip_network(value)
      elif flag == '-p':
        condition = protocol == value
      elif flag == '--dport':
        condition = port == int(value)
      else:
        assert flag in ('-m', '--comment')
        condition = True
      matched = matched and (not condition if negated else condition)
      negated = False
      index += 2
    if matched:
      return target
  return runner.policies['INPUT']


def test_idempotent_service_owns_five_rules_and_preserves_ap(fixture):
  foreign = copy.deepcopy(fixture.runner.rules)
  assert fixture.controller.step(1)['state'] == 'configured'
  assert len(own_rules(fixture)) == 5
  fixture.controller.step(2)
  assert len(fixture.dns.starts) == 1 and len(own_rules(fixture)) == 5
  fixture.controller.cleanup()
  assert fixture.runner.rules == foreign and not fixture.dns.alive
  assert fixture.store.read('owner.json') is None
  assert all(action not in args for _name, args in fixture.runner.calls for action in ('-F', '--flush', '-P'))


def test_required_dns_input_rules_are_exact_and_only_dns_is_inserted(fixture):
  fixture.runner.policies['INPUT'] = 'DROP'
  fixture.probe.value['dns_input_rules'] = True
  fixture.controller.step(1)
  assert len(own_rules(fixture)) == 7
  inserted = [args for name, args in fixture.runner.calls if name == 'iptables-legacy' and '-I' in args]
  assert len(inserted) == 4
  assert all('INPUT' in args and '--dport' in args and args[args.index('--dport') + 1] == '53' for args in inserted)
  assert input_verdict(fixture.runner, incoming='usb0', destination='192.168.60.1') == 'ACCEPT'
  assert input_verdict(fixture.runner, incoming='wlan0', destination='192.168.60.1') == 'DROP'
  fixture.controller.cleanup()
  assert len(fixture.runner.rules) == 1


def test_dns_isolation_drops_only_other_ingress_to_this_dns_address(fixture):
  foreign = ('filter', 'INPUT', ['-d', '172.28.99.1/32', '-p', 'udp', '--dport', '53', '-j', 'ACCEPT'])
  fixture.runner.rules.append(copy.deepcopy(foreign))
  fixture.controller.step(1)
  drops = [rule for rule in own_rules(fixture) if rule[1] == 'INPUT' and rule[2][-1] == 'DROP']
  assert len(drops) == 2 and {rule[2][rule[2].index('-p') + 1] for rule in drops} == {'udp', 'tcp'}
  for protocol in ('udp', 'tcp'):
    assert input_verdict(fixture.runner, incoming='usb0', destination='192.168.60.1', protocol=protocol) == 'ACCEPT'
    assert input_verdict(fixture.runner, incoming='wlan0', destination='192.168.60.1', protocol=protocol) == 'DROP'
    assert input_verdict(fixture.runner, incoming='lo', destination='192.168.60.1', protocol=protocol) == 'DROP'
  assert input_verdict(fixture.runner, incoming='wlan0', destination='192.168.60.1', port=22) == 'ACCEPT'
  assert input_verdict(fixture.runner, incoming='wljetson0', destination='172.28.99.1') == 'ACCEPT'
  assert input_verdict(fixture.runner, incoming='lo', destination='127.0.0.53') == 'ACCEPT'
  fixture.controller.cleanup()
  assert foreign in fixture.runner.rules


def test_wan_and_dns_change_reaps_old_child_and_replaces_only_owned_rules(fixture):
  fixture.controller.step(1)
  old_lease = fixture.controller.owner['lease']
  fixture.probe.value = snapshot(uplink='wwan0', uplink_index=4, dns=['198.51.100.53'])
  fixture.controller.step(2)
  assert len(fixture.dns.starts) == 2 and len(fixture.dns.stops) == 1
  assert fixture.controller.owner['lease'] != old_lease
  assert all(old_lease not in ' '.join(rule[2]) for rule in fixture.runner.rules)
  assert all('wwan0' in rule[2] for rule in own_rules(fixture) if rule[1] != 'INPUT')


def test_missing_wan_is_standby_and_cleans_up_without_touching_ap(fixture):
  fixture.controller.step(1)
  fixture.probe.reason = 'wan_default_not_verified'
  result = fixture.controller.step(2)
  assert result['state'] == 'standby' and result['internet_verified'] is False
  assert fixture.controller.owner is None and len(fixture.runner.rules) == 1 and not fixture.dns.alive


def test_existing_dns_socket_is_not_reclaimed(fixture):
  fixture.probe.port_busy = True
  with pytest.raises(service.NotReady, match='another_process'):
    fixture.controller.step(1)
  assert not own_rules(fixture) and not fixture.dns.starts


def test_partial_rule_failure_rolls_back_only_attempted_rules(fixture):
  fixture.runner.fail_add_after = 1
  with pytest.raises(service.OwnershipError, match='rule_add'):
    fixture.controller.step(1)
  assert len(fixture.runner.rules) == 1 and fixture.controller.owner is None
  assert not fixture.dns.starts


def test_inspection_error_is_not_treated_as_absent_rule(fixture):
  fixture.runner.check_error = True
  with pytest.raises(service.OwnershipError, match='rule_read'):
    fixture.controller.step(1)
  assert not any('-A' in args or '-I' in args for _name, args in fixture.runner.calls)


def test_failed_dns_start_removes_scoped_rules(fixture):
  fixture.dns.fail = True
  with pytest.raises(service.OwnershipError, match='injected_dns'):
    fixture.controller.step(1)
  assert len(fixture.runner.rules) == 1 and fixture.controller.owner is None


def test_interface_reuse_during_setup_cleans_only_prior_owned_insertions(fixture):
  calls = 0
  def changing(deadline):
    nonlocal calls
    calls += 1
    return snapshot(ncm_index=77) if calls >= 4 else snapshot()
  fixture.probe.discover = changing
  with pytest.raises(service.NotReady, match='network_changed'):
    fixture.controller.step(1)
  assert len(fixture.runner.rules) == 1 and not fixture.dns.starts
  assert fixture.controller.owner is None


def test_changed_rule_with_same_comment_is_preserved_as_conflict(fixture):
  fixture.controller.step(1)
  changed = next(rule for rule in own_rules(fixture) if rule[0] == 'nat')
  changed[2][changed[2].index('-o') + 1] = 'operator0'
  with pytest.raises(service.OwnershipError, match='rule_cleanup_failed'):
    fixture.controller.cleanup()
  assert changed in fixture.runner.rules
  assert fixture.store.read('owner.json')['phase'] == 'cleanup_failed'
  assert fixture.runner.rules[0][2][-1] == 'MASQUERADE'


def test_duplicate_same_comment_rule_is_not_blindly_deleted(fixture):
  fixture.controller.step(1)
  original = next(rule for rule in own_rules(fixture) if rule[0] == 'nat')
  fixture.runner.rules.append(copy.deepcopy(original))
  with pytest.raises(service.OwnershipError):
    fixture.controller.cleanup()
  assert fixture.runner.rules.count(original) == 2


def test_same_boot_journal_recovers_owned_resources(fixture):
  fixture.controller.step(1)
  recovered = service.Controller(fixture.probe, service.Firewall(fixture.runner), fixture.dns, fixture.store, 'fixture-boot')
  recovered.recover()
  assert recovered.owner is None and not fixture.dns.alive and len(fixture.runner.rules) == 1


def test_other_boot_journal_never_authorizes_a_pid_or_rule_cleanup(fixture):
  fixture.controller.step(1)
  before = copy.deepcopy(fixture.runner.rules)
  other = service.Controller(fixture.probe, service.Firewall(fixture.runner), fixture.dns, fixture.store, 'other-boot')
  with pytest.raises(service.OwnershipError, match='other_boot'):
    other.recover()
  assert fixture.runner.rules == before and not fixture.dns.stops


@pytest.mark.parametrize('changes', [dict(uplink='usb0'), dict(uplink='wljetson0'), dict(dns=['127.0.0.53']),
                                    dict(dns=['192.168.60.1']), dict(dns=['224.0.0.1']), dict(ncm_index=True)])
def test_invalid_or_looping_snapshot_never_creates_rules(fixture, changes):
  fixture.probe.value.update(changes)
  with pytest.raises((service.OwnershipError, service.NotReady)):
    fixture.controller.step(1)
  assert not own_rules(fixture) and not fixture.dns.starts


def test_store_rejects_symlinks_bad_modes_path_escape_and_duplicate_owner(tmp_path):
  store = service.PrivateStore(tmp_path / 'state', owner=os.getuid())
  with pytest.raises(service.OwnershipError, match='scope'):
    store.write('../outside', {})
  outside = tmp_path / 'outside'
  outside.write_text('preserved')
  (store.directory / 'owner.json').symlink_to(outside)
  with pytest.raises((OSError, service.OwnershipError)):
    store.read('owner.json')
  assert outside.read_text() == 'preserved'
  first = store.lock()
  try:
    with pytest.raises(BlockingIOError):
      store.lock()
  finally:
    os.close(first)
  bad = tmp_path / 'bad'
  bad.mkdir(mode=0o755)
  with pytest.raises(service.OwnershipError):
    service.PrivateStore(bad, owner=os.getuid())


@pytest.fixture
def kernel(tmp_path):
  root = tmp_path / 'kernel'
  gadget = root / str(service.GADGET).lstrip('/')
  target = gadget / 'functions/ncm.jetlink'
  target.mkdir(parents=True)
  (target / 'ifname').write_text('usb0\n')
  config = gadget / 'configs/c.1'
  config.mkdir(parents=True)
  (config / 'ncm.jetlink').symlink_to(target)
  (gadget / 'UDC').write_text('a600000.dwc3\n')
  state = root / str(service.MOBILE_STATE).lstrip('/')
  state.mkdir(parents=True)
  (state / 'ncm-owned').write_text('ncm.jetlink\n')
  udc = root / 'sys/class/udc/a600000.dwc3'
  udc.mkdir(parents=True)
  (udc / 'state').write_text('configured\n')
  (udc / 'current_speed').write_text('super-speed\n')
  forward = root / 'proc/sys/net/ipv4/ip_forward'
  forward.parent.mkdir(parents=True)
  forward.write_text('1\n')
  return root


def test_actual_probe_reads_only_dns_and_routes_and_preserves_forwarding(kernel):
  runner = FakeRunner()
  probe = service.SystemProbe(runner, root=kernel, marker_uid=os.getuid())
  assert probe.discover(time.monotonic() + 5) == snapshot()
  assert (kernel / 'proc/sys/net/ipv4/ip_forward').read_text() == '1\n'
  assert all(name in ('ip', 'nmcli', 'iptables-legacy') for name, _args in runner.calls)
  assert all('modify' not in args and 'set' not in args and '-A' not in args for _name, args in runner.calls)


def test_nm_dns_without_spaces_around_pipe_is_split_without_public_fallback(kernel):
  runner = FakeRunner()
  runner.dns = '192.0.2.53|198.51.100.53\n'
  result = service.SystemProbe(runner, root=kernel, marker_uid=os.getuid()).discover(time.monotonic() + 5)
  assert result['dns'] == ['192.0.2.53', '198.51.100.53']


def test_actual_probe_ignores_only_its_own_dns_isolation_when_rechecking(kernel, tmp_path):
  runner = FakeRunner()
  probe = service.SystemProbe(runner, root=kernel, marker_uid=os.getuid())
  dns = FakeDns()
  store = service.PrivateStore(tmp_path / 'real-probe-state', owner=os.getuid())
  controller = service.Controller(probe, service.Firewall(runner), dns, store, 'fixture-boot')
  assert controller.step(time.monotonic() + 10)['state'] == 'configured'
  assert len(controller.owner['rules']) == 5
  assert controller.step(time.monotonic() + 10)['state'] == 'configured'
  assert len(dns.starts) == 1
  controller.cleanup()


def test_other_ncm_ipv4_addresses_do_not_expand_the_dns_binding_scope(kernel):
  runner = FakeRunner()
  runner.extra_addresses = [dict(local='192.168.61.1', prefixlen=24)]
  with pytest.raises(service.NotReady, match='unexpected_addresses'):
    service.SystemProbe(runner, root=kernel, marker_uid=os.getuid()).discover(time.monotonic() + 5)


@pytest.mark.parametrize('problem', ['collision', 'dns_stub', 'wan_loop', 'forward_off', 'policy_drop'])
def test_actual_probe_refuses_unsupported_environment_without_mutation(kernel, problem):
  runner = FakeRunner()
  if problem == 'collision':
    runner.extra_routes = [dict(dst='192.168.60.0/24', dev='wlan0')]
  elif problem == 'dns_stub':
    runner.dns = '127.0.0.53\n'
  elif problem == 'wan_loop':
    runner.route_device = 'usb0'
  elif problem == 'forward_off':
    (kernel / 'proc/sys/net/ipv4/ip_forward').write_text('0\n')
  else:
    runner.policies['FORWARD'] = 'DROP'
  with pytest.raises(service.NotReady):
    service.SystemProbe(runner, root=kernel, marker_uid=os.getuid()).discover(time.monotonic() + 5)
  assert not any('-A' in args or '-I' in args or '-D' in args for _name, args in runner.calls)


def test_dns_arguments_are_ipv4_only_and_never_add_dhcp_or_local_stub(tmp_path):
  child = service.DnsChild('/usr/sbin/dnsmasq', tmp_path)
  args = child.args(snapshot())
  assert '--listen-address=192.168.60.1' in args and '--port=53' in args
  assert '--no-dhcp-interface=usb0' in args and '--no-resolv' in args
  assert not any(item.startswith('--interface=') or item.startswith('--except-interface=') for item in args)
  assert '--server=192.0.2.53' in args
  assert not any(item.startswith('--dhcp-range') or '127.0.0.53' in item for item in args)


def test_dns_start_identity_error_reaps_its_direct_child(tmp_path, monkeypatch):
  fake = NS(pid=123, terminated=0, waited=0, killed=0)
  fake.poll = lambda: None
  fake.terminate = lambda: setattr(fake, 'terminated', fake.terminated + 1)
  fake.wait = lambda timeout=None: setattr(fake, 'waited', fake.waited + 1)
  fake.kill = lambda: setattr(fake, 'killed', fake.killed + 1)
  monkeypatch.setattr(service.subprocess, 'Popen', lambda *args, **kwargs: fake)
  monkeypatch.setattr(service.time, 'sleep', lambda value: None)
  monkeypatch.setattr(service.DnsChild, 'process_identity', lambda *args: None)
  child = service.DnsChild('/usr/sbin/dnsmasq', tmp_path)
  with pytest.raises(service.OwnershipError, match='identity'):
    child.start(snapshot())
  assert fake.terminated == 1 and fake.waited == 1 and child.child is None


def test_dns_pid_file_symlink_is_refused_before_process_creation(tmp_path, monkeypatch):
  target = tmp_path / 'foreign'
  target.write_text('unchanged')
  (tmp_path / 'dnsmasq.pid').symlink_to(target)
  monkeypatch.setattr(service.subprocess, 'Popen', lambda *args, **kwargs: pytest.fail('프로세스를 만들면 안 됩니다'))
  with pytest.raises(service.OwnershipError, match='symlink'):
    service.DnsChild('/usr/sbin/dnsmasq', tmp_path).start(snapshot())
  assert target.read_text() == 'unchanged'


def test_reused_dns_pid_does_not_authorize_signaling(tmp_path, monkeypatch):
  child = service.DnsChild('/usr/sbin/dnsmasq', tmp_path)
  record = dict(pid=123, start='old', argv_sha256='old', args=child.args(snapshot()))
  monkeypatch.setattr(child, 'process_identity', lambda pid: dict(start='new', argv_sha256='new'))
  monkeypatch.setattr(service.os, 'kill', lambda *args: pytest.fail('재사용된 PID에 신호를 보내면 안 됩니다'))
  child.stop(record, snapshot())


def test_dns_exit_during_identity_check_is_reaped_without_signaling(tmp_path, monkeypatch):
  owner = service.DnsChild('/usr/sbin/dnsmasq', tmp_path)
  record = dict(pid=123, start='77', argv_sha256='hash', args=owner.args(snapshot()))
  sequence = iter([dict(start='77', argv_sha256='hash'), None])
  monkeypatch.setattr(owner, 'process_identity', lambda pid: next(sequence))
  monkeypatch.setattr(service.os, 'pidfd_open', lambda pid: (_ for _ in ()).throw(OSError(errno.ENOSYS, 'fixture')))
  monkeypatch.setattr(service.os, 'kill', lambda *args: pytest.fail('이미 종료된 자식에는 신호를 보내지 않습니다'))
  fake = NS(pid=123, waited=0)
  fake.poll = lambda: 0
  fake.wait = lambda timeout=None: setattr(fake, 'waited', fake.waited + 1)
  owner.child = fake
  owner.stop(record, snapshot())
  assert owner.child is None and fake.waited == 1


@pytest.mark.parametrize('ignore_term', [False, True])
def test_actual_owned_harmless_child_is_stopped_and_reaped(tmp_path, ignore_term):
  marker = tmp_path / 'harmless-ready'
  script = ('import signal,time; from pathlib import Path; ' +
            ('signal.signal(signal.SIGTERM,signal.SIG_IGN); ' if ignore_term else '') +
            'Path(' + repr(str(marker)) + ').write_text("ready"); time.sleep(30)')
  args = [sys.executable, '-I', '-S', '-B', '-c', script]
  class Harmless(service.DnsChild):
    def args(self, _snapshot):
      return args
  owner = Harmless(sys.executable, tmp_path)
  owner.child = subprocess.Popen(args)
  process = owner.child
  try:
    deadline = time.monotonic() + 2
    identity = owner.process_identity(process.pid)
    expected = service.sha(b'\0'.join(os.fsencode(item) for item in args) + b'\0')
    while identity is None or identity['argv_sha256'] != expected or not marker.exists():
      assert time.monotonic() < deadline
      time.sleep(.01)
      identity = owner.process_identity(process.pid)
    owner.stop(dict(pid=process.pid, **identity, args=args), snapshot())
    assert process.poll() is not None and owner.child is None
    assert process.returncode == (-service.signal.SIGKILL if ignore_term else -service.signal.SIGTERM)
  finally:
    if process.poll() is None:
      process.kill()
      process.wait(timeout=2)


def test_absent_environment_notifies_standby_after_fifty_seconds_without_mutation(tmp_path, monkeypatch):
  clock = NS(now=0.0)
  events, statuses = [], []
  class Stop:
    def is_set(self):
      return clock.now >= 54.0
    def wait(self, seconds):
      clock.now += seconds
  class Store:
    directory = tmp_path
    def lock(self):
      return os.open(tmp_path / 'test.lock', os.O_CREAT | os.O_RDWR, 0o600)
    def write(self, name, value):
      statuses.append((name, value))
  def unavailable():
    raise service.NotReady('ncm_not_present')
  monkeypatch.setattr(service.sys if hasattr(service, 'sys') else sys, 'argv', ['usb_internet.py'])
  monkeypatch.setattr(service.os, 'geteuid', lambda: 0)
  monkeypatch.setattr(service.time, 'monotonic', lambda: clock.now)
  monkeypatch.setattr(service.threading, 'Event', Stop)
  monkeypatch.setattr(service.signal, 'signal', lambda *_args: None)
  monkeypatch.setattr(service, 'Runner', unavailable)
  monkeypatch.setattr(service, 'PrivateStore', Store)
  monkeypatch.setattr(service, 'notify', lambda value: events.append((clock.now, value)))
  assert service.main() == 0
  ready = [(stamp, value) for stamp, value in events if value.startswith('READY=1')]
  assert len(ready) == 1 and ready[0][0] == 50.0 and 'standby' in ready[0][1]
  assert all(value['state'] == 'standby' and value['internet_verified'] is False for _name, value in statuses)
