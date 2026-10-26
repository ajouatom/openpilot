"""기존 NCM link의 Internet 공유만 관리한다. Gadget·차량·모델 설정은 쓰지 않는다."""
from __future__ import annotations

import errno
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import pwd
import grp
import re
import shlex
import shutil
import signal
import socket
import stat
import subprocess
import tempfile
import threading
import time
import uuid

NETWORK = ipaddress.IPv4Network('192.168.60.0/24')
ADDRESS = '192.168.60.1'
STATE = Path('/run/carrot-usb-internet')
GADGET = Path('/sys/kernel/config/usb_gadget/jetlink')
MOBILE_STATE = Path('/dev/shm/jetlink-mobile')
STARTUP_WAIT = 50.0
POLL_SECONDS = 2.0


class NotReady(RuntimeError):
  """아직 없는 link/WAN/DNS는 서비스 오류와 구분한다."""


class OwnershipError(RuntimeError):
  pass


def error_reason(error):
  """알려진 짧은 오류 코드만 상태에 남기고 임의 예외 본문은 숨긴다."""
  if isinstance(error, (NotReady, OwnershipError)) and re.fullmatch(r'[a-z][a-z0-9_]{0,79}', str(error)):
    return str(error)
  return type(error).__name__


def sha(data):
  return hashlib.sha256(data).hexdigest()


def ifname(value):
  if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', value):
    raise NotReady('invalid_interface')
  return value


def valid_snapshot(value):
  if set(value) != {'ncm', 'ncm_index', 'uplink', 'uplink_index', 'dns', 'dns_input_rules'}:
    raise OwnershipError('snapshot_fields')
  ifname(value['ncm'])
  ifname(value['uplink'])
  if (value['ncm'] == value['uplink'] or value['uplink'] in ('lo', 'wljetson0') or
      any(type(value[key]) is not int or value[key] <= 0 for key in ('ncm_index', 'uplink_index')) or
      value['ncm_index'] == value['uplink_index']):
    raise OwnershipError('interface_loop')
  if type(value['dns_input_rules']) is not bool or not isinstance(value['dns'], list) or not 1 <= len(value['dns']) <= 4:
    raise OwnershipError('dns_fields')
  for text in value['dns']:
    address = ipaddress.ip_address(text)
    if (address.version != 4 or address.is_loopback or address.is_unspecified or address.is_multicast or
        address.is_link_local or address in NETWORK):
      raise OwnershipError('dns_loop_or_invalid')
  return value


def rules_for(snapshot, lease):
  valid_snapshot(snapshot)
  if not re.fullmatch('[0-9a-f]{12}', lease):
    raise OwnershipError('lease_identity')
  tag = ['-m', 'comment', '--comment', 'carrot-usb-internet-' + lease]
  ncm, uplink = snapshot['ncm'], snapshot['uplink']
  rules = [
    dict(table='nat', chain='POSTROUTING', insert=False,
         args=['-s', str(NETWORK), '-o', uplink, *tag, '-j', 'MASQUERADE']),
    dict(table='filter', chain='FORWARD', insert=False,
         args=['-i', ncm, '-o', uplink, '-s', str(NETWORK), '-m', 'conntrack',
               '--ctstate', 'NEW,ESTABLISHED,RELATED', *tag, '-j', 'ACCEPT']),
    dict(table='filter', chain='FORWARD', insert=False,
         args=['-i', uplink, '-o', ncm, '-d', str(NETWORK), '-m', 'conntrack',
               '--ctstate', 'ESTABLISHED,RELATED', *tag, '-j', 'ACCEPT']),
  ]
  # 다른 인터페이스에서 이 DNS 주소로 들어오는 요청만 막는다. 기존 DNS 주소는 건드리지 않는다.
  for protocol in ('udp', 'tcp'):
    rules.append(dict(table='filter', chain='INPUT', insert=True,
      args=['!', '-i', ncm, '-d', ADDRESS + '/32', '-p', protocol, '--dport', '53',
            *tag, '-j', 'DROP']))
  if snapshot['dns_input_rules']:
    for protocol in ('udp', 'tcp'):
      rules.append(dict(table='filter', chain='INPUT', insert=True,
        args=['-i', ncm, '-s', str(NETWORK), '-d', ADDRESS + '/32', '-p', protocol, '--dport', '53',
              *tag, '-j', 'ACCEPT']))
  return rules


def known_dns_input_rule(line, ncm):
  items = shlex.split(line)
  def value(flag):
    index = items.index(flag) if flag in items else -1
    return items[index + 1] if 0 <= index < len(items) - 1 else None
  if not (re.fullmatch(r'carrot-usb-internet-[0-9a-f]{12}', value('--comment') or '') and
          value('-i') == ncm and value('-d') == ADDRESS + '/32' and value('-p') in ('udp', 'tcp') and value('--dport') == '53'):
    return False
  index = items.index('-i')
  negated = index > 0 and items[index - 1] == '!'
  return ((value('-j') == 'DROP' and negated) or
          (value('-j') == 'ACCEPT' and not negated and value('-s') == str(NETWORK)))


class PrivateStore:
  def __init__(self, directory=STATE, owner=0):
    self.directory = Path(directory)
    if self.directory.is_symlink():
      raise OwnershipError('state_symlink')
    self.directory.mkdir(mode=0o700, parents=False, exist_ok=True)
    info = self.directory.stat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != owner or stat.S_IMODE(info.st_mode) != 0o700:
      raise OwnershipError('state_owner_or_mode')
    self.owner = owner

  def lock(self):
    fd = os.open(self.directory / 'service.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
      info = os.fstat(fd)
      if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != self.owner or stat.S_IMODE(info.st_mode) != 0o600:
        raise OwnershipError('state_lock_owner_or_mode')
      fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
      return fd
    except BaseException:
      os.close(fd)
      raise

  def read(self, name):
    if name not in ('owner.json', 'status.json'):
      raise OwnershipError('state_name_scope')
    path = self.directory / name
    try:
      fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    except FileNotFoundError:
      return None
    try:
      info = os.fstat(fd)
      if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or info.st_uid != self.owner or stat.S_IMODE(info.st_mode) != 0o600:
        raise OwnershipError('state_file_owner_or_mode')
      data = os.read(fd, 65537)
      after = os.fstat(fd)
      linked = path.lstat()
      if (len(data) > 65536 or len(data) != info.st_size or
          (info.st_dev, info.st_ino, info.st_mtime_ns, info.st_ctime_ns) !=
          (after.st_dev, after.st_ino, after.st_mtime_ns, after.st_ctime_ns) or
          (after.st_dev, after.st_ino) != (linked.st_dev, linked.st_ino)):
        raise OwnershipError('state_changed_during_read')
      return json.loads(data)
    finally:
      os.close(fd)

  def write(self, name, value):
    if name not in ('owner.json', 'status.json'):
      raise OwnershipError('state_name_scope')
    path = self.directory / name
    if path.is_symlink():
      raise OwnershipError('state_file_symlink')
    fd, temporary = tempfile.mkstemp(prefix='.usb-net-', dir=self.directory)
    try:
      with os.fdopen(fd, 'w', encoding='utf-8') as stream:
        os.fchmod(stream.fileno(), 0o600)
        json.dump(value, stream, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
      os.replace(temporary, path)
      self.sync()
    finally:
      Path(temporary).unlink(missing_ok=True)

  def remove(self, name):
    if name not in ('owner.json', 'status.json'):
      raise OwnershipError('state_name_scope')
    path = self.directory / name
    if path.is_symlink():
      raise OwnershipError('state_file_symlink')
    path.unlink(missing_ok=True)
    self.sync()

  def sync(self):
    fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
      os.fsync(fd)
    finally:
      os.close(fd)


class Runner:
  def __init__(self):
    self.paths = {}
    for name in ('ip', 'nmcli', 'iptables-legacy', 'dnsmasq'):
      path = shutil.which(name, path='/usr/sbin:/usr/bin:/sbin:/bin')
      if not path:
        raise NotReady(name.replace('-', '_') + '_missing')
      self.paths[name] = path

  def run(self, name, args, *, deadline=None):
    remaining = 3.0 if deadline is None else min(2.0, deadline - time.monotonic())
    if remaining <= 0:
      raise NotReady('probe_deadline')
    try:
      return subprocess.run([self.paths[name], *args], capture_output=True, text=True, timeout=remaining,
                            env={'PATH': '/usr/sbin:/usr/bin:/sbin:/bin', 'LC_ALL': 'C'})
    except (OSError, subprocess.TimeoutExpired) as exc:
      raise NotReady('command_unavailable_or_timed_out') from exc


class SystemProbe:
  def __init__(self, runner, root=Path('/'), marker_uid=0):
    self.runner, self.root = runner, Path(root)
    self.marker_uid = marker_uid

  def path(self, absolute):
    return self.root / str(absolute).lstrip('/')

  def json(self, command, args, deadline):
    result = self.runner.run(command, args, deadline=deadline)
    if result.returncode:
      raise NotReady('network_read_failed')
    try:
      return json.loads(result.stdout)
    except (ValueError, TypeError) as exc:
      raise NotReady('network_read_invalid') from exc

  def discover(self, deadline):
    gadget = self.path(GADGET)
    marker = self.path(MOBILE_STATE) / 'ncm-owned'
    try:
      info = marker.lstat()
      if not stat.S_ISREG(info.st_mode) or info.st_uid != self.marker_uid or marker.read_text().strip() != 'ncm.jetlink':
        raise NotReady('ncm_not_owned')
      target = gadget / 'functions/ncm.jetlink'
      link = gadget / 'configs/c.1/ncm.jetlink'
      if not link.is_symlink() or link.resolve() != target.resolve():
        raise NotReady('ncm_not_linked')
      udc = (gadget / 'UDC').read_text().strip()
      if not re.fullmatch('[A-Za-z0-9_.:-]{1,80}', udc):
        raise NotReady('udc_not_bound')
      if (self.path('/sys/class/udc') / udc / 'state').read_text().strip() != 'configured':
        raise NotReady('usb_not_configured')
      if (self.path('/sys/class/udc') / udc / 'current_speed').read_text().strip() not in ('super-speed', 'super-speed-plus'):
        raise NotReady('original_usb3_not_ready')
      ncm = ifname((target / 'ifname').read_text().strip())
      if self.path('/proc/sys/net/ipv4/ip_forward').read_text().strip() != '1':
        raise NotReady('forwarding_not_already_enabled')
    except OSError as exc:
      raise NotReady('ncm_not_present') from exc
    links = self.json('ip', ['-j', 'link', 'show'], deadline)
    indexes = {item['ifname']: item['ifindex'] for item in links if 'UP' in item.get('flags', [])}
    if ncm not in indexes:
      raise NotReady('ncm_not_up')
    addresses = self.json('ip', ['-j', '-4', 'address', 'show', 'dev', ncm], deadline)
    ipv4 = [item for interface in addresses for item in interface.get('addr_info', [])]
    if not any(item.get('local') == ADDRESS and item.get('prefixlen') == 24 for item in ipv4):
      raise NotReady('official_ncm_address_not_ready')
    if any(item.get('local') != ADDRESS or item.get('prefixlen') != 24 for item in ipv4):
      raise NotReady('ncm_has_unexpected_addresses')
    routes = self.json('ip', ['-j', '-4', 'route', 'show', 'table', 'all'], deadline)
    for route in routes:
      if route.get('dst', 'default') == 'default':
        continue
      network = ipaddress.ip_network(route['dst'], strict=False)
      if network.version == 4 and network.overlaps(NETWORK) and route.get('dev') != ncm:
        raise NotReady('ncm_subnet_collision')
    defaults = self.json('ip', ['-j', '-4', 'route', 'show', 'default'], deadline)
    # 경로 조회만 한다. 이 주소로 패킷을 보내거나 DNS로 지정하지 않는다.
    route = self.json('ip', ['-j', '-4', 'route', 'get', '1.1.1.1'], deadline)
    uplink = ifname(route[0].get('dev')) if route else None
    if (uplink not in indexes or uplink in (ncm, 'lo', 'wljetson0') or
        not any(item.get('dev') == uplink for item in defaults)):
      raise NotReady('wan_default_not_verified')
    dns_result = self.runner.run('nmcli', ['-g', 'IP4.DNS', 'device', 'show', uplink], deadline=deadline)
    if dns_result.returncode:
      raise NotReady('wan_dns_unavailable')
    dns = []
    for text in re.split(r'[\s,;|]+', dns_result.stdout.strip()):
      if not text:
        continue
      try:
        value = ipaddress.ip_address(text)
      except ValueError:
        continue
      if value.version != 4 or value.is_loopback or value.is_unspecified or value.is_multicast or value.is_link_local or value in NETWORK:
        continue
      actual = self.json('ip', ['-j', '-4', 'route', 'get', str(value)], deadline)
      if actual and actual[0].get('dev') == uplink and str(value) not in dns:
        dns.append(str(value))
      if len(dns) == 4:
        break
    if not dns:
      raise NotReady('wan_dns_not_peer_reachable')
    for table, chain in (('nat', 'POSTROUTING'), ('filter', 'FORWARD'), ('filter', 'INPUT')):
      result = self.runner.run('iptables-legacy', ['-w', '2', '-t', table, '-S', chain], deadline=deadline)
      if result.returncode:
        raise NotReady('legacy_backend_unavailable')
      if chain == 'FORWARD' and '-P FORWARD ACCEPT' not in result.stdout.splitlines():
        raise NotReady('forward_policy_requires_separate_review')
      if chain == 'INPUT':
        lines = result.stdout.splitlines()
        dns_rules = ('-P INPUT ACCEPT' not in lines or any(line.startswith('-A INPUT ') and
          not (('jetlink-mobile-owned-5599' in line and '--dport 5599' in line) or known_dns_input_rule(line, ncm)) for line in lines))
    return valid_snapshot(dict(ncm=ncm, ncm_index=indexes[ncm], uplink=uplink,
                               uplink_index=indexes[uplink], dns=dns, dns_input_rules=dns_rules))

  def dns_port_free(self):
    for family in ('', '6'):
      for protocol in ('udp', 'tcp'):
        path = self.path('/proc/net/' + protocol + family)
        try:
          rows = path.read_text().splitlines()[1:]
        except FileNotFoundError:
          continue
        for row in rows:
          fields = row.split()
          address, port = fields[1].split(':')
          if int(port, 16) != 53 or (protocol == 'tcp' and fields[3] != '0A'):
            continue
          target = ipaddress.IPv4Address(int(address, 16).to_bytes(4, 'little')) if not family else None
          if (not family and (target.is_unspecified or str(target) == ADDRESS)) or (family and int(address, 16) == 0):
            raise NotReady('dns_port_owned_by_another_process')


class Firewall:
  def __init__(self, runner):
    self.runner = runner

  def call(self, action, rule, deadline=None):
    args = ['-w', '2', '-t', rule['table'], action, rule['chain']]
    if action == '-I':
      args.append('1')
    return self.runner.run('iptables-legacy', [*args, *rule['args']], deadline=deadline)

  def exists(self, rule, deadline=None):
    code = self.call('-C', rule, deadline).returncode
    if code not in (0, 1):
      raise OwnershipError('rule_read_failed')
    return code == 0

  def tagged_count(self, table, chain, lease, deadline=None):
    result = self.runner.run('iptables-legacy', ['-w', '2', '-t', table, '-S', chain], deadline=deadline)
    if result.returncode:
      raise OwnershipError('rule_list_failed')
    tag = 'carrot-usb-internet-' + lease
    count = 0
    for line in result.stdout.splitlines():
      items = shlex.split(line)
      if '--comment' in items and items.index('--comment') + 1 < len(items) and items[items.index('--comment') + 1] == tag:
        count += 1
    return count

  def install(self, owner, save, guard=lambda: None):
    for table, chain in {(rule['table'], rule['chain']) for rule in owner['rules']}:
      if self.tagged_count(table, chain, owner['lease']):
        raise OwnershipError('preexisting_rule_not_adopted')
    for index, rule in enumerate(owner['rules']):
      guard()
      if self.exists(rule):
        raise OwnershipError('preexisting_rule_not_adopted')
      owner['attempted'].append(index)
      owner['phase'] = 'rule_intent'
      save(owner)
      guard()
      if self.call('-I' if rule['insert'] else '-A', rule).returncode or not self.exists(rule):
        raise OwnershipError('rule_add_not_verified')

  def healthy(self, owner):
    return all(self.exists(owner['rules'][index]) for index in owner['attempted'])

  def cleanup(self, owner):
    errors = []
    deadline = time.monotonic() + 12.0
    groups = {}
    for index in owner['attempted']:
      rule = owner['rules'][index]
      groups.setdefault((rule['table'], rule['chain']), []).append(rule)
    for (table, chain), rules in groups.items():
      try:
        if self.tagged_count(table, chain, owner['lease'], deadline) != sum(self.exists(rule, deadline) for rule in rules):
          errors.append('changed_or_duplicate_owned_rule')
          continue
        for rule in reversed(rules):
          if self.exists(rule, deadline) and (self.call('-D', rule, deadline).returncode or self.exists(rule, deadline)):
            errors.append('rule_delete_not_verified')
      except (OwnershipError, NotReady):
        errors.append('rule_cleanup_unavailable')
    if errors:
      raise OwnershipError(errors[0])


class DnsChild:
  def __init__(self, binary, directory):
    self.binary, self.directory = binary, Path(directory)
    self.child = None

  @staticmethod
  def _pid_exists(pid):
    try:
      os.kill(pid, 0)
    except ProcessLookupError:
      return False
    except PermissionError:
      return True
    return True

  def _pid_audit(self, directory_fd, value):
    data = (json.dumps(value, sort_keys=True) + '\n').encode()
    name = 'dnsmasq.pid.audit-' + sha(data) + '.json'
    try:
      fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600, dir_fd=directory_fd)
    except FileExistsError:
      fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
      try:
        info = os.fstat(fd)
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.geteuid() or info.st_nlink != 1 or
            stat.S_IMODE(info.st_mode) != 0o600 or info.st_size != len(data) or os.read(fd, len(data) + 1) != data):
          raise OwnershipError('dns_pid_audit_changed')
      finally:
        os.close(fd)
    else:
      with os.fdopen(fd, 'wb') as stream:
        os.fchmod(stream.fileno(), 0o600)
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.fsync(directory_fd)

  def _retire_pidfile(self, expected_pid=None):
    """제어 디렉터리의 사라진 자식 PID 파일만 기록한 뒤 제거한다."""
    directory = self.directory.lstat()
    if (not stat.S_ISDIR(directory.st_mode) or directory.st_uid != os.geteuid() or
        stat.S_IMODE(directory.st_mode) != 0o700):
      raise OwnershipError('dns_pid_directory_unsafe')
    directory_fd = os.open(self.directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
    fd = None
    try:
      opened_directory = os.fstat(directory_fd)
      if (directory.st_dev, directory.st_ino) != (opened_directory.st_dev, opened_directory.st_ino):
        raise OwnershipError('dns_pid_directory_changed')
      try:
        linked = os.stat('dnsmasq.pid', dir_fd=directory_fd, follow_symlinks=False)
      except FileNotFoundError:
        return False
      if stat.S_ISLNK(linked.st_mode):
        raise OwnershipError('dns_pid_file_symlink')
      fd = os.open('dnsmasq.pid', os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK, dir_fd=directory_fd)
      info = os.fstat(fd)
      nobody = pwd.getpwnam('nobody')
      allowed = {(os.geteuid(), os.getegid()), (nobody.pw_uid, nobody.pw_gid)}
      if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or
          (info.st_uid, info.st_gid) not in allowed or stat.S_IMODE(info.st_mode) not in (0o600, 0o644) or
          not 1 <= info.st_size <= 32):
        raise OwnershipError('dns_pid_file_owner_or_shape')
      identity = lambda value: (value.st_dev, value.st_ino, value.st_size, value.st_mtime_ns, value.st_ctime_ns,
                                value.st_uid, value.st_gid, stat.S_IMODE(value.st_mode), value.st_nlink)
      raw = os.read(fd, 33)
      if (len(raw) != info.st_size or identity(linked) != identity(info) or
          identity(os.fstat(fd)) != identity(info) or not re.fullmatch(rb'[1-9][0-9]{0,9}\n?', raw)):
        raise OwnershipError('dns_pid_file_invalid_or_changed')
      pid = int(raw)
      if not 2 <= pid <= 2147483647 or (expected_pid is not None and pid != expected_pid):
        raise OwnershipError('dns_pid_file_pid_changed')
      if self._pid_exists(pid):
        raise OwnershipError('dns_pid_file_live')
      self._pid_audit(directory_fd, {'version': 1, 'pid': pid, 'raw_hex': raw.hex(),
                                     'source_identity': list(identity(info))})
      current_directory = self.directory.lstat()
      if ((current_directory.st_dev, current_directory.st_ino, current_directory.st_uid,
           stat.S_IMODE(current_directory.st_mode)) !=
          (directory.st_dev, directory.st_ino, directory.st_uid, 0o700) or
          identity(os.fstat(fd)) != identity(info) or
          identity(os.stat('dnsmasq.pid', dir_fd=directory_fd, follow_symlinks=False)) != identity(info)):
        raise OwnershipError('dns_pid_file_changed_before_cleanup')
      if self._pid_exists(pid):
        raise OwnershipError('dns_pid_file_live')
      os.unlink('dnsmasq.pid', dir_fd=directory_fd)
      os.fsync(directory_fd)
      return True
    finally:
      if fd is not None:
        os.close(fd)
      os.close(directory_fd)

  def args(self, snapshot):
    user = pwd.getpwnam('nobody')
    group = grp.getgrgid(user.pw_gid).gr_name
    return [self.binary, '--keep-in-foreground', '--conf-file=/dev/null', '--bind-interfaces',
            '--listen-address=' + ADDRESS, '--port=53', '--no-dhcp-interface=' + snapshot['ncm'],
            '--no-resolv', '--no-hosts', '--cache-size=128', '--user=nobody', '--group=' + group,
            '--pid-file=' + str(self.directory / 'dnsmasq.pid'), *('--server=' + server for server in snapshot['dns'])]

  @staticmethod
  def process_identity(pid):
    process = Path('/proc') / str(pid)
    try:
      text = (process / 'stat').read_text().rsplit(')', 1)[1].split()
      if text[0] == 'Z':
        return None
      return {'start': text[19], 'argv_sha256': sha((process / 'cmdline').read_bytes())}
    except FileNotFoundError:
      return None

  def start(self, snapshot):
    self._retire_pidfile()
    args = self.args(snapshot)
    self.child = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, close_fds=True, start_new_session=True)
    try:
      time.sleep(0.15)
      if self.child.poll() is not None:
        raise OwnershipError('dns_child_start_failed')
      identity = self.process_identity(self.child.pid)
      if identity is None or identity['argv_sha256'] != sha(b'\0'.join(os.fsencode(arg) for arg in args) + b'\0'):
        raise OwnershipError('dns_child_identity_failed')
      return {'pid': self.child.pid, **identity, 'args': args}
    except BaseException:
      # 시작 도중 실패한 직접 자식도 회수한다. PID 파일의 임의 프로세스는 사용하지 않는다.
      if self.child.poll() is None:
        self.child.terminate()
        try:
          self.child.wait(timeout=3)
        except subprocess.TimeoutExpired:
          self.child.kill()
          self.child.wait(timeout=1)
      else:
        self.child.wait()
      self.child = None
      raise

  def healthy(self, record):
    return record is not None and self.process_identity(record['pid']) == {key: record[key] for key in ('start', 'argv_sha256')}

  def stop(self, record, snapshot):
    self._stop_process(record, snapshot)
    if record is not None:
      self._retire_pidfile(expected_pid=record['pid'])

  def _stop_process(self, record, snapshot):
    if record is None:
      return
    if record.get('args') != self.args(snapshot) or type(record.get('pid')) is not int or record['pid'] <= 1:
      raise OwnershipError('dns_owner_record_invalid')
    current = self.process_identity(record['pid'])
    if current is None:
      if self.child is not None:
        self.child.wait(timeout=1)
        self.child = None
      return
    expected = {key: record[key] for key in ('start', 'argv_sha256')}
    if current.get('start') != expected['start']:
      return  # 예전 자식은 사라졌고 PID가 재사용됐다. 새 프로세스는 건드리지 않는다.
    if current != expected:
      raise OwnershipError('dns_process_changed')
    fd = None
    if hasattr(os, 'pidfd_open'):
      try:
        fd = os.pidfd_open(record['pid'])
      except OSError as exc:
        if exc.errno not in (errno.ENOSYS, errno.EINVAL, errno.ESRCH):
          raise
    try:
      latest = self.process_identity(record['pid'])
      if latest is None or latest.get('start') != expected['start']:
        return
      if latest != expected:
        raise OwnershipError('dns_process_changed_before_stop')
      if fd is not None and hasattr(signal, 'pidfd_send_signal'):
        signal.pidfd_send_signal(fd, signal.SIGTERM)
      else:
        os.kill(record['pid'], signal.SIGTERM)
      deadline = time.monotonic() + 3
      while self.process_identity(record['pid']) == expected and time.monotonic() < deadline:
        time.sleep(.02)
      if self.process_identity(record['pid']) == expected:
        if fd is not None and hasattr(signal, 'pidfd_send_signal'):
          signal.pidfd_send_signal(fd, signal.SIGKILL)
        else:
          if self.process_identity(record['pid']) != expected:
            raise OwnershipError('dns_process_changed_before_kill')
          os.kill(record['pid'], signal.SIGKILL)
      if self.child is not None and self.child.pid == record['pid']:
        self.child.wait(timeout=1)
        self.child = None
      elif self.process_identity(record['pid']) == expected:
        raise OwnershipError('dns_child_not_reaped')
    except ProcessLookupError:
      pass
    finally:
      if fd is not None:
        os.close(fd)
      if self.child is not None and self.child.pid == record['pid'] and self.child.poll() is not None:
        self.child.wait(timeout=1)
        self.child = None


class Controller:
  def __init__(self, probe, firewall, dns, store, boot_id):
    self.probe, self.firewall, self.dns, self.store, self.boot_id = probe, firewall, dns, store, boot_id
    self.owner = None
    self.last_status = None

  def status(self, state, reason):
    value = dict(state=state, reason=reason, internet_verified=False,
                 rules=len(self.owner['attempted']) if self.owner else 0,
                 dns_count=len(self.owner['snapshot']['dns']) if self.owner else 0)
    if value != self.last_status:
      self.store.write('status.json', value)
      self.last_status = value
    return value

  def validate_owner(self, owner):
    if (not isinstance(owner, dict) or owner.get('version') != 1 or owner.get('boot_id') != self.boot_id or
        owner.get('rules') != rules_for(owner.get('snapshot', {}), owner.get('lease', ''))):
      raise OwnershipError('owner_journal_invalid_or_other_boot')
    attempted = owner.get('attempted')
    if (not isinstance(attempted, list) or any(type(index) is not int for index in attempted) or
        attempted != list(range(len(attempted))) or len(attempted) > len(owner['rules'])):
      raise OwnershipError('owner_attempts_invalid')
    return owner

  def save(self, owner):
    self.store.write('owner.json', owner)

  def cleanup(self):
    if self.owner is None:
      return
    errors = []
    try:
      self.dns.stop(self.owner.get('dns_child'), self.owner['snapshot'])
    except (OSError, RuntimeError, subprocess.SubprocessError):
      errors.append('dns_cleanup_failed')
    try:
      self.firewall.cleanup(self.owner)
    except (OSError, RuntimeError, subprocess.SubprocessError):
      errors.append('rule_cleanup_failed')
    if errors:
      self.owner['phase'] = 'cleanup_failed'
      self.save(self.owner)
      raise OwnershipError(errors[0])
    self.store.remove('owner.json')
    self.owner = None

  def recover(self):
    previous = self.store.read('owner.json')
    if previous is not None:
      self.owner = self.validate_owner(previous)
      self.cleanup()

  def step(self, deadline):
    try:
      snapshot = valid_snapshot(self.probe.discover(deadline))
    except NotReady as exc:
      self.cleanup()
      return self.status('standby', str(exc))
    if (self.owner is not None and self.owner['snapshot'] == snapshot and
        self.dns.healthy(self.owner.get('dns_child')) and self.firewall.healthy(self.owner)):
      return self.status('configured', 'ncm_dns_and_scoped_rules_ready')
    self.cleanup()
    self.probe.dns_port_free()
    lease = uuid.uuid4().hex[:12]
    self.owner = dict(version=1, boot_id=self.boot_id, lease=lease, snapshot=snapshot,
                      rules=rules_for(snapshot, lease), attempted=[], dns_child=None, phase='prepared')
    self.save(self.owner)
    try:
      def confirm_current():
        if valid_snapshot(self.probe.discover(deadline)) != snapshot:
          raise NotReady('network_changed_during_setup')
      self.firewall.install(self.owner, self.save, confirm_current)
      confirm_current()
      self.owner['phase'] = 'dns_intent'
      self.save(self.owner)
      self.owner['dns_child'] = self.dns.start(snapshot)
      self.owner['phase'] = 'configured'
      self.save(self.owner)
    except BaseException:
      self.cleanup()
      raise
    return self.status('configured', 'ncm_dns_and_scoped_rules_ready')


def notify(message):
  address = os.environ.get('NOTIFY_SOCKET')
  if not address or not address.startswith(('/', '@')):
    return
  if address.startswith('@'):
    address = '\0' + address[1:]
  try:
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as channel:
      channel.connect(address)
      channel.sendall(message.encode())
  except OSError:
    pass


def main():
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--check', action='store_true')
  args = parser.parse_args()
  if os.geteuid() != 0:
    raise SystemExit('root is required for the scoped USB Internet service')
  try:
    runner = Runner()
    probe = SystemProbe(runner)
    if args.check:
      snapshot = probe.discover(time.monotonic() + 4)
      print(json.dumps(dict(state='eligible', dns_count=len(snapshot['dns']), backend='iptables-legacy', changes_made=False)))
      return 0
  except NotReady as exc:
    if args.check:
      print(json.dumps(dict(state='standby', reason=str(exc), changes_made=False)))
      return 0
    runner = probe = None
  store = PrivateStore()
  lock_fd = store.lock()
  stop = threading.Event()
  for number in (signal.SIGTERM, signal.SIGINT):
    signal.signal(number, lambda *_args: stop.set())
  start = time.monotonic()
  notified = False
  controller = None
  try:
    while not stop.is_set():
      if runner is None:
        try:
          runner = Runner()
          probe = SystemProbe(runner)
        except NotReady as exc:
          value = dict(state='standby', reason=str(exc), internet_verified=False, rules=0, dns_count=0)
          store.write('status.json', value)
      if runner is not None:
        if controller is None:
          boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
          controller = Controller(probe, Firewall(runner), DnsChild(runner.paths['dnsmasq'], store.directory), store, boot)
          controller.recover()
        try:
          value = controller.step(time.monotonic() + 4)
        except NotReady as exc:
          value = controller.status('standby', str(exc))
      if not notified and (value['state'] == 'configured' or time.monotonic() - start >= STARTUP_WAIT):
        notify('READY=1\nSTATUS=' + value['state'] + ': ' + value['reason'])
        notified = True
      elif notified:
        notify('STATUS=' + value['state'] + ': ' + value['reason'])
      stop.wait(POLL_SECONDS)
    return 0
  except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
    if controller is not None:
      controller.status('error', error_reason(exc))
    notify('STATUS=USB Internet setup or cleanup error; private journal preserved')
    return 1
  finally:
    try:
      if controller is not None:
        controller.cleanup()
    finally:
      os.close(lock_fd)


if __name__ == '__main__':
  raise SystemExit(main())
