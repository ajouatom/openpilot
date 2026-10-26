"""검증한 콤마 NCM에만 DHCP를 적용한다. 커널·기존 연결·차량 제어는 변경하지 않는다."""
from __future__ import annotations

import argparse
import configparser
from contextlib import contextmanager
import fcntl
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import signal
import stat
import subprocess
import tempfile
import time

RELEASE = '5.15.148-tegra-j401-r3647'
VID, PID, MAC = '1209', '0001', '02:4a:4c:60:00:02'
UUID = '79d2de5b-61b1-4635-aad7-ec59a09b0f95'
PROFILE = 'carrot-ncm-internet'
TAG = 'carrot-usb-net-client:v1:' + UUID
SUBNET = ipaddress.IPv4Network('192.168.60.0/24')
GATEWAY = '192.168.60.1'
PROBE = 'api.github.com'
METRIC = 700
FOREIGN_SNAPSHOT_FORMAT = 'nm_profiles_and_opaque_files_v2'
STATE = '/var/lib/carrot-usb-net-client'
JOURNAL = STATE + '/installation.json'
CONFIG = '/etc/carrot-usb-net-client.json'
PROGRAM = '/usr/local/libexec/carrot-usb-net-client.py'
RULE = '/etc/udev/rules.d/70-carrot-usb-net-client.rules'
UNIT = '/etc/systemd/system/carrot-usb-net-client@.service'
KEYFILE = '/etc/NetworkManager/system-connections/' + PROFILE + '.nmconnection'
MODULES = {
    'usbnet': ('f4aa1847ac8d19a6877c1c306d8e0797658f709564560a6913b2e40bd3b50ce6', ''),
    'cdc_ether': ('0299532b4dee842afe0a27690e1a25cefca93c29e49fd7a29c771358713995a0', 'usbnet'),
    'cdc_ncm': ('452c6b114e0753fbcdec1cbcb19834f4dde9e2e231ebf13b04d9f280052fcb0e', 'usbnet,cdc_ether'),
}
VERMAGIC = RELEASE + ' SMP preempt mod_unload modversions aarch64'
PREVIOUS_PROGRAM_SHA256 = 'cda85b96d165d93de19000a82b8feaa94afdb48b20d025f4846db82cf70abde5'
NM = '/usr/bin/nmcli'
IP = '/usr/sbin/ip'
PROPERTIES = {
    'connection.uuid': UUID, 'connection.id': PROFILE, 'connection.type': '802-3-ethernet',
    'connection.stable-id': TAG, 'connection.interface-name': '',
    'connection.autoconnect': 'no', '802-3-ethernet.mac-address': MAC.upper(),
    '802-3-ethernet.cloned-mac-address': 'preserve',
    'ipv4.method': 'auto', 'ipv4.ignore-auto-routes': 'yes', 'ipv4.route-metric': '700',
    'ipv4.dns': GATEWAY, 'ipv4.ignore-auto-dns': 'yes', 'ipv4.dns-priority': '700',
    'ipv6.method': 'disabled',
}

# NM 1.36의 nmcli는 connection.uuid를 생성 명령에서도 수정 불가로 처리한다.
# 설치된 libNM API로 UUID를 처음부터 지정하며, 연결 활성화는 하지 않는다.
NM_CREATE_CODE = r'''
import json,re,sys
import gi
gi.require_version('NM','1.0')
from gi.repository import NM,Gio,GLib
if sys.argv[1:] not in (['--apply'],['--validate-only']):
    raise SystemExit(2)
uuid='79d2de5b-61b1-4635-aad7-ec59a09b0f95'
name='carrot-ncm-internet'
c=NM.SimpleConnection.new()
connection=NM.SettingConnection.new()
for key,value in {'id':name,'uuid':uuid,'type':'802-3-ethernet','autoconnect':False,
                  'stable-id':'carrot-usb-net-client:v1:'+uuid}.items():
    connection.set_property(key,value)
c.add_setting(connection)
wired=NM.SettingWired.new()
wired.set_property('mac-address','02:4a:4c:60:00:02')
wired.set_property('cloned-mac-address','preserve')
c.add_setting(wired)
ipv4=NM.SettingIP4Config.new()
for key,value in {'method':'auto','ignore-auto-routes':True,'ignore-auto-dns':True,
                  'route-metric':700,'dns-priority':700,'dhcp-timeout':25}.items():
    ipv4.set_property(key,value)
ipv4.add_dns('192.168.60.1')
ipv4.add_route(NM.IPRoute.new(2,'0.0.0.0',0,'192.168.60.1',700))
c.add_setting(ipv4)
ipv6=NM.SettingIP6Config.new()
ipv6.set_property('method','disabled')
c.add_setting(ipv6)
c.verify()
if sys.argv[1:] == ['--validate-only']:
    print(json.dumps({'settings_verified':True,'connection_saved_or_activated':False,
                      'profile_id':connection.get_id(),'uuid':connection.get_uuid(),
                      'autoconnect':connection.get_autoconnect()}),flush=True)
    raise SystemExit(0)
client=NM.Client.new(None)
if any(x.get_uuid()==uuid or x.get_id()==name for x in client.get_connections()):
    print('NM_CREATE_EXISTING_PROFILE_REFUSED',flush=True)
    raise SystemExit(3)
loop=GLib.MainLoop()
cancel=Gio.Cancellable()
state={'success':False}
def done(source,result,_data):
    try:
        remote=source.add_connection_finish(result)
        state['success']=remote is not None and remote.get_uuid()==uuid
    except GLib.Error as error:
        domain=str(error.domain)
        print(json.dumps({'nm_create_error_code':int(error.code),
                          'nm_create_error_domain':domain if re.fullmatch('[a-z0-9_-]{1,80}',domain) else 'redacted'}),flush=True)
    finally:
        loop.quit()
def expired():
    cancel.cancel()
    loop.quit()
    return False
timer=GLib.timeout_add_seconds(18,expired)
client.add_connection_async(c,True,cancel,done,None)
loop.run()
if state['success']:
    print('NM_CREATE_FIXED_UUID_OK',flush=True)
    raise SystemExit(0)
raise SystemExit(4)
'''


class Refused(RuntimeError):
    def __init__(self, code, details=None):
        super().__init__(code)
        self.details = details or {}


# 앞서 확인한 원격 주소와 USB 인터페이스를 고정해 TLS HEAD 요청만 검사한다.
BOUND_HTTPS_CODE = r'''
import http.client,ipaddress,json,re,socket,ssl,sys
try:
    context=ssl.create_default_context()
    context.minimum_version=ssl.TLSVersion.TLSv1_2
    if not context.check_hostname or context.verify_mode!=ssl.CERT_REQUIRED:
        raise ValueError('certificate_policy')
    if sys.argv[1:]==['--validate-only']:
        print(json.dumps({'stdlib_tls_ready':True,'device_binding_available':hasattr(socket,'SO_BINDTODEVICE'),
                          'hostname_verification':context.check_hostname,'network_used':False}),flush=True)
        raise SystemExit(0)
    if len(sys.argv)!=5:
        raise ValueError('arguments')
    interface,local,remote,host=sys.argv[1:]
    if not re.fullmatch('[A-Za-z0-9][A-Za-z0-9_.-]{0,14}',interface) or host!='api.github.com':
        raise ValueError('target')
    source=ipaddress.IPv4Address(local)
    target=ipaddress.IPv4Address(remote)
    network=ipaddress.IPv4Network('192.168.60.0/24')
    if source not in network or not 2<=int(source)-int(network.network_address)<=254 or not target.is_global:
        raise ValueError('address')
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as raw:
        raw.settimeout(5)
        raw.setsockopt(socket.SOL_SOCKET,socket.SO_BINDTODEVICE,interface.encode('ascii')+b'\0')
        raw.bind((str(source),0))
        raw.connect((str(target),443))
        with context.wrap_socket(raw,server_hostname=host) as secure:
            secure.settimeout(5)
            own=secure.getsockname()[0];peer=secure.getpeername()[0]
            secure.sendall(('HEAD / HTTP/1.1\r\nHost: '+host+'\r\nUser-Agent: CarrotUSBProbe/1.0\r\nConnection: close\r\n\r\n').encode('ascii'))
            response=http.client.HTTPResponse(secure)
            try:
                response.begin();status=response.status
            finally:
                response.close()
            print(str(status)+' '+own+' '+peer,flush=True)
except ssl.SSLCertVerificationError:
    print('BOUND_HTTPS_CERTIFICATE_FAILED',flush=True);raise SystemExit(5)
except ssl.SSLError:
    print('BOUND_HTTPS_TLS_FAILED',flush=True);raise SystemExit(6)
except (socket.timeout,TimeoutError):
    print('BOUND_HTTPS_TIMEOUT',flush=True);raise SystemExit(7)
except http.client.HTTPException:
    print('BOUND_HTTPS_PROTOCOL_FAILED',flush=True);raise SystemExit(8)
except OSError:
    print('BOUND_HTTPS_SOCKET_FAILED',flush=True);raise SystemExit(9)
except (ValueError,AttributeError):
    print('BOUND_HTTPS_INPUT_OR_FEATURE_INVALID',flush=True);raise SystemExit(10)
'''


def need(value, code):
    if not value:
        raise Refused(code)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def encode(value):
    return (json.dumps(value, sort_keys=True, indent=2) + '\n').encode()


def public_code(value):
    return value if isinstance(value, str) and re.fullmatch(r'[a-z][a-z0-9_]{0,79}', value) else 'not_recorded'


def config_payload(program_sha):
    return encode({'schema': 1, 'uuid': UUID, 'tag': TAG, 'vid': VID, 'pid': PID,
                   'default_profile_mac': MAC, 'mac_policy': 'unique_target_local_unicast',
                   'release': RELEASE, 'program_sha256': program_sha})


def sealed_source(path, expected):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    try:
        info = os.fstat(fd)
        need(stat.S_ISREG(info.st_mode) and 0 < info.st_size <= 131072, 'installer_source_invalid')
        pieces = []
        remaining = 131073
        while remaining:
            block = os.read(fd, min(65536, remaining))
            if not block:
                break
            pieces.append(block)
            remaining -= len(block)
        data = b''.join(pieces)
        need(len(data) == info.st_size and digest(data) == expected, 'installer_source_changed')
        return data
    finally:
        os.close(fd)


def keyfile_semantics(data):
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    parser.optionxform = str
    parser.read_string(data.decode('utf-8'))
    value = {section: dict(parser.items(section)) for section in parser.sections()}
    stamp = value.get('connection', {}).pop('timestamp', None)
    need(stamp is None or (stamp.isdecimal() and 0 <= int(stamp) < 2**64), 'profile_timestamp_invalid')
    for name in ('ethernet', '802-3-ethernet'):
        if 'mac-address' in value.get(name, {}):
            value[name]['mac-address'] = value[name]['mac-address'].lower()
    return value


def foreign_file_fingerprint(name, data):
    """접속 설정의 시각 정보만 제외하고, 보관 파일은 원문 지문으로 보존한다."""
    if not name.startswith('.') and name.endswith('.nmconnection'):
        try:
            value = keyfile_semantics(data)
        except (configparser.Error, UnicodeError, Refused, ValueError):
            pass
        else:
            if 'connection' in value:
                return digest(b'nm-keyfile-semantics-v2\0' + encode(value))
    return digest(b'opaque-network-file-v2\0' + data)


def with_mac(semantics, mac):
    result = json.loads(json.dumps(semantics))
    names = [name for name in ('ethernet', '802-3-ethernet') if 'mac-address' in result.get(name, {})]
    need(len(names) == 1, 'profile_mac_field_invalid')
    result[names[0]]['mac-address'] = mac.lower()
    return result


def unit_payload():
    return (f'[Unit]\nDescription=Verified comma USB NCM internet client\n'
            'Wants=NetworkManager.service\nAfter=NetworkManager.service\n'
            'StartLimitIntervalSec=300\nStartLimitBurst=3\n'
            '[Service]\nType=oneshot\n'
            f'ExecStart=/usr/bin/python3 -I -B {PROGRAM} connect-index %I\n'
            'TimeoutStartSec=90\nRestart=on-failure\nRestartSec=15\nUMask=0077\n').encode()


def rule_payload():
    # idVendor와 idProduct는 같은 USB 장치 부모에 있고 driver는 별도 worker에서 확인한다.
    return ('ACTION=="add|move", SUBSYSTEM=="net", '
            f'ATTRS{{idVendor}}=="{VID}", ATTRS{{idProduct}}=="{PID}", '
            'TAG+="systemd", ENV{SYSTEMD_WANTS}+="carrot-usb-net-client@%s{ifindex}.service", '
            'RUN{program}+="/usr/bin/systemctl --no-block start carrot-usb-net-client@%s{ifindex}.service"\n').encode()


class Runner:
    def __call__(self, args, *, timeout=25):
        try:
            result = subprocess.run(args, stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                    timeout=timeout, env={'PATH': '/usr/sbin:/usr/bin:/sbin:/bin', 'LANG': 'C', 'LC_ALL': 'C'})
        except subprocess.TimeoutExpired as error:
            raise Refused('command_timeout_' + Path(args[0]).name) from error
        except FileNotFoundError as error:
            raise Refused('required_command_missing',{'command_program':Path(args[0]).name}) from error
        need(len(result.stdout) <= 131072 and len(result.stderr) <= 131072, 'command_output_too_large')
        if result.returncode != 0:
            details = {'command_exit_code': result.returncode}
            if Path(args[0]).name == 'nmcli':
                command = args[5:]
                operation = 'other'
                if command[:2] == ['connection', 'add']:
                    operation = 'connection_add'
                elif command[:2] == ['connection', 'modify']:
                    operation = 'connection_modify'
                elif command[:1] == ['-g']:
                    operation = 'connection_read'
                elif 'delete' in command:
                    operation = 'connection_delete'
                text = result.stderr.lower()
                category = ('permission_denied' if any(x in text for x in ('not authorized', 'insufficient privileges')) else
                            'invalid_argument_or_setting' if any(x in text for x in ('invalid', 'unknown property', 'unknown setting')) else
                            'missing_connection' if 'unknown connection' in text else 'unclassified')
                details.update(command_operation=operation, command_error_category=category)
            if Path(args[0]).name == 'resolvectl':
                operation = 'query' if 'query' in args else 'other'
                if len(args) > 1 and args[1] in ('dns','default-route'):
                    operation = args[1].replace('-','_') + ('_read' if len(args)==3 else '_write')
                text = result.stderr.lower()
                category = ('permission_denied' if any(x in text for x in ('permission denied','access denied','not authorized')) else
                    'missing_link' if any(x in text for x in ('no such link','failed to resolve interface','no such device')) else
                    'no_usable_dns_scope' if any(x in text for x in ('no appropriate name servers or networks','no suitable dns scope')) else
                    'dnssec_failed' if 'dnssec' in text else
                    'nxdomain' if 'nxdomain' in text else
                    'server_unreachable_or_timeout' if any(x in text for x in ('timed out','timeout','unreachable','all attempts')) else
                    'unsupported_option' if any(x in text for x in ('unrecognized option','unknown option')) else 'unclassified')
                details.update(command_operation=operation,command_error_category=category)
            if Path(args[0]).name == 'python3' and BOUND_HTTPS_CODE in args:
                markers={'BOUND_HTTPS_CERTIFICATE_FAILED':'certificate_failed','BOUND_HTTPS_TLS_FAILED':'tls_failed',
                         'BOUND_HTTPS_TIMEOUT':'timeout','BOUND_HTTPS_PROTOCOL_FAILED':'protocol_failed',
                         'BOUND_HTTPS_SOCKET_FAILED':'socket_or_binding_failed',
                         'BOUND_HTTPS_INPUT_OR_FEATURE_INVALID':'invalid_input_or_feature'}
                details.update(command_operation='bound_https_head',
                               command_error_category=markers.get(result.stdout.strip(),'unclassified'))
            raise Refused('command_failed_' + Path(args[0]).name, details)
        return result.stdout.strip()


class Client:
    def __init__(self, root=Path('/'), runner=None, release=RELEASE, uid=None):
        self.root = Path(root).resolve()
        self.run = runner or Runner()
        self.release = release
        self.uid = os.geteuid() if uid is None else uid

    def path(self, absolute):
        need(absolute.startswith('/') and '..' not in Path(absolute).parts, 'invalid_fixed_path')
        return self.root / absolute.lstrip('/')

    def safe_parent(self, path):
        for item in reversed(path.parents):
            if item == self.root.parent:
                continue
            if item.is_relative_to(self.root):
                need(item.is_dir() and not item.is_symlink(), 'unsafe_parent')
                need(item.stat().st_uid == self.uid and not item.stat().st_mode & 0o022, 'untrusted_parent')

    def fact(self, absolute):
        path = self.path(absolute)
        self.safe_parent(path)
        try:
            info = path.lstat()
        except FileNotFoundError:
            return None
        need(stat.S_ISREG(info.st_mode) and info.st_uid == self.uid and not info.st_mode & 0o022, 'unsafe_owned_file')
        return {'sha256': digest(path.read_bytes()), 'mode': stat.S_IMODE(info.st_mode),
                'inode': info.st_ino, 'uid': info.st_uid, 'bytes': info.st_size}

    def save(self, absolute, data, mode=0o600, *, new=False):
        path = self.path(absolute)
        self.safe_parent(path)
        if new:
            fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, mode)
            with os.fdopen(fd, 'wb') as output:
                os.fchmod(output.fileno(), mode)
                output.write(data)
                output.flush()
                os.fsync(output.fileno())
        else:
            self.fact(absolute)
            fd, name = tempfile.mkstemp(prefix='.carrot-usb-', dir=path.parent)
            try:
                os.fchmod(fd, mode)
                with os.fdopen(fd, 'wb') as output:
                    output.write(data)
                    output.flush()
                    os.fsync(output.fileno())
                os.replace(name, path)
            finally:
                if os.path.exists(name):
                    os.unlink(name)
        folder = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(folder)
        finally:
            os.close(folder)

    def load(self):
        need(self.fact(JOURNAL) is not None, 'installation_record_missing')
        result = json.loads(self.path(JOURNAL).read_text())
        need(result.get('schema') == 1 and result.get('uuid') == UUID and result.get('tag') == TAG, 'installation_record_mismatch')
        return result

    def module_guard(self):
        need(self.release == RELEASE, 'kernel_release_changed')
        expected_paths = []
        for name, (expected, dependencies) in MODULES.items():
            file = f'/lib/modules/{RELEASE}/kernel/drivers/net/usb/{name}.ko'
            path = self.path(file)
            need(path.is_file() and not path.is_symlink() and digest(path.read_bytes()) == expected, 'module_hash_mismatch')
            info = path.stat()
            need(info.st_uid == self.uid and not info.st_mode & 0o022, 'module_permissions_unsafe')
            resolved = Path(self.run(['/usr/sbin/modinfo', '-n', name]))
            need(resolved.resolve(strict=True) == path.resolve(strict=True), 'module_resolution_changed')
            need(self.run(['/usr/sbin/modinfo', '-F', 'vermagic', str(path)]) == VERMAGIC, 'module_vermagic_mismatch')
            actual = self.run(['/usr/sbin/modinfo', '-F', 'depends', str(path)])
            need(set(filter(None, actual.split(','))) == set(filter(None, dependencies.split(','))), 'module_dependencies_changed')
            expected_paths.append(path.resolve(strict=True))
        lines = self.run(['/usr/sbin/modprobe', '--show-depends', 'cdc_ncm']).splitlines()
        entries = [line.split() for line in lines if line.strip()]
        need(len(entries) == 3 and all(len(row) == 2 and row[0] == 'insmod' for row in entries),
             'module_install_override_or_dependency_changed')
        need([Path(row[1]).resolve(strict=True) for row in entries] == expected_paths, 'module_dependency_paths_changed')

    def identities(self):
        folder = self.path('/sys/class/net')
        return [self.identity(p.name) for p in sorted(folder.iterdir()) if self.matches(p.name)]

    def matches(self, name):
        try:
            self.identity(name)
            return True
        except (Refused, OSError, ValueError):
            return False

    def identity(self, name):
        need(re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]{0,14}', name), 'interface_name_invalid')
        folder = self.path('/sys/class/net') / name
        interface = (folder / 'device').resolve(strict=True)
        need(interface.is_relative_to(self.path('/sys/devices')), 'interface_not_usb_sysfs')
        need((interface / 'driver').resolve(strict=True).name == 'cdc_ncm', 'interface_driver_mismatch')
        parents = [p for p in (interface, *interface.parents) if p.is_relative_to(self.path('/sys/devices'))]
        devices = [p for p in parents if (p / 'idVendor').is_file() and (p / 'idProduct').is_file()]
        need(devices, 'usb_parent_missing')
        device = devices[0]
        need((device / 'idVendor').read_text().strip().lower() == VID and
             (device / 'idProduct').read_text().strip().lower() == PID, 'usb_identity_mismatch')
        mac = (folder / 'address').read_text().strip().lower()
        need(re.fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', mac) is not None and
             int(mac[:2], 16) & 3 == 2, 'interface_mac_invalid')
        return {'name': name, 'ifindex': int((folder / 'ifindex').read_text()),
                'interface': str(interface), 'usb_parent': str(device), 'mac': mac}

    def unique_identity(self, interface=None, index=None):
        identities = self.identities()
        need(len(identities) == 1, 'target_interface_count_not_one')
        result = identities[0]
        need(interface is None or result['name'] == interface, 'target_interface_name_changed')
        need(index is None or result['ifindex'] == index, 'target_interface_index_not_present')
        return result

    def nm(self, *args, timeout=25):
        return self.run([NM, '--colors', 'no', '--escape', 'no', *args], timeout=timeout)

    def profile_ids(self):
        result = self.nm('-g', 'UUID', 'connection', 'show').splitlines()
        need(len(result) <= 256 and all(re.fullmatch(r'[0-9a-f-]{36}', x) for x in result), 'profile_inventory_invalid')
        return result

    def profile_values(self):
        return {key: self.nm('-g', key, 'connection', 'show', 'uuid', UUID) for key in PROPERTIES}

    def profile_guard(self, journal):
        need(UUID in self.profile_ids(), 'owned_profile_missing')
        need(self.fact(KEYFILE) is not None and stat.S_IMODE(self.path(KEYFILE).stat().st_mode) == 0o600, 'profile_file_missing_or_permissions')
        need(keyfile_semantics(self.path(KEYFILE).read_bytes()) == journal['profile_semantics'], 'profile_changed')
        actual = self.profile_values()
        expected = dict(PROPERTIES)
        expected['802-3-ethernet.mac-address'] = journal.get('profile_mac', MAC).upper()
        need(actual == expected, 'profile_live_settings_changed')
        # 기본 경로가 이 프로필에만 명시되었는지 별도 검사한다.
        route = self.nm('-g', 'ipv4.routes', 'connection', 'show', 'uuid', UUID)
        need(re.sub(r'\s+', '', route) in ('{ip=0.0.0.0/0,nh=192.168.60.1,mt=700}',
                                          '0.0.0.0/0192.168.60.1700'), 'profile_default_route_changed')
        need(self.nm('-g', 'ipv4.addresses', 'connection', 'show', 'uuid', UUID) in ('', '--'), 'static_address_forbidden')

    def installed_guard(self):
        journal = self.load()
        need(journal.get('phase') == 'installed', 'incomplete_installation_preserved')
        for name, fact in journal['files'].items():
            need(self.fact(name) == fact, 'installed_file_changed')
        self.profile_guard(journal)
        return journal

    def foreign_files(self):
        result = {}
        folder = self.path('/etc/NetworkManager/system-connections')
        for path in folder.iterdir():
            if path.name == Path(KEYFILE).name:
                continue
            need(not path.is_symlink() and path.is_file(), 'foreign_profile_file_type')
            result[path.name] = foreign_file_fingerprint(path.name, path.read_bytes())
        return result

    def default_routes(self, *, exclude=None, timeout=25):
        rows = json.loads(self.run([IP, '-j', '-4', 'route', 'show', 'default'], timeout=timeout))
        return sorted((str(r.get('dst', 'default')), str(r.get('gateway', '')), str(r.get('dev', '')),
                       int(r.get('metric', 0)), str(r.get('table', 'main')))
                      for r in rows if r.get('dev') != exclude)

    def active(self, interface, *, timeout=25):
        value = self.nm('-g', 'GENERAL.CON-UUID', 'device', 'show', interface, timeout=timeout)
        need(value in ('', '--') or re.fullmatch(r'[0-9a-f-]{36}', value), 'active_profile_invalid')
        return '' if value == '--' else value

    def rebind_mac(self, journal, identity):
        self.profile_guard(journal)
        if journal.get('profile_mac', MAC) == identity['mac']:
            return None
        old = json.loads(json.dumps(journal))
        planned = json.loads(json.dumps(journal))
        planned['profile_mac'] = identity['mac']
        planned['profile_semantics'] = with_mac(journal['profile_semantics'], identity['mac'])
        pending = json.loads(json.dumps(journal))
        pending.update(phase='rebind_intent', rebind_old_mac=journal.get('profile_mac', MAC),
                       rebind_new_mac=identity['mac'])
        self.save(JOURNAL, encode(pending))
        try:
            need(self.unique_identity(identity['name']) == identity, 'usb_identity_changed')
            self.nm('connection', 'modify', 'uuid', UUID, '802-3-ethernet.mac-address', identity['mac'])
            self.profile_guard(planned)
            need(self.unique_identity(identity['name']) == identity, 'usb_identity_changed')
            self.save(JOURNAL, encode(planned))
            return old, planned
        except BaseException as error:
            try:
                try:
                    self.profile_guard(old)
                except Refused:
                    self.profile_guard(planned)
                    self.nm('connection', 'modify', 'uuid', UUID, '802-3-ethernet.mac-address', old.get('profile_mac', MAC))
                    self.profile_guard(old)
                self.save(JOURNAL, encode(old))
            except BaseException:
                pending['phase'] = 'rebind_recovery_required'
                self.save(JOURNAL, encode(pending))
                raise Refused('mac_rebind_recovery_required') from error
            raise

    def undo_rebind(self, token):
        if token is None:
            return
        old, current = token
        need(self.load() == current, 'mac_rebind_record_changed')
        self.profile_guard(current)
        self.nm('connection', 'modify', 'uuid', UUID, '802-3-ethernet.mac-address', old.get('profile_mac', MAC))
        self.profile_guard(old)
        self.save(JOURNAL, encode(old))

    def lease(self, interface, *, identity=None, foreign_profiles=None, foreign_routes=None,
              settle_seconds=7.0, now=None, sleep=None):
        need(type(settle_seconds) in (int, float) and math.isfinite(settle_seconds)
             and 0 < settle_seconds <= 8, 'route_settle_budget_invalid')
        now, sleep = now or time.monotonic, sleep or time.sleep
        started = now()
        deadline = started + settle_seconds
        identity = identity if identity is not None else self.unique_identity(interface)
        foreign = foreign_profiles if foreign_profiles is not None else self.foreign_files()
        other_routes = foreign_routes if foreign_routes is not None else self.default_routes(
            exclude=interface, timeout=min(2.0, settle_seconds))
        journal = self.load()
        observation = {'attempts': 0, 'ipv4_address_count': None, 'default_route_count': None,
                       'gateway_matches': False, 'metric_matches': False}
        pending_code = 'usb_default_route_invalid'

        def budget():
            remaining = deadline - now()
            need(remaining > 0, pending_code)
            return min(2.0, remaining)

        def unchanged():
            need(self.unique_identity(interface) == identity, 'usb_identity_changed')
            need(self.active(interface, timeout=budget()) == UUID, 'activation_identity_or_profile_changed')
            need(self.load() == journal, 'lease_installation_record_changed')
            need(keyfile_semantics(self.path(KEYFILE).read_bytes()) == journal['profile_semantics'], 'profile_changed')
            need(self.foreign_files() == foreign, 'foreign_profiles_changed')
            need(self.default_routes(exclude=interface, timeout=budget()) == other_routes, 'foreign_default_routes_changed')

        try:
            while True:
                unchanged()
                observation['attempts'] += 1
                rows = json.loads(self.run([IP, '-j', '-4', 'address', 'show', 'dev', interface], timeout=budget()))
                need(len(rows) == 1, 'address_query_invalid')
                addresses = [ipaddress.IPv4Interface(str(x['local']) + '/' + str(x['prefixlen']))
                             for x in rows[0].get('addr_info', []) if x.get('family') == 'inet']
                observation['ipv4_address_count'] = len(addresses)
                need(len(addresses) <= 1 and all(address.network == SUBNET and
                     2 <= int(address.ip) - int(SUBNET.network_address) <= 254 for address in addresses),
                     'dhcp_address_outside_official_range')
                routes = json.loads(self.run([IP, '-j', '-4', 'route', 'show', 'dev', interface], timeout=budget()))
                defaults = [row for row in routes if row.get('dst') == 'default']
                observation['default_route_count'] = len(defaults)
                observation['gateway_matches'] = len(defaults) == 1 and defaults[0].get('gateway') == GATEWAY
                observation['metric_matches'] = len(defaults) == 1 and defaults[0].get('metric') == METRIC
                unchanged()
                if addresses and observation['gateway_matches'] and observation['metric_matches']:
                    return str(addresses[0].ip)
                pending_code = 'usb_default_route_invalid' if addresses else 'dhcp_address_not_ready'
                remaining = deadline - now()
                need(remaining > 0, pending_code)
                sleep(min(0.25, remaining))
        except Refused as error:
            error.details.setdefault('lease_observation', {**observation,
                'settle_elapsed_seconds': round(max(0, now() - started), 3), 'settle_limit_seconds': settle_seconds})
            raise

    def link_dns_servers(self, interface, identity):
        text = self.run(['/usr/bin/resolvectl', 'dns', str(identity['ifindex'])], timeout=2)
        lines = text.strip().splitlines()
        need(bool(lines), 'usb_dns_link_response_invalid')
        match = re.fullmatch(r'Link\s+(\d+)\s+\(([^)]+)\):\s*(.*)', lines[0])
        need(match is not None and int(match[1]) == identity['ifindex'] and match[2] == interface,
             'usb_dns_link_response_invalid')
        need(all(line[:1].isspace() for line in lines[1:]), 'usb_dns_link_response_invalid')
        tokens = (match[3] + ' ' + ' '.join(line.strip() for line in lines[1:])).split()
        try:
            return [str(ipaddress.ip_address(token)) for token in tokens]
        except ValueError:
            raise Refused('usb_dns_link_response_invalid') from None

    def ensure_link_dns(self, interface, *, identity, mutation, now=None, sleep=None):
        """자신의 활성 USB 연결에서 선언된 DNS만 적용한다. 다른 링크는 쓰지 않는다."""
        now, sleep = now or time.monotonic, sleep or time.sleep
        started = now()
        deadline = started + 2.0
        observation = {'resolved_server_count': 0, 'networkmanager_dns_matches': None,
                       'runtime_apply_requested': False, 'runtime_reapplied': False}
        def guard():
            need(self.unique_identity(interface) == identity, 'usb_identity_changed')
            need(self.active(interface, timeout=2) == UUID, 'activation_identity_or_profile_changed')
            self.installed_guard()
            text = self.nm('-g', 'IP4.DNS', 'device', 'show', interface, timeout=2)
            tokens = [token for token in re.split(r'[\s,;|]+', text.strip()) if token and token != '--']
            try:
                declared = [str(ipaddress.ip_address(token)) for token in tokens]
            except ValueError:
                raise Refused('usb_nm_dns_response_invalid') from None
            observation['networkmanager_dns_matches'] = declared == [GATEWAY]
            need(not declared or declared == [GATEWAY], 'usb_nm_dns_not_ready')
            return observation['networkmanager_dns_matches']
        try:
            while True:
                nm_ready = guard()
                current = self.link_dns_servers(interface, identity)
                observation['resolved_server_count'] = len(current)
                if current == [GATEWAY] and nm_ready:
                    return False
                need(not current or current == [GATEWAY], 'usb_dns_link_settings_invalid')
                remaining = deadline - now()
                if remaining <= 0:
                    break
                sleep(min(.2, remaining))
            need(guard(), 'usb_nm_dns_not_ready')
            current = self.link_dns_servers(interface, identity)
            need(not current or current == [GATEWAY], 'usb_dns_link_settings_invalid')
            if not current:
                need(guard(), 'usb_nm_dns_not_ready')
                current = self.link_dns_servers(interface, identity)
                need(not current or current == [GATEWAY], 'usb_dns_link_settings_invalid')
            if not current:
                mutation.update(interface=interface, identity=identity, prior=[], post=[GATEWAY])
                observation['runtime_apply_requested'] = True
                self.run(['/usr/bin/resolvectl', 'dns', str(identity['ifindex']), GATEWAY], timeout=3)
            need(guard(), 'usb_nm_dns_not_ready')
            actual = self.link_dns_servers(interface, identity)
            observation['resolved_server_count'] = len(actual)
            need(actual == [GATEWAY], 'usb_dns_link_settings_invalid')
            observation['runtime_reapplied'] = observation['runtime_apply_requested']
            return observation['runtime_reapplied']
        except Refused as error:
            error.details['dns_observation'] = {**observation, 'elapsed_seconds': round(max(0, now() - started), 3)}
            raise

    def restore_link_dns(self, mutation):
        if not mutation:
            return
        interface, identity = mutation['interface'], mutation['identity']
        self.installed_guard()
        need(self.unique_identity(interface) == identity and self.active(interface, timeout=2) == UUID,
             'dns_runtime_restore_identity_changed')
        if 'scope_prior' in mutation:
            current_scope = self.link_dns_default_route(interface, identity)
            if current_scope != mutation['scope_prior']:
                need(current_scope == mutation['scope_post'], 'dns_scope_external_change_preserved')
                self.run(['/usr/bin/resolvectl','default-route',str(identity['ifindex']),
                          'yes' if mutation['scope_prior'] else 'no'],timeout=3)
                need(self.unique_identity(interface) == identity and self.active(interface,timeout=2) == UUID,
                     'dns_runtime_restore_identity_changed')
                need(self.link_dns_default_route(interface,identity) == mutation['scope_prior'],
                     'dns_scope_restore_unconfirmed')
        if 'prior' not in mutation:
            return
        current = self.link_dns_servers(interface, identity)
        if current == mutation['prior']:
            return
        need(current == mutation['post'], 'dns_runtime_external_change_preserved')
        self.run(['/usr/bin/resolvectl', 'dns', str(identity['ifindex']), *mutation['prior']] +
                 ([] if mutation['prior'] else ['']), timeout=3)
        need(self.unique_identity(interface) == identity and self.active(interface, timeout=2) == UUID,
             'dns_runtime_restore_identity_changed')
        need(self.link_dns_servers(interface, identity) == mutation['prior'], 'dns_runtime_restore_unconfirmed')

    def link_dns_default_route(self, interface, identity):
        text = self.run(['/usr/bin/resolvectl','default-route',str(identity['ifindex'])],timeout=2)
        match = re.fullmatch(r'Link\s+(\d+)\s+\(([^)]+)\):\s*(yes|no)',text.strip())
        need(match is not None and int(match[1]) == identity['ifindex'] and match[2] == interface,
             'usb_dns_scope_response_invalid')
        return match[3] == 'yes'

    def probe(self, interface, address, *, mutation=None):
        identity = self.unique_identity(interface)
        need(self.link_dns_servers(interface, identity) == [GATEWAY], 'usb_dns_link_settings_invalid')
        query = ['/usr/bin/resolvectl', '--interface=' + interface, '--type=A', '--cache=no', 'query', PROBE]
        scope_reapplied = False
        try:
            reply = self.run(query,timeout=12)
        except Refused as error:
            if mutation is None or error.details.get('command_operation') != 'query' or error.details.get('command_error_category') != 'no_usable_dns_scope':
                raise
            self.installed_guard()
            need(self.unique_identity(interface) == identity and self.active(interface,timeout=2) == UUID,
                 'usb_identity_changed')
            need(self.link_dns_servers(interface,identity) == [GATEWAY], 'usb_dns_link_settings_invalid')
            declared = self.nm('-g','IP4.DNS','device','show',interface,timeout=2)
            need([x for x in re.split(r'[\s,;|]+',declared.strip()) if x] == [GATEWAY], 'usb_nm_dns_not_ready')
            routes = json.loads(self.run([IP,'-j','-4','route','show','dev',interface],timeout=2))
            defaults = [r for r in routes if r.get('dst') == 'default']
            need(len(defaults)==1 and defaults[0].get('gateway')==GATEWAY and defaults[0].get('metric')==METRIC,
                 'usb_default_route_invalid')
            if self.link_dns_default_route(interface,identity):
                raise
            need(self.unique_identity(interface) == identity and self.active(interface,timeout=2) == UUID,
                 'usb_identity_changed')
            mutation.update(interface=interface,identity=identity,scope_prior=False,scope_post=True)
            self.run(['/usr/bin/resolvectl','default-route',str(identity['ifindex']),'yes'],timeout=3)
            need(self.unique_identity(interface) == identity and self.active(interface,timeout=2) == UUID,
                 'usb_identity_changed')
            need(self.link_dns_default_route(interface,identity), 'usb_dns_scope_update_unconfirmed')
            reply = self.run(query,timeout=12)
            scope_reapplied = True
        candidates = [x for x in re.findall(r'\b(?:\d{1,3}\.){3}\d{1,3}\b', reply)
                      if ipaddress.IPv4Address(x).is_global]
        need(candidates, 'usb_dns_query_no_public_address')
        remote = candidates[0]
        output = self.run(['/usr/bin/python3','-I','-B','-c',BOUND_HTTPS_CODE,interface,address,remote,PROBE],timeout=20).split()
        need(len(output) == 3 and 200 <= int(output[0]) < 500 and output[1] == address and output[2] == remote,
             'usb_https_path_unconfirmed')
        return {'usb_dns_interface_query': True, 'usb_bound_https_head': True, 'http_status': int(output[0]),
                'usb_dns_scope_reapplied':scope_reapplied}

    def connect(self, interface):
        journal = self.installed_guard()
        self.module_guard()
        identity = self.unique_identity(interface)
        original = self.active(interface)
        foreign = self.foreign_files()
        routes = self.default_routes(exclude=interface)
        attempted = False
        rebind = None
        dns_mutation = {}
        try:
            need(self.unique_identity(interface) == identity, 'usb_identity_changed')
            rebind = self.rebind_mac(journal, identity)
            attempted = True
            self.nm('--wait', '30', 'connection', 'up', 'uuid', UUID, 'ifname', interface, timeout=35)
            need(self.unique_identity(interface) == identity and self.active(interface) == UUID, 'activation_identity_or_profile_changed')
            address = self.lease(interface, identity=identity, foreign_profiles=foreign, foreign_routes=routes)
            dns_reapplied = self.ensure_link_dns(interface, identity=identity, mutation=dns_mutation)
            proof = self.probe(interface, address, mutation=dns_mutation)
            need(self.unique_identity(interface) == identity, 'usb_identity_changed_after_probe')
            self.installed_guard()
            need(self.foreign_files() == foreign, 'foreign_profiles_changed')
            need(self.default_routes(exclude=interface) == routes, 'foreign_default_routes_changed')
            need(self.link_dns_servers(interface, identity) == [GATEWAY], 'usb_dns_runtime_changed_after_probe')
            if 'scope_prior' in dns_mutation:
                need(self.link_dns_default_route(interface,identity), 'usb_dns_scope_changed_after_probe')
            return {'status': 'usb_internet_verified', 'profile_created': False,
                    'profile_mac_rebound': rebind is not None,
                    'usb_dns_runtime_reapplied': dns_reapplied,
                    'foreign_profiles_and_routes_preserved': True, **proof}
        except BaseException as error:
            # 다른 프로세스가 대상 연결/USB를 바꿨으면 그 변경을 덮지 않는다.
            lease_details = {key: error.details[key] for key in ('lease_observation', 'dns_observation')
                             if key in error.details} if isinstance(error, Refused) else {}
            dns_restore_error = None
            try:
                self.restore_link_dns(dns_mutation)
            except Exception as recovery_error:
                dns_restore_error = recovery_error
                lease_details['dns_recovery_code'] = str(recovery_error) if isinstance(recovery_error, Refused) else type(recovery_error).__name__
            if dns_restore_error is not None:
                raise Refused('usb_dns_runtime_recovery_required', {
                    'original_code': str(error) if isinstance(error, Refused) else type(error).__name__,
                    'recovery': 'external_change_preserved', **lease_details,
                }) from error
            recovery = 'previous_state_unchanged' if original == UUID or not attempted else 'external_change_preserved'
            if isinstance(error, Refused) and str(error) == 'mac_rebind_recovery_required':
                recovery = 'failed'
            if attempted and original != UUID:
                try:
                    self.profile_guard(self.load())
                    if self.unique_identity(interface) == identity and self.active(interface) == UUID:
                        self.nm('--wait', '10', 'connection', 'down', 'uuid', UUID, timeout=15)
                        if original and original in self.profile_ids():
                            self.nm('--wait', '20', 'connection', 'up', 'uuid', original, 'ifname', interface, timeout=25)
                        recovery = 'previous_profile_restored' if original else 'owned_profile_disconnected'
                except Exception as recovery_error:
                    raise Refused('usb_failure_recovery_required', {
                        'original_code': str(error) if isinstance(error, Refused) else type(error).__name__,
                        'recovery': 'failed',
                        'recovery_code': str(recovery_error) if isinstance(recovery_error, Refused) else type(recovery_error).__name__,
                        **lease_details,
                    }) from error
            try:
                self.undo_rebind(rebind)
            except Exception as recovery_error:
                raise Refused('mac_rebind_recovery_required', {'original_code': str(error) if isinstance(error, Refused)
                    else type(error).__name__, 'recovery_code': str(recovery_error) if isinstance(recovery_error, Refused)
                    else type(recovery_error).__name__, **lease_details}) from error
            if isinstance(error, Refused):
                error.details['recovery'] = recovery
            raise

    def connect_index(self, index):
        need(re.fullmatch(r'[1-9][0-9]{0,9}', str(index)) is not None, 'interface_index_invalid')
        identity = self.unique_identity(index=int(index))
        return self.connect(identity['name'])

    def validate_recovery_record(self, journal, expected_sha):
        need(isinstance(journal, dict) and journal.get('schema') == 1 and journal.get('uuid') == UUID
             and journal.get('tag') == TAG, 'installation_record_mismatch')
        need(journal.get('phase') in ('rolled_back', 'failed_preserved', 'installing'), 'owned_recovery_phase_invalid')
        files = journal.get('files')
        need(isinstance(files, dict) and set(files) <= {PROGRAM, RULE, UNIT, CONFIG}, 'owned_recovery_file_set_invalid')
        need(all(isinstance(value, dict) for value in files.values()), 'owned_recovery_file_fact_invalid')
        program_sha = journal.get('program_sha256') or files.get(PROGRAM, {}).get('sha256')
        known_programs = (PREVIOUS_PROGRAM_SHA256,
                          'c2f2e4a3d16b5646eaf6966297f968389a687239148c5b98a052b789d7ff5b0f',
                          '0560f4ef78e3db8391b6feee6a5ff19b2835873bc6b7d6c2aaccf5ea53918846',
                          '257a556ba46f830984685da53d2e28328f1dc169241c2deb42e2425cc3df0646',
                          '9b4cf0d2ed301d2f678936c210fcfe4538ed9bd0faca3a663139b7399217c59a',
                          '25f0ede1bc8a9b0fb1d67f949846f648028567292d085a7634a81b9496232626',
                          '041d722684c9a59facf4aaa1269ac526b1379b149020896a0aa174ac691d8775', expected_sha)
        need(program_sha in known_programs, 'owned_recovery_program_unrecognized')
        hashes = {PROGRAM: program_sha, RULE: digest(rule_payload()), UNIT: digest(unit_payload()),
                  CONFIG: digest(config_payload(program_sha))}
        for name, fact in files.items():
            need(isinstance(fact, dict) and set(fact) == {'sha256', 'mode', 'inode', 'uid', 'bytes'},
                 'owned_recovery_file_fact_invalid')
            modes = (0o600,) if name == CONFIG else (0o600, 0o644)
            known_rule = (name == RULE and program_sha != expected_sha and
                          fact['sha256'] == '8dec2e960b83be45087cf3c85e272bd78b93aaa158b63a1f739d74128d829a98')
            need((fact['sha256'] == hashes[name] or known_rule) and fact['mode'] in modes
                 and fact['uid'] == self.uid and type(fact['inode']) is int and fact['inode'] > 0
                 and type(fact['bytes']) is int and fact['bytes'] >= 0, 'owned_recovery_file_identity_invalid')
        need(type(journal.get('profile_intent')) is bool and isinstance(journal.get('foreign_profiles'), dict)
             and all(isinstance(key, str) and re.fullmatch(r'[0-9a-f]{64}', str(value))
                     for key, value in journal['foreign_profiles'].items()), 'owned_recovery_profile_record_invalid')

    def recovery_record(self, expected_sha):
        path = self.path(JOURNAL)
        self.safe_parent(path)
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
        try:
            info = os.fstat(fd)
            need(stat.S_ISREG(info.st_mode) and info.st_uid == self.uid and info.st_nlink == 1
                 and not info.st_mode & 0o077 and info.st_size <= 131072, 'owned_recovery_journal_unsafe')
            raw = os.read(fd, 131073)
            after = os.fstat(fd)
            identity = lambda item: (item.st_dev, item.st_ino, item.st_size, item.st_mtime_ns, item.st_ctime_ns)
            need(len(raw) == info.st_size and identity(info) == identity(after), 'owned_recovery_journal_changed')
        finally:
            os.close(fd)
        journal = json.loads(raw)
        self.validate_recovery_record(journal, expected_sha)
        return journal, raw

    def recovery_history(self, expected_sha, *, create=False):
        state = self.path(STATE)
        self.safe_parent(state / 'history')
        history = state / 'history'
        if not history.exists() and create:
            history.mkdir(mode=0o700)
        need(history.is_dir() and not history.is_symlink() and history.stat().st_uid == self.uid
             and not history.stat().st_mode & 0o077, 'owned_recovery_history_unsafe')
        entries = list(history.iterdir())
        need(len(entries) <= 256, 'owned_recovery_history_budget')
        for path in entries:
            info = path.lstat()
            need(re.fullmatch(r'[0-9a-f]{64}\.json', path.name) is not None and stat.S_ISREG(info.st_mode)
                 and info.st_uid == self.uid and info.st_nlink == 1 and stat.S_IMODE(info.st_mode) == 0o400
                 and info.st_size <= 131072, 'owned_recovery_history_file_unsafe')
            raw = path.read_bytes()
            need(digest(raw) + '.json' == path.name, 'owned_recovery_history_changed')
            self.validate_recovery_record(json.loads(raw), expected_sha)
        return history

    def preserve_recovery_record(self, raw, expected_sha):
        history = self.recovery_history(expected_sha, create=True)
        absolute = STATE + '/history/' + digest(raw) + '.json'
        destination = self.path(absolute)
        if destination.exists():
            need(destination.read_bytes() == raw, 'owned_recovery_history_changed')
        else:
            self.save(absolute, raw, mode=0o400, new=True)
        return history

    def recover_owned_installation(self, expected_sha):
        journal, original = self.recovery_record(expected_sha)
        print(json.dumps({'event': 'owned_recovery_record_checked',
                          'previous_phase': public_code(journal.get('phase')),
                          'previous_failure_code': public_code(journal.get('failure_code'))}), flush=True)
        state = self.path(STATE)
        need(state.is_dir() and not state.is_symlink() and state.stat().st_uid == self.uid
             and not state.stat().st_mode & 0o077 and set(item.name for item in state.iterdir()) <= {'installation.json', 'history'},
             'owned_recovery_state_unsafe')
        # 고정 경로와 배포 지문을 확인한 뒤 기존 원복의 CAS·모호한 프로필 보호를 그대로 사용한다.
        # 이전 실패 뒤 사용자가 바꾼 다른 연결은 현재 값으로 보존한다.
        foreign = self.foreign_files()
        before_ids = self.profile_ids()
        foreign_ids = set(before_ids) - {UUID}
        need(not any(self.nm('-g', 'connection.id', 'connection', 'show', 'uuid', item) == PROFILE
                     for item in foreign_ids), 'foreign_profile_name_conflict')
        if UUID in before_ids:
            need(journal.get('profile_intent') is True and journal.get('profile_semantics') is not None,
                 'uncertain_profile_creation_preserved')
            self.profile_guard(journal)
        else:
            need(self.fact(KEYFILE) is None, 'unowned_profile_file_preserved')
        for name in (PROGRAM, RULE, UNIT, CONFIG):
            current = self.fact(name)
            need(current is None or current == journal['files'].get(name), 'rollback_cas_changed')
        self.preserve_recovery_record(original, expected_sha)
        self.rollback()
        need(all(self.fact(name) is None for name in (PROGRAM, RULE, UNIT, CONFIG, KEYFILE)),
             'owned_recovery_destination_remains')
        ids = self.profile_ids()
        need(UUID not in ids and set(ids) == foreign_ids, 'owned_recovery_profile_inventory_changed')
        need(not any(self.nm('-g', 'connection.id', 'connection', 'show', 'uuid', item) == PROFILE for item in ids),
             'foreign_profile_name_conflict')
        need(self.foreign_files() == foreign, 'foreign_profiles_changed')
        rolled, rolled_raw = self.recovery_record(expected_sha)
        need(rolled.get('phase') == 'rolled_back', 'owned_recovery_rollback_incomplete')
        self.preserve_recovery_record(rolled_raw, expected_sha)
        need(self.path(JOURNAL).read_bytes() == rolled_raw, 'owned_recovery_journal_changed')
        self.path(JOURNAL).unlink()
        folder = os.open(self.path(STATE), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(folder)
        finally:
            os.close(folder)
        return {'previous_phase': public_code(journal.get('phase')),
                'previous_failure_code': public_code(journal.get('failure_code')), 'owned_recovery_completed': True}

    def install(self, source, expected_sha, *, recover_owned=False):
        source = Path(source)
        payload = sealed_source(source, expected_sha)
        self.module_guard()
        need(re.search(r'\b1\.36\.\d+\b', self.nm('--version')), 'networkmanager_version_unreviewed')
        recovery = {}
        if self.path(JOURNAL).exists():
            previous = self.load()
            if previous.get('phase') == 'installed':
                self.installed_guard()
                return {'status': 'already_installed', 'files_changed': False}
            details = {'previous_phase': public_code(previous.get('phase')),
                       'previous_failure_code': public_code(previous.get('failure_code'))}
            if not recover_owned:
                raise Refused('incomplete_installation_preserved', details)
            try:
                recovery = self.recover_owned_installation(expected_sha)
            except Refused as error:
                error.details.update(details)
                raise
        ids = self.profile_ids()
        need(len(ids) < 256, 'profile_inventory_budget_exceeded')
        need(UUID not in ids, 'foreign_profile_uuid_conflict')
        need(not any(self.nm('-g', 'connection.id', 'connection', 'show', 'uuid', item) == PROFILE for item in ids),
             'foreign_profile_name_conflict')
        targets = {PROGRAM: (payload, 0o644), RULE: (rule_payload(), 0o644),
                   UNIT: (unit_payload(), 0o644), CONFIG: (config_payload(expected_sha), 0o600)}
        need(all(self.fact(name) is None for name in (*targets, KEYFILE)), 'foreign_destination_exists')
        state = self.path(STATE)
        if state.exists():
            need(recover_owned and set(item.name for item in state.iterdir()) == {'history'}, 'foreign_state_directory_exists')
            history = self.recovery_history(expected_sha)
            need(any(history.iterdir()), 'owned_recovery_history_missing')
        else:
            self.safe_parent(state)
            state.mkdir(mode=0o700)
        journal = {'schema': 1, 'uuid': UUID, 'tag': TAG, 'phase': 'installing', 'files': {},
                   'program_sha256': expected_sha, 'profile_intent': False, 'foreign_profiles': self.foreign_files(),
                   'foreign_snapshot_format': FOREIGN_SNAPSHOT_FORMAT}
        self.save(JOURNAL, encode(journal), new=True)
        try:
            for name, (data, mode) in targets.items():
                self.save(name, data, mode, new=True)
                journal['files'][name] = self.fact(name)
                self.save(JOURNAL, encode(journal))
            journal['profile_intent'] = True
            self.save(JOURNAL, encode(journal))
            response = self.run(['/usr/bin/python3', '-I', '-B', '-c', NM_CREATE_CODE, '--apply'])
            need('NM_CREATE_FIXED_UUID_OK' in response, 'fixed_uuid_creation_unconfirmed')
            need(self.fact(KEYFILE) is not None, 'owned_profile_file_not_found')
            journal['profile_semantics'] = keyfile_semantics(self.path(KEYFILE).read_bytes())
            journal['profile_mac'] = MAC
            self.save(JOURNAL, encode(journal))
            self.profile_guard(journal)
            need(self.foreign_files() == journal['foreign_profiles'], 'foreign_profiles_changed')
            self.run(['/usr/bin/systemctl', 'daemon-reload'])
            self.run(['/usr/bin/udevadm', 'control', '--reload-rules'])
            self.run(['/usr/sbin/modprobe', 'cdc_ncm'])
            journal['phase'] = 'installed'
            self.save(JOURNAL, encode(journal))
            present = self.identities()
            need(len(present) <= 1, 'target_interface_count_not_one')
            for identity in present:
                self.run(['/usr/bin/systemctl', '--no-block', 'start',
                          'carrot-usb-net-client@' + str(identity['ifindex']) + '.service'])
            return {'status': 'installed', 'files_created': len(targets), 'owned_profiles_created': 1,
                    'kernel_dtb_initrd_or_module_files_changed': False,
                    'coldplug_rule_ready': True, 'live_target_workers_queued': len(present),
                    'live_usb_verification_performed': False, **recovery}
        except BaseException as error:
            # 출처가 불명확한 중간 상태는 남기고 정확히 소유 확인한 자산만 되돌린다.
            journal['phase'] = 'failed_preserved'
            journal['failure_code'] = str(error) if isinstance(error, Refused) else type(error).__name__
            if isinstance(error, Refused) and error.details:
                journal['failure_details'] = {key: error.details[key] for key in
                    ('command_exit_code', 'command_operation', 'command_error_category') if key in error.details}
            self.save(JOURNAL, encode(journal))
            self.rollback()
            if isinstance(error, Refused):
                error.details.update(recovery)
            raise

    def rollback(self):
        journal = self.load()
        need(journal.get('phase') in ('installed', 'failed_preserved', 'installing', 'rolled_back'), 'rollback_phase_invalid')
        if journal['phase'] == 'rolled_back':
            return {'status': 'already_rolled_back'}
        for name, fact in journal['files'].items():
            current = self.fact(name)
            need(current is None or current == fact, 'rollback_cas_changed')
        if journal.get('profile_semantics') is not None:
            if UUID in self.profile_ids():
                self.profile_guard(journal)
                self.nm('--wait', '15', 'connection', 'delete', 'uuid', UUID, timeout=20)
            else:
                need(self.fact(KEYFILE) is None, 'unowned_profile_file_preserved')
        elif journal.get('profile_intent') and UUID in self.profile_ids():
            raise Refused('uncertain_profile_creation_preserved')
        for name, fact in reversed(list(journal['files'].items())):
            current = self.fact(name)
            need(current is None or current == fact, 'rollback_cas_changed')
            if current is not None:
                self.path(name).unlink()
        self.run(['/usr/bin/systemctl', 'daemon-reload'])
        self.run(['/usr/bin/udevadm', 'control', '--reload-rules'])
        journal['phase'] = 'rolled_back'
        self.save(JOURNAL, encode(journal))
        return {'status': 'rolled_back', 'shared_kernel_modules_left_loaded': True,
                'foreign_profiles_deleted': False}


@contextmanager
def lock():
    fd = os.open('/run/lock/carrot-usb-net-client.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    try:
        info = os.fstat(fd)
        need(stat.S_ISREG(info.st_mode) and info.st_uid == 0 and not info.st_mode & 0o077, 'lock_file_unsafe')
        deadline = time.monotonic() + 8
        while True:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                break
            except BlockingIOError:
                need(time.monotonic() < deadline, 'client_lock_timeout')
                time.sleep(0.1)
        yield
    finally:
        os.close(fd)


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='action', required=True)
    install = sub.add_parser('install')
    install.add_argument('--expected-self-sha256', required=True)
    install.add_argument('--apply', action='store_true', required=True)
    connect = sub.add_parser('connect')
    connect.add_argument('interface')
    connect_index = sub.add_parser('connect-index')
    connect_index.add_argument('index')
    sub.add_parser('rollback')
    args = parser.parse_args()
    need(os.geteuid() == 0, 'root_required')
    os.umask(0o077)
    client = Client(release=os.uname().release)
    def interrupted(_signum, _frame):
        raise Refused('interrupted')
    signal.signal(signal.SIGTERM, interrupted)
    try:
        with lock():
            if args.action == 'install':
                result = client.install(__file__, args.expected_self_sha256)
            elif args.action == 'connect':
                result = client.connect(args.interface)
            elif args.action == 'connect-index':
                result = client.connect_index(args.index)
            else:
                result = client.rollback()
        print(json.dumps(result, sort_keys=True))
        return 0
    except BaseException as error:
        print(json.dumps({'status': 'stopped_preserved', 'code': str(error) if isinstance(error, Refused)
                          else type(error).__name__, **(error.details if isinstance(error, Refused) else {})}, sort_keys=True))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
