"""분리 임시 파일·가짜 NM 명령으로 소유권과 USB 전용 경계를 검사한다."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

if os.name != 'posix':
    raise unittest.SkipTest('POSIX USB 네트워크 파일·명령 검사')

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent / 'host_client.py'
spec = importlib.util.spec_from_file_location('host_usb_client_under_test', SOURCE)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class FakeNM:
    def __init__(self, root):
        self.root = root
        self.commands = []
        self.lan = '15427e93-2933-49b7-8a0c-a9e780c9f354'
        self.profiles = {self.lan: {'connection.id': 'existing-lan'}}
        self.active = self.lan
        self.bad_dns = False
        self.bad_curl_source = False
        self.bad_address = False
        self.bad_module = False
        self.mutate_foreign_route = False
        self.failed_reload = False
        self.fail_restore = False
        self.bad_module_resolution = False
        self.module_install_override = False

    def __call__(self, args, *, timeout=25):
        self.commands.append(args)
        program = Path(args[0]).name
        if program == 'modinfo':
            if args[1] == '-n':
                folder = 'updates' if self.bad_module_resolution else 'kernel/drivers/net/usb'
                path = self.root / f'lib/modules/{m.RELEASE}/{folder}/{args[2]}.ko'
                return str(path)
            if args[2] == 'vermagic':
                return 'wrong-kernel' if self.bad_module else m.VERMAGIC
            return {'cdc_ncm': 'usbnet,cdc_ether', 'cdc_ether': 'usbnet', 'usbnet': ''}[Path(args[3]).stem]
        if program == 'modprobe':
            if '--show-depends' in args:
                if self.module_install_override:
                    return 'install /unexpected/helper --override'
                return '\n'.join('insmod ' + str(self.root / f'lib/modules/{m.RELEASE}/kernel/drivers/net/usb/{name}.ko')
                                 for name in m.MODULES)
            return ''
        if program == 'udevadm':
            return ''
        if program == 'python3':
            if len(args)>4 and args[4]==m.BOUND_HTTPS_CODE:
                assert args[1:4]==['-I','-B','-c']
                assert args[5:]==['usb0','192.168.60.11','140.82.112.6',m.PROBE]
                return '200 '+('198.51.100.77' if self.bad_curl_source else '192.168.60.11')+' 140.82.112.6'
            assert args == ['/usr/bin/python3', '-I', '-B', '-c', m.NM_CREATE_CODE, '--apply']
            # 별도 대상 libNM 검사에서 실행하는 생성 요청의 응답·키파일을 모의한다.
            settings = ['connection', 'add', 'type', 'ethernet', 'con-name', m.PROFILE, 'ifname', '*']
            for key, value in m.PROPERTIES.items():
                if key not in ('connection.id', 'connection.type', 'connection.interface-name'):
                    settings.extend((key, value))
            self([m.NM, '--colors', 'no', '--escape', 'no', *settings])
            return 'NM_CREATE_FIXED_UUID_OK'
        if program == 'systemctl':
            if self.failed_reload and args[1] == 'daemon-reload':
                self.failed_reload = False
                raise m.Refused('command_failed_systemctl')
            return ''
        if program == 'nmcli':
            a = args[5:]
            if a == ['--version']:
                return 'nmcli tool, version 1.36.6'
            if a == ['-g', 'UUID', 'connection', 'show']:
                return '\n'.join(self.profiles)
            if a[:2] == ['-g', 'GENERAL.CON-UUID']:
                return self.active or '--'
            if a[:3] == ['-g', 'IP4.DNS', 'device']:
                return m.GATEWAY if self.active == m.UUID else ''
            if a[0] == '-g':
                return self.profiles[a[-1]].get(a[1], '')
            if a[:2] == ['connection', 'add']:
                if m.UUID in self.profiles:
                    raise m.Refused('duplicate_profile')
                fields = dict(m.PROPERTIES)
                fields['802-3-ethernet.cloned-mac-address'] = (a[a.index('802-3-ethernet.cloned-mac-address') + 1]
                    if '802-3-ethernet.cloned-mac-address' in a else 'random')
                fields['ipv4.routes'] = '{ ip = 0.0.0.0/0, nh = 192.168.60.1, mt = 700 }'
                fields['ipv4.addresses'] = ''
                # 실제 요청에서 자동 주소·기존 제어 설정과 별도의 경로인지 확인한다.
                assert 'ipv4.addresses' not in a
                assert a[a.index('ipv4.method') + 1] == 'auto'
                assert a[a.index('connection.autoconnect') + 1] == 'no'
                self.profiles[m.UUID] = fields
                path = self.root / m.KEYFILE.lstrip('/')
                path.write_text('[connection]\nid=' + m.PROFILE + '\nuuid=' + m.UUID +
                                '\ntype=ethernet\nstable-id=' + m.TAG + '\nautoconnect=false\n'
                                '[ethernet]\nmac-address=' + m.MAC + '\ncloned-mac-address=' +
                                fields['802-3-ethernet.cloned-mac-address'] + '\n[ipv4]\nmethod=auto\n'
                                'route1=0.0.0.0/0,192.168.60.1,700\ndns=192.168.60.1;\n'
                                'ignore-auto-dns=true\nignore-auto-routes=true\n[ipv6]\nmethod=disabled\n')
                path.chmod(0o600)
                return 'created'
            if a[:2] == ['connection', 'modify']:
                assert a[2:4] == ['uuid', m.UUID] and a[4] == '802-3-ethernet.mac-address'
                self.profiles[m.UUID]['802-3-ethernet.mac-address'] = a[5].upper()
                path = self.root / m.KEYFILE.lstrip('/')
                old = path.read_text()
                before = m.keyfile_semantics(old.encode())['ethernet']['mac-address']
                path.write_text(old.replace('mac-address=' + before, 'mac-address=' + a[5].lower()))
                return 'modified owned MAC'
            if a[:1] == ['--wait']:
                a = a[2:]
            if a[:2] == ['connection', 'up']:
                if self.fail_restore and a[3] == self.lan:
                    raise m.Refused('restore_test_failure')
                self.active = a[3]
                if self.active == m.UUID and self.profiles[m.UUID]['802-3-ethernet.cloned-mac-address'] != 'preserve':
                    (self.root / 'sys/class/net/usb0/address').write_text('02:66:55:44:33:22')
                return 'activated'
            if a[:2] == ['connection', 'down']:
                assert a[3] == m.UUID
                self.active = ''
                return 'deactivated'
            if a[:2] == ['connection', 'delete']:
                assert a[3] == m.UUID
                self.profiles.pop(m.UUID)
                (self.root / m.KEYFILE.lstrip('/')).unlink()
                if self.active == m.UUID:
                    self.active = ''
                return 'deleted'
            raise AssertionError('unexpected NM command')
        if program == 'ip':
            a = args[1:]
            lan = {'dst': 'default', 'gateway': '192.0.2.1', 'dev': 'eth0', 'metric': 100}
            usb = {'dst': 'default', 'gateway': m.GATEWAY, 'dev': 'usb0', 'metric': 700}
            if a == ['-j', '-4', 'route', 'show', 'default']:
                if self.mutate_foreign_route and self.active == m.UUID:
                    lan['metric'] = 101
                return json.dumps([lan] + ([usb] if self.active == m.UUID else []))
            if a[:4] == ['-j', '-4', 'address', 'show']:
                return json.dumps([{'addr_info': [{'family': 'inet', 'local': '192.168.55.9' if self.bad_address
                    else '192.168.60.11', 'prefixlen': 24}]}])
            if a == ['-j', '-4', 'route', 'show', 'dev', 'usb0']:
                return json.dumps([usb])
            raise AssertionError('unexpected ip command')
        if program == 'resolvectl':
            if args[1] == 'dns':
                return 'Link 5 (usb0): ' + ('192.0.2.53' if self.bad_dns else m.GATEWAY)
            assert '--interface=usb0' in args and '--type=A' in args and '--cache=no' in args
            return 'api.github.com: 140.82.112.6 -- link: usb0'
        if program == 'curl':
            assert args[1] == '--disable'
            assert args[args.index('--interface') + 1] == 'usb0'
            assert args[args.index('--resolve') + 1] == 'api.github.com:443:140.82.112.6'
            assert '--head' in args and '--insecure' not in args and '--location' not in args
            return '200 ' + ('198.51.100.77' if self.bad_curl_source else '192.168.60.11') + ' 140.82.112.6'
        raise AssertionError('unexpected executable')


class HostTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        for name in ('usr/local/libexec', 'etc/udev/rules.d', 'etc/systemd/system',
                     'etc/NetworkManager/system-connections', 'var/lib', 'sys/class/net'):
            (self.root / name).mkdir(parents=True, exist_ok=True)
        foreign = self.root / 'etc/NetworkManager/system-connections/existing-lan.nmconnection'
        foreign.write_text('[connection]\nid=existing-lan\n[ipv4]\nmethod=auto\n')
        foreign.chmod(0o600)
        self.foreign_bytes = foreign.read_bytes()
        self.fake = FakeNM(self.root)
        self.client = m.Client(self.root, self.fake)
        pins = {}
        for name, (_, dependencies) in m.MODULES.items():
            path = self.root / f'lib/modules/{m.RELEASE}/kernel/drivers/net/usb/{name}.ko'
            path.parent.mkdir(parents=True, exist_ok=True)
            data = ('fixture module ' + name).encode()
            path.write_bytes(data)
            pins[name] = (m.digest(data), dependencies)
        self.pins = patch.object(m, 'MODULES', pins)
        self.pins.start()
        self.identity_fixture()

    def tearDown(self):
        self.pins.stop()
        self.temp.cleanup()

    def identity_fixture(self, *, driver='cdc_ncm', vid=m.VID, pid=m.PID, mac=m.MAC):
        device = self.root / 'sys/devices/platform/usb1/1-2'
        interface = device / '1-2:1.5'
        interface.mkdir(parents=True)
        (device / 'idVendor').write_text(vid)
        (device / 'idProduct').write_text(pid)
        target = self.root / ('sys/bus/usb/drivers/' + driver)
        target.mkdir(parents=True)
        (interface / 'driver').symlink_to(target, target_is_directory=True)
        folder = self.root / 'sys/class/net/usb0'
        folder.mkdir()
        (folder / 'device').symlink_to(interface, target_is_directory=True)
        (folder / 'address').write_text(mac)
        (folder / 'ifindex').write_text('5')

    def install(self):
        return self.client.install(SOURCE, m.digest(SOURCE.read_bytes()))

    def test_install_repeat_and_rollback_are_owned_and_idempotent(self):
        self.assertEqual(self.install()['status'], 'installed')
        count = len(self.fake.commands)
        self.assertEqual(self.install()['status'], 'already_installed')
        self.assertFalse(any('add' in c for c in self.fake.commands[count:]))
        self.assertEqual(self.client.rollback()['status'], 'rolled_back')
        self.assertEqual(self.client.rollback()['status'], 'already_rolled_back')
        self.assertIn(self.fake.lan, self.fake.profiles)
        self.assertEqual((self.root / 'etc/NetworkManager/system-connections/existing-lan.nmconnection').read_bytes(), self.foreign_bytes)
        self.assertFalse(any('rmmod' in c or 'modprobe' in c and '-r' in c for c in self.fake.commands))

    def test_usb_bound_dns_https_and_lan_priority_preserved(self):
        self.install()
        result = self.client.connect('usb0')
        self.assertEqual(result['status'], 'usb_internet_verified')
        self.assertTrue(result['usb_dns_interface_query'] and result['usb_bound_https_head'])
        self.assertEqual(self.fake.active, m.UUID)
        self.assertEqual(self.client.default_routes(exclude='usb0')[0][3], 100)

    def test_lan_success_cannot_pass_as_usb_success_and_previous_link_restores(self):
        self.install()
        self.fake.bad_curl_source = True
        with self.assertRaisesRegex(m.Refused, 'usb_https_path_unconfirmed'):
            self.client.connect('usb0')
        self.assertEqual(self.fake.active, self.fake.lan)
        self.assertIn(m.UUID, self.fake.profiles)

    def test_failed_recovery_is_reported_instead_of_swallowed(self):
        self.install()
        self.fake.bad_dns = True
        self.fake.fail_restore = True
        with self.assertRaisesRegex(m.Refused, 'usb_failure_recovery_required') as caught:
            self.client.connect('usb0')
        self.assertEqual(caught.exception.details['original_code'], 'usb_dns_link_settings_invalid')
        self.assertEqual(caught.exception.details['recovery'], 'failed')
        self.assertEqual(caught.exception.details['recovery_code'], 'restore_test_failure')

    def test_wrong_dns_link_and_wrong_dhcp_subnet_fail_without_static_fallback(self):
        self.install()
        for name, code in [('bad_dns', 'usb_dns_link_settings_invalid'), ('bad_address', 'dhcp_address_outside_official_range')]:
            with self.subTest(name=name):
                setattr(self.fake, name, True)
                with self.assertRaisesRegex(m.Refused, code):
                    self.client.connect('usb0')
                self.assertEqual(self.fake.active, self.fake.lan)
                setattr(self.fake, name, False)

    def test_wrong_vid_pid_driver_or_mac_never_activates_profile(self):
        self.install()
        original = len(self.fake.commands)
        paths = [(self.root / 'sys/devices/platform/usb1/1-2/idVendor', '9999'),
                 (self.root / 'sys/devices/platform/usb1/1-2/idProduct', '9999'),
                 (self.root / 'sys/class/net/usb0/address', '03:00:00:00:00:99')]
        for path, wrong in paths:
            before = path.read_text()
            path.write_text(wrong)
            with self.assertRaises(m.Refused):
                self.client.connect('usb0')
            path.write_text(before)
        driver = self.root / 'sys/devices/platform/usb1/1-2/1-2:1.5/driver'
        driver.unlink()
        driver.symlink_to(self.root / 'sys/bus/usb/drivers', target_is_directory=True)
        with self.assertRaisesRegex(m.Refused, 'target_interface_count_not_one'):
            self.client.connect('usb0')
        self.assertFalse(any('up' in c for c in self.fake.commands[original:]))

    def test_foreign_same_name_and_uuid_are_refused_before_writes(self):
        for kind in ('name', 'uuid'):
            with self.subTest(kind=kind):
                key = m.UUID if kind == 'uuid' else 'a22a1157-916e-4936-ae4c-499301091394'
                self.fake.profiles[key] = {'connection.id': m.PROFILE if kind == 'name' else 'foreign'}
                with self.assertRaises(m.Refused):
                    self.install()
                self.assertFalse((self.root / m.JOURNAL.lstrip('/')).exists())
                self.fake.profiles.pop(key)

    def test_source_symlink_destinations_and_modified_owned_files_refuse(self):
        destination = self.root / m.PROGRAM.lstrip('/')
        destination.symlink_to(SOURCE)
        with self.assertRaisesRegex(m.Refused, 'unsafe_owned_file'):
            self.install()
        destination.unlink()
        self.install()
        destination.write_bytes(b'user change\n')
        with self.assertRaisesRegex(m.Refused, 'installed_file_changed'):
            self.client.connect('usb0')
        with self.assertRaisesRegex(m.Refused, 'rollback_cas_changed'):
            self.client.rollback()
        self.assertIn(m.UUID, self.fake.profiles)
        self.assertEqual(destination.read_bytes(), b'user change\n')

    def test_module_tamper_and_wrong_vermagic_never_modprobe(self):
        self.fake.bad_module = True
        with self.assertRaisesRegex(m.Refused, 'module_vermagic_mismatch'):
            self.install()
        self.fake.bad_module = False
        path = self.root / f'lib/modules/{m.RELEASE}/kernel/drivers/net/usb/cdc_ncm.ko'
        path.write_bytes(b'wrong module')
        with self.assertRaisesRegex(m.Refused, 'module_hash_mismatch'):
            self.install()
        self.assertFalse(any(Path(c[0]).name == 'modprobe' and '--show-depends' not in c for c in self.fake.commands))

    def test_module_resolution_and_install_override_are_refused(self):
        path = self.root / f'lib/modules/{m.RELEASE}/updates/usbnet.ko'
        path.parent.mkdir(parents=True)
        path.write_bytes(b'override')
        self.fake.bad_module_resolution = True
        with self.assertRaisesRegex(m.Refused, 'module_resolution_changed'):
            self.install()
        self.fake.bad_module_resolution = False
        self.fake.module_install_override = True
        with self.assertRaisesRegex(m.Refused, 'module_install_override_or_dependency_changed'):
            self.install()
        self.assertFalse(any(Path(c[0]).name == 'modprobe' and '--show-depends' not in c for c in self.fake.commands))

    def test_stage_path_replacement_after_verification_cannot_change_root_payload(self):
        staged = self.root / 'reviewed-stage.py'
        sealed = SOURCE.read_bytes()
        staged.write_bytes(sealed)
        original_guard = self.client.module_guard
        def replace_stage():
            staged.write_bytes(b'not the reviewed program\n')
            original_guard()
        with patch.object(self.client, 'module_guard', side_effect=replace_stage):
            self.client.install(staged, m.digest(sealed))
        self.assertEqual((self.root / m.PROGRAM.lstrip('/')).read_bytes(), sealed)
        self.assertNotEqual(staged.read_bytes(), sealed)
        self.assertEqual(self.fake.profiles[m.UUID]['802-3-ethernet.cloned-mac-address'], 'preserve')

    def test_install_failure_rolls_back_only_owned_profile_and_files(self):
        self.fake.failed_reload = True
        with self.assertRaisesRegex(m.Refused, 'command_failed_systemctl'):
            self.install()
        self.assertNotIn(m.UUID, self.fake.profiles)
        self.assertIn(self.fake.lan, self.fake.profiles)
        self.assertFalse((self.root / m.PROGRAM.lstrip('/')).exists())
        self.assertEqual(self.client.load()['phase'], 'rolled_back')

    def test_modified_profile_and_foreign_route_are_not_silently_accepted(self):
        self.install()
        self.fake.profiles[m.UUID]['ipv4.dns'] = '8.8.8.8'
        with self.assertRaisesRegex(m.Refused, 'profile_live_settings_changed'):
            self.client.connect('usb0')
        self.fake.profiles[m.UUID]['ipv4.dns'] = m.GATEWAY
        self.fake.mutate_foreign_route = True
        with self.assertRaisesRegex(m.Refused, 'foreign_default_routes_changed'):
            self.client.connect('usb0')
        self.assertEqual(self.fake.active, self.fake.lan)

    def test_timestamp_only_change_is_allowed_but_profile_setting_change_is_not(self):
        self.install()
        path = self.root / m.KEYFILE.lstrip('/')
        original = path.read_text()
        path.write_text(original.replace('[connection]\n', '[connection]\ntimestamp=1792944000\n'))
        self.client.installed_guard()
        path.write_text(original.replace('method=auto', 'method=manual'))
        with self.assertRaisesRegex(m.Refused, 'profile_changed'):
            self.client.installed_guard()

    def test_unknown_interface_and_command_injection_are_refused(self):
        self.install()
        for name in ('../eth0', 'usb0;reboot', '--help', 'x' * 16):
            with self.subTest(name=name):
                with self.assertRaises(m.Refused):
                    self.client.identity(name)

    def test_systemd_numeric_index_resolves_only_the_exact_current_usb(self):
        self.install()
        self.assertEqual(self.client.connect_index('5')['status'], 'usb_internet_verified')
        for value in ('0', '5/eth0', '999', '--help'):
            with self.subTest(value=value):
                with self.assertRaises(m.Refused):
                    self.client.connect_index(value)
        started = [c for c in self.fake.commands if c[:3] == ['/usr/bin/systemctl', '--no-block', 'start']]
        self.assertEqual(started[0][-1], 'carrot-usb-net-client@5.service')

    def test_kernel_generated_local_mac_is_rebound_in_the_same_owned_profile(self):
        self.install()
        actual = '02:9a:11:22:33:44'
        (self.root / 'sys/class/net/usb0/address').write_text(actual)
        result = self.client.connect_index('5')
        self.assertTrue(result['profile_mac_rebound'])
        self.assertEqual(self.fake.profiles[m.UUID]['802-3-ethernet.mac-address'], actual.upper())
        self.assertEqual(self.client.load()['profile_mac'], actual)
        self.client.installed_guard()
        self.assertEqual(len(self.fake.profiles), 2)

    def test_two_matching_targets_are_refused_before_activation_or_rebind(self):
        self.install()
        source = self.root / 'sys/class/net/usb0'
        other = self.root / 'sys/class/net/usb1'
        other.mkdir()
        (other / 'device').symlink_to((source / 'device').resolve(), target_is_directory=True)
        (other / 'address').write_text('02:9a:11:22:33:45')
        (other / 'ifindex').write_text('6')
        before = len(self.fake.commands)
        with self.assertRaisesRegex(m.Refused, 'target_interface_count_not_one'):
            self.client.connect_index('5')
        self.assertFalse(any('up' in c or 'modify' in c for c in self.fake.commands[before:]))

    def test_mac_rebind_refuses_foreign_edits_and_failed_probe_restores_old_binding(self):
        self.install()
        (self.root / 'sys/class/net/usb0/address').write_text('02:9a:11:22:33:44')
        self.fake.profiles[m.UUID]['ipv4.dns'] = '192.0.2.53'
        before = len(self.fake.commands)
        with self.assertRaisesRegex(m.Refused, 'profile_live_settings_changed'):
            self.client.connect('usb0')
        self.assertFalse(any('modify' in c for c in self.fake.commands[before:]))
        self.fake.profiles[m.UUID]['ipv4.dns'] = m.GATEWAY
        self.fake.bad_dns = True
        with self.assertRaisesRegex(m.Refused, 'usb_dns_link_settings_invalid'):
            self.client.connect('usb0')
        self.assertEqual(self.client.load()['profile_mac'], m.MAC)
        self.assertEqual(self.fake.profiles[m.UUID]['802-3-ethernet.mac-address'], m.MAC.upper())
        self.assertEqual(self.fake.active, self.fake.lan)


if __name__ == '__main__':
    unittest.main(verbosity=2)
