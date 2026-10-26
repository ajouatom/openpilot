"""실제 시간이 아닌 모의 시계·분리 NM 응답으로 정착 대기 경계를 검사한다."""
import importlib.util
import json
import os
from pathlib import Path
import unittest

if os.name != 'posix':
    raise unittest.SkipTest('POSIX USB 네트워크 파일·명령 검사')

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('route_settle_fixtures',HERE/'test_host_usb_client.py')
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)
m=fixtures.m


class RouteSettling(unittest.TestCase):
    setUp=fixtures.HostTests.setUp
    tearDown=fixtures.HostTests.tearDown
    identity_fixture=fixtures.HostTests.identity_fixture
    install=fixtures.HostTests.install

    def prepare(self,*,missing=0,wrong=False):
        self.install();self.fake.active=m.UUID
        self.clock=0.0;self.route_queries=0;self.missing=missing;self.wrong=wrong
        self.on_sleep=None
        def run(args,*,timeout=25):
            if Path(args[0]).name=='ip' and args[1:]==['-j','-4','route','show','dev','usb0']:
                self.fake.commands.append(args);self.route_queries+=1
                if self.route_queries<=self.missing:return '[]'
                return json.dumps([{'dst':'default','gateway':'192.0.2.99' if self.wrong else m.GATEWAY,
                                    'dev':'usb0','metric':m.METRIC}])
            return self.fake(args,timeout=timeout)
        self.client.run=run

    def sleep(self,seconds):
        self.clock+=seconds
        if self.on_sleep:self.on_sleep()

    def lease(self):
        return self.client.lease('usb0',now=lambda:self.clock,sleep=self.sleep)

    def test_missing_route_settles_without_creating_or_rewriting_routes(self):
        self.prepare(missing=2);start=len(self.fake.commands)
        self.assertEqual(self.lease(),'192.168.60.11')
        self.assertEqual(self.route_queries,3);self.assertEqual(self.clock,0.5)
        commands=self.fake.commands[start:]
        self.assertTrue(all('add' not in row and 'modify' not in row and 'delete' not in row for row in commands))

    def test_wrong_or_missing_route_times_out_and_preserves_minimal_observation(self):
        for wrong in (False,True):
            with self.subTest(wrong=wrong):
                if not hasattr(self,'clock'):self.prepare()
                self.clock=0.0;self.route_queries=0;self.missing=0 if wrong else 999;self.wrong=wrong
                with self.assertRaisesRegex(m.Refused,'usb_default_route_invalid') as caught:self.lease()
                value=caught.exception.details['lease_observation']
                self.assertEqual(value['default_route_count'],1 if wrong else 0)
                self.assertFalse(value['gateway_matches'])
                self.assertEqual(value['settle_elapsed_seconds'],7.0)
                self.assertEqual(value['settle_limit_seconds'],7.0)
                self.assertEqual(self.clock,7.0)
                self.assertNotIn('192.',json.dumps(value))

    def test_outside_subnet_is_rejected_immediately(self):
        self.prepare(missing=999);self.fake.bad_address=True
        with self.assertRaisesRegex(m.Refused,'dhcp_address_outside_official_range') as caught:self.lease()
        self.assertEqual(self.clock,0);self.assertEqual(self.route_queries,0)
        self.assertEqual(caught.exception.details['lease_observation']['ipv4_address_count'],1)

    def test_ifindex_or_driver_change_stops_before_another_route_query(self):
        self.prepare(missing=999)
        path=self.root/'sys/class/net/usb0/ifindex'
        self.on_sleep=lambda:path.write_text('6')
        with self.assertRaisesRegex(m.Refused,'usb_identity_changed'):self.lease()
        self.assertEqual(self.clock,0.25);self.assertEqual(self.route_queries,1)
        path.write_text('5');self.clock=0;self.route_queries=0
        driver=self.root/'sys/devices/platform/usb1/1-2/1-2:1.5/driver'
        target=self.root/'sys/bus/usb/drivers/foreign-driver';target.mkdir()
        def change_driver():driver.unlink();driver.symlink_to(target)
        self.on_sleep=change_driver
        with self.assertRaisesRegex(m.Refused,'target_interface_count_not_one'):self.lease()
        self.assertEqual(self.clock,0.25);self.assertEqual(self.route_queries,1)

    def test_active_profile_or_foreign_default_route_change_is_immediate(self):
        self.prepare(missing=999)
        self.on_sleep=lambda:setattr(self.fake,'active',self.fake.lan)
        with self.assertRaisesRegex(m.Refused,'activation_identity_or_profile_changed'):self.lease()
        self.assertEqual(self.clock,0.25)
        self.fake.active=m.UUID;self.clock=0;self.route_queries=0
        self.on_sleep=lambda:setattr(self.fake,'mutate_foreign_route',True)
        with self.assertRaisesRegex(m.Refused,'foreign_default_routes_changed'):self.lease()
        self.assertEqual(self.clock,0.25);self.assertEqual(self.route_queries,1)

    def test_foreign_timestamp_only_is_accepted_and_bytes_are_preserved(self):
        self.prepare(missing=1)
        foreign=self.root/'etc/NetworkManager/system-connections/existing-lan.nmconnection'
        before=self.client.foreign_files()
        changed=self.foreign_bytes.replace(b'[connection]\n',b'[connection]\ntimestamp=123456\n')
        self.on_sleep=lambda:foreign.write_bytes(changed)
        self.assertEqual(self.lease(),'192.168.60.11')
        self.assertEqual(self.client.foreign_files(),before)
        self.assertEqual(foreign.read_bytes(),changed)

    def test_foreign_setting_change_and_own_profile_change_are_not_ignored(self):
        self.prepare(missing=999)
        foreign=self.root/'etc/NetworkManager/system-connections/existing-lan.nmconnection'
        changed=self.foreign_bytes.replace(b'method=auto',b'method=manual')
        self.on_sleep=lambda:foreign.write_bytes(changed)
        with self.assertRaisesRegex(m.Refused,'foreign_profiles_changed'):self.lease()
        self.assertEqual(foreign.read_bytes(),changed);self.assertEqual(self.clock,0.25)
        foreign.write_bytes(self.foreign_bytes);self.clock=0;self.route_queries=0
        owned=self.client.path(m.KEYFILE);owned_raw=owned.read_bytes()
        self.on_sleep=lambda:owned.write_bytes(owned_raw.replace(b'method=auto',b'method=manual'))
        with self.assertRaisesRegex(m.Refused,'profile_changed'):self.lease()
        self.assertEqual(self.clock,0.25)

    def test_new_save_sets_explicit_mode_even_with_umask_077(self):
        path='/usr/local/libexec/test-explicit-mode'
        old=os.umask(0o077)
        try:self.client.save(path,b'example',mode=0o644,new=True)
        finally:os.umask(old)
        self.assertEqual(self.client.path(path).stat().st_mode&0o777,0o644)
        self.assertEqual(self.client.path(path).read_bytes(),b'example')

    def test_known_previous_installed_hash_and_legacy_0600_are_narrowly_accepted(self):
        self.install();self.client.rollback();journal=self.client.load()
        pin='0560f4ef78e3db8391b6feee6a5ff19b2835873bc6b7d6c2aaccf5ea53918846'
        journal['program_sha256']=pin;journal['files'][m.PROGRAM]['sha256']=pin
        journal['files'][m.CONFIG]['sha256']=m.digest(m.config_payload(pin))
        for name in (m.PROGRAM,m.RULE,m.UNIT):journal['files'][name]['mode']=0o600
        current=m.digest(fixtures.SOURCE.read_bytes())
        self.client.validate_recovery_record(journal,current)
        journal['program_sha256']='0'*64
        with self.assertRaisesRegex(m.Refused,'owned_recovery_program_unrecognized'):
            self.client.validate_recovery_record(journal,current)


if __name__=='__main__':unittest.main()
