"""USB 전용 DNS 정착·조건부 적용·실패 원복을 모의 경계에서 검사한다."""
import importlib.util
from pathlib import Path
import os
import unittest
from unittest.mock import patch


if os.name != 'posix':
    raise unittest.SkipTest('POSIX USB 네트워크 파일·명령 검사')

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('dns_runtime_fixtures',HERE/'test_host_usb_client.py')
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)
m=fixtures.m


class LinkDNS(unittest.TestCase):
    setUp=fixtures.HostTests.setUp
    tearDown=fixtures.HostTests.tearDown
    identity_fixture=fixtures.HostTests.identity_fixture
    install=fixtures.HostTests.install

    def prepare(self, resolved=(), declared=None):
        self.install()
        self.resolved=list(resolved)
        self.declared=[m.GATEWAY] if declared is None else list(declared)
        self.setters=[];self.clock=0.0;self.on_sleep=None;self.on_curl=None
        def runner(args,*,timeout=25):
            if Path(args[0]).name=='resolvectl' and args[1]=='dns':
                self.fake.commands.append(args)
                self.assertEqual(args[2],'5')
                if len(args)>3:
                    self.assertEqual(self.fake.active,m.UUID)
                    self.setters.append(args.copy())
                    self.resolved=[] if args[3:]==[''] else args[3:]
                return 'Link 5 (usb0): '+ ' '.join(self.resolved)
            if Path(args[0]).name=='nmcli' and args[5:8]==['-g','IP4.DNS','device']:
                self.fake.commands.append(args)
                return '\n'.join(self.declared)
            if (Path(args[0]).name=='curl' or (Path(args[0]).name=='python3' and len(args)>4 and args[4]==m.BOUND_HTTPS_CODE)) and self.on_curl:
                self.on_curl()
            return self.fake(args,timeout=timeout)
        self.client.run=runner

    def advance(self,seconds):
        self.clock+=seconds
        if self.on_sleep:self.on_sleep()

    def connect(self):
        with patch.object(m.time,'monotonic',side_effect=lambda:self.clock), \
             patch.object(m.time,'sleep',side_effect=self.advance):
            return self.client.connect('usb0')

    def test_normal_dns_requires_no_write(self):
        self.prepare([m.GATEWAY])
        result=self.connect()
        self.assertFalse(result['usb_dns_runtime_reapplied'])
        self.assertEqual(self.setters,[])

    def test_natural_dns_settling_requires_no_write(self):
        self.prepare()
        self.on_sleep=lambda:setattr(self,'resolved',[m.GATEWAY]) if self.clock>=.6 else None
        self.assertTrue(self.connect()['usb_bound_https_head'])
        self.assertEqual(self.setters,[])

    def test_empty_runtime_and_matching_active_nm_dns_reapply_only_usb_link(self):
        self.prepare();foreign=self.client.foreign_files()
        result=self.connect()
        self.assertTrue(result['usb_dns_runtime_reapplied'])
        self.assertTrue(result['usb_dns_interface_query'])
        self.assertEqual(self.setters,[['/usr/bin/resolvectl','dns','5',m.GATEWAY]])
        self.assertEqual(self.client.foreign_files(),foreign)

    def test_foreign_ipv4_ipv6_and_unknown_dns_are_not_overwritten(self):
        self.prepare()
        for servers,code in ((['192.0.2.53'],'usb_dns_link_settings_invalid'),
                             ([m.GATEWAY,'2001:db8::53'],'usb_dns_link_settings_invalid'),
                             (['not-an-address'],'usb_dns_link_response_invalid')):
            with self.subTest(servers=servers):
                self.resolved=servers.copy();self.fake.active=self.fake.lan
                with self.assertRaisesRegex(m.Refused,code):self.connect()
                self.assertEqual(self.resolved,servers)
                self.assertEqual(self.setters,[])

    def test_missing_or_other_nm_dns_never_writes_resolver(self):
        self.prepare()
        for declared in ([],['192.0.2.53']):
            with self.subTest(declared=declared):
                self.declared=declared;self.fake.active=self.fake.lan;self.clock=0
                with self.assertRaisesRegex(m.Refused,'usb_nm_dns_not_ready'):self.connect()
                self.assertEqual(self.setters,[])

    def test_usb_identity_change_stops_before_setting_dns(self):
        self.prepare()
        self.on_sleep=lambda:(self.root/'sys/class/net/usb0/ifindex').write_text('6')
        with self.assertRaisesRegex(m.Refused,'usb_identity_changed'):self.connect()
        self.assertEqual(self.setters,[])

    def test_https_failure_restores_dns_before_previous_profile(self):
        self.prepare();self.fake.bad_curl_source=True
        with self.assertRaisesRegex(m.Refused,'usb_https_path_unconfirmed'):self.connect()
        self.assertEqual(self.resolved,[])
        self.assertEqual(self.fake.active,self.fake.lan)
        self.assertEqual(self.setters[-1],['/usr/bin/resolvectl','dns','5',''])

    def test_post_probe_failure_also_restores_dns(self):
        self.prepare();self.on_curl=lambda:setattr(self.fake,'mutate_foreign_route',True)
        with self.assertRaisesRegex(m.Refused,'foreign_default_routes_changed'):self.connect()
        self.assertEqual(self.resolved,[])
        self.assertEqual(self.fake.active,self.fake.lan)

    def test_external_dns_change_is_preserved_and_recovery_reported(self):
        self.prepare();self.fake.bad_curl_source=True
        self.on_curl=lambda:setattr(self,'resolved',['192.0.2.53'])
        with self.assertRaisesRegex(m.Refused,'usb_dns_runtime_recovery_required'):self.connect()
        self.assertEqual(self.resolved,['192.0.2.53'])
        self.assertEqual(self.fake.active,m.UUID)
        self.assertEqual(len(self.setters),1)

    def test_runtime_dns_disappearing_after_http_is_not_accepted(self):
        self.prepare();self.on_curl=lambda:setattr(self,'resolved',[])
        with self.assertRaisesRegex(m.Refused,'usb_dns_runtime_changed_after_probe'):self.connect()
        self.assertEqual(self.resolved,[])
        self.assertEqual(self.fake.active,self.fake.lan)

    def test_dns_appearing_during_final_guard_is_not_overwritten(self):
        self.prepare();original=self.client.installed_guard;late=[0]
        def guard():
            result=original()
            if self.clock>=2:
                late[0]+=1
                if late[0]>=3:self.resolved=['192.0.2.53']
            return result
        with patch.object(self.client,'installed_guard',side_effect=guard):
            with self.assertRaisesRegex(m.Refused,'usb_dns_link_settings_invalid'):self.connect()
        self.assertEqual(self.setters,[])
        self.assertEqual(self.resolved,['192.0.2.53'])


if __name__=='__main__':unittest.main()
