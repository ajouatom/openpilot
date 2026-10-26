"""명확한 DNS 범위 오류에만 자신의 링크 플래그를 조정하는 경로를 검사한다."""
import importlib.util
from pathlib import Path
import subprocess
import os
import unittest
from unittest.mock import patch


if os.name != 'posix':
    raise unittest.SkipTest('POSIX USB 네트워크 파일·명령 검사')

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('dns_scope_fixtures',HERE/'test_link_dns_runtime.py')
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
m=base.m


class Scope(unittest.TestCase):
    setUp=base.LinkDNS.setUp
    tearDown=base.LinkDNS.tearDown
    identity_fixture=base.LinkDNS.identity_fixture
    install=base.LinkDNS.install
    advance=base.LinkDNS.advance
    connect=base.LinkDNS.connect

    def prepare(self,category='no_usable_dns_scope'):
        base.LinkDNS.prepare(self,[m.GATEWAY])
        self.category=category
        original=self.client.run;self.scope=False;self.scope_writes=[];self.query_count=0
        self.force_error=False
        def runner(args,*,timeout=25):
            if Path(args[0]).name=='resolvectl' and args[1]=='default-route':
                self.fake.commands.append(args);self.assertEqual(args[2],'5')
                if len(args)==4:
                    self.assertEqual(self.fake.active,m.UUID)
                    self.scope_writes.append(args.copy());self.scope=args[3]=='yes'
                return 'Link 5 (usb0): '+('yes' if self.scope else 'no')
            if Path(args[0]).name=='resolvectl' and 'query' in args:
                self.query_count+=1
                if not self.scope or self.force_error:
                    raise m.Refused('command_failed_resolvectl',{'command_exit_code':1,
                        'command_operation':'query','command_error_category':self.category})
            return original(args,timeout=timeout)
        self.client.run=runner

    def test_exact_scope_error_adjusts_only_the_owned_link_once(self):
        self.prepare();foreign=self.client.foreign_files()
        result=self.connect()
        self.assertTrue(result['usb_dns_scope_reapplied'])
        self.assertTrue(result['usb_bound_https_head'])
        self.assertEqual(self.query_count,2)
        self.assertEqual(self.scope_writes,[['/usr/bin/resolvectl','default-route','5','yes']])
        self.assertEqual(self.client.foreign_files(),foreign)

    def test_dnssec_or_timeout_error_does_not_change_eligibility(self):
        self.prepare()
        for category in ('dnssec_failed','server_unreachable_or_timeout','permission_denied'):
            with self.subTest(category=category):
                self.category=category;self.scope=False;self.fake.active=self.fake.lan
                with self.assertRaises(m.Refused):self.connect()
                self.assertEqual(self.scope_writes,[])

    def test_https_failure_restores_scope_before_profile(self):
        self.prepare();self.fake.bad_curl_source=True
        with self.assertRaisesRegex(m.Refused,'usb_https_path_unconfirmed'):self.connect()
        self.assertFalse(self.scope)
        self.assertEqual(self.fake.active,self.fake.lan)
        self.assertEqual(self.scope_writes[-1],['/usr/bin/resolvectl','default-route','5','no'])

    def test_already_eligible_scope_does_not_hide_query_failure(self):
        self.prepare();self.scope=True;self.force_error=True
        with self.assertRaisesRegex(m.Refused,'command_failed_resolvectl'):self.connect()
        self.assertEqual(self.scope_writes,[])

    def test_scope_disappearing_after_http_is_not_accepted(self):
        self.prepare();self.on_curl=lambda:setattr(self,'scope',False)
        with self.assertRaisesRegex(m.Refused,'usb_dns_scope_changed_after_probe'):self.connect()
        self.assertEqual(self.fake.active,self.fake.lan)

    def test_runner_retains_command_scope_and_exit_without_raw_error(self):
        args=['/usr/bin/resolvectl','--interface=usb0','--type=A','--cache=no','query',m.PROBE]
        completed=subprocess.CompletedProcess(args,1,stdout='',stderr='No appropriate name servers or networks private-error')
        with patch.object(m.subprocess,'run',return_value=completed):
            with self.assertRaises(m.Refused) as caught:m.Runner()(args)
        self.assertEqual(caught.exception.details,{'command_exit_code':1,'command_operation':'query',
                                                  'command_error_category':'no_usable_dns_scope'})
        self.assertNotIn('private',str(caught.exception.details))


if __name__=='__main__':unittest.main()
