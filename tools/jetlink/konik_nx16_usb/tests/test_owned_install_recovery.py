"""중단 기록의 명시적 소유 복구만 검사한다. 실제 NM·서비스·USB는 사용하지 않는다."""
import importlib.util
import contextlib
import io
import json
import os
from pathlib import Path
import unittest
from unittest.mock import patch
import subprocess

if os.name != 'posix':
    raise unittest.SkipTest('POSIX USB 네트워크 파일·명령 검사')

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('owned_recovery_fixtures',HERE/'test_host_usb_client.py')
fixtures=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixtures)
m=fixtures.m


class RecoveryTests(unittest.TestCase):
    setUp=fixtures.HostTests.setUp
    tearDown=fixtures.HostTests.tearDown
    identity_fixture=fixtures.HostTests.identity_fixture
    install=fixtures.HostTests.install

    def failed_install(self):
        self.fake.failed_reload=True
        with self.assertRaisesRegex(m.Refused,'command_failed_systemctl'):self.install()
        journal=self.client.load()
        self.assertEqual(journal['phase'],'rolled_back')
        return self.client.path(m.JOURNAL).read_bytes()

    def recover(self):
        return self.client.install(fixtures.SOURCE,m.digest(fixtures.SOURCE.read_bytes()),recover_owned=True)

    def test_default_still_preserves_failed_record_and_reports_safe_cause(self):
        original=self.failed_install()
        commands=len(self.fake.commands)
        with self.assertRaisesRegex(m.Refused,'incomplete_installation_preserved') as caught:self.install()
        self.assertEqual(caught.exception.details,{'previous_phase':'rolled_back','previous_failure_code':'command_failed_systemctl'})
        self.assertEqual(self.client.path(m.JOURNAL).read_bytes(),original)
        self.assertFalse(any('delete' in row or 'add' in row for row in self.fake.commands[commands:]))

    def test_explicit_recovery_archives_exact_cause_and_reinstalls_only_owned_assets(self):
        original=self.failed_install()
        result=self.recover()
        self.assertEqual(result['status'],'installed')
        self.assertEqual(result['previous_phase'],'rolled_back')
        self.assertEqual(result['previous_failure_code'],'command_failed_systemctl')
        history=self.client.path(m.STATE)/'history'
        audit=history/(m.digest(original)+'.json')
        self.assertEqual(audit.read_bytes(),original)
        self.assertEqual(audit.stat().st_mode&0o777,0o400)
        self.assertEqual(self.client.installed_guard()['phase'],'installed')
        self.assertIn(self.fake.lan,self.fake.profiles)
        self.assertEqual(self.client.foreign_files(),{'existing-lan.nmconnection':m.foreign_file_fingerprint('existing-lan.nmconnection',self.foreign_bytes)})

    def test_legacy_rolled_back_program_pin_is_accepted_without_inventing_missing_files(self):
        self.failed_install();journal=self.client.load()
        journal.pop('program_sha256')
        journal['files'][m.PROGRAM]['sha256']=m.PREVIOUS_PROGRAM_SHA256
        journal['files'][m.CONFIG]['sha256']=m.digest(m.config_payload(m.PREVIOUS_PROGRAM_SHA256))
        self.client.save(m.JOURNAL,m.encode(journal))
        self.assertEqual(self.recover()['status'],'installed')

    def test_unknown_phase_or_program_and_arbitrary_path_are_preserved(self):
        original=self.failed_install()
        for name,change,code in (
                ('phase',lambda j:j.update(phase='unknown'),'owned_recovery_phase_invalid'),
                ('program',lambda j:j.update(program_sha256='0'*64),'owned_recovery_program_unrecognized'),
                ('mode',lambda j:j['files'][m.PROGRAM].update(mode=0o666),'owned_recovery_file_identity_invalid'),
                ('path',lambda j:j['files'].update({'/etc/unrelated':j['files'][m.PROGRAM]}),'owned_recovery_file_set_invalid')):
            with self.subTest(name=name):
                journal=json.loads(original);change(journal);raw=m.encode(journal);self.client.save(m.JOURNAL,raw)
                with self.assertRaisesRegex(m.Refused,code):self.recover()
                self.assertEqual(self.client.path(m.JOURNAL).read_bytes(),raw)
                self.assertFalse((self.client.path(m.STATE)/'history').exists())

    def test_existing_foreign_edit_blocks_before_rollback_or_audit_creation(self):
        self.install();journal=self.client.load();journal['phase']='failed_preserved'
        journal['failure_code']='known_failure';self.client.save(m.JOURNAL,m.encode(journal))
        path=self.client.path(m.PROGRAM);path.write_bytes(b'foreign edited program')
        original=self.client.path(m.JOURNAL).read_bytes()
        with self.assertRaisesRegex(m.Refused,'rollback_cas_changed'):self.recover()
        self.assertEqual(path.read_bytes(),b'foreign edited program')
        self.assertEqual(self.client.path(m.JOURNAL).read_bytes(),original)
        self.assertIn(m.UUID,self.fake.profiles)

    def test_uncertain_profile_creation_stays_preserved(self):
        self.install();journal=self.client.load();journal['phase']='installing'
        journal.pop('profile_semantics');self.client.save(m.JOURNAL,m.encode(journal))
        original=self.client.path(m.JOURNAL).read_bytes()
        with self.assertRaisesRegex(m.Refused,'uncertain_profile_creation_preserved'):self.recover()
        self.assertEqual(self.client.path(m.JOURNAL).read_bytes(),original)
        self.assertIn(m.UUID,self.fake.profiles)
        self.assertTrue(self.client.path(m.PROGRAM).exists())

    def test_failed_preserved_owned_assets_use_existing_cas_rollback(self):
        self.install();journal=self.client.load();journal['phase']='failed_preserved'
        journal['failure_code']='command_failed_modprobe';self.client.save(m.JOURNAL,m.encode(journal))
        result=self.recover()
        self.assertEqual(result['previous_phase'],'failed_preserved')
        self.assertEqual(result['previous_failure_code'],'command_failed_modprobe')
        self.assertEqual(result['status'],'installed')
        self.assertEqual(self.client.foreign_files(),{'existing-lan.nmconnection':m.foreign_file_fingerprint('existing-lan.nmconnection',self.foreign_bytes)})

    def test_history_only_state_is_reusable_only_with_explicit_recovery(self):
        self.failed_install()
        result=self.client.recover_owned_installation(m.digest(fixtures.SOURCE.read_bytes()))
        self.assertTrue(result['owned_recovery_completed'])
        with self.assertRaisesRegex(m.Refused,'foreign_state_directory_exists'):self.install()
        self.assertEqual(self.recover()['status'],'installed')

    def test_foreign_state_or_modified_history_cannot_be_used(self):
        self.failed_install();state=self.client.path(m.STATE)
        foreign=state/'unexpected';foreign.write_bytes(b'foreign')
        with self.assertRaisesRegex(m.Refused,'owned_recovery_state_unsafe'):self.recover()
        foreign.unlink()
        self.client.recover_owned_installation(m.digest(fixtures.SOURCE.read_bytes()))
        audit=next((state/'history').iterdir());audit.chmod(0o600);audit.write_bytes(b'changed')
        with self.assertRaisesRegex(m.Refused,'owned_recovery_history_file_unsafe'):self.recover()

    def test_old_failure_body_is_not_exposed_as_public_code(self):
        self.failed_install();journal=self.client.load();journal['failure_code']='/private/path password=value'
        self.client.save(m.JOURNAL,m.encode(journal))
        result=self.recover()
        self.assertEqual(result['previous_failure_code'],'not_recorded')
        self.assertNotIn('password',json.dumps(result))

    def test_later_foreign_connection_changes_are_preserved_as_current(self):
        self.failed_install()
        path=self.client.path('/etc/NetworkManager/system-connections/existing-lan.nmconnection')
        changed=path.read_bytes()+b'\n# changed outside this installer\n'
        path.write_bytes(changed)
        self.assertEqual(self.recover()['status'],'installed')
        self.assertEqual(path.read_bytes(),changed)
        self.assertEqual(self.client.foreign_files(),{'existing-lan.nmconnection':m.foreign_file_fingerprint('existing-lan.nmconnection',changed)})

    def test_recovery_accepts_original_native_umask_077_permissions(self):
        before=os.umask(0o077)
        try:
            self.failed_install()
            journal=self.client.load()
            for name in (m.PROGRAM,m.RULE,m.UNIT):
                journal['files'][name]['mode']=0o600
            self.client.save(m.JOURNAL,m.encode(journal))
            self.assertEqual(journal['files'][m.PROGRAM]['mode'],0o600)
            self.assertEqual(self.recover()['status'],'installed')
        finally:
            os.umask(before)

    def test_last_actual_deployed_program_pin_is_recognized(self):
        self.failed_install()
        journal=self.client.load()
        pin='c2f2e4a3d16b5646eaf6966297f968389a687239148c5b98a052b789d7ff5b0f'
        journal['program_sha256']=pin
        journal['files'][m.PROGRAM]['sha256']=pin
        journal['files'][m.CONFIG]['sha256']=m.digest(m.config_payload(pin))
        self.client.save(m.JOURNAL,m.encode(journal))
        self.assertEqual(self.recover()['status'],'installed')

    def test_unrecorded_destination_or_unowned_uuid_is_preserved_before_rollback(self):
        original=self.failed_install()
        path=self.client.path(m.RULE);path.write_bytes(b'foreign new rule')
        with self.assertRaisesRegex(m.Refused,'rollback_cas_changed'):self.recover()
        self.assertEqual(path.read_bytes(),b'foreign new rule')
        self.assertEqual(self.client.path(m.JOURNAL).read_bytes(),original)
        path.unlink()
        journal=self.client.load();journal.pop('profile_semantics',None);journal['profile_intent']=False
        self.client.save(m.JOURNAL,m.encode(journal));self.fake.profiles[m.UUID]={'connection.id':'foreign uuid'}
        with self.assertRaisesRegex(m.Refused,'uncertain_profile_creation_preserved'):self.recover()
        self.assertIn(m.UUID,self.fake.profiles)
        self.assertFalse((self.client.path(m.STATE)/'history').exists())

    def test_original_safe_failure_is_emitted_even_if_new_installation_fails(self):
        original=self.failed_install();self.fake.failed_reload=True
        output=io.StringIO()
        with contextlib.redirect_stdout(output),self.assertRaisesRegex(m.Refused,'command_failed_systemctl'):
            self.recover()
        event=json.loads(output.getvalue().splitlines()[0])
        self.assertEqual(event,{'event':'owned_recovery_record_checked','previous_phase':'rolled_back',
                                'previous_failure_code':'command_failed_systemctl'})
        audit=self.client.path(m.STATE)/'history'/(m.digest(original)+'.json')
        self.assertEqual(audit.read_bytes(),original)

    def test_nm_command_error_retains_operation_without_raw_output(self):
        args=[m.NM,'--colors','no','--escape','no','connection','add','type','ethernet']
        raw=subprocess.CompletedProcess(args,10,stdout='private output',stderr='Error: invalid setting private-value')
        with patch.object(m.subprocess,'run',return_value=raw):
            with self.assertRaisesRegex(m.Refused,'command_failed_nmcli') as caught:
                m.Runner()(args)
        self.assertEqual(caught.exception.details,{'command_exit_code':10,'command_operation':'connection_add',
                         'command_error_category':'invalid_argument_or_setting'})
        self.assertNotIn('private',json.dumps(caught.exception.details))


if __name__=='__main__':unittest.main()
