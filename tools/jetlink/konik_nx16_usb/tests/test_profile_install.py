"""실장치 대신 분리 파일과 선언된 하드웨어 값으로 opt-in·원복 경계를 검사한다."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
PACKAGE = HERE.parent
REPO = PACKAGE.parents[2]


def load(name, path):
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


install = load('konik_install_test', PACKAGE / 'install.py')
profile = load('konik_profile_test', REPO / 'openpilot/selfdrive/modeld/jetlink/konik_usb_profile.py')


class Platform(unittest.TestCase):
  def test_unsupported_machine_or_nano_is_rejected(self):
    with self.assertRaisesRegex(install.Refused, 'arm64'):
      install.target_platform('konik', machine='x86_64', release='4.9.103')
    with tempfile.TemporaryDirectory() as folder:
      root = Path(folder)
      path = root / 'proc/device-tree/compatible'
      path.parent.mkdir(parents=True)
      path.write_bytes(b'nvidia,p3767-0003\0nvidia,tegra234\0')
      with self.assertRaisesRegex(install.Refused, 'orin_nx16'):
        install.target_platform('jetson', root, machine='aarch64', release='5.15.148-tegra-j401-r3647')
      path.write_bytes(b'nvidia,p3767-0000+p3768-0000\0nvidia,tegra234\0')
      self.assertEqual(install.target_platform('jetson', root, machine='aarch64', release='5.15.148-tegra-j401-r3647')['role'], 'jetson')
      with self.assertRaisesRegex(install.Refused, 'reviewed_j401'):
        install.target_platform('jetson', root, machine='aarch64', release='5.15.148-tegra')


@unittest.skipIf(os.name != 'posix', 'POSIX 파일 소유권·symlink 경계')
class Install(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    (self.root / 'data').mkdir(mode=0o755)
    (self.root / 'run').mkdir(mode=0o755)
    (self.root / 'run').chmod(0o755)
    self.original = b'#!/bin/bash\nprintf "%s\\n" "existing startup"\n'
    (self.root / 'data/continue.sh').write_bytes(self.original)
    (self.root / 'data/continue.sh').chmod(0o755)
    self.owner = os.geteuid()
    self.client = install.KonikInstall(self.root, owner=self.owner)

  def enabled(self, **kwargs):
    return profile.enabled(self.root, owner=self.owner, machine='aarch64', release='4.9.103', **kwargs)

  def test_plan_is_read_only_and_install_is_idempotent(self):
    before = sorted(str(p.relative_to(self.root)) for p in self.root.rglob('*'))
    self.client.plan()
    self.assertEqual(before, sorted(str(p.relative_to(self.root)) for p in self.root.rglob('*')))
    self.assertFalse(self.enabled())
    self.assertFalse(self.client.install()['reboot_or_restart_requested'])
    self.assertTrue(self.enabled())
    self.assertEqual(self.client.install()['operating_files_changed'], 0)
    body = (self.root / 'data/continue.sh').read_bytes()
    self.assertTrue(body.endswith(self.original.split(b'\n', 1)[1]))
    self.assertEqual((self.root / 'data/jetlink-transport').read_bytes(), b'auto\n')

  def test_rollback_restores_before_images_without_foreign_change(self):
    self.client.install()
    self.assertEqual(self.client.rollback()['status'], 'owned_data_files_restored')
    self.assertEqual((self.root / 'data/continue.sh').read_bytes(), self.original)
    self.assertFalse(self.enabled())
    self.assertFalse((self.root / 'data/jetlink-transport').exists())
    self.assertTrue((self.root / 'data/carrot-usb-internet/profile-install/journal.json').exists())

  def test_foreign_edit_blocks_entire_rollback(self):
    self.client.install()
    (self.root / 'data/continue.sh').write_bytes(b'#!/bin/bash\nprintf foreign\n')
    before = (self.root / 'data/carrot-usb-internet/profile.json').read_bytes()
    with self.assertRaisesRegex(install.Refused, 'foreign_changes'):
      self.client.rollback()
    self.assertEqual((self.root / 'data/carrot-usb-internet/profile.json').read_bytes(), before)

  def test_existing_exact_deployment_payload_is_preserved(self):
    self.client.safe_base(create=True)
    values = self.client.payloads()
    for name in install.TARGETS[:-1]:
      old, fact = self.client.read(name)
      self.client.write(name, values[name], fact)
    tracked = {name: self.client.read(name) for name in install.TARGETS[:-1]}
    self.assertEqual(self.client.install()['operating_files_changed'], 1)
    self.assertTrue(self.enabled())
    self.client.rollback()
    self.assertEqual({name: self.client.read(name) for name in install.TARGETS[:-1]}, tracked)

  def test_existing_changed_payload_is_not_overwritten(self):
    self.client.safe_base(create=True)
    payload = self.root / 'data/carrot-usb-internet/program.py'
    payload.write_bytes(b'foreign')
    with self.assertRaisesRegex(install.Refused, 'destination_differs'):
      self.client.install()
    self.assertEqual(payload.read_bytes(), b'foreign')

  def test_unsafe_marker_is_never_an_enable_switch(self):
    self.client.install()
    marker = self.root / 'data/carrot-usb-internet/profile.json'
    marker.chmod(0o666)
    self.assertFalse(self.enabled())
    marker.chmod(0o644)
    for value in (profile.VALUE | {'schema': True}, profile.VALUE | {'operator_confirmed': 1},
                  profile.VALUE | {'extra': 'unknown'}):
      marker.write_text(json.dumps(value))
      self.assertFalse(self.enabled())
    marker.write_text(json.dumps({'schema': 1, 'profile': 'other', 'operator_confirmed': True}))
    self.assertFalse(self.enabled())
    marker.unlink()
    target = self.root / 'data/other.json'
    target.write_text(json.dumps(profile.VALUE))
    marker.symlink_to(target)
    self.assertFalse(self.enabled())

  def test_marker_owner_hardlink_and_directory_modes_are_checked(self):
    self.client.install()
    marker = self.root / 'data/carrot-usb-internet/profile.json'
    other = self.root / 'data/second-link'
    os.link(marker, other)
    self.assertFalse(self.enabled())
    other.unlink()
    base = marker.parent
    base.chmod(0o777)
    self.assertFalse(self.enabled())
    base.chmod(0o755)
    if os.geteuid() == 0:
      os.chown(marker, 1, 1)
      self.assertFalse(self.enabled())

  def test_incomplete_installation_is_not_silently_reused(self):
    self.client.install()
    journal = self.root / 'data/carrot-usb-internet/profile-install/journal.json'
    data = json.loads(journal.read_text())
    data['phase'] = 'incomplete_preserved'
    journal.write_text(json.dumps(data))
    journal.chmod(0o600)
    with self.assertRaisesRegex(install.Refused, 'incomplete'):
      self.client.install()

  def test_partial_write_stays_disabled_and_explicit_rollback_restores_it(self):
    write = self.client.write
    def failed_write(name, raw, expected, **kwargs):
      if name == install.CONTINUE:
        raise OSError('fixture disk failure')
      return write(name, raw, expected, **kwargs)
    with patch.object(self.client, 'write', side_effect=failed_write), self.assertRaises(OSError):
      self.client.install()
    self.assertFalse(self.enabled())
    self.assertEqual((self.root / 'data/continue.sh').read_bytes(), self.original)
    self.assertEqual(self.client.rollback()['status'], 'owned_data_files_restored')
    self.assertFalse((self.root / 'data/carrot-usb-internet/program.py').exists())
    self.assertEqual((self.root / 'data/continue.sh').read_bytes(), self.original)

  def test_install_and_rollback_share_one_real_exclusive_lock(self):
    paused = threading.Event()
    release = threading.Event()
    second_started = threading.Event()
    results, errors = {}, []
    write = self.client.write
    def slow_write(name, raw, expected, **kwargs):
      result = write(name, raw, expected, **kwargs)
      if name == install.TARGETS[0]:
        paused.set()
        if not release.wait(5):
          raise RuntimeError('fixture release timeout')
      return result
    def install_worker():
      try:
        results['install'] = self.client.install()
      except BaseException as error:
        errors.append(error)
    def rollback_worker():
      second_started.set()
      try:
        results['rollback'] = install.KonikInstall(self.root, owner=self.owner).rollback()
      except BaseException as error:
        errors.append(error)
    with patch.object(self.client, 'write', side_effect=slow_write):
      first = threading.Thread(target=install_worker)
      second = threading.Thread(target=rollback_worker)
      first.start()
      try:
        self.assertTrue(paused.wait(5))
        second.start()
        self.assertTrue(second_started.wait(5))
        second.join(timeout=0.1)
        self.assertTrue(second.is_alive())
        self.assertTrue((self.root / 'data/carrot-usb-internet/program.py').exists())
      finally:
        release.set()
        first.join(timeout=5)
        if second.ident is not None:
          second.join(timeout=5)
    self.assertFalse(first.is_alive())
    self.assertFalse(second.is_alive())
    self.assertEqual(errors, [])
    self.assertEqual(results['install']['status'], 'installed_next_normal_start')
    self.assertEqual(results['rollback']['status'], 'owned_data_files_restored')
    self.assertEqual((self.root / 'data/continue.sh').read_bytes(), self.original)
    self.assertFalse((self.root / 'data/carrot-usb-internet/program.py').exists())

  def test_invalid_lock_permissions_and_hardlinks_are_rejected(self):
    self.client.install()
    lock = self.root / install.LOCK.lstrip('/')
    lock.chmod(0o666)
    with self.assertRaisesRegex(install.Refused, 'operation_lock_ownership'):
      self.client.rollback()
    lock.chmod(0o600)
    os.link(lock, self.root / 'data/second-lock-link')
    with self.assertRaisesRegex(install.Refused, 'operation_lock_ownership'):
      self.client.rollback()
    self.assertTrue(self.enabled())

  def test_two_initial_installs_are_serialized_before_data_creation(self):
    start = threading.Barrier(2)
    results, errors = [], []
    def worker():
      try:
        start.wait(timeout=5)
        results.append(install.KonikInstall(self.root, owner=self.owner).install())
      except BaseException as error:
        errors.append(error)
    threads = [threading.Thread(target=worker) for _ in range(2)]
    for thread in threads:
      thread.start()
    for thread in threads:
      thread.join(timeout=5)
      self.assertFalse(thread.is_alive())
    self.assertEqual(errors, [])
    self.assertEqual(sorted(result['status'] for result in results), ['already_installed', 'installed_next_normal_start'])
    self.assertTrue(self.enabled())

  def test_bad_backup_rejects_rollback_before_any_file_or_service_change(self):
    self.client.install()
    journal_path = self.root / install.JOURNAL.lstrip('/')
    journal_before = journal_path.read_bytes()
    entries = json.loads(journal_before)['entries']
    backup = self.root / entries[install.CONTINUE]['backup'].lstrip('/')
    backup.write_bytes(b'corrupted backup fixture')
    before = {name: self.client.read(name) for name in install.TARGETS}
    with patch.object(self.client.boot, 'rollback') as stop:
      with self.assertRaisesRegex(install.Refused, 'backup_changed'):
        self.client.rollback()
      stop.assert_not_called()
    self.assertEqual({name: self.client.read(name) for name in install.TARGETS}, before)
    self.assertEqual(journal_path.read_bytes(), journal_before)

  def test_backup_inode_replacement_after_preflight_is_detected(self):
    self.client.install()
    entries = json.loads((self.root / install.JOURNAL.lstrip('/')).read_text())['entries']
    backup = self.root / entries[install.CONTINUE]['backup'].lstrip('/')
    before = {name: self.client.read(name) for name in install.TARGETS}
    def replaced_backup(*args, **kwargs):
      replacement = backup.with_name('replacement-fixture')
      replacement.write_bytes(backup.read_bytes())
      replacement.chmod(0o600)
      os.replace(replacement, backup)
      return {'status': 'already_absent'}
    with patch.object(self.client.boot, 'rollback', side_effect=replaced_backup):
      with self.assertRaisesRegex(install.Refused, 'rollback_backup_changed'):
        self.client.rollback()
    self.assertEqual({name: self.client.read(name) for name in install.TARGETS}, before)

  def test_postimage_change_prevents_commit_and_withdraws_new_opt_in(self):
    write = self.client.write
    foreign = b'#!/bin/bash\nprintf foreign-update\n'
    def changed_write(name, raw, expected, **kwargs):
      result = write(name, raw, expected, **kwargs)
      if name == install.MARKER:
        (self.root / 'data/continue.sh').write_bytes(foreign)
      return result
    with patch.object(self.client, 'write', side_effect=changed_write):
      with self.assertRaisesRegex(install.Refused, 'installed_postimage_changed'):
        self.client.install()
    self.assertFalse(self.enabled())
    self.assertEqual((self.root / 'data/continue.sh').read_bytes(), foreign)
    journal = json.loads((self.root / install.JOURNAL.lstrip('/')).read_text())
    self.assertEqual(journal['phase'], 'incomplete_preserved')
    with self.assertRaisesRegex(install.Refused, 'foreign_changes_preserved'):
      self.client.rollback()
    self.assertEqual((self.root / 'data/continue.sh').read_bytes(), foreign)

  def test_target_inode_replacement_after_preflight_is_preserved(self):
    self.client.install()
    target = self.root / 'data/continue.sh'
    original = target.read_bytes()
    def replaced_target(*args, **kwargs):
      replacement = target.with_name('replacement-target-fixture')
      replacement.write_bytes(original)
      replacement.chmod(target.stat().st_mode & 0o777)
      os.replace(replacement, target)
      return {'status': 'already_absent'}
    with patch.object(self.client.boot, 'rollback', side_effect=replaced_target):
      with self.assertRaisesRegex(install.Refused, 'rollback_destination_changed'):
        self.client.rollback()
    self.assertEqual(target.read_bytes(), original)
    self.assertTrue(self.enabled())
    journal = json.loads((self.root / install.JOURNAL.lstrip('/')).read_text())
    self.assertEqual(journal['phase'], 'installed')

  def test_writable_state_directory_blocks_update_and_rollback(self):
    self.client.install()
    (self.root / 'data/carrot-usb-internet/profile-install').chmod(0o777)
    with self.assertRaisesRegex(install.Refused, 'state_directory'):
      self.client.install()
    with self.assertRaisesRegex(install.Refused, 'state_directory'):
      self.client.rollback()

  def test_manager_registration_uses_only_verified_opt_in(self):
    tree = ast.parse((REPO / 'openpilot/system/manager/process_config.py').read_text())
    call = next(n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == 'PythonProcess' and n.args and isinstance(n.args[0], ast.Constant)
                and n.args[0].value == 'jetlinkd')
    expression = compile(ast.Expression(call), '<actual jetlink registration>', 'eval')
    namespace = {'PythonProcess': lambda *args, **kwargs: kwargs, 'always_run': object(), 'TICI': True,
                 'konik_usb_internet_enabled': self.enabled}
    self.assertFalse(eval(expression, namespace)['spawn'])
    self.client.install()
    self.assertTrue(eval(expression, namespace)['spawn'])
    (self.root / 'data/carrot-usb-internet/profile.json').chmod(0o666)
    self.assertFalse(eval(expression, namespace)['spawn'])


@unittest.skipIf(os.name != 'posix', 'POSIX SIGTERM 정리 회귀')
class Lifecycle(unittest.TestCase):
  def test_real_sigterm_runs_finally_only_for_opted_in_child(self):
    tree = ast.parse((REPO / 'openpilot/selfdrive/modeld/jetlink/daemon.py').read_text())
    handler = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_handle_sigterm')
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    hook = next(n for n in main.body if isinstance(n, ast.If) and isinstance(n.test, ast.Call)
                and isinstance(n.test.func, ast.Name) and n.test.func.id == 'konik_usb_internet_enabled')
    production = ast.unparse(handler) + '\n' + ast.unparse(hook)
    with tempfile.TemporaryDirectory() as folder:
      for enabled in (False, True):
        marker = Path(folder) / ('cleanup-' + str(enabled))
        code = ('import signal,time;from pathlib import Path\n'
                + 'konik_usb_internet_enabled=lambda:' + repr(enabled) + '\n' + production
                + '\ntry:\n print("READY",flush=True)\n while True:time.sleep(1)\n'
                + 'finally:Path(' + repr(str(marker)) + ').write_text("cleaned")\n')
        child = subprocess.Popen([sys.executable, '-I', '-B', '-c', code], stdout=subprocess.PIPE, text=True)
        try:
          self.assertEqual(child.stdout.readline().strip(), 'READY')
          child.send_signal(signal.SIGTERM)
          self.assertEqual(child.wait(timeout=3), 1 if enabled else -signal.SIGTERM)
          self.assertEqual(marker.exists(), enabled)
        finally:
          if child.poll() is None:
            child.kill()
            child.wait(timeout=3)
          child.stdout.close()


if __name__ == '__main__':
  unittest.main()
