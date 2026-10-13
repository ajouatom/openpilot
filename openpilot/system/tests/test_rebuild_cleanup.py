import os
from pathlib import Path
import subprocess
import tempfile
import unittest  # noqa: TID251 -- preserve contributed process-lifetime cleanup fixtures

ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / 'SConstruct').is_file())


class RebuildCleanupTests(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)
    self.repo = self.root / 'repo'
    self.repo.mkdir()
    self.cache = self.root / 'scons_cache'
    self.cache.mkdir()
    (self.cache / 'entry').touch()
    (self.repo / 'prebuilt').touch()
    (self.repo / '.sconsign.dblite').touch()
    self.request = self.root / 'rebuild_all.request'
    self.request.touch()
    self.log = self.root / 'rebuild_all.last.log'
    source = (ROOT / 'launch_chffrplus.sh').read_text()
    self.source = source
    self.functions = source[source.index('BUILD_STATE_RESET=0'):source.index('function start_manager {')]
    self.functions = self.functions.replace('/data/', str(self.root) + '/')

  def shell(self, command, mocks=''):
    # Only temporary paths are used. Never invoke SCons, sudo, or reboot.
    script = self.functions + '\nDIR=' + str(self.repo) + '''
FORCE_REBUILD=0
sudo() { return 1; }
timeout() { echo simulated-clean; return 2; }
git() { echo test-head; }
''' + mocks + '\n' + command
    return subprocess.run(['bash', '-c', script], env=dict(os.environ, SCONS_CACHE_DIR=str(self.cache)),
                          capture_output=True, text=True, timeout=5)

  def test_cleanup_success_despite_scons_failure(self):
    r = self.shell('consume_rebuild_request')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.request.exists())
    self.assertFalse(self.cache.exists())
    self.assertFalse((self.repo / 'prebuilt').exists())
    self.assertIn('full build still pending', self.log.read_text())

  @unittest.skipIf(os.geteuid() == 0, 'requires ordinary user permissions')
  def test_real_permission_error_retains_request(self):
    self.repo.chmod(0o500)
    try:
      r = self.shell('consume_rebuild_request')
    finally:
      self.repo.chmod(0o700)
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.request.exists())
    self.assertIn('Permission denied', self.log.read_text())
    self.assertNotIn('reset done', self.log.read_text())

  def test_delete_error_retains_request_and_has_no_success(self):
    r = self.shell('consume_rebuild_request', 'rm() { return 1; }')
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.request.exists())
    self.assertIn('reset failed', self.log.read_text())
    self.assertNotIn('reset done', self.log.read_text())

  def test_zero_exit_with_remaining_files_is_failure(self):
    r = self.shell('consume_rebuild_request', 'rm() { return 0; }')
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.request.exists())
    self.assertIn('remains', self.log.read_text())

  def test_failure_does_not_set_success_flags(self):
    r = self.shell('reset_build_state; rc=$?; echo "$rc $BUILD_STATE_RESET $FORCE_REBUILD"', 'rm() { return 1; }')
    self.assertEqual(r.stdout.strip(), '1 0 0')

  def test_request_delete_failure_is_reported(self):
    mocks = 'rm() { if [[ "$*" == *rebuild_all.request* ]]; then return 1; fi; command rm "$@"; }'
    r = self.shell('consume_rebuild_request', mocks)
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.request.exists())
    self.assertIn('Cannot consume', self.log.read_text())
    self.assertNotIn('reset done', self.log.read_text())

  def test_retry_after_delete_failure(self):
    self.shell('consume_rebuild_request', 'rm() { return 1; }')
    r = self.shell('consume_rebuild_request')
    self.assertEqual(r.returncode, 0)
    self.assertFalse(self.request.exists())

  def test_log_open_failure_preserves_request(self):
    self.log.mkdir()
    r = self.shell('consume_rebuild_request')
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.request.exists())
    self.assertTrue(self.cache.exists())

  def test_unremovable_cache_falls_back_to_sudo(self):
    # The cache may belong to root after a `sudo scons`; only sudo can drop it.
    mocks = ('rm() { if [[ "$*" == *scons_cache* ]] && [ -z "$AS_ROOT" ]; then return 1; fi; command rm "$@"; }\n' +
             'sudo() { [ "$1" = -n ] && shift; AS_ROOT=1 "$@"; }')
    r = self.shell('consume_rebuild_request', mocks)
    self.assertEqual(r.returncode, 0, r.stderr + self.log.read_text())
    self.assertFalse(self.cache.exists())
    self.assertFalse(self.request.exists())

  def test_absent_request_does_not_clean(self):
    self.request.unlink()
    r = self.shell('consume_rebuild_request')
    self.assertEqual(r.returncode, 0)
    self.assertTrue(self.cache.exists())

  def test_dangling_symlink_is_not_treated_as_removed(self):
    (self.repo / 'prebuilt').unlink()
    (self.repo / 'prebuilt').symlink_to(self.root / 'missing')
    r = self.shell('reset_build_state', 'rm() { return 0; }')
    self.assertNotEqual(r.returncode, 0)

  def test_launch_stops_before_params_on_cleanup_failure(self):
    block = self.source[self.source.index('  if ! consume_rebuild_request;'):self.source.index('  # Build Params before')]
    script = ('consume_rebuild_request() { return 1; }\nshow_startup_failure() { echo "$*"; }\nlaunch_test() {\n' +
              block + '\necho unexpected-params;\n}\nlaunch_test')
    r = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=5)
    self.assertNotEqual(r.returncode, 0)
    self.assertNotIn('unexpected', r.stdout)
    self.assertIn('cleanup failed', r.stdout)


if __name__ == '__main__':
  unittest.main()
