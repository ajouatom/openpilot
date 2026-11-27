import os
from pathlib import Path
import subprocess
import tempfile
import time
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

  def shell(self, command, mocks='', real_git=False):
    # Only temporary paths are used. Never invoke SCons, sudo, or reboot.
    script = self.functions + '\nDIR=' + str(self.repo) + '''
FORCE_REBUILD=0
sudo() { return 1; }
timeout() { echo simulated-clean; return 2; }
''' + ('' if real_git else 'git() { echo test-head; }\n') + mocks + '\n' + command
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


  # C4: an interrupted build rebuilds from the part of the cache written before it began.
  def interrupt(self):
    time.sleep(0.05)
    self.marker.touch()
    time.sleep(0.05)
    (self.cache / 'new-entry').write_bytes(b'')
    (self.cache / 'other.tmp1234').write_bytes(b'partial')

  @property
  def marker(self):
    return self.root / 'carrot_build.inprogress'

  @property
  def ok_stamp(self):
    return self.root / 'carrot_build.ok'

  def current_stamp(self):
    r = self.shell('build_input_stamp')
    self.assertEqual(r.returncode, 0, r.stderr)
    return r.stdout

  def test_unfinished_build_marker_rebuilds_from_trusted_cache(self):
    self.request.unlink()
    self.ok_stamp.write_text(self.current_stamp())
    self.interrupt()
    r = self.shell('consume_rebuild_request; rc=$?; echo "rc=$rc $BUILD_STATE_RESET $FORCE_REBUILD"',
                   'timeout() { return 0; }')
    self.assertIn('rc=0 1 1', r.stdout, r.stderr + self.log.read_text())
    self.assertIn('trusted SCons cache', r.stdout)
    self.assertTrue((self.cache / 'entry').exists())
    self.assertFalse((self.cache / 'new-entry').exists())
    self.assertFalse((self.cache / 'other.tmp1234').exists())
    self.assertFalse((self.repo / 'prebuilt').exists())
    self.assertFalse((self.repo / '.sconsign.dblite').exists())
    self.assertFalse(self.request.exists())
    self.assertFalse((self.root / 'carrot_build.interrupted').exists())
    # The rebuild itself is protected again, even though the inputs match.
    self.assertFalse(self.ok_stamp.exists())
    self.assertTrue(self.marker.exists())
    self.assertIn('older cache kept', self.log.read_text())

  def test_interrupted_recovery_cleans_outputs_away_from_the_real_cache(self):
    # SConstruct registers Clean(".", cache_dir): scons -c must not see the real cache.
    self.request.unlink()
    self.interrupt()
    r = self.shell('consume_rebuild_request', 'timeout() { echo "clean-cache=$SCONS_CACHE_DIR"; return 0; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertIn(f'clean-cache={self.cache}.clean', self.log.read_text())
    self.assertTrue((self.cache / 'entry').exists())
    self.assertFalse(Path(f'{self.cache}.clean').exists())

  def test_failed_interrupted_recovery_falls_back_to_full_rebuild(self):
    self.request.unlink()
    self.interrupt()
    r = self.shell('consume_rebuild_request', 'find() { return 1; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.cache.exists())
    self.assertFalse(self.request.exists())
    self.assertTrue(self.marker.exists())
    self.assertIn('full build still pending', self.log.read_text())

  def test_matching_inputs_do_not_arm_marker(self):
    self.request.unlink()
    self.ok_stamp.write_text(self.current_stamp())
    r = self.shell('consume_rebuild_request')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertIn('marker not armed', r.stdout)
    self.assertFalse(self.marker.exists())
    self.assertTrue((self.cache / 'entry').exists())

  def test_changed_or_unknown_inputs_arm_marker(self):
    self.request.unlink()
    for stamp in ('older inputs\n', None):
      if stamp is None:
        self.ok_stamp.unlink()
      else:
        self.ok_stamp.write_text(stamp)
      self.marker.unlink(missing_ok=True)
      r = self.shell('consume_rebuild_request')
      self.assertEqual(r.returncode, 0, r.stderr)
      self.assertTrue(self.marker.exists())

  def test_failed_stamp_arms_marker(self):
    self.request.unlink()
    self.ok_stamp.write_text('\n')
    r = self.shell('consume_rebuild_request', 'git() { return 1; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertTrue(self.marker.exists())

  def test_explicit_request_still_clears_the_whole_cache(self):
    self.ok_stamp.write_text(self.current_stamp())
    self.marker.touch()
    r = self.shell('consume_rebuild_request')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.cache.exists())
    self.assertFalse(self.ok_stamp.exists())
    self.assertTrue(self.marker.exists())

  def test_finished_build_records_inputs_and_clears_marker(self):
    r = self.shell('BUILD_INPUT_STAMP=$(build_input_stamp); arm_build_marker; finish_build_marker')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.marker.exists())
    self.assertEqual(self.ok_stamp.read_text(), self.current_stamp())

  def test_finished_build_without_stamp_forgets_inputs(self):
    self.ok_stamp.write_text('stale\n')
    r = self.shell('arm_build_marker; finish_build_marker', 'git() { return 1; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.marker.exists())
    self.assertFalse(self.ok_stamp.exists())

  def test_stamp_tracks_tracked_changes_but_not_build_outputs(self):
    def run(*a):
      subprocess.run(['git', *a], cwd=self.repo, check=True, capture_output=True)
    run('init', '-q')
    (self.repo / 'source.c').write_text('int a;\n')
    run('add', 'source.c')
    run('-c', 'user.name=t', '-c', 'user.email=t@t', 'commit', '-qm', 'base')
    def stamp():
      return self.shell('build_input_stamp', real_git=True).stdout
    base = stamp()
    self.assertTrue(base.strip())
    (self.repo / 'output.o').write_bytes(b'built')
    self.assertEqual(stamp(), base)
    (self.repo / 'source.c').write_text('int b;\n')
    self.assertNotEqual(stamp(), base)

  def test_launch_order_arms_forced_builds_and_finishes_after_build(self):
    launch = self.source[self.source.index('function launch {'):]
    build = launch.index('run_startup_command ./build.py')
    self.assertLess(launch.index('arm_build_marker'), build)
    self.assertGreater(launch.index('\n  finish_build_marker\n'), build)
    self.assertNotIn('rm -f "$BUILD_MARKER"', launch)
    finish = self.source[self.source.index('function finish_build_marker {'):self.source.index('function prune_interrupted_cache {')]
    self.assertLess(finish.index('if ! sync;'), finish.index('rm -f "$BUILD_MARKER"'))

  def test_second_interruption_keeps_first_cache_cutoff(self):
    self.request.unlink()
    self.interrupt()
    self.marker.rename(self.root / 'carrot_build.interrupted')
    time.sleep(0.05)
    self.marker.touch()
    r = self.shell('consume_rebuild_request', 'timeout() { return 0; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse((self.cache / 'new-entry').exists())
    self.assertTrue((self.cache / 'entry').exists())

  def test_failed_clean_discards_cache(self):
    self.request.unlink()
    self.interrupt()
    r = self.shell('consume_rebuild_request')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.cache.exists())

  def test_clock_reset_discards_cache(self):
    self.request.unlink()
    self.interrupt()
    future = time.time() + 3600
    os.utime(self.marker, (future, future))
    r = self.shell('consume_rebuild_request', 'timeout() { return 0; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.cache.exists())

  def test_sync_failure_retains_marker(self):
    self.marker.touch()
    r = self.shell('finish_build_marker', 'sync() { return 1; }')
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.marker.exists())
    self.assertFalse(self.ok_stamp.exists())

  def test_changed_inputs_during_build_are_not_certified(self):
    self.ok_stamp.write_text('old\n')
    r = self.shell('BUILD_INPUT_STAMP=before; arm_build_marker; finish_build_marker')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.ok_stamp.exists())


if __name__ == '__main__':
  unittest.main()
