import ast
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest  # noqa: TID251 -- preserve contributed process-lifetime cleanup fixtures
from unittest.mock import patch  # noqa: TID251

ROOT = next(parent for parent in Path(__file__).resolve().parents if (parent / 'SConstruct').is_file())
sys.path.insert(0, str(ROOT))
from openpilot.common.scons_cache_lock import acquire_cache_lock, CacheBusyError


class CacheExclusionTests(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.root = Path(self.tmp.name)
    self.cache = self.root / 'cache'
    (self.cache / 'AB').mkdir(parents=True)
    (self.cache / 'AB/result').write_text('cached-result')
    for name in ('config', 'CACHEDIR.TAG', 'carrot_params_keys.sha256'):
      (self.cache / name).write_text(name)
    self.env = dict(os.environ, PYTHONPATH=str(ROOT))
    tree = ast.parse((ROOT / 'openpilot/system/manager/build.py').read_text())
    names = {'prunable_cache_files', 'prune_cache', 'is_cache_lock_failure', 'build'}
    body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    self.ns = {'Path': Path, 'Spinner': object, 'acquire_cache_lock': acquire_cache_lock,
               'MAX_CACHE_SIZE': 0, 'os': os, 'BASEDIR': str(ROOT), 'AGNOS': False, 'CACHE_DIR': self.cache}
    exec(compile(ast.Module(body=body, type_ignores=[]), '<cache>', 'exec'), self.ns)

  def holder(self, code=None):
    code = code or ('from openpilot.common.scons_cache_lock import acquire_cache_lock; from pathlib import Path; import sys; ' +
                    'h=acquire_cache_lock(Path(sys.argv[1])); print("ready",flush=True); sys.stdin.readline()')
    process = subprocess.Popen([sys.executable, '-c', code, str(self.cache)], env=self.env,
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    self.addCleanup(self.stop, process)
    # Bounded readiness: a child failure cannot hang the test indefinitely.
    import select
    readable, _, _ = select.select([process.stdout], [], [], 5)
    self.assertTrue(readable, 'lock holder did not become ready')
    self.assertEqual(process.stdout.readline().strip(), 'ready')
    return process

  @staticmethod
  def stop(process):
    if process.poll() is None:
      process.communicate('\n', timeout=5)
    else:
      process.communicate(timeout=5)

  def reset(self):
    (ROOT / 'launch_chffrplus.sh').read_text()
    repo = self.root / 'repo'
    repo.mkdir(exist_ok=True)
    for name in ('prebuilt', '.sconsign.dblite'):
      (repo / name).touch()
    return self.launcher('reset_build_state', repo), repo

  def launcher(self, command, repo=None, mocks='sudo() { return 1; }'):
    # Launcher cache functions on temporary paths only; sudo is never invoked.
    source = (ROOT / 'launch_chffrplus.sh').read_text()
    function = source[source.index('BUILD_STATE_RESET=0'):source.index('function start_manager {')]
    function = function.replace('/data/scons_cache', str(self.cache))
    script = function + '\n' + mocks + '\nDIR="$1"\n' + command
    return subprocess.run(['bash', '-c', script, 'test', str(repo or self.root)], env=dict(self.env, SCONS_CACHE_DIR=str(self.cache)),
                          capture_output=True, text=True, timeout=5)

  def test_another_process_blocks_reset(self):
    holder = self.holder()
    r, repo = self.reset()
    self.assertNotEqual(r.returncode, 0)
    self.assertIn('in use', r.stderr)
    self.assertTrue((self.cache / 'AB/result').exists())
    self.assertTrue((repo / 'prebuilt').exists())
    self.assertTrue((repo / '.sconsign.dblite').exists())
    self.stop(holder)
    r, _ = self.reset()
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.cache.exists())
    self.assertTrue(self.cache.with_name('cache.build.lock').exists())

  def test_another_process_blocks_pruning(self):
    holder = self.holder()
    self.ns['prune_cache'](self.cache)
    self.assertTrue((self.cache / 'AB/result').exists())
    self.stop(holder)
    self.ns['prune_cache'](self.cache)
    self.assertFalse((self.cache / 'AB/result').exists())
    self.assertEqual({p.name for p in self.cache.iterdir() if p.is_file()},
                     {'config', 'CACHEDIR.TAG', 'carrot_params_keys.sha256'})

  def test_second_builder_is_rejected(self):
    self.holder()
    with self.assertRaises(CacheBusyError):
      acquire_cache_lock(self.cache)

  def test_cleanup_guard_blocks_new_builder(self):
    lock_path = self.cache.with_name('cache.build.lock')
    holder = subprocess.Popen(['bash', '-c', 'exec 8>>"$1"; flock -n 8 || exit; echo ready; read -r x', 'test', str(lock_path)],
                              stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    self.addCleanup(self.stop, holder)
    import select
    self.assertTrue(select.select([holder.stdout], [], [], 5)[0])
    self.assertEqual(holder.stdout.readline().strip(), 'ready')
    with self.assertRaises(CacheBusyError):
      acquire_cache_lock(self.cache)

  def test_process_death_releases_guard_without_unlink(self):
    holder = self.holder()
    guard = self.cache.with_name('cache.build.lock')
    inode = guard.stat().st_ino
    holder.kill()
    holder.wait(timeout=5)
    with acquire_cache_lock(self.cache):
      self.assertEqual(guard.stat().st_ino, inode)

  def test_cache_recreation_does_not_split_guard(self):
    import shutil
    with acquire_cache_lock(self.cache):
      inode = self.cache.with_name('cache.build.lock').stat().st_ino
      shutil.rmtree(self.cache)
      self.cache.mkdir()
      with self.assertRaises(CacheBusyError):
        acquire_cache_lock(self.cache)
      self.assertEqual(self.cache.with_name('cache.build.lock').stat().st_ino, inode)

  def test_guard_open_error_preserves_cache(self):
    guard = self.cache.with_name('cache.build.lock')
    guard.mkdir()
    r, repo = self.reset()
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue((self.cache / 'AB/result').exists())
    self.assertTrue((repo / 'prebuilt').exists())
    self.ns['prune_cache'](self.cache)
    self.assertTrue((self.cache / 'AB/result').exists())

  def test_symlink_guard_is_rejected(self):
    target = self.root / 'target'
    target.touch()
    self.cache.with_name('cache.build.lock').symlink_to(target)
    with self.assertRaises(OSError):
      acquire_cache_lock(self.cache)
    r, _ = self.reset()
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue((self.cache / 'AB/result').exists())

  def test_sconstruct_keeps_guard_until_process_exit(self):
    source = (ROOT / 'SConstruct').read_text()
    block = source[source.index('# Hold a stable, external guard'):source.index('CacheDir(cache_dir)')]
    code = ('import sys; from types import SimpleNamespace; ' +
            'scope={"cache_dir":sys.argv[1],"os":__import__("os"),"SCons":SimpleNamespace(Errors=SimpleNamespace(UserError=RuntimeError))}; ' +
            'exec(' + repr(block) + ',scope); del scope; import gc; gc.collect(); ' +
            'print("ready",flush=True); sys.stdin.readline()')
    self.holder(code)
    with self.assertRaises(CacheBusyError):
      acquire_cache_lock(self.cache)

  def test_lock_failure_preserves_cache_and_stops_retries(self):
    for output in (b'SConsLockFailure: config timeout', b'SCons cache is in use: cache'):
      with self.subTest(output=output):
        calls, errors = [], []
        def popen(*a, calls=calls, output=output, **kw):
          calls.append(a)
          return SimpleNamespace(returncode=120, poll=lambda: 120, stdout=io.BytesIO(output))
        self.ns.update(subprocess=SimpleNamespace(Popen=popen, PIPE=-1, STDOUT=-2),
                       cloudlog=SimpleNamespace(error=errors.append), add_file_handler=lambda *a: None)
        with patch.dict(os.environ, {'CARROT_STARTUP_RECOVERY': '1'}):
          with self.assertRaises(SystemExit):
            self.ns['build'](SimpleNamespace(update=lambda *a: None, close=lambda: None))
        self.assertEqual(len(calls), 1)
        self.assertIn('cache preserved', errors[0])
        self.assertTrue((self.cache / 'AB/result').exists())

  def test_inherited_descriptor_reuses_lock_and_parent_keeps_it(self):
    with acquire_cache_lock(self.cache) as parent:
      code = ('from pathlib import Path; import sys; from openpilot.common.scons_cache_lock import acquire_cache_lock; ' +
              'h=acquire_cache_lock(Path(sys.argv[1]),sys.argv[2]); h.close()')
      r = subprocess.run([sys.executable, '-c', code, str(self.cache), str(parent.fileno())],
                         env=self.env, pass_fds=(parent.fileno(),), capture_output=True, text=True, timeout=5)
      self.assertEqual(r.returncode, 0, r.stderr)
      with self.assertRaises(CacheBusyError):
        acquire_cache_lock(self.cache)

  def test_wrong_inherited_guard_is_rejected(self):
    with acquire_cache_lock(self.root / 'other') as other:
      with self.assertRaises(OSError):
        acquire_cache_lock(self.cache, str(other.fileno()))

  def test_invalid_inherited_descriptor_is_rejected(self):
    with self.assertRaises((OSError, ValueError)):
      acquire_cache_lock(self.cache, 'not-a-descriptor')

  def test_params_holds_guard_through_scons_and_stamp(self):
    repo = self.root / 'params-repo'
    (repo / 'scripts').mkdir(parents=True)
    common = repo / 'openpilot/common'
    common.mkdir(parents=True)
    (common / 'params_keys.h').write_text('header')
    script = repo / 'scripts/ensure_params_build.sh'
    script.write_text((ROOT / 'scripts/ensure_params_build.sh').read_text().replace(
      'echo "Params registry changed; rebuilding params_pyx.so."', ':'))
    bin_dir = self.root / 'bin'
    bin_dir.mkdir()
    fake = bin_dir / 'scons'
    fake.write_text('#!' + sys.executable + '\n' +
      'import os,sys\nfrom pathlib import Path\nfrom openpilot.common.scons_cache_lock import acquire_cache_lock\n' +
      'h=acquire_cache_lock(Path(os.environ["SCONS_CACHE_DIR"]),os.environ["SCONS_CACHE_LOCK_FD"])\n' +
      'print("ready",flush=True)\nsys.stdin.readline()\nh.close()\n')
    fake.chmod(0o755)
    # The device-specific Params import is outside this concurrency test.
    (bin_dir / 'python3').write_text('#!/bin/sh\nexit 0\n')
    (bin_dir / 'python3').chmod(0o755)
    env = dict(self.env, SCONS_CACHE_DIR=str(self.cache), PATH=str(bin_dir)+os.pathsep+os.environ['PATH'])
    process = subprocess.Popen(['bash', str(script)], env=env, stdin=subprocess.PIPE,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    self.addCleanup(self.stop, process)
    # Only the child readiness message is emitted, avoiding buffered-line races.
    import select
    self.assertTrue(select.select([process.stdout], [], [], 5)[0])
    self.assertEqual(process.stdout.readline().strip(), 'ready')
    with self.assertRaises(CacheBusyError):
      acquire_cache_lock(self.cache)
    out, err = process.communicate('\n', timeout=5)
    self.assertEqual(process.returncode, 0, err)
    self.assertEqual(len((self.cache / 'carrot_params_keys.sha256').read_text().strip()), 64)
    with acquire_cache_lock(self.cache):
      pass

  def test_guard_created_by_another_user_still_locks(self):
    # e.g. a guard created by `sudo scons`: read-only access must be enough.
    guard = self.cache.with_name('cache.build.lock')
    guard.touch()
    guard.chmod(0o444)
    with acquire_cache_lock(self.cache):
      with self.assertRaises(CacheBusyError):
        acquire_cache_lock(self.cache)
    r, _ = self.reset()
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse(self.cache.exists())

  def test_builders_wait_for_each_other(self):
    holder = self.holder()
    import threading
    threading.Timer(0.5, self.stop, (holder,)).start()
    with acquire_cache_lock(self.cache, timeout=5):
      pass

  def test_heal_removes_stale_lock_and_keeps_healthy_cache(self):
    (self.cache / 'config.lock').touch()
    r = self.launcher('heal_scons_cache')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertFalse((self.cache / 'config.lock').exists())
    self.assertTrue((self.cache / 'AB/result').exists())

  def test_heal_never_touches_a_cache_in_use(self):
    (self.cache / 'config.lock').write_text('live')
    self.holder()
    r = self.launcher('heal_scons_cache', mocks='rm() { echo unexpected-removal; return 1; }')
    self.assertEqual(r.returncode, 0, r.stderr)
    self.assertNotIn('unexpected-removal', r.stdout)
    self.assertEqual((self.cache / 'config.lock').read_text(), 'live')

  def test_heal_does_not_follow_cache_symlink_to_external_lock(self):
    import shutil
    external = self.root / 'external'
    external.mkdir()
    (external / 'config.lock').write_text('keep')
    shutil.rmtree(self.cache)
    self.cache.symlink_to(external, target_is_directory=True)
    result = self.launcher('heal_scons_cache')
    self.assertEqual(result.returncode, 0, result.stderr)
    self.assertFalse(self.cache.is_symlink())
    self.assertEqual((external / 'config.lock').read_text(), 'keep')

  def test_sconstruct_uses_exported_params_cache_path(self):
    source = (ROOT / 'SConstruct').read_text()
    tree = ast.parse(source)
    assignment = next(node for node in tree.body if isinstance(node, ast.Assign)
                      and any(isinstance(t, ast.Name) and t.id == 'cache_dir' for t in node.targets))
    with patch.dict(os.environ, {'SCONS_CACHE_DIR': str(self.cache)}):
      scope = {'os': os, 'arch': 'larch64'}
      exec(compile(ast.Module(body=[assignment], type_ignores=[]), '<cache-path>', 'exec'), scope)
    self.assertEqual(scope['cache_dir'], str(self.cache))

  def test_heal_drops_cache_owned_by_another_user_with_sudo(self):
    # Simulate a root-owned cache: plain rm fails, only sudo can remove it.
    mocks = ('rm() { if [ -z "$AS_ROOT" ]; then return 1; fi; command rm "$@"; }\n' +
             'sudo() { [ "$1" = -n ] && shift; chmod 700 "$SCONS_CACHE"; AS_ROOT=1 "$@"; }')
    self.cache.chmod(0o500)
    try:
      r = self.launcher('heal_scons_cache', mocks=mocks)
    finally:
      if self.cache.exists():
        self.cache.chmod(0o700)
    self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
    self.assertFalse(self.cache.exists())
    self.assertIn('not usable', r.stdout)

  def test_heal_reports_cache_it_cannot_remove(self):
    self.cache.chmod(0o500)
    try:
      r = self.launcher('heal_scons_cache')
    finally:
      self.cache.chmod(0o700)
    self.assertNotEqual(r.returncode, 0)
    self.assertTrue(self.cache.exists())
    self.assertIn('Cannot remove', r.stderr)

  def test_cache_pruning_is_outside_the_optional_model_build(self):
    source = (ROOT / 'openpilot/system/manager/build.py').read_text()
    body = source[source.index('def build_usbgpu_model'):source.index('def build(', source.index('def build_usbgpu_model'))]
    self.assertNotIn('.unlink()', body)

  def test_agnos_init_has_no_unguarded_cache_removal(self):
    source = (ROOT / 'launch_chffrplus.sh').read_text()
    start = source.index('function agnos_init {')
    end = source.index('\nfunction ', start + 1)
    self.assertNotIn('scons_cache/config.lock', source[start:end])

  def test_launcher_repairs_cache_before_consuming_request(self):
    source = (ROOT / 'launch_chffrplus.sh').read_text()
    block = source[source.index('  if ! heal_scons_cache;'):source.index('  # Build Params before')]
    prefix = ('heal_scons_cache() { echo repair; return "$1"; }\n' +
              'consume_rebuild_request() { echo consume; }\n' +
              'show_startup_failure() { echo failure; }\n')
    # Test both execution order and failure propagation, without actual cleanup.
    for rc in (0, 1):
      script = prefix.replace('return "$1"', 'return '+str(rc)) + 'run() {\n' + block + '\necho params;\n}\nrun'
      result = subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=5)
      self.assertEqual(result.stdout.splitlines(), ['repair', 'consume', 'params'] if rc == 0 else ['repair', 'failure'])
      self.assertEqual(result.returncode, rc)

  def test_wait_message_is_printed_once_and_timeout_preserves_cache(self):
    from contextlib import redirect_stderr
    self.holder()
    output = io.StringIO()
    with redirect_stderr(output), self.assertRaises(CacheBusyError):
      acquire_cache_lock(self.cache, timeout=0.3)
    self.assertEqual(output.getvalue().count('Waiting for another SCons build'), 1)
    self.assertTrue((self.cache / 'AB/result').exists())

if __name__ == '__main__':
  unittest.main()
