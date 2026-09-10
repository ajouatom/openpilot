import sys
import shlex
import shutil
import subprocess
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.system.manager import params_check


SOURCE = '''
inline static std::unordered_map<std::string, ParamKeyAttributes> keys = {
  {"Existing", {PERSISTENT, BOOL}},
  {"AutoNaviSpeedBumpEndDistance", {PERSISTENT, INT, "200"}},
  // {"Disabled", {PERSISTENT, INT}},
  /* {"DisabledBlock", {PERSISTENT, INT}}, */
};
'''


def test_registered_keys_accept_bytes_and_ignore_disabled_declarations():
  params_check.check_keys(SOURCE, [b'Existing', b'AutoNaviSpeedBumpEndDistance'])


def test_incident_native_registry_requires_rebuild():
  with pytest.raises(ValueError, match='missing=.*AutoNaviSpeedBumpEndDistance'):
    params_check.check_keys(SOURCE, [b'Existing'])


def test_switching_to_branch_without_key_requires_rebuild():
  with pytest.raises(ValueError, match='obsolete=.*OldBranchKey'):
    params_check.check_keys(SOURCE, ['Existing', 'AutoNaviSpeedBumpEndDistance', 'OldBranchKey'])


def test_unreadable_registry_format_cannot_pass():
  with pytest.raises(ValueError, match='No Params keys'):
    params_check.check_keys('', [])


def test_boot_check_fails_until_native_registry_is_rebuilt(monkeypatch, tmp_path):
  header = tmp_path / 'params_keys.h'
  header.write_text(SOURCE, encoding='utf-8')
  monkeypatch.setattr(params_check, 'PARAMS_HEADER', header)
  keys = [b'Existing']
  native = SimpleNamespace(Params=lambda: SimpleNamespace(all_keys=lambda: keys))
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', native)
  assert params_check.main() == 1
  keys.append(b'AutoNaviSpeedBumpEndDistance')
  assert params_check.main() == 0


def test_broken_native_extension_fails_boot_check(monkeypatch):
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', None)
  assert params_check.main() == 1


@pytest.mark.parametrize('native_result, expected_rebuild', [(0, '0'), (1, '1')])
def test_prebuilt_launcher_requests_build_for_mismatched_registry(tmp_path, native_result, expected_rebuild):
  bash = shutil.which('bash')
  if bash is None:
    pytest.skip('bash is needed to execute the vehicle launcher function')
  for name in ('system/loggerd/loggerd', 'system/loggerd/encoderd', 'system/camerad/camerad'):
    path = tmp_path / 'openpilot' / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('#!/bin/sh\nexit 0\n')
    path.chmod(0o755)
  (tmp_path / 'prebuilt').touch()
  launcher = (Path(__file__).resolve().parents[3] / 'launch_chffrplus.sh').read_text(encoding='utf-8')
  function = launcher.split('function invalidate_native_build_if_needed {', 1)[1].split('\n}', 1)[0]
  script = f'''
DIR={shlex.quote(tmp_path.as_posix())}
FORCE_REBUILD=0
python3() {{ return {native_result}; }}
invalidate_native_build_if_needed() {{{function}
}}
invalidate_native_build_if_needed
echo "REBUILD=$FORCE_REBUILD"
'''
  result = subprocess.run([bash, '-c', script], capture_output=True, text=True, check=True)
  assert f'REBUILD={expected_rebuild}' in result.stdout


@pytest.mark.parametrize('native_result', [0, 1])
def test_post_build_check_blocks_manager_if_native_registry_stays_stale(native_result):
  bash = shutil.which('bash')
  if bash is None:
    pytest.skip('bash is needed to execute the vehicle launcher guard')
  launcher = (Path(__file__).resolve().parents[3] / 'launch_chffrplus.sh').read_text(encoding='utf-8')
  guard = launcher.split('  # Never start driving services', 1)[1].split('  start_big_model_update', 1)[0]
  guard = guard[guard.index('\n'):]
  script = f'''
python3() {{ return {native_result}; }}
launch_test() {{{guard}
echo manager_started
}}
launch_test
'''
  result = subprocess.run([bash, '-c', script], capture_output=True, text=True)
  assert result.returncode == native_result
  assert ('manager_started' in result.stdout) == (native_result == 0)
