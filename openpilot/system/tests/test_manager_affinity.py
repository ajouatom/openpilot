import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


def manager_startup_source() -> str:
  source = (Path(__file__).resolve().parents[3] / 'launch_chffrplus.sh').read_text()
  return source[source.index('function start_manager {'):source.index('function launch {')]


@pytest.mark.parametrize('agnos', [False, True])
def test_manager_startup_platform_and_exit_status(tmp_path: Path, agnos: bool) -> None:
  bash = shutil.which('bash')
  if not bash:
    pytest.skip('bash unavailable')
  manager = tmp_path / 'manager.py'
  manager.write_text('#!/usr/bin/env bash\necho manager-started\nexit 23\n')
  manager.chmod(0o755)
  # Fake only hardware detection and taskset; execute the real launch function
  # and a child script, including propagation of the manager's failure status.
  harness = '''
function [ {
  if builtin [ "$#" = 3 ] && builtin [ "$1" = -f ] && builtin [ "$2" = /AGNOS ]; then
    return AGNOS_STATUS
  fi
  builtin [ "$@"
}
taskset() {
  builtin [ "$1" = -c ] || return 99
  echo "affinity=$2"
  shift 2
  "$@"
}
'''.replace('AGNOS_STATUS', '0' if agnos else '1')
  result = subprocess.run([bash, '-c', harness + manager_startup_source() + '\nstart_manager\n'],
                          cwd=tmp_path, capture_output=True, text=True, timeout=10)
  assert result.returncode == 23, result.stdout + result.stderr
  assert result.stdout.splitlines() == (['affinity=0-5'] if agnos else []) + ['manager-started']


@pytest.mark.skipif(sys.platform != "linux", reason="Linux CPU affinity integration test")
@pytest.mark.parametrize("isolated_parent", [False, True])
def test_manager_startup_affinity_inheritance(tmp_path: Path, isolated_parent: bool) -> None:
  agnos = Path('/AGNOS').is_file()
  if isolated_parent and not agnos:
    pytest.skip("isolated camera/model CPUs are an AGNOS configuration")
  bash = shutil.which('bash')
  taskset = shutil.which('taskset')
  if not bash or (agnos and not taskset):
    pytest.skip("bash/taskset unavailable")

  startup = manager_startup_source()
  manager = tmp_path / 'manager.py'
  manager.write_text(f'#!{sys.executable}\n' + '''
import json
import os
import subprocess
import sys
import threading

result = {'manager': sorted(os.sched_getaffinity(0))}
def worker():
  result['thread'] = sorted(os.sched_getaffinity(0))
t = threading.Thread(target=worker)
t.start()
t.join()
probe = 'import json, os; print(json.dumps(sorted(os.sched_getaffinity(0))))'
result['child'] = json.loads(subprocess.check_output([sys.executable, '-c', probe], text=True))
if os.path.isfile('/AGNOS'):
  # A process with a dedicated core must still be able to override the default.
  probe = 'import os; os.sched_setaffinity(0, {7}); ' + probe
  result['dedicated'] = json.loads(subprocess.check_output([sys.executable, '-c', probe], text=True))
print(json.dumps(result))
''')
  manager.chmod(0o755)
  command = [bash, '-c', startup + '\nstart_manager\n']
  if isolated_parent:
    command = [taskset, '-c', '7', *command]
  output = subprocess.check_output(command, cwd=tmp_path, text=True, timeout=15)
  result = json.loads(output)
  expected = list(range(6)) if agnos else sorted(os.sched_getaffinity(0))
  assert result['manager'] == result['thread'] == result['child'] == expected
  if agnos:
    assert result['dedicated'] == [7]
