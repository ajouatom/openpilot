from pathlib import Path
import runpy
import subprocess
import sys

import pytest

from openpilot.common.basedir import BASEDIR

version_script = Path(BASEDIR) / 'panda/firmware_version.py'
firmware_source_version = runpy.run_path(str(version_script))['firmware_source_version']


@pytest.fixture
def source_tree(tmp_path):
  files = ('panda/SConscript', 'panda/SConstruct', 'panda/firmware_version.py', 'panda/certs/debug.pub', 'panda/certs/release.pub',
           'panda/board/main.c', 'panda/board/bootstub.c', 'panda/board/stm32h7/flash.ld', 'panda/crypto/sign.py',
           'opendbc/safety/safety.h', 'opendbc/safety/safety/safety_hyundai.h')
  for name in files:
    path = tmp_path / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('initial')
  return tmp_path


def source_version(root):
  return firmware_source_version(root / 'panda', root / 'opendbc/safety')


@pytest.mark.parametrize('name', ['ui.py', 'panda/README.md', 'panda/board/obj/gitversion.h', 'panda/board/jungle/obj/version.h',
                                'opendbc/safety/tests/test_hyundai.py', 'panda/python/__init__.py'])
def test_unrelated_changes_do_not_change_firmware(source_tree, name):
  before = source_version(source_tree)
  path = source_tree / name
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_text('unrelated change')
  assert source_version(source_tree) == before


@pytest.mark.parametrize('name', ['panda/board/main.c', 'panda/board/stm32h7/flash.ld', 'panda/SConscript', 'panda/crypto/sign.py',
                                'panda/certs/debug.pub', 'opendbc/safety/safety/safety_hyundai.h'])
def test_actual_firmware_inputs_change_version_without_git_commit(source_tree, name):
  before = source_version(source_tree)
  (source_tree / name).write_text('changed input')
  assert source_version(source_tree) != before


def test_missing_safety_sources_fail_instead_of_reusing_version(source_tree):
  with pytest.raises(FileNotFoundError):
    firmware_source_version(source_tree / 'panda', source_tree / 'missing')


def test_scons_reuses_firmware_but_rebuilds_after_safety_change(source_tree):
  # Exercise content signatures with the real version generator. No ARM toolchain
  # is needed to prove that a changed header reaches the firmware target.
  (source_tree / 'SConstruct').write_text(
    'from pathlib import Path\nimport runpy\n' +
    f"version = runpy.run_path({str(version_script)!r})['firmware_source_version'](Path('panda'), Path('opendbc/safety'))\n" +
    "Path('version.h').write_text(version)\n" +
    "env = Environment(tools=[])\n" +
    "def build(target, source, env):\n" +
    "  Path(str(target[0])).write_text(Path(str(source[0])).read_text())\n" +
    "env.Command('firmware.bin', 'version.h', build)\n"
  )
  command = [sys.executable, '-m', 'SCons', '--max-drift=-1']
  def build(*args):
    result = subprocess.run(command + list(args), cwd=source_tree, text=True, capture_output=True, check=False)
    assert result.returncode in (0, 1), result.stdout + result.stderr
    return result.returncode
  assert build() == 0
  (source_tree / 'ui.py').write_text('unrelated commit')
  assert build('-q') == 0
  (source_tree / 'opendbc/safety/safety.h').write_text('new policy')
  assert build('-q') == 1
  assert build() == 0
  assert build('-q') == 0
