from pathlib import Path
import os
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from readonly_boot import patch_nv_script


def test_patch_is_idempotent_and_rejects_unknown_scripts():
  source = '#!/bin/bash\nln -sf "$source" "$target"\n'
  patched = patch_nv_script(source)
  assert patch_nv_script(patched) == patched
  assert 'command ln -sf --' in patched
  with pytest.raises(ValueError):
    patch_nv_script('#!/bin/sh\nexit 0\n')


@pytest.mark.skipif(sys.platform != 'linux', reason='Real shell and symlinks')
def test_correct_links_need_no_write_but_incorrect_links_still_fail(tmp_path):
  target = tmp_path / 'target'
  target.symlink_to('/immutable/library')
  commands = tmp_path / 'bin'
  commands.mkdir()
  ln = commands / 'ln'
  ln.write_text('#!/bin/sh\nexit 71\n')
  ln.chmod(0o755)
  script = tmp_path / 'nv.sh'
  script.write_text(patch_nv_script(
    '#!/bin/bash\nln -sf "/immutable/library" "$1"\nprintf "runtime-step\\n"\n'))
  environment = dict(os.environ, PATH=str(commands) + ':' + os.environ['PATH'])
  good = subprocess.run(['bash', str(script), str(target)], env=environment, capture_output=True, text=True)
  assert good.returncode == 0 and good.stdout == 'runtime-step\n'
  assert str(target.readlink()) == '/immutable/library'
  target.unlink()
  target.symlink_to('/wrong/library')
  bad = subprocess.run(['bash', str(script), str(target)], env=environment, capture_output=True, text=True)
  assert bad.returncode != 0
  assert str(target.readlink()) == '/wrong/library'
