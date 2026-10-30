"""Old installed bootstraps must recognize the selected model before activation."""
import io
import json
import os
from pathlib import Path
import sys
import subprocess
import tarfile

import pytest

sys.path.insert(0, str(Path(__file__).parent))
from build_host_bundle import export_members, PATHS, ROOT


@pytest.mark.parametrize('catalog_pin', [False, True])
def test_selected_pin_is_exported_under_both_bootstrap_filenames(catalog_pin):
  prefix = 'openpilot/selfdrive/modeld/jetlink/'
  selected = json.dumps({'sha256': 'b' * 64, 'nbytes': 814591106}).encode()
  files = {prefix + 'cinque_v2.json': b'{"sha256":"old"}', prefix + 'jetson_model.json': selected,
           'tools/jetlink/server.py': b'# committed runtime\n'}
  model_path = 'openpilot/selfdrive/modeld/model_catalog/light-speed.json'
  if catalog_pin:
    selected = (ROOT / model_path).read_bytes()
    files[model_path] = selected
  original, result = io.BytesIO(), io.BytesIO()
  with tarfile.open(fileobj=original, mode='w') as archive:
    for name, data in files.items():
      member = tarfile.TarInfo(name)
      member.size = len(data)
      archive.addfile(member, io.BytesIO(data))
  original.seek(0)
  with tarfile.open(fileobj=original) as source, tarfile.open(fileobj=result, mode='w') as target:
    export_members(source, target, model_path if catalog_pin else None)
  result.seek(0)
  with tarfile.open(fileobj=result) as archive:
    assert archive.extractfile(prefix + 'cinque_v2.json').read() == selected
    assert archive.extractfile(prefix + 'jetson_model.json').read() == selected
    assert archive.extractfile('tools/jetlink/server.py').read() == files['tools/jetlink/server.py']


@pytest.mark.skipif(sys.platform != 'linux', reason='Host renderer uses Linux dependencies')
def test_exported_hud_entrypoint_has_all_dependencies(tmp_path):
  # Run outside the checkout: missing bundled modules must not be supplied by
  # the test runner's PYTHONPATH. --help executes the actual lazy imports.
  archive = subprocess.check_output(['git', '-C', str(ROOT), 'archive', 'HEAD', *PATHS])
  with tarfile.open(fileobj=io.BytesIO(archive)) as source:
    source.extractall(tmp_path, filter='data')
  env = {**os.environ, 'PYTHONPATH': str(tmp_path)}
  result = subprocess.run([sys.executable, str(tmp_path / 'tools/jetlink/hud.py'), '--help'],
                          cwd=tmp_path, env=env, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stderr
  assert '--usb-codec' in result.stdout


@pytest.mark.skipif(sys.platform != 'linux', reason='Host server uses Linux dependencies')
def test_exported_server_starts_without_checkout_pythonpath(tmp_path):
  archive = subprocess.check_output(['git', '-C', str(ROOT), 'archive', 'HEAD', *PATHS])
  release = tmp_path / 'release'
  release.mkdir()
  with tarfile.open(fileobj=io.BytesIO(archive)) as source:
    source.extractall(release, filter='data')
  env = dict(os.environ)
  env.pop('PYTHONPATH', None)
  env.pop('PYTHONSAFEPATH', None)
  result = subprocess.run([sys.executable, str(release / 'tools/jetlink/server.py'), '--check-imports'],
                          cwd=tmp_path, env=env, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stderr
  assert 'SERVER_IMPORTS_OK' in result.stdout
