import ast
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from openpilot.common.basedir import BASEDIR
from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld import big_model, big_model_status, helpers


@pytest.mark.parametrize('exists, query_result, reusable', [(True, 0, True), (True, 1, False), (True, 2, False), (False, 0, False)])
def test_optional_model_reuse_checks_dependencies(tmp_path: Path, monkeypatch, exists, query_result, reusable):
  # Isolate the production startup function from Linux hardware imports. The
  # SCons/readiness boundaries are faked; stale artifacts must reach readiness.
  tree = ast.parse((Path(BASEDIR) / 'openpilot/system/manager/build.py').read_text(encoding='utf8'))
  body = [n for n in tree.body if isinstance(n, ast.Assign) and any(
    isinstance(t, ast.Name) and t.id.startswith('USBGPU_') for t in n.targets)]
  body += [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'build_usbgpu_model']
  path = tmp_path / 'big.pkl'
  manifest_path = Path(get_manifest_path(path))
  if exists:
    manifest_path.write_text('1')
  manifest = SimpleNamespace(model_id='test', sha256='a' * 64, size=10)
  monkeypatch.setattr(big_model, 'active_model_path', lambda: tmp_path / 'big.onnx')
  monkeypatch.setattr(big_model, 'active_manifest', lambda: manifest)
  monkeypatch.setattr(big_model, 'model_cache_dir', lambda: tmp_path)
  monkeypatch.setattr(helpers, 'modeld_pkl_path', lambda **kwargs: path)
  monkeypatch.setattr(helpers, 'usbgpu_present', lambda: True)
  statuses, queries, readiness = [], [], []
  monkeypatch.setattr(big_model_status, 'write_big_model_status', lambda directory, state, **kwargs: statuses.append(state))

  def query(command, **kwargs):
    assert command == ['scons', '-q', os.path.relpath(manifest_path, tmp_path)]
    assert kwargs['env']['BUILD_USB_GPU_MODEL'] == '1'
    assert kwargs['cwd'] == str(tmp_path)
    queries.append(command)
    return SimpleNamespace(returncode=query_result)

  def unavailable(**kwargs):
    readiness.append(kwargs)
    return 'test GPU unavailable'

  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware.usbgpu', SimpleNamespace(check_usbgpu=unavailable))
  namespace = {'Path': Path, 'Spinner': object, 'BASEDIR': str(tmp_path), 'get_manifest_path': get_manifest_path,
               'os': os, 'subprocess': SimpleNamespace(run=query)}
  exec(compile(ast.Module(body=body, type_ignores=[]), '<optional model build>', 'exec'), namespace)
  result = namespace['build_usbgpu_model'](SimpleNamespace(update=lambda text: None))
  assert result is reusable
  assert len(queries) == int(exists)
  assert bool(readiness) is not reusable
  assert ('compiled' in statuses) is reusable
  assert manifest_path.exists() is exists  # Preserve old artifacts through transient failures.


def test_scons_rebuilds_model_when_serialization_helper_changes(tmp_path: Path):
  # Use the actual compiler dependency list with a tiny SCons build, avoiding
  # an ONNX/GPU compile while exercising SCons' dependency signatures.
  source = (Path(BASEDIR) / 'openpilot/selfdrive/modeld/SConscript').read_text(encoding='utf8')
  tree = ast.parse(source)
  deps = next(n for n in tree.body if isinstance(n, ast.Assign) and any(
    isinstance(t, ast.Name) and t.id == 'compile_modeld_script' for t in n.targets))
  namespace = {'File': lambda name: name.lstrip('#'), 'modeld_dir': 'openpilot/selfdrive/modeld'}
  exec(compile(ast.Module(body=[deps], type_ignores=[]), '<model dependencies>', 'exec'), namespace)
  for name in namespace['compile_modeld_script']:
    path = tmp_path / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('initial')
  helpers_path = tmp_path / 'openpilot/selfdrive/modeld/helpers.py'
  helpers_path.write_text('initial')
  (tmp_path / 'SConstruct').write_text(
    'env = Environment(tools=[])\n' +
    'def compile_model(target, source, env):\n' +
    '  open(str(target[0]), "w").write("compiled")\n' +
    f'env.Command("model.chunkmanifest", {namespace["compile_modeld_script"]!r}, compile_model)\n')
  command = [sys.executable, '-m', 'SCons', '--max-drift=-1', 'model.chunkmanifest']
  def run(*args):
    result = subprocess.run(command + list(args), cwd=tmp_path, capture_output=True, text=True, check=False)
    assert result.returncode in (0, 1), result.stdout + result.stderr
    return result.returncode
  assert run() == 0
  assert run('-q') == 0
  helpers_path.write_text('changed serialization implementation')
  modified_time = helpers_path.stat().st_mtime + 2
  os.utime(helpers_path, (modified_time, modified_time))
  assert run('-q') == 1
