import ast
import json
import os
import shlex
import shutil
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from openpilot.common.basedir import BASEDIR
from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld import big_model, big_model_status, helpers, precompiled_model


@pytest.mark.parametrize('artifact_ready, expected_rebuild', [(True, '0'), (False, '1')])
def test_launcher_accepts_precompiled_model_without_legacy_chunks(tmp_path, artifact_ready, expected_rebuild):
  bash = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else shutil.which('bash')
  if not bash or not Path(bash).is_file():
    pytest.skip('bash unavailable')
  models = tmp_path / 'openpilot/selfdrive/modeld/models'
  models.mkdir(parents=True)
  (models / '.build_stamp').write_text('stamp:')
  (models / '.big_model_build_stamp').write_text('model-sha')
  (models / 'tg_input_devices.json').write_text('{}')
  (models / 'driving_tinygrad.pkl.chunkmanifest').write_text('1')
  source = (Path(BASEDIR) / 'launch_chffrplus.sh').read_text(encoding='utf8')
  function = source[source.index('function invalidate_modeld_build_if_needed {'):]
  function = function[:function.index('\n}')+2]
  script = (f'DIR={shlex.quote(tmp_path.as_posix())}\nBIG_MODEL_SHA=model-sha\nFORCE_REBUILD=0\n'
            + 'git() { echo stamp; }\n'
            + f'big_model_artifact_ready() {{ return {0 if artifact_ready else 1}; }}\n'
            + function + '\ninvalidate_modeld_build_if_needed\necho "$FORCE_REBUILD"\n')
  result = subprocess.run([bash, '-c', script], capture_output=True, text=True, check=True)
  assert result.stdout.strip().splitlines()[-1] == expected_rebuild


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
  monkeypatch.setattr(precompiled_model, 'ensure_precompiled', lambda *a, **kw: None)
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


@pytest.mark.parametrize('delivery', ['available', 'missing', 'invalid', 'pcie_off', 'usb_reset', 'worker_timeout', 'validation_timeout'])
def test_precompiled_delivery_or_local_compile_fallback(tmp_path, monkeypatch, delivery):
  tree = ast.parse((Path(BASEDIR) / 'openpilot/system/manager/build.py').read_text(encoding='utf8'))
  body = [n for n in tree.body if isinstance(n, ast.Assign) and any(
    isinstance(t, ast.Name) and t.id.startswith('USBGPU_') for t in n.targets)]
  body += [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'build_usbgpu_model']
  manifest = SimpleNamespace(model_id='test', sha256='a' * 64, size=10)
  monkeypatch.setattr(big_model, 'active_model_path', lambda: tmp_path / 'model.onnx')
  monkeypatch.setattr(big_model, 'active_manifest', lambda: manifest)
  monkeypatch.setattr(big_model, 'model_cache_dir', lambda: tmp_path)
  monkeypatch.setattr(helpers, 'modeld_pkl_path', lambda **kw: tmp_path / 'local.pkl')
  monkeypatch.setattr(helpers, 'usbgpu_present', lambda: True)
  monkeypatch.setattr(big_model_status, 'write_big_model_status', lambda *a, **kw: None)
  path = tmp_path / 'model.pkl'
  (tmp_path / 'installed.json').write_text(json.dumps({'pickle': {'sha256': 'a' * 64}}))
  def ensure(*a, **kw):
    if delivery == 'missing':
      raise FileNotFoundError('no precompiled artifact')
    return path
  monkeypatch.setattr(precompiled_model, 'ensure_precompiled', ensure)
  rejected = []
  monkeypatch.setattr(precompiled_model, 'reject', rejected.append)
  monkeypatch.setitem(sys.modules, 'openpilot.system.hardware.usbgpu', SimpleNamespace(check_usbgpu=lambda **kw: None))
  def validate(command, **kw):
    assert 'openpilot.selfdrive.modeld.precompiled_runner' in command
    assert kw['stderr'] == subprocess.STDOUT and kw['text']
    if delivery == 'invalid':
      raise subprocess.CalledProcessError(1, command, output='ValueError: incompatible model metadata\n')
    if delivery == 'worker_timeout':
      raise subprocess.CalledProcessError(1, command, output='TimeoutError: precompiled eGPU worker timed out\n')
    if delivery == 'validation_timeout':
      raise subprocess.TimeoutExpired(command, 120)
    if delivery in ('pcie_off', 'usb_reset'):
      output = ('RuntimeError: PCIe link not up (LTSSM=0x00), custom firmware not ready\n'
                if delivery == 'pcie_off' else 'RuntimeError: USB bridge reset failed\n')
      raise subprocess.CalledProcessError(1, command, output=output)
    return SimpleNamespace(returncode=0, stdout='')
  def compile_local(command, **kw):
    assert command[0:3] == ['scons', '-j1', '--cache-populate']
    raise RuntimeError('local compiler was invoked')
  namespace = {'Path': Path, 'Spinner': object, 'BASEDIR': str(tmp_path), 'get_manifest_path': get_manifest_path,
               'os': os, 'sys': sys, 'time': SimpleNamespace(time=lambda: 0),
               'subprocess': SimpleNamespace(run=validate, Popen=compile_local, PIPE=-1, STDOUT=-2,
                                             TimeoutExpired=subprocess.TimeoutExpired)}
  exec(compile(ast.Module(body=body, type_ignores=[]), '<optional model build>', 'exec'), namespace)
  if delivery in ('available', 'pcie_off', 'usb_reset', 'worker_timeout', 'validation_timeout'):
    assert namespace['build_usbgpu_model'](SimpleNamespace(update=lambda text: None))
  else:
    with pytest.raises(RuntimeError, match='local compiler was invoked'):
      namespace['build_usbgpu_model'](SimpleNamespace(update=lambda text: None))
  assert rejected == ([path] if delivery == 'invalid' else [])


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
