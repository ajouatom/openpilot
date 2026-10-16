import ast
import builtins
import hashlib
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

SOURCE = Path(__file__).resolve().parents[1] / 'cluster/cluster_graphics_cleanup.py'


class CallRecorder:
  def __init__(self, result):
    self.result = result
    self.calls = []

  def __call__(self, *args, **kwargs):
    self.calls.append((args, kwargs))
    return self.result


@pytest.fixture
def cleanup(monkeypatch, tmp_path):
  spec = importlib.util.spec_from_file_location('test_hud_graphics_cleanup_module', SOURCE)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  actual_is_file = Path.is_file
  monkeypatch.setattr(Path, 'is_file', lambda p: str(p).replace('\\', '/') == '/TICI' or actual_is_file(p))
  monkeypatch.delenv('RAYLIB_BACKEND', raising=False)
  monkeypatch.delenv('DRM_FD', raising=False)
  monkeypatch.delitem(sys.modules, 'pyray', raising=False)
  monkeypatch.delitem(sys.modules, 'raylib', raising=False)
  binary = tmp_path / 'native.so'
  binary.write_bytes(b'verified fixture')
  monkeypatch.setattr(module, 'KNOWN_BINARIES', {'fixture': hashlib.sha256(binary.read_bytes()).hexdigest()})
  dist = SimpleNamespace(version='fixture', locate_file=lambda _: binary)
  monkeypatch.setattr(module.importlib.metadata, 'distribution', lambda _: dist)
  lib = SimpleNamespace(carrot_gbm_cleanup_enable=CallRecorder(1))
  load = CallRecorder(lib)
  monkeypatch.setattr(module.ctypes, 'CDLL', load)
  return module, dist, binary, lib, load


@pytest.mark.parametrize('borrowed', [False, True])
def test_loads_only_once_and_preserves_borrowed_drm_descriptor(cleanup, monkeypatch, borrowed):
  module, _, _, lib, load = cleanup
  if borrowed:
    monkeypatch.setenv('DRM_FD', '42')
  assert module.install_graphics_cleanup()
  monkeypatch.setitem(sys.modules, 'raylib', SimpleNamespace())
  assert module.install_graphics_cleanup()
  assert len(load.calls) == 1
  assert load.calls[0][1]['mode'] == module.ctypes.RTLD_GLOBAL
  assert lib.carrot_gbm_cleanup_enable.calls == [((0 if borrowed else 1,), {})]


@pytest.mark.parametrize('failure', ['version', 'hash', 'already_loaded'])
def test_rejects_unknown_or_already_bound_library_before_loading_native_hook(cleanup, monkeypatch, failure):
  module, dist, binary, _, load = cleanup
  if failure == 'version':
    dist.version = 'future-fixed'
  elif failure == 'hash':
    binary.write_bytes(b'different binary')
  else:
    monkeypatch.setitem(sys.modules, 'pyray', SimpleNamespace())
  with pytest.raises(RuntimeError):
    module.install_graphics_cleanup()
  assert not load.calls


def test_native_initialization_failure_does_not_report_installed(cleanup):
  module, _, _, lib, _ = cleanup
  lib.carrot_gbm_cleanup_enable.result = 0
  with pytest.raises(RuntimeError, match='initialization failed'):
    module.install_graphics_cleanup()
  assert module._library is None


def test_other_backend_is_not_interposed(cleanup, monkeypatch):
  module, _, _, _, load = cleanup
  monkeypatch.setenv('RAYLIB_BACKEND', 'desktop')
  assert not module.install_graphics_cleanup()
  assert not load.calls


def test_desktop_host_is_not_interposed(cleanup, monkeypatch):
  module, _, _, _, load = cleanup
  monkeypatch.setattr(Path, 'is_file', lambda _: False)
  assert not module.install_graphics_cleanup()
  assert not load.calls


def test_inherited_graphics_reexecs_only_hud_with_a_bounded_guard(cleanup, monkeypatch):
  module, _, _, _, _ = cleanup
  monkeypatch.setitem(sys.modules, 'pyray', SimpleNamespace())
  monkeypatch.delenv('CARROT_HUD_CLEAN_EXEC', raising=False)
  execute = CallRecorder(None)
  monkeypatch.setattr(module.os, 'execve', execute)
  module.ensure_clean_autorun_process()
  assert len(execute.calls) == 1
  args = execute.calls[0][0]
  assert args[:2] == (sys.executable, [sys.executable, '-m', 'openpilot.selfdrive.carrot.cluster_autorun'])
  assert args[2]['CARROT_HUD_CLEAN_EXEC'] == '1'
  monkeypatch.setenv('CARROT_HUD_CLEAN_EXEC', '1')
  with pytest.raises(RuntimeError, match='fresh process'):
    module.ensure_clean_autorun_process()
  assert len(execute.calls) == 1


def test_clean_process_does_not_reexec(cleanup, monkeypatch):
  module, _, _, _, _ = cleanup
  execute = CallRecorder(None)
  monkeypatch.setattr(module.os, 'execve', execute)
  module.ensure_clean_autorun_process()
  assert not execute.calls


def test_autorun_installs_cleanup_before_encoder_import_and_render():
  source = SOURCE.parent.parent / 'cluster_autorun.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_run_cluster_once')
  events = []
  entry = SimpleNamespace(configure_cluster_graphics=lambda: events.append('cleanup'),
                          main=lambda **kwargs: events.append('render'))

  def import_module(name, *args, **kwargs):
    if name == 'selfdrive.carrot':
      return SimpleNamespace(cluster_run=entry)
    if name == 'cluster_h264_pipeline':
      assert events == ['cleanup'], 'encoder can bind pyray before the cleanup hook'
      events.append('encoder')
      return SimpleNamespace(H264PipelineInitializationError=type('EncoderError', (Exception,), {}))
    return builtins.__import__(name, *args, **kwargs)

  fake_sys = SimpleNamespace(argv=['hud'])
  scope = {'__builtins__': dict(vars(builtins), __import__=import_module), 'sys': fake_sys,
           '_encoder_sequence': lambda _: [1], '_cluster_args': lambda *args: [],
           'ENCODER_NAMES': {1: 'jpeg'}, 'ENCODER_AUTO': 0}
  exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), scope)
  scope['_run_cluster_once'](1, 1)
  assert events == ['cleanup', 'encoder', 'render']
  assert fake_sys.argv == ['hud']
