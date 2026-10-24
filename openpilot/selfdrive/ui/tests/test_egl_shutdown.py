"""Real EGL ownership/shutdown code with native API doubles; no GL window."""
import ast
import importlib.util
import os
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def egl(monkeypatch):
  errors = []
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(
    exception=lambda *a: errors.append(a), error=lambda *a: None)))
  spec = importlib.util.spec_from_file_location('tested_camera_egl', ROOT / 'system/ui/lib/egl.py')
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, spec.name, module)
  spec.loader.exec_module(module)
  module._egl = NS(initialized=True, ffi=NS(NULL=None, new=lambda _, data: data),
                   display=11, NO_CONTEXT=0, NO_IMAGE_KHR=0,
                   create_image_khr=lambda *a: object(), destroy_image_khr=lambda *a: True)
  module.errors = errors
  yield module
  module.destroy_all_egl_images()


def image(egl):
  source, writer = os.pipe()
  try:
    value = egl.create_egl_image(16, 16, 16, source, 256)
    assert value is not None
    os.fstat(value.fd)
    return value
  finally:
    os.close(source)
    os.close(writer)


def test_destroy_is_idempotent_even_after_fd_number_is_reused(egl):
  value = image(egl)
  fd = value.fd
  egl.destroy_egl_image(value)
  assert value.fd == -1 and value.egl_image is None and not egl._images
  source, writer = os.pipe()
  try:
    os.dup2(source, fd)
    egl.destroy_egl_image(value)
    os.fstat(fd)  # A second cleanup must not close a later user's descriptor.
  finally:
    for owned in {source, writer, fd}:
      os.close(owned)


def test_shutdown_releases_all_remaining_imports_after_one_native_error(egl):
  values = [image(egl) for _ in range(3)]
  fds = [v.fd for v in values]
  bad = values[1].egl_image
  def destroy(display, handle):
    if handle is bad:
      raise RuntimeError('native failure')
    return True
  egl._egl.destroy_image_khr = destroy
  egl.destroy_all_egl_images()
  assert not egl._images and len(egl.errors) == 1
  for fd in fds:
    with pytest.raises(OSError): os.fstat(fd)
  egl.destroy_all_egl_images()
  for value in values: egl.destroy_egl_image(value)


def test_failed_import_closes_its_duplicate_without_entering_ledger(egl):
  egl._egl.create_image_khr = lambda *a: 0
  egl._egl.get_error = lambda: 0x3003
  source, writer = os.pipe()
  try:
    assert egl.create_egl_image(16, 16, 16, source, 256) is None
    assert not egl._images
    os.fstat(source)
  finally:
    os.close(source); os.close(writer)


def test_actual_application_close_releases_import_before_display(monkeypatch, egl):
  value = image(egl)
  fd = value.fd
  calls = []
  ready = [True]
  def window_close():
    assert not egl._images and value.egl_image is None
    with pytest.raises(OSError): os.fstat(fd)
    calls.append('window')
    ready[0] = False
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.egl', egl)
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.native_text', NS(clear=lambda: None))
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.text_measure', NS(_cache={}))
  tree = ast.parse((ROOT / 'system/ui/lib/application.py').read_text(encoding='utf-8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GuiApplication')
  close = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'close')
  code = ast.Module(body=[close], type_ignores=[])
  env = {'rl': NS(is_window_ready=lambda: ready[0], close_window=window_close), 'PC': True}
  exec(compile(code, 'actual_application_close', 'exec'), env)
  app = NS(_textures={}, _fonts={}, _render_texture=None, _burn_in_shader=None,
           close_ffmpeg=lambda: None, stop_recording=lambda: None)
  env['close'](app)
  env['close'](app)
  assert calls == ['window']
