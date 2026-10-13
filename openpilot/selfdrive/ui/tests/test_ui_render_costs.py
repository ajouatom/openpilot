"""Cache freshness and recording target lifetime without a GPU or device Params."""
import ast
import importlib.util
from pathlib import Path
import queue
from types import SimpleNamespace
import sys

import pytest

LIB = Path(__file__).parents[3] / 'system/ui/lib'


def methods(filename, class_name, names, scope):
  tree = ast.parse((LIB / filename).read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
  cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
  exec(compile(ast.Module(body=[cls], type_ignores=[]), filename, 'exec'), scope)
  return scope[class_name]


@pytest.fixture
def recording():
  calls = []
  class Raylib:
    BLACK, WHITE = 0, 1
    ffi = SimpleNamespace(buffer=lambda data, size: data)
    def __getattr__(self, name):
      def fn(*args):
        calls.append((name, args))
        if name == 'load_image_from_texture':
          return SimpleNamespace(width=1, height=1, data=b'1234')
        if name in ('Vector2', 'Rectangle'):
          return args
        return False
      return fn
  scope = {'rl': Raylib(), 'PC': False, 'RECORD': False, 'BURN_IN_MODE': False,
           'time': SimpleNamespace(monotonic=lambda: 0), 'queue': queue}
  cls = methods('application.py', 'GuiApplication', ['render', '_release_unused_render_texture'], scope)
  app = cls()
  app.__dict__.update(_profile_render_frames=0, _window_close_requested=False, _mouse=SimpleNamespace(get_events=list),
    _should_render=True, _render_texture=None, _record_enabled=False, _scale=1., _nav_stack_ticks=[], _nav_stack=[],
    _nav_stack_widgets_to_render=1, _show_fps=False, _show_touches=False, _grid_size=0, _record_frame_idx=2,
    _record_every_n=3, _ffmpeg_queue=queue.Queue(maxsize=1), _record_t0=0, _record_max_sec=60, _frame=0,
    _scaled_width=100, _scaled_height=50, _monitor_fps=lambda: None)
  return app, calls, scope


@pytest.mark.parametrize('keep', ['record', 'cli_record', 'scale', 'burn_in', 'none'])
def test_recording_target_released_only_when_unused(recording, keep):
  app, calls, scope = recording
  app._render_texture = target = SimpleNamespace(texture=9)
  app._record_enabled = keep == 'record'
  app._scale = .75 if keep == 'scale' else 1.
  scope['RECORD'] = keep == 'cli_record'
  scope['BURN_IN_MODE'] = keep == 'burn_in'
  app._release_unused_render_texture()
  if keep == 'none':
    assert app._render_texture is None
    assert calls == [('unload_render_texture', (target,))]
  else:
    assert app._render_texture is target
    assert calls == []


def test_start_recording_midframe_does_not_end_or_capture_an_unbegun_target(recording):
  app, calls, _ = recording
  frames = app.render()
  assert next(frames)
  app._record_enabled = True
  app._render_texture = SimpleNamespace(texture=9)
  app._window_close_requested = True
  assert list(frames) == []
  names = [c[0] for c in calls]
  assert 'begin_drawing' in names and 'end_drawing' in names
  assert 'end_texture_mode' not in names
  assert 'load_image_from_texture' not in names


def test_stop_recording_midframe_releases_after_matching_end(recording):
  app, calls, _ = recording
  app._record_enabled = True
  app._render_texture = SimpleNamespace(texture=9)
  frames = app.render()
  assert next(frames)
  app._record_enabled = False
  assert next(frames)
  names = [c[0] for c in calls]
  assert names.index('end_texture_mode') < names.index('unload_render_texture')
  assert app._render_texture is None
  assert 'load_image_from_texture' not in names
  app._window_close_requested = True
  assert list(frames) == []


@pytest.mark.parametrize('full', [False, True])
def test_encoder_backpressure_skips_gpu_readback(recording, full):
  app, calls, _ = recording
  app._record_enabled = True
  app._render_texture = SimpleNamespace(texture=9)
  if full:
    app._ffmpeg_queue.put_nowait(b'previous')
  frames = app.render()
  assert next(frames)
  app._window_close_requested = True
  assert list(frames) == []
  assert ('load_image_from_texture' in [c[0] for c in calls]) is not full
  assert app._ffmpeg_queue.get_nowait() == (b'previous' if full else b'1234')


def test_text_measurement_cache_bounds_and_freshness(monkeypatch):
  calls = []
  def measure(font, text, size, spacing):
    calls.append(text)
    return SimpleNamespace(x=len(text)*size, y=size)
  fake_rl = SimpleNamespace(Font=object, Vector2=object, measure_text_ex=measure)
  monkeypatch.setitem(sys.modules, 'pyray', fake_rl)
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.application', SimpleNamespace(FONT_SCALE=1.16, font_fallback=lambda f: f))
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.emoji', SimpleNamespace(find_emoji=lambda _: []))
  spec = importlib.util.spec_from_file_location('_measure_test', LIB/'text_measure.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  module._MAX_ENTRIES = 4
  font = SimpleNamespace(texture=SimpleNamespace(id=1), baseSize=32, glyphCount=100, glyphPadding=2)
  for i in range(8):
    module.measure_text_cached(font, str(i), 20)
  assert len(module._cache) == 4
  result = module.measure_text_cached(font, '7', 20)
  assert len(calls) == 8
  assert module.measure_text_cached(font, '7', 20) is result
  font.texture.id = 2
  assert module.measure_text_cached(font, '7', 20) is not result
  for _ in range(2):
    module.measure_text_cached(font, 'x'*513, 20)
  assert calls[-2:] == ['x'*513]*2
  assert len(module._cache) == 4


@pytest.mark.parametrize('native', [False, True])
def test_plain_text_hook_keeps_font_fallback_scale_and_spacing(monkeypatch, native):
  from openpilot.system.ui.lib import native_text
  original_calls, native_calls = [], []
  def original(*args):
    original_calls.append(args)
  def optimized(*args):
    native_calls.append(args)
    return native
  monkeypatch.setattr(native_text, 'try_plain_text', optimized)
  rl = SimpleNamespace(draw_text_ex=original)
  scope = {'rl': rl, 'FONT_SCALE': 1.16, 'font_fallback': lambda _: 'fallback'}
  cls = methods('application.py', 'GuiApplication', ['_patch_text_functions'], scope)
  app = cls()
  app._patch_text_functions()
  app._patch_text_functions()  # Reinitialization must not multiply the scale twice.
  rl.draw_text_ex('input_font', '한글', (10, 20), 40, 1.7, 'white')
  expected = ('fallback', '한글', (10, 20), 40*1.16, 1.7, 'white')
  assert native_calls == [(rl, *expected)]
  assert original_calls == ([] if native else [expected])


def test_gradient_uniform_reuse_invalidation_and_cleanup():
  calls = []
  rl = SimpleNamespace(
    ffi=SimpleNamespace(new=lambda _, v: [0.]*v if isinstance(v, int) else v),
    set_shader_value=lambda shader, name, value, kind: calls.append((name, tuple(value))),
    set_shader_value_v=lambda shader, name, value, kind, count: calls.append((name, tuple(value))),
    Vector2=lambda *xy: xy, WHITE=SimpleNamespace(r=255, g=255, b=255, a=255),
    unload_shader=lambda _: None,
  )
  scope = {'rl': rl, 'Any': object, 'MAX_GRADIENT_COLORS': 20}
  cls = methods('shader_polygon.py', 'ShaderState', ['__init__', 'cleanup'], scope)
  cls._instance = None
  state = cls()
  state.initialized = True
  state.shader = 1
  state.locations = {k: k for k in state.locations}
  tree = ast.parse((LIB/'shader_polygon.py').read_text(encoding='utf8'))
  func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_configure_shader_color')
  for arg in func.args.args:
    arg.annotation = None
  scope.update(cast=lambda _, v: v, Gradient=object, UNIFORM_INT=0, UNIFORM_FLOAT=1, UNIFORM_VEC2=2, UNIFORM_VEC4=4)
  exec(compile(ast.Module(body=[func], type_ignores=[]), 'shader_polygon.py', 'exec'), scope)
  configure = scope[func.name]
  color = SimpleNamespace(r=11, g=22, b=33, a=144)
  gradient = SimpleNamespace(colors=[color], stops=[0.], start=(0., 0.), end=(0., 1.))
  rect = SimpleNamespace(x=0, y=0, width=100, height=50)
  configure(state, None, gradient, rect)
  assert len(calls) == 6
  calls.clear()
  configure(state, None, gradient, rect)
  assert not calls
  color.a = 100  # Mutable colors must invalidate the stored value.
  rect.height = 60
  configure(state, None, gradient, rect)
  assert [c[0] for c in calls] == ['gradientColors', 'gradientEnd']
  calls.clear()
  configure(state, color, None, rect)
  assert [c[0] for c in calls] == ['useGradient', 'fillColor']
  calls.clear()
  configure(state, None, gradient, rect)
  assert [c[0] for c in calls] == ['useGradient']
  state.cleanup()
  assert not state.uniform_values


def test_shader_module_import_accepts_pyray_struct_factories(monkeypatch):
  # pyray exposes CFFI struct constructors as functions. Evaluating Color | None
  # at import time fails even though Optional[Color] works on the target binding.
  raylib = SimpleNamespace(Color=lambda *args: args, Rectangle=lambda *args: args,
    ShaderUniformDataType=SimpleNamespace(SHADER_UNIFORM_INT=0, SHADER_UNIFORM_FLOAT=1, SHADER_UNIFORM_VEC2=2, SHADER_UNIFORM_VEC4=4))
  monkeypatch.setitem(sys.modules, 'pyray', raylib)
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.application', SimpleNamespace(gui_app=None, GL_VERSION=''))
  name = '_shader_import_test'
  spec = importlib.util.spec_from_file_location(name, LIB/'shader_polygon.py')
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, name, module)
  spec.loader.exec_module(module)
  assert callable(module.draw_polygon)
