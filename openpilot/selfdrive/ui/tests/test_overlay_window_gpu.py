"""Desktop GPU regression for the nonzero window FBO used by PLATFORM_COMMA.

Run explicitly with CARROT_UI_GPU_TEST=1. No vehicle connection or UI process
is needed: a hidden desktop context reproduces the window's nested target.
"""
import ast
import importlib.util
import os
from pathlib import Path
import queue
import sys
import time
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(os.getenv('CARROT_UI_GPU_TEST') != '1', reason='requires an explicit desktop GPU context')
LIB = Path(__file__).parents[3] / 'system/ui/lib'


@pytest.mark.parametrize('width,height', ((2160, 1080), (536, 240)))
@pytest.mark.parametrize('recording', (False, True))
def test_camera_and_both_layers_survive_every_window_frame(monkeypatch, width, height, recording):
  import pyray as rl

  rl.set_config_flags(rl.ConfigFlags.FLAG_WINDOW_HIDDEN)
  rl.init_window(320, 180, 'overlay regression')
  rl.set_target_fps(0)
  real_begin, real_end = rl.begin_drawing, rl.end_drawing
  origin = rl.Vector2(0, 0)
  source = rl.Rectangle(0, 0, width, -height)
  rect = rl.Rectangle(0, 0, width, height)
  rotated = rl.load_render_texture(width, height)

  def comma_begin():
    real_begin()
    rl.begin_texture_mode(rotated)

  def comma_end():
    rl.end_texture_mode()
    rl.clear_background(rl.BLACK)
    rl.draw_texture_pro(rotated.texture, source, rl.Rectangle(0, 0, 320, 180), origin, 0, rl.WHITE)
    real_end()

  monkeypatch.setattr(rl, 'begin_drawing', comma_begin)
  monkeypatch.setattr(rl, 'end_drawing', comma_end)
  # Use the production frame loop without importing native messaging/Params.
  scope = {'rl': rl, 'PC': False, 'RECORD': False, 'BURN_IN_MODE': False, 'time': time, 'queue': queue}
  tree = ast.parse((LIB / 'application.py').read_text(encoding='utf8'))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GuiApplication')
  cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ('render', '_release_unused_render_texture', 'frame')]
  exec(compile(ast.Module(body=[cls], type_ignores=[]), 'application.py', 'exec'), scope)
  app = scope['GuiApplication']()
  app.__dict__.update(
    _profile_render_frames=0, _window_close_requested=False, _mouse=SimpleNamespace(get_events=list),
    _should_render=True, _render_texture=rl.load_render_texture(width, height) if recording else None,
    _record_enabled=recording, _scale=1., _nav_stack_ticks=[], _nav_stack=[], _nav_stack_widgets_to_render=1,
    _show_fps=False, _show_touches=False, _grid_size=0, _record_frame_idx=0, _record_every_n=1000,
    _ffmpeg_queue=queue.Queue(maxsize=1), _record_t0=time.monotonic(), _record_max_sec=600, _frame=0,
    width=width, height=height, _scaled_width=width, _scaled_height=height, _monitor_fps=lambda: None,
    _read_framebuffer=rl.rl_get_active_framebuffer, render_framebuffer=0)
  stub = ModuleType('openpilot.system.ui.lib.application')
  stub.gui_app = app
  monkeypatch.setitem(sys.modules, stub.__name__, stub)
  spec = importlib.util.spec_from_file_location('_gpu_overlay', LIB / 'overlay_cache.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  now = [100.]
  module.time = SimpleNamespace(monotonic=lambda: now[0])
  layers = [module.OverlayLayer(0), module.OverlayLayer(1)]
  paints = [0, 0]

  def model(_):
    paints[0] += 1
    rl.draw_rectangle(width // 10, height // 5, width // 5, height // 3, rl.RED)

  def hud(_):
    paints[1] += 1
    rl.draw_rectangle(width * 7 // 10, height // 5, width // 5, height // 3, rl.GREEN)

  def render(_):
    rl.draw_rectangle_rec(rect, rl.Color(20, 40, 70, 255))  # stand-in for the camera background
    urgent = app.frame in (7, 8, 17)  # bypass and re-enter caching, like warnings or touch
    layers[0].draw(rect, model, 'model', urgent=urgent)
    layers[1].draw(rect, hud, 'hud', urgent=urgent)

  app._nav_stack = [SimpleNamespace(render=render)]
  frames = app.render()
  try:
    for i in range(40):
      now[0] = 100 + i * .04
      assert next(frames)
      rl.rl_draw_render_batch_active()
      target = app._render_texture if recording else rotated
      assert app.render_framebuffer == target.id
      image = rl.load_image_from_texture(target.texture)
      try:
        pixels = np.frombuffer(rl.ffi.buffer(image.data, width * height * 4), dtype=np.uint8).reshape(height, width, 4)
        # Inspect both reused and repainted frames, with the background still visible.
        assert pixels[height * 2 // 3, width // 5, :3].tolist() == list(rl.RED[:3]), i
        assert pixels[height * 2 // 3, width * 4 // 5, :3].tolist() == list(rl.GREEN[:3]), i
        assert pixels[height // 5, width // 2, :3].tolist() == [20, 40, 70], i
      finally:
        rl.unload_image(image)
    assert all(20 <= count < 30 for count in paints)  # retain alternating work
    app._window_close_requested = True
    assert list(frames) == []
  finally:
    frames.close()
    for layer in layers:
      layer.close()
    if app._render_texture:
      rl.unload_render_texture(app._render_texture)
    rl.unload_render_texture(rotated)
    rl.close_window()
