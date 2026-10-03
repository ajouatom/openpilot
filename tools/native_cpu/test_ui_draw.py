"""Native ABI and exact primitive-stream comparisons without a graphics context."""
import ctypes as c
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.system.ui.lib import _draw_native as native
from openpilot.system.ui.lib import native_draw


class Point(c.Structure):
  _fields_ = [('x', c.c_float), ('y', c.c_float)]


class Color(c.Structure):
  _fields_ = [(name, c.c_ubyte) for name in ('r', 'g', 'b', 'a')]


LINE = c.CFUNCTYPE(None, Point, Point, c.c_float, Color)
STRIP = c.CFUNCTYPE(None, c.POINTER(Point), c.c_int, Color)


@pytest.mark.parametrize('count', [0, 1, 2, 3, 4, 7, 32, 66, 257])
def test_outline_matches_original_primitive_order(count):
  points = np.random.default_rng(count).uniform(-3000, 3000, (count, 2)).astype(np.float32)
  points.flags.writeable = False
  calls = []

  @LINE
  def draw(p, q, width, color):
    calls.append(((p.x, p.y), (q.x, q.y), width, (color.r, color.g, color.b, color.a)))

  native.outline(points, c.cast(draw, c.c_void_p).value, 1.25, 30, 60, 90, 120)
  expected = [(tuple(points[i]), tuple(points[(i+1) % count]), 1.25, (30, 60, 90, 120))
              for i in range(count)] if count >= 2 else []
  assert calls == expected


@pytest.mark.parametrize('count', [0, 1, 2, 3, 4, 7, 32, 66, 257])
def test_ribbon_matches_trimmed_interleaving_and_color(count):
  points = np.random.default_rng(count).uniform(-3000, 3000, (count, 2)).astype(np.float32)
  points.flags.writeable = False
  calls = []

  @STRIP
  def draw(ptr, n, color):
    calls.append(([(ptr[i].x, ptr[i].y) for i in range(n)], (color.r, color.g, color.b, color.a)))

  native.ribbon(points, c.cast(draw, c.c_void_p).value, 30, 60, 90, 120)
  n = count // 2 * 2
  expected = [tuple(points[j]) for i in range(n // 2) for j in (i, n-i-1)]
  assert calls == ([(expected, (30, 60, 90, 120))] if n else [])


@pytest.mark.parametrize('function', [native.outline, native.ribbon])
def test_reject_invalid_shape_before_calling_pointer(function):
  extra = (2.0,) if function is native.outline else ()
  with pytest.raises(ValueError):
    function(np.zeros((4, 3), np.float32), 1, *extra, 0, 0, 0, 255)
  with pytest.raises(ValueError):
    function(np.zeros((4, 2), np.float32), 0, *extra, 0, 0, 0, 255)


@pytest.mark.parametrize('unavailable', ['disabled', 'missing', 'incompatible'])
def test_python_fallback_when_native_unavailable(monkeypatch, unavailable):
  monkeypatch.setattr(native_draw, '_raylib', None)
  monkeypatch.setattr(native_draw, '_addresses', None)
  monkeypatch.setattr(native_draw, '_ENABLED', unavailable != 'disabled')
  monkeypatch.setattr(native_draw, '_draw_native', None if unavailable == 'missing' else native)
  fake_raylib = SimpleNamespace(ffi=SimpleNamespace(sizeof=lambda _: 99))
  points = np.ones((4, 2), np.float32)
  assert not native_draw.try_outline(fake_raylib, points, None, 2)
  assert not native_draw.try_ribbon(fake_raylib, points, None)


@pytest.mark.parametrize('color', [(10, 20, 30, 40), SimpleNamespace(r=10, g=20, b=30, a=40)])
def test_wrapper_accepts_raylib_constant_tuples_and_color_objects(monkeypatch, color):
  calls = []

  @LINE
  def line(_p, _q, _width, color):
    calls.append((color.r, color.g, color.b, color.a))

  @STRIP
  def strip(_ptr, _n, color):
    calls.append((color.r, color.g, color.b, color.a))

  monkeypatch.setattr(native_draw, '_functions', lambda _: (c.cast(line, c.c_void_p).value, c.cast(strip, c.c_void_p).value))
  points = np.ones((4, 2), np.float32)
  assert native_draw.try_outline(None, points.tolist(), color, 2)
  assert native_draw.try_ribbon(None, points, color)
  assert calls == [(10, 20, 30, 40)] * 5


@pytest.mark.parametrize('enabled', [False, True])
def test_c4_lead_rectangles_keep_visibility_color_and_edge_order(monkeypatch, enabled):
  path = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/ui/mici/onroad/model_renderer.py'
  tree = ast.parse(path.read_text(encoding='utf8'))
  method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_draw_lead_indicator')
  calls = []

  @LINE
  def draw(p, q, width, color):
    calls.append(((p.x, p.y), (q.x, q.y), width, (color.r, color.g, color.b, color.a)))

  functions = (c.cast(draw, c.c_void_p).value, 0) if enabled else None
  monkeypatch.setattr(native_draw, '_functions', lambda _: functions)
  raylib = SimpleNamespace(draw_line_ex=lambda p, q, w, color: calls.append((p, q, w, color)))
  scope = {'rl': raylib, 'native_draw': native_draw}
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
  rect = [(1., 2.), (20., 2.), (20., 12.), (1., 12.)]
  color = (31, 71, 121, 151)
  renderer = SimpleNamespace(_lead_vehicles=[SimpleNamespace(rect=[], color=color),
                                            SimpleNamespace(rect=rect, color=None), SimpleNamespace(rect=rect, color=color)])
  scope[method.name](renderer)
  assert calls == [(rect[i], rect[(i+1) % 4], 4., color) for i in range(4)]
