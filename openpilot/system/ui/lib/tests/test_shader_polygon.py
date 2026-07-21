import importlib.util
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SHADER_POLYGON_PATH = Path(__file__).parents[1] / "shader_polygon.py"


@pytest.fixture
def shader_polygon_module(monkeypatch):
  @dataclass(frozen=True)
  class Color:
    r: int
    g: int
    b: int
    a: int

  @dataclass(frozen=True)
  class Rectangle:
    x: float
    y: float
    width: float
    height: float

  raylib = types.ModuleType("pyray")
  raylib.Color = Color
  raylib.Rectangle = Rectangle
  raylib.WHITE = Color(255, 255, 255, 255)
  raylib.ShaderUniformDataType = SimpleNamespace(
    SHADER_UNIFORM_INT=0,
    SHADER_UNIFORM_FLOAT=1,
    SHADER_UNIFORM_VEC2=2,
    SHADER_UNIFORM_VEC4=3,
  )
  raylib.ffi = SimpleNamespace(new=lambda *_args, **_kwargs: None)

  monkeypatch.setitem(sys.modules, "pyray", raylib)
  monkeypatch.setitem(
    sys.modules,
    "openpilot.system.ui.lib.application",
    SimpleNamespace(gui_app=SimpleNamespace(width=2160, height=1080), GL_VERSION="#version 300 es\n"),
  )

  module_name = f"_test_shader_polygon_{id(raylib)}"
  spec = importlib.util.spec_from_file_location(module_name, SHADER_POLYGON_PATH)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, module_name, module)
  spec.loader.exec_module(module)
  return module


def test_solid_polygon_skips_custom_fill_shader(shader_polygon_module, monkeypatch):
  module = shader_polygon_module
  color = module.rl.Color(10, 20, 30, 120)
  points = np.array([[1.0, 10.0], [2.0, 5.0], [8.0, 5.0], [9.0, 10.0]], dtype=np.float32)
  calls = []

  monkeypatch.setattr(module.rl, "draw_triangle_strip", lambda *args: calls.append(args), raising=False)
  monkeypatch.setattr(
    module.ShaderState,
    "get_instance",
    classmethod(lambda cls: pytest.fail("solid fills must not initialize the gradient shader")),
  )

  module.draw_polygon_solid(points, color)

  assert len(calls) == 1
  strip, count, submitted_color = calls[0]
  assert strip == module.triangulate(points)
  assert count == len(strip)
  assert submitted_color is color


def test_gradient_polygon_keeps_custom_shader_path(shader_polygon_module, monkeypatch):
  module = shader_polygon_module
  rect = module.rl.Rectangle(0.0, 0.0, 2160.0, 1080.0)
  points = np.array([[1.0, 10.0], [2.0, 5.0], [8.0, 5.0], [9.0, 10.0]], dtype=np.float32)
  gradient = module.Gradient(
    start=(0.0, 1.0),
    end=(0.0, 0.0),
    colors=[module.rl.Color(255, 0, 0, 120), module.rl.Color(0, 255, 0, 120)],
    stops=[0.0, 1.0],
  )
  calls = []
  state = SimpleNamespace(shader="gradient-shader", initialize=lambda: calls.append("initialize"))

  monkeypatch.setattr(module.ShaderState, "get_instance", classmethod(lambda cls: state))
  monkeypatch.setattr(module, "_configure_shader_color", lambda *args: calls.append("configure"))
  monkeypatch.setattr(module.rl, "begin_shader_mode", lambda shader: calls.append(("begin", shader)), raising=False)
  monkeypatch.setattr(module.rl, "draw_triangle_strip", lambda *args: calls.append(("draw", args)), raising=False)
  monkeypatch.setattr(module.rl, "end_shader_mode", lambda: calls.append("end"), raising=False)

  module.draw_polygon(rect, points, gradient=gradient)

  assert calls[0:3] == ["initialize", "configure", ("begin", "gradient-shader")]
  assert calls[3][0] == "draw"
  assert calls[3][1][0] == module.triangulate(points)
  assert calls[3][1][2] is module.rl.WHITE
  assert calls[4] == "end"


@pytest.mark.parametrize(
  "color, gradient",
  [
    (None, None),
    ("color", "gradient"),
  ],
)
def test_polygon_requires_exactly_one_fill(shader_polygon_module, color, gradient):
  module = shader_polygon_module
  rect = module.rl.Rectangle(0.0, 0.0, 2160.0, 1080.0)
  points = np.ones((4, 2), dtype=np.float32)

  with pytest.raises(AssertionError, match="Either color or gradient"):
    module.draw_polygon(rect, points, color=color, gradient=gradient)
