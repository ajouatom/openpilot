import importlib.util
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


MODEL_RENDERER_PATH = Path(__file__).parents[1] / "onroad" / "model_renderer.py"


@pytest.fixture
def model_renderer_module(monkeypatch):
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

  @dataclass(frozen=True)
  class Vector2:
    x: float
    y: float

  class Font:
    pass

  raylib = types.ModuleType("pyray")
  raylib.Color = Color
  raylib.Rectangle = Rectangle
  raylib.Vector2 = Vector2
  raylib.Font = Font
  raylib.WHITE = Color(255, 255, 255, 255)
  raylib.RL_QUADS = 7
  raylib.get_time = lambda: 0.0
  for name in (
    "draw_circle",
    "draw_line_ex",
    "draw_rectangle_rounded",
    "draw_rectangle_rounded_lines_ex",
    "draw_triangle_fan",
    "rl_begin",
    "rl_color4ub",
    "rl_disable_texture",
    "rl_end",
    "rl_vertex2f",
  ):
    setattr(raylib, name, lambda *args, **kwargs: None)

  class FirstOrderFilter:
    def __init__(self, *args, **kwargs):
      pass

  class Params:
    def get(self, _key):
      return None

  class Widget:
    def __init__(self):
      pass

  @dataclass
  class Gradient:
    start: tuple[float, float]
    end: tuple[float, float]
    colors: list
    stops: list[float]

  lane_change_state = SimpleNamespace(off=0, preLaneChange=1)
  cereal = types.ModuleType("openpilot.cereal")
  cereal.messaging = SimpleNamespace(log_from_bytes=lambda *_args: None)
  cereal.car = SimpleNamespace(CarParams=object)
  cereal.log = SimpleNamespace(LaneChangeState=lane_change_state)

  stubs = {
    "pyray": raylib,
    "openpilot.cereal": cereal,
    "openpilot.common.filter_simple": SimpleNamespace(FirstOrderFilter=FirstOrderFilter),
    "openpilot.common.params": SimpleNamespace(Params=Params),
    "openpilot.selfdrive.locationd.calibrationd": SimpleNamespace(HEIGHT_INIT=(1.22,)),
    "openpilot.selfdrive.ui.ui_state": SimpleNamespace(ui_state=SimpleNamespace()),
    "openpilot.system.ui.lib.application": SimpleNamespace(
      gui_app=SimpleNamespace(target_fps=20, font=lambda _weight: Font()),
      FontWeight=SimpleNamespace(DISPLAY=0),
    ),
    "openpilot.system.ui.lib.text_draw": SimpleNamespace(draw_text_ui_style=lambda *args, **kwargs: None),
    "openpilot.system.ui.lib.shader_polygon": SimpleNamespace(
      draw_polygon=lambda *args, **kwargs: None,
      draw_polygon_solid=lambda *args, **kwargs: None,
      Gradient=Gradient,
    ),
    "openpilot.system.ui.widgets": SimpleNamespace(Widget=Widget),
  }
  for name, stub in stubs.items():
    monkeypatch.setitem(sys.modules, name, stub)

  module_name = f"_test_carrot_model_renderer_lane_{id(raylib)}"
  spec = importlib.util.spec_from_file_location(module_name, MODEL_RENDERER_PATH)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, module_name, module)
  spec.loader.exec_module(module)
  return module


def test_lane_draw_skips_zero_alpha_projection(model_renderer_module, monkeypatch):
  module = model_renderer_module
  renderer = object.__new__(module.ModelRenderer)
  renderer._carrot_show_lane_info = 1
  renderer._lane_line_probs = np.array([0.3, 0.31, np.nan, 0.9], dtype=np.float32)
  renderer._road_edge_stds = np.zeros(2, dtype=np.float32)
  renderer._lane_lines = [
    module.ModelPoints(raw_points=np.array([[0.0, i, 0.0], [20.0, i, 0.0]], dtype=np.float32))
    for i in range(4)
  ]
  renderer._road_edges = [module.ModelPoints(), module.ModelPoints()]
  renderer._rect = module.rl.Rectangle(0.0, 0.0, 2160.0, 1080.0)
  renderer._get_path_length_idx = lambda *_args: 1

  projected = []

  def project(line, *args):
    projected.append((line, args))
    return np.array([[0.0, 1.0], [0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], dtype=np.float32)

  renderer._map_line_to_polygon = project
  outlines = []
  renderer._draw_polygon_outline_carrot = lambda *args: outlines.append(args)
  draws = []
  monkeypatch.setattr(module, "draw_polygon_solid", lambda *args: draws.append(args))

  class LaneSubMaster:
    valid = {"modelV2": True, "carState": True}

    def __getitem__(self, key):
      assert key == "carState"
      return SimpleNamespace(leftLaneLine=24, rightLaneLine=0)

  renderer._draw_lane_lines_carrot(LaneSubMaster())

  actual_lines = [entry[0] for entry in projected]
  expected_lines = [
    renderer._lane_lines[1].raw_points,
    renderer._lane_lines[1].raw_points,
    renderer._lane_lines[3].raw_points,
  ]
  assert all(actual is expected for actual, expected in zip(actual_lines, expected_lines, strict=True))
  assert projected[1][1][-2:] == (True, -0.3)
  assert len(draws) == 3
  assert len(outlines) == 2
  assert all(draw[1].a == 220 for draw in draws)

  projected.clear()
  draws.clear()
  outlines.clear()
  renderer._lane_line_probs = np.array([0.31, 0.3, 0.0, 0.0], dtype=np.float32)

  renderer._draw_lane_lines_carrot(LaneSubMaster())

  assert len(projected) == 1
  assert projected[0][0] is renderer._lane_lines[0].raw_points
  assert len(draws) == 1
  assert outlines == []

  projected.clear()
  draws.clear()
  renderer._lane_line_probs = np.array([0.3, 0.0, np.nan, -1.0], dtype=np.float32)

  renderer._draw_lane_lines_carrot(LaneSubMaster())

  assert projected == []
  assert draws == []


def test_mode9_complex_path_preserves_segment_fill_outline_order(model_renderer_module, monkeypatch):
  module = model_renderer_module
  renderer = object.__new__(module.ModelRenderer)
  renderer._path = module.ModelPoints(
    projected_points=np.array([[float(i), float(i) + 0.25] for i in range(12)], dtype=np.float32),
  )
  renderer._carrot_colors = [module.rl.Color(i, i + 1, i + 2, 120) for i in range(10)]
  events = []

  monkeypatch.setattr(
    module,
    "draw_polygon_solid",
    lambda points, color: events.append(("fill", points.copy(), color)),
  )
  renderer._draw_polygon_outline_carrot = (
    lambda points, color, thickness: events.append(("outline", points.copy(), color, thickness))
  )

  renderer._draw_complex_path_carrot(color_idx=20, brake_valid=False)

  assert [event[0] for event in events] == ["fill", "fill", "outline", "fill", "fill", "outline"]
  for offset in (0, 3):
    left_fill = events[offset][1]
    right_fill = events[offset + 1][1]
    outline = events[offset + 2][1]
    np.testing.assert_array_equal(left_fill, outline[[0, 1, 2, 5]])
    np.testing.assert_array_equal(right_fill, outline[[5, 2, 3, 4]])
    assert events[offset][2] is renderer._carrot_colors[0]
    assert events[offset + 1][2] is renderer._carrot_colors[0]
    assert events[offset + 2][2] == module.rl.Color(255, 255, 255, 255)
    assert events[offset + 2][3] == 2.0

  events.clear()
  renderer._draw_complex_path_carrot(color_idx=5, brake_valid=False)
  assert [event[0] for event in events] == ["fill", "fill", "fill", "fill"]
