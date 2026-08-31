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


def test_lane_dash_segments_are_anchored_and_truncated(model_renderer_module):
  module = model_renderer_module
  line = np.array(
    [[0.0, 0.0, 0.0], [5.0, 0.5, 0.0], [10.0, 1.0, 0.0], [20.0, 2.0, 0.0]],
    dtype=np.float32,
  )

  segments = module.lane_dash_segments(line, 12.0)

  assert len(segments) == 2
  np.testing.assert_allclose(segments[0][[0, -1], 0], [0.0, module.LANE_DASH_LENGTH_M])
  np.testing.assert_allclose(segments[1][[0, -1], 0], [9.4, 12.0])
  np.testing.assert_allclose(segments[1][[0, -1], 1], [0.94, 1.2])
  assert all(np.all(np.diff(segment[:, 0]) > 0) for segment in segments)


def test_negative_lane_type_keeps_model_geometry_visible(model_renderer_module, monkeypatch):
  module = model_renderer_module
  renderer = object.__new__(module.ModelRenderer)
  renderer._carrot_show_lane_info = 1
  renderer._lane_line_probs = np.ones(4, dtype=np.float32)
  renderer._road_edge_stds = np.zeros(2, dtype=np.float32)
  renderer._lane_lines = [
    module.ModelPoints(raw_points=np.array([[0.0, i, 0.0], [20.0, i, 0.0]], dtype=np.float32))
    for i in range(4)
  ]
  renderer._road_edges = [module.ModelPoints(), module.ModelPoints()]
  renderer._rect = module.rl.Rectangle(0.0, 0.0, 2160.0, 1080.0)
  renderer._get_path_length_idx = lambda *_args: 1

  projected = []

  def project(line, *_args):
    projected.append(line)
    return np.array([[0.0, 1.0], [0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], dtype=np.float32)

  renderer._map_line_to_polygon = project
  renderer._draw_polygon_outline_carrot = lambda *_args: None
  draws = []
  monkeypatch.setattr(module, "draw_polygon_solid", lambda *args: draws.append(args))

  class LaneSubMaster:
    valid = {"modelV2": True, "carState": True}

    def __getitem__(self, key):
      assert key == "carState"
      return SimpleNamespace(leftLaneLine=-1, rightLaneLine=-1)

  renderer._draw_lane_lines_carrot(LaneSubMaster())

  assert len(projected) == 4
  assert all(actual is expected.raw_points for actual, expected in zip(projected, renderer._lane_lines, strict=True))
  assert len(draws) == 4


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


@pytest.mark.parametrize("y_shift", (-1.7, 1.7))
def test_blind_spot_barrier_vectorization_matches_scalar_reference(model_renderer_module, y_shift):
  module = model_renderer_module
  renderer = object.__new__(module.ModelRenderer)
  renderer._car_space_transform = np.array(
    [[0.0, 120.0, 960.0], [-18.0, 8.0, 720.0], [0.02, 0.0, 1.0]],
    dtype=np.float32,
  )
  renderer._clip_region = module.rl.Rectangle(-500.0, -500.0, 3000.0, 2000.0)
  model_position = np.array(
    [
      [-1.0, 0.0, 0.0],
      [0.0, 0.0, 0.0],
      [5.0, 0.2, 0.05],
      [10.0, -0.1, 0.10],
      [20.0, 0.4, 0.15],
      [30.0, -0.2, 0.20],
      [40.0, 0.1, 0.25],
      [50.0, 0.0, 0.30],
    ],
    dtype=np.float32,
  )

  max_idx = renderer._get_path_length_idx(model_position[:, 0], 40.0)
  upper_points = []
  lower_points = []
  for i in range(max_idx + 1):
    if model_position[i, 0] < 0:
      continue
    upper = renderer._map_to_screen(
      model_position[i, 0], model_position[i, 1] + y_shift, model_position[i, 2] + 1.15,
    )
    lower = renderer._map_to_screen(
      model_position[i, 0], model_position[i, 1] + y_shift, model_position[i, 2] + 0.6,
    )
    if upper is not None and lower is not None:
      if upper_points and upper[1] > upper_points[-1][1]:
        continue
      upper_points.append(upper)
      lower_points.insert(0, lower)
  expected = np.array(upper_points + lower_points, dtype=np.float32)

  actual = renderer._build_blind_spot_barrier_carrot(model_position, y_shift)

  np.testing.assert_array_equal(actual, expected)


def test_blind_spot_segment_vectorization_preserves_fill_outline_order(model_renderer_module, monkeypatch):
  module = model_renderer_module
  renderer = object.__new__(module.ModelRenderer)
  upper = np.array(
    [[100.0, 700.0], [110.0, 650.0], [120.0, 600.0], [130.0, 550.0], [140.0, 500.0], [150.0, 450.0]],
    dtype=np.float32,
  )
  lower = np.array(
    [[90.0, 710.0], [100.0, 660.0], [110.0, 610.0], [120.0, 560.0], [130.0, 510.0], [140.0, 460.0]],
    dtype=np.float32,
  )
  points = np.vstack((upper, lower[::-1]))
  color = module.rl.Color(255, 215, 0, 150)
  events = []
  monkeypatch.setattr(
    module,
    "draw_polygon_solid",
    lambda quad, fill_color: events.append(("fill", quad.copy(), fill_color)),
  )
  renderer._draw_polygon_outline_carrot = (
    lambda quad, outline_color, thickness: events.append(("outline", quad.copy(), outline_color, thickness))
  )

  expected_quads = []
  count = points.shape[0]
  half = count // 2
  for i in range(0, half - 2, 2):
    quad = np.array(
      [points[i], points[i + 1], points[count - i - 3], points[count - i - 2]],
      dtype=np.float32,
    )
    center = np.mean(quad, axis=0)
    angles = np.arctan2(quad[:, 1] - center[1], quad[:, 0] - center[0])
    expected_quads.append(quad[np.argsort(angles)])

  renderer._draw_blind_spot_segments_carrot(points, color)

  assert [event[0] for event in events] == ["fill", "outline", "fill", "outline"]
  for index, expected in enumerate(expected_quads):
    fill = events[index * 2]
    outline = events[index * 2 + 1]
    np.testing.assert_array_equal(fill[1], expected)
    np.testing.assert_array_equal(outline[1], expected)
    assert fill[2] == color
    assert outline[2] == module.rl.WHITE
    assert outline[3] == 2.0


def test_blind_spot_invalid_input_skips_draw(model_renderer_module):
  module = model_renderer_module
  renderer = object.__new__(module.ModelRenderer)
  renderer._carrot_lane_barrier_vertices = [np.ones((4, 2), dtype=np.float32), np.ones((4, 2), dtype=np.float32)]
  car_state = SimpleNamespace(leftBlindspot=True, rightBlindspot=False, vEgo=10.0)
  radar_state = SimpleNamespace(
    leadLeft=SimpleNamespace(status=False, dRel=100.0),
    leadRight=SimpleNamespace(status=False, dRel=100.0),
  )
  model = SimpleNamespace(
    meta=SimpleNamespace(
      laneChangeState=module.LaneChangeState.off,
      laneChangeDirection="none",
    ),
  )

  class BlindSpotSubMaster(dict):
    def __init__(self):
      super().__init__(modelV2=model, carState=car_state, radarState=radar_state)
      self.valid = dict.fromkeys(self, True)

  sm = BlindSpotSubMaster()
  draws = []
  renderer._update_blind_spot_barriers_carrot = lambda *_args, **_kwargs: None
  renderer._draw_blind_spot_segments_carrot = lambda *_args: draws.append("left")
  renderer._draw_blind_spot_carrot(sm)
  assert draws == ["left"]

  sm.valid["radarState"] = False
  renderer._draw_blind_spot_carrot(sm)
  assert draws == ["left"]
