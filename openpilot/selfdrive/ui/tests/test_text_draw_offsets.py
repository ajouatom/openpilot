import importlib.util
import math
import sys
import types
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest


TEXT_DRAW_PATH = Path(__file__).parents[3] / "system" / "ui" / "lib" / "text_draw.py"


@pytest.fixture
def text_draw_module(monkeypatch):
  @dataclass(frozen=True)
  class Color:
    r: int
    g: int
    b: int
    a: int

  @dataclass
  class Vector2:
    x: float
    y: float

  draw_calls = []
  raylib = types.ModuleType("pyray")
  raylib.Color = Color
  raylib.Vector2 = Vector2
  raylib.BLACK = Color(0, 0, 0, 255)
  raylib.rl = SimpleNamespace(DrawTextEx=None)
  raylib.draw_text_ex = lambda font, text, position, font_size, spacing, color: draw_calls.append(
    (font, text, SimpleNamespace(x=position.x, y=position.y), font_size, spacing, color),
  )
  monkeypatch.setitem(sys.modules, "pyray", raylib)
  monkeypatch.setitem(
    sys.modules,
    "openpilot.system.ui.lib.application",
    SimpleNamespace(
      FONT_SCALE=1.242,
      gui_app=SimpleNamespace(font=lambda weight: ("font", weight)),
      FontWeight=SimpleNamespace(DISPLAY=1),
      font_fallback=lambda font: font,
    ),
  )
  monkeypatch.setitem(
    sys.modules,
    "openpilot.system.ui.lib.text_measure",
    SimpleNamespace(measure_text_cached=lambda font, text, size: Vector2(len(text) * size, size)),
  )

  module_name = f"_test_text_draw_{id(draw_calls)}"
  spec = importlib.util.spec_from_file_location(module_name, TEXT_DRAW_PATH)
  assert spec is not None and spec.loader is not None
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, module_name, module)
  spec.loader.exec_module(module)
  return module, draw_calls


def test_outline_offsets_are_the_eight_legacy_directions(text_draw_module):
  module, _ = text_draw_module
  expected = tuple(
    (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
    for deg in range(0, 360, 45)
  )

  assert module._OUTLINE_UNIT_OFFSETS == expected


@pytest.mark.parametrize("border_width", [0.5, 1.0, 3.0, 9.0])
def test_text_outline_positions_match_legacy_geometry(text_draw_module, monkeypatch, border_width):
  module, draw_calls = text_draw_module
  base_x, base_y = 100.0, 200.0
  shadow_offset = 4.0
  main_color = module.rl.Color(1, 2, 3, 4)
  border_color = module.rl.Color(5, 6, 7, 8)
  shadow_color = module.rl.Color(9, 10, 11, 12)
  monkeypatch.setattr(module, "get_text_draw_pos", lambda *args, **kwargs: (base_x, base_y, None))

  module.draw_text_ui_style(
    "speed",
    0,
    0,
    50,
    main_color,
    font="font",
    border_width=border_width,
    shadow_offset=shadow_offset,
    border_color=border_color,
    shadow_color=shadow_color,
  )

  assert len(draw_calls) == 10
  assert all(call[0] == "font" for call in draw_calls)
  assert all(call[1] == "speed" for call in draw_calls)
  assert all(call[3] == 50.0 and type(call[3]) is float for call in draw_calls)
  assert all(call[4] == 0 for call in draw_calls)
  legacy_offsets = tuple(
    (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
    for deg in range(0, 360, 45)
  )
  for call, (unit_x, unit_y) in zip(draw_calls[:8], legacy_offsets, strict=True):
    assert (call[2].x, call[2].y) == (
      base_x + border_width * unit_x,
      base_y + border_width * unit_y,
    )
    assert call[5] == border_color

  assert (draw_calls[8][2].x, draw_calls[8][2].y) == (base_x + shadow_offset, base_y + shadow_offset)
  assert draw_calls[8][5] == shadow_color
  assert (draw_calls[9][2].x, draw_calls[9][2].y) == (base_x, base_y)
  assert draw_calls[9][5] == main_color


def test_zero_border_and_shadow_draw_only_main_text(text_draw_module, monkeypatch):
  module, draw_calls = text_draw_module
  main_color = module.rl.Color(1, 2, 3, 4)
  monkeypatch.setattr(module, "get_text_draw_pos", lambda *args, **kwargs: (10.0, 20.0, None))

  module.draw_text_ui_style("speed", 0, 0, 50, main_color, font="font", border_width=0, shadow_offset=0)

  assert len(draw_calls) == 1
  assert draw_calls[0][0] == "font"
  assert draw_calls[0][1] == "speed"
  assert draw_calls[0][3] == 50.0 and type(draw_calls[0][3]) is float
  assert draw_calls[0][4] == 0
  assert (draw_calls[0][2].x, draw_calls[0][2].y) == (10.0, 20.0)
  assert draw_calls[0][5] == main_color


def test_styled_text_reuses_one_draw_position(text_draw_module, monkeypatch):
  module, draw_calls = text_draw_module
  monkeypatch.setattr(module, "get_text_draw_pos", lambda *args, **kwargs: (10.0, 20.0, None))
  vector_constructions = []

  class TrackingVector2:
    def __init__(self, x, y):
      vector_constructions.append((x, y))
      self.x = x
      self.y = y

  monkeypatch.setattr(module.rl, "Vector2", TrackingVector2)

  module.draw_text_ui_style(
    "speed",
    0,
    0,
    50,
    module.rl.Color(1, 2, 3, 4),
    font="font",
    border_width=3,
    shadow_offset=4,
  )

  assert len(draw_calls) == 10
  assert vector_constructions == [(10.0, 20.0)]


def test_draw_call_does_not_recompute_trigonometry(text_draw_module, monkeypatch):
  module, draw_calls = text_draw_module
  monkeypatch.setattr(module, "get_text_draw_pos", lambda *args, **kwargs: (10.0, 20.0, None))

  def fail(*args, **kwargs):
    pytest.fail("trigonometry was recomputed during draw")

  monkeypatch.setattr(module.math, "cos", fail)
  monkeypatch.setattr(module.math, "sin", fail)
  monkeypatch.setattr(module.math, "radians", fail)

  module.draw_text_ui_style(
    "speed",
    0,
    0,
    50,
    module.rl.Color(1, 2, 3, 4),
    font="font",
    border_width=3,
    shadow_offset=0,
  )

  assert len(draw_calls) == 9


@pytest.mark.parametrize(("text_value", "scale"), [("speed", 1.16), ("속도", 1.242)])
def test_raw_draw_path_encodes_once_and_preserves_scaled_geometry(
  text_draw_module, monkeypatch, text_value, scale,
):
  module, wrapper_calls = text_draw_module
  raw_calls = []
  fallback_calls = []
  encode_calls = []
  base_x, base_y = 100.0, 200.0

  class TrackingText(str):
    def encode(self, encoding="utf-8", errors="strict"):
      encode_calls.append((encoding, errors))
      return super().encode(encoding, errors)

  def raw_draw_text_ex(font, text, position, font_size, spacing, color):
    raw_calls.append(
      (font, text, id(text), SimpleNamespace(x=position.x, y=position.y), font_size, spacing, color, id(position)),
    )

  def fallback(font):
    fallback_calls.append(font)
    return "fallback-font"

  monkeypatch.setattr(module, "_RAW_DRAW_TEXT_EX", raw_draw_text_ex)
  monkeypatch.setattr(module, "FONT_SCALE", scale)
  monkeypatch.setattr(module, "font_fallback", fallback)
  monkeypatch.setattr(module, "get_text_draw_pos", lambda *args, **kwargs: (base_x, base_y, None))

  text = TrackingText(text_value)
  main_color = module.rl.Color(1, 2, 3, 4)
  border_color = module.rl.Color(5, 6, 7, 8)
  shadow_color = module.rl.Color(9, 10, 11, 12)
  module.draw_text_ui_style(
    text,
    0,
    0,
    50,
    main_color,
    font="original-font",
    border_width=1.0,
    shadow_offset=4.0,
    border_color=border_color,
    shadow_color=shadow_color,
  )

  assert wrapper_calls == []
  assert encode_calls == [("utf-8", "strict")]
  assert fallback_calls == ["original-font"]
  assert len(raw_calls) == 10
  assert all(call[0] == "fallback-font" for call in raw_calls)
  assert all(call[1] == text_value.encode() for call in raw_calls)
  assert len({call[2] for call in raw_calls}) == 1
  assert len({call[7] for call in raw_calls}) == 1
  assert all(call[4] == 50.0 * scale for call in raw_calls)
  assert all(call[5] == 0 for call in raw_calls)

  for call, (unit_x, unit_y) in zip(raw_calls[:8], module._OUTLINE_UNIT_OFFSETS, strict=True):
    assert (call[3].x, call[3].y) == (base_x + unit_x, base_y + unit_y)
    assert call[6] == border_color

  assert (raw_calls[8][3].x, raw_calls[8][3].y) == (base_x + 4.0, base_y + 4.0)
  assert raw_calls[8][6] == shadow_color
  assert (raw_calls[9][3].x, raw_calls[9][3].y) == (base_x, base_y)
  assert raw_calls[9][6] == main_color


def test_missing_raw_symbol_keeps_pyray_wrapper_path(text_draw_module, monkeypatch):
  module, draw_calls = text_draw_module
  monkeypatch.setattr(module, "_RAW_DRAW_TEXT_EX", None)
  monkeypatch.setattr(module, "font_fallback", lambda _: pytest.fail("wrapper path owns font fallback"))
  monkeypatch.setattr(module, "get_text_draw_pos", lambda *args, **kwargs: (10.0, 20.0, None))

  module.draw_text_ui_style(
    "speed",
    0,
    0,
    50,
    module.rl.Color(1, 2, 3, 4),
    font="font",
    border_width=0,
    shadow_offset=0,
  )

  assert len(draw_calls) == 1
  assert draw_calls[0][0] == "font"
  assert draw_calls[0][1] == "speed"
  assert draw_calls[0][3] == 50.0


def test_empty_text_skips_both_draw_paths(text_draw_module, monkeypatch):
  module, wrapper_calls = text_draw_module
  encode_calls = []

  class EmptyTrackingText(str):
    def encode(self, encoding="utf-8", errors="strict"):
      encode_calls.append((encoding, errors))
      return super().encode(encoding, errors)

  monkeypatch.setattr(module, "_RAW_DRAW_TEXT_EX", lambda *args: pytest.fail("empty text must not draw"))
  monkeypatch.setattr(module, "font_fallback", lambda _: pytest.fail("empty text must not select a font"))

  module.draw_text_ui_style(EmptyTrackingText(""), 0, 0, 50, module.rl.Color(1, 2, 3, 4))

  assert encode_calls == []
  assert wrapper_calls == []
