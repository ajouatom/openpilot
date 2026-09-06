"""Exercise text rendering without a graphics context or the removed raygui API."""
import ast
import math
from collections.abc import Callable
from enum import IntEnum
from itertools import zip_longest
from pathlib import Path
from types import SimpleNamespace
from typing import Union

import pytest


UI_DIR = Path(__file__).resolve().parents[2]


def load_definitions(path, namespace, names=None):
  tree = ast.parse(path.read_text(encoding="utf-8"))
  tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))
               and (names is None or node.name in names)]
  exec(compile(tree, str(path), "exec"), namespace)


@pytest.fixture
def renderer():
  def vector(x=0, y=0):
    return SimpleNamespace(x=x, y=y)

  def rectangle(x=0, y=0, width=0, height=0):
    return SimpleNamespace(x=x, y=y, width=width, height=height)

  class Widget:
    def __init__(self):
      self._rect = rectangle()

    def render(self, rect):
      self._rect = rect
      self._render(rect)

  # Deliberately omit all Gui* enums and gui_* functions, as in comma-deps-raylib.
  draw_calls = []
  rl = SimpleNamespace(Vector2=vector, Rectangle=rectangle, Texture=object, Color=object, Font=object,
                       draw_calls=draw_calls, draw_text_ex=lambda *args: draw_calls.append(args),
                       begin_scissor_mode=lambda *args: None, end_scissor_mode=lambda: None)
  font = SimpleNamespace(texture=SimpleNamespace(id=1))
  namespace = {
    "IntEnum": IntEnum, "math": math, "Callable": Callable, "Union": Union, "zip_longest": zip_longest,
    "rl": rl, "Widget": Widget, "FontWeight": SimpleNamespace(NORMAL=0),
    "DEFAULT_TEXT_SIZE": 20, "DEFAULT_TEXT_COLOR": object(), "FONT_SCALE": 1, "ICON_PADDING": 15,
    "gui_app": SimpleNamespace(font=lambda weight: font), "font_fallback": lambda font: font,
    "measure_text_cached": lambda font, text, size, spacing=0: vector(len(text) * 10, size),
    "find_emoji": lambda text: [], "_cache": {},
  }
  load_definitions(UI_DIR / "lib/application.py", namespace, {"TextAlignment", "TextAlignmentVertical"})
  load_definitions(UI_DIR / "lib/wrap_text.py", namespace)
  # Evaluate defaults on the real label classes/functions to catch import-time API failures.
  load_definitions(UI_DIR / "widgets/label.py", namespace)
  return SimpleNamespace(**namespace)


@pytest.mark.parametrize("horizontal,x", [(0, 10), (1, 50), (2, 90)])
@pytest.mark.parametrize("vertical,y", [(0, 20), (1, 60), (2, 100)])
def test_label_alignment_without_raygui(renderer, horizontal, x, vertical, y):
  renderer.gui_label(renderer.rl.Rectangle(10, 20, 100, 100), "ab", font_size=20,
                     alignment=horizontal, alignment_vertical=vertical)
  position = renderer.rl.draw_calls[-1][2]
  assert (position.x, position.y) == (x, y)


def test_text_box_wraps_and_preserves_line_spacing(renderer):
  renderer.gui_text_box(renderer.rl.Rectangle(10, 20, 50, 100), "one two three", font_size=20, line_scale=1.5)
  calls = renderer.rl.draw_calls
  assert [call[1] for call in calls] == ["one", "two", "three"]
  assert [(call[2].x, call[2].y) for call in calls] == [(10, 20), (10, 50), (10, 80)]


def test_text_box_bottom_right_alignment_and_color(renderer):
  color = object()
  renderer.gui_text_box(renderer.rl.Rectangle(10, 20, 100, 100), "ab\nc", font_size=20,
                        color=color, alignment=2, alignment_vertical=2)
  calls = renderer.rl.draw_calls
  assert [(call[2].x, call[2].y) for call in calls] == [(90, 80), (100, 100)]
  assert all(call[5] is color for call in calls)


def test_text_box_empty_text(renderer):
  renderer.gui_text_box(renderer.rl.Rectangle(0, 0, 100, 100), "")
  assert not renderer.rl.draw_calls


def test_ui_does_not_reference_removed_raygui_api():
  for root in (UI_DIR, UI_DIR.parents[1] / "selfdrive/ui"):
    for path in root.rglob("*.py"):
      tree = ast.parse(path.read_text(encoding="utf-8"))
      for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "rl":
          assert not node.attr.startswith(("Gui", "gui_")), f"{path}:{node.lineno}: {node.attr}"
