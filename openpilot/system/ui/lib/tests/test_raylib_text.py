"""Exercise text rendering without a graphics context or the removed raygui API."""
import ast
import math
import re
from collections import OrderedDict
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
  def draw_text(font, text, position, *args):
    draw_calls.append((font, text, vector(position.x, position.y), *args))
  rl = SimpleNamespace(Vector2=vector, Rectangle=rectangle, Texture=object, Color=object, Font=object,
                       draw_calls=draw_calls, draw_text_ex=draw_text,
                       begin_scissor_mode=lambda *args: None, end_scissor_mode=lambda: None)
  font = SimpleNamespace(texture=SimpleNamespace(id=1))
  namespace = {
    "IntEnum": IntEnum, "math": math, "Callable": Callable, "Union": Union, "zip_longest": zip_longest,
    "rl": rl, "Widget": Widget, "FontWeight": SimpleNamespace(NORMAL=0),
    "DEFAULT_TEXT_SIZE": 20, "DEFAULT_TEXT_COLOR": object(), "FONT_SCALE": 1, "ICON_PADDING": 15,
    "gui_app": SimpleNamespace(font=lambda weight: font), "font_fallback": lambda font, text=None: font,
    "measure_text_cached": lambda font, text, size, spacing=0: vector(len(text) * 10, size),
    "find_emoji": lambda text: [], "_cache": {},
  }
  load_definitions(UI_DIR / "lib/application.py", namespace, {"TextAlignment", "TextAlignmentVertical"})
  load_definitions(UI_DIR / "lib/wrap_text.py", namespace)
  # Evaluate defaults on the real label classes/functions to catch import-time API failures.
  load_definitions(UI_DIR / "widgets/label.py", namespace)
  return SimpleNamespace(**namespace, namespace=namespace)


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


@pytest.fixture
def korean_renderer(renderer):
  """Run real fallback, measurement, wrapping and labels with distinct font widths."""
  scope = renderer.namespace
  weights = scope["FontWeight"]
  weights.DISPLAY, weights.UNIFONT = 1, 2
  normal = SimpleNamespace(texture=SimpleNamespace(id=1), advance=10, baseSize=20, glyphCount=95, glyphPadding=0)
  display = SimpleNamespace(texture=SimpleNamespace(id=2), advance=20, baseSize=20, glyphCount=12000, glyphPadding=0)
  app = scope["gui_app"]
  app.font = lambda weight: display if weight == weights.DISPLAY else normal
  app.has_font = lambda weight: True
  language = SimpleNamespace(value="en")
  scope.update(re=re, multilang=SimpleNamespace(requires_unifont=lambda: language.value == "ko"))
  tree = ast.parse((UI_DIR / "lib/application.py").read_text(encoding="utf-8"))
  pattern = next(n for n in tree.body if isinstance(n, ast.Assign) and
                 any(isinstance(t, ast.Name) and t.id == "_KOREAN_TEXT_RE" for t in n.targets))
  exec(compile(ast.Module(body=[pattern], type_ignores=[]), "application.py", "exec"), scope)
  load_definitions(UI_DIR / "lib/application.py", scope, {"font_fallback"})

  measurements = []
  def measure(font, text, size, spacing):
    measurements.append((font, text))
    return scope["rl"].Vector2(len(text) * font.advance, size)
  scope["rl"].measure_text_ex = measure
  measure_scope = {**scope, "OrderedDict": OrderedDict, "_cache": OrderedDict(), "_MAX_ENTRIES": 2048, "_MAX_TEXT_LENGTH": 512}
  load_definitions(UI_DIR / "lib/text_measure.py", measure_scope)
  scope["measure_text_cached"] = measure_scope["measure_text_cached"]
  return SimpleNamespace(**scope, namespace=scope, normal=normal, display=display, language=language, measurements=measurements)


@pytest.mark.parametrize("text", ["한글", "AB 한글", "ㄱㅏ", "가", "힣", "한글".encode(), b"\xff" + "한글".encode()])
def test_english_locale_selects_korean_font(korean_renderer, text):
  r = korean_renderer
  assert r.font_fallback(r.normal, text) is r.display


@pytest.mark.parametrize("text", [None, "", "speed 100", "35° / € £ ¥", "français – ñü", "⚠️ 🛑", b"speed", b"\xff"])
def test_latin_symbols_and_emoji_keep_requested_font(korean_renderer, text):
  r = korean_renderer
  assert r.font_fallback(r.normal, text) is r.normal


def test_legacy_locale_and_bootstrap_without_display_font(korean_renderer):
  r = korean_renderer
  assert r.font_fallback(r.normal) is r.normal
  r.language.value = "ko"
  assert r.font_fallback(r.normal) is r.display
  r.gui_app.has_font = lambda weight: False
  assert r.font_fallback(r.normal, "한글") is r.normal


@pytest.mark.parametrize("label_class", ["Label", "UnifiedLabel"])
def test_mixed_label_wraps_and_draws_with_one_font_then_restores_ascii(korean_renderer, label_class):
  r = korean_renderer
  label = getattr(r, label_class)("한글 AB CD", font_size=20)
  label.render(r.rl.Rectangle(0, 0, 105, 120))
  calls = r.rl.draw_calls
  assert [c[1] for c in calls] == ["한글 AB", "CD"]
  assert all(c[0] is r.display for c in calls)
  assert all(f is r.display for f, _ in r.measurements)
  calls.clear()
  label.set_text("AB CD")
  label.render(r.rl.Rectangle(0, 0, 105, 120))
  assert [c[1] for c in calls] == ["AB CD"]
  assert all(c[0] is r.normal for c in calls)


@pytest.mark.parametrize("label_class", ["gui_label", "Label", "UnifiedLabel"])
def test_eliding_away_korean_keeps_measurement_and_draw_font(korean_renderer, label_class):
  r = korean_renderer
  rect = r.rl.Rectangle(0, 0, 105, 40)
  if label_class == "gui_label":
    r.gui_label(rect, "ABCDEFG한글", font_size=20)
  else:
    kwargs = {"elide_right": True} if label_class == "Label" else {"elide": True, "wrap_text": False}
    label = getattr(r, label_class)("ABCDEFG한글", font_size=20, **kwargs)
    label.render(rect)
  call = r.rl.draw_calls[-1]
  assert call[1].isascii() and call[1].endswith("...")
  assert call[0] is r.display
  assert len(call[1]) * r.display.advance <= rect.width


def test_locale_change_invalidates_label_and_measurement_caches(korean_renderer):
  r = korean_renderer
  label = r.UnifiedLabel("AB", font_size=20)
  rect = r.rl.Rectangle(0, 0, 100, 40)
  for language, font, width in [("en", r.normal, 20), ("ko", r.display, 40), ("en", r.normal, 20)]:
    r.language.value = language
    label.render(rect)
    assert label.text_width == width
    assert r.rl.draw_calls[-1][0] is font


@pytest.mark.parametrize("label_class", ["Label", "UnifiedLabel"])
def test_emoji_splits_keep_korean_label_font_and_advance(korean_renderer, label_class):
  r = korean_renderer
  def find_emoji(text):
    index = text.find("🙂")
    return [(index, index + 1, "🙂")] if index >= 0 else []
  r.namespace["find_emoji"] = find_emoji
  r.namespace["emoji_tex"] = lambda emoji: SimpleNamespace(height=20)
  r.measure_text_cached.__globals__["find_emoji"] = find_emoji
  r.rl.draw_texture_ex = lambda *args: None
  label = getattr(r, label_class)("AB🙂한글", font_size=20)
  label.render(r.rl.Rectangle(0, 0, 300, 40))
  calls = r.rl.draw_calls
  assert [c[1] for c in calls] == ["AB", "한글"]
  assert all(c[0] is r.display for c in calls)
  # Two Latin glyphs in DISPLAY (40) plus the fixed emoji width (20).
  assert calls[1][2].x - calls[0][2].x == 60


def test_generated_and_source_fonts_cover_jamo_and_common_symbols(korean_renderer):
  r = korean_renderer
  font_dir = UI_DIR.parents[1] / "selfdrive/assets/fonts"
  tree = ast.parse((font_dir / "process.py").read_text(encoding="utf-8"))
  extra = next(n for n in tree.body if isinstance(n, ast.Assign) and
               any(isinstance(t, ast.Name) and t.id == "EXTRA_CHARS" for t in n.targets))
  scope = {"_languages": dict}
  exec(compile(ast.Module(body=[extra], type_ignores=[]), "process.py", "exec"), scope)
  load_definitions(font_dir / "process.py", scope, {"_char_sets"})
  base, generated = map(set, scope["_char_sets"]())

  tree = ast.parse((UI_DIR / "lib/application.py").read_text(encoding="utf-8"))
  app = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "GuiApplication")
  method = next(n for n in app.body if isinstance(n, ast.FunctionDef) and n.name == "_font_codepoints")
  scope.update(FontWeight=r.FontWeight)
  exec(compile(ast.Module(body=[method], type_ignores=[]), "application.py", "exec"), scope)
  source = scope["_font_codepoints"]
  jamo = set(range(0x1100, 0x1200)) | set(range(0x3130, 0x3190))
  common = set(map(ord, scope["EXTRA_CHARS"]))
  assert jamo <= generated
  assert not jamo & base
  for weight in (r.FontWeight.DISPLAY, r.FontWeight.UNIFONT):
    assert jamo | common <= set(source(None, weight))
  assert common <= set(source(None, r.FontWeight.NORMAL))
  assert not jamo & set(source(None, r.FontWeight.NORMAL))
