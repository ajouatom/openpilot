"""Compare actual native submissions with an independent float32 Raylib model."""
import ctypes as c
import math

import numpy as np
import pytest

from openpilot.system.ui.lib import _draw_native as native, native_draw, native_text


class Vector2(c.Structure):
  _fields_ = [('x', c.c_float), ('y', c.c_float)]


class Color(c.Structure):
  _fields_ = [(f, c.c_ubyte) for f in 'rgba']


class Rectangle(c.Structure):
  _fields_ = [(f, c.c_float) for f in ('x', 'y', 'width', 'height')]


class Texture(c.Structure):
  _fields_ = [('id', c.c_uint)] + [(f, c.c_int) for f in ('width', 'height', 'mipmaps', 'format')]


class Image(c.Structure):
  _fields_ = [('data', c.c_void_p)] + [(f, c.c_int) for f in ('width', 'height', 'mipmaps', 'format')]


class GlyphInfo(c.Structure):
  _fields_ = [(f, c.c_int) for f in ('value', 'offsetX', 'offsetY', 'advanceX')] + [('image', Image)]


class Font(c.Structure):
  _fields_ = [(f, c.c_int) for f in ('baseSize', 'glyphCount', 'glyphPadding')] + [
    ('texture', Texture), ('recs', c.POINTER(Rectangle)), ('glyphs', c.POINTER(GlyphInfo))]


TEXT = c.CFUNCTYPE(None, Font, c.c_char_p, Vector2, c.c_float, c.c_float, Color)
QUAD = c.CFUNCTYPE(None, Texture, Rectangle, Rectangle, Vector2, c.c_float, Color)
GLYPH = c.CFUNCTYPE(c.c_int, Font, c.c_int)
CODEPOINT = c.CFUNCTYPE(c.c_int, c.c_char_p, c.POINTER(c.c_int))
OFFSETS = np.array([(math.cos(math.radians(d)), math.sin(math.radians(d))) for d in range(0, 360, 45)])
WHITE, BLACK, SHADOW = (201, 221, 241, 161), (11, 21, 31, 81), (7, 17, 27, 127)


def values(obj):
  return tuple(getattr(obj, f) for f, _ in obj._fields_)


def test_native_struct_layout_matches_host_abi():
  assert native.text_abi() == {t.__name__: (c.sizeof(t), tuple(getattr(t, f).offset for f, _ in t._fields_))
                             for t in (Vector2, Color, Rectangle, Texture, Image, GlyphInfo, Font)}


@pytest.fixture
def renderer():
  codepoints = [63, 65, 66, 32, 9, 0xD55C, 0xAE00]
  rects = (Rectangle * len(codepoints))(*(Rectangle(11*i, 13*i, 5.25+i, 12.5+i) for i in range(len(codepoints))))
  glyphs = (GlyphInfo * len(codepoints))(*(GlyphInfo(cp, i-2, 3-i, 0 if i % 2 else 8+i) for i, cp in enumerate(codepoints)))
  font = Font(19, len(codepoints), 3, Texture(9, 256, 128, 1, 7), rects, glyphs)
  calls, lookups, fallback = [], [], []

  @TEXT
  def text_fn(f, text, pos, size, spacing, tint):
    fallback.append((text, values(pos), size, values(tint)))

  @QUAD
  def quad_fn(tex, src, dest, origin, rotation, tint):
    calls.append((values(tex), values(src), values(dest), values(origin), rotation, values(tint)))

  @GLYPH
  def glyph_fn(f, cp):
    lookups.append(cp)
    return next((i for i in range(f.glyphCount) if f.glyphs[i].value == cp), 0)

  @CODEPOINT
  def codepoint_fn(text, length):
    char = text.decode('utf8')[0]
    length[0] = len(char.encode('utf8'))
    return ord(char)

  callbacks = (text_fn, quad_fn, glyph_fn, codepoint_fn)
  backend = native.TextRenderer(*(c.cast(fn, c.c_void_p).value for fn in callbacks))
  return backend, font, calls, lookups, fallback, callbacks


def reference(font, text, x, y, size, border, shadow, spacing=0):
  f = np.float32
  size = f(size)
  scale = f(size / f(font.baseSize))
  layers = [(f(x+border*ux), f(y+border*uy), BLACK) for ux, uy in OFFSETS] if border > 0 else []
  if shadow:
    layers.append((f(x+shadow), f(y+shadow), SHADOW))
  layers.append((f(x), f(y), WHITE))
  expected = []
  for px, py, tint in layers:
    advance = f(0)
    for char in text.split('\0')[0]:
      idx = next((i for i in range(font.glyphCount) if font.glyphs[i].value == ord(char)), 0)
      rec, glyph = font.recs[idx], font.glyphs[idx]
      pad = f(font.glyphPadding)
      if char not in ' \t':
        src = (f(rec.x-pad), f(rec.y-pad), f(rec.width+f(2)*pad), f(rec.height+f(2)*pad))
        dest = (f(f(f(px+advance) + f(glyph.offsetX*scale)) - f(pad*scale)),
                f(f(f(py+f(0)) + f(glyph.offsetY*scale)) - f(pad*scale)), f(src[2]*scale), f(src[3]*scale))
        expected.append((values(font.texture), src, dest, (0., 0.), 0., tint))
      advance = f(advance + f(f((glyph.advanceX if glyph.advanceX else rec.width)*scale) + f(spacing)))
  return expected


@pytest.mark.parametrize('text', ['AB A\tB', '한글?☃', 'A\0B', '', '   '])
@pytest.mark.parametrize('geometry', [(0., 0., 32., 3., 8.), (1023.9999, -55.001, 83.171, .5, -4.), (-50., 22.2, 19., 0., 0.)])
def test_cached_text_preserves_exact_quad_stream(renderer, text, geometry):
  backend, font, calls, _, fallback, _callbacks = renderer
  x, y, size, border, shadow = geometry
  for _ in range(2):  # Cold and warm cache must be identical.
    calls.clear()
    backend.draw(c.addressof(font), text.encode(), x, y, size, OFFSETS, border, shadow, WHITE, BLACK, SHADOW)
    assert calls == reference(font, text, x, y, size, border, shadow)
    assert fallback == []


def test_cache_reuses_layout_but_never_stale_position_color_or_font(renderer):
  backend, font, calls, lookups, _, _callbacks = renderer
  def draw(x, color=WHITE):
    backend.draw(c.addressof(font), b'AB', x, 0, 32, OFFSETS, 0, 0, color, BLACK, SHADOW)
  draw(10)
  assert len(lookups) == 2
  calls.clear()
  draw(500, BLACK)
  assert len(lookups) == 2
  assert calls[0][-1] == BLACK
  assert calls[0][2][0] > 490
  font.glyphPadding += 1
  draw(10)
  assert len(lookups) == 4
  font.texture.id += 1
  draw(10)
  assert len(lookups) == 6
  backend.clear()
  assert backend.stats() == {'entries': 0, 'glyphs': 0, 'hits': 0, 'misses': 0}
  draw(10)
  assert len(lookups) == 8


@pytest.mark.parametrize('spacing', [-2.33, 0., 1.7, 5.])
def test_plain_text_uses_current_spacing_and_exact_quad_order(renderer, spacing):
  backend, font, calls, _, _, _callbacks = renderer
  for gap in (0., spacing, spacing):
    calls.clear()
    backend.draw_plain(c.addressof(font), 'A 한글?\tB'.encode(), 150.37, 20.17, 36.712, gap, WHITE)
    assert calls == reference(font, 'A 한글?\tB', 150.37, 20.17, 36.712, 0, 0, gap)


@pytest.mark.parametrize('reason', ['multiline', 'long', 'disabled', 'invalid_font'])
def test_original_text_fallback_keeps_style_and_order(renderer, reason):
  backend, font, calls, _, fallback, _callbacks = renderer
  text = b'A\nB' if reason == 'multiline' else b'A'*513 if reason == 'long' else b'AB'
  if reason == 'invalid_font':
    font.texture.id = 0
  backend.draw(c.addressof(font), text, 10, 20, 32, OFFSETS, 3, 8, WHITE, BLACK, SHADOW, reason != 'disabled')
  assert not calls
  assert len(fallback) == 10
  assert [v[-1] for v in fallback] == [BLACK]*8 + [SHADOW, WHITE]
  assert fallback[-1] == (text, (10., 20.), 32., WHITE)


def test_cache_limits_entries_and_total_glyphs(renderer):
  backend, font, _, _, _, _callbacks = renderer
  for text in [str(i) for i in range(300)] + ['A'*490 + str(i) for i in range(25)]:
    backend.draw(c.addressof(font), text.encode(), 0, 0, 32, OFFSETS, 0, 0, WHITE, BLACK, SHADOW)
    assert backend.stats()['entries'] <= 256
    assert backend.stats()['glyphs'] <= 8192


@pytest.mark.parametrize('reason', ['disabled', 'missing', 'old_binary', 'incompatible_abi'])
def test_optional_backend_falls_back_before_drawing(monkeypatch, reason):
  from types import SimpleNamespace
  monkeypatch.setattr(native_text, '_raylib', None)
  monkeypatch.setattr(native_text, '_renderer', None)
  monkeypatch.setattr(native_draw, '_ENABLED', reason != 'disabled')
  monkeypatch.setattr(native_draw, '_draw_native', None if reason == 'missing' else SimpleNamespace() if reason == 'old_binary' else native)
  rl = SimpleNamespace(ffi=SimpleNamespace(sizeof=lambda _: 123, offsetof=lambda *_: 0))
  assert not native_text.try_text(rl, None, b'AB', 0, 0, 32, 3, 8, WHITE, BLACK, SHADOW)


def test_unrecognized_cffi_typedef_is_an_optional_backend_failure(monkeypatch):
  from types import SimpleNamespace
  class BindingError(Exception):
    pass
  def unknown(_):
    raise BindingError('unknown Font typedef')
  monkeypatch.setattr(native_text, '_raylib', None)
  monkeypatch.setattr(native_text, '_renderer', None)
  monkeypatch.setattr(native_draw, '_ENABLED', True)
  monkeypatch.setattr(native_draw, '_draw_native', native)
  assert native_text._backend(SimpleNamespace(ffi=SimpleNamespace(sizeof=unknown))) is None
