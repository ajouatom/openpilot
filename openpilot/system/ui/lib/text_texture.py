"""Cache complete, repeatedly unchanged styled labels at frame boundaries.

GPU work stays on the render thread. Misses draw immediately through the native
text path; a new value never displays an older cached label. At most one small
build stage runs per frame, after eight observed frames with identical inputs.
Only direct display output is eligible: cached source-over preserves visible RGB,
but regrouping layers changes Raylib's nonstandard intermediate alpha. Recording,
scaled/burn-in FBOs and transformed text retain the original primitives.
"""
import math
import os
import logging
import time
from collections import OrderedDict

from openpilot.system.ui.lib import native_text

ENABLED = os.getenv('CARROT_UI_TEXT_TEXTURE', '1') != '0'
MAX_ENTRIES = 64
MAX_PIXELS = 512 * 1024  # Retained RGBA + depth: roughly 4 MiB, plus two temporary targets.
MAX_LABEL_PIXELS = 64 * 1024
STABLE_FRAMES = 8

_VERTEX = """
in vec3 vertexPosition;
in vec2 vertexTexCoord;
out vec2 fragTexCoord;
uniform mat4 mvp;
void main() {
  fragTexCoord = vertexTexCoord;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""

_FRAGMENT = """
in vec2 fragTexCoord;
out vec4 finalColor;
uniform sampler2D texture0;
uniform sampler2D transmittance;
void main() {
  vec3 rgb = texture(texture0, fragTexCoord).rgb;
  float a = 1.0 - texture(transmittance, fragTexCoord).r;
  finalColor = vec4(a > 0.0 ? rgb / a : vec3(0.0), a);
}
"""


class TextTextureCache:
  def __init__(self):
    self.entries = OrderedDict()
    self.observed = OrderedDict()
    self.frame = 0
    self.pixels = 0
    self.ready = False
    self.compositor = None
    self.disabled = False
    self.shader = None
    self.pending = None
    self.hits = self.builds = 0
    self.frame_hits = 0
    self.build_cpu_ms = 0.

  def clear(self, rl):
    if self.pending is not None:
      self.pending[1].close()
      self.pending = None
    for target, _, _pixels in self.entries.values():
      rl.unload_render_texture(target)
    self.entries.clear()
    self.observed.clear()
    self.pixels = 0
    self.ready = False
    if self.shader is not None:
      rl.unload_shader(self.shader)
      self.shader = None

  def initialize_shader(self, rl):
    """Called during window setup, before onroad frame deadlines begin."""
    if not ENABLED or self.disabled or self.shader is not None or native_text._backend(rl) is None:
      return
    try:
      version = '#version 300 es\nprecision highp float;\n' if rl.rl_get_version() >= 5 else '#version 330\n'
      self.shader = rl.load_shader_from_memory(version + _VERTEX, version + _FRAGMENT)
      self.mask_location = rl.get_shader_location(self.shader, 'transmittance')
      if not self.shader.id or self.mask_location < 0:
        logging.warning('UI label shader unavailable; using native text')
        if self.shader.id:
          rl.unload_shader(self.shader)
        self.shader = None
        self.ready = False
        self.disabled = True
    except Exception:
      logging.exception('UI label shader initialization failed; using native text')
      if self.shader is not None:
        rl.unload_shader(self.shader)
      self.shader = None
      self.ready = False
      self.disabled = True

  def prepare(self, rl, scale=1., direct=True):
    """Call before begin_drawing/texture_mode, outside any shader or scissor."""
    self.frame += 1
    self.frame_hits = 0
    self.build_cpu_ms = 0.
    self.ready = ENABLED and not self.disabled and direct and scale == 1. and native_text._ENABLED and native_text.native_draw._ENABLED
    if not self.ready:
      if self.pending is not None:
        self.pending[1].close()
        self.pending = None
      return
    if self.compositor is None:
      try:
        native = native_text.native_draw._draw_native
        if native_text._backend(rl) is None or not hasattr(native, 'TextCompositor'):
          self.ready = False
          return
        fields = [f'm{r + c*4}' for r in range(4) for c in range(4)]
        if rl.ffi.sizeof('Matrix') != 64 or any(rl.ffi.offsetof('Matrix', f) != 4*i for i, f in enumerate(fields)):
          self.ready = False
          return
        self.compositor = native.TextCompositor(*(int(rl.ffi.cast('uintptr_t', rl.ffi.addressof(rl.rl, name))) for name in
                                                  ('DrawTexturePro', 'rlGetMatrixTransform', 'rlGetMatrixModelview')))
      except Exception:
        logging.exception('UI label texture initialization failed; using native text')
        self.ready = False
        self.disabled = True
        return
    if self.pending is not None:
      key, work = self.pending
      seen = self.observed.get(key)
      if seen is None or seen[0] < self.frame - 1:
        work.close()
        self.pending = None
      else:
        self._advance(rl)
      return
    for key, (last, count, font, args) in list(self.observed.items()):
      if last < self.frame - 1:
        del self.observed[key]
        continue
      if count < STABLE_FRAMES or key in self.entries:
        continue
      try:
        self.pending = (key, self._build(rl, key, font, args))
        self._advance(rl)
      except Exception:
        # Optional caching must not take down the UI on an unfamiliar GL binding.
        logging.exception('UI label texture build failed; using native text')
        self.clear(rl)
        self.disabled = True
      break

  def _advance(self, rl):
    started = time.thread_time_ns()
    key, work = self.pending
    try:
      next(work)
    except StopIteration:
      self.pending = None
      self.observed.pop(key, None)
    except Exception:
      logging.exception('UI label texture build failed; using native text')
      self.pending = None
      self.clear(rl)
      self.disabled = True
    finally:
      self.build_cpu_ms = (time.thread_time_ns() - started) / 1e6

  def _build(self, rl, key, font, args):
    renderer = native_text._backend(rl)
    if renderer is None or not hasattr(renderer, 'bounds'):
      return
    text, x, y, size, border, shadow, color, border_color, shadow_color = args
    address = int(rl.ffi.cast('uintptr_t', rl.ffi.addressof(font)))
    bounds = renderer.bounds(address, text, size, x, y, native_text._OFFSETS, border, shadow)
    if bounds is None or not all(math.isfinite(v) for v in bounds):
      return
    x0, y0 = math.floor(bounds[0]) - 1, math.floor(bounds[1]) - 1
    width, height = math.ceil(bounds[2]) + 1 - x0, math.ceil(bounds[3]) + 1 - y0
    pixels = width * height
    if not (0 < width <= 1024 and 0 < height <= 256 and 0 < pixels <= MAX_LABEL_PIXELS):
      return
    if self.shader is None:
      self.initialize_shader(rl)
      if self.disabled or self.shader is None:
        return
    while self.entries and (len(self.entries) >= MAX_ENTRIES or self.pixels + pixels > MAX_PIXELS):
      _, (old, _, cost) = self.entries.popitem(last=False)
      rl.unload_render_texture(old)
      self.pixels -= cost
    targets = []
    try:
      for _ in range(3):
        targets.append(rl.load_render_texture(width, height))
    except Exception:
      for target in targets:
        if target.id:
          rl.unload_render_texture(target)
      raise
    if any(not target.id or not target.texture.id for target in targets):
      for target in targets:
        if target.id:
          rl.unload_render_texture(target)
      return
    try:
      for i, target in enumerate(targets[:2]):
        rl.begin_texture_mode(target)
        try:
          # Store RGB accumulation and transmittance separately, then convert
          # to straight RGB/coverage for ordinary alpha blending on the display.
          rl.clear_background(rl.BLANK if i == 0 else rl.WHITE)
          if i == 1:
            rl.rl_set_blend_factors(0, 0x0303, 0x8006)  # ZERO, ONE_MINUS_SRC_ALPHA, ADD
          rl.begin_blend_mode(rl.BlendMode.BLEND_ALPHA if i == 0 else rl.BlendMode.BLEND_CUSTOM)
          rl.rl_push_matrix()
          try:
            rl.rl_translatef(-x0, -y0, 0.)
            renderer.draw(address, text, x, y, size, native_text._OFFSETS, border, shadow,
                          color, border_color, shadow_color, True)
          finally:
            rl.rl_pop_matrix()
            rl.end_blend_mode()
        finally:
          rl.end_texture_mode()
        yield  # Ink and coverage are independent frame-budgeted stages.
      rl.begin_texture_mode(targets[2])
      try:
        rl.clear_background(rl.BLANK)
        rl.rl_set_blend_factors(1, 0, 0x8006)  # Copy unpremultiplied RGB/coverage.
        rl.begin_blend_mode(rl.BlendMode.BLEND_CUSTOM)
        rl.begin_shader_mode(self.shader)
        try:
          rl.set_shader_value_texture(self.shader, self.mask_location, targets[1].texture)
          rl.draw_texture_pro(targets[0].texture, rl.Rectangle(0, 0, width, -height), rl.Rectangle(0, 0, width, height),
                              rl.Vector2(0, 0), 0., rl.WHITE)
        finally:
          rl.end_shader_mode()
          rl.end_blend_mode()
      finally:
        rl.end_texture_mode()
    except BaseException:
      for target in targets:
        rl.unload_render_texture(target)
      raise
    for target in targets[:2]:
      rl.unload_render_texture(target)
    target = targets[2]
    rl.set_texture_filter(target.texture, rl.TextureFilter.TEXTURE_FILTER_POINT)
    self.entries[key] = (target, (x0, y0, width, height), pixels)
    self.pixels += pixels
    self.builds += 1

  def draw(self, rl, font, text, x, y, size, border, shadow, color, border_color, shadow_color):
    if (not self.ready or not native_text._ENABLED or not native_text.native_draw._ENABLED or len(text) > 128 or b'\n' in text
        or not self.compositor.identity()):
      return False
    # Absolute position preserves original subpixel rounding; do not quantize it.
    args = (text, x, y, size, border, shadow, tuple(native_text._rgba(color)),
            tuple(native_text._rgba(border_color)), tuple(native_text._rgba(shadow_color)))
    key = (font.texture.id, font.texture.width, font.texture.height, font.texture.mipmaps, font.texture.format,
           font.baseSize, font.glyphCount, font.glyphPadding,
           int(rl.ffi.cast('uintptr_t', font.recs)), int(rl.ffi.cast('uintptr_t', font.glyphs)), args)
    hit = self.entries.get(key)
    if hit is None:
      previous = self.observed.get(key)
      count = previous[1] + (previous[0] != self.frame) if previous and previous[0] >= self.frame - 1 else 1
      self.observed[key] = (self.frame, count, font, args)
      self.observed.move_to_end(key)
      while len(self.observed) > MAX_ENTRIES * 2:
        self.observed.popitem(last=False)
      return False
    self.entries.move_to_end(key)
    target, rect, _ = hit
    self.compositor.draw(int(rl.ffi.cast('uintptr_t', rl.ffi.addressof(target.texture))), *rect)
    self.hits += 1
    self.frame_hits += 1
    return True


cache = TextTextureCache()
