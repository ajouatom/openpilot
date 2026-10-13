"""Bounded, alternating display layers. Cameras and input handling stay live."""
import time

import pyray as rl

from openpilot.system.ui.lib.application import gui_app


class OverlayLayer:
  MAX_AGE = 0.1

  def __init__(self, phase: int):
    self.phase = phase
    self._target = None
    self._size = (0, 0)
    self._key = None
    self._frame = -1
    self._painted_at = 0.0
    self._ready = False
    self._failed = False
    self._source = rl.Rectangle(0, 0, 0, 0)
    self._destination = rl.Rectangle(0, 0, 0, 0)
    self._origin = rl.Vector2(0, 0)

  def invalidate(self):
    self._ready = False

  def close(self):
    if self._target is not None:
      rl.unload_render_texture(self._target)
      self._target = None
    self._ready = False

  def draw(self, rect, callback, key, *, enabled=True, urgent=False):
    # Interactive/warning frames use the ordinary renderer, with no old layer
    # eligible for reuse afterwards. No camera frame is ever stored here.
    if not enabled or urgent or self._failed:
      self.invalidate()
      callback(rect)
      return

    size = (gui_app._scaled_width, gui_app._scaled_height)
    if self._target is None or size != self._size:
      previous = gui_app.render_framebuffer
      rl.rl_draw_render_batch_active()
      self.close()
      self._target = rl.load_render_texture(*size)
      complete = bool(self._target.id and self._target.texture.id and rl.rl_framebuffer_complete(self._target.id))
      rl.rl_enable_framebuffer(previous)
      self._size = size
      if not complete:
        self.close()
        rl.rl_enable_framebuffer(previous)
        self._failed = True
        callback(rect)
        return
      self._source = rl.Rectangle(0, 0, size[0], -size[1])
      self._destination = rl.Rectangle(0, 0, gui_app.width, gui_app.height)

    now = time.monotonic()
    frame = gui_app.frame
    layout_key = (rect.x, rect.y, rect.width, rect.height, key)
    age = now - self._painted_at
    repaint = (not self._ready or layout_key != self._key or
               age < 0 or age >= self.MAX_AGE or
               (frame != self._frame and frame % 2 == self.phase))
    if repaint:
      # Switch only the FBO: retain the caller's viewport, projection, scale and
      # scissor. This also works inside the recording texture and scaled UI.
      previous = gui_app.render_framebuffer
      rl.rl_draw_render_batch_active()
      rl.rl_enable_framebuffer(self._target.id)
      rl.clear_background(rl.BLANK)
      # Accumulate coverage alpha separately; RGB in the layer is premultiplied.
      rl.rl_set_blend_factors_separate(0x0302, 0x0303, 1, 0x0303, 0x8006, 0x8006)
      rl.begin_blend_mode(rl.BlendMode.BLEND_CUSTOM_SEPARATE)
      try:
        callback(rect)
      finally:
        rl.end_blend_mode()
        rl.rl_draw_render_batch_active()
        rl.rl_enable_framebuffer(previous)
      self._key = layout_key
      self._painted_at = now
      self._frame = frame
      self._ready = True

    rl.begin_blend_mode(rl.BlendMode.BLEND_ALPHA_PREMULTIPLY)
    rl.draw_texture_pro(self._target.texture, self._source, self._destination, self._origin, 0, rl.WHITE)
    rl.end_blend_mode()


class DisplayClock:
  """Elapsed nominal 20 Hz frames for display-only filters and moving patterns."""
  def __init__(self):
    self._previous = None
    self.frames = 1.0

  def tick(self):
    now = time.monotonic()
    self.frames = 1.0 if self._previous is None else max(0.0, min(5.0, (now - self._previous) * 20.0))
    self._previous = now
    return self.frames

  def retention(self, value):
    return value ** self.frames

  def update_filter(self, filt, value):
    if filt.initialized:
      retention = self.retention(1.0 - filt.alpha)
      filt.x = retention * filt.x + (1.0 - retention) * value
    else:
      filt.x = value
      filt.initialized = True
    return filt.x
