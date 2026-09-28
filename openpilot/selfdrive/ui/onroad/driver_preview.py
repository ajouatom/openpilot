"""Passive onroad camera inset. Never enters driver-preview/demo mode."""
import time

import pyray as rl
from msgq.visionipc import VisionIpcClient, VisionStreamType

from openpilot.selfdrive.ui.dm_preview import face_crop, preview_rect, preview_state, sample_fresh
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware import TICI
from openpilot.system.ui.lib.application import FontWeight, gui_app


class DriverPreview(CameraView):
  def __init__(self, compact: bool = False):
    self._closed = False
    super().__init__('camerad', VisionStreamType.VISION_STREAM_DRIVER)
    self.compact = compact
    self.set_enabled(False)  # Display only; touches keep their normal onroad meaning.
    self._font = gui_app.font(FontWeight.BOLD)
    self._frame_time = 0.0
    self._next_poll = 0.0
    self._was_visible = False

  def close(self):
    if not self._closed:
      self._closed = True
      super().close()

  def _offroad_transition(self):
    if not self._closed:
      self._reset_stream()

  def _reset_stream(self):
    self.frame = None
    self._frame_time = 0.0
    self._next_poll = 0.0
    self._clear_textures()
    self.client = VisionIpcClient(self._name, self._stream_type, conflate=True)

  def draw_onroad(self, parent: rl.Rectangle, alert_visible: bool) -> bool:
    sm = ui_state.sm
    visible = ui_state.started and (sm['selfdriveState'].enabled or ui_state.always_on_dm) and not alert_visible
    if not visible:
      if self._was_visible:
        self._reset_stream()
      self._was_visible = False
      return False
    self._was_visible = True
    self.render(rl.Rectangle(*preview_rect(parent.x, parent.y, parent.width, parent.height, self.compact)))
    return True

  @staticmethod
  def _fresh(service: str, now: float) -> bool:
    sm = ui_state.sm
    return (sm.valid[service] and sm.alive[service] and sm.recv_frame[service] > ui_state.started_frame and
            sample_fresh(now, sm.logMonoTime[service] / 1e9))

  def _render(self, rect):
    now = time.monotonic()
    sm = ui_state.sm
    dm = sm['driverMonitoringState']
    monitoring_fresh = self._fresh('driverMonitoringState', now)
    driver_fresh = self._fresh('driverStateV2', now)
    camera_allowed = monitoring_fresh and not dm.cameraUnavailable and driver_fresh

    # Consume only the existing camera stream, at most 10 Hz, without waiting.
    if camera_allowed and now >= self._next_poll:
      self._next_poll = now + 0.1
      if self._ensure_connection():
        buffer = self.client.recv(timeout_ms=0)
        if buffer is not None:
          self.frame = buffer
          self._frame_time = self.client.timestamp_sof / 1e9
          self._texture_needs_update = True
    frame_fresh = camera_allowed and self.frame is not None and sample_fresh(now, self._frame_time)
    if not frame_fresh:
      self.frame = None

    vision = dm.visionPolicyState
    state = preview_state(monitoring_fresh=monitoring_fresh, camera_unavailable=dm.cameraUnavailable,
                          driver_fresh=driver_fresh, frame_fresh=frame_fresh, face_detected=vision.faceDetected,
                          distracted=vision.isDistracted, lockout=dm.lockout or dm.alwaysOnLockout,
                          wheel_policy=str(dm.activePolicy) == 'wheeltouch')
    color = {'tracking': rl.Color(70, 220, 140, 255), 'wheel': rl.Color(110, 190, 255, 255),
             'warning': rl.Color(255, 90, 60, 255)}.get(state.kind, rl.Color(255, 195, 70, 255))
    rl.draw_rectangle_rec(rect, rl.Color(12, 18, 24, 245))
    label_h = 14 if self.compact else 32
    viewport = rl.Rectangle(rect.x + 2, rect.y + 2, rect.width - 4, rect.height - label_h - 4)
    if frame_fresh:
      driver = sm['driverStateV2'].rightDriverData if dm.isRHD else sm['driverStateV2'].leftDriverData
      crop = face_crop(self.frame.width, self.frame.height, driver.facePosition, viewport.width / viewport.height) \
        if vision.faceDetected else None
      if crop is None:
        crop = (0, 0, self.frame.width, self.frame.height)
      sx, sy, sw, sh = crop
      scale = min(viewport.width / sw, viewport.height / sh)
      target = rl.Rectangle(viewport.x + (viewport.width - sw * scale) / 2,
                            viewport.y + (viewport.height - sh * scale) / 2, sw * scale, sh * scale)
      source = rl.Rectangle(sx, sy, -sw, sh)
      # Base renderer rebinds the external texture every draw, including repeated frames.
      if TICI:
        self._render_egl(source, target)
      else:
        self._render_textures(source, target)
    elif state.kind == 'wheel':
      cx, cy = viewport.x + viewport.width / 2, viewport.y + viewport.height / 2
      radius = min(viewport.width, viewport.height) * 0.32
      rl.draw_circle_lines(int(cx), int(cy), radius, color)
      for dx, dy in ((-0.9, -0.2), (0.9, -0.2), (0, 0.9)):
        rl.draw_line_ex(rl.Vector2(cx, cy), rl.Vector2(cx + dx * radius, cy + dy * radius), 2, color)
    else:
      size = 20 if self.compact else 48
      rl.draw_text_ex(self._font, 'DM', rl.Vector2(viewport.x + 8, viewport.y + viewport.height / 2 - size / 2),
                      size, 0, color)
    rl.draw_rectangle_lines_ex(rect, 1 if self.compact else 2, color)
    size = 9 if self.compact else 21
    rl.draw_text_ex(self._font, state.label, rl.Vector2(rect.x + 3, rect.y + rect.height - label_h + 1), size, 0, color)
