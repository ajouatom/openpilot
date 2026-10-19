"""Passive pearl-eye DM character shared by C3 and C4. No camera consumer."""
import time

import pyray as rl

from openpilot.selfdrive.ui.dm_preview import PREVIEW_MAX_AGE, eye_expression, preview_rect, preview_state, sample_fresh, smooth_expression
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


PEARL = rl.Color(244, 237, 223, 255)
PUPIL = rl.Color(58, 53, 48, 255)
SHADOW = rl.Color(15, 17, 22, 210)
UNKNOWN = rl.Color(145, 145, 153, 220)
WARNING = rl.Color(255, 151, 105, 255)
HANDS = rl.Color(160, 204, 229, 255)


class DriverPreview(Widget):
  def __init__(self, compact: bool = False):
    super().__init__()
    self.compact = compact
    self.set_enabled(False)  # Display only; touches keep their normal onroad meaning.
    self._font = gui_app.font(FontWeight.BOLD)
    self._closed = False
    self._reset_expression()

  def _reset_expression(self):
    self._started_frame = ui_state.started_frame
    self._expression = None
    self._driver_side = None
    self._draw_time = 0.0
    self._model_frame = None
    self._model_progress = 0.0

  def close(self):
    self._closed = True
    self._reset_expression()

  def draw_onroad(self, parent: rl.Rectangle, alert_visible: bool) -> bool:
    sm = ui_state.sm
    # Parent views may not call us at all while offroad.
    if self._started_frame != ui_state.started_frame:
      self._reset_expression()
    visible = (not self._closed and ui_state.started and
               (sm['selfdriveState'].enabled or ui_state.always_on_dm) and not alert_visible)
    if not visible:
      self._reset_expression()
      return False
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
    dm, model = sm['driverMonitoringState'], sm['driverStateV2']
    monitoring_fresh = self._fresh('driverMonitoringState', now)
    driver_fresh = self._fresh('driverStateV2', now)
    if driver_fresh and model.frameId != self._model_frame:
      self._model_frame, self._model_progress = model.frameId, now
    driver_fresh = driver_fresh and now - self._model_progress < PREVIEW_MAX_AGE
    vision = dm.visionPolicyState
    state = preview_state(monitoring_fresh=monitoring_fresh, camera_unavailable=dm.cameraUnavailable,
                          driver_fresh=driver_fresh, face_detected=vision.faceDetected,
                          distracted=vision.isDistracted, lockout=dm.lockout or dm.alwaysOnLockout,
                          wheel_policy=str(dm.activePolicy) == 'wheeltouch')
    animate = (monitoring_fresh and driver_fresh and not dm.cameraUnavailable and vision.faceDetected and
               state.kind in ('tracking', 'warning'))
    if animate:
      if self._driver_side != dm.isRHD or now - self._draw_time >= PREVIEW_MAX_AGE:
        self._expression = None
      self._driver_side = dm.isRHD
      driver = model.rightDriverData if dm.isRHD else model.leftDriverData
      self._expression = smooth_expression(self._expression, eye_expression(driver), now - self._draw_time)
    else:
      self._expression = None  # Never retain a reassuring pose after lost/stale data.
    self._draw_time = now

    color = {'tracking': PEARL, 'warning': WARNING, 'wheel': HANDS}.get(state.kind, UNKNOWN)
    label_h, font_size = (16, 11) if self.compact else (32, 21)
    scale = min(rect.width / 154, (rect.height - label_h - 8) / 90)
    cx, cy = rect.x + rect.width / 2, rect.y + (rect.height - label_h) * 0.47
    if state.kind == 'wheel':
      radius = 24 * scale
      rl.draw_circle_lines(int(cx), int(cy), radius, color)
      for dx, dy in ((-0.9, -0.2), (0.9, -0.2), (0, 0.9)):
        rl.draw_line_ex(rl.Vector2(cx, cy), rl.Vector2(cx + dx * radius, cy + dy * radius), max(1, 2 * scale), color)
    else:
      expression = self._expression
      for left in (True, False):
        ex = cx + (-29 if left else 29) * scale
        known = expression is not None and (expression.left_known if left else expression.right_known)
        openness = (expression.left_open if left else expression.right_open) if known else 0.08
        self._draw_eye(ex, cy, scale, openness, known, expression, color)
        if state.kind == 'warning':
          direction = 1 if left else -1
          rl.draw_line_ex(rl.Vector2(ex - 14 * scale, cy - (35 + direction * 3) * scale),
                          rl.Vector2(ex + 14 * scale, cy - (35 - direction * 3) * scale), max(2, 3 * scale), color)

    label = 'DM EYES?' if state.kind == 'tracking' and not (self._expression.left_known or self._expression.right_known) else state.label
    width = measure_text_cached(self._font, label, font_size).x
    pos = rl.Vector2(rect.x + (rect.width - width) / 2, rect.y + rect.height - label_h)
    rl.draw_text_ex(self._font, label, rl.Vector2(pos.x + 1, pos.y + 1), font_size, 0, SHADOW)
    rl.draw_text_ex(self._font, label, pos, font_size, 0, color)

  @staticmethod
  def _draw_eye(cx, cy, scale, openness, known, expression, color):
    width, height = 42 * scale, max(4, 54 * openness) * scale
    eye_color = color if known or color == WARNING else UNKNOWN
    # Only the eyes get a narrow dark silhouette; there is no camera/card box.
    rl.draw_rectangle_rounded(rl.Rectangle(cx - width / 2 - scale, cy - height / 2 - scale,
                                           width + 2 * scale, height + 2 * scale), 1, 12, SHADOW)
    rl.draw_rectangle_rounded(rl.Rectangle(cx - width / 2, cy - height / 2, width, height), 1, 12, eye_color)
    if known and openness > 0.22:
      pw, ph = width * 0.43, height * 0.48
      px, py = cx + expression.x * width * 0.16, cy + expression.y * height * 0.12
      rl.draw_rectangle_rounded(rl.Rectangle(px - pw / 2, py - ph / 2, pw, ph), 1, 10, PUPIL)
      rl.draw_circle_v(rl.Vector2(px + pw * 0.16, py - ph * 0.2), min(pw, ph) * 0.14, PEARL)
