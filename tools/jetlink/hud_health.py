"""Jetson-local temperature display; never substitute it for vehicle statistics."""
import math
import time

from host_health import STATUS
from openpilot.common.jetlink_status import _fresh
from cluster_renderer import (ClusterUiRenderer, DESIGN_WIDTH, DESIGN_HEIGHT,
                              EGPU_STATUS_CENTER_X, TOP_STATUS_CENTER_Y, rl, rl_color)

NORMAL = (100, 225, 170)
WARNING = (255, 205, 70)
ERROR = (255, 90, 90)
UNKNOWN = (190, 190, 190)


def temperature_label(health):
  value = health.get('temp_c')
  if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or not -20 <= value <= 150:
    return '--°C', UNKNOWN
  severity = health.get('thermal_severity', 'unknown')
  color = {'ok': NORMAL, 'warning': WARNING, 'error': ERROR}.get(severity, UNKNOWN)
  return f'{value:.1f}°C', color


def health_banner(health, language):
  thermal = health.get('thermal_severity')
  if thermal in ('warning', 'error'):
    trip = health.get('thermal_trip') or {}
    temp, limit = trip.get('temp_c'), trip.get('limit_c')
    detail = ''
    if all(isinstance(v, (int, float)) and math.isfinite(v) for v in (temp, limit)):
      detail = f" · {temp:.1f}°C / {'기준' if language == 'ko' else 'limit'} {limit:.1f}°C"
    if language == 'ko':
      title = 'Jetson 과열' if thermal == 'error' else 'Jetson 온도 높음'
      message = f'{title}{detail} · 냉각 상태 확인'
    else:
      title = 'Jetson overheating' if thermal == 'error' else 'Jetson temperature high'
      message = f'{title}{detail} · Check cooling'
    return message, ERROR if thermal == 'error' else WARNING
  if health.get('severity', 'unknown') != 'ok':
    reason = health.get('reason') or ('온도·상태 정보 없음' if language == 'ko' else 'Temperature/health unavailable')
    return f'Jetson: {reason}', ERROR if health.get('severity') == 'error' else WARNING
  return '', NORMAL


class HealthRenderer(ClusterUiRenderer):
  def render(self, state, signal_lights=None):
    from hud_protocol import read_snapshot
    from hud_bsd import warning_sides
    snapshot = read_snapshot()
    now = time.monotonic()
    self._bsd_warning = warning_sides(state, snapshot, now)
    previous = getattr(self, '_bsd_started', (None, None))
    self._bsd_started = tuple((stamp if stamp is not None else now) if active else None
                              for active, stamp in zip(self._bsd_warning, previous, strict=True))
    self.host_health = _fresh(STATUS, time.monotonic())
    self.host_badge_drawn = False
    super().render(state, signal_lights)

  def _temperature_badge(self, x, y):
    label, color = temperature_label(self.host_health)
    rect = rl.Rectangle(x - 52, y - 17, 104, 60)
    rl.draw_rectangle_rounded(rect, .25, 8, rl_color((0, 0, 0), 190))
    rl.draw_rectangle_rounded_lines_ex(rect, .25, 8, 2., rl_color(color))
    self._draw_text('jetSON', x, y, 19, color, anchor='center')
    self._draw_text(label, x, y + 26, 20, color, anchor='center')
    self.host_badge_drawn = True

  def _draw_egpu_status(self, state):
    # The normal status row's transform also follows left/right panel swaps.
    self._temperature_badge(EGPU_STATUS_CENTER_X + 20, TOP_STATUS_CENTER_Y)

  def _draw_alert_overlay(self, alert):
    left, right = getattr(self, '_bsd_warning', (False, False))
    if left or right:
      # Finish the existing HUD batch before the gradient's native quad changes
      # drawing state; otherwise preceding panel text can be clipped. No GPU wait.
      rl.rl_draw_render_batch_active()
    rl.rl_push_matrix()
    rl.rl_scalef(self.width / DESIGN_WIDTH, self.height / DESIGN_HEIGHT, 1.)
    try:
      # Fixed geometry reuses the current frame; no new detector, CAN poll or
      # process. Existing vehicle alerts are still drawn last over this cue.
      from hud_bsd import edge_alpha, GRADIENT_ALPHA
      now = time.monotonic()
      starts = getattr(self, '_bsd_started', (now, now))
      # Physical screen sides stay left/right even when the panels are swapped.
      for side, active in enumerate((left, right)):
        if active:
          # One constant, translucent native quad per side. No image buffers,
          # shader, thread or timer; reuses the 10 Hz renderer. The center third
          # remains untinted, and the broad background never flashes.
          shade, clear = rl_color((255, 30, 30), GRADIENT_ALPHA), rl_color((255, 30, 30), 0)
          width = int(DESIGN_WIDTH / 3)
          rl.draw_rectangle_gradient_h(0 if side == 0 else int(DESIGN_WIDTH) - width,
                                       0, width, int(DESIGN_HEIGHT),
                                       shade if side == 0 else clear, clear if side == 0 else shade)
          alpha = edge_alpha(now, starts[side])
          # Flush, full-height edges; the halo extends only into the display.
          for edge_width, opacity in ((48, int(alpha * .3)), (24, alpha)):
            x = 0 if side == 0 else int(DESIGN_WIDTH) - edge_width
            rl.draw_rectangle(x, 0, edge_width, int(DESIGN_HEIGHT), rl_color((255, 30, 30), opacity))
      if not self.host_badge_drawn:
        # Full navigation/graph modes omit the driving status row.
        self._temperature_badge(DESIGN_WIDTH - 66, TOP_STATUS_CENTER_Y)
      message, color = health_banner(self.host_health, self.language)
      vehicle_alert = alert is not None and alert.size > 0 and (alert.text1 or alert.text2)
      if message and not vehicle_alert:
        rl.draw_rectangle(0, 0, int(DESIGN_WIDTH), 32, rl_color((0, 0, 0), 220))
        self._draw_text(self._ellipsize_text(message, 22, DESIGN_WIDTH - 40),
                        DESIGN_WIDTH / 2, 16, 22, color, anchor='center')
    finally:
      rl.rl_pop_matrix()
    # Vehicle alerts remain last and take precedence over the health strip.
    super()._draw_alert_overlay(alert)
