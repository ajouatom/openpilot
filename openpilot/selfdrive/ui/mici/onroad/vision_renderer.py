import time

import pyray as rl

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.vision_status import blindspot_source_packet, blindspot_sources, parse_vision_display_packet, vision_display_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget


GRAY = rl.Color(160, 168, 177, 255)
CYAN = rl.Color(100, 220, 255, 255)
AMBER = rl.Color(255, 215, 0, 255)
BLUE = rl.Color(64, 156, 255, 255)
LIGHT_RED = rl.Color(255, 128, 128, 255)


class VisionRenderer(Widget):
  """Small status card and persistent BSD warnings, also with the camera hidden."""

  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.DISPLAY)
    self._packet = None

  def _update_state(self):
    sm = ui_state.sm
    if not ui_state.share_data or not ui_state.started:
      self._packet = None
    elif sm.updated['customReservedRawData0']:
      try:
        self._packet = parse_vision_display_packet(bytes(sm['customReservedRawData0'])) if sm.valid['customReservedRawData0'] else None
      except (UnicodeDecodeError, ValueError, TypeError):
        self._packet = None

  def _text(self, text: str, x: float, y: float, color, size: int = 13, max_width: float = 110):
    measured = measure_text_cached(self._font, text, size)
    fitted_size = size * min(1.0, max_width / max(measured.x, 1.0))
    rl.draw_text_ex(self._font, text, rl.Vector2(x, y), fitted_size, 0, color)

  def _lane_icon(self, x: float, y: float, lane_type: int):
    if lane_type < 0:
      self._text("?", x + 4, y - 1, GRAY, 16)
    elif lane_type == 1:
      rl.draw_line_ex(rl.Vector2(x, y + 14), rl.Vector2(x + 12, y), 2, rl.WHITE)
    else:
      for offset in (0, 6, 12):
        rl.draw_line_ex(rl.Vector2(x + offset * 0.85, y + 14 - offset), rl.Vector2(x + (offset + 3) * 0.85, y + 11 - offset), 2, rl.WHITE)

  def _render(self, rect: rl.Rectangle):
    if not ui_state.started:
      return
    sm = ui_state.sm
    car_fresh = (sm.valid['carState'] and sm.alive['carState'] and sm.recv_frame['carState'] >= ui_state.started_frame)
    car_state = sm['carState'] if car_fresh else None
    left = bool(car_state and car_state.leftBlindspot)
    right = bool(car_state and car_state.rightBlindspot)
    source_packet = blindspot_source_packet(sm, time.monotonic_ns())

    source_colors = {}
    for side, active, is_left in (("left", left, True), ("right", right, False)):
      if active:
        oem, vision = blindspot_sources(car_state, side, source_packet)
        source_color = LIGHT_RED if oem and vision else BLUE if vision else AMBER
        source_colors[side] = source_color
        x = rect.x + 6 if is_left else rect.x + rect.width - 12
        rl.draw_rectangle_rounded(rl.Rectangle(x, rect.y + 78, 6, 56), 0.8, 6, source_color)
        self._text("BSD", rect.x + 16 if is_left else rect.x + rect.width - 45, rect.y + 84, source_color, 12)

    if not ui_state.share_data:
      return
    state = vision_display_state(self._packet, time.monotonic_ns())
    # Above the DM inset, below eGPU, beside the relocated right BSD label/bar.
    card = rl.Rectangle(rect.x + rect.width - 94, rect.y + rect.height - 184, 84, 78)
    rl.draw_rectangle_rounded(card, 0.14, 6, rl.Color(0, 0, 0, 190))
    color = CYAN if state.state == "running" else AMBER if state.state == "stale" else GRAY
    self._text("VISION", card.x + 6, card.y + 5, color, 10, 40)
    status_labels = {"running": tr("ON"), "waiting": tr("WAIT"), "stale": tr("STALE")}
    status = f"{state.latency_ms / 1000:.1f}s" if state.latency_ms is not None else status_labels[state.state]
    self._text(status, card.x + 49, card.y + 5, color, 10, 29)
    self._text("L", card.x + 6, card.y + 24, GRAY)
    self._lane_icon(card.x + 22, card.y + 23, state.left_lane)
    self._text("R", card.x + 44, card.y + 24, GRAY)
    self._lane_icon(card.x + 60, card.y + 23, state.right_lane)

    label = ""
    if left or right:
      label = "L+R" if left and right else "L" if left else "R"
      active_colors = [source_colors[side] for side in ("left", "right") if side in source_colors]
      color = LIGHT_RED if any(item.r == LIGHT_RED.r and item.g == LIGHT_RED.g for item in active_colors) else \
              BLUE if any(item.r == BLUE.r and item.g == BLUE.g for item in active_colors) else AMBER
      text = tr("DETECTED")
    elif car_fresh and state.clear_side:
      label = "L" if state.clear_side == "left" else "R"
      text, color = tr("NO DETECTION"), CYAN
    else:
      text, color = tr("STANDBY"), GRAY
    self._text(f"BSD {label}".rstrip(), card.x + 6, card.y + 44, color, 11, 72)
    self._text(text, card.x + 6, card.y + 59, color, 11, 72)
