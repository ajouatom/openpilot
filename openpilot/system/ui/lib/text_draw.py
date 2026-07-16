import math
import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached


_OUTLINE_UNIT_OFFSETS = tuple(
  (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
  for deg in range(0, 360, 45)
)


def draw_text_raw(font, text, x, y, font_size, color):
  rl.draw_text_ex(
    font,
    text,
    rl.Vector2(float(x), float(y)),
    float(font_size),
    0,
    color,
  )


def get_text_draw_pos(font, text, x, y, font_size, align="center_bottom", y_offset=6.0):
  text_size = measure_text_cached(font, text, font_size)

  draw_x = x
  draw_y = y + y_offset

  if align == "center_bottom":
    draw_x = x - text_size.x * 0.5
    draw_y = (y + y_offset) - text_size.y
  elif align == "center_top":
    draw_x = x - text_size.x * 0.5
    draw_y = y + y_offset
  elif align == "left_top":
    draw_x = x
    draw_y = y + y_offset
  elif align == "right_top":
    draw_x = x - text_size.x
    draw_y = y + y_offset
  elif align == "left_center":
    draw_x = x
    draw_y = y + y_offset - text_size.y * 0.5
  elif align == "center":
    draw_x = x - text_size.x * 0.5
    draw_y = y + y_offset - text_size.y * 0.5
  elif align == "right_center":
    draw_x = x - text_size.x
    draw_y = y + y_offset - text_size.y * 0.5

  return draw_x, draw_y, text_size


def draw_text_ui_style(text: str,
                       x: float,
                       y: float,
                       font_size: float,
                       color: rl.Color,
                       font=None,
                       border_width: float = 3.0,
                       shadow_offset: float = 8.0,
                       border_color: rl.Color = rl.BLACK,
                       shadow_color: rl.Color = rl.BLACK,
                       align: str = "center_bottom",
                       y_offset: float = 6.0) -> None:
  if not text:
    return

  if font is None:
    font = gui_app.font(FontWeight.DISPLAY)

  draw_x, draw_y, _ = get_text_draw_pos(font, text, x, y, font_size, align, y_offset)

  if border_width > 0.0:
    for unit_x, unit_y in _OUTLINE_UNIT_OFFSETS:
      ox = border_width * unit_x
      oy = border_width * unit_y
      draw_text_raw(font, text, draw_x + ox, draw_y + oy, font_size, border_color)

  if shadow_offset != 0.0:
    draw_text_raw(font, text, draw_x + shadow_offset, draw_y + shadow_offset, font_size, shadow_color)

  draw_text_raw(font, text, draw_x, draw_y, font_size, color)

