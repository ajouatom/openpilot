import math
import pyray as rl
from openpilot.system.ui.lib.application import FONT_SCALE, FontWeight, font_fallback, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached


_OUTLINE_UNIT_OFFSETS = tuple(
  (math.cos(math.radians(deg)), math.sin(math.radians(deg)))
  for deg in range(0, 360, 45)
)
_RAW_DRAW_TEXT_EX = getattr(getattr(rl, "rl", None), "DrawTextEx", None)


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
  draw_size = float(font_size)
  position = rl.Vector2(float(draw_x), float(draw_y))

  # The pyray wrapper encodes Python strings and converts every argument on each
  # call. Styled text can submit the same string up to ten times, so use the raw
  # CFFI symbol when available and prepare its immutable arguments once.
  draw_text_ex = _RAW_DRAW_TEXT_EX
  if draw_text_ex is not None:
    font = font_fallback(font)
    draw_text = text.encode("utf-8")
    draw_size *= FONT_SCALE
  else:
    draw_text_ex = rl.draw_text_ex
    draw_text = text

  if border_width > 0.0:
    for unit_x, unit_y in _OUTLINE_UNIT_OFFSETS:
      position.x = float(draw_x + border_width * unit_x)
      position.y = float(draw_y + border_width * unit_y)
      draw_text_ex(font, draw_text, position, draw_size, 0, border_color)

  if shadow_offset != 0.0:
    position.x = float(draw_x + shadow_offset)
    position.y = float(draw_y + shadow_offset)
    draw_text_ex(font, draw_text, position, draw_size, 0, shadow_color)

  position.x = float(draw_x)
  position.y = float(draw_y)
  draw_text_ex(font, draw_text, position, draw_size, 0, color)

