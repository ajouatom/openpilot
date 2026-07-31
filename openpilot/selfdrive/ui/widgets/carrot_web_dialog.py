import pyray as rl

from openpilot.selfdrive.ui.carrot_web import CarrotWebQrSession, fit_single_line_font_size
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, MousePos, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label


class CarrotWebDialog(Widget):
  def __init__(self):
    super().__init__()
    self._session = CarrotWebQrSession(ui_state.params_memory)

  def show_event(self):
    super().show_event()
    self._session.open(rl.get_time())

  def _update_state(self):
    if self._session.update(rl.get_time()):
      gui_app.pop_widget()

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._session.can_close(rl.get_time()):
      gui_app.pop_widget()

  def _render(self, rect: rl.Rectangle):
    rl.clear_background(rl.BLACK)

    scale = min(rect.width / 536, rect.height / 240)
    content_x = rect.x + (rect.width - 536 * scale) / 2
    content_y = rect.y + (rect.height - 240 * scale) / 2

    qr_rect = rl.Rectangle(
      content_x + 8 * scale,
      content_y + 8 * scale,
      224 * scale,
      224 * scale,
    )

    if not self._session.qr.render(qr_rect):
      rl.draw_rectangle_rounded(qr_rect, 0.04, 8, rl.Color(38, 38, 38, 255))
      gui_label(qr_rect, tr("Offline"), int(32 * scale), rl.Color(175, 175, 175, 255))

    text_x = content_x + 254 * scale
    text_width = 274 * scale
    gui_label(
      rl.Rectangle(text_x, content_y + 42 * scale, text_width, 52 * scale),
      "Carrot Web",
      int(38 * scale),
      rl.WHITE,
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
      font_weight=FontWeight.BOLD,
    )

    address_text = (self._session.url or tr("Offline")).removeprefix("http://")
    preferred_address_size = int(26 * scale)
    address_font = gui_app.font()
    address_size = fit_single_line_font_size(
      preferred_address_size,
      measure_text_cached(address_font, address_text, preferred_address_size).x,
      text_width,
    )
    while address_size > 1 and measure_text_cached(address_font, address_text, address_size).x > text_width:
      address_size -= 1

    gui_label(
      rl.Rectangle(text_x, content_y + 104 * scale, text_width, 36 * scale),
      address_text,
      address_size,
      rl.Color(205, 205, 205, 255),
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_TOP,
      elide_right=False,
    )

    if self._session.updated_time is not None:
      footer_rect = rl.Rectangle(text_x, content_y + 190 * scale, text_width, 28 * scale)
      gui_label(
        footer_rect,
        self._session.updated_time,
        int(20 * scale),
        rl.Color(135, 135, 135, 255),
        alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
      )
      gui_label(
        footer_rect,
        f"{self._session.seconds_until_close(rl.get_time())}s",
        int(20 * scale),
        rl.Color(135, 135, 135, 255),
        alignment=rl.GuiTextAlignment.TEXT_ALIGN_RIGHT,
      )

  def __del__(self):
    self._session.destroy()
