import threading
from pathlib import Path

import pyray as rl

from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld.helpers import active_usbgpu_compiled_path
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.usbgpu import check_usbgpu, get_usbgpu_device, usbgpu_status
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.scroller import NavScroller


class UsbGpuInfoLayout(Widget):
  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, 360, 180))
    sub_color = rl.Color(255, 255, 255, int(255 * 0.9 * 0.65))
    self._status = UnifiedLabel("status", 48, max_width=340, font_weight=FontWeight.DISPLAY, wrap_text=False)
    self._status_value = UnifiedLabel("", 32, max_width=340, text_color=sub_color, font_weight=FontWeight.ROMAN, wrap_text=False)
    self._link = UnifiedLabel("USB link", 48, max_width=340, font_weight=FontWeight.DISPLAY, wrap_text=False)
    self._link_value = UnifiedLabel("", 32, max_width=340, text_color=sub_color, font_weight=FontWeight.ROMAN, wrap_text=False)

  def _update_state(self):
    self._status_value.set_text(usbgpu_status(ui_state.usbgpu_compiled, ui_state.usbgpu_loading, ui_state.usbgpu_active,
                                              ui_state.usbgpu_startup_failed,
                                              compile_pending=ui_state.usbgpu_compile_pending))
    device = get_usbgpu_device()
    if device is None:
      value = "not connected"
    elif device.speed_mbps >= 1000 and device.speed_mbps % 1000 == 0:
      value = f"{device.speed_mbps // 1000} Gbps"
    else:
      value = f"{device.speed_mbps} Mbps"
    self._link_value.set_text(value)

  def _render(self, _):
    x, y = self._rect.x + 20, self._rect.y
    for label, offset in ((self._status, -10), (self._status_value, 43), (self._link, 84), (self._link_value, 136)):
      label.set_position(x, y + offset)
      label.render()


class CheckUsbGpuButton(BigButton):
  def __init__(self):
    super().__init__("check connection", "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56))
    self._running = False
    self._result: str | None = None

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    if ui_state.started or self._running:
      return
    self._running = True
    self._result = None
    self.set_value("checking...")
    self.set_rotate_icon(True)

    def run():
      try:
        self._result = check_usbgpu()
      finally:
        self._running = False

    threading.Thread(target=run, name="usbgpu-check", daemon=True).start()

  def _update_state(self):
    super()._update_state()
    if not self._running and self.get_value() == "checking...":
      self.set_rotate_icon(False)
      self.set_value("no errors" if self._result is None else self._result)
    self.set_enabled(not ui_state.started and not self._running)


class UsbGpuLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    compile_button = BigButton("compile model", "keep ignition on",
                               gui_app.texture("icons_mici/settings/device/reboot.png", 64, 70))
    compile_button.set_visible(lambda: not ui_state.usbgpu_compiled)
    compile_button.set_enabled(lambda: not ui_state.started)
    compile_button.set_click_callback(self._force_compile)
    self._scroller.add_widgets([UsbGpuInfoLayout(), CheckUsbGpuButton(), compile_button])

  @staticmethod
  def _force_compile():
    def reboot():
      path = active_usbgpu_compiled_path()
      if path is not None:
        Path(get_manifest_path(path)).unlink(missing_ok=True)
      ui_state.params.put_bool("DoReboot", True, block=True)

    icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 70)
    gui_app.push_widget(BigConfirmationDialog("slide to reboot and compile", icon, reboot, exit_on_confirm=False))
