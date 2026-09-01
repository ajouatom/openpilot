import threading

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.usbgpu import check_usbgpu, get_usbgpu_device, usbgpu_status
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


class UsbGpuLayout(Widget):
  def __init__(self):
    super().__init__()
    self._checking = False
    self._check_result = "not checked"
    self._scroller = Scroller([
      text_item(lambda: tr("eGPU Status"), self._status),
      text_item(lambda: tr("USB Link"), self._link),
      text_item(lambda: tr("Connection Check"), lambda: self._check_result),
      button_item(lambda: tr("Check eGPU"), lambda: tr("CHECK"),
                  lambda: tr("Checks USB 5 Gbps, firmware, 12V/PCIe, and GPU execution."),
                  callback=self._start_check, enabled=ui_state.is_offroad),
    ], line_separator=True, spacing=0)

  @staticmethod
  def _status() -> str:
    return usbgpu_status(ui_state.usbgpu_compiled, ui_state.usbgpu_loading, ui_state.usbgpu_active,
                         ui_state.usbgpu_startup_failed, compile_pending=ui_state.usbgpu_compile_pending)

  @staticmethod
  def _link() -> str:
    device = get_usbgpu_device()
    if device is None:
      return "not connected"
    return f"{device.speed_mbps // 1000} Gbps" if device.speed_mbps >= 1000 and device.speed_mbps % 1000 == 0 \
      else f"{device.speed_mbps} Mbps"

  def _start_check(self):
    if self._checking or not ui_state.is_offroad():
      return
    self._checking = True
    self._check_result = "checking..."

    def run():
      try:
        result = check_usbgpu()
        self._check_result = "no errors" if result is None else result
      finally:
        self._checking = False

    threading.Thread(target=run, name="usbgpu-check", daemon=True).start()

  def show_event(self):
    super().show_event()
    self._scroller.show_event()

  def _render(self, rect):
    self._scroller.render(rect)
