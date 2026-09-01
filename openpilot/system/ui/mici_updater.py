#!/usr/bin/env python3
import os
import re
import sys
import subprocess
import threading
from collections import deque
import pyray as rl

from openpilot.common.realtime import config_realtime_process, set_core_affinity
from openpilot.system.hardware.tici.agnos import (manifest_download_urls, mark_update_confirmed,
                                                  update_confirmed)
from openpilot.system.hardware import HARDWARE, TICI
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets.nav_widget import NavWidget
from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.mici_setup import (NetworkSetupPage, FailedPage, NetworkConnectivityMonitor,
                                            GreyBigButton, BigPillButton)


class UpdaterNetworkSetupPage(NetworkSetupPage):
  def __init__(self, network_monitor, continue_callback):
    super().__init__(network_monitor, continue_callback, back_callback=None)
    self._continue_button.set_text("download\n& install")
    self._continue_button.set_green(False)

  def _nav_stack_tick(self):
    super()._nav_stack_tick()
    has_internet = self._has_internet
    # The downloader has its own bounded retry and resume handling. Keep an
    # explicit install action available when a connectivity probe is blocked
    # even though the actual manifest hosts are reachable.
    self._continue_button.set_visible(True)
    self._continue_button.set_text("download\n& install" if has_internet else "try download\n& install")
    if not has_internet:
      self._waiting_button.set_text("update server\nnot ready")


class ProgressPage(NavWidget):
  def __init__(self):
    super().__init__()

    self._progress_title_label = UnifiedLabel("", 64, text_color=rl.Color(255, 255, 255, int(255 * 0.9)),
                                              font_weight=FontWeight.DISPLAY, line_height=0.8)
    self._progress_percent_label = UnifiedLabel("", 132, text_color=rl.Color(255, 255, 255, int(255 * 0.9 * 0.65)),
                                                font_weight=FontWeight.ROMAN,
                                                alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_BOTTOM)

  def _back_enabled(self) -> bool:
    return False

  def set_progress(self, text: str, value: int):
    self._progress_title_label.set_text(text.replace("_", "_\n") + "...")
    self._progress_percent_label.set_text(f"{value}%")

  def show_event(self):
    super().show_event()
    self._nav_bar._alpha = 0.0  # not dismissable
    self.set_progress("downloading", 0)

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, rl.BLACK)
    self._progress_title_label.render(rl.Rectangle(
      rect.x + 12,
      rect.y + 2,
      rect.width,
      self._progress_title_label.get_content_height(int(rect.width - 20)),
    ))

    self._progress_percent_label.render(rl.Rectangle(
      rect.x + 12,
      rect.y + 18,
      rect.width,
      rect.height,
    ))


class Updater(Scroller):
  def __init__(self, updater_path, manifest_path):
    super().__init__()
    self.updater = updater_path
    self.manifest = manifest_path

    self.progress_value = 0
    self.progress_text = "loading"
    self.process = None
    self.update_thread = None
    self._failure_reason: str | None = None
    self._last_output: deque[str] = deque(maxlen=12)

    self._network_monitor = NetworkConnectivityMonitor(probe_urls=manifest_download_urls(manifest_path))
    self._network_monitor.start()

    self._network_setup_page = UpdaterNetworkSetupPage(self._network_monitor, self._network_setup_continue_callback)

    self._progress_page = ProgressPage()

    self._failed_page = FailedPage(self._retry, title="update failed")

    self._continue_button = BigPillButton("next")
    self._continue_button.set_click_callback(lambda: gui_app.push_widget(self._network_setup_page))

    self._scroller.add_widgets([
      GreyBigButton("update required", "the download size\nis approximately 1 GB",
                    gui_app.texture("icons_mici/offroad_alerts/green_wheel.png", 64, 64)),
      self._continue_button,
    ])

    gui_app.add_nav_stack_tick(self._nav_stack_tick)

  def _network_setup_continue_callback(self, _):
    self.install_update()

  def _retry(self):
    gui_app.pop_widgets_to(self)

  def _nav_stack_tick(self):
    self._progress_page.set_progress(self.progress_text, self.progress_value)

    if self._failure_reason is not None:
      reason = self._failure_reason
      self._failure_reason = None
      self._failed_page.set_reason(reason)
      self.show_event()
      gui_app.pop_widgets_to(self, lambda: gui_app.push_widget(self._failed_page))

  def install_update(self):
    if self.update_thread is not None and self.update_thread.is_alive():
      return
    try:
      mark_update_confirmed(self.manifest)
    except OSError as e:
      self._failure_reason = f"unable to save update confirmation: {e}"
      return

    self.progress_value = 0
    self.progress_text = "starting update"
    self._last_output.clear()

    def start_update():
      self.update_thread = threading.Thread(target=self._run_update_process, daemon=True)
      self.update_thread.start()

    # Start the update process in a separate thread *after* show animation completes
    self._progress_page.set_shown_callback(start_update)
    gui_app.push_widget(self._progress_page)

  def _run_update_process(self):
    # TODO: just import it and run in a thread without a subprocess
    try:
      cmd = [self.updater, "--swap", self.manifest]
      self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
    except Exception as e:
      self._failure_reason = f"unable to start updater: {e}"
      return

    if self.process.stdout is not None:
      for line in self.process.stdout:
        self._handle_process_line(line)

    exit_code = self.process.wait()
    if exit_code == 0:
      self.progress_text = "rebooting"
      self.progress_value = 100
      HARDWARE.reboot()
    else:
      detail = self._last_output[-1] if self._last_output else "no error output"
      self._failure_reason = f"updater exited with code {exit_code}: {detail}"

  def _handle_process_line(self, line: str):
    text = line.strip()
    if not text:
      return
    print(text, flush=True)
    self._last_output.append(text[-240:])
    match = re.fullmatch(r"(.+):\s*(\d{1,3})", text)
    if match is not None:
      self.progress_text = match.group(1).lower()
      self.progress_value = max(0, min(int(match.group(2)), 100))

  def close(self):
    self._network_monitor.stop()
    if self.process is not None and self.process.poll() is None:
      self.process.terminate()
      try:
        self.process.wait(timeout=5)
      except subprocess.TimeoutExpired:
        self.process.kill()
        self.process.wait()


def main():
  config_realtime_process(0, 51)
  # attempt to affine. AGNOS will start setup with all cores, should only fail when manually launching with screen off
  if TICI:
    try:
      set_core_affinity([5])
    except OSError:
      cores = sorted(os.sched_getaffinity(0))
      cloudlog.warning(f"Skipping core 5 affinity for updater process, available cores: {cores}")

  if len(sys.argv) < 3:
    print("Usage: updater.py <updater_path> <manifest_path>")
    sys.exit(1)

  updater_path = sys.argv[1]
  manifest_path = sys.argv[2]

  updater = None
  try:
    gui_app.init_window("System Update")
    updater = Updater(updater_path, manifest_path)
    gui_app.push_widget(updater)
    if update_confirmed(manifest_path):
      updater.install_update()
    for _ in gui_app.render():
      pass
  except Exception as e:
    print(f"Updater error: {e}")
    raise
  finally:
    if updater is not None:
      updater.close()
    gui_app.close()


if __name__ == "__main__":
  main()
