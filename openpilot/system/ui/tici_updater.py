#!/usr/bin/env python3
import re
import sys
import subprocess
import threading
from collections import deque
import pyray as rl
from enum import IntEnum

from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.tici.agnos import mark_update_confirmed, update_confirmed
from openpilot.system.ui.lib.application import gui_app, FontWeight, FONT_SCALE
from openpilot.system.ui.lib.wifi_manager import WifiManager
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.label import gui_text_box, gui_label
from openpilot.system.ui.widgets.network import WifiManagerUI

# Constants
MARGIN = 50
BUTTON_HEIGHT = 160
BUTTON_WIDTH = 400
PROGRESS_BAR_HEIGHT = 72
TITLE_FONT_SIZE = 80
BODY_FONT_SIZE = 65
BACKGROUND_COLOR = rl.BLACK
PROGRESS_BG_COLOR = rl.Color(41, 41, 41, 255)
PROGRESS_COLOR = rl.Color(54, 77, 239, 255)


class Screen(IntEnum):
  PROMPT = 0
  WIFI = 1
  PROGRESS = 2


class Updater(Widget):
  def __init__(self, updater_path, manifest_path):
    super().__init__()
    self.updater = updater_path
    self.manifest = manifest_path
    self.current_screen = Screen.PROMPT

    self.progress_value = 0
    self.progress_text = "Loading..."
    self.show_reboot_button = False
    self.failure_detail = ""
    self.process = None
    self.update_thread = None
    self._last_output: deque[str] = deque(maxlen=12)
    self.wifi_manager_ui = WifiManagerUI(WifiManager())

    # Buttons
    self._wifi_button = Button("Connect to Wi-Fi", click_callback=lambda: self.set_current_screen(Screen.WIFI))
    self._install_button = Button("Install", click_callback=self.install_update, button_style=ButtonStyle.PRIMARY)
    self._back_button = Button("Back", click_callback=lambda: self.set_current_screen(Screen.PROMPT))
    self._retry_button = Button("Retry", click_callback=self.install_update, button_style=ButtonStyle.PRIMARY)
    self._reboot_button = Button("Reboot", click_callback=lambda: HARDWARE.reboot())

  def set_current_screen(self, screen: Screen):
    self.current_screen = screen

  def install_update(self):
    if self.update_thread is not None and self.update_thread.is_alive():
      return
    try:
      mark_update_confirmed(self.manifest)
    except OSError as e:
      self.set_current_screen(Screen.PROGRESS)
      self.progress_text = "Update paused"
      self.failure_detail = f"Unable to save confirmation: {e}"
      self.show_reboot_button = True
      return

    self.set_current_screen(Screen.PROGRESS)
    self.progress_value = 0
    self.progress_text = "Starting update..."
    self.show_reboot_button = False
    self.failure_detail = ""
    self._last_output.clear()

    # Start the update process in a separate thread
    self.update_thread = threading.Thread(target=self._run_update_process)
    self.update_thread.daemon = True
    self.update_thread.start()

  def _run_update_process(self):
    # TODO: just import it and run in a thread without a subprocess
    try:
      cmd = [self.updater, "--swap", self.manifest]
      self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                      stderr=subprocess.STDOUT, text=True, bufsize=1, universal_newlines=True)
    except Exception as e:
      self.progress_text = "Update failed"
      self.failure_detail = f"Unable to start updater: {e}"
      self.show_reboot_button = True
      return

    if self.process.stdout is not None:
      for line in self.process.stdout:
        self._handle_process_line(line)

    exit_code = self.process.wait()
    if exit_code == 0:
      self.progress_text = "Rebooting..."
      self.progress_value = 100
      HARDWARE.reboot()
    else:
      self.progress_text = "Update failed"
      detail = self._last_output[-1] if self._last_output else "no error output"
      self.failure_detail = f"Updater exited with code {exit_code}: {detail}"
      self.show_reboot_button = True

  def _handle_process_line(self, line: str):
    text = line.strip()
    if not text:
      return
    print(text, flush=True)
    self._last_output.append(text[-240:])
    match = re.fullmatch(r"(.+):\s*(\d{1,3})", text)
    if match is not None:
      self.progress_text = match.group(1)
      self.progress_value = max(0, min(int(match.group(2)), 100))

  def close(self):
    if self.process is not None and self.process.poll() is None:
      self.process.terminate()
      try:
        self.process.wait(timeout=5)
      except subprocess.TimeoutExpired:
        self.process.kill()
        self.process.wait()

  def render_prompt_screen(self, rect: rl.Rectangle):
    # Title
    title_rect = rl.Rectangle(MARGIN + 50, 250, rect.width - MARGIN * 2 - 100, TITLE_FONT_SIZE * FONT_SCALE)
    gui_label(title_rect, "Update Required", TITLE_FONT_SIZE, font_weight=FontWeight.BOLD)

    # Description
    desc_text = ("An operating system update is required. Connect your device to Wi-Fi for the fastest update experience. " +
                 "The download size is approximately 1GB.")

    desc_rect = rl.Rectangle(MARGIN + 50, 250 + TITLE_FONT_SIZE * FONT_SCALE + 75, rect.width - MARGIN * 2 - 100, BODY_FONT_SIZE * FONT_SCALE * 4)
    gui_text_box(desc_rect, desc_text, BODY_FONT_SIZE)

    # Buttons at the bottom
    button_y = rect.height - MARGIN - BUTTON_HEIGHT
    button_width = (rect.width - MARGIN * 3) // 2

    # WiFi button
    wifi_button_rect = rl.Rectangle(MARGIN, button_y, button_width, BUTTON_HEIGHT)
    self._wifi_button.render(wifi_button_rect)

    # Install button
    install_button_rect = rl.Rectangle(MARGIN * 2 + button_width, button_y, button_width, BUTTON_HEIGHT)
    self._install_button.render(install_button_rect)

  def render_wifi_screen(self, rect: rl.Rectangle):
    # Draw the Wi-Fi manager UI
    wifi_rect = rl.Rectangle(rect.x + MARGIN, rect.y + MARGIN, rect.width - MARGIN * 2,
                             rect.height - BUTTON_HEIGHT - MARGIN * 3)
    rl.draw_rectangle_rounded(wifi_rect, 0.035, 10, rl.Color(51, 51, 51, 255))
    wifi_content_rect = rl.Rectangle(wifi_rect.x + 50, wifi_rect.y, wifi_rect.width - 100, wifi_rect.height)
    self.wifi_manager_ui.render(wifi_content_rect)

    back_button_rect = rl.Rectangle(MARGIN, rect.height - MARGIN - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
    self._back_button.render(back_button_rect)

  def render_progress_screen(self, rect: rl.Rectangle):
    title_rect = rl.Rectangle(MARGIN + 100, 330, rect.width - MARGIN * 2 - 200, 100)
    gui_label(title_rect, self.progress_text, 90, font_weight=FontWeight.SEMI_BOLD)

    # Progress bar
    bar_rect = rl.Rectangle(MARGIN + 100, 330 + 100 + 100, rect.width - MARGIN * 2 - 200, PROGRESS_BAR_HEIGHT)
    rl.draw_rectangle_rounded(bar_rect, 0.5, 10, PROGRESS_BG_COLOR)

    # Calculate the width of the progress chunk
    progress_width = (bar_rect.width * self.progress_value) / 100
    if progress_width > 0:
      progress_rect = rl.Rectangle(bar_rect.x, bar_rect.y, progress_width, bar_rect.height)
      rl.draw_rectangle_rounded(progress_rect, 0.5, 10, PROGRESS_COLOR)

    # Show reboot button if needed
    if self.show_reboot_button:
      detail_rect = rl.Rectangle(MARGIN + 100, 650, rect.width - MARGIN * 2 - 200, BODY_FONT_SIZE * FONT_SCALE * 2)
      gui_text_box(detail_rect, self.failure_detail, BODY_FONT_SIZE)
      retry_rect = rl.Rectangle(MARGIN + 100, rect.height - MARGIN - BUTTON_HEIGHT, BUTTON_WIDTH, BUTTON_HEIGHT)
      reboot_rect = rl.Rectangle(rect.width - MARGIN - 100 - BUTTON_WIDTH, rect.height - MARGIN - BUTTON_HEIGHT,
                                 BUTTON_WIDTH, BUTTON_HEIGHT)
      self._retry_button.render(retry_rect)
      self._reboot_button.render(reboot_rect)

  def _render(self, rect: rl.Rectangle):
    if self.current_screen == Screen.PROMPT:
      self.render_prompt_screen(rect)
    elif self.current_screen == Screen.WIFI:
      self.render_wifi_screen(rect)
    elif self.current_screen == Screen.PROGRESS:
      self.render_progress_screen(rect)


def main():
  if len(sys.argv) < 3:
    print("Usage: updater.py <updater_path> <manifest_path>")
    sys.exit(1)

  updater_path = sys.argv[1]
  manifest_path = sys.argv[2]

  updater = None
  try:
    # This UI can run from a clean source checkout before font atlases exist.
    gui_app.init_window("System Update", font_weights=(
      FontWeight.NORMAL, FontWeight.MEDIUM, FontWeight.BOLD, FontWeight.SEMI_BOLD,
    ))
    updater = Updater(updater_path, manifest_path)
    gui_app.push_widget(updater)
    if update_confirmed(manifest_path):
      updater.install_update()
    for _ in gui_app.render():
      pass
  finally:
    # Make sure we clean up even if there's an error
    if updater is not None:
      updater.close()
    gui_app.close()


if __name__ == "__main__":
  main()
