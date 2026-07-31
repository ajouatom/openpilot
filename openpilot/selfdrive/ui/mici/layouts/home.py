import datetime
import time

from openpilot.cereal import log
import pyray as rl
from collections.abc import Callable
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.layouts import HBoxLayout
from openpilot.system.ui.widgets.icon_widget import IconWidget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.version import RELEASE_BRANCHES

HEAD_BUTTON_FONT_SIZE = 40
HOME_PADDING = 8

NetworkType = log.DeviceState.NetworkType

NETWORK_TYPES = {
  NetworkType.none: "Offline",
  NetworkType.wifi: "WiFi",
  NetworkType.cell2G: "2G",
  NetworkType.cell3G: "3G",
  NetworkType.cell4G: "LTE",
  NetworkType.cell5G: "5G",
  NetworkType.ethernet: "Ethernet",
}


class NetworkIcon(Widget):
  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, 54, 44))  # max size of all icons
    self._net_type = NetworkType.none
    self._net_strength = 0

    self._wifi_slash_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_slash.png", 50, 44)
    self._wifi_none_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_none.png", 50, 37)
    self._wifi_low_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_low.png", 50, 37)
    self._wifi_medium_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_medium.png", 50, 37)
    self._wifi_full_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 50, 37)

    self._cell_none_txt = gui_app.texture("icons_mici/settings/network/cell_strength_none.png", 54, 36)
    self._cell_low_txt = gui_app.texture("icons_mici/settings/network/cell_strength_low.png", 54, 36)
    self._cell_medium_txt = gui_app.texture("icons_mici/settings/network/cell_strength_medium.png", 54, 36)
    self._cell_high_txt = gui_app.texture("icons_mici/settings/network/cell_strength_high.png", 54, 36)
    self._cell_full_txt = gui_app.texture("icons_mici/settings/network/cell_strength_full.png", 54, 36)

  def _update_state(self):
    device_state = ui_state.sm['deviceState']
    self._net_type = device_state.networkType
    strength = device_state.networkStrength
    self._net_strength = max(0, min(5, strength.raw + 1)) if strength.raw > 0 else 0

  def _render(self, _):
    if self._net_type == NetworkType.wifi:
      # There is no 1
      draw_net_txt = {0: self._wifi_none_txt,
                      2: self._wifi_low_txt,
                      3: self._wifi_medium_txt,
                      4: self._wifi_full_txt,
                      5: self._wifi_full_txt}.get(self._net_strength, self._wifi_low_txt)
    elif self._net_type in (NetworkType.cell2G, NetworkType.cell3G, NetworkType.cell4G, NetworkType.cell5G):
      draw_net_txt = {0: self._cell_none_txt,
                      2: self._cell_low_txt,
                      3: self._cell_medium_txt,
                      4: self._cell_high_txt,
                      5: self._cell_full_txt}.get(self._net_strength, self._cell_none_txt)
    else:
      draw_net_txt = self._wifi_slash_txt

    draw_x = self._rect.x + (self._rect.width - draw_net_txt.width) / 2
    draw_y = self._rect.y + (self._rect.height - draw_net_txt.height) / 2

    if draw_net_txt == self._wifi_slash_txt:
      # Offset by difference in height between slashless and slash icons to make center align match
      draw_y -= (self._wifi_slash_txt.height - self._wifi_none_txt.height) / 2

    rl.draw_texture_ex(draw_net_txt, rl.Vector2(draw_x, draw_y), 0.0, 1.0, rl.Color(255, 255, 255, int(255 * 0.9)))


class MiciHomeLayout(Widget):
  def __init__(self):
    super().__init__()
    self._on_settings_click: Callable | None = None
    self._on_carrot_web_click: Callable | None = None

    self._last_refresh = 0
    self._mouse_down_t: None | float = None
    self._did_long_press = False
    self._is_pressed_prev = False
    self._carrot_web_pressed = False

    self._version_text = None
    self._experimental_mode = False

    self._ip_address = "Offline"

    self._settings_icon = IconWidget("icons_mici/settings.png", (48, 48), opacity=0.9)
    self._carrot_web_icon = IconWidget("icons/carrot_web.png", (48, 48), opacity=0.9)
    self._experimental_icon = IconWidget("icons_mici/experimental_mode.png", (48, 48))
    self._mic_icon = IconWidget("icons_mici/microphone.png", (32, 46))

    self._status_bar_layout = HBoxLayout([
      IconWidget("icons_mici/settings.png", (48, 48), opacity=0.9),
      NetworkIcon(),
      self._carrot_web_icon,
      self._experimental_icon,
      self._mic_icon,
    ], spacing=18)

    self._openpilot_label = UnifiedLabel("carrotpilot", font_size=55, font_weight=FontWeight.DISPLAY, max_width=480, wrap_text=False)
    self._version_label = UnifiedLabel("", font_size=30, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)
    self._large_version_label = UnifiedLabel("", font_size=64, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)
    self._date_label = UnifiedLabel("", font_size=30, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)
    self._branch_label = UnifiedLabel("", font_size=30, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, scroll=True)
    self._ip_label = UnifiedLabel("", font_size=30, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)

  def show_event(self):
    super().show_event()
    self._version_text = self._get_version_text()
    self._update_params()

  def _update_params(self):
    self._experimental_mode = ui_state.params.get_bool("ExperimentalMode")

  @staticmethod
  def _read_network_address(params_memory) -> str:
    address = (params_memory.get("NetworkAddress") or "").strip()
    return address if address and address != "0.0.0.0" else "Offline"

  def _update_state(self):
    if self.is_pressed and not self._is_pressed_prev:
      self._mouse_down_t = time.monotonic()
    elif not self.is_pressed and self._is_pressed_prev:
      self._mouse_down_t = None
      self._did_long_press = False
    self._is_pressed_prev = self.is_pressed

    if self._mouse_down_t is not None:
      if not self._carrot_web_pressed and time.monotonic() - self._mouse_down_t > 0.5:
        # long gating for experimental mode - only allow toggle if longitudinal control is available
        if ui_state.has_longitudinal_control:
          self._experimental_mode = not self._experimental_mode
          ui_state.params.put("ExperimentalMode", self._experimental_mode)
        self._mouse_down_t = None
        self._did_long_press = True

    if rl.get_time() - self._last_refresh > 5.0:
      # Update version text
      self._version_text = self._get_version_text()
      self._ip_address = self._read_network_address(ui_state.params_memory)
      self._last_refresh = rl.get_time()
      self._update_params()

  def set_callbacks(self, on_settings: Callable | None = None, on_carrot_web: Callable | None = None):
    self._on_settings_click = on_settings
    self._on_carrot_web_click = on_carrot_web

  def _handle_mouse_press(self, mouse_pos: MousePos):
    self._carrot_web_pressed = rl.check_collision_point_rec(mouse_pos, self._carrot_web_icon.rect)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._carrot_web_pressed and rl.check_collision_point_rec(mouse_pos, self._carrot_web_icon.rect):
      if self._on_carrot_web_click:
        self._on_carrot_web_click()
    elif not self._did_long_press:
      if self._on_settings_click:
        self._on_settings_click()
    self._carrot_web_pressed = False
    self._did_long_press = False

  def _get_version_text(self) -> tuple[str, str, str, str] | None:
    version = ui_state.params.get("Version")
    branch = ui_state.params.get("GitBranch")
    commit = ui_state.params.get("GitCommit")

    if not all((version, branch, commit)):
      return None

    commit_date_raw = ui_state.params.get("GitCommitDate")
    try:
      unix_ts = int(commit_date_raw.strip("'").split()[0])
      date_str = datetime.datetime.fromtimestamp(unix_ts).strftime("%Y/%m/%d")
    except (ValueError, IndexError, TypeError, AttributeError):
      date_str = ""

    return version, branch, commit[:7], date_str

  def _render(self, _):
    # TODO: why is there extra space here to get it to be flush?
    text_pos = rl.Vector2(self.rect.x - 2 + HOME_PADDING, self.rect.y - 5)
    self._openpilot_label.set_position(text_pos.x, text_pos.y)
    self._openpilot_label.render()

    if self._version_text is not None:
      # release branch
      release_branch = self._version_text[1] in RELEASE_BRANCHES

      # 1踰덉㎏ 以? carrotpilot ?놁뿉 踰꾩쟾
      version_y = text_pos.y + self._openpilot_label.font_size - self._version_label.font_size
      self._version_label.set_text(" " + self._version_text[0])
      self._version_label.set_position(text_pos.x + self._openpilot_label.text_width + 8, version_y)
      self._version_label.render()

      line_x = text_pos.x
      line_y = text_pos.y + self._openpilot_label.font_size + 12

      # 2踰덉㎏ 以? 釉뚮옖移섎챸
      branch_name = "release" if release_branch else self._version_text[1]
      self._branch_label.set_max_width(self.rect.width - HOME_PADDING * 2)
      self._branch_label.set_text(branch_name)
      self._branch_label.set_position(line_x, line_y)
      self._branch_label.render()
      line_y += self._branch_label.font_size + 6

      # 3踰덉㎏ 以? ?좎쭨 (而ㅻ컠)
      date_text = self._version_text[3] if release_branch else f"{self._version_text[3]} ({self._version_text[2]})"
      self._date_label.set_text(date_text)
      self._date_label.set_position(line_x, line_y)
      self._date_label.render()
      line_y += self._date_label.font_size + 6

      # 4踰덉㎏ 以? wifi(IP) 二쇱냼
      self._ip_label.set_text(self._ip_address)
      self._ip_label.set_position(line_x, line_y)
      self._ip_label.render()

    # ***** Center-aligned bottom section icons *****
    self._experimental_icon.set_visible(self._experimental_mode)
    self._mic_icon.set_visible(ui_state.recording_audio)

    footer_rect = rl.Rectangle(self.rect.x + HOME_PADDING, self.rect.y + self.rect.height - 48, self.rect.width - HOME_PADDING, 48)
    self._status_bar_layout.render(footer_rect)
