from types import SimpleNamespace

import pytest
import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout
from openpilot.system.ui.lib.application import MousePos


class FakeParamsMemory:
  def __init__(self, network_address):
    self.network_address = network_address
    self.requested_keys = []

  def get(self, key):
    self.requested_keys.append(key)
    return self.network_address


@pytest.mark.parametrize(
  "network_address,expected",
  [
    (None, "Offline"),
    ("", "Offline"),
    ("0.0.0.0", "Offline"),
    (" 192.168.43.10\n", "192.168.43.10"),
  ],
)
def test_read_network_address(network_address, expected):
  params_memory = FakeParamsMemory(network_address)

  assert MiciHomeLayout._read_network_address(params_memory) == expected
  assert params_memory.requested_keys == ["NetworkAddress"]


def test_carrot_web_icon_routes_tap_without_opening_settings():
  calls = []
  layout = object.__new__(MiciHomeLayout)
  layout._carrot_web_icon = SimpleNamespace(rect=rl.Rectangle(100, 100, 48, 48))
  layout._on_carrot_web_click = lambda: calls.append("carrot_web")
  layout._on_settings_click = lambda: calls.append("settings")
  layout._did_long_press = False
  layout._carrot_web_pressed = False

  layout._handle_mouse_press(MousePos(120, 120))
  layout._handle_mouse_release(MousePos(120, 120))

  assert calls == ["carrot_web"]
  assert not layout._carrot_web_pressed
