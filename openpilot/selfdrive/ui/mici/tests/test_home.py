import pytest

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout


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
