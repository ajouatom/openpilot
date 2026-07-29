from dataclasses import dataclass

import pytest

from openpilot.selfdrive.pandad import panda_helpers


@dataclass
class FakePanda:
  serial: str
  hw_type: bytes
  internal: bool
  closed: bool = False

  def get_usb_serial(self) -> str:
    return self.serial

  def get_type(self) -> bytes:
    return self.hw_type

  def is_internal(self) -> bool:
    return self.internal

  def close(self) -> None:
    self.closed = True


def test_connect_all_pandas_sorts_c3_internal_first():
  pandas = {
    "red": FakePanda("red", b"\x07", False),
    "dos": FakePanda("dos", b"\x06", True),
  }
  connected = panda_helpers.connect_all_pandas(["red", "dos"], lambda serial: pandas[serial])

  assert [p.serial for p in connected] == ["dos", "red"]
  assert panda_helpers.pandas_include_internal(connected)


@pytest.mark.parametrize("hw_type", [b"\x09", b"\x0a"])
def test_connect_all_pandas_preserves_single_internal(hw_type):
  panda = FakePanda("internal", hw_type, True)
  connected = panda_helpers.connect_all_pandas(["internal"], lambda _serial: panda)

  assert connected == [panda]
  assert panda_helpers.pandas_include_internal(connected)


def test_connect_all_pandas_closes_partial_connection():
  panda = FakePanda("dos", b"\x06", True)

  def fake_flash(serial):
    if serial == "red":
      raise RuntimeError("connect failed")
    return panda

  with pytest.raises(RuntimeError, match="connect failed"):
    panda_helpers.connect_all_pandas(["dos", "red"], fake_flash)

  assert panda.closed
