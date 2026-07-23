import os

import pytest

if os.name == "nt":
  pytest.skip("TICI hardware modules require Linux", allow_module_level=True)

from openpilot.system.hardware.tici import hardware
from openpilot.system.hardware.tici.amplifier import Amplifier


def test_c3x_lite_skips_missing_amplifier(monkeypatch):
  tici = hardware.Tici()
  monkeypatch.setattr(tici, "get_device_type", lambda: "tici")
  monkeypatch.setattr(hardware, "is_c3x_lite", lambda: True)

  assert tici.amplifier is None


def test_standard_tici_keeps_amplifier(monkeypatch):
  tici = hardware.Tici()
  monkeypatch.setattr(tici, "get_device_type", lambda: "tici")
  monkeypatch.setattr(hardware, "is_c3x_lite", lambda: False)

  assert isinstance(tici.amplifier, Amplifier)
