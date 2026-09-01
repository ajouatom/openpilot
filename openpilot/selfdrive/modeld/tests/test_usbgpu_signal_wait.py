import sys
from types import SimpleNamespace

import pytest

if sys.platform != "linux":
  pytest.skip("AMD runtime is only available on Linux", allow_module_level=True)

from tinygrad.runtime import ops_amd


class FakeOwner:
  def __init__(self, usb: bool):
    self.usb = usb
    self.iface_sleep_calls: list[int] = []
    self.iface = SimpleNamespace(sleep=self.iface_sleep_calls.append)

  def is_usb(self) -> bool:
    return self.usb


def make_signal(usb: bool) -> ops_amd.AMDSignal:
  signal = object.__new__(ops_amd.AMDSignal)
  signal.owner = FakeOwner(usb)
  return signal


def test_usb_busy_waits_before_threshold(monkeypatch):
  sleep_calls: list[float] = []
  monkeypatch.setattr(ops_amd.time, "sleep", sleep_calls.append)
  signal = make_signal(True)

  signal._sleep(ops_amd.AMD_USB_WAIT_SPIN_MS)

  assert sleep_calls == []
  assert signal.owner.iface_sleep_calls == []


def test_usb_yields_after_threshold(monkeypatch):
  sleep_calls: list[float] = []
  monkeypatch.setattr(ops_amd.time, "sleep", sleep_calls.append)
  signal = make_signal(True)

  signal._sleep(ops_amd.AMD_USB_WAIT_SPIN_MS + 1)

  assert sleep_calls == [ops_amd.AMD_USB_WAIT_SLEEP_US / 1_000_000]
  assert signal.owner.iface_sleep_calls == []


def test_non_usb_keeps_interrupt_wait(monkeypatch):
  sleep_calls: list[float] = []
  monkeypatch.setattr(ops_amd.time, "sleep", sleep_calls.append)
  signal = make_signal(False)

  signal._sleep(201)

  assert sleep_calls == []
  assert signal.owner.iface_sleep_calls == [200]
