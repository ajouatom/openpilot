from pathlib import Path

import pytest

from openpilot.common.file_chunker import get_manifest_path
from openpilot.selfdrive.modeld.big_model import BigModelManifest, _write_state
import openpilot.selfdrive.modeld.helpers as helpers
from openpilot.selfdrive.modeld.helpers import (modeld_pkl_path, refresh_usbgpu_device_cache, select_vision_streams,
                                                usbgpu_compiled_path, usbgpu_pcie_not_ready)


@pytest.mark.parametrize(
  ("available", "use_wide_camera", "main", "use_extra"),
  [
    ({"road", "wide"}, True, "road", True),
    ({"road", "wide"}, False, "road", False),
    ({"road"}, True, "road", False),
    ({"wide"}, True, "wide", False),
    ({"wide"}, False, None, False),
    (set(), True, None, False),
  ],
)
def test_select_vision_streams(available, use_wide_camera, main, use_extra):
  assert select_vision_streams(available, "road", "wide", use_wide_camera) == (main, use_extra)


def test_usbgpu_artifact_uses_checked_in_model_name():
  assert modeld_pkl_path(False).name == "driving_tinygrad.pkl"
  assert modeld_pkl_path(True, "a" * 64).name == "big_driving_aaaaaaaaaaaaaaaa_tinygrad.pkl"


def test_usbgpu_pcie_readiness_error_is_recognized_through_wrappers():
  try:
    try:
      raise RuntimeError("PCIe link not up (LTSSM=0x00), custom firmware not ready")
    except RuntimeError as exc:
      raise RuntimeError("eGPU model load failed") from exc
  except RuntimeError as exc:
    assert usbgpu_pcie_not_ready(exc)

  assert usbgpu_pcie_not_ready("PCIe link not up (LTSSM=0x00)")
  assert usbgpu_pcie_not_ready("AssertionError: read(0xB450, 1) failed: -1")
  assert usbgpu_pcie_not_ready("libusb_open: No such device (it may have been disconnected)")
  assert usbgpu_pcie_not_ready("AMD:0 does not exist (0 devices available)")
  assert not usbgpu_pcie_not_ready(RuntimeError("USB device disconnected"))


def test_usbgpu_pcie_readiness_error_is_recognized_in_exception_group():
  error = ExceptionGroup(
    "No interface for AMD:0 is available",
    [
      FileNotFoundError("/dev/kfd"),
      RuntimeError("no pcie"),
      ExceptionGroup(
        "USB backend",
        [RuntimeError("PCIe link not up (LTSSM=0x00), custom firmware not ready")],
      ),
    ],
  )

  assert usbgpu_pcie_not_ready(error)


def test_custom_usb_controller_waits_for_pcie_link(monkeypatch):
  from tinygrad.runtime.support.usb import CustomASM24Controller

  ltssm_values = iter((bytes([0x00]), bytes([0x00]), bytes([0x78])))
  power_calls = []
  monkeypatch.setattr(CustomASM24Controller, "read", lambda *_args: next(ltssm_values))
  monkeypatch.setattr(CustomASM24Controller, "set_pcie_power", lambda _self, enabled: power_calls.append(enabled))
  monkeypatch.setattr("tinygrad.runtime.support.usb.time.sleep", lambda _seconds: None)

  CustomASM24Controller(object())
  assert power_calls == [False, True]


def test_custom_usb_controller_preserves_ready_pcie_link(monkeypatch):
  from tinygrad.runtime.support.usb import CustomASM24Controller

  power_calls = []
  monkeypatch.setattr(CustomASM24Controller, "read", lambda *_args: bytes([0x78]))
  monkeypatch.setattr(CustomASM24Controller, "set_pcie_power", lambda _self, enabled: power_calls.append(enabled))

  CustomASM24Controller(object())
  assert power_calls == []


def test_custom_usb_controller_resets_bridge_after_failed_retrain(monkeypatch):
  from tinygrad.runtime.support.usb import CustomASM24Controller

  power_calls = []
  reset_calls = []
  monotonic_values = iter((0.0, 6.0))
  monkeypatch.setattr(CustomASM24Controller, "read", lambda *_args: bytes([0x00]))
  monkeypatch.setattr(CustomASM24Controller, "set_pcie_power", lambda _self, enabled: power_calls.append(enabled))
  monkeypatch.setattr(CustomASM24Controller, "reset_usb_bridge", lambda _self: reset_calls.append(True))
  monkeypatch.setattr("tinygrad.runtime.support.usb.time.monotonic", lambda: next(monotonic_values))
  monkeypatch.setattr("tinygrad.runtime.support.usb.time.sleep", lambda _seconds: None)

  with pytest.raises(RuntimeError, match="USB bridge reset requested"):
    CustomASM24Controller(object())

  assert power_calls == [False, True]
  assert reset_calls == [True]


def test_usb_pci_device_releases_resources_after_init_failure(monkeypatch):
  from tinygrad.runtime.support import system

  events = []

  class FakeUSB:
    product = "custom eGPU"
    is_custom = True

    def __init__(self, *_args):
      events.append("usb-open")

    def close(self):
      events.append("usb-close")

  monkeypatch.setattr(system.System, "flock_acquire", lambda _name: 123)
  monkeypatch.setattr(system, "USB3", FakeUSB)

  def fail_controller(_usb):
    raise RuntimeError("PCIe link not up")

  monkeypatch.setattr(system, "CustomASM24Controller", fail_controller)
  monkeypatch.setattr(system.os, "close", lambda fd: events.append(("lock-close", fd)))

  with pytest.raises(RuntimeError, match="PCIe link not up"):
    system.USBPCIDevice("AM", object(), "usb:4-90")

  assert events == ["usb-open", "usb-close", ("lock-close", 123)]


def test_refresh_usbgpu_device_cache(monkeypatch):
  from tinygrad.runtime.support.usb import USB3

  cleared = []
  monkeypatch.setattr(USB3.list_devices.__func__, "cache_clear", lambda: cleared.append(True))

  refresh_usbgpu_device_cache()

  assert cleared == [True]


def test_usbgpu_present_accepts_both_supported_usb_ids(monkeypatch, tmp_path: Path):
  monkeypatch.setattr(helpers.Path, "glob", lambda _self, _pattern: [tmp_path])
  (tmp_path / "speed").write_text("5000")

  for vendor in ("add1", "3801"):
    (tmp_path / "idVendor").write_text(vendor)
    (tmp_path / "idProduct").write_text("0001")
    assert helpers.usbgpu_present()

  (tmp_path / "idVendor").write_text("ffff")
  assert not helpers.usbgpu_present()


def test_usbgpu_present_rejects_usb2_fallback(monkeypatch, tmp_path: Path):
  monkeypatch.setattr(helpers.Path, "glob", lambda _self, _pattern: [tmp_path])
  (tmp_path / "idVendor").write_text("3801")
  (tmp_path / "idProduct").write_text("0001")
  (tmp_path / "speed").write_text("480")

  assert not helpers.usbgpu_present()


def test_usbgpu_compiled_path_falls_back_to_previous_model(monkeypatch, tmp_path: Path):
  cache_dir = tmp_path / "cache"
  compiled_dir = tmp_path / "compiled"
  monkeypatch.setenv("CARROT_BIG_MODEL_DIR", str(cache_dir))
  monkeypatch.setattr(helpers, "MODELS_DIR", compiled_dir)
  active = BigModelManifest("big-401", "big_driving_supercombo.onnx", 1, "a" * 64, "https://example.com/401.onnx")
  previous = BigModelManifest("big-400", "big_driving_supercombo.onnx", 1, "b" * 64, "https://example.com/400.onnx")
  _write_state(active, previous, cache_dir)

  previous_pkl = modeld_pkl_path(True, previous.sha256)
  Path(get_manifest_path(previous_pkl)).parent.mkdir(parents=True)
  Path(get_manifest_path(previous_pkl)).write_text("1")
  assert usbgpu_compiled_path() == previous_pkl
